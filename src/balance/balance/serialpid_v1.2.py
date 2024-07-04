from module.foc_motor_serial import MotorControl
from module.DXL_motor_control import DXL_Conmunication
import rclpy
import math
import time
import sys
import os
import threading
import numpy as np
from rclpy.node import Node
# from simple_pid import PID
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import traceback
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from geometry_msgs.msg import Twist


# balance_kp = 103*0.6
# balance_ki = 2*balance_kp/0.5
# balance_kd = 0.125*103*0.5
angularv_kp = 30
angularv_ki = 10

balance_kp = 0  # *0.75
balance_kd = 0  # 0.5
COM_bias = 0

velocity_kp = 0
velocity_ki = 0
velocity_setpoint = 0

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.previous_error = 0

    def update(self, target, now, dt):

        error = target - now
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.previous_error = error
        return output
    
    # def output_limits(self,min,max):

    #     self.max = max
    #     self.min = min



class RosTopicSubscriber(Node):

    def __init__(self):
        super().__init__('imusubscriber')
        self.subscription = self.create_subscription(
            Imu, 'imu/data_raw', self.listener_callback, 1)
        # self.theta_Pub = self.create_publisher(Float32 , 'theta' , 1)
        # self.X_dot_Pub = self.create_publisher(Float32 , 'X_dot' , 1)
        self.pitch_init = 0
        self.angularvelocity_y = 0

        self.twist_sub = self.create_subscription(
            Twist, '/cmd_vel', self.twist_callback, 1)
        self.linear_vel = 0.0
        self.angular_vel = 0.0

    def listener_callback(self, msg):

        qua_x = msg.orientation.x
        qua_y = msg.orientation.y
        qua_z = msg.orientation.z
        qua_w = msg.orientation.w
        self.angularvelocity_y = msg.angular_velocity.y
        self.pitch_init = (
            math.asin(2 * (qua_w * qua_y - qua_z * qua_x)))
        
        t3 = 2 * (qua_w * qua_z + qua_x * qua_y)
        t4 = 1 - 2 * (qua_y * qua_y + qua_z * qua_z)
        self.yaw_init = math.atan2(t3, t4)

    def getImuOrientation(self) -> float:
        return -self.pitch_init, self.yaw_init
    
    def twist_callback(self, msg):
        self.linear_vel = msg.linear.x * 200
        self.angular_vel = msg.angular.z


class robotcontrol:

    def __init__(self, motor01, motor02, motor11, motor12, wheel1, wheel2):
        rclpy.init()
        self.dxl = DXL_Conmunication(device_name="/dev/dxl", b_rate=57600)
        self.mc = MotorControl(device_name="/dev/foc", baudrate=2e6)
        self.subscriber = RosTopicSubscriber()
        subscriber_thread = threading.Thread(
            target=rclpy.spin, args=(self.subscriber,), daemon=True)
        subscriber_thread.start()
        self.pid_thread = None
        self.motor01 = motor01
        self.motor02 = motor02
        self.motor11 = motor11
        self.motor12 = motor12
        self.wheel1 = wheel1
        self.wheel2 = wheel2
        self.createdxlmotor()
        

        self.balance_pid = PID(10, 0, 1)
        # self.balance_pid.output_limits = (-10, 10)
        self.velocity_pid = PID(0.008, 0.0, 0.00001)
        # self.velocity_pid.output_limits = (0, 0)
        self.angularvelocity_pid = PID(65, 0, 0)
        # self.angularvelocity_pid.output_limits = (-300, 300)

        self.yaw_pid = PID(350, 0, 0.4)

        self.dt = 1 / 200
        self.prev_pitch = 0
        self.prev_yaw = 0
        self.isRunning = False


    def getControllerPIDParam(self):
        print(f'Balance kp: {self.balance_pid.Kp}, kd: {self.balance_pid.Kd}')
        print(f'Angular kp: {self.angularvelocity_pid.Kp}, ki: {self.angularvelocity_pid.Ki}')
        print(f'Velocity kp: {self.velocity_pid.Kp}, ki: {self.velocity_pid.Ki}')
        return self.balance_pid.Kp, self.balance_pid.Kd, self.angularvelocity_pid.Kp, self.angularvelocity_pid.Kd

    def setAngularPI(self, kp, ki):
        self.angularvelocity_pid.Kp = kp
        self.angularvelocity_pid.Ki = ki
    
    def setBalancePD(self, kp, kd):
        self.balance_pid.Kp = kp
        self.balance_pid.Kd = kd
    
    def setVelocityPID(self, kp, kd):
        self.velocity_pid.Kp = kp
        self.velocity_pid.Kd = kd

    def createdxlmotor(self):

        self.motor01 = self.dxl.createMotor("motor01", 1)
        self.motor02 = self.dxl.createMotor("motor02", 2)
        self.motor11 = self.dxl.createMotor("motor11", 11)
        self.motor12 = self.dxl.createMotor("motor12", 12)
        self.dxl.addAllBuckPrarmeter()
        self.enableAllMotor()
        self.dxl.updateMotorData()

    def enableAllMotor(self):
        self.motor01.enableMotor()
        self.motor02.enableMotor()
        self.motor11.enableMotor()
        self.motor12.enableMotor()

    def startfocmotor(self):

        self.mc.startmotor(0x01)
        self.mc.startmotor(0x02)

    def lockleg(self):

        self.enableAllMotor()
        self.dxl.updateMotorData()
        self.motor01.writePosition(2760)  # 2760-1795
        self.motor02.writePosition(3795)  # 3800-2900
        self.motor11.writePosition(2760)
        self.motor12.writePosition(2900)
        self.dxl.sentAllCmd()

        _ = self.mc.torquecontrol(0x01, 300)
        # _ = self.mc.torquecontrol(0x02, -300)
        time.sleep(0.1)

        self.motor01.writePosition(2760)  # 2760-1795
        self.motor02.writePosition(3795)  # 4095-2900
        self.motor11.writePosition(2760)
        self.motor12.writePosition(3795)
        self.dxl.sentAllCmd()
        

    def motortorquecommand(self, id, torque):
        a = self.mc.torquecontrol(id, torque)

        return a

    def motorspeedcommand(self, id, speed):
        a = self.mc.speedcontrol(id, speed)

        return a

    def disableALLmotor(self):
        self.isRunning = False
        if self.pid_thread is not None:
            self.pid_thread.join()
        self.dxl.disableAllMotor()
        self.mc.stopmotor(0x01)
        self.mc.stopmotor(0x02)

    def closeSystem(self):
        self.dxl.closeHandler()
        self.mc.closeport()
    
    def getPitchDot(self, pitch):
        pitch_dot = (pitch - self.prev_pitch) / self.dt
        self.prev_pitch = pitch
        return pitch_dot
    
    def getYawDot(self, yaw):
        yaw_dot = (yaw - self.prev_yaw) / self.dt
        self.prev_yaw = yaw
        return yaw_dot
               

    def controller(self):
        
        dt = 1 / 200
        motorrpm = 0
        desire_pitch = 0
        position = 0.0
        middle_ang = -0.05     #-0.028
        motor_speed = 0.0
        count = 0
        count_drop = 0
        yaw_vel_odom = 0.0
        
        while True:
            start = time.time()
            if not self.isRunning:
                break
            pitch, yaw = self.subscriber.getImuOrientation()
            # print(f'pitch : {pitch}')
            pitch_velocity = self.getPitchDot(pitch)
            yaw_velocity = self.getYawDot(yaw)
            
            # print(f'pitch vel: {pitch_velocity}')

            ## Adapt COG offset angle ##
            # angle_bias = desire_pitch - pitch
            output2 = self.balance_pid.update(desire_pitch, pitch,dt)
            # output2 = self.balance_pid.update(desire_pitch, pitch,dt)
            # desire_pitch -= angle_bias * 0.00001
            # print(f'dersire ang {desire_pitch}')
            output3 = self.angularvelocity_pid.update(output2, pitch_velocity,dt)
            output3 = int(output3)
            # print(f'output {output3}')
            if abs(pitch) > math.radians(40):
                output3 = 0
            
            output4 = self.yaw_pid.update(self.subscriber.angular_vel, yaw_vel_odom, dt)
            # print(yaw_velocity, output4)

            output4_a = int(output3-output4)
            output4_b = int(output3+output4)
            a = self.mc.torquecontrol(0x01, output4_a)
            b = self.mc.torquecontrol(0x02, -output4_b)
            count += 1
            if a is None or b is None:
                count_drop += 1
            else:
                # motorrpm = abs(a[2]) + abs(b[2])
                # motorrpm = -a[2]
                # print('a+b', a[2]+b[2])
                yaw_vel_odom = (a[2]+b[2])*0.03076*dt/0.2
                # print('angular_vel', yaw_vel_odom)
                motor_speed = -(a[2]+(-b[2])) / 2 * 2 * np.pi / 60   # rad/s
                # d_pos = motor_speed * 0.06152/2 * dt
                # position += d_pos
            # print('speed:', motor_speed)
            desire_pitch = self.velocity_pid.update(self.subscriber.linear_vel, motor_speed, dt)
            # print('desire_pitch:', desire_pitch)
            # print('drop_rate:', count_drop/count*100)
            # print('frequency:', 1/dt)
            # print(f'motor speed {motorrpm}')
            desire_pitch += middle_ang
            end = time.time()
            dt = start - end
            # print('dt', dt)
        # finally:
        #     self.disableALLmotor()

    def startController(self):
        self.prev_pitch = 0
        self.isRunning = True
        self.lockleg()
        
        # self.startfocmotor()
        self.pid_thread = threading.Thread(target=self.controller)
        self.pid_thread.start()

def main():
    robot_motor = robotcontrol(
        'motor01', 'motor02', 'motor11', 'motor12', 'wheel1', 'wheel2')
    command_dict = {
        "d": robot_motor.disableALLmotor,
        "start": robot_motor.startController,
        "get": robot_motor.getControllerPIDParam,
        "clear": robot_motor.mc.cleanerror,
    }

    while True:
        try:
            cmd = input("CMD :")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "exit":
                robot_motor.disableALLmotor()
                robot_motor.closeSystem()
                break

        except Exception as e:
            traceback.print_exc()
            break


if __name__ == '__main__':

    main()
