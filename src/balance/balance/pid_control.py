import sys
import os

# Manually add the full path to 'balance/module' to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'balance', 'module'))

from balance.module.foc_motor_serial import MotorControl
from balance.module.DXL_motor_control import DXL_Conmunication
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
from std_msgs.msg import String
import matplotlib.pyplot as plt
from tutorial_interfaces.srv import SetMode


WHEEL_DIS = 0.03076

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



class RosTopicSubscriber(Node):

    def __init__(self):
        super().__init__('imusubscriber')
        self.subscription = self.create_subscription(
            Imu, 'imu/data_raw', self.listener_callback, 1)
        self.pitch_init = 0
        self.yaw_init = 0
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

class PoseService(Node):
    def __init__(self):
        super().__init__('PoseService')
        self.srv = self.create_service(SetMode, 'get_pose', self.pose_callback)
        self.get_logger().info('Pose service ready')

        self.current_mode = ""
        self.previous_mode = ""
        

    def pose_callback(self, request, response):
        # Provide the current pose data upon request
        if request.mode == "b" or request.mode == "x":
            self.current_mode = request.mode  # Store the received mode
            response.success = True
            response.message = f"Mode set to: {self.current_mode}"
        else:
            self.current_mode = ""
        return response


class robotcontrol:

    def __init__(self, motor01, motor02, motor11, motor12, wheel1, wheel2, service):

        # self.dxl = DXL_Conmunication(device_name="/dev/dxl", b_rate=57600)
        # self.mc = MotorControl(device_name="/dev/foc", baudrate=2e6)
        self.dxl = DXL_Conmunication(device_name="/dev/ttyUSB2", b_rate=57600)
        self.mc = MotorControl(device_name="/dev/ttyUSB0", baudrate=2e6)

        # Create the subscriber and service
        self.subscriber = RosTopicSubscriber()
        self.service = service

        # Use a single thread to spin both the subscriber and the service
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.subscriber)
        self.executor.add_node(self.service)

        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.pid_thread = None
        self.motor01 = motor01
        self.motor02 = motor02
        self.motor11 = motor11
        self.motor12 = motor12
        self.wheel1 = wheel1
        self.wheel2 = wheel2
        self.createdxlmotor()
        
        self.balance_pid = PID(16, 0.0, 0.13)             #current best:20,0.0, 0.1
        self.velocity_pid = PID(0.008,  0.00, 0.00001)   # 0.008 0.00001
        self.angularvelocity_pid = PID(40, 0, 0.0)       #current best:40, 0, 0.0

        self.yaw_pid = PID(200, 0, 0.4) # 350, 0, 0.4

        self.dt = 1 / 200
        self.prev_pitch = 0
        self.prev_yaw = 0
        self.isRunning = False
        self.yaw_robot_vel = 0
        self.linear_robot_vel = 0

        self.time_list = []
        self.pitch_list = []
        self.angularvel_list = []
        self.linear_vel_list = []


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
        self.motor12 = self.dxl.createMotor("motor12", 22)
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
        self.motor01.writePosition(2150)  
        self.motor02.writePosition(2150)  
        self.motor11.writePosition(2048)
        self.motor12.writePosition(2048)
        self.dxl.sentAllCmd()

        self.enableAllMotor()
        self.dxl.updateMotorData()
        self.motor01.writePosition(1650)  # 2760-1795
        self.motor02.writePosition(2150)  # 3800-2900
        self.motor11.writePosition(2090)
        self.motor12.writePosition(2048)
        self.dxl.sentAllCmd()

        self.enableAllMotor()
        self.dxl.updateMotorData()
        self.motor01.writePosition(2150)  
        self.motor02.writePosition(2150)  
        self.motor11.writePosition(2048)
        self.motor12.writePosition(2048)
        self.dxl.sentAllCmd()
        
    def dxlMotorTest(self):
        self.enableAllMotor()
        self.dxl.updateMotorData()
        self.motor01.writePosition(2048)  # 2760-1795
        self.motor02.writePosition(2048)  # 3800-2900
        self.motor11.writePosition(2048)
        self.motor12.writePosition(2048)
        self.dxl.sentAllCmd()

    def focMotorTest(self):
        self.motortorquecommand(0x02, 80)
        self.motortorquecommand(0x01, -80)

    def motortorquecommand(self, id, torque):
        result = self.mc.torquecontrol(id, torque)
        return result

    def motorspeedcommand(self, id, speed):
        result = self.mc.speedcontrol(id, speed)

        return result

    def disableALLmotor(self):
        self.isRunning = False
        if self.pid_thread is not None:
            self.pid_thread.join()
        self.mc.stopmotor(0x01)
        self.mc.stopmotor(0x02)
        self.dxl.disableAllMotor()

    def plotResult(self):
        
        plt.figure(figsize=(10, 6))

        plt.subplot(3, 1, 1)
        plt.plot(self.time_list, self.pitch_list, label="pitch")
        plt.title('PID Controller')
        plt.ylabel('pitch')
        plt.grid(True)

        plt.subplot(3, 1, 2)
        plt.plot(self.time_list, self.angularvel_list, label="angular vel", color='orange')
        plt.ylabel('angular vel')
        plt.grid(True)

        plt.subplot(3, 1, 3)
        plt.plot(self.time_list, self.linear_vel_list, label="linear vel", color='green')
        plt.ylabel('linear vel')
        plt.xlabel('Time')
        plt.grid(True)

        plt.tight_layout()
        plt.savefig('pid_controller_output.png')
        print("The image has been saved as 'pid_controller_output.png'")

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
    
    def odomEstimate(self, rw_motor_st, lw_motor_st):
        if lw_motor_st is not None and rw_motor_st is not None:
            self.yaw_robot_vel = (rw_motor_st[2] + lw_motor_st[2]) * WHEEL_DIS * self.dt/0.2
            self.linear_robot_vel = -(rw_motor_st[2]+(-lw_motor_st[2])) / 2 * 2 * np.pi / 60   # rad/s

    def balanceControlLoop(self):

        dt = 1 / 200
        middle_ang = 0.109    #original=>0.11
        desire_pitch = 0     #0
        motor_speed = 0
        start_time = time.time()
        

        while True:
            current_time = time.time()-start_time
            if not self.isRunning:
                break
            pitch, yaw = self.subscriber.getImuOrientation()
            # print(f'pitch : {pitch}')
            pitch_velocity = self.getPitchDot(pitch)
            # print(f'pitch vel: {pitch_velocity}')

            ## Adapt COG offset angle ##

            desire_angular_vel = self.balance_pid.update(desire_pitch, pitch,dt)
            torque_cmd = self.angularvelocity_pid.update(desire_angular_vel, pitch_velocity,dt)
            torque_cmd = int(torque_cmd)
            if abs(pitch) > math.radians(40):
                torque_cmd = 0
            self.pitch_list.append(pitch)
            self.angularvel_list.append(pitch_velocity)

            steering_cmd = self.yaw_pid.update(-self.subscriber.angular_vel, self.yaw_robot_vel, dt)
            # print(yaw_velocity, steering_cmd)

            torque_cmd_right = int(torque_cmd-steering_cmd)
            torque_cmd_left = int(torque_cmd+steering_cmd)
            rw_motor_st = self.mc.torquecontrol(0x01, torque_cmd_left) # right
            lw_motor_st = self.mc.torquecontrol(0x02, -torque_cmd_right)

            self.odomEstimate(rw_motor_st, lw_motor_st)
            desire_pitch = self.velocity_pid.update(self.subscriber.linear_vel, self.linear_robot_vel, dt)
            self.linear_vel_list.append(self.linear_robot_vel)

            desire_pitch += middle_ang
            end = time.time()
            dt = start_time - end
            # print(f"dt : {dt}")

            self.time_list.append(current_time)

    def controller(self):
        
        dt = 1 / 200
        desire_pitch = 0     #0.0
        middle_ang = 0.0     #0.0
        yaw_vel_odom = 0.06
        
        while True:
            start = time.time()
            if not self.isRunning:
                break
            pitch, yaw = self.subscriber.getImuOrientation()
            # print(f'pitch : {pitch}')
            pitch_velocity = self.getPitchDot(pitch)
            
            # print(f'pitch vel: {pitch_velocity}')

            ## Adapt COG offset angle ##

            output2 = self.balance_pid.update(desire_pitch, pitch,dt)
            output3 = self.angularvelocity_pid.update(output2, pitch_velocity,dt)
            output3 = int(output3)
            if abs(pitch) > math.radians(40):
                output3 = 0
            
            output4 = self.yaw_pid.update(self.subscriber.angular_vel, self.yaw_robot_vel, dt)
            # print(yaw_velocity, output4)

            output4_a = int(output3-output4)
            output4_b = int(output3+output4)
            rw_motor_st = self.mc.torquecontrol(0x01, output4_a) # right
            lw_motor_st = self.mc.torquecontrol(0x02, -output4_b)
            self.odomEstimate(rw_motor_st, lw_motor_st)
            # print('speed:', motor_speed)
            desire_pitch = self.velocity_pid.update(self.subscriber.linear_vel, self.linear_robot_vel, dt)
            # print('desire_pitch:', desire_pitch)
            # print('drop_rate:', count_drop/count*100)
            # print('frequency:', 1/dt)
            # print(f'motor speed {motorrpm}')
            desire_pitch += middle_ang
            end = time.time()
            dt = start - end

    def startController(self):
        self.prev_pitch = 0
        self.isRunning = True
        self.lockleg()
        
        # self.startfocmotor()
        self.pid_thread = threading.Thread(target=self.controller)
        self.pid_thread.start()
    
    def startBalance(self):
        self.prev_pitch = 0
        self.isRunning = True
        self.lockleg()
        
        # self.startfocmotor()
        self.pid_thread = threading.Thread(target=self.balanceControlLoop)
        self.pid_thread.start()
    
    def clearWheelMotorEr(self):
        self.mc.cleanerror(0x01)
        self.mc.cleanerror(0x02)

def main():
    rclpy.init()
    service = PoseService()

    robot_motor = robotcontrol(
        'motor01', 'motor02', 'motor11', 'motor12', 'wheel1', 'wheel2', service)
    command_dict = {
        "d": robot_motor.disableALLmotor,
        "start": robot_motor.startController,
        "get": robot_motor.getControllerPIDParam,
        "clear": robot_motor.clearWheelMotorEr,
        "lock": robot_motor.lockleg,
        "foct": robot_motor.focMotorTest,
        "b": robot_motor.startBalance,
    }
    while True:
        try:
            #cmd = input("CMD :")
            cmd = ""  
            time.sleep(0.1)
            
            if service.current_mode != service.previous_mode:
                cmd = service.current_mode
                #print("cmd",cmd)
                service.previous_mode = service.current_mode
                if cmd in command_dict:
                    command_dict[cmd]()
                elif cmd == "x":
                    robot_motor.disableALLmotor()
                    robot_motor.closeSystem()
                    break
            
        except Exception as e:
            traceback.print_exc()
            break

if __name__ == '__main__':
    main()