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
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
import sys 
from std_msgs.msg import Float32, Float32MultiArray


# balance_kp = 103*0.6
# balance_ki = 2*balance_kp/0.5
# balance_kd = 0.125*103*0.5
angularv_kp = 30
angularv_ki = 10

balance_kp = 0  # *0.75
balance_kd = 0  # 0.5
COM_bias = 0

LinearControlCoeff = 6

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
        self.max = sys.maxsize
        self.min = -sys.maxsize-1

    def update(self, target, now, dt):

        error = target - now
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.previous_error = error
        if output < self.min:
            output = self.min
        elif output > self.max:
            output = self.max
        return output
    
    def output_limits(self,min,max):
         self.max = max
         self.min = min



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
        self.subscription = self.create_subscription(
            String, 'body_pose', self.pose_callback, 1)
        self.bodypose = " "
    def getBodyPose(self):
        return self.bodypose
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
    
    def getPitch(self):
        return -self.pitch_init
    def twist_callback(self, msg):
        self.linear_vel = msg.linear.x * LinearControlCoeff
        self.angular_vel = msg.angular.z*0.6
    def pose_callback(self, msg):
        self.bodypose = msg.data


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
        self.wheel_pos_x = -0.025
        self.wheel_pos_y = -0.06
        self.createdxlmotor()

        self.balance_pid = PID(10, 0, 1)
        # self.balance_pid.output_limits = (-10, 10)
        self.velocity_pid = PID(0.008, 0.0, 0.00001)
        # self.velocity_pid.output_limits = (0, 0)
        self.angularvelocity_pid = PID(65, 0, 0)
        # self.angularvelocity_pid.output_limits = (-300, 300)
        self.center_pid = PID(10e-6, 0, 10e-8)

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
        theta1_dxl = 1200#int(1682 - theta1*4096/(2*np.pi))+1024
        theta2_dxl = int(3500)
        
        
        self.enableAllMotor()
        self.dxl.updateMotorData()
        self.motor01.writePosition(theta2_dxl)  # 2760-1795
        self.motor02.writePosition(theta1_dxl)  # 3800-2900
        self.motor11.writePosition(theta2_dxl)
        self.motor12.writePosition(600)
        self.dxl.sentAllCmd()

        _ = self.mc.torquecontrol(0x01, 300)
        # # _ = self.mc.torquecontrol(0x02, -300)
        time.sleep(0.15)
        

        self.motor01.writePosition(theta2_dxl)  # 2760-1795
        self.motor02.writePosition(theta1_dxl)  # 3800-2900
        self.motor11.writePosition(theta2_dxl)
        self.motor12.writePosition(theta1_dxl)
        self.dxl.sentAllCmd()            
    def frontStandUp(self):
        pass  
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
    def getRelWheelPos(self):
        return self.wheel_pos_x-0.025              
    def inverse_kinematics(self, x, y, L1=0.12, L2=0.12):
        # 計算 d    
        d = np.sqrt(x**2 + y**2)
        # 計算 theta2
        cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
        theta2 = np.arccos(cos_theta2)
        
        # 計算 theta1
        theta1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))
    
        return theta1, theta2

    def changeHeight(self, dx=0.0, dy=0.0):

        theta1, theta2 = self.inverse_kinematics(self.wheel_pos_x+dx, self.wheel_pos_y+dy)
        theta1_dxl = int(-900 - theta1*4096/(2*np.pi))
        theta2_dxl = int(0 + 2*theta2*4096/(2*np.pi))


        if theta1_dxl>2000 or theta1_dxl<-500:
            print('theta1_dxl  over constrain')
        elif theta2_dxl>3800 or theta2_dxl<975:
            print('theta2_dxl over constrain')
        else:
            
            self.motor01.writePosition(theta2_dxl) # 2760-1795
            self.motor02.writePosition(theta1_dxl)  # 3800-2900
            self.motor11.writePosition(theta2_dxl)
            self.motor12.writePosition(theta1_dxl)
            self.dxl.sentAllCmd()
            self.wheel_pos_x += dx
            self.wheel_pos_y += dy

    def controller(self):
        dt = 1 / 200
        motorrpm = 0
        desire_pitch = 0
        position = 0.0
        middle_ang = -0.04
        motor_speed = 0.0
        count = 0
        count_drop = 0
        yaw_vel_odom = 0.0
        HeightCount = 0
        while True:
            start = time.time()
            if not self.isRunning:
                break
#D mode     
            if self.subscriber.getBodyPose() == "a":
                if self.subscriber.getPitch() <0:
                    self.lockleg()#stand up when fall on back
                else:#fall on face stand up
                    self.frontStandUp()
            elif  self.subscriber.getBodyPose() == "y":
                self.changeHeight(dy=-0.08)
                time.sleep(1.5)
                self.changeHeight(dy=0.08)
            elif  self.subscriber.getBodyPose() == "b":
                pass
            elif  self.subscriber.getBodyPose() == "up":
                self.changeHeight(dy=-0.001)
                print("UPPPPPPPPPPPPPPPPPPP")
                HeightCount = HeightCount + 1#to be changed according to changeheight
            elif  self.subscriber.getBodyPose() == "down":
                self.changeHeight(dy=0.001)
                HeightCount = HeightCount - 1
            elif  self.subscriber.getBodyPose() == "left":
                self.changeHeight(dx=0.001)
            elif  self.subscriber.getBodyPose() == "right":
                self.changeHeight(dx=-0.001)
            if self.subscriber.getBodyPose() == "stop":
                self.disableALLmotor()
                self.closeSystem()



            self.setAngularPI(-((self.wheel_pos_y+0.06)/0.0005)*0.35+65,0)#(-(self.wheel_pos_y+0.06/0.0005*0.25))+65)
            #print(-((self.wheel_pos_y+0.06)/0.0005)*0.35+65)
            # #-(OldWheelX-self.wheel_pos_y)/0.0005*0.25,0)
            LinearControlCoeff=6+((self.wheel_pos_y-0.06)*10)
            #print(LinearControlCoeff)
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
                if self.subscriber.getBodyPose() == "x":
                    #print('motor_speed:', motor_speed)
                    centerOutput = self.center_pid.update(0.0,motor_speed, dt)#motorSpeed = 0
                    #print('centerOutput:', centerOutput)
                    self.changeHeight(dx=-centerOutput)
                    
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
            elif cmd == "i":
                robot_motor.changeHeight(dy=0.005)
            elif cmd == "k":
                robot_motor.changeHeight(dy=-0.005)
            elif cmd == "j":
                robot_motor.changeHeight(dx=0.005)
            elif cmd == "l":
                robot_motor.changeHeight(dx=-0.005)
            elif cmd == "exit":
                robot_motor.disableALLmotor()
                robot_motor.closeSystem()
                break
        except KeyboardInterrupt:
            robot_motor.disableALLmotor()
            robot_motor.closeSystem()
        except Exception as e:
            traceback.print_exc()
            break


if __name__ == '__main__':

    main()
