import rclpy
import math
import time
import sys
import os
import numpy as np
from rclpy.node import Node
from simple_pid import PID
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from module.DXL_motor_control import DXL_Conmunication
from module.foc_motor_serial import MotorControl
import matplotlib.pyplot as plt

# balance_kp = 103*0.6
# balance_ki = 2*balance_kp/0.5
# balance_kd = 0.125*103*0.5 
balance_kp = 75*0.6 #*0.75
balance_ki = 0
balance_kd = 0.3*0.6#0.5
balance_setpoint = 0

velocity_kp = 70
velocity_ki = velocity_kp/200
velocity_setpoint = 0

class ImuSubscriber(Node):

    def __init__(self):
        super().__init__('imusubscriber')
        self.subscription = self.create_subscription(Imu,'imu/data_raw' , self.listener_callback, 1)
        # self.theta_Pub = self.create_publisher(Float32 , 'theta' , 1)
        # self.X_dot_Pub = self.create_publisher(Float32 , 'X_dot' , 1)

    def listener_callback(self, msg):
        
            qua_x = msg.orientation.x
            qua_y = msg.orientation.y
            qua_z = msg.orientation.z
            qua_w = msg.orientation.w
            self.angularvelocity_y = msg.angular_velocity.y
            # self.pitch_degree =math.degrees(math.atan2(2*(qua_w*qua_z+qua_x*qua_y),1-2*(qua_y*qua_y+qua_z*qua_z)))
            self.pitch_init= (math.asin(2 * (qua_w * qua_y - qua_z * qua_x)))*(180/math.pi)
            self.pitch_least = 0
            # self.pitch_degree , _ =  self.complementary_filter(msg.augular_velocity.x    , msg.augular_velocity.y   , msg.augular_velocity.z,
            #                                                    msg.linear_acceleration.x , msg.linear_acceleration.y, msg.linear_acceleration.z,
            #                                                    0 , 0 , 0.98 , 0.005)
            
    def returndegree(self):

    #     # if (self.pitch_degree > 0):
    #     #     self.pitch_degree = 180 - self.pitch_degree
    #     # elif (self.pitch_degree < 0):
    #     #     self.pitch_degree = -(180 + self.pitch_degree)
        # self.pitch_init *= 0.7
        # self.pitch_init += self.pitch_least*0.3
        # self.pitch_least = self.pitch_init
        # self.theta_Pub.publish(float(-self.pitch_init))
        A = [-self.pitch_init,self.angularvelocity_y]
        return A 
    # def get_accelerometer_angles(self, ax, ay, az):
    #     pitch_acc = np.arctan2(ay, np.sqrt(ax**2 + az**2))
    #     roll_acc = np.arctan2(-ax, az)
    #     return pitch_acc, roll_acc
    
    # def complementary_filter(self,ax, ay, az, gx, gy, gz, pitch, roll, alpha, dt):
    # # 使用加速度計計算的角度
    #     pitch_acc, roll_acc = self.get_accelerometer_angles(ax, ay, az)
        
    #     # 使用陀螺儀計算的角度
    #     pitch_gyro = pitch + gx * dt
    #     roll_gyro = roll + gy * dt
        
    #     # 互補濾波器融合
    #     pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc
    #     roll = alpha * roll_gyro + (1 - alpha) * roll_acc
        
    #     return pitch, roll
    
class robotmotor:
    
    def __init__(self, motor01, motor02, motor11, motor12, wheel1, wheel2):

        self.dxl = DXL_Conmunication(device_name="/dev/dxl",b_rate=57600)
        self.mc = MotorControl(device_name="/dev/foc",baudrate=1000000)
        self.motor01 = motor01
        self.motor02 = motor02
        self.motor11 = motor11  
        self.motor12 = motor12
        self.wheel1 = wheel1
        self.wheel2 = wheel2
        self.createdxlmotor()
        self.lastberr = 0
        self.berrsum = 0
        self.lastverr = 0
        # self.balance_pid = PID(balance_kp, balance_ki ,balance_kd,setpoint=balance_setpoint)
        # self.balance_pid.output_limits = (-4000,4000)
        # self.balance_pid.output_limits = (-2000,2000)
        # self.velocity_pid = PID(velocity_kp,velocity_ki, 0 ,setpoint=velocity_setpoint)
        # self.velocity_pid.output_limits = (-4000,4000)
        # self.balance_pid.output_limits = (-2000,2000)
        

    def createdxlmotor(self):

        self.motor01 = self.dxl.createMotor("motor01", 1)
        self.motor02 = self.dxl.createMotor("motor02", 2)
        self.motor11 = self.dxl.createMotor("motor11", 11)
        self.motor12 = self.dxl.createMotor("motor12", 12)
        self.dxl.addAllBuckPrarmeter()
        self.motor01.enableMotor()
        self.motor02.enableMotor()
        self.motor11.enableMotor()
        self.motor12.enableMotor()
        self.dxl.updateMotorData()

    def startfocmotor(self):

        self.mc.startmotor(0x01)
        self.mc.startmotor(0x02)

    def lockleg(self):
        
        self.dxl.updateMotorData()
        self.motor01.writePosition(2760) #2760-1795
        self.motor02.writePosition(3795) #4095-2900
        self.motor11.writePosition(2760)
        self.motor12.writePosition(3795)
        self.dxl.sentAllCmd()


    def motortorquecommand(self,id,torque):
        a = self.mc.torquecontrol(id ,torque)
        
        return a
        
    def motorspeedcommand(self, id, speed):
        a = self.mc.speedcontrol(id ,speed)

        return a
        
    def disableALLmotor(self):

        self.dxl.disableAllMotor()
        self.mc.stopmotor(0x01)
        self.mc.stopmotor(0x02)
        time.sleep(1)
        self.dxl.closeHandler()
        self.mc.closeport()
    
    def balancepid(self, degree):
        # output1 = self.balance_pid(degree)
        err = balance_setpoint - degree
        derr = err - self.lastberr
        self.lastErr = err
        self.berrSum += err
        outPID = self.kp * err + (self.ki * self.berrSum) + (self.kd * derr)
        return(output1)
    
    def velocitypid(self, motor_rpm):

        output2 = self.velocity_pid(motor_rpm)
        return(output2)
    

def main():
   
    rclpy.init()
    imu_subscriber = ImuSubscriber()
    robot_motor = robotmotor('motor01','motor02','motor11','motor12','wheel1','wheel2')
    robot_motor.lockleg()
    speed = 0
    try:

        while rclpy.ok():
            rclpy.spin_once(imu_subscriber) 
            pitch = imu_subscriber.returndegree()
            
            if  -35 <=  pitch <= 35:

                # print(pitch)
                output1 = robot_motor.balancepid(pitch)
                output2 = robot_motor.velocitypid(speed)
                outputall = output1
                - output2
                outputall = int(outputall)
                a1 = robot_motor.motorspeedcommand(0x01, outputall)
                a2 = robot_motor.motorspeedcommand(0x02, -outputall)
                if a1 is None or a2 is None:
                    pass
                else:
                    speed = abs(a1[2]) + abs(a2[2])
                # imu_subscriber.X_dot_Pub.publish(float(speed))
                print(outputall)
                # print(pitch-balance_setpoint)
                # print(speed)

            else:
                robot_motor.motorspeedcommand(0x01, 0)
                robot_motor.motorspeedcommand(0x02, 0)
            

            # # if e1 and e2:
            #     speed = e1 + e2
            # # else:
            #     print("error")


    except KeyboardInterrupt:

        print("Shutting down gracefully.")
        robot_motor.disableALLmotor()
        rclpy.shutdown()
        

    finally:
        robot_motor.disableALLmotor()
        rclpy.shutdown()
        

if __name__ == '__main__':

    main()
    

    




    
    

    



    

    





    





  
    


       



        