from LQR import InvertedPendulumLQR
import threading
import matplotlib.pyplot as plt
from module.foc_motor_serial import MotorControl
from module.DXL_motor_control import DXL_Conmunication
import rclpy
import math
import time
import sys
import os
import numpy as np 
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float32MultiArray
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        angle01 = 2760
        angle02 = 3795
        hip = (angle02-2706)/4096*360
        knee = (angle01-1760)*81/1000+64

        # imu thread
        self.imu_subscriber = ImuSubscriber()
        self.lqr_controller = InvertedPendulumLQR(hip, knee, delta_t=1/80)
        # self.imu_subscriber.pitch_bias = self.lqr_controller.theta_bias
        self.imu_subscriber.pitch_bias = math.radians(1.3)
        self.imu_thread = threading.Thread(
            target=rclpy.spin, args=(self.imu_subscriber,), daemon=True)
        self.imu_thread.start()
        print('imu thread activated')

        # motor thread
        self.motor_control = robotmotor(
            'motor01', 'motor02', 'motor11', 'motor12', 'wheel1', 'wheel2')
        self.motor_control.lockleg(angle01=angle01, angle02=angle02)
        # self.motor_thread = threading.Thread(
        #     target=self.motor_control.motor_loop, daemon=True)
        # self.motor_thread.start()
        # print('motor thread activated')

        # lqr thread
        time.sleep(2)
        self.lqr_thread = threading.Thread(target=self.update_u, daemon=True)
        self.run_lqr_flag = True
        self.lqr_thread.start()
        print('lqr thread activated')

        # ros2 publisher
        self.X_arr_pub = self.create_publisher(Float32MultiArray, 'X_arr', 1)
        self.u_pub = self.create_publisher(Float32, 'u', 1)

    def closeSystem(self):
        self.motor_control.run_motor_flag = False
        self.run_lqr_flag = False
        # self.motor_thread.join()
        self.motor_control.disableALLmotor()
        print('motor thread closed')
        self.imu_thread.join()
        print('imu thread closed')
        self.lqr_thread.join()
        print('lqr thread closed')

    def update_u(self):
        X_arr_msg = Float32MultiArray()
        u_msg = Float32()
        X = np.zeros((4, 1))    # X = [x, x_dot, theta, theta_dot]
        t0 = time.time()
        count = 0
        count_drop = 0

        while self.run_lqr_flag:
            count += 1
            # get pitch data
            X_last = np.copy(X)
            X[2, 0], X[3, 0] = self.imu_subscriber.returndegree()
            if abs(X[2, 0]) > math.radians(50):
                # print('u=0')
                # print(self.motor_control.u[0, 0])
                self.motor_control.u[0, 0] = 0.0
                self.motor_control.motor_loop()
                continue

            # motor control
            self.motor_control.motor_loop()

            # get x data
            a1, a2 = self.motor_control.get_motor_feedback()
            t1 = time.time()
            dt = t1 - t0
            t0 = t1
            if a1 is None or a2 is None:
                count_drop += 1
            else:
                X[1, 0] = (float(a2[2])+(-float(a1[2])))/2 * 2 * np.pi / 60     # rpm to rad/s
                X[0, 0] = X_last[0, 0] + X[1, 0] * dt * 0.06152/2
                print('lqr freqency:', 1/dt)

            # get u from lqr
            self.motor_control.u = np.copy(self.lqr_controller.lqr_control(X))

            # pub ros2 topic
            u_msg.data = float(self.motor_control.u[0, 0])
            X_arr_msg.data = [float(X[0, 0]), float(
                X[1, 0]), float(X[2, 0]), float(X[3, 0])]
            print('drop_rate:', count_drop/count*100)
            print("X", X)
            print('u', self.motor_control.u)
            self.X_arr_pub.publish(X_arr_msg)
            self.u_pub.publish(u_msg)
            
class ImuSubscriber(Node):

    def __init__(self):
        super().__init__('imusubscriber')
        self.subscription = self.create_subscription(
            Imu, 'imu/data_raw', self.listener_callback, 1)
        self.theta_pub = self.create_publisher(Float32, 'theta', 1)
        self.pitch_middle = 0
        self.pitch_bias = 0
        self.pitch_vel = 0
        self.pitch_last = 0
        self.pitch_dot = 0
        self.dt = 1/200

    def listener_callback(self, msg):
        qua_x = msg.orientation.x
        qua_y = msg.orientation.y
        qua_z = msg.orientation.z
        qua_w = msg.orientation.w
        # *(180/math.pi)+1.5
        self.pitch = -(math.asin(2 * (qua_w * qua_y - qua_z * qua_x)) - self.pitch_bias) 
        self.pitch_dot = (self.pitch - self.pitch_last) / self.dt
        self.pitch_last = self.pitch
        # self.pitch_dot = msg.angular_velocity.y
        # print('pitch', -self.pitch, -self.pitch_dot)

    def returndegree(self):
        theta_msg = Float32()
        theta_msg.data = float(self.pitch)
        self.theta_pub.publish(theta_msg)

        return self.pitch, self.pitch_dot


class robotmotor:
    def __init__(self, motor01, motor02, motor11, motor12, wheel1, wheel2):
        self.dxl = DXL_Conmunication(device_name="/dev/dxl", b_rate=57600)
        self.mc = MotorControl(device_name="/dev/foc", baudrate=2e6)
        self.motor01 = motor01
        self.motor02 = motor02
        self.motor11 = motor11
        self.motor12 = motor12
        self.wheel1 = wheel1
        self.wheel2 = wheel2
        self.a1 = None
        self.a2 = None
        self.u = np.zeros((1, 1))
        self.createdxlmotor()
        self.run_motor_flag = True

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

    def lockleg(self, angle01=2514, angle02=3530):
        self.dxl.updateMotorData()
        self.motor01.writePosition(angle01)
        self.motor02.writePosition(angle02)
        self.motor11.writePosition(angle01)
        self.motor12.writePosition(angle02)
        self.dxl.sentAllCmd()

    def motor_loop(self):
        t0 = time.time()
        # while self.run_motor_flag:
        if self.u[0, 0] > 0.142:
            self.u[0, 0] = 0.142
        elif self.u[0, 0] < -0.142:
            self.u[0, 0] = -0.142
        outputall = int(self.u[0, 0] * 300/0.142)
        self.a1 = self.motortorquecommand(0x01, -outputall)
        self.a2 = self.motortorquecommand(0x02, outputall)
        t1 = time.time()
        # print('motor frequency:', 1/(t1-t0)) 
        t0 = t1

    def get_motor_feedback(self):
        return self.a1, self.a2

    def motortorquecommand(self, id, torque):
        # if abs(torque) >= 0.33:
        # a = self.mc.torquecontrol(id ,torque)
        a = self.mc.torquecontrol(id, torque)
        return a

    def motorspeedcommand(self, id, speed):
        a = self.mc.speedcontrol(id, speed)
        return a

    def disableALLmotor(self):
        self.dxl.disableAllMotor()
        self.mc.stopmotor(0x01)
        self.mc.stopmotor(0x02)
        time.sleep(1)
        self.dxl.closeHandler()
        self.mc.closeport()


def main():
    try:
        rclpy.init()
        controller = Controller()
        while (True):
            print('running proscess')
            time.sleep(10)

    except KeyboardInterrupt:
        rclpy.shutdown()
        controller.closeSystem()

    finally:
        rclpy.shutdown()
        controller.closeSystem()


if __name__ == '__main__':
    main()
