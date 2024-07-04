import serial
import math
import time
import threading
import numpy as np



class MotorControl:
    def __init__(self, device_name, baudrate, timeout=0):
        self.DEVICE_NAME = device_name
        self.Baud = baudrate
        self.TIME_OUT = timeout
        self.ser = serial.Serial(self.DEVICE_NAME, self.Baud, timeout=timeout)

        # self.time_ess = 0
        self.command_delay = 1/2000
        while not self.ser.is_open:
            pass

    def calculate_checksum(self, data):
        return sum(data) & 0xFF

    def closeport(self):
        self.ser.close()

    def clear_buffers(self):
        if self.ser.is_open:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

    def read_serial(self):
        # while not self.stop_thread:
        if self.ser.in_waiting > 0:
            # try:
            #     data = self.ser.read(self.ser.in_waiting)
            #     try:
            #         decoded_data = data.decode('utf-8')
            #         print(f"Received data: {decoded_data}")
            #     except UnicodeDecodeError:
            #         print(f"Received non-UTF-8 data: {data}")
            # except Exception as e:
            #     print(f"Error reading data: {e}")
            try:
                data = self.ser.read(self.ser.in_waiting)
                cheaksum = self.calculate_checksum(data[5:12])
                if cheaksum == data[12]:
                    id = data[2]
                    motortamperature = data[5]
                    motorcurrent = (data[7] << 8) | data[6]
                    if motorcurrent >= 0x8000:
                        motorcurrent = motorcurrent - 0x10000
                    motorrpm = (data[9] << 8) | data[8]  # high byte|low byte
                    if motorrpm >= 0x8000:
                        motorrpm = motorrpm - 0x10000
                    # motorrpm = motorrpm / 360
                    encoder = (data[11] << 8) | data[10]  # high byte|low byte
                    motorinfo = [0, id, motorrpm/6, encoder, motorcurrent, motortamperature]
                    time.sleep(self.command_delay)
                    return (motorinfo)
                # return ("id:",id,"Position:",encoder,"rpm:",motorrpm/360,"tamperature:",motortamperature)
                else:

                    pass

            except Exception as e:
                # return ([1, f"Error reading data: {e}"])

                pass
        # else:

        #     return 1

    def startmotor(self, id):
        # self.clear_buffers()
        ALLdata = [0x3e, 0x88, id, 0x00]
        cmd_sum = self.calculate_checksum(ALLdata)
        ALLdata.append(cmd_sum)
        self.ser.write(ALLdata)
        # self.ser.flush()
        time.sleep(self.command_delay)

    def stopmotor(self, id):
        # self.clear_buffers()
        ALLdata = [0x3e, 0x81, id, 0x00]
        cmd_sum = self.calculate_checksum(ALLdata)
        ALLdata.append(cmd_sum)
        self.ser.write(ALLdata)
        # self.ser.flush()
        time.sleep(self.command_delay)

    def readmotorstate(self, id):
        ALLdata = [0x3e, 0x9c, id, 0x00]
        cmd_sum = self.calculate_checksum(ALLdata)
        ALLdata.append(cmd_sum)
        self.ser.write(ALLdata)
        self.ser.flush()
        time.sleep(self.command_delay)
        return (self.read_serial())

    def torquecontrol(self, id, torque):
        # self.clear_buffers()
        ALLdata = [0x3e, 0xa1, id, 0x02]
        cmd_sum = self.calculate_checksum(ALLdata)
        ALLdata.append(cmd_sum)
        low_byte = torque & 0xFF
        high_byte = (torque >> 8) & 0xFF
        DATA = [low_byte, high_byte]
        data_sum = self.calculate_checksum(DATA)
        DATA.append(data_sum)
        ALLdata.extend(DATA)
        self.ser.write(ALLdata)
        self.ser.flush()
        time.sleep(self.command_delay)
        return (self.read_serial())
        # if self.ser.in_waiting > 0:
        #     readdata = self.ser.read_all()
        #     print(f'ID: {id}    Data: {readdata}')

    def cleanerror(self, id):

        ALLdata = [0x3e,0x9b, id, 0x00]
        cmd_sum = self.calculate_checksum(ALLdata)
        ALLdata.append(cmd_sum)
        self.ser.write(ALLdata)
        self.ser.flush()

    def speedcontrol(self, id, speed):  # speed unit 0.01dps/

        ALLdata = [0x3e, 0xa2, id, 0x04]
        cmd_sum = self.calculate_checksum(ALLdata)
        ALLdata.append(cmd_sum)
        speed = speed*100
        low_byte = speed & 0xff
        second_byte = (speed >> 8) & 0xff
        third_byte = (speed >> 16) & 0xff
        high_byte = (speed >> 24) & 0xff
        DATA = [low_byte, second_byte, third_byte, high_byte]
        data_sum = self.calculate_checksum(DATA)
        DATA.append(data_sum)
        ALLdata.extend(DATA)
        # ALLdata = [0x3E ,0xA2 ,0x02 ,0x04 ,0xE6 ,0x40 ,0x0D ,0x03 ,0x00 ,0x50]
        self.ser.write(ALLdata)
        self.ser.flush()
        time.sleep(self.command_delay)
        return (self.read_serial())


def test():

    DEVICE_NAME = "/dev/foc"
    BAUDRATE = 2000000
    mc = MotorControl(DEVICE_NAME, BAUDRATE)

    mc.startmotor(0x01)
    mc.startmotor(0x02)
    time.sleep(1)

    try:
        #     while True:
        #         for _ in range(250):
        #             print(mc.torquecontrol(0x01, 25))
        #             print(mc.torquecontrol(0x02, 25))

        #         # time.sleep(2)
        #         print('stop\n')
        #         for _ in range(250):
        #             print(mc.torquecontrol(0x01, 0))
        #             print(mc.torquecontrol(0x02, 0))
        #         # time.sleep(2)
        #         print("start\n")

        # while True:
        #     i = 0
        #     for _ in range(200):
        #         i += 1
        #         # print(mc.speedcontrol(0x01, int(math.degrees(20))))
        #         # print(mc.speedcontrol(0x02, int(math.degrees(20))))
        #         print(mc.speedcontrol(0x01, 20*i))
        #         print(mc.speedcontrol(0x02, 20*i))

        #     # time.sleep(2)
        #     print('stop\n')

        #     for _ in range(200):
        #         i-=1
        #         print(mc.speedcontrol(0x01, 20*i))
        #         print(mc.speedcontrol(0x02, 20*i))
        #     # time.sleep(2)
        #     print("start\n")

        # print(mc.readmotorstate(0x02))
        # mc.torquecontrol(0x01, -50)
        # mc.torquecontrol(0x02, 50)
        # t0 = time.time()
        # x = 0
        # while True:
        #     A = mc.readmotorstate(0x01)
        #     B = mc.readmotorstate(0x02)
        #     print(A)
        #     if A != None and B!=None:
        #         a = A[2]*2*np.pi/60

        #         b = B[2]*2*np.pi/60
        #         print(a, b)
        #         t1 = time.time()
        #         x = x + (t1-t0)*a*0.065/2
        #         print('x', x)
        #         t0 = t1
        #     else:
        #         pass
        mc.cleanerror(0x01)
        mc.cleanerror(0x02)

    except KeyboardInterrupt:

        mc.stopmotor(0x01)
        mc.stopmotor(0x02)
        time.sleep(1)
        mc.closeport()


if __name__ == '__main__':
    test()
