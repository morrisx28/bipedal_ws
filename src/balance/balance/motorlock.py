import time
from module.DXL_motor_control import DXL_Conmunication

dxl = DXL_Conmunication(device_name="/dev/dxl", b_rate=57600)


def disableALLmotor():

    dxl.disableAllMotor()
    dxl.closeHandler()


def main():

    motor01 = dxl.createMotor("motor01", 1)
    motor02 = dxl.createMotor("motor02", 2)
    motor11 = dxl.createMotor("motor11", 11)
    motor12 = dxl.createMotor("motor12", 12)
    dxl.addAllBuckPrarmeter()
    motor01.switchMode('position')
    motor02.switchMode('position')
    motor11.switchMode('position')
    motor12.switchMode('position')
    motor01.enableMotor()
    motor02.enableMotor()
    motor11.enableMotor()
    motor12.enableMotor()
    dxl.updateMotorData()
    time.sleep(2)
    motor01.writePosition(2760)  # 2760-1795
    motor02.writePosition(3795)  # 3800-2900
    motor11.writePosition(2760)
    motor12.writePosition(3795)
    dxl.sentAllCmd()

    try:

        while True:
            pass

    except KeyboardInterrupt:

        disableALLmotor()


if __name__ == '__main__':

    main()
