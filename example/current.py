from dynamixel_helper import DynamixelCtrlU2D2, OperatingModes, Motors
import sys
import time

if __name__ == "__main__":
    dyn = DynamixelCtrlU2D2(
        port="/dev/ttyUSB0",
        operating_mode=OperatingModes.EXTENDED_POSITION,
        motor=Motors.X_SERIES,
    )
    id = int(sys.argv[1])
    dyn.add_motor(id=id)
    while True:
        print(dyn.get_current(id))
        time.sleep(0.5)
