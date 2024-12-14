from dynamixel_helper import DynamixelCtrlU2D2,OperatingModes,Motors

if __name__ == '__main__':
    dyn = DynamixelCtrlU2D2(port="/dev/ttyUSB0",operating_mode=OperatingModes.EXTENDED_POSITION,motor=Motors.X_SERIES)
    dyn.add_motor(id=1)
    print(f"Current Position {dyn.get_goal(id=1)}")
    dyn.set_goal(id=1, pos=8000, block_thread=True)
    print(f"Current Position {dyn.get_goal(id=1)}")
    dyn.set_goal(id=1, pos=0, block_thread=True)
    print(f"Current Position {dyn.get_goal(id=1)}")
