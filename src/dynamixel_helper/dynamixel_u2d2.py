"""A module to control the dynamixel"""

import warnings
from enum import Enum

from .vendor.dynamixel_sdk.src.dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    COMM_SUCCESS,
)

DXL_MINIMUM_POSITION_VALUE: int = 0
DXL_MAXIMUM_POSITION_VALUE: int = 4095
BAUDRATE: int = 57600
PROTOCOL_VERSION: int = 2
DEVICENAME: str = "/dev/ttyUSB0"
TORQUE_ENABLE: int = 1
TORQUE_DISABLE: int = 0


class Motors(Enum):
    X_SERIES = {
        "ADDR_TORQUE_ENABLE": 64,
        "ADDR_GOAL_POSITION": 116,
        "ADDR_PRESENT_POSITION": 132,
        "ADDR_HOMING_OFFSET": 20,
    }
    MX_SERIES = {
        "ADDR_TORQUE_ENABLE": 24,
        "ADDR_GOAL_POSITION": 596,
        "ADDR_PRESENT_POSITION": 36,
    }
    PRO_SERIES = {
        "ADDR_TORQUE_ENABLE": 562,
        "ADDR_GOAL_POSITION": 564,
        "ADDR_PRESENT_POSITION": 596,
    }

    def __init__(self, control_table):
        self.control_table = control_table

    def get_torque_addr(self) -> int:
        return self.control_table["ADDR_TORQUE_ENABLE"]

    def get_goal_addr(self) -> int:
        return self.control_table["ADDR_GOAL_POSITION"]

    def get_position_addr(self) -> int:
        return self.control_table["ADDR_PRESENT_POSITION"]

    def get_homing_addr(self) -> int:
        return self.control_table["ADDR_HOMING_OFFSET"]


class OperatingModes(Enum):
    POSITION = 3
    EXTENDED_POSITION = 4


class DynamixelCtrlU2D2:
    def __init__(
        self,
        motor: Motors = Motors.X_SERIES,
        operating_mode: OperatingModes = OperatingModes.EXTENDED_POSITION,
        port: str = DEVICENAME,
        protocol_version: int = PROTOCOL_VERSION,
        baudrate: int = BAUDRATE,
        moving_threshold: int = 20,
    ):
        """
        Initializes the Dynamixel control object.

        Args:
            port (str): The serial port name to use for communication with the Dynamixel servo.
            protocol_version (float): The protocol version of the Dynamixel servo.
            baud-rate (int): The baud-rate to use for communication with the Dynamixel servo.
        """
        self.used_dxls: set[int] = set()
        self.OPERATING_MODE: int = operating_mode.value
        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(protocol_version)
        self.ADDR_TORQUE_ENABLE = motor.get_torque_addr()
        self.ADDR_GOAL_POSITION = motor.get_goal_addr()
        self.ADDR_PRESENT_POSITION = motor.get_position_addr()
        self.ADDR_HOMING_OFF = motor.get_homing_addr()
        self.ADDR_HOMING_OFF = 20
        self.DXL_MOVING_STATUS_THRESHOLD = moving_threshold
        self.ADDR_OPERATING_MODE = 11

        self.DXL_ORIGIN_POS = (1 + 4065) // 2
        if not self.portHandler.openPort():
            raise OSError(f"Failed to open the port {port}")
        if not self.portHandler.setBaudRate(baudrate):
            raise OSError("Failed to change the baudrate")
        if self.OPERATING_MODE == 4:
            self.DXL_ORIGIN_POS = 0

    def set_homing_offset(self, id, homing_offset: int):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, id, self.ADDR_HOMING_OFF, homing_offset
        )
        if not self._handle_dxl_errors(dxl_comm_result, dxl_error):
            warnings.warn(f"Unable reach the Dynamixel id {id}", RuntimeWarning)

    def position_reached(self, id: int, pos: int):
        """
        Waits until the Dynamixel servo reaches the specified position.

        Args:
            id (int): The ID of the Dynamixel servo to control.
            pos (int): The position to move the Dynamixel servo to.
        """
        while 1:
            if not abs(pos - self.get_goal(id)) > self.DXL_MOVING_STATUS_THRESHOLD:
                break

    def set_goal(self, id: int, pos: int, block_thread=False) -> None:
        """
        Sends a command to move the specified Dynamixel servo to the specified position.

        Args:
            id (int): The ID of the Dynamixel servo to control.
            pos (int): The position to move the Dynamixel servo to.
            block_thread (int): If true waits till dynamixel reaches the goal position
        """
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, id, self.ADDR_GOAL_POSITION, pos
        )
        if not self._handle_dxl_errors(dxl_comm_result, dxl_error):
            warnings.warn("Unable to get position", RuntimeWarning)
        if block_thread:
            self.position_reached(id, pos)

    def add_motor(self, id: int, center=False):
        """
        Initializes the specified Dynamixel servo.

        Args:
            id (int): The ID of the Dynamixel servo to initialize.
        """
        self._set_opmode(id)
        if center:
            home = self.get_goal(id=id)
            home += 522239 - home
            self.set_homing_offset(id, home)

        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, id, self.ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        if self._handle_dxl_errors(dxl_comm_result, dxl_error):
            # print("Dynamixel has been successfully connected")
            # self.resetDxl(id)
            self.used_dxls.add(id)
        else:
            warnings.warn(f"Unable reach the Dynamixel id {id}", RuntimeWarning)

    def _set_opmode(self, id):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, id, self.ADDR_OPERATING_MODE, self.OPERATING_MODE
        )
        if not self._handle_dxl_errors(dxl_comm_result, dxl_error):
            warnings.warn(f"Unable reach the Dynamixel id {id}", RuntimeWarning)

    def reset_dxl(self, id: int):
        """
        Resets the specified Dynamixel servo to its origin position.

        Args:
            id (int): The ID of the Dynamixel servo
        """
        self.set_goal(id, self.DXL_ORIGIN_POS)

    def get_goal(self, id: int) -> int:
        """
        This method is used to get the current position of a Dynamixel motor with the given ID. It takes the ID of the motor as an integer parameter and returns the current position of the motor as an integer value.
        If there is an error while attempting to get the position of the motor,
        a warning will be raised.
        """
        dxl_present_position, dxl_comm_result, dxl_error = (
            self.packetHandler.read4ByteTxRx(
                self.portHandler, id, self.ADDR_PRESENT_POSITION
            )
        )
        if not self._handle_dxl_errors(dxl_comm_result, dxl_error):
            warnings.warn(f"Get position failed on Dynamixel id {id}", RuntimeWarning)
        return dxl_present_position

    def _handle_dxl_errors(self, dxl_comm_result, dxl_error) -> bool:
        """
        This method is used to handle communication errors with a Dynamixel motor. It takes two integer parameters: dxl_comm_result and dxl_error. dxl_comm_result indicates the communication result returned by the Dynamixel SDK and dxl_error indicates the error code returned by the Dynamixel motor.
        If there is an error, a warning will be raised and the method will
        return False. Otherwise, the method will return True.
        """
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            return True
        return False

    def __del__(self):
        for i in self.used_dxls:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, i, self.ADDR_TORQUE_ENABLE, TORQUE_DISABLE
            )
            if not self._handle_dxl_errors(dxl_comm_result, dxl_error):
                warnings.warn("Unable to close the Dynamixel", RuntimeWarning)
        self.portHandler.closePort()
