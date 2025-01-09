from enum import (IntEnum, auto)

#region RoboRio Constants
# included to help with communication and readability
class Rio_DIO(IntEnum):
    ZERRO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()
    FOUR = auto()
    FIVE = auto()
    SIX = auto()
    SEVEN = auto()
    EIGHT = auto()
    NINE = auto()
    # TODO figure out how CAN works..
    TEN = auto()
    ELEVEN = auto()
    TWELVE = auto()
    THIRTEEN = auto()
    FOURTEEN = auto()
    FIFTEEN = auto()
    SIXTEEN = auto()
    SEVENTEEN = auto()

class Rio_Pnue(IntEnum):
    ZERRO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()
    FOUR = auto()
    FIVE = auto()
    SIX = auto()
    SEVEN = auto()


class Rio_PWM(IntEnum):
    ONE = 0
    TWO = auto()
    THREE = auto()
    FOUR = auto()
    FIVE = auto()
    SIX = auto()
    SEVEN = auto()
    EIGHT = auto()
    NINE = auto()
    TEN = auto()

class Rio_Relay(IntEnum):
    ZERO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()

class Rio_Analog(IntEnum):
    ZERO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()
#endregion

#region CAN Constants
class CAN_Address(IntEnum):
    ZERRO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()
    FOUR = auto()
    FIVE = auto()
    SIX = auto()
    SEVEN = auto()
    EIGHT = auto()
    NINE = auto()
    TEN = auto()
    ELEVEN = auto()
    TWELVE = auto()

#endregion

class DriveConstant:
    # used Phoenix Tuner to get addresses on 1/9/25
    #shooter angler motor CAN_Address.TEN
    #PDP CAN_Address.ZERO
    kLeftMotor1Port = CAN_Address.ONE
    kLeftMotor2Port = CAN_Address.THREE
    kRightMotor1Port = CAN_Address.TWO
    kRightMotor2Port = CAN_Address.FOUR
    kWheelBase = 0.5 #meters
    kTrackWidth = 0.5 #meters
    ...

class OIConstant:
    kDriver1ControllerPort = 0
    kDriver2ControllerPort = 1