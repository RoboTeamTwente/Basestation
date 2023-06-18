import enum
import math
import os
import time
from datetime import datetime

import utils
from REMParser import REMParser
from roboteam_embedded_messages.python import REM_BaseTypes
from roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand


class CalibrationDirections(enum.Enum):
    """
    What actions a robot needs to take in order to be calibrated
    """
    FORWARD = 0
    BACKWARDS = 1
    LEFT = 2
    RIGHT = 3
    CW = 4
    CCW = 5
    STOP = 6


def generate_packet(direction: CalibrationDirections, tick_counter: int) -> REM_RobotCommand:
    """
    Generates a new direction packet.

    :param direction Where the robot should drive to
    :param tick_counter Used for adding a time stamp into the packet
    """

    # Create new empty robot command. Fill required fields
    cmd = REM_RobotCommand()
    cmd.header = REM_BaseTypes.REM_PACKET_TYPE_REM_ROBOT_COMMAND
    cmd.toRobotId = robot_id
    cmd.fromPC = True
    cmd.remVersion = REM_BaseTypes.REM_LOCAL_VERSION
    cmd.messageId = tick_counter
    cmd.payloadSize = REM_BaseTypes.REM_PACKET_SIZE_REM_ROBOT_COMMAND

    cmd.useAbsoluteAngle = 1

    if direction == CalibrationDirections.FORWARD:
        cmd.rho = 2
        cmd.angle = 0
    elif direction == CalibrationDirections.BACKWARDS:
        cmd.rho = 2
        cmd.angle = math.pi
    elif direction == CalibrationDirections.RIGHT:
        cmd.rho = 2
        cmd.angle = math.pi * 0.5
    elif direction == CalibrationDirections.LEFT:
        cmd.rho = 2
        cmd.angle = math.pi * 1.5
    elif direction == CalibrationDirections.CCW:
        cmd.rho = 0
        cmd.useAbsoluteAngle = 0
        cmd.angularVelocity = 2 * math.pi
    elif direction == CalibrationDirections.CW:
        cmd.rho = 0
        cmd.useAbsoluteAngle = 0
        cmd.angularVelocity = -2 * math.pi
    else:
        cmd.rho = 0
        cmd.angularVelocity = 0

    return cmd


robot_id = 0        # The id of the robot
basestation = None  # The basestation that sends the commands
parser = None       # The parser that will store the commands and collects the robot feedback

period_length = 150 # How long should one test take?
packetHz = 60       # How often should a packet be sent?
tick_counter = 0    # How many ticks have occurred?

current_calibration = CalibrationDirections.FORWARD

# INITIALIZATION
last_tick_time = time.time()

# Connect basestation
if basestation is None or not basestation.isOpen():
    basestation = utils.openContinuous(timeout=0.01)
    print("Basestation opened")
    if parser is not None:
        parser.device = basestation

# Open writer / parser
if parser is None and basestation is not None:
    datetime_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    os.makedirs(f"logs/robot_calibration", exist_ok=True)
    output_file = f"logs/robot_calibration/log_{datetime_str}.bin"

    parser = REMParser(basestation, output_file=output_file)

# LOOP
while True:
    current_time = time.time()
    s_until_next_tick = last_tick_time + 1. / packetHz - current_time
    tick_required = s_until_next_tick < 0

    # Release resources to the CPU since there is no need to check this too often.
    if not tick_required and .1 / packetHz < s_until_next_tick:
        time.sleep(.1 / packetHz)

    # WRITING
    if tick_required:
        last_tick_time += 1./packetHz
        tick_counter += 1
        period = tick_counter % period_length
        period_nr = tick_counter // period_length

        # Check whether we have to switch to a new calibration
        if period == 0:
            if period_nr >= len(list(CalibrationDirections)):
                break
            current_calibration = list(CalibrationDirections)[period_nr]
            input(f"Next calibration is about to start: {current_calibration.name}, press enter to continue.")

        # Create a new robot command
        cmd = generate_packet(current_calibration, tick_counter)
        encoded_cmd = cmd.encode()

        # Send the command
        basestation.write(encoded_cmd)
        parser.writeBytes(encoded_cmd)

    # READING
    parser.read()
    parser.process()

    # Handle and store all incoming packets
    while parser.hasPackets():
        packet = parser.getNextPacket()