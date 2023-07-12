import utils
import time

import roboteam_embedded_messages.python.REM_BaseTypes as BaseTypes
from roboteam_embedded_messages.python.REM_RobotKillCommand import REM_RobotKillCommand
from roboteam_embedded_messages.python.REM_BasestationConfiguration import REM_BasestationConfiguration

connection = utils.openContinuous(timeout=0.01)

def generate_command(int: robot_id) -> REM_RobotKillCommand:
	cmd = REM_RobotKillCommand()
	cmd.header = BaseTypes.REM_PACKET_TYPE_REM_ROBOT_KILL_COMMAND
	cmd.toRobotId = robot_id
	cmd.fromPC = True	
	cmd.remVersion = BaseTypes.REM_LOCAL_VERSION
	cmd.messageId = tick_counter
	cmd.payloadSize = BaseTypes.REM_PACKET_SIZE_REM_ROBOT_KILL_COMMAND
	return cmd

def generate_basestation_command(bool: yellow) -> REM_BasestationConfiguration:
	cmd = REM_BasestationConfiguration()
	cmd.header = BaseTypes.REM_PACKET_TYPE_REM_BASESTATION_CONFIGURATION
	cmd.toBS = True
	cmd.fromPC = True
	cmd.remVersion = BaseTypes.REM_LOCAL_VERSION
	cmd.payloadSize = BaseTypes.REM_PACKET_SIZE_REM_BASESTATION_CONFIGURATION
	cmd.channel = yellow
	return cmd

while True:

	for robot_id in range(16):

		# Send on the yellow channel
		bs_cmd = generate_basestation_command(True)
		connection.write(bs_cmd)
		cmd = generate_command(robot_id)
		connection.write(cmd.encode())

	for robot_id in range(16):

		# Send on the blue channel
		bs_cmd = generate_basestation_command(False)
		connection.write(bs_cmd)
		cmd = generate_command(robot_id)
		connection.write(cmd.encode())

	time.sleep(0.1)