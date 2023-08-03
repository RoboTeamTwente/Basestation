import utils
import time

import roboteam_embedded_messages.python.REM_BaseTypes as BaseTypes
from roboteam_embedded_messages.python.REM_RobotKillCommand import REM_RobotKillCommand
from roboteam_embedded_messages.python.REM_BasestationConfiguration import REM_BasestationConfiguration

connection = utils.openContinuous(timeout=0.01)

def generate_command(robot_id: int) -> REM_RobotKillCommand:
	cmd = REM_RobotKillCommand()
	cmd.header = BaseTypes.REM_PACKET_TYPE_REM_ROBOT_KILL_COMMAND
	cmd.toRobotId = robot_id
	cmd.fromPC = True	
	cmd.remVersion = BaseTypes.REM_LOCAL_VERSION
	cmd.payloadSize = BaseTypes.REM_PACKET_SIZE_REM_ROBOT_KILL_COMMAND
	return cmd

def generate_basestation_command(yellow: bool) -> REM_BasestationConfiguration:
	cmd = REM_BasestationConfiguration()
	cmd.header = BaseTypes.REM_PACKET_TYPE_REM_BASESTATION_CONFIGURATION
	cmd.toBS = True
	cmd.fromPC = True
	cmd.remVersion = BaseTypes.REM_LOCAL_VERSION
	cmd.payloadSize = BaseTypes.REM_PACKET_SIZE_REM_BASESTATION_CONFIGURATION
	cmd.channel = yellow
	return cmd

while True:
	for team in [True, False]:
		bs_cmd = generate_basestation_command(team)
		connection.write(bs_cmd.encode())
		for robot_id in range(16):
			cmd = generate_command(robot_id)
			connection.write(cmd.encode())
	time.sleep(0.1)