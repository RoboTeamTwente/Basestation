import utils

import roboteam_embedded_messages.python.REM_BaseTypes as BaseTypes
from roboteam_embedded_messages.python.REM_RobotMusicCommand import REM_RobotMusicCommand

connection = utils.openContinuous(timeout=0.01)

def cleanRMC(robot_id):
	""" Generates and returns a clean REM_RobotMusicCommand

	Args:
		robot_id (int): The id of the robot to send the command to
 
	Returns:
		REM_RobotMusicCommand: The clean REM_RobotMusicCommand
	"""
	cmd = REM_RobotMusicCommand()
	cmd.header = BaseTypes.REM_PACKET_TYPE_REM_ROBOT_MUSIC_COMMAND
	cmd.remVersion = BaseTypes.REM_LOCAL_VERSION
	cmd.id = robot_id
	return cmd

robot_id = 0

while True:
	cmd = cleanRMC()

	arg = input(f"[robot {robot_id}](folder $ song $)(next|previous)(volume up,down,$)(play|pause|stop) Command: ")

	## Song
	if arg.startswith("folder"):
		_, folderId, _, songId = arg.split(" ")
		cmd.folderId = int(folderId)
		cmd.songId = int(songId)
	if arg == "next": cmd.nextSong = 1
	if arg == "previous": cmd.previousSong = 1
	## Volume
	if arg.startswith("volume"):
		subarg = arg.split(" ")[1]
		if subarg == "up": cmd.volumeUp = 1
		elif subarg == "down": cmd.volumeDown = 1
		else: cmd.volume = int(subarg)
	## Mode
	if arg == "play": cmd.play = 1
	if arg == "pause": cmd.pause = 1
	if arg == "stop": cmd.stop = 1

	connection.write(cmd.encode())