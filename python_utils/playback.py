import argparse
import os
import sys
import time 
from typing import List

import utils
from REMParser import REMParser

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Core.Inc.roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand

def parse_args() -> argparse.Namespace:
	"""
	Parse command line arguments.

	Returns:
		argparse.Namespace: The parsed command line arguments.
	"""
	argparser = argparse.ArgumentParser()
	argparser.add_argument('--input_file', help='File to parse, which contains REM packets', default='latest.rembin')
	return argparser.parse_args()

def parse_file(file_path: str) -> List[REM_RobotCommand]:
	"""
	Parse the given file and return a list of RobotCommands.

	Args:
		file_path (str): The path to the file to parse.

	Returns:
		List[REM_RobotCommand]: A list of parsed RobotCommands.
	"""
	print("Parsing file", file_path)
	parser = REMParser(device=None)
	parser.parse_file(file_path)
	return [ packet for packet in parser.packet_buffer if type(packet) == REM_RobotCommand ]

def adjust_timestamps(commands: List[REM_RobotCommand], start_time: float) -> None:
	"""
	Adjust timestamps of commands to start from current time.

	Args:
		commands (List[REM_RobotCommand]): The list of commands to adjust.
		start_time (float): The start time to adjust from.
	"""
	t_now = round((time.time() + 1) * 1000)
	for command in commands:
		command.timestamp = command.timestamp - start_time + t_now

def send_commands(commands: List[REM_RobotCommand]) -> None:
	"""
	Send commands to the robot.

	Args:
		commands (List[REM_RobotCommand]): The list of commands to send.
	"""
	serial = utils.open_continuous(timeout=0.001)
	rc_index = 0
	while True and rc_index < len(commands):
		time.sleep(1/60)
		serial.write(commands[rc_index].encode())
		rc_index += 1

def main() -> None:
	"""
	Main function to parse the input file and send commands to the robot.
	"""
	args = parse_args()
	robot_commands = parse_file(args.input_file)
	if not robot_commands:
		print("No RobotCommands found in the input file.")
		return
	start_time, stop_time = robot_commands[0].timestamp, robot_commands[-1].timestamp
	print("Start time: ", time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(start_time/1000)))
	print("Stop time: ", time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(stop_time/1000)))
	adjust_timestamps(robot_commands, start_time)
	send_commands(robot_commands)

if __name__ == "__main__":
	main()
	print("Playback completed.")