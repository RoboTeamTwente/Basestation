import argparse
import atexit
import os
import sys
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Core.Inc.roboteam_embedded_messages.python import REM_BaseTypes as BaseTypes
from Core.Inc.roboteam_embedded_messages.python.REM_RobotSetPIDGains import REM_RobotSetPIDGains
from REMParser import REMParser
from getPid import process_parser_packets
import utils

basestation = None

def close_basestation() -> None:
	"""Closes the basestation on exit."""
	global basestation
	if basestation is not None:
		basestation.close()
		print("Basestation closed, enjoy your day")

atexit.register(close_basestation)

def create_set_PID_Command() -> REM_RobotSetPIDGains:
	"""Create a new setPID command with the given parameters."""
	setPID = utils.generate_empty_set_pid_gains()

	setPID.PbodyX = 1.5
	setPID.IbodyX = 0.0
	setPID.DbodyX = 0.0
	setPID.PbodyY = 1.5
	setPID.IbodyY = 0.0
	setPID.DbodyY = 0.0
	setPID.PbodyW = 0.0
	setPID.IbodyW = 0.0
	setPID.DbodyW = 0.0
	setPID.PbodyYaw = 26
	setPID.IbodyYaw = 5
	setPID.DbodyYaw = 0
	setPID.Pwheels = 2
	setPID.Iwheels = 0
	setPID.Dwheels = 0
 
	return setPID

def parse_and_process_args() -> argparse.Namespace:
	"""Parse command line arguments and process related logic."""
	global basestation
	parser = argparse.ArgumentParser()
	parser.add_argument("robot_ids", type=int, nargs='+', help="An array of integers for the robot ids")
	args = parser.parse_args()

	if (basestation is None or not basestation.isOpen()):
		basestation = utils.open_continuous(timeout=0.1)
		print("Basestation opened")

	return args

def main() -> None:
	"""Main function for the setPid script."""
	global basestation
	args = parse_and_process_args()
	parser = REMParser(basestation)
	last_tick_time = 0
	while True:
		time_till_next_tick = last_tick_time + 1/10 - time.time()
		time.sleep(max(0,time_till_next_tick))
		last_tick_time = time.time()
		for robot_id in args.robot_ids:
			cmd = create_set_PID_Command(robot_id)
			basestation.write(cmd)
			parser.write_bytes(cmd.encode())
			process_parser_packets(parser)

if __name__ == "__main__":
	main()