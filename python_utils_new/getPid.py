import argparse
import atexit
import os
import sys
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Core.Inc.roboteam_embedded_messages.python import REM_BaseTypes as BaseTypes
from Core.Inc.roboteam_embedded_messages.python.REM_RobotGetPIDGains import REM_RobotGetPIDGains
from Core.Inc.roboteam_embedded_messages.python.REM_RobotPIDGains import REM_RobotPIDGains
from REMParser import REMParser
import utils

basestation = None

def close_basestation() -> None:
	"""Closes the basestation on exit."""
	global basestation
	if basestation is not None:
		basestation.close()
		print("Basestation closed, enjoy your day")

atexit.register(close_basestation)

def create_robot_command() -> REM_RobotGetPIDGains:
	"""Create a new getPID command with the given parameters."""
	cmd = utils.generate_empty_get_pid_gains()
	return cmd

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

def process_parser_packets(parser: REMParser) -> None:
	"""Process packets from the parser and update feedback counts."""
	parser.read()
	parser.process()
	while parser.has_packets():
		packet = parser.get_next_packet()
		if isinstance(packet, REM_RobotPIDGains):
			print(f"Received PID gains for robot {packet.fromRobotId}")
			print(f"PbodyU: {packet.PbodyX}, IbodyU: {packet.IbodyX}, DbodyU: {packet.DbodyX}")
			print(f"PbodyV: {packet.PbodyY}, IbodyV: {packet.IbodyY}, DbodyV: {packet.DbodyY}")
			print(f"PbodyW: {packet.PbodyW}, IbodyW: {packet.IbodyW}, DbodyW: {packet.DbodyW}")
			print(f"PbodyYaw: {packet.PbodyYaw}, IbodyYaw: {packet.IbodyYaw}, DbodyYaw: {packet.DbodyYaw}")
			print(f"Pwheels: {packet.Pwheels}, Iwheels: {packet.Iwheels}, Dwheels: {packet.Dwheels}")
			print()

def main() -> None:
	"""Main function for the getPid script."""
	global basestation
	args = parse_and_process_args()
	parser = REMParser(basestation)
	last_tick_time = 0
	while True:
		time_till_next_tick = last_tick_time + 1/60 - time.time()
		time.sleep(max(0,time_till_next_tick))
		last_tick_time = time.time()
		cmd = create_robot_command()
		for robot_id in args.robot_ids:
			cmd.toRobotId = robot_id
			basestation.write(cmd)
			parser.write_bytes(cmd.encode())
			process_parser_packets(parser)

if __name__ == "__main__":
	main()