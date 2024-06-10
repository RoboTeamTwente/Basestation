import argparse
import atexit
import os
import sys
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Core.Inc.roboteam_embedded_messages.python import REM_BaseTypes as BaseTypes
from Core.Inc.roboteam_embedded_messages.python.REM_RobotSetPIDGains import REM_RobotSetPIDGains
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

def create_set_PID_Command(robot_id: int, PbodyX: float = 0.2, IbodyX: float = 0.0, DbodyX: float = 0.0, PbodyY: float = 0.3, IbodyY: float = 0.0, DbodyY: float = 0.0, PbodyW: float = 0.25, IbodyW: float = 5.0, DbodyW: float = 0.0, PbodyYaw: float = 20.0, IbodyYaw: float = 5.0, DbodyYaw: float = 0.0, Pwheels: float = 2.0, Iwheels: float = 0.0, Dwheels: float = 0.0) -> REM_RobotSetPIDGains:
	"""Create a new setPID command with the given parameters."""
	setPID = utils.generate_empty_set_pid_gains()
	setPID.fromPC = True

	setPID.PbodyX = PbodyX
	setPID.IbodyX = IbodyX
	setPID.DbodyX = DbodyX
	setPID.PbodyY = PbodyY
	setPID.IbodyY = IbodyY
	setPID.DbodyY = DbodyY
	setPID.PbodyW = PbodyW
	setPID.IbodyW = IbodyW
	setPID.DbodyW = DbodyW
	setPID.PbodyYaw = PbodyYaw
	setPID.IbodyYaw = IbodyYaw
	setPID.DbodyYaw = DbodyYaw
	setPID.Pwheels = Pwheels
	setPID.Iwheels = Iwheels
	setPID.Dwheels = Dwheels

	return setPID

def parse_and_process_args() -> argparse.Namespace:
	"""Parse command line arguments and process related logic."""
	global basestation
	parser = argparse.ArgumentParser()
	parser.add_argument("--simulate", action="store_true", help="Don't actually use the basestation. This can be useful for testing without a basestation present.")
	parser.add_argument("--team", choices=["yellow", "blue"], default="yellow", help="Specify which team's robots to send commands to. Options are 'yellow' or 'blue'. Default is 'yellow'.")
	parser.add_argument('robot_id', type=int, help='An integer for the robot id')
	args = parser.parse_args()

	if args.simulate:
		print("Not using basestation. No commands will be sent.")

	if (basestation is None or not basestation.isOpen()) and not args.simulate:
		basestation = utils.open_continuous(timeout=0.1)
		print("Basestation opened")

	return args

def main() -> None:
	"""Main function for the setPid script."""
	global basestation
	args = parse_and_process_args()
	parser = REMParser(basestation) if not args.simulate else None
	last_tick_time = 0
	while True:
		time_till_next_tick = last_tick_time + 1/60 - time.time()
		time.sleep(max(0,time_till_next_tick))
		last_tick_time = time.time()
		cmd = create_set_PID_Command(args.robot_id)
		if not args.simulate:
			basestation.write(cmd.encode())
			parser.write_bytes(cmd.encode())

if __name__ == "__main__":
	main()