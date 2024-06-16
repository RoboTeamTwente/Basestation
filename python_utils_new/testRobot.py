import argparse
import atexit
import datetime
import math
import os
import sys
import time
import numpy as np

# Add parent directory to path to allow importing from Core.Inc
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# Import roboteam embedded messages
from Core.Inc.roboteam_embedded_messages.python import REM_BaseTypes as BaseTypes
from Core.Inc.roboteam_embedded_messages.python.REM_RobotFeedback import REM_RobotFeedback
from Core.Inc.roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand
from Core.Inc.roboteam_embedded_messages.python.REM_Log import REM_Log
from Core.Inc.roboteam_embedded_messages.python.REM_RobotStateInfo import REM_RobotStateInfo
from visualization import visualize
# Import local modules
from REMParser import REMParser
import utils

basestation = None

def rotate(origin, point, angle):
	ox, oy = origin
	px, py = point

	qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
	qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
	return qx, qy

def close_basestation() -> None:
	"""
	Closes the basestation on exit.
	"""
	global basestation
	if basestation is not None:
		basestation.close()
		print("Basestation closed, enjoy your day")

atexit.register(close_basestation)

def create_robot_command(test: str, tick_number: int) -> REM_RobotCommand:
	"""
	Creates a robot command for a given robot ID.

	Args:
		test (str): The test to run.
		tick_number (int): The current tick number.

	Returns:
		REM_RobotCommand: The created robot command.
	"""
	cmd = utils.generate_empty_robot_command()
	if test == "nothing":
		cmd.rho = 0
		cmd.theta = 0
		cmd.angularVelocity = 0
		cmd.useYaw = False
	elif test == "kicker":
		if tick_number % 120 < 10:
			cmd.doKick = 1
			cmd.doForce = 1 # Ignore ball sensor
			cmd.kickChipPower = 6
	elif test == "chipper":
		if tick_number % 120 < 10:
			cmd.doChip = 1
			cmd.doForce = 1
			cmd.kickChipPower = 6
	elif test == "dribbler":
		cmd.dribblerOn = 1
	elif test == "rotate":
		# Full rotation every 2 seconds
		cmd.yaw = -math.pi + 2 * math.pi * ((tick_number / 120 + 0.5) % 1)
	elif test == "forward":
		cmd.rho = 0.3 - 0.3 * math.cos( 4 * math.pi * tick_number / 120 )
		cmd.theta = -math.pi if tick_number % 120 < 60 else 0
	elif test == "sideways":
		cmd.theta = math.pi/2
		cmd.rho = 0.3 - 0.3 * math.cos( 4 * math.pi * tick_number / 120 )
		cmd.theta = -math.pi/2 if tick_number % 120 < 60 else math.pi/2
	elif test == "rotate-discrete":
		cmd.yaw = -math.pi + math.pi/2 * (int(tick_number / 30) % 4)
	elif test == "angular-velocity":
		cmd.angularVelocity = math.pi
		cmd.useYaw = False
	elif test == "circle":
		cmd.rho = 1
		cmd.theta = 2 * math.pi * tick_number / 240
	elif test == "circle-forward":
		# move in a circle while facing forward
		cmd.rho = 1
		cmd.theta = 2 * math.pi * tick_number / 240
		cmd.yaw = 2 * math.pi * tick_number / 240


	return cmd

def parse_and_process_args() -> argparse.Namespace:
	"""
	Parse command line arguments and process related logic.
	"""
	global basestation
	testsAvailable = ["nothing", "kicker", "chipper", "dribbler", "rotate", "forward", "sideways", "rotate-discrete", "angular-velocity", "circle", "circle-forward"]
	parser = argparse.ArgumentParser()
	parser.add_argument("robot_ids", type=int, nargs='+', help="An array of integers for the robot ids")
	parser.add_argument("test", choices=testsAvailable, default="nothing", help="Specify which test to run. Default is 'nothing'.")
	parser.add_argument('--output-dir', '-d', help="REMParser output directory. Logs will be placed under 'logs/OUTPUT_DIR'")
	args = parser.parse_args()
 
	if (basestation is None or not basestation.isOpen()):
		basestation = utils.open_continuous(timeout=0.1)
		print("Basestation opened")

	return args

def main() -> None:
	"""
    Main function
    """
	global basestation
	args = parse_and_process_args()
	datetime_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
	output_file = None
	if args.output_dir is not None:
		os.makedirs(f"logs/{args.output_dir}", exist_ok=True)
		output_file = f"logs/{args.output_dir}/log_{datetime_str}.bin"
	parser = REMParser(basestation, output_file=output_file)
	last_tick_time = 0
	tick_number = 0
	last_packet_feedback = None
	last_packet_state_info = None
	latest_feedback_time = time.time()
	image_vis = np.zeros((500, 500, 3), dtype=float)
	while True:
		if time.time() - latest_feedback_time > 1:
			print("No feedback received in the last second")
		cmd = create_robot_command(args.test, tick_number)
		time_till_next_tick = last_tick_time + 1/60 - time.time()
		time.sleep(max(0,time_till_next_tick))
		last_tick_time = time.time()
		for robot_id in args.robot_ids:
			cmd.toRobotId = robot_id
			basestation.write(cmd.encode())
			parser.write_bytes(cmd.encode())
		parser.read()
		parser.process()
		while parser.has_packets():
			packet = parser.get_next_packet()
			if isinstance(packet, REM_RobotFeedback):
				# print(f"Received feedback from robot {packet.fromRobotId}")
				last_packet_feedback = packet
				latest_feedback_time = time.time()
			elif isinstance(packet, REM_RobotStateInfo):
				# print(f"Received state info from robot {packet.fromRobotId}")
				last_packet_state_info = packet
			elif isinstance(packet, REM_Log):
				print(packet.message)

		tick_number += 1

		# ========== VISUALISING ========== #
		visualize(args, image_vis, last_packet_feedback, last_packet_state_info, cmd)
     
if __name__ == "__main__":
	main()