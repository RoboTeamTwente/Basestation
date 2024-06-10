import argparse
import atexit
import os
import sys
import time
from typing import List
# Add parent directory to path to allow importing from Core.Inc
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# Import roboteam embedded messages
from Core.Inc.roboteam_embedded_messages.python import REM_BaseTypes as BaseTypes
from Core.Inc.roboteam_embedded_messages.python.REM_RobotFeedback import REM_RobotFeedback
from Core.Inc.roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand
from Core.Inc.roboteam_embedded_messages.python.REM_Log import REM_Log
# Import local modules
from REMParser import REMParser
import utils

basestation = None

def close_basestation() -> None:
	"""
	Closes the basestation on exit.
	"""
	global basestation
	if basestation is not None:
		basestation.close()
		print("Basestation closed, enjoy your day")

atexit.register(close_basestation)

def create_robot_command(robot_id: int) -> REM_RobotCommand:
	"""
	Creates an empty robot command for a given robot ID.

	Args:
		robot_id (int): The ID of the robot.

	Returns:
		REM_RobotCommand: The created robot command.
	"""
	cmd = utils.generate_empty_robot_command()
	cmd.toRobotId = robot_id
	cmd.feedback = 1
	return cmd

def print_feedback(feedback_last_second: List[int], tick_number: int) -> List[int]:
	"""
	Print feedback and reset feedback counts.
	"""
	hours, remainder = divmod(tick_number//60, 3600)
	minutes, seconds = divmod(remainder, 60)
	print(f"\033[2J\033[0;0H{hours:02}:{minutes:02}:{seconds:02}")
	for i in range(4):
		print("".join([f"\033[38;2;{255*(1-feedback_last_second[j]/30):.0f};{255*feedback_last_second[j]/30:.0f};0m{j:02} ({feedback_last_second[j]:02})\033[0m".ljust(30) for j in range(i, 16, 4)]))
	print()
	return [0] * 16

def parse_and_process_args() -> argparse.Namespace:
	"""
	Parse command line arguments and process related logic.
	"""
	global basestation
	parser = argparse.ArgumentParser()
	parser.add_argument("--send", action="store_true", help="Actually send commands to the robots. If not set, this will only listen for feedback. Note that the robots won't send feedback if they don't receive any commands. If you want to use this without --send, make sure something else is sending commands (like AI during a game). Using --send while someone else is also sending commands, will result in interference.")
	parser.add_argument("--simulate", action="store_true", help="Don't actually use the basestation. This can be useful for testing without a basestation present.")
	parser.add_argument("--team", choices=["yellow", "blue"], default="yellow", help="Specify which team's robots to send commands to. Options are 'yellow' or 'blue'. Default is 'yellow'.")
	args = parser.parse_args()

	if not args.send:
		print("Not sending commands. Listening for feedback only.")
	else:
		print("Sending commands and listening for feedback.")
	if args.simulate:
		print("Not using basestation. No commands will be sent.")

	if (basestation is None or not basestation.isOpen()) and not args.simulate:
		basestation = utils.open_continuous(timeout=0.1)
		print("Basestation opened")

	return args

def process_parser_packets(parser: REMParser, feedback_last_second: List[int]) -> List[int]:
    """
    Process packets from the parser and update feedback counts.
    """
    parser.read()
    parser.process()
    while parser.has_packets():
        packet = parser.get_next_packet()
        if isinstance(packet, REM_RobotFeedback):
            feedback_last_second[packet.fromRobotId] += 1
        if isinstance(packet, REM_Log):
            print(packet.message)
    return feedback_last_second

def main() -> None:
	"""
    Main function for the testRobotsAlive script.

    This function parses command line arguments, initializes the parser, and enters a loop where it sends commands to robots and processes feedback packets. 
    It sends commands to odd-numbered robots when the tick counter is odd and to even-numbered robots when the tick counter is even, as the basestation can only handle <=11 robots at once.
    It also prints feedback every second.
    """
	global basestation
	args = parse_and_process_args()
	parser = REMParser(basestation) if not args.simulate else None
	feedback_last_second: List[int] = [0] * 16
	last_tick_time = 0
	tick_number = 0
	# basestation_config_command = utils.generate_basestation_config_command(args.team == "yellow")
	while True:
		# basestation.write(basestation_config_command.encode())
		time_till_next_tick = last_tick_time + 1/60 - time.time()
		time.sleep(max(0,time_till_next_tick))
		last_tick_time = time.time()
		for i in range(0, 16):
			if args.send and tick_number % 2 == i % 2:
				cmd = create_robot_command(i)
				if not args.simulate:
					basestation.write(cmd.encode())
					parser.write_bytes(cmd.encode())
		if not args.simulate:
			feedback_last_second = process_parser_packets(parser, feedback_last_second)
		tick_number += 1
		if tick_number % 60 == 0:
			feedback_last_second = print_feedback(feedback_last_second, tick_number)

if __name__ == "__main__":
	main()