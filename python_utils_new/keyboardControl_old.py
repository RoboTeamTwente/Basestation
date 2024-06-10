import argparse
import atexit
from datetime import datetime
import math
import os
import sys
import time
from enum import Enum
from pynput import keyboard
from pynput.keyboard import Key
# Add parent directory to path to allow importing from Core.Inc
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# Import roboteam embedded messages
from Core.Inc.roboteam_embedded_messages.python import REM_BaseTypes as BaseTypes
from Core.Inc.roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand
from Core.Inc.roboteam_embedded_messages.python.REM_Log import REM_Log
# Import local modules
import utils
from REMParser import REMParser

ROTATION_SPEED = 3 # radians per second
TICK_RATE = 60 # ticks per second
SPEED = 0.7 # meters per second

class Direction(Enum):
	UP = 0
	DOWN = 1
	LEFT = 2
	RIGHT = 3
	STOP = 4

class Rotation(Enum):
	CLOCKWISE = 0
	COUNTERCLOCKWISE = 1
	STOP = 2

class KickChip(Enum):
	KICK = 0
	CHIP = 1
	STOP = 2

class RobotController:
	def __init__(self, args):
		self.basestation = None
		self.direction = Direction.STOP
		self.rotation = Rotation.STOP
		self.kick_chip = KickChip.STOP
		self.kick_chip_pressed = False
		self.dribbler_pressed = False
		self.current_yaw = 0
		self.cmd = self.create_empty_robot_command()
		self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
		atexit.register(self.close_basestation)
		self.simulate = args.simulate
		self.ids = args.robot_ids

	def close_basestation(self) -> None:
		"""Closes the basestation if it's open."""
		if self.basestation is not None:
			self.basestation.close()
		print("Basestation closed, enjoy your day")

	def create_empty_robot_command(self) -> REM_RobotCommand:
		"""Creates an empty robot command with default values."""
		cmd = REM_RobotCommand()
		cmd.header = BaseTypes.REM_PACKET_TYPE_REM_ROBOT_COMMAND
		cmd.toRobotId = 0
		cmd.fromPC = True    
		cmd.remVersion = BaseTypes.REM_LOCAL_VERSION
		cmd.payloadSize = BaseTypes.REM_PACKET_SIZE_REM_ROBOT_COMMAND
		cmd.timestamp = int(time.time() * 100)
		cmd.useYaw = 1
		return cmd

	def update_command(self):
		"""Updates the robot command based on current direction, rotation, and mode."""
		direction_map = {
			Direction.UP: (SPEED, 0),
			Direction.DOWN: (SPEED, math.pi),
			Direction.LEFT: (SPEED, math.pi / 2),
			Direction.RIGHT: (SPEED, -math.pi / 2),
			Direction.STOP: (0, 0),
		}
		rotation_map = {
			Rotation.CLOCKWISE: -ROTATION_SPEED,
			Rotation.COUNTERCLOCKWISE: ROTATION_SPEED,
			Rotation.STOP: 0,
		}
		self.cmd.rho, self.cmd.theta = direction_map.get(self.direction, (0, 0))
		self.current_yaw += rotation_map.get(self.rotation, 0) / TICK_RATE
		self.cmd.yaw = self.current_yaw
		self.cmd.theta += self.current_yaw
   
		self.cmd.doKick = 0
		self.cmd.doChip = 0
		if self.kick_chip == KickChip.KICK and not self.kick_chip_pressed:
			self.cmd.doKick = 1
			self.cmd.doForce = 1
			self.cmd.kickChipPower = 6
			self.kick_chip_pressed = True
		elif self.kick_chip == KickChip.CHIP and not self.kick_chip_pressed:
			# REMOVE WHEN CHIPPING IS IMPLEMENTED
			# self.cmd.doChip = 1
			# self.cmd.doForce = 1
			self.kick_chip_pressed = True   

	def on_press(self, key: Key) -> None:
		print(key)
		"""Handles key press events to update robot command parameters."""
		prev_robot_id = self.cmd.toRobotId
		try:
			char = key.char
			self.handle_key_press(char)
			if self.cmd.toRobotId != prev_robot_id and not self.ids:
				print(f"Robot id: {self.cmd.toRobotId}")
			self.cmd.toRobotId = self.cmd.toRobotId
		except AttributeError:
			pass

	def handle_key_press(self, char: str):
		"""Handles specific character key press events."""
		if char == '-':
			self.cmd.toRobotId = (self.cmd.toRobotId - 1) % 16
		elif char in ('+', '='):
			self.cmd.toRobotId = (self.cmd.toRobotId + 1) % 16
		elif char.isdigit() and 0 <= int(char) <= 9:
			self.cmd.toRobotId = int(char)
		elif char == 'w':
			self.direction = Direction.UP
		elif char == 's':
			self.direction = Direction.DOWN
		elif char == 'a':
			self.direction = Direction.LEFT
		elif char == 'd':
			self.direction = Direction.RIGHT
		elif char == 'q':
			self.rotation = Rotation.COUNTERCLOCKWISE
		elif char == 'e':
			self.rotation = Rotation.CLOCKWISE
		elif char == 'c':
			self.kick_chip = KickChip.CHIP
		elif char == 'k':
			self.kick_chip = KickChip.KICK
		elif char == 'b' and not self.dribbler_pressed:
			self.cmd.dribbler = not self.cmd.dribbler
			self.drbbler_pressed = True

	def on_release(self, key: Key) -> None:
		print(key)
		"""Handles key release events to stop robot movement."""
		keys_that_should_stop_direction = ['w', 's', 'a', 'd']
		keys_that_should_stop_rotation = ['q', 'e']
		keys_that_should_release_kick_chip = ['c', 'k']
		try:
			if key.char in keys_that_should_stop_direction:
				self.direction = Direction.STOP
			elif key.char in keys_that_should_stop_rotation:
				self.rotation = Rotation.STOP
			elif key.char in keys_that_should_release_kick_chip:
				self.kick_chip_pressed = False
				self.kick_chip = KickChip.STOP
			elif key.char == 'b':
				self.dribbler_pressed = False
		except AttributeError:
			pass

	def handle_rem_log(self, rem_log: REM_Log):
		"""Handles REM_Log packets by printing formatted log messages."""
		log_from = "[?]  "
		if rem_log.fromBS:
			log_from = "[BS] "
		if not rem_log.fromPC and not rem_log.fromBS:
			log_from = f"[{str(rem_log.fromRobotId).rjust(2)}] "
		message = log_from + rem_log.message.strip()
		nwhitespace = os.get_terminal_size().columns - len(message) - 2
		print(f"\r{message}{' ' * nwhitespace}")

	def run(self) -> None:
		"""Starts the keyboard listener and handles robot commands and logging."""
		self.listener.start()
		print("Keyboard listener started")
		if (self.basestation is None or not self.basestation.isOpen()) and not self.simulate:
			self.basestation = utils.open_continuous(timeout=0.1)
			print("Basestation opened")
		if self.simulate:
			print("Simulation mode enabled, no basestation will be used")
		else:
			current_dir = os.path.dirname(os.path.abspath(__file__))
			log_dir = os.path.join(current_dir, "logs/keyboard")
			os.makedirs(log_dir, exist_ok=True)
			filename = datetime.now().strftime("%Y-%m-%d_%H:%M:%S") + ".rembin"
			self.logger = REMParser(self.basestation, f"{log_dir}/{filename}")

		last_tick_time = time.time()
		tick_number = 0

		while True:
			self.tick(last_tick_time)
			last_tick_time = time.time()
			tick_number += 1

	def tick(self, last_tick_time):
		"""Handles one tick of the main loop, updating and sending commands."""
		time_till_next_tick = last_tick_time + 1/TICK_RATE - time.time()
		time.sleep(max(0, time_till_next_tick))
		self.update_command()
		self.cmd.timestamp = int(time.time() * 100)
		if not self.simulate:
			# Send to all provided robot ids or just the current robot id
			for i in self.ids if self.ids else [self.cmd.toRobotId]:
				self.cmd.toRobotId = i
				self.basestation.write(self.cmd.encode())
				self.logger.write_bytes(self.cmd.encode())
			self.logger.read()
			self.logger.process()

			while self.logger.has_packets():
				packet = self.logger.get_next_packet()
				if isinstance(packet, REM_Log):
					self.handle_rem_log(packet)


if __name__ == "__main__":
	print("Use the following keys to control the robot:")
	print("w/s/a/d to move the robot (w: forward, s: backward, a: left, d: right)")
	print("q/e to rotate the robot (q: counterclockwise, e: clockwise)")
	print()
	print("0-9 to change the robot id")
	print("Press + or = to increase the robot id by 1")
	print("Press - to decrease the robot id by 1")
	print()
	print("c to chip the ball")
	print("k to kick the ball")
	print("b to toggle the dribbler")
	print()
	print("Press Ctrl+C to exit")
 
	parser = argparse.ArgumentParser()
	parser.add_argument("-s", "--simulate", action="store_true", help="Don't actually use the basestation. This can be useful for testing without a basestation present.")
	parser.add_argument("-r", "--robot_ids", nargs="+", type=int, help="List of robot ids to send commands to, changing the robot id will be disabled")
	args = parser.parse_args()

	controller = RobotController(args)
	controller.run()
