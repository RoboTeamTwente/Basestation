import os
import math
import sys
import time
import threading
import argparse
from datetime import datetime
from glob import glob
from typing import Callable, Dict, List, Optional

import numpy as np
# from xbox360controller import Xbox360Controller
import pygame
from pynput import keyboard

# Add parent directory to path to allow importing from Core.Inc and REMParser
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Core.Inc.roboteam_embedded_messages.python import REM_BaseTypes as BaseTypes
from Core.Inc.roboteam_embedded_messages.python.REM_Log import REM_Log
from Core.Inc.roboteam_embedded_messages.python.REM_RobotFeedback import REM_RobotFeedback

from REMParser import REMParser
import utils

MAX_SPEED = 1.5
KICK_SPEED = 3.5
ROTATION_SPEED = 2.5 # radians per second
BASESTATION_FREQUENCY = 60 # ticks per second

parser = argparse.ArgumentParser(description='Joystick to robot controller')
parser.add_argument('-r', '--robot_ids', type=int, nargs='+', help='List of robot IDs to be controlled by the joysticks')
parser.add_argument("--simulate", action="store_true", help="Simulate the basestation")
args = parser.parse_args()
# Initialize pygame and the joystick module
pygame.init()
pygame.joystick.init()

# Check if any joysticks are connected
if pygame.joystick.get_count() == 0:
    print("No joysticks connected!")
else:
    # Initialize the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick initialized: {joystick.get_name()}")

class EventHandler:
	"""Handles events and updates the status of the system."""

	def __init__(self, shutdown: Callable[[], None]) -> None:
		self.shutdown = shutdown
		self.running = True
		self.events: List[str] = []
		self.batteryVoltages: dict = {}

	def start(self, joystick_handler: 'JoystickHandler') -> None:
		"""Starts the event handling loop."""
		self.joystick_handler = joystick_handler
		self.thread = threading.Thread(target=self.loop)
		self.thread.start()

	def record_event(self, id: int, event: str) -> None:
		"""Records a new event."""
		current_time = datetime.now().strftime("%H:%M:%S")
		id = int(id) + 1
		self.events.insert(0, f"[{id} | {current_time}] {event}")
		self.events = self.events[:5]

	def loop(self) -> None:
		"""Main event handling loop."""
		try:
			while self.running:
				string = "\r"
				for id in self.joystick_handler.controllers:
					controller = self.joystick_handler.controllers[id]
					id = int(id) + 1
					if controller.robot_ids:
						string += f" Joystick {id} -> Robots {controller.robot_ids} | Voltages "
						for r_id in controller.robot_ids:
							if r_id in self.batteryVoltages:
								string += f"{round(self.batteryVoltages[r_id], 2)}V "
						string += f"| "
					else:
						string += f" Joystick {id} -> Robot {controller.robot_id} | "
						if controller.robot_id in self.batteryVoltages:
							string += f"Voltage {round(self.batteryVoltages[controller.robot_id],2)}V | "
				print(string, end="")
				for event in self.events:
					print(event)
				self.events = []
				time.sleep(0.1)
		except Exception as e:
			self.record_event(-1, str(e))
			print(e)
			self.shutdown()


class JoystickHandler:
	"""Handles the detection and management of joysticks."""

	def __init__(self, event_handler: EventHandler, shutdown: Callable[[], None], robot_ids: Optional[List[int]] = None) -> None:
		self.shutdown = shutdown
		self.running = True
		self.controllers: Dict[int, 'Joystick'] = {}
		self.lost_controllers: Dict[int, int] = {}
		self.event_handler = event_handler
		self.robot_ids = robot_ids
		self.thread = threading.Thread(target=self.loop)
		self.thread.start()

	def loop(self) -> None:
		"""Main loop for handling joysticks."""
		try:
			while self.running:
				# Check if controllers in threads are still alive
				for id in list(self.controllers.keys()):
					if not self.controllers[id].controller._event_thread.is_alive():
						self.event_handler.record_event(id, "Controller disconnected")
						self.lost_controllers[id] = self.controllers[id].robot_id
						del self.controllers[id]
				# Discover new controllers that are not paired yet
				for path in glob("/dev/input/js*"):
					id = path.replace("/dev/input/js", "")
					if id not in self.controllers:
						try:
							robot_id = self.lost_controllers.get(id, 0)
							# controller = Xbox360Controller(id)
							# wrapper = Joystick(controller, robot_id, self.robot_ids)
							wrapper = Joystick(int(id), robot_id, self.robot_ids)

							self.controllers[id] = wrapper
							self.event_handler.record_event(id, "New controller discovered")
						except Exception as e:
							print(e)
							time.sleep(1)
							pass
				time.sleep(0.1)
		except Exception as e:
			self.event_handler.record_event(-1, str(e))
			print(f"\n{e}")
			self.shutdown()

# ...existing imports...

class Joystick:
    def __init__(self, joystick_index: int, robot_id: int, robot_ids: Optional[List[int]] = None) -> None:
        self.id = joystick_index
        self.joystick = pygame.joystick.Joystick(joystick_index)
        self.joystick.init()

        self.robot_id = robot_id
        self.robot_ids = robot_ids
        self.kick_speed = KICK_SPEED
        self.dribblerOn = True
        self.yaw = 0
        self.ignore_joystick = 0

        self.command = utils.generate_empty_robot_command()

        # Debounce variables for shoot buttons
        self.button_b_prev = False
        self.trigger_r_prev = False

        self.assign_open_robot(1)

    def assign_open_robot(self, addition: int = 0) -> None:
        if self.robot_ids:
            self.robot_id = 16
        else:
            claimed_ids = [
                joystick.robot_id
                for joystick in joystick_handler.controllers.values()
                if joystick.id != self.id
            ]
            while self.robot_id in claimed_ids:
                self.robot_id = (self.robot_id + addition) % 16

    def get_payload(self, keyboard_input: Dict[str, bool]) -> bytes:
        pygame.event.pump()  # Refresh controller state

        if self.ignore_joystick == 0:
            if not self.robot_ids:
                hat = self.joystick.get_hat(0)
                if hat[0] != 0:
                    self.robot_id = (self.robot_id + hat[0]) % 16
                    self.assign_open_robot(addition=hat[0])

            self.command.dribblerOn = self.dribblerOn

            # Handle button B (kick) with debounce
            button_b = self.joystick.get_button(1)
            if button_b and not self.button_b_prev:
                # Send the shoot command only on button press
                self.command.kickChipPower = self.kick_speed
                self.command.doKick = True
                self.command.doForce = True
            elif not button_b:
                # Reset the state when the button is released
                self.command.doKick = False
                self.command.doForce = False
            self.button_b_prev = button_b

            # Handle trigger R (kick) with debounce
            trigger_r = self.joystick.get_button(7)
            if trigger_r and not self.trigger_r_prev:
                # Send the shoot command only on button press
                self.command.kickChipPower = self.kick_speed
                self.command.doKick = True
                self.command.doForce = True
            elif not trigger_r:
                # Reset the state when the button is released
                self.command.doKick = False
                self.command.doForce = False
            self.trigger_r_prev = trigger_r

            axis_rx = self.joystick.get_axis(2)  # Right stick X (corrected)
            if abs(axis_rx) > 0.3:
                self.yaw -= axis_rx * 0.1

            axis_lx = self.joystick.get_axis(0)
            axis_ly = self.joystick.get_axis(1)

            deadzone = 0.3
            velocity_x = max(0, abs(axis_lx) - deadzone) / (1 - deadzone) * np.sign(axis_lx)
            velocity_y = max(0, abs(axis_ly) - deadzone) / (1 - deadzone) * np.sign(axis_ly)

            rho = math.sqrt(velocity_x**2 + velocity_y**2) * MAX_SPEED
            theta = math.atan2(velocity_x, velocity_y)

            self.command.toRobotId = self.robot_id
            self.command.rho = rho
            self.command.theta = theta + self.yaw + math.pi
            self.command.yaw = self.yaw

        else:
            self.command.rho = 0
            self.command.theta = 0
            self.command.yaw = self.yaw

        if any(keyboard_input.values()):
            self.ignore_joystick = 120
            if any(keyboard_input[str(i)] for i in range(10)):
                self.robot_id = int([i for i in range(10) if keyboard_input[str(i)]][0])
            self.yaw += (keyboard_input['q'] - keyboard_input['e']) * ROTATION_SPEED / BASESTATION_FREQUENCY
            if keyboard_input['w'] or keyboard_input['s'] or keyboard_input['a'] or keyboard_input['d']:
                velocity_x = (keyboard_input['w'] - keyboard_input['s']) * MAX_SPEED
                velocity_y = (keyboard_input['a'] - keyboard_input['d']) * MAX_SPEED
                rho = math.sqrt(velocity_x**2 + velocity_y**2)
                theta = math.atan2(velocity_y, velocity_x)
                self.command.rho = rho
                self.command.theta = theta + self.yaw
            else:
                self.command.rho = 0
                self.command.theta = 0
            self.command.doKick = keyboard_input['k']
            self.command.doForce = keyboard_input['k'] or keyboard_input['c']
            self.command.kickChipPower = self.kick_speed
            self.command.doChip = keyboard_input['c']
            self.command.dribblerOn = keyboard_input['b']
            self.command.yaw = self.yaw
        else:
            self.ignore_joystick = max(0, self.ignore_joystick - 1)

        return self.command
class BasestationHandler:
	"""Handles communication with the basestation."""

	def __init__(self, event_handler: EventHandler, joystick_handler: JoystickHandler, shutdown: Callable[[], None], simulate: bool) -> None:
		self.shutdown = shutdown
		self.packet_Hz = BASESTATION_FREQUENCY
		self.running = True
		if args.simulate:
			self.basestation = utils.open_simulated_basestation()
		else:
			self.basestation = utils.open_continuous(timeout=0.01)
		self.event_handler = event_handler
		self.joystick_handler = joystick_handler
		self.thread = threading.Thread(target=self.loop)
		self.thread.start()

	def loop(self) -> None:
		print("starting base loop")
		"""Main loop for handling basestation communication."""
		try:
			current_dir = os.path.dirname(os.path.abspath(__file__))
			log_dir = os.path.join(current_dir, "logs/joystick")
			os.makedirs(log_dir, exist_ok=True)
			filename = datetime.now().strftime("%Y-%m-%d_%H:%M:%S") + ".rembin"
			logger = REMParser(self.basestation, f"{log_dir}/{filename}")

			last_written = time.time()
			while self.running:
				time_till_next_tick = last_written + 1. / self.packet_Hz - time.time()
				time.sleep(max(0, time_till_next_tick))
				last_written += 1. / self.packet_Hz

				for joystick in joystick_handler.controllers.values():
					payload = joystick.get_payload(keyboard_handler.get_keyboard_input())
					for i in joystick.robot_ids if joystick.robot_ids else [payload.toRobotId]:
						payload.toRobotId = i
						self.basestation.write(payload)
						logger.write_bytes(payload.encode())

				logger.read()
				logger.process()

				def handle_rem_log(rem_log: REM_Log) -> None:
					log_from = "[?]  "
					if rem_log.fromBS:
						log_from = "[BS] "
					if not rem_log.fromPC and not rem_log.fromBS:
						log_from = f"[{str(rem_log.fromRobotId).rjust(2)}] "

					message = rem_log.message.strip()
					message = log_from + message

					nwhitespace = os.get_terminal_size().columns - len(message) - 2
					print(f"\r{message}{' ' * nwhitespace}")

				def handle_rem_robotfeedback(rem_robotfeedback: REM_RobotFeedback) -> None:
					self.event_handler.batteryVoltages[rem_robotfeedback.fromRobotId] = rem_robotfeedback.batteryLevel

				while logger.has_packets():
					packet = logger.get_next_packet()
					if isinstance(packet, REM_Log):
						handle_rem_log(packet)
					elif isinstance(packet, REM_RobotFeedback):
						handle_rem_robotfeedback(packet)
		except Exception as e:
			self.event_handler.record_event(-1, str(e))
			print(e)
			self.shutdown()


class KeyboardHandler:
	"""Handles keyboard input and controls the robots."""

	def __init__(self, joystick_handler: JoystickHandler) -> None:
		self.joystick_handler = joystick_handler
		self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
		self.listener.start()
		self.keyboard_input: Dict[str, bool] = {'w': False, 's': False, 'a': False, 'd': False, 'k': False, 'c': False, 'b': False, 'q': False, 'e': False, '1' : False, '2' : False, '3' : False, '4' : False, '5' : False, '6' : False, '7' : False, '8' : False, '9' : False, '0' : False}

	def on_press(self, key: keyboard.Key) -> None:
		"""Handles key press events."""
		try:
			char = key.char
			if char in self.keyboard_input:
				self.keyboard_input[char] = True
		except AttributeError:
			pass

	def on_release(self, key: keyboard.Key) -> None:
		"""Handles key release events."""
		try:
			char = key.char
			if char in self.keyboard_input:
				self.keyboard_input[char] = False
		except AttributeError:
			pass

	def get_keyboard_input(self) -> Dict[str, bool]:
		"""Returns the current keyboard input."""
		return self.keyboard_input


def shutdown() -> None:
	"""Shuts down the system."""
	print("Exiting")
	event_handler.running = False
	basestation_handler.running = False
	joystick_handler.running = False
	for joystick in joystick_handler.controllers.values():
		joystick.controller.close()


event_handler = EventHandler(shutdown)


def thread_exception_handler(args: threading.ExceptHookArgs) -> None:
	"""Handles exceptions raised in threads."""
	event_handler.record_event(-1, f"Caught: {args}")

threading.excepthook = thread_exception_handler

joystick_handler = JoystickHandler(event_handler, shutdown, args.robot_ids)
basestation_handler = BasestationHandler(event_handler, joystick_handler, shutdown, args.simulate)
keyboard_handler = KeyboardHandler(joystick_handler)
event_handler.start(joystick_handler)

try:
	event_handler.thread.join()
except:
	shutdown()