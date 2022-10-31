import os
import sys
import math
import time
import signal
from xbox360controller import Xbox360Controller
import utils
import serial
from datetime import datetime
import numpy as np
import threading
from glob import glob

import roboteam_embedded_messages.python.REM_BaseTypes as BaseTypes
from roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand as RobotCommand
from roboteam_embedded_messages.python.REM_RobotBuzzer import REM_RobotBuzzer as RobotBuzzer


basestation_handler = None
joystick_handler = None
event_handler = None


class EventHandler:
	def __init__(self, shutdown):
		self.shutdown = shutdown
		self.running = True
		self.events = []

	def start(self, joystick_handler):
		self.joystick_handler = joystick_handler

		# Start sending in a thread
		self.thread = threading.Thread(target=self.loop)
		self.thread.start()

	def record_event(self, id, event):
		current_time = datetime.now().strftime("%H:%M:%S")
		id = int(id) + 1
		self.events.insert(0, f"[{id} | {current_time}] {event}")
		self.events = self.events[:5]

	def loop(self):
		try:
			while self.running:
				# Clear terminal
				os.system('cls' if os.name == 'nt' else 'clear')

				for id in self.joystick_handler.controllers:
					controller = self.joystick_handler.controllers[id]
					id = int(id) + 1
					print(f"Controller: {id} | Robot: {controller.robot_id} (Dribbler: {controller.dribbler}) (Super: {controller.super})")
				print()

				for event in self.events:
					print(event)

				time.sleep(0.1)
		except Exception as e:
			self.event_handler.record_event(-1, e)
			print(e)
			self.shutdown()

class JoystickHandler:
	def __init__(self, event_handler, shutdown):
		self.shutdown = shutdown
		self.running = True
		self.controllers = {}
		self.lost_controllers = {}
		self.event_handler = event_handler

		# Start sending in a thread
		self.thread = threading.Thread(target=self.loop)
		self.thread.start()

	def loop(self):
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
							robot_id = self.lost_controllers[id] if id in self.lost_controllers else 0
							controller = Xbox360Controller(id)
							wrapper = Joystick(self, controller, robot_id=robot_id)
							self.controllers[id] = wrapper
							self.event_handler.record_event(id, "New controller discovered")
						except Exception as e:
							print(e)
							pass

				time.sleep(0.1)
		except Exception as e:
			self.event_handler.record_event(-1, e)
			print(e)
			self.shutdown()

class Joystick:
	def __init__(self, joystick_handler, controller, robot_id=0):
		self.id = controller.index
		self.controller = controller
		self.robot_id = robot_id
		self.kick_speed = 3
		self.chip_speed = 6.5
		self.dribbler = False
		self.absolute_angle = 0
		self.super = False

		self.X = False
		self.Y = False
		self.HAT_X = 0
		self.HAT_Y = 0
		self.command = RobotCommand()

		self.assign_open_robot(1)

	def assign_open_robot(self, addition=0):
		# Skip already claimed ids
		claimed_ids = []
		for id in joystick_handler.controllers:
			if id != self.id:
				claimed_ids.append(joystick_handler.controllers[id].robot_id)
		while self.robot_id in claimed_ids:
			self.robot_id = (self.robot_id + addition) % 16

	def get_payload(self):
		if self.controller.button_select.is_pressed and self.controller.button_mode.is_pressed and self.controller.button_start.is_pressed:
			self.super = not self.super
		
		# Left or right arrow pressed, loop through available robots
		if self.controller.button_select.is_pressed and self.HAT_X != self.controller.hat.x:
			self.HAT_X = self.controller.hat.x
			self.robot_id = (self.robot_id + self.controller.hat.x) % 16
			self.assign_open_robot(addition=self.controller.hat.x)

		# Toggle dribbler
		if self.controller.button_y._value and not self.Y:
			self.dribbler = not self.dribbler
		self.Y = self.controller.button_y._value
		self.command.dribbler = self.dribbler

		# Kick or chip
		self.command.doKick = False
		self.command.doChip = False
		if self.controller.button_a._value:
			self.command.kickChipPower = self.chip_speed if not self.super else 6.5
			self.command.doChip = True
			self.command.doForce = True

		if self.controller.button_b._value:
			self.command.kickChipPower = self.kick_speed if not self.super else 6.5
			self.command.doKick = True
			self.command.doForce = True
			
		r_x = self.controller.axis_r.x
		l_x = self.controller.axis_l.x
		l_y = self.controller.axis_l.y

		if self.controller.button_trigger_r._value:
			r_x = -0.4
			l_x = 0.5
			l_y = 0

		if self.controller.button_trigger_l._value:
			r_x = 0.4
			l_x = -0.5
			l_y = 0

		# Calculate angle
		if 0.3 < abs(r_x): self.absolute_angle -= r_x * 0.1

		# Forward backward left right
		deadzone = 0.3

		velocity_x = 0
		if deadzone < abs(l_x):
			velocity_x = ( abs(l_x) - deadzone) / (1 - deadzone)
			velocity_x *= np.sign(l_x)

		velocity_y = 0
		if deadzone < abs(l_y):
			velocity_y = ( abs(l_y) - deadzone) / (1 - deadzone)
			velocity_y *= np.sign(l_y)
		
		if self.super:
			velocity_x *= 4
			velocity_y *= 4

		rho = math.sqrt(velocity_x * velocity_x + velocity_y * velocity_y);
		theta = math.atan2(-velocity_x, -velocity_y);

		if self.controller.button_x._value and not self.X:
			rho = 8.0
			theta = math.pi
			self.absolute_angle = self.absolute_angle + math.pi if self.absolute_angle <= 0 else self.absolute_angle - math.pi
		self.X = self.controller.button_x._value

		self.command.header = BaseTypes.PACKET_TYPE_REM_ROBOT_COMMAND
		self.command.remVersion = BaseTypes.LOCAL_REM_VERSION
		self.command.id = self.robot_id

		self.command.rho = rho
		self.command.theta = theta + self.absolute_angle
		self.command.angle = self.absolute_angle
		self.command.useAbsoluteAngle = 1

		buzzer_value = self.controller.trigger_l._value
		if 0.3 < buzzer_value:
			buzzer_command = RobotBuzzer()
			buzzer_command.header = BaseTypes.PACKET_TYPE_REM_ROBOT_BUZZER
			buzzer_command.remVersion = BaseTypes.LOCAL_REM_VERSION
			buzzer_command.id = self.robot_id
			buzzer_command.period = int(buzzer_value * 1000)
			buzzer_command.duration = 0.1
			return buzzer_command.encode()

		return self.command.encode()

class BasestationHandler:
	def __init__(self, event_handler, joystick_handler, shutdown):
		self.shutdown = shutdown
		self.packet_Hz = 60
		self.running = True
		self.basestation = utils.openContinuous(timeout=0.001)
		self.event_handler = event_handler
		self.joystick_handler = joystick_handler

		# Start sending in a thread
		self.thread = threading.Thread(target=self.loop)
		self.thread.start()

	def loop(self):
		try:
			last_written = time.time()
			while self.running:
				if 1./self.packet_Hz <= time.time() - last_written:
					last_written += 1./self.packet_Hz

					for i in joystick_handler.controllers:
						self.basestation.write(joystick_handler.controllers[i].get_payload())

				time.sleep(0.005)
		except Exception as e:
			self.event_handler.record_event(-1, e)
			print(e)
			self.shutdown()

def shutdown():
	print("Exiting")
	event_handler.running = False
	basestation_handler.running = False
	joystick_handler.running = False
	for id in joystick_handler.controllers:
		joystick_handler.controllers[id].controller.close()

event_handler = EventHandler(shutdown)

def thread_exception_handler(args):
	event_handler.record_event(-1, f"Caught: {args}")

threading.excepthook = thread_exception_handler

joystick_handler = JoystickHandler(event_handler, shutdown)
basestation_handler = BasestationHandler(event_handler, joystick_handler, shutdown)
event_handler.start(joystick_handler)

try:
	event_handler.thread.join()
except:
	shutdown()
