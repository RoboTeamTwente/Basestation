import math
import os
import subprocess
import signal
import pty
import select
import socket
import sys
import threading
import time
import numpy as np
import serial
import zmq

# Add parent directory to path to allow importing from Core.Inc
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "proto"))

# Import roboteam embedded messages
from Core.Inc.roboteam_embedded_messages.python import REM_BaseTypes as BaseTypes
from Core.Inc.roboteam_embedded_messages.python.REM_RobotFeedback import REM_RobotFeedback
from Core.Inc.roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand
from proto.ssl_simulation_robot_control_pb2 import RobotCommand, MoveLocalVelocity, RobotControl, RobotMoveCommand
from proto import State_pb2


BLUE_CONTROL_PORT = 10301
YELLOW_CONTROL_PORT = 10302

class WorldSubscriber:
	def __init__(self, address="127.0.0.1", port="5558"):
		self.context = zmq.Context()
		self.socket = self.context.socket(zmq.SUB)
		self.socket.connect(f'tcp://{address}:{port}')
		self.socket.setsockopt_string(zmq.SUBSCRIBE, '')
		print(f"Connected to {address}:{port} as subscriber")
		self.world_state = State_pb2.State()
		self.lock = threading.Lock()
		self.thread = threading.Thread(target=self._receive_data)
		self.thread.daemon = True
		self.thread.start()

	def _receive_data(self):
		while True:
			data = self.socket.recv()
			with self.lock:
				self.world_state.ParseFromString(data)

	def get_robot_position(self, robot_id: int, is_yellow: bool) -> tuple:
		with self.lock:
			for robot in (self.world_state.last_seen_world.yellow if is_yellow else self.world_state.last_seen_world.blue):
				if robot.id == robot_id:
					rho = math.sqrt(robot.vel.x**2 + robot.vel.y**2)
					theta = math.atan2(robot.vel.y, robot.vel.x)
					return rho, theta, robot.angle
		print("Robot not found")
		return 0, 0, 0

observer = WorldSubscriber()

class SerialSimulator:
	"""
	A class used to simulate a serial connection for sending robot commands.

	...

	Attributes
	----------
	master : int
		a file descriptor for the master end of the pty
	slave : int
		a file descriptor for the slave end of the pty
	ser : serial.Serial
		a serial connection to the slave end of the pty

	Methods
	-------
	write(text)
		Writes a robot command to the serial connection.
	"""

	def __init__(self):
		"""
		Constructs all the necessary attributes for the SerialSimulator object.
		"""
		self.master, self.slave = pty.openpty()
		s_name = os.ttyname(self.slave)
		self.ser = serial.Serial(s_name)
		self.port = -53
		# run 'docker pull roboteamtwente/roboteam:latest' before running this script
		self.proc = subprocess.Popen(['./simulator-cli'], cwd=os.path.dirname(os.path.abspath(__file__)), stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

	def write(self, text):
		"""
		Writes a robot command to the serial connection.

		Parameters:
			text (REM_RobotCommand): The robot command to write.
		"""
		# check if text is a robot command
		if not isinstance(text, REM_RobotCommand):
			# print warning and return
			print("Warning: text is not a robot command")
			return
		rho_feedback, theta_feedback, yaw_feedback = observer.get_robot_position(text.toRobotId, True)
		if not text.useYaw:
			angular_velocity = text.angularVelocity
		else:
			current_yaw = yaw_feedback
			desired_yaw = text.yaw
			shortest_angle = (desired_yaw - current_yaw + np.pi) % (2 * np.pi) - np.pi
			angular_velocity = max(shortest_angle*5, 0.3) if shortest_angle > 0 else min(shortest_angle*5, -0.3)

		rho = text.rho
		theta = text.theta - text.yaw
		forward_velocity = rho * np.cos(theta)
		left_velocity = rho * np.sin(theta)
		move_local_velocity = MoveLocalVelocity(forward=forward_velocity, left=left_velocity, angular=angular_velocity)

		robot_move_command = RobotMoveCommand()
		robot_move_command.local_velocity.CopyFrom(move_local_velocity)
		robot_command = RobotCommand(id=text.toRobotId, dribbler_speed=text.dribblerOn*16000)
		# if we need to chip, set kick_angle to 45 degrees
		if text.doChip and text.kickChipPower > 0:
			robot_command.kick_angle = 45
			robot_command.kick_speed = text.kickChipPower
		if text.doKick and text.kickChipPower > 0:
			robot_command.kick_angle = 0
			robot_command.kick_speed = text.kickChipPower
		robot_command.move_command.CopyFrom(robot_move_command)
		robot_control = RobotControl()
		robot_control.robot_commands.extend([robot_command])
		sock.sendto(robot_control.SerializeToString(), ("localhost", YELLOW_CONTROL_PORT))
		# from observer, get the robot velocity ect and create fake feedback
		robot_feedback = REM_RobotFeedback()
		robot_feedback.fromRobotId = text.toRobotId
		robot_feedback.batteryLevel = 23
		robot_feedback.rho = rho_feedback
		robot_feedback.theta = theta_feedback
		robot_feedback.yaw = yaw_feedback
		robot_feedback.packetType = BaseTypes.REM_PACKET_TYPE_REM_ROBOT_FEEDBACK
		robot_feedback.remVersion = BaseTypes.REM_LOCAL_VERSION
		robot_feedback.payloadSize = BaseTypes.REM_PACKET_SIZE_REM_ROBOT_FEEDBACK
		robot_feedback.timestamp = int(time.time()*1000)

		self.ser.write(robot_feedback.encode())
	
	def inWaiting(self):
		r, w, e = select.select([self.master], [], [], 0)
		return len(r) > 0
 
	def read(self, _):
		if self.inWaiting():
			data = os.read(self.master, 1024)  # read up to 1024 bytes
			return data
		else:
			return b''  # return empty bytes if there's no data available

	def close(self):
		self.ser.close()
		os.close(self.master)
		os.close(self.slave)
		self.proc.send_signal(signal.SIGINT)
		print("Simulated basestation connection closed")

print("Opening socket")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP