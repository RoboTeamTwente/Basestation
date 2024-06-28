import argparse
import atexit
import datetime
import math
import os
import subprocess
import sys
import threading
import time

import numpy as np
import zmq

# Add parent directory to path to allow importing from Core.Inc
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "proto"))
# Import roboteam embedded messages
from Core.Inc.roboteam_embedded_messages.python import REM_BaseTypes as BaseTypes
from Core.Inc.roboteam_embedded_messages.python.REM_RobotFeedback import REM_RobotFeedback
from Core.Inc.roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand
from Core.Inc.roboteam_embedded_messages.python.REM_Log import REM_Log
from Core.Inc.roboteam_embedded_messages.python.REM_RobotStateInfo import REM_RobotStateInfo
from visualization import visualize
from proto import State_pb2
# Import local modules
from REMParser import REMParser
import utils
import dockerUtils

X_LOCATION_HOMING = -2
Y_LOCATION_HOMING = -0
# Every additional robot will be placed at the following offset from the previous robot
X_OFFSET_ADDITIONAL_ROBOT = 0
Y_OFFSET_ADDITIONAL_ROBOT = 1 
HOMING_TIME = 5
TEST_TIME = 4
BASESTATION_FREQUENCY = 60  # ticks per second

# Trapezoid Test Parameters
MAX_ACCELERATION = [1, 2]
START_ACCELERATION = 0.5
END_ACCELERATION = 1.5
START_DECELERATION = 2.5
END_DECELERATION = START_DECELERATION + END_ACCELERATION - START_ACCELERATION

# Check feasibility of the test parameters
if END_DECELERATION > TEST_TIME:
	print("[Homing] The test is not possible with the given parameters")
	sys.exit()
if (START_DECELERATION - START_ACCELERATION) * (END_ACCELERATION - START_ACCELERATION) * max(MAX_ACCELERATION) > 5:
	print("[Homing] Robot will drive more than 5 meters, this is not allowed by walls")
	sys.exit()

basestation = None
subscriber = None
observer_file = None

def rotate(origin, point, angle):
	"""Rotate a point around a given origin by a specified angle."""
	ox, oy = origin
	px, py = point

	qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
	qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
	return qx, qy

class WorldSubscriber:
	def __init__(self, simulate, address="127.0.0.1", port="5558"):
		"""
		Initialize the WorldSubscriber with the given parameters.
		
		:param simulate: Flag to determine whether to run in simulation mode.
		:param address: The address to connect to.
		:param port: The port to connect to.
		"""
		self.context = zmq.Context()
		self.socket = self.context.socket(zmq.SUB)
		self.socket.connect(f'tcp://{address}:{port}')
		self.socket.setsockopt_string(zmq.SUBSCRIBE, '')
		print(f"[Homing] Connected to {address}:{port} as subscriber")

		self._pull_docker_image()
		self._run_docker_container(simulate)
		
		self.world_state = State_pb2.State()
		self.running = True
		self.lock = threading.Lock()
		self.thread = threading.Thread(target=self._receive_data)
		self.thread.daemon = True
		self.thread.start()

	def _pull_docker_image(self):
		"""Pull the latest Docker image."""
		proc_pull_docker = subprocess.Popen(['docker', 'pull', 'roboteamtwente/roboteam:latest'], stdout=None, stderr=None)
		proc_pull_docker.wait()  # Wait for the pull to complete before proceeding

	def _run_docker_container(self, simulate):
		"""Run the Docker container."""
		command_base = ['docker', 'run', '-it', '--rm', '--network', 'host', 'roboteamtwente/roboteam:latest', '/bin/sh', '-c']
		command_suffix = './bin/roboteam_observer --vision-port 10020' if simulate else './bin/roboteam_observer'
		
		if not dockerUtils.is_container_running('roboteamtwente/roboteam:latest'):
			self.proc_docker = subprocess.Popen(
				command_base + [command_suffix],
				stdout=subprocess.DEVNULL,
				stderr=subprocess.DEVNULL,
			)
			print("[Homing] RoboTeam Observer started.")
		else:
			self.proc_docker = None
			# Serial simulator already starts the observer, hence it's normal that it's already running
			if not simulate:
				print("\033[93m[Homing] RoboTeam Observer is already running.\033[0m")
   
	def _receive_data(self):
		"""Receive data from the ZMQ socket."""
		while self.running:
			data = self.socket.recv()
			with self.lock:
				self.world_state.ParseFromString(data)

	def get_robot_position(self, robot_id: int, is_yellow: bool) -> tuple:
		"""
		Get the position of a robot.
		
		:param robot_id: The ID of the robot.
		:param is_yellow: Flag indicating if the robot is yellow.
		:return: Tuple containing the (x, y) position of the robot.
		"""
		while self.running:
			with self.lock:
				for robot in (self.world_state.last_seen_world.yellow if is_yellow else self.world_state.last_seen_world.blue):
					if robot.id == robot_id:
						return robot.pos.x, robot.pos.y
			print("[Homing] Robot not found, waiting for new data")
			time.sleep(1 / 60 * 0.1)
		return None

	def write_output(self, robot_id: int, is_yellow: bool) -> None:
		"""
		Write the robot's state to the observer file.
		
		:param robot_id: The ID of the robot.
		:param is_yellow: Flag indicating if the robot is yellow.
		"""
		global observer_file
		if observer_file is None:
			return
		with self.lock:
			for robot in (self.world_state.last_seen_world.yellow if is_yellow else self.world_state.last_seen_world.blue):
				if robot.id == robot_id:
					observer_file.write(f"{self.world_state.last_seen_world.time / 1000000},{robot.id},{robot.pos.x},{robot.pos.y},{robot.angle},{robot.vel.x},{robot.vel.y},{robot.w}\n".encode())
					return

	def get_robot_angle(self, robot_id: int, is_yellow: bool) -> float:
		"""
		Get the angle of a robot.
		
		:param robot_id: The ID of the robot.
		:param is_yellow: Flag indicating if the robot is yellow.
		:return: The angle of the robot.
		"""
		while self.running:
			with self.lock:
				for robot in (self.world_state.last_seen_world.yellow if is_yellow else self.world_state.last_seen_world.blue):
					if robot.id == robot_id:
						return robot.angle
			print("[Homing] Robot not found, waiting for new data")
			time.sleep(1 / 60 * 0.1)
		return None
   
	def close(self):
		"""Close the subscriber and clean up resources."""
		self.running = False
		self.thread.join()
		self.socket.close()

def close_basestation() -> None:
	"""
	Closes the basestation on exit.
	"""
	global basestation
	if basestation is not None:
		basestation.close()
		print("[Homing] Basestation closed, enjoy your day")

def is_homing(tick_number: int) -> bool:
	"""
	Checks if the current tick number is within the homing time.

	Args:
		tick_number (int): The current tick number

	Returns:
		bool: True if the current tick number is within the homing time, False otherwise.
	"""
	cycle_time = (HOMING_TIME + TEST_TIME) * BASESTATION_FREQUENCY
	tick_number %= cycle_time
	# print ("Is homing: ", tick_number < HOMING_TIME * BASESTATION_FREQUENCY)
	return tick_number < HOMING_TIME * BASESTATION_FREQUENCY

def create_robot_command(tick_number: int, counter: int, robot_id: int, vision_id: int, test: str)  -> REM_RobotCommand:
	"""
	Creates a robot command for a given robot ID.

	Args:
  		tick_number (int): The current tick number
		counter (int): The counter of which robot this is
		robot_id (int): The ID of the robot
		vision_id (int): The ID of the vision system
		test (str): The test to run

	Returns:
		REM_RobotCommand: The created robot command.
	"""
	cmd = utils.generate_empty_robot_command()
	cmd.toRobotId = robot_id
	subscriber.write_output(vision_id, True)
	if is_homing(tick_number):
		cmd.useCameraYaw = 1
		cmd.cameraYaw = subscriber.get_robot_angle(vision_id, True) 
		target_x = X_LOCATION_HOMING + counter * X_OFFSET_ADDITIONAL_ROBOT
		target_y = Y_LOCATION_HOMING + counter * Y_OFFSET_ADDITIONAL_ROBOT

		current_x, current_y = subscriber.get_robot_position(vision_id, True)
		distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
		direction = math.atan2(target_y - current_y, target_x - current_x)
		cmd.theta = direction
		cmd.rho = min(distance, 1) # Limit the speed to prevent sad things from happening
		cmd.yaw = 0
	else:
		if test == "trapezoid":
			test_number = tick_number // (BASESTATION_FREQUENCY * (HOMING_TIME + TEST_TIME))
			if test_number == len(MAX_ACCELERATION):
				print("[Homing] All tests are done")
				exit()
			max_acceleration = MAX_ACCELERATION[test_number]
			time_since_start = (tick_number % (BASESTATION_FREQUENCY * (HOMING_TIME + TEST_TIME)) - BASESTATION_FREQUENCY * HOMING_TIME) / BASESTATION_FREQUENCY
			if time_since_start < START_ACCELERATION:
				cmd.rho = 0
				cmd.theta = 0
			elif time_since_start < END_ACCELERATION:
				cmd.rho = max_acceleration * (time_since_start - START_ACCELERATION)
				cmd.theta = 0
			elif time_since_start < START_DECELERATION:
				cmd.rho = max_acceleration * (END_ACCELERATION - START_ACCELERATION)
				cmd.theta = 0
			else:
				cmd.rho = max_acceleration * (END_ACCELERATION - START_ACCELERATION) - max_acceleration * (time_since_start - START_DECELERATION)
				cmd.rho = max(0, cmd.rho)
				cmd.theta = 0
		else:
			cmd.rho = 0
			cmd.theta = 0

	return cmd

def create_observer_file(args, datetime_str: str):
	"""
	Create a new observer file and a symlink to it.

	Args:
		args: Command line arguments
		datetime_str (str): Current date and time as a string

	Returns:
		observer_file: The newly created observer file
	"""
	global observer_file
	observer_file = f"logs/{args.output_dir}/observer_{datetime_str}.csv"
	current_dir = os.path.dirname(os.path.abspath(__file__))
	observer_file_path = os.path.join(current_dir, observer_file)
	print(f"\033[92m[Homing] Creating output file {observer_file_path}\033[0m")
	observer_file = open(observer_file_path, "wb")
	latest_file_path = os.path.join(current_dir, "latest_observer.csv")
	if os.path.lexists(latest_file_path):
		os.remove(latest_file_path)
	os.symlink(observer_file_path, latest_file_path)
	observer_file.write("timestamp,id,position_x,position_y,yaw,velocity_x,velocity_y,angular_velocity\n".encode())

def parse_and_process_args() -> argparse.Namespace:
	"""
	Parse command line arguments and process related logic.
	"""
	global basestation
	parser = argparse.ArgumentParser()
	testsAvailable = ["nothing", "trapezoid"]
	parser.add_argument("--test", choices=testsAvailable, default="nothing", help="Specify which test to run. Default is 'nothing'.")
	parser.add_argument("robot_ids", type=int, nargs='+', help="An array of integers for the robot ids. These are the IDs the basestation will use to communicate with the robots.")
	parser.add_argument('--output-dir', '-d', help="REMParser output directory. Logs will be placed under 'logs/OUTPUT_DIR'")
	parser.add_argument('--simulate', action='store_true', help="Use a fake basestation for simulation")
	parser.add_argument('--vision_ids', type=int, nargs='+', help="An array of integers for the vision ids")
	
	args = parser.parse_args()
	if args.vision_ids is None:
		args.vision_ids = args.robot_ids
	elif len(args.vision_ids) != len(args.robot_ids):
		print("[Homing] Vision ids should be the same length as robot ids")
		exit()
 
	if args.simulate:
		basestation = utils.open_simulated_basestation()
		print("[Homing] Simulated basestation opened")
	elif (basestation is None or not basestation.isOpen()):
		basestation = utils.open_continuous(timeout=0.1)
		print("[Homing] Basestation opened")
	return args

def main() -> None:
	"""
	Main function
	"""
	global basestation, subscriber
	args = parse_and_process_args()
	subscriber = WorldSubscriber(args.simulate)
	atexit.register(dockerUtils.kill_docker_containers_by_image, 'roboteamtwente/roboteam:latest')
	atexit.register(close_basestation)
	atexit.register(subscriber.close)
	datetime_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
	output_file = None
	if args.output_dir is not None:
		os.makedirs(f"logs/{args.output_dir}", exist_ok=True)
		output_file = f"logs/{args.output_dir}/log_{datetime_str}.bin"
		create_observer_file(args, datetime_str)
	parser = REMParser(basestation, output_file=output_file)
	last_tick_time = time.time()
	latest_feedback_time = time.time()
	tick_number = 0
	last_packet_feedback = None
	last_packet_state_info = None
	cmd = None
	image_vis = np.zeros((500, 500, 3), dtype=float)
	while True:
		current_time = time.time()
		time_till_next_tick = last_tick_time + 1/BASESTATION_FREQUENCY - current_time
		if time_till_next_tick > 0.1 / BASESTATION_FREQUENCY:
			time.sleep(0.1 / BASESTATION_FREQUENCY)
		if time_till_next_tick < 0:
			if (current_time - latest_feedback_time > 1) and (tick_number % BASESTATION_FREQUENCY == 0):
				print("\033[93m[Homing] No feedback received in the last second\033[0m")
			last_tick_time += 1/BASESTATION_FREQUENCY
			tick_number += 1
			counter = 0
			for robot_id, vision_id in zip(args.robot_ids, args.vision_ids):
				cmd = create_robot_command(tick_number, counter, robot_id, vision_id, args.test)
				basestation.write(cmd)
				parser.write_bytes(cmd.encode())
				counter += 1
		parser.read()
		parser.process()
		while parser.has_packets():
			packet = parser.get_next_packet()
			if isinstance(packet, REM_RobotFeedback):
				last_packet_feedback = packet
				latest_feedback_time = time.time()
			elif isinstance(packet, REM_RobotStateInfo):
				last_packet_state_info = packet
			elif isinstance(packet, REM_Log):
				print(packet.message)


		# ========== VISUALISING ========== #
		image_vis = visualize(args, image_vis, last_packet_feedback, last_packet_state_info, cmd)
	 
if __name__ == "__main__":
	main()