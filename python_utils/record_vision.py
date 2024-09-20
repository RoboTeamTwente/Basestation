import argparse
import atexit
import datetime
import math
import os
import subprocess
import sys
import threading
import zmq

# Add parent directory to path to allow importing from Core.Inc
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "proto"))
from proto import State_pb2
# Import local modules
import dockerUtils

subscriber = None
observer_file = None

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
		print(f"[Vision recording] Connected to {address}:{port} as subscriber")

		self._pull_docker_image()
		self._run_docker_container(simulate)
		
		self.world_state = State_pb2.State()
		self.running = True
		self.thread = threading.Thread(target=self._receive_data)
		self.thread.daemon = True
		self.thread.start()

	def _pull_docker_image(self):
		"""Pull the latest Docker image."""
		proc_pull_docker = subprocess.Popen(['docker', 'pull', 'roboteamtwente/roboteam:latest'], stdout=None, stderr=None)
		proc_pull_docker.wait()  # Wait for the pull to complete before proceeding

	def _run_docker_container(self, simulate):
		"""Run the Docker container."""
		command_base = ['docker', 'run', '--rm', '--network', 'host', 'roboteamtwente/roboteam:latest', '/bin/sh', '-c']
		command_suffix = './bin/roboteam_observer --vision-port 10020' if simulate else './bin/roboteam_observer'
		
		if not dockerUtils.is_container_running('roboteamtwente/roboteam:latest'):
			self.proc_docker = subprocess.Popen(
				command_base + [command_suffix],
				stdout=subprocess.DEVNULL,
				stderr=subprocess.DEVNULL,
			)
			print("[Vision recording]] RoboTeam Observer started.")
		else:
			self.proc_docker = None
			print("\033[93m[Vision recording]] RoboTeam Observer is already running.\033[0m")
   
	def _receive_data(self):
		"""Receive data from the ZMQ socket."""
		global observer_file
		while self.running:
      	# observer_file.write("timestamp,is_yellow,id,position_x,position_y,yaw,velocity_x,velocity_y,angular_velocity,ball_position_x,ball_position_y,ball_velocity_x,ball_velocity_y\n".encode())
			data = self.socket.recv()
			self.world_state.ParseFromString(data)
			ball = self.world_state.last_seen_world.ball
			for is_yellow in [True, False]:
				for robot in (self.world_state.last_seen_world.yellow if is_yellow else self.world_state.last_seen_world.blue):
					observer_file.write(f"{self.world_state.last_seen_world.time / 1000000},{is_yellow},{robot.id},{robot.pos.x},{robot.pos.y},{robot.angle},{robot.vel.x},{robot.vel.y},{robot.w},{ball.pos.x},{ball.pos.y},{ball.vel.x},{ball.vel.y}\n".encode())
    
	def close(self):
		"""Close the subscriber and clean up resources."""
		self.running = False
		self.thread.join()
		self.socket.close()

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
	os.makedirs(os.path.dirname(observer_file_path), exist_ok=True)
	print(f"\033[92m[Vision recording]] Creating output file {observer_file_path}\033[0m")
	observer_file = open(observer_file_path, "wb")
	latest_file_path = os.path.join(current_dir, "latest_observer.csv")
	if os.path.lexists(latest_file_path):
		os.remove(latest_file_path)
	os.symlink(observer_file_path, latest_file_path)
	observer_file.write("timestamp,is_yellow,id,position_x,position_y,yaw,velocity_x,velocity_y,angular_velocity,ball_position_x,ball_position_y,ball_velocity_x,ball_velocity_y\n".encode())

def parse_and_process_args() -> argparse.Namespace:
	"""
	Parse command line arguments and process related logic.
	"""
	parser = argparse.ArgumentParser()
	parser.add_argument('--output-dir', '-d', help="Output directory. Output will be placed under 'logs/OUTPUT_DIR'")
	parser.add_argument('--simulate', action='store_true', help="Use observation data from the simulator")
	
	args = parser.parse_args()

	return args

def main() -> None:
	"""
	Main function
	"""
	global subscriber
	args = parse_and_process_args()
	atexit.register(dockerUtils.kill_docker_containers_by_image, 'roboteamtwente/roboteam:latest')
	datetime_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
	if args.output_dir is None:
		args.output_dir = 'default'
	create_observer_file(args, datetime_str)
	subscriber = WorldSubscriber(args.simulate)
	atexit.register(subscriber.close)
	try:
		while True:
			pass
	except KeyboardInterrupt:
		print("\033[91m[Vision recording] Stopping...\033[0m")
	 
if __name__ == "__main__":
	main()