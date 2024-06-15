import argparse
import atexit
import datetime
import math
import os
import sys
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
from proto import State_pb2
# Import local modules
from REMParser import REMParser
import utils

X_LOCATION_HOMING = -2
Y_LOCATION_HOMING = -0
# Every additional robot will be placed at the following offset from the previous robot
X_OFFSET_ADDITIONAL_ROBOT = 0
Y_OFFSET_ADDITIONAL_ROBOT = 1 
HOMING_TIME = 5
TEST_TIME = 4
BASESTATION_FREQUENCY = 60 # ticks per second

# TRAPEZOID TEST
MAX_ACCELERATION = [1, 2]
START_ACCERLERATION = 0.5
END_ACCELERATION = 1.5
START_DECELERATION = 2.5
END_DECELERATION = START_DECELERATION + END_ACCELERATION - START_ACCERLERATION

if END_DECELERATION > TEST_TIME:
	print("The test is not possible with the given parameters")
	exit()
if (START_DECELERATION - START_ACCERLERATION) * (END_ACCELERATION - START_ACCERLERATION) * max(MAX_ACCELERATION) > 5:
	print("Robot will drive more than 5 meters, this is not allowed by walls")
	exit()

try:
	import cv2
	cv2_available = True
except:
	print("Warning! Could not import cv2. Can't visualize.")
	cv2_available = False

basestation = None

def rotate(origin, point, angle):
	ox, oy = origin
	px, py = point

	qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
	qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
	return qx, qy

class WorldSubscriber:
	def __init__(self, address="127.0.0.1", port="5558"):
		self.context = zmq.Context()
		self.socket = self.context.socket(zmq.SUB)
		self.socket.connect(f'tcp://{address}:{port}')
		self.socket.setsockopt_string(zmq.SUBSCRIBE, '')
		print(f"Connected to {address}:{port} as subscriber")

	def get_robot_position(self, robot_id: int, is_yellow: bool) -> tuple:
		data = self.socket.recv()
		world_state = State_pb2.State()
		world_state.ParseFromString(data)
		while True:
			for robot in (world_state.last_seen_world.yellow if is_yellow else world_state.last_seen_world.blue):
				if robot.id == robot_id:
					return robot.pos.x, robot.pos.y
			print("Robot not found, waiting for new data")
			time.sleep(1/60*0.1)

	def get_robot_angle(self, robot_id: int, is_yellow: bool) -> float:
		data = self.socket.recv()
		world_state = State_pb2.State()
		world_state.ParseFromString(data)
		while True:
			for robot in (world_state.last_seen_world.yellow if is_yellow else world_state.last_seen_world.blue):
				if robot.id == robot_id:
					return robot.angle
			print("Robot not found, waiting for new data")
			time.sleep(1/60*0.1)

def close_basestation() -> None:
	"""
	Closes the basestation on exit.
	"""
	global basestation
	if basestation is not None:
		basestation.close()
		print("Basestation closed, enjoy your day")

atexit.register(close_basestation)

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

def create_robot_command(tick_number: int, counter: int, robot_id: int, test: str) -> REM_RobotCommand:
	"""
	Creates a robot command for a given robot ID.

	Args:
  		tick_number (int): The current tick number
		counter (int): The counter of which robot this is
		robot_id (int): The ID of the robot
		test (str): The test to run

	Returns:
		REM_RobotCommand: The created robot command.
	"""
	cmd = utils.generate_empty_robot_command()
	cmd.toRobotId = robot_id
	if is_homing(tick_number):
		cmd.useCameraYaw = 1
		cmd.cameraYaw = subscriber.get_robot_angle(robot_id, True) 
		target_x = X_LOCATION_HOMING + counter * X_OFFSET_ADDITIONAL_ROBOT
		target_y = Y_LOCATION_HOMING + counter * Y_OFFSET_ADDITIONAL_ROBOT

		current_x, current_y = subscriber.get_robot_position(robot_id, True)
		distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
		direction = math.atan2(target_y - current_y, target_x - current_x)
		cmd.theta = direction
		cmd.rho = min(distance, 1) # Limit the speed to prevent sad things from happening
		cmd.yaw = 0
	else:
		if test == "trapezoid":
			test_number = tick_number // (BASESTATION_FREQUENCY * (HOMING_TIME + TEST_TIME))
			if test_number == len(MAX_ACCELERATION):
				print("All tests are done")
				exit()
			max_acceleration = MAX_ACCELERATION[test_number]
			time_since_start = (tick_number % (BASESTATION_FREQUENCY * (HOMING_TIME + TEST_TIME)) - BASESTATION_FREQUENCY * HOMING_TIME) / BASESTATION_FREQUENCY
			if time_since_start < START_ACCERLERATION:
				cmd.rho = 0
				cmd.theta = 0
			elif time_since_start < END_ACCELERATION:
				cmd.rho = max_acceleration * (time_since_start - START_ACCERLERATION)
				cmd.theta = 0
			elif time_since_start < START_DECELERATION:
				cmd.rho = max_acceleration * (END_ACCELERATION - START_ACCERLERATION)
				cmd.theta = 0
			else:
				cmd.rho = max_acceleration * (END_ACCELERATION - START_ACCERLERATION) - max_acceleration * (time_since_start - START_DECELERATION)
				cmd.rho = max(0, cmd.rho)
				cmd.theta = 0
		else:
			cmd.rho = 0
			cmd.theta = 0

	return cmd

def parse_and_process_args() -> argparse.Namespace:
	"""
	Parse command line arguments and process related logic.
	"""
	global basestation
	parser = argparse.ArgumentParser()
	testsAvailable = ["nothing", "trapezoid"]
	parser.add_argument("--test", choices=testsAvailable, default="nothing", help="Specify which test to run. Default is 'nothing'.")
	parser.add_argument("robot_ids", type=int, nargs='+', help="An array of integers for the robot ids")
	parser.add_argument('--output-dir', '-d', help="REMParser output directory. Logs will be placed under 'logs/OUTPUT_DIR'")
	args = parser.parse_args()
 
	if (basestation is None or not basestation.isOpen()):
		basestation = utils.open_continuous(timeout=0.1)
		print("Basestation opened")

	return args

subscriber = WorldSubscriber()

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
	rate_of_turn_avg = 0
	wheel_speeds_avg = np.zeros(4)
	while True:
		if time.time() - latest_feedback_time > 1:
			print("No feedback received in the last second")
		time_till_next_tick = last_tick_time + 1/BASESTATION_FREQUENCY - time.time()
		time.sleep(max(0,time_till_next_tick))
		last_tick_time = time.time()
		counter = 0
		for robot_id in args.robot_ids:
			cmd = create_robot_command(tick_number, counter, robot_id, args.test)
			cmd.toRobotId = robot_id
			basestation.write(cmd.encode())
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

		tick_number += 1

		# ========== VISUALISING ========== #

		# Break if cv2 is not imported
		if not cv2_available : continue
		# Break if we have more than 1 robot
		if len(args.robot_ids) > 1: continue

		# Draw robot on the image
		s = 101.2
		cv2.line(image_vis, (int(250-s/2), 250-73), (int(250+s/2), 250-73), (255,255,255),2)
		cv2.ellipse(image_vis, (250, 250), (90, 90), -90, 35, 325, (255,255,255), 2)			
		if last_packet_feedback:			
			# Ballsensor
			if last_packet_feedback.ballSensorWorking:
				cv2.line(image_vis, (int(250-s/2), 250-73-5), (int(250+s/2), 250-73-5), (0, 1, 0),2)
				if last_packet_feedback.ballSensorSeesBall:
					#Ball is centered in the drawing, this might not refelct the real world
					cv2.circle(image_vis, (250, 250-90), 10, (0, 0.4, 1), -1) 
			else:
				cv2.line(image_vis, (int(250-s/2), 250-73-5), (int(250+s/2), 250-73-5), (0, 0, 1),2)

			length = int(last_packet_feedback.rho * 500)
			px, py = rotate((250, 250), (250, 250+length), last_packet_feedback.theta)
			cv2.line(image_vis, (250,250), (int(px), int(py)), (1, 0, 0), 8)
			# Battery
			cv2.rectangle(image_vis, (10, 10), (50, 30), color=(1, 1, 1))
			cv2.rectangle(image_vis, (51, 15), (55, 25), color=(1, 1, 1), thickness=-1)
			#COLOR IS IN (B,G,R)
			if last_packet_feedback.batteryLevel >= 18.0:
				v = str(round(last_packet_feedback.batteryLevel, 2)) + 'V'
				if last_packet_feedback.batteryLevel < 20.0:
					background_color = (0,0,255) # red
				elif last_packet_feedback.batteryLevel < 22.0:
					background_color = (0,140,255) # orange
				elif last_packet_feedback.batteryLevel < 24.0:
					background_color = (0,255,255) # yellow
				else:
					background_color = (0,255,0) # green
				length_rectangle = (int) (5.27777777 * last_packet_feedback.batteryLevel - 84)
				cv2.rectangle(image_vis, (11, 11), (length_rectangle, 29), color=background_color, thickness=-1)
				cv2.putText(image_vis, v, org=(60, 26), color=(255,255,255), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1, thickness=1, lineType=cv2.LINE_AA)
			else:
				cv2.putText(image_vis, '?', org=(25, 26), color=(255,255,255), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1)
			
		if last_packet_state_info:
			# XSens yaw
			px, py = rotate((250, 250), (250, 150), -last_packet_state_info.xsensYaw)
			cv2.line(image_vis, (250, 250), (int(px), int(py)), (1, 1, 1), 1)
			cv2.circle(image_vis, (int(px), int(py)), 5, (1, 1, 1), -1)
			# Commanded yaw
			px, py = rotate((250, 250), (250, 150), -cmd.yaw)
			cv2.line(image_vis, (250, 250), (int(px), int(py)), (0, 1, 0), 1)
			cv2.circle(image_vis, (int(px), int(py)), 5, (0, 1, 0), -1)
			# XSens rate of turn
			rate_of_turn_avg = rate_of_turn_avg * 0.99 + last_packet_state_info.rateOfTurn * 0.01
			cv2.ellipse(image_vis, (250, 250), (40, 40), -90, 0, 0.5*-rate_of_turn_avg * 180 / math.pi, (1,.45, .5), 12)
			cv2.ellipse(image_vis, (250, 250), (40, 40), -90, 0, 0.5*-last_packet_state_info.rateOfTurn * 180 / math.pi, (1, 1, 1), 4)
			# Wheel speeds
			wheel_speeds = np.array([last_packet_state_info.wheelSpeed1, last_packet_state_info.wheelSpeed2, last_packet_state_info.wheelSpeed3, last_packet_state_info.wheelSpeed4])
			wheel_speeds_exp = np.log(np.abs(wheel_speeds))
			wheel_speeds_exp = np.clip(wheel_speeds_exp, 0, None)
			wheel_speeds_exp = wheel_speeds_exp * .25 * np.sign(wheel_speeds)
			wheel_speeds_avg = wheel_speeds_avg * 0.99 + wheel_speeds_exp * 0.01
			# XSens wheel speed 1
			rx, ry = rotate((330, 170), (330, 170 - wheel_speeds_avg[0] * 80), -30 * np.pi / 180.)
			cv2.line(image_vis, (330, 170), (int(rx), int(ry)), (.15, .15, 1), 10)
			rx, ry = rotate((330, 170), (330, 170 - wheel_speeds_exp[0] * 80), -30 * np.pi / 180.)
			cv2.line(image_vis, (330, 170), (int(rx), int(ry)), (1, 1, 1), 4)
			# XSens wheel speed 2
			rx, ry = rotate((170, 170), (170, 170 + wheel_speeds_avg[1] * 80), 30 * np.pi / 180.)
			cv2.line(image_vis, (170, 170), (int(rx), int(ry)), (.15, .15, 1), 10)
			rx, ry = rotate((170, 170), (170, 170 + wheel_speeds_exp[1] * 80), 30 * np.pi / 180.)
			cv2.line(image_vis, (170, 170), (int(rx), int(ry)), (1, 1, 1), 4)
			# XSens wheel speed 3
			rx, ry = rotate((170, 330), (170, 330 + wheel_speeds_avg[2] * 80), -60 * np.pi / 180.)
			cv2.line(image_vis, (170, 330), (int(rx), int(ry)), (.15, .15, 1), 10)
			rx, ry = rotate((170, 330), (170, 330 + wheel_speeds_exp[2] * 80), -60 * np.pi / 180.)
			cv2.line(image_vis, (170, 330), (int(rx), int(ry)), (1, 1, 1), 4)
			# XSens wheel speed 4
			rx, ry = rotate((330, 330), (330, 330 - wheel_speeds_avg[3] * 80), 60 * np.pi / 180.)
			cv2.line(image_vis, (330, 330), (int(rx), int(ry)), (.15, .15, 1), 10)
			rx, ry = rotate((330, 330), (330, 330 - wheel_speeds_exp[3] * 80), 60 * np.pi / 180.)
			cv2.line(image_vis, (330, 330), (int(rx), int(ry)), (1, 1, 1), 4)
		cv2.imshow("Press esc to quit", image_vis)
		if cv2.waitKey(1) == 27: 
			exit()
		image_vis *= 0.7
		tick_number += 1
	 
if __name__ == "__main__":
	main()