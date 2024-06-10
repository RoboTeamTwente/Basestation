import argparse
import atexit
import math
import os
import sys
import time
from typing import List

import matplotlib.pyplot as plt
import collections

import numpy as np

# Add parent directory to path to allow importing from Core.Inc
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# Import roboteam embedded messages
from Core.Inc.roboteam_embedded_messages.python import REM_BaseTypes as BaseTypes
from Core.Inc.roboteam_embedded_messages.python.REM_RobotFeedback import REM_RobotFeedback
from Core.Inc.roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand
from Core.Inc.roboteam_embedded_messages.python.REM_Log import REM_Log
from Core.Inc.roboteam_embedded_messages.python.REM_RobotStateInfo import REM_RobotStateInfo
# Import local modules
from REMParser import REMParser
import utils

try:
	import cv2
	cv2_available = True
except:
	print("Warning! Could not import cv2. Can't visualize.")
	cv2_available = False

basestation = None
store_seconds = 10
rho_data = collections.deque(maxlen=int(60*store_seconds))
theta_data = collections.deque(maxlen=int(60*store_seconds))
yaw_data = collections.deque(maxlen=int(60*store_seconds))
time_data = collections.deque(maxlen=int(60*store_seconds))


plt.ion()
fig, (ax1, ax2, ax3) = plt.subplots(3)


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
	Creates an empty robot command for a given robot ID.

	Args:
		robot_id (int): The ID of the robot.

	Returns:
		REM_RobotCommand: The created robot command.
	"""
	cmd = utils.generate_empty_robot_command()
	if test == "nothing":
		cmd.rho = 0
		cmd.theta = 0
		cmd.angularVelocity = 0
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
		cmd.dribbler = 1
	elif test == "rotate":
		cmd.useYaw = 1
		# Full rotation every 2 seconds
		cmd.yaw = -math.pi + 2 * math.pi * ((tick_number / 120 + 0.5) % 1)
	elif test == "forward":
		cmd.rho = 0.3 - 0.3 * math.cos( 4 * math.pi * tick_number / 120 )
		cmd.theta = -math.pi if tick_number % 120 < 60 else 0
		cmd.useYaw = 1
	elif test == "sideways":
		cmd.theta = math.pi/2
		cmd.rho = 0.3 - 0.3 * math.cos( 4 * math.pi * tick_number / 120 )
		cmd.theta = -math.pi/2 if tick_number % 120 < 60 else math.pi/2
		cmd.useYaw = 1
	elif test == "rotate-discrete":
		cmd.useYaw = 1
		cmd.yaw = -math.pi + math.pi/2 * (int(tick_number / 30) % 4)
	elif test == "angular-velocity":
		cmd.angularVelocity = math.pi
	elif test == "circle":
		cmd.useYaw = 1
		cmd.rho = 1
		cmd.theta = 2 * math.pi * tick_number / 240
	elif test == "circle-forward":
		# move in a circle while facing forward
		cmd.useYaw = 1
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
	parser.add_argument("--test", choices=testsAvailable, default="nothing", help="Specify which test to run. Default is 'nothing'.")
	parser.add_argument("robot_id", type=int, nargs='+', help="An array of integers for the robot ids")
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
	parser = REMParser(basestation)
	last_tick_time = 0
	tick_number = 0
	last_packet_feedback = None
	last_packet_state_info = None
	# unix time in centi seconds
	first_time = int(time.time()*100)
	latest_feedback_time = time.time()
	image_vis = np.zeros((500, 500, 3), dtype=float)
	# basestation_config_command = utils.generate_basestation_config_command(args.team == "yellow")
	while True:
		if time.time() - latest_feedback_time > 1:
			print("No feedback received in the last second")
		# print("Sending commands, tick number:", tick_number)
		cmd = create_robot_command(args.test, tick_number)
		time_till_next_tick = last_tick_time + 1/60 - time.time()
		time.sleep(max(0,time_till_next_tick))
		last_tick_time = time.time()
		for robot_id in args.robot_id:
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
				# print()
			elif isinstance(packet, REM_RobotStateInfo):
				# print(f"Received state info from robot {packet.fromRobotId}")
				last_packet_state_info = packet
				# print()
			elif isinstance(packet, REM_Log):
				print(packet.message)
		if last_packet_feedback:
			rho_data.append(last_packet_feedback.rho)
			theta_data.append(last_packet_feedback.theta)
			yaw_data.append(last_packet_feedback.yaw)
			time_data.append((last_packet_feedback.timestamp - first_time)/100)
			print(last_packet_feedback.timestamp, last_packet_feedback.timestamp - first_time)
		# print("Rho data:", rho_data)
		# print("Time data:", time_data)


		# Update the plots
		if tick_number % 120 == 0:
			ax1.clear()
			ax1.plot(time_data, rho_data)
			ax1.set_xlabel("Time (s)")
			ax1.set_ylabel("Rho")
			ax1.set_ylim(0, 3)
   
			ax2.clear()
			ax2.plot(time_data, theta_data)
			ax2.set_xlabel("Time (s)")
			ax2.set_ylabel("Theta")
			ax2.set_ylim(-math.pi, math.pi)
   
			ax3.clear()
			ax3.plot(time_data, yaw_data)
			ax3.set_xlabel("Time (s)")
			ax3.set_ylabel("Yaw")
			ax3.set_ylim(-math.pi, math.pi)

			plt.draw()
			plt.pause(0.01)
		tick_number += 1

		# ========== VISUALISING ========== #

		# Break if cv2 is not imported
		if not cv2_available : continue
		continue

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