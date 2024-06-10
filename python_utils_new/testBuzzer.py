import os
import sys
import time
import utils
import argparse

# Add parent directory to path to allow importing from Core.Inc
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Core.Inc.roboteam_embedded_messages.python import REM_BaseTypes as BaseTypes
from Core.Inc.roboteam_embedded_messages.python.REM_RobotBuzzer import REM_RobotBuzzer

# Argument parser
parser = argparse.ArgumentParser(description='Set robot id.')
parser.add_argument('robot_id', type=int, help='An integer for the robot id')
args = parser.parse_args()

basestation = utils.open_continuous(timeout=0.01)

# For example, this could be the frequencies for the C major scale
frequencies = [261.63, 293.66, 329.63, 349.23, 392.00, 440.00, 493.88, 523.25]

robot_id = args.robot_id
time_per_note = 0.5

robot_buzzer = utils.generate_empty_robot_buzzer()
robot_buzzer.toRobotId = robot_id
robot_buzzer.duration = time_per_note

note_index = 0
while True:
	robot_buzzer.frequency = frequencies[note_index]
	basestation.write(robot_buzzer.encode())
	print(f"Sent buzzer command to robot {robot_id} with frequency {robot_buzzer.frequency}")
	# Move to the next note in the song
	note_index = (note_index + 1) % len(frequencies)
	# Sleep for the duration of the note
	time.sleep(time_per_note)