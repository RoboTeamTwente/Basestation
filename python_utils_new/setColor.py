import os
import sys
import time
import argparse

# Add parent directory to path to allow importing from Core.Inc
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import utils

parser = argparse.ArgumentParser(description='Set color for robots.')
parser.add_argument('color', type=str, help='Color of the robots (yellow or blue)', choices=['yellow', 'blue', 'y', 'b'])
args = parser.parse_args()

# Determine if the team is yellow based on the color argument
is_yellow = (args.color == 'yellow' or args.color == 'y')

basestation = utils.open_continuous(timeout=0.01)
basestation_command = utils.generate_basestation_config_command(is_yellow)

for i in range(10):
	basestation.write(basestation_command)
	time.sleep(0.1)
print("Color set, enjoy your day")