import argparse
import time

import roboteam_embedded_messages.python.REM_BaseTypes as BaseTypes
import utils
from REMParser import REMParser

if __name__ == "__main__":
	print("Running REMParser directly")

	argparser = argparse.ArgumentParser()
	argparser.add_argument('input_file', help='File to parse')
	args = argparser.parse_args()

	print("Parsing file", args.input_file)

	parser = REMParser(device=None)
	parser.parseFile(args.input_file)

	# Get all RobotCommands
	rcs = [ packet for packet in parser.packet_buffer if type(packet) == BaseTypes.REM_RobotCommand ]
	t_start, t_stop = rcs[0].timestamp_parser_ms, rcs[-1].timestamp_parser_ms
	print(t_start, t_stop)

	t_now = time.time() + 1

	for command in rcs:
		command.timestamp_parser_ms = (command.timestamp_parser_ms - t_start)/1000 + t_now
	
	basestation = utils.Basestation()

	rc_index = 0
	while True:
		time.sleep(0.001)
		t_now = time.time()
		if rcs[rc_index].timestamp_parser_ms < t_now:
			# print(rc_index)
			rc_index += 1
			basestation.write(rcs[rc_index].encode())