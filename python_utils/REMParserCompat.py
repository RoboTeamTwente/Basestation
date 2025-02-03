import argparse
import array
import json
import os
import sys
from collections import deque
from datetime import datetime, timedelta
from typing import Any, BinaryIO, Deque, Dict, Optional, Type
import numpy as np
import libusb_package
import usb.core
import usb.backend.libusb1

# Add parent directory to path to allow importing from Core.Inc
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# Import roboteam embedded messages
from Core.Inc.roboteam_embedded_messages.python import REM_BaseTypes as BaseTypes
from Core.Inc.roboteam_embedded_messages.python.REM_RobotFeedback import REM_RobotFeedback
from Core.Inc.roboteam_embedded_messages.python.REM_Packet import REM_Packet

class BasestationDevice:
    def __init__(self, timeout: np.int64 =10):
        libusb1_backend = usb.backend.libusb1.get_backend(find_library=libusb_package.find_library)
        dev = usb.core.find(idVendor=0x0483, idProduct=0x374b)
        if dev is None:
            raise ValueError("Basestation not found.")
        
        dev.set_configuration()

        # get an endpoint instance
        cfg = dev.get_active_configuration()
        intf = cfg[(0,0)]
        print(list(usb.util.find_descriptor(intf, find_all=True)))
        outp = usb.util.find_descriptor(
            intf,
            # match the first OUT endpoint
            custom_match = \
            lambda e: \
                usb.util.endpoint_direction(e.bEndpointAddress) == \
                usb.util.ENDPOINT_OUT)
        inp = usb.util.find_descriptor(
            intf,
            # match the first IN endpoint
            custom_match = \
            lambda e: \
                usb.util.endpoint_direction(e.bEndpointAddress) == \
                usb.util.ENDPOINT_IN)

        self.__dev = dev
        self.__inp = inp
        self.__outp = outp
        self.timeout = timeout
    
    def write(self, payload):
        self.__dev.write(self.__outp, payload, self.timeout)

    def read(self):
        self.__dev.read(self.__inp, BaseTypes.REM_PACKET_SIZE_REM_PACKET, self.timeout)

class REMParser:
	def __init__(self, device: BasestationDevice, output_file: Optional[str] = None) -> None:
		"""
		Initializes a new REMParser.

		Args:
			device (BasestationDevice): The device to parse REM from.
			output_file (Optional[str], optional): The file to output parsed REM to. Defaults to None.
		"""
		print("[REMParser] New REMParser")
		if device is not None:
			print(f"[REMParser] Device thingy")

		self.device = device
		self.buffer = []
		self.packet_buffer: Deque = deque()
		self.output_file: Optional[BinaryIO] = None

		if output_file:
			current_dir = os.path.dirname(os.path.abspath(__file__))
			output_file_path = os.path.join(current_dir, output_file)
			print(f"\033[92m[REMParser] Creating output file {output_file_path}\033[0m")
			self.output_file = open(output_file_path, "wb")
			print("flonk")
			latest_file_path = os.path.join(current_dir, "latest.rembin")
			if os.path.lexists(latest_file_path):
				os.remove(latest_file_path)
			os.symlink(output_file_path, latest_file_path)

	def read(self) -> None:
		"""
		Reads bytes from the device if any are available and appends them to the byte buffer.
		"""

		self.buffer = self.device.read()

	def process(self) -> None:
		"""
		Processes the byte buffer, decoding packets and adding them to the packet buffer.
		"""
		while self.buffer:
			packet_type = self.buffer[0]
			packet_valid = BaseTypes.REM_PACKET_TYPE_TO_VALID(packet_type)
			if not packet_valid:
				self.buffer = []
				continue
			if len(self.buffer) < BaseTypes.REM_PACKET_SIZE_REM_PACKET:
				break

			packet = REM_Packet()
			packet.decode(self.buffer[:BaseTypes.REM_PACKET_SIZE_REM_PACKET])

			rem_packet_size = BaseTypes.REM_PACKET_TYPE_TO_SIZE(packet.packetType)

			if len(self.buffer) < packet.payloadSize: 
				break

			if packet.packetType != BaseTypes.REM_PACKET_TYPE_REM_LOG and packet.payloadSize != rem_packet_size:
				self.buffer = bytes()
				continue

			packet_bytes = self.buffer[:packet.payloadSize]
			packet = BaseTypes.REM_PACKET_TYPE_TO_OBJ(packet_type)()
			packet.decode(packet_bytes)

			if packet.packetType == BaseTypes.REM_PACKET_TYPE_REM_LOG:
				message = packet_bytes[BaseTypes.REM_PACKET_SIZE_REM_LOG:]
				packet.message = message.decode()

			self.add_packet(packet)
			self.write_bytes(packet_bytes)
			self.buffer = self.buffer[packet.payloadSize:]

	def add_packet(self, packet: Any) -> None:
		"""
		Adds a packet to the packet buffer.

		Args:
			packet (Any): The packet to add.
		"""
		self.packet_buffer.append(packet)

	def write_bytes(self, _bytes: bytes) -> None:
		"""
		Writes bytes to the output file, if it exists.

		Args:
			_bytes (bytes): The bytes to write.
		"""
		if self.output_file is not None:
			self.output_file.write(_bytes)

	def has_packets(self) -> bool:
		"""
		Checks if there are any packets in the packet buffer.

		Returns:
			bool: True if there are packets in the buffer, False otherwise.
		"""
		return bool(self.packet_buffer)

	def get_next_packet(self) -> Optional[Any]:
		"""
		Gets the next packet from the packet buffer, if any.

		Returns:
			Optional[Any]: The next packet, or None if the buffer is empty.
		"""
		if self.has_packets():
			return self.packet_buffer.popleft()
		print("[REMParser] No packets in buffer")
		return None

	def parse_file(self, filepath: str, print_statistics: bool = True) -> None:
		"""
		Parses a file and optionally prints statistics about the parsed packets.

		Args:
			filepath (str): The path to the file to parse.
			print_statistics (bool, optional): Whether to print statistics about the parsed packets. Defaults to True.
		"""
		print(f"[REMParser] Parsing file {filepath}")
		with open(filepath, "rb") as file:
			self.buffer = file.read()
			self.process()

		if not print_statistics:
			return

		packet_counts: Dict[Type, int] = {}
		packet_timestamps: Dict[Type, Dict[str, float]] = {}

		print("   ", "PACKET TYPE".ljust(20), "COUNT", " ", "START DATE".ljust(23), " ", "STOP DATE".ljust(23), " ", "DURATION H:M:S:MS")
		for packet in self.packet_buffer:
			packet_type = type(packet)
			packet_counts.setdefault(packet_type, 0)
			packet_timestamps.setdefault(packet_type, {'start': packet.timestamp / 1000})
			packet_counts[packet_type] += 1
			packet_timestamps[packet_type]['stop'] = packet.timestamp / 1000
			# print(packet.timestamp, packet_type.__name__)

		for packet_type, count in packet_counts.items():
			start_sec, stop_sec = packet_timestamps[packet_type].values()
			start_msec, stop_msec = start_sec % 1, stop_sec % 1
			datetime_str_start = datetime.fromtimestamp(np.floor(start_sec)).strftime("%Y-%m-%d %H:%M:%S")  + (f".{start_msec:.3f}"[2:])
			datetime_str_stop  = datetime.fromtimestamp(np.floor(stop_sec )).strftime("%Y-%m-%d %H:%M:%S")  + (f".{stop_msec :.3f}"[2:])
			duration_sec = stop_sec - start_sec
			duration_str = str(timedelta(seconds=duration_sec))[:-3]
			# if i'ts log, dont print data/duration stuff
			if packet_type == BaseTypes.REM_Log:
				print("   ", packet_type.__name__.ljust(20), str(count).rjust(5))
			else:
				print("   ", packet_type.__name__.ljust(20), str(count).rjust(5), " ", datetime_str_start, " ", datetime_str_stop, " ", duration_str)


if __name__ == "__main__":
	print("Running REMParser directly")

	argparser = argparse.ArgumentParser()
	argparser.add_argument('input_file', nargs='?', default='latest.rembin', help='File to parse')
	args = argparser.parse_args()

	print("Parsing file", args.input_file)

	parser = REMParser(device=None)
	parser.parse_file(args.input_file)

	packet_dicts = []
	for packet in parser.packet_buffer:
		if type(packet) in [REM_RobotFeedback]:
			packet_dict = utils.packet_to_dict(packet)
			packet_dicts.append(packet_dict)

	# Split up packets into types
	packets_by_type = {}
	for packet in parser.packet_buffer:
		type_str = type(packet).__name__
		if type_str not in packets_by_type:
			packets_by_type[type_str] = []
		packets_by_type[type_str].append(utils.packet_to_dict(packet))

	output_file_no_ext = os.path.splitext(args.input_file)[0]

	if args.input_file == "latest.rembin":
		output_file_no_ext = os.path.join("logs", output_file_no_ext)

	for type_str in packets_by_type:
		packets = packets_by_type[type_str]
		
		# json
		output_file_json = f"{output_file_no_ext}_{type_str}.json"
		with open(output_file_json, 'w') as file:
			file.write(json.dumps(packets))

		# CSV
		output_file_csv = f"{output_file_no_ext}_{type_str}.csv"
		with open(output_file_csv, 'w') as file:
			header = ",".join(list(packets[0].keys()))
			file.write(header + "\n")
			for packet in packets:
				values = list(packet.values())
				string = ",".join([str(v) for v in values])
				file.write(string + "\n")

	print("Done!")