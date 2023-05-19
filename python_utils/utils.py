import os
import time
import serial
from bitarray import bitarray
from bitarray.util import int2ba, ba2int
from inspect import getmembers
import enum
import re
import usb.core
import usb.util


class ConnectionVersion(enum.Enum):
	BASESTATION_V1 = 0
	ST_LINK = 1
	BASESTATION_V2 = 2


class Basestation(object):

	def __init__(self, version=ConnectionVersion.BASESTATION_V1):
		"""
		Opens a connection with a basestation.

		:param version Which hardware version of the basestation are we using?
		"""
		self.byte_buffer = bytes()

		if type(version) != ConnectionVersion:
			raise NotImplementedError(f"Incompatible version selected")

		self.version = version
		
		if self.version == ConnectionVersion.BASESTATION_V1 or self.version == ConnectionVersion.ST_LINK:
			self.connection = openContinuous(timeout=0.01)
		elif self.version == ConnectionVersion.BASESTATION_V2:
			self._connectUSB()

	def _connectUSB(self):
		"""
		Opens a connection with the basestation.
		"""
		# Find the ST usb device
		device = None
		i = 0
		while device is None:
			# TODO: Should use findAll=True, in order to detect multiple basestations.
			device = usb.core.find(idVendor=1155, idProduct=25432)
			if device is None:
				print(f"\r", end='')
				print(f"\r[connect USB] Basestation could not be found {chr(33 + i)}", end='')
				i = (i + 1) % 93
				time.sleep(0.1)
		print(f"\r[connect USB] Connected to basestation")

		# Configure the basestation for high and low priority interfaces
		device.set_configuration()
		device.set_interface_altsetting(interface=0, alternate_setting=1)
		configuration = device.get_active_configuration()
		
		# Find the high and low priority interfaces
		high_prio_interface = configuration[(0, 1)]
		low_prio_interface = configuration[(1, 0)]

		# Connect to the endpoints (IN, OUT) for each interface (high, low)
		self._high_prio_out = usb.util.find_descriptor(
			high_prio_interface,
			# match the first OUT endpoint
			custom_match= lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT
		)
		self._high_prio_in = usb.util.find_descriptor(
			high_prio_interface,
			# match the first IN endpoint,
			custom_match= lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
		)
		self._low_prio_out = usb.util.find_descriptor(
			low_prio_interface,
			# match the first OUT endpoint
			custom_match= lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT
		)
		self._low_prio_in = usb.util.find_descriptor(
			low_prio_interface,
			# match the first IN endpoint,
			custom_match= lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
		)

	def write(data, high_priority=False):
		"""
		Writes data to the basestation.

		:param high_priority When set to true, the data is written with a high priority. If set to False we use the low priority mode. (Only relevant for basestation V2)
		"""
		if self.version == ConnectionVersion.BASESTATION_V1 or self.version == COnenctionVersion.ST_LINK:
			self.connection.write(data)
		elif self.version == ConenctionVersion.BASESTATION_V2:
			if high_priority:
				self._high_prio_out.write(data)
			else:
				self._low_prio_out.write(data)

	def read(high_priority=False):
		"""
		Reads data from the basestation.

		:param high_priority When set to true, the data is read from the high priority interface. If set to False we use the low priority interface. (Only relevant for basestation V2)
		"""
		if self.version == ConnectionVersion.BASESTATION_V1 or self.version == ConnectionVersion.ST_LINK:
			bytes_waiting = self.connection.inWaiting()
			if bytes_waiting > 0:
				self.byte_buffer += self.connection.read(bytes_waiting)
		elif self.version == ConnectionVersion.BASESTATION_V2:
			# TODO: How do you know how much bytes one should read?
			if high_priority:
				self.byte_buffer += self._high_prio_in.read(self._high_prio_in.wMaxPacketSize)
			else:
				self.byte_buffer += self._low_prio_in.read(self._low_prio_in.wMaxPacketSize)


def openPort(port=None, suppressError = True, timeout=None):
	ser = None
	if port is None:
		return None

	try:
		ser = serial.Serial(
		    port=port,
		    baudrate=115200,
		    parity=serial.PARITY_NONE,
		    stopbits=serial.STOPBITS_ONE,
		    bytesize=serial.EIGHTBITS,
		    timeout=timeout
		)
	except serial.serialutil.SerialException as e:
		print("[open][SerialException] Could not open port", port)

	return ser

def openContinuous(*args, **kwargs):
	ser = None
	i = -1

	while ser is None:
		i = (i + 1) % 93

		if "port" not in kwargs or kwargs["port"] is None:
			kwargs["port"] = getBasestationPath()
		
		if "port" not in kwargs or kwargs["port"] is None:
			kwargs["port"] = getSTLinkPath()
		
		if kwargs["port"] is None:
			print("\r[openContinuous] Basestation path not found %s " % chr(33 + i), end="")
		else:
			ser = openPort(**kwargs, suppressError = True)	
			print("\r[openContinuous] %s " % chr(33 + i), end="")
		
		time.sleep(0.1)
	print(f"\r[openContinuous] Basestation opened at {ser.port}")
	return ser

def getBasestationPath():
	try:
		usb_devices = os.listdir("/dev/serial/by-id")
		basestations = [device for device in usb_devices if device.startswith("usb-RTT_BaseStation")]
		if 0 < len(basestations):
			return os.path.join("/dev/serial/by-id", basestations[0])
		return None
	except Exception:
		return None

def getSTLinkPath():
	try:
		usb_devices = os.listdir("/dev/serial/by-id")
		stlinks = [device for device in usb_devices if device.startswith("usb-STMicroelectronics_STM32_STLink")]
		if 0 < len(stlinks):
			return os.path.join("/dev/serial/by-id", stlinks[0])
		return None
	except Exception:
		return None

def printCompletePacket(rc):
	types_allowed = [int, str, bool, float]
	maxLength = max([len(k) for k, v in getmembers(rc)])
	title = re.findall(r"_(\w+) ", str(rc))[0]
	
	lines = [("┌─ %s "%title) + ("─"*100)[:maxLength*2+2-len(title)] + "┐" ]	
	members = [ [k,v] for k,v in getmembers(rc) if type(v) in types_allowed and not k.startswith("__")]

	lines += [ "│ %s : %s │" % ( k.rjust(maxLength) , str(v).strip().ljust(maxLength) ) for k, v in members ]
	lines += [ "└" + ("─"*(maxLength*2+5)) + "┘"]
	print("\n".join(lines))

def packetToDict(rc):
	types_allowed = [int, str, bool, float]
	members = { k:v for k,v in getmembers(rc) if type(v) in types_allowed and not k.startswith("__") }
	return members