import os
import re
import sys
import time
from inspect import getmembers
from typing import Any, Dict, Optional
import libusb_package
import usb.core
import usb.backend.libusb1

# Add parent directory to path to allow importing from Core.Inc
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Core.Inc.roboteam_embedded_messages.python import REM_BaseTypes as BaseTypes
from Core.Inc.roboteam_embedded_messages.python.REM_BasestationConfiguration import REM_BasestationConfiguration
from Core.Inc.roboteam_embedded_messages.python.REM_RobotKillCommand import REM_RobotKillCommand
from Core.Inc.roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand
from Core.Inc.roboteam_embedded_messages.python.REM_RobotBuzzer import REM_RobotBuzzer
from Core.Inc.roboteam_embedded_messages.python.REM_RobotGetPIDGains import REM_RobotGetPIDGains
from Core.Inc.roboteam_embedded_messages.python.REM_RobotSetPIDGains import REM_RobotSetPIDGains

def print_complete_packet(rc: Any) -> None:
	"""
	Prints a REM packet of any type in a nice format.

	Args:
		rc (Any): The REM packet to print.
	"""
	types_allowed = [int, str, bool, float]
	members = [m for m in getmembers(rc) if isinstance(m[1], tuple(types_allowed)) and not m[0].startswith("__")]

	if not members:
		print("No printable members found.")
		return

	max_length = max(len(m[0]) for m in members)
	title = re.findall(r"_(\w+) ", str(rc))[0]

	lines = [("┌─ %s " % title) + ("─" * 100)[:max_length * 2 + 2 - len(title)] + "┐"]
	lines += ["│ %s : %s │" % (m[0].rjust(max_length), str(m[1]).strip().ljust(max_length)) for m in members]
	lines += ["└" + ("─" * (max_length * 2 + 5)) + "┘"]

	print("\n".join(lines))

def packet_to_dict(rc: Any) -> Dict[str, Any]:
	"""
	Converts a REM packet of any type to a dictionary.

	Args:
		rc (Any): The REM packet to convert.

	Returns:
		Dict[str, Any]: The REM packet as a dictionary.
	"""
	types_allowed = (int, str, bool, float)
	members = {k: v for k, v in getmembers(rc) if isinstance(v, types_allowed) and not k.startswith("__")}
	return members

def generate_basestation_config_command(is_yellow_team: bool) -> REM_BasestationConfiguration:
	"""
	Generate a basestation configuration command.

	Args:
		is_yellow_team (bool): The channel of the basestation.

	Returns:
		REM_BasestationConfiguration: The generated command.
	"""
	config_command = REM_BasestationConfiguration()
	config_command.packetType = BaseTypes.REM_PACKET_TYPE_REM_BASESTATION_CONFIGURATION
	config_command.toBS = True
	config_command.fromPC = True
	config_command.remVersion = BaseTypes.REM_LOCAL_VERSION
	config_command.payloadSize = BaseTypes.REM_PACKET_SIZE_REM_BASESTATION_CONFIGURATION
	config_command.channel = 0 if is_yellow_team else 1
	return config_command

def generate_empty_robot_kill_command() -> REM_RobotKillCommand:
	"""
	Generate an empty robot kill command.

	Returns:
		REM_RobotKillCommand: The generated command.
	"""
	kill_command = REM_RobotKillCommand()
	kill_command.packetType = BaseTypes.REM_PACKET_TYPE_REM_ROBOT_KILL_COMMAND
	kill_command.fromPC = True
	kill_command.remVersion = BaseTypes.REM_LOCAL_VERSION
	kill_command.payloadSize = BaseTypes.REM_PACKET_SIZE_REM_ROBOT_KILL_COMMAND
	kill_command.timestamp = int(time.time()*1000)
	return kill_command

def generate_empty_robot_command() -> REM_RobotCommand:
	"""
	Generate an empty robot command.

	Returns:
		REM_RobotCommand: The generated command.
	"""
	cmd = REM_RobotCommand()
	cmd.packetType = BaseTypes.REM_PACKET_TYPE_REM_ROBOT_COMMAND
	cmd.fromPC = True
	cmd.remVersion = BaseTypes.REM_LOCAL_VERSION
	cmd.payloadSize = BaseTypes.REM_PACKET_SIZE_REM_ROBOT_COMMAND
	cmd.timestamp = int(time.time()*1000)
	cmd.sendStateInfo = True
	cmd.useYaw = True
	return cmd

def generate_empty_robot_buzzer() -> REM_RobotBuzzer:
	"""
	Generate an empty buzzer command.

	Returns:
		REM_RobotBuzzer: The generated command.
	"""
	cmd = REM_RobotBuzzer()
	cmd.packetType = BaseTypes.REM_PACKET_TYPE_REM_ROBOT_BUZZER
	cmd.fromPC = True
	cmd.remVersion = BaseTypes.REM_LOCAL_VERSION
	cmd.payloadSize = BaseTypes.REM_PACKET_SIZE_REM_ROBOT_BUZZER
	cmd.timestamp = int(time.time()*1000)
	cmd.duration = 4.0
	cmd.period = 2000
	return cmd

def generate_empty_set_pid_gains() -> REM_RobotSetPIDGains:
	"""
	Generate an empty set PID gains command.

	Returns:
		REM_RobotSetPIDGains: The generated command.
	"""
	setPID = REM_RobotSetPIDGains()
	setPID.packetType = BaseTypes.REM_PACKET_TYPE_REM_ROBOT_SET_PIDGAINS
	setPID.fromPC = True
	setPID.remVersion = BaseTypes.REM_LOCAL_VERSION
	setPID.payloadSize = BaseTypes.REM_PACKET_SIZE_REM_ROBOT_SET_PIDGAINS
	setPID.timestamp = int(time.time()*1000)

	setPID.PbodyX = 0
	setPID.IbodyX = 0
	setPID.DbodyX = 0
	setPID.PbodyY = 0
	setPID.IbodyY = 0
	setPID.DbodyY = 0
	setPID.PbodyW = 0
	setPID.IbodyW = 0
	setPID.DbodyW = 0
	setPID.PbodyYaw = 0
	setPID.IbodyYaw = 0
	setPID.DbodyYaw = 0
	setPID.Pwheels = 0
	setPID.Iwheels = 0
	setPID.Dwheels = 0

	return setPID

def generate_empty_get_pid_gains() -> REM_RobotGetPIDGains:
	"""
	Generate an empty get PID gains command.

	Returns:
		REM_RobotGetPIDGains: The generated command.
	"""
	cmd = REM_RobotGetPIDGains()
	cmd.packetType = BaseTypes.REM_PACKET_TYPE_REM_ROBOT_GET_PIDGAINS
	cmd.fromPC = True
	cmd.remVersion = BaseTypes.REM_LOCAL_VERSION
	cmd.payloadSize = BaseTypes.REM_PACKET_SIZE_REM_ROBOT_GET_PIDGAINS
	cmd.timestamp = int(time.time()*1000)
	return cmd