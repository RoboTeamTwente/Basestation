import time
import math
import serial
import zmq
import State_pb2
from REMParser import REMParser
import roboteam_embedded_messages.python.REM_BaseTypes as BaseTypes
from roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand
import utils

class RobotCommander:
	def __init__(self, robot_id: int, is_yellow: bool):
		self.robot_id = robot_id
		self.is_yellow = is_yellow
		self.tick_counter = 0
		self.packetHz = 60
		self.basestation = None

	def create_empty_robot_command(self, robot_angle) -> REM_RobotCommand:
		cmd = REM_RobotCommand()
		cmd.header = BaseTypes.REM_PACKET_TYPE_REM_ROBOT_COMMAND
		cmd.toRobotId = self.robot_id
		cmd.toColor = 0 if self.is_yellow else 1
		cmd.fromPC = True    
		cmd.remVersion = BaseTypes.REM_LOCAL_VERSION
		cmd.payloadSize = BaseTypes.REM_PACKET_SIZE_REM_ROBOT_COMMAND
		cmd.timestamp = int(time.time()*100)
		cmd.useAbsoluteAngle = 1
		cmd.angle = 0 # This angle can be overwritten by the rotate_robot function. 0 is already the default, for readiablity it's still included.
		cmd.useCameraAngle = 1
		cmd.cameraAngle = robot_angle
		return cmd

	def drive_robot_to_position(self, current_x: float, current_y: float, target_x: float, target_y: float, current_robot_angle: float) -> REM_RobotCommand:
		cmd = self.create_empty_robot_command(current_robot_angle)
		distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
		direction = math.atan2(target_y - current_y, target_x - current_x)
		cmd.theta = -direction
		cmd.rho = min(distance, 0.5) # Limit the speed to prevent sad things from happening
		cmd.rho = max(cmd.rho, 0.3) # This line can be removed if you have the code that the robot can drive low speeds :)). We tested on robot 7 which didn't have the code yet I guess
		return cmd

	def rotate_robot(self, current_robot_angle: float, target_robot_angle) -> REM_RobotCommand:
		cmd = self.create_empty_robot_command(current_robot_angle)
		cmd.angle = target_robot_angle
		return cmd

	def should_stop_drive_to_position(self, distance: float) -> bool:
		return distance < 0.1 # Change to whatever you need :))

class WorldSubscriber:
	def __init__(self, address="127.0.0.1", port="5558"):
		self.context = zmq.Context()
		self.socket = self.context.socket(zmq.SUB)
		self.socket.connect(f'tcp://{address}:{port}')
		self.socket.setsockopt_string(zmq.SUBSCRIBE, '')
		print(f"Connected to {address}:{port} as subscriber")

	def get_robot_position(self, id_vision: int, is_yellow: bool) -> tuple:
		data = self.socket.recv()
		world_state = State_pb2.State()
		world_state.ParseFromString(data)
		while True:
			for robot in (world_state.last_seen_world.yellow if is_yellow else world_state.last_seen_world.blue):
				if robot.id == id_vision:
					return robot.pos.x, robot.pos.y
			print("Robot not found, waiting for new data")
			time.sleep(1/60*0.1)

	def get_robot_angle(self, id_vision: int, is_yellow: bool) -> float:
		data = self.socket.recv()
		world_state = State_pb2.State()
		world_state.ParseFromString(data)
		while True:
			for robot in (world_state.last_seen_world.yellow if is_yellow else world_state.last_seen_world.blue):
				if robot.id == id_vision:
					return robot.angle
			print("Robot not found, waiting for new data")
			time.sleep(1/60*0.1)

def command_robot(id_vision: int, id_robot: int, is_yellow: bool, target_x: float = None, target_y: float = None, target_angle: float = None, calibrate: bool = False) -> None:
	commander = RobotCommander(id_robot, is_yellow)
	subscriber = WorldSubscriber()
	try:
		last_tick_time = time.time()
		if commander.basestation is None or not commander.basestation.isOpen():
			commander.basestation = utils.openContinuous(timeout=0.01)
			print("Basestation opened")
		parser = REMParser(commander.basestation)
		while True:
			current_time = time.time()
			s_until_next_tick = last_tick_time + 1./commander.packetHz - current_time
			tick_required = s_until_next_tick < 0
			if not tick_required and 0.1 / commander.packetHz < s_until_next_tick: 
				time.sleep(0.1 / commander.packetHz)
			if tick_required:
				current_robot_angle = subscriber.get_robot_angle(id_vision, is_yellow)
				last_tick_time += 1./commander.packetHz
				commander.tick_counter += 1
				if calibrate:
					cmd = commander.create_empty_robot_command(current_robot_angle)
					# This simply stops after 1/3seconds, cause it will be fine in that time. No clue how much time it really takes. Same for rotation
					if commander.tick_counter > 20:
						print("Done calibrating")
						print("\n")
						break
				elif target_angle is not None:
					cmd = commander.rotate_robot(current_robot_angle, target_angle)
					if commander.tick_counter > 20:
						print("Done rotating")
						print("\n")
						break
				else:
					current_x, current_y = subscriber.get_robot_position(id_vision, is_yellow)
					cmd = commander.drive_robot_to_position(current_x, current_y, target_x, target_y, current_robot_angle)
					distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
					if commander.should_stop_drive_to_position(distance):
						print("Done driving to position")
						print("\n")
						break
				cmd_encoded = cmd.encode()
				commander.basestation.write(cmd_encoded)
				parser.writeBytes(cmd_encoded)
			parser.read()
			parser.process()
	# All these error's shouldn't happen so these excepts are kinda useless.
	except serial.SerialException as se:
		print("SerialException", se)
		commander.basestation = None
	except KeyError as ke:
		print("[Error] KeyError", ke, "{0:b}".format(int(str(ke))))
	except Exception as e:
		print("[Error]", e)
		commander.basestation = None
		raise e

if __name__ == "__main__":
	id_vision = 13 # The id of the dots on top of the robot which visions sees
	id_robot = 15 # The id of the robot set with the pins
	is_yellow = True # Indicate if the robot we are talking to is yellow
	subscriber = WorldSubscriber()
	print("Starting, don't forget to start roboteam observer :)) (if you do forget, nothing will work)")
	command_robot(id_vision, id_robot, is_yellow, calibrate=True) # Command to calibrate the angle, will also happen during everything else
	# Funny loop driving in a triangle. If you grab the robot and move it elsewhere, it will just continue :)
	while True:
		command_robot(id_vision, id_robot, is_yellow, target_x=0, target_y=-2) 
		command_robot(id_vision, id_robot, is_yellow, target_x=-1, target_y=-1)
		command_robot(id_vision, id_robot, is_yellow, target_x=0, target_y=0)
	# This command can be used to rotate the boy to some angle if you ever want to do that, positive angle is counter clockwise.
	# command_robot(id_vision, id_robot, is_yellow, target_angle=math.pi/2)
