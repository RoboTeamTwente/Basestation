import zmq
import argparse
import State_pb2
from datetime import datetime

WORLD_ADDRESS = "127.0.0.1"
WORLD_PORT = "5558"
first_data_point = True

# Parse possible user arguments
parser = argparse.ArgumentParser()
parser.add_argument('--world-address', '-wa', help="The address on which world is broadcasting (default 127.0.0.1)")
parser.add_argument('--world-port', '-wp', help="The port on which world is broadcasting (default 5558)")
args = parser.parse_args()

world_address = WORLD_ADDRESS if not args.world_address else parser.world_address
world_port = WORLD_PORT if not args.world_port else parser.world_port

# Connect to the world publisher socket
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect(f'tcp://{world_address}:{world_port}')
socket.setsockopt_string(zmq.SUBSCRIBE, '')
print(f"Connected to {world_address}:{world_port} as subscriber")

# Start reading incoming protobuf packages
while True:
    data = socket.recv()
    world_state = State_pb2.State()
    world_state.ParseFromString(data)

    world = world_state.last_seen_world
    time = world.time
    
    for robot in world.yellow:
        if first_data_point:
            print("First data point was read")
            first_data_point = False

        # make a command to drive the robot to the position (3,2)
        
        # output.write(f"{time},{robot.id},{robot.pos.x},{robot.pos.y},{robot.angle},{robot.vel.x},{robot.vel.y},{robot.w}\r\n")
    for robot in world.blue:
        if first_data_point:
            print("First data point was read")
            first_data_point = False
        # output.write(f"{time},{robot.id},{robot.pos.x},{robot.pos.y},{robot.angle},{robot.vel.x},{robot.vel.y},{robot.w}\r\n")
