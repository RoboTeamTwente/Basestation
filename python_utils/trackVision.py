import zmq
import argparse
from proto import State_pb2
from datetime import datetime
WORLD_ADDRESS = "127.0.0.1"
WORLD_PORT = "5558"

# Parse possible user arguments
parser = argparse.ArgumentParser()
parser.add_argument('output_dir', help="Where the resulting tracking data should be stored")
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

datetime_str = datetime.now().strftime("%Y%m%d_%H%M%S")
output_filename = f'log_{datetime_str}_Vision.csv'
output_file = args.output_dir + "/" + output_filename
with open(output_file, 'x') as output:

    # Create a csv so define all the column names
    output.write("time_in_ms,id,pos_x,pos_y,angle,vel_x,vel_y,w\r\n")

    # Start reading incoming protobuf packages
    while True:
        data = socket.recv()
        world_state = State_pb2.State()
        world_state.ParseFromString(data)

        world = world_state.last_seen_world
        time = world.time
        for robot in world.yellow:
            output.write(f"{time},{robot.id},{robot.pos.x},{robot.pos.y},{robot.angle},{robot.vel.x},{robot.vel.y},{robot.w}\r\n")
            output.flush()
        for robot in world.blue:
            output.write(f"{time},{robot.id},{robot.pos.x},{robot.pos.y},{robot.angle},{robot.vel.x},{robot.vel.y},{robot.w}\r\n")
            output.flush()