import os
import math
import sys
import time
import threading
import argparse
from datetime import datetime
from glob import glob
from typing import Callable, Dict, List, Optional

import numpy as np

# Add parent directory to path to allow importing from Core.Inc and REMParser
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


from Core.Inc.roboteam_embedded_messages.python import REM_BaseTypes as BaseTypes
from Core.Inc.roboteam_embedded_messages.python.REM_Log import REM_Log
from Core.Inc.roboteam_embedded_messages.python.REM_RobotFeedback import REM_RobotFeedback

from REMParser import REMParser
import utils

# Constants
SQUARE_SIDE_LENGTH = 0.7  # meters
SQUARE_SPEED = 0.8  # meters per second
BASESTATION_FREQUENCY = 60  # ticks per second

class SquareMovement:
    def __init__(self, robot_id: int) -> None:
        self.robot_id = robot_id
        self.running = True
        self.command = utils.generate_empty_robot_command()

        # Initialize basestation connection
        self.basestation = utils.open_continuous(timeout=0.01)

        # Initialize logger
        current_dir = os.path.dirname(os.path.abspath(__file__))
        log_dir = os.path.join(current_dir, "logs/squareMovement")
        os.makedirs(log_dir, exist_ok=True)
        filename = datetime.now().strftime("%Y-%m-%d_%H:%M:%S") + ".rembin"
        self.logger = REMParser(self.basestation, f"{log_dir}/{filename}")

    def generate_square_command(self, time_elapsed: float) -> bytes:
        """Generates movement commands to move the robot in a square with rotation."""
        # Calculate the total time to complete one side of the square
        side_time = SQUARE_SIDE_LENGTH / SQUARE_SPEED

        # Determine which side of the square the robot is on
        side = int((time_elapsed % (4 * side_time)) // side_time)

        # Calculate velocity components and yaw based on the current side
        if side == 0:  # Move right
            velocity_x = SQUARE_SPEED
            velocity_y = 0
            yaw = 0  # Facing right
        elif side == 1:  # Move up
            velocity_x = 0
            velocity_y = SQUARE_SPEED
            yaw = math.pi / 2  # Facing up
        elif side == 2:  # Move left
            velocity_x = -SQUARE_SPEED
            velocity_y = 0
            yaw = math.pi  # Facing left
        elif side == 3:  # Move down
            velocity_x = 0
            velocity_y = -SQUARE_SPEED
            yaw = -math.pi / 2  # Facing down

        # Set command parameters
        self.command.toRobotId = self.robot_id
        self.command.rho = math.sqrt(velocity_x**2 + velocity_y**2)
        self.command.theta = math.atan2(velocity_y, velocity_x)
        self.command.yaw = yaw  # Include rotation for square movement

        return self.command

    def loop(self) -> None:
        """Main loop for sending movement commands."""
        try:
            print(f"Starting square movement for robot {self.robot_id}")
            start_time = time.time()

            while self.running:
                # Calculate elapsed time
                time_elapsed = time.time() - start_time

                # Generate and send movement command
                payload = self.generate_square_command(time_elapsed)
                self.basestation.write(payload)
                self.logger.write_bytes(payload.encode())

                # Sleep to maintain frequency
                time.sleep(1 / BASESTATION_FREQUENCY)
        except Exception as e:
            print(f"Error: {e}")
            self.shutdown()

    def shutdown(self) -> None:
        """Shuts down the system."""
        print("Shutting down square movement...")
        self.running = False
        self.basestation.close()

if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Move a robot in a square.")
    parser.add_argument("robot_id", type=int, help="ID of the robot to control")
    args = parser.parse_args()

    # Initialize and start square movement
    square_movement = SquareMovement(robot_id=args.robot_id)

    try:
        square_movement.loop()
    except KeyboardInterrupt:
        square_movement.shutdown()