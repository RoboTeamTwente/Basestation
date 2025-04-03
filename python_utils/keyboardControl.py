import os
import math
import sys
import time
import threading
import argparse
from datetime import datetime
from typing import Callable, Dict, List

from pynput import keyboard

# Add parent directory to path to allow importing from Core.Inc
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Core.Inc.roboteam_embedded_messages.python import REM_BaseTypes as BaseTypes
from Core.Inc.roboteam_embedded_messages.python.REM_Log import REM_Log
from REMParser import REMParser
import utils

MAX_SPEED = 0.7
KICK_SPEED =1.0
ROTATION_SPEED = 3 # radians per second
BASESTATION_FREQUENCY = 60 # ticks per second

parser = argparse.ArgumentParser(description='Keyboard to robot controller')
parser.add_argument("robot_ids", type=int, nargs='+', help="An array of integers for the robot ids")
parser.add_argument("--simulate", action="store_true", help="Simulate the basestation")
args = parser.parse_args()

class EventHandler:
    """Handles events and updates the status of the system."""

    def __init__(self, shutdown: Callable[[], None]) -> None:
        self.shutdown = shutdown
        self.running = True
        self.events: List[str] = []

    def start(self) -> None:
        """Starts the event handling loop."""
        self.thread = threading.Thread(target=self.loop)
        self.thread.start()

    def record_event(self, id: int, event: str) -> None:
        """Records a new event."""
        current_time = datetime.now().strftime("%H:%M:%S")
        id = int(id) + 1
        self.events.insert(0, f"[{id} | {current_time}] {event}")
        self.events = self.events[:5]

    def loop(self) -> None:
        """Main event handling loop."""
        try:
            while self.running:
                for event in self.events:
                    print(event)
                self.events = []
                time.sleep(0.1)
        except Exception as e:
            self.record_event(-1, str(e))
            print(e)
            self.shutdown()

class KeyboardHandler:
    """Handles keyboard input and controls the robots."""

    def __init__(self) -> None:
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        self.keyboard_input: Dict[str, bool] = {'w': False, 's': False, 'a': False, 'd': False, 'k': False, 'c': False, 'b': False, 'q': False, 'e': False}

    def on_press(self, key: keyboard.Key) -> None:
        """Handles key press events."""
        try:
            char = key.char
            if char in self.keyboard_input:
                self.keyboard_input[char] = True
        except AttributeError:
            pass

    def on_release(self, key: keyboard.Key) -> None:
        """Handles key release events."""
        try:
            char = key.char
            if char in self.keyboard_input:
                self.keyboard_input[char] = False
        except AttributeError:
            pass

    def get_keyboard_input(self) -> Dict[str, bool]:
        """Returns the current keyboard input."""
        return self.keyboard_input

class BasestationHandler:
    """Handles communication with the basestation."""

    def __init__(self, event_handler: EventHandler, shutdown: Callable[[], None], simulate: bool) -> None:
        self.shutdown = shutdown
        self.packet_Hz = BASESTATION_FREQUENCY
        self.running = True
        if simulate:
            self.basestation = utils.open_simulated_basestation()
        else:
            self.basestation = utils.open_continuous(timeout=0.01)
        self.event_handler = event_handler
        self.thread = threading.Thread(target=self.loop)
        self.thread.start()
        self.command = utils.generate_empty_robot_command()
        self.yaw = 0
        self.dribblerPressed = False

    def loop(self) -> None:
        print("starting base loop")
        """Main loop for handling basestation communication."""
        try:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            log_dir = os.path.join(current_dir, "logs/keyboard")
            os.makedirs(log_dir, exist_ok=True)
            filename = datetime.now().strftime("%Y-%m-%d_%H:%M:%S") + ".rembin"
            logger = REMParser(self.basestation, f"{log_dir}/{filename}")

            last_written = time.time()
            while self.running:
                time_till_next_tick = last_written + 1. / self.packet_Hz - time.time()
                time.sleep(max(0, time_till_next_tick))
                last_written += 1. / self.packet_Hz

                payload = self.get_payload(keyboard_handler.get_keyboard_input())
                for robot_id in args.robot_ids:
                    payload.toRobotId = robot_id
                    self.basestation.write(self.command)
                    logger.write_bytes(payload.encode())

                logger.read()
                logger.process()

                def handle_rem_log(rem_log: REM_Log) -> None:
                    log_from = "[?]  "
                    if rem_log.fromBS:
                        log_from = f" Keyboard -> Robots {args.robot_ids} | "

                    if not rem_log.fromPC and not rem_log.fromBS:
                        log_from = f"[{str(rem_log.fromRobotId).rjust(2)}] "

                    message = rem_log.message.strip()
                    message = log_from + message

                    nwhitespace = os.get_terminal_size().columns - len(message) - 2
                    print(f"\r{message}{' ' * nwhitespace}")

                while logger.has_packets():
                    packet = logger.get_next_packet()
                    if isinstance(packet, REM_Log):
                        handle_rem_log(packet)
        except Exception as e:
            self.event_handler.record_event(-1, str(e))
            print(e)
            self.shutdown()

    def get_payload(self, keyboard_input: Dict[str, bool]) -> bytes:
        """Generates the command payload for the robot based on keyboard inputs."""
        self.yaw += (keyboard_input['q'] - keyboard_input['e']) * ROTATION_SPEED / BASESTATION_FREQUENCY
        # ADDED
        # Value on how much robot moves per key press
        MOVE_STEP = 0.5
        if keyboard_input['w'] or keyboard_input['s'] or keyboard_input['a'] or keyboard_input['d']:
            # robot moves if target and current position is not equal
            if ((self.command.targetX != self.command.currentX ) and (self.command.targetY != self.command.currentY)): 
                deltaX = (keyboard_input['w'] - keyboard_input['s']) * MOVE_STEP
                deltaY = (keyboard_input['a'] - keyboard_input['d']) * MOVE_STEP
                self.command.targetX = deltaX + self.command.currentX
                self.command.targetY = deltaY + self.command.currentY
            else:
                self.command.targetX = self.command.currentX
                self.command.targetY = self.command.currentY
        else:
            # if no key is pressed, robot stays put
            self.command.targetX = self.command.currentX
            self.command.targetY = self.command.currentY
            
        # END ADDED
        self.command.doKick = keyboard_input['k']
        self.command.doForce = keyboard_input['k'] or keyboard_input['c']
        self.command.kickChipPower = KICK_SPEED
        self.command.doChip = keyboard_input['c']
        if (keyboard_input['b'] and not self.dribblerPressed):
            self.command.dribblerOn = not self.command.dribblerOn
            self.dribblerPressed = True
        elif not keyboard_input['b']:
            self.dribblerPressed = False
        self.command.yaw = self.yaw

        # Return the command payload
        return self.command

def shutdown() -> None:
    """Shuts down the system."""
    print("Exiting")
    event_handler.running = False
    basestation_handler.running = False

event_handler = EventHandler(shutdown)

def thread_exception_handler(args: threading.ExceptHookArgs) -> None:
    """Handles exceptions raised in threads."""
    event_handler.record_event(-1, f"Caught: {args}")

threading.excepthook = thread_exception_handler

basestation_handler = BasestationHandler(event_handler, shutdown, args.simulate)
keyboard_handler = KeyboardHandler()
event_handler.start()

try:
    event_handler.thread.join()
except:
    shutdown()
