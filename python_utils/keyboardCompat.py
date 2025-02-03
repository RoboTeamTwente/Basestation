import os
import math
import sys
import time
import threading
import argparse
from datetime import datetime
from typing import Callable, Dict, List
import libusb_package
import usb.core
import usb.util
import usb.backend.libusb1
import numpy as np
from pynput import keyboard
import utilscompat as utils

# Add parent directory to path to allow importing from Core.Inc
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Core.Inc.roboteam_embedded_messages.python import REM_BaseTypes as BaseTypes
from Core.Inc.roboteam_embedded_messages.python.REM_RobotFeedback import REM_RobotFeedback
from Core.Inc.roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand
from Core.Inc.roboteam_embedded_messages.python.REM_RobotStateInfo import REM_RobotStateInfo
from Core.Inc.roboteam_embedded_messages.python.REM_Log import REM_Log
from visualization import visualize
from REMParserCompat import REMParser, BasestationDevice

MAX_SPEED = 1.0
KICK_SPEED =1.0
ROTATION_SPEED = 3 # radians per second
BASESTATION_FREQUENCY = 60 # ticks per second

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
            print("Error in event handling loop: " + str(e))
            self.shutdown()

class KeyboardHandler:
    """Handles keyboard input and controls the robots."""

    def __init__(self) -> None:
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        self.keyboard_input: Dict[keyboard.KeyCode, bool] = {keyboard.KeyCode.from_char('w'): False, keyboard.KeyCode.from_char('s'): False, keyboard.KeyCode.from_char('a'): False, keyboard.KeyCode.from_char('d'): False, keyboard.KeyCode.from_char('k'): False, keyboard.KeyCode.from_char('c'): False, keyboard.KeyCode.from_char('b'): False, keyboard.KeyCode.from_char('q'): False, keyboard.KeyCode.from_char('e'): False, keyboard.Key.esc: False}

    def on_press(self, key: keyboard.KeyCode) -> None:
        """Handles key press events."""
        try:
            # print(key + " Pressed")
            if key in self.keyboard_input:
                self.keyboard_input[key] = True
        except AttributeError:
            pass

    def on_release(self, key: keyboard.KeyCode) -> None:
        """Handles key release events."""
        try:
            if key in self.keyboard_input:
                self.keyboard_input[key] = False
        except AttributeError:
            pass

    def get_keyboard_input(self) -> Dict[keyboard.KeyCode, bool]:
        """Returns the current keyboard input."""
        return self.keyboard_input

class BasestationHandler:
    """Handles communication with the basestation."""

    def __init__(self, event_handler: EventHandler, shutdown: Callable[[], None], keyboard_handler: KeyboardHandler) -> None:
        self.shutdown = shutdown
        self.packet_Hz = BASESTATION_FREQUENCY
        self.running = True
        self.basestation = BasestationDevice(timeout=10)
        self.event_handler = event_handler
        self.keyboard_handler = keyboard_handler
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
            log_dir = os.path.join(current_dir, "logs\\keyboard")
            os.makedirs(log_dir, exist_ok=True)
            filename = datetime.now().strftime("%Y-%m-%d_%H:%M:%S") + ".rembin"
            # logger = REMParser(self.basestation, f"{log_dir}\\{filename}")
            logger = REMParser(self.basestation, None)

            last_written = time.time()
            last_packet_feedback = None
            last_packet_state_info = None
            while self.running:
                image_vis = np.zeros((500, 500, 3), dtype=float)
                time_till_next_tick = last_written + 1. / self.packet_Hz - time.time()
                time.sleep(max(0, time_till_next_tick))
                last_written += 1. / self.packet_Hz

                if keyboard_handler.get_keyboard_input()[keyboard.Key.esc]:
                    self.shutdown()
                    break
                payload = self.get_payload(keyboard_handler.get_keyboard_input())
                for robot_id in args.robot_ids:
                    payload.toRobotId = robot_id
                    encoded = payload.encode()
                    self.basestation.write(encoded)
                    logger.write_bytes(encoded)

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
                    elif isinstance(packet, REM_RobotFeedback):
                        last_packet_feedback = packet
                    elif isinstance(packet, REM_RobotStateInfo):
                        last_packet_state_info = packet

                image_vis = visualize(args, image_vis, last_packet_feedback, last_packet_state_info, self.command)
        except Exception as e:
            self.event_handler.record_event(-1, str(e))
            print("Error in main loop: " + e)
            self.shutdown()

    def get_payload(self, keyboard_input: Dict[keyboard.KeyCode, bool]) -> REM_RobotCommand:
        """Generates the command payload for the robot based on keyboard inputs."""
        q_key = keyboard.KeyCode.from_char("q")
        e_key = keyboard.KeyCode.from_char("e")
        w_key = keyboard.KeyCode.from_char("w")
        s_key = keyboard.KeyCode.from_char("s")
        a_key = keyboard.KeyCode.from_char("a")
        d_key = keyboard.KeyCode.from_char("d")
        k_key = keyboard.KeyCode.from_char("k")
        c_key = keyboard.KeyCode.from_char("c")
        b_key = keyboard.KeyCode.from_char("b")

        self.yaw += (keyboard_input[q_key] - keyboard_input[e_key]) * ROTATION_SPEED / BASESTATION_FREQUENCY
        if keyboard_input[w_key] or keyboard_input[s_key] or keyboard_input[a_key] or keyboard_input[d_key]:
            velocity_x = (keyboard_input[w_key] - keyboard_input[s_key]) * MAX_SPEED
            velocity_y = (keyboard_input[a_key] - keyboard_input[d_key]) * MAX_SPEED
            rho = math.sqrt(velocity_x ** 2 + velocity_y ** 2)
            theta = math.atan2(velocity_y, velocity_x)
            self.command.rho = rho
            self.command.theta = theta + self.yaw
        else:
            self.command.rho = 0
            self.command.theta = 0
        self.command.doKick = keyboard_input[k_key]
        self.command.doForce = keyboard_input[k_key] or keyboard_input[c_key]
        self.command.kickChipPower = KICK_SPEED
        self.command.doChip = keyboard_input[c_key]
        if (keyboard_input[b_key] and not self.dribblerPressed):
            self.command.dribblerOn = not self.command.dribblerOn
            self.dribblerPressed = True
        elif not keyboard_input[b_key]:
            self.dribblerPressed = False
        self.command.yaw = self.yaw

        # Return the command payload
        return self.command

def shutdown() -> None:
    """Shuts down the system."""
    print("Exiting")
    event_handler.running = False
    basestation_handler.running = False


def thread_exception_handler(args: threading.ExceptHookArgs) -> None:
    """Handles exceptions raised in threads."""
    event_handler.record_event(-1, f"Caught: {args}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Keyboard to robot controller')
    parser.add_argument("robot_ids", type=int, nargs='+', help="An array of integers for the robot ids")
    parser.add_argument("--simulate", action="store_true", help="Simulate the basestation")
    args = parser.parse_args()

    event_handler = EventHandler(shutdown)
    threading.excepthook = thread_exception_handler

    keyboard_handler = KeyboardHandler()
    basestation_handler = BasestationHandler(event_handler, shutdown, keyboard_handler)
    event_handler.start()

    try:
        event_handler.thread.join()
    except:
        shutdown()
