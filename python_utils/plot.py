import os
import sys
import argparse
import matplotlib.pyplot as plt
from typing import List, Tuple, Any
import numpy as np
import pandas as pd

# Append the necessary directory to sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Core.Inc.roboteam_embedded_messages.python import REM_BaseTypes as BaseTypes
from Core.Inc.roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand
from Core.Inc.roboteam_embedded_messages.python.REM_RobotFeedback import REM_RobotFeedback
from Core.Inc.roboteam_embedded_messages.python.REM_ControlDebug import REM_ControlDebug
from REMParser import REMParser

def parse_arguments() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--input_file', default='latest.rembin', help='Input file to parse')
    return parser.parse_args()

def parse_packets(parser: REMParser) -> Tuple[List[REM_RobotCommand], List[REM_RobotFeedback], List[REM_ControlDebug]]:
    """Parse the packets from the input file."""
    robot_commands = [packet for packet in parser.packet_buffer if isinstance(packet, REM_RobotCommand)]
    robot_feedback = [packet for packet in parser.packet_buffer if isinstance(packet, REM_RobotFeedback)]
    robot_state_info = [packet for packet in parser.packet_buffer if isinstance(packet, REM_ControlDebug)]
    return robot_commands, robot_feedback, robot_state_info

def extract_values(packets: List[Any], attribute: str, first_timestamp: int) -> Tuple[List[float], List[float]]:
    """Extract specific attribute values and their timestamps from packets."""
    data = [(getattr(packet, attribute), (packet.timestamp - first_timestamp) / 1000) 
        for packet in packets if (packet.timestamp - first_timestamp) / 1000 >= 0]
    values, timestamps = zip(*data) if data else ([], [])
    return list(timestamps), list(values)

def plot_values(t_rc: List[float], values_rc: List[float], t_rf: List[float], values_rf: List[float], attribute: str, observer_data: pd.DataFrame = None) -> None:
    """Plot values for a specific attribute."""
    figure = plt.figure(figsize=(10, 6))
    figure.canvas.toolbar.zoom()
    plt.plot(t_rc, values_rc, label="Reference", linewidth=2, linestyle='--')
    plt.plot(t_rf, values_rf, label="Achieved", linewidth=2, linestyle='-')
    if observer_data is not None and attribute in observer_data.columns:
        plt.plot(observer_data['timestamp'], observer_data[attribute], label="Observer", linewidth=2, linestyle=':')
    
    plt.xlabel("Timestamp (s)", fontsize=14)
    plt.ylabel(attribute, fontsize=14)
    plt.title(attribute, fontsize=16)
    plt.legend(fontsize=12)
    plt.grid(True)
    plt.tight_layout()

def main() -> None:
    """Main function to parse and plot data."""
    args = parse_arguments()
    
    # Parse the input file
    parser = REMParser(device=None)
    parser.parse_file(args.input_file)
    robot_commands, robot_feedback, robot_state_info = parse_packets(parser)
    
    # Ensure the first timestamp is consistent across all plots
    first_timestamp = robot_commands[0].timestamp
    # try:
    if args.input_file != 'latest.rembin':
        observer_data = pd.read_csv(args.input_file.replace('.bin', '.csv').replace('.log', '.observer'))
    else:
        observer_data = pd.read_csv('latest_observer.csv')
    observer_data = observer_data[observer_data['timestamp'] >= first_timestamp]
    observer_data['timestamp'] = (observer_data['timestamp'] - first_timestamp) / 1000
    observer_data['rho'] = (observer_data['velocity_x']**2 + observer_data['velocity_y']**2)**0.5
    observer_data['theta'] = np.arctan2(observer_data['velocity_y'], observer_data['velocity_x'])

    # Extract and plot 'rho' values
    t_rc, rho_rc = extract_values(robot_commands, 'rho', first_timestamp)
    t_rf, rho_rf = extract_values(robot_feedback, 'rho', first_timestamp)
    plot_values(t_rc, rho_rc, t_rf, rho_rf, 'rho', observer_data)

    # t_rc, rho_rc = extract_values(robot_commands, 'acceleration_angle', first_timestamp)
    # t_rf, rho_rf = extract_values(robot_feedback, 'rho', first_timestamp)
    # plot_values(t_rc, rho_rc, t_rf, rho_rf, 'rho', observer_data)

    figure = plt.figure(figsize=(10, 6))
    figure.canvas.toolbar.zoom()
    plt.plot(t_rc, rho_rc, label="Reference rho", linewidth=2, linestyle='--')
    plt.plot(t_rf, rho_rf, label="Achieved rho", linewidth=2, linestyle='-')
    if observer_data is not None and 'rho' in observer_data.columns:
        plt.plot(observer_data['timestamp'], observer_data['rho'], label="Observer rho", linewidth=2, linestyle=':')

    t_rc, acc_ang_rc = extract_values(robot_commands, 'acceleration_angle', first_timestamp)
    # plt.plot(t_rc, acc_ang_rc, label="Acc ang", linewidth=2, linestyle='--')
    t_rc, acc_mag_rc = extract_values(robot_commands, 'acceleration_magnitude', first_timestamp)

    acc_x = []
    acc_y = []
    for i in range(len(acc_ang_rc)):
        acc_x.append(np.cos(acc_ang_rc[i]) * acc_mag_rc[i])
        acc_y.append(np.sin(acc_ang_rc[i]) * acc_mag_rc[i])

    # print(acc_y)
    # print(acc_mag_rc)
    plt.plot(t_rc, acc_x, label="Acc x", linewidth=2, linestyle='--')
    plt.plot(t_rc, acc_y, label="Acc y", linewidth=2, linestyle='--')

    
    plt.xlabel("Timestamp (s)", fontsize=14)
    plt.ylabel('haha yes', fontsize=14)
    plt.legend(fontsize=12)
    plt.grid(True)
    plt.tight_layout()
    
    # Extract and plot 'theta' values
    t_rc, theta_rc = extract_values(robot_commands, 'theta', first_timestamp)
    t_rf, theta_rf = extract_values(robot_feedback, 'theta', first_timestamp)
    plot_values(t_rc, theta_rc, t_rf, theta_rf, 'theta', observer_data)
    
    # Extract and plot 'wheelSpeed' values from state info
    tRef_si, wheel_speed_ref_1_si = extract_values(robot_state_info, 'wheelSpeedRef1', first_timestamp)
    t_si, wheel_speed_1_si = extract_values(robot_state_info, 'wheelSpeed1', first_timestamp)
    plot_values(tRef_si, wheel_speed_ref_1_si, t_si, wheel_speed_1_si, 'wheelSpeed1')

    tRef_si, wheel_speed_ref_1_si = extract_values(robot_state_info, 'wheelSpeedRef2', first_timestamp)
    t_si, wheel_speed_1_si = extract_values(robot_state_info, 'wheelSpeed2', first_timestamp)
    plot_values(tRef_si, wheel_speed_ref_1_si, t_si, wheel_speed_1_si, 'wheelSpeed2')

    tRef_si, wheel_speed_ref_1_si = extract_values(robot_state_info, 'wheelSpeedRef3', first_timestamp)
    t_si, wheel_speed_1_si = extract_values(robot_state_info, 'wheelSpeed3', first_timestamp)
    plot_values(tRef_si, wheel_speed_ref_1_si, t_si, wheel_speed_1_si, 'wheelSpeed3')

    tRef_si, wheel_speed_ref_1_si = extract_values(robot_state_info, 'wheelSpeedRef4', first_timestamp)
    t_si, wheel_speed_1_si = extract_values(robot_state_info, 'wheelSpeed4', first_timestamp)
    plot_values(tRef_si, wheel_speed_ref_1_si, t_si, wheel_speed_1_si, 'wheelSpeed4')
    plt.show()

if __name__ == "__main__":
    main()