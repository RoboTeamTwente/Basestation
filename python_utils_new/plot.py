import os
import sys
import argparse
import matplotlib.pyplot as plt

# Append the necessary directory to sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Core.Inc.roboteam_embedded_messages.python import REM_BaseTypes as BaseTypes
from Core.Inc.roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand
from Core.Inc.roboteam_embedded_messages.python.REM_RobotFeedback import REM_RobotFeedback
from Core.Inc.roboteam_embedded_messages.python.REM_RobotStateInfo import REM_RobotStateInfo
from REMParser import REMParser

def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--input_file', default='latest.rembin', help='Input file to parse')
    return parser.parse_args()

def parse_packets(parser):
    """Parse the packets from the input file."""
    robot_commands = [packet for packet in parser.packet_buffer if isinstance(packet, REM_RobotCommand)]
    robot_feedback = [packet for packet in parser.packet_buffer if isinstance(packet, REM_RobotFeedback)]
    robot_state_info = [packet for packet in parser.packet_buffer if isinstance(packet, REM_RobotStateInfo)]
    return robot_commands, robot_feedback, robot_state_info

def extract_values(packets, attribute, first_timestamp):
    """Extract specific attribute values and their timestamps from packets."""
    values = [getattr(packet, attribute) for packet in packets]
    timestamps = [(packet.timestamp - first_timestamp) / 1000 for packet in packets]
    return timestamps, values

def plot_values(t_rc, values_rc, t_rf, values_rf, attribute):
    """Plot values for a specific attribute."""
    figure = plt.figure(figsize=(10, 6))
    figure.canvas.toolbar.zoom()
    plt.plot(t_rc, values_rc, label="Reference", linewidth=2, linestyle='--')
    plt.plot(t_rf, values_rf, label="Achieved", linewidth=2, linestyle='-')
    plt.xlabel("Timestamp (s)", fontsize=14)
    plt.ylabel(attribute, fontsize=14)
    plt.title(attribute, fontsize=16)
    plt.legend(fontsize=12)
    plt.grid(True)
    plt.tight_layout()

def main():
    args = parse_arguments()
    
    # Parse the input file
    parser = REMParser(device=None)
    parser.parse_file(args.input_file)
    
    robot_commands, robot_feedback, robot_state_info = parse_packets(parser)
    
    # Ensure the first timestamp is consistent across all plots
    first_timestamp = robot_commands[0].timestamp
    
    # Extract and plot 'rho' values
    t_rc, rho_rc = extract_values(robot_commands, 'rho', first_timestamp)
    t_rf, rho_rf = extract_values(robot_feedback, 'rho', first_timestamp)
    plot_values(t_rc, rho_rc, t_rf, rho_rf, 'rho')
    
    # Extract and plot 'theta' values
    t_rc, theta_rc = extract_values(robot_commands, 'theta', first_timestamp)
    t_rf, theta_rf = extract_values(robot_feedback, 'theta', first_timestamp)
    plot_values(t_rc, theta_rc, t_rf, theta_rf, 'theta')
    
    # Extract and plot 'wheelSpeed' values from state info
    tRef_si, wheel_speed_ref_1_si = extract_values(robot_state_info, 'wheelSpeedRef1', first_timestamp)
    t_si, wheel_speed_1_si = extract_values(robot_state_info, 'wheelSpeed1', first_timestamp)
    plot_values(tRef_si, wheel_speed_ref_1_si, t_si, wheel_speed_1_si, 'wheelSpeed1')
    plt.show()

if __name__ == "__main__":
    main()
