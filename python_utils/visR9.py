# This file contains everything needed to print a single visualization window of a robot
import numpy as np
import math
from typing import Tuple
try:
    import cv2
    cv2_available = True
except ImportError:
	print("Warning! Could not import cv2. Can't visualize.")
	cv2_available = False

rate_of_turn_avg = 0
wheel_speeds_avg = np.zeros(4)

def rotate(origin: Tuple[float, float], point: Tuple[float, float], angle: float) -> Tuple[float, float]:
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.

    Args:
        origin (Tuple[float, float]): The coordinates of the origin point (ox, oy)
        point (Tuple[float, float]): The coordinates of the point to rotate (px, py)
        angle (float): The angle of rotation in radians

    Returns:
        Tuple[float, float]: The coordinates of the rotated point (qx, qy)
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy

def visualize(args, image_vis, last_packet_feedback, last_packet_state_info, cmd):
    # Return if cv2 is not imported or if there are multiple robots
    if not cv2_available or len(args.robot_ids) > 1:
        return image_vis
    
    global rate_of_turn_avg
    global wheel_speeds_avg

    # Draw robot on the image
    s = 101.2
    cv2.line(image_vis, (int(250 - s / 2), 250 - 73), (int(250 + s / 2), 250 - 73), (255, 255, 255), 2)
    cv2.ellipse(image_vis, (250, 250), (90, 90), -90, 35, 325, (255, 255, 255), 2)
    if last_packet_feedback:
        # Ball sensor
        if last_packet_feedback.ballSensorWorking:
            cv2.line(image_vis, (int(250 - s / 2), 250 - 73 - 5), (int(250 + s / 2), 250 - 73 - 5), (0, 1, 0), 2)
            if last_packet_feedback.ballSensorSeesBall:
                # Ball is centered in the drawing, this might not reflect the real world
                cv2.circle(image_vis, (250, 250 - 90), 10, (0, 0.4, 1), -1)
        else:
            cv2.line(image_vis, (int(250 - s / 2), 250 - 73 - 5), (int(250 + s / 2), 250 - 73 - 5), (0, 0, 1), 2)
        # Velocity estimate
        length = int(last_packet_feedback.rho * 100)
        px, py = rotate((250, 250), (250, 250 - length), -last_packet_feedback.theta)
        cv2.line(image_vis, (250, 250), (int(px), int(py)), (1, 0, 0), 8)

        # Battery
        cv2.rectangle(image_vis, (10, 10), (50, 30), color=(1, 1, 1))
        cv2.rectangle(image_vis, (51, 15), (55, 25), color=(1, 1, 1), thickness=-1)
        # COLOR IS IN (B,G,R)
        if last_packet_feedback.batteryLevel >= 18.0:
            v = str(round(last_packet_feedback.batteryLevel, 2)) + 'V'
            if last_packet_feedback.batteryLevel < 20.0:
                background_color = (0, 0, 255)  # red
            elif last_packet_feedback.batteryLevel < 22.0:
                background_color = (0, 140, 255)  # orange
            elif last_packet_feedback.batteryLevel < 24.0:
                background_color = (0, 255, 255)  # yellow
            else:
                background_color = (0, 255, 0)  # green
            length_rectangle = int(5.27777777 * last_packet_feedback.batteryLevel - 84)
            cv2.rectangle(image_vis, (11, 11), (length_rectangle, 29), color=background_color, thickness=-1)
            cv2.putText(image_vis, v, org=(60, 26), color=(255, 255, 255), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1,
                        thickness=1, lineType=cv2.LINE_AA)
        else:
            cv2.putText(image_vis, '?', org=(25, 26), color=(255, 255, 255), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1)
    
    if last_packet_state_info and last_packet_feedback:
        # Velocity reference
        length = int(cmd.rho * 100)
        px, py = rotate((250, 250), (250, 250 - length), -cmd.theta+last_packet_state_info.xsensYaw)
        cv2.line(image_vis, (250, 250), (int(px), int(py)), (0, 1, 0), 4)
    
    if last_packet_state_info:
        # XSens yaw
        px, py = rotate((250, 250), (250, 150), -last_packet_state_info.xsensYaw)
        cv2.line(image_vis, (250, 250), (int(px), int(py)), (1, 1, 1), 1)
        cv2.circle(image_vis, (int(px), int(py)), 5, (1, 1, 1), -1)
        # Commanded yaw
        px, py = rotate((250, 250), (250, 150), -cmd.yaw)
        cv2.line(image_vis, (250, 250), (int(px), int(py)), (0, 1, 0), 1)
        cv2.circle(image_vis, (int(px), int(py)), 5, (0, 1, 0), -1)
        # XSens rate of turn
        rate_of_turn_avg = rate_of_turn_avg * 0.99 + last_packet_state_info.rateOfTurn * 0.01
        cv2.ellipse(image_vis, (250, 250), (40, 40), -90, 0, 0.5 * -rate_of_turn_avg * 180 / math.pi, (1, .45, .5), 12)
        cv2.ellipse(image_vis, (250, 250), (40, 40), -90, 0, 0.5 * -last_packet_state_info.rateOfTurn * 180 / math.pi,
                    (1, 1, 1), 4)
        # Wheel speeds
        wheel_speeds = np.array(
            [last_packet_state_info.wheelSpeed1, last_packet_state_info.wheelSpeed2, last_packet_state_info.wheelSpeed3,
             last_packet_state_info.wheelSpeed4])
        wheel_speeds_exp = np.log(np.abs(wheel_speeds))
        wheel_speeds_exp = np.clip(wheel_speeds_exp, 0, None)
        wheel_speeds_exp = wheel_speeds_exp * .25 * np.sign(wheel_speeds)
        wheel_speeds_avg = wheel_speeds_avg * 0.99 + wheel_speeds_exp * 0.01
        # XSens wheel speed 1
        rx, ry = rotate((330, 170), (330, 170 - wheel_speeds_avg[0] * 80), -30 * np.pi / 180.)
        cv2.line(image_vis, (330, 170), (int(rx), int(ry)), (.15, .15, 1), 10)
        rx, ry = rotate((330, 170), (330, 170 - wheel_speeds_exp[0] * 80), -30 * np.pi / 180.)
        cv2.line(image_vis, (330, 170), (int(rx), int(ry)), (1, 1, 1), 4)
        # XSens wheel speed 2
        rx, ry = rotate((170, 170), (170, 170 + wheel_speeds_avg[1] * 80), 30 * np.pi / 180.)
        cv2.line(image_vis, (170, 170), (int(rx), int(ry)), (.15, .15, 1), 10)
        rx, ry = rotate((170, 170), (170, 170 + wheel_speeds_exp[1] * 80), 30 * np.pi / 180.)
        cv2.line(image_vis, (170, 170), (int(rx), int(ry)), (1, 1, 1), 4)
        # XSens wheel speed 3
        rx, ry = rotate((170, 330), (170, 330 + wheel_speeds_avg[2] * 80), -60 * np.pi / 180.)
        cv2.line(image_vis, (170, 330), (int(rx), int(ry)), (.15, .15, 1), 10)
        rx, ry = rotate((170, 330), (170, 330 + wheel_speeds_exp[2] * 80), -60 * np.pi / 180.)
        cv2.line(image_vis, (170, 330), (int(rx), int(ry)), (1, 1, 1), 4)
        # XSens wheel speed 4
        rx, ry = rotate((330, 330), (330, 330 - wheel_speeds_avg[3] * 80), 60 * np.pi / 180.)
        cv2.line(image_vis, (330, 330), (int(rx), int(ry)), (.15, .15, 1), 10)
        rx, ry = rotate((330, 330), (330, 330 - wheel_speeds_exp[3] * 80), 60 * np.pi / 180.)
        cv2.line(image_vis, (330, 330), (int(rx), int(ry)), (1, 1, 1), 4)
        # Debug ports
        for i in range(32):
            eval("cv2.putText(image_vis, \"Debug " + str(i) + ": \"+str(round(last_packet_state_info.Debug" + str(i) + ", 2)), org=(10, " + str(46+i*20) + "), color=(255, 255, 255), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1,thickness=1, lineType=cv2.LINE_AA)")

    cv2.imshow("Press esc to quit", image_vis)
    if cv2.waitKey(1) == 27: 
        exit()
    if cv2.getWindowProperty("Press esc to quit", cv2.WND_PROP_VISIBLE) < 1:
        exit()
    image_vis *= 0.7

    return image_vis