from TMMC_Wrapper import *
import rclpy
import numpy as np
import math
import time
from ultralytics import YOLO

# Variable for controlling which level of the challenge to test -- set to 0 for pure keyboard control
challengeLevel = 2

# Set to True if you want to run the simulation, False if you want to run on the real robot
is_SIM = False

# Set to True if you want to run in debug mode with extra print statements, False otherwise
Debug = False

# Initialization    
if not "robot" in globals():
    robot = Robot(IS_SIM=is_SIM, DEBUG=Debug)
    
control = Control(robot)
camera = Camera(robot)
imu = IMU(robot)
logging = Logging(robot)
lidar = Lidar(robot)

def auto_avoid_obstacle(lidar, 
                        control,
                        robot, 
                        trigger_distance=0.4, 
                        safe_back_speed=0.1, back_duration=2.0, cone_angle=20):
    rclpy.spin_once(robot, timeout_sec=0.01)
    laserMsg = lidar.checkScan()
    if laserMsg is None:
        return False

    dis, angle = lidar.detect_obstacle_in_cone(laserMsg, trigger_distance, center=0, offset_angle=cone_angle)
    
    if dis != -1:
        print(f"dis = {dis:.2f} m, angle = {angle:.2f}°")

        control.stop_keyboard_control()
        control.set_cmd_vel(0.0, 0.0, 0)
        time.sleep(1)

        control.set_cmd_vel(-safe_back_speed, 0.0, back_duration)
        control.start_keyboard_control()
        time.sleep(0.1)
        return True

    return False

def handle_stop_sign_and_apriltag(camera, control, robot,
                                   stop_duration=2, cooldown_after_stop=30,
                                   check_interval=5):
    stop_detect = False
    stop_cooldown = 0
    frame_count = 0

    print("stop sign + apriltag handler activated")

    while rclpy.ok():
        rclpy.spin_once(robot, timeout_sec=0.1)
        time.sleep(0.1)
        frame_count += 1

        img = camera.rosImg_to_cv2()

        if frame_count % check_interval == 0 and not stop_detect:
            detected_stop, x1, y1, x2, y2 = camera.ML_predict_stop_sign(img)
            apriltags = camera.estimate_apriltag_pose(img)

            tag2_detected = any(tag[0] == 2 for tag in apriltags)
            tag3_detected = any(tag[0] == 3 for tag in apriltags)

            if detected_stop and (tag2_detected or tag3_detected):
                print("stop sign detected")
                for tag in apriltags:
                    print("Detected tagID:", tag[0])
                    break

                control.set_cmd_vel(0.0, 0.0, stop_duration)
                time.sleep(stop_duration)
                stop_detect = True
                stop_cooldown = cooldown_after_stop
                print("please continue")

        if stop_detect:
            stop_cooldown -= 1
            if stop_cooldown <= 0:
                stop_detect = False


if challengeLevel <= 2:
    control.start_keyboard_control()
    rclpy.spin_once(robot, timeout_sec=0.1)


try:
    if challengeLevel == 0:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Challenge 0 is pure keyboard control, you do not need to change this it is just for your own testing

    if challengeLevel == 1:
        while rclpy.ok():
            print("level 1 start!")
            auto_avoid_obstacle(lidar, control, robot)
            
    if challengeLevel == 2:
        print("level 2 start!") 
        handle_stop_sign_and_apriltag(camera, control, robot)

    if challengeLevel == 3:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 3 (or 3.5)

    if challengeLevel == 4:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 4

    if challengeLevel == 5:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 5
            

except KeyboardInterrupt:
    print("Keyboard interrupt received. Stopping...")

finally:
    control.stop_keyboard_control()
    robot.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
