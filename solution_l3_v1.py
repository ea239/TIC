from TMMC_Wrapper import *
import rclpy
import numpy as np
import math
import time
from ultralytics import YOLO

# Variable for controlling which level of the challenge to test -- set to 0 for pure keyboard control
challengeLevel = 3

# Set to True if you want to run the simulation, False if you want to run on the real robot
is_SIM = False

# Set to True if you want to run in debug mode with extra print statements, False otherwise
Debug = False

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

# Initialization    
if not "robot" in globals():
    robot = Robot(IS_SIM=is_SIM, DEBUG=Debug)
    
control = Control(robot)
camera = Camera(robot)
imu = IMU(robot)
logging = Logging(robot)
lidar = Lidar(robot)

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
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.01)
            control.turn_right()
            # Write your solution here for challenge level 1
            # It is recommended you use functions for aspects of the challenge that will be resused in later challenges
            # For example, create a function that will detect if the robot is too close to a wall

    if challengeLevel == 2:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 2
            
    if challengeLevel == 3:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.01)
            # Write your solution here for challenge level 3 (or 3.5)
            stats = camera.estimate_apriltag_pose(camera.rosImg_to_cv2())
            if len(stats) >= 1:
                index = min(range(len(stats)), key=lambda i: stats[i][2])
                bearing = stats[index][1]
                print(1)
                if abs(bearing) > 2:
                    control.rotate(abs(bearing), 1 if bearing > 0 else -1)
                else:
                    control.move_forward()
            elif len(stats) == 0:
                print(0)
                dist, _ = lidar.detect_obstacle_in_cone(lidar.checkScan(), 0.5, 0, 20)
                if dist != -1:
                    control.set_cmd_vel(0.0, -0.5, 1.3)
                else:
                    control.set_cmd_vel(1, 0, 1)

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
