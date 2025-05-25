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
        turning_right = False
        waiting_for_right_wall = False
        waiting_for_left_wall = False
        use_left_wall = False  
        
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.01)
            # Write your solution here for challenge level 3 (or 3.5)
            print("start detections")
            scan = lidar.checkScan()
            print("0")
            front_dist, _ = lidar.detect_obstacle_in_cone(scan, 0.25, 0, 5)
            front_wall = front_dist != -1

            front_far_dist, _ = lidar.detect_obstacle_in_cone(scan, 1.0, 0, 15)
            front_clear_far = front_far_dist == -1

            front_wide_dist, _ = lidar.detect_obstacle_in_cone(scan, 0.4, 0, 34)
            front_clear_wide = front_wide_dist == -1

            right_dist, _ = lidar.detect_obstacle_in_cone(scan, 0.6, 270, 5) 
            right_clear = right_dist == -1
            right_wall_found = not right_clear
            print("start 1")
            left_dist, _ = lidar.detect_obstacle_in_cone(scan, 0.6, 90, 5)
            left_clear = left_dist == -1
            left_wall_not_found = left_clear

            print("2")
            if(left_wall_not_found):
                use_left_wall = True  

            print("3")
            if not left_clear and use_left_wall:
                print("[✅] 左墙恢复，退出左墙转向模式")
                use_left_wall = False
            
            print("start moving")
            if turning_right:
                if front_clear_far and front_clear_wide and not use_left_wall:
                    print("[✅] 前方 1m + 宽区域都清空，停止右转，恢复前进")
                    turning_right = False
                    waiting_for_right_wall = True
                    control.set_cmd_vel(0.0, 0.0, 0.2)
                elif not left_clear:
                    print("[✅] stop turning, start to move forward")
                    turning_right= False
                    control.set_cmd_vel(0.0, 0.0, 0.2)
                else:
                    print("[↪️] 正在右转...（前方仍有东西）")
                    control.set_cmd_vel(0.0, -0.5, 0.2)
            
            elif waiting_for_right_wall and not use_left_wall:
                if right_wall_found:
                    print("[🧱] 右墙已恢复，重新进入贴墙模式")
                    waiting_for_right_wall = False
                    control.set_cmd_vel(0.2, 0.0, 0.2)
                else:
                    print("[🔍] 等待右墙出现中...")
                    control.set_cmd_vel(0.2, 0.0, 0.2)  # 先直走寻找右墙
            else:
                if front_wall:
                    print("[⛔] 前方有墙，开始右转")
                    turning_right = True
                elif right_clear:
                    print("[🚪] 右边无墙，开始右转")
                    turning_right = True
                else:
                    print("[➡️] 正常贴墙前进")
                    control.set_cmd_vel(0.2, 0.0, 0.2)

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
