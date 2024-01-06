#!/usr/bin/env python3
#coding=utf-8

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
# from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

rospy.init_node("Disparity")
speed_param = rospy.get_param('~speed_param', 4) # Car driving speed
P_param = rospy.get_param('~P_param', 0.1) # According to the proportion of the corner car turning and decelerating

FILTER_VALUE = 10.0
# Get the distance measured by lidar
def get_range(data, angle, deg=True):
    if deg:
        angle = np.deg2rad(angle)
    dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
    print(int((angle - data.angle_min) / data.angle_increment))
    if dis < data.range_min or dis > data.range_max:
        dis = FILTER_VALUE
    return dis

DISPARITY_DIF = 1.0 # The threshold to determine whether to touch an obstacle
CAR_WIDTH = 0.2 # Trolley width, from simulator of params.yaml get

# Callback function for laser data/scan
def disparity_extender_callback(data):
    
    dis = []
    # Get the measured distance of 180° forward
    # for angle in range(-90, 91):
    #     dis.append(get_range(data, angle))
    
    dis = list(data.ranges[270:360] + data.ranges[0:90])

    # print(len(dis))

    disparities = []
    for i in range(len(dis)):
        if i == len(dis) - 1:
            continue
        # If the threshold value is exceeded, it is considered that a collision may occur
        if abs(dis[i] - dis[i + 1]) > DISPARITY_DIF:
            min_dis = min(dis[i], dis[i + 1]) # Calculate the minimum distance that may collide
            angle_range = math.ceil(
                math.degrees(math.atan(CAR_WIDTH / 2 / min_dis))) # 根据小车宽度计算不会碰撞的最小角度
            angle_range += 15 # 添加容差
            # 需要裁剪的激光数据范围
            side_range = range(int(i - angle_range + 1), i + 1) if dis[i + 1] == min_dis else range(i + 1, int(i + 1 + angle_range))
            disparities.append((min_dis, side_range))

    # 裁剪激光数据
    for min_dis, side_range in disparities:
        for i in side_range:
            if i >= 0 and i < len(dis):
                dis[i] = min(dis[i], min_dis)

    max_index = np.argmax(dis) # 得到最可行距离的角度
    max_dis = dis[max_index]

    # 对角度进行限制，主要是避免激光数据抖动产生的影响
    angle = max_index - 90 if abs(max_index - 90) > 15 else 0
    # angle = angle * np.pi / 180

    Accel = (1-abs(angle/90))**3
    # print(angle, Accel)

    offset = 19.4799995422
    angle = (-angle+offset)/(2*offset)
    if angle>1.0:
        angle=1.0
    elif angle<0:
        angle=0
    

    ang = Float64()
    ang.data = angle
    ang_pub.publish(ang)

    # speed = speed_param - P_param * abs(angle)
    speed = 1000 + 1000*Accel
    print(speed)
    spd = Float64()
    spd.data = speed
    spd_pub.publish(spd)

    # drive_msg = AckermannDriveStamped()
    # drive_msg.drive.steering_angle=angle
    # drive_msg.drive.speed=speed
    # drive_pub.publish(drive_msg)
    # print(drive_msg.drive.steering_angle)


if __name__ == '__main__': 
  try:
    scan_sub = rospy.Subscriber('/lidar2D_RB', LaserScan, disparity_extender_callback)
    # drive_pub = rospy.Publisher('/Disparity', AckermannDriveStamped, queue_size=1)
    
    ang_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
    spd_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)

    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
