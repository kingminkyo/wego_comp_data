#!/usr/bin/env python3
import rospy
import numpy as np
import math
from std_msgs.msg import Float64
from morai_msgs.msg import EgoVehicleStatus
from sensor_msgs.msg import LaserScan


class lidar():
    def __init__(self):
        self.nowPT = 0
        self.pub_speed = rospy.Publisher(
            '/commands/motor/speed', Float64, queue_size=1)
        self.pub_angle = rospy.Publisher(
            '/commands/servo/position', Float64, queue_size=1)

        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.Ego_topic_callback)
        rospy.Subscriber("/lidar2D", LaserScan, self.lidar_callback)

    # 50 hz
    def Ego_topic_callback(self, data):
        myXY = [data.position.x, data.position.y]
        now_hd = data.heading

        degree = 0

        print(myXY, "car point")
        
        try:
            for i in range(len(self.obstacle)):
                degree = now_hd + self.obstacle[i]
                obs_X = self.obstacle_length[i]*math.cos(degree*math.pi/180) + myXY[0] + 0.1 * math.cos(now_hd*math.pi/180)
                obs_Y = self.obstacle_length[i]*math.sin(degree*math.pi/180) + myXY[1] + 0.1 * math.sin(now_hd*math.pi/180)
                print(len(self.obstacle), obs_X, obs_Y)

        except:
            pass

    # 9 hz
    def lidar_callback(self, data):
        stack = 0
        cont = 0

        # initializing obstacle status
        self.obstacle = []
        self.obstacle_length = []

        # lidar clustering
        for i in range(270,360):
            if cont == 0:
                if data.ranges[i] < 2 and abs(data.ranges[i]- data.ranges[i-1]) < 0.05:
                    stack += 1

                else:
                    stack = 0
                    cont = 0

                if stack >= 3:
                    self.obstacle.append(i)
                    self.obstacle_length.append(data.ranges[i])
                    stack = 0
                    cont = 1

            else:
                if abs(self.obstacle_length[-1] - data.ranges[i]) < 0.05:
                    pass

                else:
                    cont = 0

        for i in range(0,90):
            if cont == 0:
                if data.ranges[i] < 2 and abs(data.ranges[i]- data.ranges[i-1]) < 0.05:
                    stack += 1

                else:
                    stack = 0
                    cont = 0

                if stack >= 3:
                    self.obstacle.append(i)
                    self.obstacle_length.append(data.ranges[i])
                    stack = 0
                    cont = 1
                    
            else:
                if abs(self.obstacle_length[-1] - data.ranges[i]) < 0.05:
                    pass

                else:
                    cont = 0

def run():
    rospy.init_node("lidar")
    lidar()
    rospy.spin()

if __name__ == '__main__':
    run()
