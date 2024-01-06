#!/usr/bin/env python3
import rospy
import time
import numpy as np
from morai_msgs.msg import EgoVehicleStatus
import math as m


class make_RB():
    def __init__(self):
        self.beforeXY = [0,0]
        self.start_time = time.time()
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.Ego_topic_callback)
        self.cnt = 0
    def Ego_topic_callback(self, data):
        self.currentXY = [data.position.x, data.position.y]
        self.heading = data.heading
        if data.heading <= 0:
            self.heading = data.heading +360
        k1 = self.heading -90
        k2 = self.heading +90
        de = m.pi/180

        # if data.heading >=0 and data.heading <= 90:
        self.left_RB=[data.position.x+0.6*m.cos(k2*de), data.position.y+0.6*m.sin(k2*de)]
        self.right_RB=[data.position.x+0.6*m.cos(k1*de), data.position.y+0.6*m.sin(k1*de)]
        # print(self.left_RB, self.right_RB)
        # 일정 거리마다 좌표값 출력 (0.2 마다)
        if self.distance(self.beforeXY, self.currentXY)>0.2:
            
            self.cnt+=1
            self.beforeXY = [data.position.x, data.position.y]

            right = {
            "DataID": 40200086,
            "UNIQUEID": 100+self.cnt,
            "m_eobstacleObjType": 4,
            "pos": {
                "x": self.right_RB[0],
                "y": self.right_RB[1],
                "z": 0.0,
                "_x": self.right_RB[0],
                "_y": self.right_RB[1],
                "_z": 0.000
            },
            "rot": {
                "roll": "0.000",
                "pitch": "0.000",
                "yaw": "-89.593"
            },
            "scale": {
                "x": 0.30000001192092896,
                "y": 0.30000001192092896,
                "z": 0.30000001192092896,
                "_x": "0.300",
                "_y": "0.300",
                "_z": "0.300"
            }
            }

            left = {
            "DataID": 40200086,
            "UNIQUEID": self.cnt,
            "m_eobstacleObjType": 4,
            "pos": {
                "x": self.left_RB[0],
                "y": self.left_RB[1],
                "z": 0.0,
                "_x": self.left_RB[0],
                "_y": self.left_RB[1],
                "_z": 0.000
            },
            "rot": {
                "roll": "0.000",
                "pitch": "0.000",
                "yaw": "-89.593"
            },
            "scale": {
                "x": 0.30000001192092896,
                "y": 0.30000001192092896,
                "z": 0.30000001192092896,
                "_x": "0.300",
                "_y": "0.300",
                "_z": "0.300"
            }

            }

            print("{")
            print("\"DataID\":", right["DataID"],",")
            print("\"UNIQUEID\":", right["UNIQUEID"],",")
            print("\"m_eobstacleObjType\":", right["m_eobstacleObjType"],",")
            print("\"pos\": {")
            print("\"x\":", right["pos"]["x"],",")
            print("\"y\":", right["pos"]["y"],",")
            print("\"z\":", right["pos"]["z"],",")
            print("\"_x\":", right["pos"]["_x"],",")
            print("\"_y\":", right["pos"]["_y"],",")
            print("\"_z\":", right["pos"]["_z"])
            print("}",",")
            print("\"rot\":","{")
            print("\"roll\":", right["rot"]["roll"],",")
            print("\"pitch\":", right["rot"]["pitch"],",")
            print("\"yaw\":", right["rot"]["yaw"])
            print("}",",")
            print("\"scale\":","{")
            print("\"x\":", right["scale"]["x"],",")
            print("\"y\":", right["scale"]["y"],",")
            print("\"z\":", right["scale"]["z"],",")
            print("\"_x\":", right["scale"]["_x"],",")
            print("\"_y\":", right["scale"]["_y"],",")
            print("\"_z\":", right["scale"]["_z"])
            print("}")
            print("}",",")
            
            print("{")
            print("\"DataID\":", left["DataID"],",")
            print("\"UNIQUEID\":", left["UNIQUEID"],",")
            print("\"m_eobstacleObjType\":", left["m_eobstacleObjType"],",")
            print("\"pos\": {")
            print("\"x\":", left["pos"]["x"],",")
            print("\"y\":", left["pos"]["y"],",")
            print("\"z\":", left["pos"]["z"],",")
            print("\"_x\":", left["pos"]["_x"],",")
            print("\"_y\":", left["pos"]["_y"],",")
            print("\"_z\":", left["pos"]["_z"])
            print("}",",")
            print("\"rot\":","{")
            print("\"roll\":", left["rot"]["roll"],",")
            print("\"pitch\":", left["rot"]["pitch"],",")
            print("\"yaw\":", left["rot"]["yaw"])
            print("}",",")
            print("\"scale\":","{")
            print("\"x\":", left["scale"]["x"],",")
            print("\"y\":", left["scale"]["y"],",")
            print("\"z\":", left["scale"]["z"],",")
            print("\"_x\":", left["scale"]["_x"],",")
            print("\"_y\":", left["scale"]["_y"],",")
            print("\"_z\":", left["scale"]["_z"])
            print("}")
            print("}",",")

    def distance(self, myXY, wayXY):
        return ((myXY[0]-wayXY[0])**2+(myXY[1]-wayXY[1])**2)**0.5
    
def run():
    rospy.init_node("MAKE_RB")
    make_RB()
    rospy.spin()


if __name__ == '__main__':
    run()
