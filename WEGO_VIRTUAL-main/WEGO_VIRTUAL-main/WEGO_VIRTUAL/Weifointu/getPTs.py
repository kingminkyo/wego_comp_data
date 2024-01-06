#!/usr/bin/env python3
import rospy
import time
import numpy as np
from morai_msgs.msg import ObjectStatusList

# 웨이포인트

class getPTs():
    def __init__(self):
        self.beforeXY = [0,0]
        self.start_time = time.time()
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.Ego_topic_callback)

    def Ego_topic_callback(self, data):
        self.currentXY = [data.npc_list[0].position.x, data.npc_list[0].position.y]
        
        # 1초마다 좌표값 출력
        # if (time.time() - self.start_time) > 1 :
        #     self.start_time = time.time()
        #     print(f'{self.nowXY},')


        # 일정 거리마다 좌표값 출력 (0.2 마다)
        if self.distance(self.beforeXY, self.currentXY)>0.2:
            self.beforeXY = [data.npc_list[0].position.x, data.npc_list[0].position.y]
            print(f'{self.beforeXY},')
            
    def distance(self, myXY, wayXY):
        return ((myXY[0]-wayXY[0])**2+(myXY[1]-wayXY[1])**2)**0.5
    
def run():
    rospy.init_node("getPTs")
    getPTs()
    rospy.spin()


if __name__ == '__main__':
    run()
