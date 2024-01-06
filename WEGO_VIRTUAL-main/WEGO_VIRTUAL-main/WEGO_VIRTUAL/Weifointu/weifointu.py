#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64
from morai_msgs.msg import EgoVehicleStatus

from weifointu_container import *
wayPT = wayPTcontainer()

class weifointu():
    def __init__(self):
        self.nowPT = -1

        self.pub_angle = rospy.Publisher(
            '/control/angle/PP', Float64, queue_size=1)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.Ego_topic_callback)
        self.pub_finish = rospy.Publisher("/final_index_cnt", Float64, queue_size=1)
        
    def Ego_topic_callback(self, data):
        myXY = [data.position.x, data.position.y]
        now_hd = data.heading
        
        # 초기화 시, 가장 가까운 웨이포인트를 찾아감.
        # 마지막 인덱스 도달하면 다시 루프 돌림.
        if self.nowPT == -1 or self.nowPT==len(wayPT):
            closest_distance = 10000
            for i in range(len(wayPT)):
                if closest_distance > self.distance(myXY, wayPT[i]):
                    closest_distance = self.distance(myXY, wayPT[i])
                    self.nowPT = i
            print(f'첫 번째 추종 위치  = {self.nowPT}, {wayPT[self.nowPT]}')
        
        nextXY = wayPT[self.nowPT]

        # 차 길이 = 0.3
        if self.distance(myXY, nextXY) < 0.8:
            print(self.nowPT)
            self.nowPT += 1
            k= self.nowPT
            self.pub_finish.publish(Float64(k))
        steer = Float64()
        angle_diff = self.angle_cal(myXY, nextXY, now_hd)
        # print(angle_diff, self.nowPT)
        offset = 19.4799995422
        steer.data = (-angle_diff+offset)/(2*offset)
        # steer.data = -angle_diff/19.4799995422 + 0.50000
        if steer.data>1.0:
            steer.data=1.0
        elif steer.data<0:
            steer.data=0
        
        self.pub_angle.publish(steer)
        # print(angle_diff, steer,self.nowPT)
        y = myXY[1] - nextXY[1]
        x = myXY[0] - nextXY[0]
        # print(np.arctan2(y, x)*(180/np.pi), self.nowPT)
        speed = Float64()
        speed.data = 20000
        self.pub_speed.publish(speed)

    def distance(self, myXY, wayXY):
        return ((myXY[0]-wayXY[0])**2+(myXY[1]-wayXY[1])**2)**0.5

    def angle_cal(self, myXY, nextXY, now_hd):
        y = myXY[1] - nextXY[1]
        x = myXY[0] - nextXY[0]

        angle =  (np.arctan2(y, x)*(180/np.pi)) - now_hd
 
        if angle>90:
            angle-=180
        elif angle<-90:
            angle+=180

        return angle


def run():
    rospy.init_node("weifointu")
    weifointu()
    rospy.spin()


if __name__ == '__main__':
    run()


