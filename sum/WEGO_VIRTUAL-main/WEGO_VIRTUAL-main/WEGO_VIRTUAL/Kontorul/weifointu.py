#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64, Bool
from morai_msgs.msg import EgoVehicleStatus
from weifointu_container import *

wayPT2 = wayPT2container()
wayPT1 = wayPT1container()

class weifointu():
    def __init__(self):
        self.nowPT = -1
        self.lane_select = 2
        
        self.hana = 0

        self.one_true = 0
        self.false_cnt = 0

        self.pub_angle = rospy.Publisher(
            '/control/angle/PP', Float64, queue_size=1)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.Ego_topic_callback)
        rospy.Subscriber("/control/Lane_Change", Bool, self.Lane_Change_callback)
        rospy.Subscriber("/control/Lane_Return", Bool, self.Lane_Return_callback)
        self.pub_finish = rospy.Publisher("/final_index_cnt", Float64, queue_size=1)
        
    def Lane_Return_callback(self, data):
        '''when mission is finished
        '''
        lane_return = data.data
        if lane_return:
            print('차선재진입')
            self.nowPT = -1
            self.lane_select = 2
        
    def Lane_Change_callback(self, data):
        '''use when lane change situation
        '''
        lane_change = data.data
        if lane_change==True and self.one_true == 0:
            self.false_cnt = 0
            self.one_true = 1
            print('차선변경')
            if self.lane_select==2:
                self.lane_select = 1
                self.nowPT = -1
            else:
                self.lane_select = 2
                self.nowPT = -1

        elif lane_change==False:
            self.false_cnt += 1
            
        if self.one_true == 1 and self.false_cnt >= 20:
            self.one_true = 0

    
    def Ego_topic_callback(self, data):
        myXY = [data.position.x, data.position.y]
        now_hd = data.heading
        
        # Follow 2nd Lane
        if self.lane_select == 2:
            # when initializing, find the closest waypoint of 2nd Lane.
            if self.nowPT == -1:
                closest_distance = 10000
                for i in range(len(wayPT2)):
                    if closest_distance > self.distance(myXY, wayPT2[i]):
                        closest_distance = self.distance(myXY, wayPT2[i])
                        self.nowPT = i+3
            # 마지막 인덱스 도달하면 다시 루프 돌림.
            if self.nowPT==len(wayPT2):
                self.nowPT=0
            
            nextXY = wayPT2[self.nowPT]
        
        # Follow 1st Lane
        if self.lane_select == 1:
            # when initializing, find the closest waypoint of 1nd Lane.
            if self.nowPT == -1:
                closest_distance = 10000
                for i in range(len(wayPT1)):
                    if closest_distance > self.distance(myXY, wayPT1[i]):
                        closest_distance = self.distance(myXY, wayPT1[i])
                        # set lookahead point to prevent Divergence
                        self.nowPT = i+6
            # return to lane 2 when last index reached
            if self.nowPT==len(wayPT1):
                self.lane_select = 2
                self.nowPT = -1
                return
            nextXY = wayPT1[self.nowPT]
        
        # set lookahead distance 0.8m
        if self.distance(myXY, nextXY) < 0.8:
            self.nowPT += 1
            k= self.nowPT
            self.pub_finish.publish(Float64(k))
            
        steer = Float64()
        angle_diff = self.angle_cal(myXY, nextXY, now_hd)
        offset = 19.4799995422
        
        # set steer angle use error btw heading angle and waypoint angle
        steer.data = (-angle_diff+offset)/(2*offset)
        
        # publish angle btw (L) 0.0 ~ 1.0 (R)
        if steer.data>1.0:
            steer.data=1.0
        elif steer.data<0:
            steer.data=0
        
        self.pub_angle.publish(steer)

    def distance(self, myXY, wayXY):
        return ((myXY[0]-wayXY[0])**2+(myXY[1]-wayXY[1])**2)**0.5

    def angle_cal(self, myXY, nextXY, now_hd):
        '''calculate angle difference btw heading angle and waypoint angle
        '''
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


