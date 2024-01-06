#!/usr/bin/env python3
import rospy
import numpy as np
import math
import copy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from morai_msgs.msg import EgoVehicleStatus
from sensor_msgs.msg import LaserScan
import time

class lidar():
    def __init__(self):
        self.myXY = []
        self.now_hd = 2
        self.guarantee_straight = 0
        self.change_lane = 0
        self.only_one = 0
        self.mission_mode = 0
        self.change_lane_semifinal = 0
        self.normal_drive_stack = 0
        self.re_1 = True
        self.speed_factor = 1
        self.screen = 0
        self.only_one = 0

        self.re_start = 0
        self.stopstop = 0

        self.my_lane_number = 2

        self.boigisick = 0

        # 라바콘 진입/탈출시 받아오는 변수
        self.detectRB = 0
        self.lane_change_bool = 0

        # 차선변경이 여러번 나오지 않도록 막는다.
        self.straight_stack = 0

        self.moving_obs_pos = 0

        self.lane_change = rospy.Publisher('/control/Lane_Change', Bool, queue_size=1)
        self.pub_speed = rospy.Publisher('/control/speed/SODO', Float64, queue_size=1)

        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.Ego_topic_callback)
        rospy.Subscriber("/lidar2D", LaserScan, self.lidar_callback)
        # rospy.Subscriber("/control/Lane_Return_1", Bool, self.isRB)

        rospy.Subscriber("/detect/RB", Bool, self.detectRB_callback)
        rospy.Subscriber("/control/Lane_Return", Bool, self.lane_return_callback)

    def detectRB_callback(self, data):
        self.detectRB = data.data
        if  self.detectRB == 1:
            print("labacooo")
            self.mission_mode = 1

    def lane_return_callback(self, data):
        self.lane_change_bool = data.data
        if  self.lane_change_bool == 1:
            # 라바콘을 탙출하면 일반주행
            self.mission_mode = 0

    # 50 hz
    def Ego_topic_callback(self, data):
        """  수정  """
        self.myXY = [data.position.x, data.position.y]
        self.now_hd = data.heading

        if (-3 < self.now_hd and self.now_hd < 3)       or   (177 < self.now_hd or self.now_hd < -177):
            self.guarantee_straight = 1             # 차선 변경 명령을 한 번만 보내도록 설계한 변수 초기화
                
        else:
            self.guarantee_straight = 0


            self.guarantee_straight = 0  # 차선 변경 명령을 한 번만 보내도록 설계한 변수 초기화
            

        # 차선변경하고 좀 기다린다.(30스택)
        if self.change_lane == 1:
            self.straight_stack += 1

        if self.straight_stack >= 50:
            self.change_lane = 0      
            self.only_one = 0                       # 차선 변경 초기화
            self.straight_stack = 0 

        

        """  수정  """
    


    """  수정  """
    # 장외 체크
    def not_field_check(self, idx, length):
        degree = self.now_hd + idx

        obs_X = length*math.cos(degree*math.pi/180) + self.myXY[0] + 0.1 * math.cos(self.now_hd*math.pi/180)
        obs_Y = length*math.sin(degree*math.pi/180) + self.myXY[1] + 0.1 * math.sin(self.now_hd*math.pi/180)

        c = math.sqrt(((12.8 - obs_X) ** 2) + (obs_Y ** 2))

        if 20.3 < obs_X or -20.3 > obs_X:
            return 0
            
        elif 6.5 < obs_Y or -6.5 > obs_Y:
            return 0

        elif c < 2.5:
            print("로터리")
            return 0

        elif 2 < obs_Y and -10 > obs_X:
            # 라바콘 구역
            print("라바콘 발견(new obstacle)")
            print(obs_X, obs_Y)
            self.mission_mode = 1
            return 0
            
        else:
            return 1
    """  수정  """ 

    # 9 hz
    def lidar_callback(self, data):
        
        """  수정  """

        k_1 = 0
        k_2 = 0
        m = 0

        for i in range(270,450):
            if i >= 360:
                i -= 360       

            """정적 장애물"""

            if self.guarantee_straight == 1 and self.mission_mode == 0 and self.myXY[1]<=0:
                #if i == 0 and data.ranges[i] <= 2.5:
                if (i<=5 or i >=355) and data.ranges[i] <= 2.5 :
                        # 장외 체크
                        if self.not_field_check(i, data.ranges[i]):
                            print("*******정적장애물 미션*******")
                            print(i, "정적 장애물 미션을 판독했을때 정면에 있는 장애물의 거리", data.ranges[i])
                            self.mission_mode = 3

            """정적 장애물"""
            # 정적 장애물 차선 변경하기 위한 조건 제어
            if self.change_lane != 1:
                if self.mission_mode == 3:
                    if self.guarantee_straight == 1:
                        if (i<=2 or i >=358) and data.ranges[i] <= 2:
                            # 옆에 차가 없는지를 체크해야함 (장외 처리를 해야..)
                            self.change_lane_semifinal = 1

            """동적 장애물"""
            # 동적 장애물 판정
            if self.guarantee_straight == 1 and self.mission_mode == 0 and self.myXY[1]>=0:
                # 정적에서 못걸렀으면 동적이라는 뜻 그냥 
                if data.ranges[i] <= 1.5 and (data.ranges[i]>=1):

                    if 120>i < 90:
                        if self.not_field_check(i, data.ranges[i]):
                            print("*******동적장애물 미션       left*******")
                            print(i, "동적 장애물 미션을 판독했을때 장애물과의 거리", data.ranges[i])
                            self.mission_mode = 2
                            self.moving_obs_pos = 1

                    if 300>i > 270:
                        if self.not_field_check(i, data.ranges[i]):
                            print("*******동적장애물 미션       right*******")
                            print(i, "동적 장애물 미션을 판독했을때 장애물과의 거리", data.ranges[i])
                            self.mission_mode = 2
                            self.moving_obs_pos = 2
            
            """동적 장애물"""
            # 동적 장애물의 정지상태를 위한 코드
            if self.mission_mode == 2:
                # 정지시작
                if data.ranges[i] < 1.5 and (i < 60 or i > 300):
                    if self.not_field_check(i, data.ranges[i]):
                        self.stopstop = 1
                        self.re_start = 0
                
                if self.moving_obs_pos == 1:
                    if self.stopstop == 1 and (data.ranges[i] > 1.5 and (i < 70 or i > 355)):            
                        k_1 += 1
                        # 74개

                elif self.moving_obs_pos == 2:
                    if self.stopstop == 1 and (data.ranges[i] > 1.5 and (i < 5 or i > 290)):
                        k_2 += 1
                        # 74개


            """일반 주행"""
            if self.mission_mode == 2 or self.mission_mode == 3:
                if data.ranges[i] < 3:
                    if self.not_field_check(i, data.ranges[i]):
                        m += 1

        # 인덱스 5개 써서 차선변경하기    

        """동적 장애물"""   
        # 재출발
        # print(k, "K check")
        if self.stopstop == 1 and (k_1 == 74 or k_2 == 74):
            self.re_start = 1
            self.stopstop = 0

        if self.stopstop == 0 and (self.mission_mode == 2 or self.mission_mode == 3):
            if m == 0:
                self.normal_drive_stack += 1
            else:
                self.normal_drive_stack = 0

        if self.normal_drive_stack >= 60:
            self.mission_mode = 0
            self.normal_drive_stack = 0

        """정적 장애물"""
        # 전방에 발견했고
        # 이제 옆에 차만 없으면 되는 상황
        if self.change_lane_semifinal == 1:
            # 2차선 -> 1차선
            if self.my_lane_number == 2:
                for i in range(250,340):
                    if data.ranges[i] < 2:
                        if self.not_field_check(i, data.ranges[i]):
                            break
                    # 마지막까지 버텨주면
                    if i == 339:
                        self.change_lane = 1
                        self.change_lane_semifinal = 0

            # 1차선 -> 2차선
            if self.my_lane_number == 1:
                for i in range(20,110):
                    if data.ranges[i] < 2:
                        if self.not_field_check(i, data.ranges[i]):
                            break
                    # 마지막까지 버텨주면
                    if i == 109:
                        self.change_lane = 1
                        self.change_lane_semifinal = 0

        # 라바콘 미션
        if self.mission_mode == 1:
            # 혹시몰라서 주는 낮은 속도값
            self.speed_factor = 0.8
            self.straight_stack = 0
            self.screen = 0

        # 동적 장애물 미션
        if self.mission_mode == 2:
            self.screen = 0
            if self.stopstop == 1 and self.boigisick == 0:
                print("정지상태")
                self.speed_factor = 0
                self.boigisick = 1
            if self.re_start == 1 and self.boigisick == 1:
                print("재출발")
                self.speed_factor = 1
                self.boigisick = 0
                
        # 정적 장애물 미션
        elif self.mission_mode == 3:
            self.speed_factor = 0.9
            self.screen = 0
            if self.change_lane == 1 and self.only_one == 0:
                print("차선변경")
                print(data.ranges[0],"차선변경을 인식했을때 정면의 장애물과의 거리")
                self.lane_change.publish(True)
                # 한번만 보내게 컨트롤  차선 변경 모드 해체와 동시에 초기화
                self.only_one = 1
            else:
                self.lane_change.publish(False)

        # 일반 주행 모드
        # 추후에 waypoint구간으로 컷
        # 지금은 3미터 이내에 장애물이 30회 이상 등장하지 않으면 이걸로 처리
        elif self.mission_mode == 0:
            self.detectRB = 0
            self.lane_change_bool = 0
            self.speed_factor = 1
            if self.screen == 0:
                print("일반 주행")
                self.screen = 1
        # print(self.mission_mode)
        speed = Float64()
        speed.data = self.speed_factor
        self.pub_speed.publish(speed)
        # print(self.mission_mode)
        """  수정  """
        # print(self.mission_mode)
    # def isRB(self, data):
    #     self.re_1 =data.data

def run():
    rospy.init_node("lidar")
    lidar()
    rospy.spin()

if __name__ == '__main__':
    run()
