#!/usr/bin/env python3
import rospy
import numpy as np
import math
from std_msgs.msg import Float64
from morai_msgs.msg import EgoVehicleStatus
from sensor_msgs.msg import LaserScan

# 실험 중
# 장외는 걸러냈다.
# 동적 장애물과 정적 장애물의 구분
# 우선 동적 장애물은 움직임의 의심포인트가 많다.
# 정적 장애물은 각도를 좁히니가 의심포인트가 거의 발생하지 않았다.

# 의심점이 생겼을 때 2개이상 나오면 받아들이는데         그럼 2개이상이 나오지 않으면    어떤식으로 무시할 것인지를 계획
# 그러려면 우선 움직임의 의심점이 얼마나 다다다다 나오는지를 체크해야한다


# 추가로 라이더 클러스터링 성능 점검  특히 동적 장애물을 2개로 잡지 않도록 하자

# speed 0을 몇 번을 먹여야 정지하는가를 체크

# 조향이 없을 때만 동적장애물 체크 수행!

# 미션을 나눠서 해보자!

class lidar():
    def __init__(self):
        self.switch = 0
        self.hz_check = 0
        self.nowPT = 0

        self.obs_pt = []
        self.obs_new_pt= []
        self.obstacle = []
        self.obstacle_length = []

        self.game_over = 0

        self.ccc = 0

        self.low_speed = 1

        self.moving_stack = 0

        self.checkkk = 0

        self.moving_bomb = 0

        self.pub_speed = rospy.Publisher(
            '/commands/motor/speed', Float64, queue_size=1)
        self.pub_angle = rospy.Publisher(
            '/commands/servo/position', Float64, queue_size=1)

        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.Ego_topic_callback)
        rospy.Subscriber("/lidar2D", LaserScan, self.lidar_callback)

    # 50 hz
    def Ego_topic_callback(self, data):
        myXY = [data.position.x, data.position.y]
        # print(myXY)
        now_hd = data.heading

        degree = 0

        speed = Float64()

        if self.game_over == 1 or self.checkkk == 1:
            self.ccc += 1
            if self.ccc <= 30:
                speed.data = 0
                # speed.data = 1500 * self.low_speed
                self.pub_speed.publish(speed)
            else:
                speed.data = 0
                self.pub_speed.publish(speed)

        else:
            speed.data = self.low_speed * 2500
            self.pub_speed.publish(speed)

        if now_hd <= 0 and -180 <= now_hd:
            check_degree = now_hd + 360
        else:
            check_degree = now_hd

        if check_degree <= 90:
            right = check_degree - 90 + 360
            left = check_degree + 90
        elif check_degree >= 270:
            right = check_degree - 90
            left = check_degree + 90 - 360
        else:
            right = check_degree - 90
            left = check_degree + 90

        try:
            # 오차를 줄이기 위한 필터링 (ego topic이 들어오는 주파수와 lidar의 주파수가 약 5배 차이가 나므로 라이다 값이 들어왔을 때만 딱 한 번 ego topic에서 받아온 것과 계산하게 구현)
            # 차의 길이 0.4 자전거 길이 0.35
            if self.hz_check != self.switch:
                self.hz_check = self.switch
                for i in range(len(self.obstacle)):
                    degree = now_hd + self.obstacle[i]
                    obs_X = self.obstacle_length[i]*math.cos(degree*math.pi/180) + myXY[0] + 0.1 * math.cos(now_hd*math.pi/180)
                    obs_Y = self.obstacle_length[i]*math.sin(degree*math.pi/180) + myXY[1] + 0.1 * math.sin(now_hd*math.pi/180)
                    
                    # print("좌표", obs_X, obs_Y)
                    
                    # 바운더리(경기장) 바깥이면 아예 처리하지 않는다.  맵바깥의 장애물들은 아예 처리를 하지 않는다.
                    if 20.3 < obs_X or -20.3 > obs_X:
                        continue
                    elif 6.5 < obs_Y or -6.5 > obs_Y:
                        continue
                    else:
                        pass    

                    self.obs_new_pt.append([obs_X, obs_Y])
                    
                    # if self.game_over == 1 and (300 < self.obstacle[i] and self.obstacle[i] < 345):
                    #     print("재출발!!!!", self.obstacle[i])
                    #     self.game_over = 2
                    #     speed.data = 2500
                    #     self.pub_speed.publish(speed)

                    print("장애물 개수", len(self.obstacle))

                    if self.checkkk == 1:
                        print("봐라")
                        self.moving_bomb += 1

                    # 의심을 거둬~
                    if self.moving_bomb >= 6:
                        print("의심을 거둬~~~!")
                        self.moving_stack = 0
                        self.checkkk = 0
                            
                    for j in range(len(self.obs_pt)):
                        d = self.distance(self.obs_new_pt[-1], self.obs_pt[j])
                        angle =  self.vector(self.obs_pt[j], self.obs_new_pt[-1])                   
                        
                        # 자전거의 속도는 5km/h이고 약 0.1초마다 한 번씩 이 과정을 반복하므로 동적 장애물이 움직인다면 매순간 약 0.138m정도의 변화로 움직일 것으로 추정
                        # 거리는 대충 0.1 0.2로 thresh hold하였다.
                        # 정적장애물은 d가 존나 작아서 if문에 못들어간다.
                        
                        # 해결해야 할 것 장애물0들의 거리가 0.2이하라면 if문안에 들어가버려 문제가 생긴다.   근데 라바콘이 아닌 이상 그럴 일은 없을 것 같아 일단 유예    
                        if ((0.05 < d and d < 0.2) and ((abs(angle-right) < 7) or abs(angle-right) > 346)) and self.game_over == 0:
                            # print("움직인다")
                            self.moving_bomb = 0
                            self.moving_stack += 1
                            # print(angle, right)
                            print("이전의 장애물 절대 좌표들 : ", self.obs_pt)
                            print("현재 장애물 절대 좌표들 : ", self.obs_new_pt)
                            print("현재 내 위치 : ", data.position.y)
                            print("움직임이 의심")
                            self.checkkk = 1
                            
                            speed.data = 0
                            # speed.data = 1500 * self.low_speed
                            self.pub_speed.publish(speed)

                        # else:
                        #     if self.moving_stack >= 1:
                        #         self.moving_stack -= 1
                            
                        if self.moving_stack >= 2:
                            # print("장애물 위치", len(self.obstacle))
                            print("이전의 장애물 절대 좌표들 : ", self.obs_pt)
                            print("현재 장애물 절대 좌표들 : ", self.obs_new_pt)
                            print("현재 내 위치 : ", data.position.y)
                            print("자전거 이동거리 :", d)
                            print("angle :", angle)
                            print("right :", right)
                            print("!!!!! 동적 장애물 !!!!!!")
                            self.moving_stack = 0
                            self.game_over = 1
                            speed.data = 0
                            # speed.data = 1500 * self.low_speed
                            self.pub_speed.publish(speed)
                            
                        # 추후에 정지 했다가 다시 출발할 때 self.cnt = 0으로 다시 만들어줘야한다.

            # self.obs_pt = self.obs_new_pt
            # # 2개전까지의 장애물 정보들을 받을 수 있도록 처리
            # if self.hz_check == 1:
            #     self.obs_new_pt = []
            if len(self.obs_new_pt) >= 1:
                self.obs_pt.append(self.obs_new_pt[0])
            # print("이거다 이거 시발련아 : ", self.obs_pt)
            # print("이년이다 : ", self.obs_new_pt)
            if len(self.obs_pt) >= 7:
                del self.obs_pt[:len(self.obs_pt)-7]



            self.obs_new_pt =[]

        except:
            pass

    # 순서는 늘 정해져있고 움직이는 것을 체크 리스트에 계속 담아두면서 

    # 9 hz
    def lidar_callback(self, data):
        # initializing obstacle status
        self.obstacle = []
        self.obstacle_length = []

        # print(data.ranges, "asdfasdf")

        start = 0
        reference = 0
        stack = 0
        bomb_stack = 0  # 유예를 주는 것
        result = 0

        decide_obs = 0

        # self.low_speed = 1

        for i in range(270,360):
        # 이 때부터만 추적 시작
            if start == 1:
                if bomb_stack >= 5:
                    start = 0
                    reference = 0
                    stack = 0
                    bomb_stack = 0 
                    decide_obs = 0
                    continue

                if abs(reference - data.ranges[i]) < 0.05:
                    reference = data.ranges[i]
                    result = 1
                    stack += 1
                    bomb_stack = 0

                    if stack >= 2 and decide_obs == 0:
                        self.obstacle.append(i)
                        self.obstacle_length.append(data.ranges[i])
                        decide_obs = 1
                
                else:
                    result = 2
                    bomb_stack += 1
            
            else:
                pass

            # 0.1을 한 이유 : 신호등 교차로 부근에서 뭔지 모를 라이다 미세한 에러값이 잡히는 걸로 발견해서 최하단 거리 지정
            # 3.5보다 작으면 자전거가 반응해서 먼저 움직이는 속도보다 우리의 반응 속도가 느리므로 동적 장애물의 발견이 늦어진다.
            if (0.1 < data.ranges[i] and data.ranges[i] < 3.5) and start == 0:
                self.low_speed = 0.8
                start = 1
                reference = data.ranges[i]
            
        for i in range(0,90):
            if start == 1:
                if bomb_stack >= 5:
                    start = 0
                    reference = 0
                    stack = 0
                    bomb_stack = 0 
                    decide_obs = 0
                    continue

                if abs(reference - data.ranges[i]) < 0.05:
                    reference = data.ranges[i]
                    result = 1
                    stack += 1
                    bomb_stack = 0

                    if stack >= 2 and decide_obs == 0:
                        self.obstacle.append(i)
                        self.obstacle_length.append(data.ranges[i])
                        decide_obs = 1

                else:
                    result = 2
                    bomb_stack += 1
            
            else:
                pass

            if (0.1 < data.ranges[i] and data.ranges[i] < 3.5) and start == 0:
                self.low_speed = 0.8
                start = 1
                reference = data.ranges[i]

        print(self.obstacle, "구분", self.obstacle_length)

        if self.switch == 0:
            self.switch = 1
        
        else:
            self.switch = 0

    def distance(self, myXY, wayXY):
        return ((myXY[0]-wayXY[0])**2+(myXY[1]-wayXY[1])**2)**0.5


    def vector(self, pastXY, nowXY):
        y = nowXY[1] - pastXY[1]
        x = nowXY[0] - pastXY[0]

        angle =  (np.arctan2(y, x)*(180/np.pi))
 
        if angle < 0:
            angle+=360

        return angle

def run():
    rospy.init_node("lidar")
    lidar()
    rospy.spin()

if __name__ == '__main__':
    run()
