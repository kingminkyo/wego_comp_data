#!/usr/bin/env python3
import rospy
import numpy as np
import math
import copy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from morai_msgs.msg import EgoVehicleStatus
from sensor_msgs.msg import LaserScan

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

        # 직진성이 보장
        # if (   (   (-1 < self.now_hd and self.now_hd < 1) or (89 < self.now_hd and self.now_hd < 91)   )    or (-89 > self.now_hd and self.now_hd > -91)   )   or   (179 < self.now_hd or self.now_hd < -179):
        #     self.guarantee_straight = 1
        
        # print(self.now_hd,"이거다 이놈이다")

        if (-3 < self.now_hd and self.now_hd < 3)       or   (177 < self.now_hd or self.now_hd < -177):
            self.guarantee_straight = 1             # 차선 변경 명령을 한 번만 보내도록 설계한 변수 초기화
                
        else:
            self.guarantee_straight = 0

        # 차선변경하고 좀 기다린다.(20스택)
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

        # if (10.8< self.MYpoint[0]<14.8 ) and  (-2.6<self.MYpoint[1]<2.6):

        # 장외면 -1을 준다.                         # 로터리도 좌표따서 추가하면 됨
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
        #print(data.ranges[0],"asfdasdfasdf")
        # lidar clustering
        # 반시계 방향으로 180도를 탐지   시뮬 라이다는 360개의 데이터를 1도 각도 간격으로 받아온다.
        for i in range(270,450):
            if i >= 360:
                i -= 360       

            # """라바콘 판정 끼워넣기"""  
            # if data.ranges[i] <= 3 and (i < 90 or i > 270):
            #     if self.not_field_check(i, data.ranges[i]):
                    
            #         pass

            """정적 장애물"""
            # 정적 장애물 판정
            # print(self.guarantee_straight, "nanananannanana", self.mission_mode)
            if self.guarantee_straight == 1 and self.mission_mode == 0:
                if i == 0 and data.ranges[i] <= 2.5:
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
                        if (i<=4 or i >=356) and data.ranges[i] <= 2:
                            # 옆에 차가 없는지를 체크해야함 (장외 처리를 해야..)
                            self.change_lane_semifinal = 1

            """동적 장애물"""
            # 동적 장애물 판정
            if self.guarantee_straight == 1 and self.mission_mode == 0:
                # 정적에서 못걸렀으면 동적이라는 뜻 그냥 
                if i>=300 and i <=30:
                    if data.ranges[i] <= 1.5 and (data.ranges[i]>=1):
                        if self.not_field_check(i, data.ranges[i]):
                            print("*******동적장애물 미션       left*******")
                            self.mission_mode = 2
                            self.re_start = 0
                            self.moving_obs_pos = 1
                            
                        if self.moving_obs_pos == 1:
                            if self.stopstop == 1 and (data.ranges[i] > 1.5 ):            
                                k_1 += 1
                # if data.ranges[i] <= 1.5 and (data.ranges[i]>=1):
                #     if i < 90:
                #         if self.not_field_check(i, data.ranges[i]):
                #             print("*******동적장애물 미션       left*******")
                #             print(i, "동적 장애물 미션을 판독했을때 장애물과의 거리", data.ranges[i])
                #             self.mission_mode = 2
                #             self.moving_obs_pos = 1
                #     if i > 270:
                #         if self.not_field_check(i, data.ranges[i]):
                #             print("*******동적장애물 미션       right*******")
                #             print(i, "동적 장애물 미션을 판독했을때 장애물과의 거리", data.ranges[i])
                #             self.mission_mode = 2
                #             self.moving_obs_pos = 2
            
            """동적 장애물"""
            # 동적 장애물의 정지상태를 위한 코드
            # if self.mission_mode == 2:
            #     # 정지시작
            #     if data.ranges[i] < 1.5 and (i < 60 or i > 300):
            #         if self.not_field_check(i, data.ranges[i]):
            #             self.stopstop = 1
            #             self.re_start = 0
                
                
                        # 74개

                # elif self.moving_obs_pos == 2:
                #     if self.stopstop == 1 and (data.ranges[i] > 1.5 and (i < 5 or i > 290)):
                #         k_2 += 1
                        # 74개

            # """동적 장애물"""
            # # 동적 장애물 판정
            # if self.guarantee_straight == 1 and self.mission_mode == 0:
            #     # 정적에서 못걸렀으면 동적이라는 뜻 그냥 
            #     if data.ranges[i] <= 1.5 and (i < 70 or i > 340) and data.ranges[i]>=1:
            #         print("딱대")
            #         if self.not_field_check(i, data.ranges[i]):
            #             print("*******동적장애물 미션*******")
            #             print(i, "동적 장애물 미션을 판독했을때 정면에 있는 장애물의 거리", data.ranges[i])
            #             self.mission_mode = 2
            
            # """동적 장애물"""
            # # 동적 장애물의 정지상태를 위한 코드
            # if self.mission_mode == 2:
            #     # 정지시작
            #     if data.ranges[i] < 1.5 and (i < 90 or i > 350):
            #         if self.not_field_check(i, data.ranges[i]):
            #             self.stopstop = 1
            #             self.re_start = 0
                
            #     # 재출발은 정지시작인 상태에서만 발생할 수 있다.
            #     # 정면에 아무것도 안집히면 
            #     elif self.stopstop == 1 and (data.ranges[i] > 1.2 and (i < 70 or i > 355)):
            #         k += 1
            #         # if self.not_field_check(i, data.ranges[i]):

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


# #!/usr/bin/env python3
# import rospy
# import numpy as np
# import math
# import copy
# from std_msgs.msg import Float64
# from std_msgs.msg import Bool
# from morai_msgs.msg import EgoVehicleStatus
# from sensor_msgs.msg import LaserScan

# # 실험 중
# # 장외는 걸러냈다.    로터리도 거를 예정
# # 동적 장애물과 정적 장애물의 구분

# # 추가로 라이더 클러스터링 성능 점검  특히 동적 장애물을 2개로 잡지 않도록 하자

# # speed 0을 몇 번을 먹여야 정지하는가를 체크

# # 조향이 없을 때만 동적장애물 체크 수행!

# class lidar():
#     def __init__(self):
#         self.nearest_idx = 0
#         self.nearest_length = 0

#         # 모드 출력 관련 변수
#         self.stack = 0
#         self.screen = 0                # 일반 주행과 다른 미션들을 구분
#         self.hanbun = 0 # 위협 모드 1번만 출력하도록

#         # 주파수 조절 변수
#         self.switch = 0
#         self.hz_check = 0

#         # 장애물들의 정보
#         self.obstacle = []                      # 장애물들의 인덱스
#         self.obstacle_length = []               # 장애물들의 거리

#         self.speed_factor = 1                   # 속도 인자
#         self.mission_mode = 0                   # 미션 모드
#         self.accessing_length = []              # 장애물이 있고 직진할 때만 받아오는 정보로 

#         # 동적 장애물
#         self.straight_stack = 0
#         self.check = -1
#         self.boigisick = 0

#         # 정적 장애물
#         self.my_lane_number = 2
#         self.done_zikzun = 0
#         self.done = 0
#         self.only_one = 0
#         self.not_first = 0
#         self.signal = 0

#         self.lane_change = rospy.Publisher('/control/Lane_Change', Bool, queue_size=1)
#         self.pub_speed = rospy.Publisher('/control/Speed/SODO', Float64, queue_size=1)

#         # self.pub_speed = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
#         # self.pub_angle = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
#         rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.Ego_topic_callback)
#         rospy.Subscriber("/lidar2D", LaserScan, self.lidar_callback)

#     # 50 hz
#     def Ego_topic_callback(self, data):
#         myXY = [data.position.x, data.position.y]
#         now_hd = data.heading
        
#         idx_temp = []

#         try:
#             # print("장애물 개수", len(self.obstacle))
#             # ========== 장외인 장애물들을 걸러내기 위한 코드 ==========
#             # 오차를 줄이기 위한 필터링 (ego topic이 들어오는 주파수와 lidar의 주파수가 약 5배 차이가 나므로 라이다 값이 들어왔을 때만 딱 한 번 ego topic에서 받아온 것과 계산하게 구현)
#             # 차의 길이 0.4 자전거 길이 0.35
#             if self.hz_check != self.switch:
#                 self.hz_check = self.switch
#                 for i in range(len(self.obstacle)):
#                     degree = now_hd + self.obstacle[i]




#                     # 절
#                     # 대
#                     # 좌
#                     # 표  






#                     # 절대 좌표 생성      내 라이더로부터 떨어진 거리 * 삼각함수 값 + 나의 에고 토픽 좌표 + 나의 에고 토픽 좌표가 차의 뒷부분에 찍히기 때문에 그 만큼 보정     (0.1은 차의 중심과 에고토픽이 찍히는 지점 사이의 거리임) 
#                     obs_X = self.obstacle_length[i]*math.cos(degree*math.pi/180) + myXY[0] + 0.1 * math.cos(now_hd*math.pi/180)
#                     obs_Y = self.obstacle_length[i]*math.sin(degree*math.pi/180) + myXY[1] + 0.1 * math.sin(now_hd*math.pi/180)

                    
#                     # 바운더리(경기장) 바깥이면 아예 처리하지 않는다.  맵바깥의 장애물들은 아예 처리를 하지 않는다.        로터리 원도 처리해두기
#                     # 또한 장애물로 받아왔던 정보들도 삭제해준다.(del)   
#                     # self.obstacle 장애물의 인덱스 정보   self.obstacle_length 장애물의 라이다 길이

#                     if 20.3 < obs_X or -20.3 > obs_X:
#                         idx_temp.append(i)
#                         # print(i)
#                         pass
#                     elif 6.5 < obs_Y or -6.5 > obs_Y:
#                         idx_temp.append(i)
#                         # print(i)
#                         pass
#                     else:
#                         self.danger = 0.5
#                         pass   

#                 # del을 사용하다가 인덱스의 문제가 생기는 바람에 이렇게 처리하게 되었다.
#                 for j in range(len(idx_temp)):
#                     del self.obstacle[idx_temp[j]-j]
#                     del self.obstacle_length[idx_temp[j]-j]  

#                 # print(self.obstacle, "걸렀다.", self.obstacle_length)                  

#         except:
#             pass

#         try:
#             # 여기까지면 장애물이 완벽히 처리

#             if len(self.obstacle) == 0:
#                 # 스택이 쌓이면 그냥 일반 driving으로 고정                                                       # 스택 갯수 튜닝
#                 if self.stack >= 50:
#                     self.mission_mode = 0
#                     self.stack = 0
#                 elif self.mission_mode != 0:
#                     self.stack += 1
#                 else:
#                     pass

#             # 장애물 있으면 무조건 미션이라 볼 수 있다.
#             # 여기까지오면 동적 장애물 또는 정적 장애물 상황이다. (+ 라바콘)
#             # 그래서 여기까지 오면 조금 감속을 하자
#             # 나중에 튜닝해서 이 펙터를 최대한 증가시키는 방향으로 구현
#             # case 분류하기                           모드로 구분하기
#             # 1. 직진하는 경우                         # 동적 장애물 혹은 정적 장애물                      직진할 때만 accessing_length를 저장해둔다.                  
#             # 2. 직진하지 않는 경우                     # 따로 처리하지 않는다.
#             # 3. 라바콘                               # 라바콘을 가장 먼저 걸러내야한다.
#             else:
#                 # 스택 0으로 만들어주기
#                 self.stack = 0
#                 # 가장 가까운 장애물 찾기
#                 nearest_length = 10000             # 뒤에 해결하려고
#                 for i in range(len(self.obstacle)):
#                     if nearest_length >= self.obstacle_length[i]:
#                         nearest_length = self.obstacle_length[i]
#                         self.nearest_idx = self.obstacle[i]

#                 self.nearest_length = nearest_length

#                 # 이미 모드가 정해져 있는 경우
#                 # 4번 위협모드일 때는 고려하지 않는다.
#                 if self.mission_mode >= 1 and self.mission_mode <= 3:
#                     pass

#                 # 모드가 없으면 
#                 # 무슨 미션인지를 판정을 먼저 해야한다.
#                 else:
#                     # 위협지역에 장애물이 측정되면
#                     # 일단 위협모드로 넣어놓는다.
#                     self.mission_mode = 4


#                     # ========= 라바콘 판정=========                      카메라를 활용하거나 라이다가 엄청 나게 들어오는 식으로 처리해야할 듯
#                     # self.mission_mode = 1
                    




#                     # 우선 직진을 먼저 체크 직진을 하지 않는다면 미션 판정 자체를 하지 않는다.
#                     # ========= 직진하고 있냐 판단하는 코드 ==========
#                     # 방법1 차선 구간 별로 직진의 헤딩방향이 어딘지 마킹을 해주거나
#                     # 방법2 그냥 수직의 움직임에서만 직진으로 판단                        일단 이걸로 구현함
#                     if (   (   (-5 < now_hd and now_hd < 5) or (85 < now_hd and now_hd < 95)   )    or (-85 > now_hd and now_hd > -95)   )   or   (175 < now_hd or now_hd < -175):
#                         if 270 < self.nearest_idx and self.nearest_idx < 360:
#                             # 차의 오른쪽에 있는 각도를 편하게 계산하기 위해서
#                             gaeshan = 90 - (self.nearest_idx - 270)

#                         else:
#                             gaeshan = self.nearest_idx

#                         now_sin = self.nearest_length * math.sin(gaeshan*math.pi/180)

#                         # print(now_sin, "가로 길이", self.nearest_length, gaeshan)
#                         self.accessing_length.append(now_sin)
#                         # print(self.accessing_length, "첫번째놈!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

#                         # ========= 동적 장애물 판정=========
                            
#                         #                              튜닝해야할 숫자 0.15  
                        
#                         if abs(self.accessing_length[0] - now_sin) > 0.3:
#                             # print(nearest_idx)
#                             # print(self.accessing_length[0])
#                             # print(now_sin)
#                             # print(abs(self.accessing_length[0] - now_sin))
#                             print("!!!!!!!동적장애물 미션!!!!!!!")
#                             print(self.nearest_length, "튜닝", self.nearest_idx)
#                             self.mission_mode = 2

#                         # self.accessing_length.append(now_sin)

#                         # 이만큼 왔는데도 판정이 안된다?? 그냥 정적 장애물 모드다.
#                         #                                                  추가 이거랑 or걸어서 가장 가까운 장애물이 내 정면에 있을 때 정적장애물로 판정할까 싶다.
#                         # ========= 정적 장애물 판정=========


#                         # 동적 장애물 판정이 언제 이루어지는지 체크를 하고 거기 정도에서 끊어서 정적 장애물로 보낸다.

#                         #                      동적 장애물 미션에서 거의 다 1.5에서 커트가 난다.
#                         # if self.nearest_length < 1.5:
#                         #     print("*******정적장애물 미션*******")
#                         #     self.mission_mode = 3


#                     # ========= 직진하고 있지 않을 때는 미션 판정하지 말자 =========
#                     else:
#                         pass
                    
#             # 미션 모드별 처리
#             if self.mission_mode == 1:
#                 self.screen = 0
#                 self.labba_cone()
#                 pass

#             # 동적 장애물 미션
#             elif self.mission_mode == 2:
#                 self.screen = 0
#                 self.moving_obs()

#             elif self.mission_mode == 3:
#                 self.screen = 0
#                 self.speed_factor = 0
#                 # 일단 처음에는 위급하니까 바로 차선 변경하는 명령을 보내주고
#                 if self.only_one == 0:
#                     self.lane_change.publish(True)
#                     self.signal = 1
#                     self.only_one = 1
                
                

#                 # 두번째 차선 변경부터는 이걸 사용
#                 if self.not_first == 1:
#                 # 완전 직진인 상황에서만
#                     if (   (   (-1 < now_hd and now_hd < 1) or (89 < now_hd and now_hd < 91)   )    or (-89 > now_hd and now_hd > -91)   )   or   (179 < now_hd or now_hd < -179):
#                         self.straight_stack += 1
#                         # 3번 연달아 직진할 때까지 기다려주려고 만든 코드 그래야 직진성이 보장되니까

#                     # 이제 차선 변경
#                     # 모든 조건을 만족한 케이스 차선변경해주고 변수를 초기화 해주면 된다.
#                     if self.done == 1:
#                         # 차선 변경
#                         # bool인자로만 보내주면 된다.
#                         self.lane_change.publish(True)
#                         self.signal = 1
#                         self.straight_stack = 0                           # 위에서부터 밑으로 쌓여가며 처리해가며 결국 차선 변경을 판단한다.
#                         self.done_zikzun = 0
#                         self.done = 0

#                 if self.my_lane_number == 2:
#                     self.my_lane_number = 1
#                 else:
#                     self.my_lane_number = 2

#                 self.not_first = 1

#                 if self.signal != 1:
#                     self.lane_change.publish(False)
#                 else:
#                     self.signal = 0

#             # 위협모드
#             elif self.mission_mode == 4:
#                 if self.hanbun == 0:
#                     print("위협모드")
#                     self.hanbun = 1
#                 self.screen = 0
#                 self.speed_factor = 0.7
#                 pass

#             else:
#                 self.hanbun = 0
#                 if self.screen == 0:
#                     print("일반 주행")
#                     self.screen = 1

#                 self.speed_factor = 1
#                 pass

#             speed = Float64()
#             speed.data = self.speed_factor
#             self.pub_speed.publish(speed)

#             # 스피드와 조향을 최종적으로 보낸다.(테스트할 때만)

#             # 이건 그냥 내 테스트용
#             # speed = Float64()
#             # speed.data = self.speed_factor * 2500
#             # self.pub_speed.publish(speed)

#             # angle = Float64()
#             # angle.data = 0.5 + self.go_next_lane
#             # self.pub_angle.publish(angle)

#         except:
#             pass

#     # 9 hz
#     def lidar_callback(self, data):
#         # initializing obstacle status
#         self.obstacle = []
#         self.obstacle_length = []

#         start = 0
#         reference = 0
#         stack = 0
#         bomb_stack = 0  # 유예를 주는 것
#         result = 0

#         decide_obs = 0

#         # lidar clustering
#         # 반시계 방향으로 180도를 탐지   시뮬 라이다는 360개의 데이터를 1도 각도 간격으로 받아온다.
#         for i in range(270,450):
#             if i >= 360:
#                 i -= 360
#             # 정적 장애물에서의 차선 변경을 위해 저장해두는 코드
#             # 클러스터링 갈꺼까지 없다.

#             # 정적 장애물
#             # 직진의 확인되고 3번이 지났을 때 체크
#             # 직진이 보장된 상황이며 이때 완전 극정면 인덱스에 장애물이 들어왔는지를 체크
#             if self.straight_stack >= 3:
#                 if data.ranges[i] < 1.5 and (359 <= i or i <= 1):
#                     self.done_zikzun = 1

#             # 정적 장애물 판정을 위한 코드
#             if data.ranges[i] < 1.25:
#                 if self.mission_mode == 4 or self.mission_mode == 0:
#                     print("*******정적장애물 미션*******")
#                     print(i, "asdf", data.ranges[i])
#                     self.mission_mode = 3

#             # 동적 장애물의 정지상태를 위한 코드
#             if data.ranges[i] < 1.2 and (i < 90 or i > 350):
#                 self.check = 0
            
#             # 이 때부터만 추적 시작
#             if start == 1:
#                 if bomb_stack >= 5:
#                     start = 0
#                     reference = 0
#                     stack = 0
#                     bomb_stack = 0 
#                     decide_obs = 0
#                     continue

#                 if abs(reference - data.ranges[i]) < 0.05:
#                     reference = data.ranges[i]
#                     result = 1
#                     stack += 1
#                     bomb_stack = 0

#                     if stack >= 1 and decide_obs == 0:
#                         self.obstacle.append(i)
#                         self.obstacle_length.append(data.ranges[i])
#                         decide_obs = 1
                
#                 else:
#                     result = 2
#                     bomb_stack += 1
            
#             else:
#                 pass

#             # 0.1을 한 이유 : 신호등 교차로 부근에서 뭔지 모를 라이다 미세한 에러값이 잡히는 걸로 발견해서 최하단 거리 지정
#             # 3.5보다 작으면 자전거가 반응해서 먼저 움직이는 속도보다 우리의 반응 속도가 느리므로 동적 장애물의 발견이 늦어진다.
#             if (0.1 < data.ranges[i] and data.ranges[i] < 3.5) and start == 0:
#                 start = 1
#                 reference = data.ranges[i]

#         # 전방에 발견했고 
#         # 이제 옆에 차만 없으면 되는 상황
#         if self.done_zikzun == 1:
#             # 2차선 -> 1차선
#             if self.my_lane_number == 2:
#                 for i in range(250,340):
#                     self.done = 1
#                     if data.ranges[i] < 0.6:
#                         self.done = -1
#                         break
#             # 1차선 -> 2차선
#             if self.my_lane_number == 1:
#                 for i in range(0,110):
#                     self.done = 1
#                     if data.ranges[i] < 0.6:
#                         self.done = -1
#                         break

#         # print(self.obstacle, "구분", self.obstacle_length)

#         if self.switch == 0:
#             self.switch = 1
        
#         else:
#             self.switch = 0

#     def labba_cone(self):
#         pass

#     # 바운더리에 걸리지 않는 장애물들의 인덱스를 쏴준다.
#     # 또는 바운더리에 걸리면 그냥 그 해당 인덱스를 지우고 보내는 것이 맞겠다!
#     # 한 번 미션 모드 설정이 되면 끝났다고 판단할 때 까지 끝내지 않는다.
#     def moving_obs(self):
#         # 재출발 영역
#         #                                                            인덱스보다는 거리 구한 걸로 해볼까 싶다.  인덱스 즉 각도는 장애물과 떨어진 거리마다 너무 다르니까
#         if 300 < self.nearest_idx and self.nearest_idx < 345:
#             if self.boigisick == 1:
#                 print("재출발!!!!!")
#                 self.boigisick = 0
#             self.check = 1
#             self.speed_factor = 0.7

#         elif self.check == 0 or ((350 <= self.nearest_idx or self.nearest_idx < 18) and self.nearest_length < 1.5):
#             if self.boigisick == 0:
#                 print("정지시작!!!!!")
#                 self.boigisick = 1
#             self.check = 0
#             self.speed_factor = 0

#         else:
#             # 초기 디폴트
#             self.speed_factor = 0.7
        
#         # # 정지 시작 다음은 무조건 재출발이 되어야한다!!
#         # elif self.speed_factor != 0:
#         #     # print("아직은 전진!!!!!")
#         #     self.speed_factor = 1

#         # accessing_length가 줄어든다는 판단이 섰을 때 부터 여기를 처리

#     # 제일 가까운 놈으로 하게 되면 한놈 제꼈을때 그 이후가 힘들다.


#     """딱히 필요가 없었다"""


#     # def stay_obs(self, nearest_idx, nearest_length):
#     #     # 나는 직진으로 가는 중이며 동시에 직전 앞의 인덱스에 일정 거리 안에 들어가는 값이 있다면?
#     #     # ++ 내 옆에는 아무것도 없을 때
#     #     if 350 < nearest_idx or nearest_idx < 10:
#     #         if nearest_length < 1.5:
#     #             # print("이동해!!!!!!!!!!!!!!!!!!!!!!!!!!!")
#     #             return -5
                
#     #             # 옆차선으로 이동
#     #             # 앞에 있는 것 중에 가장 가까운 장애물이 일정 거리 안으로 들어오면 옆차선으로 옮기기

#     #     return 0
        

# def run():
#     rospy.init_node("lidar")
#     lidar()
#     rospy.spin()

# if __name__ == '__main__':
#     run()
    
