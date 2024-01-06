import rospy
import numpy as np
import math
import cv2
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import Bool

from morai_msgs.msg import EgoVehicleStatus
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import time
import matplotlib.pyplot as plt
from collections import deque

class Disparity():
    def __init__(self):
        self.camRate = 0
        rospy.Subscriber('/lidar2D', LaserScan, self.disparity_extender_callback)
        rospy.Subscriber("/lidar2D_RB", LaserScan, self.lidarRB_callback)
        rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.camera_callback, queue_size=1)
        rospy.Subscriber('/Point_RB', Int32, self.point_callback)
        rospy.Subscriber('/control/angle/mcmcmc', Float64, self.angle_callback)
        # rospy.Subscriber('/Nowall', LaserScan, self.nowall_callback)

        self.steer_pub = rospy.Publisher('/control/angle/RB', Float64, queue_size=3)
        self.bool_pub = rospy.Publisher('/control/Lane_Return', Bool, queue_size=1)
        self.RBbool_pub = rospy.Publisher('/detect/RB', Bool, queue_size=1)
        
        self.DISPARITY_DIF = 0.6        # 각 1도 차이나는 두 거리차에 대한 임계값
        self.CAR_WIDTH = 0.5


        #################### 카메라 관련 ####################
        self.cam_key = True      # camera callback에서 라바콘 탐지하면 camera callback에 진입 못하도록 함.
        # self.camRate = 0             # camera hz 낮추기 위한 변수
        self.bridge = CvBridge()    # ros 데이터와 cv2 데이터 연결성 제공
        self.threshold=0      # 필터에 걸린 픽셀 수


        #################### 라이다 관련 ####################
        self.rb_key = False       # 라바콘 탐지시 RBDetect 함수 내부 진입용 변수
        self.lidar = 0              # 라이다 데이터 전역변수
        self.wall_data = 0          # 벽 세워진 라이다 데이터 전역변수 from MCRB.py
        self.lidarRate = 0           # lidar hz 낮추기 위한 변수
        self.minrange=100           # 라바콘 판단을 위한 정면 방향 최소거리

        #################### disparity 관련 ####################
        self.dp_key = False       # 라바콘 진입 이후 disparity 함수 내부 진입용 변수
        self.speed_param = rospy.get_param('~speed_param', 1000.0)
        self.P_param = rospy.get_param('~P_param', 0.0)
        self.offset = 19.4799995422     # 조향 오프셋
        self.line = 0       # 벽 세울 때 생기는 연장선의 개수
        self.list_line = []
        self.stack = deque([])      # 
        self.point_add = 0
        self.point_avg = 0
        
        # self.initialized = False      # 트랙바용 변수
        
        # self.avg_threshold = []
        # rospy.Timer(rospy.Duration(0.2), self.timer_callback)       # 카메라 연산 줄이기

        
        



    # def timer_callback(self, _event):
    #     # print('TIMER CALLBACK', time.time())
        
    #     rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.camera_callback)
    #     # print('CAMERA CALLBACK')

    def camera_callback(self, img):
        if self.cam_key==True:
            if self.camRate != 5:       # 30개 들어오는 데이터 중 5개만 쓸겨
                self.camRate += 1
                return
            else:
                self.camRate = 0
                
            print('camera_callback')
            
            
            # if self.cam_key == True and self.camRate <= 3:

            # print(self.camRate)

            ######################## 트랙바 생성 ########################

            # if self.initialized == False:
            #     cv2.namedWindow("LANE", cv2.WINDOW_NORMAL)       # Simulator_Image를 원하는 크기로 조정가능하도록 만드는 코드
            # #     # 흰색 차선을 검출할 때, 이미지를 보면서 트랙바를 움직이면서 흰색선이 검출되는 정도를 파악하고 코드안에 그 수치를 넣어준다.
            #     cv2.createTrackbar('low_H', 'LANE', 0, 255, nothing)    # Trackbar 만들기
            #     cv2.createTrackbar('low_S', 'LANE', 210, 255, nothing)
            #     cv2.createTrackbar('low_V', 'LANE', 175, 255, nothing)
            #     cv2.createTrackbar('high_H', 'LANE', 32, 255, nothing)    
            #     cv2.createTrackbar('high_S', 'LANE', 244, 255, nothing)
            #     cv2.createTrackbar('high_V', 'LANE', 255, 255, nothing)
            #     self.initialized = True  # 두 번 다시 여기 들어오지 않도록 처리
            
            ################################################################################


            #============== 카메라 imgmsg --> cv2 img --> hsv --> filtered hsv =========================

            img = self.bridge.compressed_imgmsg_to_cv2(img,'bgr8') # [y, x, rgb]    # ros 이미지 데이터 --> cv2 이미지 데이터로 변환
            # print(img.shape[0], img.shape[1])
            img1 = img[120:500,:,:]  # roi  1280 720 3(channel)
            img2 = img[:img.shape[0]//2, :, :]
            hsv1 = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
            hsv2 = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)


            # low_H = cv2.getTrackbarPos('low_H', 'LANE')
            # low_S = cv2.getTrackbarPos('low_S', 'LANE')
            # low_V = cv2.getTrackbarPos('low_V', 'LANE')
            # high_H = cv2.getTrackbarPos('high_H', 'LANE')
            # high_S = cv2.getTrackbarPos('high_S', 'LANE')
            # high_V = cv2.getTrackbarPos('high_V', 'LANE')
            # lower = np.array([low_H, low_S, low_V])
            # upper = np.array([high_H, high_S, high_V])
            lower = np.array([5, 150, 60])      # 라바콘 lower hsv 값
            upper = np.array([15, 250, 200])    # 라바콘 upper hsv 값
            
            
            # person_lower = np.array([15, 10, 133])
            # person_upper = np.array([22, 34, 161])
            
            mask = cv2.inRange(hsv1, lower, upper)      # lower ~ upper 안의 값만 추출
            # person_mask = cv2.inRange(hsv2, person_lower, person_upper)
            # print(1)
            self.threshold = np.nonzero(mask)[0]  # 빨강인 픽셀 개수
            # self.person_threshold = np.nonzero(person_mask)[0]
            
            # if self.person
            # print(2,'\n')
            # masked = mask.reshape(1,-1)
            # print(person)
            # roi = img[:,:1000,:]
            # img_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            # circles = cv2.HoughCircles(img_gray, cv2.HOUGH_GRADIENT, 1, 100, param1 = 50, param2 = 30, minRadius = 0, maxRadius = 50)
            
            # for i in circles[0]:
            #     cv2.circle(img_gray, (i[0], i[1]), i[2], (255, 255, 255), 5)

            # cv2.imshow('img',img)


            # cv2.imshow('mask',mask)
            # cv2.imshow('mask',person_mask)
            # cv2.waitKey(1)

            # print(self.threshold.shape)
            # print(masked.shape[1])
            # print(self.threshold.shape[0]/masked.shape[1])
            
            # self.threshold = 0      # hsv_gray filter에서 값 잡히는 픽셀 갯수 임계값으로 신호등과 라바콘 구분
            # print(self.threshold.shape[0])
            # for i in gray_masked:
            #     if (i>0).any():
            #         # print('###################################')
            #         # print('RB DETECTED')
            #         # print('###################################')
            #         self.threshold += len(np.nonzero(i)[0])
            
            dis_storage = []
            #############################################################################

            for i in range(len(self.lidar.ranges)):
                if 0<=i<45 or 330<=i<=359:      # 정면 기준 왼쪽 45도, 오른쪽 30도로 roi 잡음.
                    dis_storage.append(self.lidar.ranges[i])
            self.minrange = min(dis_storage)        # roi 내의 라이다 데이터 최소값 받아옴.
            # print(f' minrange : {self.minrange}')

            if (self.threshold.shape[0] > 8000) and self.minrange<=3:      # 라바콘 판단 조건 : 픽셀 8000개 이상 && 라이다 roi 내에서 라이다 데이터 거리 최소값이 3m 이내인 경우
                # pass
                self.rb_key=True                                        # 라바콘 판단 ON
                self.cam_key=False                                      # 카메라 데이터 이용한 이미지 처리부분 OFF(Camera callback OFF)
                self.RBbool_pub.publish(True)                           # 동적, 정적 객체 인식과 라바콘 분리하기 위해 토픽 쏴줌.

                    
            # print(self.threshold.shape[0])

            #############################################################################


        #############################################################################
        # 라바콘 인지하여 RBDetect 함수 진입

        if self.rb_key == True:
            self.RBDetect()
            # print(f'픽셀 개수 : {self.threshold.shape[0]}')
            

    ############################################################################################################################################################
    # 라바콘 내부로 진입
        
    def angle_callback(self, data):    
        self.vector = data.data
        # print(f'data : {data.data}')
    #     self.nowall = data

    def RBDetect(self):     # 라바콘 내부 진입 조향 알고리즘
        data = self.wall_data       # 라이다 데이터가 라바콘 인지했을 때 라바콘 사이사이를 메꾸어 벽처럼 라이다데이터 전처리함.
        print('RB DETECTED')
        if self.rb_key == True:
            
            org_ranges = list(data.ranges)
            roi_gap = 30            # 정면 라이다 각도 roi 
            
            left_storage = []        # 정면 20도 안에 있는 라이다 데이터의 [angle, range]
            right_storage = []

            for angle in range(0,roi_gap+1):    # 0 ~ roi_gap
                left_storage.append([angle, org_ranges[angle]])

            for angle in range(360-roi_gap,360):    # 359-roi_gap ~ 359도
                right_storage.append([angle, org_ranges[angle]])

            left_storage.sort(key=lambda x: x[1])     # 거리순으로 오름차순
            right_storage.sort(key=lambda x: x[1])     # "
            
            closest = []        # 가장 가까운 두 라이다 데이터의 [[angle, range], [angle, rnage]]
            
            closest.append(left_storage.pop(0))  
            closest.append(right_storage.pop(0))
            
            closest[1][0] += -360
            vector = (closest[0][0]+closest[1][0])//2
            
            vector = (-vector+self.offset)/(2*self.offset)
            # print(vector)
            if vector>1.0:
                vector=1.0
            elif vector<0:
                vector=0
            
            angle = Float64()
            angle.data = self.vector
            # print(f'vector : {self.vector}')
            self.steer_pub.publish(angle)


            disparity_list = data.ranges[80:96]+data.ranges[270:286]
            stack = 0           # 옆구리쪽 거리가 1m 이내인 인덱스 개수
            for i in disparity_list:
                if i<=0.5:
                    # print(disparity_list[i])
                    stack += 1
            # print(stack)
            if stack >= 15:              # 디스파리티 진입 조건
                self.dp_key=True        # 디스파리티 진입
                self.rb_key=False       # RBDetect 함수 진입 차단



    ############################################################################################################################################################|

    # 벽 세운 라이다 데이터 받아와서 전역변수로 지정
    def lidarRB_callback(self, data):
        self.wall_data = data


    # 벽 세울 때 생긴 연장선의 개수 받아와서 전역변수로 지정
    def point_callback(self, data):
        # print('disparity_v1')
        count = 10
        self.point_rb = data.data
        self.stack.append(self.point_rb)
        self.point_add += self.point_rb 
        if len(self.stack)==count+1:
            self.point_add -= self.stack.popleft()
            self.point_avg = self.point_add/count
        # self.stack += 1
        # self.point_rb = data.data
        # # print(self.point_rb)
        # self.point_add += self.point_rb
        # self.point_avg = self.point_add // self.stack
        # # print(f'point_avg : {self.point_avg}')
        # # print(f'point_rb : {self.point_rb}')
        


    def disparity_extender_callback(self, data):
        self.lidar = data
        
        roiBox = sorted(list(data.ranges[0:50]) + list(data.ranges[329:359]))  #정면 기준 왼쪽 50도, 오른쪽 30도에서 제일 작은값찾는 과정. 거리를 오름차순으로 분류
        self.minrange = roiBox[0]   # 가장 작은 값
        wall_data = self.wall_data            # 라이다 데이터를 벽으로 세운 데이터 활용. 전처리된 데이터가 아니라면 disparity 활용 불가.

        if self.dp_key == True:
            rospy.loginfo('DISPARITY ON')
            # len(dat1a.ranges) = 360, head 기준 반시계방향으로 0~359
            # print(np.argmin(da1ta.ranges))
            
            dis = list(wall_data.ranges[270:360] + wall_data.ranges[0:90])      # 정면 기준 180도 라이다 데이터(거리) 사용 roi
            disparities = []

            for angle in range(len(dis)):
                if angle == len(dis)-1: # indexerror 방지
                    continue
                if abs(dis[angle] - dis[angle+1]) > self.DISPARITY_DIF: # 각 1도 차이(예시_ 60도, 61도)에서의 거리차가 0.6m이상일 경우 --> 곡률)좌회전구간, 우회전구간)을 판단하는 기준임. 그림으로 그려보면 이해 쉬움.
                    min_dis = min(dis[angle], dis[angle+1])     # 두 각도에 대한 거리 중 가까운 거리 할당
                    angle_range = math.ceil(math.degrees(math.atan(self.CAR_WIDTH / 2 / min_dis)))      # 차폭/(2*두 각도 중 작은 거리)를 atan한 값
                    angle_range += 3           # 3도 정도 더 여유를 둠.     # 나도 사실 어떤 기준으로 주는지 잘 모르겠다!! 
                    side_range = range(int(angle-angle_range+1), angle+1) if dis[angle+1] == min_dis else range(angle+1, int(angle+1+angle_range)) 
                    # 좌회전의 경우) angle = 60, 61도를 예를 들면, dis[angle=60] = 1.3, dis[angle=61] = 0.5와 같은 상황임. 이러한 경우 min_dis = dis[61]인 경우이므로
                    # dis[angle] != min_dis임. 따라서 이러한 경우 angle_range만큼 더 반시계방향으로 돌려서 자신한테 더 가까운 영역(예시_80~90도)을 보겠다는 의미
                    # 그 값이 side_range
                    disparities.append((min_dis, side_range))   # 즉, disparities는 1도 간격의 두 앵글에 대한 최소값과 그 앵글에서의 좌회전 혹은 우회전인지의 상황을 담고 있음.
                    
            for min_dis, side_range in disparities:
                # print(side_range)
                for i in side_range:
                    if i >= 0 and i < len(dis):     # side_range 안의 각도가 우리의 관심 범위 안에 있다면,
                        dis[i] = min(dis[i], min_dis)   # side_range 범위 안의 각도에 대한 dis값과 현재 angle에 대해 1도 간격으로 확인한 최소거리 중 더 가까운 값으로 바꿈.
                        # 이 부분이 물리적으로 어떤 의미를 담고 있는지는 아직 정확하게 모르겠다.
                        # -90~90도에 대한 각들이(dis[angle]) side_range에 대한 값으로 바뀌거나 min_dis로 바뀌므로 roi에서의 거리는 작아진다.

            max_index = np.argmax(dis)      # dis 리스트에서 가장 큰 값에 대한 인덱스(=각도) 할당.
            max_dis = dis[max_index]
            
            angle = max_index-90 if abs(max_index - 90) > 4 else 0      # 가장 큰 각에서 90도를 뺐을 때 절대값이 4도보다 크면 max_index-90 그대로 사용. 아니라면 0도 사용.
            # 4도 이내의 값이라면 조향하지 않겠다는 의미

            angle = (-angle+self.offset)/(2*self.offset)        # morai 기준 조향에 대한 조향각 계산.

            if angle>1.0:
                angle=1.0       # 최대 조향각(1도) 넘어가면 그냥 최대 조향각 할당
            elif angle<0:
                angle=0         # 최소 조향각(0도) 넘어가면 그냥 최소 조향각 할당

            ang = Float64()
            ang.data = angle
            self.steer_pub.publish(ang)

            

            
            
            # self.avg_threshold.append(self.threshold.shape[0]//2)

            # print('#############################')
            # print(self.threshold.shape[0])
            # print(self.avg_threshold[1])
            # print('#############################')
            # self.list_line.append(self.line)      # [이전 line, 현재 line]
            
            # print(angle)

            # if self.threshold.shape[0] <= self.avg_threshold[0]:
            # print('###############')
            # print(self.line)
            # print(self.list_line)
            # print('###############')
            

            # try:
            # print(self.point_rb , self.point_avg//2)
            if self.point_rb <= self.point_avg//2:
                self.dp_key=False
                print('DISPARITY OFF')
                self.bool_pub.publish(True)
                self.steer_pub.publish(Float64(999))
                self.dp_key = False
                self.cam_key = True


            # except IndexError: pass

            # if self.threshold.shape[0] <= 200:
            #     self.dp_key=False
            #     print('DISPARITY OFF')
            #     self.bool_pub.publish(True)
            #     self.steer_pub.publish(Float64(999))
            #     self.dp_key = False
            #     self.cam_key = True


    ############################################################################################################################################################|            




    
# createTrackbar를 위한 함수
def nothing(a=1):
    return
        
def run():
    rospy.init_node('Disparity')
    Disparity()
    rospy.spin()

if __name__=='__main__':
    run()