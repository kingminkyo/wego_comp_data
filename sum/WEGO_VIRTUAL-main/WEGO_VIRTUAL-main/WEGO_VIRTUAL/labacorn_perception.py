#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy  
import numpy as np                                                
from sensor_msgs.msg import LaserScan  
from std_msgs.msg import Bool
from sensor_msgs.msg import Image,CompressedImage
import cv2

enter_state = False  
end_state = False
class laba():
    def __init__(self):
        rospy.Subscriber("/lidar2D",LaserScan,self.laba_lidar_callback)
        rospy.Subscriber("/image_jpeg/compressed",CompressedImage,self.laba_camera_callback)
        self.Diparity_init = rospy.Publisher('/Disparity/init', Bool , queue_size=1)
        self.minrange = 100  #걍 아무값이나 설정, 0이면 오류 날 수도 있어서 100으로 함
    def laba_lidar_callback(self,_data):
        
        if end_state == False:
            range = sorted(list(_data.ranges[0:90]) + list(_data.ranges[270:359]))  #정면에서 제일 작은값찾는 과정
            self.minrange = range[0]
            #rospy.loginfo(self.minrange)
    def laba_camera_callback(self,_image):
        global enter_state  #진입상태에 대한 전역변수
        global end_state   #끝났는지에 대한 전역 변수
        if end_state == False:
            np_arr = np.fromstring(_image.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            lower = np.array([4,150,60])
            higher = np.array([15,255,255])
            laba_image = cv2.inRange(img,lower,higher)
            thresh_hold = np.nonzero(laba_image)[0]  #빨강인 픽셀이 몇개인지
            #rospy.loginfo(thresh_hold.shape[0])
            if enter_state == False: #라바모드에 진입하지 않았다면?
                if ((thresh_hold.shape[0]>4000) and (self.minrange<0.7)):  #조건 만족시 진입   4000, 0.7변경가능
                    rospy.loginfo('labamode on!')
                    self.Diparity_init.publish(True)   #디스파리티 켜주기, #일반 주행모드 여기서 꺼줘야 하나?
                    enter_state = True  #들어 왔는지 켜주기
            if  (enter_state == True) and (thresh_hold.shape[0]<500):  #들어온 상태고 라바콘이 거의 안보인다면 끝난거  500변경 가능
                rospy.loginfo('labamode end!')
                self.Diparity_init.publish(False)   #디스파리티 꺼주기, 디스파리티 노드에서 주행모드 켜줘도 되고 여기서 켜줘도 됨 나중에 얘기해봅시다.
                end_state = True
        else:
            rospy.loginfo('laba end!')    
        
            
       
    
def run():
    rospy.init_node("laba_perception")  #이 이름으로 노드를 생성할거니 ros mater에 등록하는 부분  중복만 하지 않으면 가능
    new_class = laba()  #실제 동작을 담당할 객체
    rospy.spin()  #반복 

if __name__ == "__main__":
   run()  
