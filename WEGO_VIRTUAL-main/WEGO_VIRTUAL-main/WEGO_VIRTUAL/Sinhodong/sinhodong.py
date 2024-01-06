#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from morai_msgs.msg import GetTrafficLightStatus, GPSMessage


# 신호등 인덱스 / GPS 상 위치(latitude, longitude) (1차선 / 2차선)
# 'SN000005' / 6.342 34.033 / 6.650 34.015
# 'SN000000' / 5.927 29.663 / 5.619 29.667
# 'SN000002' / (37.61214013054629,126.99406094053042) / (37.61214057472765, 4164675.786437934) //우리가 사용하는 신호등
# 'SN000009' / 7.866 31.597 / 7.887 31.184

# 신호등 상태 / trafficLightType / trafficLightStatus
# 초직 / 2 / 16
# 노랑 / 2 / 4 (파란불 --> 자회전)
# 초좌 / 2 / 33 
# 노랑 / 2 / 5 (자회전 --> 빨간불)
# 빨정 / 2 / 1
# -------------------신호등 순서는 위와 같음-------------

TLIDX = [37.61213265847612,126.99406477665751]
        # 'SN000005':[6.500, 34.015],
        # 'SN000000': [5.778, 29.667],
        # 'SN000002': [37.61214042014249,126.99406493812783],
        # 'SN000009': [7.866, 31.400]
        

Stop_line = [[37.61213373074034,126.99406089937604],[37.61213368333758,126.99406504443137]] #1차선, 2차선

class sinhodong():

    def __init__(self):
        self.now_loc = [0,0]
        
        self.pub = rospy.Publisher("/control/speed/TL", Float64, queue_size=1)
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.TL_callback)
        rospy.Subscriber("/gps", GPSMessage, self.gps_callback)    

    def TL_callback(self, data):
        status = data.trafficLightStatus
        index = data.trafficLightIndex
        const = Float64()
        stopline_distance = self.distance(self.now_loc, TLIDX)
        const.data = 1
        distance_value=5.838285955486403e-06
        if index == "SN000002":
            if status == 33 : # 빨간불일때 (1,2,3,4는 빨간불, 신호등마다 다른듯)
                if stopline_distance<=distance_value:
                    # print("stop!")
                    const.data = 0
                # print("자회전")

            elif status == 1:
                if stopline_distance<=distance_value:
                    print("stop!")
                    const.data = 0
                # print("빨간불")

            elif status == 4 :
                if stopline_distance<=distance_value:
                    # print("stop!")
                    const.data = 0
                # print("노란불")

            elif status == 5 :
                if stopline_distance<=distance_value:
                    # print("stop!")
                    const.data = 0
                # print("노란불")    
            
            # elif status == 16:
                # print("파란불")
            self.pub.publish(const)  
            # print(stopline_distance) 
            # print(status) 

    def gps_callback(self, data):
        self.now_loc = [data.latitude, data.longitude]
    
    def distance(self, X, Y):
        return ((X[0]-Y[0])**2+(X[1]-Y[1])**2)**0.5    

def run():
    rospy.init_node("sinhodong")
    sinhodong()
    rospy.spin()

if __name__ == '__main__':
    run()
