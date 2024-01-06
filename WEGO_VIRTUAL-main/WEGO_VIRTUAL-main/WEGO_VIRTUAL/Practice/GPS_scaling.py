#!/usr/bin/env python3
import rospy
from morai_msgs.msg import GPSMessage

# 끝과 끝 좌표
# 37.61220764589936
# 126.99374631532125
# 37.61209401372003
# 126.99421197632516

class GPS_scaler():
    def __init__(self):
        self.gps = rospy.Publisher('/gps_scaled', GPSMessage, queue_size=1)
        rospy.Subscriber("/gps", GPSMessage, self.gps_callback)

    # GPS값 크게 보기
    def gps_callback(self, data):
        data.latitude-=37.61209
        data.longitude-=126.99374
        data.latitude*=10**5
        data.longitude*=10**5
        self.gps.publish(data)

def run():
    rospy.init_node("GPS_scaler")
    GPS_scaler()
    rospy.spin()

if __name__ == '__main__':
    run()