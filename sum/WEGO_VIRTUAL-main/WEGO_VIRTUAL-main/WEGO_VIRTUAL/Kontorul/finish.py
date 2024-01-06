#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

class finish_line():
    def __init__(self):
        self.cnt =0
        rospy.Subscriber("/final_index_cnt", Float64, self.Ego_topic_callback)
        self.pub = rospy.Publisher("/control/speed/FIN",Float64, queue_size =1)

    def Ego_topic_callback(self, data):
        nowPT = data.data
        if nowPT == 201:
            self.cnt+=1
        # print(self.cnt)
        if self.cnt ==2:
            speed =0
            self.pub.publish(speed)

def run():
    rospy.init_node("finish")
    finish_line()
    rospy.spin()

if __name__ == '__main__':
    run()