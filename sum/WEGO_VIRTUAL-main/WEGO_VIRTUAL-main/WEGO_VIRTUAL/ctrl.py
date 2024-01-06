#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

class CTRL():
    def __init__(self):
        self.pub_speed = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.pub_speed = rospy.Publisher('/commands/servo/angle', Float64, queue_size=1)
        
        # 흠,,

def run():
    rospy.init_node("CTRL")
    CTRL()
    rospy.spin()

if __name__ == '__main__':
    run()