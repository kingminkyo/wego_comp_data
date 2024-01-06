#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import time
from  sinhodong import *
from weifointu import *
from Rotary import *
from new_obstacle import *
from MCRB import *
from finish import *
from disparity import *

class kontotul():
    def __init__(self):
        self.TL_mul = 1.0   # Traffic Light
        self.SODO_mul = 1.0 # Static and Dynamic Obstacle
        self.PP_ang = 0.5   # Pure Persuit
        self.RC_mul = 1.0   # Rotary Crosssection
        self.RB_mul = 1.0   # Rabacone multiplier
        self.RB_ang = 999   # Rabacone angle
        self.FIN = 1        # use if we have to stop car at finish point
        self.RB_time = 0    # Rabacone input time
        
        rospy.Subscriber('/control/angle/RB', Float64, self.angle_RB_callback)

        
        self.pub_speed = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.pub_angle = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)

    def angle_RB_callback(self, data):
        self.RB_ang = data.data
        self.RB_time = time.time()

    def angle_PP_callback(self, data):
        # at Rabacon situation, use Rabacon angle
        speed = 1000
        self.pub_angle.publish(self.RB_ang)
        # modify speed at corner
        Accel = (1-abs((self.RB_ang-0.5)/0.5))**2
        # maximum speed 1500
        speed = 1000 + 500*Accel
        # none Rabacon situation, use pure pursuit angle
    
        
        self.pub_speed.publish(Float64(speed))
        self.PP_ang = data.data
            
def run():
    rospy.init_node("kontotul")
    kontotul()      # 통합 제어
    LIDAR_RB()      # 라바콘 라이다
    Disparity()
    rospy.spin()

if __name__ == '__main__':
    run()
