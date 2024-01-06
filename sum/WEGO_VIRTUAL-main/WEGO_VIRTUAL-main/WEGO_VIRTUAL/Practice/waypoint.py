#!/usr/bin/env python3
import rospy
from morai_msgs.msg import EgoVehicleStatus
import sys
import termios
import tty

# spacebar 누르면 list append, 종료 키 = q

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def callback(data):
    global x_position
    global y_position
    x_position = data.position.x
    y_position = data.position.y

def receive_position():
    list_position_x = []
    list_position_y = []
    count = 0
    rospy.init_node('get_waypoint')
    rospy.Subscriber("/Ego_topic",EgoVehicleStatus, callback)
    rate = rospy.Rate(10)
    print("space bar를 누르면 waypoint 저장")
    print("종료하려면 q를 누르세요")
    while not rospy.is_shutdown():
        
        char = getch()
        if char == ' ':
            list_position_x.append(x_position)
            list_position_y.append(y_position)    
            count+=1
            print("waypoint 개수 :" , count) 
        rate.sleep()
        
        if char == 'q':
            for i in range(len(list_position_x)):
                print(list_position_x[i],list_position_y[i])
            break

if __name__ == '__main__':
    x_position = 0
    y_position = 0
    try:
        receive_position()
    except rospy.ROSInterruptException:
        pass
