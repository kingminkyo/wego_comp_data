#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Float64



class LIDAR_RB():
    def __init__(self):
        self.myXY = [0,0]
        self.now_hd = 0
        
        self.pub_angle = rospy.Publisher("/control/angle/mcmcmc", Float64, queue_size=1)
        self.lidar_rb = rospy.Publisher("/lidar2D_RB", LaserScan, queue_size=1)
        self.point_rb = rospy.Publisher('/Point_RB', Int32, queue_size=1)
        rospy.Subscriber("/lidar2D", LaserScan, self.lidar_callback)


        
    def lidar_callback(self, data):
        # lidar original data
        org_ranges = list(data.ranges)

        # convert coordinate to match with gps
        for i in range(len(org_ranges)):
            degree = self.now_hd + i
            obs_X = org_ranges[i]*np.cos(degree*np.pi/180) + self.myXY[0] + 0.1 * np.cos(self.now_hd*np.pi/180)
            obs_Y = org_ranges[i]*np.sin(degree*np.pi/180) + self.myXY[1] + 0.1 * np.sin(self.now_hd*np.pi/180)
            # ignore outside of the boundary
            if (-19.6<obs_X<19.6 and -5.9<obs_Y<5.9):
                pass
            else:
                org_ranges[i] = 10
                
        # constant number
        angle_increment = 0.01745329238474369

        # define search angle
        sa = 90
        # convert angle for searching
        ranges = org_ranges[360-sa:360] + org_ranges[0:sa]
        
        ALL_XY = []
        ROI_TXY = []
        for i in range(len(ranges)):
            x = ranges[i]*np.cos(angle_increment*i)
            y = ranges[i]*np.sin(angle_increment*i)
            ALL_XY.append([x,y])
            # set ROI 5m around the car
            if self.distance([0,0], [x,y])<5:
                ROI_TXY.append([i,x,y])
                
        # exception process
        try:
            RB_TXY = [ROI_TXY[0]]
        except:
            new_ranges = list(data.ranges)
            for i in range(len(new_ranges)):
                degree = self.now_hd + i
                obs_X = new_ranges[i]*np.cos(degree*np.pi/180) + self.myXY[0] + 0.1 * np.cos(self.now_hd*np.pi/180)
                obs_Y = new_ranges[i]*np.sin(degree*np.pi/180) + self.myXY[1] + 0.1 * np.sin(self.now_hd*np.pi/180)
                if (-19.6<obs_X<19.6 and -5.9<obs_Y<5.9):
                    pass
                else:
                    new_ranges[i] = 10
            data.ranges = new_ranges
            self.lidar_rb.publish(data)
            return
        
        
        Angle_Relation = {}
        Angle_Relation[ROI_TXY[0][0]]=[]
        for i in range(len(ROI_TXY)):
            fac=True
            for j in range(len(RB_TXY)):
                # use given information that rabacon diameter is under 0.1m
                if self.distance([RB_TXY[j][1], RB_TXY[j][2]], [ROI_TXY[i][1],ROI_TXY[i][2]])<0.1:
                    fac=False
            if fac:
                RB_TXY.append(ROI_TXY[i])
                Angle_Relation[ROI_TXY[i][0]] = []

        for i in range(len(RB_TXY)):
            for j in range(i+1,len(RB_TXY)):
                # use given information that each rabacon locate 0.3 meters away from each other
                if self.distance([RB_TXY[i][1],RB_TXY[i][2]], [RB_TXY[j][1],RB_TXY[j][2]])<0.3:
                    # make relation graph of each rabacon
                    Angle_Relation[RB_TXY[i][0]].append(RB_TXY[j][0])
                    Angle_Relation[RB_TXY[j][0]].append(RB_TXY[i][0])


        # use depth first search to make tree of rabacon relation
        self.Angle_Relation = Angle_Relation
        self.mc_visited = [False for _ in range(sa*2)]
        Lidar_Correction = []
        for i in range(len(RB_TXY)):
            if self.mc_visited[RB_TXY[i][0]]==False:
                temp = self.dfs(RB_TXY[i][0],[])
                # ignore short tree and re-dfs at the end for make correction list
                if len(temp)>2:
                    Lidar_Correction.append(self.dfs(temp[-1],[]))
        
        
        min_distance_angle = [] # [distance, angle]
        Temp_D= {}
        
        CORRECTION_TXY = []
        for relation_list in Lidar_Correction:
            temp = []
            for i in range(len(relation_list)-1):
                # range correction by using inner branch
                ang_s, x_s, y_s = relation_list[i], ALL_XY[relation_list[i]][0], ALL_XY[relation_list[i]][1]
                ang_e, x_e, y_e = relation_list[i+1], ALL_XY[relation_list[i+1]][0], ALL_XY[relation_list[i+1]][1]
                if ang_s>ang_e:
                    ang_e, x_e, y_e = relation_list[i], ALL_XY[relation_list[i]][0], ALL_XY[relation_list[i]][1]
                    ang_s, x_s, y_s = relation_list[i+1], ALL_XY[relation_list[i+1]][0], ALL_XY[relation_list[i+1]][1]
                # use for maneuver car in front of rabacon
                Temp_D[ang_s] = self.distance([0,0], [x_s, y_s])
                Temp_D[ang_e] = self.distance([0,0], [x_e, y_e])
                for j in range(ang_s+1,ang_e):
                    corr_x = x_s + (x_e-x_s)*((j-ang_s)/(ang_e-ang_s))
                    corr_y = y_s + (y_e-y_s)*((j-ang_s)/(ang_e-ang_s))
                    
                    # calculate minimum distance with each tree
                    if 60<j<120:
                        temp.append([self.distance([0,0], [corr_x, corr_y]), j])
                    try:
                        if Temp_D[j]>self.distance([0,0], [corr_x, corr_y]):
                            Temp_D[j] = self.distance([0,0], [corr_x, corr_y])
                            CORRECTION_TXY.append([j, corr_x, corr_y])
                    except:
                        Temp_D[j] = self.distance([0,0], [corr_x, corr_y])
                        CORRECTION_TXY.append([j, corr_x, corr_y])
            # sort tree by distance with car
            temp.sort()
            # exception process
            if temp:
                min_distance_angle.append(temp[0])
        try:
            # sort angle information by distance with car
            min_distance_angle.sort()
            
            # two angle that car will pursuit
            a = min_distance_angle[0][0]
            b = min_distance_angle[1][0]
            
            # correction control angle by adding weight of distance
            johanggack = -(min_distance_angle[0][1]*(a/(a+b)) + min_distance_angle[1][1]*(b/(a+b))) + 90
            print(johanggack)
            offset = 19.4799995422
            steer = Float64()
            steer.data = (johanggack+offset)/(2*offset)
            print(steer.data)
            
            if steer.data>1.0:
                steer.data=1.0
            elif steer.data<0:
                steer.data=0
            self.pub_angle.publish(steer)
            print('\n')
        except: pass
                
        new_ranges = list(data.ranges)
        for i in range(len(CORRECTION_TXY)):
            angle = CORRECTION_TXY[i][0]
            x = CORRECTION_TXY[i][1]
            y = CORRECTION_TXY[i][2]
            # reconvert angle for lidar data
            if angle<sa:
                # new_ranges[360-sa+angle] = min(org_ranges[360-sa+angle], self.distance([0,0], [x,y]))
                new_ranges[360-sa+angle] = self.distance([0,0], [x,y])
            else:
                # new_ranges[angle-sa] = min(org_ranges[angle-sa], self.distance([0,0], [x,y]))
                new_ranges[angle-sa] = self.distance([0,0], [x,y])
    
        # convert coordinate to match with gps
        for i in range(len(new_ranges)):
            degree = self.now_hd + i
            obs_X = new_ranges[i]*np.cos(degree*np.pi/180) + self.myXY[0] + 0.1 * np.cos(self.now_hd*np.pi/180)
            obs_Y = new_ranges[i]*np.sin(degree*np.pi/180) + self.myXY[1] + 0.1 * np.sin(self.now_hd*np.pi/180)
            # ignore outside of boundary
            if (-19.6<obs_X<19.6 and -5.9<obs_Y<5.9):
                pass
            else:
                new_ranges[i] = 10

        # count number of angle that modified
        cnt = 0
        for i in range(len(new_ranges)):
            if new_ranges[i]<10 and new_ranges[i]!=org_ranges[i]:
                cnt += 1

        data.ranges = new_ranges
        self.lidar_rb.publish(data)
        self.point_rb.publish(cnt)

    def distance(self, myXY, wayXY):
        '''return distance btw 2pt
        '''
        return ((myXY[0]-wayXY[0])**2+(myXY[1]-wayXY[1])**2)**0.5

    def dfs(self, start, visited=[]):
        '''deep first search
        
        search relationship btw rabacons
        '''
        global Angle_Relation, mc_visited
        visited.append(start)
        self.mc_visited[start] = True
        for node in self.Angle_Relation[start]:
            if node not in visited:
                self.mc_visited[start] = True
                self.dfs(node, visited)
        return visited
def run():
    rospy.init_node("lidarRB")
    LIDAR_RB()
    rospy.spin()


if __name__ == '__main__':
    run()

