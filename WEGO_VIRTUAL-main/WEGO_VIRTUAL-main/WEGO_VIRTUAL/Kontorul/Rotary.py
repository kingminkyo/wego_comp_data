#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy  #Python에서 ros를 활용할 수 있도록 하는 부분
import numpy as np                                                
from sensor_msgs.msg import LaserScan  #sensor_msgs/LaserScan 타입의 /scan 토픽을 받는 코드 작성 예정
from morai_msgs.msg import EgoVehicleStatus
from std_msgs.msg import Float64


class myDBSCAN():  #DBSCAN클러스터링  3d라이다에서 많이사용

    def __init__(self, eps=0.3, min_samples=2):
        self.eps = eps
        self.min_samples = min_samples
        self.is_core = None
        self.labels = None
        self.X = None
    def get_neighbor(self, point, data, eps):
        ## eps 반경에 들어오는 데이터 여부
        return np.linalg.norm(data-point, axis=1)<eps
    def get_closest(self, target_idx, others_idx, X, eps):
        ## 자신을 제외한 가장 가까운 데이터 인덱스 반환
        point = X[target_idx]
        data = X[others_idx]
        return others_idx[np.argmin(np.linalg.norm(data-point, axis=1))]
    def fit(self, X):
        def assign_cluster(core_idx, X, is_core, eps):
            if core_idx in visited_idx:
                return
            visited_idx.append(core_idx)
            neighbor_idx = np.where(self.get_neighbor(X[core_idx], X, eps) == True)[0]
            neighbor_idx = set(neighbor_idx)-set(non_core_idx)
            if len(neighbor_idx) == 0:
                return
            for ni in neighbor_idx:
                if labels[ni] == -1:
                    labels[ni] = cluster_label
            other_idx = np.array(list(set(neighbor_idx)-set(visited_idx)))
            for ci in other_idx:
                assign_cluster(ci, X, is_core, eps)
        ## Assign core and non-core
        is_core = []
        for i in range(X.shape[0]):
            target_data = X[i]
            if np.sum(self.get_neighbor(target_data, X, self.eps)) >= self.min_samples:
                is_core.append(1)
            else:
                is_core.append(-1)
        is_core = np.array(is_core)
        self.is_core = is_core    #core인 인덱스 = 1
        ## Assign Cluster 
        labels = np.array([-1]*X.shape[0])
        core_indice = np.where(is_core == 1)[0]  #core인 인덱스
        init_core_idx = core_indice[0] ## initial core point
        cluster_label = 0
        non_core_idx = np.where(is_core == -1)[0]
        visited_idx = [] ## 클러스터 할당된 core point
        prev_cluster_list = [] ## 이전 클래스 라벨 리스트
        while True:
            assign_cluster(init_core_idx, X, is_core, self.eps) ## Assign core point cluster
            temp_labels = labels.copy()
            temp_not_assign_cluster_idx = np.where(labels == -1)[0]
            assign_cluster_idx = [i for i, x in enumerate(labels) if x != -1 and x in prev_cluster_list]
            ## Assign non core point cluster
            for nci in non_core_idx:
                others_idx = np.array(list(set(range(X.shape[0]))-set([nci])))
                neighbor_idx = np.where(self.get_neighbor(X[nci], X, self.eps) == True)[0]
                if len(neighbor_idx) == 0:
                    continue
                others_idx = set(neighbor_idx) - set(temp_not_assign_cluster_idx)-set([nci])-set(assign_cluster_idx)
                if len(others_idx) == 0:
                    continue
                others_idx = np.array(list(others_idx))
                closest_neighbor_idx = self.get_closest(nci, others_idx, X, self.eps)
                labels[nci] = labels[closest_neighbor_idx]
            not_assign_cluster_idx = np.where(labels == -1)[0]
            if len(set(core_indice).intersection(not_assign_cluster_idx)) == 0:
                break
            core_indice = np.array(list(set(core_indice).intersection(not_assign_cluster_idx)))
            non_core_idx = np.array(list(set(non_core_idx).intersection(not_assign_cluster_idx)))
            init_core_idx = core_indice[0] ## initial core point
            prev_cluster_list.append(cluster_label)
            cluster_label += 1
            visited_idx = np.where(labels != -1)[0].tolist()
        self.labels = labels
        return self
        



class Rotary():   #Lidar 데이터를 받아서, 처리하는 클래스 작성
    def __init__(self): #생성자 메서드 작성
        # rospy.loginfo("Lidar Receiver object is Created")
        rospy.Subscriber("Ego_topic",EgoVehicleStatus,self.ego_callback)
        rospy.Subscriber("/lidar2D",LaserScan,self.lidar_callback)  #Subscriber 필수입력 3가지  ----> 섭스크라이브하는 토픽이름,메시지 타입,동장 콜백함수  
        
        self.in_range = 0
        self.MYpoint = [0,0]
        self.heading = 0
        self.RC_speed_pub = rospy.Publisher('/control/speed/RC', Float64, queue_size=1)
        pass
    
    def ego_callback(self,_data):
        self.MYpoint = [_data.position.x,_data.position.y]
        self.heading = _data.heading
    def lidar_callback(self,_data):    #데이터를 받고 처리하는 부분
        if (10.8< self.MYpoint[0]<14.8 ) and  (-2.6<self.MYpoint[1]<2.6):   #로터리 절대좌표 설정
            self.data = _data.ranges
            self.in_range = []
            ROI = list(range(90))+list(range(270,360))
            for i in ROI:
                if _data.ranges[i] < 2.5:
                    x = _data.ranges[i]*np.cos((i+self.heading)*np.pi/180)+self.MYpoint[0]  #절대좌표 변환
                
                    y = _data.ranges[i]*np.sin((i+self.heading)*np.pi/180)+self.MYpoint[1]
                    if ((-20.5< x <20.5) and (-6.5< y <6.5)):
                        self.in_range.append([round(x,1),round(y,1)])
                
        
            np_in_range = np.array(self.in_range)
        
            if len(self.in_range)>3:
                Dbscan = myDBSCAN()
                Dbscan.fit(np_in_range)   #Dbscan.labels에 인덱스에 해당하는 클래스 할당
                data = zip(Dbscan.labels,self.in_range) 
                labels_dic = {}
                for k in range(int(np.max(Dbscan.labels)+1)):
                    index = np.where(Dbscan.labels == k)[0]
                    index = index.tolist()
                    a = np.empty((0,2),float)
                    for i in index:
                        a = np.append(a,np.array([self.in_range[i]]),axis = 0)
                    labels_dic[k] = np.round(np.mean(a,axis = 0),2)  #장애물 좌표 평균
                
                
                if np.max(Dbscan.labels)==0:   #장애물 1개인 경우
                    if 11.6<labels_dic[0][0]<13.4 and  0.72<labels_dic[0][1]:
                        self.RC_speed_pub.publish(0.0)
                        
                    else:
                        self.RC_speed_pub.publish(0.7)
                        if ((labels_dic[0][0]-self.MYpoint[0])**2+(labels_dic[0][1]-self.MYpoint[1])**2)**0.5 < 1.2:
                            self.RC_speed_pub.publish(0.12)
                            if((13.3<self.MYpoint[0]) and (self.MYpoint[1]<1.1)):   #탈출 가속구간
                                self.RC_speed_pub.publish(1)
                    
                
                if np.max(Dbscan.labels)==1:
                    if (11.6<labels_dic[0][0]<13.4 and  0.72<labels_dic[0][1]) or (11.6<labels_dic[1][0]<13.4 and  0.72<labels_dic[1][1]):
                        self.RC_speed_pub.publish(0.0)
                        
                    else:
                        self.RC_speed_pub.publish(0.7)
                        if ((labels_dic[0][0]-self.MYpoint[0])**2+(labels_dic[0][1]-self.MYpoint[1])**2)**0.5 < 1.2 or \
                            ((labels_dic[1][0]-self.MYpoint[0])**2+(labels_dic[1][1]-self.MYpoint[1])**2)**0.5 < 1.2:
                            self.RC_speed_pub.publish(0.12)  
                            if((13.3<self.MYpoint[0]) and (self.MYpoint[1]<1.1)):
                                self.RC_speed_pub.publish(1)
                 
                if np.max(Dbscan.labels)==2:  #정지할때 3개 잡힐 수도 있어서 넣었음
                    self.RC_speed_pub.publish(0.0)
            
            
            else:   #장애물 감지 안되는경우
                self.RC_speed_pub.publish(0.9)
                rospy.loginfo("noobject")
        else:
            self.RC_speed_pub.publish(1.0)
    
        

def run(): #실제 코드 동작시, 실행할 부분
    rospy.init_node("lidar_example")  #이 이름으로 노드를 생성할거니 ros mater에 등록하는 부분  중복만 하지 않으면 가능
    new_class = Rotary()  #실제 동작을 담당할 객체
    rospy.spin()  #반복 
    
if __name__ == "__main__":
    run()
