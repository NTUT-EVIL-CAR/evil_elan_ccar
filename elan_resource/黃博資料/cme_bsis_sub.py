#!/usr/bin/env python3
import os
import cv2
import rospy
import math
import numpy as np

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from cv_bridge import CvBridge
#from sklearn import datasets
#from sklearn.cluster import DBSCAN
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

bridge = CvBridge()
count = 0
pub = rospy.Publisher('pointcloud_topic', PointCloud2, queue_size=5)


######### DBSCAN Start ######### 
class DB_Point:
    x = np.zeros(1)
    y = np.zeros(1)
    z = np.zeros(1)
    pts = np.ones(1, dtype=int)
    cluster = np.zeros(1, dtype=int)
    visited = np.zeros(1, dtype=int)
    corepts = []
    
    def __init__(self, x, y, z, c):
        self.x = x
        self.y = y
        self.z = z
        self.cluster = c
      
DBSCAN_point = []
corePoint    = []

def squarDistance(dataA, dataB):
    return math.sqrt((dataA.x - dataB.x) * (dataA.x - dataB.x) + (dataA.y - dataB.y) * (dataA.y - dataB.y) + (dataA.z - dataB.z) * (dataA.z - dataB.z))

def cluster_DBSCAN(num, db_data, EPS=0.3, MinPts=10):
    
    # calculate pts
    for i in range(num):
        for j in range(i+1, num):
            if squarDistance(db_data[i], db_data[j]) < EPS:
                db_data[i].pts = db_data[i].pts + 1
                db_data[j].pts = db_data[j].pts + 1
    
    # core point
    for i in range(num):
        if db_data[i].pts >= MinPts:
            corePoint.append(db_data[i])
    
    # joint point
    # nn = np.zeros(len(corePoint), dtype=int)
    for i in range(0, len(corePoint)):
        for j in range(i+1, len(corePoint)):
            if squarDistance(corePoint[i], corePoint[j]) < EPS:
                corePoint[i].corepts = np.append(corePoint[i].corepts, j)
                corePoint[j].corepts = np.append(corePoint[j].corepts, i)
                # nn[i] = nn[i] + 1
                # nn[j] = nn[j] + 1
        # print(corePoint[i].corepts)
    
    for i in range(0, len(corePoint)):
        psv = np.zeros(len(corePoint[i].corepts), dtype=int)
        for j in range(0, len(corePoint[i].corepts)):
            psv[j] = corePoint[i].corepts[j]
        # print(psv)
        
        if len(psv) >= 1:
            for j in range(0, len(psv)):
                hh = 0
                pss = np.zeros(len(corePoint[psv[j]].corepts), dtype=int)
                for k in range(0, len(pss)):
                    pss[k] = corePoint[psv[j]].corepts[k]
                
                while hh < len(corePoint[psv[j]].corepts):
                    corePoint[pss[hh]].visited = 1
                    if pss[hh] < i:
                        for sc in range(0, len(corePoint)):
                            if corePoint[i].cluster == corePoint[sc].cluster:
                                corePoint[sc].cluster = corePoint[pss[hh]].cluster
                    else:
                        corePoint[pss[hh]].cluster = corePoint[i].cluster
                    hh = hh + 1
            
                corePoint[psv[j]].visited = 1
                corePoint[psv[j]].cluster = corePoint[i].cluster
    
    db_data = []
    
    sss = len(corePoint)
    c = 0; i = 0; clust = 0
    
    while sss != 0:
        ss = 0; ssss = 0
        
        for j in range(0, sss):
            if clust == corePoint[j].cluster:
                db_data.append(corePoint[j])
                c = c + 1
                ssss = ssss + 1
            else:
                corePoint[ss] = corePoint[j]
                ss = ss + 1
        
        if ssss < MinPts and ssss > 0:
            for j in range(c-1, c-1-ssss, -1):
                db_data.remove(j)
            c = c - ssss
        
        sss = ss
        clust = clust + 1
        i = i + 1
    
    for i in range(len(db_data)):
        c = db_data[i].cluster
        print(c)
            
                
                
    
######### DBSCAN End ######### 

######### publish pointcloud2 Start ######### 
def publish_data(data):

    msg = PointCloud2()
    msg.header.stamp = rospy.Time().now()
    msg.header.frame_id = "livox_frame"

    if len(data.shape) == 3:
        msg.height = data.shape[1]
        msg.width = data.shape[0]
    else:
        msg.height = 1
        msg.width = len(data)

    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * data.shape[0]
    msg.is_dense = False
    msg.data = np.asarray(data, np.float32).tostring()

    pub.publish(msg)
    print("published...")

######### publish pointcloud2 End ######### 


def process_lidar_data(data):

    DBSCAN_point = [] 
    f = open('output.txt','w')
    
    assert isinstance(data, PointCloud2)
    gen = point_cloud2.read_points(data, field_names=("x", "y", "z", "intensity"), skip_nans=True)
    
    points = np.zeros(shape=(1,4))
    for p in gen:
        if len(DBSCAN_point) > 30: 
            break
        if p[0] == 0 and p[1] == 0 and p[2] == 0:
            continue
        elif p[2] < 0 or p[2] > 0.5:
            continue
        elif p[0] > 10:
            continue
        elif p[0] > 7.18 and p[0] < 14.36:
            long_side  = 14.36 - p[0]
            short_side = math.tan(19.2) * long_side * -1
            if p[1] < short_side:
                continue
        elif p[0] >= 14.36:
            long_side  = p[0] - 14.36
            short_side = math.tan(19.2) * long_side
            if p[1] <= short_side:
                continue

        #DBSCAN_point.append(DB_Point(p[0], p[1], p[2], len(DBSCAN_point)))
        points = np.append(points, [[p[0], p[1], p[2], p[3]]], 0)        
        print("x: %.3f, y: %.3f, z: %.3f, intensity: %d"%(p[0], p[1], p[2], p[3]), file=f)
    f.close()
    print("save lidar data file")
    
    #cluster_DBSCAN(len(DBSCAN_point), DBSCAN_point)
    publish_data(points)


def callback_cam(data):
    rospy.loginfo('CAM' + rospy.get_caller_id())
    
    frame = bridge.imgmsg_to_cv2(data, 'bgr8')
    cv2.imshow("SUB_FRAME", frame)
    cv2.waitKey(1)
    
def callback_lidar(data):
    global count
    count = count + 1
    rospy.loginfo('Lidar' + rospy.get_caller_id())
    assert isinstance(data, PointCloud2)
    gen = point_cloud2.read_points(data, field_names=("x", "y", "z", "intensity"), skip_nans=True)
    process_lidar_data(data)
    print(gen)
    if count == 10:
        process_lidar_data(data)
    print(count)
    
def callback_convert(data):
    rospy.loginfo('CONVERT' + rospy.get_caller_id()) 
    
def listener():
    rospy.init_node('cme_listen_node', anonymous=True)
    
    #sub_cam = rospy.Subscriber('cme_cam', Image, callback_cam)
    sub_lidar = rospy.Subscriber('/rslidar_points', PointCloud2, callback_lidar)
    #sub_convert = rospy.Subscriber('cme_convert', MarkerArray, callback_convert)
    
    rospy.spin()
    
if __name__ == '__main__':
    
    listener()