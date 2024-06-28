import rospy
import rosbag
import numpy as np
import os
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField


# 新的主題用來發布處理後的點雲
pub = rospy.Publisher('/filtered_points', PointCloud2, queue_size=10)

######### publish pointcloud2 Start ######### 
def publish_data(data):

    msg = PointCloud2()
    msg.header.stamp = rospy.Time().now()
    msg.header.frame_id = "filterd"

    # if len(data.shape) == 3:
    #     msg.height = data.shape[1]
    #     msg.width = data.shape[0]
    # else:
    #     msg.height = 1
    #     msg.width = len(data)

    msg.height = 1
    msg.width = len(data)

    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * 1
    msg.is_dense = False
    msg.data = np.asarray(data, np.float32).tostring()

    pub.publish(msg)
    print("published...")

######### publish pointcloud2 End ######### 


def process_lidar_data(data):
    
    # assert isinstance(data, PointCloud2)
    gen = point_cloud2.read_points(data, field_names=("x", "y", "z", "intensity"), skip_nans=True)
    
    points = np.zeros(shape=(1,4))
    for p in gen:

        ######### Filter Lidar Points #########

        # if p[0] == 0 and p[1] == 0 and p[2] == 0:
        #     continue
        # elif p[2] < 0 or p[2] > 0.5:
        #     continue
        # elif p[0] > 10:
        #     continue
        # elif p[0] > 7.18 and p[0] < 14.36:
        #     long_side  = 14.36 - p[0]
        #     short_side = math.tan(19.2) * long_side * -1
        #     if p[1] < short_side:
        #         continue
        # elif p[0] >= 14.36:
        #     long_side  = p[0] - 14.36
        #     short_side = math.tan(19.2) * long_side
        #     if p[1] <= short_side:
        #         continue

        points = np.append(points, [[p[0], p[1], p[2], p[3]]], axis=0)
        # print("x: %.3f, y: %.3f, z: %.3f, intensity: %d"%(p[0], p[1], p[2], p[3]))

    publish_data(points)

def read_from_bag():

    # bag_file = "./2023-12-08-15-02-03.bag"
    bagpath = './'
    for files in os.listdir(bagpath):
        if files[-4:] == ".bag":
            bag_file = os.path.join(bagpath, files)

            bag = rosbag.Bag(bag_file, 'r')

            for topic, msg, t in bag.read_messages(topics=["/rslidar_points"]):
                process_lidar_data(msg)
            
            print("Finish reading from bag")

def listen_from_topic():
    # 訂閱 ROS bag 中的 LiDAR 點雲主題
    rospy.Subscriber('/rslidar_points', PointCloud2, process_lidar_data)

    # ROS While
    rospy.spin()

def test(data):
    gen = point_cloud2.read_points(data, field_names=("x", "y", "z", "intensity"), skip_nans=True)
    publish_data(gen)

if __name__ == '__main__':
    rospy.init_node('lidar_processing_node')

    # listen_from_topic()
    read_from_bag()
