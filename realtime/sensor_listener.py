import rospy
import numpy as np
import os
import cv2
import tf
from radarParser import prettyhex2, Radar_Header_505, Radar_VehInfo_300, Radar_Sta_506, Radar_Target_A_508, Radar_Target_B_509
from cv_bridge import CvBridge, CvBridgeError
from can_msgs.msg import Frame
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, NavSatFix, Imu, PointCloud2, CameraInfo
from rosgraph_msgs.msg import Clock, Log

csv_directory = None
radar_data = None
No_Obj = 0

def create_csv():
    global csv_directory
    current_directory = os.getcwd()
    csv_directory = os.path.join(current_directory, "csv")
    if not os.path.exists(csv_directory):
        os.makedirs(csv_directory)
    with open(os.path.join(csv_directory, "radar_data.csv"), mode='w') as file:
        file.write("Timestamp,AEB_CIPVFlag,ACC_CIPVFlag,CIPVFlag,Vel_Y,Vel_X,Pos_Y,Pos_X,ID,Type,ProbExist,DynProp,MeasStat,Accel_X\n")
    with open(os.path.join(csv_directory, "navsatfix_data.csv"), mode='w') as file:
        file.write("Timestamp,Latitude,Longitude,Altitude,Covariance_0,Covariance_1,Covariance_2,Covariance_3,Covariance_4,Covariance_5,Covariance_6,Covariance_7,Covariance_8\n")
    with open(os.path.join(csv_directory, "imu_data.csv"), mode='w') as file:
        file.write("Timestamp,Orientation_x,Orientation_y,Orientation_z,Orientation_w,Angular_velocity_x,Angular_velocity_y,Angular_velocity_z,Linear_acceleration_x,Linear_acceleration_y,Linear_acceleration_z\n")
    with open(os.path.join(csv_directory, "lidar_data.csv"), mode='w') as file:
        file.write("Timestamp,x,y,z,intensity,ring\n")

def publish_image(img):
    image_pub = rospy.Publisher("/processed_image", Image, queue_size=10)
    
    try:
        image_message = CvBridge().cv2_to_imgmsg(img, encoding="passthrough")
        image_message.header.frame_id= "camera_link"

        image_pub.publish(image_message)
    except CvBridgeError as e:
        print(e)

def publish_camera_info():
    pub = rospy.Publisher("/camera_info", CameraInfo, queue_size=10)

    camera_info_msg = CameraInfo()
    camera_info_msg.width = 1280
    camera_info_msg.height = 720
    camera_info_msg.header.frame_id = "camera_link"
    camera_info_msg.K = [
        1.418667e+03, 0.000e+00, 6.4e+02,
        0.000e+00, 1.418667e+03, 3.6e+02,
        0.000e+00, 0.000e+00, 1.0e+00
    ]
    camera_info_msg.P = [
        1.418667e+03, 0.000e+00, 6.4e+02, 0.0e+00,
        0.000e+00, 1.418667e+03, 3.6e+02, 0.0e+00,
        0.000e+00, 0.000e+00, 1.0e+00, 0.0e+00
    ]

    pub.publish(camera_info_msg)

def broadcast_static_tf():
    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "camera_link", "world")

def get_ros_timestamp(data):
    timestamp = data.header.stamp.to_sec()
    return timestamp

def can_callback(data, topic_name):
    # rospy.loginfo(f"Recived data on {topic_name}: {data}")
    global radar_data, No_Obj
    timestamp = get_ros_timestamp(data)
    data_str = prettyhex2(data.dlc, data.data, '-').split('-')

    if (topic_name == "/can0/received_msg"):
        ### Radar IMU ###
        if (data.id == 0x300):
            radar_veh_info = np.array(Radar_VehInfo_300(data_str))
        ### Radar ###
        elif (0x505 <= data.id <= 0x547):
            rospy.loginfo(f"Received Radar data {data.id}")
            if (data.id == 0x505):
                No_Obj = Radar_Header_505(data_str)[5]
                radar_data = np.array([timestamp] + list(Radar_Header_505(data_str)))
            elif (data.id == 0x506):
                Radar_Sta_506(data_str)
            if (No_Obj > 0):
                if (radar_data is not None):
                    if (0x508 <= data.id <= 0x546) and (data.id % 2 == 0):
                        radar_data = np.concatenate((radar_data, Radar_Target_A_508(data_str)[0:8]))
                    elif (0x509 <= data.id <= 0x547) and (data.id % 2 == 1):
                        radar_data = np.concatenate((radar_data, Radar_Target_B_509(data_str)[0:5]))
                    No_Obj -= 1
                
                ### Radar Data ###
                if (No_Obj == 0):
                    file_path = os.path.join(csv_directory, "radar_data.csv")
                    with open(file_path, 'a') as file:
                        np.savetxt(file, [radar_data], delimiter=',', fmt='%.8f')

def image_callback(data):
    rospy.loginfo("Received Image data")
    try:
        publish_camera_info()
        img = CvBridge().imgmsg_to_cv2(data, desired_encoding="passthrough")
        publish_image(img)
    except CvBridgeError as e:
        print(e)

def fix_callback(data):
    global csv_directory
    rospy.loginfo("Received NavSatFix data")

    timestamp = get_ros_timestamp(data)
    latitude = data.latitude
    longitude = data.longitude
    altitude = data.altitude
    position_covariance = data.position_covariance
    navsatfix_data = np.array([latitude, longitude, altitude] + list(position_covariance))

    file_path = os.path.join(csv_directory, "navsatfix_data.csv")
    with open(file_path, 'a') as file:
        np.savetxt(file, [[timestamp] + list(navsatfix_data)], delimiter=',', fmt='%.8f')

def imu_callback(data):
    rospy.loginfo("Received IMU data")

    timestamp = get_ros_timestamp(data)
    orientation = [
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w
    ]
    angular_velocity = [
        data.angular_velocity.x,
        data.angular_velocity.y,
        data.angular_velocity.z
    ]
    linear_acceleration = [
        data.linear_acceleration.x,
        data.linear_acceleration.y,
        data.linear_acceleration.z
    ]
    imu_data = np.array(orientation + angular_velocity + linear_acceleration)

    file_path = os.path.join(csv_directory, "imu_data.csv")
    with open(file_path, 'a') as file:
        np.savetxt(file, [[timestamp] + list(imu_data)], delimiter=',', fmt='%.8f')

def pointcloud_callback(data):
    rospy.loginfo("Received PointCloud2 data")

    timestamp = get_ros_timestamp(data)
    gen = point_cloud2.read_points(data, field_names=("x", "y", "z", "intensity", "ring"), skip_nans=True)
    lidar_data = np.array(list(gen))

    file_path = os.path.join(csv_directory, "lidar_data.csv")
    with open(file_path, 'a') as file:
        np.savetxt(file, np.hstack((np.full((lidar_data.shape[0], 1), timestamp), lidar_data)), delimiter=',', fmt='%.8f')

def listener():
    rospy.init_node("listener", anonymous=True)

    rospy.Subscriber("/can0/received_msg", Frame, can_callback, callback_args="/can0/received_msg")
    # rospy.Subscriber("/can1/received_msg", Frame, can_callback, callback_args="/can1/received_msg")
    # rospy.Subscriber("/can2/received_msg", Frame, can_callback, callback_args="/can2/received_msg")
    rospy.Subscriber("/cme_cam", Image, image_callback, queue_size=10)
    rospy.Subscriber("/fix", NavSatFix, fix_callback)
    rospy.Subscriber("/imu", Imu, imu_callback)
    rospy.Subscriber("/velodyne_points", PointCloud2, pointcloud_callback)

    rospy.Timer(rospy.Duration(1.0), lambda event: broadcast_static_tf())
    rospy.spin()

if __name__ == "__main__":
    # 顯示完整浮點數精度
    np.set_printoptions(precision=8, suppress=True)

    create_csv()
    try:
        listener()
    except KeyboardInterrupt:
        print("Shuting down")
    finally:
        cv2.destroyAllWindows()