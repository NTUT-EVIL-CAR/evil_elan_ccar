import numpy as np
import os

import open3d as o3d
import copy
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg

root_dir = os.getcwd() # os.path.dirname(os.path.realpath(__file__))
ws_dir = os.path.join(root_dir, "citystreet_sunny_day_2024-06-12-09-19-23")
lidar_dir = os.path.join(ws_dir, "VLS128_pcdnpy")

def RT_matrix(radian=[0, 0, 0], translation=[0, 0, 0]):
    theta_x = np.radians(radian[0])
    theta_y = np.radians(radian[1])
    theta_z = np.radians(radian[2])
    rotation_x = np.array([
        [np.cos(theta_x), -np.sin(theta_x), 0],
        [np.sin(theta_x), np.cos(theta_x), 0],
        [0, 0, 1]
    ])
    rotation_y = np.array([
        [1, 0, 0],
        [0, np.cos(theta_y), -np.sin(theta_y)],
        [0, np.sin(theta_y), np.cos(theta_y)]
    ])
    rotation_z = np.array([
        [np.cos(theta_z), 0, np.sin(theta_z)],
        [0, 1, 0],
        [-np.sin(theta_z), 0, np.cos(theta_z)]
    ])
    rotation_matrix = rotation_x @ rotation_y @ rotation_z
    translation_vector = np.array(translation).reshape(3, 1)

    return rotation_matrix, translation_vector

def transform_points(points, rotation_matrix, translation_vector):
    transformed_points = points @ rotation_matrix.T + translation_vector.T

    return transformed_points

def publish_point_cloud():
    rospy.init_node("point_cloud_publisher", anonymous=True)
    pub = rospy.Publisher("/point_cloud", PointCloud2, queue_size=10)
    rate = rospy.Rate(1)

    points, intensities = merge_point()
    cloud_data = np.column_stack((points, intensities))

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "lidar_map"
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
        PointField('ring', 16, PointField.FLOAT32, 1)
    ]

    pc2_msg = pc2.create_cloud(header, fields, cloud_data)

    while not rospy.is_shutdown():
        pub.publish(pc2_msg)
        rate.sleep()

def merge_point():
    pcd_files = sorted([
        f for f in os.listdir(lidar_dir) if f.endswith(".pcd")
    ])
    npy_files = sorted([
        f for f in os.listdir(lidar_dir) if f.endswith("npy")
    ])
    extrinsics = [
        RT_matrix([0, 0, 0], [i * 1.6, 0, 0]) for i in range(len(pcd_files))
    ]

    combined_points = None
    combined_intensities = None

    for pcd_file, npy_file, extrinsic in zip(pcd_files, npy_files, extrinsics):
        pcd = o3d.io.read_point_cloud(os.path.join(lidar_dir, pcd_file))
        points = np.asarray(pcd.points)
        rotation_matrix, translation_vector = extrinsic
        transformed_points = transform_points(points, rotation_matrix, translation_vector)

        intensities = np.load(os.path.join(lidar_dir, npy_file))

        if combined_points is None:
            combined_points = transformed_points
            combined_intensities = intensities
        else:
            combined_points = np.concatenate((combined_points, transformed_points))
            combined_intensities = np.concatenate((combined_intensities, intensities))

    return combined_points, combined_intensities

def main():
    try:
        print("Publishing ...")
        publish_point_cloud()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()