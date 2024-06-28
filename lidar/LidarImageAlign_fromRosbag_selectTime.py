import rosbag 
import os
from sensor_msgs import point_cloud2

import numpy as np 
import matplotlib.pyplot as plt
import cv2
from cv_bridge import CvBridge

extri = np.array([[1.000e+00, 0.000e+00, 0.000e+00, 0], # left right
                  [0.000e+00, 9.9778e-01, -8.653e-02, -0.28], # up down
                  [0.000e+00, 8.653e-02, 9.9778e-01, -0.865], # front back
                  [0, 0, 0, 1]])

intrist = np.array([[1.418667e+03, 0.000e+00, 6.4e+02],
                    [0.000e+00, 1.418667e+03, 3.6e+02],
                    [0.000e+00, 0.000e+00, 1.0e+00]])

dirs = "./"

for files in os.listdir(dirs):
    if files[-4:] == ".bag":
        bag_file = os.path.join(dirs,files)

        with rosbag.Bag(bag_file, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=["/cme_cam", "/velodyne_points"]):

                timestr = f"{msg.header.stamp.to_sec():.6f}"

                # 1713927356.444217 1713927356.554039 1713927362.703352 1713927362.813168 
                if timestr == "1713927356.444217":
                    
                    gen = point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
                    points = np.array(list(gen))

                    x = points[:, 0]
                    y = points[:, 1]
                    z = points[:, 2]
                    intensity = points[:, 3]
                # 1713927356.433500 1713927356.466761 1713927356.800221 1713927362.566759 1713927362.799900

                if timestr == "1713927356.800221":
                    
                    img = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # 點雲過濾
        valid_indices = (x >= 0)
        filtered_x = x[valid_indices]
        filtered_y = y[valid_indices]
        filtered_z = z[valid_indices]

        world_points = np.column_stack((-filtered_y, -filtered_z, filtered_x, np.ones_like(filtered_x)))
        camera_points = np.dot(extri, world_points.T).T
        normalized_camera_points = camera_points[:, :3] / camera_points[:, 2, np.newaxis]
        image_points = np.dot(intrist, normalized_camera_points.T)[:2]

        # print(world_points)
        # print(camera_points)
        # print(normalized_camera_points)
        # print(image_points)

        # 畫圖
        fig, ax = plt.subplots()

        ax.imshow(img)

        xoffset, yoffset = 60, -6
        scatter = ax.scatter(x=(image_points[0]+xoffset), y=(image_points[1]+yoffset), c=filtered_x, cmap="jet", alpha=1, s=1)

        fig.colorbar(scatter)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
                    
        plt.show()