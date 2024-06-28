#!/usr/bin/env python3
import os
import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Imu

angle_x = []
angle_y = []
angle_z = []
accel_x = []
accel_y = []
accel_z = []


def callback_imu(data):
#	print(data)
	frame = np.zeros((256, 256, 3), np.uint8)
	frame2 = np.zeros((256, 256, 3), np.uint8)	
	
	print('Angular: %f, %f, %f' % (data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z))
	print('Acceleration: %f, %f, %f' % (data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z))	
	
	angle_x.append(data.angular_velocity.x)
	angle_y.append(data.angular_velocity.y)
	angle_z.append(data.angular_velocity.z)	
	
	accel_x.append(data.linear_acceleration.x)
	accel_y.append(data.linear_acceleration.y)
	accel_z.append(data.linear_acceleration.z)
	
	if len(angle_x) > 256:
		angle_x.pop(0)
		angle_y.pop(0)
		angle_z.pop(0)
		accel_x.pop(0)
		accel_y.pop(0)
		accel_z.pop(0)	
		
	for i in range(len(angle_x)-1):
		cv2.line(frame, (i, 128+int(angle_x[i]/np.pi*128)), (i+1, 128+int(angle_x[i+1]/np.pi*128)), (255, 0, 0))
		cv2.line(frame, (i, 128+int(angle_y[i]/np.pi*128)), (i+1, 128+int(angle_y[i+1]/np.pi*128)), (0, 255, 0))
		cv2.line(frame, (i, 128+int(angle_z[i]/np.pi*128)), (i+1, 128+int(angle_z[i+1]/np.pi*128)), (0, 0, 255))
		
		cv2.line(frame2, (i, 128+int(accel_x[i]*128)), (i+1, 128+int(accel_x[i+1]*128)), (255, 0, 0))
		cv2.line(frame2, (i, 128+int(accel_y[i]*128)), (i+1, 128+int(accel_y[i+1]*128)), (0, 255, 0))
		cv2.line(frame2, (i, 128+int(accel_z[i]*128)), (i+1, 128+int(accel_z[i+1]*128)), (0, 0, 255))
		
	
	cv2.imshow("Angular", frame)
	cv2.imshow("Acceleration", frame2)
	cv2.waitKey(1)



def listener():
	rospy.init_node('livox_lidar_publisher', anonymous=True)
	rospy.Subscriber('/imu', Imu, callback_imu, queue_size=1)
	
	rospy.spin()	
	
	
	
if __name__ == '__main__':
	listener()
