import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo


class SensorListener:
    def __init__(self):
        self.camera_info_pub = rospy.Publisher("/camera_info", CameraInfo, queue_size=10)

    def publish_camera_info(self, msg):
        camera_info_msg = CameraInfo()
        camera_info_msg.width = 1280
        camera_info_msg.height = 720
        camera_info_msg.header.stamp = msg.header.stamp
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
        camera_info_msg.distortion_model = "plumb_bob"
        camera_info_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_pub.publish(camera_info_msg)

    def image_callback(self, data):
        rospy.loginfo("Received Image data")

        try:
            self.publish_camera_info(data)
        except CvBridgeError as e:
            print(e)

    def listener(self):
        rospy.init_node("listener", anonymous=True)
        rospy.Subscriber("/cme_cam", Image, self.image_callback, queue_size=10)
        rospy.spin()


if __name__ == "__main__":
    publisher = SensorListener()
    publisher.listener()