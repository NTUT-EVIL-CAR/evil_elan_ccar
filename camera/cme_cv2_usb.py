import time
import rospy
import cv2
import argparse

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as ROI_Image

class image_converter:

    def __init__(self):
        self.cme_pub = rospy.Publisher('cme_cam', ROI_Image, queue_size=0)
        self.bridge = CvBridge()
    
    def publisher(self, opencv_image):
        ros_image = self.bridge.cv2_to_imgmsg(opencv_image, "bgr8")
        ros_image.header.stamp = rospy.Time.now()

        try:
            self.cme_pub.publish(ros_image)
        except CvBridgeError as e:
            print(e)
        
        cv2.putText(opencv_image, str(time.time()), (10, 30), 1, 1, (128, 255, 255))
        cv2.imshow("VIDEO", opencv_image)

def main():
    parser = argparse.ArgumentParser(description='ROS Camera Node')
    parser.add_argument('--camera_device', type=str, default='/dev/video2', help='Camera device path')
    args = parser.parse_args()

    rospy.init_node('cme_node', anonymous=True)
    ic = image_converter()
    video_width = 1280
    video_height = 720

    # cap = cv2.VideoCapture(args.camera_device)
    cap = cv2.VideoCapture(args.camera_device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, video_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, video_height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    print(f"captureOpen: {cap.isOpened()}")
    print(f"Requested resolution: {video_width}x{video_height}")

    # 測試
    actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Actual resolution: {actual_width}x{actual_height}")
    print(f"Actual FPS: {actual_fps}")

    while cap.isOpened():
        ret, image = cap.read()
        if not ret:
            print("Error reading video frame.")
            break
        ic.publisher(image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()