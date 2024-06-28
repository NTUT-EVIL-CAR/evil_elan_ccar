import os, time
import ctypes
import sys
import rospy

import vlc
import cv2
import numpy

from PIL import Image

from cv_bridge import CvBridge
from sensor_msgs.msg import Image as ROI_Image

rospy.init_node('cme_node', anonymous=True)
cme_pub = rospy.Publisher('cme_cam', ROI_Image, queue_size=0)
bridge = CvBridge()

if __name__ == '__main__':
    
    video_width = 1280#1280
    video_height = 720#720

    size = video_width * video_height * 4
    buf = (ctypes.c_ubyte * size)()
    buf_p = ctypes.cast(buf, ctypes.c_void_p)
    VideoLockCb = ctypes.CFUNCTYPE(ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_void_p))

    @VideoLockCb
    def _lockcb(opaque, planes):
        planes[0] = buf_p

    @vlc.CallbackDecorators.VideoDisplayCb
    def _display(opaque, picture):
        img = Image.frombuffer("RGBA", (video_width, video_height), buf, "raw", "BGRA", 0, 1)
        opencv_image = cv2.cvtColor(numpy.array(img), cv2.COLOR_RGB2BGR)
        ros_image = bridge.cv2_to_imgmsg(opencv_image, 'bgr8')
        ros_image.header.stamp = rospy.Time.now() #get_time()
        print(ros_image.header.stamp)
        cme_pub.publish(ros_image)

        cv2.putText(opencv_image, str(time.time()), (10, 30), 1, 1, (128, 255, 255))
        cv2.imshow("VIDEO", opencv_image)
        cv2.waitKey(10)

    instance = vlc.Instance() #"--network-caching=0"
    media = instance.media_new("v4l2:///dev/video0") #'rtsp://192.168.0.4/liveRTSP/av4', 'rtsp://192.168.0.33/ch01/0', "v4l2:///dev/video0"
    mediaplayer = vlc.libvlc_media_player_new_from_media(media)
    vlc.libvlc_video_set_callbacks(mediaplayer, _lockcb, None, _display, None)
    mediaplayer.video_set_format("BGRA" ,video_width, video_height, video_width*4)

    while True:
        mediaplayer.play()
        print("-----------------------------------------------------")
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
        # print(mediaplayer.get_fps())
        time.sleep(1)