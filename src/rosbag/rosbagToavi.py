import cv2
import sys
import numpy as np
import os
import rosbag

from cv_bridge import CvBridge
bridge = CvBridge()

sub_name = ""

# SETTINGS FOR SAVING OUTPUT VIDEO
bag_file = sys.argv[1]
out_file = os.path.splitext(bag_file)[0] + sub_name + '.avi'

# Filepath to save the video as
fourcc_settings = cv2.VideoWriter_fourcc('M','J','P','G')

out_vid_dims = (1280, 720)
fps = 30    # adjust based on input video

out = cv2.VideoWriter(out_file,
                      fourcc=fourcc_settings,
                      fps=fps,
                      frameSize=out_vid_dims,
                      isColor= True)

# ROSBAG SETTINGS
# bag_file_dir = "/home/data-record/"
# bag_file_name = sys.argv[1]
# bag_file = os.path.join(bag_file_dir, bag_file_name)

# OPEN BAG
bag = rosbag.Bag(bag_file, "r")

messages = bag.read_messages(topics=["/cme_cam"])  # Only get images data
num_images = bag.get_message_count(topic_filters=["/cme_cam"])  #/camera_0/rgb/image_raw

print('num_images:', num_images)
for i in range(num_images):
    # READ NEXT MESSAGE IN BAG
    topic, msg, t = messages.__next__()
    print(msg.header.stamp.to_sec(), t.to_sec())
    # CONVERT MESSAGE TO A NUMPY ARRAY
    # img = np.fromstring(msg.data, dtype=np.uint8)
    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    img = img.reshape(msg.height, msg.width,-1)
    
    timeq = msg.header.stamp.to_sec() 
    timestr = "{:.6f}".format(timeq)
    cv2.putText(img, timestr, (800, 40), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 0, 0), 5, cv2.LINE_AA)
    out.write(img)

    # RESIZE TO OUTPUT DIMENNSIONS
    img = cv2.resize(img, out_vid_dims)

    # QUIT IF ESCAPE BUTTON PRESSED
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

out.release()
cv2.destroyAllWindows()

print("DONE!")

