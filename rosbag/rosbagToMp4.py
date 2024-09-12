import os
import cv2
import sys
import rosbag
from cv_bridge import CvBridge
from tqdm import tqdm

def images_to_video(bag, video_name, fps):
    bridge = CvBridge()
    video = None
    messages = bag.read_messages(topics=["/cme_cam"])
    num_images = bag.get_message_count("/cme_cam")
    
    with tqdm(total=num_images, desc="Making video", ascii=True) as pbar:
        for topic, msg, t in messages:
            try:
                img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

                if video is None:
                    height, width, layers = img.shape
                    video = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))

                video.write(img)
            except Exception as e:
                print(f"Error converting ROS image message: {e}")
            
            pbar.update(1)

    if video is not None:
        video.release()
        cv2.destroyAllWindows()
    else:
        print("No images were processed from the ROS bag.")

def parse_bags(fps=30):
    bag_file = sys.argv[1]
    bag_name = f"{bag_file}.mp4"

    try:
        with rosbag.Bag(bag_file, 'r') as bag:
            images_to_video(bag, bag_name, fps)
    except rosbag.bag.ROSBagException as e:
        print(f"Error opening ROS Bag file: {e}")

if __name__ == "__main__":
    parse_bags()
