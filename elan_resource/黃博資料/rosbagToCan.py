import cv2
import sys
import numpy as np
import os
import rosbag

def prettyhex(nums, sep=''):
    return sep.join(f'{a:02x}' for a in nums)
    
def prettyhex2(dlc,nums, sep=''):
#    return sep.join(f'{a:02x}' for a in nums)
    return sep.join(f'{a:02x}' for a in nums[0:dlc])

# SETTINGS FOR SAVING OUTPUT VIDEO
# dirs = sys.argv[1]
dirs = '/data2/G5Q4_bag/20230918'

for files in os.listdir(dirs):
    if files[-4:]=='.bag':
        bag_file = os.path.join(dirs,files)
        out_file = os.path.splitext(bag_file)[0] + '_can1.txt'
        print(bag_file)
        f = open(out_file, 'w')

        # OPEN BAG
        bag = rosbag.Bag(bag_file, "r")
        messages = bag.read_messages(topics=["/can1/received_messages"])  # Only get images data
        num_images = bag.get_message_count(topic_filters=["/can1/received_messages"])

        for i in range(num_images):
            # READ NEXT MESSAGE IN BAG
            topic, msg, t = messages.__next__()
         #   print(msg.header.stamp.to_sec())
            # CONVERT MESSAGE TO A NUMPY ARRAY
            # img = np.fromstring(msg.data, dtype=np.uint8)
        #    print(msg)
        #    print(hex(msg.id))
        #    print(msg.data.hex(),type(msg.data.hex()))
        #    print(msg.dlc,type(msg.dlc))

            
            timeq = msg.header.stamp.to_sec() 
            timestr = "{:.6f}".format(timeq)
            message_str = timestr + ' id : ' + str(hex(msg.id)) + ' dlc : ' + str(msg.dlc) + ' data : ' + prettyhex(msg.data,'-')

            print(message_str, file=f)


        out_file = os.path.splitext(bag_file)[0] + '_can2.txt'
        f = open(out_file, 'w')

        # OPEN BAG
        bag = rosbag.Bag(bag_file, "r")
        messages = bag.read_messages(topics=["/can2/received_messages"])  # Only get images data
        num_images = bag.get_message_count(topic_filters=["/can2/received_messages"])

        for i in range(num_images):
            # READ NEXT MESSAGE IN BAG
            topic, msg, t = messages.__next__()
            
            timeq = msg.header.stamp.to_sec() 
            timestr = "{:.6f}".format(timeq)
            message_str = timestr + ' id : ' + str(hex(msg.id)) + ' dlc : ' + str(msg.dlc) + ' data : ' + prettyhex2(msg.dlc,msg.data,'-')
            print(message_str, file=f)

        print("DONE!")
