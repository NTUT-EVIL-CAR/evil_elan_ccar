import numpy as np
import os
import rosbag

from sensor_msgs import point_cloud2

def prettyhex(nums, sep=''):
    return sep.join(f'{a:02x}' for a in nums)
    
def prettyhex2(dlc,nums, sep=''):
    return sep.join(f'{a:02x}' for a in nums[0:dlc])

dirs = "./"
out_dir = os.path.join(dirs, "canDataTxt")

bag_files = [f for f in os.listdir(dirs) if f.endswith(".bag")]
sorted_bag_files = sorted(bag_files)

if not os.path.exists(out_dir):
    os.makedirs(out_dir)

for files in bag_files:
    if files[-4:]=='.bag':
        bag_dir = os.path.join(dirs,files)

        ########can0########
        out_file = os.path.splitext(bag_dir)[0] + '_can0.txt'
        out_can0_dir = os.path.join(out_dir, out_file)
        f = open(out_can0_dir, 'w')
   
        with rosbag.Bag(bag_dir, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=["/can0/received_msg"]):
                timeq = msg.header.stamp.to_sec() 
                timestr = "{:.6f}".format(timeq)

                message_str = timestr + ' id : ' + str(hex(msg.id)) + ' dlc : ' + str(msg.dlc) + ' data : ' + prettyhex2(msg.dlc,msg.data,'-')
                print(message_str, file=f)
        
        f.close()

        ########can1########
        out_file = os.path.splitext(bag_dir)[0] + '_can1.txt'
        out_can1_dir = os.path.join(out_dir, out_file)
        f = open(out_can1_dir, 'w')
   
        with rosbag.Bag(bag_dir, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=["/can1/received_msg"]):
                timeq = msg.header.stamp.to_sec() 
                timestr = "{:.6f}".format(timeq)

                message_str = timestr + ' id : ' + str(hex(msg.id)) + ' dlc : ' + str(msg.dlc) + ' data : ' + prettyhex2(msg.dlc,msg.data,'-')
                print(message_str, file=f)
        
        f.close()

        ########can2########
        out_file = os.path.splitext(bag_dir)[0] + '_can2.txt'
        out_can2_dir = os.path.join(out_dir, out_file)
        f = open(out_can2_dir, 'w')
     
        with rosbag.Bag(bag_dir, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=["/can2/received_msg"]):
                timeq = msg.header.stamp.to_sec() 
                timestr = "{:.6f}".format(timeq)

                message_str = timestr + ' id : ' + str(hex(msg.id)) + ' dlc : ' + str(msg.dlc) + ' data : ' + prettyhex2(msg.dlc,msg.data,'-')
                print(message_str, file=f)
        
        f.close()

        ########lidar########
        out_file = os.path.splitext(bag_dir)[0] + '_lidar.txt'
        out_lidar_dir = os.path.join(out_dir, out_file)
        f = open(out_lidar_dir, 'w')

        with rosbag.Bag(bag_dir, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=["/rslidar_points"]):
                timeq = msg.header.stamp.to_sec() 
                timestr = "{:.6f}".format(timeq)

                gen = point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
                for p in gen:
                    message_str = f"{timestr} X : {p[0]:.3f} Y : {p[1]:.3f} Z : {p[2]:.3f} Intensity : {int(p[3])}"
                    print(message_str, file=f)
        
        f.close()

        ########imu########
        out_file = os.path.splitext(bag_dir)[0] + '_imu.txt'
        out_imu_dir = os.path.join(out_dir, out_file)
        f = open(out_imu_dir, 'w')

        with rosbag.Bag(bag_dir, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=["/imu"]):
                timeq = msg.header.stamp.to_sec() 
                timestr = "{:.6f}".format(timeq)

                message_str = f"{timestr} type : Angular X : {msg.angular_velocity.x} Y : {msg.angular_velocity.y} Z : {msg.angular_velocity.z}"
                print(message_str, file=f)

                message_str = f"{timestr} type : Acceleration X : {msg.linear_acceleration.x} Y : {msg.linear_acceleration.y} Z : {msg.linear_acceleration.z}"
                print(message_str, file=f)
        
        f.close()

        ########gps########
        out_file = os.path.splitext(bag_dir)[0] + "_gps.txt"
        out_gps_dir = os.path.join(out_dir, out_file)
        f = open(out_gps_dir, 'w')

        # OPEN BAG
        with rosbag.Bag(bag_dir, 'r') as bag:
            for toipc, msg, t in bag.read_messages(topics=["/fix"]):

                timeq = msg.header.stamp.to_sec()
                timestr = "{:.6f}".format(timeq)

                message_str = f"{timestr} latitude : {msg.latitude:.6f} longitude : {msg.longitude:.6f}"
                print(message_str, file=f)
        
        f.close

        ########VCS128-lidar########
        out_file = os.path.splitext(bag_dir)[0] + '_VLS128.txt'
        out_lidar_dir = os.path.join(out_dir, out_file)
        f = open(out_lidar_dir, 'w')

        with rosbag.Bag(bag_dir, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=["/velodyne_points"]):
                timeq = msg.header.stamp.to_sec() 
                timestr = "{:.6f}".format(timeq)

                gen = point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity", "ring", "time"), skip_nans=True)
                for p in gen:
                    message_str = f"{timestr}\tX: {p[0]:.3f}\tY: {p[1]:.3f}\tZ: {p[2]:.3f}\tIntensity: {int(p[3])}\tRing: {int(p[4])}\tTime: {int(p[5])}"
                    print(message_str, file=f)
        
        f.close()

print("DONE!")