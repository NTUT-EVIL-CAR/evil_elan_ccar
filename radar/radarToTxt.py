import os
import rosbag
    
def prettyhex2(dlc,nums, sep=''):
    return sep.join(f'{a:02x}' for a in nums[0:dlc])

dirs = "./"
out_dir = os.path.join(dirs, "txt")

bag_files = [f for f in os.listdir(dirs) if f.endswith(".bag")]
sorted_bag_files = sorted(bag_files)

if not os.path.exists(out_dir):
    os.makedirs(out_dir)

for files in sorted_bag_files:
    if files[-4:]==".bag":
        bag_dir = os.path.join(dirs, files)

        ########radar########
        out_file = os.path.splitext(bag_dir)[0] + "_radar.txt"
        out_radar_dir = os.path.join(out_dir, out_file)
        f = open(out_radar_dir, 'w')

        # OPEN BAG
        with rosbag.Bag(bag_dir, 'r') as bag:
            for toipc, msg, t in bag.read_messages(topics=["/can0/received_msg"]):

                timeq = msg.header.stamp.to_sec()
                timestr = "{:.6f}".format(timeq)

                message_str = timestr + ' id : ' + str(hex(msg.id)) + ' dlc : ' + str(msg.dlc) + ' data : ' + prettyhex2(msg.dlc,msg.data,'-')
                print(message_str, file=f)
        
        f.close

print("DONE!")