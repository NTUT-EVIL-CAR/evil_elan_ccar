import os
import rosbag

dirs = "./"

for files in os.listdir(dirs):
    if files[-4:]==".bag":
        bag_file = os.path.join(dirs,files)

        ########gps########
        out_file = os.path.splitext(bag_file)[0] + "_gps.txt"
        f = open(out_file, 'w')

        # OPEN BAG
        with rosbag.Bag(bag_file, 'r') as bag:
            for toipc, msg, t in bag.read_messages(topics=["/fix"]):

                timeq = msg.header.stamp.to_sec()
                timestr = "{:.6f}".format(timeq)

                message_str = f"{timestr} latitude : {msg.latitude:.6f} longitude : {msg.longitude:.6f}"
                print(message_str, file=f)
        
        f.close

print("DONE!")