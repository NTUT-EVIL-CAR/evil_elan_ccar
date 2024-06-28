import rosbag
import os

# 設定輸入和輸出的bag檔案路徑
input_bag_dir = './FullBag'
output_bag_dir = './ImageImuBag'

if not os.path.exists(output_bag_dir):
    os.mkdir(output_bag_dir)

# 設定要提取的特定主題(topic)
target_topics = ['/cme_cam', '/imu']

for files in os.listdir(input_bag_dir):
    if files[-4:] == ".bag":
        print(f"parsing {files} ...")
        input_bag_path = os.path.join(input_bag_dir, files)
        input_bag_name = files.split('.')[0]
        output_bag_path = os.path.join(output_bag_dir, f"EVIL-{input_bag_name}.bag")

        # 打開輸入和輸出的bag檔案
        with rosbag.Bag(input_bag_path, 'r') as input_bag, rosbag.Bag(output_bag_path, 'w') as output_bag:
            # 迭代輸入bag中的每一條紀錄
            for topic, msg, t in input_bag.read_messages():
                # 如果該紀錄的主題在目標主題列表中，將其寫入輸出bag
                if topic in target_topics:
                    output_bag.write(topic, msg, t)
                    
        print(f"saved {target_topics} to {output_bag_path}")

print("Done 3人3")