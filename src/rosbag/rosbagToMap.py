import folium
import os
import rosbag
import subprocess

dirs = "./"
bag_name = ""
bag_coordinates = []

def sort_key(file_name):
    return file_name.split('_')[-1]

bag_files = [f for f in os.listdir(dirs) if f.endswith(".bag")]
sorted_bag_files = sorted(bag_files, key=sort_key)

for file in sorted_bag_files:
    bag_name = file
    bag_file = os.path.join(dirs,file)

    with rosbag.Bag(bag_file, 'r') as bag:
        for toipc, msg, t in bag.read_messages(topics=["/fix"]):
            if (f"{msg.latitude}" != "nan"):
                bag_coordinates.append({
                    "coordinates": [msg.latitude, msg.longitude],
                    "bag_name": bag_name
                })

if (bag_coordinates):
    taiwan_map = folium.Map(location=bag_coordinates[0]["coordinates"], zoom_start=16, tiles="OpenStreetMap")

    for c in bag_coordinates:
        folium.Marker(
            location=c["coordinates"],
            popup=c["bag_name"],
            icon=folium.Icon(
                color="black",
                icon_color="red",
                icon="car",
                prefix="fa"
            )
        ).add_to(taiwan_map)

        folium.PolyLine(
            locations=[c["coordinates"] for c in bag_coordinates],
            # tooltip=c["bag_name"],
            # popup="line",
            color="blue",
            weight=3
        ).add_to(taiwan_map)

    # 顯示地圖 以父目錄名稱命名
    parent_dir_name = os.path.basename(os.path.dirname(os.path.realpath(__file__)))
    taiwan_map.save(f"{parent_dir_name}.html")

    # subprocess.call(["google-chrome", "taiwan_map.html"])

    print("Map DONE !")