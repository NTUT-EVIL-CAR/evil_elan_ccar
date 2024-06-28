import folium
import subprocess

# 初始化地圖
bag_name = "123456789"
coordinates = [[25.042158, 121.534920], [25.043674, 121.532946], [25.044141, 121.536293], [25.041886, 121.537280], [25.047815, 121.536894], [25.055279, 121.536980], [25.067136, 121.534598]]
taiwan_map = folium.Map(location=coordinates[0], zoom_start=16, tiles="OpenStreetMap")

for c in coordinates:
    folium.Marker(
        location=c,
        popup=bag_name,
        icon=folium.Icon(
            color="black",
            icon_color="red",
            icon="car",
            prefix="fa"
        )
    ).add_to(taiwan_map)

folium.PolyLine(
    locations=coordinates,
    tooltip="abc",
    popup="line",
    color="blue",
    weight=3
).add_to(taiwan_map)

# 顯示地圖
taiwan_map.save('taiwan_map.html')

subprocess.call(["google-chrome", "taiwan_map.html"])