import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

# 讀取 pcd 檔案
pcd = o3d.io.read_point_cloud("000000.pcd")

# 取得點雲數據
points = np.asarray(pcd.points)

# 轉換到相機坐標系
theta_x = np.radians(0.5)
theta_y = np.radians(5.4)
theta_z = np.radians(2.1)
rotation_x = np.array(
    [[np.cos(theta_x), -np.sin(theta_x), 0, 0],
     [np.sin(theta_x), np.cos(theta_x), 0, 0],
     [0, 0, 1, 0],
     [0, 0, 0, 1]]
)
rotation_y = np.array(
    [[1, 0, 0, 0],
     [0, np.cos(theta_y), -np.sin(theta_y), 0],
     [0, np.sin(theta_y), np.cos(theta_y), 0],
     [0, 0, 0, 1]]
)
rotation_z = np.array(
    [[np.cos(theta_z), 0, np.sin(theta_z), 0],
     [0, 1, 0, 0],
     [-np.sin(theta_z), 0, np.cos(theta_z), 0],
     [0, 0, 0, 1]]
)
translation = np.array(
    [[1, 0, 0, 0],
     [0, 1, 0, -0.43],
     [0, 0, 1, -0.3],
     [0, 0, 0, 1]]
)
rotation = rotation_x @ rotation_y @ rotation_z
extri = rotation @ translation

intrist = np.array([[1.418667e+03, 0.000e+00, 6.4e+02],
                    [0.000e+00, 1.418667e+03, 3.6e+02],
                    [0.000e+00, 0.000e+00, 1.0e+00]])

valid_indices = (points[:, 0] >= 0)
filtered_x = points[:, 0][valid_indices]
filtered_y = points[:, 1][valid_indices]
filtered_z = points[:, 2][valid_indices]

world_points = np.column_stack((-filtered_y, -filtered_z, filtered_x, np.ones_like(filtered_x)))
camera_points = np.dot(extri, world_points.T).T
normalized_camera_points = camera_points[:, :3] / camera_points[:, 2, np.newaxis]
image_points = np.dot(intrist, normalized_camera_points.T)[:2]

# 顯示結果
print("相機坐標系下的點雲數據: ")
print(camera_points)

print("投影到圖像平面的點: ")
print(image_points)

# 顯示點雲
o3d.visualization.draw_geometries([pcd])


# 點雲圖片合成
img = mpimg.imread("000000.png")
plt.close()
fig, ax = plt.subplots(figsize=(img.shape[1] / 100, img.shape[0] / 100))

ax.imshow(img)

scatter = ax.scatter(x=(image_points[0]), y=(image_points[1]), c=filtered_x, cmap="jet", alpha=1, s=1, linewidths=1)

# 顏色條 & xy軸
# fig.colorbar(scatter)
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
ax.set_axis_off()

# 指定輸出範圍
ax.set_xlim(0, img.shape[1])
ax.set_ylim(img.shape[0], 0)

plt.show()