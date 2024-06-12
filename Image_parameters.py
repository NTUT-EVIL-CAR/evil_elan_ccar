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
     [0, 1, 0, -0.43], # -0.43
     [0, 0, 1, -0.3], # -0.68
     [0, 0, 0, 1]]
)
rotation = rotation_x @ rotation_y @ rotation_z
extri = rotation @ translation

// 順序是 -y -z x
world_points = np.column_stack((-filtered_y, -filtered_z, filtered_x, np.ones_like(filtered_x)))
camera_points = np.dot(extri, world_points.T).T
normalized_camera_points = camera_points[:, :3] / camera_points[:, 2, np.newaxis]
image_points = np.dot(intrist, normalized_camera_points.T)[:2]
