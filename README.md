# 收資料使用的code
## /shell/ros_start.sh
開啟並連接 canbus, imu, gps, 相機
## /shell/record_time.sh
錄製rosbag用，設定情境並訂閱每一個設備的topic
## /camera/cme_cv2_usb.py
開啟車上的g5相機

# 使用方式
./ros_start.sh \
./record_time.sh
