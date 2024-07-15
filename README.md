# 收資料使用的code
## /shell/ros_start.sh
開啟並連接 canbus, imu, gps, 相機
## /shell/record_time.sh
錄製rosbag用，設定情境並訂閱每一個設備的topic
## /camera/cme_cv2_usb.py
開啟車上的g5相機

# 使用方式
先執行./ros_start.sh 在此須確認有無設備沒有連接上 \
./record_time.sh 執行後需要先設定情境，按r可以進行錄製。

# 設備環境
系統：Ubuntu20.04
ROS：noetic
