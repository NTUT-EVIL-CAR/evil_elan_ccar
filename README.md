# 收資料
## /shell/ros_start.sh
開啟並連接 canbus, imu, gps, 相機
## /shell/record_time.sh
錄製rosbag用，設定情境並訂閱每一個設備的topic
## /camera/cme_cv2_usb.py
開啟車上的g5相機，並Publish影像訊息。
執行./ros_start.sh時就會同時執行。
## 使用方式
先執行./ros_start.sh 在此須確認有無設備沒有連接上 \
./record_time.sh 執行後需要先設定情境，按r可以進行錄製。

# 解析rosbag用的code
## /parser/rosbagParser_v4.py
會依據bag內各個topic去做timestamp對齊，再把資料輸出。
## 使用方式
須放在與bag同一個資料夾內，直接執行即可 python3 rosbagParser_v2.py

# 備註
其餘程式皆為測試用

# 設備環境
系統：Ubuntu20.04
ROS：noetic
