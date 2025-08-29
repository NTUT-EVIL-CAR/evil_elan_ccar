# 專案說明

本專案提供 **資料收集** 與 **rosbag 解析** 的工具，並針對車載感測器進行整合。

---

## 📦 收資料

### `/shell/ros_start.sh`
- 開啟並連接 **CAN bus、IMU、GPS、相機**

### `/shell/record_time.sh`
- 錄製 rosbag  
- 需先設定情境，並訂閱每一個設備的 topic

### `/camera/cme_cv2_usb.py`
- 開啟車上的 **G5 相機**，並 Publish 影像訊息  
- 在執行 `./ros_start.sh` 時會自動啟動

#### 使用方式
1. 先執行 `./ros_start.sh`，確認所有設備皆已連接  
2. 執行 `./record_time.sh` → 輸入情境設定  
3. 按下 `r` 開始錄製

---

## 🔍 解析 rosbag

### `/parser/rosbagParser_v4.py`
- 解析 **CAN bus** 資訊  
- 以 **G5 timestamp** 為基準（頻率最慢），與 rosbag 內各 topic 做 timestamp 對齊（往後抓最接近的時間點）  
- 輸出整合後的資料

#### 使用方式
- 將檔案放置於與 `.bag` 檔同一資料夾  
- 執行：
  ```bash
  python3 rosbagParser_v4.py
