# 資料收集與解析工具

本專案提供一套完整的工具，用於收集多種車載感測器（CAN bus, IMU, GPS, 相機, LiDAR）的數據，並將其錄製為 `rosbag` 檔案。此外，專案還包含一個解析腳本，能夠同步不同感測器的時間戳，並將原始數據轉換為結構化的資料格式。

---

## 系統工作流程

整個系統分為兩個主要階段：

1.  **數據收集**：
    -   `ros_start.sh`：一鍵啟動所有感測器的 ROS 驅動節點。
    -   `record_time.sh`：根據使用者設定的情境（如路況、天氣），錄製指定的 ROS topics 並儲存為 `.bag` 檔案。

2.  **數據解析**：
    -   `rosbagParser_v4.py`：讀取 `.bag` 檔案，以 G5 相機的 CAN 訊號時間戳為基準，對齊所有感測器的數據，最後將影像、點雲、CAN 資訊等分別導出至對應資料夾。

---

## 使用說明

### 數據收集

1.  **啟動感測器**：
    在terminal執行啟動腳本，這將會初始化所有硬體並啟動 ROS 節點。
    ```bash
    ./shell/ros_start.sh
    ```
    確認所有新開啟的終端機分頁中，各個節點都已正常運行。

2.  **錄製 Rosbag**：
    執行錄製腳本，並手動設定當前的駕駛情境。
    ```bash
    ./shell/record_time.sh
    ```
    -   輸入 `c` 可隨時更改情境設定。
    -   輸入 `r` 開始錄製（預設10秒）。
    -   輸入 `r 60` 開始錄製60秒。
    -   輸入 `q` 退出程式。

### 數據解析

1.  **準備檔案**：
    將 `rosbagParser_v4.py` 腳本放置於存放 `.bag` 檔案的同一層資料夾中。

2.  **執行解析**：
    在該資料夾中執行 Python 腳本。
    ```bash
    python3 rosbagParser_v4.py
    ```
    腳本會自動尋找所有 `.bag` 檔案，逐一進行解析，並將結果儲存在名為 `YYYY-MM-DD_parsed_data` 的新資料夾內。

---

## 核心腳本詳解

### `/shell/ros_start.sh`

**功能**：初始化所有硬體介面 (CAN, IMU, GPS) 與 ROS 節點。

此腳本透過 `gnome-terminal` 在多個分頁中並行啟動 `roscore` 以及各個感測器的驅動程式，實現一鍵部署數據收集環境。

```bash
# 啟動 roscore
gnome-terminal --tab -t "roscore" -- bash -c "roscore;exec bash"
sleep 6

# 啟動各個感測器的 ROS 節點 (以 CAN 和 IMU 為例)
source ~/catkin_ws/devel/setup.bash
gnome-terminal --tab -t "can" -- bash -c "roslaunch ~/Desktop/can2topic.launch;exec bash"
gnome-terminal --tab -t "imu" -- bash -c "roslaunch razor_imu_9dof razor-pub.launch;exec bash"
