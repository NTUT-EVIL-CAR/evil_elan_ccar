# 車載多感測器數據收集與解析工具

本專案提供一套完整的工具，用於收集多種車載感測器（CAN bus, IMU, GPS, 相機, LiDAR）的數據，並將其錄製為 `rosbag` 檔案。此外，專案還包含一個強大的解析腳本，能夠以 G5 相機的 CAN 訊號為基準，同步所有感測器的時間戳，並將原始數據轉換為可用於分析的結構化資料。

## 系統架構與工作流程

整個系統分為兩個主要階段：數據收集與數據解析。

1.  **數據收集 (`Data Collection`)**
    * **`./shell/ros_start.sh`**: 一鍵式啟動腳本。它會自動設定硬體權限（CAN, IMU, GPS），並在新的終端機分頁中啟動 `roscore` 以及所有感測器的 ROS 驅動節點。
    * **`./shell/record_time.sh`**: 互動式錄製腳本。使用者可以根據當前的路況、天氣和時間設定駕駛情境，然後錄製指定時長的 `rosbag` 檔案。

2.  **數據解析 (`Data Parsing`)**
    * **`rosbagParser_v4.py`**: 自動化解析腳本。它會遍歷資料夾中所有的 `.bag` 檔案，以 **G5 相機的 CAN 訊號時間戳**為同步基準，對齊所有感測器數據。最終，影像、點雲、IMU 以及解析後的 CAN 資訊會被分別導出至對應的結構化資料夾中。

## 環境與依賴

### 硬體需求
* [cite_start]**CAN Interfaces**: 3x CAN bus (can0, can1, can2) [cite: 1]
* **IMU**: Razor 9-DOF IMU (或類似設備，位於 `/dev/ttyACM0`)
* **GPS**: NMEA 相容 GPS (位於 `/dev/ttyUSB0`)
* [cite_start]**相機**: USB Camera (位於 `/dev/video2`, 支援 1280x720 @ 30fps MJPG) [cite: 2]
* **LiDAR**:
    * Velodyne VLS-128
    * RoboSense LiDAR (可選)

### 軟體需求
* **OS**: Ubuntu 20.04 (建議)
* **ROS**: ROS Noetic
* **Python 3.8+**
* **ROS Packages**:
    ```bash
    sudo apt-get install ros-noetic-socketcan-bridge ros-noetic-razor-imu-9dof ros-noetic-nmea-navsat-driver ros-noetic-velodyne
    ```
* **Python Packages**:
    ```bash
    pip install numpy opencv-python pyyaml rospkg open3d folium tqdm
    ```

## 安裝與設定

1.  **建立 ROS 工作空間**:
    本專案需要兩個獨立的 ROS 工作空間，一個用於通用感測器，一個用於 GPS。

    ```bash
    # 建立通用 catkin 工作空間
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make
    
    # 建立 GPS 工作空間
    mkdir -p ~/gps_ws/src
    cd ~/gps_ws/
    catkin_make
    ```
    *將本專案的 ROS packages（如 `can2topic`）放置於 `~/catkin_ws/src` 下並編譯。*

2.  **設定硬體權限**:
    `ros_start.sh` 腳本會嘗試自動設定權限，但建議您將使用者加入 `dialout` 和 `tty` 群組以避免權限問題。
    ```bash
    sudo usermod -a -G dialout $USER
    sudo usermod -a -G tty $USER
    ```

3.  **設定 CAN 接口**:
    腳本會自動以 500000 的 bitrate 啟動 `can0`, `can1`, `can2`。請確保您的硬體支援此設定。

## 使用說明

### 步驟一：數據收集

1.  **啟動所有感測器**:
    執行啟動腳本，這將會初始化硬體並在多個終端機分頁中啟動所有 ROS 節點。
    ```bash
    ./shell/ros_start.sh
    ```
    確認所有新開啟的分頁（roscore, can, imu, video, VLS128, gps）都已正常運行且無錯誤。

2.  **錄製 Rosbag**:
    執行錄製腳本，並根據提示設定當前的駕駛情境。
    ```bash
    ./shell/record_time.sh
    ```
    腳本會進入互動模式：
    * **設定情境**: 根據提示輸入 `路況 (road_type)`、`天氣 (weather)`、`時段 (time_period)` 的代號。例如，`highway_sunny_day`。
    * 輸入 `c` 可隨時重新設定情境。
    * 輸入 `r` 開始錄製（預設10秒）。
    * 輸入 `r 60` 開始錄製60秒。
    * 輸入 `t 120` 錄製120秒的隧道情境。
    * 輸入 `q` 退出程式。

    錄製的 `.bag` 檔案會儲存在 `./bag_record/YYYY-MM-DD/` 路徑下。

### 步驟二：數據解析

1.  **準備檔案**:
    將 `rosbagParser_v4.py` 腳本與所有 `.bag` 檔案放置於同一個資料夾中。

2.  **執行解析**:
    在該資料夾中開啟終端機並執行 Python 腳本。
    ```bash
    python3 rosbagParser_v4.py
    ```
    腳本會自動尋找所有 `.bag` 檔案，逐一進行時間同步與解析，並將結果儲存在名為 `YYYY-MM-DD_parsed_data` 的新資料夾內。解析完成後，原始的 `.bag` 檔案會被移至 `YYYY-MM-DD_bag` 資料夾。

## 核心腳本詳解

#### `/shell/ros_start.sh`
此腳本為一鍵部署工具，它執行以下任務：
* 設定 `ttyACM0` (IMU) 和 `ttyUSB0` (GPS) 的讀寫權限。
* 設定並啟動 `can0`, `can1`, `can2` 網路接口。
* 在獨立分頁中啟動 `roscore`。
* 啟動各感測器的 ROS 驅動節點，包括：
    * [cite_start]**CAN**: `can2topic.launch` (將 can0, can1, can2 數據轉發至 ROS topics) [cite: 1]。
    * **IMU**: `razor_imu_9dof`。
    * [cite_start]**相機**: `cme_cv2_usb.py` (自定義 OpenCV 影像發布節點) [cite: 2]。
    * **LiDAR**: `velodyne_pointcloud` for VLS128。
    * **GPS**: `nmea_navsat_driver`。
* 啟動 `RViz` 並載入預設的可視化設定。

#### `/shell/record_time.sh`
互動式錄製腳本，負責錄製以下核心 ROS topics：
```
/can0/received_msg
/can1/received_msg
/can2/received_msg
/cme_cam
/rslidar_points
/imu
/fix
/velodyne_points
```

#### `rosbagParser_v4.py`
此腳本是整個專案的核心，其主要功能包括：
* **時間同步**: 以 `/can0/received_msg` 中 G5 相機的 `0x4e1` ID 作為時間基準，搜尋並對齊其他所有感測器最接近的數據幀。
* **CAN 訊息解碼**: 內建 `IVAG5MessageParser`, `MobQ4MessageParser`, `RadarMessageParser` 等類別，可將原始 CAN data bytes 解碼為有意義的物理數值（如車道線參數、目標物體位置與速度等）。
* **數據導出**: 將同步後的數據以幀為單位（frame-by-frame）儲存。影像存為 `.jpg`，點雲存為 `.pcd` 和 `.npy`，其他數據存為 `.txt`。
* **GPS路徑圖生成**: 提取所有 `.bag` 檔中的 GPS 軌跡，並使用 `folium` 套件生成一張包含所有路徑點的 `HTML` 地圖。

## 輸出檔案結構

執行 `rosbagParser_v4.py` 後，將會生成如下結構的資料夾：
```
.
├── YYYY-MM-DD_parsed_data/
│   └── [SITUATION_NAME]/             # 例如 highway_sunny_day
│       └── [BAG_FILE_NAME]/          # Rosbag 檔名
│           ├── image/                # 影像幀
│           │   ├── 000000.jpg
│           │   └── ...
│           ├── VLS128_pcdnpy/        # VLS-128 點雲
│           │   ├── 000000.pcd
│           │   ├── 000000.npy
│           │   └── ...
│           ├── IVA_g5/               # G5 相機 CAN 解析數據
│           │   ├── 000000.txt
│           │   └── ...
│           ├── Mobileye_q4/          # Q4 相機 CAN 解析數據
│           │   └── ...
│           ├── imu/                  # IMU 數據
│           │   └── ...
│           ├── radar/                # Radar CAN 解析數據
│           │   └── ...
│           ├── timestamp/            # 各感測器對齊後的時間戳
│           │   ├── image.txt
│           │   ├── VLS128.txt
│           │   └── ...
│           └── gps.txt               # 該 Rosbag 的 GPS 軌跡
│
├── YYYY-MM-DD_bag/                   # 存放已處理完的 Rosbag
│   └── [BAG_FILE_NAME].bag
│
└── [FOLDER_NAME].html                # 所有 Rosbag 軌跡的 HTML 地圖
```
