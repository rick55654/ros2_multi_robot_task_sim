# ROS 2 Multi-Robot Task Simulation

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.10-blue.svg)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-green.svg)

A decentralized multi-robot simulation for dynamic task assignment in ROS 2 (Humble), using the Hungarian algorithm for optimal allocation and OpenCV for visualization.

---

## 核心功能

* **動態任務分配：** 使用者可以隨時在 OpenCV 視窗中點擊以新增任務。
* **最佳化指派：** Leader 智能體 (Agent) 使用 `scipy.optimize.linear_sum_assignment` (匈牙利演算法) 來計算最佳的「機器人-任務」指派。
* **分離式架構：**
    * `simulation_node` (顯示介面) 負責物理模擬和視覺化。
    * `robot_agent` (機器人) 負責路徑規劃成本計算和任務決策。
* **多種規劃器：** 支援多種路徑規劃演算法 (A*, RRT, RRT*)。
* **多種控制器：** 支援多種控制演算法 (PID, LQR, Stanley, Pure Pursuit)。
* **硬體整合：** (可選) 透過序列埠 (`pyserial`) 與 Arduino 連接，以實體 LED 燈號和馬達震動來回饋模擬狀態（任務數量、機器人活動）。

## 系統架構

本專案採用「中央化界面顯示」與「分離式模擬」架構。

1.  **`simulation_node` (顯示介面)**
    * 作為單一節點啟動，是所有資訊的來源。
    * 管理 `Robot` 物件（物理實體）並執行路徑追蹤。
    * 發布 `/task_list` (所有待處理任務) 和 `/<robot_name>/pose` (每個機器人的真實姿態)。
    * 訂閱 `/assign_goal` (來自 Leader 的指令) 並將其傳達給對應的 `Robot` 物件。

2.  **`robot_agent` (機器人)**
    * 系統中的**每個**機器人都需要啟動**一個** `robot_agent` 節點。
    * **所有**機器人都會訂閱 `/task_list` 和自己的 `/pose`。
    * **所有**機器人都會獨立計算自己到**所有**任務的成本，並將其發布到 `/agent_reports`。
    * **僅有 Leader** (ID 最小的 `robot_agent`) 會訂閱 `/agent_reports`，建立成本矩陣，運行匈牙利演算法，並將指派結果發布到 `/assign_goal`。

## 安裝與執行

**需求：**
* ROS 2 Humble
* Python 3.10+
* OpenCV (`python3-opencv`)
* SciPy (`python3-scipy`)
* PySerial (`python3-serial`)

**1. 建立與編譯**

```bash
# 1. 進入您的 ROS 2 工作區 src 資料夾
cd ~/ros2_ws/src/

# 2. 將專案複製到此處
# git clone [https://github.com/](https://github.com/)[your_username]/ros2_multi_robot_task_sim.git

# 3. 回到工作區根目錄
cd ~/ros2_ws/

# 4. 編譯專案
colcon build --packages-select multi_robot_sim

# 5. Source 環境
source install/setup.bash
```
**2. 執行模擬**

您需要開啟至少 3 個終端機。

```bash
# 終端機 1: 啟動模擬器 (世界/伺服器)
source ~/ros2_ws/install/setup.bash
ros2 run multi_robot_sim simulation_node
```
```bash

# 終端機 2: 啟動機器人 1 (大腦)
source ~/ros2_ws/install/setup.bash
ros2 launch multi_robot_sim robot1_launch.py
```
```bash

# 終端機 3: 啟動機器人 2 (大腦)
source ~/ros2_ws/install/setup.bash
ros2 launch multi_robot_sim robot2_launch.py
```
**3. 使用方式**
1. 啟動所有節點後，一個名為 "ROS 2 Multi-Robot Simulation" 的 OpenCV 視窗將會出現。

2. 在視窗的白色可通行區域內點擊滑鼠左鍵。

3. 每次點擊都會在 `/task_list `中新增一個任務。

4. Leader 智能體將自動計算最佳分配，並派遣閒置的機器人前往執行。

5. (可選) 連接 Arduino，LED 燈將顯示待處理任務的數量。
