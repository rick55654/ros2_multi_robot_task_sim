# ROS 2 Multi-Robot Task Simulation

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.10-blue.svg)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-green.svg)

A centralized multi-robot simulation for dynamic task assignment in ROS 2 (Humble), using the Hungarian algorithm for optimal allocation and OpenCV for visualization.

---

## 核心功能

* **動態任務分配：** 使用者可以隨時在 OpenCV 視窗中點擊以新增任務。
* **最佳化指派：** Leader 智能體 (Agent) 使用 `scipy.optimize.linear_sum_assignment` (匈牙利演算法) 來計算全域最佳的「機器人-任務」指派。
* **分離式架構：**
    * `simulation_node` (世界/伺服器) 負責物理模擬和視覺化。
    * `robot_agent` (大腦/智能體) 負責路徑規劃成本計算和任務決策。
* **可插拔規劃器：** 支援多種路徑規劃演算法 (A*, RRT, RRT*)。
* **硬體整合：** (可選) 透過序列埠 (`pyserial`) 與 Arduino 連接，以實體 LED 燈號和馬達震動來回饋模擬狀態（任務數量、機器人活動）。
* **高強健性 (Robustness)：**
    * 使用 `BEST_EFFORT` QoS 確保姿態 (Pose) 數據的即時性。
    * 透過「起始點吸附」邏輯，確保「大腦」和「身體」的規劃器行為一致，解決了無效起始點的常見錯誤。
    * 使用「懲罰成本」和 `try-except` 區塊，防止 `infeasible matrix` 錯誤導致 Leader 崩潰。

## 系統架構

本專案採用「中央化任務分配」與「分離式模擬」架構。

1.  **`simulation_node` (世界/伺服器)**
    * 作為單一節點啟動，是所有資訊的「真理之源」。
    * 管理 `Robot` 物件（物理實體）並執行路徑追蹤。
    * 發布 `/task_list` (所有待處理任務) 和 `/<robot_name>/pose` (每個機器人的真實姿態)。
    * 訂閱 `/assign_goal` (來自 Leader 的指令) 並將其傳達給對應的 `Robot` 物件。

2.  **`robot_agent` (大腦/智能體)**
    * 系統中的**每個**機器人都需要啟動**一個** `robot_agent` 節點。
    * **所有**智能體都會訂閱 `/task_list` 和自己的 `/pose`。
    * **所有**智能體都會獨立計算自己到**所有**任務的成本，並將其發布到 `/agent_reports`。
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
