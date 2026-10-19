# 結合視覺規劃與觸覺回饋之機械手臂夾取系統：以 ROS 2 模擬為基礎的開發

**Vision-Guided Robotic Grasping with Tactile Feedback: A ROS 2 Simulation-First Approach**

國立臺北科技大學 機電工程系
專題論文（期中草稿，版本日期：2026-10-19）

> 本文為進行中的版本。第五章只記錄已在模擬中實際跑出的結果；尚未完成的部分標示為「規劃中」，不填入預估數據。

---

## 摘要

傳統工業機械手臂擅長重複的位置控制，卻難以處理軟硬程度不同的物體：夾力不足會滑落，夾力過大會損壞物體。本專題提出一套「移動交給成熟演算法、觸覺交給學習」的分層架構：以深度相機建立環境地圖、以物體辨識取得目標座標、以 MoveIt 2 規劃避障軌跡，並以接觸／壓力感測搭配回饋控制調整夾力。

開發採「模擬優先」策略，於 ROS 2 Jazzy 與 Gazebo Harmonic 中逐步驗證。目前已完成：(1) 四軸教學手臂的建模、ros2_control 控制與接觸感測器整合；(2) 自行推導之幾何逆運動學與可達性分析；(3) MoveIt 2 軌跡規劃與障礙物迴避驗證；(4) 完整的夾取與放置（pick & place），放置誤差約 1 cm；(5) 自行設計之六軸手臂由 CAD 轉為 URDF 並完成關節極限校正。後續將進行深度感知避障、物體辨識與夾力控制。

**關鍵字**：ROS 2、MoveIt 2、Gazebo、逆運動學、觸覺感測、力控制、夾取

---

## 第一章 緒論

### 1.1 研究背景

機械手臂的夾取任務可拆成三個問題：**東西在哪**（感知）、**怎麼過去**（規劃）、**要夾多緊**（施力）。前兩者已有成熟工具，例如深度相機、物體辨識模型與運動規劃框架；第三者則高度依賴物體特性，難以用單一規則處理。

### 1.2 研究動機

多數學生專題把重心放在「讓手臂動起來」，但夾取失敗的主因常常不是位置錯誤，而是力道不對。本專題希望把有限的時間投入在真正缺乏標準解的環節：觸覺回饋與夾力控制。

### 1.3 研究目的

1. 建立一套從感知、規劃到夾取的完整模擬流程
2. 驗證「規劃不需訓練、施力才需要學習」的分層設計
3. 在模擬中完成夾力回饋控制，並設計實機上的對照實驗

### 1.4 研究範圍與限制

- 現階段全部在模擬器中進行，尚未使用實體硬體
- Gazebo 接觸感測器的力量數值不穩定，力控將參考關節施力（effort）
- 模擬與真實感測器存在差距（sim-to-real gap），實機階段需另行校正

---

## 第二章 文獻與工具探討

### 2.1 機器人作業系統 ROS 2

ROS 2 以 node（獨立程式）、topic（發布／訂閱的資料管線）、service（請求／回應）組成分散式架構，使感測、規劃、控制各自獨立開發與替換。本專題使用長期支援版本 Jazzy。

### 2.2 模擬環境 Gazebo

Gazebo 提供重力、碰撞、摩擦等物理模擬，並能模擬相機、IMU、深度相機與接觸感測器。透過 ros_gz_bridge 與 ROS 2 交換訊息。

### 2.3 運動規劃框架 MoveIt 2

MoveIt 2 整合逆運動學求解、碰撞檢查與取樣式路徑規劃（OMPL，如 RRTConnect）。給定目標與場景後以演算法搜尋可行路徑，因此不需要訓練資料。

### 2.4 學習式控制

| 方法 | 代表工具 | 資料來源 | 特性 |
| --- | --- | --- | --- |
| 模仿學習 | LeRobot（ACT、Diffusion Policy） | 人類遙控示範 | 適合實機、資料量小 |
| 強化學習 | Isaac Lab（PPO） | 模擬中自主試錯 | 需要大量平行模擬 |

Gazebo 依真實時間運行，不適合大量強化學習訓練；本專題以 Gazebo 驗證運動學與流程，學習式方法留待感測器整合後評估。

### 2.5 教材

主要依循匈牙利布達佩斯科技經濟大學（BME）MOGI 實驗室公開之 ROS 2 課程（MOGI-ROS），以及 ROS 2 與 MoveIt 官方文件。

---

## 第三章 系統架構

### 3.1 整體流程

```
深度鏡頭 ──▶ 點雲 / OctoMap ──┐
                              ├──▶ MoveIt 2 軌跡規劃 ──▶ 手臂移動 ──▶ 夾爪執行
物體辨識 ──▶ 目標 3D 座標 ────┘                                       ▲
                                                                       │
                                         壓力感測 ──▶ 夾力回饋控制 ─────┘
```

| 環節 | 內容 | 是否訓練 |
| --- | --- | --- |
| 1. 空間掃描 | 點雲 → OctoMap 障礙地圖 | 否 |
| 2. 物體辨識 | YOLO 辨識目標並換算 3D 座標 | 微調 |
| 3. 軌跡規劃 | MoveIt 2 自動避障 | 否 |
| 4. 夾爪執行 | 移動至目標並夾取 | 否 |
| **5. 力控** | **校正 → 回饋控制夾力 → 實機收資料** | **是** |

### 3.2 開發環境

| 項目 | 規格 |
| --- | --- |
| 電腦 | AMD R7 9700X、RTX 5060 Ti 16 GB、32 GB RAM |
| 作業系統 | Windows 11 + WSL2 Ubuntu 24.04 |
| 軟體 | ROS 2 Jazzy、Gazebo Harmonic、MoveIt 2 |
| CAD | SolidWorks 2024、Autodesk Inventor |

### 3.3 預定實機硬體（尚未採購）

| 元件 | 規劃 |
| --- | --- |
| 主控 | Jetson Orin Nano Super |
| 馬達驅動 | 伺服馬達 + PCA9685（I²C，16 通道 PWM） |
| 下位控制 | Arduino 或 ESP32（USB 序列通訊） |
| 感測器 | 深度相機、夾爪壓力感測器（FSR 等） |

---

## 第四章 研究方法

### 4.1 模擬手臂建模

以 URDF/xacro 描述手臂為 link 與 joint 組成的樹狀結構，`world → base_link` 以固定關節連接避免倒塌。每個 link 包含外觀（visual）、碰撞（collision）與慣量（inertial）三部分。

教學手臂為 4 個旋轉關節（shoulder_pan、shoulder_lift、elbow、wrist）加兩指夾爪，左手指裝設接觸感測器。

### 4.2 關節控制

採用 ros2_control：`controller_manager` 管理 `joint_state_broadcaster`（發布關節狀態）與 `arm_controller`（接收軌跡指令）。Gazebo 端以 gz_ros2_control 外掛驅動模擬關節。

### 4.3 逆運動學推導

**(1) 底座方位角**

$$\theta_1 = \operatorname{atan2}(y,\ x)$$

使用 `atan2` 而非 `atan(y/x)`，以正確處理目標位於側邊（x = 0）與後方（x < 0）的情況。

**(2) 手腕位置**

由末端目標點沿夾爪方向回推夾爪長度 d = 0.175 m，得到手腕中心。

**(3) 兩連桿平面問題**

設肩關節至手腕距離為 c，兩節手臂長度為 L₁、L₂，由餘弦定理：

$$\cos\gamma = \frac{L_1^2 + L_2^2 - c^2}{2L_1L_2}$$

當 c > L₁ + L₂（本手臂約 0.45 m）時無實數解，即目標不可達。

### 4.4 可達性分析

目標方塊位於手臂座標 (0.50, 0, 0.02) m，肩關節高度 0.1 m：

| 夾取姿態 | 手腕位置 | c (m) | 可達 |
| --- | --- | --- | --- |
| 夾爪朝下 | 目標正上方 0.175 m | ≈ 0.509 | 否 |
| 夾爪水平 | 目標後方 0.175 m | ≈ 0.335 | 是 |

因此採用水平夾取。

### 4.5 MoveIt 2 設定與程式控制

以 MoveIt Setup Assistant 建立 `arm` 與 `gripper` 兩個規劃群組、碰撞矩陣及預設姿勢。由於四軸手臂無法任意控制末端姿態，啟用僅位置（position-only）IK。以 Python API（moveit_py）的位置約束指定目標，容許誤差 1 cm。

### 4.6 夾取策略

結合兩者優點：自行推導的 IK 決定夾爪水平時的關節角度（控制姿態），MoveIt 規劃無碰撞的路徑。

| 步驟 | 動作 |
| --- | --- |
| 1 | 張開夾爪，移至目標後方 0.12 m 的預備點 |
| 2 | 水平前伸至目標 |
| 3 | 夾爪閉合 |
| 4 | 抬升 |
| 5 | 移至放置點 (0.30, 0.30) |
| 6 | 張開夾爪 |

夾爪採位置控制，閉合目標設為 0（位於物體內部），使手指接觸後持續施力以產生摩擦力。此為目前的開迴路施力方式，第 7 階段將以感測回饋取代。

### 4.7 自行設計手臂之 URDF 轉換

將 SolidWorks 模型以公尺為單位匯出 STL、轉為 Z 軸朝上，建立 r005_arm 套件：7 個 link、6 個旋轉關節（yaw–pitch–pitch–roll–pitch–roll）。質量與慣量以鋁（2700 kg/m³）計算。於 RViz 逐一調整關節，找出零件互不干涉的範圍，作為關節極限。

---

## 第五章 實驗結果（目前進度）

### 5.1 手臂控制與接觸感測

- 四軸手臂可由控制面板及指令驅動，並以摩擦力夾起方塊
- 接觸感測器可正確回報接觸對象、接觸點與法向量
- **觀察**：接觸力數值跳動大，不適合直接作為力控回授，後續改參考關節施力

### 5.2 障礙物迴避驗證

在 (0.35, 0, 0.15) m 加入邊長 0.08 m 的方塊障礙物，依序規劃三個目標：

| 目標 (m) | 結果 | 規劃耗時 |
| --- | --- | --- |
| (0.35, 0, 0.15) | 失敗（目標被障礙物佔據） | 約 1 秒 |
| (0, 0.35, 0.15) | 成功 | 約 0.02 秒 |
| (−0.30, 0, 0.25) | 成功 | 約 0.02 秒 |

**討論**：規劃器確實將障礙物納入碰撞檢查。成功時取樣式規劃器迅速收斂；失敗時會持續嘗試至時間上限，因此「規劃耗時長」本身可作為目標不可達的徵兆。

### 5.3 Pick & Place

| | x (m) | y (m) |
| --- | --- | --- |
| 方塊初始位置（世界座標） | 0.50 | 0 |
| 程式設定放置點 | 0.30 | 0.30 |
| 方塊最終位置 | 0.30 | 0.31 |

放置誤差約 1 cm，方塊偏航角約 1.02 rad，顯示方塊是被夾持並隨手臂旋轉，而非被推動。

### 5.4 六軸手臂關節極限

匯出工具給定之暫定極限（±120°、±170°、±180°）在 RViz 中會使 joint1、2、4、5 發生零件干涉，經逐一調整後取不干涉範圍（例：joint2 改為 ±1.85 rad）。並撰寫測試程式以正弦函數驅動六個關節，確認運動鏈正確。

### 5.5 規劃中項目

| 項目 | 驗收條件 |
| --- | --- |
| 深度感知避障 | 放置箱子擋路，手臂自行繞開 |
| 物體辨識 | 辨識物體 → MoveIt 移動並夾起 |
| 夾力控制 | 夾力穩定於設定值，加入雜訊仍不振盪 |
| 六軸手臂進入 Gazebo | 以 MoveIt 控制自行設計之手臂 |

---

## 第六章 討論

### 6.1 自由度對任務的影響

第一版自製模型只有 3 個旋轉關節（缺少肘關節），末端只能在固定半徑球面上移動；教學手臂為 4 軸，大多數末端姿態無法達成，必須使用僅位置 IK，且夾取只能採水平姿態。六軸的 r005_arm 理論上可同時控制位置與姿態，可解除上述限制。

### 6.2 模擬與現實的差距

- 模擬力量為理想物理值，真實壓力感測器（如 FSR）具非線性與漂移
- 模型質量以實心材料估算，與實際製造（中空、3D 列印）不同，會影響馬達扭力選型
- 因應：感測器以砝碼校正至牛頓、訓練時加入雜訊、以實物重新量測質量

### 6.3 開發方法

以模擬優先能在零硬體成本下驗證整條流程；分階段設定「可執行的過關條件」，避免只停留在理解層面。

---

## 第七章 結論與未來工作

### 7.1 目前結論

1. 已在模擬中建立「規劃 → 移動 → 夾取」完整流程，放置誤差約 1 cm
2. 自行推導之 IK 可用於可達性分析，並與 MoveIt 分工互補
3. 接觸感測器可可靠判斷接觸，但力量數值需另行處理
4. 自行設計之六軸手臂已能在 ROS 中正確顯示與運動

### 7.2 未來工作

1. 六軸手臂加入 ros2_control 與夾爪，進入 Gazebo 並重建 MoveIt 設定
2. 深度相機點雲接入 MoveIt Perception，產生 OctoMap 避障
3. YOLO 辨識 + 深度資訊 + TF 轉換，取得目標於 base_link 之座標
4. 夾力 PID 回饋控制與雜訊測試
5. 實機驗證：設計對照實驗，比較「影像 + 關節」與「影像 + 關節 + 壓力」兩組在軟硬物體上的夾取成功率與損壞率

---

## 參考資料

1. Open Robotics. *ROS 2 Jazzy Documentation*. https://docs.ros.org/en/jazzy/
2. MOGI-ROS. *Week 1-2: Introduction to ROS 2*. https://github.com/MOGI-ROS/Week-1-2-Introduction-to-ROS2
3. MOGI-ROS. *Week 3-4: Gazebo basics*. https://github.com/MOGI-ROS/Week-3-4-Gazebo-basics
4. MOGI-ROS. *Week 5-6: Gazebo sensors*. https://github.com/MOGI-ROS/Week-5-6-Gazebo-sensors
5. MOGI-ROS. *Week 9-10: Simple arm*. https://github.com/MOGI-ROS/Week-9-10-Simple-arm
6. Open Robotics. *Gazebo Harmonic Documentation*. https://gazebosim.org/docs/harmonic/
7. PickNik Robotics. *MoveIt 2 Documentation*. https://moveit.picknik.ai/
8. PickNik Robotics. *MoveIt Perception Pipeline Tutorial*. https://moveit.picknik.ai/main/doc/examples/perception_pipeline/perception_pipeline_tutorial.html
9. Ultralytics. *ROS Quickstart Guide*. https://docs.ultralytics.com/guides/ros-quickstart
10. Hugging Face. *LeRobot*. https://github.com/huggingface/lerobot
11. NVIDIA. *Isaac Lab Documentation*. https://isaac-sim.github.io/IsaacLab/

---

## 附錄

- A. [學習整理](notes.md)
- B. [學習歷程](portfolio.md)
- C. [指令與程式架構複習網站](https://rekisss.github.io/bot/)
