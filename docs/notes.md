# 機械手臂專題｜學習整理

> 整理期間：2026-09-30 ～ 2026-10-19
> 環境：Windows 11 + WSL2 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic、MoveIt 2
> 指令小抄請看 [複習網站](https://rekisss.github.io/bot/)；這份只整理**觀念**和**為什麼**。

---

## 目錄

1. [開發環境](#1-開發環境)
2. [ROS 2 核心觀念](#2-ros-2-核心觀念)
3. [機器人描述：URDF / xacro / TF](#3-機器人描述urdf--xacro--tf)
4. [Gazebo 模擬與感測器](#4-gazebo-模擬與感測器)
5. [ros2_control：讓模擬手臂真的動](#5-ros2_control讓模擬手臂真的動)
6. [逆運動學（IK）](#6-逆運動學ik)
7. [MoveIt 2 軌跡規劃](#7-moveit-2-軌跡規劃)
8. [Pick & Place](#8-pick--place)
9. [自己的手臂：CAD → URDF](#9-自己的手臂cad--urdf)
10. [AI 訓練路線：模仿學習 vs 強化學習](#10-ai-訓練路線模仿學習-vs-強化學習)
11. [除錯方法論](#11-除錯方法論)
12. [還沒學、接下來要學的](#12-還沒學接下來要學的)

---

## 1. 開發環境

| 項目 | 選擇 | 為什麼 |
| --- | --- | --- |
| 作業系統 | WSL2 + Ubuntu **24.04** | 不用灌雙系統；ROS 2 Jazzy 只支援 24.04（Microsoft Store 預設會裝到 26.04，要指定版本） |
| ROS 版本 | Jazzy（LTS） | 長期支援，MOGI 教材也是 Jazzy |
| 模擬器 | Gazebo Harmonic | Jazzy 官方搭配版本 |
| GPU 算圖 | `GALLIUM_DRIVER=d3d12` | WSL2 預設用 CPU 算圖（llvmpipe），會非常卡 |
| Python 環境 | ROS 用系統 Python；LeRobot 用 conda | conda 的 Python 會蓋掉 ROS 的，跑 ROS 前要 `conda deactivate` |

**學到的事**：環境問題佔了前期一半的時間。最常見的是「開錯視窗」（Ubuntu 26.04 / PowerShell）和「新終端機沒 source」。

---

## 2. ROS 2 核心觀念

### 2.1 node、topic、message

```
[node A] ──publish──▶ /topic 名稱 ──subscribe──▶ [node B]
```

- **node**：一支只做一件事的程式（例如「讀相機」、「算 IK」）
- **topic**：有名字的管線，誰都可以發、誰都可以聽
- **message**：管線裡傳的資料格式（`String`、`JointState`、`Image`…）

比喻：topic 像群組聊天室。同一個聊天室裡**只能有一個人負責發關節角度**，否則手臂會在兩個角度之間抽搐（滑桿 GUI 和自己的 `dance.py` 同時發 `/joint_states` 就會這樣）。

### 2.2 service、parameter、launch

| 機制 | 特性 | 用在哪 |
| --- | --- | --- |
| topic | 單向、持續 | 感測器資料、關節狀態 |
| service | 一問一答 | 「幫我生一顆球」、控制器開關 |
| parameter | node 的設定值，可執行中修改 | 影像壓縮品質、`use_sim_time` |
| launch 檔 | 一張「要開哪些 node」的清單 | 一個指令開 Gazebo + RViz + bridge + 控制器 |

### 2.3 套件與工作區

- **工作區**：`~/ros2_ws`（MOGI 練習）、`~/arm_ws`（專題）。`src/` 是倉庫，`install/` 是店面；RViz、Gazebo 只去店面拿東西
- `colcon build --symlink-install`：新增檔案、改 `setup.py` 一定要重 build；只改內容可以省掉 build，但**不能省掉重開 launch**
- `setup.py` 的 `entry_points`：不登記，`ros2 run` 就找不到你的程式
- `CMakeLists.txt` 的 `install(DIRECTORY ...)`：沒列出來的資料夾不會被裝進 `install/`

---

## 3. 機器人描述：URDF / xacro / TF

### 3.1 URDF = 零件 + 關節串成一棵樹

```
world ─(fixed)─ base_link ─(revolute)─ link1 ─(revolute)─ link2 ─ … ─ 夾爪
```

| 標籤 | 意思 |
| --- | --- |
| `<link>` | 一個零件：`visual`（長怎樣）、`collision`（碰撞形狀）、`inertial`（質量、慣量） |
| `<joint>` | 兩個零件的連接：`type`（fixed / revolute / prismatic）、`origin`（在父零件的哪裡）、`axis`（繞哪個軸轉）、`limit` |

- **沒有 `inertial` 的零件在 Gazebo 會被丟掉**
- 第一個 link 要用 fixed joint 釘在 `world`，手臂才不會倒；KDL 也不接受根部 link 有質量
- xacro 是 URDF 加上變數和 include，方便拆檔

### 3.2 加任何感測器的三步驟

1. `.urdf`：感測器**裝在哪**（link + fixed joint）→ 換實機時保留
2. `.gazebo`：**模擬**它的規格（解析度、更新率、雜訊）→ 換實機時改成真驅動
3. bridge：加一行 topic，ROS 才收得到

### 3.3 TF 座標轉換

- 每個 link 都有自己的座標系，TF 負責算「A 座標系裡的點，在 B 座標系是哪裡」
- **同一段轉換只能有一個 node 發布**（加 EKF 時要把 bridge 的 `/tf` 關掉）
- 相機有兩個慣例：ROS 是 X 前 / Y 左 / Z 上；影像是 X 右 / Y 下 / Z 前，所以有 `camera_link_optical`
- Gazebo 給的是**世界座標**，MoveIt 用的是 **base_link 座標**，兩者要相減（例：方塊世界座標 z = 1.06，手臂底座 z = 1.04，手臂眼中方塊高 0.02）

### 3.4 RViz vs Gazebo

| 工具 | 顯示什麼 |
| --- | --- |
| Gazebo | 真實世界（物理、重力、碰撞） |
| RViz | 機器人「以為」的世界（只靠感測器算） |

車子卡牆時 Gazebo 停住、RViz 繼續跑，因為輪子編碼器和 IMU 分不出「等速前進」和「卡住」。

---

## 4. Gazebo 模擬與感測器

| 感測器 | 學到的重點 |
| --- | --- |
| 相機 | image transport 壓縮影像省頻寬；RViz 看壓縮影像要 relay `camera_info` |
| IMU | 量「怎麼動」不知道「在哪」；靜止 z 軸讀 9.8 m/s²；會漂移 → 要和輪子里程計用 **EKF 融合** |
| RGBD | 點雲；Decay Time 太大 RViz 只剩 2-3 fps |
| 接觸感測器 | 左手指加 contact sensor，能讀到碰到誰、接觸點、法向量和力；**碰撞判斷可靠，但力的數值不穩**，力控之後改參考關節 effort |

**OpenCV 追紅球（chase_the_ball.py）**：一個 node 同時聽 `/camera/image`、說 `/cmd_vel`。

```
只留紅色（HSV 遮罩）→ 找最大輪廓 → 算重心 → 跟畫面中心比 → 決定左轉或右轉
```

這是第 6 階段「物體辨識 → 3D 座標」的前身。

---

## 5. ros2_control：讓模擬手臂真的動

```
controller_manager（總機，1000 Hz 迴圈）
├── joint_state_broadcaster  只讀：把關節角度發到 /joint_states
└── arm_controller           只寫：收軌跡指令 → 推關節
```

- URDF 要有 `<ros2_control>` 區塊、`.gazebo` 要掛 gz_ros2_control 外掛，兩者缺一 controller_manager 就叫不醒
- 啟動訊息看到 `Configured and activated arm_controller` 才算成功
- 位置控制器像一根彈簧：**夾爪目標設在方塊「裡面」（0.0）**，手指碰到後持續往內推才有夾力；設在方塊表面就不出力、方塊會滑掉。這是「粗暴的力控」，第 7 階段要換成感測器 + PID

---

## 6. 逆運動學（IK）

### 6.1 問題

正運動學：給關節角度 → 算末端位置（簡單，一路乘下去）
逆運動學：給末端位置 → 算關節角度（難，可能多解或無解）

### 6.2 MOGI 4 軸手臂的幾何解法

1. **底座朝向**：`shoulder_pan = atan2(y, x)`
   - 一開始用 `atan(y/x)`，目標在側邊（x = 0）和背後（x < 0）時出錯；`atan2` 會看 x、y 的正負號，分得出四個象限
2. **扣掉夾爪長度**：先由目標點和夾爪朝向（水平 / 朝下），往回推 0.175 m 得到**手腕位置**
3. **兩連桿平面問題**：肩到手腕的距離 c，用餘弦定理求肘角

$$\cos\gamma = \frac{L_1^2 + L_2^2 - c^2}{2 L_1 L_2}$$

   γ 是兩節手臂之間的夾角，肘關節角度再由 γ 換算。

4. **可達性**：c 大於兩節手臂總長（0.45 m）時，等號右邊會小於 −1，`acos` 算不出來，Python 噴 `math domain error`，意思就是「手太短」

### 6.3 可達性分析的實際例子

目標方塊在手臂眼中 (0.50, 0, 0.02)，肩膀高 0.1 m：

| 夾法 | 手腕位置 | c | 結果 |
| --- | --- | --- | --- |
| 夾爪朝下 | 方塊正上方 0.175 | ≈ 0.509 | ❌ > 0.45，到不了 |
| 夾爪水平 | 方塊後方 0.175 | ≈ 0.335 | ✅ 夾得到 |

**動手前先算一次可達性**，比讓 MoveIt 一直報錯有效率得多。

---

## 7. MoveIt 2 軌跡規劃

### 7.1 規劃 ≠ 預測

MoveIt 是用演算法（OMPL 的 RRTConnect + IK 求解器）**搜尋**一條不撞的路徑，不是用資料訓練出來的模型，所以這一環「不用訓練」。

### 7.2 Setup Assistant 產生了什麼

| 設定 | 意思 |
| --- | --- |
| planning group | `arm`（4 個關節）、`gripper`（兩根手指） |
| 碰撞矩陣（ACM） | 哪些零件本來就會貼在一起、不用檢查 |
| 預設姿勢 | `home`、`open`、`closed` |
| `position_only_ik` | 4 軸手臂大部分朝向都做不到，只要求位置 |
| `moveit_controllers.yaml` | 名字要和 `ros2 control list_controllers` 一模一樣 |

### 7.3 move_group vs MoveItPy

- **move_group**：獨立的規劃伺服器，RViz 的 MotionPlanning 外掛靠它畫手臂和場景
- **MoveItPy**：把一個小型 move_group 包進 Python 程式裡；兩個同時開時，障礙物要廣播到 `/collision_object`，RViz 才看得到

### 7.4 用程式控制

```python
target = construct_link_constraint(
    link_name='end_effector_link', source_frame='base_link',
    cartesian_position=[x, y, z], cartesian_position_tolerance=0.01)
arm.set_start_state_to_current_state()
arm.set_goal_state(motion_plan_constraints=[target])
plan_result = arm.plan()
robot.execute(plan_result.trajectory, controllers=[])
```

重複的步驟包成 `move_to(x, y, z)`，用 list + for 迴圈走多個點。

### 7.5 障礙物實驗

在 (0.35, 0, 0.15) 放一個 8 cm 方塊，再叫手臂去那裡：

| 目標 | 結果 | 花費時間 |
| --- | --- | --- |
| (0.35, 0, 0.15) | ❌ 規劃失敗（被箱子佔住） | 約 1 秒 |
| (0, 0.35, 0.15) | ✅ | 約 0.02 秒 |
| (−0.30, 0, 0.25) | ✅ | 約 0.02 秒 |

**觀察**：成功時 OMPL 很快找到路；失敗時會試到時間用完。「卡很久才失敗」通常代表目標被擋住或根本到不了。

---

## 8. Pick & Place

分工：**自己寫的 IK** 算出「夾爪水平」的關節角度（控制朝向），**MoveIt** 負責規劃不撞的路徑。

| 步驟 | 動作 |
| --- | --- |
| ① | 張開夾爪，移到方塊後方 12 cm 的預備點 |
| ② | 水平往前伸到方塊位置 |
| ③ | 夾爪閉合（目標 0.0，持續施力） |
| ④ | 抬起 |
| ⑤ | 移到放置點 (0.30, 0.30) |
| ⑥ | 放開 |

結果：方塊從世界座標 (0.50, 0) 搬到 (0.30, 0.31)，**與目標相差約 1 cm**，Yaw 變成約 58°，代表是被夾著轉過去的。

---

## 9. 自己的手臂：CAD → URDF

### 9.1 三種路線

| 路線 | 優點 | 缺點 |
| --- | --- | --- |
| 手動匯出 STL + 手寫 URDF | 每一行都看得懂 | 座標、慣量要自己算 |
| SolidWorks 外掛（SW2GZ 等） | 質量慣量自動帶出 | 要學外掛的設定 |
| Inventor → STEP → 轉檔 | 不用重畫 | Fusion 轉完零件散開 |

### 9.2 匯出 STL 的關鍵設定

- 單位選**公尺**（選公釐會大 1000 倍）
- 勾「不轉換 STL 輸出資料到正向座標空間」，零件才保持在組合件裡的位置
- 一個 link 一個檔
- SolidWorks 預設 Y 朝上，ROS 是 Z 朝上，要繞 X 轉 90°

### 9.3 第一版 arm_description 的教訓

| 問題 | 學到 |
| --- | --- |
| 只有 3 個旋轉關節，少了手肘 | 末端只能在球面上，MoveIt 到不了大部分目標；**自由度要先數清楚** |
| 關節軸是從圓柱推出來的 | 配合條件只有「重合 / 同軸」時，軸向不一定對 |
| 總重約 26 kg | 實心 PLA 估算太重，Gazebo 會抬不起來 |

### 9.4 r005_arm（6 軸）

- 7 個 link、6 個 revolute 關節（yaw / pitch / pitch / roll / pitch / roll）
- STL 已是公尺、Z 朝上；質量以鋁 2700 kg/m³ 計算，若 3D 列印 PLA 要 ×0.46
- RViz 拉滑桿發現 joint1、2、4、5 拉到極限會撞到 → 自己逐一找出不撞的範圍，改掉匯出工具的暫定值（例：joint2 改成 ±1.85 rad）
- **關節極限 = min(零件不會撞的範圍, 馬達能轉的範圍)**
- 寫 `dance.py`：自己當 `/joint_states` 的發送者，用 `A·sin(t)` 讓 6 個關節自己擺動；振幅不能超過 limit

---

## 10. AI 訓練路線：模仿學習 vs 強化學習

| | 模仿學習（LeRobot：ACT、Diffusion Policy） | 強化學習（Isaac Lab：PPO） |
| --- | --- | --- |
| 資料從哪來 | 人遙控示範 | 模擬器裡自己試錯 |
| 適合 | 實機 | GPU 平行跑上千個環境 |
| 難點 | 示範品質 | sim-to-real gap |

- Gazebo 照真實時間跑、一次一個環境，**不適合 RL 訓練**；它的角色是學運動學、驗證流程
- 壓力感測器在訓練裡可以當：觀測輸入（最簡單，先做這個）、RL 的 reward、直接輸出
- FSR 很「爛」：非線性、會漂移 → 固定安裝位置、先用砝碼校正、訓練和執行用同一顆
- 模擬的力是乾淨的牛頓值，真實是有雜訊的 ADC 讀值 → 校正成牛頓，或訓練時加雜訊（domain randomization）
- 對照實驗設計：A 組「影像 + 關節」vs B 組「影像 + 關節 + 壓力」，比較夾軟硬物體的成功率與捏壞率

---

## 11. 除錯方法論

1. **先偵察再動手**：`ls`、`cat README`、`ros2 node list` 花 10 秒看清楚
2. **兩邊對照找斷點**：`gz topic -l` 和 `ros2 topic list` 一起看，判斷是 URDF 還是 bridge 的問題
3. **改完先驗證**：`xacro ... > /dev/null && echo "URDF OK"`
4. **用行號定位**：`cat -n 檔案 | sed -n '100,120p'`
5. **懷疑背景程式**：改了沒反應又很卡 → `ps aux | grep "gz sim"`，殭屍 Gazebo 會讓新模型生到舊世界
6. **過濾雜訊**：`... 2>&1 | grep -E "關鍵字"`
7. **看時間戳**：規劃花多久，本身就是線索

常見錯誤對照表見 [學習手冊](https://rekisss.github.io/bot/) 的「卡住時」分類。

---

## 12. 還沒學、接下來要學的

| 階段 | 內容 | 狀態 |
| --- | --- | --- |
| 5 | RGBD 點雲 → MoveIt Perception → OctoMap，讓手臂自己繞過障礙 | 點雲會接，OctoMap 未做 |
| 6 | YOLO 辨識 → 像素 + 深度 → TF 轉到 base_link → MoveIt 夾取 | 未開始 |
| 7 | 接觸感測器 + PID 控夾力，加雜訊測試 | 感測器已裝，控制未寫 |
| — | r005_arm 進 Gazebo（world link、ros2_control）→ 重跑 Setup Assistant（6 軸） | 下一步 |
| — | 加夾爪、實機採購、hand-eye 校正 | 模擬做完後 |
