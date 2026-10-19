# 機械手臂指令小抄

ROS 2 Jazzy / Gazebo Harmonic / MoveIt 專題的複習網站：

- **指令**：照「要做什麼」分類，外加 Windows ↔ WSL 搬檔案的方法
- **程式架構**：工作區和套件的資料夾結構、launch 啟動流程、訊息怎麼流
- **每個檔案在做什麼**：xacro、.gazebo、控制器 yaml、bridge、launch、Python node 的原理和常改的地方
- **複製模組**：新增關節、感測器、相機、launch node、Python node 的範本

**網站**：https://rekisss.github.io/bot/

## 專題文件

| 文件 | 內容 |
| --- | --- |
| [學習整理](docs/notes.md) | ROS 2、URDF/TF、Gazebo 感測器、ros2_control、逆運動學、MoveIt、CAD 轉 URDF、AI 訓練路線、除錯方法的觀念筆記 |
| [學習歷程](docs/portfolio.md) | 從環境建置到 pick & place 的時間軸、遇到的困難、反思與能力成長 |
| [專題論文（期中草稿）](docs/thesis.md) | 摘要、系統架構、研究方法、目前實驗結果、討論與未來工作 |

## 怎麼改內容

所有內容都在 `index.html` 裡 `<script type="application/json" id="data">` 那一段，一張卡片長這樣：

```json
{
  "id": "c99",
  "where": "term",
  "title": "卡片標題",
  "desc": "說明文字",
  "blocks": [{ "label": "終端機 1", "code": "ros2 launch ..." }],
  "note": "提醒"
}
```

每個分類還有 `group`（目錄分組：指令／程式架構／複製模組）。`where` 可以填：`term`（Ubuntu 終端機）、`ps`（PowerShell）、`exp`（Windows 檔案總管）、`file`（VS Code 檔案）、`none`（不用指令）。

改法二選一：

1. **GitHub 網頁上改**：點 `index.html` → 右上角鉛筆 → 改完按 Commit changes，一兩分鐘後網站就更新。
2. **叫 Claude 改**：跟 Claude 說要改什麼，它會直接改好推上來。

> 注意：JSON 裡的 `<` 要寫成 `<`，雙引號要寫成 `\"`。
