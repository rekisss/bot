# 機械手臂指令小抄

ROS 2 Jazzy / Gazebo Harmonic / MoveIt 專題用過的指令，照「要做什麼」分類，外加 Windows ↔ WSL 搬檔案的方法。

**網站**：https://rekisss.github.io/bot/

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

`where` 可以填：`term`（Ubuntu 終端機）、`ps`（PowerShell）、`exp`（Windows 檔案總管）、`file`（VS Code 檔案）、`none`（不用指令）。

改法二選一：

1. **GitHub 網頁上改**：點 `index.html` → 右上角鉛筆 → 改完按 Commit changes，一兩分鐘後網站就更新。
2. **叫 Claude 改**：跟 Claude 說要改什麼，它會直接改好推上來。

> 注意：JSON 裡的 `<` 要寫成 `<`，雙引號要寫成 `\"`。
