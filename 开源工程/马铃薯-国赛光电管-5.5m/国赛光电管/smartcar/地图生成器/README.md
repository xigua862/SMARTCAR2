# 智能车地图生成器（区赛快速/精确求解版）

这是区赛工作区使用的本地地图编辑器。网页负责编辑地图、路线和速度约束，Python 服务负责保存白名单文件并调用路线求解器。当前版本在 `8642` 端口运行，精确后端和有界快速回退均使用 HiGHS。

## 环境

- Python 3.10 或更高版本
- Windows、Linux 或 macOS 均可运行服务；一键启动脚本面向 Windows

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

## 启动

Windows 可双击 `启动地图编辑器.bat`，或在终端运行：

```powershell
python map_server.py
```

服务仅监听本机，浏览器打开 `http://localhost:8642/map_editor.html`。不要把该端口暴露到公网；保存接口没有设计成公共网络 API。

## 与固件的目录关系

一键写入按以下相对布局寻找固件：

```text
smartcar/
├─ 地图生成器_快速解法/
└─ Crazy_Car/
   └─ Seekfree_TC264_Opensource_Library/
      └─ code/nav_path.h
```

路线、速度或物理特征发生变化时，应先保存地图 JSON，再通过编辑器重新导出 `nav_path.h`。不要手工修改生成数组。

## 主要文件

| 文件 | 用途 |
| --- | --- |
| `map_editor.html` | 单页地图编辑器 |
| `map_server.py` | 本地 HTTP 服务、白名单保存和求解任务 |
| `fast_route_solver.py` | 有界快速求解器及 HiGHS 后端 |
| `exact_route_solver.py` | 精确求解接口 |
| `yusai.json` / `juesai.json` | 当前地图源数据 |
| `old/` | 历史地图配置，仅用于追溯 |

## 许可证

本工具按 `GPL-3.0-only` 发布，见 `LICENSE`。HiGHS/highspy 通过 Python 环境安装并使用 MIT License，详见 `THIRD_PARTY_NOTICES.md`。
