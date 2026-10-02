# -*- coding: utf-8 -*-
"""地图编辑器本地服务:双击 启动地图编辑器.bat 使用。

GET  /             静态文件(map_editor.html 等)
GET  /save-info    告诉网页后端在线,返回可写目标列表
POST /save         {name, content} → 按白名单直接写进车上工程

网页检测到本服务后,"下载 nav_path.h"按钮变为"写入车上工程",
一键把生成结果落盘到 Crazy_Car 的 code/ 目录,之后去 ADS 编译即可。
"""
import json
import sys
import threading
import time
import webbrowser
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PORT = 8623

# 允许写入的目标(白名单):文件名 → 绝对路径。想写别的文件在这里加。
TARGETS = {
    "nav_path.h": ROOT.parent / "Crazy_Car" / "Seekfree_TC264_Opensource_Library" / "code" / "nav_path.h",
    "map_config.json": ROOT / "map_config.json",
}


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def _json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/save-info":
            self._json({"targets": {k: str(v) for k, v in TARGETS.items()}})
            return
        super().do_GET()

    def do_POST(self):
        if self.path != "/save":
            self.send_error(404)
            return
        try:
            n = int(self.headers.get("Content-Length", 0))
            data = json.loads(self.rfile.read(n).decode("utf-8"))
            target = TARGETS.get(data.get("name", ""))
            if target is None:
                self._json({"ok": False, "error": "文件名不在白名单里"}, 403)
                return
            target.write_text(data["content"], encoding="utf-8", newline="\n")
            # 回读校验:读回来和收到的内容逐字节比对,不一致直接报错
            back = target.read_text(encoding="utf-8")
            if back != data["content"]:
                self._json({"ok": False, "error": "回读校验不一致,文件可能被占用"}, 500)
                return
            st = target.stat()
            self._json({
                "ok": True,
                "path": str(target),
                "bytes": st.st_size,
                "mtime": time.strftime("%H:%M:%S", time.localtime(st.st_mtime)),
            })
        except Exception as e:  # noqa: BLE001 局部工具,报错原样回给网页
            self._json({"ok": False, "error": str(e)}, 500)

    def log_message(self, fmt, *args):
        pass  # 不刷终端


if __name__ == "__main__":
    url = f"http://localhost:{PORT}/map_editor.html"
    if "--no-browser" not in sys.argv:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    print(f"地图编辑器已启动: {url}")
    print(f"nav_path.h 将写入: {TARGETS['nav_path.h']}")
    print("关闭本窗口即退出。")
    try:
        # 多线程:浏览器会开预连接空占连接,单线程服务器会被堵死
        ThreadingHTTPServer(("127.0.0.1", PORT), Handler).serve_forever()
    except OSError:
        print(f"端口 {PORT} 被占用:大概已经开着一个了,直接用浏览器里那个页面即可。")
        input("按回车退出...")
