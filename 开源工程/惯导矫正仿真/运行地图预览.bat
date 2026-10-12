@echo off
chcp 65001 >nul
cd /d "%~dp0"

echo ========================================
echo   解析并绘制 地图.DXF
echo ========================================
echo.

where py >nul 2>&1
if %errorlevel%==0 (
    py -3 plot_map.py
    goto :after
)

where python >nul 2>&1
if %errorlevel%==0 (
    python plot_map.py
    goto :after
)

echo [错误] 未找到 Python。请先安装 Python 并勾选 "Add python.exe to PATH"。
pause
exit /b 1

:after
if errorlevel 1 (
    echo.
    echo [错误] 运行失败，请检查上方信息。
    pause
    exit /b 1
)

echo.
echo 完成。预览图已保存为 地图预览.png
pause
