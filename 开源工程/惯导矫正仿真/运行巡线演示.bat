@echo off
cd /d "%~dp0"
echo ========================================
echo   line follow demo
echo ========================================
echo.

set "PY="
where python >nul 2>&1 && set "PY=python"
if not defined PY (
    where py >nul 2>&1 && set "PY=py -3"
)
if not defined PY (
    echo [ERROR] Python not found.
    pause
    exit /b 1
)

%PY% run_follow.py
if errorlevel 1 (
    echo.
    echo [ERROR] see traceback above.
    pause
    exit /b 1
)
