@echo off
chcp 65001 >nul
cd /d "%~dp0"
python -c "import PyQt5" >nul 2>nul
if errorlevel 1 (
  echo 尚未安装 PyQt5。
  echo 请先运行: python -m pip install -r requirements-host.txt
  pause
  exit /b 1
)
python scara_pyqt_host.py
