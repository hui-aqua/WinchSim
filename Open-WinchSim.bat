@echo off
setlocal
cd /d "%~dp0"
where py >nul 2>nul
if %errorlevel%==0 (
  start "WinchSim viewer" "http://127.0.0.1:8000/viewer/viewer.html"
  py -m http.server 8000 --bind 127.0.0.1
  goto :eof
)
where python >nul 2>nul
if %errorlevel%==0 (
  start "WinchSim viewer" "http://127.0.0.1:8000/viewer/viewer.html"
  python -m http.server 8000 --bind 127.0.0.1
  goto :eof
)
echo Python was not found. Install Python 3 and run this file again.
pause
