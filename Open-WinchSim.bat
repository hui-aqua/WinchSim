@echo off
setlocal
cd /d "%~dp0viewer"
where node >nul 2>nul
if errorlevel 1 (
  echo Node.js 22 or newer is required to run the WinchSim viewer.
  pause
  exit /b 1
)
if not exist node_modules\vite\bin\vite.js (
  echo Installing viewer dependencies...
  call npm install --no-audit --no-fund
  if errorlevel 1 exit /b 1
)
start "WinchSim viewer" "http://127.0.0.1:8000/"
node_modules\.bin\vite.cmd --host 127.0.0.1 --port 8000 --strictPort
