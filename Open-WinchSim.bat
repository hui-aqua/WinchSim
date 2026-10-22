@echo off
setlocal
cd /d "%~dp0viewer"
where node >nul 2>nul
if errorlevel 1 (
  echo Node.js 20.19+ or 22.12+ is required to run the WinchSim viewer.
  pause
  exit /b 1
)
node --version
set NEED_INSTALL=0
if not exist node_modules\vite\dist\node\cli.js set NEED_INSTALL=1
if not exist node_modules\three\build\three.module.js set NEED_INSTALL=1
if "%NEED_INSTALL%"=="1" (
  echo Installing viewer dependencies...
  if exist node_modules\vite rmdir /s /q node_modules\vite
  if exist node_modules\three rmdir node_modules\three
  call npm install --no-audit --no-fund
  if errorlevel 1 (
    echo.
    echo Dependency installation failed. Check that Node.js and npm are installed.
    pause
    exit /b 1
  )
)
if not exist node_modules\vite\dist\node\cli.js (
  echo.
  echo Vite is still incomplete after installation. Check the npm error above.
  pause
  exit /b 1
)
if not exist node_modules\three\build\three.module.js (
  echo.
  echo Three.js is still incomplete after installation. Check the npm error above.
  pause
  exit /b 1
)
start "WinchSim server" cmd /k "node_modules\.bin\vite.cmd --host 127.0.0.1 --port 8000 --strictPort"
powershell -NoProfile -Command "$ready = $false; for ($i = 0; $i -lt 30; $i++) { try { $r = Invoke-WebRequest -UseBasicParsing http://127.0.0.1:8000/ -TimeoutSec 2; if ($r.StatusCode -eq 200) { $ready = $true; break } } catch { Start-Sleep -Seconds 1 } }; if ($ready) { exit 0 } else { exit 1 }"
if errorlevel 1 (
  echo.
  echo The viewer did not respond at http://127.0.0.1:8000/ after 30 seconds.
  echo Check the WinchSim server window for the startup error.
  pause
  exit /b 1
)
start "WinchSim viewer" "http://127.0.0.1:8000/"
