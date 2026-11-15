@echo off
rem Lite2 实机调试上位机启动脚本（Windows）
rem 默认离线演示；实机部署请把 LINK 改成 ssh_ros 并设置 SSH 口令环境变量。
setlocal
cd /d %~dp0\..

set LINK=mock
set HOST=127.0.0.1
set PORT=8710

if not "%~1"=="" set LINK=%~1
if not "%~2"=="" set HOST=%~2
if not "%~3"=="" set PORT=%~3

if "%LINK%"=="ssh_ros" (
  if "%HOST_UI_SSH_PASSWORD%"=="" (
    echo [警告] 未设置 HOST_UI_SSH_PASSWORD，SSH 认证可能失败。
    echo        在 PowerShell 中执行: $env:HOST_UI_SSH_PASSWORD = "你的密码"
    echo.
  )
)

echo 启动上位机: link=%LINK% host=%HOST% port=%PORT%
python host_ui\run.py --link %LINK% --host %HOST% --port %PORT%
if errorlevel 1 pause
endlocal