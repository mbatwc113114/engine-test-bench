@echo off
setlocal

title DAQ Application Installer

echo ========================================
echo       DAQ Application Installer
echo ========================================
echo.

cd /d "%~dp0"

echo [1/4] Checking Python...

where python >nul 2>&1

if errorlevel 1 (
    echo.
    echo Python is not installed.
    echo Please install Python 3.11 or newer.
    echo.
    echo Download from:
    echo https://www.python.org/downloads/
    pause
    exit /b 1
)

python --version
echo.

echo [2/4] Creating virtual environment...

if not exist ".venv\Scripts\python.exe" (
    python -m venv .venv
)

echo.
echo [3/4] Installing required packages...

".venv\Scripts\python.exe" -m pip install --upgrade pip

".venv\Scripts\python.exe" -m pip install -r requirements.txt

echo.
echo [4/4] Creating shortcut...

powershell -ExecutionPolicy Bypass -Command "$WshShell = New-Object -ComObject WScript.Shell; $Shortcut = $WshShell.CreateShortcut([Environment]::GetFolderPath('Desktop') + '\DAQ Application.lnk'); $Shortcut.TargetPath = '%~dp0launcher.bat'; $Shortcut.WorkingDirectory = '%~dp0'; $Shortcut.IconLocation = '%~dp0assets\icon.ico'; $Shortcut.Save()"

echo.
echo ========================================
echo       Installation Complete!
echo ========================================
echo.
echo Desktop shortcut created.
echo.
pause