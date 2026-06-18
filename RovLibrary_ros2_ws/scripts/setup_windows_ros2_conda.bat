@echo off

set "ENV_NAME=ros2"
set "PYTHON_VERSION=3.8.3"
set "WORKSPACE_DIR=%~dp0.."
for %%I in ("%WORKSPACE_DIR%") do set "WORKSPACE_DIR=%%~fI"

echo.
echo [INFO] RovLibrary ROS 2 Windows setup
echo [INFO] Workspace: %WORKSPACE_DIR%
echo [INFO] Conda env: %ENV_NAME%  Python: %PYTHON_VERSION%
echo.

where conda >nul 2>nul
if errorlevel 1 (
  echo [ERROR] conda was not found on PATH.
  echo [ERROR] Open an Anaconda Prompt, or install Miniconda/Anaconda and retry.
  exit /b 1
)

for /f "delims=" %%I in ('conda info --base') do set "CONDA_BASE=%%I"
if not exist "%CONDA_BASE%\Scripts\activate.bat" (
  echo [ERROR] Could not find conda activate script at:
  echo [ERROR]   %CONDA_BASE%\Scripts\activate.bat
  exit /b 1
)

call "%CONDA_BASE%\Scripts\activate.bat"
if errorlevel 1 (
  echo [ERROR] Failed to initialize conda.
  exit /b 1
)

conda env list | findstr /R /C:"^%ENV_NAME%[ ][ ]*" >nul 2>nul
if errorlevel 1 (
  echo [INFO] Creating conda env "%ENV_NAME%" with Python %PYTHON_VERSION%...
  call conda create -y -n "%ENV_NAME%" python=%PYTHON_VERSION%
  if errorlevel 1 (
    echo [ERROR] Failed to create conda env "%ENV_NAME%".
    exit /b 1
  )
) else (
  echo [INFO] Using existing conda env "%ENV_NAME%".
)

call conda activate "%ENV_NAME%"
if errorlevel 1 (
  echo [ERROR] Failed to activate conda env "%ENV_NAME%".
  exit /b 1
)

for /f "tokens=2" %%V in ('python --version 2^>^&1') do set "ACTIVE_PYTHON=%%V"
if not "%ACTIVE_PYTHON%"=="%PYTHON_VERSION%" (
  echo [ERROR] Conda env "%ENV_NAME%" is using Python %ACTIVE_PYTHON%, expected %PYTHON_VERSION%.
  echo [ERROR] Remove/recreate the env, for example:
  echo [ERROR]   conda env remove -n %ENV_NAME%
  echo [ERROR]   %~nx0
  exit /b 1
)

echo [INFO] Installing pip, setuptools, wheel, and colcon...
python -m pip install --upgrade "pip<25" "setuptools<76" "wheel"
if errorlevel 1 exit /b 1

python -m pip install --upgrade colcon-common-extensions
if errorlevel 1 exit /b 1

echo [INFO] Installing workspace Python requirements...
python -m pip install --upgrade -r "%WORKSPACE_DIR%\requirements.txt"
if errorlevel 1 exit /b 1

call :FindRos2Setup
if not defined ROS2_SETUP_BAT_FOUND (
  echo.
  echo [ERROR] ROS 2 Humble was not found.
  echo [ERROR] Install/source ROS 2 Humble for Windows, or set ROS2_SETUP_BAT to the setup file.
  echo [ERROR] Examples:
  echo [ERROR]   set ROS2_SETUP_BAT=C:\dev\ros2_humble\local_setup.bat
  echo [ERROR]   set ROS2_SETUP_BAT=C:\opt\ros\humble\x64\setup.bat
  echo.
  echo [INFO] Python dependencies were installed before this ROS 2 check failed.
  exit /b 2
)

echo [INFO] Sourcing ROS 2 setup:
echo [INFO]   %ROS2_SETUP_BAT_FOUND%
call "%ROS2_SETUP_BAT_FOUND%"
if errorlevel 1 (
  echo [ERROR] Failed while sourcing ROS 2 setup.
  exit /b 1
)

if /I not "%ROS_DISTRO%"=="humble" (
  echo [ERROR] ROS_DISTRO is "%ROS_DISTRO%", expected "humble".
  echo [ERROR] Make sure the ROS 2 Humble setup file is sourced.
  exit /b 2
)

where ros2 >nul 2>nul
if errorlevel 1 (
  echo [ERROR] ROS 2 setup was sourced, but ros2 is still not available on PATH.
  exit /b 2
)

echo [INFO] Running preflight...
python "%WORKSPACE_DIR%\scripts\preflight_ros2_windows.py"
if errorlevel 1 exit /b 1

echo.
echo [OK] Windows ROS 2 conda setup completed.
echo [OK] Next:
echo [OK]   cd /d "%WORKSPACE_DIR%"
echo [OK]   colcon build --symlink-install
echo [OK]   call install\setup.bat
exit /b 0

:FindRos2Setup
set "ROS2_SETUP_BAT_FOUND="
if defined ROS2_SETUP_BAT (
  if exist "%ROS2_SETUP_BAT%" (
    set "ROS2_SETUP_BAT_FOUND=%ROS2_SETUP_BAT%"
    exit /b 0
  )
)

for %%P in (
  "C:\dev\ros2_humble\local_setup.bat"
  "C:\dev\ros2_humble\setup.bat"
  "C:\opt\ros\humble\x64\setup.bat"
  "C:\ros2_humble\local_setup.bat"
  "%ProgramFiles%\ros2_humble\local_setup.bat"
) do (
  if exist "%%~P" (
    set "ROS2_SETUP_BAT_FOUND=%%~P"
    exit /b 0
  )
)
exit /b 0
