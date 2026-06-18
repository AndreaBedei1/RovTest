@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "WORKSPACE_DIR=%~dp0.."
for %%I in ("%WORKSPACE_DIR%") do set "WORKSPACE_DIR=%%~fI"

call :FindRos2Setup
call :InferRosDefaults

echo.
echo [INFO] RovLibrary ROS 2 Windows setup
echo [INFO] Workspace: %WORKSPACE_DIR%
echo [INFO] Conda env: %ENV_NAME%  Python: %PYTHON_VERSION%
echo [INFO] Expected ROS_DISTRO: %ROS_DISTRO_EXPECTED%
if defined ROS2_SETUP_BAT_FOUND (
  echo [INFO] ROS 2 setup: %ROS2_SETUP_BAT_FOUND%
) else (
  echo [INFO] ROS 2 setup: not found yet
)
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
  echo [ERROR] Remove/recreate the env, or override ENV_NAME/PYTHON_VERSION intentionally.
  echo [ERROR] Example:
  echo [ERROR]   conda env remove -n %ENV_NAME%
  echo [ERROR]   %~nx0
  exit /b 1
)

echo [INFO] Installing pip, setuptools, wheel, and colcon...
if "%PYTHON_VERSION%"=="3.8.3" (
  python -m pip install --upgrade "pip<25" "setuptools<76" "wheel"
) else (
  python -m pip install --upgrade pip "setuptools<80" wheel
)
if errorlevel 1 exit /b 1

python -m pip install --upgrade colcon-common-extensions
if errorlevel 1 exit /b 1

if /I "%ROS_DISTRO_EXPECTED%"=="lyrical" (
  echo [INFO] Installing conda yaml runtime for ROS 2 Lyrical...
  call conda install -y -c conda-forge yaml
  if errorlevel 1 (
    echo [ERROR] Failed to install the conda yaml runtime. ROS 2 Lyrical needs yaml.dll.
    exit /b 1
  )
)

echo [INFO] Installing workspace Python requirements...
python -m pip install --upgrade -r "%WORKSPACE_DIR%\requirements.txt"
if errorlevel 1 exit /b 1

if not defined ROS2_SETUP_BAT_FOUND (
  echo.
  echo [ERROR] ROS 2 %ROS_DISTRO_EXPECTED% was not found.
  echo [ERROR] Set ROS2_SETUP_BAT to the setup.bat/local_setup.bat file and retry.
  echo [ERROR] Examples:
  echo [ERROR]   set ROS2_SETUP_BAT=C:\dev\ros2_lyrical\setup.bat
  echo [ERROR]   set ROS2_SETUP_BAT=C:\dev\ros2_humble\local_setup.bat
  echo.
  echo [INFO] Python dependencies were installed before this ROS 2 check failed.
  exit /b 2
)

for %%I in ("%ROS2_SETUP_BAT_FOUND%") do set "ROS2_ROOT=%%~dpI"
if "!ROS2_ROOT:~-1!"=="\" set "ROS2_ROOT=!ROS2_ROOT:~0,-1!"

if exist "!ROS2_ROOT!\preinstall_setup_windows.py" (
  if /I not "%SKIP_ROS2_PREINSTALL_PATCH%"=="1" (
    set "ACTIVE_PYTHON_EXE=%CONDA_PREFIX%\python.exe"
    findstr /I /C:"!ACTIVE_PYTHON_EXE!" "!ROS2_ROOT!\local_setup.bat" >nul 2>nul
    if not errorlevel 1 (
      echo [INFO] ROS 2 Windows scripts already point at !ACTIVE_PYTHON_EXE!.
    ) else (
      echo [INFO] Patching ROS 2 Windows scripts for the active Python...
      pushd "!ROS2_ROOT!"
      python preinstall_setup_windows.py
      set "PATCH_RC=!ERRORLEVEL!"
      popd
      if not "!PATCH_RC!"=="0" (
        echo [ERROR] ROS 2 preinstall_setup_windows.py failed.
        exit /b 1
      )
    )
  )
)

echo [INFO] Sourcing ROS 2 setup:
echo [INFO]   %ROS2_SETUP_BAT_FOUND%
call "%ROS2_SETUP_BAT_FOUND%"
if errorlevel 1 (
  echo [ERROR] Failed while sourcing ROS 2 setup.
  exit /b 1
)

if /I not "%ROS_DISTRO%"=="%ROS_DISTRO_EXPECTED%" (
  echo [ERROR] ROS_DISTRO is "%ROS_DISTRO%", expected "%ROS_DISTRO_EXPECTED%".
  echo [ERROR] Make sure the matching ROS 2 setup file is sourced.
  exit /b 2
)

where ros2 >nul 2>nul
if errorlevel 1 (
  echo [ERROR] ROS 2 setup was sourced, but ros2 is still not available on PATH.
  exit /b 2
)

echo [INFO] Running preflight...
set "EXPECTED_PYTHON_VERSION=%PYTHON_VERSION%"
python "%WORKSPACE_DIR%\scripts\preflight_ros2_windows.py"
if errorlevel 1 exit /b 1

call :FindVcvars64

echo.
echo [OK] Windows ROS 2 conda setup completed.
echo [OK] Next:
echo [OK]   cd /d "%WORKSPACE_DIR%"
if defined VCVARS64_BAT_FOUND echo [OK]   call "%VCVARS64_BAT_FOUND%"
if not defined VCVARS64_BAT_FOUND echo [WARN] Visual Studio vcvars64.bat was not found; install VS Build Tools before building rov_msgs.
echo [OK]   colcon build --merge-install
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
  "C:\dev\ros2_lyrical\setup.bat"
  "C:\dev\ros2_lyrical\local_setup.bat"
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

:InferRosDefaults
if not defined ROS_DISTRO_EXPECTED (
  set "ROS_DISTRO_EXPECTED=humble"
  echo %ROS2_SETUP_BAT_FOUND% | findstr /I "lyrical" >nul 2>nul
  if not errorlevel 1 set "ROS_DISTRO_EXPECTED=lyrical"
)

if not defined ENV_NAME (
  if /I "%ROS_DISTRO_EXPECTED%"=="lyrical" (
    set "ENV_NAME=ros2_lyrical"
  ) else (
    set "ENV_NAME=ros2"
  )
)

if not defined PYTHON_VERSION (
  if /I "%ROS_DISTRO_EXPECTED%"=="lyrical" (
    set "PYTHON_VERSION=3.12.3"
  ) else (
    set "PYTHON_VERSION=3.8.3"
  )
)
exit /b 0

:FindVcvars64
set "VCVARS64_BAT_FOUND="
if defined VCVARS64_BAT (
  if exist "%VCVARS64_BAT%" (
    set "VCVARS64_BAT_FOUND=%VCVARS64_BAT%"
    exit /b 0
  )
)

for %%P in (
  "%ProgramFiles(x86)%\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
  "%ProgramFiles%\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
) do (
  if exist "%%~P" (
    set "VCVARS64_BAT_FOUND=%%~P"
    exit /b 0
  )
)
exit /b 0
