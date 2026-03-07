@echo off
setlocal enabledelayedexpansion

REM ==========================================================
REM build_mingw.bat
REM Stable MinGW build script for this project.
REM
REM Usage:
REM   build_mingw.bat
REM   build_mingw.bat clean
REM ==========================================================

cd /d "%~dp0"

if not exist CMakeLists.txt (
  echo [ERROR] CMakeLists.txt not found. Please place this script in the project root.
  exit /b 1
)

echo [INFO] Project root: %CD%

echo [CHECK] Looking for required tools...
where cmake >nul 2>nul
if errorlevel 1 (
  echo [ERROR] cmake not found in PATH.
  echo         Please open a MinGW/MSYS2 shell or make sure cmake is installed.
  exit /b 1
)

where gcc >nul 2>nul
if errorlevel 1 (
  echo [ERROR] gcc not found in PATH.
  echo         Please ensure MinGW gcc is installed and added to PATH.
  exit /b 1
)

where g++ >nul 2>nul
if errorlevel 1 (
  echo [ERROR] g++ not found in PATH.
  echo         Please ensure MinGW g++ is installed and added to PATH.
  exit /b 1
)

where mingw32-make >nul 2>nul
if errorlevel 1 (
  echo [ERROR] mingw32-make not found in PATH.
  echo         Please ensure MinGW Make is installed and added to PATH.
  exit /b 1
)

if /I "%~1"=="clean" (
  if exist build (
    echo [CLEAN] Removing build directory...
    rd /s /q build
  )
)

if not exist build mkdir build
cd /d build

echo [CONFIGURE] cmake .. -G "MinGW Makefiles" -DREFUEL_BUILD_TESTS=ON -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++
cmake .. -G "MinGW Makefiles" -DREFUEL_BUILD_TESTS=ON -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++
if errorlevel 1 (
  echo [ERROR] CMake configure failed.
  exit /b 1
)

echo [BUILD] cmake --build .
cmake --build .
if errorlevel 1 (
  echo [ERROR] Build failed.
  exit /b 1
)

echo [TEST] ctest --output-on-failure
ctest --output-on-failure
if errorlevel 1 (
  echo [ERROR] Some tests failed.
  exit /b 1
)

echo [DONE] Build and tests completed successfully.
echo [INFO] Useful commands:
echo        refuel_mode0_tests.exe
echo        refuel_mode1_tests.exe
echo        refuel_stage_tests.exe
echo        refuel_many_to_many_tests.exe
echo        refuel_demo.exe ..\demo\input\0210 ..\build\test_outputs\0210
echo        refuel_demo.exe ..\demo\input\0306_many_to_many ..\build\test_outputs\0306_many_to_many

exit /b 0
