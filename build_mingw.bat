@echo off
setlocal enabledelayedexpansion

REM ==========================================
REM Usage:
REM   build_mingw.bat          -> configure + build + test (no clean)
REM   build_mingw.bat clean    -> delete build then configure + build + test
REM ==========================================

REM Go to the directory where this .bat is located (project root)
cd /d "%~dp0"

REM Optional clean
if /I "%~1"=="clean" (
  if exist build (
    echo [CLEAN] Removing build\ ...
    rd /s /q build
  )
)

REM Ensure build dir exists
if not exist build mkdir build

REM Enter build dir
cd /d build

REM Configure
cmake .. -G "MinGW Makefiles" -DREFUEL_BUILD_TESTS=ON -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++

REM Stop if configure failed
if errorlevel 1 (
  echo [ERROR] CMake configure failed.
  exit /b 1
)

REM Build
mingw32-make -j

REM Stop if build failed
if errorlevel 1 (
  echo [ERROR] Build failed.
  exit /b 1
)

REM Test
ctest --output-on-failure

REM Return the proper exit code
exit /b %errorlevel%
