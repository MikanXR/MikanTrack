@echo off
setlocal

:: Shared body of the GenerateProjectFiles_X64_VS*.bat wrappers.
:: %1 is the CMake Visual Studio generator name, e.g. "Visual Studio 17 2022".
IF "%~1"=="" (
  echo "Usage: GenerateProjectFiles_X64.bat <Visual Studio generator name>"
  exit /b 1
)

:: The Visual Studio 18 2026 generator needs CMake 4.2 or newer. Visual Studio bundles a new
:: enough CMake but only puts it on PATH inside its Developer Command Prompt.
where cmake > nul 2>&1
IF %ERRORLEVEL% NEQ 0 (
  echo "cmake was not found on PATH. Install CMake 4.2 or newer, or run this from a Visual Studio Developer Command Prompt."
  exit /b 1
)

:: This script lives in <repo>/tools, so the repo root is one level up.
for %%I in ("%~dp0..") do set REPO_ROOT_PATH=%%~fI

IF NOT EXIST "%REPO_ROOT_PATH%\build" mkdir "%REPO_ROOT_PATH%\build"
pushd "%REPO_ROOT_PATH%\build"
cmake .. -G "%~1" -A x64
IF %ERRORLEVEL% NEQ 0 (
  echo "Error generating project files"
  popd
  exit /b 1
)
popd
echo "Generated build/MikanTrack.sln"
exit /b 0
