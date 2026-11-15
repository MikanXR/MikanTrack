@echo off
setlocal

:: Starts the Tracy profiler's viewer and has it connect to MikanTrack on this
:: machine, on the port the build gives its client (MIKANTRACK_TRACY_PORT in
:: CMakeLists.txt). The viewer waits for the app if it is not running yet.
::
:: The viewer lives in deps\tracy, where InitialSetup_x64.bat puts it. A deps
:: folder from before the profiler has none, so it is fetched here on first
:: use: the same archive, the version of the client under thirdparty\tracy.

set DEPS_DIR=%~dp0deps
set TRACY_DIR=%DEPS_DIR%\tracy
set TRACY_EXE=%TRACY_DIR%\tracy-profiler.exe
set UNZIP_EXE=%~dp0tools\7zip\7za.exe

IF EXIST "%TRACY_EXE%" goto launch

IF NOT EXIST "%DEPS_DIR%" (
  echo "No deps folder. Run InitialSetup_x64.bat first."
  exit /b 1
)

echo "Downloading Tracy 0.15.0 viewer..."
curl -L https://github.com/wolfpld/tracy/releases/download/v0.15.0/windows-0.15.0.zip --output "%DEPS_DIR%\tracy.zip"
IF %ERRORLEVEL% NEQ 0 goto failure
"%UNZIP_EXE%" x "%DEPS_DIR%\tracy.zip" -o"%TRACY_DIR%" -y > nul
IF %ERRORLEVEL% NEQ 0 goto failure
del "%DEPS_DIR%\tracy.zip"

:launch
start "" "%TRACY_EXE%" -a 127.0.0.1 -p 8286
exit /b 0

:failure
echo "Error fetching the Tracy viewer"
exit /b 1
