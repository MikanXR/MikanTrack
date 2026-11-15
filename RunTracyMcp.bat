@echo off
setlocal

:: Starts Tracy's MCP server, which lets an AI assistant load .tracy captures,
:: connect to a running MikanTrack and query the data through Tracy's own
:: analysis code. It listens on http://127.0.0.1:47380/mcp, runs as a
:: singleton (a second launch exits), and stays up in its own window.
::
:: The server needs Tracy's Python server bindings, built from the client under
:: thirdparty\tracy so the capture format matches. On first use this script
:: makes a venv with the server's requirements and builds the bindings into
:: deps\tracy-mcp, which needs Visual Studio 2022 and a network connection.
::
:: Set TRACY_CAPTURES_DIR before running to let the server list the captures
:: in a folder.

set DEPS_DIR=%~dp0deps
set MCP_DIR=%DEPS_DIR%\tracy-mcp
set TRACY_SRC=%~dp0thirdparty\tracy
set VENV_PY=%MCP_DIR%\.venv\Scripts\python.exe
set BINDINGS_DIR=%MCP_DIR%\build\python\Release

IF NOT EXIST "%DEPS_DIR%" (
  echo "No deps folder. Run InitialSetup_x64.bat first."
  exit /b 1
)

IF EXIST "%VENV_PY%" goto bindings
echo "Creating the Tracy MCP venv..."
py -3 -m venv "%MCP_DIR%\.venv"
IF %ERRORLEVEL% NEQ 0 goto failure
"%VENV_PY%" -m pip install -q -r "%TRACY_SRC%\extra\mcp\requirements.txt"
IF %ERRORLEVEL% NEQ 0 goto failure

:bindings
IF EXIST "%BINDINGS_DIR%\TracyServerBindings*.pyd" goto launch
echo "Building the Tracy server Python bindings..."
cmake -S "%TRACY_SRC%" -B "%MCP_DIR%\build" -G "Visual Studio 17 2022" -A x64 -DTRACY_CLIENT_PYTHON=ON -DTRACY_STATIC=OFF -DNO_LTO=ON -DPython_EXECUTABLE="%VENV_PY%"
IF %ERRORLEVEL% NEQ 0 goto failure
cmake --build "%MCP_DIR%\build" --config Release --target TracyServerBindings -- -m
IF %ERRORLEVEL% NEQ 0 goto failure

:launch
set PYTHONPATH=%BINDINGS_DIR%
start "Tracy MCP" "%VENV_PY%" "%TRACY_SRC%\extra\mcp\tracy_mcp.py"
exit /b 0

:failure
echo "Error setting up the Tracy MCP server"
exit /b 1
