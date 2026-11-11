@echo off
call "%~dp0tools\GenerateProjectFiles_X64.bat" "Visual Studio 17 2022"
exit /b %ERRORLEVEL%
