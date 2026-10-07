@echo off
rem Runs so101.sh from cmd or PowerShell through Git Bash. Same commands: so101\so101.cmd teleop
set "GITBASH=%ProgramFiles%\Git\bin\bash.exe"
if not exist "%GITBASH%" (
  echo Git Bash not found at %GITBASH%. Install Git for Windows.
  exit /b 1
)
"%GITBASH%" "%~dp0so101.sh" %*
