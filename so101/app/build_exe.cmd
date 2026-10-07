@echo off
rem Builds so101\SO101.exe (Windows launcher for the operator console) and puts a shortcut on the desktop.
setlocal
cd /d "%~dp0..\.."
set "CSC=%WINDIR%\Microsoft.NET\Framework64\v4.0.30319\csc.exe"
if not exist "%CSC%" (
  echo C# compiler not found at %CSC%. It ships with Windows ^(.NET Framework 4^).
  exit /b 1
)
if not exist so101\app\assets\so101.ico (
  uv pip install --python .venv PySide6 || exit /b 1
  .venv\Scripts\python so101\app\main.py --write-icon so101\app\assets\so101.ico || exit /b 1
)
"%CSC%" /nologo /target:winexe /optimize /win32icon:so101\app\assets\so101.ico /reference:System.Windows.Forms.dll /out:so101\SO101.exe so101\app\launcher\SO101Launcher.cs || exit /b 1
echo Built so101\SO101.exe
powershell -NoProfile -Command ^
  "$s = (New-Object -ComObject WScript.Shell).CreateShortcut([Environment]::GetFolderPath('Desktop') + '\SO-101.lnk');" ^
  "$s.TargetPath = (Resolve-Path 'so101\SO101.exe').Path; $s.WorkingDirectory = (Resolve-Path 'so101').Path;" ^
  "$s.IconLocation = $s.TargetPath + ',0'; $s.Description = 'SO-101 Operator Console'; $s.Save()"
echo Desktop shortcut: SO-101
