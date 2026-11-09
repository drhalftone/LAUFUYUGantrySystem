@echo off
rem Build GantryController (release) with the newest Visual Studio x64 toolset and Qt 6.9.3 msvc2022_64.
cd /d %~dp0
set VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe
for /f "usebackq tokens=*" %%I in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2^>nul`) do set VSPATH=%%I
if not defined VSPATH echo ERROR: no Visual Studio with the x64 C++ toolset found & exit /b 1
call "%VSPATH%\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
set PATH=C:\Qt\6.9.3\msvc2022_64\bin;%PATH%
if not exist _bc mkdir _bc
cd _bc
qmake ..\GantryController.pro || exit /b 1
nmake release 2>&1 || exit /b 1
windeployqt --release --no-translations release\GantryController.exe >nul
echo BUILD OK
