@echo off
cd /d %~dp0
rem Locate the newest VS with the x64 C++ toolset (VS install paths are versioned, e.g. ...\2022\, ...\18\).
set VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe
for /f "usebackq tokens=*" %%I in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2^>nul`) do set VSPATH=%%I
if not defined VSPATH echo ERROR: no Visual Studio with the x64 C++ toolset found & exit /b 1
call "%VSPATH%\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
set PATH=C:\Qt\6.9.3\msvc2022_64\bin;%PATH%
if not exist _bc mkdir _bc
cd _bc
qmake ..\FuyuRailController.pro || exit /b 1
nmake 2>&1
echo ==== copied dll size ====
for %%F in (release\FMC4030-Dll.dll) do echo %%~zF bytes
