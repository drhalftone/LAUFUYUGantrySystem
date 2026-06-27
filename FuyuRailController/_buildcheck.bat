@echo off
cd /d %~dp0
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
set PATH=C:\Qt\6.9.3\msvc2022_64\bin;%PATH%
if not exist _bc mkdir _bc
cd _bc
qmake ..\FuyuRailController.pro || exit /b 1
nmake 2>&1
echo ==== copied dll size ====
for %%F in (release\FMC4030-Dll.dll) do echo %%~zF bytes
