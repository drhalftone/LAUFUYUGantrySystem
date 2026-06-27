FMC4030 SDK -- runtime DLL loading
==================================

The app loads the FMC4030 vendor library at RUN TIME (via QLibrary), so the build needs
no .lib and no .h, and has no link-time dependency on the SDK.  A "hardware" build even
launches when the DLL is absent -- it just reports "disconnected".

CURRENT DLL: FMC4030-Dll.dll is the NON-BLOCKING, 64-bit (X64) build, extracted from:
    ../../国外FMC4030/FMC4030 API file (non blocking).rar
        -> FMC4030二次开发库文件（非阻塞）/FMC4030-DLL-X64-20240920/FMC4030-Dll.dll

The non-blocking build is REQUIRED: LAUVelmexController issues one motion command and then
polls FMC4030_Check_Axis_Is_Stop from a 500 ms timer.  With the *blocking* build the motion
call would not return until the move finished, freezing the controller thread (no live
position, and Stop/abort could not be processed mid-move).  Do not swap in the blocking DLL.

This file is auto-copied next to the built .exe by the .pro (win32 post-link step).  For a
manual deploy, place FMC4030-Dll.dll next to FuyuRailController.exe (or on the PATH).

Bitness must match the build (use this 64-bit DLL with the 64-bit MSVC kit).  The app
resolves these symbols at startup:
    FMC4030_Open_Device, FMC4030_Close_Device, FMC4030_Jog_Single_Axis,
    FMC4030_Check_Axis_Is_Stop, FMC4030_Home_Single_Axis, FMC4030_Stop_Single_Axis,
    FMC4030_Get_Axis_Current_Pos
If any is missing it logs a warning and runs disconnected.

NOTE: the simulation build needs none of these files.
