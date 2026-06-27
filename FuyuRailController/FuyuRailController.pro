#-------------------------------------------------------------------------------
# FuyuRailController.pro
#
# Qt GUI application for driving the FUYU FMC4030 motion controller (dual-X gantry
# + Y beam).  It REUSES the LAU3DVideoRecorder Velmex widget classes verbatim
# (lauvelmexwidget.*) and swaps only the controller: LAUVelmexController is
# reimplemented to drive the FMC4030 over Ethernet instead of a serial VXM.
#
# BUILD MODES
#   Default (simulation):   qmake && make            -> software-simulated controller.
#   Real hardware:          qmake CONFIG+=hardware    -> talks to the FMC4030 via the
#                           vendor DLL, loaded at RUN TIME (no link-time SDK dependency).
#
# sdk/FMC4030-Dll.dll is auto-copied next to the built .exe (see the win32 block at the
# bottom), so a hardware build finds it at run time with no manual copying.
#-------------------------------------------------------------------------------

QT       += core gui widgets
CONFIG   += c++17
TEMPLATE  = app
TARGET    = FuyuRailController
CONFIG   += hardware

HEADERS += \
    lauvelmexwidget.h

SOURCES += \
    main.cpp \
    lauvelmexwidget.cpp

hardware {
    message("Building FuyuRailController for real hardware (FMC4030-Dll loaded at run time).")
    # No link-time dependency on the SDK: FMC4030-Dll.dll is loaded dynamically via QLibrary
    # at startup, so the build needs neither the .lib nor the .h.  At run time, place
    # FMC4030-Dll.dll next to the .exe (Windows) or libFMC4030-Lib.so on the library path (Linux).
    # If the library is missing the app still launches and reports "disconnected".
} else {
    message("Building FuyuRailController in SIMULATION mode (no FMC4030 DLL needed).")
    DEFINES += FUYU_SIMULATE
}

# --- Copy the FMC4030 runtime DLL next to the built executable (Windows) --------------
# Whenever sdk/FMC4030-Dll.dll is present, copy it beside the linked .exe after the link
# step so a hardware build can load it at run time with no manual copying.  (Harmless for
# a simulation build, which never loads it.)
win32 {
    FMC_DLL = $$PWD/sdk/FMC4030-Dll.dll
    exists($$FMC_DLL) {
        CONFIG(debug, debug|release): FMC_DST = $$OUT_PWD/debug
        else:                         FMC_DST = $$OUT_PWD/release
        QMAKE_POST_LINK += $$QMAKE_COPY $$shell_quote($$shell_path($$FMC_DLL)) $$shell_quote($$shell_path($$FMC_DST)) $$escape_expand(\\n\\t)
    }
}
