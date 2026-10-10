#-------------------------------------------------------------------------------
# ColorController.pro
#
# i1Pro spectral measurement + ArgyllCMS profiling for the FUYU gantry.  Ported from
# LAUXRiteController (i1Pro path only; the i1 iSis path was dropped).
#
#   LAUColorChart / LAUColorPatch   patch data model and Argyll .ti1/.ti2/.ti3 I/O
#   LAUArgyllRunner                 runs targen / colprof as separate processes
#   LAUi1ProDevice                  headless i1Pro SDK wrapper (spot + scan)
#   LAUChartLayout                  lays patches out in scan lines sized to the gantry's soft limits
#   LAUColorProfilerWindow          main window: Argyll tools + chart preview + FUYU gantry controls
#   LAUEyeOneDialog                 bench UI for spot readings (Tools menu)
#
# FUYU GANTRY
#   lauvelmexwidget.* is built straight from ../FuyuRailController (not copied).  Hardware build by
#   default, as in FuyuRailController; qmake CONFIG+=simulate runs the gantry in software simulation.
#
# X-RITE SDK (not redistributed in this repo)
#   Defaults to the copy in the sibling LAUXRiteController checkout.  Override with
#       qmake I1PRO_SDK=C:/path/to/Developer_i1Pro2_i1Pro_SDK_4.2.9
#   i1Pro64.dll is copied next to the built .exe after linking.
#
# ARGYLLCMS
#   Not linked.  targen/colprof are found at run time (QSettings
#   LAUArgyllRunner::binDirectory, then C:/usr/bin, C:/Argyll/bin, then PATH).
#-------------------------------------------------------------------------------

QT       += core gui widgets printsupport
CONFIG   += c++17
TEMPLATE  = app

# ../FuyuRailController HAS ITS OWN main.cpp; NMAKE'S BATCH INFERENCE RULES WOULD COMPILE THAT ONE
# INSTEAD OF OURS, SO BUILD EVERY SOURCE BY ITS EXPLICIT PATH
CONFIG   += no_batch
TARGET    = ColorController

INCLUDEPATH += $$PWD/../FuyuRailController

HEADERS += \
    ../FuyuRailController/lauvelmexwidget.h \
    laucolorprofilerwindow.h \
    lauchartlayout.h \
    lauargyllchart.h \
    lauargyllrunner.h \
    laui1prodevice.h \
    laueyeonedialog.h \
    lauspectraltablewidget.h \
    lauaboutdialog.h \
    qcustomplot.h

SOURCES += \
    main.cpp \
    ../FuyuRailController/lauvelmexwidget.cpp \
    laucolorprofilerwindow.cpp \
    lauchartlayout.cpp \
    lauargyllchart.cpp \
    lauargyllrunner.cpp \
    laui1prodevice.cpp \
    laueyeonedialog.cpp \
    lauspectraltablewidget.cpp \
    lauaboutdialog.cpp \
    qcustomplot.cpp

simulate {
    message("FUYU gantry in SIMULATION mode (no FMC4030 DLL needed).")
    DEFINES += FUYU_SIMULATE
}

isEmpty(I1PRO_SDK): I1PRO_SDK = $$(I1PRO_SDK)
isEmpty(I1PRO_SDK): I1PRO_SDK = $$PWD/../../LAUXRiteController/XRiteSDKs/Developer_i1Pro2_i1Pro_SDK_4.2.9
!exists($$I1PRO_SDK/include/i1Pro.h): error("i1Pro SDK not found at $$I1PRO_SDK -- pass qmake I1PRO_SDK=<path>")
message("Using i1Pro SDK at $$I1PRO_SDK")

INCLUDEPATH += $$I1PRO_SDK/include

win32 {
    LIBS += $$I1PRO_SDK/lib/x64/i1Pro64.lib

    # COPY THE RUNTIME DLL NEXT TO THE EXE (THE SDK SHIPS IT WITH ITS EXAMPLES)
    I1PRO_DLL = $$I1PRO_SDK/examples/bin/x64/i1Pro64.dll
    exists($$I1PRO_DLL) {
        CONFIG(debug, debug|release): I1PRO_DST = $$OUT_PWD/debug
        else:                         I1PRO_DST = $$OUT_PWD/release
        QMAKE_POST_LINK += $$QMAKE_COPY $$shell_quote($$shell_path($$I1PRO_DLL)) $$shell_quote($$shell_path($$I1PRO_DST)) $$escape_expand(\\n\\t)
    }

    # FMC4030 RUNTIME DLL, IF PRESENT IN FuyuRailController/sdk (LOADED AT RUN TIME VIA QLibrary)
    FMC_DLL = $$PWD/../FuyuRailController/sdk/FMC4030-Dll.dll
    exists($$FMC_DLL) {
        CONFIG(debug, debug|release): FMC_DST = $$OUT_PWD/debug
        else:                         FMC_DST = $$OUT_PWD/release
        QMAKE_POST_LINK += $$QMAKE_COPY $$shell_quote($$shell_path($$FMC_DLL)) $$shell_quote($$shell_path($$FMC_DST)) $$escape_expand(\\n\\t)
    }
}
