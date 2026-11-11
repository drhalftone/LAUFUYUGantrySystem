#-------------------------------------------------------------------------------
# GantryController.pro
#
# One Qt window with the carriage camera above the rail sliders.  Builds the sources of
# ../BreadboardCalibration and ../FuyuRailController directly; see those two projects for what each needs:
#   OpenCV 4.9.0  -- headers C:/usr/OpenCV490/include, libs/DLLs C:/usr/OpenCV/x64/vc17
#   pylon 8 SDK   -- C:/Program Files/Basler/pylon 8/Development
#   FMC4030-Dll.dll -- ../FuyuRailController/sdk/, loaded at run time and copied next to the .exe
#
# Real rail hardware by default; qmake CONFIG+=simulate for the software-simulated controller.
#-------------------------------------------------------------------------------

QT       += core gui widgets opengl openglwidgets
CONFIG   += c++17
CONFIG   += no_batch          # nmake batch rules otherwise compile ../FuyuRailController/main.cpp in place of ours
TEMPLATE  = app
DEFINES  += NOMINMAX          # pylon pulls in windows.h; keep its min/max macros away from std::min/max
TARGET    = GantryController

CAMERA_DIR = $$PWD/../BreadboardCalibration
RAILS_DIR  = $$PWD/../FuyuRailController
COLOR_DIR  = $$PWD/../ColorController
INCLUDEPATH += $$CAMERA_DIR $$RAILS_DIR $$COLOR_DIR

simulate: DEFINES += FUYU_SIMULATE

HEADERS += \
    laugantrymainwindow.h \
    lauplatenmosaic.h \
    lauimageinspectdialog.h \
    lauplatenoverlay.h \
    laurailpredictor.h \
    lauprinttargetdialog.h \
    lautargetfinder.h     laucornertunedialog.h \
    lauprinttarget.h \
    laui1worker.h \
    laui1attachdialog.h \
    $$COLOR_DIR/laui1prousb.h \
    $$COLOR_DIR/lauargyllrunner.h \
    $$CAMERA_DIR/lauholegrid.h \
    $$CAMERA_DIR/laubreadboardcalibration.h \
    $$CAMERA_DIR/laubaslergrabber.h \
    $$CAMERA_DIR/lauglimageview.h \
    $$CAMERA_DIR/laubreadboardcalibrationdialog.h \
    $$RAILS_DIR/lauvelmexwidget.h

SOURCES += \
    main.cpp \
    laugantrymainwindow.cpp \
    laugantrymeasure.cpp     laugantrytune.cpp     laucornertunedialog.cpp \
    lauplatenmosaic.cpp \
    lauimageinspectdialog.cpp \
    lauplatenoverlay.cpp \
    lauprinttargetdialog.cpp \
    lautargetfinder.cpp \
    lauprinttarget.cpp \
    laui1worker.cpp \
    laui1attachdialog.cpp \
    $$COLOR_DIR/laui1prousb.cpp \
    $$COLOR_DIR/lauargyllrunner.cpp \
    $$CAMERA_DIR/lauholegrid.cpp \
    $$CAMERA_DIR/laubreadboardcalibration.cpp \
    $$CAMERA_DIR/laubaslergrabber.cpp \
    $$CAMERA_DIR/lauglimageview.cpp \
    $$CAMERA_DIR/laubreadboardcalibrationdialog.cpp \
    $$RAILS_DIR/lauvelmexwidget.cpp

win32 {
    OPENCV_INC = C:/usr/OpenCV490/include
    OPENCV_LIB = C:/usr/OpenCV/x64/vc17/lib
    OPENCV_BIN = C:/usr/OpenCV/x64/vc17/bin
    PYLON_DEV  = "C:/Program Files/Basler/pylon 8/Development"

    INCLUDEPATH += $$OPENCV_INC
    DEPENDPATH  += $$OPENCV_INC
    LIBS        += -L$$OPENCV_LIB

    # without the SDK the UI still builds and runs, the camera panel just reports it has no camera
    exists("$$PYLON_DEV/include/pylon/PylonIncludes.h") {
        DEFINES     += ENABLEPYLON
        INCLUDEPATH += $$quote($$PYLON_DEV/include)
        LIBS        += -L$$quote($$PYLON_DEV/lib/x64)
    } else {
        message("pylon SDK not found: building GantryController without camera support.")
    }

    CONFIG(debug, debug|release) {
        CV_SUFFIX = 490d
        CV_DST    = $$OUT_PWD/debug
    } else {
        CV_SUFFIX = 490
        CV_DST    = $$OUT_PWD/release
    }
    CV_MODULES = core imgproc calib3d features2d flann stitching
    for(m, CV_MODULES) {
        LIBS += -lopencv_$${m}$${CV_SUFFIX}
        QMAKE_POST_LINK += $$QMAKE_COPY $$shell_quote($$shell_path($$OPENCV_BIN/opencv_$${m}$${CV_SUFFIX}.dll)) \
                           $$shell_quote($$shell_path($$CV_DST)) $$escape_expand(\\n\\t)
    }

    FMC_DLL = $$RAILS_DIR/sdk/FMC4030-Dll.dll
    exists($$FMC_DLL) {
        QMAKE_POST_LINK += $$QMAKE_COPY $$shell_quote($$shell_path($$FMC_DLL)) $$shell_quote($$shell_path($$CV_DST)) $$escape_expand(\\n\\t)
    }
}
