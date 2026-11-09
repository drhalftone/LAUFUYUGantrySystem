#-------------------------------------------------------------------------------
# BreadboardCalibration.pro
#
# Standalone Qt dialog: live Basler picture (Mono10, auto-exposed, locally levelled) and one-button lens
# calibration from the breadboard's hole grid.  C++ port of vision/holegrid.py, calibrate_frame.py,
# vanishing_lines.py and live_clahe.py.
#
# Needs (Windows, MSVC 2022 x64 kit):
#   OpenCV 4.9.0  -- headers C:/usr/OpenCV490/include, libs/DLLs C:/usr/OpenCV/x64/vc17 (same install
#                    LAU3DVideoRecorder uses); the DLLs are copied next to the .exe after linking.
#   pylon 8 SDK   -- C:/Program Files/Basler/pylon 8/Development (its headers auto-link the libs;
#                    the runtime DLLs are on the PATH from the pylon install).
#-------------------------------------------------------------------------------

QT       += core gui widgets opengl openglwidgets
CONFIG   += c++17
TEMPLATE  = app
DEFINES  += NOMINMAX          # pylon pulls in windows.h; keep its min/max macros away from std::min/max
TARGET    = BreadboardCalibration

HEADERS += \
    lauholegrid.h \
    laubreadboardcalibration.h \
    laubaslergrabber.h \
    lauglimageview.h \
    laubreadboardcalibrationdialog.h

SOURCES += \
    main.cpp \
    lauholegrid.cpp \
    laubreadboardcalibration.cpp \
    laubaslergrabber.cpp \
    lauglimageview.cpp \
    laubreadboardcalibrationdialog.cpp

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
        message("pylon SDK not found: building BreadboardCalibration without camera support.")
    }

    CONFIG(debug, debug|release) {
        CV_SUFFIX = 490d
        CV_DST    = $$OUT_PWD/debug
    } else {
        CV_SUFFIX = 490
        CV_DST    = $$OUT_PWD/release
    }
    CV_MODULES = core imgproc calib3d features2d flann
    for(m, CV_MODULES) {
        LIBS += -lopencv_$${m}$${CV_SUFFIX}
        QMAKE_POST_LINK += $$QMAKE_COPY $$shell_quote($$shell_path($$OPENCV_BIN/opencv_$${m}$${CV_SUFFIX}.dll)) \
                           $$shell_quote($$shell_path($$CV_DST)) $$escape_expand(\\n\\t)
    }
}
