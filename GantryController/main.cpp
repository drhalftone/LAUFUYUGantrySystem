/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
#include <QApplication>
#ifdef ENABLEPYLON
#include <pylon/PylonIncludes.h>
#endif

#include "laugantrymainwindow.h"

int main(int argc, char *argv[])
{
    QApplication application(argc, argv);

    // same QSettings store as FuyuRailController, so the rails keep their IP, travel and zero point
    QCoreApplication::setOrganizationName(QString("LAU"));
    QCoreApplication::setApplicationName(QString("FuyuRailController"));

    // pylon must be initialised for the lifetime of the program (before any camera is created)
#ifdef ENABLEPYLON
    Pylon::PylonAutoInitTerm pylonInit;
#endif

    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType<LAUBreadboardCalibrationResult>("LAUBreadboardCalibrationResult");

    LAUGantryMainWindow window;
    window.showMaximized();

    return application.exec();
}
