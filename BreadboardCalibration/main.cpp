/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
#include <QApplication>
#include <pylon/PylonIncludes.h>

#include "laubreadboardcalibrationdialog.h"

int main(int argc, char *argv[])
{
    QApplication application(argc, argv);
    QCoreApplication::setOrganizationName(QString("LAU"));
    QCoreApplication::setApplicationName(QString("BreadboardCalibration"));

    // pylon must be initialised for the lifetime of the program (before any camera is created)
    Pylon::PylonAutoInitTerm pylonInit;

    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType<LAUBreadboardCalibrationResult>("LAUBreadboardCalibrationResult");

    // Test mode, no window:  BreadboardCalibration --file frame_raw10.png --out FOLDER
    // calibrates a saved 16-bit frame (10-bit values, as written by the dialog or vision/calibrate_frame.py)
    // and writes calibration.json, report.txt and the pictures to FOLDER.
    const QStringList args = QCoreApplication::arguments();
    const int fi = args.indexOf(QString("--file"));
    if (fi > 0 && fi + 1 < args.count()) {
        const int oi = args.indexOf(QString("--out"));
        const QString out = (oi > 0 && oi + 1 < args.count()) ? args.at(oi + 1) : QString("calibration_out");
        QImage img(args.at(fi + 1));
        if (img.isNull()) {
            return 2;
        }
        img = img.convertToFormat(QImage::Format_Grayscale16);
        cv::Mat raw(img.height(), img.width(), CV_16UC1);
        for (int r = 0; r < img.height(); r++) {
            memcpy(raw.ptr(r), img.constScanLine(r), img.width() * sizeof(quint16));
        }
        const LAUBreadboardCalibrationResult result = LAUBreadboardCalibration::calibrate(raw);
        LAUBreadboardCalibration::save(result, out);
        return result.ok ? 0 : 1;
    }

    LAUBreadboardCalibrationDialog dialog;
    dialog.exec();
    return 0;
}
