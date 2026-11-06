/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAUBreadboardCalibration -- C++/Qt port of vision/calibrate_frame.py + vanishing_lines.py + the local
// brightness levelling of vision/live_clahe.py.
//
// From ONE frame of the breadboard (Mono10 raw, 0..1023 in a CV_16U Mat):
//   1. level it (each neighbourhood's brightness pulled toward the picture's overall level),
//   2. detect the holes and fit the lens-aware hole grid (LAUHoleGrid),
//   3. calibrate the lens on the labelled holes -- the board is a plane of points at a known 25.4 mm
//      pitch -- with cv::calibrateCamera (principal point at the image centre, square pixels, k1/k2),
//   4. remove the lens distortion and draw the hole columns running into the distance and the point
//      where they meet (vanishing point).
// Lives in its own thread: onCalibrate() in, emitResult() out.
//
#ifndef LAUBREADBOARDCALIBRATION_H
#define LAUBREADBOARDCALIBRATION_H

#include <QObject>
#include <QImage>
#include <QString>
#include <QMetaType>
#include <opencv2/core.hpp>

struct LAUBreadboardCalibrationResult
{
    bool ok = false;
    QString message;                 // plain-language summary, or why it failed
    cv::Size imageSize;
    double exposureMs = 0.0;

    // holes
    int holesDetected = 0, holesOnGrid = 0, holesUsed = 0, holesExpected = 0;
    double gridRms = 0.0;            // lens-aware grid fit residual, px

    // calibration
    cv::Matx33d K = cv::Matx33d::eye();
    double dist[5] = { 0, 0, 0, 0, 0 };
    double rms = 0.0, maxError = 0.0;
    double focalMM = 0.0, tiltDeg = 0.0, heightMM = 0.0, cornerShiftPx = 0.0;

    // lines into the distance, lens distortion removed
    int columns = 0;
    double straightRms = 0.0, straightMax = 0.0;
    double vanishingAbovePx = 0.0;   // how far above the top of the corrected picture the columns meet

    // pictures
    cv::Mat raw;                     // the frame (CV_16U, 10-bit values)
    QImage holes, undistorted, lines, vanishing;
};
Q_DECLARE_METATYPE(LAUBreadboardCalibrationResult)
Q_DECLARE_METATYPE(cv::Mat)

class LAUBreadboardCalibration : public QObject
{
    Q_OBJECT

public:
    explicit LAUBreadboardCalibration(QObject *parent = nullptr) : QObject(parent) { }

    static constexpr double PIXEL_MM = 5.86e-3;        // acA1920-150um pixel pitch
    static constexpr double NOMINAL_FOCAL_MM = 6.0;    // starting guess only; the calibration measures it
    static constexpr double NEIGHBOURHOOD_PX = 120.0;  // levelling: size of a neighbourhood (blur sigma)
    static constexpr double STRENGTH = 0.85;           // levelling: 0 = none, 1 = every neighbourhood equal

    // Local brightness levelling of a Mono10 frame.  picture() is log-encoded 8-bit for the screen;
    // levelLinear() keeps brightness ratios (holes stay as many times darker than the board) for measuring.
    static cv::Mat picture(const cv::Mat &raw16);
    static cv::Mat levelLinear(const cv::Mat &raw16);    // CV_32F, ~0..1

    static QImage toQImage(const cv::Mat &img);          // CV_8U grey or CV_8UC3 BGR -> deep copy

    static LAUBreadboardCalibrationResult calibrate(const cv::Mat &raw16, double exposureMs = 0.0);

    // Writes calibration.json (same keys as vision/calibrate_frame.py), report.txt and the pictures.
    static bool save(const LAUBreadboardCalibrationResult &result, const QString &folder, QString *error = nullptr);

public slots:
    void onCalibrate(cv::Mat raw16, double exposureMs)
    {
        emit emitResult(calibrate(raw16, exposureMs));
    }

signals:
    void emitResult(LAUBreadboardCalibrationResult result);
};

#endif // LAUBREADBOARDCALIBRATION_H
