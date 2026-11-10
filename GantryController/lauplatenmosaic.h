/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAUPlatenMosaic -- C++ port of vision/scan_map.py, mosaic.py, label_mosaic.py and hole_sliders.py.
//
// A bare-platen scan (no bareFolder in the request) calibrates: holes in every frame -> camera and carriage
// model -> carriage mask, flat-field, which lattice positions are real holes, and where the i1 aperture is.
// A scan with a target reuses that calibration.  Either way the frames are blended onto the platen and written
// to <scan>/mosaic/: platen_mosaic.png, platen_mosaic_grid.png, holes_inches.csv, hole_sliders.csv, report.txt, and
// platen_mosaic.json, which LAUPlatenGeometry reads to turn any point of the picture into board mm and rail positions.
//
// The pictures use the gantry's axes: X is the single rail along the columns, Y the rail pair down the rows, both
// increasing with their slider, and the origin is the board's first hole at the sliders' low ends.
// Lives in its own thread: onMerge() in, emitFinished() out.
//
#ifndef LAUPLATENMOSAIC_H
#define LAUPLATENMOSAIC_H

#include <atomic>
#include <QObject>
#include <QString>
#include <QMetaType>
#include <opencv2/core.hpp>

// Where a merged picture lies on the platen and how the rails reach it, saved with every merge in
// <scan>/mosaic/platen_mosaic.json.  Board mm is the calibration's board frame: X along the rail pair, Y along the
// single rail, from the board's corner.  The rails put the i1 aperture's centre on board point P when
//     pair * ex + single * ey = P - A0,
// A0 being where it sits with both sliders at zero.
class LAUPlatenGeometry
{
public:
    bool load(const QString &mosaicFolder, QString *error = nullptr);
    bool save(const QString &mosaicFolder) const;

    cv::Point2d boardOf(const cv::Point2d &picture) const;      // platen_mosaic.png pixel -> board mm
    cv::Point2d pictureOf(const cv::Point2d &board) const;

    bool hasRails() const
    {
        return railsValid;
    }
    // false if there is no aperture calibration or the rail axes are degenerate
    bool railsFor(const cv::Point2d &board, double *singleMM, double *pairMM) const;
    bool reachable(double singleMM, double pairMM) const
    {
        return singleMM >= singleLow && singleMM <= singleHigh && pairMM >= pairLow && pairMM <= pairHigh;
    }

    cv::Matx23d pictureToBoard = cv::Matx23d(1, 0, 0, 0, 1, 0);
    bool railsValid = false;
    cv::Point2d A0;
    cv::Vec2d ex, ey;                            // board mm per mm of the rail pair, and of the single rail
    double singleLow = 0.0, singleHigh = 0.0;    // the travel the i1 may use, mm
    double pairLow = 0.0, pairHigh = 0.0;
};

struct LAUPlatenMosaicRequest
{
    QString scanFolder;
    QString bareFolder;                  // empty: this scan is the bare platen and calibrates
    double sliderXLow = 0.0, sliderXHigh = 0.0;   // the travel the i1 can use, single rail, mm
    double sliderYLow = 0.0, sliderYHigh = 0.0;   // rail pair, mm
};
Q_DECLARE_METATYPE(LAUPlatenMosaicRequest)

struct LAUPlatenMosaicResult
{
    bool ok = false;
    QString message;                     // plain-language summary, or why it failed
    QString folder;                      // <scan>/mosaic
    QString gridImage;                   // the labelled mosaic
};
Q_DECLARE_METATYPE(LAUPlatenMosaicResult)

class LAUPlatenMosaic : public QObject
{
    Q_OBJECT

public:
    explicit LAUPlatenMosaic(QObject *parent = nullptr) : QObject(parent) { }

    LAUPlatenMosaicResult process(const LAUPlatenMosaicRequest &request);
    void cancel()
    {
        cancelFlag = true;
    }

public slots:
    void onMerge(LAUPlatenMosaicRequest request)
    {
        emit emitFinished(process(request));
    }

signals:
    void emitProgress(QString text);
    void emitFinished(LAUPlatenMosaicResult result);

private:
    std::atomic<bool> cancelFlag { false };
};

#endif // LAUPLATENMOSAIC_H
