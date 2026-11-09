/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAUPlatenMosaic -- C++ port of vision/scan_map.py, mosaic.py, label_mosaic.py and hole_sliders.py.
//
// A bare-platen scan (no bareFolder in the request) calibrates: holes in every frame -> camera and carriage
// model -> carriage mask, flat-field, which lattice positions are real holes, and where the i1 aperture is.
// A scan with a target reuses that calibration.  Either way the frames are blended onto the platen and written
// to <scan>/mosaic/: platen_mosaic.png, platen_mosaic_grid.png, holes_inches.csv, hole_sliders.csv, report.txt.
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
