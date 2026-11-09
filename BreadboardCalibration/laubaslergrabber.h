/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAUBaslerGrabber -- live frames from the carriage Basler (acA1920-150um) at its full bit depth (Mono10),
// with the exposure adjusting itself continuously (the brightest 0.5% kept just under clipping, damped so
// it doesn't flicker).  Port of vision/live_clahe.py's camera side.
//
// Runs its own grab loop in a QThread.  Emits raw CV_16U frames for the screen (levelled there, in a shader,
// at up to ~60/s), and on request an average of several frames (less noise for the sub-pixel hole centres).
//
#ifndef LAUBASLERGRABBER_H
#define LAUBASLERGRABBER_H

#include <QThread>
#include <QImage>
#include <atomic>
#include <opencv2/core.hpp>

class LAUBaslerGrabber : public QThread
{
    Q_OBJECT

public:
    explicit LAUBaslerGrabber(QObject *parent = nullptr) : QThread(parent) { }
    ~LAUBaslerGrabber() override
    {
        stop();
    }

    void stop()
    {
        stopFlag = true;
        wait();
    }

    // the receiver has taken the last emitFrame; until then new frames are skipped, not queued up
    void frameDone()
    {
        framePending = false;
    }

    // ask for the average of the next n frames (delivered by emitAveragedFrame)
    void requestAverage(int n = 8)
    {
        averageRequest = n;
    }

protected:
    void run() override;

private:
    std::atomic<bool> stopFlag{ false };
    std::atomic<int> averageRequest{ 0 };
    std::atomic<bool> framePending{ false };

signals:
    void emitFrame(cv::Mat raw16, double exposureMs);
    void emitAveragedFrame(cv::Mat raw16, double exposureMs);
    void emitCameraName(QString name);
    void emitError(QString message);
};

#endif // LAUBASLERGRABBER_H
