/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
#include "laubaslergrabber.h"

#ifdef ENABLEPYLON
#include <cmath>
#include <vector>
#include <algorithm>
#include <QElapsedTimer>
#include <opencv2/imgproc.hpp>
#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

namespace
{
    const double FULL = 1023.0;              // Mono10 full scale
    const double TARGET = 0.90 * FULL;       // where the bright end (99.5th percentile) is kept

    double brightEnd(const cv::Mat &raw16)
    {
        std::vector<quint16> v;
        v.reserve((raw16.rows / 4 + 1) * (raw16.cols / 4 + 1));
        for (int r = 0; r < raw16.rows; r += 4) {
            const quint16 *p = raw16.ptr<quint16>(r);
            for (int c = 0; c < raw16.cols; c += 4) {
                v.push_back(p[c]);
            }
        }
        const size_t k = (size_t)std::llround(0.995 * (v.size() - 1));
        std::nth_element(v.begin(), v.begin() + k, v.end());
        return v[k];
    }

    // one gentle auto-exposure step: move the bright end toward TARGET, half the way (in log terms)
    void adjustExposure(Pylon::CBaslerUniversalInstantCamera &cam, const cv::Mat &raw16)
    {
        const double v = brightEnd(raw16);
        double factor;
        if (v >= FULL - 1) {
            factor = 0.5;                                    // clipped: true level unknown, back off
        } else if (std::abs(v - TARGET) < 0.05 * TARGET) {
            return;                                          // close enough
        } else {
            factor = std::min(std::max(std::sqrt(TARGET / std::max(v, 1.0)), 0.5), 2.0);
        }
        const double e = cam.ExposureTime.GetValue() * factor;
        cam.ExposureTime.SetValue(std::min(std::max(e, cam.ExposureTime.GetMin()), cam.ExposureTime.GetMax()));
    }
}

void LAUBaslerGrabber::run()
{
    try {
        Pylon::CBaslerUniversalInstantCamera cam(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
        cam.Open();
        emit emitCameraName(QString("%1 (%2)").arg(QString(cam.GetDeviceInfo().GetModelName().c_str()))
                                .arg(QString(cam.GetDeviceInfo().GetSerialNumber().c_str())));
        cam.PixelFormat.SetValue(Basler_UniversalCameraParams::PixelFormat_Mono10);
        cam.ExposureAuto.SetValue(Basler_UniversalCameraParams::ExposureAuto_Off);
        cam.GainAuto.SetValue(Basler_UniversalCameraParams::GainAuto_Off);
        cam.Gain.SetValue(0.0);
        cam.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

        Pylon::CGrabResultPtr res;
        QElapsedTimer previewClock;
        previewClock.start();
        int frame = 0;
        int averaging = 0, averaged = 0;
        cv::Mat acc;
        while (!stopFlag && cam.IsGrabbing()) {
            if (!cam.RetrieveResult(2000, res, Pylon::TimeoutHandling_Return) || !res || !res->GrabSucceeded()) {
                continue;
            }
            const cv::Mat raw((int)res->GetHeight(), (int)res->GetWidth(), CV_16UC1, res->GetBuffer());

            // start an average if one was asked for; while averaging, hold the exposure steady
            if (averaging == 0) {
                const int n = averageRequest.exchange(0);
                if (n > 0) {
                    averaging = n;
                    averaged = 0;
                    acc = cv::Mat::zeros(raw.size(), CV_64F);
                }
            }
            if (averaging > 0) {
                cv::accumulate(raw, acc);
                if (++averaged >= averaging) {
                    cv::Mat out;
                    acc.convertTo(out, CV_16U, 1.0 / averaged);
                    emit emitAveragedFrame(out, cam.ExposureTime.GetValue() / 1000.0);
                    averaging = 0;
                }
            } else if (autoExposure && ++frame % 3 == 0) {
                adjustExposure(cam, raw);                    // a new exposure takes a frame or two to show
            }

            if (previewClock.elapsed() >= 16 && !framePending.exchange(true)) {    // about one per screen refresh
                previewClock.restart();
                emit emitFrame(raw.clone(), cam.ExposureTime.GetValue() / 1000.0);
            }
        }
        cam.StopGrabbing();
        cam.Close();
    } catch (const GenICam::GenericException &e) {
        emit emitError(QString("Camera problem: %1").arg(e.GetDescription()));
    }
}
#else
// built without the pylon SDK: the UI still runs, it just never gets a frame
void LAUBaslerGrabber::run()
{
    emit emitError(QString("No camera support: this program was built without the Basler pylon SDK."));
}
#endif
