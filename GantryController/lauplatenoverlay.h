/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAUPlatenOverlay -- where the platen should appear in the camera at a given rail position, from the calibration
// that LAUPlatenMosaic saved for the bare platen: every hole, the platen's edges, and how far the i1's measuring
// circle can reach within the sliders' travel.  The result is in raw camera pixels, for LAUImageView to draw.
//
#ifndef LAUPLATENOVERLAY_H
#define LAUPLATENOVERLAY_H

#include <QString>
#include <opencv2/core.hpp>

#include "lauglimageview.h"

class LAUPlatenOverlay
{
public:
    // reads <bareFolder>/mosaic/calibration.json
    bool load(const QString &bareFolder);
    bool isValid() const
    {
        return valid;
    }

    // pairMM, singleMM: the rails now.  The limits are the travel the i1 can use.
    LAUViewOverlay build(double pairMM, double singleMM, double singleLow, double singleHigh, double pairLow, double pairHigh) const;

private:
    bool valid = false;
    bool haveAperture = false;
    cv::Matx33d K = cv::Matx33d::eye();
    cv::Matx33d R = cv::Matx33d::eye();
    double dist[5] = { 0, 0, 0, 0, 0 };
    cv::Vec3d C0, ex, ey;
    cv::Point2d A0;
    int I0 = 0, J0 = 0;
};

#endif // LAUPLATENOVERLAY_H
