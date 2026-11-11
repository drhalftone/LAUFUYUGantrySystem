/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAUHoleView -- what the carriage camera sees through the i1 holder's hole, with the i1 out of it.
//
// Everything is measured on the paper plane in board mm: the camera frame is mapped square-on onto the platen at
// 0.02 mm a pixel around the hole, using the platen calibration's camera.  The hole is fixed to the camera, so
// the part of the paper it shows is the same patch of the picture at any rail position.
//
//   findHole    over plain paper: the bright disc seen through the hole gives where the hole's centre is (the
//               plate's rim sits 1 mm above the paper and hides one side of the disc, which is allowed for, as
//               the platen calibration does), and the disc itself, kept as the window for findCorner.
//   findCorner  over a corner of the page: the two paper edges inside the disc, each a line through sub-pixel edge
//               points along the expected direction, and where they meet.
//
#ifndef LAUHOLEVIEW_H
#define LAUHOLEVIEW_H

#include <QString>
#include <opencv2/core.hpp>

class LAUHoleView
{
public:
    bool load(const QString &bareFolder);    // <bare>/mosaic/calibration.json
    bool isValid() const
    {
        return valid;
    }

    // float picture of the paper around the board point centre, half mm each way at res mm a pixel, rails at pair, single
    cv::Mat ortho(const cv::Mat &gray, double pairMM, double singleMM, const cv::Point2d &centre, double half, double res) const;

    struct Hole
    {
        cv::Point2d centre;                  // board mm, at the rails it was seen from
        cv::Point2d aperture;                // the same, rails at zero: the A0 of this session
        cv::Mat window;                      // the paper seen through it, 8-bit mask around centre (WINDOW_HALF, RES)
        double areaMM2 = 0.0;
    };
    // guess: where the calibration puts the hole, board mm
    bool findHole(const cv::Mat &gray, double pairMM, double singleMM, const cv::Point2d &guess, Hole *hole, QString *why, cv::Mat *check = nullptr) const;

    struct Corner
    {
        cv::Point2d at;                      // board mm
        double rmsMM = 0.0;                  // of the edge points about their lines
        int points[2] = { 0, 0 };
        double angleDeg = 0.0;               // between the edges
    };
    // holeCentre: the hole at these rails; expected: the corner, roughly; along1, along2: the page's edges leaving it
    bool findCorner(const cv::Mat &gray, double pairMM, double singleMM, const cv::Point2d &holeCentre, const cv::Mat &window,
                    const cv::Point2d &expected, cv::Point2d along1, cv::Point2d along2, Corner *corner, QString *why, cv::Mat *check = nullptr) const;

    cv::Point2d boardAxis(int rail) const    // board mm per mm of the rail pair (0) or the single rail (1)
    {
        return rail == 0 ? cv::Point2d(ex[0], ex[1]) : cv::Point2d(ey[0], ey[1]);
    }
    cv::Point2d calibratedAperture() const
    {
        return A0;
    }

    static constexpr double RES = 0.02, WINDOW_HALF = 9.0;

private:
    bool valid = false;
    cv::Matx33d K = cv::Matx33d::eye(), R = cv::Matx33d::eye();
    double dist[5] = { 0, 0, 0, 0, 0 };
    cv::Vec3d C0, ex, ey;
    cv::Point2d A0;
};

#endif // LAUHOLEVIEW_H
