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

#include <vector>
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
        // the best-fit ellipse to the disc's outline, board mm: the hole as the camera sees it
        bool hasEllipse = false;
        cv::Point2d ellipseCentre;
        double ellipseSemiA = 0.0, ellipseSemiB = 0.0, ellipseAngleDeg = 0.0;   // semi-axes, and the A axis from board X
    };
    // guess: where the calibration puts the hole, board mm
    bool findHole(const cv::Mat &gray, double pairMM, double singleMM, const cv::Point2d &guess, Hole *hole, QString *why, cv::Mat *check = nullptr) const;

    struct Corner
    {
        cv::Point2d at;                      // board mm
        double rmsMM = 0.0;                  // of the edge points about their lines
        int points[2] = { 0, 0 };
        double angleDeg = 0.0;               // between the edges
        // what was seen of each edge, kept even when the corner isn't found, so it can be drawn: its sub-pixel
        // points (board mm) and, when enough of them lined up, the line through them
        std::vector<cv::Point2d> edgePoints[2];
        bool edgeFitted[2] = { false, false };
        cv::Point2d edgeAt[2], edgeDir[2];
        bool atValid = false;                // the two lines met at 'at'
    };
    // holeCentre: the hole at these rails; expected: the corner, roughly; along1, along2: the page's edges leaving it
    bool findCorner(const cv::Mat &gray, double pairMM, double singleMM, const cv::Point2d &holeCentre, const cv::Mat &window,
                    const cv::Point2d &expected, cv::Point2d along1, cv::Point2d along2, Corner *corner, QString *why, cv::Mat *check = nullptr) const;

    // A page corner inside the hole, a little off the hole's centre so the paper fills most of it: the corner from its
    // two edges (findCorner), and the hole from a circle through the rim's arc where it crosses the paper.  Both in
    // board mm.  The circle is the rim as the camera sees it, a fixed shift from the true hole, so differences between
    // views are true to the hole's movement.
    bool findHoleAtCorner(const cv::Mat &gray, double pairMM, double singleMM, const cv::Point2d &holeGuess, cv::Point2d along1, cv::Point2d along2,
                          cv::Point2d *holeCentre, Corner *corner, QString *why, cv::Mat *check = nullptr, double *radiusMM = nullptr,
                          double knownRadiusMM = 0.0) const;   // > 0: the rim's radius is known, fit only its centre (a short arc)

    // A straight page edge across the hole: the edge as a line through sub-pixel edge points (in edge->edgeAt[0],
    // edgeDir[0], edgePoints[0]), and the hole from the rim's arc over the paper.  along: the edge's direction, roughly;
    // intoPaper: which side of it the paper is on.
    bool findHoleAtEdge(const cv::Mat &gray, double pairMM, double singleMM, const cv::Point2d &holeGuess, cv::Point2d along, cv::Point2d intoPaper,
                        cv::Point2d *holeCentre, Corner *edge, QString *why, cv::Mat *check = nullptr, double *radiusMM = nullptr,
                        double knownRadiusMM = 0.0) const;

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
    bool holeFromRim(const cv::Mat &L, const cv::Point2d &holeGuess, const cv::Mat &window, double windowMM, bool edgesOk, int edgeCount,
                     Corner *corner, cv::Point2d *holeCentre, QString *why, cv::Mat *check, double *radiusMM, double knownRadiusMM) const;

    bool valid = false;
    cv::Matx33d K = cv::Matx33d::eye(), R = cv::Matx33d::eye();
    double dist[5] = { 0, 0, 0, 0, 0 };
    cv::Vec3d C0, ex, ey;
    cv::Point2d A0;
};

#endif // LAUHOLEVIEW_H
