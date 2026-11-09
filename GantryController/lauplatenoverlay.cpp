/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
#include "lauplatenoverlay.h"

#include <cmath>
#include <vector>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>

namespace
{
    const double PITCH = 25.4, EDGE = 12.7;      // hole pitch, and board edge to the first hole, mm
    const int HOLES_PER_SIDE = 24;
    const double HOLE_RADIUS_MM = 3.19;
    const double APERTURE_RADIUS_MM = 4.9;       // cad/xrite_i1_plate.py
    const int CIRCLE_POINTS = 16;
    const double LINE_STEP_MM = 10.0;            // long lines are curved by the lens, so draw them in short pieces
    const double MAX_RAY_R2 = 1.5;               // beyond this the lens model isn't meaningful, and it is off the sensor

    template <int N> void readArray(const QJsonArray &a, double *out)
    {
        for (int n = 0; n < N && n < a.count(); n++) {
            out[n] = a.at(n).toDouble();
        }
    }

    struct Projector
    {
        const cv::Matx33d &K, &R;
        const double *dist;
        cv::Vec3d C;

        bool operator()(const cv::Point2d &board, QPointF *pixel) const
        {
            const cv::Vec3d p = R * cv::Vec3d(board.x - C[0], board.y - C[1], -C[2]);
            if (p[2] <= 1.0) {
                return false;
            }
            const double xn = p[0] / p[2], yn = p[1] / p[2], r2 = xn * xn + yn * yn;
            if (r2 > MAX_RAY_R2) {
                return false;
            }
            const double d = 1.0 + dist[0] * r2 + dist[1] * r2 * r2;
            *pixel = QPointF(K(0, 0) * xn * d + K(0, 2), K(1, 1) * yn * d + K(1, 2));
            return true;
        }
    };

    // a closed outline, broken wherever it leaves the part of the view the lens model covers
    void addOutline(QVector<QPolygonF> &paths, const std::vector<cv::Point2d> &corners, const Projector &project)
    {
        QPolygonF piece;
        const auto flush = [&]() {
            if (piece.count() > 1) {
                paths.append(piece);
            }
            piece.clear();
        };
        for (size_t n = 0; n < corners.size(); n++) {
            const cv::Point2d a = corners[n], b = corners[(n + 1) % corners.size()];
            const int steps = std::max(1, (int)std::ceil(std::hypot(b.x - a.x, b.y - a.y) / LINE_STEP_MM));
            for (int s = 0; s < steps; s++) {
                QPointF pixel;
                if (project(a + (b - a) * (s / (double)steps), &pixel)) {
                    piece.append(pixel);
                } else {
                    flush();
                }
            }
        }
        QPointF pixel;
        if (!corners.empty() && project(corners[0], &pixel)) {
            piece.append(pixel);
        }
        flush();
    }
}

bool LAUPlatenOverlay::load(const QString &bareFolder)
{
    valid = false;
    QFile file(QDir(bareFolder).filePath(QString("mosaic/calibration.json")));
    if (bareFolder.isEmpty() || !file.open(QIODevice::ReadOnly)) {
        return false;
    }
    const QJsonObject c = QJsonDocument::fromJson(file.readAll()).object();
    if (c["K"].toArray().count() != 9 || c["R"].toArray().count() != 9) {
        return false;
    }
    readArray<9>(c["K"].toArray(), K.val);
    readArray<9>(c["R"].toArray(), R.val);
    readArray<5>(c["dist"].toArray(), dist);
    readArray<3>(c["C0"].toArray(), C0.val);
    readArray<3>(c["ex"].toArray(), ex.val);
    readArray<3>(c["ey"].toArray(), ey.val);
    I0 = c["hole_I0"].toInt();
    J0 = c["hole_J0"].toInt();
    haveAperture = c.contains(QString("A0_mm"));
    if (haveAperture) {
        A0 = cv::Point2d(c["A0_mm"].toArray().at(0).toDouble(), c["A0_mm"].toArray().at(1).toDouble());
    }
    valid = true;
    return true;
}

bool LAUPlatenOverlay::reachSize(double singleLow, double singleHigh, double pairLow, double pairHigh, double *xMM, double *yMM) const
{
    if (!valid || !haveAperture) {
        return false;
    }
    double lowX = 1e12, highX = -1e12, lowY = 1e12, highY = -1e12;
    for (double pair : { pairLow, pairHigh }) {
        for (double single : { singleLow, singleHigh }) {
            const double bx = A0.x + ex[0] * pair + ey[0] * single, by = A0.y + ex[1] * pair + ey[1] * single;
            lowX = qMin(lowX, bx);
            highX = qMax(highX, bx);
            lowY = qMin(lowY, by);
            highY = qMax(highY, by);
        }
    }

    // grown by the circle's radius, and no further than the platen itself.  Board X is the rail pair, board Y the single rail.
    const double boardX = qMin(highX + APERTURE_RADIUS_MM, PITCH * (I0 + HOLES_PER_SIDE)) - qMax(lowX - APERTURE_RADIUS_MM, PITCH * I0);
    const double boardY = qMin(highY + APERTURE_RADIUS_MM, PITCH * (J0 + HOLES_PER_SIDE)) - qMax(lowY - APERTURE_RADIUS_MM, PITCH * J0);
    if (boardX <= 0.0 || boardY <= 0.0) {
        return false;
    }
    *xMM = std::floor(boardY);
    *yMM = std::floor(boardX);
    return true;
}

LAUViewOverlay LAUPlatenOverlay::build(double pairMM, double singleMM, double singleLow, double singleHigh, double pairLow, double pairHigh) const
{
    LAUViewOverlay overlay;
    if (!valid) {
        return overlay;
    }
    const Projector project{ K, R, dist, C0 + ex * pairMM + ey * singleMM };

    LAUViewOverlayLayer holes, edges, reach;
    holes.color = QColor(80, 255, 80);
    holes.width = 1.0;
    holes.label = QString("Platen holes");
    edges.color = QColor(255, 70, 70);
    edges.width = 2.0;
    edges.label = QString("Platen edge");
    reach.color = QColor(0, 220, 255);
    reach.width = 2.0;
    reach.label = QString("Farthest reach of the i1 circle");

    for (int J = J0; J < J0 + HOLES_PER_SIDE; J++) {
        for (int I = I0; I < I0 + HOLES_PER_SIDE; I++) {
            const cv::Point2d centre(EDGE + PITCH * I, EDGE + PITCH * J);
            QPolygonF circle;
            for (int n = 0; n <= CIRCLE_POINTS; n++) {
                const double angle = 2.0 * CV_PI * n / CIRCLE_POINTS;
                QPointF pixel;
                if (project(centre + cv::Point2d(std::cos(angle), std::sin(angle)) * HOLE_RADIUS_MM, &pixel)) {
                    circle.append(pixel);
                }
            }
            if (circle.count() == CIRCLE_POINTS + 1) {
                holes.paths.append(circle);
            }
        }
    }

    // the holes sit half a pitch in from the board's edges
    const double left = PITCH * I0, right = PITCH * (I0 + HOLES_PER_SIDE);
    const double top = PITCH * J0, bottom = PITCH * (J0 + HOLES_PER_SIDE);
    addOutline(edges.paths, { { left, top }, { right, top }, { right, bottom }, { left, bottom } }, project);

    if (haveAperture) {
        // where the aperture's centre can go, then that outline grown by the circle's radius with rounded corners
        const auto at = [&](double pair, double single) {
            return A0 + cv::Point2d(ex[0], ex[1]) * pair + cv::Point2d(ey[0], ey[1]) * single;
        };
        std::vector<cv::Point2d> centres{ at(pairLow, singleLow), at(pairHigh, singleLow), at(pairHigh, singleHigh), at(pairLow, singleHigh) };
        double area = 0.0;
        for (size_t n = 0; n < 4; n++) {
            area += centres[n].x * centres[(n + 1) % 4].y - centres[(n + 1) % 4].x * centres[n].y;
        }
        const double outward = (area > 0.0) ? 1.0 : -1.0;
        const auto normalAngle = [&](size_t n) {
            const cv::Point2d e = centres[(n + 1) % 4] - centres[n];
            return std::atan2(-outward * e.x, outward * e.y);
        };
        std::vector<cv::Point2d> outline;
        for (size_t n = 0; n < 4; n++) {
            double from = normalAngle((n + 3) % 4), to = normalAngle(n);
            // the short way round this corner
            while (to - from > CV_PI) {
                to -= 2.0 * CV_PI;
            }
            while (to - from < -CV_PI) {
                to += 2.0 * CV_PI;
            }
            for (int s = 0; s <= 6; s++) {
                const double angle = from + (to - from) * s / 6.0;
                outline.push_back(centres[n] + cv::Point2d(std::cos(angle), std::sin(angle)) * APERTURE_RADIUS_MM);
            }
        }
        addOutline(reach.paths, outline, project);
    }

    overlay.layers = { holes, edges };
    if (haveAperture) {
        overlay.layers.append(reach);
    }
    return overlay;
}
