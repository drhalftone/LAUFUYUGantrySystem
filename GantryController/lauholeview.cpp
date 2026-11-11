/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
#include "lauholeview.h"

#include <cmath>
#include <vector>
#include <algorithm>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <opencv2/imgproc.hpp>

namespace
{
    const double APERTURE_RADIUS_MM = 4.9, APERTURE_FLOOR_MM = 1.0;   // cad/xrite_i1_plate.py
    const double MIN_DISC_MM2 = 30.0, MAX_DISC_MM2 = 95.0;           // the hole is 75 mm2; the rim hides some of it
    const double MIN_CONTRAST = 25.0;                                 // grey levels between paper and plate or board
    const double MAX_PAPER_SPREAD = 0.12;                             // of the disc's brightness: more means print under it
    const double STAY_OFF_RIM_MM = 0.4;
    const double PROFILE_HALF_MM = 1.0, PROFILE_STEP_MM = 0.04, FIRST_STEP_MM = 0.25;
    const int MIN_EDGE_POINTS = 20;
    const double MAX_CORNER_OFF_SQUARE_DEG = 10.0;

    template <int N> void readArray(const QJsonArray &a, double *out)
    {
        for (int n = 0; n < N && n < a.count(); n++) {
            out[n] = a.at(n).toDouble();
        }
    }

    double percentileOf(std::vector<double> v, double q)
    {
        if (v.empty()) {
            return 0.0;
        }
        const size_t k = (size_t)std::lround(q / 100.0 * (v.size() - 1));
        std::nth_element(v.begin(), v.begin() + k, v.end());
        return v[k];
    }

    struct Line
    {
        cv::Point2d c, d;                    // a point on it and its unit direction
        double rms = 0.0;
        int points = 0;
    };

    // total-least-squares line, refitted on the points within three robust sigmas
    bool fitLine(const std::vector<cv::Point2d> &p, Line *line)
    {
        std::vector<bool> keep(p.size(), true);
        for (int iteration = 0; iteration < 5; iteration++) {
            cv::Point2d c(0, 0);
            int n = 0;
            for (size_t k = 0; k < p.size(); k++) {
                if (keep[k]) {
                    c += p[k];
                    n++;
                }
            }
            if (n < 2) {
                return false;
            }
            c *= 1.0 / n;
            double sxx = 0, sxy = 0, syy = 0;
            for (size_t k = 0; k < p.size(); k++) {
                if (keep[k]) {
                    const cv::Point2d q = p[k] - c;
                    sxx += q.x * q.x;
                    sxy += q.x * q.y;
                    syy += q.y * q.y;
                }
            }
            const double theta = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
            line->c = c;
            line->d = cv::Point2d(std::cos(theta), std::sin(theta));
            const cv::Point2d normal(-line->d.y, line->d.x);
            std::vector<double> r(p.size()), kept;
            for (size_t k = 0; k < p.size(); k++) {
                r[k] = std::abs((p[k] - c).dot(normal));
                if (keep[k]) {
                    kept.push_back(r[k]);
                }
            }
            const double limit = std::max(0.02, 3.0 * 1.4826 * percentileOf(kept, 50.0));
            double sum = 0.0;
            n = 0;
            for (size_t k = 0; k < p.size(); k++) {
                keep[k] = r[k] < limit;
                if (keep[k]) {
                    sum += r[k] * r[k];
                    n++;
                }
            }
            line->points = n;
            line->rms = n ? std::sqrt(sum / n) : 0.0;
        }
        return line->points >= 2;
    }

    bool intersect(const Line &a, const Line &b, cv::Point2d *at)
    {
        const double det = a.d.x * (-b.d.y) - (-b.d.x) * a.d.y;
        if (std::abs(det) < 1e-9) {
            return false;
        }
        const cv::Point2d r = b.c - a.c;
        const double t = (r.x * (-b.d.y) - (-b.d.x) * r.y) / det;
        *at = a.c + t * a.d;
        return true;
    }

    cv::Point2d unit(const cv::Point2d &v)
    {
        return v * (1.0 / std::max(cv::norm(v), 1e-12));
    }

    // bilinear, or false off the picture
    bool sample(const cv::Mat &L, double x, double y, double *value)
    {
        const int x0 = (int)std::floor(x), y0 = (int)std::floor(y);
        if (x0 < 0 || y0 < 0 || x0 + 1 >= L.cols || y0 + 1 >= L.rows) {
            return false;
        }
        const double fx = x - x0, fy = y - y0;
        const float *r0 = L.ptr<float>(y0), *r1 = L.ptr<float>(y0 + 1);
        *value = (1 - fy) * ((1 - fx) * r0[x0] + fx * r0[x0 + 1]) + fy * ((1 - fx) * r1[x0] + fx * r1[x0 + 1]);
        return true;
    }

    cv::Mat colourOf(const cv::Mat &L)
    {
        cv::Mat grey, colour;
        L.convertTo(grey, CV_8U);
        cv::cvtColor(grey, colour, cv::COLOR_GRAY2BGR);
        return colour;
    }
}

bool LAUHoleView::load(const QString &bareFolder)
{
    valid = false;
    QFile file(QDir(bareFolder).filePath(QString("mosaic/calibration.json")));
    if (bareFolder.isEmpty() || !file.open(QIODevice::ReadOnly)) {
        return false;
    }
    const QJsonObject c = QJsonDocument::fromJson(file.readAll()).object();
    if (c["K"].toArray().count() != 9 || c["R"].toArray().count() != 9 || c["A0_mm"].toArray().count() != 2) {
        return false;
    }
    readArray<9>(c["K"].toArray(), K.val);
    readArray<9>(c["R"].toArray(), R.val);
    readArray<5>(c["dist"].toArray(), dist);
    readArray<3>(c["C0"].toArray(), C0.val);
    readArray<3>(c["ex"].toArray(), ex.val);
    readArray<3>(c["ey"].toArray(), ey.val);
    A0 = cv::Point2d(c["A0_mm"].toArray().at(0).toDouble(), c["A0_mm"].toArray().at(1).toDouble());
    valid = true;
    return true;
}

cv::Mat LAUHoleView::ortho(const cv::Mat &gray, double pairMM, double singleMM, const cv::Point2d &centre, double half, double res) const
{
    const int n = (int)std::lround(2.0 * half / res) + 1;
    cv::Mat mapU(n, n, CV_32F), mapV(n, n, CV_32F);
    const cv::Vec3d C = C0 + ex * pairMM + ey * singleMM;
    for (int r = 0; r < n; r++) {
        for (int c = 0; c < n; c++) {
            const cv::Vec3d p = R * cv::Vec3d(centre.x - half + c * res - C[0], centre.y - half + r * res - C[1], -C[2]);
            const double xn = p[0] / p[2], yn = p[1] / p[2], r2 = xn * xn + yn * yn;
            const double d = 1.0 + dist[0] * r2 + dist[1] * r2 * r2;
            mapU.at<float>(r, c) = (float)(K(0, 0) * xn * d + K(0, 2));
            mapV.at<float>(r, c) = (float)(K(1, 1) * yn * d + K(1, 2));
        }
    }
    cv::Mat L, out;
    gray.convertTo(L, CV_32F);
    cv::remap(L, out, mapU, mapV, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
    return out;
}

bool LAUHoleView::findHole(const cv::Mat &gray, double pairMM, double singleMM, const cv::Point2d &guess, Hole *hole, QString *why, cv::Mat *check) const
{
    if (!valid) {
        *why = QString("There's no platen calibration to see the hole with.");
        return false;
    }
    const cv::Mat L = ortho(gray, pairMM, singleMM, guess, WINDOW_HALF, RES);
    const int n = L.cols;
    cv::Mat soft, bw;
    cv::GaussianBlur(L, soft, cv::Size(0, 0), 2.0);
    soft.convertTo(soft, CV_8U);
    cv::threshold(soft, bw, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
    cv::Mat lab, stats, centroids;
    const int count = cv::connectedComponentsWithStats(bw, lab, stats, centroids);
    int best = -1;
    double bestDistance = 1e12;
    for (int k = 1; k < count; k++) {
        const int l = stats.at<int>(k, cv::CC_STAT_LEFT), t = stats.at<int>(k, cv::CC_STAT_TOP);
        const int w = stats.at<int>(k, cv::CC_STAT_WIDTH), h = stats.at<int>(k, cv::CC_STAT_HEIGHT);
        if (l > 0 && t > 0 && l + w < n && t + h < n && stats.at<int>(k, cv::CC_STAT_AREA) * RES * RES > 20.0) {
            const double d = std::hypot(centroids.at<double>(k, 0) - n / 2.0, centroids.at<double>(k, 1) - n / 2.0);
            if (d < bestDistance) {
                bestDistance = d;
                best = k;
            }
        }
    }
    if (best < 0) {
        *why = QString("No bright paper showed through the hole.");
        return false;
    }
    // the paper the hole shows is round (or round with a bite out by the rim): its convex outline, so anything printed
    // inside it counts against the spread below instead of being left out
    cv::Mat body = cv::Mat::zeros(lab.size(), CV_8U);
    {
        std::vector<std::vector<cv::Point>> outline;
        cv::findContours(cv::Mat(lab == best), outline, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        const auto largest = std::max_element(outline.begin(), outline.end(), [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
            return cv::contourArea(a) < cv::contourArea(b);
        });
        std::vector<cv::Point> hull;
        cv::convexHull(*largest, hull);
        cv::fillConvexPoly(body, hull, cv::Scalar(255), cv::LINE_8);
    }
    hole->areaMM2 = cv::countNonZero(body) * RES * RES;
    if (hole->areaMM2 < MIN_DISC_MM2 || hole->areaMM2 > MAX_DISC_MM2) {
        *why = QString("What showed through the hole (%1 mm2) isn't the size of the hole.").arg(hole->areaMM2, 0, 'f', 0);
        return false;
    }
    cv::Mat region, core;
    cv::dilate(body, region, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15)));
    const int in = (int)std::lround(2.0 * STAY_OFF_RIM_MM / RES) | 1;
    cv::erode(body, core, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(in, in)));
    std::vector<double> outside, inside;
    double sum = 0.0, sum2 = 0.0;
    int coreCount = 0;
    for (int r = 0; r < n; r++) {
        for (int c = 0; c < n; c++) {
            const double v = L.at<float>(r, c);
            if (!region.at<uchar>(r, c)) {
                outside.push_back(v);
            }
            if (body.at<uchar>(r, c)) {
                inside.push_back(v);
            }
            if (core.at<uchar>(r, c)) {
                sum += v;
                sum2 += v * v;
                coreCount++;
            }
        }
    }
    const double black = percentileOf(outside, 50.0), white = percentileOf(inside, 90.0);
    if (white - black < MIN_CONTRAST) {
        *why = QString("The paper through the hole is too dark against the plate.");
        return false;
    }
    const double mean = coreCount ? sum / coreCount : 0.0, spread = coreCount ? std::sqrt(std::max(sum2 / coreCount - mean * mean, 0.0)) : 1e9;
    if (spread > MAX_PAPER_SPREAD * mean) {
        *why = QString("Something is printed on the paper under the hole.");
        return false;
    }

    double sw = 0.0, sx = 0.0, sy = 0.0;
    for (int r = 0; r < n; r++) {
        for (int c = 0; c < n; c++) {
            if (region.at<uchar>(r, c)) {
                const double wgt = std::min(std::max((L.at<float>(r, c) - black) / (white - black), 0.0), 1.0);
                sw += wgt;
                sx += wgt * c;
                sy += wgt * r;
            }
        }
    }
    const cv::Point2d visible(guess.x - WINDOW_HALF + sx / sw * RES, guess.y - WINDOW_HALF + sy / sw * RES);
    // the plate's rim, above the paper, hides one side of the disc: its visible centroid sits half the rim's shift off
    const cv::Vec3d C = C0 + ex * pairMM + ey * singleMM;
    cv::Point2d c = visible;
    for (int k = 0; k < 3; k++) {
        const double shift = APERTURE_FLOOR_MM / (C[2] - APERTURE_FLOOR_MM);
        c = visible - 0.5 * shift * (c - cv::Point2d(C[0], C[1]));
    }
    hole->centre = c;
    hole->aperture = c - boardAxis(0) * pairMM - boardAxis(1) * singleMM;

    // the window, recentred on the hole
    const cv::Matx23d shift(1, 0, (guess.x - c.x) / RES, 0, 1, (guess.y - c.y) / RES);
    cv::warpAffine(body, hole->window, shift, body.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));

    if (check) {
        *check = colourOf(L);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(body, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        cv::drawContours(*check, contours, -1, cv::Scalar(255, 160, 0), 2, cv::LINE_AA);
        const cv::Point at((int)std::lround((c.x - guess.x + WINDOW_HALF) / RES), (int)std::lround((c.y - guess.y + WINDOW_HALF) / RES));
        cv::drawMarker(*check, at, cv::Scalar(0, 200, 0), cv::MARKER_CROSS, 40, 2);
        cv::circle(*check, at, (int)std::lround(APERTURE_RADIUS_MM / RES), cv::Scalar(0, 200, 0), 1, cv::LINE_AA);
    }
    return true;
}

bool LAUHoleView::findCorner(const cv::Mat &gray, double pairMM, double singleMM, const cv::Point2d &holeCentre, const cv::Mat &window,
                             const cv::Point2d &expected, cv::Point2d along1, cv::Point2d along2, Corner *corner, QString *why, cv::Mat *check) const
{
    if (!valid || window.empty()) {
        *why = QString("The hole hasn't been seen yet.");
        return false;
    }
    const cv::Mat L = ortho(gray, pairMM, singleMM, holeCentre, WINDOW_HALF, RES);
    const int n = L.cols;
    const auto toPixel = [&](const cv::Point2d &b) {
        return cv::Point2d((b.x - holeCentre.x + WINDOW_HALF) / RES, (b.y - holeCentre.y + WINDOW_HALF) / RES);
    };
    cv::Mat mask;
    const int in = (int)std::lround(2.0 * STAY_OFF_RIM_MM / RES) | 1;
    cv::erode(window, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(in, in)));
    const auto inMask = [&](const cv::Point2d &px) {
        const int x = (int)std::lround(px.x), y = (int)std::lround(px.y);
        return x >= 0 && y >= 0 && x < n && y < n && mask.at<uchar>(y, x);
    };

    // paper and board inside the hole
    std::vector<uchar> values;
    for (int r = 0; r < n; r++) {
        for (int c = 0; c < n; c++) {
            if (mask.at<uchar>(r, c)) {
                values.push_back(cv::saturate_cast<uchar>(L.at<float>(r, c)));
            }
        }
    }
    if (values.size() < 1000) {
        *why = QString("Too little of the hole is open.");
        return false;
    }
    cv::Mat packed(1, (int)values.size(), CV_8U, values.data()), ignored;
    const double threshold = cv::threshold(packed, ignored, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
    std::vector<double> dark, bright;
    std::vector<cv::Point2d> paper;
    for (int r = 0; r < n; r++) {
        for (int c = 0; c < n; c++) {
            if (mask.at<uchar>(r, c)) {
                const double v = L.at<float>(r, c);
                if (v > threshold) {
                    bright.push_back(v);
                    paper.emplace_back(holeCentre.x - WINDOW_HALF + c * RES, holeCentre.y - WINDOW_HALF + r * RES);
                } else {
                    dark.push_back(v);
                }
            }
        }
    }
    const double paperShare = (double)bright.size() / values.size();
    const double black = percentileOf(dark, 50.0), white = percentileOf(bright, 50.0);
    if (paperShare < 0.03 || paperShare > 0.97 || white - black < MIN_CONTRAST) {
        *why = (paperShare <= 0.03) ? QString("The hole shows no paper: the corner is too far off.")
               : (paperShare >= 0.97) ? QString("The hole shows only paper: the corner is too far off.")
                                      : QString("The paper and the board look too alike through the hole.");
        return false;
    }

    // the paper is the quadrant between the two edges.  m1 and m2 point into the paper, across edges 1 and 2.
    along1 = unit(along1);
    along2 = unit(along2);
    const auto inward = [](const cv::Point2d &along, const cv::Point2d &other) {
        cv::Point2d m(-along.y, along.x);
        return (m.dot(other) < 0.0) ? -m : m;
    };
    cv::Point2d m1 = inward(along1, along2), m2 = inward(along2, along1);
    // first guess: the paper's nearest reach toward each edge
    std::vector<double> p1, p2;
    for (const cv::Point2d &q : paper) {
        p1.push_back(q.dot(m1));
        p2.push_back(q.dot(m2));
    }
    const double a = percentileOf(p1, 1.0), b = percentileOf(p2, 1.0), det = m1.x * m2.y - m1.y * m2.x;
    if (std::abs(det) < 1e-6) {
        *why = QString("The page's edges are parallel.");
        return false;
    }
    cv::Point2d P((a * m2.y - m1.y * b) / det, (m1.x * b - a * m2.x) / det);

    Line lines[2];
    const double contrast = 0.5 * (white - black);
    for (int round = 0; round < 5; round++) {
        for (int e = 0; e < 2; e++) {
            const cv::Point2d u = e ? along2 : along1, m = e ? m2 : m1;
            std::vector<cv::Point2d> points;
            const int samples = (int)std::lround(2.0 * PROFILE_HALF_MM / RES) + 1;
            std::vector<double> profile(samples);
            for (double t = FIRST_STEP_MM; t < 2.0 * WINDOW_HALF; t += PROFILE_STEP_MM) {
                const cv::Point2d base = P + u * t;
                if (!inMask(toPixel(base - m * PROFILE_HALF_MM)) || !inMask(toPixel(base + m * PROFILE_HALF_MM)) || !inMask(toPixel(base))) {
                    continue;
                }
                bool ok = true;
                for (int s = 0; s < samples && ok; s++) {
                    const cv::Point2d px = toPixel(base + m * (-PROFILE_HALF_MM + s * RES));
                    ok = sample(L, px.x, px.y, &profile[s]);
                }
                if (!ok) {
                    continue;
                }
                const int edge = 12;
                double low = 0.0, high = 0.0;
                for (int s = 0; s < edge; s++) {
                    low += profile[s] / edge;
                    high += profile[samples - 1 - s] / edge;
                }
                if (high - low < contrast) {
                    continue;                // no step from board to paper along this profile
                }
                int k = 1;
                double steepest = -1e9;
                for (int s = 1; s < samples - 1; s++) {
                    const double g = profile[s + 1] - profile[s - 1];
                    if (g > steepest) {
                        steepest = g;
                        k = s;
                    }
                }
                if (k < 2 || k > samples - 3) {
                    continue;
                }
                const double g0 = profile[k] - profile[k - 2], g1 = steepest, g2 = profile[k + 2] - profile[k];
                const double den = g0 - 2.0 * g1 + g2;
                const double s = -PROFILE_HALF_MM + (k + ((den != 0.0) ? 0.5 * (g0 - g2) / den : 0.0)) * RES;
                points.push_back(base + m * s);
            }
            if ((int)points.size() < MIN_EDGE_POINTS || !fitLine(points, &lines[e]) || lines[e].points < MIN_EDGE_POINTS) {
                *why = QString("Couldn't see enough of the page's %1 edge through the hole.").arg(e ? QString("second") : QString("first"));
                return false;
            }
        }
        cv::Point2d next;
        if (!intersect(lines[0], lines[1], &next)) {
            *why = QString("The page's two edges came out parallel.");
            return false;
        }
        along1 = (lines[0].d.dot(along1) < 0.0) ? -lines[0].d : lines[0].d;
        along2 = (lines[1].d.dot(along2) < 0.0) ? -lines[1].d : lines[1].d;
        m1 = inward(along1, along2);
        m2 = inward(along2, along1);
        const double moved = cv::norm(next - P);
        P = next;
        if (moved < 0.002) {
            break;
        }
    }

    corner->at = P;
    corner->points[0] = lines[0].points;
    corner->points[1] = lines[1].points;
    corner->rmsMM = std::sqrt(0.5 * (lines[0].rms * lines[0].rms + lines[1].rms * lines[1].rms));
    corner->angleDeg = std::acos(std::min(1.0, std::abs(along1.dot(along2)))) * 180.0 / CV_PI;
    if (check) {
        *check = colourOf(L);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        cv::drawContours(*check, contours, -1, cv::Scalar(255, 160, 0), 1, cv::LINE_AA);
        for (int e = 0; e < 2; e++) {
            const cv::Point2d u = e ? along2 : along1;
            const cv::Point2d a0 = toPixel(P), a1 = toPixel(P + u * 2.0 * WINDOW_HALF);
            cv::line(*check, cv::Point((int)std::lround(a0.x), (int)std::lround(a0.y)), cv::Point((int)std::lround(a1.x), (int)std::lround(a1.y)),
                     cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        }
        const cv::Point2d c = toPixel(P), h = toPixel(holeCentre);
        cv::drawMarker(*check, cv::Point((int)std::lround(c.x), (int)std::lround(c.y)), cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 40, 2);
        cv::drawMarker(*check, cv::Point((int)std::lround(h.x), (int)std::lround(h.y)), cv::Scalar(0, 200, 0), cv::MARKER_CROSS, 40, 2);
        cv::circle(*check, cv::Point((int)std::lround(h.x), (int)std::lround(h.y)), (int)std::lround(APERTURE_RADIUS_MM / RES), cv::Scalar(0, 200, 0), 1, cv::LINE_AA);
    }
    if (std::abs(corner->angleDeg - 90.0) > MAX_CORNER_OFF_SQUARE_DEG) {
        *why = QString("The two edges found meet at %1 deg, not square.").arg(corner->angleDeg, 0, 'f', 1);
        return false;
    }
    if (cv::norm(P - expected) > WINDOW_HALF) {
        *why = QString("The corner found is %1 mm from where the picture put it.").arg(cv::norm(P - expected), 0, 'f', 1);
        return false;
    }
    return true;
}
