/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
#include "lautargetfinder.h"

#include <cmath>
#include <algorithm>
#include <opencv2/imgproc.hpp>

namespace
{
    const double RECT_MM = 0.1;                  // sheet-frame image resolution
    const double SHEET_PROFILE_MM = 4.0;         // half-length of the profiles across each paper edge
    const double SKIP_MM = 2.5;                  // paper next to its own edge is not trusted
    const double DROP = 0.04;                    // a swatch starts this much darker than the paper

    // findPrint
    const int COARSE_STEP = 5;                   // whole-sheet search at 0.5 mm
    const double TEMPLATE_MARGIN_MM = 3.0;       // white paper kept around the patch block in its expected look
    const double WINDOW_HALF_MM = 15.0;          // local matches: up to 30 x 30 mm
    const double SEARCH_MM = 2.5;                // how far a local match may move
    const double MIN_MATCH = 0.4, MIN_WINDOW_MATCH = 0.5;
    const int REFINE_ROUNDS = 3;

    struct Line
    {
        cv::Point2d c, d;                        // a point on it and its unit direction
        double keptShare = 0.0;
        int points = 0;
    };

    // as numpy.percentile (linear interpolation); v is reordered
    double percentile(std::vector<float> &v, double q)
    {
        const double position = q / 100.0 * (v.size() - 1);
        const size_t lo = (size_t)std::floor(position);
        std::nth_element(v.begin(), v.begin() + lo, v.end());
        const double low = v[lo];
        if (lo + 1 >= v.size()) {
            return low;
        }
        const double high = *std::min_element(v.begin() + lo + 1, v.end());
        return low + (position - lo) * (high - low);
    }

    // Robust total-least-squares line through p: refit on the points within a few robust sigmas, up to 6 times
    Line fitLine(const std::vector<cv::Point2d> &p, double tol)
    {
        std::vector<bool> keep(p.size(), true);
        Line line;
        for (int iteration = 0; iteration < 6; iteration++) {
            cv::Point2d c(0, 0);
            int n = 0;
            for (size_t k = 0; k < p.size(); k++) {
                if (keep[k]) {
                    c += p[k];
                    n++;
                }
            }
            if (n < 2) {
                break;
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
            const double theta = 0.5 * std::atan2(2.0 * sxy, sxx - syy);   // the principal direction
            line.c = c;
            line.d = cv::Point2d(std::cos(theta), std::sin(theta));
            const cv::Point2d normal(-line.d.y, line.d.x);

            std::vector<double> r(p.size());
            std::vector<float> kept;
            for (size_t k = 0; k < p.size(); k++) {
                r[k] = std::abs((p[k] - c).dot(normal));
                if (keep[k]) {
                    kept.push_back((float)r[k]);
                }
            }
            const double limit = std::max(tol, 3.0 * 1.4826 * percentile(kept, 50.0));
            std::vector<bool> fresh(p.size());
            for (size_t k = 0; k < p.size(); k++) {
                fresh[k] = r[k] < limit;
            }
            const bool same = (fresh == keep);
            keep = fresh;
            if (same) {
                break;
            }
        }
        line.points = (int)p.size();
        line.keptShare = p.empty() ? 0.0 : (double)std::count(keep.begin(), keep.end(), true) / p.size();
        return line;
    }

    bool intersect(const Line &a, const Line &b, cv::Point2d *at)
    {
        // c1 + t d1 = c2 + s d2
        const double det = a.d.x * (-b.d.y) - (-b.d.x) * a.d.y;
        if (std::abs(det) < 1e-12) {
            return false;
        }
        const cv::Point2d r = b.c - a.c;
        const double t = (r.x * (-b.d.y) - (-b.d.x) * r.y) / det;
        *at = a.c + t * a.d;
        return true;
    }

    // sub-sample position of the strongest step in a 1-D profile, in index units (numpy.gradient, then a parabola)
    double edgeAlong(const float *prof, int n)
    {
        std::vector<double> g(n);
        g[0] = prof[1] - prof[0];
        g[n - 1] = prof[n - 1] - prof[n - 2];
        for (int i = 1; i < n - 1; i++) {
            g[i] = 0.5 * (prof[i + 1] - prof[i - 1]);
        }
        int k = 1;
        for (int i = 2; i < n - 1; i++) {
            if (std::abs(g[i]) > std::abs(g[k])) {
                k = i;
            }
        }
        const double a = std::abs(g[k - 1]), b = std::abs(g[k]), c = std::abs(g[k + 1]);
        const double den = a - 2.0 * b + c;
        return k + (den != 0.0 ? 0.5 * (a - c) / den : 0.0);
    }

    // the sheet's corners: brightest blob -> rough rectangle -> each side refined from profiles across it
    bool findSheet(const cv::Mat &mosaic, double mm, std::vector<cv::Point2d> *corners, QString *why)
    {
        std::vector<float> seen;
        for (int r = 0; r < mosaic.rows; r++) {
            const uchar *row = mosaic.ptr<uchar>(r);
            for (int c = 0; c < mosaic.cols; c++) {
                if (row[c] > 0) {
                    seen.push_back(row[c]);
                }
            }
        }
        if (seen.size() < 100) {
            *why = QString("The platen picture is empty.");
            return false;
        }
        const double median = percentile(seen, 50.0);
        const double t = (median + percentile(seen, 99.5)) / 2.0;     // platen (most of what was seen) / white paper
        cv::Mat bw = mosaic > t;
        cv::Mat labels, stats, centroids;
        const int n = cv::connectedComponentsWithStats(bw, labels, stats, centroids);
        if (n < 2) {
            *why = QString("Couldn't find the sheet: nothing on the platen is brighter than the platen itself.");
            return false;
        }
        int best = 1;
        for (int k = 2; k < n; k++) {
            if (stats.at<int>(k, cv::CC_STAT_AREA) > stats.at<int>(best, cv::CC_STAT_AREA)) {
                best = k;
            }
        }
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(cv::Mat(labels == best), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        if (contours.empty()) {
            *why = QString("Couldn't find the sheet's outline.");
            return false;
        }
        const auto largest = std::max_element(contours.begin(), contours.end(), [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
            return cv::contourArea(a) < cv::contourArea(b);
        });
        cv::Point2f box[4];
        cv::minAreaRect(*largest).points(box);

        // order clockwise on screen, then start where u runs along the short side, roughly +x, and v roughly +y
        std::vector<cv::Point2d> quad(box, box + 4);
        cv::Point2d centre(0, 0);
        for (const cv::Point2d &q : quad) {
            centre += q * 0.25;
        }
        std::sort(quad.begin(), quad.end(), [&](const cv::Point2d &a, const cv::Point2d &b) {
            return std::atan2(a.y - centre.y, a.x - centre.x) < std::atan2(b.y - centre.y, b.x - centre.x);
        });
        double bestScore = -1e9;
        std::vector<cv::Point2d> ordered;
        for (int s = 0; s < 4; s++) {
            std::vector<cv::Point2d> q{ quad[s], quad[(s + 1) % 4], quad[(s + 2) % 4], quad[(s + 3) % 4] };
            const cv::Point2d u = q[1] - q[0], v = q[3] - q[0];
            if (cv::norm(u) > cv::norm(v)) {
                continue;
            }
            const double score = u.x / cv::norm(u) + v.y / cv::norm(v);
            if (ordered.empty() || score > bestScore) {
                bestScore = score;
                ordered = q;
            }
        }
        if (ordered.empty()) {
            *why = QString("Couldn't tell the sheet's sides apart.");
            return false;
        }

        cv::Mat image;
        mosaic.convertTo(image, CV_32F);
        const double half = SHEET_PROFILE_MM / mm;
        std::vector<double> offsets;
        for (double o = -half; o < half + 1.0 - 1e-9; o += 1.0) {
            offsets.push_back(o);
        }
        const int samples = 300;
        std::vector<Line> lines;
        for (int i = 0; i < 4; i++) {
            const cv::Point2d a = ordered[i], b = ordered[(i + 1) % 4];
            const cv::Point2d d = (b - a) * (1.0 / cv::norm(b - a));
            const cv::Point2d normal(-d.y, d.x);
            cv::Mat mapX(samples, (int)offsets.size(), CV_32F), mapY(samples, (int)offsets.size(), CV_32F);
            std::vector<cv::Point2d> base(samples);
            for (int s = 0; s < samples; s++) {
                base[s] = a + (0.08 + s * (0.84 / (samples - 1))) * (b - a);
                for (size_t o = 0; o < offsets.size(); o++) {
                    const cv::Point2d p = base[s] + offsets[o] * normal;
                    mapX.at<float>(s, (int)o) = (float)p.x;
                    mapY.at<float>(s, (int)o) = (float)p.y;
                }
            }
            cv::Mat profiles;
            cv::remap(image, profiles, mapX, mapY, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
            std::vector<cv::Point2d> points(samples);
            for (int s = 0; s < samples; s++) {
                const double e = edgeAlong(profiles.ptr<float>(s), profiles.cols) - half;
                points[s] = base[s] + e * normal;
            }
            lines.push_back(fitLine(points, 0.3 / mm));
        }
        corners->resize(4);
        for (int i = 0; i < 4; i++) {
            if (!intersect(lines[(i + 3) % 4], lines[i], &(*corners)[i])) {
                *why = QString("Two of the sheet's edges came out parallel.");
                return false;
            }
        }
        return true;
    }

    // The sheet seen square on, at RECT_MM: u right along corners 0->1, v down along 0->3.  Pixel (i, j) of S is the
    // sheet point (i, j) * RECT_MM mm from corner 0; H takes those pixel coordinates to the mosaic's.
    void rectifySheet(const cv::Mat &mosaic, double mm, LAUTargetFinderResult *result, cv::Mat *S, cv::Mat *H)
    {
        result->sheetW = cv::norm(result->sheet[1] - result->sheet[0]) * mm;
        result->sheetL = cv::norm(result->sheet[3] - result->sheet[0]) * mm;
        const int w = (int)std::nearbyint(result->sheetW / RECT_MM), h = (int)std::nearbyint(result->sheetL / RECT_MM);
        const std::vector<cv::Point2f> rect{ { 0.0f, 0.0f }, { (float)w, 0.0f }, { (float)w, (float)h }, { 0.0f, (float)h } };
        std::vector<cv::Point2f> sheet;
        for (const cv::Point2d &p : result->sheet) {
            sheet.push_back(cv::Point2f((float)p.x, (float)p.y));
        }
        *H = cv::getPerspectiveTransform(rect, sheet);
        cv::Mat image;
        mosaic.convertTo(image, CV_32F);
        cv::warpPerspective(image, *S, *H, cv::Size(w, h), cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);
    }

    // the swatch block's edges in the sheet image: scan in from each paper edge to the first swatch
    bool findBlock(const cv::Mat &S, std::vector<cv::Point2d> *corners, std::vector<Line> *edges, QString *why)
    {
        const int h = S.rows, w = S.cols;
        const int sk = (int)(SKIP_MM / RECT_MM);
        if (w < 4 * sk || h < 4 * sk) {
            *why = QString("The sheet is too small.");
            return false;
        }
        std::vector<float> margin;
        const auto take = [&](const cv::Mat &region) {
            for (int r = 0; r < region.rows; r++) {
                margin.insert(margin.end(), region.ptr<float>(r), region.ptr<float>(r) + region.cols);
            }
        };
        take(S(cv::Rect(sk, 0, sk, h)));
        take(S(cv::Rect(w - 2 * sk, 0, sk, h)));
        take(S(cv::Rect(0, sk, w, sk)));
        take(S(cv::Rect(0, h - 2 * sk, w, sk)));
        const double paper = percentile(margin, 50.0);
        const double threshold = paper * (1.0 - DROP);

        // left, top, right, bottom -> u0, v0, u1, v1
        const int passes[4][2] = { { 1, 0 }, { 0, 0 }, { 1, 1 }, { 0, 1 } };
        edges->clear();
        for (const auto &pass : passes) {
            const bool alongRows = pass[0] == 1, reversed = pass[1] == 1;
            cv::Mat A = alongRows ? S : S.t();
            if (reversed) {
                cv::flip(A, A, 1);
            }
            const int lines = A.rows, n = A.cols;
            std::vector<cv::Point2d> points;
            for (int r = (int)(0.05 * lines); r < (int)(0.95 * lines); r += 3) {
                cv::Mat row;
                cv::GaussianBlur(A.row(r).clone(), row, cv::Size(0, 0), 1.0);
                const float *v = row.ptr<float>(0);
                int k = -1;
                for (int i = sk; i < n / 2; i++) {
                    if (v[i] < threshold) {
                        k = i;
                        break;
                    }
                }
                if (k < 0) {
                    continue;
                }
                const int lo = std::max(k - 15, 1), hi = std::min(k + 15, n - 2);
                double e = lo + edgeAlong(v + lo, hi - lo);
                if (reversed) {
                    e = n - 1 - e;
                }
                points.push_back(alongRows ? cv::Point2d(e, r) : cv::Point2d(r, e));
            }
            if (points.size() < 10) {
                *why = QString("Couldn't find the edge of the swatches inside the sheet.");
                return false;
            }
            edges->push_back(fitLine(points, 0.4 / RECT_MM));
        }
        const Line &u0 = (*edges)[0], &v0 = (*edges)[1], &u1 = (*edges)[2], &v1 = (*edges)[3];
        corners->resize(4);
        const bool ok = intersect(u0, v0, &(*corners)[0]) && intersect(u1, v0, &(*corners)[1]) &&
                        intersect(u1, v1, &(*corners)[2]) && intersect(u0, v1, &(*corners)[3]);
        if (!ok) {
            *why = QString("Two of the swatch block's edges came out parallel.");
        }
        return ok;
    }

    std::vector<cv::Point2d> transformed(const std::vector<cv::Point2d> &p, const cv::Mat &H)
    {
        std::vector<cv::Point2d> out;
        cv::perspectiveTransform(p, out, H);
        return out;
    }

    // close-up of the sheet, four times the mosaic's size: the sheet, the block, every swatch and its centre
    cv::Mat drawCheck(const cv::Mat &mosaic, const LAUTargetFinderResult &result, const std::vector<std::vector<cv::Point2d>> &swatches)
    {
        double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
        for (const cv::Point2d &p : result.sheet) {
            x0 = std::min(x0, p.x);
            y0 = std::min(y0, p.y);
            x1 = std::max(x1, p.x);
            y1 = std::max(y1, p.y);
        }
        const int cx0 = std::max((int)std::floor(x0 - 20), 0), cy0 = std::max((int)std::floor(y0 - 20), 0);
        const int cx1 = std::min((int)std::ceil(x1 + 20), mosaic.cols), cy1 = std::min((int)std::ceil(y1 + 20), mosaic.rows);
        cv::Mat check;
        if (cx1 <= cx0 || cy1 <= cy0) {
            return check;
        }
        const int up = 4;
        cv::Mat colour;
        cv::cvtColor(mosaic(cv::Rect(cx0, cy0, cx1 - cx0, cy1 - cy0)), colour, cv::COLOR_GRAY2BGR);
        cv::resize(colour, check, cv::Size(), up, up, cv::INTER_CUBIC);
        const auto toCheck = [&](const std::vector<cv::Point2d> &p) {
            std::vector<cv::Point> q;
            for (const cv::Point2d &v : p) {
                q.emplace_back((int)std::lround(((v.x - cx0 + 0.5) * up - 0.5) * 16), (int)std::lround(((v.y - cy0 + 0.5) * up - 0.5) * 16));
            }
            return q;
        };
        cv::polylines(check, std::vector<std::vector<cv::Point>>{ toCheck(result.sheet) }, true, cv::Scalar(0, 200, 0), 2, cv::LINE_AA, 4);
        cv::polylines(check, std::vector<std::vector<cv::Point>>{ toCheck(result.block) }, true, cv::Scalar(255, 160, 0), 2, cv::LINE_AA, 4);
        for (size_t k = 0; k < swatches.size(); k++) {
            cv::polylines(check, std::vector<std::vector<cv::Point>>{ toCheck(swatches[k]) }, true, cv::Scalar(0, 220, 255), 1, cv::LINE_AA, 4);
            const cv::Point q = toCheck({ result.centres[k] })[0];
            cv::drawMarker(check, cv::Point(q.x >> 4, q.y >> 4), cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 14, 2);
        }
        return check;
    }

    cv::Point2d apply(const cv::Matx23d &A, const cv::Point2d &p)
    {
        return cv::Point2d(A(0, 0) * p.x + A(0, 1) * p.y + A(0, 2), A(1, 0) * p.x + A(1, 1) * p.y + A(1, 2));
    }

    cv::Matx23d inverted(const cv::Matx23d &A)
    {
        cv::Matx23d inverse;
        cv::invertAffineTransform(A, inverse);
        return inverse;
    }

    // sub-sample peak of a correlation map near its maximum, by a parabola along each axis
    cv::Point2d subPixelPeak(const cv::Mat &response, const cv::Point &peak)
    {
        cv::Point2d p(peak.x, peak.y);
        if (peak.x > 0 && peak.x < response.cols - 1) {
            const double a = response.at<float>(peak.y, peak.x - 1), b = response.at<float>(peak.y, peak.x), c = response.at<float>(peak.y, peak.x + 1);
            const double den = a - 2.0 * b + c;
            p.x += (den != 0.0) ? 0.5 * (a - c) / den : 0.0;
        }
        if (peak.y > 0 && peak.y < response.rows - 1) {
            const double a = response.at<float>(peak.y - 1, peak.x), b = response.at<float>(peak.y, peak.x), c = response.at<float>(peak.y + 1, peak.x);
            const double den = a - 2.0 * b + c;
            p.y += (den != 0.0) ? 0.5 * (a - c) / den : 0.0;
        }
        return p;
    }
}

LAUTargetFinderResult LAUTargetFinder::find(const cv::Mat &mosaic, double mm, int rows, int cols, double gapMM)
{
    LAUTargetFinderResult result;
    if (mosaic.empty() || mosaic.type() != CV_8UC1 || rows < 1 || cols < 1) {
        result.message = QString("No platen picture to look for the target in.");
        return result;
    }

    if (!findSheet(mosaic, mm, &result.sheet, &result.message)) {
        return result;
    }
    cv::Mat S, H;
    rectifySheet(mosaic, mm, &result, &S, &H);

    std::vector<cv::Point2d> blockRect;
    std::vector<Line> edges;
    if (!findBlock(S, &blockRect, &edges, &result.message)) {
        return result;
    }
    for (int k = 0; k < 4; k++) {
        result.edgeFit[k] = edges[k].keptShare;
        result.edgeScanlines[k] = edges[k].points;
    }

    // swatch centres from the block's corners and the layout
    result.blockW = cv::norm(blockRect[1] - blockRect[0]) * RECT_MM;
    result.blockL = cv::norm(blockRect[3] - blockRect[0]) * RECT_MM;
    const double pu = (result.blockW + gapMM) / cols, pv = (result.blockL + gapMM) / rows;
    result.swatchW = pu - gapMM;
    result.swatchH = pv - gapMM;
    std::vector<cv::Point2f> blockMM{ { 0.0f, 0.0f }, { (float)result.blockW, 0.0f }, { (float)result.blockW, (float)result.blockL }, { 0.0f, (float)result.blockL } };
    std::vector<cv::Point2f> blockPx;
    for (const cv::Point2d &p : blockRect) {
        blockPx.push_back(cv::Point2f((float)p.x, (float)p.y));
    }
    const cv::Mat Hb = cv::getPerspectiveTransform(blockMM, blockPx);
    std::vector<cv::Point2d> uv;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            uv.emplace_back(c * pu + result.swatchW / 2.0, r * pv + result.swatchH / 2.0);
        }
    }
    const std::vector<cv::Point2d> centresRect = transformed(uv, Hb);
    result.centres = transformed(centresRect, H);
    result.block = transformed(blockRect, H);

    std::vector<std::vector<cv::Point2d>> swatches;
    const double hw = result.swatchW / 2.0 / RECT_MM, hh = result.swatchH / 2.0 / RECT_MM;
    for (const cv::Point2d &c : centresRect) {
        swatches.push_back(transformed({ c + cv::Point2d(-hw, -hh), c + cv::Point2d(hw, -hh), c + cv::Point2d(hw, hh), c + cv::Point2d(-hw, hh) }, H));
    }
    result.check = drawCheck(mosaic, result, swatches);

    result.ok = true;
    result.message = QString("Found the target: a %1 x %2 mm sheet, %3 x %4 swatches of %5 x %6 mm.")
                         .arg(result.sheetW, 0, 'f', 1).arg(result.sheetL, 0, 'f', 1).arg(cols).arg(rows)
                         .arg(result.swatchW, 0, 'f', 2).arg(result.swatchH, 0, 'f', 2);
    return result;
}

LAUTargetFinderResult LAUTargetFinder::findPrint(const cv::Mat &mosaic, double mm, const LAUPrintTarget &target)
{
    LAUTargetFinderResult result;
    if (mosaic.empty() || mosaic.type() != CV_8UC1) {
        result.message = QString("No platen picture to look for the target in.");
        return result;
    }
    if (!target.isValid()) {
        result.message = QString("No print target to look for.");
        return result;
    }
    if (!findSheet(mosaic, mm, &result.sheet, &result.message)) {
        return result;
    }
    cv::Mat S, H;
    rectifySheet(mosaic, mm, &result, &S, &H);

    // the patch block's expected look in page millimetres: white paper, each patch as bright as the .ti2 expects.
    // Pixel i of the template is the page point origin + i * RECT_MM.
    double bx0 = 1e9, by0 = 1e9, bx1 = -1e9, by1 = -1e9;
    for (const LAUPrintPatch &p : target.patches) {
        bx0 = std::min(bx0, p.boxMM.x);
        by0 = std::min(by0, p.boxMM.y);
        bx1 = std::max(bx1, p.boxMM.x + p.boxMM.width);
        by1 = std::max(by1, p.boxMM.y + p.boxMM.height);
    }
    result.blockW = bx1 - bx0;
    result.blockL = by1 - by0;
    const cv::Point2d origin(bx0 - TEMPLATE_MARGIN_MM, by0 - TEMPLATE_MARGIN_MM);
    const int tw = COARSE_STEP * (int)std::ceil((result.blockW + 2.0 * TEMPLATE_MARGIN_MM) / (COARSE_STEP * RECT_MM));
    const int th = COARSE_STEP * (int)std::ceil((result.blockL + 2.0 * TEMPLATE_MARGIN_MM) / (COARSE_STEP * RECT_MM));
    cv::Mat look(th, tw, CV_32F, cv::Scalar(1.0f));
    for (const LAUPrintPatch &p : target.patches) {
        const int i0 = (int)std::lround((p.boxMM.x - origin.x) / RECT_MM), i1 = (int)std::lround((p.boxMM.x + p.boxMM.width - origin.x) / RECT_MM);
        const int j0 = (int)std::lround((p.boxMM.y - origin.y) / RECT_MM), j1 = (int)std::lround((p.boxMM.y + p.boxMM.height - origin.y) / RECT_MM);
        const cv::Rect r = cv::Rect(i0, j0, i1 - i0, j1 - j0) & cv::Rect(0, 0, tw, th);
        look(r).setTo(cv::Scalar(std::min(std::max(p.xyz[1] / 100.0, 0.0), 1.2)));
    }

    // ---- whole sheet, 0.5 mm, four orientations.  Coarse pixel i averages fine pixels 5i..5i+4, so it sits 0.2 mm on.
    const double coarse = COARSE_STEP * RECT_MM, shift = 0.5 * (COARSE_STEP - 1) * RECT_MM;
    cv::Mat lookCoarse, sheetCoarse;
    cv::resize(look, lookCoarse, cv::Size(tw / COARSE_STEP, th / COARSE_STEP), 0, 0, cv::INTER_AREA);
    const cv::Rect whole(0, 0, (S.cols / COARSE_STEP) * COARSE_STEP, (S.rows / COARSE_STEP) * COARSE_STEP);
    cv::resize(S(whole), sheetCoarse, cv::Size(whole.width / COARSE_STEP, whole.height / COARSE_STEP), 0, 0, cv::INTER_AREA);
    const int Wt = lookCoarse.cols, Ht = lookCoarse.rows;
    double best = -2.0;
    int bestTurns = -1;
    cv::Point bestAt;
    for (int turns = 0; turns < 4; turns++) {
        cv::Mat turned = lookCoarse;
        if (turns == 1) {
            cv::rotate(lookCoarse, turned, cv::ROTATE_90_CLOCKWISE);
        } else if (turns == 2) {
            cv::rotate(lookCoarse, turned, cv::ROTATE_180);
        } else if (turns == 3) {
            cv::rotate(lookCoarse, turned, cv::ROTATE_90_COUNTERCLOCKWISE);
        }
        if (turned.cols > sheetCoarse.cols || turned.rows > sheetCoarse.rows) {
            continue;
        }
        cv::Mat response;
        cv::matchTemplate(sheetCoarse, turned, response, cv::TM_CCOEFF_NORMED);
        double top;
        cv::Point at;
        cv::minMaxLoc(response, nullptr, &top, nullptr, &at);
        if (top > best) {
            best = top;
            bestTurns = turns;
            bestAt = at;
        }
    }
    if (bestTurns < 0) {
        result.message = QString("The printed target is larger than the sheet it was found on.");
        return result;
    }
    result.matchScore = best;
    result.quarterTurns = bestTurns;
    if (best < MIN_MATCH) {
        result.message = QString("The sheet on the platen doesn't look like this target (best match %1).").arg(best, 0, 'f', 2);
        return result;
    }

    // page mm -> sheet mm, as found: coarse template pixel -> turned -> placed -> sheet mm
    const cv::Matx33d toTemplate(1.0 / coarse, 0, -(origin.x + shift) / coarse, 0, 1.0 / coarse, -(origin.y + shift) / coarse, 0, 0, 1);
    const cv::Matx33d turnings[4] = { cv::Matx33d(1, 0, 0, 0, 1, 0, 0, 0, 1), cv::Matx33d(0, -1, Ht - 1, 1, 0, 0, 0, 0, 1),
                                      cv::Matx33d(-1, 0, Wt - 1, 0, -1, Ht - 1, 0, 0, 1), cv::Matx33d(0, 1, 0, -1, 0, Wt - 1, 0, 0, 1) };
    const cv::Matx33d placed(1, 0, bestAt.x, 0, 1, bestAt.y, 0, 0, 1);
    const cv::Matx33d toSheet(coarse, 0, shift, 0, coarse, shift, 0, 0, 1);
    const cv::Matx33d A3 = toSheet * placed * turnings[bestTurns] * toTemplate;
    cv::Matx23d A(A3(0, 0), A3(0, 1), A3(0, 2), A3(1, 0), A3(1, 1), A3(1, 2));

    // ---- pinned down by local matches at the block's corners and centre, fine, each redrawn at the placement so far
    const double half = std::min(WINDOW_HALF_MM, 0.5 * std::min(result.blockW, result.blockL) + TEMPLATE_MARGIN_MM);
    const int n = (int)std::lround(2.0 * half / RECT_MM), search = (int)std::lround(SEARCH_MM / RECT_MM);
    const std::vector<cv::Point2d> anchors{ { bx0, by0 }, { bx1, by0 }, { bx1, by1 }, { bx0, by1 }, { 0.5 * (bx0 + bx1), 0.5 * (by0 + by1) } };
    std::vector<cv::Point2d> pages, sheets;
    for (int round = 0; round < REFINE_ROUNDS; round++) {
        pages.clear();
        sheets.clear();
        const cv::Matx23d back = inverted(A);
        for (const cv::Point2d &anchor : anchors) {
            const cv::Point2d predicted = apply(A, anchor);
            const cv::Point2i corner((int)std::lround((predicted.x - half) / RECT_MM), (int)std::lround((predicted.y - half) / RECT_MM));
            const cv::Rect area(corner.x - search, corner.y - search, n + 2 * search, n + 2 * search);
            if ((area & cv::Rect(0, 0, S.cols, S.rows)) != area) {
                continue;
            }
            // window pixel q is the sheet point (corner + q) * RECT_MM; its template pixel is (back(that) - origin) / RECT_MM
            const cv::Matx33d windowToSheet(RECT_MM, 0, corner.x * RECT_MM, 0, RECT_MM, corner.y * RECT_MM, 0, 0, 1);
            const cv::Matx33d sheetToPage(back(0, 0), back(0, 1), back(0, 2), back(1, 0), back(1, 1), back(1, 2), 0, 0, 1);
            const cv::Matx33d pageToTemplate(1.0 / RECT_MM, 0, -origin.x / RECT_MM, 0, 1.0 / RECT_MM, -origin.y / RECT_MM, 0, 0, 1);
            const cv::Matx33d M3 = pageToTemplate * sheetToPage * windowToSheet;
            cv::Mat window;
            cv::warpAffine(look, window, cv::Matx23d(M3(0, 0), M3(0, 1), M3(0, 2), M3(1, 0), M3(1, 1), M3(1, 2)), cv::Size(n, n),
                           cv::INTER_LINEAR | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, cv::Scalar(1.0));
            cv::Scalar mean, deviation;
            cv::meanStdDev(window, mean, deviation);
            if (deviation[0] < 0.05) {
                continue;                        // plain paper: nothing to match
            }
            cv::Mat response;
            cv::matchTemplate(S(area), window, response, cv::TM_CCOEFF_NORMED);
            double top;
            cv::Point at;
            cv::minMaxLoc(response, nullptr, &top, nullptr, &at);
            if (top < MIN_WINDOW_MATCH) {
                continue;
            }
            const cv::Point2d moved = (subPixelPeak(response, at) - cv::Point2d(search, search)) * RECT_MM;
            pages.push_back(anchor);
            sheets.push_back(predicted + moved);
        }
        if (pages.size() < 3) {
            result.message = QString("Found roughly where the target is, but couldn't pin it down: only %1 of its corners matched.").arg(pages.size());
            return result;
        }
        // least-squares placement: sheet = A page
        cv::Mat lhs((int)pages.size() * 2, 6, CV_64F, cv::Scalar(0)), rhs((int)pages.size() * 2, 1, CV_64F), x;
        for (int k = 0; k < (int)pages.size(); k++) {
            lhs.at<double>(2 * k, 0) = pages[k].x;
            lhs.at<double>(2 * k, 1) = pages[k].y;
            lhs.at<double>(2 * k, 2) = 1.0;
            lhs.at<double>(2 * k + 1, 3) = pages[k].x;
            lhs.at<double>(2 * k + 1, 4) = pages[k].y;
            lhs.at<double>(2 * k + 1, 5) = 1.0;
            rhs.at<double>(2 * k) = sheets[k].x;
            rhs.at<double>(2 * k + 1) = sheets[k].y;
        }
        cv::solve(lhs, rhs, x, cv::DECOMP_SVD);
        A = cv::Matx23d(x.at<double>(0), x.at<double>(1), x.at<double>(2), x.at<double>(3), x.at<double>(4), x.at<double>(5));
    }
    double squares = 0.0;
    for (size_t k = 0; k < pages.size(); k++) {
        const cv::Point2d d = apply(A, pages[k]) - sheets[k];
        squares += d.dot(d);
    }
    result.windowsUsed = (int)pages.size();
    result.fitRmsMM = std::sqrt(squares / pages.size());

    // ---- every patch on the mosaic
    const auto toMosaic = [&](const std::vector<cv::Point2d> &page) {
        std::vector<cv::Point2d> rect;
        for (const cv::Point2d &p : page) {
            rect.push_back(apply(A, p) * (1.0 / RECT_MM));
        }
        return transformed(rect, H);
    };
    std::vector<cv::Point2d> centres;
    std::vector<std::vector<cv::Point2d>> swatches;
    for (const LAUPrintPatch &p : target.patches) {
        centres.push_back(p.centreMM());
        const cv::Rect2d &b = p.boxMM;
        swatches.push_back(toMosaic({ { b.x, b.y }, { b.x + b.width, b.y }, { b.x + b.width, b.y + b.height }, { b.x, b.y + b.height } }));
        result.locations << p.location;
    }
    result.centres = toMosaic(centres);
    result.block = toMosaic({ { bx0, by0 }, { bx1, by0 }, { bx1, by1 }, { bx0, by1 } });
    result.swatchW = target.patches.first().boxMM.width;
    result.swatchH = target.patches.first().boxMM.height;
    result.check = drawCheck(mosaic, result, swatches);

    result.ok = true;
    result.message = QString("Found the target: %1 patches in a %2 x %3 mm block on a %4 x %5 mm sheet, turned %6 deg, match %7, "
                             "%8 local matches agreeing to %9 mm.")
                         .arg(target.patches.count()).arg(result.blockW, 0, 'f', 1).arg(result.blockL, 0, 'f', 1)
                         .arg(result.sheetW, 0, 'f', 1).arg(result.sheetL, 0, 'f', 1).arg(90 * bestTurns)
                         .arg(best, 0, 'f', 2).arg(result.windowsUsed).arg(result.fitRmsMM, 0, 'f', 2);
    return result;
}
