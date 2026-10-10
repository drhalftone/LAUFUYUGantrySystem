#include "lauchartlayout.h"

#include <QPainter>
#include <cmath>

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
int LAUChartLayout::patchesPerLine(double scanLengthMM) const
{
    // N PATCHES NEED N*WIDTH + (N-1)*BAR + 2*MARGIN
    int count = static_cast<int>(std::floor((scanLengthMM - 2.0 * marginMM + barWidthMM) / (patchWidthMM + barWidthMM)));
    return (count >= 6 ? count : 0);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
int LAUChartLayout::lines(double crossLengthMM) const
{
    return (qMax(0, static_cast<int>(std::floor(crossLengthMM / patchHeightMM))));
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
QString LAUChartLayout::locationName(int row, int col)
{
    // ROWS A..Z, AA..AZ, BA.. ; COLUMNS 1-BASED, E.G. "C12"
    QString letters;
    int n = row;
    do {
        letters.prepend(QChar('A' + n % 26));
        n = n / 26 - 1;
    } while (n >= 0);
    return (QString("%1%2").arg(letters).arg(col + 1));
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
int LAUChartLayout::layout(LAUColorChart &chart, const QSizeF &areaMM)
{
    barList.clear();
    int perLine = patchesPerLine(areaMM.width());
    int numLines = lines(areaMM.height());
    if (perLine == 0 || numLines == 0) {
        size = QSizeF();
        return (0);
    }

    // CAPACITY RESERVES A BAR BETWEEN EVERY PAIR, BUT PATCHES ONLY GET ONE WHERE THEY NEED IT;
    // ELSEWHERE THEY ABUT, SO NO UNINTENDED WHITE GAP APPEARS BETWEEN TWO LIGHT PATCHES
    int placed = qMin(chart.patches.count(), perLine * numLines);
    int usedLines = (placed + perLine - 1) / perLine;
    double lineLength = 0.0;
    double x = marginMM;

    for (int n = 0; n < chart.patches.count(); n++) {
        LAUColorPatch &patch = chart.patches[n];
        if (n >= placed) {
            patch.row = -1;
            patch.col = -1;
            patch.rectMM = QRectF();
            continue;
        }
        patch.row = n / perLine;
        patch.col = n % perLine;
        patch.location = locationName(patch.row, patch.col);
        if (patch.col == 0) {
            x = marginMM;
        }

        // SEPARATION BAR BEFORE THIS PATCH IF IT IS TOO CLOSE IN COLOR TO ITS LEFT NEIGHBOR
        // (WITHOUT PREDICTED COLORS WE CAN'T TELL, SO BAR EVERY GAP TO BE SAFE)
        if (patch.col > 0) {
            const LAUColorPatch &previous = chart.patches.at(n - 1);
            bool needBar = true;
            double meanL = 50.0;
            if (patch.hasExpected && previous.hasExpected) {
                double labA[3], labB[3];
                LAUColor::xyzToLab(previous.expectedXYZ, labA);
                LAUColor::xyzToLab(patch.expectedXYZ, labB);
                needBar = (LAUColor::deltaE76(labA, labB) <= minDeltaE);
                meanL = 0.5 * (labA[0] + labB[0]);
            }
            if (needBar) {
                // BLACK AGAINST LIGHT PATCHES, WHITE AGAINST DARK ONES
                Bar bar;
                bar.black = (meanL >= 50.0);
                bar.rectMM = QRectF(x, patch.row * patchHeightMM, barWidthMM, patchHeightMM);
                barList << bar;
                x += barWidthMM;
            }
        }
        patch.rectMM = QRectF(x, patch.row * patchHeightMM, patchWidthMM, patchHeightMM);
        x += patchWidthMM;
        lineLength = qMax(lineLength, x + marginMM);
    }
    size = QSizeF(lineLength, usedLines * patchHeightMM);
    return (placed);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
QImage LAUChartLayout::preview(const LAUColorChart &chart, double pixelsPerMM) const
{
    if (size.isEmpty()) {
        return (QImage());
    }

    QImage image(qCeil(size.width() * pixelsPerMM), qCeil(size.height() * pixelsPerMM), QImage::Format_RGB32);
    image.fill(Qt::white);
    QPainter painter(&image);
    painter.scale(pixelsPerMM, pixelsPerMM);
    painter.setPen(Qt::NoPen);

    for (const LAUColorPatch &patch : chart.patches) {
        if (patch.rectMM.isEmpty()) {
            continue;
        }
        if (chart.deviceSpace() == QString("CMYK") && patch.device.count() == 4) {
            double c = patch.device[0] / 100.0, m = patch.device[1] / 100.0, y = patch.device[2] / 100.0, k = patch.device[3] / 100.0;
            painter.setBrush(QColor::fromRgbF((1.0 - c) * (1.0 - k), (1.0 - m) * (1.0 - k), (1.0 - y) * (1.0 - k)));
        } else if (chart.deviceSpace() == QString("RGB") && patch.device.count() == 3) {
            painter.setBrush(QColor::fromRgbF(patch.device[0] / 100.0, patch.device[1] / 100.0, patch.device[2] / 100.0));
        } else {
            continue;
        }
        painter.drawRect(patch.rectMM);
    }
    for (const Bar &bar : barList) {
        painter.setBrush(bar.black ? Qt::black : Qt::white);
        painter.drawRect(bar.rectMM);
    }
    return (image);
}
