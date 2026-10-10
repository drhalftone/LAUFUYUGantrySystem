#ifndef LAUCHARTLAYOUT_H
#define LAUCHARTLAYOUT_H

#include <QList>
#include <QImage>
#include <QRectF>
#include <QSizeF>

#include "lauargyllchart.h"

/****************************************************************************/
/* LAUChartLayout                                                           */
/*                                                                          */
/* Lays patches out in scan lines sized to the gantry's scan area, after    */
/* X-Rite's i1Pro2 chart design rules for scanning without a zebra ruler:   */
/*   - every patch in a line has the same width, >= 10 mm along the scan    */
/*   - patch height >= 8 mm across the scan                                 */
/*   - >= 6 patches per line                                                */
/*   - >= 12 mm of paper white before and after each line                   */
/*   - neighbors differ by > 20 dE, otherwise a 0.5-1.0 mm black or white   */
/*     separation bar (whichever contrasts more) goes between them          */
/* Lines run along X (the dual rails); successive lines step along Y.       */
/****************************************************************************/
class LAUChartLayout
{
public:
    double patchWidthMM  = 10.0;   // ALONG THE SCAN
    double patchHeightMM = 8.0;    // ACROSS THE SCAN
    double barWidthMM    = 1.0;    // SPACE RESERVED BETWEEN EVERY PAIR OF PATCHES
    double marginMM      = 12.0;   // PAPER WHITE AT EACH END OF A LINE
    double minDeltaE     = 20.0;   // NEIGHBORS CLOSER THAN THIS GET A SEPARATION BAR

    struct Bar {
        QRectF rectMM;
        bool black;
    };

    int patchesPerLine(double scanLengthMM) const;
    int lines(double crossLengthMM) const;
    int capacity(const QSizeF &areaMM) const
    {
        return (patchesPerLine(areaMM.width()) * lines(areaMM.height()));
    }

    // ASSIGNS row, col, location AND rectMM TO EVERY PATCH THAT FITS AND RETURNS HOW MANY FIT.
    // POSITIONS ARE IN CHART MILLIMETRES WITH (0,0) AT THE TOP-LEFT CORNER OF THE SCAN AREA.
    int layout(LAUColorChart &chart, const QSizeF &areaMM);

    QSizeF chartSizeMM() const
    {
        return (size);
    }

    const QList<Bar> &bars() const
    {
        return (barList);
    }

    // ON-SCREEN PREVIEW OF CMYK OR RGB CHARTS (NAIVE CONVERSION; NOT COLOR MANAGED)
    QImage preview(const LAUColorChart &chart, double pixelsPerMM = 4.0) const;

    static QString locationName(int row, int col);

private:
    QSizeF size;
    QList<Bar> barList;
};

#endif // LAUCHARTLAYOUT_H
