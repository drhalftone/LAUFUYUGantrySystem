#ifndef LAUARGYLLCHART_H
#define LAUARGYLLCHART_H

#include <QList>
#include <QPair>
#include <QRectF>
#include <QString>
#include <QVector>
#include <QStringList>

// SPECTRAL SAMPLING SHARED BY THE I1PRO SDK (SPECTRUM_SIZE) AND THE .TI3 SPEC_XXX FIELDS
#define LAU_SPECTRUM_SIZE      36      // 380-730 NM IN 10 NM STEPS
#define LAU_SPECTRUM_START_NM  380
#define LAU_SPECTRUM_STEP_NM   10
#define LAU_DENSITY_SIZE       4

/****************************************************************************/
/* LAUColorPatch                                                            */
/*                                                                          */
/* One test patch, followed from targen's .ti1 through the printed chart    */
/* layout to the i1Pro measurement that ends up in the .ti3.  Device values */
/* use Argyll's 0-100 scale; colorimetry uses Argyll's D50, Y = 100 scale.  */
/****************************************************************************/
class LAUColorPatch
{
public:
    int id = 0;                                   // SAMPLE_ID, 1-BASED, KEPT FROM THE .TI1 TO THE .TI3
    QString location;                             // SAMPLE_LOC ON THE PRINTED CHART, E.G. "A1"
    int row = -1;                                 // SCAN LINE ON THE CHART, -1 UNTIL LAID OUT
    int col = -1;                                 // POSITION ALONG THE SCAN LINE, -1 UNTIL LAID OUT
    QRectF rectMM;                                // PATCH RECTANGLE IN CHART MILLIMETRES, EMPTY UNTIL LAID OUT

    QVector<double> device;                       // DEVICE VALUES 0-100, ORDERED AS LAUColorChart::deviceFields()

    bool hasExpected = false;                     // TRUE IF THE .TI1 CARRIED TARGEN'S PREDICTED XYZ
    double expectedXYZ[3] = { 0.0, 0.0, 0.0 };    // PREDICTED XYZ, USED TO ORDER PATCHES FOR SCAN CONTRAST

    bool measured = false;
    double xyz[3] = { 0.0, 0.0, 0.0 };            // MEASURED XYZ, D50 / 2 DEGREE, Y NORMALIZED TO 100
    float spectrum[LAU_SPECTRUM_SIZE] = {};       // MEASURED REFLECTANCE, 0-1
    float density[LAU_DENSITY_SIZE] = {};         // MEASURED C, M, Y, K STATUS DENSITIES

    void clearMeasurement();
};

/****************************************************************************/
/* COLOR MATH SHARED BY THE LAYOUT (PATCH CONTRAST) AND THE SCAN (ORDERING) */
/****************************************************************************/
namespace LAUColor
{
    void xyzToLab(const double xyz[3], double lab[3]);       // D50 WHITE, XYZ WITH Y = 100
    double deltaE76(const double labA[3], const double labB[3]);
    int wavelength(int band);                                // NM OF SPECTRAL BAND 0..LAU_SPECTRUM_SIZE-1
}

/****************************************************************************/
/* LAUColorChart                                                            */
/*                                                                          */
/* A set of patches plus the Argyll metadata needed to round-trip them:     */
/*   targen  -> .ti1  -> readTI1()                                          */
/*   layout  -> writeTI2()  (patch locations, for the record / chartread)   */
/*   i1Pro   -> writeTI3()  -> colprof -> .icc                              */
/****************************************************************************/
class LAUColorChart
{
public:
    QString colorRep = QString("CMYK");               // ARGYLL DEVICE SPACE, E.G. "CMYK", "RGB" OR "iRGB" (PRINT RGB)
    QList<QPair<QString, QString>> keywords;          // EXTRA HEADER KEYWORDS FROM THE .TI1 (TOTAL_INK_LIMIT, ...)
    QList<LAUColorPatch> patches;

    bool isEmpty() const
    {
        return (patches.isEmpty());
    }

    int measuredCount() const;
    QString keyword(const QString &key) const;
    void setKeyword(const QString &key, const QString &value);

    // CGATS FIELD NAMES OF THE DEVICE CHANNELS, E.G. CMYK_C CMYK_M CMYK_Y CMYK_K
    QStringList deviceFields() const;
    static QStringList deviceFields(const QString &rep);

    // DEVICE SPACE WITHOUT ARGYLL'S 'i' (PRINT) PREFIX: "iRGB" -> "RGB"
    QString deviceSpace() const
    {
        return (deviceSpace(colorRep));
    }
    static QString deviceSpace(const QString &rep);

    // READS THE FIRST TABLE OF ANY ARGYLL CGATS FILE (.TI1, .TI2 OR .TI3): DEVICE VALUES,
    // SAMPLE_LOC, XYZ (AS EXPECTED VALUES FOR .TI1/.TI2, AS MEASUREMENTS FOR .TI3) AND SPEC_XXX
    static LAUColorChart readArgyll(const QString &filename, QString *error = nullptr);

    bool writeTI2(const QString &filename, QString *error = nullptr) const;
    bool writeTI3(const QString &filename, QString *error = nullptr) const;

    // SELF-DESCRIBING CSV (HEADER-KEYED, SO OLDER LAUXRiteController CSV FILES ALSO LOAD)
    bool writeCSV(const QString &filename, QString *error = nullptr) const;
    static LAUColorChart readCSV(const QString &filename, QString *error = nullptr);

    // EXCHANGE FORMATS FOR TOOLS OTHER THAN ARGYLL
    bool exportI1ProfilerCGATS(const QString &filename, QString *error = nullptr) const;
    bool exportCxF3(const QString &filename, QString *error = nullptr) const;
};

#endif // LAUARGYLLCHART_H
