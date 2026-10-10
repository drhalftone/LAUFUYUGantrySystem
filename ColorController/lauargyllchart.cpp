#include "lauargyllchart.h"

#include <QDate>
#include <QFile>
#include <QHash>
#include <QDateTime>
#include <QTextStream>
#include <QXmlStreamWriter>
#include <cmath>

// ICC D50 WHITE POINT, Y = 100 (ARGYLL'S PCS WHITE)
static const double D50_WHITE[3] = { 96.42, 100.0, 82.49 };

// HEADER KEYWORDS THAT CGATS DEFINES ITSELF AND SO MUST NOT BE RE-DECLARED WITH 'KEYWORD'
static const QStringList STANDARD_KEYWORDS = QStringList() << "DESCRIPTOR" << "ORIGINATOR" << "CREATED";

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUColorPatch::clearMeasurement()
{
    measured = false;
    for (int k = 0; k < 3; k++) {
        xyz[k] = 0.0;
    }
    for (int k = 0; k < LAU_SPECTRUM_SIZE; k++) {
        spectrum[k] = 0.0f;
    }
    for (int k = 0; k < LAU_DENSITY_SIZE; k++) {
        density[k] = 0.0f;
    }
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUColor::xyzToLab(const double xyz[3], double lab[3])
{
    double f[3];
    for (int k = 0; k < 3; k++) {
        double t = xyz[k] / D50_WHITE[k];
        f[k] = (t > 216.0 / 24389.0) ? std::cbrt(t) : (24389.0 / 27.0 * t + 16.0) / 116.0;
    }
    lab[0] = 116.0 * f[1] - 16.0;
    lab[1] = 500.0 * (f[0] - f[1]);
    lab[2] = 200.0 * (f[1] - f[2]);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
double LAUColor::deltaE76(const double labA[3], const double labB[3])
{
    double dL = labA[0] - labB[0];
    double da = labA[1] - labB[1];
    double db = labA[2] - labB[2];
    return (std::sqrt(dL * dL + da * da + db * db));
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
int LAUColor::wavelength(int band)
{
    return (LAU_SPECTRUM_START_NM + band * LAU_SPECTRUM_STEP_NM);
}

/****************************************************************************/
/* CGATS HELPERS                                                            */
/****************************************************************************/
namespace
{
    struct CgatsTable {
        QString fileType;
        QList<QPair<QString, QString>> keywords;
        QStringList fields;
        QList<QStringList> rows;
    };

    // SPLIT A CGATS LINE ON WHITESPACE, KEEPING "QUOTED STRINGS" TOGETHER (QUOTES REMOVED)
    QStringList tokenize(const QString &line)
    {
        QStringList tokens;
        QString current;
        bool inQuotes = false;
        bool hasToken = false;
        for (QChar ch : line) {
            if (ch == QChar('"')) {
                inQuotes = !inQuotes;
                hasToken = true;
            } else if (ch.isSpace() && !inQuotes) {
                if (hasToken) {
                    tokens << current;
                    current.clear();
                    hasToken = false;
                }
            } else {
                current.append(ch);
                hasToken = true;
            }
        }
        if (hasToken) {
            tokens << current;
        }
        return (tokens);
    }

    // READ ONLY THE FIRST TABLE; TARGEN APPENDS EXTRA TABLES (DENSITY EXTREMES, ETC.) WE DON'T NEED
    bool readFirstTable(const QString &filename, CgatsTable &table, QString *error)
    {
        QFile file(filename);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            if (error) {
                *error = QString("Unable to open %1 for reading.").arg(filename);
            }
            return (false);
        }

        enum { StateHeader, StateFormat, StateData } state = StateHeader;
        QTextStream stream(&file);
        while (!stream.atEnd()) {
            QString line = stream.readLine().trimmed();
            if (line.isEmpty() || line.startsWith(QChar('#'))) {
                continue;
            }
            QStringList tokens = tokenize(line);
            if (tokens.isEmpty()) {
                continue;
            }

            if (table.fileType.isEmpty()) {
                table.fileType = tokens.first();
                continue;
            }

            if (state == StateHeader) {
                const QString &key = tokens.first();
                if (key == QString("BEGIN_DATA_FORMAT")) {
                    state = StateFormat;
                } else if (key == QString("BEGIN_DATA")) {
                    state = StateData;
                } else if (key == QString("KEYWORD") || key == QString("NUMBER_OF_FIELDS") || key == QString("NUMBER_OF_SETS")) {
                    continue;
                } else if (tokens.count() >= 2) {
                    table.keywords << qMakePair(key, tokens.mid(1).join(QChar(' ')));
                }
            } else if (state == StateFormat) {
                if (tokens.first() == QString("END_DATA_FORMAT")) {
                    state = StateHeader;
                } else {
                    table.fields << tokens;
                }
            } else {
                if (tokens.first() == QString("END_DATA")) {
                    return (true);
                }
                table.rows << tokens;
            }
        }

        if (error) {
            *error = QString("%1 ended before END_DATA.").arg(filename);
        }
        return (false);
    }

    void writeKeyword(QTextStream &stream, const QString &key, const QString &value)
    {
        if (!STANDARD_KEYWORDS.contains(key)) {
            stream << "KEYWORD \"" << key << "\"\n";
        }
        stream << key << " \"" << value << "\"\n";
    }

    QString number(double value, int precision = 5)
    {
        return (QString::number(value, 'f', precision));
    }

    QString locationOf(const LAUColorPatch &patch)
    {
        return (patch.location.isEmpty() ? QString::number(patch.id) : patch.location);
    }

    bool openForWriting(QFile &file, QString *error)
    {
        if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            return (true);
        }
        if (error) {
            *error = QString("Unable to open %1 for writing.").arg(file.fileName());
        }
        return (false);
    }
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
int LAUColorChart::measuredCount() const
{
    int count = 0;
    for (const LAUColorPatch &patch : patches) {
        if (patch.measured) {
            count++;
        }
    }
    return (count);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
QString LAUColorChart::keyword(const QString &key) const
{
    for (const QPair<QString, QString> &pair : keywords) {
        if (pair.first == key) {
            return (pair.second);
        }
    }
    return (QString());
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUColorChart::setKeyword(const QString &key, const QString &value)
{
    for (QPair<QString, QString> &pair : keywords) {
        if (pair.first == key) {
            pair.second = value;
            return;
        }
    }
    keywords << qMakePair(key, value);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
QStringList LAUColorChart::deviceFields() const
{
    return (deviceFields(colorRep));
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
QStringList LAUColorChart::deviceFields(const QString &rep)
{
    // ARGYLL NAMES EACH CHANNEL <SPACE>_<LETTER>, E.G. CMYK_C, RGB_R.  A LEADING 'i' (E.G. "iRGB",
    // targen's "Print RGB") MARKS AN INVERTED/PRINT DEVICE SPACE BUT IS NOT PART OF THE FIELD NAMES.
    QString space = deviceSpace(rep);
    QStringList fields;
    for (QChar letter : space) {
        fields << QString("%1_%2").arg(space).arg(letter);
    }
    return (fields);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
QString LAUColorChart::deviceSpace(const QString &rep)
{
    return (rep.startsWith(QChar('i')) ? rep.mid(1) : rep);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
LAUColorChart LAUColorChart::readArgyll(const QString &filename, QString *error)
{
    LAUColorChart chart;
    CgatsTable table;
    if (!readFirstTable(filename, table, error)) {
        return (LAUColorChart());
    }

    // .TI1/.TI2 GIVE "CMYK"; .TI3 GIVES "CMYK_XYZ" OR "CMYK_LAB" -- KEEP THE DEVICE SIDE
    chart.keywords = table.keywords;
    QString rep = chart.keyword(QString("COLOR_REP"));
    if (!rep.isEmpty()) {
        chart.colorRep = rep.section(QChar('_'), 0, 0);
        chart.setKeyword(QString("COLOR_REP"), chart.colorRep);
    }
    bool isMeasurement = table.fileType.startsWith(QString("CTI3"));

    // LOCATE EACH FIELD WE UNDERSTAND
    QStringList devFields = chart.deviceFields();
    QVector<int> devIndex;
    for (const QString &field : devFields) {
        devIndex << table.fields.indexOf(field);
    }
    if (devIndex.contains(-1)) {
        if (error) {
            *error = QString("%1 has no %2 device fields.").arg(filename).arg(chart.colorRep);
        }
        return (LAUColorChart());
    }
    int idIndex  = table.fields.indexOf(QString("SAMPLE_ID"));
    int locIndex = table.fields.indexOf(QString("SAMPLE_LOC"));
    int xyzIndex[3];
    for (int k = 0; k < 3; k++) {
        xyzIndex[k] = table.fields.indexOf(QString("XYZ_%1").arg(QChar("XYZ"[k])));
    }
    bool hasXYZ = (xyzIndex[0] >= 0 && xyzIndex[1] >= 0 && xyzIndex[2] >= 0);
    QVector<int> specIndex;
    for (int k = 0; k < LAU_SPECTRUM_SIZE; k++) {
        specIndex << table.fields.indexOf(QString("SPEC_%1").arg(LAUColor::wavelength(k)));
    }
    bool hasSpectrum = !specIndex.contains(-1);

    for (int r = 0; r < table.rows.count(); r++) {
        const QStringList &row = table.rows.at(r);
        if (row.count() != table.fields.count()) {
            if (error) {
                *error = QString("%1: data row %2 has %3 values, expected %4.").arg(filename).arg(r + 1).arg(row.count()).arg(table.fields.count());
            }
            return (LAUColorChart());
        }

        LAUColorPatch patch;
        patch.id = (idIndex >= 0) ? row.at(idIndex).toInt() : r + 1;
        if (locIndex >= 0) {
            patch.location = row.at(locIndex);
        }
        for (int index : devIndex) {
            patch.device << row.at(index).toDouble();
        }
        if (hasXYZ) {
            double *target = isMeasurement ? patch.xyz : patch.expectedXYZ;
            for (int k = 0; k < 3; k++) {
                target[k] = row.at(xyzIndex[k]).toDouble();
            }
            patch.hasExpected = !isMeasurement;
            patch.measured = isMeasurement;
        }
        if (hasSpectrum) {
            // .TI3 SPECTRA ARE PERCENT; WE KEEP THE I1PRO'S 0-1 REFLECTANCE
            for (int k = 0; k < LAU_SPECTRUM_SIZE; k++) {
                patch.spectrum[k] = static_cast<float>(row.at(specIndex.at(k)).toDouble() / 100.0);
            }
        }
        chart.patches << patch;
    }
    return (chart);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
bool LAUColorChart::writeTI2(const QString &filename, QString *error) const
{
    QFile file(filename);
    if (!openForWriting(file, error)) {
        return (false);
    }

    // CARRY THE .TI1 KEYWORDS THROUGH, AS PRINTTARG DOES
    QTextStream stream(&file);
    stream << "CTI2\n\n";
    writeKeyword(stream, QString("DESCRIPTOR"), QString("Argyll Calibration Target chart information 2"));
    writeKeyword(stream, QString("ORIGINATOR"), QString("LAU ColorController"));
    writeKeyword(stream, QString("CREATED"), QDateTime::currentDateTime().toString(Qt::TextDate));
    bool wroteRep = false;
    for (const QPair<QString, QString> &pair : keywords) {
        if (STANDARD_KEYWORDS.contains(pair.first)) {
            continue;
        }
        if (pair.first == QString("COLOR_REP")) {
            wroteRep = true;
        }
        writeKeyword(stream, pair.first, pair.second);
    }
    if (!wroteRep) {
        writeKeyword(stream, QString("COLOR_REP"), colorRep);
    }

    bool hasExpected = !patches.isEmpty();
    for (const LAUColorPatch &patch : patches) {
        hasExpected = hasExpected && patch.hasExpected;
    }

    QStringList fields = QStringList() << "SAMPLE_ID" << "SAMPLE_LOC" << deviceFields();
    if (hasExpected) {
        fields << "XYZ_X" << "XYZ_Y" << "XYZ_Z";
    }
    stream << "\nNUMBER_OF_FIELDS " << fields.count() << "\n";
    stream << "BEGIN_DATA_FORMAT\n" << fields.join(QChar(' ')) << "\nEND_DATA_FORMAT\n\n";
    stream << "NUMBER_OF_SETS " << patches.count() << "\n";
    stream << "BEGIN_DATA\n";
    for (const LAUColorPatch &patch : patches) {
        stream << patch.id << " \"" << locationOf(patch) << "\"";
        for (double value : patch.device) {
            stream << " " << number(value);
        }
        if (hasExpected) {
            for (int k = 0; k < 3; k++) {
                stream << " " << number(patch.expectedXYZ[k]);
            }
        }
        stream << "\n";
    }
    stream << "END_DATA\n";
    return (true);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
bool LAUColorChart::writeTI3(const QString &filename, QString *error) const
{
    if (measuredCount() == 0) {
        if (error) {
            *error = QString("No measured patches to write.");
        }
        return (false);
    }

    QFile file(filename);
    if (!openForWriting(file, error)) {
        return (false);
    }

    // SAME LAYOUT CHARTREAD PRODUCES: DEVICE VALUES + XYZ + FULL SPECTRUM, SO COLPROF CAN
    // ALSO APPLY FWA COMPENSATION AND ALTERNATE ILLUMINANTS FROM THE SPECTRA
    QTextStream stream(&file);
    stream << "CTI3\n\n";
    writeKeyword(stream, QString("DESCRIPTOR"), QString("Argyll Calibration Target chart information 3"));
    writeKeyword(stream, QString("ORIGINATOR"), QString("LAU ColorController"));
    writeKeyword(stream, QString("CREATED"), QDateTime::currentDateTime().toString(Qt::TextDate));
    writeKeyword(stream, QString("DEVICE_CLASS"), QString("OUTPUT"));
    writeKeyword(stream, QString("COLOR_REP"), QString("%1_XYZ").arg(colorRep));
    // COLPROF ONLY USES THIS FOR FWA COMPENSATION (-f), TO LOOK UP THE INSTRUMENT'S LAMP; ANY i1Pro NAME ARGYLL KNOWS WILL DO
    writeKeyword(stream, QString("TARGET_INSTRUMENT"), QString("Xrite i1 Pro"));
    writeKeyword(stream, QString("INSTRUMENT_TYPE_SPECTRAL"), QString("YES"));
    QString inkLimit = keyword(QString("TOTAL_INK_LIMIT"));
    if (!inkLimit.isEmpty()) {
        writeKeyword(stream, QString("TOTAL_INK_LIMIT"), inkLimit);
    }
    writeKeyword(stream, QString("SPECTRAL_BANDS"), QString::number(LAU_SPECTRUM_SIZE));
    writeKeyword(stream, QString("SPECTRAL_START_NM"), number(LAUColor::wavelength(0), 6));
    writeKeyword(stream, QString("SPECTRAL_END_NM"), number(LAUColor::wavelength(LAU_SPECTRUM_SIZE - 1), 6));

    QStringList fields = QStringList() << "SAMPLE_ID" << "SAMPLE_LOC" << deviceFields() << "XYZ_X" << "XYZ_Y" << "XYZ_Z";
    for (int k = 0; k < LAU_SPECTRUM_SIZE; k++) {
        fields << QString("SPEC_%1").arg(LAUColor::wavelength(k));
    }
    stream << "\nNUMBER_OF_FIELDS " << fields.count() << "\n";
    stream << "BEGIN_DATA_FORMAT\n" << fields.join(QChar(' ')) << "\nEND_DATA_FORMAT\n\n";
    stream << "NUMBER_OF_SETS " << measuredCount() << "\n";
    stream << "BEGIN_DATA\n";
    for (const LAUColorPatch &patch : patches) {
        if (!patch.measured) {
            continue;
        }
        stream << patch.id << " \"" << locationOf(patch) << "\"";
        for (double value : patch.device) {
            stream << " " << number(value);
        }
        for (int k = 0; k < 3; k++) {
            stream << " " << number(patch.xyz[k]);
        }
        for (int k = 0; k < LAU_SPECTRUM_SIZE; k++) {
            stream << " " << number(100.0 * patch.spectrum[k], 4);
        }
        stream << "\n";
    }
    stream << "END_DATA\n";
    return (true);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
bool LAUColorChart::writeCSV(const QString &filename, QString *error) const
{
    QFile file(filename);
    if (!openForWriting(file, error)) {
        return (false);
    }

    QTextStream stream(&file);
    QStringList header = QStringList() << "SAMPLE_ID" << "SAMPLE_LOC" << "ROW" << "COL" << deviceFields()
                         << "MEASURED" << "XYZ_X" << "XYZ_Y" << "XYZ_Z" << "LAB_L" << "LAB_A" << "LAB_B"
                         << "D_C" << "D_M" << "D_Y" << "D_K";
    for (int k = 0; k < LAU_SPECTRUM_SIZE; k++) {
        header << QString("SPEC_%1").arg(LAUColor::wavelength(k));
    }
    stream << header.join(QChar(',')) << "\n";

    for (const LAUColorPatch &patch : patches) {
        double lab[3];
        LAUColor::xyzToLab(patch.xyz, lab);

        QStringList values = QStringList() << QString::number(patch.id) << locationOf(patch) << QString::number(patch.row) << QString::number(patch.col);
        for (double value : patch.device) {
            values << number(value);
        }
        values << QString::number(patch.measured ? 1 : 0);
        for (int k = 0; k < 3; k++) {
            values << number(patch.xyz[k]);
        }
        for (int k = 0; k < 3; k++) {
            values << number(lab[k]);
        }
        for (int k = 0; k < LAU_DENSITY_SIZE; k++) {
            values << number(patch.density[k]);
        }
        for (int k = 0; k < LAU_SPECTRUM_SIZE; k++) {
            values << number(patch.spectrum[k], 6);
        }
        stream << values.join(QChar(',')) << "\n";
    }
    return (true);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
LAUColorChart LAUColorChart::readCSV(const QString &filename, QString *error)
{
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (error) {
            *error = QString("Unable to open %1 for reading.").arg(filename);
        }
        return (LAUColorChart());
    }

    QTextStream stream(&file);
    QStringList header = stream.readLine().split(QChar(','));
    for (QString &name : header) {
        name = name.trimmed();
    }
    auto column = [&header](const QStringList &names) -> int {
        for (const QString &name : names) {
            int index = header.indexOf(name);
            if (index >= 0) {
                return (index);
            }
        }
        return (-1);
    };

    // THE OLD LAUXRiteController CSV USED "sample ID", "cyan" (0-1), ... AND ONE "spectral" COLUMN
    // FOLLOWED BY 35 UNNAMED ONES; MAP BOTH LAYOUTS ONTO THE SAME FIELDS
    LAUColorChart chart;
    bool legacy = header.contains(QString("cyan"));
    double deviceScale = legacy ? 100.0 : 1.0;
    QVector<int> devIndex;
    if (legacy) {
        chart.colorRep = QString("CMYK");
        devIndex << column({ "cyan" }) << column({ "magenta" }) << column({ "yellow" }) << column({ "black" });
    } else {
        for (const QString &name : header) {
            if (name.contains(QChar('_')) && name.section(QChar('_'), 1).length() == 1 && name.section(QChar('_'), 0, 0).length() > 1 &&
                !name.startsWith(QString("XYZ_")) && !name.startsWith(QString("LAB_")) && !name.startsWith(QString("D_"))) {
                chart.colorRep = name.section(QChar('_'), 0, 0);
                break;
            }
        }
        for (const QString &field : chart.deviceFields()) {
            devIndex << column({ field });
        }
    }
    if (devIndex.isEmpty() || devIndex.contains(-1)) {
        if (error) {
            *error = QString("%1 has no device value columns.").arg(filename);
        }
        return (LAUColorChart());
    }

    int idIndex       = column({ "SAMPLE_ID", "sample ID" });
    int locIndex      = column({ "SAMPLE_LOC" });
    int rowIndex      = column({ "ROW", "chart row" });
    int colIndex      = column({ "COL", "chart col" });
    int measuredIndex = column({ "MEASURED" });
    int xyzIndex      = column({ "XYZ_X", "X" });
    int densityIndex  = column({ "D_C", "density(C)" });
    int specIndex     = column({ "SPEC_380", "spectral" });

    int lineNumber = 1;
    while (!stream.atEnd()) {
        lineNumber++;
        QString line = stream.readLine().trimmed();
        if (line.isEmpty()) {
            continue;
        }
        QStringList values = line.split(QChar(','));
        auto value = [&values](int index) -> double {
            return ((index >= 0 && index < values.count()) ? values.at(index).trimmed().toDouble() : 0.0);
        };

        LAUColorPatch patch;
        patch.id = (idIndex >= 0) ? static_cast<int>(value(idIndex)) : chart.patches.count() + 1;
        if (locIndex >= 0 && locIndex < values.count()) {
            patch.location = values.at(locIndex).trimmed();
        }
        patch.row = (rowIndex >= 0) ? static_cast<int>(value(rowIndex)) : -1;
        patch.col = (colIndex >= 0) ? static_cast<int>(value(colIndex)) : -1;
        for (int index : devIndex) {
            patch.device << deviceScale * value(index);
        }
        if (xyzIndex >= 0) {
            for (int k = 0; k < 3; k++) {
                patch.xyz[k] = value(xyzIndex + k);
            }
        }
        if (densityIndex >= 0) {
            for (int k = 0; k < LAU_DENSITY_SIZE; k++) {
                patch.density[k] = static_cast<float>(value(densityIndex + k));
            }
        }
        if (specIndex >= 0 && specIndex + LAU_SPECTRUM_SIZE <= values.count()) {
            for (int k = 0; k < LAU_SPECTRUM_SIZE; k++) {
                patch.spectrum[k] = static_cast<float>(value(specIndex + k));
            }
        }
        // LEGACY FILES HAVE NO FLAG; TREAT A NONZERO Y AS A MEASUREMENT
        patch.measured = (measuredIndex >= 0) ? (value(measuredIndex) != 0.0) : (patch.xyz[1] != 0.0);
        chart.patches << patch;
    }
    return (chart);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
bool LAUColorChart::exportI1ProfilerCGATS(const QString &filename, QString *error) const
{
    QFile file(filename);
    if (!openForWriting(file, error)) {
        return (false);
    }

    QTextStream stream(&file);
    stream << "CGATS.17\n\n";
    stream << "DESCRIPTOR \"i1Profiler CGATS Spectral measurement data\"\n";
    stream << "ORIGINATOR \"LAU ColorController\"\n";
    stream << "CREATED \"" << QDateTime::currentDateTime().toString("ddd MMM dd hh:mm:ss yyyy") << "\"\n";
    stream << "MANUFACTURER \"X-Rite\"\n";
    stream << "PROD_DATE \"" << QDate::currentDate().toString("yyyy:MM:dd") << "\"\n";
    stream << "INSTRUMENTATION \"i1Pro\"\n\n";
    writeKeyword(stream, QString("MEASUREMENT_SOURCE"), QString("Reflectance"));
    writeKeyword(stream, QString("MEASUREMENT_GEOMETRY"), QString("45:0"));
    writeKeyword(stream, QString("ILLUMINATION_NAME"), QString("D50"));
    writeKeyword(stream, QString("OBSERVER_ANGLE"), QString("2"));
    writeKeyword(stream, QString("SPECTRAL_BANDS"), QString::number(LAU_SPECTRUM_SIZE));
    writeKeyword(stream, QString("SPECTRAL_START_NM"), QString::number(LAUColor::wavelength(0)));
    writeKeyword(stream, QString("SPECTRAL_END_NM"), QString::number(LAUColor::wavelength(LAU_SPECTRUM_SIZE - 1)));
    writeKeyword(stream, QString("SPECTRAL_NORM"), QString("1.00"));

    QStringList fields = QStringList() << "SAMPLE_ID" << "SAMPLE_NAME" << deviceFields() << "XYZ_X" << "XYZ_Y" << "XYZ_Z" << "LAB_L" << "LAB_A" << "LAB_B";
    for (int k = 0; k < LAU_SPECTRUM_SIZE; k++) {
        fields << QString("SPECTRAL_%1").arg(LAUColor::wavelength(k));
    }
    stream << "\nNUMBER_OF_FIELDS " << fields.count() << "\n";
    stream << "BEGIN_DATA_FORMAT\n" << fields.join(QChar(' ')) << "\nEND_DATA_FORMAT\n\n";
    stream << "NUMBER_OF_SETS " << measuredCount() << "\n";
    stream << "BEGIN_DATA\n";
    for (const LAUColorPatch &patch : patches) {
        if (!patch.measured) {
            continue;
        }
        double lab[3];
        LAUColor::xyzToLab(patch.xyz, lab);
        stream << patch.id << " \"" << locationOf(patch) << "\"";
        for (double value : patch.device) {
            stream << " " << number(value, 3);
        }
        for (int k = 0; k < 3; k++) {
            stream << " " << number(patch.xyz[k], 4);
        }
        for (int k = 0; k < 3; k++) {
            stream << " " << number(lab[k], 4);
        }
        for (int k = 0; k < LAU_SPECTRUM_SIZE; k++) {
            stream << " " << number(patch.spectrum[k], 4);
        }
        stream << "\n";
    }
    stream << "END_DATA\n";
    return (true);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
bool LAUColorChart::exportCxF3(const QString &filename, QString *error) const
{
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly)) {
        if (error) {
            *error = QString("Unable to open %1 for writing.").arg(filename);
        }
        return (false);
    }

    QXmlStreamWriter xml(&file);
    xml.setAutoFormatting(true);
    xml.setAutoFormattingIndent(2);
    xml.writeStartDocument(QString("1.0"));
    xml.writeStartElement(QString("CxF"));
    xml.writeAttribute(QString("xmlns"), QString("http://colorexchangeformat.com/CxF3-core"));
    xml.writeAttribute(QString("xmlns:xsi"), QString("http://www.w3.org/2001/XMLSchema-instance"));
    xml.writeAttribute(QString("xsi:schemaLocation"), QString("http://colorexchangeformat.com/CxF3-core http://colorexchangeformat.com/CxF3-core.xsd"));

    xml.writeStartElement(QString("FileInformation"));
    xml.writeTextElement(QString("Creator"), QString("LAU ColorController"));
    xml.writeTextElement(QString("CreationDate"), QDateTime::currentDateTime().toString(Qt::ISODate));
    xml.writeTextElement(QString("Description"), QString("Spectral measurement data from an X-Rite i1Pro"));
    xml.writeEndElement();

    xml.writeStartElement(QString("Resources"));
    xml.writeStartElement(QString("ObjectCollection"));
    for (const LAUColorPatch &patch : patches) {
        if (!patch.measured) {
            continue;
        }
        xml.writeStartElement(QString("Object"));
        xml.writeAttribute(QString("Id"), QString("Sample_%1").arg(patch.id));
        xml.writeAttribute(QString("Name"), locationOf(patch));
        xml.writeAttribute(QString("ObjectType"), QString("Standard"));
        xml.writeStartElement(QString("ColorValues"));

        QStringList spectrum;
        for (int k = 0; k < LAU_SPECTRUM_SIZE; k++) {
            spectrum << number(patch.spectrum[k], 4);
        }
        xml.writeStartElement(QString("ReflectanceSpectrum"));
        xml.writeAttribute(QString("StartWL"), QString::number(LAUColor::wavelength(0)));
        xml.writeAttribute(QString("EndWL"), QString::number(LAUColor::wavelength(LAU_SPECTRUM_SIZE - 1)));
        xml.writeAttribute(QString("Increment"), QString::number(LAU_SPECTRUM_STEP_NM));
        xml.writeAttribute(QString("Name"), QString("MeasuredSpectrum"));
        xml.writeCharacters(spectrum.join(QChar(' ')));
        xml.writeEndElement();

        double lab[3];
        LAUColor::xyzToLab(patch.xyz, lab);
        xml.writeStartElement(QString("ColorCIEXYZ"));
        xml.writeAttribute(QString("IlluminantName"), QString("D50"));
        xml.writeAttribute(QString("ObserverAngle"), QString("2"));
        xml.writeTextElement(QString("X"), number(patch.xyz[0], 6));
        xml.writeTextElement(QString("Y"), number(patch.xyz[1], 6));
        xml.writeTextElement(QString("Z"), number(patch.xyz[2], 6));
        xml.writeEndElement();

        xml.writeStartElement(QString("ColorCIELab"));
        xml.writeAttribute(QString("IlluminantName"), QString("D50"));
        xml.writeAttribute(QString("ObserverAngle"), QString("2"));
        xml.writeTextElement(QString("L"), number(lab[0], 4));
        xml.writeTextElement(QString("A"), number(lab[1], 4));
        xml.writeTextElement(QString("B"), number(lab[2], 4));
        xml.writeEndElement();

        xml.writeEndElement(); // ColorValues
        xml.writeEndElement(); // Object
    }
    xml.writeEndElement(); // ObjectCollection
    xml.writeEndElement(); // Resources
    xml.writeEndElement(); // CxF
    xml.writeEndDocument();
    return (true);
}
