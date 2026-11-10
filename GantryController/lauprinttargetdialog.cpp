/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
#include "lauprinttargetdialog.h"
#include "lauargyllrunner.h"
#include "laulastdirectory.h"

#include <QDir>
#include <QUrl>
#include <QFile>
#include <QLabel>
#include <cstring>
#include <QMap>
#include <QDateTime>
#include <QDataStream>
#include <QFileInfo>
#include <QFileDialog>
#include <QMessageBox>
#include <QStandardPaths>
#include <QSpinBox>
#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QPlainTextEdit>
#include <QDoubleSpinBox>
#include <QDesktopServices>
#include <QDialogButtonBox>
#include <QRegularExpression>

namespace
{
    const double MIN_SIDE_MM = 100.0;
    const double PAGE_MARGIN_MM = 6.0;           // white border, kept inside the TIFF so its size is the size asked for
    const int PROBE_PATCHES = 30;
    const int MAX_ATTEMPTS = 6;
    const int FEWEST_PATCHES = 50;

    const int TIFF_ENTRY_BYTES = 12;
    const int TIFF_TAG_XMP = 700;
    const int TIFF_TYPE_BYTE = 1;
    const QString CHART_NAMESPACE = QStringLiteral("urn:lau:gantry-print-target:1");   // <lau:ti1> <lau:ti2> <lau:cht> ...: see LAUPrintTarget

    int firstNumber(const QString &text, const QString &pattern)
    {
        const QRegularExpressionMatch match = QRegularExpression(pattern).match(text);
        return match.hasMatch() ? match.captured(1).toInt() : -1;
    }
}

LAUPrintTargetDialog::LAUPrintTargetDialog(double maxWidthMM, double maxHeightMM, const QString &baseFolder, QWidget *parent)
    : QDialog(parent), maxWidth(maxWidthMM), maxHeight(maxHeightMM), base(baseFolder)
{
    setWindowTitle(QString("Generate Print Target"));
    resize(560, 520);

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(new QLabel(QString("The largest target the i1 can reach is %1 x %2 mm (%3 x %4 in).")
                                     .arg(maxWidth, 0, 'f', 0).arg(maxHeight, 0, 'f', 0)
                                     .arg(maxWidth / 25.4, 0, 'f', 1).arg(maxHeight / 25.4, 0, 'f', 1)));

    QFormLayout *form = new QFormLayout();
    const auto sizeBox = [](double maximum) {
        QDoubleSpinBox *box = new QDoubleSpinBox();
        box->setRange(qMin(MIN_SIDE_MM, maximum), maximum);
        box->setDecimals(0);
        box->setSuffix(QString(" mm"));
        box->setValue(maximum);
        return box;
    };
    widthBox = sizeBox(maxWidth);
    heightBox = sizeBox(maxHeight);
    connect(widthBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &LAUPrintTargetDialog::onSizeChanged);
    connect(heightBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &LAUPrintTargetDialog::onSizeChanged);
    inchLabel = new QLabel();
    form->addRow(QString("Width (along X, the single rail):"), widthBox);
    form->addRow(QString("Height (along Y, the rail pair):"), heightBox);
    form->addRow(QString(), inchLabel);

    patchBox = new QSpinBox();
    patchBox->setRange(0, 20000);
    patchBox->setSingleStep(100);
    patchBox->setSpecialValueText(QString("As many as fit"));
    patchBox->setValue(0);
    form->addRow(QString("Patches:"), patchBox);

    modeBox = new QComboBox();
    modeBox->addItem(QString("Sliding (the i1 reads while moving)"), false);
    modeBox->addItem(QString("Stop and go (the i1 stops on each patch)"), true);
    modeBox->setToolTip(QString("Sliding charts have spacers between patches so the i1 can tell them apart on the move. "
                                "Stop-and-go charts don't need them, so more patches fit."));
    form->addRow(QString("How the i1 will read it:"), modeBox);

    dpiBox = new QComboBox();
    for (int dpi : { 300, 360, 600, 720 }) {
        dpiBox->addItem(QString("%1 dpi").arg(dpi), dpi);
    }
    form->addRow(QString("TIFF resolution:"), dpiBox);

    inkBox = new QSpinBox();
    inkBox->setRange(100, 400);
    inkBox->setSingleStep(10);
    inkBox->setSuffix(QString(" %"));
    inkBox->setValue(300);
    inkBox->setToolTip(QString("The most total ink (C + M + Y + K) any patch may use. Set this for your printer and media."));
    form->addRow(QString("Total ink limit:"), inkBox);
    layout->addLayout(form);

    logBox = new QPlainTextEdit();
    logBox->setReadOnly(true);
    logBox->setMaximumBlockCount(400);
    layout->addWidget(logBox, 1);

    resultLabel = new QLabel();
    resultLabel->setWordWrap(true);
    layout->addWidget(resultLabel);

    QHBoxLayout *buttons = new QHBoxLayout();
    folderButton = new QPushButton(QString("Open Folder"));
    folderButton->setEnabled(false);
    generateButton = new QPushButton(QString("Generate"));
    generateButton->setDefault(true);
    QPushButton *closeButton = new QPushButton(QString("Close"));
    connect(folderButton, &QPushButton::clicked, this, &LAUPrintTargetDialog::onOpenFolder);
    connect(generateButton, &QPushButton::clicked, this, &LAUPrintTargetDialog::onGenerate);
    connect(closeButton, &QPushButton::clicked, this, &QDialog::reject);
    buttons->addWidget(folderButton);
    buttons->addStretch();
    buttons->addWidget(generateButton);
    buttons->addWidget(closeButton);
    layout->addLayout(buttons);

    runner = new LAUArgyllRunner(this);           // going away with the dialog also stops any tool still running
    connect(runner, &LAUArgyllRunner::output, this, &LAUPrintTargetDialog::onOutput);
    connect(runner, &LAUArgyllRunner::finished, this, &LAUPrintTargetDialog::onToolFinished);
    onSizeChanged();
}

void LAUPrintTargetDialog::setRequest(double widthMM, double heightMM, int patchCount, int dpi, int inkLimit)
{
    widthBox->setValue(widthMM);
    heightBox->setValue(heightMM);
    patchBox->setValue(patchCount);
    dpiBox->setCurrentIndex(qMax(0, dpiBox->findData(dpi)));
    inkBox->setValue(inkLimit);
}

void LAUPrintTargetDialog::onSizeChanged()
{
    inchLabel->setText(QString("%1 x %2 in").arg(widthBox->value() / 25.4, 0, 'f', 2).arg(heightBox->value() / 25.4, 0, 'f', 2));
}

void LAUPrintTargetDialog::setBusy(bool state)
{
    generateButton->setEnabled(!state);
    widthBox->setEnabled(!state);
    heightBox->setEnabled(!state);
    patchBox->setEnabled(!state);
    modeBox->setEnabled(!state);
    dpiBox->setEnabled(!state);
    inkBox->setEnabled(!state);
}

QStringList LAUPrintTargetDialog::printtargArguments(const QString &name) const
{
    // -M keeps the margin in the TIFF, -C leaves it uncompressed so any RIP can read it
    // printtarg runs its strips along the page's height, so a wide target is laid out on its side and turned back after
    const double shortSide = qMin(widthBox->value(), heightBox->value()), longSide = qMax(widthBox->value(), heightBox->value());
    QStringList arguments{ QString("-v"), QString("-i"), QString("i1"),
                           QString("-p"), QString("%1x%2").arg(shortSide, 0, 'f', 0).arg(longSide, 0, 'f', 0),
                           QString("-M"), QString::number(PAGE_MARGIN_MM, 'f', 0),
                           QString("-t"), QString::number(dpiBox->currentData().toInt()),
                           QString("-C"),
                           QString("-P"),             // no limit on strip length: the gantry isn't a hand-held ruler
                           QString("-s") };           // a .cht: where every patch is on the page, in mm
    if (modeBox->currentData().toBool()) {
        arguments << QString("-n");               // stop and go: no spacers between the patches
    }
    arguments << name;
    return arguments;
}

void LAUPrintTargetDialog::onGenerate()
{
    folder = QDir(base).filePath(QDateTime::currentDateTime().toString(QString("yyyyMMdd_HHmmss")));
    if (!QDir().mkpath(folder)) {
        finish(false, QString("Couldn't create %1").arg(QDir::toNativeSeparators(folder)));
        return;
    }
    tiff.clear();
    attempts = 0;
    logBox->clear();
    resultLabel->clear();
    folderButton->setEnabled(false);
    setBusy(true);

    step = ProbeTargen;
    toolOutput.clear();
    resultLabel->setText(QString("Working out how many patches fit..."));
    if (!runner->targen(QDir(folder).filePath(QString("probe")), LAUArgyllRunner::ColorantsCMYK, PROBE_PATCHES, inkBox->value())) {
        finish(false, runner->lastError());
    }
}

bool LAUPrintTargetDialog::runMakeTargen()
{
    step = MakeTargen;
    toolOutput.clear();
    resultLabel->setText(QString("Choosing %1 patch colors. A large target can take a few minutes...").arg(patches));
    if (!runner->targen(QDir(folder).filePath(QString("target")), LAUArgyllRunner::ColorantsCMYK, patches, inkBox->value())) {
        finish(false, runner->lastError());
        return false;
    }
    return true;
}

void LAUPrintTargetDialog::onOutput(QString text)
{
    toolOutput += text + QChar('\n');
    // printtarg's progress is a long run of percentages: keep the log to what says something
    const QStringList lines = text.split(QChar('\n'));
    for (const QString &line : lines) {
        if (!line.trimmed().isEmpty() && !QRegularExpression(QString("^\\s*\\d+%\\s*$")).match(line).hasMatch()) {
            logBox->appendPlainText(line.trimmed());
        }
    }
}

void LAUPrintTargetDialog::onToolFinished(QString tool, bool success)
{
    if (!success) {
        finish(false, QString("%1 failed: %2").arg(tool, runner->lastError()));
        return;
    }

    if (step == ProbeTargen) {
        step = ProbePrinttarg;
        toolOutput.clear();
        if (!runner->start(QString("printtarg"), printtargArguments(QString("probe")), folder)) {
            finish(false, runner->lastError());
        }
        return;
    }

    if (step == ProbePrinttarg) {
        perPage = firstNumber(toolOutput, QString("patches per page = (\\d+)"));
        perRow = firstNumber(toolOutput, QString("patches per row = (\\d+)"));
        for (const QString &suffix : { QString(".ti1"), QString(".ti2"), QString(".tif"), QString(".cht") }) {
            QFile::remove(QDir(folder).filePath(QString("probe") + suffix));
        }
        if (perPage < FEWEST_PATCHES || perRow <= 0) {
            finish(false, QString("A %1 x %2 mm page holds too few i1 patches to make a useful target. Make it larger.")
                              .arg(widthBox->value(), 0, 'f', 0).arg(heightBox->value(), 0, 'f', 0));
            return;
        }
        patches = (patchBox->value() > 0) ? qMin(patchBox->value(), perPage) : perPage;
        runMakeTargen();
        return;
    }

    if (step == MakeTargen) {
        step = MakePrinttarg;
        toolOutput.clear();
        resultLabel->setText(QString("Laying out the patches and writing the TIFF..."));
        if (!runner->start(QString("printtarg"), printtargArguments(QString("target")), folder)) {
            finish(false, runner->lastError());
        }
        return;
    }

    // MakePrinttarg: printtarg adds a few patches of its own, so a full page can spill onto a second
    const int pages = firstNumber(toolOutput, QString("Total pages needed = (\\d+)"));
    if (pages > 1) {
        for (const QString &name : QDir(folder).entryList({ QString("target*") }, QDir::Files)) {
            QFile::remove(QDir(folder).filePath(name));
        }
        if (++attempts >= MAX_ATTEMPTS || patches - perRow < FEWEST_PATCHES) {
            finish(false, QString("Couldn't fit the patches on one page."));
            return;
        }
        patches -= perRow;
        logBox->appendPlainText(QString("That needs %1 pages; retrying with %2 patches.").arg(pages).arg(patches));
        runMakeTargen();
        return;
    }

    tiff = QDir(folder).filePath(QString("target.tif"));
    if (pages != 1 || !QFile::exists(tiff)) {
        finish(false, QString("printtarg finished but didn't write a single-page TIFF."));
        return;
    }
    QString embedError;
    const bool turned = widthBox->value() > heightBox->value();
    if (turned && !turnTiffQuarter(tiff, &embedError)) {
        finish(false, QString("Couldn't turn the TIFF: %1").arg(embedError));
        return;
    }

    const QString working = QDir(folder).filePath(QString("target"));
    if (!embedChartData(tiff, working + QString(".ti1"), working + QString(".ti2"), working + QString(".cht"), turned,
                        widthBox->value(), heightBox->value(), &embedError)) {
        finish(false, QString("Couldn't put the chart data in the TIFF: %1").arg(embedError));
        return;
    }

    const QString summary = QString("%1 patches on a %2 x %3 mm CMYK target at %4 dpi.")
                                .arg(patches).arg(widthBox->value(), 0, 'f', 0).arg(heightBox->value(), 0, 'f', 0)
                                .arg(dpiBox->currentData().toInt());
    finish(true, summary + QString("\nPrint %1").arg(QDir::toNativeSeparators(saveTarget())));
}

// Rewrites an uncompressed 8-bit CMYK TIFF turned a quarter turn counter-clockwise: its top edge becomes its left edge,
// so the strips start at the low end of X.  Written to a new file first, so a failure leaves the original alone.
bool LAUPrintTargetDialog::turnTiffQuarter(const QString &tiffPath, QString *error)
{
    const auto fail = [error](const QString &text) {
        if (error) {
            *error = text;
        }
        return false;
    };
    QFile in(tiffPath);
    if (!in.open(QIODevice::ReadOnly)) {
        return fail(in.errorString());
    }
    const QByteArray data = in.readAll();
    in.close();
    if (data.size() < 8 || (!data.startsWith("II") && !data.startsWith("MM"))) {
        return fail(QString("it isn't a TIFF"));
    }

    const bool little = data.startsWith("II");
    const uchar *bytes = reinterpret_cast<const uchar *>(data.constData());
    const auto u16 = [&](qint64 at) -> quint32 {
        return little ? (bytes[at] | (bytes[at + 1] << 8)) : ((bytes[at] << 8) | bytes[at + 1]);
    };
    const auto u32 = [&](qint64 at) -> quint32 {
        return little ? (bytes[at] | (bytes[at + 1] << 8) | (bytes[at + 2] << 16) | ((quint32)bytes[at + 3] << 24))
                      : (((quint32)bytes[at] << 24) | (bytes[at + 1] << 16) | (bytes[at + 2] << 8) | bytes[at + 3]);
    };

    struct Tag { quint32 type = 0, count = 0; qint64 entry = 0; };
    QMap<int, Tag> tags;
    const qint64 directory = u32(4);
    const int entries = (directory + 2 <= data.size()) ? (int)u16(directory) : 0;
    if (entries == 0 || directory + 2 + entries * TIFF_ENTRY_BYTES > data.size()) {
        return fail(QString("its tag directory is damaged"));
    }
    for (int n = 0; n < entries; n++) {
        const qint64 entry = directory + 2 + n * TIFF_ENTRY_BYTES;
        tags[u16(entry)] = { u16(entry + 2), u32(entry + 4), entry };
    }
    const auto valueAt = [&](const Tag &tag, int index) -> quint32 {
        const int size = (tag.type == 3) ? 2 : 4;
        const qint64 at = ((qint64)tag.count * size <= 4) ? tag.entry + 8 + index * size : (qint64)u32(tag.entry + 8) + index * size;
        return (tag.type == 3) ? u16(at) : u32(at);
    };
    for (int required : { 256, 257, 258, 259, 262, 273, 277, 279, 282, 283, 296 }) {
        if (!tags.contains(required)) {
            return fail(QString("it has no tag %1").arg(required));
        }
    }
    const int W = (int)valueAt(tags[256], 0), H = (int)valueAt(tags[257], 0);
    if (valueAt(tags[259], 0) != 1 || valueAt(tags[262], 0) != 5 || valueAt(tags[277], 0) != 4 || valueAt(tags[258], 0) != 8) {
        return fail(QString("it isn't uncompressed 8-bit CMYK"));
    }

    // the picture in one piece, whatever strips it came in
    const qint64 expected = (qint64)W * H * 4;
    QByteArray source;
    source.reserve(expected);
    for (quint32 n = 0; n < tags[273].count && n < tags[279].count; n++) {
        const qint64 offset = valueAt(tags[273], (int)n), length = valueAt(tags[279], (int)n);
        if (offset + length > data.size()) {
            return fail(QString("its picture data is cut short"));
        }
        source.append(data.constData() + offset, length);
    }
    if (source.size() != expected) {
        return fail(QString("its picture data is the wrong size"));
    }

    const int newW = H, newH = W;
    QByteArray pixels(expected, Qt::Uninitialized);
    for (int y = 0; y < newH; y++) {
        char *out = pixels.data() + (qint64)y * newW * 4;
        for (int x = 0; x < newW; x++) {
            memcpy(out + x * 4, source.constData() + ((qint64)x * W + (W - 1 - y)) * 4, 4);
        }
    }

    const auto rational = [&](int tag, int part) {
        return u32((qint64)u32(tags[tag].entry + 8) + part * 4);
    };
    const quint32 resolutionUnit = valueAt(tags[296], 0);
    const quint32 xNum = rational(282, 0), xDen = rational(282, 1), yNum = rational(283, 0), yDen = rational(283, 1);

    enum { Count = 14, BitsAt = 8 + 2 + Count * 12 + 4, XAt = BitsAt + 8, YAt = XAt + 8, PixelsAt = YAt + 8 };
    QByteArray output;
    output.reserve(PixelsAt + expected);
    QDataStream s(&output, QIODevice::WriteOnly);
    s.setByteOrder(QDataStream::LittleEndian);
    s.writeRawData("II", 2);
    s << (quint16)42 << (quint32)8 << (quint16)Count;
    const auto entry = [&s](int tag, int type, quint32 count, quint32 value) {
        s << (quint16)tag << (quint16)type << count << value;
    };
    entry(256, 4, 1, newW);
    entry(257, 4, 1, newH);
    entry(258, 3, 4, BitsAt);
    entry(259, 3, 1, 1);
    entry(262, 3, 1, 5);
    entry(273, 4, 1, PixelsAt);
    entry(277, 3, 1, 4);
    entry(278, 4, 1, newH);
    entry(279, 4, 1, (quint32)expected);
    entry(282, 5, 1, XAt);                        // the two resolutions swap with the axes
    entry(283, 5, 1, YAt);
    entry(284, 3, 1, 1);
    entry(296, 3, 1, resolutionUnit);
    entry(332, 3, 1, 1);
    s << (quint32)0 << (quint16)8 << (quint16)8 << (quint16)8 << (quint16)8;
    s << yNum << yDen << xNum << xDen;
    s.writeRawData(pixels.constData(), pixels.size());

    const QString temporary = tiffPath + QString(".turning");
    QFile out(temporary);
    if (!out.open(QIODevice::WriteOnly | QIODevice::Truncate) || out.write(output) != output.size()) {
        return fail(out.errorString());
    }
    out.close();
    QFile::remove(tiffPath);
    return QFile::rename(temporary, tiffPath) ? true : fail(QString("the turned file couldn't replace the original"));
}

// Adds an XMP packet (TIFF tag 700) holding the two files, by appending the packet and a new copy of the image's tag
// directory and pointing the header at it.  Nothing already in the file moves, so the pixels are untouched.
bool LAUPrintTargetDialog::embedChartData(const QString &tiffPath, const QString &ti1Path, const QString &ti2Path, const QString &chtPath,
                                          bool turned, double pageWidthMM, double pageHeightMM, QString *error)
{
    const auto fail = [error](const QString &text) {
        if (error) {
            *error = text;
        }
        return false;
    };
    QFile ti1(ti1Path), ti2(ti2Path), cht(chtPath), file(tiffPath);
    if (!ti1.open(QIODevice::ReadOnly) || !ti2.open(QIODevice::ReadOnly) || !cht.open(QIODevice::ReadOnly)) {
        return fail(QString("the .ti1, .ti2 or .cht couldn't be read"));
    }
    const QString escapedTi1 = QString::fromLatin1(ti1.readAll()).toHtmlEscaped();
    const QString escapedTi2 = QString::fromLatin1(ti2.readAll()).toHtmlEscaped();
    const QString escapedCht = QString::fromLatin1(cht.readAll()).toHtmlEscaped();
    const QString page = QString("%1 %2").arg(pageWidthMM, 0, 'f', 3).arg(pageHeightMM, 0, 'f', 3);
    if (!file.open(QIODevice::ReadWrite)) {
        return fail(file.errorString());
    }

    QByteArray header = file.read(8);
    if (header.size() != 8 || (!header.startsWith("II") && !header.startsWith("MM"))) {
        return fail(QString("it isn't a TIFF"));
    }
    const QDataStream::ByteOrder order = header.startsWith("II") ? QDataStream::LittleEndian : QDataStream::BigEndian;
    QDataStream in(header);
    in.setByteOrder(order);
    in.skipRawData(2);
    quint16 magic;
    quint32 firstDirectory;
    in >> magic >> firstDirectory;
    if (magic != 42) {
        return fail(QString("it isn't a classic TIFF"));
    }

    if (!file.seek(firstDirectory)) {
        return fail(file.errorString());
    }
    QByteArray countBytes = file.read(2);
    QDataStream countStream(countBytes);
    countStream.setByteOrder(order);
    quint16 count = 0;
    countStream >> count;
    const QByteArray entries = file.read(count * TIFF_ENTRY_BYTES);
    const QByteArray nextBytes = file.read(4);
    if (entries.size() != count * TIFF_ENTRY_BYTES || nextBytes.size() != 4) {
        return fail(QString("its tag directory is cut short"));
    }

    const QByteArray packet = QString("<?xpacket begin=\"\xEF\xBB\xBF\" id=\"W5M0MpCehiHzreSzNTczkc9d\"?>\n"
                                      "<x:xmpmeta xmlns:x=\"adobe:ns:meta/\"><rdf:RDF xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\">"
                                      "<rdf:Description rdf:about=\"\" xmlns:lau=\"%1\">\n<lau:ti1>%2</lau:ti1>\n<lau:ti2>%3</lau:ti2>\n"
                                      "<lau:cht>%4</lau:cht>\n<lau:turned>%5</lau:turned>\n<lau:page_mm>%6</lau:page_mm>\n"
                                      "</rdf:Description></rdf:RDF></x:xmpmeta>\n<?xpacket end=\"r\"?>")
                                  .arg(CHART_NAMESPACE, escapedTi1, escapedTi2, escapedCht, turned ? QString("1") : QString("0"), page).toUtf8();

    // TIFF offsets must be even, and tags must stay in ascending order
    qint64 packetAt = file.size();
    packetAt += packetAt % 2;
    qint64 directoryAt = packetAt + packet.size();
    directoryAt += directoryAt % 2;
    if (directoryAt + 6 + (count + 1) * TIFF_ENTRY_BYTES > Q_INT64_C(0xFFFFFFFF)) {
        return fail(QString("it would be too large for a TIFF"));
    }

    QByteArray entry;
    QDataStream entryStream(&entry, QIODevice::WriteOnly);
    entryStream.setByteOrder(order);
    entryStream << (quint16)TIFF_TAG_XMP << (quint16)TIFF_TYPE_BYTE << (quint32)packet.size() << (quint32)packetAt;

    QByteArray directory;
    QDataStream directoryStream(&directory, QIODevice::WriteOnly);
    directoryStream.setByteOrder(order);
    int kept = 0;
    bool placed = false;
    for (int n = 0; n < count; n++) {
        const QByteArray old = entries.mid(n * TIFF_ENTRY_BYTES, TIFF_ENTRY_BYTES);
        QDataStream tagStream(old);
        tagStream.setByteOrder(order);
        quint16 tag = 0;
        tagStream >> tag;
        if (tag == TIFF_TAG_XMP) {
            continue;                                 // replaced
        }
        if (!placed && tag > TIFF_TAG_XMP) {
            directoryStream.writeRawData(entry.constData(), entry.size());
            placed = true;
        }
        directoryStream.writeRawData(old.constData(), old.size());
        kept++;
    }
    if (!placed) {
        directoryStream.writeRawData(entry.constData(), entry.size());
    }

    QByteArray countOut;
    QDataStream countOutStream(&countOut, QIODevice::WriteOnly);
    countOutStream.setByteOrder(order);
    countOutStream << (quint16)(kept + 1);

    QByteArray offsetOut;
    QDataStream offsetOutStream(&offsetOut, QIODevice::WriteOnly);
    offsetOutStream.setByteOrder(order);
    offsetOutStream << (quint32)directoryAt;

    // the header is changed last: until then the file is still a complete TIFF
    const QByteArray pad(1, '\0');
    bool ok = file.seek(file.size());
    if (ok && file.size() != packetAt) {
        ok = file.write(pad) == 1;
    }
    ok = ok && file.write(packet) == packet.size();
    if (ok && file.pos() != directoryAt) {
        ok = file.write(pad) == 1;
    }
    ok = ok && file.write(countOut) == 2 && file.write(directory) == directory.size() && file.write(nextBytes) == 4;
    ok = ok && file.flush() && file.seek(4) && file.write(offsetOut) == 4;
    return ok ? true : fail(file.errorString());
}

// Offers the finished target to the user: one file, since the chart data is inside it.  Cancelling leaves it in the
// working folder.  Returns where the TIFF is.
QString LAUPrintTargetDialog::saveTarget()
{
    const QString suggested = QDir(LAULastDirectory::get()).filePath(QString("print_target_%1x%2mm_%3.tif")
                                                           .arg(widthBox->value(), 0, 'f', 0).arg(heightBox->value(), 0, 'f', 0)
                                                           .arg(modeBox->currentData().toBool() ? QString("stopgo") : QString("sliding")));
    const QString chosen = QFileDialog::getSaveFileName(this, QString("Save Print Target"), suggested, QString("CMYK TIFF (*.tif *.tiff)"));
    if (chosen.isEmpty()) {
        return tiff;
    }

    LAULastDirectory::rememberFile(chosen);
    QFile::remove(chosen);
    if (!QFile::copy(tiff, chosen)) {
        QMessageBox::warning(this, QString("Save Print Target"), QString("Couldn't save to %1. The target is still in %2")
                                                                     .arg(QDir::toNativeSeparators(chosen), QDir::toNativeSeparators(folder)));
        return tiff;
    }
    tiff = chosen;
    folder = QFileInfo(chosen).absolutePath();
    return tiff;
}

void LAUPrintTargetDialog::finish(bool ok, const QString &message)
{
    step = Idle;
    setBusy(false);
    folderButton->setEnabled(!folder.isEmpty());
    resultLabel->setText(message);
    resultLabel->setStyleSheet(ok ? QString() : QString("color: #c62828;"));
    emit emitDone(ok);
}

void LAUPrintTargetDialog::onOpenFolder()
{
    QDesktopServices::openUrl(QUrl::fromLocalFile(folder));
}
