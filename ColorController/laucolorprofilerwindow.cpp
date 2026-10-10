#include "laucolorprofilerwindow.h"
#include "laueyeonedialog.h"
#include "lauaboutdialog.h"

#include <QDir>
#include <QFile>
#include <QMenuBar>
#include <QSettings>
#include <QGroupBox>
#include <QFileInfo>
#include <QScrollArea>
#include <QFormLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QStandardPaths>

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
LAUColorProfilerWindow::LAUColorProfilerWindow(QWidget *parent) : QMainWindow(parent), gantryBox(nullptr), gantry(nullptr), runner(nullptr), lastCapacity(-1)
{
#ifdef FUYU_SIMULATE
    this->setWindowTitle(QString("LAU Color Profiler  -  gantry SIMULATION (no hardware)"));
#else
    this->setWindowTitle(QString("LAU Color Profiler"));
#endif

    // SCRATCH FILES FOR THE ARGYLL TOOLS
    QDir work(QDir::temp().filePath(QString("LAUColorController")));
    work.mkpath(QString("."));
    workBasename = work.filePath(QString("chart"));

    runner = new LAUArgyllRunner(this);
    connect(runner, &LAUArgyllRunner::output, this, &LAUColorProfilerWindow::onArgyllOutput);
    connect(runner, &LAUArgyllRunner::finished, this, &LAUColorProfilerWindow::onArgyllFinished);

    // MENUS
    QMenu *menu = menuBar()->addMenu(QString("&Tools"));
    menu->addAction(QString("i1Pro &Spot Tool..."), this, &LAUColorProfilerWindow::onShowSpotTool);
    menu->addAction(QString("Set &Argyll Directory..."), this, &LAUColorProfilerWindow::onSetArgyllDirectory);
    menu = menuBar()->addMenu(QString("&Help"));
    menu->addAction(QString("&About..."), this, &LAUColorProfilerWindow::onShowAboutDialog);

    QWidget *central = new QWidget();
    central->setLayout(new QHBoxLayout());
    central->layout()->setContentsMargins(6, 6, 6, 6);
    setCentralWidget(central);

    // ---- LEFT: ARGYLL TOOLS --------------------------------------------------
    QGroupBox *argyllBox = new QGroupBox(QString("Argyll Tools"));
    argyllBox->setFixedWidth(320);
    QVBoxLayout *argyllLayout = new QVBoxLayout(argyllBox);
    argyllLayout->setContentsMargins(6, 6, 6, 6);
    argyllLayout->setSpacing(6);

    QFormLayout *form = new QFormLayout();
    areaLabel = new QLabel();
    areaLabel->setWordWrap(true);
    form->addRow(QString("Scan area:"), areaLabel);
    colorSpaceComboBox = new QComboBox();
    colorSpaceComboBox->addItem(QString("CMYK"), static_cast<int>(LAUArgyllRunner::ColorantsCMYK));
    colorSpaceComboBox->addItem(QString("RGB"), static_cast<int>(LAUArgyllRunner::ColorantsPrintRGB));
    colorSpaceComboBox->setToolTip(QString("CMYK: printer driven with ink values (RIP / CMYK TIFF).\n"
                                           "RGB: printer driven through an RGB driver (targen \"Print RGB\")."));
    colorSpaceComboBox->setCurrentText(QSettings().value(QString("LAUColorProfilerWindow::colorSpace"), QString("CMYK")).toString());
    form->addRow(QString("Color space:"), colorSpaceComboBox);
    patchSpinBox = new QSpinBox();
    patchSpinBox->setRange(0, 0);
    form->addRow(QString("Patches:"), patchSpinBox);
    inkLimitSpinBox = new QSpinBox();
    inkLimitSpinBox->setRange(100, 400);
    inkLimitSpinBox->setSuffix(QString(" %"));
    inkLimitSpinBox->setValue(QSettings().value(QString("LAUColorProfilerWindow::inkLimit"), 300).toInt());
    form->addRow(QString("Total ink limit:"), inkLimitSpinBox);
    argyllLayout->addLayout(form);
    connect(colorSpaceComboBox, &QComboBox::currentIndexChanged, this, &LAUColorProfilerWindow::onColorSpaceChanged);

    generateButton = new QPushButton(QString("Generate Argyll Test Chart"));
    connect(generateButton, &QPushButton::clicked, this, &LAUColorProfilerWindow::onGenerateChart);
    argyllLayout->addWidget(generateButton);

    saveButton = new QPushButton(QString("Save Chart..."));
    connect(saveButton, &QPushButton::clicked, this, &LAUColorProfilerWindow::onSaveChart);
    argyllLayout->addWidget(saveButton);

    loadButton = new QPushButton(QString("Load Measurements (.ti3 / .csv)..."));
    connect(loadButton, &QPushButton::clicked, this, &LAUColorProfilerWindow::onLoadMeasurements);
    argyllLayout->addWidget(loadButton);

    processButton = new QPushButton(QString("Process Argyll Test Chart"));
    connect(processButton, &QPushButton::clicked, this, &LAUColorProfilerWindow::onProcessChart);
    argyllLayout->addWidget(processButton);

    logTextEdit = new QTextEdit();
    logTextEdit->setReadOnly(true);
    argyllLayout->addWidget(logTextEdit);
    central->layout()->addWidget(argyllBox);

    // ---- MIDDLE: CHART PREVIEW ---------------------------------------------
    QGroupBox *workspaceBox = new QGroupBox(QString("Workspace"));
    QVBoxLayout *workspaceLayout = new QVBoxLayout(workspaceBox);
    workspaceLayout->setContentsMargins(6, 6, 6, 6);
    chartLabel = new QLabel(QString("No chart yet. Set the gantry limits, then Generate Argyll Test Chart."));
    chartLabel->setWordWrap(true);
    workspaceLayout->addWidget(chartLabel);
    previewLabel = new QLabel();
    previewLabel->setAlignment(Qt::AlignCenter);
    previewLabel->setMinimumSize(480, 360);
    previewLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);   // LET IT SHRINK; THE PIXMAP IS RESCALED TO FIT
    workspaceLayout->addWidget(previewLabel, 1);

    // ---- RIGHT, BELOW THE PREVIEW: FUYU GANTRY -----------------------------
    // THE CONTROLS ARE BUILT NOW (SHOWING THE SAVED LIMITS) BUT THE BOX STARTS UNCHECKED AND THE
    // CONTROLLER UNCONNECTED; CHECKING THE BOX ASKS FOR THE ETHERNET SETTINGS AND THEN CONNECTS.
    //   FMC4030 SDK AXIS 0 = X GANTRY (BOTH RAILS SHARE ONE CONTROLLER AXIS)
    //   FMC4030 SDK AXIS 2 = Y CROSS-BEAM
#ifdef FUYU_SIMULATE
    gantryBox = new QGroupBox(QString("FUYU Gantry  -  SIMULATION"));
#else
    gantryBox = new QGroupBox(QString("FUYU Gantry"));
#endif
    gantryBox->setCheckable(true);
    QVBoxLayout *gantryLayout = new QVBoxLayout(gantryBox);
    gantryLayout->setContentsMargins(6, 6, 6, 6);
    gantry = new LAUMultiVelmexWidget(QList<int>() << 0 << 2, nullptr, false);
    gantryLayout->addWidget(gantry);
    gantryBox->setChecked(false);
    connect(gantryBox, &QGroupBox::toggled, this, &LAUColorProfilerWindow::onGantryToggled);

    QWidget *rightColumn = new QWidget();
    QVBoxLayout *rightLayout = new QVBoxLayout(rightColumn);
    rightLayout->setContentsMargins(0, 0, 0, 0);
    rightLayout->addWidget(workspaceBox, 1);
    rightLayout->addWidget(gantryBox, 0);
    central->layout()->addWidget(rightColumn);

    // THE GANTRY WIDGET HAS NO LIMIT-CHANGED SIGNAL, SO POLL THE SOFT LIMITS
    areaTimer = new QTimer(this);
    connect(areaTimer, &QTimer::timeout, this, &LAUColorProfilerWindow::onRefreshScanArea);
    areaTimer->start(1000);
    onRefreshScanArea();
    updateButtons();

    QSettings settings;
    restoreGeometry(settings.value(QString("LAUColorProfilerWindow::geometry")).toByteArray());

    // SHOW A DEFAULT TEST CHART AS SOON AS THE WINDOW IS UP
    QTimer::singleShot(0, this, &LAUColorProfilerWindow::loadDefaultChart);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
LAUColorProfilerWindow::~LAUColorProfilerWindow()
{
    QSettings settings;
    settings.setValue(QString("LAUColorProfilerWindow::geometry"), saveGeometry());
    settings.setValue(QString("LAUColorProfilerWindow::inkLimit"), inkLimitSpinBox->value());
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUColorProfilerWindow::onGantryToggled(bool on)
{
    if (!on || gantry->isControllerStarted()) {
        // UNCHECKING JUST DISABLES THE CONTROLS; THE CONNECTION IS KEPT FOR WHEN IT IS RE-CHECKED
        return;
    }

    // FIRST TIME ON: CONFIRM THE IP ADDRESS / PORT (THE DIALOG TESTS THE CONNECTION BEFORE ACCEPTING
    // AND SAVES IT TO QSETTINGS, WHERE THE CONTROLLER READS IT WHEN IT STARTS)
    LAUFuyuConnectDialog dialog(this);
    if (dialog.exec() != QDialog::Accepted) {
        gantryBox->blockSignals(true);
        gantryBox->setChecked(false);
        gantryBox->blockSignals(false);
        return;
    }
    gantry->connectToController();
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
QSizeF LAUColorProfilerWindow::gantryLimitsMM() const
{
    // THE LIMIT SPIN BOXES HOLD THE SAVED LIMITS EVEN BEFORE THE CONTROLLER CONNECTS.
    // THEY SHOW mm OR INCHES PER LAUVelmexWidget::unitIndex (0 = INCHES).
    double scale = (QSettings().value(QString("LAUVelmexWidget::unitIndex"), 1).toInt() == 0) ? 25.4 : 1.0;
    bool okX = false, okY = false;
    double x = qAbs(gantry->right(0, &okX) - gantry->left(0)) * scale;
    double y = qAbs(gantry->right(2, &okY) - gantry->left(2)) * scale;
    if (!okX || !okY) {
        return (QSizeF());
    }
    return (QSizeF(x, y));
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
QSizeF LAUColorProfilerWindow::scanAreaMM() const
{
    // THE GANTRY CAN ALWAYS COVER AT LEAST A US LETTER SHEET, LONG SIDE ALONG THE SCAN (X)
    QSizeF limits = gantryLimitsMM();
    return (QSizeF(qMax(limits.width(), MINIMUM_SCAN_AREA_MM.width()), qMax(limits.height(), MINIMUM_SCAN_AREA_MM.height())));
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUColorProfilerWindow::loadDefaultChart()
{
    // REUSE LAST SESSION'S CHART IF IT STILL HAS THE RIGHT COLOR SPACE AND PATCH COUNT; OTHERWISE ASK TARGEN FOR ONE
    QString filename = workBasename + QString(".ti1");
    if (QFile::exists(filename)) {
        LAUColorChart previous = LAUColorChart::readArgyll(filename);
        if (previous.deviceSpace() == colorSpaceComboBox->currentText() && previous.patches.count() == patchSpinBox->value()) {
            setChart(previous);
            log(QString("Loaded the default test chart (%1 patches).").arg(chart.patches.count()));
            return;
        }
    }
    log(QString("Creating the default test chart..."));
    onGenerateChart();
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUColorProfilerWindow::onRefreshScanArea()
{
    QSizeF area = scanAreaMM();
    QSizeF limits = gantryLimitsMM();
    int perLine = chartLayout.patchesPerLine(area.width());
    int lines = chartLayout.lines(area.height());
    int capacity = perLine * lines;

    QString source;
    if (limits.width() < MINIMUM_SCAN_AREA_MM.width() || limits.height() < MINIMUM_SCAN_AREA_MM.height()) {
        source = QString(" (letter-size minimum)");
    } else if (!gantry->isControllerStarted()) {
        source = QString(" (saved limits)");
    }
    areaLabel->setText(QString("%1 x %2 mm%6\n%3 patches/line x %4 lines = %5")
                       .arg(area.width(), 0, 'f', 1).arg(area.height(), 0, 'f', 1)
                       .arg(perLine).arg(lines).arg(capacity).arg(source));

    // KEEP THE PATCH COUNT AT CAPACITY UNLESS THE USER CHOSE FEWER
    if (capacity != lastCapacity) {
        bool atMax = (patchSpinBox->value() == patchSpinBox->maximum());
        patchSpinBox->setRange(capacity > 0 ? qMin(capacity, 50) : 0, capacity);
        if (atMax) {
            patchSpinBox->setValue(capacity);
        }
        lastCapacity = capacity;
        updateButtons();
    }
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUColorProfilerWindow::onColorSpaceChanged()
{
    QSettings().setValue(QString("LAUColorProfilerWindow::colorSpace"), colorSpaceComboBox->currentText());
    updateButtons();

    // SWAP THE PREVIEW TO A CHART IN THE NEW COLOR SPACE
    loadDefaultChart();
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUColorProfilerWindow::updateButtons()
{
    bool idle = !runner->isRunning();
    bool cmyk = (colorSpaceComboBox->currentData().toInt() == LAUArgyllRunner::ColorantsCMYK);
    colorSpaceComboBox->setEnabled(idle);
    inkLimitSpinBox->setEnabled(cmyk);
    generateButton->setEnabled(idle && patchSpinBox->value() > 0);
    saveButton->setEnabled(idle && !chart.isEmpty());
    loadButton->setEnabled(idle);
    processButton->setEnabled(idle && chart.measuredCount() > 0);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUColorProfilerWindow::log(const QString &text)
{
    logTextEdit->append(text);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUColorProfilerWindow::onArgyllOutput(QString text)
{
    log(text);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUColorProfilerWindow::onGenerateChart()
{
    LAUArgyllRunner::Colorants colorants = static_cast<LAUArgyllRunner::Colorants>(colorSpaceComboBox->currentData().toInt());
    if (colorants == LAUArgyllRunner::ColorantsCMYK) {
        log(QString("Generating %1 CMYK patches (ink limit %2%)...").arg(patchSpinBox->value()).arg(inkLimitSpinBox->value()));
    } else {
        log(QString("Generating %1 RGB patches...").arg(patchSpinBox->value()));
    }
    if (!runner->targen(workBasename, colorants, patchSpinBox->value(), inkLimitSpinBox->value())) {
        log(runner->lastError());
        QMessageBox::warning(this, windowTitle(), runner->lastError());
    }
    updateButtons();
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUColorProfilerWindow::onArgyllFinished(QString tool, bool success)
{
    updateButtons();
    if (!success) {
        log(runner->lastError());
        QMessageBox::warning(this, windowTitle(), runner->lastError());
        return;
    }

    if (tool == QString("targen")) {
        QString error;
        LAUColorChart newChart = LAUColorChart::readArgyll(workBasename + QString(".ti1"), &error);
        if (newChart.isEmpty()) {
            log(error);
            return;
        }
        setChart(newChart);
        if (!chart.writeTI2(workBasename + QString(".ti2"), &error)) {
            log(error);
        }
        log(QString("Test chart ready: %1 patches.").arg(chart.patches.count()));
    } else if (tool == QString("colprof")) {
        // ARGYLL NAMES THE PROFILE .icm ON WINDOWS AND .icc ELSEWHERE
        QString profile = workBasename + QString(".icm");
        if (!QFile::exists(profile)) {
            profile = workBasename + QString(".icc");
        }
        if (!QFile::exists(profile)) {
            log(QString("colprof finished but no profile was found next to %1.").arg(workBasename));
            return;
        }
        log(QString("Profile complete."));

        QString suffix = QFileInfo(profile).suffix();
        QString directory = QSettings().value(QString("LAUColorProfilerWindow::lastDirectory"), QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
        QString filename = QFileDialog::getSaveFileName(this, QString("Save ICC profile"), directory, QString("ICC profile (*.%1)").arg(suffix));
        if (filename.isEmpty()) {
            return;
        }
        if (QFileInfo(filename).suffix().isEmpty()) {
            filename.append(QString(".") + suffix);
        }
        QSettings().setValue(QString("LAUColorProfilerWindow::lastDirectory"), QFileInfo(filename).absolutePath());
        QFile::remove(filename);
        if (QFile::copy(profile, filename)) {
            log(QString("Saved %1").arg(filename));
        } else {
            log(QString("Unable to save %1").arg(filename));
        }
    }
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUColorProfilerWindow::setChart(const LAUColorChart &newChart)
{
    chart = newChart;
    updatePreview();
    updateButtons();
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUColorProfilerWindow::updatePreview()
{
    if (chart.isEmpty()) {
        previewImage = QImage();
        previewLabel->clear();
        return;
    }

    int placed = chartLayout.layout(chart, scanAreaMM());
    QSizeF size = chartLayout.chartSizeMM();
    chartLabel->setText(QString("%1 of %2 patches placed, %3 separation bars, chart %4 x %5 mm (%6 measured).")
                        .arg(placed).arg(chart.patches.count()).arg(chartLayout.bars().count())
                        .arg(size.width(), 0, 'f', 1).arg(size.height(), 0, 'f', 1)
                        .arg(chart.measuredCount()));
    previewImage = chartLayout.preview(chart, 4.0);
    fitPreview();
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUColorProfilerWindow::fitPreview()
{
    if (!previewImage.isNull()) {
        previewLabel->setPixmap(QPixmap::fromImage(previewImage.scaled(previewLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation)));
    }
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUColorProfilerWindow::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
    fitPreview();
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUColorProfilerWindow::onSaveChart()
{
    QString directory = QSettings().value(QString("LAUColorProfilerWindow::lastDirectory"), QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
    QString filename = QFileDialog::getSaveFileName(this, QString("Save test chart"), directory, QString("Argyll chart (*.ti2)"));
    if (filename.isEmpty()) {
        return;
    }
    QString basename = QFileInfo(filename).absoluteDir().filePath(QFileInfo(filename).completeBaseName());
    QSettings().setValue(QString("LAUColorProfilerWindow::lastDirectory"), QFileInfo(filename).absolutePath());

    // .TI1 FOR ARGYLL, .TI2 WITH PATCH LOCATIONS, PNG PREVIEW (THE CMYK PRINT TIFF COMES WITH THE LAYOUT GENERATOR)
    QString error;
    QFile::remove(basename + QString(".ti1"));
    QFile::copy(workBasename + QString(".ti1"), basename + QString(".ti1"));
    if (!chart.writeTI2(basename + QString(".ti2"), &error)) {
        log(error);
        return;
    }
    chartLayout.preview(chart, 10.0).save(basename + QString("_preview.png"));
    log(QString("Saved %1.ti1, .ti2 and _preview.png").arg(basename));
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUColorProfilerWindow::onLoadMeasurements()
{
    QString directory = QSettings().value(QString("LAUColorProfilerWindow::lastDirectory"), QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
    QString filename = QFileDialog::getOpenFileName(this, QString("Load measurements"), directory, QString("Measurements (*.ti3 *.csv)"));
    if (filename.isEmpty()) {
        return;
    }
    QSettings().setValue(QString("LAUColorProfilerWindow::lastDirectory"), QFileInfo(filename).absolutePath());

    QString error;
    LAUColorChart loaded = filename.toLower().endsWith(QString(".csv")) ? LAUColorChart::readCSV(filename, &error) : LAUColorChart::readArgyll(filename, &error);
    if (loaded.isEmpty()) {
        log(error);
        QMessageBox::warning(this, windowTitle(), error);
        return;
    }
    setChart(loaded);
    log(QString("Loaded %1: %2 patches, %3 measured.").arg(filename).arg(chart.patches.count()).arg(chart.measuredCount()));
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUColorProfilerWindow::onProcessChart()
{
    QString error;
    if (!chart.writeTI3(workBasename + QString(".ti3"), &error)) {
        log(error);
        QMessageBox::warning(this, windowTitle(), error);
        return;
    }
    log(QString("Wrote %1 measured patches. Running colprof (this can take a minute)...").arg(chart.measuredCount()));
    if (!runner->colprof(workBasename, QStringList() << QString("-v") << QString("-qm"))) {
        log(runner->lastError());
        QMessageBox::warning(this, windowTitle(), runner->lastError());
    }
    updateButtons();
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUColorProfilerWindow::onShowSpotTool()
{
    LAUEyeOneDialog dialog(this);
    dialog.exec();
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUColorProfilerWindow::onSetArgyllDirectory()
{
    QString current = LAUArgyllRunner::binDirectory();
    if (current.isEmpty()) {
        current = QFileInfo(LAUArgyllRunner::toolPath(QString("targen"))).absolutePath();
    }
    QString directory = QFileDialog::getExistingDirectory(this, QString("Folder containing targen and colprof"), current);
    if (!directory.isEmpty()) {
        LAUArgyllRunner::setBinDirectory(directory);
        log(QString("Argyll tools: %1").arg(LAUArgyllRunner::toolPath(QString("targen"))));
    }
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUColorProfilerWindow::onShowAboutDialog()
{
    LAUAboutDialog dialog(this);
    dialog.exec();
}
