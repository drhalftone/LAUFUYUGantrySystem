#ifndef LAUCOLORPROFILERWINDOW_H
#define LAUCOLORPROFILERWINDOW_H

#include <QLabel>
#include <QTimer>
#include <QSpinBox>
#include <QGroupBox>
#include <QComboBox>
#include <QTextEdit>
#include <QMainWindow>
#include <QPushButton>

#include "lauargyllchart.h"
#include "lauargyllrunner.h"
#include "lauchartlayout.h"
#include "lauvelmexwidget.h"

/****************************************************************************/
/* LAUColorProfilerWindow                                                   */
/*                                                                          */
/* Main window of the gantry color profiler.  Left: Argyll tools (generate  */
/* the test chart, build the ICC profile).  Middle: preview of the chart as */
/* laid out for the scan area.  Right: the FUYU gantry controls, whose soft */
/* limits define that scan area (X = scan direction, Y = line step).        */
/****************************************************************************/
class LAUColorProfilerWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit LAUColorProfilerWindow(QWidget *parent = nullptr);
    ~LAUColorProfilerWindow();

    // SMALLEST AREA THE GANTRY IS ASSUMED TO COVER: US LETTER, 11" ALONG THE SCAN (X) BY 8.5" ACROSS (Y)
    static inline const QSizeF MINIMUM_SCAN_AREA_MM = QSizeF(279.4, 215.9);

    // SCAN AREA IN MILLIMETRES (WIDTH = X, HEIGHT = Y): THE GANTRY SOFT LIMITS, BUT NEVER LESS THAN LETTER
    QSizeF scanAreaMM() const;
    QSizeF gantryLimitsMM() const;

public slots:
    void loadDefaultChart();
    void onColorSpaceChanged();
    void onRefreshScanArea();
    void onGenerateChart();
    void onSaveChart();
    void onLoadMeasurements();
    void onProcessChart();
    void onShowSpotTool();
    void onSetArgyllDirectory();
    void onShowAboutDialog();

private slots:
    void onGantryToggled(bool on);
    void onArgyllOutput(QString text);
    void onArgyllFinished(QString tool, bool success);

private:
    QGroupBox *gantryBox;
    LAUMultiVelmexWidget *gantry;      // BUILT AT STARTUP, CONNECTED WHEN THE USER CHECKS gantryBox
    LAUArgyllRunner *runner;
    LAUColorChart chart;
    LAUChartLayout chartLayout;
    QString workBasename;              // <TEMP>/LAUColorController/chart -> chart.ti1, .ti2, .ti3, .icm

    QLabel *areaLabel;
    QLabel *previewLabel;
    QLabel *chartLabel;
    QSpinBox *patchSpinBox;
    QSpinBox *inkLimitSpinBox;
    QComboBox *colorSpaceComboBox;     // CMYK OR RGB (targen -d 4 / -d 2)
    QTextEdit *logTextEdit;
    QPushButton *generateButton;
    QPushButton *saveButton;
    QPushButton *loadButton;
    QPushButton *processButton;
    QTimer *areaTimer;
    int lastCapacity;

    QImage previewImage;               // FULL-RESOLUTION PREVIEW, RESCALED TO THE LABEL ON RESIZE

    void setChart(const LAUColorChart &newChart);
    void updatePreview();
    void fitPreview();

protected:
    void resizeEvent(QResizeEvent *event) override;

private:
    void updateButtons();
    void log(const QString &text);
};

#endif // LAUCOLORPROFILERWINDOW_H
