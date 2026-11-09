/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAUPrintTargetDialog -- asks how large a print target to make, no larger than the area the i1's measuring circle
// can reach, then has ArgyllCMS make it: targen chooses the CMYK patch values and printtarg lays them out on the
// page.  The result is one CMYK TIFF to print, carrying the .ti1 and .ti2 needed for profiling in its XMP metadata.
//
// printtarg says how many patches fit a page but only after being run, so it is run once on a few patches to find
// out, then targen makes that many and printtarg lays them out for real.
//
#ifndef LAUPRINTTARGETDIALOG_H
#define LAUPRINTTARGETDIALOG_H

#include <QDialog>
#include <QString>

class QLabel;
class QComboBox;
class QPushButton;
class QPlainTextEdit;
class QDoubleSpinBox;
class QSpinBox;
class LAUArgyllRunner;

class LAUPrintTargetDialog : public QDialog
{
    Q_OBJECT

public:
    LAUPrintTargetDialog(double maxWidthMM, double maxHeightMM, const QString &baseFolder, QWidget *parent = nullptr);

    void setRequest(double widthMM, double heightMM, int patches, int dpi, int inkLimit);
    QString tiffPath() const
    {
        return tiff;
    }

    static bool turnTiffQuarter(const QString &tiffPath, QString *error = nullptr);

    // Puts the .ti1 and .ti2 inside the TIFF, in its XMP metadata, so the target is one file
    static bool embedChartData(const QString &tiffPath, const QString &ti1Path, const QString &ti2Path, QString *error = nullptr);

public slots:
    void onGenerate();

signals:
    void emitDone(bool ok);

private slots:
    void onOutput(QString text);
    void onToolFinished(QString tool, bool success);
    void onOpenFolder();
    void onSizeChanged();

private:
    enum Step { Idle, ProbeTargen, ProbePrinttarg, MakeTargen, MakePrinttarg };

    double maxWidth, maxHeight;
    QString base, folder, tiff;

    QDoubleSpinBox *widthBox = nullptr, *heightBox = nullptr;
    QSpinBox *patchBox = nullptr, *inkBox = nullptr;
    QComboBox *dpiBox = nullptr, *modeBox = nullptr;
    QLabel *inchLabel = nullptr, *resultLabel = nullptr;
    QPlainTextEdit *logBox = nullptr;
    QPushButton *generateButton = nullptr, *folderButton = nullptr;

    LAUArgyllRunner *runner = nullptr;
    Step step = Idle;
    QString toolOutput;
    int perRow = 0, perPage = 0, patches = 0, attempts = 0;

    QStringList printtargArguments(const QString &name) const;
    bool runMakeTargen();
    QString saveTarget();
    void setBusy(bool state);
    void finish(bool ok, const QString &message);
};

#endif // LAUPRINTTARGETDIALOG_H
