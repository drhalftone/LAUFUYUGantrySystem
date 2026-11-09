/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAUGantryMainWindow -- one window for the gantry: the carriage camera in a group box on top and the
// rail sliders in a group box below it.  The camera starts by itself; the sliders' group box is checkable
// and the first time it is ticked the user confirms the FMC4030's address and the rails connect.
//
#ifndef LAUGANTRYMAINWINDOW_H
#define LAUGANTRYMAINWINDOW_H

#include <QGroupBox>
#include <QMainWindow>

#include "laubreadboardcalibrationdialog.h"
#include "lauvelmexwidget.h"

// shown while the rails are moving; the only way out is Abort or the move finishing
class LAUHomeDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LAUHomeDialog(QWidget *parent = nullptr);

    void finish();

public slots:
    void reject() override;                  // Abort, Esc and the title bar's X all stop the rails

signals:
    void emitAbort();

private:
    QLabel *label = nullptr;
    QPushButton *abortButton = nullptr;
};

class LAUGantryMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit LAUGantryMainWindow(QWidget *parent = nullptr);

private slots:
    void onRailsEnabled(bool state);
    void onGoHome();
    void onHomeFinished(bool ok, QString message);

private:
    LAUBreadboardCalibrationWidget *cameraWidget = nullptr;
    LAUMultiVelmexWidget *railWidget = nullptr;
    QGroupBox *railBox = nullptr;
    LAUHomeDialog *homeDialog = nullptr;
};

#endif // LAUGANTRYMAINWINDOW_H
