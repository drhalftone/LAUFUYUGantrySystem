#ifndef LAUEYEONEDIALOG_H
#define LAUEYEONEDIALOG_H

#include <QDebug>
#include <QLabel>
#include <QDialog>
#include <QGroupBox>
#include <QPushButton>

#include "qcustomplot.h"
#include "laui1prodevice.h"
#include "lauspectraltablewidget.h"

/******************************************************************************/
/* LAUEyeOneDialog                                                            */
/*                                                                            */
/* Bench tool for the i1Pro: connect, calibrate, and take spot readings with  */
/* the device button, plotting each spectrum.  All SDK access goes through    */
/* LAUi1ProDevice; this class is UI only.                                     */
/******************************************************************************/
class LAUEyeOneDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LAUEyeOneDialog(QWidget *parent = nullptr);
    ~LAUEyeOneDialog();

    QString error() const
    {
        return (errorString);
    }

    bool isValid() const
    {
        return (device->isOpen());
    }

    LAUColorPatch lastMeasurement() const
    {
        return (patch);
    }

protected:
    void accept();

public slots:
    void onUpdatePlot();
    void onButtonClicked();
    void onActionAboutBox();
    void onDeviceConnected();
    void onDeviceDisconnected();
    void onCalibrateButtonClicked();

    void onContextMenuTriggered()
    {
        if (tableWidget) {
            tableWidget->hide();
            tableWidget->show();
        }
    }

    void onMousePressEvent(QMouseEvent *event)
    {
        if (event->button() == Qt::RightButton) {
            if (contextMenu) {
                contextMenu->popup(event->globalPosition().toPoint());
            }
        }
    }

private:
    LAUi1ProDevice *device;
    LAUColorPatch patch;
    QString errorString;
    QCustomPlot *customPlot;
    QLabel *statusLabel;
    QPushButton *calibrateButton;
    LAUSpectralTableWidget *tableWidget;
    QMenu *contextMenu;

    void updateStatus();
};

#endif // LAUEYEONEDIALOG_H
