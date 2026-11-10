/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAUi1AttachDialog -- live camera while the i1 Pro goes into its cradle on the carriage.  The cradle is fixed to the
// camera, so it is always the same pixels: the carriage mask from the platen calibration.  The dialog first learns
// how that region looks empty, then asks for the i1 and waits until the region looks clearly different and has held
// still for a moment -- the i1 seated, not a hand passing through.  Only then is OK offered, though the user may go
// on without the camera's word if it is unsure.
//
#ifndef LAUI1ATTACHDIALOG_H
#define LAUI1ATTACHDIALOG_H

#include <vector>
#include <QDialog>
#include <opencv2/core.hpp>

class QLabel;
class QPushButton;
class LAUImageView;

class LAUi1AttachDialog : public QDialog
{
    Q_OBJECT

public:
    // mask: camera-sized, non-zero over the cradle
    explicit LAUi1AttachDialog(const cv::Mat &mask, QWidget *parent = nullptr);

    bool isAttached() const
    {
        return attached;
    }
    double change() const
    {
        return lastChange;
    }

public slots:
    void onFrame(cv::Mat raw16);

private slots:
    void onOverride();

private:
    LAUImageView *view = nullptr;
    QLabel *status = nullptr;
    QPushButton *okButton = nullptr;

    cv::Mat mask;
    std::vector<int> samples;                    // pixel offsets inside the cradle, every 4th in each direction
    std::vector<double> reference, previous;
    int referenceFrames = 0;
    int steadyFrames = 0;
    double lastChange = 0.0;
    bool attached = false;

    void setStatus(const QString &text, const QString &color);
};

#endif // LAUI1ATTACHDIALOG_H
