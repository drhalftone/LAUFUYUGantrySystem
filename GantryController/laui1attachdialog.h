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
#include <QString>
#include <opencv2/core.hpp>

class QLabel;
class QPushButton;
class LAUImageView;

// Is the i1 off the carriage?  Compares the live picture with the empty cradle that Calibrate platen recorded
// (cradle_reference.png, over the pixels in cradle_steady.png) when it found the aperture, which it can only do
// with the cradle empty.  By correlation, so the camera's exposure doesn't matter; by brightness where the cradle
// is too plain to correlate.
class LAUCradleCheck
{
public:
    bool load(const QString &bareScanFolder);    // false: no recording yet, so no check
    bool isLoaded() const
    {
        return !samples.empty();
    }

    // false when there's nothing to compare (no recording, or no frame of the right size)
    bool compare(const cv::Mat &raw16, double *similarity, bool *empty) const;

private:
    std::vector<int> samples;                    // pixel offsets of the steady cradle
    std::vector<double> values;                  // their empty-cradle grey, 0..255
    cv::Size size;
    double meanValue = 0.0, deviation = 0.0;
};

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
