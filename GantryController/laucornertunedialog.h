/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAUCornerTuneDialog -- the message shown at each page corner during Fine-tune page corners.  It stays open beside
// the live picture without blocking it: the arrows nudge the rails so the paper moves that way in the picture, and
// Centered confirms that the corner sits in the middle of the i1 hole before the rails go on to the next corner.
//
#ifndef LAUCORNERTUNEDIALOG_H
#define LAUCORNERTUNEDIALOG_H

#include <QDialog>

class QLabel;
class QComboBox;
class QPushButton;

class LAUCornerTuneDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LAUCornerTuneDialog(QWidget *parent = nullptr);

    void setMessage(const QString &text);
    void setNudge(const QString &text);
    void setBusy(bool busy);                 // while the rails move: only Cancel works
    double stepMM() const;                   // how far, on the paper, one arrow press moves it

public slots:
    void reject() override;                  // Cancel, Esc and the title bar's X

signals:
    void emitNudge(int right, int down);     // move the paper this way in the picture (-1, 0 or 1 each)
    void emitCentered();
    void emitSkip();
    void emitCancel();

private:
    QLabel *message = nullptr;
    QLabel *nudge = nullptr;
    QComboBox *step = nullptr;
    QList<QPushButton *> arrows;
    QPushButton *centeredButton = nullptr;
    QPushButton *skipButton = nullptr;
};

#endif // LAUCORNERTUNEDIALOG_H
