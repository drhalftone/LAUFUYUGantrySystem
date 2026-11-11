/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
#include "laucornertunedialog.h"

#include <QLabel>
#include <QComboBox>
#include <QSettings>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout>
#include <QDialogButtonBox>

namespace
{
    const double STEPS_MM[] = { 0.1, 0.25, 0.5, 1.0, 2.0 };
    const char *STEP_KEY = "LAUCornerTuneDialog::step";
}

LAUCornerTuneDialog::LAUCornerTuneDialog(QWidget *parent) : QDialog(parent)
{
    setWindowTitle(QString("Fine-tune Page Corners"));
    setModal(false);                         // the live picture and its zoom stay usable

    QVBoxLayout *layout = new QVBoxLayout(this);
    message = new QLabel();
    message->setWordWrap(true);
    message->setMinimumWidth(380);
    layout->addWidget(message);

    QHBoxLayout *row = new QHBoxLayout();
    QGridLayout *pad = new QGridLayout();
    const auto arrow = [&](const QString &text, const QString &tip, int right, int down, int r, int c) {
        QPushButton *b = new QPushButton(text);
        b->setToolTip(tip);
        b->setFixedSize(44, 36);
        b->setAutoRepeat(false);
        connect(b, &QPushButton::clicked, this, [this, right, down]() { emit emitNudge(right, down); });
        pad->addWidget(b, r, c);
        arrows << b;
    };
    arrow(QString::fromUtf8("▲"), QString("Move the paper up in the picture"), 0, -1, 0, 1);
    arrow(QString::fromUtf8("◀"), QString("Move the paper left in the picture"), -1, 0, 1, 0);
    arrow(QString::fromUtf8("▶"), QString("Move the paper right in the picture"), 1, 0, 1, 2);
    arrow(QString::fromUtf8("▼"), QString("Move the paper down in the picture"), 0, 1, 2, 1);
    row->addLayout(pad);

    QVBoxLayout *side = new QVBoxLayout();
    side->addWidget(new QLabel(QString("Each press moves the paper")));
    step = new QComboBox();
    for (double mm : STEPS_MM) {
        step->addItem(QString("%1 mm").arg(mm), mm);
    }
    step->setCurrentIndex(qBound(0, QSettings().value(QString(STEP_KEY), 1).toInt(), step->count() - 1));
    connect(step, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [](int index) { QSettings().setValue(QString(STEP_KEY), index); });
    side->addWidget(step);
    nudge = new QLabel();
    side->addWidget(nudge);
    side->addStretch(1);
    row->addLayout(side, 1);
    layout->addLayout(row);

    QDialogButtonBox *buttons = new QDialogButtonBox();
    centeredButton = buttons->addButton(QString("Centered"), QDialogButtonBox::AcceptRole);
    centeredButton->setToolTip(QString("The corner is in the middle of the hole: remember these rail positions and go on to the next corner"));
    skipButton = buttons->addButton(QString("Skip corner"), QDialogButtonBox::ActionRole);
    QPushButton *cancel = buttons->addButton(QDialogButtonBox::Cancel);
    connect(centeredButton, &QPushButton::clicked, this, &LAUCornerTuneDialog::emitCentered);
    connect(skipButton, &QPushButton::clicked, this, &LAUCornerTuneDialog::emitSkip);
    connect(cancel, &QPushButton::clicked, this, &LAUCornerTuneDialog::reject);
    layout->addWidget(buttons);
    centeredButton->setDefault(true);
}

void LAUCornerTuneDialog::setMessage(const QString &text)
{
    message->setText(text);
}

void LAUCornerTuneDialog::setNudge(const QString &text)
{
    nudge->setText(text);
}

void LAUCornerTuneDialog::setBusy(bool busy)
{
    for (QPushButton *b : arrows) {
        b->setEnabled(!busy);
    }
    centeredButton->setEnabled(!busy);
    skipButton->setEnabled(!busy);
}

double LAUCornerTuneDialog::stepMM() const
{
    return step->currentData().toDouble();
}

void LAUCornerTuneDialog::reject()
{
    emit emitCancel();                       // the main window decides when it closes: a move may have to stop first
}
