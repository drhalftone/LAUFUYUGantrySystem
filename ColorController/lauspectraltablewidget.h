#ifndef LAUSPECTRALTABLEWIDGET_H
#define LAUSPECTRALTABLEWIDGET_H

#include <QWidget>
#include <QDialog>
#include <QPushButton>
#include <QVBoxLayout>
#include <QTableWidget>
#include <QDialogButtonBox>

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
class LAUSpectralTableWidget : public QWidget
{
    Q_OBJECT

public:
    explicit LAUSpectralTableWidget(QWidget *parent = nullptr);
    explicit LAUSpectralTableWidget(QString filename, QWidget *parent = nullptr);

    void saveTableToDisk(QString string = QString());
    void loadTableFromDisk(QString string = QString());

public slots:
    void onSaveTableToDisk();
    void onLoadTableFromDisk();
    void onAddSpectralMeasurement(QVector<double> vector);

private:
    QTableWidget *table;
};

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
class LAUSpectralTableDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LAUSpectralTableDialog(QWidget *parent = nullptr) : QDialog(parent)
    {
        this->setWindowTitle(QString("Classifier Filter"));
        this->setLayout(new QVBoxLayout());
        this->layout()->setContentsMargins(6, 6, 6, 6);
        widget = new LAUSpectralTableWidget();
        this->layout()->addWidget(widget);

        QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        connect(buttonBox->button(QDialogButtonBox::Ok), SIGNAL(clicked()), this, SLOT(accept()));
        connect(buttonBox->button(QDialogButtonBox::Cancel), SIGNAL(clicked()), this, SLOT(reject()));
        this->layout()->addWidget(buttonBox);
    }

    QSize size()
    {
        return (widget->size());
    }

public slots:

private:
    LAUSpectralTableWidget *widget;
};

#endif // LAUSPECTRALTABLEWIDGET_H
