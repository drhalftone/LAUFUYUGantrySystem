#include <QApplication>
#include "laucolorprofilerwindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // SHARE QSETTINGS WITH FuyuRailController SO THE GANTRY IP ADDRESS, CALIBRATION AND UNITS CARRY OVER
    a.setOrganizationName(QString("LAU"));
    a.setApplicationName(QString("FuyuRailController"));

    // THE GANTRY STARTS DISABLED; ITS CONNECT DIALOG APPEARS WHEN THE USER ENABLES IT IN THE WINDOW
    LAUColorProfilerWindow window;
    window.show();
    return (a.exec());
}
