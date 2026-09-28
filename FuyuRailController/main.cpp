/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/

#include <QApplication>
#include "lauvelmexwidget.h"

int main(int argc, char *argv[])
{
    QApplication application(argc, argv);

    // QSettings (used by the controller + widgets to persist limits / units / etc.)
    QCoreApplication::setOrganizationName(QString("LAU"));
    QCoreApplication::setApplicationName(QString("FuyuRailController"));

    // Let the user set the FMC4030 IP address / port (with a Test Connection button) before the
    // controller is created.  The dialog persists the values to QSettings, which the
    // LAUVelmexController reads on construction.  Cancel quits the application.
    LAUFuyuConnectDialog connectDialog;
    if (connectDialog.exec() != QDialog::Accepted) {
        return (0);
    }

    // The Velmex multi-axis widget, now driven by the FMC4030-backed LAUVelmexController.
    //   FMC4030 SDK axis 0 = X gantry (both rails share one controller axis)
    //   FMC4030 SDK axis 2 = Y cross-beam
    QList<int> axes;
    axes << 0 << 2;

    LAUMultiVelmexWidget widget(axes);
#ifdef FUYU_SIMULATE
    widget.setWindowTitle(QString("FUYU FMC4030 Rail Controller  —  SIMULATION (no hardware)"));
#else
    widget.setWindowTitle(QString("FUYU FMC4030 Rail Controller"));
#endif
    widget.show();

    return (application.exec());
}
