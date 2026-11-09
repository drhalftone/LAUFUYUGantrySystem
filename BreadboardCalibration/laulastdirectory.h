/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// The folder the file dialogs start in: wherever the user last saved something, kept in QSettings.
//
#ifndef LAULASTDIRECTORY_H
#define LAULASTDIRECTORY_H

#include <QDir>
#include <QFileInfo>
#include <QSettings>
#include <QStandardPaths>

namespace LAULastDirectory
{
    inline QString get()
    {
        const QString saved = QSettings().value(QString("LAULastDirectory::path")).toString();
        return (!saved.isEmpty() && QDir(saved).exists()) ? saved : QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    }

    inline void rememberFile(const QString &filePath)
    {
        QSettings().setValue(QString("LAULastDirectory::path"), QFileInfo(filePath).absolutePath());
    }
}

#endif // LAULASTDIRECTORY_H
