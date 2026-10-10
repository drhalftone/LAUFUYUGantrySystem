#include "lauargyllrunner.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSettings>
#include <QStandardPaths>

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
LAUArgyllRunner::LAUArgyllRunner(QObject *parent) : QObject(parent), process(nullptr)
{
    ;
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
LAUArgyllRunner::~LAUArgyllRunner()
{
    if (process) {
        process->kill();
        process->waitForFinished(3000);
    }
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
QString LAUArgyllRunner::binDirectory()
{
    QSettings settings;
    return (settings.value(QString("LAUArgyllRunner::binDirectory")).toString());
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUArgyllRunner::setBinDirectory(const QString &directory)
{
    QSettings settings;
    settings.setValue(QString("LAUArgyllRunner::binDirectory"), directory);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
QString LAUArgyllRunner::toolPath(const QString &tool)
{
    // 1) USER-CONFIGURED DIRECTORY, 2) COMMON ARGYLL INSTALL LOCATIONS, 3) PATH
    QStringList directories;
    QString configured = binDirectory();
    if (!configured.isEmpty()) {
        directories << configured;
    }
#if defined(Q_OS_WIN)
    directories << QString("C:/usr/bin")
                << QString("C:/Argyll/bin")
                << QString("C:/Program Files/Argyll/bin")
                << QString("C:/Program Files (x86)/Argyll/bin");
    QString executable = tool + QString(".exe");
#else
    directories << QString("/usr/local/bin") << QString("/usr/bin");
    QString executable = tool;
#endif
    for (const QString &directory : directories) {
        QString candidate = QDir(directory).filePath(executable);
        if (QFile::exists(candidate)) {
            return (candidate);
        }
    }

    QString onPath = QStandardPaths::findExecutable(tool);
    return (onPath.isEmpty() ? tool : onPath);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
bool LAUArgyllRunner::start(const QString &tool, const QStringList &arguments, const QString &workingDirectory)
{
    if (process) {
        errorString = QString("Argyll '%1' is still running.").arg(currentTool);
        return (false);
    }

    currentTool = tool;
    process = new QProcess(this);
    process->setProcessChannelMode(QProcess::MergedChannels);
    if (!workingDirectory.isEmpty()) {
        process->setWorkingDirectory(workingDirectory);
    }
    connect(process, &QProcess::readyRead, this, &LAUArgyllRunner::onReadyRead);
    connect(process, &QProcess::finished, this, &LAUArgyllRunner::onFinished);

    QString program = toolPath(tool);
    emit output(QString("> %1 %2").arg(program).arg(arguments.join(QChar(' '))));
    process->start(program, arguments);
    if (!process->waitForStarted()) {
        errorString = QString("Argyll '%1' did not launch (%2). Set the Argyll bin directory.").arg(tool).arg(process->errorString());
        process->deleteLater();
        process = nullptr;
        return (false);
    }
    errorString.clear();
    return (true);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
bool LAUArgyllRunner::targen(const QString &basename, Colorants colorants, int patches, int inkLimit, const QStringList &extra)
{
    QStringList arguments;
    arguments << QString("-d") << QString::number(static_cast<int>(colorants))
              << QString("-G")
              << QString("-f") << QString::number(patches);
    if (colorants == ColorantsCMYK && inkLimit > 0) {
        arguments << QString("-l") << QString::number(inkLimit);
    }
    arguments << extra << QFileInfo(basename).fileName();
    return (start(QString("targen"), arguments, QFileInfo(basename).absolutePath()));
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
bool LAUArgyllRunner::colprof(const QString &basename, const QStringList &extra)
{
    QStringList arguments;
    arguments << extra << QFileInfo(basename).fileName();
    return (start(QString("colprof"), arguments, QFileInfo(basename).absolutePath()));
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUArgyllRunner::onReadyRead()
{
    if (process) {
        QString text = QString::fromLocal8Bit(process->readAll()).trimmed();
        if (!text.isEmpty()) {
            emit output(text);
        }
    }
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUArgyllRunner::onFinished(int exitCode, QProcess::ExitStatus status)
{
    onReadyRead();
    bool success = (status == QProcess::NormalExit && exitCode == 0);
    if (!success) {
        errorString = QString("Argyll '%1' exited with code %2.").arg(currentTool).arg(exitCode);
    }

    QString tool = currentTool;
    process->deleteLater();
    process = nullptr;
    currentTool.clear();
    emit finished(tool, success);
}
