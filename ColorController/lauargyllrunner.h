#ifndef LAUARGYLLRUNNER_H
#define LAUARGYLLRUNNER_H

#include <QObject>
#include <QProcess>
#include <QStringList>

/****************************************************************************/
/* LAUArgyllRunner                                                          */
/*                                                                          */
/* Runs one ArgyllCMS command-line tool at a time (targen, colprof, ...)    */
/* without blocking the GUI.  Argyll is only ever run as a separate         */
/* process, never linked in, which keeps its AGPL licence out of this app.  */
/****************************************************************************/
class LAUArgyllRunner : public QObject
{
    Q_OBJECT

public:
    explicit LAUArgyllRunner(QObject *parent = nullptr);
    ~LAUArgyllRunner();

    // DIRECTORY HOLDING targen.exe ETC.; EMPTY MEANS SEARCH THE USUAL PLACES, THEN PATH
    static QString binDirectory();
    static void setBinDirectory(const QString &directory);
    static QString toolPath(const QString &tool);

    bool isRunning() const
    {
        return (process != nullptr);
    }

    QString lastError() const
    {
        return (errorString);
    }

    bool start(const QString &tool, const QStringList &arguments, const QString &workingDirectory = QString());

    // targen's -d COLORANT COMBINATIONS WE USE
    enum Colorants { ColorantsPrintRGB = 2, ColorantsCMYK = 4 };

    // targen -d <colorants> -G -f <patches> [-l <inkLimit>] <basename>  ->  <basename>.ti1
    // (THE INK LIMIT ONLY APPLIES TO CMYK; PASS inkLimit <= 0 TO OMIT IT)
    bool targen(const QString &basename, Colorants colorants, int patches, int inkLimit = 0, const QStringList &extra = QStringList());

    // colprof <extra> <basename>  ->  <basename>.icc FROM <basename>.ti3
    bool colprof(const QString &basename, const QStringList &extra = QStringList());

signals:
    void output(QString text);
    void finished(QString tool, bool success);

private slots:
    void onReadyRead();
    void onFinished(int exitCode, QProcess::ExitStatus status);

private:
    QProcess *process;
    QString currentTool;
    QString errorString;
};

#endif // LAUARGYLLRUNNER_H
