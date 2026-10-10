#ifndef LAUABOUTDIALOG_H
#define LAUABOUTDIALOG_H

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPixmap>

/****************************************************************************/
/* LAUAboutDialog
/* 
/* A modal dialog that displays application information including:
/* - Application name and version
/* - Software description
/* - Copyright information
/* - Build details (date, OS, architecture)
/* - Framework dependencies
/* 
/* Usage:
/*   LAUAboutDialog about(this);
/*   about.exec();  // Shows modal dialog
/* 
/* Features:
/* - Fixed size for consistent appearance
/* - Professional styling with different font sizes
/* - Dynamic copyright year
/* - Platform-specific build information
/* - Standard OK button for closing
/* - Keyboard navigation support (Enter/Escape)
/****************************************************************************/
class LAUAboutDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LAUAboutDialog(QWidget *parent = nullptr);
    ~LAUAboutDialog();

protected:
    void showEvent(QShowEvent *event)
    {
        this->setFixedSize(this->size());
    }

private:
    // UI setup method - creates and arranges all visual elements
    void setupUI();
    
    // Returns formatted version string (e.g., "Version 1.0.0")
    QString getVersionString() const;
    
    // Returns detailed build information including OS and architecture
    QString getBuildInfo() const;
};

#endif // LAUABOUTDIALOG_H
