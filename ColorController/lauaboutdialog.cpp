#include "lauaboutdialog.h"
#include <QApplication>
#include <QDate>
#include <QSysInfo>

// Application version - update this when releasing new versions
#define APP_VERSION "1.0.0"
// Build date is automatically set by the compiler at compile time
#define BUILD_DATE __DATE__

/****************************************************************************/
/* LAUAboutDialog Constructor
/* 
/* Creates and displays an About dialog with application information.
/* The dialog is modal and shows:
/* - Application name and version
/* - Description of the software
/* - Copyright information with current year
/* - Build information (date, OS, architecture)
/* - Framework dependencies (Qt, X-Rite SDK)
/****************************************************************************/
LAUAboutDialog::LAUAboutDialog(QWidget *parent) : QDialog(parent)
{
    setupUI();
    setWindowTitle("About LAU Color Controller");
    //setFixedSize(450, 350);  // Fixed size for consistent appearance
    // Remove the '?' help button from the title bar
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
LAUAboutDialog::~LAUAboutDialog()
{
}

/****************************************************************************/
/* setupUI
/* 
/* Creates the user interface layout for the About dialog.
/* Uses a vertical layout with proper spacing and margins.
/* Includes different sections with appropriate styling:
/* - Title: Large, bold application name
/* - Version: Current version number
/* - Description: What the software does
/* - Copyright: Legal information with dynamic year
/* - Build Info: Technical details about the build
/* - Dependencies: Framework information
/* - OK Button: Standard dialog button to close
/****************************************************************************/
void LAUAboutDialog::setupUI()
{
    // Main vertical layout with generous spacing for readability
    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(10);
    mainLayout->setContentsMargins(20, 20, 20, 20);
    
    // Application title - large, bold, and centered
    QLabel *titleLabel = new QLabel("LAU Color Controller");
    QFont titleFont = titleLabel->font();
    titleFont.setPointSize(18);
    titleFont.setBold(true);
    titleLabel->setFont(titleFont);
    titleLabel->setAlignment(Qt::AlignCenter);
    mainLayout->addWidget(titleLabel);
    
    // Version information - displayed below title
    QLabel *versionLabel = new QLabel(getVersionString());
    versionLabel->setAlignment(Qt::AlignCenter);
    mainLayout->addWidget(versionLabel);
    
    mainLayout->addSpacing(10);
    
    // Software description - what this application does
    QLabel *descLabel = new QLabel("i1Pro color measurement and ArgyllCMS profiling\nfor the FUYU gantry");
    descLabel->setAlignment(Qt::AlignCenter);
    descLabel->setWordWrap(true);  // Allow text wrapping if needed
    mainLayout->addWidget(descLabel);
    
    mainLayout->addSpacing(10);
    
    // Copyright notice - dynamically includes current year
    QLabel *copyrightLabel = new QLabel(QString("Copyright © %1 Lau Consulting Inc.\nAll rights reserved.")
                                       .arg(QDate::currentDate().year()));
    copyrightLabel->setAlignment(Qt::AlignCenter);
    mainLayout->addWidget(copyrightLabel);
    
    mainLayout->addSpacing(10);
    
    // Build information - shows when/where this was compiled
    QLabel *buildLabel = new QLabel(getBuildInfo());
    buildLabel->setAlignment(Qt::AlignCenter);
    QFont buildFont = buildLabel->font();
    buildFont.setPointSize(10);  // Smaller font for technical details
    buildLabel->setFont(buildFont);
    buildLabel->setStyleSheet("QLabel { color: #666; }");  // Gray color for less emphasis
    mainLayout->addWidget(buildLabel);
    
    // Flexible spacer to push framework info and button to bottom
    mainLayout->addStretch();
    
    // Framework dependencies - shows what libraries we're using
    QLabel *frameworkLabel = new QLabel("Using X-Rite Device SDK\nQt " QT_VERSION_STR);
    frameworkLabel->setAlignment(Qt::AlignCenter);
    QFont frameworkFont = frameworkLabel->font();
    frameworkFont.setPointSize(10);
    frameworkLabel->setFont(frameworkFont);
    frameworkLabel->setStyleSheet("QLabel { color: #666; }");
    mainLayout->addWidget(frameworkLabel);
    
    mainLayout->addSpacing(10);
    
    // OK button layout - centered at bottom of dialog
    QHBoxLayout *buttonLayout = new QHBoxLayout();
    buttonLayout->addStretch();  // Push button to center
    
    QPushButton *okButton = new QPushButton("OK");
    okButton->setDefault(true);  // Makes this the default button (activated by Enter)
    okButton->setFixedWidth(80);  // Standard button width
    // Connect button click to accept() which closes the dialog with Accepted result
    connect(okButton, &QPushButton::clicked, this, &QDialog::accept);
    buttonLayout->addWidget(okButton);
    
    buttonLayout->addStretch();  // Push button to center
    mainLayout->addLayout(buttonLayout);
}

/****************************************************************************/
/* getVersionString
/* 
/* Returns the formatted version string for display.
/* Currently uses a hardcoded version number defined at compile time.
/* In a production environment, this could be read from:
/* - A version file
/* - Git tags
/* - CMake/qmake variables
/* - Application resources
/****************************************************************************/
QString LAUAboutDialog::getVersionString() const
{
    return QString("Version %1").arg(APP_VERSION);
}

/****************************************************************************/
/* getBuildInfo
/* 
/* Constructs detailed build information including:
/* - Build date (automatically set by compiler's __DATE__ macro)
/* - Operating system and version (detected at runtime)
/* - CPU architecture (x86_64, arm64, etc.)
/* 
/* This information helps with:
/* - Debugging platform-specific issues
/* - Verifying correct builds for different platforms
/* - Support and troubleshooting
/****************************************************************************/
QString LAUAboutDialog::getBuildInfo() const
{
    QString buildInfo = QString("Built on %1").arg(BUILD_DATE);
    
    // Add platform-specific OS version information
#ifdef Q_OS_MAC
    buildInfo += QString("\nmacOS %1").arg(QSysInfo::productVersion());
#elif defined(Q_OS_WIN)
    buildInfo += QString("\nWindows %1").arg(QSysInfo::productVersion());
#elif defined(Q_OS_LINUX)
    buildInfo += QString("\nLinux %1").arg(QSysInfo::productVersion());
#endif
    
    // Add CPU architecture (useful for Apple Silicon vs Intel, etc.)
    buildInfo += QString("\n%1").arg(QSysInfo::buildCpuArchitecture());
    
    return buildInfo;
}
