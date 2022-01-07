#include <QApplication>
#include <QObject>

#include <QCommandLineParser>
#include <QCommandLineOption>

#include "mainwindow.h"

int main(int argc, char *argv[])
{
    // append resource figures
    Q_INIT_RESOURCE(AutoVision);
#ifdef Q_OS_ANDROID
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif

    if(QCoreApplication::startingUp()) {
        // Enable OpenGL context sharing globally.
        QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
        QCoreApplication::setAttribute(Qt::AA_UseDesktopOpenGL);
    }
    QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QApplication app(argc, argv);
    // Set the application icon.
    QIcon mainWindowIcon;
    mainWindowIcon.addFile(":/images/logo.png");
    // Set the application name.
    QCoreApplication::setApplicationName(QStringLiteral("AutoVision"));
    QCoreApplication::setOrganizationName(QObject::tr("AtuoVision"));
    QCoreApplication::setOrganizationDomain("AutoVision.org");
    QCoreApplication::setApplicationVersion(QStringLiteral("1.0"));
    QApplication::setWindowIcon(mainWindowIcon);

    QCommandLineParser parser;
    parser.setApplicationDescription(QCoreApplication::applicationName());
    parser.addHelpOption();
    parser.addVersionOption();
    parser.addPositionalArgument("file", "The file to open.");
    parser.process(app);

    MainWindow mainWin;
    if (!parser.positionalArguments().isEmpty())
        mainWin.loadFile(parser.positionalArguments().first());
    mainWin.show();

    return app.exec();
}
//! [0]
