#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QHBoxLayout>

QT_BEGIN_NAMESPACE
class QAction;
class QMenu;
class QSessionManager;
QT_END_NAMESPACE

#pragma push_macro("slots")
#undef slots
#include "Python.h"
#pragma pop_macro("slots")
#include <arrayobject.h>

//! [0]
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void loadFile(const QString &fileName);
/*
protected:
    void closeEvent(QCloseEvent *event) override;*/

private slots:
    void open();
    void about();

private:
    /// initialize python
    bool python_initialization();
    /// release python resource
    void python_shutdown();
    /// create actions
    void createActions();
    /// create status bar
    void createStatusBar();

    void readSettings();
    void writeSettings();
    bool maybeSave();
    bool saveFile(const QString &fileName);
    void setCurrentFile(const QString &fileName);
private:
    ///
    bool python_avail = true;
    /// python pickle module
    PyObject* p_pickle_module_g = nullptr;
    PyObject* p_io_module_g = nullptr;

    QString strippedName(const QString &fullFileName);

    QString curFile;

    QWidget* _viewportsPanel;
    /// The UI panel containing the data inspector tabs.
    QWidget* _dataInspector;
    /// The upper main toolbar.
    QToolBar* _mainToolbar;
    /// The internal status bar widget.
    //StatusBar* _statusBar;
    /// The layout manager for the status bar area of the main window.
    QHBoxLayout* _statusBarLayout;
    /// The title string to use for the main window (without any dynamic content).
    QString _baseWindowTitle;
};
//! [0]

#endif
