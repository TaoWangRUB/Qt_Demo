//! [0]
#include <functional>

#include <QtWidgets>
#include <QSlider>
#include <QProgressBar>

#include "ViewPortsPanel.h"
#include "treemodel.h"
#include "mainwindow.h"

#pragma push_macro("slots")
#undef slots
#include "Python.h"
#pragma pop_macro("slots")
#include <arrayobject.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
//! [1] //! [2]
{
    // load python module
    python_avail = python_initialization();

    // set current windown title
    setWindowTitle(tr("%1 (Vehicle log visualization tool)").arg(QCoreApplication::applicationName()));
    //setAttribute(Qt::WA_DeleteOnClose);

    // Create the viewports panel and the data inspector panel.
    QSplitter* dataInspectorSplitter = new QSplitter();
    dataInspectorSplitter->setOrientation(Qt::Vertical);
    dataInspectorSplitter->setChildrenCollapsible(false);
    dataInspectorSplitter->setHandleWidth(0);
    // add viewport to splitter
    _viewportsPanel = new ViewPortsPanel(this);

    dataInspectorSplitter->addWidget(_viewportsPanel);
    // add dataInspector to splitter
    _dataInspector = new QWidget(this);
    QGridLayout* layout = new QGridLayout();
    QFile file(":/examples/default.txt");
    file.open(QIODevice::ReadOnly);
    TreeModel* model = new TreeModel(file.readAll());
    file.close();
    /*QFileSystemModel *model = new QFileSystemModel;
    model->setRootPath(QDir::currentPath());*/
    QTreeView* view = new QTreeView();
    view->setModel(model);
    //view->show();
    //view->setEnabled(true);
    layout->addWidget(view);
    _dataInspector->setLayout(layout);
    dataInspectorSplitter->addWidget(_dataInspector);
    dataInspectorSplitter->setStretchFactor(0, 1);
    dataInspectorSplitter->setStretchFactor(1, 0);

    setCentralWidget(dataInspectorSplitter);
    _viewportsPanel->setFocus(Qt::OtherFocusReason);

    // Set up the layout of docking widgets.
    setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
    setCorner(Qt::BottomRightCorner, Qt::RightDockWidgetArea);

    // set menu bar and tool bar
    createActions();
    // set status bar
    _statusBar = this->statusBar();
    createStatusBar();

    // Accept files via drag & drop.
    setAcceptDrops(true);

    setUnifiedTitleAndToolBarOnMac(true);
}

// destruction to release python resources
MainWindow::~MainWindow(){
    python_shutdown();
}

bool MainWindow::python_initialization(){
    Py_Initialize();
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("print(sys.version)");
    if (Py_IsInitialized() > 0){
        PyRun_SimpleString("print('load pickle')");
        //PyRun_SimpleString("sys.path.append('./')");
        py_io_module = PyImport_ImportModule("io");
        py_pickle_module = PyImport_ImportModule("pickle");

        if (py_pickle_module && py_io_module)
            return true;
    }
    PyErr_Print();
    return false;
}

void MainWindow::python_shutdown() {
    Py_CLEAR(py_pickle_module);
    Py_CLEAR(py_io_module);
    py_io_module = nullptr, py_pickle_module = nullptr;
    Py_Finalize();
}

void MainWindow::open(){
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("load pickle"),
                                                    QDir::currentPath()+"/../../../../examples",
                                                    tr("*.pickle"));
    if (!fileName.isEmpty())
        loadFile(fileName);
}

void MainWindow::about(){
   QMessageBox::about(this, tr("About AutoVision"),
            tr("The <b>Application</b> visualize vehicle logs with"
               "pickle formate."));
}

void MainWindow::createActions(){
    QMenuBar* menuBar = new QMenuBar(this);
    // create menu bar for file operation
    QMenu *fileMenu = menuBar->addMenu(tr("&File"));
    // create tool bar for file operation
    _mainToolbar = addToolBar(tr("Main Toolbar"));
    _mainToolbar->setObjectName("MainToolbar");
    //_mainToolbar->setMovable(false);

    // open action
    const QIcon openIcon = QIcon(":/images/file_open.bw.svg");
    QAction *openAct = new QAction(openIcon, tr("&Open..."), this);
    openAct->setShortcuts(QKeySequence::Open);
    openAct->setStatusTip(tr("Open an existing file"));
    connect(openAct, &QAction::triggered, this, &MainWindow::open);
    // add open action to file menu and tool bars
    fileMenu->addAction(openAct);
    _mainToolbar->addAction(openAct);

    // save action
    const QIcon saveIcon = QIcon(":/images/file_save.bw.svg");
    QAction *saveAct = new QAction(saveIcon, tr("&Save..."), this);
    saveAct->setShortcuts(QKeySequence::Save);
    saveAct->setStatusTip(tr("Save current configure"));
    connect(saveAct, &QAction::triggered, this, &QWidget::saveGeometry);
    // add open action to file menu and tool bars
    fileMenu->addAction(saveAct);
    _mainToolbar->addAction(saveAct);

    // add seperator
    fileMenu->addSeparator();
    _mainToolbar->addSeparator();

    // copy operation
    QMenu *editMenu = menuBar->addMenu(tr("&Edit"));
    const QIcon copyIcon = QIcon(":/images/copy.png");
    QAction *copyAct = new QAction(copyIcon, tr("&Copy"), this);
    copyAct->setShortcuts(QKeySequence::Copy);
    copyAct->setStatusTip(tr("Copy the current selection's contents to the "
                             "clipboard"));
    //connect(copyAct, &QAction::triggered, this, &QPlainTextEdit::copy);
    editMenu->addAction(copyAct);
    _mainToolbar->addAction(copyAct);

    // past operation
    const QIcon pasteIcon = QIcon(":/images/paste.png");
    QAction *pasteAct = new QAction(pasteIcon, tr("&Paste"), this);
    pasteAct->setShortcuts(QKeySequence::Paste);
    pasteAct->setStatusTip(tr("Paste the clipboard's contents into the current "
                              "selection"));
    //connect(pasteAct, &QAction::triggered, textEdit, &QPlainTextEdit::paste);
    editMenu->addAction(pasteAct);
    _mainToolbar->addAction(pasteAct);

    // cut
    const QIcon cutIcon = QIcon(":/images/cut.png");
    QAction *cutAct = new QAction(cutIcon, tr("Cu&t"), this);
//! [21]
    cutAct->setShortcuts(QKeySequence::Cut);
    cutAct->setStatusTip(tr("Cut the current selection's contents to the "
                            "clipboard"));
    //connect(cutAct, &QAction::triggered, textEdit, &QPlainTextEdit::cut);
    editMenu->addAction(cutAct);
    _mainToolbar->addAction(cutAct);

    menuBar->addSeparator();
    _mainToolbar->addSeparator();

    // rendering
    const QIcon renderIcon = QIcon(":/images/render_active_viewport.bw.svg");
    QAction *renderAct = new QAction(renderIcon, tr("&Screenshot..."), this);
    renderAct->setShortcuts(QKeySequence::Print);
    renderAct->setStatusTip(tr("save current screenshot"));
    //connect(openAct, &QAction::triggered, this, &QWidget::close);
    // add open action to tool bars
    _mainToolbar->addAction(renderAct);
    _mainToolbar->addSeparator();

    // exit action
    const QIcon exitIcon = QIcon(":/images/file_quit.bw.svg");
    QAction *exitAct = fileMenu->addAction(exitIcon, tr("E&xit"));
    exitAct->setShortcuts(QKeySequence::Quit);
    exitAct->setStatusTip(tr("Exit the application"));
    connect(exitAct, &QAction::triggered, this, &QWidget::close);
    // add open action to file menu and tool bars
    fileMenu->addAction(exitAct);
    _mainToolbar->addAction(exitAct);

    // create menu bar for help operation
    QMenu *helpMenu = menuBar->addMenu(tr("&Help"));
    // about action --> help menu
    QAction *aboutAct = helpMenu->addAction(tr("&About"), this, &MainWindow::about);
    aboutAct->setStatusTip(tr("Show the application's About box"));
    // Qtabout action --> help menu
    QAction *aboutQtAct = helpMenu->addAction(tr("About &Qt"), qApp, &QApplication::aboutQt);
    aboutQtAct->setStatusTip(tr("Show the Qt library's About box"));

    setMenuBar(menuBar);
}
//! [24]

//! [32]
void MainWindow::createStatusBar() {
    // Create the animation panel below the viewports.
    QWidget* animationPanel = new QWidget();
    QHBoxLayout* animationPanelLayout = new QHBoxLayout();
    animationPanelLayout->setSpacing(0);
    animationPanelLayout->setContentsMargins(0, 1, 0, 0);
    animationPanel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    animationPanel->setLayout(animationPanelLayout);

    // Create animation time slider
    QLabel *label1 = new QLabel(tr("Frame"));
    label1->setStyleSheet(" QLabel{ color: yellow }");
    animationPanelLayout->addWidget(label1);
    QSlider* timeSlider = new QSlider(Qt::Horizontal, this);
    animationPanelLayout->addWidget(timeSlider);
    QLineEdit *frameLabel = new QLineEdit;
    frameLabel->setStyleSheet(" QLineEdit{ color: yellow }");
    frameLabel->setPlaceholderText(tr("%1").arg(0));
    animationPanelLayout->addWidget(frameLabel);
    QProgressBar* trackBar = new QProgressBar(this);
    animationPanelLayout->addWidget(trackBar);
    _statusBar->addPermanentWidget(animationPanel);
    //statusBar()->showMessage(tr("Ready"));
    _statusBar->showMessage(tr("Ready"));
}
//! [33]
/*
//! [34] //! [35]
void MainWindow::readSettings()
//! [34] //! [36]
{
    QSettings settings(QCoreApplication::organizationName(), QCoreApplication::applicationName());
    const QByteArray geometry = settings.value("geometry", QByteArray()).toByteArray();
    if (geometry.isEmpty()) {
        const QRect availableGeometry = screen()->availableGeometry();
        resize(availableGeometry.width() / 3, availableGeometry.height() / 2);
        move((availableGeometry.width() - width()) / 2,
             (availableGeometry.height() - height()) / 2);
    } else {
        restoreGeometry(geometry);
    }
}
//! [35] //! [36]

//! [37] //! [38]
void MainWindow::writeSettings()
//! [37] //! [39]
{
    QSettings settings(QCoreApplication::organizationName(), QCoreApplication::applicationName());
    settings.setValue("geometry", saveGeometry());
}
//! [38] //! [39]

//! [40]
bool MainWindow::maybeSave()
//! [40] //! [41]
{
    if (!textEdit->document()->isModified())
        return true;
    const QMessageBox::StandardButton ret
        = QMessageBox::warning(this, tr("Application"),
                               tr("The document has been modified.\n"
                                  "Do you want to save your changes?"),
                               QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
    switch (ret) {
    case QMessageBox::Save:
        return save();
    case QMessageBox::Cancel:
        return false;
    default:
        break;
    }
    return true;
}
//! [41]
*/
//! [42]
void MainWindow::loadFile(const QString &fileName) {
    QFile file(fileName);
    bool signalOk = true;
    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        QMessageBox::warning(this, tr("Application"),
                             tr("Cannot read file %1:\n%2.").arg(QDir::toNativeSeparators(fileName), file.errorString()));
        return;
    } else {
        PyObject *p_handle = PyObject_CallMethod(py_io_module, "open", "ss", fileName.toStdString().c_str(), "rb");
        PyObject* py_dict = PyObject_CallMethod(py_pickle_module, "load", "O", p_handle);
        // return New Reference
        auto key = "SP2021";
        auto val = PyDict_GetItemWithError(py_dict, PyUnicode_FromString(key));
        if(val == NULL){
            signalOk = false;
            _statusBar->showMessage(tr("%1 is missing from pickle log file").arg(key));
        }
        //testing
        PyObject* subkey, *subitem;
        std::function<void(PyObject*, PyObject*)> dfs = [&](PyObject* key, PyObject* item){
            int level = 0;
            PyObject* keys = PyDict_Keys(item);
            for (int i = 0; i < PyList_Size(keys); ++i){
                level += 1;
                subkey = PyList_GetItem(keys, i);
                subitem = PyDict_GetItem(item, subkey);
                if(PyDict_Check(subitem) && PyUnicode_AsUTF8(subkey) != "undecoded"){
                    qDebug() << PyUnicode_AsUTF8(key) << " " << PyUnicode_AsUTF8(subkey) << " "<< level;
                    dfs(subkey, subitem);
                } else
                    qDebug() << PyUnicode_AsUTF8(key) << " " << PyUnicode_AsUTF8(subkey) << " "<< level;
            }
        };
        dfs(PyUnicode_FromString(key), val);
        Py_DECREF(subitem);
        Py_DECREF(subkey);
        Py_CLEAR(p_handle);
        Py_CLEAR(py_dict);
    }
    //QMessageBox::information(this, tr("open log file"), tr("open file %1").arg(fileName));

    _statusBar->showMessage(tr("File loaded %1").arg(_statusBar->currentMessage()));

}

PyObject* MainWindow::PyDict_GetItemWrapper(PyObject* const py_dict_object, char* const key)
{
    /// this function returns a borrowed reference; no need to call decrease reference
    if (py_dict_object == NULL)
    {
        return NULL;
    }
    else
    {
        // return New Reference
        PyObject *py_key = PyUnicode_FromString(key);
        // returns NULL if object is not found in dictionary, else pointer to object in dict
        PyObject *pobj_object_found = PyDict_GetItemWithError(py_dict_object, py_key);
        if (NULL == pobj_object_found)
        {
            _statusBar->showMessage(tr("%1 is missing from pickle log file").arg(key));
        }
        // destruct object
        Py_CLEAR(py_key);
        // this is a borrowed reference, no need to deallocated
        return pobj_object_found;
    }
}

//! [43]
/*
//! [44]
bool MainWindow::saveFile(const QString &fileName)
//! [44] //! [45]
{
    QString errorMessage;

    QGuiApplication::setOverrideCursor(Qt::WaitCursor);
    QSaveFile file(fileName);
    if (file.open(QFile::WriteOnly | QFile::Text)) {
        QTextStream out(&file);
        out << textEdit->toPlainText();
        if (!file.commit()) {
            errorMessage = tr("Cannot write file %1:\n%2.")
                           .arg(QDir::toNativeSeparators(fileName), file.errorString());
        }
    } else {
        errorMessage = tr("Cannot open file %1 for writing:\n%2.")
                       .arg(QDir::toNativeSeparators(fileName), file.errorString());
    }
    QGuiApplication::restoreOverrideCursor();

    if (!errorMessage.isEmpty()) {
        QMessageBox::warning(this, tr("Application"), errorMessage);
        return false;
    }

    setCurrentFile(fileName);
    statusBar()->showMessage(tr("File saved"), 2000);
    return true;
}
//! [45]

//! [46]
void MainWindow::setCurrentFile(const QString &fileName)
//! [46] //! [47]
{
    curFile = fileName;
    textEdit->document()->setModified(false);
    setWindowModified(false);

    QString shownName = curFile;
    if (curFile.isEmpty())
        shownName = "untitled.txt";
    setWindowFilePath(shownName);
}
//! [47]

//! [48]
QString MainWindow::strippedName(const QString &fullFileName)
//! [48] //! [49]
{
    return QFileInfo(fullFileName).fileName();
}
//! [49]
#ifndef QT_NO_SESSIONMANAGER
void MainWindow::commitData(QSessionManager &manager)
{
    if (manager.allowsInteraction()) {
        if (!maybeSave())
            manager.cancel();
    } else {
        // Non-interactive: save without asking
        if (textEdit->document()->isModified())
            save();
    }
}
#endif
*/
