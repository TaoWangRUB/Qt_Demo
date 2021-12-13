#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QtWidgets/QAction>
#include <QFileDialog>

#include <Python.h>
#include <arrayobject.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    python_avail = python_initialization();
    ui->setupUi(this);
    connect(ui->actionOpen, &QAction::triggered, this, &MainWindow::open);
    connect(ui->actionExit, &QAction::triggered, this, &MainWindow::close);
    statusBar() ;
}

MainWindow::~MainWindow()
{
    delete ui;
    python_shutdown();
}

void MainWindow::open(){

    QString fileName = QFileDialog::getOpenFileName(this, tr("open log file"), "", tr("*.pickle"));
    if (!fileName.isNull()){
        auto pos = fileName.lastIndexOf("/");
        QString filePath = fileName.left(pos);
        QFile file(fileName);
        // open file
        if (!python_avail) {
            QMessageBox::warning(this, tr("python not available"), tr("module cant load"));
        }else if (!file.open(QFile::ReadOnly) ){
            QMessageBox::warning(this, tr("file open error"), tr(" can't open %1").arg(fileName));
        } else {
            //
            //import_array();
            PyObject* p_handle = PyObject_CallMethod(p_io_module_g, "open", "ss", fileName.toStdString().c_str(), "rb");
            PyObject* p_fd = PyObject_CallMethod(p_pickle_module_g, "load", "O", p_handle);
            // return New Reference
            PyObject *pobj_temp_key = PyUnicode_FromString("SP2021");
            // returns NULL if object is not found in dictionary, else pointer to object in dict
            //PyObject *pobj_object_found = PyDict_GetItemWithError(pobj_input_object, pobj_temp_key);

            //Py_Finalize();
            QMessageBox::information(this, tr("open log file"), tr("open file %1").arg(fileName));
        }
    }
}
