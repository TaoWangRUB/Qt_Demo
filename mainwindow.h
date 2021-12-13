#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>


#include <Python.h>
#include <arrayobject.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void open();
private:
    bool python_initialization(){
        Py_Initialize();
        PyRun_SimpleString("import sys");
        PyRun_SimpleString("print(sys.version)");
        if (Py_IsInitialized() > 0){
            PyRun_SimpleString("print('load pickle')");
            //PyRun_SimpleString("sys.path.append('./')");
            p_io_module_g = PyImport_ImportModule("io");
            p_pickle_module_g = PyImport_ImportModule("pickle");

            if (p_pickle_module_g && p_io_module_g){
                return true;
            }
        }
        PyErr_Print();
        return false;
    }
    void python_shutdown() {
        Py_CLEAR(p_pickle_module_g);
        Py_CLEAR(p_io_module_g);
        p_io_module_g = NULL,
        p_pickle_module_g = NULL;
        Py_Finalize();
    }
private:
    Ui::MainWindow *ui;

    ///
    bool python_avail = true;
    /// python pickle module
    PyObject* p_pickle_module_g = nullptr;
    PyObject* p_io_module_g = nullptr;
};
#endif // MAINWINDOW_H
