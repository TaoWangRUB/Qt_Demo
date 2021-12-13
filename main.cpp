#include <QApplication>
#include <iostream>
#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>

#include "mainwindow.h"

int main(int argc, char *argv[])
{
    std::cout << "Qt version: " << qVersion() << std::endl;

    QApplication app(argc, argv);

    MainWindow* win = new MainWindow();
    win->show();

    return app.exec();
}
