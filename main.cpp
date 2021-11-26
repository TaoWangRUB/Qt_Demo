#include <QApplication>
#include <iostream>
#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>

#include "mainwindow.h"
#include "MyWidget.h"

#include <Python.h>

void my_slot3(){
    qDebug() << "Call my_slot3 with message ";
}
int main(int argc, char *argv[])
{
    std::cout << "Qt version: " << qVersion() << std::endl;
    QApplication a(argc, argv);
    //MainWindow w;
    //w.show();

    // create main window
    MyWidget widget;
    widget.ms1 = "Test 1";
    widget.ms2 = "Test 2";
    QObject::connect(&widget, &MyWidget::my_signal, &widget, &MyWidget::my_slot1);
    QObject::connect(&widget, &MyWidget::my_signal, &widget, &MyWidget::my_slot2);
    QObject::connect(&widget, &MyWidget::my_signal, &widget, &my_slot3);
    widget.setWindowTitle("QVBoxLayout vertical");

    // create a new layout
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setDirection(QBoxLayout::BottomToTop);

    // create new labels and others
    QLabel lab1("Label1");
    lab1.setStyleSheet("QLabel{background::#dddddd;font:20px}");
    lab1.setAlignment(Qt::AlignCenter);
    QLabel lab2("Label2");
    lab2.setStyleSheet("QLabel{background::#dddddd;font:20px}");
    lab2.setAlignment(Qt::AlignCenter);
    QLabel lab3("Label3");
    lab3.setStyleSheet("QLabel{background::#dddddd;font:20px}");
    lab3.setAlignment(Qt::AlignCenter);
    QPushButton button("close", &widget);
    button.setGeometry(10, 10, 100, 50);
    QObject::connect(&button, &QPushButton::clicked, &widget, &QWidget::close);

    // add label to layout
    layout->addWidget(&lab1, 1);
    layout->addWidget(&lab2, 1);
    layout->addWidget(&lab3, 1);
    layout->addStretch(2);
    layout->addWidget(&button, 1);

    // add layout into widget window
    widget.setLayout(layout);
    widget.show();
    // send signal
    widget.send();
    return a.exec();
}
