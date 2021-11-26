#ifndef MYWIDGET_H
#define MYWIDGET_H

#include <QWidget>
#include <QtDebug>
class MyWidget : public QWidget {
    Q_OBJECT;
signals:
    void my_signal(QString ms1, QString ms2);
public:
    void send(){
        emit my_signal(ms1, ms2);
    }
    void my_slot1(QString ms){
        qDebug() << " Call my_slot1 with message: " << ms;
    }
public slots:
    void my_slot2(QString ms1, QString ms2) {
        qDebug() << " call my_slot2 with message: " << ms1 << " " << ms2;
    }
public:
    QString ms1, ms2;
};

#endif // MYWIDGET_H
