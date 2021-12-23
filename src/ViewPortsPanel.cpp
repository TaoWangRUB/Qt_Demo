#include <QPainter>
#include <QPainterPath>
#include <QGridLayout>
#include <QDebug>
#include <QListWidget>

#include "ViewPortsPanel.h"

ViewPortsPanel::ViewPortsPanel(QWidget *parent)
    : QWidget{parent}
{
    // Prevent the viewports from collpasing and disappearing completely.
    setMinimumSize(40, 40);
    // Set the background color of the panel.
    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Window, QColor(80, 80, 80));
    setPalette(std::move(pal));

    auto r = geometry(); //rect();
    // set layout for current viewports
    createLayout();
    // Enable mouse tracking to implement hover effect for splitter handles.
    setMouseTracking(true);
    setAttribute(Qt::WA_Hover);

}

void ViewPortsPanel::createLayout() {
    QWidget *left = new QWidget(this);
    left->setObjectName(QStringLiteral("leftWidget"));
    QWidget *right = new QWidget(this);
    right->setObjectName(QStringLiteral("rightWidget"));
    QGridLayout *layout = new QGridLayout;
    layout->addWidget(left, 0, 0);
    layout->addWidget(right, 0, 1);
    setLayout(layout);
}

void ViewPortsPanel::paintEvent(QPaintEvent* event) {
    QPainter painter(this);
    // Render a border around the active viewport.
    QPen boardPen(QColor(255, 0, 0), 1);
    QPen linePen(QColor(255, 255, 0), 5);
    painter.setPen(boardPen);
    painter.setBrush(Qt::NoBrush);
    QRect rect = geometry();
    rect.adjust(0, 0, 0, -1);
    painter.drawRect(rect);
    //
    int index = 0;
    QList<QWidget *> childWidget = findChildren<QWidget *>(QString(), Qt::FindDirectChildrenOnly);

    for(auto child : childWidget) {
        //qDebug() << child->objectName() << child->metaObject()->className();
        painter.setPen(boardPen);
        painter.drawRect(child->geometry());

        index++;
        // transform world coordinate to windown coordinate
        painter.save();
        painter.translate(child->geometry().left() + child->geometry().width()/2.,
                          child->geometry().top() + child->geometry().height());
        // scale WCS to WCS
        painter.scale(child->geometry().width()/10., child->geometry().height()/110.);
        painter.translate(0., -10.);
        // flip y axis
        painter.scale(1, -1);
        QPen pen(Qt::cyan, 2.);
        // keep line width unscaled
        pen.setCosmetic(true);
        painter.setPen(pen);
        //painter.restore();
        double aLeft[4]{-2.0, 1e-2, 1e-4, 1e-6};
        double aRight[4]{2.0, 1e-2, 1e-4, 1e-6};

        auto getPoly = [](double* coeff, double xRange = 80., int num = 80) ->QPainterPath{
            QPainterPath path;
            path.moveTo(coeff[0], 0.);
            for (int i = 1; i <= num; ++i){
                auto x = i * xRange / num;
                auto y = coeff[0] + coeff[1] * x + coeff[2] * x * x + coeff[3] * x * x *x;
                path.lineTo(y, x);
            }
            return path;
        };
        if (child->objectName() == QString("leftWidget")){
            //qDebug() << child->objectName();
            auto lane = getPoly(aLeft);
            painter.drawPath(lane);
            lane = getPoly(aRight);
            painter.drawPath(lane);
        } else {
            // placeholder
            painter.drawLine(-5, 0, -5, 100);
            painter.drawLine(0, 0, 0, 100);
            painter.drawLine(5, 0, 5, 100);
            painter.drawLine(-5, 50, 5, 50);
        }
        // restore painter
        painter.restore();
    }
    /*painter.save();
    painter.setPen(QPen(Qt::black, 1.));
    painter.setBrush(Qt::red);
    painter.drawRect(0,0,100,100);
    painter.translate(geometry().width()/4, geometry().height()/2);
    painter.scale(2,3);
    painter.setBrush(Qt::green);
    painter.drawRect(-50,-50,100,100);
    painter.restore();*/
}
