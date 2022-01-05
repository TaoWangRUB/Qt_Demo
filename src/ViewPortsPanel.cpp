#include <QPainter>
#include <QPainterPath>
#include <QGridLayout>
#include <QDebug>
#include <QListWidget>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>

//#include <qwt_plot.h>

#include "ViewPortsPanel.h"

QT_CHARTS_USE_NAMESPACE

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

    // main plot widget
    QGridLayout* widgetLayoutLeft = new QGridLayout();
    QChartView *chartViewLeft = new QChartView(left);
    widgetLayoutLeft->addWidget(chartViewLeft);
    left->setLayout(widgetLayoutLeft);
    QGridLayout* widgetLayoutRight = new QGridLayout();
    QChartView *chartViewRight = new QChartView(right);
    widgetLayoutRight->addWidget(chartViewRight);
    right->setLayout(widgetLayoutRight);
    //chartView->setSceneRect(child->geometry());
    //chartView->setRenderHint(QPainter::Antialiasing);
    //child->update(child->geometry());
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
        //qDebug() << child->objectName() << " " << child->width() << " " << child->height();
        painter.setPen(boardPen);
        painter.drawRect(child->geometry());
        index++;
        // transform world coordinate to windown coordinate
        /*painter.save();
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
        painter.setPen(pen);*/
        // plot lane representation
        if (child->objectName() == "leftWidget"){
            double aLeft[4]{-2.0, 1e-2, 1e-4, 1e-6};
            double aRight[4]{2.0, 1e-2, 1e-4, 1e-6};
            auto getPoly = [](QLineSeries* path, double* coeff, double xRange = 80., int num = 80){
                for (int i = 0; i <= num; ++i){
                    auto x = i * xRange / num;
                    auto y = coeff[0] + coeff[1] * x + coeff[2] * x * x + coeff[3] * x * x *x;
                    path->append(y, x);
                }
                return;
            };
            QChart *chart = new QChart();
            chart->setTitle("Plan Viewer");
            QLineSeries* path1 = new QLineSeries();
            QLineSeries* path2 = new QLineSeries();
            getPoly(path1, aLeft);
            path1->setName("left");
            chart->addSeries(path1);
            getPoly(path2, aRight);
            path2->setName("right");
            chart->addSeries(path2);
            //chart->createDefaultAxes();
            QValueAxis *xAxis = new QValueAxis();
            QValueAxis *yAxis = new QValueAxis();
            chart->addAxis(xAxis, Qt::AlignBottom);
            chart->addAxis(yAxis, Qt::AlignLeft);
            xAxis->setRange(-5, 5);
            yAxis->setRange(-10, 100);
            xAxis->setTickCount(11);
            yAxis->setTickCount(12);
            xAxis->setTitleText(QStringLiteral("lateral / m"));
            yAxis->setTitleText(QStringLiteral("long / m"));
            xAxis->setLabelFormat("%d");
            yAxis->setLabelFormat("%d");

            path1->attachAxis(xAxis);
            path1->attachAxis(yAxis);
            path2->attachAxis(xAxis);
            path2->attachAxis(yAxis);
            auto chartView = child->findChild<QChartView*>(QString(), Qt::FindDirectChildrenOnly);
            chartView->setChart(chart);
        }
    }

}
