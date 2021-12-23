#ifndef VIEWPORTSPANEL_H
#define VIEWPORTSPANEL_H

#include <QWidget>

class ViewPortsPanel : public QWidget
{
    Q_OBJECT
public:
    explicit ViewPortsPanel(QWidget *parent = nullptr);
    /// Returns the recommended size for this window.
    virtual QSize sizeHint() const override { return QSize(1024,768); }
    /// Create layout
    void createLayout();
protected:
    void paintEvent(QPaintEvent *event) override;
signals:

private:
    struct SplitterRectangle
    {
        QRect area;
        QWidget* widget;
        int childCellIndex;
        double dragFactor;
    };
    std::vector<SplitterRectangle> _splitterRegions;
};

#endif // VIEWPORTSPANEL_H
