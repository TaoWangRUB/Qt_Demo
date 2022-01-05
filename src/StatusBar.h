#ifndef STATUSBAR_H
#define STATUSBAR_H
#include <QHBoxLayout>
#include <QStatusBar>

class StatusBar : public QStatusBar
{
    Q_OBJECT
public:
    explicit  StatusBar(QWidget *parent = nullptr);
private:
    /// The layout manager for the status bar area of the main window.
    QHBoxLayout* _statusBarLayout;
};

#endif // STATUSBAR_H
