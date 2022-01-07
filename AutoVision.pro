QT += widgets
QT += charts
requires(qtConfig(filedialog))

HEADERS       = src/mainwindow.h \
    src/ViewPortsPanel.h \
    src/treeitem.h \
    src/treemodel.h
SOURCES       = src/main.cpp \
                src/ViewPortsPanel.cpp \
                src/mainwindow.cpp \
                src/treeitem.cpp \
                src/treemodel.cpp

RESOURCES     = AutoVision.qrc

INCLUDEPATH += /Users/archer/python38/include/python3.8
INCLUDEPATH += /Users/archer/python38/lib/python3.8/site-packages/numpy/core/include/numpy
LIBS += -L/Users/archer/python38/lib -lpython3.8
#include ( /usr/local/qwt-6.2.0/features/qwt.prf )
#INCLUDEPATH += /usr/local/qwt-6.2.0/include
#LIBS += -L/usr/local/qwt-6.2.0/lib -lqwt

# install
