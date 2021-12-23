QT += widgets
requires(qtConfig(filedialog))

HEADERS       = src/mainwindow.h \
    src/ViewPortsPanel.h
SOURCES       = src/main.cpp \
                src/ViewPortsPanel.cpp \
                src/mainwindow.cpp

RESOURCES     = AutoVision.qrc

INCLUDEPATH += /Users/archer/python38/include/python3.8
INCLUDEPATH += /Users/archer/python38/lib/python3.8/site-packages/numpy/core/include/numpy
LIBS += -L/Users/archer/python38/lib -lpython3.8

# install
