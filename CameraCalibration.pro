#-------------------------------------------------
#
# Project created by QtCreator 2018-12-12T01:37:02
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = CameraCalibration
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

SOURCES += \
        main.cpp \
        mainwindow.cpp \
        Preprocessing/preprocessing.cpp \
        Preprocessing/utils.cpp \
        Patron/structures.cpp \
        Patron/mapping.cpp \
    setting.cpp \
    calibratedgrid.cpp

HEADERS += \
        mainwindow.h \
        Preprocessing/preprocessing.h \
        Preprocessing/utils.h \
        Patron/mapping.h \
        Patron/structures.h \
        setting.h \
    calibratedgrid.h

# OpenCV
INCLUDEPATH += /usr/local/include/opencv2
LIBS += -L/usr/local/lib -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_videoio
CONFIG += link_pkgconfig
PKGCONFIG += opencv

# OpenMP
QMAKE_CXXFLAGS += -fopenmp
LIBS += -fopenmp

FORMS += \
        mainwindow.ui \
    setting.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

DISTFILES += \
    README.md
