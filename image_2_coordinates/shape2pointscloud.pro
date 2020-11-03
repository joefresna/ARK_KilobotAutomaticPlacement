#-------------------------------------------------
#
# Project created by QtCreator 2017-09-26T11:08:59
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = shape_to_pointscloud
TEMPLATE = app

INCLUDEPATH += /usr/local/include/
#INCLUDEPATH += /usr/local/Cellar/opencv/2.4.13/include

#DEFINES += USE_OPENCV3
CONFIG += link_pkgconfig
PKGCONFIG += opencv

SOURCES += main.cpp\
        mainwindow.cpp \
    clicksignalqlabel.cpp

HEADERS  += mainwindow.h \
    clicksignalqlabel.h

FORMS    += mainwindow.ui

LIBS += -L/usr/local/lib/ \
        -lopencv_core \
        -lopencv_imgproc \
        -lopencv_highgui

RESOURCES += \
    ressources.qrc

#QMAKE_MAC_SDK = macosx10.14
