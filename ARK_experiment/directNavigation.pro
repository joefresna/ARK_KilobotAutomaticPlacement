#-------------------------------------------------
#
# Project created by QtCreator 2017-02-06T17:20:42
#
#-------------------------------------------------

QT       -= gui

QT += widgets

TARGET = directNavigationExp
TEMPLATE = lib

DEFINES += DIRECTNAVIGATIONEXP_LIBRARY


SOURCES += \
    kilobot.cpp \
    navigationEnv.cpp \
    navigationExp.cpp \
    hungarian.cpp

HEADERS +=\
    kilobot.h \
    kilobotexperiment.h \
    kilobotenvironment.h \
    navigationExp.h \
    navigationEnv.h \
    global.h \
    hungarian.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}

INCLUDEPATH += /usr/local/include/
INCLUDEPATH += /usr/local/Cellar/
INCLUDEPATH += /usr/local/Cellar/opencv/2.4.13/include

LIBS += -L/usr/local/lib \
        -lopencv_core \
        -L/usr/local/Cellar/opencv/2.4.13/lib \
        -lopencv_core \
        -lopencv_highgui \
        -lopencv_imgproc


