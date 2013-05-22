#-------------------------------------------------
#
# Project created by QtCreator 2013-05-21T20:33:13
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = userspace
TEMPLATE = app

INCLUDEPATH += ../module

SOURCES += main.cpp\
        mainwindow.cpp \
    qcustomplot.cpp \
    scopeconst.cpp \
    deviceworker.cpp

HEADERS  += mainwindow.h \
    qcustomplot.h \
    scopeconst.h \
    deviceworker.h

FORMS    += mainwindow.ui
