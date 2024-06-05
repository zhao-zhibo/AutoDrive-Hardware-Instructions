#-------------------------------------------------
#
# Project created by QtCreator 2019-02-19T11:38:09
#
#-------------------------------------------------

QT       -= core gui

QMAKE_CXXFLAGS += -std=c++0x

TARGET = navislam
TEMPLATE = lib
DEFINES += NAVISLAM_LIBRARY


SOURCES += \
    GaussNewtonSearch.cpp \
    ProbabilityValue.cpp \
    QuadLikeliHoodLayer.cpp \
    QuadNode.cpp \
    Envelope.cpp \
    Layer.cpp \
    transform.cpp \
    matlib.cpp \
    GaussProjCal.cpp \
    BaseNavTime.cpp \
    BaseNavModel.cpp \
    Sensor.cpp \
    ExGaussNewtonMatcher.cpp \
    ExQuadLikeliHoodLayer.cpp \
    ProbabilityValue16.cpp \
    ExBaseNavModel.cpp \
    IntegrateNav.cpp

HEADERS += \
    GaussNewtonSearch.h \
    ProbabilityValue.h \
    QuadLikeliHoodLayer.h \
    QuadNode.h \
    navislam_global.h \
    Envelope.h \
    Layer.h \
    transform.h \
    matlib.h \
    GaussProjCal.h \
    BaseNavTime.h \
    NavVisitor.h \
    BaseNavModel.h \
    Sensor.h \
    ExGaussNewtonMatcher.h \
    ExQuadLikeliHoodLayer.h \
    ProbabilityValue16.h \
    ExBaseNavModel.h \
    IntegrateNav.h
unix {
    target.path = /usr/lib
    INSTALLS += target
}
