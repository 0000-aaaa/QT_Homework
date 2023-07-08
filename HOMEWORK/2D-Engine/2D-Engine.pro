QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    block.cpp \
    bvh.cpp \
    circle.cpp \
    constraint.cpp \
    floor.cpp \
    longblock.cpp \
    main.cpp \
    point.cpp \
    polygon.cpp \
    precisebouncejudge.cpp \
    rope.cpp \
    widget.cpp

HEADERS += \
    Const.h \
    block.h \
    bvh.h \
    circle.h \
    constraint.h \
    floor.h \
    longblock.h \
    point.h \
    polygon.h \
    precisebouncejudge.h \
    rope.h \
    widget.h

FORMS += \
    widget.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    image.qrc
