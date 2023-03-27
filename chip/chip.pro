RESOURCES += images.qrc

HEADERS += mainwindow.h view.h chip.h \
    edge.h \
    graph.h \
    house.h \
    vertex.h
SOURCES += main.cpp \
    graph.cpp \
    flagitem.h \
    house.h
SOURCES += main.cpp \
    flagitem.cpp \
    house.cpp
SOURCES += mainwindow.cpp view.cpp chip.cpp

QT += widgets
qtHaveModule(printsupport): QT += printsupport

build_all:!build_pass {
    CONFIG -= build_all
    CONFIG += release
}

# install
target.path = $$[QT_INSTALL_EXAMPLES]/widgets/graphicsview/chip
INSTALLS += target

