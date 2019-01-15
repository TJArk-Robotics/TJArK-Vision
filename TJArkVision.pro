TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

QMAKE_CXXFLAGS += "-mssse3"

SOURCES += \
        main.cpp \
    lodepng/lodepng.cpp \
    image.cpp \
    fieldcolorprovider.cpp \
    scanlinesprovider.cpp \
    classifyimageprovider.cpp \
    fieldboundaryprovider.cpp \
    lineprovider.cpp \
    fieldspotprovider.cpp \
    centercircleprovider.cpp \
    ballprovider.cpp \
    edgeimageprovider.cpp

HEADERS += \
    imagetool.h \
    lodepng/lodepng.h \
    SIMD.h \
    image.h \
    fieldcolorprovider.h \
    scanlinesprovider.h \
    classifyimageprovider.h \
    color.h \
    fieldboundaryprovider.h \
    point.h \
    lineprovider.h \
    fieldspotprovider.h \
    centercircleprovider.h \
    ballprovider.h \
    edgeimageprovider.h \
    tjarkmath.h \
    cnn_classifier.h

