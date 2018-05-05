TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    harris_suzuki.cpp

# add open CV
unix {
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv
}
