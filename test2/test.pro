TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += static

DEFINES += __USE_MINGW_ANSI_STDIO=1
DEFINES += USING_QT_CREATOR
DEFINES += NNG_STATIC_LIB

QMAKE_CFLAGS += -Wno-unused-function

SOURCES += test.c
SOURCES += flecs.c



INCLUDEPATH += C:/msys64/mingw64/include

LIBS += -LC:/msys64/mingw64/lib

LIBS += -Wl,-Bstatic
LIBS += -lnng
LIBS += -lws2_32 -lmswsock -ladvapi32 -lkernel32 -luser32 -lgdi32 -lwinspool -lshell32 -lole32 -loleaut32 -luuid -lcomdlg32 -ladvapi32
#LIBS += -Wl,-Bdynamic

LIBS += -lopenblas
#LIBS += -llapack
#LIBS += -llapacke
#LIBS += -lcblas
#LIBS += -lblas
LIBS += -lm
