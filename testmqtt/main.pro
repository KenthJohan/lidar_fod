TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

QMAKE_CFLAGS += -Wno-unused-function
QMAKE_CFLAGS += -Wno-unused-parameter

SOURCES += main.c

INCLUDEPATH += C:/msys64/mingw64/include


LIBS += -Wl,-Bdynamic
LIBS += -lmosquitto

LIBS += -Wl,-Bdynamic
LIBS += -lmosquitto

LIBS += -Wl,-Bstatic
#LIBS += -Wl,-Bdynamic
LIBS += -lnng
LIBS += -lws2_32 -lmswsock -ladvapi32 -lkernel32 -luser32 -lgdi32 -lwinspool -lshell32 -lole32 -loleaut32 -luuid -lcomdlg32 -ladvapi32
LIBS += -lopenblas
#LIBS += -llapack
#LIBS += -llapacke
#LIBS += -lcblas
#LIBS += -lblas
LIBS += -lm

