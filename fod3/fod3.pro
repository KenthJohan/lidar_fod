# -f "../ce30_clouds/ce30_pointcloud.out" -c
# -f "../ce30_clouds/ce30_pointcloud.out" -c


TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
#CONFIG += static

QMAKE_CFLAGS += -Wno-unused-function
QMAKE_CFLAGS += -Wno-unused-parameter

DEFINES += USING_QT_CREATOR
DEFINES += NNG_STATIC_LIB


SOURCES += probe/probe.c
SOURCES += milo/milomqtt_dummy.c
SOURCES += flecs.c
SOURCES += fod3.c

HEADERS += flecs.h
HEADERS += components.h
HEADERS += myent.h
HEADERS += misc.h
HEADERS += detection.h
HEADERS += tracker.h
HEADERS += graphics.h
HEADERS += fodcontext.h

HEADERS += probe/probe.h
HEADERS += milo/milomqtt.h

HEADERS += ../shared/mg_comp.h
HEADERS += ../shared/mg_attr.h
HEADERS += ../shared/mg_send.h
HEADERS += ../shared/ce30.h

HEADERS += csc_math.h
HEADERS += csc_linmat.h
HEADERS += csc_m3f32.h
HEADERS += csc_m3f32_print.h
HEADERS += csc_m4f32.h
HEADERS += csc_v3f32.h
HEADERS += csc_v3f32_print.h
HEADERS += csc_qf32.h
HEADERS += csc_xlog.h


INCLUDEPATH += C:/msys64/mingw64/include

LIBS += -LC:/msys64/mingw64/lib

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


