gcc -ofod_probe.exe fod.c probe/probe.c \
-DIMPLEMENT_PROBE \
-Wall \
-Wno-unused-function \
-D__USE_MINGW_ANSI_STDIO=1 \
-DNNG_STATIC_LIB \
-I. \
-IC:/msys64/mingw64/include \
-I../shared/csc \
-LC:/msys64/mingw64/lib \
-Wl,-Bstatic \
-lnng \
-lws2_32 -lmswsock -ladvapi32 -lkernel32 -luser32 -lgdi32 -lwinspool -lshell32 -lole32 -loleaut32 -luuid -lcomdlg32 -ladvapi32 \
-lopenblas \
-lm


gcc -ofod_milomqtt.exe fod.c milo/milomqtt.c \
-DIMPLEMENT_MILOMQTT \
-Wall \
-Wno-unused-function \
-D__USE_MINGW_ANSI_STDIO=1 \
-DNNG_STATIC_LIB \
-I. \
-IC:/msys64/mingw64/include \
-I../shared/csc \
-LC:/msys64/mingw64/lib \
-Wl,-Bstatic \
-lnng \
-lws2_32 -lmswsock -ladvapi32 -lkernel32 -luser32 -lgdi32 -lwinspool -lshell32 -lole32 -loleaut32 -luuid -lcomdlg32 -ladvapi32 \
-lopenblas \
-lm