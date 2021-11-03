gcc -ofod_probe fod.c flecs.c probe/probe.c \
-DIMPLEMENT_PROBE \
-I. \
-DNNG_STATIC_LIB \
-lnng \
-latomic \
-llapacke \
-llapack \
-lblas \
-lm \
-lpthread

gcc -ofod_milomqtt fod.c flecs.c milo/milomqtt.c \
-DIMPLEMENT_MILOMQTT \
-I. \
-latomic \
-llapacke \
-llapack \
-lblas \
-lm \
-lpthread \
-lmosquitto