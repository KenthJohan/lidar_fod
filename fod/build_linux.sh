gcc -ofod_probe fod.c probe/probe.c \
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

gcc -ofod_milomqtt fod.c milo/milomqtt.c \
-DIMPLEMENT_MILOMQTT \
-I. \
-DNNG_STATIC_LIB \
-lnng \
-latomic \
-llapacke \
-llapack \
-lblas \
-lm \
-lpthread