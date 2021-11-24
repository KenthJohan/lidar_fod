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

#gcc -ofod_milomqtt fod.c flecs.c milo/milomqtt.c \
#-DIMPLEMENT_MILOMQTT \
#-I. \
#-latomic \
#-llapacke \
#-llapack \
#-lblas \
#-lm \
#-lpthread \
#-lmosquitto

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

cp -f fod_probe /usr/local/bin/fod_probe
cp -f fod_milomqtt /usr/local/bin/fod_milomqtt