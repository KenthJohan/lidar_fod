gcc fod.c -ofod \
-I../shared/csc \
-DNNG_STATIC_LIB \
-lnng \
-latomic \
-llapacke \
-llapack \
-lblas \
-lm \
-lpthread