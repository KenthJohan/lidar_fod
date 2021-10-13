gcc fod.c -ofod \
-I. \
-DNNG_STATIC_LIB \
-lnng \
-latomic \
-llapacke \
-llapack \
-lblas \
-lm \
-lpthread