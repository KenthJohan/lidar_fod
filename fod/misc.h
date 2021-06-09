#pragma once


#ifdef __MINGW32__
//pacman -S mingw64/mingw-w64-x86_64-openblas
//-lopenblas
#include <OpenBLAS/lapack.h>
#include <OpenBLAS/lapacke.h>
#include <OpenBLAS/cblas.h>
#else
//sudo apt-get install libblas-dev
//sudo apt-get install libopenblas-dev
//sudo apt-get install liblapacke-dev
#include <lapacke.h>
#include <cblas.h>
#endif

#include <stdint.h>

#include "csc_math.h"
#include "csc_v3f32.h"
#include "csc_v4f32.h"
#include "csc_m3f32.h"
#include "../shared/shared.h"
#include "../shared/ce30.h"



#define DISTANCE_ABOVE_GROUND 6.0f


#define POINTLABEL_OBJ 0x04 //This is a object
#define POINTLABEL_SECTOR 0x02 //This is a sector from LiDAR origin where objects can be labeled.
#define POINTLABEL_SEARCH 0x01 //This is all points that define PCA.
























