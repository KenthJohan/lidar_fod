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

#include "csc/csc_math.h"
#include "csc/csc_v3f32.h"
#include "csc/csc_v4f32.h"
#include "csc/csc_m3f32.h"


// Stupid:
#ifndef FLT_MAX
#define FLT_MAX __FLT_MAX__
#endif
#ifndef DBL_EPSILON
#define DBL_EPSILON __DBL_EPSILON__
#endif




// Sample configurations:
#define MIN_POINTS_IN_LIDAR 1000
#define MIN_POINTS_IN_BALL 150


// Detection configurations:
#define DETECT_ARCLENGTH 800
#define DETECT_BALL_RADIUS 0.3f
#define DETECT_BALL_RADIUS2 (DETECT_BALL_RADIUS*DETECT_BALL_RADIUS)


#define DETECT_MIN_EIGEN_FACTOR 6.0f
#define DETECT_MIN_EIGEN_FACTOR2 (DETECT_MIN_EIGEN_FACTOR*DETECT_MIN_EIGEN_FACTOR)

#define DETECT_MIN_GROUND_THICKNESS_RATIO 5.0f
#define DETECT_MIN_GROUND_THICKNESS_RATIO2 (DETECT_MIN_GROUND_THICKNESS_RATIO*DETECT_MIN_GROUND_THICKNESS_RATIO)


// Tracker configurations:
#define TRACKER_RADIUS 0.15f
#define TRACKER_CAPACITY 6
#define TRACKER_HITINCREMENT 0.2f
#define TRACKER_DISSIPATION 0.03f
#define TRACKER_MIN_HITS_RESCAN 0.15f
#define TRACKER_RESCAN_RADIUS (CE30_H*10)





// Visual only:
#define ENABLE_GRAPHIC
#define POINTLABEL_CLUSTER 0x08 //Not used.
#define POINTLABEL_OBJ 0x04 //Points above ground thus potential objects.
#define POINTLABEL_SECTOR 0x02 //This is a sector from LiDAR origin where objects can be labeled.
#define POINTLABEL_SEARCH 0x01 //This is all points that define PCA.


























