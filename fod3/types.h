#pragma once
#include "csc/csc_math.h"
#include "../shared/ce30.h"


#define TRACKER_CAPACITY 5



struct fodpca
{
	v3f32 o; // Offset from origin. Centroid. Mean. 3 vector.
	m3f32 c; // Coveriance matrix. 3x3 matrix
	v3f32 e[3]; // Eigen vectors. rotation matrix
	float w[3]; // Eigen values. variance
};

struct poitracker
{
	float r[TRACKER_CAPACITY];//Radius
	v3f32 x[TRACKER_CAPACITY];//Position
	uint32_t q[TRACKER_CAPACITY];//Position
};



struct fodcontext
{
	v3f32 x1[CE30_WH];     // Point position source
	v3f32 x2[CE30_WH];     // Point position rectified
	float h[CE30_WH];      // Point heightmap from ground
	float calib[CE30_WH];  // Calibration heightmap
	float a1[CE30_WH];     // Point amplitude
	uint8_t tags[CE30_WH]; // Point tags

	float kernel[3*3];     // Heightmap convolution kernel
	struct fodpca ground_pca;
	struct poitracker tracker;
	uint32_t num_above;
	uint32_t num_above_tot;
};



