#pragma once
#include "csc/csc_math.h"
#include "../shared/ce30.h"



// Sample configurations:
#define MIN_POINTS_IN_LIDAR 1000
#define MIN_POINTS_IN_BALL 50


// Detection configurations:
#define DETECT_ARCLENGTH 700
// Note: Changing the radius will change the ratio between shortest and medium eigen values:
#define DETECT_BALL_RADIUS 0.2f
#define DETECT_BALL_RADIUS2 (DETECT_BALL_RADIUS*DETECT_BALL_RADIUS)


#define DETECT_MIN_EIGEN_RATIO 5.0f
#define DETECT_MIN_EIGEN_RATIO2 (DETECT_MIN_EIGEN_RATIO*DETECT_MIN_EIGEN_RATIO)

#define DETECT_MIN_GROUND_THICKNESS_RATIO 4.0f
#define DETECT_MIN_GROUND_THICKNESS_RATIO2 (DETECT_MIN_GROUND_THICKNESS_RATIO*DETECT_MIN_GROUND_THICKNESS_RATIO)


// Tracker configurations:
#define TRACKER_RADIUS 0.15f
#define TRACKER_CAPACITY 6
#define TRACKER_HITINCREMENT 0.2f
#define TRACKER_DISSIPATION 0.03f
#define TRACKER_MIN_HITS_RESCAN 0.15f
#define TRACKER_RESCAN_RADIUS (CE30_H*10)



struct fodpca
{
	v3f32 o; // Offset from origin. Centroid. Mean. 3 vector.
	m3f32 c; // Coveriance matrix. 3x3 matrix
	v3f32 e[3]; // Eigen vectors
	float w[3]; // Eigen values
};

struct fodcontext
{
	v3f32 x1[CE30_WH];
	v3f32 x2[CE30_WH];
	float calib[CE30_WH];
	float a1[CE30_WH];
	uint8_t tags[CE30_WH];
	float kernel[3*3];

	struct fodpca ground_pca;


	struct
	{
		v3f32 x[CE30_WH];
	} trackers;


	uint32_t num_above;
	uint32_t num_above_tot;
};


struct poitracker
{
	uint32_t count;//Not used currently
	float r[TRACKER_CAPACITY];//Radius
	v3f32 x[TRACKER_CAPACITY];//Position
	float h[TRACKER_CAPACITY];//History
	uint32_t i[TRACKER_CAPACITY];//Pointcloud Index
};
