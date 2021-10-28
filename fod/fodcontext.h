#pragma once

#include "csc/csc_math.h"

#include "../shared/ce30.h"

#include "misc.h"
#include "tracker.h"


#define PROX_SECTOR_COUNT 5
#define FOD_PROXIMITY_ALERT UINT32_C(0x00000010)

struct fodcontext
{
	uint32_t flags;


	// Pointclouds LiDAR source:
	v4f32 pc_src[CE30_WH];

	// Pointclouds LiDAR source using only position xyz:
	v3f32 pc_x1[CE30_WH];
	v3f32 pc_xtemp1[CE30_WH];
	v3f32 pc_xtemp2[CE30_WH];

	// Pointclouds LiDAR source only amplitude:
	float pc_amplitude1[CE30_WH];

	// Pointclouds per point flags:
	uint8_t pc_tags[CE30_WH];

	// Orientation of the sample:
	struct fodpca pca_sample;

	// Orientation of the cluster:
	struct fodpca pca_cluster;

	//
	struct fodpca pca_ground;

	// Selected cluster index:
	int32_t pc_index_cluster;

	int32_t sample_index;
	v3f32 sample_normal;
	float sample_delta;
	float sample_mean_variance;
	float sample_mean_roll;
	float sample_mean_elevation;

	struct poitracker tracker;



	uint32_t proximity[PROX_SECTOR_COUNT];
};

