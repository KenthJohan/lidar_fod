#pragma once

#include "csc/csc_math.h"

#include "../shared/ce30.h"

#include "misc.h"
#include "tracker.h"




struct fodcontext
{
	// Pointclouds LiDAR source:
	v4f32 pc_src[CE30_WH];

	// Pointclouds LiDAR source using only position xyz:
	v3f32 pc_x1[CE30_WH];
	v3f32 pc_x2[CE30_WH];
	v3f32 pc_xtemp[CE30_WH];

	// Pointclouds LiDAR source only amplitude:
	float pc_amplitude1[CE30_WH];

	// Pointclouds per point flags:
	uint8_t pc_tags[CE30_WH];

	// Orientation of the ground:
	struct fodpca pca_sample;

	// Orientation of points whithin and around cluster the selected index:
	struct fodpca pca_cluster;

	// Selected cluster index:
	int32_t pc_index_cluster;

	float avg_roll;
	float avg_elevation;

	struct poitracker tracker;
};

