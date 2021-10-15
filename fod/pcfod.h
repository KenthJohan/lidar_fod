#pragma once
#include "csc/csc_xlog.h"
#include "csc/csc_crossos.h"
#include "csc/csc_malloc_file.h"
#include "csc/csc_filecopy.h"
#include "csc/csc_argv.h"
#include "csc/csc_assert.h"
#include "csc/csc_xlog.h"
#include "csc/csc_math.h"
#include "misc.h"
#include "../shared/ce30.h"




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
};


static void fodcontext_read (struct fodcontext * fod, FILE * f)
{
	ASSERT_PARAM_NOTNULL (fod);
	ASSERT_PARAM_NOTNULL (f);
	// Read 5 frames from CE30 LiDAR because it updates pointcloud per 5 frame.
	ce30_read (f, fod->pc_src, 5);
	memset (fod->pc_tags, 0, sizeof(uint8_t)*CE30_WH);
	ce30_xyzw_to_pos_amp_flags (fod->pc_src, fod->pc_x1, fod->pc_amplitude1, fod->pc_tags);
	ce30_detect_incidence_edges (fod->pc_tags);
}



