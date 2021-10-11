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
	v4f32 pc_src[CE30_WH];
	v3f32 pc_x0[CE30_WH];
	v3f32 pc_x1[CE30_WH];
	v3f32 pc_xd[CE30_WH];
	float pc_amplitude1[CE30_WH];
	uint8_t pc_flags[CE30_WH];
	float pc_alpha[CE30_WH];
};


static void fodcontext_read (struct fodcontext * fod, FILE * f)
{
	ASSERT_PARAM_NOTNULL(fod);
	ASSERT_PARAM_NOTNULL(f);
	// Read 5 frames from CE30 LiDAR because it updates pointcloud per 5 frame.
	ce30_read (f, fod->pc_src, 5);
	memset (fod->pc_flags, 0, sizeof(uint8_t)*CE30_WH);
	ce30_xyzw_to_pos_amp_flags (fod->pc_src, fod->pc_x1, fod->pc_amplitude1, fod->pc_flags);
	ce30_detect_incidence_edges (fod->pc_flags);
}
