#pragma once

#include <stdint.h>
#include "csc/csc_debug.h"
#include "csc/csc_m3f32.h"
#include "csc/csc_m3f32_print.h"
#include "csc/csc_v3f32.h"
#include "csc/csc_v3f32_print.h"

#include "../shared/shared.h"
#include "../shared/log.h"
#include "mathmisc.h"




struct pointcloud
{
	uint32_t framenr;
	//All points of pointcloud (x,y,z,a),(x,y,z,a)
	uint32_t n;
	struct v4f32 * x;
};

static void pointcloud_allocate (struct pointcloud * pc)
{
	pc->x = calloc(1, pc->n * sizeof (float) * POINT_STRIDE);
}

static void pointcloud_readfile (struct pointcloud * pc, FILE * f)
{
	pc->framenr = (float)ftell(f) / (float)(sizeof (float) * LIDAR_WH * POINT_STRIDE);
	printf ("Framenr: %i\n", pc->framenr);
	int r = fread (pc->x, sizeof (float) * POINT_STRIDE * LIDAR_WH, 1, f);
	ASSERTF (r == 1, "fread %i", r);
	pc->n = LIDAR_WH;
}


static void pointcloud_filter1 (struct pointcloud * pc, float d2)
{
	v4f32_filter_norm2_gt (pc->x, &pc->n, d2);
	printf ("Number of points: %i\n", pc->n);
}



static void pointcloud_process (struct pointcloud * pc)
{

}
