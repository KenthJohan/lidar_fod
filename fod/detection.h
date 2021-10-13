#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <float.h>

#include "csc/csc_assert.h"
#include "csc/csc_f32.h"
#include "csc/csc_m3f32.h"
#include "csc/csc_m3f32_print.h"
#include "csc/csc_v3f32.h"
#include "csc/csc_v3f32_print.h"
#include "csc/csc_v4f32.h"
#include "csc/csc_xlog.h"

#include "../shared/ce30.h"

#include "misc.h"
#include "graphics.h"
#include "tracker.h"
#include "pointcloud.h"
#include "pcfod.h"


#define DECTECT_FAILED 0
#define DECTECT_SUCCESS 1


uint32_t select_pca_points (v3f32 const x[], uint32_t n, v3f32 const * c, v3f32 y[], float r)
{
	uint32_t m = 0;
	for(uint32_t i = 0; i < n; ++i)
	{
		v3f32 d;
		v3f32_sub (&d, x + i, c);
		if (v3f32_norm2 (&d) < (r*r))
		{
			y[m] = x[i];
			m++;
		}
	}
	return m;
}


static uint32_t detection_sample
(struct poitracker * trackers, int32_t sample_index, struct fodcontext * fod)
{
	uint8_t * cid = fod->pc_flags;
	v3f32 const * x = fod->pc_x1;


	if ((cid[sample_index] & CE30_POINT_GOOD) == 0)
	{
		return DECTECT_FAILED;
	}

	// Select sample coordinate
	v3f32 const * s = x + sample_index;

	// Temporary subcloud:
	v3f32 x1[CE30_WH];

	// Number of point in ball:
	uint32_t m;


	{
		// Copy (n) amount of points (x) at position (s) within sphere to (x1):
		// Where (m) is number of point in ball:
		m = select_pca_points (x, CE30_WH, s, x1, DETECT_BALL_RADIUS);

		// Visual only:
		for(uint32_t i = 0; i < CE30_WH; ++i)
		{
			v3f32 d;
			v3f32_sub (&d, x + i, s);
			if (v3f32_norm2 (&d) < DETECT_BALL_RADIUS2)
			{
				// Tag this point as part of ball:
				cid[i] |= CE30_POINT_SEARCH;
			}
		}

		// Enough points must be within the sphere to make a good detection:
		if (m < MIN_POINTS_IN_BALL)
		{
			return DECTECT_FAILED;
		}
	}

	// Begin PCA calculation from subcloud (x1).
	// PCA will produce: orientation (e), centroid (o), variance (w) of subcloud (x1).
	// o : Centroid of subcloud (x1). i.e. subcloud offset (o) from origin (0,0,0).
	// c : 3x3 coveriance matrix of subcloud (x1). Contains orientation and variance.
	// e : Three eigen column vectors (Shortest, Medium, Farthest) of coveriance matrix (c). Contains only orientation.
	// w : Three eigen values (Shortest, Medium, Farthest) of coveriance matrix (c). Contains variance.
	{

		v3f32 * o = &(fod->pca.o);
		m3f32 * c = &(fod->pca.c);
		v3f32 * e = fod->pca.e;
		float * w = fod->pca.w;
		v3f32_set1 (o, 0.0f);
		v3f32_meanacc (o, x1, m);
		v3f32_subv (x1, x1, o, 1, 1, 0, m); // x1[i] := x1[i] - o[0], (where i is 0 .. m)
		pointcloud_covariance (x1, m, c, 1.0f);
		pointcloud_eigen (c, e, w);
		//https://stackoverflow.com/questions/2782647/how-to-get-yaw-pitch-and-roll-from-a-3d-vector
		fod->pca.elevation = atan2(e[0].x, e[0].z);
		fod->pca.roll = asin(-e[0].y);
	}
	// End PCA calculation





	// Check if the ground box is thin enough to reliably detect points above the ground box:
	// These are the ground box dimensions:
	// w[0] = Shortest length m^2, w[1] = Medium length m^2, w[2] = Farthest length m^2.
	// Note that the ground box dimensions is expressed as eigen values (w[0], w[1], w[2]) which is in square meter (m^2).
	// To get the actual thickness of the box do sqrtf(w[0]).
	if (DETECT_MIN_GROUND_THICKNESS_RATIO2*fod->pca.w[0] > fod->pca.w[1])
	{
		return DECTECT_FAILED;
	}

	// Flip eigen vectors if neccecery:
	pointcloud_conditional_basis_flip (fod->pca.e);

	{
		// Rectify pointcloud:
		// Rotate the pointcloud using rotation matrix:
		v3f32 x2[CE30_WH];
		v3f32_subv (x2, x, &(fod->pca.o), 1, 1, 0, CE30_WH);
		pointcloud_rotate ((m3f32 *)fod->pca.e, x2, x1, CE30_WH); // (x1) := e (x2)
	}


	//Reliably detect points above the ground box:
	{
		// Label points within a pointcloud sector (0, a, b)
		//     POINTCLOUD SECTOR
		//  0     a   r    b    n
		//   \====\=======/====/
		//     \===\=====/===/
		//        \=\===/=/
		//          \\=//
		//            0
		//          LIDAR
		int32_t a = MAX (sample_index - DETECT_ARCLENGTH, 0);
		int32_t b = MIN (sample_index + DETECT_ARCLENGTH, (int32_t)CE30_WH);
		int32_t iobj[CE30_WH]; // Indices to points that are above the ground
		uint32_t j = 0; // Number of points above the ground
		for (int32_t i = a; i < b; ++i)
		{
			if ((cid[i] & CE30_POINT_GOOD) == 0) {continue;;}
			// Visual only:
			cid[i] |= CE30_POINT_SECTOR;

			// Check if point is above the ground:
			float x = x1[i].x;
			if ((x*x) > (fod->pca.w[0]*DETECT_MIN_EIGEN_RATIO2))
			{
				// Store point index (i) that is above the ground:
				iobj[j] = i;
				j++;
				// Visual only:
				cid[i] |= CE30_POINT_ABOVE;
			}
		}

		// Check any point above the ground got succefully detected:
		if (j > 0)
		{
			// Simple cluster selection.
			// Randomly selected point is garanteed to belong to only one cluster:
			fod->clusteri = iobj[rand() % j];
			ASSERT (fod->clusteri < (int32_t)CE30_WH);
			ASSERT (fod->clusteri >= a);
			ASSERT (fod->clusteri <= b);
			if (cid[fod->clusteri] & CE30_POINT_GOOD)
			{
				poitracker_update (trackers, x + fod->clusteri, sample_index);
			}
		}

	}


	return DECTECT_SUCCESS;
}





static void detection_input (struct graphics * g, struct poitracker * tracker, struct fodcontext * fod)
{
	ASSERT_PARAM_NOTNULL (g);
	ASSERT_PARAM_NOTNULL (tracker);
	ASSERT_PARAM_NOTNULL (fod);
	v3f32 * x = fod->pc_x1;

	{
		int32_t randomi;
		for (uint32_t i = 0; i < 4; ++i)
		{
			randomi = rand() % CE30_WH;
			detection_sample (tracker, randomi, fod);
			//memset (fod->pc_flags, 0, sizeof(uint8_t)*CE30_WH);
		}
		randomi = rand() % CE30_WH;
		detection_sample (tracker, randomi, fod);

#ifdef ENABLE_GRAPHIC
		//graphics_draw_pointcloud_alpha (g, n, x1, amp);
		graphics_draw_pca (g, fod->pca.e, fod->pca.w, &(fod->pca.o));
		//csc_v3f32_print_rgb(e);
		printf ("roll      %f\n", f32_rad_to_deg(fod->pca.roll));
		printf ("elevation %f\n", f32_rad_to_deg(fod->pca.elevation));
		graphics_draw_obj (g, x + fod->clusteri, 0.1f, (u8rgba){{0xCC, 0xEE, 0xFF, 0xFF}});
		graphics_draw_obj (g, x + randomi, 0.05f, (u8rgba){{0x99, 0x33, 0xFF, 0xAA}});
		graphics_draw_pointcloud_cid (g, CE30_WH, x, fod->pc_flags);
#endif
	}

	/*
	for (uint32_t i = 0; i < TRACKER_CAPACITY; ++i)
	{
		if (tracker->h[i] > TRACKER_MIN_HITS_RESCAN)
		{
			//getchar();
			//printf ("Recheck tracker %i\n", i);
			int32_t randomi = tracker->i[i];
			int32_t spread = (rand() % (TRACKER_RESCAN_RADIUS*2)) - TRACKER_RESCAN_RADIUS;
			randomi = CLAMP(randomi + spread, 0, CE30_WH);
			detection_sample (tracker, randomi, fod);
			//graphics_flush (g);
		}
	}
	*/

	tracker_update2 (tracker);


#ifdef ENABLE_GRAPHIC
	if (g)
	{
		for (uint32_t i = 0; i < TRACKER_CAPACITY; ++i)
		{
			char buf[10];
			//snprintf(buf, 10, "%i:%4.2f", i, tracker->h[i]);
			snprintf(buf, 10, "%i", i);
			graphics_draw_obj (g, tracker->x + i, tracker->r[i], (u8rgba){{0xFF, 0xEE, 0x66, MIN(0xFF * tracker->h[i] * 2.0f, 0xFF)}});
			graphics_draw_text (g, i, tracker->x + i, buf);
		}
	}
#endif



#ifdef ENABLE_GRAPHIC
	for (uint32_t i = 0; i < TRACKER_CAPACITY; ++i)
	{
		if (tracker->r[i] != FLT_MAX)
		{
			graphics_draw_line (g, x + tracker->i[i], tracker->x + i, (u8rgba){{0x44, 0xAA, 0xAA, 0xFF}});
			//graphics_draw_obj (g, x + tracker->i[i], tracker->r[i], (u8rgba){{0x00, 0x00, 0x66, 0xFF}});
		}
	}
	graphics_flush (g);
#endif

}












/*
void ball_process(v3f32 const * x, uint32_t n)
{
	m3f32 c;
	v3f32 y[CE30_WH];
	v3f32 y1[CE30_WH];
	v3f32 e[3];
	float w[3];
	v3f32 o;
	uint32_t m = sampleball(x, n, y, 0.2f); //0.2m radius ball
	pointcloud_centering (y, y, m, 1.0f, &o);
	pointcloud_covariance (y, m, &c, 1.0f);
	pointcloud_eigen (&c, e, w);
	pointcloud_conditional_basis_flip (e);
	pointcloud_reorder_eigen (e, w);
	memcpy (y1, y, sizeof (v3f32) * CE30_WH);
	pointcloud_rotate ((m3f32 *)e, y1, y, m);
}
*/
