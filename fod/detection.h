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
#include "pcfod.h"


#define DECTECT_FAILED 0
#define DECTECT_SUCCESS 1










static uint32_t detection_sample
(struct poitracker * trackers, int32_t sample_index, struct fodcontext * fod)
{
	uint8_t     * tags1 = fod->pc_tags;
	v3f32 const *    x1 = fod->pc_x1;
	v3f32       *    x2 = fod->pc_x2;


	if ((tags1[sample_index] & CE30_POINT_GOOD) == 0)
	{
		return DECTECT_FAILED;
	}



	{
		v3f32 const * s = x1 + sample_index; // Select sample coordinate
		// Copy (n) amount of points (x) at position (s) within sphere to (x1):
		// Where (m) is number of point in ball:
		// Returns Nnmber of point in ball:
		uint32_t m = select_pca_points (x1, CE30_WH, s, x2, DETECT_BALL_RADIUS);
		// Currently this is only good fod visualds:
		for (uint32_t i = 0; i < CE30_WH; ++i)
		{
			v3f32 d;
			v3f32_sub (&d, x1 + i, s);
			if (v3f32_norm2 (&d) < DETECT_BALL_RADIUS2)
			{
				// Tag this point as part of ball:
				tags1[i] |= CE30_POINT_SEARCH;
			}
		}
		// Enough points must be within the sphere to make a good detection:
		if (m < MIN_POINTS_IN_BALL)
		{
			return DECTECT_FAILED;
		}
		calculate_pca (&fod->pca_sample, x2, m, 1.0f);
	}




	// Check if the ground box is thin enough to reliably detect points above the ground box:
	// These are the ground box dimensions:
	// w[0] = Shortest length m^2, w[1] = Medium length m^2, w[2] = Farthest length m^2.
	// Note that the ground box dimensions is expressed as eigen values (w[0], w[1], w[2]) which is in square meter (m^2).
	// To get the actual thickness of the box do sqrtf(w[0]).
	// w[1] must be k times larger than w[0] to pass this test:
	if (DETECT_MIN_GROUND_THICKNESS_RATIO2*fod->pca_sample.w[0] > fod->pca_sample.w[1])
	{
		//printf ("DECTECT_FAILED DETECT_MIN_GROUND_THICKNESS_RATIO2\n");
		return DECTECT_FAILED;
	}

	{
		// Calculate moving average of ground slope:
		fod->avg_roll = f32_lerp2 (fod->avg_roll, fod->pca_sample.roll, 0.5f);
		fod->avg_elevation = f32_lerp2 (fod->avg_elevation, fod->pca_sample.elevation, 0.5f);

	}

	{
		// Rectify pointcloud (x1):
		// Rotate the pointcloud using rotation matrix triple eigen column vectors:
		v3f32       * xt = fod->pc_xtemp; // Temporary pointcloud
		v3f32       *  e = fod->pca_sample.e; // rotation matrix, triple eigen column vectors
		v3f32 const *  o = &fod->pca_sample.o; // PCA centroid, offset from origin
		pointcloud_conditional_basis_flip (e); // Flip eigen vectors if neccecery:
		v3f32_subv (xt, x1, o, 1, 1, 0, CE30_WH);
		pointcloud_rotate ((m3f32 *)e, xt, x2, CE30_WH);
	}


	// Reliably detect points above the ground box:
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
			if ((tags1[i] & CE30_POINT_GOOD) == 0) {continue;}
			tags1[i] |= CE30_POINT_SECTOR;
			float x = x2[i].x;
			if (x < 0) {continue;} // Ignore points under the ground
			// Check if point higher than the noise i.e. eigen value:
			if ((x*x) < (fod->pca_sample.w[0]*DETECT_MIN_EIGEN_RATIO2)) {continue;}
			iobj[j] = i; // Store point index (i) that is above the ground:
			j++;
			tags1[i] |= CE30_POINT_ABOVE;
		}




		// Check any point above the ground got succefully detected:
		if (j > 0)
		{
			// Simple cluster selection.
			// Randomly selected point is garanteed to belong to only one cluster:
			fod->pc_index_cluster = iobj[rand() % j];

			{
				v3f32 xcluster[CE30_WH];
				uint32_t k = select_pca_points (x1, CE30_WH, (x1 + fod->pc_index_cluster), xcluster, 0.2f);
				if (k > 10)
				{
					calculate_pca (&(fod->pca_cluster), xcluster, k, 1.0f);
					// w[0] must be k times smaller than w[0] to pass this test:
					int criteria1 = DETECT_MIN_GROUND_THICKNESS_RATIO2*fod->pca_cluster.w[0] < fod->pca_cluster.w[1];
					int criteria2 = fabs (fod->avg_elevation - fod->pca_cluster.elevation) < 10.0f;
					int criteria3 = fabs (fod->avg_roll - fod->pca_cluster.roll) < 10.0f;
					printf ("DETECT(%i,%i,%i): (%f*%f) < %f, roll=%f, elevation=%f\n", criteria1, criteria2, criteria3, DETECT_MIN_GROUND_THICKNESS_RATIO2, fod->pca_cluster.w[0], fod->pca_cluster.w[1], f32_rad_to_deg(fod->pca_cluster.roll), f32_rad_to_deg(fod->pca_cluster.elevation));
					if (criteria1 && criteria2 && criteria3)
					{
						//printf ("DECTECT_FAILED: (%f*%f) < %f, roll=%f, elevation=%f\n", DETECT_MIN_GROUND_THICKNESS_RATIO2, fod->pca1.w[0], fod->pca1.w[1], f32_rad_to_deg(fod->pca1.roll), f32_rad_to_deg(fod->pca1.elevation));
						//printf ("roll      %f\n", f32_rad_to_deg(fod->pca.roll));
						//printf ("elevation %f\n", f32_rad_to_deg(fod->pca.elevation));
						return DECTECT_FAILED;
					}
				}
			}


			ASSERT (fod->pc_index_cluster < (int32_t)CE30_WH);
			ASSERT (fod->pc_index_cluster >= a);
			ASSERT (fod->pc_index_cluster <= b);
			if (tags1[fod->pc_index_cluster] & CE30_POINT_GOOD)
			{
				poitracker_update (trackers, (x1 + fod->pc_index_cluster), sample_index);
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
			//randomi = rand() % CE30_WH;
			//detection_sample (tracker, randomi, fod);
			//memset (fod->pc_flags, 0, sizeof(uint8_t)*CE30_WH);
		}
		randomi = rand() % CE30_WH;
		detection_sample (tracker, randomi, fod);

#ifdef ENABLE_GRAPHIC
		//graphics_draw_pointcloud_alpha (g, n, x1, amp);
		graphics_draw_pca (g, fod->pca_sample.e, fod->pca_sample.w, &(fod->pca_sample.o));
		//csc_v3f32_print_rgb(e);
		printf ("roll      %f\n", f32_rad_to_deg(fod->avg_roll));
		printf ("elevation %f\n", f32_rad_to_deg(fod->avg_elevation));
		printf ("w %f %f %f\n", fod->pca_sample.w[0], fod->pca_sample.w[1], fod->pca_sample.w[2]);
		printf ("w %f %f %f\n", fod->pca_cluster.w[0], fod->pca_cluster.w[1], fod->pca_cluster.w[2]);

		graphics_draw_pca (g, fod->pca_cluster.e, fod->pca_cluster.w, &(fod->pca_cluster.o));
		//graphics_draw_obj (g, x + fod->clusteri, 0.1f, (u8rgba){{0xCC, 0xEE, 0xFF, 0xFF}});
		graphics_draw_obj (g, x + randomi, 0.05f, (u8rgba){{0x99, 0x33, 0xFF, 0xAA}});
		graphics_draw_pointcloud_cid (g, CE30_WH, x, fod->pc_tags);
#endif
	}

	// Smart sampling:

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
			//graphics_draw_text (g, i, tracker->x + i, buf);
		}
	}
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
