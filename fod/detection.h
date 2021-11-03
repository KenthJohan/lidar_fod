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

#include "components.h"
#include "flecs.h"
#include "misc.h"
#include "tracker.h"
#include "fodcontext.h"
#include "probe/probe.h"

#define DECTECT_FAILED 0
#define DECTECT_SUCCESS 1

static uint32_t detection_sample (ecs_world_t *world, struct fodcontext * fod)
{
	if ((fod->pc_tags[fod->sample_index] & CE30_POINT_GOOD) == 0)
	{
		return DECTECT_FAILED;
	}





	{
		uint8_t     * tags1 = fod->pc_tags;
		v3f32 const *    x1 = fod->pc_x1; // Input pointcloud
		v3f32       *    x2 = fod->pc_xtemp1; // Temporary pointcloud
		v3f32 const *     s = x1 + fod->sample_index; // Select sample coordinate
		// Copy (n) amount of points (x) at position (s) within sphere to (x1):
		// Where (m) is number of point in ball:
		// Returns Nnmber of point in ball:
		uint32_t m = select_pca_points (x1, CE30_WH, s, x2, DETECT_BALL_RADIUS);
		// Currently this is only good for visuals:
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

	/*
	{
		//uint8_t     * tags1 = fod->pc_tags;
		v3f32 const *    x1 = fod->pc_x1; // Input pointcloud
		v3f32       *    x2 = fod->pc_x2; // Temporary pointcloud
		v3f32 const *     s = x1 + sample_index; // Select sample coordinate
		float r = DETECT_BALL_RADIUS;
		for (int i = 0; i < 1; ++i)
		{
			r += 0.2f;
			uint32_t m = select_pca_points (x1, CE30_WH, s, x2, r);
			calculate_pca (&fod->pca_ground, x2, m, 1.0f);
			{
				// Rectify pointcloud (x1):
				// Rotate the pointcloud using rotation matrix triple eigen column vectors:
				v3f32       * xt = fod->pc_xtemp; // Temporary pointcloud
				v3f32       *  e = fod->pca_ground.e; // rotation matrix, triple eigen column vectors
				v3f32 const *  o = &fod->pca_ground.o; // PCA centroid, offset from origin
				pointcloud_conditional_basis_flip (e); // Flip eigen vectors if neccecery:
				v3f32_subv (xt, x1, o, 1, 1, 0, CE30_WH);
				pointcloud_rotate ((m3f32 *)e, xt, x2, CE30_WH);
			}
		}
	}
	*/



	{
		v3f32_normalize (&(fod->sample_normal));
		float k = 0.1f;
		v3f32_lerp2 (&(fod->sample_normal), &(fod->sample_normal), fod->pca_sample.e + 0, k);
		fod->sample_delta = v3f32_dot (&(fod->sample_normal), fod->pca_sample.e + 0);
		fod->sample_mean_variance = f32_lerp2 (fod->sample_mean_variance, fod->sample_delta, 0.5f);
		fod->sample_mean_roll = ce30_roll (&(fod->sample_normal));
		fod->sample_mean_elevation = ce30_elevation (&(fod->sample_normal));
	}




	// Reliably detect points above the ground box:
	{
		v3f32 const *    x1 = fod->pc_x1;
		v3f32       *    x2 = fod->pc_xtemp1;
		uint8_t     * tags1 = fod->pc_tags;
		{
			// Rectify pointcloud (x1):
			// Rotate the pointcloud using rotation matrix triple eigen column vectors:
			v3f32       * xt = fod->pc_xtemp2; // Temporary pointcloud
			v3f32       *  e = fod->pca_sample.e; // rotation matrix, triple eigen column vectors
			v3f32 const *  o = &fod->pca_sample.o; // PCA centroid, offset from origin
			pointcloud_conditional_basis_flip (e); // Flip eigen vectors if neccecery:
			v3f32_subv (xt, x1, o, 1, 1, 0, CE30_WH);
			pointcloud_rotate ((m3f32 *)e, xt, x2, CE30_WH);
		}
		// Label points within a pointcloud sector (0, a, b)
		//     POINTCLOUD SECTOR
		//  0     a   r    b    n
		//   \====\=======/====/
		//     \===\=====/===/
		//        \=\===/=/
		//          \\=//
		//            0
		//          LIDAR
		int32_t a = MAX (fod->sample_index - DETECT_ARCLENGTH, 0);
		int32_t b = MIN (fod->sample_index + DETECT_ARCLENGTH, (int32_t)CE30_WH);
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
					int criteria2 = fabs (fod->sample_mean_elevation - ce30_elevation(fod->pca_cluster.e + 0)) < 10.0f;
					int criteria3 = fabs (fod->sample_mean_roll - ce30_roll(fod->pca_cluster.e + 0)) < 10.0f;
					printf ("IsGround(%i,%i,%i): (%f*%f) < %f, roll=%f, elevation=%f\n", criteria1, criteria2, criteria3, DETECT_MIN_GROUND_THICKNESS_RATIO2, fod->pca_cluster.w[0], fod->pca_cluster.w[1], f32_rad_to_deg(fod->pca_cluster.roll), f32_rad_to_deg(fod->pca_cluster.elevation));
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
				poitracker_update (&fod->tracker, (x1 + fod->pc_index_cluster), fod->sample_index);
			}
		}

	}


	return DECTECT_SUCCESS;
}





static void detection_input (ecs_world_t *world, struct fodcontext * fod)
{
	ASSERT_PARAM_NOTNULL (world);
	ASSERT_PARAM_NOTNULL (fod);

	v3f32 * x = fod->pc_x1;
	memset (&fod->pca_sample, 0, sizeof (struct fodpca));
	memset (&fod->pca_cluster, 0, sizeof (struct fodpca));
	fod->flags = 0;

	// Proximity detection:
	{
		uint32_t c = 0;
		for (uint32_t s = 0; s < PROX_SECTOR_COUNT; ++s)
		{
			uint32_t n = 0;
			uint32_t r = 0;
			for (r = 0; r < (CE30_WH / PROX_SECTOR_COUNT)-1; ++r)
			{
				uint32_t i = MIN (c + r, CE30_WH-1);
				if (CE30_POINT_IS_UNDEFINED(x[i])) {continue;}
				float l2 = v3f32_norm2 (x + i);
				float lenght_min = 1.75f; // Meter
				if (l2 < (lenght_min*lenght_min))
				{
					n++;
				}
			}
			//printf ("n: %i %i %i\n", c, c+r, n);
			fod->proximity[s] = n;
			c += CE30_WH / PROX_SECTOR_COUNT;
		}

		for (uint32_t s = 0; s < PROX_SECTOR_COUNT; ++s)
		{
			uint32_t const * p = fod->proximity;
			if (p[s] > 10)
			{
				fod->flags |= FOD_PROXIMITY_ALERT;
			}
		}
	}


	// Object detection by random sampling:
	if ((fod->flags & FOD_PROXIMITY_ALERT) != FOD_PROXIMITY_ALERT)
	{
		fod->sample_index = rand() % CE30_WH;
		detection_sample (world, fod);
	}
	probe_fodcontext (fod, x);



	// Object detection by smart sampling:
	if ((fod->flags & FOD_PROXIMITY_ALERT) != FOD_PROXIMITY_ALERT)
	{
		for (uint32_t i = 0; i < TRACKER_CAPACITY; ++i)
		{
			if (fod->tracker.h[i] > TRACKER_MIN_HITS_RESCAN)
			{
				//getchar();
				//printf ("Recheck tracker %i\n", i);
				int32_t spread = (rand() % (TRACKER_RESCAN_RADIUS*2)) - TRACKER_RESCAN_RADIUS;
				fod->sample_index = CLAMP(fod->tracker.i[i] + spread, 0, CE30_WH);
				detection_sample (world, fod);
				//graphics_flush (g);
			}
		}
		tracker_update2 (&fod->tracker);
		probe_tracker (&fod->tracker, x);
	}

	probe_flush ();
}







































