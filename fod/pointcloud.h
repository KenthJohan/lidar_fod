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

#include "../shared/shared.h"
#include "../shared/ce30.h"

#include "misc.h"
#include "graphics.h"
#include "tracker.h"
#include "flecs.h"




static void pointcloud_centering (v3f32 const x[], v3f32 y[], uint32_t n, float k, v3f32 * centroid)
{
	ASSERT (n > 0); //Divide by zero protection
	//Move the center of all points to origin:
	v3f32 mean = {{0.0f, 0.0f, 0.0f}};
	for (uint32_t i = 0; i < n; ++i)
	{
		mean.x += x[i].x;
		mean.y += x[i].y;
		mean.z += x[i].z;
	}
	float s = 1.0f / n;
	mean.x *= s;
	mean.y *= s;
	mean.z *= s;

	v3f32_add_mul (centroid, centroid, &mean, 1.0f - k, k);
	v3f32_subv (y, x, centroid, 1, 1, 0, n);
}


static void pointcloud_covariance (v3f32 const x[], uint32_t n, m3f32 * c, float k)
{
	uint32_t dim = 3; //Number of dimensions in point
	uint32_t ldx = 3; //Number of floats per point
	ASSERT (n > 1);
	float alpha = (1.0f / ((float)n - 1.0f)) * k;
	float beta = (1.0f - k);
	//https://stattrek.com/matrix-algebra/covariance-matrix.aspx
	//https://software.intel.com/content/www/us/en/develop/articles/sgemm-for-intel-processor-graphics.html
	//matrix(x) := scalar(alpha) * matrix(x) * matrix(x)^T + scalar(beta) * matrix(x)
	//c := c*k + (x*x^t) * (1-k)
	cblas_sgemm (CblasColMajor, CblasNoTrans, CblasTrans, dim, dim, n, alpha, (float const*)x, ldx, (float const*)x, ldx, beta, (float *)c, dim);
}


//Calculate the eigen vectors (c) and eigen values (w) from covariance matrix (c) which will get the orientation of the points:
static void pointcloud_eigen (m3f32 const * c, v3f32 e[3], float w[3])
{
	uint32_t dim = 3; //Number of dimensions in point
	//https://software.intel.com/sites/products/documentation/doclib/mkl_sa/11/mkl_lapack_examples/dsyev.htm
	memcpy (e, c, sizeof (float)*3*3);
	LAPACKE_ssyev (LAPACK_COL_MAJOR, 'V', 'U', dim, (float*)e, dim, (float*)w);
}



static void pointcloud_conditional_basis_flip (v3f32 e[3])
{
	//((0,1,0) dot (c[6], c[7], c[8]) < 0) = (c[7] < 0)
	if (e[2].y < 0.0f)
	{
		//Flip Y vector of pointcloud
		e[1].x *= -1.0f;
		e[1].y *= -1.0f;
		e[1].z *= -1.0f;
		//Flip Z vector of pointcloud
		e[2].x *= -1.0f;
		e[2].y *= -1.0f;
		e[2].z *= -1.0f;
	}
}



//Reorders the eigen column vectors in the matrix
static void pointcloud_reorder_eigen (v3f32 e[3], float w[3])
{
	{
		v3f32 r[3];
		//Eigen column vectors in (c) are sorted by eigen values, shortest vector first:
		r[0] = e[1];//  Medium length PCA basis to x standard basis
		r[1] = e[2];//Farthest length PCA basis to y standard basis
		r[2] = e[0];//Shortest length PCA basis to z standard basis
		memcpy (e, r, sizeof(r));
	}
	{
		float v[3];
		v[0] = w[1];
		v[1] = w[2];
		v[2] = w[0];
		memcpy (w, &v, sizeof(v));
	}
}


static void pointcloud_rotate (m3f32 const * r, v3f32 const x[], v3f32 y[], uint32_t n)
{
	uint32_t dim = 3; //Number of dimensions in point
	uint32_t ldx = 3; //Number of floats per point
	cblas_sgemm (CblasColMajor, CblasTrans, CblasNoTrans, dim, n, dim, 1.0f, (float const *)r, dim, (float const*)x, ldx, 0.0f, (float*)y, ldx);
}





















#define DECTECT_FAILED 0
#define DECTECT_SUCCESS 1





//Maximize n and minimize h

static uint32_t pointcloud_process1 (struct graphics * g, struct poitracker * tracker, uint32_t n, v3f32 x[], uint8_t cid[CE30_WH], int32_t randomi)
{
	float const radius = 0.3f;
	v3f32 const * s = x + randomi;
	v3f32 x1[CE30_WH];
	memset (cid, 0, sizeof(uint8_t)*CE30_WH);

	uint32_t m;
	//int32_t randomi;

	{
		//Sample random point in pointcloud
		//randomi = (rand() * n) / RAND_MAX;
		//v3f32 const * s = x + randomi;
		m = v3f32_ball (x, n, s, x1, radius);
		//XLOG (XLOG_INF, XLOG_GENERAL, "%i", m);
		//Visual only:
		for(uint32_t i = 0; i < n; ++i)
		{
			v3f32 d;
			v3f32_sub (&d, x + i, s);
			if (v3f32_norm2 (&d) < (radius*radius))
			{
				//Tag this point as part of ball:
				cid[i] |= POINTLABEL_SEARCH;
			}
		}
		if (m < MIN_POINTS_IN_BALL)
		{
			return DECTECT_FAILED;
		}
		//XLOG (XLOG_INF, XLOG_GENERAL, "randomi %i", randomi);
	}


	v3f32 o = V3F32_ZERO;
	m3f32 c;
	v3f32 e[3]; //Eigen column vectors (Shortest, Medium, Farthest)
	float w[3]; //Eigen values (Shortest, Medium, Farthest)
	v3f32_meanacc (&o, x1, m);
	v3f32_subv (x1, x1, &o, 1, 1, 0, m);
	pointcloud_covariance (x1, m, &c, 1.0f);
	pointcloud_eigen (&c, e, w);

	// Check if the ground box is thin enough to reliably detect points above the ground box:
	// These are the ground box dimensions:
	// w[0] = Shortest length m^2, w[1] = Medium length m^2, w[2] = Farthest length m^2.
	// Note that the ground box dimensions is expressed as eigen values (w[0], w[1], w[2]) which is in square meter (m^2).
	// To get the actual thickness of the box do sqrtf(w[0]).
	if (DETECT_MIN_GROUND_THICKNESS_RATIO2*w[0] > w[1])
	{
		return DECTECT_FAILED;
	}

	// Prevent eigen vector flip
	pointcloud_conditional_basis_flip (e);

	{
		v3f32 x2[CE30_WH];
		v3f32_subv (x2, x, &o, 1, 1, 0, n);
		pointcloud_rotate ((m3f32 *)e, x2, x1, n); // (x1) := e (x2)
	}

	if (g)
	{
		//graphics_draw_pointcloud_alpha (g, n, x1, amp);
		graphics_draw_pca (g, e, w, &o);
	}


	//XLOG (XLOG_INF, XLOG_GENERAL, "%f", sqrt(w[0]));//0.000985
	//printf ("ball=%i, w=(%f,%f,%f) ratio:%f\n", m, w[0], w[1], w[2], w[1] / w[0]);


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
		int32_t const arclength = 600;
		int32_t a = MAX(randomi - arclength, 0);
		int32_t b = MIN(randomi + arclength, (int32_t)n);
		int32_t iobj[CE30_WH];
		uint32_t j = 0;
		for (int32_t i = a; i < b; ++i)
		{
			cid[i] |= POINTLABEL_SECTOR;
			//Label points that is far above ground
			if (x1[i].x < sqrtf(w[0])*DETECT_MIN_DISTANCE_ABOVE_GROUND) {continue;}
			cid[i] |= POINTLABEL_OBJ;
			iobj[j] = i;
			j++;
		}

		if (j > 0)
		{
			int32_t randomj = rand() % j;
			ASSERT (iobj[randomj] < (int32_t)n);
			ASSERT (iobj[randomj] >= a);
			ASSERT (iobj[randomj] <= b);
			//printf ("%i %i\n", cluster[randomj], randomj);
			//csc_v3f32_print_rgb (stdout, x + cluster[randomj]);
			if (g)
			{
				graphics_draw_obj (g, x + iobj[randomj], 0.1f, (u8rgba){{0xCC, 0xEE, 0xFF, 0xFF}});
			}
			poitracker_update (tracker, x + iobj[randomj], randomi);
		}

	}








	return DECTECT_SUCCESS;
}





static void pointcloud_process (struct graphics * g, struct poitracker * tracker, int32_t n, v3f32 x[], float amp[])
{
	UNUSED (amp);
	if (n < MIN_POINTS_IN_LIDAR)
	{
		return;
	}


	uint8_t cid[CE30_WH];
	int32_t randomi = (rand() % n);
	pointcloud_process1 (g, tracker, n, x, cid, randomi);


	if (g)
	{
		graphics_draw_pointcloud_cid (g, n, x, cid);
		graphics_draw_obj (g, x + randomi, 0.1f, (u8rgba){{0x99, 0x33, 0xFF, 0xFF}});
	}



	for (uint32_t i = 0; i < TRACKER_CAPACITY; ++i)
	{
		if (tracker->h[i] > TRACKER_MIN_HITS_RESCAN)
		{
			//getchar();
			//printf ("Recheck tracker %i\n", i);
			randomi = tracker->i[i];
			int32_t spread = (rand() % (TRACKER_RESCAN_RADIUS*2)) - TRACKER_RESCAN_RADIUS;
			randomi = CLAMP(randomi + spread, 0, n);
			pointcloud_process1 (NULL, tracker, n, x, cid, randomi);
			//graphics_flush (g);
		}
	}

	tracker_update2 (tracker);

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





	for (uint32_t i = 0; i < TRACKER_CAPACITY; ++i)
	{
		if (tracker->r[i] != FLT_MAX)
		{
			graphics_draw_line (g, x + tracker->i[i], tracker->x + i, (u8rgba){{0x44, 0xAA, 0xAA, 0xFF}});
			//graphics_draw_obj (g, x + tracker->i[i], tracker->r[i], (u8rgba){{0x00, 0x00, 0x66, 0xFF}});
		}
	}
	graphics_flush (g);

	/*
	for (uint32_t i = 0; i < TRACKER_CAPACITY; ++i)
	{
		tracker->h[i] += -0.02f;//Reduce intersect hits
		if (tracker->h[i] < 0.0f)
		{
			tracker->h[i] = 0.0f;
			tracker->r[i] = FLT_MAX;
		}
	}
	*/

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







