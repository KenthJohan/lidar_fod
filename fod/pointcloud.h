#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <float.h>

#include "csc_debug.h"
#include "csc_f32.h"
#include "csc_m3f32.h"
#include "csc_m3f32_print.h"
#include "csc_v3f32.h"
#include "csc_v3f32_print.h"
#include "csc_v4f32.h"
#include "csc_xlog.h"

#include "../shared/shared.h"
#include "../shared/ce30.h"

#include "misc.h"
#include "graphics.h"




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






/**
 * @brief tracker_update1
 * @param h Intersection hits
 * @param r Radiuses in meter^2
 * @param y Tracker positions
 * @param count
 * @param x Detection coordinate
 */
static void poitracker_update1 (float h[], float r[], v3f32 y[], uint32_t count, v3f32 const * x)
{
	//Check if (x) is inside a existing tracker sphere:
	uint32_t i;
	for (i = 0; i < count; ++i)
	{
		v3f32 d;
		v3f32_sub (&d, x, y + i);
		if (v3f32_norm2 (&d) < r[i])
		{
			h[i] += 0.2f;//Increase intersect hits
			h[i] = MIN (h[i], 1.0f);
			r[i] = 0.3f * 0.3f;
			y[i] = *x;
			break;
		}
	}

	//Merge trackers if their sphere intersects:
	for (uint32_t j = 0; j < count; ++j)
	{
		v3f32 d;
		v3f32_sub (&d, x, y + j);
		float l2 = v3f32_norm2 (&d);
		if (j == i) {continue;}
		if (r[j] == FLT_MAX) {continue;}
		if (l2 > (r[j] + r[i])) {continue;}
		r[j] = FLT_MAX;
		y[j] = (v3f32){{0.0f, 0.0f, 0.0f}};
		XLOG (XLOG_INF, XLOG_GENERAL, "Merging object tracker %i %i", i, j);
	}
}











#define POITRACKER_CAPACITY 5
struct poitracker
{
	uint32_t count;
	float r[POITRACKER_CAPACITY];
	v3f32 x[POITRACKER_CAPACITY];
	float h[POITRACKER_CAPACITY];
};

static void poitracker_init (struct poitracker * tracker)
{
	memset (tracker, 0, sizeof (struct poitracker));
	vf32_set1 (POITRACKER_CAPACITY, tracker->r, FLT_MAX);
	vf32_set1 (POITRACKER_CAPACITY, tracker->h, 0.0f);
}


static void poitracker_update (struct poitracker * tracker, v3f32 * x)
{
	poitracker_update1 (tracker->h, tracker->r, tracker->x, POITRACKER_CAPACITY, x);
}


























//Maximize n and minimize h

static void pointcloud_process (struct graphics * g, struct poitracker * tracker, uint32_t n, v3f32 x[], float a[])
{
	float const radius = 0.3f;

	v3f32 x1[CE30_WH];
	uint8_t cid[CE30_WH];
	memset (cid, 0, sizeof(cid));

	uint32_t m;
	int32_t randomi;

	{
		randomi = (rand() * n) / RAND_MAX;
		v3f32 const * s = x + randomi;
		m = v3f32_ball (x, n, s, x1, radius);
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
	}

	v3f32 o = V3F32_ZERO;
	m3f32 c;
	v3f32 e[3]; //Eigen column vectors (Shortest, Medium, Farthest)
	float w[3]; //Eigen values (Shortest, Medium, Farthest)
	v3f32_meanacc (&o, x1, m);
	v3f32_subv (x1, x1, &o, 1, 1, 0, m);
	pointcloud_covariance (x1, m, &c, 1.0f);
	pointcloud_eigen (&c, e, w);
	//if ((2.0f*w[0] > w[1])){return;}
	pointcloud_conditional_basis_flip (e);

	{
		v3f32 x2[CE30_WH];
		v3f32_subv (x2, x, &o, 1, 1, 0, n);
		pointcloud_rotate ((m3f32 *)e, x2, x1, n); // (x1) := e (x2)
	}

	XLOG (XLOG_INF, XLOG_GENERAL, "%f", sqrt(w[0]));//0.000985

	//Check if PCA is formed by a planar pointcloud:
	//w[0] = Shortest, w[1] = Medium, w[2] = Farthest.
	if ((3.0f*w[0] < w[1]))
	{
		//Classify objects within circle sector at ball location:
		//  0     a   r    b    n
		//   \====\=======/====/
		//     \===\=====/===/
		//        \=\===/=/
		//          \\=//
		//            0
		int32_t const arclength = 600;
		int32_t a = MAX(randomi - arclength, 0);
		int32_t b = MIN(randomi + arclength, (int32_t)n);
		int32_t cluster[CLUSTER_CAPACITY];
		uint32_t j = 0;
		for (int32_t i = a; (i < b) && (j < CLUSTER_CAPACITY); ++i)
		{
			cid[i] |= POINTLABEL_SECTOR;
			//Label points that is far above ground
			if (x1[i].x < sqrtf(w[0])*DISTANCE_ABOVE_GROUND) {continue;}
			cid[i] |= POINTLABEL_OBJ;
			cluster[j] = i;
			j++;
		}

		if (j > 0)
		{
			int32_t randomj = (rand() * j) / RAND_MAX;
			ASSERT (cluster[randomj] < (int32_t)n);
			ASSERT (cluster[randomj] >= a);
			ASSERT (cluster[randomj] <= b);
			//printf ("%i %i\n", cluster[randomj], randomj);
			//csc_v3f32_print_rgb (stdout, x + cluster[randomj]);
			graphics_draw_obj (g, x + cluster[randomj], 0.1f, (u8rgba){{0xCC, 0xEE, 0xFF, 0xFF}});
			poitracker_update (tracker, x + cluster[randomj]);
		}

	}


	for (uint32_t i = 0; i < POITRACKER_CAPACITY; ++i)
	{
		tracker->h[i] += -0.02f;//Reduce intersect hits
		if (tracker->h[i] < 0.0f)
		{
			tracker->h[i] = 0.0f;
			tracker->r[i] = FLT_MAX;
		}
	}

	for (uint32_t i = 0; i < POITRACKER_CAPACITY; ++i)
	{
		graphics_draw_obj (g, tracker->x + i, sqrtf(tracker->r[i]), (u8rgba){{0xFF, 0xEE, 0x66, MIN(0xFF * tracker->h[i] * 2.0f, 0xFF)}});
	}


	graphics_draw_pointcloud_cid (g, n, x, cid);
	graphics_draw_pointcloud_alpha (g, n, x1, a);
	graphics_draw_pca (g, e, w, &o);
	//graphics_draw_obj (g, po->x);
	graphics_flush (g);

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







