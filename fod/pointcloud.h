#pragma once

#include <stdint.h>
#include "csc/csc_debug.h"
#include "csc/csc_m3f32.h"
#include "csc/csc_m3f32_print.h"
#include "csc/csc_v3f32.h"
#include "csc/csc_v3f32_print.h"
#include "csc/csc_v4f32.h"

#include "../shared/shared.h"
#include "../shared/log.h"
#include "mathmisc.h"

static void pointcloud_centering (struct v4f32 const x[], struct v4f32 y[], uint32_t n, float k, struct v3f32 * centroid)
{
	ASSERT (n > 0); //Divide by zero protection
	//Move the center of all points to origin:
	struct v3f32 mean = {0.0f, 0.0f, 0.0f};
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
	centroid->x = centroid->x * (1.0f - k) + k * mean.x;
	centroid->y = centroid->y * (1.0f - k) + k * mean.y;
	centroid->z = centroid->z * (1.0f - k) + k * mean.z;
	centroid->y = 0.0f;
	for (uint32_t i = 0; i < n; ++i)
	{
		y[i].x = x[i].x - centroid->x;
		y[i].y = x[i].y - centroid->y;
		y[i].z = x[i].z - centroid->z;
		y[i].w = x[i].w;
	}
}


static void pointcloud_covariance (struct v4f32 const x[], uint32_t n, struct m3f32 * c, float k)
{
	uint32_t dim = 3; //Number of dimensions in point
	uint32_t ldx = 4; //Number of floats per point
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
static void pointcloud_eigen (struct m3f32 const * c, struct v3f32 e[3], struct v3f32 * w)
{
	uint32_t dim = 3; //Number of dimensions in point
	//https://software.intel.com/sites/products/documentation/doclib/mkl_sa/11/mkl_lapack_examples/dsyev.htm
	memcpy (e, c, sizeof (float)*3*3);
	LAPACKE_ssyev (LAPACK_COL_MAJOR, 'V', 'U', dim, (float*)e, dim, (float*)w);
}



static void pointcloud_conditional_basis_flip (struct v3f32 e[3])
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



//Rotate the pointcloud
static void pointcloud_rotate (struct m3f32 const * r, struct v4f32 const x[], struct v4f32 y[], uint32_t n)
{
	uint32_t dim = 3; //Number of dimensions in point
	uint32_t ldx = 4; //Number of floats per point
	cblas_sgemm (CblasColMajor, CblasTrans, CblasNoTrans, dim, n, dim, 1.0f, (float const *)r, dim, (float const*)x, ldx, 0.0f, (float*)y, ldx);
}






struct pointcloud
{
	uint32_t framenr;
	uint32_t n;
	struct v4f32 * x; //n number of points in pointcloud (x,y,z,a),(x,y,z,a)
	struct v4f32 * x1; //n number of points in pointcloud (x,y,z,a),(x,y,z,a)
	struct v4f32 * x2; //n number of points in pointcloud (x,y,z,a),(x,y,z,a)
	struct m3f32 c; //Coveriance matrix
	struct v3f32 e[3]; //Eigen column vectors
	struct v3f32 w; //Eigen values
	struct v3f32 o; //Centroid
	float thickness;
};

static void pointcloud_allocate (struct pointcloud * pc)
{
	pc->x = calloc(1, pc->n * sizeof (float) * POINT_STRIDE);
	pc->x1 = calloc(1, pc->n * sizeof (float) * POINT_STRIDE);
	pc->x2 = calloc(1, pc->n * sizeof (float) * POINT_STRIDE);
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
	struct v4f32 * x = pc->x;
	uint32_t j = 0;
	for (uint32_t i = 0; i < pc->n; ++i)
	{
		struct v3f32 xi = {x[i].x, x[i].y, x[i].z};
		if (v3f32_norm2 (&xi) > d2)
		{
			x[j] = x[i];
			j++;
		}
	}
	pc->n = j;
}



static void pointcloud_process (struct pointcloud * pc)
{
	pointcloud_filter1 (pc, 1.0f);
	pointcloud_centering (pc->x, pc->x1, pc->n, 0.1f, &pc->o);
	pointcloud_covariance (pc->x1, pc->n, &pc->c, 1.0f);
	pointcloud_eigen (&pc->c, pc->e, &pc->w);
	memcpy(pc->x2, pc->x1, sizeof (struct v4f32) * LIDAR_WH);
	//pointcloud_rotate ((struct m3f32 *)pc->e, pc->x2, pc->x1, pc->n);
}
