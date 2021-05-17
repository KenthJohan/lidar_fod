#pragma once

#include <stdint.h>
#include "csc_debug.h"
#include "csc_m3f32.h"
#include "csc_m3f32_print.h"
#include "csc_v3f32.h"
#include "csc_v3f32_print.h"
#include "csc_v4f32.h"

#include "../shared/shared.h"
#include "../shared/log.h"
#include "../shared/ce30.h"
#include "mathmisc.h"

static void pointcloud_centering (v3f32 const x[], v3f32 y[], uint32_t n, float k, v3f32 * centroid)
{
	ASSERT (n > 0); //Divide by zero protection
	//Move the center of all points to origin:
	v3f32 mean = {0.0f, 0.0f, 0.0f};
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
	//centroid->z -= 0.05f;
	for (uint32_t i = 0; i < n; ++i)
	{
		y[i].x = x[i].x - centroid->x;
		y[i].y = x[i].y - centroid->y;
		y[i].z = x[i].z - centroid->z;
	}
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


static uint32_t pointcloud_box_intersect (v4f32 const x[], v4f32 const y[], uint32_t n)
{
	for (uint32_t i = 0; i < n; ++i)
	{
		if (x[i].z)
		{

		}
	}
}









#define POINTCLOUD_MAX_CAPACITY 100000

struct pointcloud
{
	uint32_t framenr;
	uint32_t capacity;
	uint32_t n;
	float * a;
	v3f32 * x; //n number of points in pointcloud (x,y,z),(x,y,z)
	v3f32 * x1; //n number of points in pointcloud (x,y,z),(x,y,z)
	v3f32 * x2; //n number of points in pointcloud (x,y,z),(x,y,z)
	m3f32 c; //Coveriance matrix
	v3f32 e[3]; //Eigen column vectors (Shortest, Medium, Farthest)
	float w[3]; //Eigen values (Shortest, Medium, Farthest)
	v3f32 o; //Centroid
	float h; //Box height
};

static void pointcloud_allocate (struct pointcloud * pc)
{
	printf ("pointcloud_allocate %i\n", pc->capacity);
	ASSERT (pc->capacity < POINTCLOUD_MAX_CAPACITY);
	pc->a = calloc(1, pc->capacity * sizeof (float));
	pc->x = calloc(1, pc->capacity * sizeof (v3f32));
	pc->x1 = calloc(1, pc->capacity * sizeof (v3f32));
	pc->x2 = calloc(1, pc->capacity * sizeof (v3f32));
	pc->h = 1.0f; // 1 meter?
	ASSERT_NOTNULL (pc->a);
	ASSERT_NOTNULL (pc->x);
	ASSERT_NOTNULL (pc->x1);
	ASSERT_NOTNULL (pc->x2);
}



//Maximize n and minimize h

static void pointcloud_process (struct pointcloud * pc)
{
	pointcloud_centering (pc->x, pc->x1, pc->n, 1.0f, &pc->o);
	pointcloud_covariance (pc->x1, pc->n, &pc->c, 1.0f);
	pointcloud_eigen (&pc->c, pc->e, pc->w);
	pointcloud_conditional_basis_flip (pc->e);
	pointcloud_reorder_eigen (pc->e, pc->w);
	memcpy (pc->x2, pc->x1, sizeof (v3f32) * CE30_WH);
	pointcloud_rotate ((m3f32 *)pc->e, pc->x2, pc->x1, pc->n);
	pc->h *= 0.9f;
}















//Copy all points inside the ball
uint32_t sampleball (v3f32 const * x, uint32_t n, v3f32 * y, float r)
{
	uint32_t m = 0;
	v3f32 const * q = x + (rand() * n) / RAND_MAX;
	for(uint32_t i = 0; i < n; ++i)
	{
		v3f32 d;
		v3f32_sub (&d, x + i, q);
		if (v3f32_norm2 (&d) < (r*r))
		{
			y[m] = x[i];
			m++;
		}
	}
	return m;
}




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








