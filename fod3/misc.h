#pragma once


#ifdef __MINGW32__
//pacman -S mingw64/mingw-w64-x86_64-openblas
//-lopenblas
#include <OpenBLAS/lapack.h>
#include <OpenBLAS/lapacke.h>
#include <OpenBLAS/cblas.h>
#else
//sudo apt-get install libblas-dev
//sudo apt-get install libopenblas-dev
//sudo apt-get install liblapacke-dev
#include <lapacke.h>
#include <cblas.h>
#endif

#include <stdint.h>

#include "csc/csc_math.h"
#include "csc/csc_v3f32.h"
#include "csc/csc_v4f32.h"
#include "csc/csc_m3f32.h"
#include "../shared/ce30.h"
#include "types.h"





// Unused:
static void pointcloud_centering (v3f32 const x[], v3f32 y[], uint32_t n, float k, v3f32 * centroid)
{
	ASSERT_NOTNULL (x);
	ASSERT_NOTNULL (y);
	ASSERT_NOTNULL (centroid);
	ASSERT_GTEF (k, 0.0f);
	ASSERT_LTEF (k, 1.0f);
	ASSERT_GTU (n, 0);
	// Move the center of all points to origin:
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
	ASSERT_NOTNULL (x);
	ASSERT_NOTNULL (c);
	ASSERT_GTU (n, 1);
	ASSERT_GTEF (k, 0.0f);
	ASSERT_LTEF (k, 1.0f);
	uint32_t dim = 3; // Number of dimensions in point
	uint32_t ldx = 3; // Number of floats per point
	float alpha = (1.0f / ((float)n - 1.0f)) * k;
	float beta = (1.0f - k);
	//https://stattrek.com/matrix-algebra/covariance-matrix.aspx
	//https://software.intel.com/content/www/us/en/develop/articles/sgemm-for-intel-processor-graphics.html
	//matrix(x) := scalar(alpha) * matrix(x) * matrix(x)^T + scalar(beta) * matrix(x)
	//c := c*k + (x*x^t) * (1-k)
	cblas_sgemm (CblasColMajor, CblasNoTrans, CblasTrans, dim, dim, n, alpha, (float const*)x, ldx, (float const*)x, ldx, beta, (float *)c, dim);
}


// Calculate the eigen vectors (c) and eigen values (w) from covariance matrix (c) which will get the orientation of the points:
static void pointcloud_eigen (m3f32 const * c, v3f32 e[3], float w[3])
{
	ASSERT_NOTNULL (c);
	ASSERT_NOTNULL (e);
	ASSERT_NOTNULL (w);
	uint32_t dim = 3; // Number of dimensions in point
	//https://software.intel.com/sites/products/documentation/doclib/mkl_sa/11/mkl_lapack_examples/dsyev.htm
	memcpy (e, c, sizeof (float)*3*3);
	LAPACKE_ssyev (LAPACK_COL_MAJOR, 'V', 'U', dim, (float*)e, dim, (float*)w);
}


static void pointcloud_conditional_basis_flip (v3f32 e[3])
{
	ASSERT_NOTNULL (e);
	// ((0,1,0) dot (c[6], c[7], c[8]) < 0) = (c[7] < 0)
	if (e[2].y < 0.0f)
	{
		// Flip Y vector of pointcloud
		e[1].x *= -1.0f;
		e[1].y *= -1.0f;
		e[1].z *= -1.0f;
		// Flip Z vector of pointcloud
		e[2].x *= -1.0f;
		e[2].y *= -1.0f;
		e[2].z *= -1.0f;
	}
}


static void pointcloud_rotate (m3f32 const * r, v3f32 const x[], v3f32 y[], uint32_t n)
{
	uint32_t dim = 3; // Number of dimensions in point
	uint32_t ldx = 3; // Number of floats per point
	cblas_sgemm (CblasColMajor, CblasTrans, CblasNoTrans, dim, n, dim, 1.0f, (float const *)r, dim, (float const*)x, ldx, 0.0f, (float*)y, ldx);
}







static void calculate_pca (struct fodpca * pca, v3f32 * x1, uint32_t m, float k, float h)
{
	// PCA will produce: orientation (e), centroid (o), variance (w) of subcloud (x1).
	// o : Centroid of subcloud (x1). i.e. subcloud offset (o) from origin (0,0,0).
	// c : 3x3 coveriance matrix of subcloud (x1). Contains orientation and variance.
	// e : Three eigen column vectors (Shortest, Medium, Farthest) of coveriance matrix (c). Contains only orientation.
	// w : Three eigen values (Shortest, Medium, Farthest) of coveriance matrix (c). Contains variance.
	v3f32 * o = &pca->o;
	m3f32 * c = &(pca->c);
	v3f32 * e = pca->e;
	float * w = pca->w;
	v3f32 centroid = V3F32_ZERO;
	v3f32_meanacc (&centroid, x1, m);
	v3f32_add_mul (o, o, &centroid, 1.0f - h, h);
	v3f32_subv (x1, x1, o, 1, 1, 0, m); // x1[i] := x1[i] - o[0], (where i is 0 .. m)
	pointcloud_covariance (x1, m, c, k);
	pointcloud_eigen (c, e, w);
	//https://stackoverflow.com/questions/2782647/how-to-get-yaw-pitch-and-roll-from-a-3d-vector
	//pca->elevation = atan2(e[0].x, e[0].z);
	//pca->roll = asin(-e[0].y);
}


