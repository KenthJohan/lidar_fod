#pragma once

#include "csc/csc_math.h"
#include "../shared/ce30.h"
#include "probe/probe.h"
#include "types.h"
#include "csc/csc_vf32_convolution.h"


// # Robust PCA per frame
// x1: Pointcloud source
// x2: Pointcloud rectified
static void calculate_RPCA(v3f32 x[], uint8_t tags[], uint32_t xn, v3f32 y[], uint32_t n, struct fodpca * pca)
{
	// Robust PCA iterations:
	n = 4;
	// Changing constants for each iteration:
	// lv: Positive inliers threshold.
	// ln: Negative inliers threshold.
	// kv: Low-pass single-pole IIR filter: coveriance(c).
	// ov: Low-pass single-pole IIR filter: centroid, offset(o).
	float lp[4] = {+FLT_MAX,   +0.01f, +0.03f, +0.09f};
	float ln[4] = {-FLT_MAX, -FLT_MAX,  -0.1f, -0.09f};
	float kv[4] = {    0.1f,     1.0f,   1.0f,   1.0f};
	float ov[4] = {    0.1f,     0.5f,   1.0f,   1.0f};

	// Robust PCA calculation:
	for (uint32_t k = 0; k < n; ++k)
	{
		uint32_t yn = 0;
		for (uint32_t i = 0; i < xn; ++i)
		{
			// New iteration, reset tagged point:
			tags[i] &= ~CE30_POINT_GROUND;
			if ((tags[i] & CE30_POINT_GOOD) == 0){continue;}
			// Get point height above plane:
			float h = y[i].x;
			// Ignore outliers:
			if(h > lp[k]) {continue;}
			if(h < ln[k]) {continue;}
			// Store inliers i.e. points which approximately can be fitted to the plane:
			y[yn] = x[i];
			yn++;
			// Tag point as ground:
			tags[i] |= CE30_POINT_GROUND;
		}

		// Calculate PCA model of stored inliers which will be the ground model:
		calculate_pca (pca, y, yn, kv[k], ov[k]);

		// Flip eigen vectors if neccecery:
		pointcloud_conditional_basis_flip (pca->e);

		// Use input source pointcloud(x) to make a rectified pointcloud(y)
		// using ground model PCA centroid(pca->o) and rotation matrix(pca->e):
		v3f32_subv (x, x, &pca->o, 1, 1, 0, xn);
		// This is matrix matrix multiplication:
		// y          := (pca->e) * y
		// pointcloud := rotation * pointcloud
		pointcloud_rotate ((m3f32 *)pca->e, x, y, xn);
		// Undo modification of input source:
		v3f32_addv (x, x, &pca->o, 1, 1, 0, xn);

		/*
		printf("n=%i, l=(%f,%f) w0=%f\n", yn, lp[k], ln[k], pca->w[0]);
		probe_pointcloud (x, tags, xn);
		probe_pca(pca);
		probe_flush();
		getchar();
		*/
	}
}


static float conv3(float x[], uint32_t u, uint32_t v, float k[])
{

	float f[9];
	f[0] = x[CE30_XY_INDEX (u-1, v-1)];
	f[1] = x[CE30_XY_INDEX (u+0, v-1)];
	f[2] = x[CE30_XY_INDEX (u+1, v-1)];
	f[3] = x[CE30_XY_INDEX (u-1,  v+0)];
	f[4] = x[CE30_XY_INDEX (u+0,  v+0)];//Middle
	f[5] = x[CE30_XY_INDEX (u+1,  v+0)];
	f[6] = x[CE30_XY_INDEX (u-1, v+1)];
	f[7] = x[CE30_XY_INDEX (u+0, v+1)];
	f[8] = x[CE30_XY_INDEX (u+1, v+1)];
	return vf32_dot(9, k, f);
}


// # Robust PCA over history
// x1: Pointcloud source
// x2: Pointcloud rectified
static void calculate_RPCA2
(v3f32 x1[], v3f32 x2[], uint8_t tags[], uint32_t xn, struct fodpca * pca, float heightmap[])
{
	// ## Reset pointcloud rectified
	memset (x2, 0, sizeof(v3f32)*xn);

	// ## Use already existing ground model to estimate a new ground model
	struct fodpca pca1 = *pca;
	calculate_RPCA (x1, tags, xn, x2, 1, &pca1);

	// ## Update ground model
	pca->e[0] = pca1.e[0];
	pca->e[1] = pca1.e[1];
	pca->e[2] = pca1.e[2];
	pca->w[0] = pca1.w[0];
	pca->w[1] = pca1.w[1];
	pca->w[2] = pca1.w[2];
	{
		// Low-pass single-pole IIR filter of coveriance(c) and centroid(o):
		float k = 0.01f;
		v3f32_add_mul (&(pca->o), &(pca->o), &(pca1.o), k, 1.0f - k);
		m3f32_add_mul (&(pca->c), &(pca->c), &(pca1.c), k, 1.0f - k);
	}

	for (uint32_t i = 0; i < xn; ++i)
	{
		if ((tags[i] & CE30_POINT_GOOD) == 0)
		{
			//x2[i].x = -1.0f;
			x2[i].x = 0.0f;
		}
		heightmap[i] = x2[i].x;
	}
}


static void thres (float heightmap[], float calib[], uint8_t tags[], uint32_t xn, float kernel[3*3], float w)
{
	// ## LIDAR might be distorted.
	// A plane can no be fitted perfectly to a convex or concave ground.
	// This builds a mean ground using lowpass filter
	// which can be used to strighten out the ground height:
	for (uint32_t i = 0; i < xn; ++i)
	{
		// Find ground points:
		if(fabs(heightmap[i]) < w*6.0f)
		{
			float k = 0.1f;
			// Low-pass single-pole IIR filter:
			calib[i] = (k * heightmap[i]) + ((1.0f - k) * calib[i]);
			calib[i] = CLAMP(calib[i], -0.4f, 0.4f);
			heightmap[i] = heightmap[i] - calib[i];
		}
	}


	for (uint32_t u = 1; u < CE30_W-1; ++u)
	{
		for (uint32_t v = 1; v < CE30_H-1; ++v)
		{
			uint32_t i = CE30_XY_INDEX(u, v);
			float h = conv3 (heightmap, u, v, kernel);
			tags[i] &= ~CE30_POINT_ABOVE;
			if (h > (w*2.0f))
			{
				tags[i] |= CE30_POINT_ABOVE;
				calib[i] = 0.0f;
			}
		}
	}
}




static uint32_t number_of_tag (uint8_t tags[], uint32_t n, uint8_t filter)
{
	uint32_t sum = 0;
	for (uint32_t i = 0; i < n; ++i)
	{
		if ((filter == 0) && ((tags[i] & CE30_POINT_GOOD) == 0))
		{
			sum++;
		}
		else if (tags[i] & filter)
		{
			sum++;
		}
	}
	return sum;
}





static char testc = 0;
static void findobj (uint8_t tags[], v3f32 x[], uint32_t xn, v3f32 y[], uint32_t yn)
{
	uint32_t k = 0;
	for (uint32_t i = 0; i < xn; ++i)
	{
		if ((tags[i] & CE30_POINT_ABOVE) == 0){continue;}
		for(uint32_t j = 0; j < yn; ++j)
		{
			if (y[j].x == 0.0f)
			{
				k = j;
				continue;
			}
			v3f32 d;
			v3f32_sub (&d, x+i, y+j);
			float l2 = v3f32_norm2 (&d);
			float t = 0.4f;
			if (l2 < (t*t))
			{
				k = j;
				break;
			}
		}

		//TODO: lerp this
		y[k] = x[i];

		//TODO: Frame 1570 tracker bug
		if(testc == 't')
		{
			probe_obj (y + 0, PROBE_OBJ);
			probe_obj (y + 1, PROBE_OBJ);
			probe_obj (y + 2, PROBE_OBJ);
			probe_obj (y + 3, PROBE_OBJ);
			probe_obj (y + k, PROBE_OBJ1);
			probe_pointcloud (x, tags, xn);
			probe_flush();
			getchar();
		}
		/*
		*/
	}
}





















static struct fodcontext * fodcontext_create()
{
	struct fodcontext * fodctx = calloc (1, sizeof (struct fodcontext));

	float kernel[9] =
	{
	0.5f, 0.5f, 0.5f,
	0.5f, 1.0f, 0.5f,
	0.5f, 0.5f, 0.5f
	};
	vsf32_mul (9, kernel, kernel, 1.0f/vf32_sum(9, kernel));
	memcpy(fodctx->kernel, kernel, sizeof(kernel));

	return fodctx;
}


static void fodcontext_input (struct fodcontext * fod, v4f32 xyzw[CE30_WH])
{
	// Covert pointcloud to points and bightness-amplitudes, point-tags:
	memset(fod->tags, 0, sizeof(uint8_t)*CE30_WH);
	memset(fod->tracker.x, 0, sizeof(v3f32)*TRACKER_CAPACITY);

	ce30_xyzw_to_pos_amp_flags (xyzw, fod->x1, fod->a1, fod->tags);
	ce30_detect_incidence_edges (fod->tags);

	// Robust PCA over history:
	calculate_RPCA2 (fod->x1, fod->x2, fod->tags, CE30_WH, &(fod->ground_pca), fod->h);

	// ## Ground thickness
	// Noise level in the direction of ground normal:
	float w = sqrtf (fod->ground_pca.w[0]);
	thres (fod->h, fod->calib, fod->tags, CE30_WH, fod->kernel, w);

	findobj (fod->tags, fod->x1, CE30_WH, fod->tracker.x, TRACKER_CAPACITY);




	fod->num_above = number_of_tag(fod->tags, CE30_WH, CE30_POINT_ABOVE);
	fod->num_above_tot += fod->num_above;

	printf("Reflected: %u\n", number_of_tag(fod->tags, CE30_WH, CE30_POINT_GOOD));
	printf("HoleEdges: %u\n", number_of_tag(fod->tags, CE30_WH, CE30_POINT_EDGE));
	printf("Undefined: %u\n", number_of_tag(fod->tags, CE30_WH, 0));
	printf("Above:     %u, tot=%u\n", fod->num_above, fod->num_above_tot);
	printf("Ground:    %u\n", number_of_tag(fod->tags, CE30_WH, CE30_POINT_GROUND));

	// Probe result show graphics:
	//probe_pointcloud_alpha (fod->x2, fod->calib, CE30_WH, 10000.0f);

	probe_obj (fod->tracker.x + 0, PROBE_OBJ);
	probe_obj (fod->tracker.x + 1, PROBE_OBJ);
	probe_obj (fod->tracker.x + 2, PROBE_OBJ);
	probe_obj (fod->tracker.x + 3, PROBE_OBJ);
	probe_pointcloud_pn (fod->x2, fod->calib, CE30_WH, 10000.0f);
	probe_pointcloud (fod->x1, fod->tags, CE30_WH);
	probe_pca(&fod->ground_pca);
	probe_flush();
	testc = getchar();
}

