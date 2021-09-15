#pragma once

#include <stdint.h>
#include "csc/csc_assert.h"
#include "csc/csc_m3f32.h"
#include "csc/csc_m3f32_print.h"
#include "csc/csc_v3f32.h"
#include "csc/csc_xlog.h"
#include "csc/csc_v3f32_print.h"
#include "csc/csc_math.h"

#include "../shared/shared.h"
#include "misc.h"

#define MAXOBJ 10

struct physobjects
{
	uint32_t count;
	v3f32 x[MAXOBJ];//Positions
	v3f32 d[MAXOBJ];//Directions
	m3f32 c[MAXOBJ];//Coveriances
	uint32_t n[MAXOBJ];//Counts
};

void physobjects_print (struct physobjects * obj)
{
	for (uint32_t i = 0; i < obj->count; ++i)
	{
		printf ("%i: N=%i\n", i, obj->n[i]);
	}
}

void physobjects_newobj (struct physobjects * obj, v3f32 x[], uint32_t n)
{
	ASSERT_PARAM_NOTNULL (obj);
	int dim = 3;
	int ldx = 3;
	float alpha = 1.0f;
	float beta = 0.0f;
	m3f32 c;
	cblas_sgemm (CblasColMajor, CblasNoTrans, CblasTrans, dim, dim, n, alpha, (float const*)x, ldx, (float const*)x, ldx, beta, (float*)&c, dim);
}

