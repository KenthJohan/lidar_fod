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

#include "csc_math.h"
#include "csc_v3f32.h"
#include "csc_v4f32.h"
#include "csc_m3f32.h"
#include "../shared/shared.h"
#include "../shared/ce30.h"






struct mballs
{
	v3f32 * x;
	uint32_t n;
	uint32_t * m;
	uint32_t cap;
};



void balls_init (struct mballs * balls)
{
	uint32_t n = balls->n;
	uint32_t cap = balls->cap;
	balls->x = calloc (1, sizeof (v3f32) * n * cap);
}

























