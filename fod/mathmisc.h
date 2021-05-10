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
#include "csc/csc_v4f32.h"
#include "csc/csc_m3f32.h"
#include "../shared/shared.h"





static void v4f32_filter_norm2_gt (struct v4f32 x[], uint32_t * n, float d2)
{
	uint32_t j = 0;
	for (uint32_t i = 0; i < (*n); ++i)
	{
		if (v4f32_norm2 (x + i) > d2)
		{
			x[j] = x[i];
			j++;
		}
	}
	(*n) = j;
}





