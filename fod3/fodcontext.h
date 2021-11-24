#pragma once

#include "csc/csc_math.h"
#include "../shared/ce30.h"
#include "probe/probe.h"
#include "types.h"







static struct fodcontext * fodcontext_create()
{
	struct fodcontext * fodctx = calloc (1, sizeof (struct fodcontext));
	return fodctx;
}

static void fodcontext_input(struct fodcontext * fod, v4f32 xyzw[CE30_WH])
{
	ce30_xyzw_to_pos_amp_flags (xyzw, fod->x1, fod->a1, fod->tags);
	ce30_detect_incidence_edges (fod->tags);
	probe_fodcontext (fod);
	probe_flush();
}

