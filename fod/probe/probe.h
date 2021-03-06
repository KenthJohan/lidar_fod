#pragma once

#include "fodcontext.h"
#include "tracker.h"



void probe_init (char const * address);
void probe_fodcontext (struct fodcontext * fod, v3f32 x[CE30_WH]);
void probe_tracker (struct poitracker tracker[], v3f32 x[CE30_WH]);
void probe_flush ();
void probe_quit();


#ifndef IMPLEMENT_PROBE
void probe_init (char const * address){}
void probe_fodcontext (struct fodcontext * fod, v3f32 x[CE30_WH]){}
void probe_tracker (struct poitracker tracker[], v3f32 x[CE30_WH]){}
void probe_quit(){}
void probe_flush(){}
#endif
