#include <unistd.h>
#include <stdio.h>
#include <inttypes.h>

#include <nng/nng.h>
#include <nng/protocol/pair0/pair.h>
#include <nng/supplemental/util/platform.h>

#include "csc/csc_debug_nng.h"
#include "csc/csc_crossos.h"
#include "csc/csc_malloc_file.h"
#include "csc/csc_filecopy.h"
#include "csc/csc_argv.h"
#include "csc/csc_assert.h"
#include "csc/csc_xlog.h"
#include "csc/csc_crossos.h"
#include "csc/csc_malloc_file.h"
#include "csc/csc_filecopy.h"
#include "csc/csc_math.h"

#include "../shared/ce30.h"

#include "components.h"
#include "misc.h"
#include "detection.h"
#include "fodcontext.h"
#include "flecs.h"

#include "probe/probe.h"
#include "milo/milomqtt.h"



#define ARG_HELP             UINT32_C(0x00000001)
#define ARG_VERBOSE          UINT32_C(0x00000002)
#define ARG_STDIN            UINT32_C(0x00000010)
#define ARG_LEGACY_FILENAME  UINT32_C(0x00000100)
#define ARG_CTRLMODE         UINT32_C(0x00001000)

struct
{
	char const * address; //Grahpic remote address
	char const * filename; //Load filename
	uint32_t flags; //Misc flags
	uint32_t usleep; //Microseconds sleep each frame
	uint32_t frame; //Start from this frame when loading a file.

	// The hostname or ip address of the broker to connect to.
	char const * mqtthost;
	// The network port to connect to. Usually 1883.
	int mqttport;
	// The number of seconds after which the broker should send a PING message to the client if no other messages have been exchanged in that time.
	int mqttkeepalive;
} mainarg;




static void fodcontext_read (struct fodcontext * fod, FILE * f)
{
	ASSERT_PARAM_NOTNULL (fod);
	ASSERT_PARAM_NOTNULL (f);
	memset (fod->pc_tags, 0, sizeof(uint8_t)*CE30_WH);
	// Read 5 frames from CE30 LiDAR because it updates pointcloud per 5 frame.
	ce30_read (f, fod->pc_src, 5);
	ce30_xyzw_to_pos_amp_flags (fod->pc_src, fod->pc_x1, fod->pc_amplitude1, fod->pc_tags);
	ce30_detect_incidence_edges (fod->pc_tags);
}


void loop_stdin (ecs_world_t *world, struct fodcontext * fod, FILE * f)
{
	while (1)
	{
		fodcontext_read (fod, f);
		detection_input (fod);
		milomqtt_send (fod);
		if (mainarg.usleep){usleep (mainarg.usleep);}
	}
}




void loop_file (ecs_world_t *world, struct fodcontext * fod, FILE * f)
{
	int c = '\n';
	while (1)
	{
		XLOG (XLOG_INF, XLOG_GENERAL, "Frame %i", ce30_ftell(f));
		fodcontext_read (fod, f);
		detection_input (fod);
		milomqtt_send (fod);
		ecs_progress(world, 0.0f);
		if (mainarg.flags & ARG_CTRLMODE){c = getchar();}
		if (mainarg.usleep){usleep (mainarg.usleep);}
		if (c == 'q'){return;}
	}
}



struct fodcontext * fodcontext_create()
{
	struct fodcontext * fodctx = calloc (1, sizeof (struct fodcontext));
	fodctx->sample_mean_variance = 1.0f; //This is dotproduct result. 1.0 == 0 deg
	fodctx->sample_normal = (v3f32){{0.0f, 0.0f, 1.0f}};
	poitracker_init (&fodctx->tracker);
	return fodctx;
}






int main (int argc, char const * argv[])
{
	UNUSED (argc);
	setbuf(stdout, NULL);
	csc_crossos_enable_ansi_color();

	ecs_world_t *world = ecs_init_w_args(argc, (char **)argv);
	ecs_set(world, EcsWorld, EcsRest, {0});
	ECS_META_COMPONENT(world, Position);
	ECS_META_COMPONENT(world, Velocity);

	mainarg.address = "tcp://localhost:9002";
	mainarg.filename = NULL;
	mainarg.flags = 0;
	mainarg.usleep = 0;
	mainarg.frame = 0;
	mainarg.mqttport = 1883;
	mainarg.mqttkeepalive = 60;
	mainarg.mqtthost = "192.168.1.195";//Logserver

	struct csc_argv_option option[] =
	{
	{'h', "help",            CSC_TYPE_U32,    &mainarg.flags,         ARG_HELP,            "Show help"},
	{'v', "verbose",         CSC_TYPE_U32,    &mainarg.flags,         ARG_VERBOSE,         "Show verbose"},
	{'i', "input",           CSC_TYPE_U32,    &mainarg.flags,         ARG_STDIN,           "Get pointcloud data from stdin"},
	{'c', "ctrlmode",        CSC_TYPE_U32,    &mainarg.flags,         ARG_CTRLMODE,        "Step forward by pressing enter"},
	{'a', "address",         CSC_TYPE_STRING, &mainarg.address,       0,                   "Probe: Algorithm probing server address"},
	{'h', "mqtthost",        CSC_TYPE_STRING, &mainarg.mqtthost,      0,                   "MQTT: The hostname or ip address of the broker to connect to."},
	{'p', "mqttport",        CSC_TYPE_STRING, &mainarg.mqttport,      0,                   "MQTT: The network port to connect to. Usually 1883."},
	{'k', "mqttkeepalive",   CSC_TYPE_STRING, &mainarg.mqttkeepalive, 0,                   "MQTT: The number of seconds after which the broker should send a PING message to the client if no other messages have been exchanged in that time."},
	{'f', "filename",        CSC_TYPE_STRING, &mainarg.filename,      0,                   "Filename to f32 xyzw 320x20 file"},
	{'F', "frame",           CSC_TYPE_U32,    &mainarg.frame,         0,                   "The starting frame"},
	{'d', "duration",        CSC_TYPE_U32,    &mainarg.usleep,        0,                   "Duration for each frame"},
	{CSC_ARGV_END}};

	csc_argv_parseall (argv+1, option);

	if (mainarg.flags & ARG_HELP)
	{
		csc_argv_description0 (option, stdout);
		csc_argv_description1 (option, stdout);
		return 0;
	}

	// Either use probe or milomqtt:
	probe_init (mainarg.address);
	milomqtt_init (mainarg.mqtthost, mainarg.mqttport, mainarg.mqttkeepalive);

	struct fodcontext * fodctx = fodcontext_create();

	FILE * f = NULL;
	if (mainarg.filename)
	{
		XLOG (XLOG_INF, XLOG_GENERAL, "Opening binary file %s to read LiDAR frames.", mainarg.filename);
		f = fopen (mainarg.filename, "rb");
		ce30_seek_set (f, mainarg.frame);
		for (uint32_t i = 0; i < mainarg.frame; ++i)
		{
			rand();
		}
	}
	else
	{
		f = stdin;
	}

	ASSERT_NOTNULL (f);

	if (f == stdin)
	{
		XLOG (XLOG_INF, XLOG_GENERAL, "Reading from STDIN");
		//printf ("[INFO] Reading from STDIN\n");
		loop_stdin (world, fodctx, f);
	}
	else if (f != NULL)
	{
		XLOG (XLOG_INF, XLOG_GENERAL, "Reading from file %s", mainarg.filename);
		loop_file (world, fodctx, f);
	}
	else
	{
		return -1;
	}

	probe_quit();
	return 0;
}
