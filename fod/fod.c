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

#include "misc.h"
#include "detection.h"
#include "fodcontext.h"
#include "mainarg.h"


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


void loop_stdin (struct poitracker * pc, struct fodcontext * fod, FILE * f)
{
	while (1)
	{
		fodcontext_read (fod, f);
		detection_input (pc, fod);
		if (mainarg.usleep){usleep (mainarg.usleep);}
	}
}




void loop_file (struct poitracker * pc, struct fodcontext * fod, FILE * f)
{
	int c = '\n';
	while (1)
	{
		XLOG (XLOG_INF, XLOG_GENERAL, "Frame %i", ce30_ftell(f));
		fodcontext_read (fod, f);
		detection_input (pc, fod);
		if (mainarg.flags & ARG_CTRLMODE){c = getchar();}
		if (mainarg.usleep){usleep (mainarg.usleep);}
		if (c == 'q'){return;}
	}
}










int main (int argc, char const * argv[])
{
	UNUSED (argc);
	setbuf(stdout, NULL);
	csc_crossos_enable_ansi_color();

	mainarg.address = "tcp://localhost:9002";
	mainarg.filename = NULL;
	mainarg.flags = 0;
	mainarg.usleep = 0;
	mainarg.frame = 0;

	struct csc_argv_option option[] =
	{
	{'h', "help",            CSC_TYPE_U32,    &mainarg.flags,      ARG_HELP,            "Show help"},
	{'v', "verbose",         CSC_TYPE_U32,    &mainarg.flags,      ARG_VERBOSE,         "Show verbose"},
	{'i', "input",           CSC_TYPE_U32,    &mainarg.flags,      ARG_STDIN,           "Get pointcloud from stdin"},
	{'c', "ctrlmode",        CSC_TYPE_U32,    &mainarg.flags,      ARG_CTRLMODE,        "Step forward foreach keypress"},
	{'a', "address",         CSC_TYPE_STRING, &mainarg.address,    0,                   "The MQTT address to send to"},
	{'f', "filename",        CSC_TYPE_STRING, &mainarg.filename,   0,                   "Filename to f32 xyzw 320x20 file"},
	{'F', "frame",           CSC_TYPE_U32,    &mainarg.frame,      0,                   "The starting frame"},
	{'d', "duration",        CSC_TYPE_U32,    &mainarg.usleep,     0,                   "Duration for each frame"},
	{CSC_ARGV_END}};

	csc_argv_parseall (argv+1, option);

	if (mainarg.flags & ARG_HELP)
	{
		csc_argv_description0 (option, stdout);
		csc_argv_description1 (option, stdout);
		return 0;
	}

	probe_init (mainarg.address);

	struct poitracker tracker;
	poitracker_init (&tracker);

	struct fodcontext * fodctx = calloc(1, sizeof (struct fodcontext));

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
		loop_stdin (&tracker, fodctx, f);
	}
	else if (f != NULL)
	{
		XLOG (XLOG_INF, XLOG_GENERAL, "Reading from file %s", mainarg.filename);
		loop_file (&tracker, fodctx, f);
	}
	else
	{
		return -1;
	}

	probe_quit();
	return 0;
}
