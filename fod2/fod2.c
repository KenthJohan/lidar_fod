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

#include "../shared/ce30.h"
#include "../shared/mg_send.h"

#include "detection2.h"
#include "graphics.h"
#include "pcfod.h"



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
} mainarg;


void loop_stdin (struct graphics * g, struct fodcontext * fod, FILE * f)
{
	while (1)
	{
		fodcontext_read (fod, f);
		detection_input (g, fod);
		if (mainarg.usleep){usleep (mainarg.usleep);}
	}
}



void loop_file (struct graphics * g, struct fodcontext * fod, FILE * f)
{
	fodcontext_read (fod, f);
	for (int i = 0; i < 1000; ++i)
	{
		detection_input (g, fod);
	}
	int c = '\n';
	while (1)
	{
		XLOG (XLOG_INF, XLOG_GENERAL, "Frame %i", ce30_ftell(f));
		fodcontext_read (fod, f);
		detection_input (g, fod);
		graphics_draw_pointcloud_alpha (g, CE30_WH, fod->pc_input, fod->pc_alpha);
		graphics_flush (g);
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




	struct graphics g;
	g.lines.count = 200;
	g.points.count = CE30_WH*2;
	graphics_init (&g, mainarg.address);


	struct fodcontext * fod = fodcontext_init();




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
		loop_stdin (&g, fod, f);
	}
	else if (f != NULL)
	{
		XLOG (XLOG_INF, XLOG_GENERAL, "Reading from file %s", mainarg.filename);
		loop_file (&g, fod, f);
	}
	else
	{
		return -1;
	}

	nng_close (g.sock);
	return 0;
}
