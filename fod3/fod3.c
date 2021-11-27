#include <unistd.h>
#include <stdio.h>
#include <inttypes.h>


#include "csc/csc_crossos.h"
#include "csc/csc_malloc_file.h"
#include "csc/csc_filecopy.h"
#include "csc/csc_argv.h"
#include "csc/csc_assert.h"
#include "csc/csc_xlog.h"
#include "csc/csc_math.h"

#include "../shared/ce30.h"

#include "fodcontext.h"

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







void loop_stdin (struct fodcontext * fod, FILE * f)
{
	while (1)
	{
		v4f32 xyzw[CE30_WH];
		ce30_read (f, xyzw, 5);
		fodcontext_input (fod, xyzw);
		milomqtt_send (fod);
		if (mainarg.usleep){usleep (mainarg.usleep);}
	}
}




void loop_file (struct fodcontext * fod, FILE * f)
{
	int c = '\n';
	while (1)
	{
		XLOG (XLOG_INF, XLOG_GENERAL, "Frame %i", ce30_ftell(f));
		v4f32 xyzw[CE30_WH];
		ce30_read (f, xyzw, 5);
		fodcontext_input (fod, xyzw);
		milomqtt_send (fod);
		//if (mainarg.flags & ARG_CTRLMODE){c = getchar();}
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
		loop_stdin (fodctx, f);
	}
	else if (f != NULL)
	{
		XLOG (XLOG_INF, XLOG_GENERAL, "Reading from file %s", mainarg.filename);
		loop_file (fodctx, f);
	}
	else
	{
		return -1;
	}

	probe_quit();
	return 0;
}
