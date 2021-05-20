#pragma once

#include <stdio.h>
#include <stdarg.h>
#include "csc_tcol.h"

enum xloglvl
{
	XLOG_ERR,
	XLOG_WRN,
	XLOG_INF
};


void xlog(enum xloglvl level, char * format, ...)
{
	va_list args;
	va_start (args, format);
	char * color = "";
	switch (level)
	{
	case XLOG_ERR:
		color = TCOL(TCOL_NORMAL, TCOL_RED, TCOL_DEFAULT);
		printf ("%sERR: ", color);
		break;
	case XLOG_WRN:
		color = TCOL(TCOL_NORMAL, TCOL_YELLOW, TCOL_DEFAULT);
		printf ("%sWRN: ", color);
		break;
	case XLOG_INF:
		color = TCOL(TCOL_NORMAL, TCOL_BLUE, TCOL_DEFAULT);
		printf ("%sINF: ", color);
		break;
	}
	printf ("%s", TCOL_RST);
	vprintf (format, args);
	va_end (args);
}

