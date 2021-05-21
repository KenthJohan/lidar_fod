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

static char const * xloglvl_tostr(enum xloglvl level)
{
	switch (level)
	{
	case XLOG_ERR:
		return TFG(255,50,50) "ERR" TCOL_RST;
		break;
	case XLOG_WRN:
		return TFG(200,200,50) "WRN" TCOL_RST;
		break;
	case XLOG_INF:
		return TFG(180,180,255) "INF" TCOL_RST;
		break;
	}
	return "";
}

#define XLOG(level, format, ...) xlog(__COUNTER__, __FILE__, __LINE__, __func__, level, (format), ## __VA_ARGS__)

void xlog(int counter, char const * file, int line, char const * func, enum xloglvl level, char * format, ...)
{
	va_list args;
	va_start (args, format);
	char * w = TFG(100,100,100);
	printf ("%s%s [%i] %s:%i %s(): "TCOL_RST, xloglvl_tostr(level), w, counter, file, line, func);
	vprintf (format, args);
	va_end (args);
}

