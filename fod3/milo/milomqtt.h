#pragma once
#include "fodcontext.h"

void milomqtt_init (char const * host, int port, int keepalive);
void milomqtt_send (struct fodcontext * fod);

