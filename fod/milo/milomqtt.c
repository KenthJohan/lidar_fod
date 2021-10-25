#include <mosquitto.h>
#include "milomqtt.h"




static void publish_float (struct mosquitto *mosq, int qos, char const * topic, float number)
{
	char payload[20];
	snprintf (payload, sizeof(payload), "%f", number);
	//printf ("Msg %s: %s\n", topic, payload);
	int rc = mosquitto_publish (mosq, NULL, topic, strlen (payload), payload, qos, 0);
	ASSERTF (rc == MOSQ_ERR_SUCCESS, "%s", mosquitto_strerror (rc));
	if (rc != MOSQ_ERR_SUCCESS)
	{
		fprintf (stderr, "Error publishing: %s\n", mosquitto_strerror (rc));
		exit (0);
	}
}

static void publish_int (struct mosquitto *mosq, int qos, char const * topic, int number)
{
	char payload[20];
	snprintf (payload, sizeof(payload), "%i", number);
	//printf ("Msg %s: %s\n", topic, payload);
	int rc = mosquitto_publish (mosq, NULL, topic, strlen (payload), payload, qos, 0);
	ASSERTF (rc == MOSQ_ERR_SUCCESS, "%s", mosquitto_strerror (rc));
	if (rc != MOSQ_ERR_SUCCESS)
	{
		fprintf (stderr, "Error publishing: %s\n", mosquitto_strerror (rc));
		exit (0);
	}
}


struct mosquitto * global_mosq;


static void on_connect (struct mosquitto *mosq, void *obj, int reason_code)
{
	UNUSED (obj);
	printf ("on_connect: %s\n", mosquitto_connack_string (reason_code));
	ASSERTF (reason_code == 0, "reason_code: %i", reason_code);
	if (reason_code != 0)
	{
		mosquitto_disconnect (mosq);
	}
}

static void on_publish (struct mosquitto *mosq, void *obj, int mid)
{
	UNUSED (mosq);
	UNUSED (obj);
	UNUSED (mid);
	//printf ("Message with mid %d has been published.\n", mid);
}

void milomqtt_init (char const * host, int port, int keepalive)
{
	ASSERT_PARAM_NOTNULL (host);
	global_mosq = mosquitto_new (NULL, 1, NULL);
	ASSERT_NOTNULL (global_mosq);
	if (global_mosq == NULL)
	{
		fprintf (stderr, "Error: Out of memory.\n");
		exit (1);
	}

	mosquitto_connect_callback_set (global_mosq, on_connect);
	mosquitto_publish_callback_set (global_mosq, on_publish);

	if (global_mosq != NULL)
	{
		int rc = mosquitto_connect (global_mosq, host, port, keepalive);
		ASSERTF (rc == MOSQ_ERR_SUCCESS, "mosquitto_connect %s %i %i: %s", host, port, keepalive, mosquitto_strerror (rc));
		if (rc != MOSQ_ERR_SUCCESS)
		{
			mosquitto_destroy (global_mosq);
			fprintf (stderr, "Error: %s\n", mosquitto_strerror(rc));
			exit (1);
		}
	}

	{
		int rc = mosquitto_loop_start(global_mosq);
		ASSERTF (rc == MOSQ_ERR_SUCCESS, "mosquitto_loop_start: %s", mosquitto_strerror (rc));
		if (rc != MOSQ_ERR_SUCCESS)
		{
			mosquitto_destroy (global_mosq);
			fprintf (stderr, "Error: %s\n", mosquitto_strerror (rc));
			exit (1);
		}
	}

}




void milomqtt_send (struct fodcontext * fod)
{
	int qos = 0;
	for (uint32_t i = 0; i < fod->tracker.count; ++i)
	{
		if (fod->tracker.r[i] != FLT_MAX)
		{
			publish_int (global_mosq, qos, "/command/c2h/lidarfod/obj/id", i);
			publish_float (global_mosq, qos, "/command/c2h/lidarfod/obj/h", fod->tracker.h[i]);
			publish_float (global_mosq, qos, "/command/c2h/lidarfod/obj/x", fod->tracker.x[i].x);
			publish_float (global_mosq, qos, "/command/c2h/lidarfod/obj/y", fod->tracker.x[i].y);
			publish_float (global_mosq, qos, "/command/c2h/lidarfod/obj/z", fod->tracker.x[i].z);
		}
	}
}
