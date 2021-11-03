#include <mosquitto.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

#define MOSQ_CHECK_RC(mosq, rc) mosq_checkerror(__FILE__, __LINE__, (mosq), (rc))
static void mosq_checkerror(char const * filename, int line, struct mosquitto *mosq, int rc)
{
	if (rc != MOSQ_ERR_SUCCESS)
	{
		fprintf (stderr, "mosquitto: %s:%i: %s\n", filename, line, mosquitto_strerror (rc));
		mosquitto_destroy (mosq);
		exit (1);
	}
}



static void publish_float (struct mosquitto *mosq, int qos, char const * topic, float number)
{
	char payload[20];
	snprintf (payload, sizeof(payload), "%f", number);
	int rc = mosquitto_publish (mosq, NULL, topic, strlen (payload), payload, qos, 0);
	MOSQ_CHECK_RC(mosq, rc);
}

static void publish_int (struct mosquitto *mosq, int qos, char const * topic, int number)
{
	char payload[20];
	snprintf (payload, sizeof(payload), "%i", number);
	int rc = mosquitto_publish (mosq, NULL, topic, strlen (payload), payload, qos, 0);
	MOSQ_CHECK_RC(mosq, rc);
}

static void on_connect (struct mosquitto *mosq, void *obj, int reason_code)
{
	printf ("on_connect: %s\n", mosquitto_connack_string (reason_code));
	if (reason_code != 0)
	{
		mosquitto_disconnect (mosq);
	}
}

static void on_publish (struct mosquitto *mosq, void *obj, int mid)
{

}


void milomqtt_sendloop (struct mosquitto *mosq)
{
	int i = 0;
	int qos = 0;
	while(1)
	{
		usleep(1000);
		i++;
		publish_int(mosq, qos, "/command/c2h/lidar/dummy", i);
	}
}



int main(int argc, char * argv[])
{
	struct mosquitto * mosq = mosquitto_new (NULL, 1, NULL);
	if (mosq == NULL)
	{
		fprintf (stderr, "Error: Out of memory.\n");
		exit (1);
	}
	mosquitto_connect_callback_set (mosq, on_connect);
	mosquitto_publish_callback_set (mosq, on_publish);
	
	int keepalive = 60;
	int port = 1883;
	char const * host = "192.168.1.101";
	
	if (mosq != NULL)
	{
		printf ("mosquitto_connect: %s:%i %i\n", host, port, keepalive);
		int rc = mosquitto_connect (mosq, host, port, keepalive);
		MOSQ_CHECK_RC (mosq, rc);
	}

	{
		int rc = mosquitto_loop_start (mosq);
		MOSQ_CHECK_RC (mosq, rc);
	}
	
	milomqtt_sendloop (mosq);
	
}

