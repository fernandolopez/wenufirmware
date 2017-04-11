#ifndef MQTT_SENSORES_H
#define MQTT_SENSORES_H

#include "contiki-conf.h"
#include "contiki.h"
#include "rpl/rpl-private.h"
#include "net/rpl/rpl.h"
#include "net/ip/uip.h"
#include "net/ip/uip-debug.h"
#include "net/ipv6/uip-icmp6.h"
#include "net/ipv6/sicslowpan.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "lib/sensors.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include <string.h>
#include "dev/sht25.h"
#include "dev/z1-phidgets.h"
#include "mqtt-service.h"
#include "dev/battery-sensor.h"
#include <msp430.h>
#include "reports.h"

#ifdef MOTA_DE_CONTROL
#define ID_MOTA "linti_control"
#define TEMP_ONLY
#else
// ID para MQTT y para JSON. El estándar MQTT define que el tamaño
// máximo del ID debe ser 23 bytes, abajo hay 23 "-" como guia.
// Regla de 23: |-----------------------|
//#define ID_MOTA "linti_cocina"
//#define ID_MOTA "linti_servidores"
#define ID_MOTA "linti_oficina_1"
//#define TEMP_ONLY
#endif
#define PERIODO CLOCK_SECOND * 5 * 60 // 5 minutos por reporte.
#define DEBUGEAR

// Parece que al ser un sensor de 5v y como la mota tiene un divisor
// de tensión, cuando el sensor está en 0, la mota lee 2100 aproximadamente,
// las siguientes macros son para ajustar el valor a algo razonable
#define CURRENT_SENSOR_ZERO 2105
#define CURRENT_SENSOR_RELATIVE(measurement) do {\
    (measurement) = (measurement) - (CURRENT_SENSOR_ZERO);\
    (measurement) = ((measurement) > 4096)?0:(measurement);\
} while (0);

/*---------------------------------------------------------------------------*/
#ifndef BOARD_STRING
#define BOARD_STRING  "Zolertia Z1 Node"
#endif
/*---------------------------------------------------------------------------*/

/* Buffers para MQTT, si el tamaño no es suficiente no transmite los
 * datos y falla silenciosamente*/
#define IN_BUFFER_SIZE 24
#define OUT_BUFFER_SIZE 128

/*---------------------------------------------------------------------------*/
#define QUICKSTART "quickstart"

/*---------------------------------------------------------------------------*/
const char *format_message(const char *mote_id, int temp_deg, int temp_dec, int current, int movement, int voltage);

void temperature_split(int16_t temperature, int16_t *degrees, uint16_t *dec);

int validate(int16_t degrees, uint16_t dec, uint16_t curr, uint16_t volt, uint16_t mov);

#endif
