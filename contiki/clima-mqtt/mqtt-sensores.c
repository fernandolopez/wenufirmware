/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
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
#include "dev/battery-sensor.h"
#include <msp430.h>
#include "reports.h"

#include "simple-udp.h"
static struct simple_udp_connection broadcast_connection;
#define UDP_PORT 5000

#ifdef MOTA_DE_CONTROL
#define ID_MOTA "linti_control"
#define TEMP_ONLY
#else
// ID para MQTT y para JSON. El estándar MQTT define que el tamaño
// máximo del ID debe ser 23 bytes, abajo hay 23 "-" como guia.
// Regla de 23: |-----------------------|
//#define ID_MOTA "linti_cocina"
#define ID_MOTA "linti_servidores"
// #define ID_MOTA "linti_oficina_1"
//#define TEMP_ONLY
#endif
#define PERIODO CLOCK_SECOND * 5 // 5 minutos por reporte.
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

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
#define QUICKSTART "quickstart"
/*---------------------------------------------------------------------------*/
static struct etimer read_sensors_timer;
static int16_t temperatura=0;
static uint16_t decimas;
static uint16_t corriente;
static uint16_t movimiento;
static uint16_t voltaje;
static uint16_t bateria;


static char fmt_mensaje[] = "{"\
                             "\"mote_id\":\"%s\","\
                             "\"temperature\":%u.%u,"\
                             "\"current\":%u,"\
                             "\"movement\":%u,"\
                             "\"voltage\":%u"\
                             "}";
static char mensaje[sizeof(fmt_mensaje) - 8 + 23 + 4 + 4 + 1 + 1 + 4];


/*---------------------------------------------------------------------------*/
PROCESS(mqtt_demo_process, "MQTT Demo");
AUTOSTART_PROCESSES(&mqtt_demo_process);
/*---------------------------------------------------------------------------*/


void receiver(struct simple_udp_connection *c,
        const uip_ipaddr_t *sender_addr,
        uint16_t sender_port,
        const uip_ipaddr_t *receiver_addr,
        uint16_t receiver_port,
        const uint8_t *data,
        uint16_t datalen){
    printf("Recibi: %*s\n", datalen, data);
}

const char *format_message(const char *mote_id, int temp_deg, int temp_dec, int current, int movement, int voltage){
    /** Recibe una serie de valores y returna un puntero a una variable global
     * con el string del mensaje json formado */
    snprintf(mensaje, sizeof(mensaje), fmt_mensaje, mote_id, temperatura, decimas,
            corriente, movimiento, voltaje);
    return mensaje;
}

void temperature_split(int16_t temperature, int16_t *degrees, uint16_t *dec){
    /** Recibe una temperatura en decimas de grado celcius y la descompone en
     * parte entera (degrees) y decimal (dec) */
    *dec = temperature % 100;
    *degrees = (temperature / 100) % 100;
}

int validate(int16_t degrees, uint16_t dec, uint16_t curr, uint16_t volt, uint16_t mov){
    /** Valida que las lecturas de los sensores (ya procesadas) sean válidas
     * para no enviar valores fuera de rango a la base de datos */
    return (degrees > -40 && degrees < 124) && (dec >= 0 && dec <= 99) &&\
           (curr >= 0 && curr <= 4095) && (volt >= 0 && volt < 4000) &&\
           (mov == 0 || mov == 1);
}

PROCESS_THREAD(mqtt_demo_process, ev, data)
{
    static uip_ip6addr_t server_address;

    PROCESS_BEGIN();
    REPORT();		
    //uip_ip6addr(&server_address, 0x2800, 0x340, 0x52, 0x66, 0x201, 0x2ff, 0xfe0e, 0xce94);
		// nueva ip asignada por cespi en el nuevo server pc
    // uip_ip6addr(&server_address, 0x2800, 0x340, 0x52, 0x66, 0x2fa3, 0xdbac, 0x4c72, 0x7aa0);
		// vieja ip estatica
    simple_udp_register(&broadcast_connection, UDP_PORT, NULL, UDP_PORT, receiver);
    uip_create_linklocal_allnodes_mcast(&server_address);

    SENSORS_ACTIVATE(sht25); // Temperatura
#ifndef TEMP_ONLY
    SENSORS_ACTIVATE(phidgets); // Analógicos (mov y corriente)
#endif

    printf("MQTT Demo Process\n");
    etimer_set(&read_sensors_timer, PERIODO);

    while(1) {

        PROCESS_YIELD();

        if (etimer_expired(&read_sensors_timer)){
            leds_on(LEDS_GREEN);
            temperatura = sht25.value(SHT25_VAL_TEMP);
            temperature_split(temperatura, &temperatura, &decimas);

            SENSORS_DEACTIVATE(phidgets);
            SENSORS_ACTIVATE(battery_sensor);
	          bateria = battery_sensor.value(0);
            voltaje = (bateria * 5000l) / 4096l;
            printf("%d\n", voltaje);
            SENSORS_DEACTIVATE(battery_sensor);
            SENSORS_ACTIVATE(phidgets);

#ifdef TEMP_ONLY
            corriente = movimiento = 0;
#else
            corriente = phidgets.value(PHIDGET5V_1);
            CURRENT_SENSOR_RELATIVE(corriente);
            movimiento = phidgets.value(PHIDGET3V_2) > 2000;
#endif

            if (validate(temperatura, decimas, corriente, voltaje, movimiento)){
                format_message(ID_MOTA, temperatura, decimas, corriente, movimiento, voltaje);
                //mqtt_publish("linti/ipv6/temp", mensaje, 0, 1);
                simple_udp_sendto(&broadcast_connection, mensaje, strlen(mensaje), &server_address);
#ifdef DEBUGEAR
                puts(mensaje);
#endif
            }else{
#ifdef DEBUGEAR
                puts("Error en el rango de los datos");
#endif
                //mqtt_publish("linti/ipv6/fueraderango", ID_MOTA, 0, 1);
                simple_udp_sendto(&broadcast_connection, "Fuera de rango", 14, &server_address);
            }
#ifdef DEBUGEAR
            static char buf[40];
            uip_debug_ipaddr_print(&uip_ds6_get_global(ADDR_PREFERRED)->ipaddr);
            //printf("IP global %s, Conectado? %d\n\r", buf, mqtt_connected());
            printf("Publicando cada %lu segundos\n\r", PERIODO / CLOCK_SECOND);
#endif
    }

    if (etimer_expired(&read_sensors_timer)){
      leds_off(LEDS_GREEN);
      etimer_set(&read_sensors_timer, PERIODO);
    }


    /*
    if (mqtt_connected()){
      if (mqtt_event_is_publish(data)){
          printf("%s\n", ((mqtt_event_data_t*)data)->data);
          // Relay the received message to a new topic
      }
    }
    */

  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
