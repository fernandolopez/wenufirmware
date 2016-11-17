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

#include "mqtt-sensores.h"

/*---------------------------------------------------------------------------*/
PROCESS(mqtt_demo_process, "MQTT Demo");
AUTOSTART_PROCESSES(&mqtt_demo_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(mqtt_demo_process, ev, data)
{
    static uip_ip6addr_t server_address;
    static uint8_t in_buffer[IN_BUFFER_SIZE];
    static uint8_t out_buffer[OUT_BUFFER_SIZE];

    static mqtt_connect_info_t connect_info = {
        .client_id = ID_MOTA,
        .username = NULL,
        .password = NULL,
        .will_topic = NULL,
        .will_message = NULL,
        .keepalive = 60,
        .will_qos = 0,
        .will_retain = 0,
        .clean_session = 1,
    };

    PROCESS_BEGIN();
    REPORT();
    uip_ip6addr(&server_address, 0x2800, 0x340, 0x52, 0x66, 0x2fa3, 0xdbac, 0x4c72, 0x7aa0);
    mqtt_init(in_buffer, sizeof(in_buffer),
            out_buffer, sizeof(out_buffer));
    mqtt_connect(&server_address, UIP_HTONS(1883), 1,
            &connect_info);
    PROCESS_WAIT_EVENT_UNTIL(ev == mqtt_event);

    mqtt_subscribe("/motaID/accion"); // La mota se subscribe al topico

    SENSORS_ACTIVATE(sht25); // Temperatura
#ifndef TEMP_ONLY
    SENSORS_ACTIVATE(phidgets); // Analógicos (mov y corriente)
#endif

    printf("MQTT Demo Process\n");
    etimer_set(&read_sensors_timer, PERIODO);

    while(1) {

        PROCESS_YIELD();

        if (etimer_expired(&read_sensors_timer) && mqtt_connected()){
            leds_on(LEDS_GREEN);
            temperatura = sht25.value(SHT25_VAL_TEMP);
            temperature_split(temperatura, &temperatura, &decimas);
	          bateria = battery_sensor.value(0);
            voltaje = (bateria * 5000l) / 4096l;
#ifdef TEMP_ONLY
            corriente = movimiento = 0;
#else
            corriente = phidgets.value(PHIDGET5V_1);
            CURRENT_SENSOR_RELATIVE(corriente);
            movimiento = phidgets.value(PHIDGET3V_2) > 2000;
#endif

            if (validate(temperatura, decimas, corriente, voltaje, movimiento)){
                format_message(ID_MOTA, temperatura, decimas, corriente, movimiento, voltaje);
                mqtt_publish("linti/ipv6/temp", mensaje, 0, 1);
#ifdef DEBUGEAR
                puts(mensaje);
#endif
            }else{
#ifdef DEBUGEAR
                puts("Error en el rango de los datos");
#endif
                mqtt_publish("linti/ipv6/fueraderango", ID_MOTA, 0, 1);
            }
#ifdef DEBUGEAR
            static char buf[40];
            uip_debug_ipaddr_print(&uip_ds6_get_global(ADDR_PREFERRED)->ipaddr);
            printf("IP global %s, Conectado? %d\n\r", buf, mqtt_connected());
            printf("Publicando cada %lu segundos\n\r", PERIODO / CLOCK_SECOND);
#endif
    }

    if (etimer_expired(&read_sensors_timer)){
      leds_off(LEDS_GREEN);
      etimer_set(&read_sensors_timer, PERIODO);
    }

    if (mqtt_connected()){
      if (mqtt_event_is_publish(data)){
          printf("%s\n", ((mqtt_event_data_t*)data)->data);
          // Relay the received message to a new topic
      }
    }

  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
