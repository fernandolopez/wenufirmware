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
#include <msp430.h>
#include <msp430f2617.h>

#ifdef MOTA_DE_CONTROL
#define ID_MOTA "linti_control"
#define TEMP_ONLY
#else
// ID para MQTT y para JSON. El estándar MQTT define que el tamaño
// máximo del ID debe ser 23 bytes, abajo hay 23 "-" como guia.
// Regla de 23: |-----------------------|
//#define ID_MOTA "linti_cocina"
#define ID_MOTA "linti_servidores"
//#define ID_MOTA "linti_oficina_1"
#define TEMP_ONLY
#endif
#define PERIODO CLOCK_SECOND * 60 * 5
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
/*---------------------------------------------------------------------------*/
#define QUICKSTART "quickstart"
/*---------------------------------------------------------------------------*/
static struct etimer publish_periodic_timer;
static struct etimer read_sensors_timer;
static struct ctimer ct;
static char *buf_ptr;
static int16_t seq_nr_value = 0;
static int16_t temperatura=0;
static int16_t decimas;
static uint16_t corriente;
static int16_t movimiento;
static unsigned int voltaje;

/*
* Blocking Delay in Milliseconds
* delays by at least the specified number of milliseconds (mSec)
* @pre MCLK = 8MHz
* @param ms number of milliseconds to delay
*/
void delayMs(unsigned int ms)
{
  int i;
  int j;
    for (i = ms; i; i--)
    {
        for (j = CLOCK_SECOND / 1000; j; j--);   //delay "m" mSec
    }
}

/** Private helper method to setup ADC for one-shot conversion and read out
 * value according to registers.
 * Inserts a delay before beginning conversion if REFON
 * @return the raw ADC value with the specified commands.
 * @todo move the VREF warmup to startup and leave on to avoid 17mSec blocking
 * delay each time?
 * */
unsigned int getAnalogInput(unsigned int adc12ctl0, unsigned int adc12ctl1, unsigned char adc12mctl0)
{
#define ADC_VREF_DELAY_MS 17
	ADC12CTL0 = adc12ctl0;
	ADC12CTL1 = adc12ctl1;
	ADC12MCTL0 = adc12mctl0;
	if (adc12ctl0 & REFON)                    // if internal reference is used...
		delayMs(ADC_VREF_DELAY_MS);           // 17mSec delay required to Vref capacitors
	ADC12CTL0 |= ENC;                         // Enable conversions
	ADC12CTL0 |= ADC12SC;                     // Start conversions
	while (!(ADC12IFG & 0x01));               // Conversion done?
	return ADC12MEM0;    // Read out 1st ADC value
}

/* Measures Vcc to the MSP430, nominally 3.3V
 * - ADC measures VCC/2 compared to 2.5V reference
 *   - If Vcc = 3.3V, ADC output should be (1.65/2.5)*4095 = 2703
 *   - Therefore (halfVcc/2.5)*4095 = ADC reading and (Vcc/2.5)*4095 = 2*ADC
 *   - So Vcc*4096 = 5*ADC and VCC=5*ADC/4095
 *
 *   @return Vcc in millivolts
 *   */
unsigned int getVcc3()
{
	unsigned int ctl0 = REFON + REF2_5V + ADC12ON + SHT0_15;  // turn on 2.5V ref, set samp time=1024 cycles
	unsigned int ctl1 = SHP;                                  // Use sampling timer, internal ADC12OSC
	unsigned char mctl0 = SREF_1 + INCH_11;                   // Channel A10, Vcc/2
	unsigned long vcc = (unsigned long) getAnalogInput(ctl0, ctl1, mctl0);
	unsigned long mult = vcc * 5000l;
	return ((unsigned int)(mult / 4096l));
}

/*---------------------------------------------------------------------------*/
PROCESS(mqtt_demo_process, "MQTT Demo");
AUTOSTART_PROCESSES(&mqtt_demo_process);
/*---------------------------------------------------------------------------*/
int ipaddr_sprintf(char *buf, uint8_t buf_len, const uip_ipaddr_t *addr)
{
  uint16_t a;
  uint8_t len = 0;
  int i, f;
  for(i = 0, f = 0; i < sizeof(uip_ipaddr_t); i += 2) {
    a = (addr->u8[i] << 8) + addr->u8[i + 1];
    if(a == 0 && f >= 0) {
      if(f++ == 0) {
        len += snprintf(&buf[len], buf_len - len, "::");
      }
    } else {
      if(f > 0) {
        f = -1;
      } else if(i > 0) {
        len += snprintf(&buf[len], buf_len - len, ":");
      }
      len += snprintf(&buf[len], buf_len - len, "%x", a);
    }
  }
  return len;
}

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
  uip_ip6addr(&server_address, 0x2800, 0x340, 0x52, 0x66, 0x2fa3, 0xdbac, 0x4c72, 0x7aa0);
  mqtt_init(in_buffer, sizeof(in_buffer),
            out_buffer, sizeof(out_buffer));
  mqtt_connect(&server_address, UIP_HTONS(1883), 1,
               &connect_info);
  PROCESS_WAIT_EVENT_UNTIL(ev == mqtt_event);

  SENSORS_ACTIVATE(sht25); // Temperatura
#ifndef TEMP_ONLY
  SENSORS_ACTIVATE(phidgets); // Analógicos (mov y corriente)
#endif

  printf("MQTT Demo Process\n");
  static char fmt_mensaje[] = "{"\
    "\"mote_id\":\"" ID_MOTA "\","\
    "\"temperature\":%u.%u,"\
    "\"current\":%u,"\
    "\"movement\":%u,"\
	  "\"voltage\":%u"\
    "}";
  static char mensaje[sizeof(fmt_mensaje) - 8 + 4 + 4 + 1 + 1 + 4];

  etimer_set(&read_sensors_timer, PERIODO);

  while(1) {

    PROCESS_YIELD();


    if (etimer_expired(&read_sensors_timer) && mqtt_connected()){
      leds_on(LEDS_GREEN);
      temperatura = sht25.value(SHT25_VAL_TEMP);
      decimas = temperatura % 100;
      temperatura = (temperatura / 100) % 100; // Limitado para que de 2 dígitos si o si
#ifdef TEMP_ONLY
      corriente = movimiento = 0;
#else
      corriente = phidgets.value(PHIDGET5V_1);
      CURRENT_SENSOR_RELATIVE(corriente);
      movimiento = phidgets.value(PHIDGET3V_2) > 2000;
#endif

      voltaje = getVcc3();

      snprintf(mensaje, sizeof(mensaje), fmt_mensaje, temperatura, decimas,
               corriente, movimiento, voltaje);
      //snprintf(mensaje, 1000, "Hola");
      mqtt_publish("linti/ipv6/temp", mensaje, 0, 1);
#ifdef DEBUGEAR
      static char buf[40];
      ipaddr_sprintf(buf, 40, &uip_ds6_get_global(ADDR_PREFERRED)->ipaddr);
      printf("IP global %s, Conectado? %d\n\r", buf, mqtt_connected());
      printf("Publicando cada %lu segundos\n\r", PERIODO);
      puts(mensaje);
#endif
    }
    leds_off(LEDS_GREEN);
    etimer_set(&read_sensors_timer, PERIODO);
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
