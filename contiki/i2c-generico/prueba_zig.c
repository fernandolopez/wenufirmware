#include "contiki.h"
#include "lib/sensors.h"
#include "dev/sht25.h"
#include "i2c-generico.h"
#include <stdio.h> /* For printf() */

/* afanada tal como está de sht25.c */
static int16_t
sht25_convert(uint8_t variable, uint16_t value)
{
  int16_t rd;
  uint32_t buff;
  buff = (uint32_t)value;
  if(variable == SHT25_VAL_TEMP) {
    buff *= 17572;
    buff = buff >> 16;
    rd = (int16_t)buff - 4685;
  } else {
    buff *= 12500;
    buff = buff >> 16;
    rd = (int16_t)buff - 600;
    rd = (rd > 10000) ? 10000 : rd;
  }
  return rd;
}

/*---------------------------------------------------------------------------*/
PROCESS(prueba_i2c_generico, "Prueba I2C generico con SHT25");
AUTOSTART_PROCESSES(&prueba_i2c_generico);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(prueba_i2c_generico, ev, data)
{
  PROCESS_BEGIN();

	int16_t valor_contiki;
	int16_t valor_generico;


	/* obtener valor mediante metodos de contiki */
	SENSORS_ACTIVATE(sht25);
	valor_contiki = sht25.value(SHT25_VAL_TEMP);
	SENSORS_DEACTIVATE(sht25);

	printf("Valor obtenido usando metodos de contiki: %d\n", valor_contiki);


	/* obtener valor mediante metodos`propios (excepto el sht25_convert al final que no involucra i2c) */
	uint8_t enviar = SHT25_VAL_TEMP;
	uint8_t recibir[2];
	uint16_t valor_recibido;
	// uint16_t recibir;

	enable(1);
	set_address(SHT25_ADDR);

	transmit(&enviar, 1);
	receive(recibir, 2);
	valor_recibido = (uint16_t)(recibir[0] << 8 | (recibir[1]));
	valor_generico = sht25_convert(SHT25_VAL_TEMP, valor_recibido);

	enable(0);

	printf("Valor obtenido usando metodos de generico_multiple.h: %d\n", valor_generico);
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/