/*
como enchufar el i2c al z1 usando el coso de 5 pines del zig001
contando desde el pin mas cercano a la ficha micro usb:
1.      (pin 22)
2. GND  (pin 24)
3. SCL  (pin 26)
4. SDA  (pin 28)
5.      (pin 30)

cuando esta conectado el i2c al attiny y este esta sin alimentar, el z1 prende un led rojo y no anda
pero cuando esta alimentado anda bien
ALIMENTAR ATTiny CON 3.3V por las dudas. el attiny aguanta 5v pero el z1 no, solo 3.3v
*/
#include "i2c-generico.h"
#include <stdio.h>

#define I2C_SLAVE_ADDR 0x0A
#define DELAY(...)	\
	etimer_set(&et, CLOCK_SECOND/128);	\
	PROCESS_WAIT_EVENT()
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
	(byte & 0x80 ? '1' : '0'), \
	(byte & 0x40 ? '1' : '0'), \
	(byte & 0x20 ? '1' : '0'), \
	(byte & 0x10 ? '1' : '0'), \
	(byte & 0x08 ? '1' : '0'), \
	(byte & 0x04 ? '1' : '0'), \
	(byte & 0x02 ? '1' : '0'), \
	(byte & 0x01 ? '1' : '0') 

/*---------------------------------------------------------------------------*/
PROCESS(prueba_i2c_generico, "Climatizacion I2C Master");
AUTOSTART_PROCESSES(&prueba_i2c_generico);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(prueba_i2c_generico, ev, data)
{
	PROCESS_BEGIN();

	/* variables en static para que preserven sus valores en cambios de contexto */

	/* COPIAR Y PEGAR ACÁ ABAJO LOS ARREGLOS OBTENIDOS CON codificador.ino */
	static uint16_t tiempos[] = {4400, 4350, 600, 1550, 500, 1600, 550, 5150};
	static uint8_t indices[] = {1, 35, 36, 35, 37, 36, 36, 35, 36, 36, 37, 36, 36, 35, 37, 36, 37, 36, 37, 35, 37, 37, 36, 37, 35, 37, 36, 36, 36, 36, 35, 36, 36, 37, 35, 37, 36, 36, 36, 36, 100, 36, 36, 36, 35, 37, 35, 37, 35, 39, 1, 35, 36, 35, 37, 36, 36, 35, 36, 36, 37, 36, 36, 35, 37, 36, 37, 36, 35, 37, 35, 37, 36, 37, 35, 37, 36, 36, 36, 36, 35, 36, 36, 37, 37, 35, 36, 36, 36, 36, 36, 36, 36, 36, 35, 37, 35, 37, 37};

	static uint16_t tiemposLength, indicesLength, i;
	static struct etimer et;

	tiemposLength = sizeof(tiempos)/sizeof(tiempos[0]);
	indicesLength = sizeof(indices)/sizeof(indices[0]);
	enable(1);  // i2c
	set_address(I2C_SLAVE_ADDR);
	printf("Envio al ATtiny: \n");

	printf("Longitud de tiempos[]:\t%u\n", tiemposLength);
	DELAY();
	transmit((uint8_t*) &tiemposLength, 2);

	for (i = 0; i < tiemposLength; i++) {
		printf("tiempos[%u]:\t%u\t("BYTE_TO_BINARY_PATTERN BYTE_TO_BINARY_PATTERN")\n", i, tiempos[i], BYTE_TO_BINARY(tiempos[i] >> 8), BYTE_TO_BINARY(tiempos[i]));
		DELAY();
		transmit((uint8_t*) (tiempos + i), 2);
	}

	printf("Longitud de indices[]:\t%u\n", indicesLength);
	DELAY();
	transmit((uint8_t*) &indicesLength, 2);

	for (i = 0; i < indicesLength; i++) {
		printf("indices[%u]:\t%u\t("BYTE_TO_BINARY_PATTERN")\n", i, indices[i], BYTE_TO_BINARY(indices[i]));
		DELAY();
		transmit((uint8_t*) (indices + i), 1);
	}
	printf("Fin de la transmision\n");
	enable(0);  // i2c


	// dump para debug
	printf("static uint16_t tiempos[] = {");
	for (i = 0; i < tiemposLength-1; i++) {
		printf("%u, ", tiempos[i]);
	}
	printf("%u};\n", tiempos[tiemposLength-1]);
	printf("static uint8_t indices[] = {");
	for (i = 0; i < indicesLength-1; i++) {
		printf("%u, ", indices[i]);
	}
	printf("%u};\n", indices[indicesLength-1]);

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/