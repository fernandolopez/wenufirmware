/* funciones para manejar dispositivo genérico i2c
(robado bastante de sht25.c) */
#include "i2cmaster.h"
/*---------------------------------------------------------------------------*/
static uint8_t enabled = 0;
static uint8_t own_address = 0;
/*---------------------------------------------------------------------------*/
static void
set_address(uint8_t address)
{
	own_address = address;
}
/*---------------------------------------------------------------------------*/
static void
set_rate(uint16_t rate)
{
  i2c_setrate((uint8_t) (rate & 0xFF), (uint8_t) ((rate >> 8) & 0xFF));
}
/*---------------------------------------------------------------------------*/
static int
enable(int value)
{
  if(value) {
    i2c_enable();
    enabled = 1;
  } else {
    i2c_disable();
    enabled = 0;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
    return enabled;
}
/*---------------------------------------------------------------------------*/
/* transmite la cantidad de bytes dada por size */
static void
transmit(uint8_t *buf, int size)
{
  i2c_transmitinit(own_address);
  while(i2c_busy());
  i2c_transmit_n(size, &buf[0]);
  while(i2c_busy());
}
/*---------------------------------------------------------------------------*/
/* recibe la cantidad de bytes dada por size
el resultado se debe convertir al formato correspondiente,
por ej. para recibir un uint16_t desde la funcion principal:
uint8_t buffer[2];
uint16_t recibido;
receive(buffer, 2);
recibido = (uint16_t)(recibir[0] << 8 | (recibir[1]));
*/
static uint16_t
receive(uint8_t *buf, int size)
{
  int i;
  for (i = 0; i < size; i++) {
    buf[i] = 0x07;
  }

  i2c_receiveinit(own_address);
  while(i2c_busy());
  i2c_receive_n(size, buf);
  while(i2c_busy());
  return *buf;
}