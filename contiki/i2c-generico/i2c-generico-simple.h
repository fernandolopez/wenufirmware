/* funciones para manejar dispositivo genérico i2c
(robado bastante de sht25.c)
transmit y receive se manejan con 2 y 1 bytes respectivamente */
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
/* transmite 1 byte */
static void
transmit(uint8_t reg)
{
  uint8_t rtx = reg;

  i2c_transmitinit(own_address);
  while(i2c_busy());
  i2c_transmit_n(1, &rtx);
  while(i2c_busy());
}
/*---------------------------------------------------------------------------*/
/* recibe 2 bytes */
static uint16_t
receive()
{
  uint8_t buf[] = { 0x00, 0x00 };

  i2c_receiveinit(own_address);
  while(i2c_busy());
  i2c_receive_n(2, &buf[0]);
  while(i2c_busy());

  return (uint16_t) (buf[0] << 8 | (buf[1]));
}