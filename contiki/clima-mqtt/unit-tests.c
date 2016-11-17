#include "unit-test.h"
#include <stdio.h>
#include <string.h>

#include "mqtt-sensores.h"

// /*---------------------------------------------------------------------------*/

// UNIT_TEST_REGISTER(temp_rango, "Verificar si la temperatura leida esta en un rango razonable");

// UNIT_TEST(temp_rango) {
// 	UNIT_TEST_BEGIN();

// 	// rango de temperatura: -40º - 123.8º
// 	// http://zolertia.sourceforge.net/wiki/index.php/Z1_Sensors#ZIG001_Temperature-Humidity_Sensor
// 	UNIT_TEST_ASSERT(temperatura >= -40 && temperatura <= 124);

// 	UNIT_TEST_END();
// }

// /*---------------------------------------------------------------------------*/

// UNIT_TEST_REGISTER(temp_digitos, "Verificar si la parte entera de la temperatura son dos digitos");

// UNIT_TEST(temp_digitos) {
// 	UNIT_TEST_BEGIN();

// 	UNIT_TEST_ASSERT(temperatura == temperatura % 100);

// 	UNIT_TEST_END();
// }

// /*---------------------------------------------------------------------------*/

// UNIT_TEST_REGISTER(bateria_voltaje_rango, "Verificar si el voltaje leido esta en un rango razonable");

// UNIT_TEST(bateria_voltaje_rango) {
// 	UNIT_TEST_BEGIN();

// 	// rango de voltaje: -30V - 30V
// 	// http://www.phidgets.com/products.php?product_id=1117
// 	UNIT_TEST_ASSERT(voltaje >= -30 && voltaje <= 30);

// 	UNIT_TEST_END();
// }

// /*---------------------------------------------------------------------------*/

// UNIT_TEST_REGISTER(mov_correcto, "Verificar si el sensor de movimiento es leido correctamente");

// UNIT_TEST(mov_correcto) {
// 	UNIT_TEST_BEGIN();

// 	UNIT_TEST_ASSERT((movimiento == 0) || (movimiento == 1));

// 	UNIT_TEST_END();
// }

/*---------------------------------------------------------------------------*/

#define STRING_PRUEBA "string_prueba"
// #define TEMP_PRUEBA 2
// #define DEC_PRUEBA 3
// #define CURR_PRUEBA 4
// #define MOV_PRUEBA 0
// #define VOLT_PRUEBA 1

UNIT_TEST_REGISTER(format_message_correcto, "Verificar si la funcion format_message funciona bien");

UNIT_TEST(format_message_correcto) {
	UNIT_TEST_BEGIN();

	const char * stringFormatMessage = format_message(STRING_PRUEBA, 2, 3, 4, 0, 1);
	const char * stringUnitTest =	"{"							\
							"\"mote_id\":\""STRING_PRUEBA"\","		\
							"\"temperature\":2.3,"	\
							"\"current\":4,"			\
							"\"movement\":0,"			\
							"\"voltage\":1"			\
							"}";
	// strcmp() devuelve 0 si los dos strings son equivalentes
	UNIT_TEST_ASSERT(strcmp(stringFormatMessage, stringUnitTest) == 0);

	UNIT_TEST_END();
}