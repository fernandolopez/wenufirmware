#include "unit-test.h"
#include <stdio.h>
#include <string.h>

// Is hard to link the main application and unittests, so an ugly hack
// is to copy mqtt-sensores.c to mqtt-sensores.h and remove the procceses.
#include "mqtt-sensores.h"

#define STRING_PRUEBA "string_prueba"
#define TEMP_PRUEBA 2813
// TEMP_INT_PRUEBA Y DEC_PRUEBA deben estar basados en TEMP_PRUEBA
#define TEMP_INT_PRUEBA 28
#define DEC_PRUEBA 13
#define CURR_PRUEBA 3000
#define MOV_PRUEBA 0
#define VOLT_PRUEBA 2000

UNIT_TEST_REGISTER(format_message_correcto, "Verificar si format_message funciona bien");

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

/*---------------------------------------------------------------------------*/

UNIT_TEST_REGISTER(temperatura_split_correcto, "Verificar si temperatura_split devuelve valores razonables");

UNIT_TEST(temperatura_split_correcto) {
	UNIT_TEST_BEGIN();

	int16_t temperatura_test = TEMP_PRUEBA;
	int16_t tempint_test;
	uint16_t decimas_test;
	temperature_split(temperatura_test, &tempint_test, &decimas_test);

	UNIT_TEST_ASSERT(tempint_test == TEMP_INT_PRUEBA);
	UNIT_TEST_ASSERT(decimas_test == DEC_PRUEBA);

	UNIT_TEST_END();
}

/*---------------------------------------------------------------------------*/

UNIT_TEST_REGISTER(validate_correcto, "Verificar si validate funciona bien");

UNIT_TEST(validate_correcto) {
	UNIT_TEST_BEGIN();

	// todo correcto
	UNIT_TEST_ASSERT(validate(TEMP_INT_PRUEBA, DEC_PRUEBA, CURR_PRUEBA, VOLT_PRUEBA, MOV_PRUEBA) == 1);
	// temperaturas invalidas
	UNIT_TEST_ASSERT(validate(-60, DEC_PRUEBA, CURR_PRUEBA, VOLT_PRUEBA, MOV_PRUEBA) == 0);
	UNIT_TEST_ASSERT(validate(140, DEC_PRUEBA, CURR_PRUEBA, VOLT_PRUEBA, MOV_PRUEBA) == 0);
	// decimas invalidas
	UNIT_TEST_ASSERT(validate(TEMP_INT_PRUEBA, 150, CURR_PRUEBA, VOLT_PRUEBA, MOV_PRUEBA) == 0);
	// corriente invalida
	UNIT_TEST_ASSERT(validate(TEMP_INT_PRUEBA, DEC_PRUEBA, 5000, VOLT_PRUEBA, MOV_PRUEBA) == 0);
	// voltaje invalido
	UNIT_TEST_ASSERT(validate(TEMP_INT_PRUEBA, DEC_PRUEBA, CURR_PRUEBA, 5000, MOV_PRUEBA) == 0);
	// movimiento invalido
	UNIT_TEST_ASSERT(validate(TEMP_INT_PRUEBA, DEC_PRUEBA, CURR_PRUEBA, VOLT_PRUEBA, 5) == 0);

	UNIT_TEST_END();
}
