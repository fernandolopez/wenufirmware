#include "contiki.h"
#include "unit-test.h"
#include "unit-test.c"

#include "unit-tests.c"

/*---------------------------------------------------------------------------*/
PROCESS(unit_tests_clima_mqtt, "Unit Tests Clima MQTT");
AUTOSTART_PROCESSES(&unit_tests_clima_mqtt);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(unit_tests_clima_mqtt, ev, data) {
	PROCESS_BEGIN();

	UNIT_TEST_RUN(format_message_correcto);

	PROCESS_END();
}