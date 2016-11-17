/* Programa en C para probar cosas antes de ponerlas en el unit test format_message_correcto 
(para no tener que probarlo en la placa o sin tener el resto de las cosas andando) */

#include <string.h>
#include <stdio.h>

#define ID_MOTA "linti_cocina"

static char fmt_mensaje[] = "{"\
                             "\"mote_id\":\"%s\","\
                             "\"temperature\":%u.%u,"\
                             "\"current\":%u,"\
                             "\"movement\":%u,"\
                             "\"voltage\":%u"\
                             "}";
static char mensaje[sizeof(fmt_mensaje) - 8 + 4 + 4 + 1 + 1 + 4];

const char *format_message(const char *mote_id, int temp_deg, int temp_dec, int current, int movement, int voltage){
    /** Recibe una serie de valores y returna un puntero a una variable global
     * con el string del mensaje json formado */
    snprintf(mensaje, sizeof(mensaje), fmt_mensaje, mote_id, temp_deg, temp_dec,
            current, movement, voltage);
    return mensaje;
}

#define STRING_PRUEBA "string_prueba"
// #define TEMP_PRUEBA 2
// #define DEC_PRUEBA 3
// #define CURR_PRUEBA 4
// #define MOV_PRUEBA 0
// #define VOLT_PRUEBA 1

int main() {
	const char * stringFormatMessage = format_message(STRING_PRUEBA, 2, 3, 4, 0, 1);

	const char * stringUnitTest =	"{"							\
							"\"mote_id\":\""STRING_PRUEBA"\","		\
							"\"temperature\":2.3,"	\
							"\"current\":4,"			\
							"\"movement\":0,"			\
							"\"voltage\":1"			\
							"}";

	printf("\n");
	printf("stringFormatMessage: %s\n", stringFormatMessage);
	printf("stringUnitTest: %s\n", stringUnitTest);
	printf("Son iguales? ");
	if (strcmp(stringFormatMessage, stringUnitTest) == 0) {
		printf("Si\n");
	} else {
		printf("No\n");
	}

	return 0;
}