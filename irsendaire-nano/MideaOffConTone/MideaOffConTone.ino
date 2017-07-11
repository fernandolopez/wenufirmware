/*
 * Ṕrueba para bypassear IRremote y enviar una señal IR usando tone() y noTone().
 * tone(pin, frequency) genera un PWM con ciclo de trabajo de 50%
 * noTone(pin) detiene el PWM anterior
 */

#define FREQ_PWM 38000
#define LED_PIN 13
#define SENIAL_MODIF 1
// 1 para usar señal modificada (que funciona con los arduinos)
// 0 para usar señal original dumpeada del remoto (no funciona con los arduinos)
#if SENIAL_MODIF
  unsigned int irSignal[] = {4200,4500, 450,1700, 450,600, 500,1650, 500,1650, 450,600, 500,550, 500,1700, 450,600, 450,600, 450,1700, 500,550, 500,550, 450,1700, 450,1700, 500,600, 450,1700, 450,600, 500,1650, 500,1650, 450,1700, 500,1650, 500,600, 450,1700, 450,1700, 450,1700, 450,600, 450,600, 500,550, 450,600, 500,1700, 450,600, 450,600, 450,1700, 450,1700, 500,1650, 450,600, 500,550, 500,600, 450,600, 450,600, 450,600, 500,550, 500,600, 450,1700, 450,1700, 450,1700, 450,1700, 500,1650, 550,4500, 4250,4500, 450,1700, 450,600, 450,1700, 500,1650, 500,550, 450,600, 500,1700, 450,600, 450,600, 500,1650, 500,550, 500,550, 500,1700, 450,1700, 450,600, 500,1650, 500,550, 450,1700, 500,1650, 500,1700, 450,1700, 450,600, 500,1650, 500,1650, 450,1700, 450,600, 500,600, 450,600, 450,600, 500,1650, 500,550, 450,600, 500,1700, 450,1700, 450,1700, 500,550, 500,550, 450,600, 500,600, 450,600, 450,600, 500,550, 500,550, 500,1650, 500,1700, 450,1700, 450,1700, 450,1700, 550};
#else
  unsigned int irSignal[] = {4400,4350, 600,1550, 550,550, 550,1600, 550,1600, 550,500, 600,500, 600,1550, 550,550, 550,500, 600,1550, 600,500, 550,550, 550,1550, 600,1600, 550,500, 550,1600, 600,500, 550,1600, 550,1600, 550,1600, 550,1600, 600,500, 550,1600, 550,1600, 550,1600, 550,500, 600,500, 600,450, 600,500, 600,1550, 600,500, 600,450, 600,1600, 550,1550, 600,1600, 550,500, 550,550, 550,500, 600,500, 600,500, 550,500, 600,500, 550,500, 600,1550, 600,1600, 550,1550, 600,1600, 550,1550, 600};
#endif
unsigned int size = sizeof(irSignal)/sizeof(irSignal[0]);

void setup() {
  pinMode(LED_PIN, OUTPUT);      // sets the digital pin as output
}
void loop() {
  for (int i = 0; i < size; i += 2) {
    tone(LED_PIN, FREQ_PWM);  // LED on (IR mark)
    delayMicroseconds(irSignal[i]);
    noTone(LED_PIN);  // LED off (IR space)
    delayMicroseconds(irSignal[i+1]);
  }
  digitalWrite(LED_PIN, LOW);  // terminar con LED off por las dudas
  delay(1000);
}
