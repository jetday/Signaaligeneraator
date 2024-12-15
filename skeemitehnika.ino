#include <avr/io.h>
#include <avr/interrupt.h>


#define SAMPLES 14
#define DEBOUNCE_DELAY 500
#define NPN_FILTER_PIN 9 // Pin to control NPN transistor (RC filter)
#define PNP_BYPASS_PIN 10 // Pin to control PNP transistor (bypass path)


volatile uint8_t sineWave[SAMPLES];
volatile uint8_t squareWave[SAMPLES];
volatile uint8_t sawtoothWave[SAMPLES];


const uint8_t sineTable[SAMPLES] = {
  31, 45, 56, 62, 62, 56, 45, 31, 
  17, 6, 0,  0,  6,  17
};

const uint8_t sawTable[SAMPLES] = {
  8, 12, 16, 20, 24, 28, 32, 36, 
  40, 44, 48, 52,  56,  60
};

volatile uint8_t *pWaveformTable;
volatile int sampleIndex = 0;
volatile int currentFreq = 5000;
volatile int waveformType = 0;
unsigned long lastDebounceTime = 0;

void setup() {
  DDRD |= 0b11111100;
  pinMode(8, INPUT_PULLUP);
  pinMode(NPN_FILTER_PIN, OUTPUT); // Set as output for RC filter
  pinMode(PNP_BYPASS_PIN, OUTPUT); // Set as output for bypass path

  // Initialize both transistors as OFF
  digitalWrite(NPN_FILTER_PIN, LOW);
  digitalWrite(PNP_BYPASS_PIN, HIGH);

  // Precompute waveforms
  for (int i = 0; i < SAMPLES; i++) {
    sineWave[i] = (sineTable[i] & 0b11111100);
    squareWave[i] = (i < SAMPLES / 2) ? 63 & 0b11111100 : 0b00000000;
    sawtoothWave[i] = (sawTable[i] & 0b11111100);
  }
  pWaveformTable = sineWave;

  noInterrupts();
 
  TCCR1A = (1 << WGM12); // CTC mode
  TCCR1B = (1 << WGM12) | (1 << CS10); // No prescaler


  OCR1A = (16000000 / SAMPLES / currentFreq) - 1;

  TIMSK1 |= (1 << OCIE1A);
  interrupts();
  analogReference(DEFAULT);
}

ISR(TIMER1_COMPA_vect) {
  
  PORTD = (PORTD & 0b00000011) | pWaveformTable[sampleIndex];
  sampleIndex++;
  if (sampleIndex >= SAMPLES) sampleIndex = 0;
}

void loop() {
  
  int potValue = analogRead(A0);
  currentFreq = map(potValue, 0, 1023, 20, 20000);
  OCR1A = (16000000 / SAMPLES / currentFreq) - 1;
  
  
  if (digitalRead(8) == HIGH && (millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    waveformType = (waveformType + 1) % 3;
    switch (waveformType) {
    case 0:
      pWaveformTable = sineWave;
      digitalWrite(NPN_FILTER_PIN, HIGH); // Enable RC filter
      digitalWrite(PNP_BYPASS_PIN, HIGH); // Disable bypass
      break;
    case 1:
      pWaveformTable = squareWave;
      digitalWrite(NPN_FILTER_PIN, LOW); // Disable RC filter
      digitalWrite(PNP_BYPASS_PIN, LOW); // Enable bypass
      break;
    case 2:
      pWaveformTable = sawtoothWave;
      digitalWrite(NPN_FILTER_PIN, LOW); // Disable RC filter
      digitalWrite(PNP_BYPASS_PIN, LOW); // Enable bypass
      break;
  }
    lastDebounceTime = millis();
  }
  

}
