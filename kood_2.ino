#include <avr/io.h>
#include <avr/interrupt.h>

#define BITS 4
#define SAMPLES 16
#define DEBOUNCE_DELAY 500
#define NPN_FILTER_PIN 9 //Pin to control NPN transistor (RC filter)
#define PNP_BYPASS_PIN 10 //Pin to control PNP transistor (bypass path)

volatile uint8_t sineWave[SAMPLES];
volatile uint8_t squareWave[SAMPLES];
volatile uint8_t sawtoothWave[SAMPLES];

const uint8_t sineTable[SAMPLES] = {
  32, 43, 53, 60, 63, 60, 53, 43, 
  32, 21, 11, 4,  0,  4, 11, 21
};

volatile uint8_t *pWaveformTable;
volatile int sampleIndex = 0;
volatile int currentFreq = 20000;
volatile int waveformType = 0;
volatile int delayPerSample = 1;
unsigned long lastDebounceTime = 0;

void setup() {
  DDRD |= 0b11111100; 
  pinMode(8, INPUT_PULLUP); 
  pinMode(NPN_FILTER_PIN, OUTPUT); 
  pinMode(PNP_BYPASS_PIN, OUTPUT); 

  //Initialize both transistors as OFF
  digitalWrite(NPN_FILTER_PIN, LOW);
  digitalWrite(PNP_BYPASS_PIN, HIGH);
  
  //Precompute waveforms
  for (int i = 0; i < SAMPLES; i++) {
    sineWave[i] = (sineTable[i] & 0b11111100);
    squareWave[i] = (i < SAMPLES / 2) ? 63 & 0b11111100 : 0b00000000;
    sawtoothWave[i] = (i * 4 & 0b11111100);
  }
  pWaveformTable = sineWave;
  
  noInterrupts();
  
  TCCR1A = 0;                        
  TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10); 
  OCR1A = 15624;                     
  
  TIMSK1 |= (1 << OCIE1A);    
  interrupts();
  
  analogReference(DEFAULT);
  
}


ISR(TIMER1_COMPA_vect) {

  int potValue = analogRead(A0);
  currentFreq = map(potValue, 0, 1023, 1000, 1);

  
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

void loop() {
  
  PORTD = (PORTD & 0b00000011) | pWaveformTable[sampleIndex];
  sampleIndex++;
  if (sampleIndex >= SAMPLES) sampleIndex = 0;

  
  delayMicroseconds(currentFreq);
}
