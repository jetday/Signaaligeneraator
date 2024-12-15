#include <avr/io.h>
#include <avr/interrupt.h>

#define BITS 4
#define SAMPLES 14
#define DEBOUNCE_DELAY 500
#define NPN_FILTER_PIN 9 //Pin to control NPN transistor
#define PNP_BYPASS_PIN 10 //Pin to control PNP transistor

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
  pinMode(NPN_FILTER_PIN, OUTPUT); 
  pinMode(PNP_BYPASS_PIN, OUTPUT); 

  //Initialize both transistors as OFF
  digitalWrite(NPN_FILTER_PIN, LOW);
  digitalWrite(PNP_BYPASS_PIN, HIGH);
  
  //Precompute waveforms
  for (int i = 0; i < SAMPLES; i++) {
    sineWave[i] = (sineTable[i] & 0b11111100);
    squareWave[i] = (i < SAMPLES / 2) ? 63 & 0b11111100 : 0b00000000;
    sawtoothWave[i] = (sawTable[i] & 0b11111100);
  }
  pWaveformTable = sineWave;
  
  noInterrupts();
  
  TCCR1A = 0;                        
  TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10); 
  OCR1A = 15624; //Interrutps roughly every second                   
  
  TIMSK1 |= (1 << OCIE1A);    
  interrupts();
  
  analogReference(DEFAULT);
  
}


ISR(TIMER1_COMPA_vect) {
  //Change frequency
  int potValue = analogRead(A0);
  currentFreq = map(potValue, 0, 1023, 1000, 1);

  //Change waveform
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
  
  //Generate signal
  PORTD = (PORTD & 0b00000011) | pWaveformTable[sampleIndex];
  sampleIndex++;
  if (sampleIndex >= SAMPLES) sampleIndex = 0;

  
  delayMicroseconds(currentFreq);
}
