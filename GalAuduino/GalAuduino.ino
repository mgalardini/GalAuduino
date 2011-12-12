// Auduino, the Lo-Fi granular synthesiser
//
// by Peter Knight, Tinker.it http://tinker.it
// (with some hacks by galactus)
//
// Help:      http://code.google.com/p/tinkerit/wiki/Auduino
// More help: http://groups.google.com/group/auduino
//
// Analog in 0: Grain 1 pitch
// Analog in 1: Grain 2 decay
// Analog in 2: Grain 1 decay
// Analog in 3: Grain 2 pitch
// Analog in 4: Grain repetition frequency
//
// Digital 3: Audio out (Digital 11 on ATmega8)
//
// Changelog:
// 19 Nov 2008: Added support for ATmega8 boards
// 21 Mar 2009: Added support for ATmega328 boards
// 7 Apr 2009: Fixed interrupt vector for ATmega328 boards
// 8 Apr 2009: Added support for ATmega1280 boards (Arduino Mega)

// Added by galactus on November 2011:
// On Arduino UNO the sound is coming out from pin 3 and 11
// A button to mute the whole thing (and a led to signal it)
// A potenziometer to switch from steady state to beat-box state, with different frequencies
//   (the same led as the "mute" one will blink to show the beat speed)
// A button to insert some pauses when the beat-box mode is on (and a led to signal it)
//  the rythm is the following: ..-...-...-..- (. = beat, - = max value)
// The various syncs are borrowed from Dave's auduino (http://blog.lewissykes.info/daves-auduino/)
// Just uncomment the one you like

#include <avr/io.h>
#include <avr/interrupt.h>

uint16_t syncPhaseAcc;
uint16_t syncPhaseInc;
uint16_t grainPhaseAcc;
uint16_t grainPhaseInc;
uint16_t grainAmp;
uint8_t grainDecay;
uint16_t grain2PhaseAcc;
uint16_t grain2PhaseInc;
uint16_t grain2Amp;
uint8_t grain2Decay;

// Map Analogue channels
#define SYNC_CONTROL         (0)//(4)
#define GRAIN_FREQ_CONTROL   (1)//(0)
#define GRAIN_DECAY_CONTROL  (2)//(2)
#define GRAIN2_FREQ_CONTROL  (4)//(3)
#define GRAIN2_DECAY_CONTROL (3)//(1)

// Button to add a rythm when the delay is on
#define BUTTON 10
#define LED_SWTCH 12
int rythm = 0;
int tap = 0;
// Button to oscillate the whole thing
#define MUTE 9
#define LED_MUTE 6
// Pot the set the oscillation frequency
#define OSCILLATOR 5
int oscillatorFreq;

// Debug
#define BAUD 9600

// Changing these will also requires rewriting audioOn()

#if defined(__AVR_ATmega8__)
//
// On old ATmega8 boards.
//    Output is on pin 11
//
#define LED_PIN       13
#define LED_PORT      PORTB
#define LED_BIT       5
#define PWM_PIN       11
#define PWM_PIN2       6
#define PWM_VALUE     OCR2
#define PWM_INTERRUPT TIMER2_OVF_vect
#elif defined(__AVR_ATmega1280__)
//
// On the Arduino Mega
//    Output is on pin 3
//
#define LED_PIN       13
#define LED_PORT      PORTB
#define LED_BIT       7
#define PWM_PIN       3
#define PWM_PIN2      5
#define PWM_VALUE     OCR3C
#define PWM_INTERRUPT TIMER3_OVF_vect
#else
//
// For modern ATmega168 and ATmega328 boards
//    Output is on pin 3
//
#define PWM_PIN       3
#define PWM_PIN2      11
#define PWM_VALUE     OCR2B
#define LED_PIN       13
#define LED_PORT      PORTB
#define LED_BIT       5
#define PWM_INTERRUPT TIMER2_OVF_vect
#endif

// Smooth logarithmic mapping
//
uint16_t antilogTable[] = {
  64830,64132,63441,62757,62081,61413,60751,60097,59449,58809,58176,57549,56929,56316,55709,55109,
  54515,53928,53347,52773,52204,51642,51085,50535,49991,49452,48920,48393,47871,47356,46846,46341,
  45842,45348,44859,44376,43898,43425,42958,42495,42037,41584,41136,40693,40255,39821,39392,38968,
  38548,38133,37722,37316,36914,36516,36123,35734,35349,34968,34591,34219,33850,33486,33125,32768
};
uint16_t mapPhaseInc(uint16_t input) {
  return (antilogTable[input & 0x3f]) >> (input >> 6);
}

// Stepped chromatic mapping
//
uint16_t midiTable[] = {
  17,18,19,20,22,23,24,26,27,29,31,32,34,36,38,41,43,46,48,51,54,58,61,65,69,73,
  77,82,86,92,97,103,109,115,122,129,137,145,154,163,173,183,194,206,218,231,
  244,259,274,291,308,326,346,366,388,411,435,461,489,518,549,581,616,652,691,
  732,776,822,871,923,978,1036,1097,1163,1232,1305,1383,1465,1552,1644,1742,
  1845,1955,2071,2195,2325,2463,2610,2765,2930,3104,3288,3484,3691,3910,4143,
  4389,4650,4927,5220,5530,5859,6207,6577,6968,7382,7821,8286,8779,9301,9854,
  10440,11060,11718,12415,13153,13935,14764,15642,16572,17557,18601,19708,20879,
  22121,23436,24830,26306
};
uint16_t mapMidi(uint16_t input) {
  return (midiTable[(1023-input) >> 3]);
}

// Stepped Pentatonic mapping
//
uint16_t pentatonicTable[54] = {
  0,19,22,26,29,32,38,43,51,58,65,77,86,103,115,129,154,173,206,231,259,308,346,
  411,461,518,616,691,822,923,1036,1232,1383,1644,1845,2071,2463,2765,3288,
  3691,4143,4927,5530,6577,7382,8286,9854,11060,13153,14764,16572,19708,22121,26306
};

uint16_t mapPentatonic(uint16_t input) {
  uint8_t value = (1023-input) / (1024/53);
  return (pentatonicTable[value]);
}

// Lewis added - I've got an Excel spreadsheet with these workings out on my blog...
// Stepped major Diatonic mapping
//
uint16_t majordiatonicTable[76] = {
  0,17,19,22,23,26,29,32,34,38,43,46,51,58,65,69,77,86,92,103,115,129,137,154,173,183,206,231,259,274,308,346,366,
  411,461,518,549,616,691,732,822,923,1036,1097,1232,1383,1465,1644,1845,2071,2195,2463,2765,2930,3288,
  3691,4143,4389,4927,5530,5859,6577,7382,8286,8779,9854,11060,11718,13153,14764,16572,17557,19708,22121,23436,26306
};

uint16_t mapmajorDiatonic(uint16_t input) {
  uint8_t value = (1023-input) / (1024/53);
  return (majordiatonicTable[value]);
}

// Stepped minor Diatonic mapping
//
uint16_t minordiatonicTable[76] = {
  0,17,19,20,23,26,27,31,34,38,41,46,51,54,61,69,77,82,92,103,109,122,137,154,163,183,206,218,244,274,308,326,366,
  411,435,489,549,616,652,732,822,871,978,1097,1232,1305,1465,1644,1742,1955,2195,2463,2610,2930,3288,
  3484,3910,4389,4927,5220,5859,6577,6968,7821,8779,9854,10440,11718,13153,13935,15642,17557,19708,20879,23436,26306
};

uint16_t mapminorDiatonic(uint16_t input) {
  uint8_t value = (1023-input) / (1024/53);
  return (minordiatonicTable[value]);
}

// Stepped major Pentatonic mapping
//
uint16_t majorpentatonicTable[55] = {
  0,17,19,22,26,29,34,38,43,51,58,69,77,86,103,115,137,154,173,206,231,274,308,346,
  411,461,549,616,691,822,923,1097,1232,1383,1644,1845,2195,2463,2765,3288,
  3691,4389,4927,5530,6577,7382,8779,9854,11060,13153,14764,17557,19708,22121,26306
};

uint16_t mapmajorPentatonic(uint16_t input) {
  uint8_t value = (1023-input) / (1024/53);
  return (majorpentatonicTable[value]);
}

// Stepped minor Pentatonic mapping
//
uint16_t minorpentatonicTable[55] = {
  0,17,20,23,26,31,34,41,46,51,61,69,82,92,103,122,137,163,183,206,244,274,326,366,
  411,489,549,652,732,822,978,1097,1305,1465,1644,1955,2195,2610,2930,3288,
  3910,4389,5220,5859,6577,7821,8779,10440,11718,13153,15642,17557,20879,23436,26306
};

uint16_t mapminorPentatonic(uint16_t input) {
  uint8_t value = (1023-input) / (1024/53);
  return (pentatonicTable[value]);
}
// MAPPINGS - END


void audioOn() {
#if defined(__AVR_ATmega8__)
  // ATmega8 has different registers
  TCCR2 = _BV(WGM20) | _BV(COM21) | _BV(CS20);
  TIMSK = _BV(TOIE2);
#elif defined(__AVR_ATmega1280__)
  TCCR3A = _BV(COM3C1) | _BV(WGM30);
  TCCR3B = _BV(CS30);
  TIMSK3 = _BV(TOIE3);
#else
  // Set up PWM to 31.25kHz, phase accurate
  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  TIMSK2 = _BV(TOIE2);
#endif
}


void setup() {
  // Setup the signaling led
  pinMode(LED_SWTCH, OUTPUT);
  pinMode(LED_MUTE, OUTPUT);
  // Setup the button
  pinMode(BUTTON, INPUT);
  pinMode(MUTE, INPUT);
  
  pinMode(PWM_PIN,OUTPUT);
  pinMode(PWM_PIN2,OUTPUT);
  audioOn();
  pinMode(LED_PIN,OUTPUT);
  
  digitalWrite(LED_MUTE, HIGH);
  
  // Setup the serial port
  //Serial.begin(BAUD);
}

int oldButton = LOW;
int nowButton = LOW;

int oldMute = LOW;
int nowMute = LOW;
int oscillate = 0;
int beat = 0;
int mute = 1;

void loop() {
  // The loop is pretty simple - it just updates the parameters for the oscillators.
  //
  // Avoid using any functions that make extensive use of interrupts, or turn interrupts off.
  // They will cause clicks and poops in the audio.
  
  // Mute
  nowMute = digitalRead(MUTE);
   
   // Anti-bouncing, anti-interferences step
   if(nowMute == HIGH &&
   oldMute == LOW)
   {
     delay(10);
   }
   
   if (nowMute == HIGH &&
    oldMute == HIGH)
   { 
     // The button has been pressed
     if (oscillate == 0) {
       oscillate = 1;
       mute = 0;
       digitalWrite(LED_MUTE, LOW);
     }
     else {
       oscillate = 0;
       mute = 1;
       digitalWrite(LED_MUTE, HIGH);
     }
     delay(300);
   }
   oldMute = nowMute;
   
  // Switch implementation
  nowButton = digitalRead(BUTTON);
   
   // Anti-bouncing, anti-interferences step
   if(nowButton == HIGH &&
   oldButton == LOW)
   {
   delay(10);
   }
   
   if (nowButton == HIGH &&
    oldButton == HIGH)
   {
     if (rythm == 0){
       rythm = 1;
       tap = 0;
       digitalWrite(LED_SWTCH, HIGH);
     }
     else{
       rythm = 0;
       tap = 0;
       digitalWrite(LED_SWTCH, LOW);
     }
   }
   oldButton = nowButton;
  
  // Oscillate
  if (oscillate == 1 && oscillatorFreq != 0) {
    if (beat == 0) {
      beat = 1;
      digitalWrite(LED_MUTE, HIGH);
    }
    else{
      beat = 0;
      if (rythm == 1){
        tap = tap + 1;
        if (tap == 14){
          tap = 0;
        }
      }
      digitalWrite(LED_MUTE, LOW);
    }
    if (rythm == 1){
      if(tap == 2 || tap == 6 || tap == 10 || tap == 13){
        digitalWrite(LED_SWTCH, LOW);
      }
      else{digitalWrite(LED_SWTCH, HIGH);}
    }
    delay(oscillatorFreq);
  }
  
  syncPhaseInc = mapminorPentatonic(analogRead(SYNC_CONTROL));
  // Rythm!
  if(tap == 2 || tap == 6 || tap == 10 || tap == 13){
    syncPhaseInc = 1023;
  }
  //syncPhaseInc = mapmajorPentatonic(analogRead(SYNC_CONTROL));
  //syncPhaseInc = mapminorDiatonic(analogRead(SYNC_CONTROL));
  //syncPhaseInc = mapmajorDiatonic(analogRead(SYNC_CONTROL));
  //syncPhaseInc = mapMidi(analogRead(SYNC_CONTROL));
  //syncPhaseInc = mapPhaseInc(analogRead(SYNC_CONTROL)) / 4;

  grainPhaseInc  = mapPhaseInc(analogRead(GRAIN_FREQ_CONTROL)) / 2;
  grainDecay     = analogRead(GRAIN_DECAY_CONTROL) / 8;
  grain2PhaseInc = mapPhaseInc(analogRead(GRAIN2_FREQ_CONTROL)) / 2;
  grain2Decay    = analogRead(GRAIN2_DECAY_CONTROL) / 4;
  
  oscillatorFreq = analogRead(OSCILLATOR) / 8;
  if (oscillatorFreq > 24){
    oscillatorFreq += 5;
    if (oscillatorFreq > 165){
      oscillatorFreq = 165;
    }
  }
  else{
    oscillatorFreq = 0;
    beat = 0;
    if (mute == 0){
      digitalWrite(LED_MUTE, LOW);
    }
    
  }
}

SIGNAL(PWM_INTERRUPT)
{ 
  uint8_t value;
  uint16_t output;

  syncPhaseAcc += syncPhaseInc;
  if (syncPhaseAcc < syncPhaseInc) {
    // Time to start the next grain
    grainPhaseAcc = 0;
    grainAmp = 0x7fff;
    grain2PhaseAcc = 0;
    grain2Amp = 0x7fff;
    LED_PORT ^= 1 << LED_BIT; // Faster than using digitalWrite
  }
  
  // Increment the phase of the grain oscillators
  grainPhaseAcc += grainPhaseInc;
  grain2PhaseAcc += grain2PhaseInc;

  // Convert phase into a triangle wave
  value = (grainPhaseAcc >> 7) & 0xff;
  if (grainPhaseAcc & 0x8000) value = ~value;
  // Multiply by current grain amplitude to get sample
  output = value * (grainAmp >> 8);

  // Repeat for second grain
  value = (grain2PhaseAcc >> 7) & 0xff;
  if (grain2PhaseAcc & 0x8000) value = ~value;
  output += value * (grain2Amp >> 8);

  // Make the grain amplitudes decay by a factor every sample (exponential decay)
  grainAmp -= (grainAmp >> 8) * grainDecay;
  grain2Amp -= (grain2Amp >> 8) * grain2Decay;

  // Scale output to the available range, clipping if necessary
  output >>= 9;
  if (output > 255) output = 255;

  // Do nothing if we are in the mute state or making the beat box
  if (mute == 1 || beat ==1){
    return;
  }

  // Output to PWM (this is faster than using analogWrite)  
  PWM_VALUE = output;
}
