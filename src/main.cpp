#include <Arduino.h>

/******************************************************************************
LED Controller
Mitch Berkson

Teensy 3.2

*****************************************************************************/
#define WHITE_LED_PIN 6
#define RED_LED_PIN 5
#define PUSHBUTTON_L_PIN 2

// PWM frequency in Hz
// This is a tradeoff between ripple current in the injector and
// switching losses in the MOSFET and diode.  At 20KHz, those were getting
// warm.
const uint16_t PWM_FREQ_HZ = 1000;

const uint16_t LED_OFF = 0;
const uint16_t LED_MAX = 256;

//---------------------- setup -------------------------------------------------
void setup() {

  pinMode(WHITE_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(PUSHBUTTON_L_PIN, INPUT);

  analogWriteFrequency(WHITE_LED_PIN, PWM_FREQ_HZ);

  analogWrite(WHITE_LED_PIN, LED_OFF);
  digitalWriteFast(RED_LED_PIN, LED_OFF);

}
unsigned long prevMillis = 0;
const uint16_t brightnessInc = 1;  // brightness increment
uint16_t whiteLevel = 0;  // current brightness
const uint16_t minBrightness = 0;
const uint16_t maxBrightness = 50;
const uint16_t timeIncrement = 100;  // milliseconds
uint16_t currentBrightness;
bool redOn = false;
const uint16_t redOnTime = 1000;

uint16_t bump_brightness(uint16_t inc) {
  whiteLevel += inc;
  if (whiteLevel >= maxBrightness) {
    whiteLevel = minBrightness;
  }
  return whiteLevel;
}

//----------------------- loop --------------------------------------------------
void loop() {

  unsigned long currentMillis;
  uint8_t pbVal;

  currentMillis = millis();

  pbVal = digitalReadFast(PUSHBUTTON_L_PIN);

  if (pbVal == LOW) {
    if (redOn) {
      if ((currentMillis - prevMillis) > redOnTime) {
        digitalWriteFast(RED_LED_PIN, 0);
        redOn = false;
        currentBrightness = bump_brightness(brightnessInc);
        analogWrite(WHITE_LED_PIN, currentBrightness);
        prevMillis = currentMillis;
      }
    } else {
      if ((currentMillis - prevMillis) > timeIncrement) {
        currentBrightness = bump_brightness(brightnessInc);
        analogWrite(WHITE_LED_PIN, currentBrightness);
        prevMillis = currentMillis;
      }
      if (currentBrightness == minBrightness) { // turn on red
        digitalWriteFast(RED_LED_PIN, 1);
        redOn = true;
      }
    }
  }
}
