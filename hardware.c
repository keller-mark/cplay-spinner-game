#include "hardware.h"
#include "pitches.h"
#include "tones.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "serial_interface.h"
#include "prototype_sketch.h"

#define BIT5 (1<<5)
#define BIT6 (1<<6)

// #define CPLAY_REDLED 13
// #define CPLAY_NEOPIXELPIN 17
// #define CPLAY_SLIDESWITCHPIN 21
#define CPLAY_LEFTBUTTON 4
#define CPLAY_RIGHTBUTTON 19
// #define CPLAY_LIGHTSENSOR A5
// #define CPLAY_LIS3DH_CS 8
// #define CPLAY_THERMISTORPIN A0
// #define CPLAY_SOUNDSENSOR A4
#define CPLAY_BUZZER 5
#define CPLAY_CAPSENSE_SHARED 30

#define PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint8_t
#define DIRECT_READ(base, mask)         (((*(base)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)   ((*((base)+1)) &= ~(mask), (*((base)+2)) &= ~(mask))
#define DIRECT_MODE_OUTPUT(base, mask)  ((*((base)+1)) |= (mask))
#define DIRECT_WRITE_LOW(base, mask)    ((*((base)+2)) &= ~(mask))
#define DIRECT_WRITE_HIGH(base, mask)   ((*((base)+2)) |= (mask))

uint8_t white[3] = {1,1,1};
uint8_t green[3] = {2,0,0};
uint8_t red[3] = {0,2,0};
uint8_t off[3] = {0,0,0};

enum pressSpot getTouch(){
  /* if in capacitive mode, turn on neopixels with touch input */
  int sensor_id, receivePin;
  /* check each of the 8 capacitive sensors for touch input */
  for(sensor_id = 0; sensor_id < 8; sensor_id++) {
    if(capsense(sensor_id)) {
      return sensorToNeoPixel(sensor_id);
    }
  }
  return NONE;
}

void setPixels(Pixels pixels, uint8_t pixel[3]){
  int i;
  for (i = 0; i < 10; i++){
    cpyPixel(pixels[i], pixel);
  }
}

void flash(const uint8_t color[3], int speed){
  Pixels off = { { 0 } };
  Pixels pixels = { { 0 } };
  int i;
  setPixels(pixels, color);
  for (i = 0; i < 4; i++){
    neopixel_blit(pixels);
    if(color == white) {
      speaker_tone(NOTE_A4, 40);
    } else if(color == red) {
      speaker_tone(NOTE_A5/pow(2, i), 40);
    } else {
      speaker_tone(NOTE_A3*pow(2, i), 40);
    }
    delay(speed - 40);
    neopixel_blit(off);
    delay(speed);
  }
}

int buttonPressed() {
  /* read the button state */
  int left = digitalRead(CPLAY_LEFTBUTTON);
  int right = digitalRead(CPLAY_RIGHTBUTTON);
  return left || right;
}

unsigned long sensorIds[] = {0,0,0,0,0,0,0,0,0};

int capsense(int sensorId) {
  /* adapted from both the cap.c worksheet and CPlay_CapacitiveSensor */

  /* a counter for how many iterations until the cap discharges */
  unsigned int cnt = 0;
  int receivePin = sensorToReceivePin(sensorId);

  IO_REG_TYPE sBit;   /* send pin's ports and bitmask */
  volatile IO_REG_TYPE *sReg;
  IO_REG_TYPE rBit;    /* receive pin's ports and bitmask */
  volatile IO_REG_TYPE *rReg;

  pinMode(CPLAY_CAPSENSE_SHARED, OUTPUT);
  pinMode(receivePin, INPUT);
  digitalWrite(CPLAY_CAPSENSE_SHARED, LOW);

  sBit = PIN_TO_BITMASK(CPLAY_CAPSENSE_SHARED); /* get send pin's ports and bitmask */
  sReg = PIN_TO_BASEREG(CPLAY_CAPSENSE_SHARED); /* get pointer to output register */

  rBit = PIN_TO_BITMASK(receivePin);	  /* get receive pin's ports and bitmask */
  rReg = PIN_TO_BASEREG(receivePin);

  noInterrupts();
  DIRECT_WRITE_LOW(sReg, sBit);   /* sendPin Register low */
  DIRECT_MODE_INPUT(rReg, rBit);  /* receivePin to input (pullups are off) */
  DIRECT_MODE_OUTPUT(rReg, rBit); /* receivePin to OUTPUT */
  DIRECT_WRITE_LOW(rReg, rBit);   /* pin is now LOW AND OUTPUT */
  delayMicroseconds(10);
  DIRECT_MODE_INPUT(rReg, rBit);  /* receivePin to input (pullups are off) */
  DIRECT_WRITE_HIGH(sReg, sBit);  /* sendPin High */
  interrupts();

  while ( !DIRECT_READ(rReg, rBit) && cnt < 5 ) {
    /* while receive pin is LOW AND total is positive value */
    cnt++;
  }

  /* if the sensor is being touched, return true */
  if(cnt > 4) {
    int index = sensorToNeoPixel(sensorId);
    unsigned long old = sensorIds[index];
    unsigned long new = millis();
    sensorIds[index] = new;
    if (new - old > 100){
      return 1;
    }
  }
  return 0;
}

int sensorToReceivePin(int sensor_id) {
  if(sensor_id < 4) {
    return sensor_id;
  }
  switch(sensor_id) {
    case 4:
    return 6;
    case 5:
    return 9;
    case 6:
    return 10;
    case 7:
    return 12;
    default:
    return sensor_id;
  }
}

enum pressSpot sensorToNeoPixel(int sensor_id) {
  if(sensor_id == 2 || sensor_id == 3) {
    return BOTTOM_RIGHT;
  }
  if(sensor_id == 0 || sensor_id == 1) {
    return TOP_RIGHT;
  }
  if(sensor_id == 5 || sensor_id == 6) {
    return BOTTOM_LEFT;
  }
  return TOP_LEFT;
}
