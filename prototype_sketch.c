#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "serial_interface.h"
#include "prototype_sketch.h"

enum mode{
  MODE_OFF,
  MODE_ON,
};

enum pressSpot{
  NONE = -1,
  TOP_LEFT = 6,
  TOP_RIGHT = 3,
  BOTTOM_LEFT = 8,
  BOTTOM_RIGHT = 1
};

void setup() {DDRD = BIT5;
  DDRB &= ~(BIT5 | BIT6);

  init_serial_stdio();
  init_neopixel_blit();
}

#define DISPLAY_TIME 500 
const uint8_t white[3] = {1,1,1};
const uint8_t green[3] = {2,0,0};
const uint8_t red[3] = {0,2,0};

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

void flash(const uint8_t color[3]){
  Pixels off = { { 0 } };
  Pixels pixels = { { 0 } };
  int i;
  for (i = 0; i < 10; i++){
    cpyPixel(pixels[i], color);
  }
  for (i = 0; i < 4; i++){
    neopixel_blit(pixels);
    delay(200);
    neopixel_blit(off);
    delay(200);
  }
}

enum mode mode_on(){
  Pixels pixels = { { 0 } };
  flash(green);
  printf("mode on\n");
  int lastBlit = millis();
  int turnedOn = 0;
  while(1){
    int location;
    int time = millis();
    if (time - lastBlit >= DISPLAY_TIME){
        lastBlit = time;
	rotate(pixels, white);
	neopixel_blit(pixels);
    }
    location = getTouch();
    if (location != -1){
      if (pixels[location][0]){
	flash(red);
	return MODE_OFF;
      }
      turnedOn++;
      cpyPixel(pixels[location], white);
      neopixel_blit(pixels);
      if (turnedOn == 10){
	flash(green);
	return MODE_OFF;
      }
    }
  }
  return MODE_OFF;
}

enum mode mode_off(){
  Pixels pixels = { { 0 } };
  neopixel_blit(pixels);
  while(!buttonPressed()){
    delay(50);
  }
  printf("changing to mode on\n");
  return MODE_ON;
}

void loop() {
  enum mode mode = MODE_OFF;
  while(1){
    switch(mode){
	case MODE_OFF:
	mode = mode_off();
	break;
	case MODE_ON:
	mode = mode_on();
	break;
    }
  }
}

void cpyPixel( uint8_t dest[3], const uint8_t src[3]){
  dest[0] = src[0];
  dest[1] = src[1];
  dest[2] = src[2];
}

void rotate(Pixels pixels, uint8_t color[3]){
  uint8_t tmp[3];
  int i;
  cpyPixel(tmp, pixels[0]);
  for(i = 0; i < 9; i++){
    cpyPixel(pixels[i], pixels[i+1]);
  }
  cpyPixel(pixels[9], tmp);
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
  return 6;
}

