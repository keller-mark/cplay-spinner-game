#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "serial_interface.h"
#include "prototype_sketch.h"
#include "hardware.h"

enum mode{
  MODE_OFF,
  MODE_ON,
};

void cpyPixel(uint8_t dest[3], uint8_t src[3]){
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

enum mode mode_on(){
  Pixels pixels = { { 0 } };
  int display_time = 800;
  flash(white, display_time / 4);
  printf("mode on\n");
  int lastBlit = millis();
  int turnedOn = 0;
  while(1){
    int location;
    int time = millis();
    if (time - lastBlit >= display_time){
      lastBlit = time;
      rotate(pixels, white);
      neopixel_blit(pixels);
    }
    location = getTouch();
    if (location != -1){
      if (pixels[location][0]){
        flash(red, display_time / 4);
        return MODE_OFF;
      }
      turnedOn++;
      cpyPixel(pixels[location], white);
      neopixel_blit(pixels);
      if (turnedOn == 10){
        flash(green, display_time / 4);
        display_time -= 100;
        turnedOn = 0;
        setPixels(pixels, off);
        continue;
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

void setup() {
  init_serial_stdio();
  init_neopixel_blit();
  init_speaker();

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
