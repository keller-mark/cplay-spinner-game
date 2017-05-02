#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "serial_interface.h"
#include "prototype_sketch.h"
#include "hardware.h"

/*This file defines the logic assiciated with the game itself*/

/*Represents whether the game is currently running*/
enum mode{
  MODE_OFF,
  MODE_ON,
};

/*Copies a pixel value from one location to another. A pixel is uint8_t[3]*/
void cpyPixel(uint8_t dest[3], uint8_t src[3]){
  dest[0] = src[0];
  dest[1] = src[1];
  dest[2] = src[2];
}

/*Given Pixels, will rotate around clockwise.
The color variable is currently unused, but is here for a future improvement
where the color will change as the lights move faster.*/
void rotate(Pixels pixels, uint8_t color[3]){
  uint8_t tmp[3];
  int i;
  cpyPixel(tmp, pixels[0]);
  for(i = 0; i < 9; i++){
    cpyPixel(pixels[i], pixels[i+1]);
  }
  cpyPixel(pixels[9], tmp);
}

/*Defines at a high level what happens when the game is running*/
enum mode mode_on(){
  Pixels pixels = { { 0 } };/*Create empty pixels with all lights off*/
  int display_time = 800;/*This is the speed that the game starts at*/
  flash(white, display_time / 4);/*Flash to signify start of game*/
  printf("mode on\n");/*For debugging*/
  int lastBlit = millis();/*Keep track of last time that pixels moved*/
  int turnedOn = 0;/*Number of lights user has turned on so far*/
  while(1){
    int location;
    int time = millis();/*current time*/
    if (time - lastBlit >= display_time){/*Rotate lights if enough time passed*/
      lastBlit = time;
      rotate(pixels, white);/*Perform actual rotation*/
      neopixel_blit(pixels);/*Redraw display*/
    }
    location = getTouch();/*Check to see if capacitors touched*/
    if (location != -1){/*If capacative sensor was touched*/
      if (pixels[location][0]){/*Check for collisions*/
        flash(red, display_time / 4);/*Flash red if game over*/
        return MODE_OFF;/*Reset to off*/
      }
      turnedOn++;/*Keep track of the fact that a light was turned on*/
      cpyPixel(pixels[location], white);/*turn on pixel*/
      neopixel_blit(pixels);/*Redraw*/
      if (turnedOn == 10){/*If all lights turned on*/
        flash(green, display_time / 4);/*Flash green*/
        display_time -= 100;/*Make rotation faster*/
        turnedOn = 0;/*Reset turned on lights to 0*/
        setPixels(pixels, off);/*Turn off pixels*/
        continue;
      }
    }
  }
  return MODE_OFF;
}

/*This runs while arduino is in standby mode*/
enum mode mode_off(){
  Pixels pixels = { { 0 } };
  neopixel_blit(pixels);/*Turn off all pixels*/
  while(!buttonPressed()){/*Check if button pressed*/
    delay(50);
  }
  printf("changing to mode on\n");
  return MODE_ON;
}

void setup() {/*initialize things on startup*/
  init_serial_stdio();
  init_neopixel_blit();
  init_speaker();

}

void loop() {/*Continually check which mode were in and run corresponding
  function.*/
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
