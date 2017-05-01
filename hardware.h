#if !defined(HARDWARE)
#define HARDWARE

#include <stdint.h>
#include "neopixel_blit.h"
#include "pitches.h"
#include "tones.h"

/*Functions that deal with accessing the arduino's hardware*/

enum pressSpot{
  NONE = -1,
  TOP_LEFT = 6,
  TOP_RIGHT = 3,
  BOTTOM_LEFT = 8,
  BOTTOM_RIGHT = 1
};

uint8_t white[3];
uint8_t green[3];
uint8_t red[3];
uint8_t off[3];


int buttonPressed();
int capsense(int);
int sensorToReceivePin(int sensor_id);
int sensorToNeoPixel(int sensor_id);
enum pressSpot getTouch();
void flash(const uint8_t color[3], int speed);
void setPixels(Pixels pixels, uint8_t pixel[3]);

#endif
