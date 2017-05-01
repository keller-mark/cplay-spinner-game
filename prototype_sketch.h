#include <stdint.h>
#include "neopixel_blit.h"
#include "pitches.h"
#include "tones.h"

#define BIT5 (1<<5)
#define BIT6 (1<<6)

#define CPLAY_REDLED 13
#define CPLAY_NEOPIXELPIN 17
#define CPLAY_SLIDESWITCHPIN 21
#define CPLAY_LEFTBUTTON 4
#define CPLAY_RIGHTBUTTON 19
#define CPLAY_LIGHTSENSOR A5
#define CPLAY_LIS3DH_CS 8
#define CPLAY_THERMISTORPIN A0
#define CPLAY_SOUNDSENSOR A4
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

void cpyPixel(uint8_t dest[3], uint8_t src[3]);
void rotate(Pixels pixels, uint8_t color[3]);
void setColor(uint8_t color[3]);
void checkForModeSwitch();
void delayWithCheck(int);
int buttonPressed();
int capsense(int);
int sensorToReceivePin(int sensor_id);
int sensorToNeoPixel(int sensor_id);
