#include "hardware.h"

/*Functions that deal with the logic of the game itself*/

void cpyPixel(uint8_t dest[3], uint8_t src[3]);
void rotate(Pixels pixels, uint8_t color[3]);
void setColor(uint8_t color[3]);
void checkForModeSwitch();
void delayWithCheck(int);
