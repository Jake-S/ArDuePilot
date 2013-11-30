#include "pins_map.h"
#include <Arduino.h>



void controlLEDs(int green_led, int yellow_led, int red_led)
{
  if(green_led > .5)
    digitalWrite(GREEN_LED, HIGH);
  else
    digitalWrite(GREEN_LED, LOW);

  if(yellow_led > .5)
    digitalWrite(YELLOW_LED, HIGH);
  else
    digitalWrite(YELLOW_LED, LOW);

  if(red_led > .5)
    digitalWrite(RED_LED, HIGH);
  else
    digitalWrite(RED_LED, LOW);


}

