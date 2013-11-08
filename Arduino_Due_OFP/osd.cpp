#include <Arduino.h>
#include "osd.h"

void osd_display(float theta)
{
  
  Serial.write((short int)theta);
  
  
}
