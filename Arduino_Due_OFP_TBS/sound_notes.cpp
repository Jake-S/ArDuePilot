#include <Arduino.h>
#include "sound_notes.h"

void SoundNoTimer(int _pin, unsigned long duration,  unsigned int frequency)
{
  // Try to get something working on DUE...
  long toggle_count = 0;
  long lusDelayPerHalfCycle;
  boolean fHigh = false;
  // Set the pinMode as OUTPUT
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  toggle_count = 2 * frequency * duration / 1000;
  lusDelayPerHalfCycle = 1000000L/(frequency * 2);

  // if we are using an 8 bit timer, scan through prescalars to find the best fit
  while (toggle_count--) {
    // toggle the pin
    fHigh  = !fHigh;
    digitalWrite(_pin, fHigh? LOW : HIGH);
    // delay a half cycle
    delayMicroseconds(lusDelayPerHalfCycle);
  }    
  digitalWrite(_pin, LOW);
}
