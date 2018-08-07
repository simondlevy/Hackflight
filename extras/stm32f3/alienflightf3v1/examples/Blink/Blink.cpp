/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */

#include <main.h>

void setup() {                

    ledInit();
}

// the loop routine runs over and over again forever:
void loop() {

    ledSet(true);  
    delay(1000);  
    ledSet(false);
    delay(1000); 
}
