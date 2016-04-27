#include <Servo.h>
#include <FiveChannel.h>

void setup()
{
  fiveChannelInitDue();

  Serial.begin(9600); 
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }    
  
}

void loop()
{
  fiveChannelUpdate();
}

void modifyChannels(int & chan1, int & chan2, int & chan3, int & chan4, int & chan5) {
    
  char buf[100];
  sprintf(buf, "%04d %04d %04d %04d %04d", chan1, chan2, chan3, chan4, chan5);
  Serial.println(buf);
  
}

