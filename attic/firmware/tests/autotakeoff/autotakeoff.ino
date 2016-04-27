#include <Servo.h>
#include <FiveChannel.h>

void setup()
{  
  fiveChannelInitMega();  

  Serial.begin(9600);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
}

void loop() {
  
  fiveChannelUpdate();
}

void modifyChannels(int & chan1Val,  int & chan2Val,  int & chan3Val, int & chan4Val,  int & chan5Val) {
  
  static unsigned long timePrev;
  static int throttle;
  
  unsigned long time = millis();
  
  if (!timePrev) {
    
    timePrev = time;
  }
    
  if (chan5Val > 1000) {
   
   if (throttle < 1500) {
        
      if ((time - timePrev) > 10) {
        
        throttle++;
        timePrev = time;
      }
    }
  }
  
  else {
    
    throttle = chan3Val;
  }
      
  chan3Val = throttle;
  
  Serial.println(chan3Val);
  
  //char buf[100];
  //sprintf(buf, "%04d %04d %04d %04d %04d", chan1Val, chan2Val, chan3Val, chan4Val, chan5Val);
  //Serial.println(buf);
  
}







