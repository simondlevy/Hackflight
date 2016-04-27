#define PORT Serial3
    
#include <NMEA.h>
#include <stdio.h>
    
static void dumptime(Time & time) {
        
    Serial.print(time.hours);
    Serial.print(":");
    Serial.print(time.minutes);
    Serial.print(":");
    Serial.print(time.seconds);
    Serial.println();
}

static void dumpdate(Date & date) {
        
    Serial.print(date.day);
    Serial.print(":");
    Serial.print(date.month);
    Serial.print(":");
    Serial.print(date.year);
    Serial.println();
}
    
static void dumpcoord(Coordinate & coord) {
        
    Serial.print(coord.sign < 0 ? '-' : '+');
    Serial.print(coord.degrees);
    Serial.print(':');
    Serial.println(coord.minutes, 5);
}
    
static void dumpheight(Height & height) {
        
    Serial.print(height.value);
    Serial.println(height.units);
}
        
class MyParser : public NMEA_Parser {
      
    void handleGPGGA(GPGGA_Message & gpgga) {
    
        Serial.println(gpgga.debug);
            
        dumptime(gpgga.time);
            
        dumpcoord(gpgga.latitude);
        dumpcoord(gpgga.longitude);
            
        Serial.println(gpgga.fixQuality);
    
        Serial.println(gpgga.numSatellites);
            
        Serial.println(gpgga.hdop);
            
        dumpheight(gpgga.altitude);
        dumpheight(gpgga.geoid);
            
        Serial.println();
    }
        
    void handleGPGLL(GPGLL_Message & gpgll) {
            
        Serial.println(gpgll.debug);
            
        dumpcoord(gpgll.latitude);
        dumpcoord(gpgll.longitude);
        
        dumptime(gpgll.time);
            
        Serial.println();
    }    
    
    void handleGPGSA(GPGSA_Message & gpgsa) {
            
        Serial.println(gpgsa.debug);
            
        Serial.println(gpgsa.mode);
        Serial.println(gpgsa.fixType);
            
        for (int k=0; k<gpgsa.nsats; ++k) {
            Serial.print(gpgsa.satids[k]);
            Serial.print(" ");
        }
        Serial.println();
        Serial.println(gpgsa.pdop);
        Serial.println(gpgsa.hdop);
        Serial.println(gpgsa.vdop);
            
        Serial.println();
    }   
   
    void handleGPGSV(GPGSV_Message & gpgsv) {
        
        Serial.println(gpgsv.debug);
        
        Serial.println(gpgsv.nmsgs);
        Serial.println(gpgsv.msgno);
        Serial.println(gpgsv.nsats);
        
        for (int j=0; j<gpgsv.ninfo; ++j) {
            
            SatelliteInView sv = gpgsv.svs[j];
            
            Serial.print("    ");
            Serial.print(sv.prn);
            Serial.print(" ");
            Serial.print(sv.elevation);
            Serial.print(" ");
            Serial.print(sv.azimuth);
            Serial.print(" ");
            Serial.println(sv.snr);
        }
        
        Serial.println();
    }     
    
    void handleGPRMC(GPRMC_Message & gprmc) {
        
         Serial.println(gprmc.debug);
            
         dumptime(gprmc.time);
         
         Serial.println(gprmc.warning);
            
         dumpcoord(gprmc.latitude);
         dumpcoord(gprmc.longitude);
         
         Serial.println(gprmc.groundspeedKnots);
         Serial.println(gprmc.trackAngle);
        
         dumpdate(gprmc.date);
         
         Serial.println();
    }
    
    void handleGPVTG(GPVTG_Message & gpvtg) {
        
         Serial.println(gpvtg.debug);
         
        Serial.println(gpvtg.trackMadeGoodTrue);
        Serial.println(gpvtg.trackMadeGoodMagnetic);
        Serial.println(gpvtg.speedKnots);
        Serial.println(gpvtg.speedKPH);

         Serial.println();
     }    
        
}; // MyParser


        
MyParser parser;
    
    
void setup(void) 
{
  Serial.begin(9600);
      
  PORT.begin(9600);
}
    
void loop(void)  {  
        
    if (PORT.available()) {
          
        parser.parse(PORT.read());
    }
      
}
