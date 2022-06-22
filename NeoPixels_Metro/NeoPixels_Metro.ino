/********************************************************
 ARDUINO NEOPIXEL CODE FOR CHARLESWORTH DYNAMICS GHOST TRAP
 v1.0 (Michael Rajotte) - March 2022.
 -  Another Arduino micro controller connects to this one through the Serial port connections to issue commands for operating the NeoPixels.
/********************************************************
*/

#include <millisDelay.h> 
#include <Adafruit_NeoPixel.h> 
#define PIXELS_PIN 5
#define PIXELS_COUNT 21
Adafruit_NeoPixel strip(PIXELS_COUNT, PIXELS_PIN, NEO_GRBW + NEO_KHZ800);

String rxString = "";
int rxByte = 0;

byte activeFlasher = 0;
byte activeFlasherPurple = 0;
long whiteFlashTime = 0;
int trapOpenDelay = 10; // milliseconds
int trapOpenStage = 0;
int trapOpenStageL = 32;
int R = 255;
int G = 102;
int B = 255;

int strobeUp = 95;
int strobeDown = 50;
int strobeCycle = 0;

void setup() {
  Serial.begin(9600);

  strip.begin();
  pixelsOff();
}

void loop() {
    if(Serial.available() > 0) {
      rxByte = Serial.read();
              
      if(rxByte == 1) {
        pixelsOff();
      }
      else if(rxByte == 2) {
        trapOpen1();
      }
      else if(rxByte == 3) {
        trapOpen2();
      }
      else if(rxByte == 4) {
        trapOpeningLoop();
      }
      else if(rxByte == 5) {
        loopOpen();
      }
      else if(rxByte == 6) {
        capture84_burst();
      }
      else if(rxByte == 7) {
        capture_strobe_reset();
      }
      else if(rxByte == 8) {
        capture_strobe();
      }
      else if(rxByte == 9) {
        capture_strobe_up();
      }
      else if(rxByte == 10) {
        capture_strobe_down();
      }
      else if(rxByte == 11) {
        capture_84_strobe_6000_up();
      }
      else if(rxByte == 13) {
        capture_84_strobe_6500_up();
      }
      else if(rxByte == 14) {
        capture_84_strobe_7000_up();
      }
      else if(rxByte == 15) {
        captureMisc_burst();
      }
      else if(rxByte == 16) {
        capture_misc_4500_up();
      }
      else if(rxByte == 17) {
        capture_misc_6000_up();
      }
      else if(rxByte == 18) {
        capture_misc_6500_up();
      }
      else if(rxByte == 19) {
        capture_misc_7000_up();
      }
      else if(rxByte == 20) {
        capture_misc_up();
      }
      else if(rxByte == 21) {
        sparksOn();
      }
      else if(rxByte == 22) {
        sparksOff();
      }
      else if(rxByte == 23) {
        sparksOnGreen();
      }
      
      rxByte = 0;
    }
}

void sparksOnGreen() {
  activeFlasher = random(0, 22);
  strip.setPixelColor(activeFlasher, strip.Color(140,255,0,0));
  strip.show();
    
  activeFlasherPurple = random(0, 22);
  strip.setPixelColor(activeFlasherPurple, strip.Color(0,255,0,0));
  strip.show();
}

void sparksOn() {
  activeFlasher = random(0, 22);
  strip.setPixelColor(activeFlasher, strip.Color(0,0,255,0));
  strip.show();
    
  activeFlasherPurple = random(0, 22);
  strip.setPixelColor(activeFlasherPurple, strip.Color(202,0,255,0));
  strip.show();
}

void sparksOff() {
  strip.setPixelColor(activeFlasher, strip.Color(0,0,0,0));
  strip.show();
  
  strip.setPixelColor(activeFlasherPurple, strip.Color(0,0,0,0));
  strip.show();
}

void capture_misc_4500_up() {
  pixelsOn(0,255,0,strobeDown);  
}

void capture_misc_6000_up() {
  pixelsOn(0,255,60,strobeDown);
}

void capture_misc_6500_up() {
  pixelsOn(0,255,90,strobeUp);  
}

void capture_misc_7000_up() {
  pixelsOn(0,255,140,strobeUp);
}

void capture_misc_up() {
  pixelsOn(0,255,154,strobeUp);        
}

void capture_84_strobe_7000_up() {
  pixelsOn(255,215,90,strobeUp);
}

void capture_84_strobe_6500_up() {
  pixelsOn(140,255,140,strobeUp);
}

void capture_84_strobe_6000_up() {
  pixelsOn(255,102,255,strobeUp);
}

void capture_strobe_up() {
  pixelsOn(strobeUp,strobeUp,strobeUp,strobeUp);
}

void capture_strobe_down() {
  pixelsOn(strobeDown,strobeDown,strobeDown,strobeDown);
}

void capture_strobe() {
  // NeoPixel strobe
  if(strobeCycle == 0){
    if(strobeUp<=251){
      strobeUp = strobeUp + 4;
    }
    
    if(strobeDown>=1){
      strobeDown = strobeDown - 1;
    }
  }
  
  strobeCycle++;
  if(strobeCycle==3){
    strobeCycle = 0;
  }
}


void capture_strobe_reset() {
  strobeUp = 95;
  strobeDown = 50;
  strobeCycle = 0;
}

void captureMisc_burst() {
  for (uint8_t i=0; i<255; i=i+5) {
    pixelsOn(0,255,0,i);
    
    trapDelay(1);
  }

  Serial.write(1);
}

void capture84_burst() {
  for (uint8_t i=0; i<255; i=i+5) {
    if(i>102) {
      pixelsOn(255,i,255,i);
    } 
    else {
      pixelsOn(255,i,255,i);
    }
    
    trapDelay(1);
  }

  Serial.write(1);
}

void loopOpen() {
  // Flash NeoPixels randomly
  if(millis() > whiteFlashTime) {
    whiteFlashTime = millis() + 50;
    
    strip.setPixelColor(activeFlasher, strip.Color(255,102,255,255));
    strip.show();

    trapDelay(30);
    
    strip.setPixelColor(activeFlasher, strip.Color(255,102,255,0));
    strip.show();
    activeFlasher = random(0, 22);
  } 

  Serial.write(1);
}

void trapOpeningLoop() {    
  millisDelay trapOpenTimer;
  boolean b_trapOpen = true;
  boolean b_trapOpenBright = true;
  trapOpenTimer.start(trapOpenDelay);
  
  while(b_trapOpen == true) {
    if(trapOpenTimer.remaining() < 1 && trapOpenStage < trapOpenStageL) {
      if(b_trapOpenBright == false) {
        pixelsOn(R,G,B,0);
        trapDelay(trapOpenDelay);
        b_trapOpenBright = true;
      }
      else {
        pixelsOn(255,255,255,255);

        trapDelay(trapOpenDelay);
        b_trapOpenBright = false;
      }
  
      trapOpenTimer.start(trapOpenDelay);
      trapOpenStage++;
    }
    else if(trapOpenTimer.remaining() < 1 && trapOpenStage == trapOpenStageL) {
      b_trapOpen = false;
      b_trapOpenBright = true;
    }
  }

  for (uint8_t i=0; i<250; i=i+10) {
    pixelsOn(255,102,255,i);

    trapDelay(1);
  }
  
  for(uint8_t i=240; i>0; i=i-10) {
    pixelsOn(255,102,255,i);

    trapDelay(1);
  }

  Serial.write(1);
}

void trapOpen2() {
  trapOpenStageL = 32;
  trapOpenStage = 0;
  R = 255;
  G = 102;
  B = 255;
  
  pixelsOn(255,255,255,255);
}

void trapOpen1() {
  trapOpenStageL = 64;
  trapOpenStage = 0;
  R = 0;
  G = 255;
  B = 0;
    
  pixelsOn(0,255,0,255);
}

void pixelsOff() {
  strip.fill(strip.Color(0,0,0,0));
  strip.show();
}

void pixelsOn(int R, int G, int B, int A) {
  strip.fill(strip.Color(R,G,B,A));
  strip.show();
}

void trapDelay(int bgDelayTime) {
  millisDelay trapDelay;
  boolean bg_delay = true;

  trapDelay.start(bgDelayTime);
  
  while(bg_delay == true) {
    
    if(trapDelay.justFinished()) {
      bg_delay = false;
    }
  }
}
