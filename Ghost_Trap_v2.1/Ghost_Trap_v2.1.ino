/********************************************************
 ARDUIO CODE FOR CHARLESWORTH DYNAMICS GHOST TRAP
 v2.1 (Michael Rajotte) - May 2022.
 -  Manually trigger smoke for 5 seconds at a time by adding a switch connected to GPIO 7 on the music maker shield.
 
 v2.0 (Michael Rajotte) - March 2022.
 -  Modifed door closure timing.
 -  Added smokeOff() and smokeOn() function to turn on or off the smoke machine.
 -  Modifed smoke machine turn off to synchronise with the door closure better.
 -  Universal volume control from the volumeR and volumeL settings during the main capture phases. Useful when testing your build.
 -  Slightly increased the volume of the startup sound.
 -  Modified the startup sequence to where the start up sound effect plays when the startup begins.
 -  Modified colour sequence during the trap sequence.
 -  Added purple to the spark sequence.
 -  Rear LED fades in and out instead of flashing on/off after the trap sequence is completed, then it stays on to a solid red after.
 -  When the trap first opens, a large flash of white strobe light appears before it settles down to the darker pink/purple strobe in the waiting sequence.
 
 NOTE: Connect the db+12 and db+6 on the musicmaker to prevent the hissing sound during the bootup sequence. This will make the trap louder but you can always adjust the sound volume in the code. I prefer it loud to help hide the smoke pump motor.
 NOTE: Motor pins are moved from digital i/o #4 and #5 to digital i/o #9 and digital i/o #10
 NOTE: NeoPixel Jewels are on digital i/o #5 pin.
 NOTE: Rear LED is moved from analog #1 pin to digital i/o #3.
 
 v1.0 (EctoLabs/Dave Tremaine 2020). Modified from original
 code by Jeremy Williams 2016
 -  Using 3x RGBW NeoPixel Jewels in place of single LEDs.
 -  Added reusable trapDoor() function to open and close
    doors. Includes immediate servo.detach() after each
    movement to avoid conflict with NeoPixel library
    during LED animation.
 -  Removed sine tone and replaced with bargraph startup 
    indicator.
 -  Rewritten trap states to better mimic movie behaviour.
 -  Added 'millisDelay' library as alternative to delay()
    and improve timing issues.
 -  Added blue sparking FX after ghost capture.
 -  Removed extraneous unused code and test functions.

 Ghost trap kit designed by Sean Charlesworth 2016-2018
 Programming and electronics by Jeremy Williams 2016
  
 ********************************************************/

#include <millisDelay.h> 
#include <SPI.h>
#include <Adafruit_VS1053.h>
#include <Adafruit_LEDBackpack.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <SD.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h> 

// Servos
Servo servoR;
Servo servoL;

int servoCloseL = 180;
int servoCloseR = 0;
int servoOpenL = 70;
int servoOpenR = 100;
int doorDelay = 50;

// Volume (0 = loudest, 255 = quietest)
uint8_t volumeR = 0;
uint8_t volumeL = 0;

#define SERVO_RIGHT_PIN 9
#define SERVO_LEFT_PIN 10

// Pins for single LEDs
//byte ledRed = 15;
byte ledRed = 3;
byte ledYellow = 16;

// Pedal & side knob activation pin
byte activationSwitch = 5;

// Smoke machine
byte smokePin = 6;
boolean endingSmoke = false; // smoke effect after trap close?
millisDelay smokeTimer;
int smokeOnLength = 5000; // Amount of time in milliseconds for the smoke to run when manually activating. 5000 = 5 seconds. Do not run it too long or you risk burnign out the e-cig coil.
boolean b_smokeOn = false;
byte manualSmokeButton = 7; // Extra button for activating the smoke manually. GPIO 7 on the music maker.

// Bargraph
Adafruit_24bargraph bar = Adafruit_24bargraph();

// Music Maker Shield
#define SHIELD_RESET -1 // VS1053 reset pin (unused!)
#define SHIELD_CS 7 // VS1053 chip select pin (output)
#define SHIELD_DCS 6 // VS1053 Data/command select pin (output)
#define CARDCS 4 // Card chip select pin
#define DREQ 2 // VS1053 Data request, ideally an Interrupt pin

Adafruit_VS1053_FilePlayer musicPlayer = Adafruit_VS1053_FilePlayer(SHIELD_RESET, SHIELD_CS, SHIELD_DCS, DREQ, CARDCS);

// NeoPixels
#define PIXELS_PIN 5
#define PIXELS_COUNT 21
Adafruit_NeoPixel strip(PIXELS_COUNT, PIXELS_PIN, NEO_GRBW + NEO_KHZ800);

// Initial states
int trapState = 0;
boolean autoreset = false;
boolean redLEDState = true;
boolean remotePressed = false;
boolean smokeActive = false;
boolean opened = false;
boolean beeping = false;
boolean sparking = false;
unsigned long captureStart = 0;
long debounceBuffer = 0;
long redFlashTime = 0;
long whiteFlashTime = 0;
long smokeToggleTime = 0;
byte activeFlasher = 0;
byte activeFlasherPurple = 0;

// Setup non-blocking timers
millisDelay captureDelay;
millisDelay bgDelay;
millisDelay sparkDelay;
millisDelay sparkTime;
millisDelay redLED;
millisDelay redLEDPauseTimer;
millisDelay s_timer1;
millisDelay s_timer2;

// Feature toggles
boolean smoke = true;
boolean sfx = true;
boolean sparkOn = false;
boolean bLEDUp = true;
boolean bLEDPause = false;
boolean bLEDStart = true;

// Red LED pulse control.
int rLED = 0; // red led starts at 0 (off).
int rLEDHigh = 255; // high light setting during the fade up.
int rLEDLow = 15; // low light setting during the fade down.
int redLEDTimer = 2; // millisecond speed control for the fade.
int redLEDPause = 100; // milliseconds for pausing the red led. Used when reaching full value of low or high.
int rLEDMultiplier = 5;

// Spark delays.
int sparkDelayShort = 30;
int sparkDelayLong = 100;

/////////
// SETUP
/////////


void setup() {

  // Start debug output
  //Serial.begin(57600);
  Serial.begin(9600);
  Serial.println(F("\n** STARTUP **"));

    // Check SD card
  if (!SD.begin(CARDCS)) {
    Serial.println(F("=> SD failed, or not present"));
    while (1);
  }
  Serial.println(F("=> SD Card OK"));

  // Initialise music player
  if (! musicPlayer.begin()) {
    Serial.println(F("=> Couldn't find music player, do you have the right pins defined?"));
    while (1);
  }

  Serial.println(F("=> Music player found"));
  
  musicPlayer.GPIO_pinMode(activationSwitch, INPUT);

  // Interrupt pin
  if (! musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT)) {
    Serial.println(F("=> DREQ pin is not an interrupt pin"));
  }

  // Calibrate closed door position
  trapDoors("calibrate");
  
  // Play the startup sequence sound effect.
  if(sfx==true){
    Serial.println("=> Playing startup sound");

    musicPlayer.setVolume(40, 40);
    musicPlayer.startPlayingFile("start.mp3");
  }
  
  // Bargraph red startup indicator
  bar.begin(0x70);
  bar.writeDisplay();
  for (uint8_t b = 0; b < 12; b++) {
    bar.setBar(23 - b, LED_RED);
    bar.writeDisplay();
    delay(30);
  }
  
  // Initialise static LEDs 
  pinMode (ledYellow, OUTPUT);
  pinMode (ledRed, OUTPUT);

  // Initialise NeoPixels and set to off
  strip.begin();
  strip.show();
  
  // Make sure smoke pump is inactive
  smokeOff();

  // Clear bargraph
  for (uint8_t b = 0; b < 12; b++) {
    bar.setBar(23 - b, LED_OFF);
    bar.writeDisplay();
    delay(50);
  }
 
  Serial.println(F("=> Startup complete (State: 0)"));
  Serial.println(F("\nWAITING FOR INPUT..."));
}

void startEnd() {
    // Fill bargraph
    for (uint8_t b = 0; b < 12; b++) {
      bar.setBar(23 - b, LED_YELLOW);  
      bar.writeDisplay();
      delay(20);
    }
 
    // Yellow LED on
    digitalWrite(ledYellow, HIGH);
    
    // Wait 1 second
    delay(1000);

    // Start red LED beep loop
    if (sfx == true) {
      musicPlayer.stopPlaying();
      musicPlayer.setVolume(volumeL, volumeR);
      musicPlayer.startPlayingFile("beepzap.mp3");
    }

    // Start timer for sparks
    sparkDelay.start(4400); 
    redLED.start(redLEDTimer);
    // Update trap state
    trapState = 2;
    beeping = true;

    trapDoors("open");
}

void loop() {
  checkSmokeButton();
  checkRemote();

////////////////////
// MAIN TRAP STATES
////////////////////
  
  // OPEN TRAP
  if (remotePressed && trapState == 0) {

    // NeoPixels on
    strip.fill(strip.Color(255,255,255,255));
    strip.show();
    
    // Play SFX
    if(sfx==true) {
      musicPlayer.stopPlaying();
      musicPlayer.setVolume(volumeL, volumeR);
      musicPlayer.startPlayingFile("open.mp3");
    }
    
    // Open doors
    trapDoors("open");

    delay(10);
    strip.fill(strip.Color(255,102,255,0));
    strip.show();

    delay(10);
    strip.fill(strip.Color(255,255,255,255));
    strip.show();

    delay(10);
    strip.fill(strip.Color(255,102,255,0));
    strip.show();

    delay(10);
    strip.fill(strip.Color(255,255,255,255));
    strip.show();
    delay(10);

    delay(10);
    strip.fill(strip.Color(255,102,255,0));
    strip.show();

    delay(10);
    strip.fill(strip.Color(255,255,255,255));
    strip.show();

    delay(10);
    strip.fill(strip.Color(255,102,255,0));
    strip.show();

    delay(10);
    strip.fill(strip.Color(255,255,255,255));
    strip.show();
    delay(10);

    delay(10);
    strip.fill(strip.Color(255,102,255,0));
    strip.show();

    delay(10);
    strip.fill(strip.Color(255,255,255,255));
    strip.show();

    delay(10);
    strip.fill(strip.Color(255,102,255,0));
    strip.show();

    delay(10);
    strip.fill(strip.Color(255,255,255,255));
    strip.show();
    delay(10);
    
    // Update trap state
    trapState = 1;
    Serial.println(F("\n** TRAP OPENED (State: 1) **"));
    Serial.println(F("\nWAITING FOR INPUT OR TIMEOUT..."));

    
  // CAPTURING
  } 
  else if (remotePressed && trapState == 1) {
    
    captureDelay.start(7800); // time synced with capture SFX
    boolean capturing = true;
    int strobeUp = 95;
    int strobeDown = 50;
    int strobeCycle = 0;
    
    // Play capture SFX
    if(sfx==true){
      musicPlayer.stopPlaying();
      musicPlayer.setVolume(volumeL, volumeR);
      musicPlayer.startPlayingFile("cap1984.mp3");
    }

    // NeoPixels burst
    for (uint8_t i=0; i<255; i=i+5) {
      //Serial.println(i);
      if (i>102) {
        strip.fill(strip.Color(255,i,255,i));
      } else {
        strip.fill(strip.Color(255,102,255,i));
      }
      strip.show();
      delay(1);
    }
    
    while(capturing == true) {
      //Serial.println(captureDelay.remaining());

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
            
      if(captureDelay.remaining() <= 4500) {
        strip.fill(strip.Color(strobeUp,strobeUp,strobeUp,strobeUp));
        strip.show();
        delay(20);
        strip.fill(strip.Color(strobeDown,strobeDown,strobeDown,strobeDown));
        strip.show();
        delay(20);
      }
      else if(captureDelay.remaining() <= 6000) {
        strip.fill(strip.Color(255,102,255,strobeUp));
        strip.show();
        delay(20);
        strip.fill(strip.Color(strobeDown,strobeDown,strobeDown,strobeDown));
        strip.show();
        delay(20);
      }
      else if(captureDelay.remaining() <= 6500) {
        strip.fill(strip.Color(140,255,140,strobeUp));
        strip.show();
        delay(20);
        strip.fill(strip.Color(strobeDown,strobeDown,strobeDown,strobeDown));
        strip.show();
        delay(20);
      }
      else if(captureDelay.remaining() <= 7000) {
        strip.fill(strip.Color(255,215,90,strobeUp));
        strip.show();
        delay(20);
        strip.fill(strip.Color(strobeDown,strobeDown,strobeDown,strobeDown));
        strip.show();
        delay(20);
      }
      else {
        strip.fill(strip.Color(strobeUp,strobeUp,strobeUp,strobeUp));
        strip.show();
        delay(20);
        strip.fill(strip.Color(strobeDown,strobeDown,strobeDown,strobeDown));
        strip.show();
        delay(20);
      }
     
      // Smoke on for final 7 seconds
      if((captureDelay.remaining() <= 7800) && smokeActive == false && smoke == true) {
        smokeOn();
        smokeActive = true;
      }
      
      if((captureDelay.remaining() <= 3000) && smokeActive == true) {
        // When the smoke machine is turned off, it still buzzes a slight bit. If you keep on turning it off, it will stop the humming/buzz. So keep on turning it off during the loop when required.
        smokeOff();
      }

      // Close the door to match the end of the sound effects.
      if((captureDelay.remaining() <= 2900) && capturing == true) {
          trapDoors("close");
          strip.fill(strip.Color(255,255,255,255));
          strip.show();  
          capturing = false;
      }

      // Close doors after 8 seconds if the door is still open.
      if(captureDelay.justFinished() && capturing == true) {
        trapDoors("close");
        strip.fill(strip.Color(255,255,255,255));
        strip.show(); 
        capturing = false;
      }
      
    }

    // Smoke off if it is still on for some reason.
    if(smokeActive==true) {
      smokeOff();
    }
    
    // NeoPixels off
    strip.fill(strip.Color(0,0,0,0));
    strip.show();
    
    // Silence for 4 seconds before indicators
    bgDelay.start(4000);
    boolean silence = true;

    while(silence == true) {
      if(bgDelay.justFinished()){
        // Play bargraph SFX
        if (sfx == true) {
          musicPlayer.setVolume(30, 30);
          musicPlayer.startPlayingFile("bargraph.mp3");
        }
        
        silence = false;
      }
    }

    // Fill bargraph
    for (uint8_t b = 0; b < 12; b++) {
      bar.setBar(23 - b, LED_YELLOW);  
      bar.writeDisplay();
      delay(20);
    }
 
    // Yellow LED on
    digitalWrite(ledYellow, HIGH);
    
    // Wait 1.5 second
    delay(1500);

    // Start red LED beep loop
    if (sfx == true) {
      musicPlayer.stopPlaying();
      musicPlayer.setVolume(volumeL, volumeR);
      musicPlayer.startPlayingFile("beepzap.mp3");
    }

    // Start timer for sparks
    sparkDelay.start(4400); 

    // Timer for the Red LED.
    redLED.start(redLEDTimer);
    
    // Update trap state
    trapState = 2;
    beeping = true;
    Serial.println(F("\n** FULL TRAP (State: 2 FLASHING) **"));
    Serial.println(F("\nWAITING FOR INPUT OR TIMEOUT..."));
    

  // RESET TRAP
  } 
  else if ((remotePressed && trapState == 2) || autoreset == true) {
  
    // Make sure doors are closed
    trapDoors("close");

    // Music player stop
    musicPlayer.stopPlaying();
    
    // Yellow and Red LEDs off
    digitalWrite(ledRed, LOW);
    digitalWrite(ledYellow, LOW);
    
    // NeoPixels off
    strip.fill(strip.Color(0,0,0,0));
    strip.show();

    // Bargraph reset
    for (uint8_t b = 12; b > 0; b--) {
      bar.setBar(23 - b + 1, LED_OFF);
      bar.writeDisplay();
      delay(50);
    }

    // Make sure smoke is off
    smokeActive = false;
    smokeToggleTime = 0;
    smokeOff();
    b_smokeOn = false;
    
    // Update trap state
    trapState = 0;
    autoreset = false;
    opened = false;
    Serial.println(F("\n** RESET TRAP (State: 0)**"));
    Serial.println(F("\nWAITING FOR INPUT..."));
    

//////////////////
// LOOPING STATES
//////////////////


  
  // OPEN LOOP
  } 
  else if (trapState == 1) {
    if (sfx == true && musicPlayer.stopped()) {
      autoreset = true;
      Serial.println(F("\n** TIMEOUT **"));
    }

    if (opened == false) {
      for (uint8_t i=0; i<250; i=i+10) {
        strip.fill(strip.Color(255,102,255,i));
        strip.show();
        delay(1);
      }
      
      for (uint8_t i=240; i>0; i=i-10) {
        strip.fill(strip.Color(255,102,255,i));
        strip.show();
        delay(1);
      }
      
      opened = true;
    }
    
    // Flash NeoPixels randomly
    if (millis() > whiteFlashTime) {
      whiteFlashTime = millis() + 50;
      strip.setPixelColor(activeFlasher, strip.Color(255,102,255,255));
      strip.show();
      delay(30);
      strip.setPixelColor(activeFlasher, strip.Color(255,102,255,0));
      strip.show();
      activeFlasher = random(0, 22);
    }
  // FULL TRAP LOOP
  } 
  else if (trapState == 2 && beeping == true) {    
    // Beeping stops when the sound effect has finished playing, which is a 1 minute long beeping sound file.
    if (sfx == true && musicPlayer.stopped()) {
      // Solid red light once beeping stops
      digitalWrite(ledRed, HIGH);

      // Update trap state
      beeping = false;
      Serial.println(F("\n** FULL TRAP (State: 2 IDLE) **"));
      Serial.println(F("\nWAITING FOR INPUT..."));
    
    } 
    else {
      // Flash red light during beeps      
      analogWrite(ledRed, rLED);

      if(redLED.remaining() < 1) {
        redLED.start(redLEDTimer);

        if(bLEDPause == false) {
          if(rLED == rLEDHigh) {
            bLEDUp = false;
            redLEDPauseTimer.start(redLEDPause);
            bLEDPause = true;
          }
          else if(rLED == rLEDLow && bLEDStart == false) {
            bLEDUp = true;
  
            redLEDPauseTimer.start(redLEDPause);
            bLEDPause = true;
          }
        }

        if(redLEDPauseTimer.remaining() < 1) {
          bLEDPause = false;

          if(bLEDStart == true && rLED == rLEDLow) {
            bLEDStart = false;
          }
          
          if(bLEDUp == true) {
            rLED = rLED + rLEDMultiplier;

            if(rLED > rLEDHigh) {
               rLED = rLEDHigh;
            }
          }
          else {
             rLED = rLED - rLEDMultiplier;

             if(rLED < rLEDLow) {
               rLED = rLEDLow;
             }
          }
        }
      }
            
      // Blue & purple sparks synched with SFX
      if(sparkDelay.justFinished()) {
        sparkTime.start(4500);
        s_timer1.start(1);
        s_timer2.start(1);
        sparking = true;
        sparkOn = false;
        Serial.println(F("=> Blue & purple sparks"));
      }
      
      if(sparkTime.justFinished()) {
        sparking = false;
      }
      else if(sparking == true) { 
        if(s_timer2.remaining() < 1) {
          if(s_timer1.remaining() < 1 && sparkOn == false) {
              activeFlasher = random(0, 22);
              strip.setPixelColor(activeFlasher, strip.Color(0,0,255,0));
              strip.show();
              
              activeFlasherPurple = random(0, 22);
              strip.setPixelColor(activeFlasherPurple, strip.Color(202,0,255,0));
              strip.show();
              sparkOn = true;

              s_timer1.start(sparkDelayShort);
          }
          else if(s_timer1.remaining() < 1 && sparkOn == true) {
              strip.setPixelColor(activeFlasher, strip.Color(0,0,0,0));
              strip.show();
      
              strip.setPixelColor(activeFlasherPurple, strip.Color(0,0,0,0));
              strip.show();
              sparkOn = false;

              s_timer1.start(sparkDelayShort);
              s_timer2.start(sparkDelayLong);
          }

        }

      }
    }
    
  }
  
}

/////////////
// FUNCTIONS
/////////////

void smokeOn() {
  musicPlayer.GPIO_pinMode(smokePin, OUTPUT);
  musicPlayer.GPIO_digitalWrite(smokePin, HIGH);

  //Serial.println(F("Smoke On"));
}

void smokeOff() {
  musicPlayer.GPIO_digitalWrite(smokePin, LOW);
  musicPlayer.GPIO_pinMode(smokePin, INPUT);

  //Serial.println(F("Smoke Off"));
  b_smokeOn = false;
}

void trapDoors(String mode) {
  if (mode=="calibrate") {
    servoR.write(servoCloseR);
    servoL.write(servoCloseL);
  }
  
  servoR.attach(SERVO_RIGHT_PIN);
  servoL.attach(SERVO_LEFT_PIN);
  if (mode=="open") {
    servoR.write(servoOpenR);
    servoL.write(servoOpenL);
  } 
  else {
    servoL.write(servoCloseL);
    delay(doorDelay);
    servoR.write(servoCloseR);
  }
  
  delay(300);
  servoR.detach();
  servoL.detach();
}

void checkRemote() {
  if(musicPlayer.GPIO_digitalRead(activationSwitch) == HIGH && millis() > debounceBuffer) {
    debounceBuffer = millis() + 1000;
    remotePressed = true;
  }
  else { 
    remotePressed = false;
  }
}

void checkSmokeButton() {
  if(musicPlayer.GPIO_digitalRead(manualSmokeButton) == HIGH && b_smokeOn == false) {
    // Play some smoke for 5 seconds.
    smokeTimer.start(smokeOnLength);
    b_smokeOn = true;
  }

  // Let's generate a smoking trap.
  if(smokeTimer.justFinished() && b_smokeOn == true) {
    smokeOff();
    b_smokeOn = false;
  }
  else if(b_smokeOn == true) {
    smokeOn();
  }
}
