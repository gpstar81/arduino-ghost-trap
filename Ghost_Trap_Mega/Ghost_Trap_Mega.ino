/********************************************************
 ARDUINO CODE FOR CHARLESWORTH DYNAMICS GHOST TRAP
 v3.0 (Michael Rajotte) - March 2022.
 -  Cleaned up and rewrote the code, removing delays and using the millisecond library instead.
 -  Volume control added to the rotary encoder switch.
 -  Mode selector switch added to switch between different modes.

NOTE: This code is for the Arduino Mega relevant chipsets clones. Use the same wiring as before in the guides and notes below.
******* WIRING CHANGES *******
ledYellow: pin 8 on musicmaker
move bargraph pins: yellow/blue on analog 5 and 4 to: (yellow) -> SCL on music maker. (blue) -> SDA music maker. (make a new 3 pin plug).
rotary encoder: new 3 pin plug: middle pin is ground and goes to ground. 2 outside pins: When viewing the 3 pins from the top (button side) of the encoder with the 3 pins facing north (top), the outside left pin goes to pin #39 and right side pin goes to pin #38.
The Neopixel's are moved to a 12-bit PWM/Servo Driver - I2C interface - PCA9685 Board Adafruit part # 815), this is due to how the NeoPixel library operates and turns off interrupts on the pwm pins, this will affect Servo Motors (twitching) and other timing issues regarding to the millisecond library.
8 way rotary switch: connect to pins: 22 to 29. #1 pin on the switch goes to #22 pin work your way up to #8 pin on switch going to pin 29. Then centre pin goes to ground.
The 2 servos go onto pins 44 and 45.
    
 ARDUINO CODE FOR CHARLESWORTH DYNAMICS GHOST TRAP
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
 NOTE: Motor pins are moved from digital i/o #4 a-nd #5 to digital i/o #9 and digital i/o #10
 NOTE: NeoPixel Jewels are on digital i/o #5 pin.
 NOTE: Rear LED is moved from analog #1 pin to digital i/o #3. It requries a PWM pin to pulse the LED. This could also be moved to pin #6 as well as #3 is a interrupt pin.
 
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
//#include <Servo.h>
/* 
known_16bit_timers and Adafruit_TiCoServo will stop servo twitching when the servos are always attached when using neopixels and or anything like rotary encoders which operate on interrupts.
The only downside is serial output can be buggy. Not a problem once your code is all setup though.
*/
#include <known_16bit_timers.h>
#include <Adafruit_TiCoServo.h>
#include <Adafruit_NeoPixel.h> 

/*
******* WIRING CHANGES *******
   What is happening is the NeoPixel library changes the milliseconds and becomes slower based on how often the neopixels are firing.
   Move the servos to the PCA9685 interface, and use the milliseconds timer from that to control the program timing.
*/

/*
// https://learn.adafruit.com/neopixels-and-servos?view=all
For Mega boards, must use pins: 2, 3, 5, 6, 7, 8, 11, 12, 13, 44, 45, 46 with this library.
*/
// Servos
//Servo servoR;
//Servo servoL;
Adafruit_TiCoServo servoR;
Adafruit_TiCoServo servoL;
#define SERVO_RIGHT_PIN 44
#define SERVO_LEFT_PIN 45

int servoCloseL = 180;
int servoCloseR = 0;
int servoOpenL = 70;
int servoOpenR = 120;
int doorDelay = 30;
//int doorWait = 300;

// Volume (0 = loudest, 255 = quietest)
uint8_t volumeMax = 10; // Max volume
uint8_t volumeMaxMid = 30; // Max mid volume
uint8_t volumeMidMin = 90; // Minimum mid volume
uint8_t volumeMin = 60; // Minimum volume
uint8_t volumeR = 30;
uint8_t volumeL = 30;
uint8_t volumeMidL = 30;
uint8_t volumeMidR = 30;
uint8_t volInt = 1;

boolean b_volume = false; // used to know if playing high or medium volume.

// Pins for single LEDs
byte ledRed = 3;
#define ledYellow 8

// Smoke machine
byte smokePin = 6;

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
long volumeBuffer = 0;
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

// Manual smoke control.
millisDelay smokeTimer;
boolean b_smokeOn = false;
int smokeOnLength = 5000;

// Feature toggles
boolean smoke = true;
boolean sparkOn = false;
boolean bLEDUp = true;
boolean bLEDPause = false;
boolean bLEDStart = true;

// Bargraph
Adafruit_24bargraph bar = Adafruit_24bargraph();
int bargraphTimerStart = 30; // milliseconds
int bargraphTimerEnd = 50;
millisDelay bargraphTimer;
boolean b_bargraphStart = false;
uint8_t bargraphLed = 0;
boolean bargraphFill = false;
millisDelay b_timer;
int b_timer_length = 2000;

// Red LED pulse control.
int rLED = 0; // red led starts at 0 (off).
int rLEDHigh = 255; // high light setting during the fade up.
int rLEDLow = 15; // low light setting during the fade down.
int redLEDTimer = 2.2; // millisecond speed control for the fade.
int redLEDPause = 125; // milliseconds for pausing the red led. Used when reaching full value of low or high.
int rLEDMultiplier = 5;

// Spark delays.
int sparkDelayShort = 30;
int sparkDelayLong = 100;

/* 
 -- NOT OK PINS --
  13 SD card from music maker 
  12 music maker
  11 music maker
  7 music maker

/* 
 --- OK PINS ---
  0
  6
  8
*/

/*
https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/ 
attachInterrupt(digitalPinToInterrupt(pin), ISR, mode) (recommended)
attachInterrupt(interrupt, ISR, mode) (not recommended)
attachInterrupt(pin, ISR, mode) (Not recommended. Additionally, this syntax only works on Arduino SAMD Boards, Uno WiFi Rev2, Due, and 101.)
*/
// Interrupt pins: Mega -> (2, 3, 18, 19, 20, 21). Uno (2-3)
// Mega pwm pins: (2 - 13, 44 - 46)
// Must be pwm pins for the rotary encoder 3 pin side (middle pin is ground)
#define rEncoderA 38
#define rEncoderB 39
int rEncoderCount = 0;
int rEncoderState;
int rEncoderLastState;

// Pedal & side knob activation pin on the rotary encoder.
byte activationSwitch = 5;

/*
 Rotary switch for the different modes.
 1 = 1984 w/smoke
 2 = 1984 no smoke
 3 = 1989 w/smoke
 4 = 1989 no smoke
 5 = Misc w/smoke
 6 = Misc no smoke
 7 = Trap captured blinking state on repeat with random sparks. Press trigger knob or pedal to add 5 seconds of smoke.
 8 = Trap captured solid state. Press trigger knob or pedal to add 5 seconds of smoke.
*/
int rSwitchCurrentPosition = 0;
int rSwitchLastPosition = 0;
#define r1 22
#define r2 23
#define r3 24
#define r4 25
#define r5 26
#define r6 27
#define r7 28
#define r8 29

boolean setupMode = true;
  
void setup() {
  // Start debug output
  Serial.begin(9600);
  
  // Initialise music player
  if(!musicPlayer.begin()) {
    serialOutput("Couldn't find music player, do you have the right pins defined?");
    while (1);
  }

  serialOutput("Music player found");

  // Check SD card
  if(!SD.begin(CARDCS)) {
    serialOutput("SD failed, or not present");
    while (1);
  }
  serialOutput("SD Card OK");

  musicPlayer.GPIO_pinMode(activationSwitch, INPUT);

  // Interrupt pin
  if(!musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT)) {
    serialOutput("DREQ pin is not an interrupt pin");
  }
  
  //servoL.attach(SERVO_LEFT_PIN);
  //servoR.attach(SERVO_RIGHT_PIN);
  
  // Calibrate trap doors into the closed position.
  trapDoors("close");
  
  // Rotary encoder for adjusting the volume of the track.
  pinMode(rEncoderA, INPUT_PULLUP);
  pinMode(rEncoderB, INPUT_PULLUP);
  
  // Reads the initial state of rotary encoder,
  rEncoderLastState = digitalRead(rEncoderA);
  
  // Rotary switch for changing the mode of the trap.
  pinMode(r1, INPUT_PULLUP); 
  pinMode(r2, INPUT_PULLUP);
  pinMode(r3, INPUT_PULLUP);
  pinMode(r4, INPUT_PULLUP);
  pinMode(r5, INPUT_PULLUP);
  pinMode(r6, INPUT_PULLUP);
  pinMode(r7, INPUT_PULLUP);
  pinMode(r8, INPUT_PULLUP);
  
  checkSwitchPosition();

  // Initialise static LEDs 
  pinMode(ledYellow, OUTPUT);
  pinMode(ledRed, OUTPUT);
  
  // Initialise NeoPixels and set to off
  //strip.begin();
  //strip.fill(//strip.Color(0,0,0,0));
  //strip.show();
  
  // Make sure smoke pump is inactive
  smokeOff();

  // Initalise the bargraph.
  bar.begin(0x70);
  bar.writeDisplay();
}

void bargraphStartUp() {
  if(b_bargraphStart == false) {
    // Play the startup sequence sound effect.
    
    b_volume = false;
    
    musicPlayer.setVolume(volumeMidL, volumeMidR);
    musicPlayer.startPlayingFile("start.mp3");
  
    bargraphFill = true;
    
    // Yellow LED on
    digitalWrite(ledYellow, HIGH);

    b_bargraphStart = true;

    bargraphTimer.start(bargraphTimerStart);

    bar.setBar(23 - bargraphLed, LED_RED);
    bar.writeDisplay();
  }

  if(bargraphTimer.remaining() < 1 && bargraphLed < 12 && bargraphFill == true) {
    bargraphTimer.start(bargraphTimerStart);

    bar.setBar(23 - bargraphLed, LED_RED);
    bar.writeDisplay();

    bargraphLed++;
    
    if(bargraphLed == 12) {
      bargraphFill = false;
      bargraphLed = 0;
    }
  }
  else if(bargraphTimer.remaining() < 1 && bargraphLed < 12 && bargraphFill == false) {   
    bargraphTimer.start(bargraphTimerEnd);

    bar.setBar(23 - bargraphLed, LED_OFF);
    bar.writeDisplay();

    bargraphLed++;
    
    if(bargraphLed == 12) {
      // Yellow LED on
      digitalWrite(ledYellow, LOW);
      bargraphLed = 0;
    }
  }

  if(musicPlayer.stopped()) {
    setupMode = false;
    serialOutput("Startup complete (State: 0)");
  }
}

void loop() {
  checkVolumeEncoder();
  checkSwitchPosition();
  
  if(setupMode == true) {
    bargraphStartUp();
  }
  else {
    checkRemote();

    if(trapState == 0 && rSwitchCurrentPosition != 7 && rSwitchCurrentPosition != 8) {
      if(b_timer.justFinished()) {
       bargraphOff();
      }
    }
    
    // 1984 mode
    if(rSwitchCurrentPosition == 1 || rSwitchCurrentPosition == 2) {          
      if(remotePressed && trapState == 0) {
        bargraphOff();
        
        // Open trap.
        trapOpen();
      }
      else if(remotePressed && trapState == 1) {
        // Capture sequence.
        trapCapture_1984();
      } 
      else if((remotePressed && trapState == 2) || autoreset == true) {
        // Reset the trap.
        trapReset();
      } 
      else if (trapState == 1) {
        // The loop when the trap is open and waiting.
        loopOpen();
      } 
      else if (trapState == 2 && beeping == true) {
        // The entity has been captured. Loop through the fading rear led etc.
        loopCaptured();
      }
    }
    else if(rSwitchCurrentPosition == 3 || rSwitchCurrentPosition == 4) {
      // 1989 mode.      
      if(remotePressed && trapState == 0) {
        bargraphOff();
        trapOpen();
      }
      else if(trapState == 1) {
        trapCapture_1989();
      }
      else if((remotePressed && trapState == 2) || autoreset == true) {
        // Reset the trap.
        trapReset();
      } 
      else if(trapState == 2 && beeping == true) {
        loopCaptured();
      }
    }
    else if(rSwitchCurrentPosition == 5 || rSwitchCurrentPosition == 6) {     
      // Misc mode.
      if(remotePressed && trapState == 0) {
        bargraphOff();
        trapOpen();
      }
      else if(trapState == 1) {
        trapCapture_Misc();
      }
      else if((remotePressed && trapState == 2) || autoreset == true) {
        // Reset the trap.
        trapReset();
      } 
      else if(trapState == 2 && beeping == true) {
        loopCaptured();
      }
    }
    else if(rSwitchCurrentPosition == 7) {
      // Solid red light, captured.      
      if(trapState == 0) {
        // Captured state
        preLoopCaptured();
      }

      if(remotePressed && trapState == 2 && b_smokeOn == false) {
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
    else if(rSwitchCurrentPosition == 8) {
      // Repeating blinking red light with some sparks, captured.
      
      if(remotePressed && trapState == 2 && b_smokeOn == false) {
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
      
      if(trapState == 0) {
        // Beeping captured.
        preLoopCapturedBeeping();
      }
      else if(trapState == 2 && beeping == true) {
        loopCaptured();
      }
    }
  }
}

void trapOpen() {
  // Trap open stage (waiting to capture)
  int trapOpenDelay = 10; // milliseconds
  int trapOpenStage = 0;
  int trapOpenStageL = 32;

  int R = 255;
  int G = 102;
  int B = 255;

  // Opening trap in Misc mode.
  if(rSwitchCurrentPosition == 5 || rSwitchCurrentPosition == 6) {
    // Pulse longer
    trapOpenStageL = 64;

    // Change the opening flash colours.
    R = 0;
    G = 255;
    B = 0;
    
    //strip.fill(//strip.Color(R,G,B,255));
  }
  else {
    //strip.fill(//strip.Color(255,255,255,255));
  }
  //strip.show();

  musicPlayer.stopPlaying();

  if(rSwitchCurrentPosition == 5 || rSwitchCurrentPosition == 6) {
    b_volume = false;
    musicPlayer.setVolume(volumeMidL, volumeMidR);
    musicPlayer.startPlayingFile("turn_on.mp3");
  }
  else {
    b_volume = true;
    musicPlayer.setVolume(volumeL, volumeR);
    musicPlayer.startPlayingFile("open.mp3");
  }

  // Open doors
  trapDoors("open");

  // GB 2 Mode.
  if(rSwitchCurrentPosition == 3 || rSwitchCurrentPosition == 4) {
     trapDelay(100);
     trapState = 1;
     serialOutput("** TRAP OPENED (State: 1) **");
  }
  else {
    millisDelay trapOpenTimer;
    boolean b_trapOpen = true;
    boolean b_trapOpenBright = true;
    trapOpenTimer.start(trapOpenDelay);
  
    while (b_trapOpen == true) {
      if(trapOpenTimer.remaining() < 1 && trapOpenStage < trapOpenStageL) {
        if(b_trapOpenBright == false) {
          //strip.fill(//strip.Color(R,G,B,0));
          trapDelay(trapOpenDelay);
          b_trapOpenBright = true;
        }
        else {
          //strip.fill(//strip.Color(255,255,255,255));
          trapDelay(trapOpenDelay);
          b_trapOpenBright = false;
        }
        //strip.show();
    
        trapOpenTimer.start(trapOpenDelay);
        trapOpenStage++;
      }
      else if(trapOpenTimer.remaining() < 1 && trapOpenStage == trapOpenStageL) {
        b_trapOpen = false;
        b_trapOpenBright = true;
        
        // Update trap state
        trapState = 1;
        serialOutput("** TRAP OPENED (State: 1) **");
      }
    }
  }
}

void trapCapture_Misc() {
  captureDelay.start(8000);
  boolean capturing = true;
  int strobeUp = 95;
  int strobeDown = 50;
  int strobeCycle = 0;
    
  musicPlayer.stopPlaying();
  b_volume = true;
  musicPlayer.setVolume(volumeL, volumeR);
  musicPlayer.startPlayingFile("capmisc.mp3");
  
  // NeoPixels burst
  for (uint8_t i=0; i<255; i=i+5) {
    //strip.fill(//strip.Color(0,255,0,i));
  
    //strip.show();

    trapDelay(1);
  }

  boolean b_doorsOpen = true;
  boolean b_light = true;
  float timer1Secs = 0;
  startTimer1();
  
  while(capturing == true) {   
    timer1Secs = getTimer1Seconds(); 
        
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
      //strip.fill(//strip.Color(0,255,0,strobeDown));
      //strip.show();
      trapDelay(20);
      
      //strip.fill(//strip.Color(strobeDown,strobeDown,strobeDown,strobeDown));
      //strip.show();
      trapDelay(20);
    }
    else if(captureDelay.remaining() <= 6000) {
      //strip.fill(//strip.Color(0,255,60,strobeDown));
      //strip.show();
      trapDelay(20);

      //strip.fill(//strip.Color(strobeDown,strobeDown,strobeDown,strobeDown));
      //strip.show();
      trapDelay(20);
    }
    else if(captureDelay.remaining() <= 6500) {
      //strip.fill(//strip.Color(0,255,90,strobeUp));
      //strip.show();
      trapDelay(20);

      //strip.fill(//strip.Color(strobeDown,strobeDown,strobeDown,strobeDown));
      //strip.show();
      trapDelay(20);
    }
    else if(captureDelay.remaining() <= 7000) {
      //strip.fill(//strip.Color(0,255,140,strobeUp));
      //strip.show();
      trapDelay(20);

      //strip.fill(//strip.Color(strobeDown,strobeDown,strobeDown,strobeDown));
      //strip.show();
      trapDelay(20);
    }
    else {
      ////strip.fill(//strip.Color(strobeUp,strobeUp,strobeUp,strobeUp));
      //strip.fill(//strip.Color(0,255,154,strobeUp));
      //strip.show();
      trapDelay(20);

      //strip.fill(//strip.Color(strobeDown,strobeDown,strobeDown,strobeDown));
      //strip.show();
      trapDelay(20);
    }

    // Smoke on
    if(timer1Secs > 0.5 && smokeActive == false && smoke == true) {
      smokeOn();
      smokeActive = true;
    }
    
    // Close the door to match the end of the sound effects.
    if(timer1Secs > 6.7 && capturing == true && b_doorsOpen == true) {
      smokeOff();
      smokeActive = false;
      
      if(b_doorsOpen == true) {
        trapDoors("close");

        b_doorsOpen = false;
      }
    } 

    // When the smoke machine is turned off, it still buzzes a slight bit. If you keep on turning it off, it will stop the humming/buzz.
    if(b_doorsOpen == false && smoke == true) {
      smokeOff();
    }
    
    if(timer1Secs > 7.4 && b_light == true) {
       //strip.fill(//strip.Color(0,0,0,0));
       //strip.show();
  
       digitalWrite(ledRed, HIGH);
       trapDelay(redLEDPause);
       digitalWrite(ledRed, LOW);
       trapDelay(redLEDPause);
       digitalWrite(ledRed, HIGH);
       trapDelay(redLEDPause);
       digitalWrite(ledRed, LOW);
       trapDelay(redLEDPause);
       digitalWrite(ledRed, HIGH);
       trapDelay(redLEDPause);
       trapDelay(1000);

       b_light = false;
       capturing = false;
    }
  }

  // Smoke off if it is still on for some reason.
  if(smokeActive == true) {
    smokeOff();
  }

  if(b_light == true) {
    // NeoPixels off
    //strip.fill(//strip.Color(0,0,0,0));
    //strip.show();
  }

  preLoopCaptured();
}

void trapCapture_1989() {
  captureDelay.start(8000);
  boolean capturing = true;
  int strobeUp = 95;
  int strobeDown = 50;
  int strobeCycle = 0;
    
  musicPlayer.stopPlaying();
  b_volume = true;
  musicPlayer.setVolume(volumeL, volumeR);
  musicPlayer.startPlayingFile("cap1989.mp3");
  
  // NeoPixels burst
  for (uint8_t i=0; i<255; i=i+5) {
    if(i>102) {
      //strip.fill(//strip.Color(255,i,255,i));
    } 
    else {
      //strip.fill(//strip.Color(255,102,255,i));
    }
    
    //strip.show();

    trapDelay(1);
  }

  boolean b_doorsOpen = true;
  boolean b_light = true;
  float timer1Secs = 0;
  startTimer1();
  
  while(capturing == true) {   
    timer1Secs = getTimer1Seconds(); 
        
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
      //strip.fill(//strip.Color(strobeUp,strobeUp,strobeUp,strobeUp));
      //strip.show();
      trapDelay(20);
      
      //strip.fill(//strip.Color(strobeDown,strobeDown,strobeDown,strobeDown));
      //strip.show();
      trapDelay(20);
    }
    else if(captureDelay.remaining() <= 6000) {
      //strip.fill(//strip.Color(255,102,255,strobeUp));
      //strip.show();
      trapDelay(20);

      //strip.fill(//strip.Color(strobeDown,strobeDown,strobeDown,strobeDown));
      //strip.show();
      trapDelay(20);
    }
    else if(captureDelay.remaining() <= 6500) {
      //strip.fill(//strip.Color(140,255,140,strobeUp));
      //strip.show();
      trapDelay(20);

      //strip.fill(//strip.Color(strobeDown,strobeDown,strobeDown,strobeDown));
      //strip.show();
      trapDelay(20);
    }
    else if(captureDelay.remaining() <= 7000) {
      //strip.fill(//strip.Color(255,215,90,strobeUp));
      //strip.show();
      trapDelay(20);

      //strip.fill(//strip.Color(strobeDown,strobeDown,strobeDown,strobeDown));
      //strip.show();
      trapDelay(20);
    }
    else {
      //strip.fill(//strip.Color(strobeUp,strobeUp,strobeUp,strobeUp));
      //strip.show();
      trapDelay(20);

      //strip.fill(//strip.Color(strobeDown,strobeDown,strobeDown,strobeDown));
      //strip.show();
      trapDelay(20);
    }

    // Smoke on
    if(timer1Secs > 0 && smokeActive == false && smoke == true) {
      smokeOn();
      smokeActive = true;
    }
    
    // Close the door to match the end of the sound effects.
    if(timer1Secs > 6 && capturing == true && b_doorsOpen == true) {
      smokeOff();
      smokeActive = false;
      
      if(b_doorsOpen == true) {
        trapDoors("close");

        b_doorsOpen = false;
      }
    } 

    // When the smoke machine is turned off, it still buzzes a slight bit. If you keep on turning it off, it will stop the humming/buzz.
    if(b_doorsOpen == false && smoke == true) {
      smokeOff();
    }
    
    if(timer1Secs > 6.7 && b_light == true) {
       //strip.fill(//strip.Color(0,0,0,0));
       //strip.show();
  
       digitalWrite(ledRed, HIGH);
       trapDelay(redLEDPause);
       digitalWrite(ledRed, LOW);
       trapDelay(redLEDPause);
       digitalWrite(ledRed, HIGH);
       trapDelay(redLEDPause);
       trapDelay(1000);

       b_light = false;
       capturing = false;
    }
  }

  // Smoke off if it is still on for some reason.
  if(smokeActive == true) {
    smokeOff();
  }

  if(b_light == true) {
    // NeoPixels off
    //strip.fill(//strip.Color(0,0,0,0));
    //strip.show();
  }

  preLoopCapturedBeeping();

  loopCapturedReset();
}

void trapCapture_1984() {
  captureDelay.start(8000);
  boolean capturing = true;
  int strobeUp = 95;
  int strobeDown = 50;
  int strobeCycle = 0;
    
  musicPlayer.stopPlaying();
  b_volume = true;
  musicPlayer.setVolume(volumeL, volumeR);
  musicPlayer.startPlayingFile("cap1984.mp3");

  // NeoPixels burst
  for (uint8_t i=0; i<255; i=i+5) {
    if(i>102) {
      //strip.fill(//strip.Color(255,i,255,i));
    } 
    else {
      //strip.fill(//strip.Color(255,102,255,i));
    }
    
    //strip.show();

    trapDelay(1);
  }

  boolean b_doorsOpen = true;
  float timer1Secs = 0;
  startTimer1();
  
  while(capturing == true) {   
    timer1Secs = getTimer1Seconds();
     
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
      //strip.fill(//strip.Color(strobeUp,strobeUp,strobeUp,strobeUp));
      //strip.show();
      trapDelay(20);
      
      //strip.fill(//strip.Color(strobeDown,strobeDown,strobeDown,strobeDown));
      //strip.show();
      trapDelay(20);
    }
    else if(captureDelay.remaining() <= 6000) {
      //strip.fill(//strip.Color(255,102,255,strobeUp));
      //strip.show();
      trapDelay(20);

      //strip.fill(//strip.Color(strobeDown,strobeDown,strobeDown,strobeDown));
      //strip.show();
      trapDelay(20);
    }
    else if(captureDelay.remaining() <= 6500) {
      //strip.fill(//strip.Color(140,255,140,strobeUp));
      //strip.show();
      trapDelay(20);

      //strip.fill(//strip.Color(strobeDown,strobeDown,strobeDown,strobeDown));
      //strip.show();
      trapDelay(20);
    }
    else if(captureDelay.remaining() <= 7000) {
      //strip.fill(//strip.Color(255,215,90,strobeUp));
      //strip.show();
      trapDelay(20);

      //strip.fill(//strip.Color(strobeDown,strobeDown,strobeDown,strobeDown));
      //strip.show();
      trapDelay(20);
    }
    else {
      //strip.fill(//strip.Color(strobeUp,strobeUp,strobeUp,strobeUp));
      //strip.show();
      trapDelay(20);

      //strip.fill(//strip.Color(strobeDown,strobeDown,strobeDown,strobeDown));
      //strip.show();
      trapDelay(20);
    }

    // Smoke on
    if(timer1Secs > 0.5 && smokeActive == false && smoke == true) {
      smokeOn();
      smokeActive = true;
    }

    if(timer1Secs > 7.5 && capturing == true) {
      smokeOff();
      smokeActive = false;
      
      //strip.fill(//strip.Color(255,255,255,255));
      //strip.show();  

      if(b_doorsOpen == true) {
        trapDoors("close");

        b_doorsOpen = false;
      }
    } 

    // When the smoke machine is turned off, it still buzzes a slight bit. If you keep on turning it off, it will stop the humming/buzz.
    if(b_doorsOpen == false && smoke == true) {
      smokeOff();
    }
    
    if(musicPlayer.stopped() && capturing == true) {
      capturing = false;
    }
  }

  // Smoke off if it is still on for some reason.
  if(smokeActive == true) {
    smokeOff();
  }
  
  // NeoPixels off
  //strip.fill(//strip.Color(0,0,0,0));
  //strip.show();

  trapDelay(5000);
  
  preLoopCapturedBeeping();

  loopCapturedReset();
}

void trapDoors(String mode) {
  /*
  serialOutput("Trap Doors: " + mode);
  
  //servoR.attach(SERVO_RIGHT_PIN);
  //servoL.attach(SERVO_LEFT_PIN);
  
  if(mode=="open") {
    servoR.write(servoOpenR);
    servoL.write(servoOpenL);
  } 
  else {
    servoL.write(servoCloseL);
    trapDelay(doorDelay);
    servoR.write(servoCloseR);
  }

  //delay(doorWait);
  
  //millisDelay doorsTimer;
  //boolean b_wait = true;
  
  //doorsTimer.start(doorWait);
  
 // while(b_wait == true) {
 //   if(doorsTimer.justFinished()) {
 //     b_wait = false;
   
 //     servoR.detach();
 //     servoL.detach();      
 //   }
 // }
 */
}

void trapReset() {
  // Music player stop
  musicPlayer.stopPlaying();
  
  // Make sure doors are closed
  trapDoors("close");
  
  rSwitchLastPosition = 0;
  
  // Yellow and Red LEDs off
  digitalWrite(ledRed, LOW);
  digitalWrite(ledYellow, LOW);
  
  // NeoPixels off
  //strip.fill(//strip.Color(0,0,0,0));
  //strip.show();
  
  // Bargraph reset
  for(uint8_t b = 12; b > 0; b--) {
    bar.setBar(23 - b + 1, LED_OFF);
    bar.writeDisplay();
    trapDelay(50);
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

  serialOutput("** RESET TRAP (State: 0)**");
}

void loopOpen() {
  if(musicPlayer.stopped()) {
    autoreset = true;
    serialOutput(F("\n** TIMEOUT **"));
  }
  
  if(opened == false) {
    for (uint8_t i=0; i<250; i=i+10) {
      //strip.fill(//strip.Color(255,102,255,i));
      //strip.show();
      trapDelay(1);
    }
    
    for(uint8_t i=240; i>0; i=i-10) {
      //strip.fill(//strip.Color(255,102,255,i));
      //strip.show();
      trapDelay(1);
    }
    
    opened = true;
  }
  
  // Flash NeoPixels randomly
  if(millis() > whiteFlashTime) {
    whiteFlashTime = millis() + 50;
    //strip.setPixelColor(activeFlasher, //strip.Color(255,102,255,255));
    //strip.show();
    trapDelay(30);
    //strip.setPixelColor(activeFlasher, //strip.Color(255,102,255,0));
    //strip.show();
    activeFlasher = random(0, 22);
  }  
}

void preLoopCapturedBeeping() {
  // Play bargraph SFX
  musicPlayer.stopPlaying();
  b_volume = false;
  musicPlayer.setVolume(volumeMidL, volumeMidR);
  musicPlayer.startPlayingFile("bargraph.mp3");
      
  // Fill bargraph
  for(uint8_t b = 0; b < 12; b++) {
    bar.setBar(23 - b, LED_YELLOW);  
    bar.writeDisplay();
    trapDelay(20);
  }

  // Yellow LED on
  digitalWrite(ledYellow, HIGH);
  
  // Wait 1.5 second
  trapDelay(1500);
    
  // Update trap state
  trapState = 2;
  beeping = true;
  serialOutput("** FULL TRAP (State: 2 FLASHING) **");
  serialOutput("WAITING FOR INPUT OR TIMEOUT");
}

void preLoopCaptured() {
  if(rSwitchCurrentPosition != 5 && rSwitchCurrentPosition != 6) {
    // Red LED
    musicPlayer.stopPlaying();
    b_volume = true;
    musicPlayer.setVolume(volumeL, volumeR);
    musicPlayer.startPlayingFile("beep1989.mp3");
    
    digitalWrite(ledRed, HIGH);
    trapDelay(redLEDPause);
    digitalWrite(ledRed, LOW);
    trapDelay(redLEDPause);
    digitalWrite(ledRed, HIGH);
    trapDelay(redLEDPause);
    trapDelay(1000);
  }
  
  // Play bargraph SFX
  musicPlayer.stopPlaying();
  b_volume = false;
  musicPlayer.setVolume(volumeMidL, volumeMidR);
  musicPlayer.startPlayingFile("bargraph.mp3");
      
  // Fill bargraph
  for(uint8_t b = 0; b < 12; b++) {
    if(rSwitchCurrentPosition != 5 && rSwitchCurrentPosition != 6) {
      bar.setBar(23 - b, LED_YELLOW);  
    }
    else {
      bar.setBar(23 - b, LED_GREEN);  
    }
    
    bar.writeDisplay();
    trapDelay(20);
  }

  // Yellow LED on
  digitalWrite(ledYellow, HIGH);
   
  // Update trap state
  trapState = 2;
  beeping = false;
  serialOutput("** FULL TRAP (State: 2 FLASHING) **");
  serialOutput("WAITING FOR INPUT OR TIMEOUT");
}

void loopCapturedReset() {
  musicPlayer.stopPlaying();
  b_volume = true;
  musicPlayer.setVolume(volumeL, volumeR);
  musicPlayer.startPlayingFile("beepzap.mp3");

  // Start timer for sparks
  sparkDelay.start(4000); 

  // Timer for the Red LED.
  redLED.start(redLEDTimer);
}

void loopCaptured() {
  // Beeping stops when the sound effect has finished playing, which is a 1 minute long beeping sound file.
  if(musicPlayer.stopped() && rSwitchCurrentPosition == 8) {
    loopCapturedReset();
  }
  
  if(musicPlayer.stopped() && rSwitchCurrentPosition != 8) {
      serialOutput("Beeping stopped, make RED led full on");

      // Solid red light once beeping stops
      digitalWrite(ledRed, HIGH);
      
      // Update trap state
      beeping = false;
      serialOutput("** FULL TRAP (State: 2 IDLE) **");
  } 
  else {
    // Fade the red light in and out during beeps      
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
      serialOutput("Blue & purple sparks");
    }
    
    if(sparkTime.justFinished()) {
      sparking = false;
    }
    else if(sparking == true) { 
      if(s_timer2.remaining() < 1) {
        if(s_timer1.remaining() < 1 && sparkOn == false) {
          activeFlasher = random(0, 22);
          //strip.setPixelColor(activeFlasher, //strip.Color(0,0,255,0));
          //strip.show();
          
          activeFlasherPurple = random(0, 22);
          //strip.setPixelColor(activeFlasherPurple, //strip.Color(202,0,255,0));
          //strip.show();
          sparkOn = true;
          
          s_timer1.start(sparkDelayShort);
        }
        else if(s_timer1.remaining() < 1 && sparkOn == true) {
          //strip.setPixelColor(activeFlasher, //strip.Color(0,0,0,0));
          //strip.show();

          //strip.setPixelColor(activeFlasherPurple, //strip.Color(0,0,0,0));
          //strip.show();
          sparkOn = false;

          s_timer1.start(sparkDelayShort);
          s_timer2.start(sparkDelayLong);
        }
      }
    }
  }
}

void smokeOn() {
  // Only using smoke in certain modes. See notes at the top.
  if(rSwitchCurrentPosition == 1 || rSwitchCurrentPosition == 3 || rSwitchCurrentPosition == 5 || rSwitchCurrentPosition == 7 || rSwitchCurrentPosition == 8) {
    musicPlayer.GPIO_pinMode(smokePin, OUTPUT);
    musicPlayer.GPIO_digitalWrite(smokePin, HIGH);
  }
}

void smokeOff() {
  musicPlayer.GPIO_digitalWrite(smokePin, LOW);
  musicPlayer.GPIO_pinMode(smokePin, INPUT);
}

// Changes the modes of the trap. Modes only change when in state 0 (resting state). If you change the state while in another state, it will not change until the trap resets itself.
void checkSwitchPosition() {
  if(trapState == 0 || rSwitchCurrentPosition == 7 || rSwitchCurrentPosition == 8) {
    rSwitchLastPosition = rSwitchCurrentPosition;
    int led_colour = LED_OFF;
    boolean led_mix = false;
    
    if(digitalRead(r1) == LOW) {
      rSwitchCurrentPosition = 1;
      led_colour = LED_YELLOW;
      smoke = true;
    }
    
    if(digitalRead(r2) == LOW) {
      rSwitchCurrentPosition = 2;
      led_colour = LED_YELLOW;
      led_mix = true;
      smoke = false;
    }
      
    if(digitalRead(r3) == LOW) {
      rSwitchCurrentPosition = 3;
      led_colour = LED_RED;
      smoke = true;
    }
      
    if(digitalRead(r4) == LOW) {
      rSwitchCurrentPosition = 4;
      led_colour = LED_RED;
      led_mix = true;
      smoke = false;
    }
      
    if(digitalRead(r5) == LOW) {
      rSwitchCurrentPosition = 5;
      led_colour = LED_GREEN;
      smoke = true;
    }
      
    if(digitalRead(r6) == LOW) {
      rSwitchCurrentPosition = 6;
      led_colour = LED_GREEN;
      led_mix = true;
      smoke = false;
    }
      
    if(digitalRead(r7) == LOW) {
      rSwitchCurrentPosition = 7;
    }
      
    if(digitalRead(r8) == LOW) {
      rSwitchCurrentPosition = 8;
    }

    if(rSwitchCurrentPosition != rSwitchLastPosition) {
      serialOutput(String(rSwitchCurrentPosition));

      if(setupMode == false) {
        bargraphOff();

        trapReset();

        if(rSwitchCurrentPosition != 7 && rSwitchCurrentPosition != 8) {
            bargraphOperate(led_colour, led_mix);
        }
      }
      /*
      if(setupMode == false) {
        if(musicPlayer.playingMusic == false) {
            b_volume = false;
            musicPlayer.setVolume(volumeMidL, volumeMidR);
            musicPlayer.startPlayingFile("modes.mp3");
        }
      }
      */
    }
    
  }
}

void bargraphOperate(int colour, boolean mix) {
  b_timer.start(b_timer_length);
    
  int l_int = 1;

  for(int l = 0; l < 12; l++) {
    if(mix == true) {
      l++;
    }
    
    bar.setBar(23 - l, colour);
    bar.writeDisplay();
  }
}

void bargraphOff() {
  for(int l = 0; l < 12; l++) {
    bar.setBar(23 - l, LED_OFF);
    bar.writeDisplay();
  }
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

void checkVolumeEncoder() {
  //if(trapState == 0) {

    if(millis() > volumeBuffer) {
      volumeBuffer = millis() + 1;
      rEncoderState = digitalRead(rEncoderA); // reads the "current" state of the clock pin
        
      // If the previous and the current state of the clock are different, that means a step has occurred.
      if(rEncoderState != rEncoderLastState) {
        // If the data state is different to the clock state, that means the encoder is rotating clockwise.
        if(digitalRead(rEncoderB) != rEncoderState) {
          if(volumeL - volInt < volumeMax) {
             volumeL = volumeMax;
          }
          else {
            volumeL = volumeL - volInt;
          }
    
          if(volumeR - volInt < volumeMax) {
            volumeR = volumeMax;
          }
    
          if(volumeMidL - volInt < volumeMaxMid) {
            volumeMidL = volumeMaxMid;
          }
          else {
            volumeMidL = volumeMidL - volInt;
          }
    
          if(volumeMidR - volInt < volumeMaxMid) {
            volumeMidR = volumeMaxMid;
          }
          else {
            volumeMidR = volumeMidR - volInt;
          }
          
          if(volumeL < volumeMax) {
            volumeL = volumeMax;
          }
    
          if(volumeR < volumeMax) {
            volumeR = volumeMax;
          }
    
          if(volumeMidL < volumeMaxMid) {
            volumeMidL = volumeMaxMid;
          }
    
          if(volumeMidR < volumeMaxMid) {
            volumeMidR = volumeMaxMid;
          }
          
          rEncoderCount ++;
    
          serialOutput(String(volumeL));
          serialOutput(" -> Increase volume");
        }
        else {
          volumeL = volumeL + volInt;
          volumeR = volumeR + volInt;
    
          volumeMidL = volumeMidL + volInt;
          volumeMidR = volumeMidR + volInt;
    
          if(volumeR > volumeMin) {
            volumeR = volumeMin;
          }
    
          if(volumeL > volumeMin) {
            volumeL = volumeMin;
          }
    
          if(volumeMidL > volumeMidMin) {
            volumeMidL = volumeMidMin;
          }
    
          if(volumeMidR > volumeMidMin) {
            volumeMidR = volumeMidMin;
          }

          rEncoderCount --;
          
          serialOutput(String(volumeL));
          serialOutput(" -> Decrease volume");
        }
    
        if(b_volume == true) {
          musicPlayer.setVolume(volumeL, volumeR);
        }
        else {
          musicPlayer.setVolume(volumeMidL, volumeMidR);
        }

        /*
        if(setupMode == false) {
          if(musicPlayer.playingMusic == false) {
              b_volume = false;
              musicPlayer.setVolume(volumeMidL, volumeMidR);
              musicPlayer.startPlayingFile("volume.mp3");
          }
        }
        */
      }

      rEncoderLastState = rEncoderState;
    }
  //}
}

void trapDelay(int bgDelayTime) {
  //delay(bgDelayTime);

  millisDelay trapDelay;
  boolean bg_delay = true;

  trapDelay.start(bgDelayTime);
  
  while(bg_delay == true) {
    //checkVolumeEncoder();
    
    if(trapDelay.justFinished()) {
      bg_delay = false;
    }
  }
}

/*  
 // https://forum.arduino.cc/t/millis-gives-strange-results/323221/10
 A work around the neopixels library and it's conflicts with the milliseconds timer slows timing when you change or add more lights etc.
 This timer1 is separate and can count up up to 8.388 seconds, after that it will miss a overflow and will read low. 8 seconds is enough for all our timing needs during the capture phases where timing is the most important.
*/
void startTimer1() {
  // reset Timer 1
  TCCR1A = 0;
  TCCR1B = 0;
  
  // zero it
  TCNT1 = 0;  
  TIFR1 |= bit (TOV1);  // clear overflow flag
  
  // start Timer 1
  TCCR1B =  bit (CS10) | bit (CS12);  //  prescaler of 1024
}
  
unsigned long getTimer1Reading() {
  unsigned long elapsed = TCNT1;
  
  if(TIFR1 &  bit (TOV1)) {
     elapsed += 65536;
  }
  
  return elapsed;    
}

float getTimer1Seconds() {
  float secs = (1.0 / F_CPU * 1024) * getTimer1Reading ();

  return secs;
}

void serialOutput(String text) {
  //if(Serial.available()) {
    Serial.println(text);
    //Serial.println(F(text));
  //}
}
