/********************************************************
 ARDUINO CODE FOR CHARLESWORTH DYNAMICS GHOST TRAP
 v3.1 (Michael Rajotte) - April 2022.
 -  Added working LED to trap pedal.
 -  Spark sequence switched over to timer1 for more timing accuracy.
 -  (Optional) The trap will smoke automatically during the sparks sequence when in smoke capture modes. (mode 7 and 8 are still manually triggered smoke). Uncomment code in the capture loop to enable this feature.

Trap Pedal LED and switch:
Replace the existing off - (on) switch provided by Charlesworth for the trap pedal with one that is on - (off) (mouser part# 642-MBZ101A01A02X01) You may need to take the roller switch off the old one to put on the new one. 
Then on the wire that connects in the middle of the foster connection, run it to the new on - (off) switch then, make another wire run to another pin on the switch running to the LED (it must be a 5mm red LED with specs between 1.8v-2.2V) on the other side of the pedal. Put a 180ohm resistor between the LED and the trap pedal switch. The wire that connects on the outside of the foster connection, run that one to the ground on the LED. 
On the trap side, you can remove the wires that split into the side trap trigger switch that go to the foster connector, and instead run them to the ground rail (the ground wire) and the LED wire to pin #5 on the music maker.

 v3.0 (Michael Rajotte) - March 2022.
 -  Based on EctoLabs/Dave Tremaine original code and his idea of using NeoPixels for main trap lighting, in which his code was modified from original
 code by Jeremy Williams 2016

This has been created with the following micro controller setup:
Arudino Mega (I stack a Mega protoshield (adafruit part# 192) on top of the mega (do not forget to solder the pins) and then a music maker on top of the protoshield.) I included in my link a STL file to 3D print to hold the Mega and give it a bit of ground clearance over the wheels and v-hook bolts. I cut the ends flush with the nuts on my v-hook to additional clearance.)
Adafruit Metro Mini (just a mini version of a Arduino Uno) (for the Adafruit NeoPixels)
PCA9685 Controller (for the servos)

******* WIRING CHANGES FROM CHARLESWORTH DYNAMICS ORIGINAL GHOST TRAP GUIDE *******
Yellow LED: Move to pin 8 on the music maker shield.
Bargraph pins: yellow and blue wires on analog 5 and 4 get moved to: (yellow) -> SCL on music maker. (blue) -> SDA music maker.
Rotary encoder (if you need one: adafruit part #377): Make a new 3 pin plug: middle pin goes to ground. The 2 outside pins: When viewing the 3 pins from the top (button side) of the encoder with the 3 pins facing north (top), the outside left pin goes to pin #39 and right side pin goes to pin #38. on the mega. The 2 pin side stays the same as CHARLESWORTH DYNAMICS updated guide.
8 way rotary switch from Adafruit (part #2925): connect to pins: 22 to 29 on the Mega. #1 pin on the switch goes to #22 pin work your way up to #8 pin on switch going to pin 29. Then centre pin goes to ground on the breadboard power/ground rail.

NeoPixels:
The Neopixel's are control by a Metro Mini from Adafruit. The NeoPixels can conflict with anything that requires interrupts (random crashing and resets), such as the Servo motors and the music maker. This is due to how the NeoPixel library operates and turns off interrupts on pins. 
The Mega will send commands over the tx channels to tell it what to do. Sometimes the mega will wait for a response back before it continues.
You can connect any Arduino board, it can be a Uno, Mini, Mega or whatever. It just needs to have a TX and RX pins and attach the NeoPixels to a digital pin.
For my example I did the following: NeoPixels connect to pin #5 on a Adafruit Metro Mini, and TX (mini: pin #1) to RX1 (mega pin# 19) and RX (mini pin #0) to TX (mega pin #18). Then run 5v and ground from the metro mini into the breadboard power/ground rails.

NeoPixel wiring: Follow Ectolabs guide of chaining them together. In a sense it goes like this: (I use a 3 pin plug) with the ground/5v power wires going to the breadboard power/ground rails and the other going to the Adafruit Metro Min pin #5: 
Pin #5 on a Adafruit Metro Mini -> 470ohm resistor -> (NeoPixel #1) IN. (NeoPixel #1) OUT -> (NeoPixel #2) IN. (NeoPixel #2) OUT -> (NeoPixel #3) IN.
(NeoPixel #3) GND -> (NeoPixel #2) OUT -> (NeoPixel #2) GND -> (NeoPixel #1) OUT -> (NeoPixel #1) GND -> GROUND (on the proto shield ground rail)
(NeoPixel #3) PWR -> (NeoPixel #2) PWR -> (NeoPixel #1) PWR -> 5V (on the breadboard power/ground rails)

Servos:
The servos are controlled separately by a I2C PCA9685 controller, so they do not interfere with the NeoPixels and or the Music Maker. (no more servo twitching or crashing). 
This needs to be connected to SDA and SCL on the Mega and the Left servo on PWM 0 and Right servo on PWM 1. If you switch it up, just change the pin settings in the code.
Connect 5V and ground on the I2C to the breadboard power/ground rails (This only powers the board and not the servos).
Then I run a separate 5v and ground on the middle/top of the PCA9685 with 2pin terminal block to the breadboard power/ground rails. This only powers the servos and not the PCA9685 controller. Then wire the servos power and ground to the appropriate V+ and GND pins under the PWM pins.

Music Maker:
Before attaching to the Mega, you need to connect the MISO, SCK and MOSI on the ICSP side. Optionally you can cut the #11, #12, #13 connectors next to them to free up those pins on the Music Maker, but it is not required since we do not use them.
Rear red LED is moved from analog #1 pin to digital i/o #3. This could also be moved to pin #6 as well as #3 is a interrupt pin which by default is used by the DREQ on the Music Maker. However the original guide had us move the DREQ to pin #2 on the Music Maker so I just left my rear red led connected to pin #3 as it was unused. If you do not follow the original guide, I would leave DREQ on the music maker shield at the default #3. Then just change the DREQ pin number to 3 in the code.
NOTE: Connect the db+12 and db+6 on the music maker to prevent the hissing sound during the bootup sequence. This will make the trap louder but you can always adjust volume. I prefer it loud to help hide the smoke pump motor while in smoke capturing modes.
NOTE: When using the music maker with +12 and +6db, at max volume, basically anything less than 40 volume level can crash the arduino if you are powering it by USB only.
if you plug in your battery to the power booster and flip the power switch on, it will solve the problem (even with usb still plugged in).

 Ghost trap kit designed by Sean Charlesworth 2016-2018
 With original Programming and electronics by Jeremy Williams 2016
  
 ********************************************************/

#include <millisDelay.h> 
#include <SPI.h>
#include <Adafruit_VS1053.h>
#include <Adafruit_LEDBackpack.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <SD.h>
#include <Adafruit_PWMServoDriver.h>

// Volume
// (0 = loudest, 255 = quietest)
uint8_t volumeMax = 0; // Max volume
uint8_t volumeMaxMid = 30; // Max mid volume
uint8_t volumeMidMin = 90; // Minimum mid volume
uint8_t volumeMin = 60; // Minimum volume
uint8_t volumeR = 0;
uint8_t volumeL = 0;
uint8_t volumeMidL = 30; // Used on bargraph sound effects.
uint8_t volumeMidR = 30; // Used on bargraph sound effects.
uint8_t volInt = 1;
long volumeBuffer = 0;
boolean b_volume = false; // used to know if playing high or medium volume when adjusting the volume.

// Rear Red LED.
byte ledRed = 3; // on the music maker.
millisDelay redLED;
millisDelay redLEDPauseTimer;
boolean redLEDState = true;
long redFlashTime = 0;

// Yellow LED
#define ledYellow 8 // on the music maker.

// Music Maker Shield
#define SHIELD_RESET -1 // VS1053 reset pin (unused!)
#define SHIELD_CS 7 // VS1053 chip select pin (output)
#define SHIELD_DCS 6 // VS1053 Data/command select pin (output)
#define CARDCS 4 // Card chip select pin
#define DREQ 2 // VS1053 Data request, ideally an Interrupt pin
Adafruit_VS1053_FilePlayer musicPlayer = Adafruit_VS1053_FilePlayer(SHIELD_RESET, SHIELD_CS, SHIELD_DCS, DREQ, CARDCS);

// Various trap settings.
int trapState = 0;
boolean autoreset = false;
boolean remotePressed = false;
boolean beeping = false;
unsigned long captureStart = 0;
long debounceBuffer = 0;

// Setup non-blocking timers
millisDelay captureDelay;
millisDelay bgDelay;

// Sparks
boolean sparking = false;
boolean b_s_timer1 = false;
int sparkStart = 4500;
float s_timer1Secs = 0;
millisDelay s_timer1;
millisDelay s_timer2;

// Smoke
byte smokePin = 6; // Smoke machine -> gpio on the music maker.
millisDelay smokeTimer;
int smokeOnLength = 5000;
long smokeToggleTime = 0; // Not used anymore?
boolean b_smokeOn = false;
boolean smoke = true;
boolean smokeActive = false;

// Feature toggles
boolean sparkOn = false;
boolean bLEDUp = true;
boolean bLEDPause = false;
boolean bLEDStart = true;
boolean setupMode = true;

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
boolean isBargraphOn = false;

// Pedal
#define pedalLED 5
boolean b_pedalConnected = false;
boolean b_pedalDown = false;

// Red LED pulse control.
int rLED = 0; // red led starts at 0 (off).
int rLEDHigh = 255; // high light setting during the fade up.
int rLEDLow = 15; // low light setting during the fade down.
int redLEDTimer = 2; // millisecond speed control for the fade.
int redLEDPause = 20; //125; // milliseconds for pausing the red led. Used when reaching full value of low or high.
int redLEDLongPause = 125; // milliseconds for pausing the red led. Used when reaching full value of low or high.
int rLEDMultiplier = 5;

// Spark delays.
int sparkDelayShort = 30;
int sparkDelayLong = 100;

// Volume control on the right side rotary encoder.
#define rEncoderA 38 // clk
#define rEncoderB 39 // data
static uint8_t prevNextCode = 0;
static uint16_t store=0;

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

// Trap doors
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
uint8_t servoL = 0; // pwm pin 0 on the PCA9586 board
uint8_t servoR = 1; // pwm pin 1 on the PCA9586 board
int servoClose = 0;
int servoOpen = 120;
int doorDelay = 30;

void setup() {
  // Start debug output.
  Serial.begin(9600);

  // This will be our communication to the controller which controls the NeoPixels.
  Serial1.begin(9600);
  
  // Initialise music player
  if(!musicPlayer.begin()) {
    //Serial.println(F("Couldn't find music player, do you have the right pins defined?"));
    while (1);
  }

  //Serial.println(F("Music player found"));

  // Check SD card
  if(!SD.begin(CARDCS)) {
    //Serial.println(F("SD failed, or not present"));
    while (1);
  }
  //Serial.println(F("SD Card OK"));

  musicPlayer.GPIO_pinMode(activationSwitch, INPUT);

  // Interrupt pin
  if(!musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT)) {
    //Serial.println(F("DREQ pin is not an interrupt pin"));
  }
  
  // Connect the servos.
  pwm1.begin();
  pwm1.setPWMFreq(SERVO_FREQ);
 
  // Calibrate trap doors into the closed position.
  trapDoors("close");
  
  // Rotary encoder for adjusting the volume of the track.
  pinMode(rEncoderA, INPUT_PULLUP);
  pinMode(rEncoderB, INPUT_PULLUP);
    
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
  
  // Make sure the NeoPixels are off.
  Serial.write(1);   

  // Make sure smoke pump is inactive
  smokeOff();

  // Check if the trap pedal is connected.
  // Can remove this?
  checkForPedalLED();
  
  // Set the pedal LED to off.
  pinMode(pedalLED, OUTPUT);
  digitalWrite(pedalLED, LOW);
  
  // Initalise the bargraph.
  bar.begin(0x70);
  bar.writeDisplay();
}

void checkForPedalLED() {
  // Check if the trap pedal is connected.
  pinMode(pedalLED, INPUT_PULLUP);
  digitalWrite(pedalLED, LOW);

  if(digitalRead(pedalLED) == LOW) {
    // The pedal is connected for the trap.
    b_pedalConnected = true;
    //Serial.println(F("Trap pedal connected."));
  }  
  else {
    //Serial.println(F("Trap pedal NOT connected."));
  }
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
    //Serial.println(F("Startup complete (State: 0)"));
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

    if(isBargraphOn == false && rSwitchCurrentPosition != 7 && rSwitchCurrentPosition != 8) {
      if(b_timer.justFinished()) {
       bargraphOff();
      }
    }
    else if(isBargraphOn == true && b_timer.justFinished()) {
      bargraphOn();
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
      else if(trapState == 1) {
        // The loop when the trap is open and waiting.
        loopOpen();
      } 
      else if(trapState == 2 && beeping == true) {
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
        loopCapturedReset();
      }
      else if(trapState == 2 && beeping == true) {
        loopCaptured();
      }
    }
  }
}

void trapOpen() {
  // Opening trap in Misc mode.
  if(rSwitchCurrentPosition == 5 || rSwitchCurrentPosition == 6) {
    Serial.write(2);   
  }
  else {
    // GB1 or GB2 opening.
    Serial.write(3);   
  }
  
  musicPlayer.stopPlaying();

  if(rSwitchCurrentPosition == 5 || rSwitchCurrentPosition == 6) {
    b_volume = false;
    musicPlayer.setVolume(volumeL, volumeR);
    musicPlayer.startPlayingFile("turn_on.mp3");
  }
  else {
    b_volume = true;
    musicPlayer.setVolume(volumeL, volumeR);
    musicPlayer.startPlayingFile("open.mp3");
  }

  // Open doors
  trapDoors("open");

  // GB 2 Mode or Misc mode.
  if(rSwitchCurrentPosition == 3 || rSwitchCurrentPosition == 4) {
     // GB2 mode.
     trapDelay(100);
     
     trapState = 1;
     //Serial.println(F("** TRAP OPENED (State: 1) **"));
  }
  else {       
    boolean b_trapOpen = true;
    Serial.write(4);
    while(b_trapOpen == true) {
      if(Serial1.available() > 0) {
        b_trapOpen = false;
        
        // Update trap state
        trapState = 1;
        //Serial.println(F("** TRAP OPENED (State: 1) **"));
      }
    }
  }
}

void trapCapture_Misc() {
  captureDelay.start(8000);
  boolean capturing = true;
    
  musicPlayer.stopPlaying();
  b_volume = true;
  musicPlayer.setVolume(volumeL, volumeR);
  musicPlayer.startPlayingFile("capmisc.mp3");

  boolean b_wait = true;

  // NeoPixels burst
  Serial.write(15);
  while(b_wait == true) {
    if(Serial1.available() > 0) {
      b_wait = false;
    }
  }

  // Reset the strobe values.
  Serial.write(7);

  boolean b_doorsOpen = true;
  boolean b_light = true;
  float timer1Secs = 0;
  startTimer1();
  
  while(capturing == true) {   
    timer1Secs = getTimer1Seconds(); 

    // Update strobe values.
    Serial.write(8);
          
    if(captureDelay.remaining() <= 4500) {
      // Strobe colours up.
      Serial.write(16);
      trapDelay(20);
      
      // Strobe down
      Serial.write(10);
      trapDelay(20);
    }
    else if(captureDelay.remaining() <= 6000) {
      // Strobe colours up.
      Serial.write(17);
      trapDelay(20);
      
      // Strobe down
      Serial.write(10);
      trapDelay(20);
    }
    else if(captureDelay.remaining() <= 6500) {
      // Strobe colours up.
      Serial1.write(18);
      trapDelay(20);

      // Strobe down
      Serial1.write(10);
      trapDelay(20);
    }
    else if(captureDelay.remaining() <= 7000) {
      // Strobe colours up.
      Serial1.write(19);
      trapDelay(20);

      // Strobe down
      Serial1.write(10);
      trapDelay(20);
    }
    else {
      // Strobe colours up.
      Serial1.write(20);
      trapDelay(20);

      // Strobe down
      Serial1.write(10);
      trapDelay(20);
    }

    // Smoke on
    if(timer1Secs > 2 && b_doorsOpen == true && smokeActive == false && smoke == true) {
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
       // NeoPixels off
       Serial1.write(1);
  
       digitalWrite(ledRed, HIGH);
       digitalWrite(pedalLED, HIGH);
       trapDelay(redLEDLongPause);
       digitalWrite(ledRed, LOW);
       digitalWrite(pedalLED, LOW);
       trapDelay(redLEDLongPause);
       digitalWrite(ledRed, HIGH);
       digitalWrite(pedalLED, HIGH);
       trapDelay(redLEDLongPause);
       digitalWrite(ledRed, LOW);
       digitalWrite(pedalLED, LOW);
       trapDelay(redLEDLongPause);
       digitalWrite(ledRed, HIGH);
       digitalWrite(pedalLED, HIGH);
       trapDelay(redLEDLongPause);
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
     Serial1.write(1);
  }

  preLoopCapturedBeeping();

  loopCapturedReset();
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
  
  boolean b_wait = true;

  // NeoPixels burst
  Serial1.write(6);
  while(b_wait == true) {
    if(Serial1.available() > 0) {
      b_wait = false;
    }
  }

  // Reset the strobe values.
  Serial1.write(7);

  boolean b_doorsOpen = true;
  boolean b_light = true;
  float timer1Secs = 0;
  startTimer1();
  
  while(capturing == true) {   
    timer1Secs = getTimer1Seconds(); 
        
    // Update strobe values.
    Serial1.write(8);
          
    if(captureDelay.remaining() <= 4500) {
      // Strobe up
      Serial1.write(9);
      trapDelay(20);
      
      // Strobe down
      Serial1.write(10);
      trapDelay(20);
    }
    else if(captureDelay.remaining() <= 6000) {
      // Strobe pixel colours up
      Serial1.write(11);
      trapDelay(20);

      // Strobe down
      Serial1.write(10);
      trapDelay(20);
    }
    else if(captureDelay.remaining() <= 6500) {
      // Strobe pixel colours up
      Serial1.write(13);
      trapDelay(20);

      // Strobe down
      Serial1.write(10);
      trapDelay(20);
    }
    else if(captureDelay.remaining() <= 7000) {
      // Strobe pixel colours up
      Serial1.write(14);
      trapDelay(20);

      // Strobe down
      Serial1.write(10);
      trapDelay(20);
    }
    else {
      // Strobe up
      Serial1.write(9);
      trapDelay(20);

      // Strobe down
      Serial1.write(10);
      trapDelay(20);
    }

    // Smoke on
    if(timer1Secs > 0 && b_doorsOpen == true && smokeActive == false && smoke == true) {
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
       // NeoPixels off.
       Serial1.write(1);
  
       digitalWrite(ledRed, HIGH);
       digitalWrite(pedalLED, HIGH);
       trapDelay(redLEDLongPause);
       digitalWrite(ledRed, LOW);
       digitalWrite(pedalLED, LOW);
       trapDelay(redLEDLongPause);
       digitalWrite(ledRed, HIGH);
       digitalWrite(pedalLED, HIGH);
       trapDelay(redLEDLongPause);
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
     // NeoPixels off.
     Serial1.write(1);
  }

  preLoopCapturedBeeping();

  loopCapturedReset();
}

void trapCapture_1984() {
  captureDelay.start(8000);
  boolean capturing = true;
    
  musicPlayer.stopPlaying();
  b_volume = true;
  musicPlayer.setVolume(volumeL, volumeR);
  musicPlayer.startPlayingFile("cap1984.mp3");

  boolean b_wait = true;

  // NeoPixels burst
  Serial1.write(6);
  while(b_wait == true) {
    if(Serial1.available() > 0) {
      b_wait = false;
    }
  }

  // Reset the strobe values.
  Serial1.write(7);
  
  boolean b_doorsOpen = true;
  float timer1Secs = 0;
  startTimer1();
  
  while(capturing == true) {   
    timer1Secs = getTimer1Seconds();

    // Update strobe values.
    Serial1.write(8);
          
    if(captureDelay.remaining() <= 4500) {
      // Strobe up
      Serial1.write(9);
      trapDelay(20);

      // Strobe down
      Serial1.write(10);
      trapDelay(20);
    }
    else if(captureDelay.remaining() <= 6000) {
      // Strobe pixels colours up
      Serial1.write(11);
      trapDelay(20);

      // Strobe down
      Serial1.write(10);
      trapDelay(20);
    }
    else if(captureDelay.remaining() <= 6500) {
      // Strobe pixels colours up
      Serial1.write(13);
      trapDelay(20);

      // Strobe down
      Serial1.write(10);
      trapDelay(20);
    }
    else if(captureDelay.remaining() <= 7000) {
      // Strobe pixels colours up
      Serial1.write(14);
      trapDelay(20);

      // Strobe down
      Serial1.write(10);
      trapDelay(20);
    }
    else {
      // Strobe up
      Serial1.write(9);
      trapDelay(20);

      // Strobe down
      Serial1.write(10);
      trapDelay(20);
    }

    // Smoke on
    if(timer1Secs > 0.5 && b_doorsOpen == true && smokeActive == false && smoke == true) {
      smokeOn();
      smokeActive = true;
    }

    if(timer1Secs > 7.8 && capturing == true) {
      smokeOff();
      smokeActive = false;
      
      // NeoPixels off.
      Serial1.write(1);

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
  
  // NeoPixels off.
  Serial1.write(1);

  trapDelay(5000);
  
  preLoopCapturedBeeping();

  loopCapturedReset();
}

void trapDoors(String mode) { 
  //Serial.println(F("Trap Doors: " + mode));

  int pulse = 0;

  if(mode == "open") {
    pulse = map(servoOpen,0,180,150,560);
    pwm1.setPWM(servoR,0,pulse);

    trapDelay(doorDelay);

    pulse = map(20,0,180,150,560);
    pwm1.setPWM(servoL,0,pulse);
  }
  else {
    pulse = map(servoOpen,0,180,150,560);
    pwm1.setPWM(servoL,0,pulse);
    
    trapDelay(doorDelay);
    
    pulse = map(servoClose,0,180,150,560);
    pwm1.setPWM(servoR,0,pulse);
  }
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

  // Trap pedal LED off
  digitalWrite(pedalLED, LOW);
  
  // NeoPixels off
  Serial1.write(1);   

  isBargraphOn = false;
  // Bargraph reset
  for(uint8_t b = 12; b > 0; b--) {
    bar.setBar(23 - b + 1, LED_OFF);
    bar.writeDisplay();
    trapDelay(50);
  }

  // Reset the spark timer flag.
  b_s_timer1 = false;
  
  // Make sure smoke is off
  smokeActive = false;
  smokeToggleTime = 0;
  smokeOff();
  b_smokeOn = false;
  
  // Update trap state
  trapState = 0;
  autoreset = false;

  //Serial.println(F("** RESET TRAP (State: 0)**"));
}

void loopOpen() {
  if(musicPlayer.stopped()) {
    autoreset = true;
    //Serial.println(F("** TIMEOUT **"));
  }

  boolean b_wait = true;
  Serial1.write(5);

  while(b_wait == true) {
    checkVolumeEncoder();

    if(Serial1.available() > 0) {
      b_wait = false;
    }
  }
}

void preLoopCaptured() {
  if(rSwitchCurrentPosition != 5 && rSwitchCurrentPosition != 6) {
    // Red LED
    musicPlayer.stopPlaying();
    b_volume = true;
    musicPlayer.setVolume(volumeL, volumeR);
    musicPlayer.startPlayingFile("beep1989.mp3");
    
    digitalWrite(ledRed, HIGH);
    digitalWrite(pedalLED, HIGH);
    trapDelay(redLEDLongPause);
    digitalWrite(ledRed, LOW);
    digitalWrite(pedalLED, LOW);
    trapDelay(redLEDLongPause);
    digitalWrite(ledRed, HIGH);
    digitalWrite(pedalLED, HIGH);
    trapDelay(redLEDLongPause);
    trapDelay(1000);
  }
  
  // Play bargraph SFX
  musicPlayer.stopPlaying();
  b_volume = false;
  musicPlayer.setVolume(volumeMidL, volumeMidR);
  musicPlayer.startPlayingFile("bargraph.mp3");

  isBargraphOn = true;

  // Fill bargraph
  for(uint8_t b = 0; b < 12; b++) {
    if(rSwitchCurrentPosition == 5 || rSwitchCurrentPosition == 6) {
      bar.setBar(23 - b, LED_GREEN);  
    }
    else {
      bar.setBar(23 - b, LED_YELLOW);  
    }
    
    bar.writeDisplay();
    trapDelay(20);
  }

  // Yellow LED on
  digitalWrite(ledYellow, HIGH);
   
  // Update trap state
  trapState = 2;
  beeping = false;
  //Serial.println(F("** FULL TRAP (State: 2 FLASHING) **"));
  //Serial.println(F("WAITING FOR INPUT OR TIMEOUT"));
}

void preLoopCapturedBeeping() {
  // Play bargraph SFX
  musicPlayer.stopPlaying();
  b_volume = false;
  musicPlayer.setVolume(volumeMidL, volumeMidR);
  musicPlayer.startPlayingFile("bargraph.mp3");

  isBargraphOn = true;
  // Fill bargraph
  for(uint8_t b = 0; b < 12; b++) {
    if(rSwitchCurrentPosition == 5 || rSwitchCurrentPosition == 6) {
      bar.setBar(23 - b, LED_GREEN);  
    }
    else {
      bar.setBar(23 - b, LED_YELLOW);  
    }
     
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
  //Serial.println(F("** FULL TRAP (State: 2 FLASHING) **"));
  //Serial.println(F("WAITING FOR INPUT OR TIMEOUT"));
}

void loopCapturedReset() {
  musicPlayer.stopPlaying();
  b_volume = true;
  musicPlayer.setVolume(volumeL, volumeR);
  musicPlayer.startPlayingFile("beepzap.mp3");

  startTimer1();
  b_s_timer1 = false;

  // Timer for the Red LED.
  redLED.start(redLEDTimer);
}

void loopCaptured() {
  // Beeping stops when the sound effect has finished playing, which is a 1 minute long beeping sound file.
  if(musicPlayer.stopped() && rSwitchCurrentPosition == 8) {
    loopCapturedReset();
  }
  
  if(musicPlayer.stopped() && rSwitchCurrentPosition != 8) {
      //Serial.println(F("Beeping stopped, make RED led full on"));

      // Solid red light once beeping stops
      digitalWrite(ledRed, HIGH);
      
      // Pedal LED is controlled in checkRemote();
      //digitalWrite(pedalLED, HIGH);
      
      // Update trap state
      beeping = false;
      //Serial.println(F("** FULL TRAP (State: 2 IDLE) **"));
  } 
  else {
    // Fade the red light in and out during beeps      
    analogWrite(ledRed, rLED);

    // Pedal LED is controlled in checkRemote();
    //analogWrite(pedalLED, rLED);
    
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

    s_timer1Secs = getTimer1Seconds();
    
    // Blue & purple sparks synched with SFX
    if(s_timer1Secs > 6.5 && sparking == false && b_s_timer1 == false) {      
      startTimer1();
      s_timer1Secs = getTimer1Seconds();

      s_timer1.start(1);
      s_timer2.start(1);
      sparking = true;
      sparkOn = false;
      //Serial.println(F("Blue & purple sparks"));
      b_s_timer1 = true;

      /*
      // Have some more smoke during spark sound effects, which hides the pump motor and gives you a even more smoking trap in smoke capturing modes.
      if(rSwitchCurrentPosition == 1 || rSwitchCurrentPosition == 3 || rSwitchCurrentPosition == 5) {
          smokeOn();
      }
      */
    }

    /*
    if(s_timer1Secs > 4 && sparking == true) {
      // If in smoke capture modes, turn off the smoke to match sound effects.
      if(rSwitchCurrentPosition == 1 || rSwitchCurrentPosition == 3 || rSwitchCurrentPosition == 5) {
        smokeOff();
      }
    }
    */
    
    if(s_timer1Secs > 6 && sparking == true) {
      sparking = false;
      b_s_timer1 = true;

      // Sparks off.
      Serial1.write(22);
    }
    else if(sparking == true) {            
      if(s_timer2.remaining() < 1) {
        if(s_timer1.remaining() < 1 && sparkOn == false) {

          if(rSwitchCurrentPosition == 5 || rSwitchCurrentPosition == 6) {
            // Misc green sparks on.
            Serial1.write(23);
          }
          else {
            // 1984 / 1989 sparks on.
            Serial1.write(21);
          }
         
          sparkOn = true;

          s_timer1.start(sparkDelayShort);
        }
        else if(s_timer1.remaining() < 1 && sparkOn == true) {
          // Sparks off.
          Serial1.write(22);

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
  if((trapState == 0 || rSwitchCurrentPosition == 7 || rSwitchCurrentPosition == 8) || (trapState == 2 && beeping == true && rSwitchCurrentPosition < 7)) {
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
      if(setupMode == false) {
        bargraphOff();

        trapReset();

        if(rSwitchCurrentPosition != 7 && rSwitchCurrentPosition != 8) {
            bargraphOperate(led_colour, led_mix);
        }
      }
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

void bargraphOn() {
  for(int l = 0; l < 12; l++) {
    if(rSwitchCurrentPosition == 5 || rSwitchCurrentPosition == 6) {
      bar.setBar(23 - l, LED_GREEN);
    }
    else {
      bar.setBar(23 - l, LED_YELLOW);
    }
    bar.writeDisplay();
  }
}

void checkRemote() {
  // If pressed button on side of trap to trigger it.
  if(musicPlayer.GPIO_digitalRead(activationSwitch) == HIGH && millis() > debounceBuffer) {
    debounceBuffer = millis() + 1000;
    remotePressed = true;
  }
  else { 
    remotePressed = false;
  }
 
  // If pressed down on the pedal.
  if(b_pedalConnected == true) {
    pinMode(pedalLED, INPUT_PULLUP);
    
    digitalWrite(pedalLED, LOW);

    if(digitalRead(pedalLED) != LOW && b_pedalDown == false && millis() > debounceBuffer) {
      remotePressed = true;

      // If the operator keeps the pedal pressed down, we keep it from doing constant trap triggers and instead wait for the pedal to be released before it can trigger the trap again.
      b_pedalDown = true;
    }
    else if(digitalRead(pedalLED) == LOW && b_pedalDown == true) {
      b_pedalDown = false;
      debounceBuffer = millis() + 1000;
    }

    if(b_pedalDown == true) {
      debounceBuffer = millis() + 1000;
    }
    
    pinMode(pedalLED, OUTPUT);

    if(trapState == 2 && beeping == true) {
      // Make the pedal led fade in and out.
      analogWrite(pedalLED, rLED);
    }
    else if(trapState == 2 && beeping == false) {
      digitalWrite(pedalLED, HIGH);
    }
    else {
      digitalWrite(pedalLED, LOW);
    }
  }
}

// Adjust the volume from the right side rotary encoder.
void checkVolumeEncoder() {
  static int8_t c,val;

  if(val = readRotary()) {
    c += val;
  
    // Counter clockwise. Decrease the volume.
    if(prevNextCode == 0x0b) {
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
    }
  
    // Clockwise. Increase the volume.
    if(prevNextCode == 0x07) {         
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
    }

    if(b_volume == true) {
      musicPlayer.setVolume(volumeL, volumeR);
    }
    else {
      musicPlayer.setVolume(volumeMidL, volumeMidR);
    }

     bargraphVolume();
  }
}

void bargraphVolume() {
  int value = 12;

  if(volumeL > 59) {
    value = 1;
  }
  else if(volumeL > 55) {
    value = 2;
  }
  else if(volumeL > 50) {
    value = 3;
  }
  else if(volumeL > 45) {
    value = 4;
  }
  else if(volumeL > 40) {
    value = 5;
  }
  else if(volumeL > 35) {
    value = 6;
  }
  else if(volumeL > 30) {
    value = 7;
  }
  else if(volumeL > 25)  {
    value = 8;
  }
  else if(volumeL > 20) {
    value = 9;
  }
  else if(volumeL > 15) {
    value = 10;
  }
  else if(volumeL > 10) {
    value = 11;
  }
  else if(volumeL < 5) {
    value = 12;
  }
  
  int l_int = 1;

  for(int l = 0; l < 12; l++) {
    if(l < value) {
      bar.setBar(23 - l, LED_RED);
    }
    else {
      bar.setBar(23 - l, LED_OFF);
    }
    
    bar.writeDisplay();
  }

  b_timer.start(b_timer_length);
}

// A vald CW or CCW move returns 1, invalid returns 0.
int8_t readRotary() {
  static int8_t rot_enc_table[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};

  prevNextCode <<= 2;
  
  if(digitalRead(rEncoderB)) { 
    prevNextCode |= 0x02;
  }
  
  if(digitalRead(rEncoderA)) {
    prevNextCode |= 0x01;
  }
  
  prevNextCode &= 0x0f;

   // If valid then store as 16 bit data.
   if(rot_enc_table[prevNextCode]) {
      store <<= 4;
      store |= prevNextCode;

      if ((store&0xff)==0x2b) return -1;
      if ((store&0xff)==0x17) return 1;
   }
   
   return 0;
}

void trapDelay(int bgDelayTime) {
  millisDelay trapDelay;
  boolean bg_delay = true;

  trapDelay.start(bgDelayTime);
  
  while(bg_delay == true) {
    checkVolumeEncoder();
    
    if(trapDelay.justFinished()) {
      bg_delay = false;
    }
  }
}

/*  
 A work around for anything that turns off interrupts such as the neopixels library or music maker. This conflicts with the milliseconds timer, slowing the timing down.
 timer1 is can count up up to 8.388 seconds, after that it will miss a overflow and will read low. 8 seconds is enough for all our timing needs during the capture phases where timing is the most important.
*/
void startTimer1() {
  // Reset Timer 1
  TCCR1A = 0;
  TCCR1B = 0;
  
  // Zero it
  TCNT1 = 0;  
  TIFR1 |= bit (TOV1);  // clear overflow flag
  
  // Start Timer 1
  TCCR1B =  bit (CS10) | bit (CS12);  // prescaler of 1024
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
