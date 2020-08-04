// Protogasm Code, forked from Nogasm Code Rev. 3


// This file includes modifications to make the nogasm into a power spectrum
// analyser for your butt (an ANALyser).
// Instead of using pressure level to cut off the vibe, we use the power spectra
// in the 4-8hz range, otherwise the functionality is pretty much the same

// NOTE that instead of using a motor pin, I send a command over UART
// to another arduino. This is so that it can work together with a previous project
// of mine. If you want to use this on a regular protogasm, you'll have to
// modify the motor control back to the previous version. This also means
// that the beeps have been excised.

// TODO(dmo): The display for the edging mode is still very jumpy

/* Drives a vibrator and uses changes in pressure of an inflatable buttplug
 * to estimate a user's closeness to orgasm, and turn off the vibrator
 * before that point.
 * A state machine updating at 60Hz creates different modes and option menus
 * that can be identified by the color of the LEDs, especially the RGB LED
 * in the central button/encoder knob.
 * 
 * [Red]    Manual Vibrator Control
 * [Blue]   Automatic vibrator edging, knob adjusts orgasm detection sensitivity
 * [Green]  Setting menu for maximum vibrator speed in automatic mode
 * [White]  Debubbing menu to show data from the pressure sensor ADC
 * [Off]    While still plugged in, holding the button down for >3 seconds turns
 *          the whole device off, until the button is pressed again.
 * 
 * Settings like edging sensitivity, or maximum motor speed are stored in EEPROM,
 * so they are saved through power-cycling.
 * 
 * In the automatic edging mode, the vibrator speed will linearly ramp up to full
 * speed (set in the green menu) over 30 seconds. If a near-orgasm is detected,
 * the vibrator abruptly turns off for 15 seconds, then begins ramping up again.
 * 
 * The motor will beep during power on/off, and if the plug pressure rises above
 * the maximum the board can read - this condition could lead to a missed orgasm 
 * if unchecked. The analog gain for the sensor is adjustable via a trimpot to
 * accomidate different types of plugs that have higher/lower resting pressures.
 * 
 * Motor speed, current pressure, and average pressure are reported via USB serial
 * at 115200 baud. Timestamps can also be enabled, from the main loop.
 * 
 * There is some framework for more features like an adjustable "cool off" time 
 * other than the default 15 seconds, and options for LED brightness and enabling/
 * disabling beeps.
 * 
 * Note - Do not set all the LEDs to white at full brightness at once
 * (RGB 255,255,255) It may overheat the voltage regulator and cause the board 
 * to reset.
 */
//=======Libraries===============================
#include <Encoder.h>
#include <EEPROM.h>
#include "FastLED.h"

#define FHT_N 256
#define LIN_OUT 1
#include "FHT.h"

//=======Hardware Setup===============================
//LEDs
#define NUM_LEDS 24
#define LED_PIN 10
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define BRIGHTNESS 50 //Subject to change, limits current that the LEDs draw


#define RECORDMODE 0

//Encoder
#define ENC_SW   5 //Pushbutton on the encoder
Encoder myEnc(2, 3); //Quadrature inputs
#define ENC_SW_UP   HIGH
#define ENC_SW_DOWN LOW

//Pressure Sensor Analog In
#define BUTTPIN A0
// Sampling 4x and not dividing keeps the samples from the Arduino Uno's 10 bit
// ADC in a similar range to the Teensy LC's 12-bit ADC.  This helps ensure the
// feedback algorithm will behave similar to the original.
#define ADC_MAX 1023

//=======Software/Timing options=====================
#define FREQUENCY 60 //Update frequency in Hz
#define LONG_PRESS_MS 600 //ms requirements for a long press, to move to option menus
#define V_LONG_PRESS_MS 2500 //ms for a very long press, which turns the device off

//Update/render period
#define period (1000/FREQUENCY)
#define longBtnCount (LONG_PRESS_MS / period)

int sensitivity = 0; //orgasm detection sensitivity, persists through different states

//=======State Machine Modes=========================
#define MANUAL      1
#define AUTO        2
#define OPT_SPEED   3
#define OPT_RAMPSPD 4
#define OPT_BEEP    5
#define OPT_PRES    6


//Button states - no press, short press, long press
#define BTN_NONE   0
#define BTN_SHORT  1
#define BTN_LONG   2
#define BTN_V_LONG 3


uint8_t state = MANUAL;
//=======Global Settings=============================
#define MOT_MAX 255 // Motor PWM maximum
#define MOT_MIN 20  // Motor PWM minimum.  It needs a little more than this to start.

CRGB leds[NUM_LEDS];

int pressure = 0;
int specPower = 0;
//int bri =100; //Brightness setting
int rampTimeS = 30; //Ramp-up time, in seconds
#define DEFAULT_SPLIMIT 300
#define SPECTRAL_NOISE_FLOOR 80
int spLimit = DEFAULT_SPLIMIT; //Limit in change of pressure before the vibrator turns off
int maxSpeed = 255; //maximum speed the motor will ramp up to in automatic mode
float motSpeed = 0; //Motor speed, 0-255 (float to maintain smooth ramping to low speeds)

int pressureHist[FHT_N];
int pressure_index;

//=======EEPROM Addresses============================
//128b available on teensy LC
#define BEEP_ADDR         1
#define MAX_SPEED_ADDR    2
#define SENSITIVITY_ADDR  3
//#define RAMPSPEED_ADDR    4 //For now, ramp speed adjustments aren't implemented

void sendSpeed(uint8_t speed) {
	#if RECORDMODE
		return;
	#endif
	if (Serial.availableForWrite() < 8) {
		return;
	}
	uint8_t buf[6];
	buf[0] = 0xAA;
	buf[1] = 0xAA;
	buf[2] = 0x02;
	buf[3] = 0x08;
	buf[4] = speed;
	buf[5] = (uint8_t)255 - (buf[3] + buf[4]);
	Serial.write(buf, 6);
}

void setup() {
  pinMode(ENC_SW,   INPUT); //Pin to read when encoder is pressed
  digitalWrite(ENC_SW, HIGH); // Encoder switch pullup

  analogReference(EXTERNAL);
  
  pinMode(BUTTPIN,INPUT); //default is 10 bit resolution (1024), 0-3.3

  delay(3000); // 3 second delay for recovery

  pressure_index = 0;

  // fill the initial history with one value to avoid obvious artifacts
  // during startup
  pressure = analogRead(BUTTPIN)<<5;

  for (int i = 0; i < FHT_N; i++) {
    pressureHist[i] = pressure;
  }

  #if RECORDMODE
  Serial.begin(115200);
  #else
  Serial.begin(57600);
  #endif

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  // limit power draw to .6A at 5v... Didn't seem to work in my FastLED version though
  //FastLED.setMaxPowerInVoltsAndMilliamps(5,DEFAULT_PLIMIT);
  FastLED.setBrightness(BRIGHTNESS);

  //Recall saved settings from memory
  sensitivity = EEPROM.read(SENSITIVITY_ADDR);
  maxSpeed = min(EEPROM.read(MAX_SPEED_ADDR),MOT_MAX); //Obey the MOT_MAX the first power  cycle after chaning it.
}

//=======LED Drawing Functions=================

//Draw a "cursor", one pixel representing either a pressure or encoder position value
//C1,C2,C3 are colors for each of 3 revolutions over the 13 LEDs (39 values)
void draw_cursor_3(int pos,CRGB C1, CRGB C2, CRGB C3){
  pos = constrain(pos,0,NUM_LEDS*3-1);
  int colorNum = pos/NUM_LEDS; //revolution number
  int cursorPos = pos % NUM_LEDS; //place on circle, from 0-12
  switch(colorNum){
    case 0:
      leds[cursorPos] = C1;
      break;
    case 1:
      leds[cursorPos] = C2;
      break;
    case 2:
      leds[cursorPos] = C3;
      break;
  }
}

//Draw a "cursor", one pixel representing either a pressure or encoder position value
void draw_cursor(int pos,CRGB C1){
  pos = constrain(pos,0,NUM_LEDS-1);
  leds[pos] = C1;
}

//Draw 3 revolutions of bars around the LEDs. From 0-39, 3 colors
void draw_bars_3(int pos,CRGB C1, CRGB C2, CRGB C3){
  pos = constrain(pos,0,NUM_LEDS*3-1);
  int colorNum = pos/NUM_LEDS; //revolution number
  int barPos = pos % NUM_LEDS; //place on circle, from 0-12
  switch(colorNum){
    case 0:
      fill_gradient_RGB(leds,0,C1,barPos,C1);
      //leds[barPos] = C1;
      break;
    case 1:
      fill_gradient_RGB(leds,0,C1,barPos,C2);
      break;
    case 2:
      fill_gradient_RGB(leds,0,C2,barPos,C3);
      break;
  }
}

//Provide a limited encoder reading corresponting to tacticle clicks on the knob.
//Each click passes through 4 encoder pulses. This reduces it to 1 pulse per click
int encLimitRead(int minVal, int maxVal){
  if(myEnc.read()>maxVal*4)myEnc.write(maxVal*4);
  else if(myEnc.read()<minVal*4) myEnc.write(minVal*4);
  return constrain(myEnc.read()/4,minVal,maxVal);
}

//=======Program Modes/States==================

// Manual vibrator control mode (red), still shows orgasm closeness in background
void run_manual() {
  //In manual mode, only allow for 13 cursor positions, for adjusting motor speed.
  int knob = encLimitRead(0,NUM_LEDS-1);
  motSpeed = map(knob, 0, NUM_LEDS-1, 0., (float)MOT_MAX);
  sendSpeed(motSpeed);

  //gyrGraphDraw(avgPressure, 0, 4 * 3 * NUM_LEDS);
  int presDraw = map(specPower,SPECTRAL_NOISE_FLOOR,spLimit,0,NUM_LEDS*3);
  draw_bars_3(presDraw, CRGB::Green,CRGB::Yellow,CRGB::Red);
  draw_cursor(knob, CRGB::Red);
}

// Automatic edging mode, knob adjust sensitivity.
void run_auto() {
  static float motIncrement = 0.0;
  motIncrement = ((float)maxSpeed / ((float)FREQUENCY * (float)rampTimeS));

  int knob = encLimitRead(0,(3*NUM_LEDS)-1);
  sensitivity = knob*4; //Save the setting if we leave and return to this state
  //Reverse "Knob" to map it onto a pressure limit, so that it effectively adjusts sensitivity
  spLimit = map(knob, 0, 3 * (NUM_LEDS - 1), 500, 1); //set the limit of delta pressure before the vibrator turns off
  //When someone clenches harder than the pressure limit
  if (specPower > spLimit) {
    motSpeed = -.5*(float)rampTimeS*((float)FREQUENCY*motIncrement);//Stay off for a while (half the ramp up time)
  }
  else if (motSpeed < (float)maxSpeed) {
    motSpeed += motIncrement;
  }
  if (motSpeed > MOT_MIN) {
    sendSpeed((uint8_t) motSpeed);
  } else {
    sendSpeed(0);
  }

  int presDraw = map(specPower,SPECTRAL_NOISE_FLOOR,spLimit,0,NUM_LEDS*3);
  draw_bars_3(presDraw, CRGB::Green,CRGB::Yellow,CRGB::Red);
  draw_cursor_3(knob, CRGB(50,50,200),CRGB::Blue,CRGB::Purple);

}

//Setting menu for adjusting the maximum vibrator speed automatic mode will ramp up to
void run_opt_speed() {
  // Serial.println("speed settings");
  int knob = encLimitRead(0,NUM_LEDS-1);
  motSpeed = map(knob, 0, NUM_LEDS-1, 0., (float)MOT_MAX);
  sendSpeed(motSpeed);
  maxSpeed = motSpeed; //Set the maximum ramp-up speed in automatic mode
  //Little animation to show ramping up on the LEDs
  static int visRamp = 0;
  if(visRamp <= FREQUENCY*NUM_LEDS-1) visRamp += 16;
  else visRamp = 0;
  draw_bars_3(map(visRamp,0,(NUM_LEDS-1)*FREQUENCY,0,knob),CRGB::Green,CRGB::Green,CRGB::Green);
}

//Not yet added, but adjusts how quickly the vibrator turns back on after being triggered off
void run_opt_rampspd() {
  //Serial.println("rampSpeed");
}

//Also not completed, option for enabling/disabling beeps
void run_opt_beep() {
  //Serial.println("Brightness Settings");
}

//Simply display the pressure analog voltage. Useful for debugging sensitivity issues.
void run_opt_pres() {
  int p = map(analogRead(BUTTPIN),0,ADC_MAX,0,NUM_LEDS-1);
  draw_cursor(p,CRGB::White);
}

//Poll the knob click button, and check for long/very long presses as well
uint8_t check_button(){
  static bool lastBtn = ENC_SW_DOWN;
  static unsigned long keyDownTime = 0;
  uint8_t btnState = BTN_NONE;
  bool thisBtn = digitalRead(ENC_SW);

  //Detect single presses, no repeating, on keyup
  if(thisBtn == ENC_SW_DOWN && lastBtn == ENC_SW_UP){
    keyDownTime = millis();
  }
  
  if (thisBtn == ENC_SW_UP && lastBtn == ENC_SW_DOWN) { //there was a keyup
    if((millis()-keyDownTime) >= V_LONG_PRESS_MS){
      btnState = BTN_V_LONG;
    }
    else if((millis()-keyDownTime) >= LONG_PRESS_MS){
      btnState = BTN_LONG;
      }
    else{
      btnState = BTN_SHORT;
      }
    }

  lastBtn = thisBtn;
  return btnState;
}

//run the important/unique parts of each state. Also, set button LED color.
void run_state_machine(uint8_t state){
  switch (state) {
      case MANUAL:
        run_manual();
        break;
      case AUTO:
        run_auto();
        break;
      case OPT_SPEED:
        run_opt_speed();
        break;
      case OPT_RAMPSPD:
        run_opt_rampspd();
        break;
      case OPT_BEEP:
        run_opt_beep();
        break;
      case OPT_PRES:
        run_opt_pres();
        break;
      default:
        run_manual();
        break;
    }
}

//Switch between state machine states, and reset the encoder position as necessary
//Returns the next state to run. Very long presses will turn the system off (sort of)
uint8_t set_state(uint8_t btnState, uint8_t state){
  if(btnState == BTN_NONE){
    return state;
  }
  if(btnState == BTN_V_LONG){
    //Turn the device off until woken up by the button
    //Serial.println("power off");
    fill_gradient_RGB(leds,0,CRGB::Black,NUM_LEDS-1,CRGB::Black);//Turn off LEDS
    FastLED.show();
    sendSpeed(0); //Turn Motor off
    while(!digitalRead(ENC_SW))delay(1);
    return MANUAL ;
  }
  else if(btnState == BTN_SHORT){
    switch(state){
      case MANUAL:
        myEnc.write(sensitivity);//Whenever going into auto mode, keep the last sensitivity
        motSpeed = 0; //Also reset the motor speed to 0
        return AUTO;
      case AUTO:
        myEnc.write(0);//Whenever going into manual mode, set the speed to 0.
        motSpeed = 0;
        EEPROM.update(SENSITIVITY_ADDR, sensitivity);
        return MANUAL;
      case OPT_SPEED:
        myEnc.write(0);
        EEPROM.update(MAX_SPEED_ADDR, maxSpeed);
        //return OPT_RAMPSPD;
        //return OPT_BEEP;
        motSpeed = 0;
        sendSpeed(motSpeed); //Turn the motor off for the white pressure monitoring mode
        return OPT_PRES; //Skip beep and rampspeed settings for now
      case OPT_RAMPSPD: //Not yet implimented
        //motSpeed = 0;
        //myEnc.write(0);
        return OPT_BEEP;
      case OPT_BEEP:
        myEnc.write(0);
        return OPT_PRES;
      case OPT_PRES:
        myEnc.write(map(maxSpeed,0,255,0,4*(NUM_LEDS)));//start at saved value
        return OPT_SPEED;
    }
  }
  else if(btnState == BTN_LONG){
    switch (state) {
          case MANUAL:
            myEnc.write(map(maxSpeed,0,255,0,4*(NUM_LEDS)));//start at saved value
            return OPT_SPEED;
          case AUTO:
            myEnc.write(map(maxSpeed,0,255,0,4*(NUM_LEDS)));//start at saved value
            return OPT_SPEED;
          case OPT_SPEED:
            myEnc.write(0);
            return MANUAL;
          case OPT_RAMPSPD:
            return MANUAL;
          case OPT_BEEP:
            return MANUAL;
          case OPT_PRES:
            myEnc.write(0);
            return MANUAL;
        }
  }
  else return MANUAL;
}

//=======Main Loop=============================
void loop() {
  static uint8_t state = MANUAL;
  static int pressureAccum = 0;
  static uint8_t tick = 0;
  static unsigned long lastsample = 0;
  // sample every other millisecond as a
  // noise reduction strategy
  unsigned long time = millis();
  if (time % 2 == 0 && time != lastsample) {
    lastsample = time;
    pressureAccum += analogRead(BUTTPIN);
    tick++;

    // every 8th tick, run the display
    // this resolves to a 60hz update for the display
    if (tick % 8 == 0) {
      fadeToBlackBy(leds,NUM_LEDS,20); //Create a fading light effect. LED buffer is not otherwise cleared
      uint8_t btnState = check_button();
      state = set_state(btnState,state); //Set the next state based on this state and button presses
      run_state_machine(state);
      FastLED.show(); //Update the physical LEDs to match the buffer in software
    }
    // update the FHT and pressure at 1000ms/64ms hz
    // around 15.625
    // This should get us the most oversample + the largest
    // window for the frequencies we want, between 4 and 8.
    // (we don't actually get 8, but it's close enough to give a decent
    // power spectrum
    if (tick % 32 == 0) {
      pressure = pressureAccum;
      pressureAccum = 0;

      pressureHist[pressure_index++] = pressure;
      pressure_index &= FHT_N - 1;
      for (int i = 0; i < pressure_index; i++) {
        fht_input[(FHT_N - pressure_index) + i] = pressureHist[i];
      }
      for (int i = pressure_index; i < FHT_N; i++) {
        fht_input[i - pressure_index] = pressureHist[i];
      }

      fht_window(); // window the data for better frequency response
      fht_reorder(); // reorder the data before doing the fht
      fht_run(); // process the data in the fht

      // TODO(dmo): try this with a decibel or linear 8-bit scaled
      // result. Right now the difference between noise and signal is
      // low, so the display jumps all over the place
      fht_mag_lin(); // turn it into a linear result

      tick = 0;
      specPower = 0;
      for (int i = 64; i < FHT_N/2; i++) {
        specPower += fht_lin_out[i];
      }

      //Report pressure and motor data over USB for analysis / other uses. timestamps disabled by default
      #if RECORDMODE
      Serial.print(millis());
      Serial.print(",");
      Serial.print(pressure);
      Serial.print(",");
      Serial.print(specPower);
      Serial.print(",");
      Serial.print(spLimit);
      Serial.print(",");
      Serial.print(motSpeed);
      Serial.print("\n");
      #endif
    }
  }
}
