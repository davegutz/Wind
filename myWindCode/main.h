//Sometimes useful for debugging
//#pragma SPARK_NO_PREPROCESSOR
//

// Standard
#ifdef ARDUINO
#include <Servo.h>
#include <Arduino.h>     // Used instead of Print.h - breaks Serial
#else                    // Photon
#include "application.h" // Should not be needed if file ino or Arduino
#ifdef PARTICLE
#include <Particle.h>     // Needed for CLI standalone
#endif
SYSTEM_THREAD(ENABLED); // Make sure code always run regardless of network status
#endif
#include "myAnalyzer.h"
#include "myTables.h"
#include "math.h"

// Test features
// in myClaw.h  #define CTYPE 1   // 0=P+I, 1=I, 2=PID
// in myClaw.h  #define KIT   1   // -1=Photon, 0-4 = Arduino
#define TTYPE 4          // 0=STEP, 1=FREQ, 2=VECT, 3=RAMP (ramp is open loop only), 4=RAND
//#define CALIBRATING         // Use this to port converted v4 to the vpot serial signal for calibration
int     verbose = 1;     // [1] Debug, as much as you can tolerate.   For Photon set using "v#"
bool    bare = false;    // [false] The microprocessor is completely disconnected.  Fake inputs and sensors for test purposes.  For Photon set using "b"
bool    test = true;    // [false] The turbine and ESC are disconnected.  Fake inputs and sensors for test purposes.  For Photon set using "t"
double  stepVal = 6;     // [6] Step input, %nf.  Try to make same as freqRespAdder

#if TTYPE==0  // STEP
testType testOnButton = STEP;
#elif TTYPE==1  // FREQ
testType testOnButton = FREQ;
#elif TTYPE==2  // VECT
testType testOnButton = VECT;
#elif TTYPE==3  // RAMP
testType testOnButton = RAMP;
#elif TTYPE==4  // RAND
testType testOnButton = RAND;
#else
#error "TTYPE bad"
#endif

/*
Controlling a servo position using a potentiometer (variable resistor)
by Dave Gutz

Connections for Photon:
  ESC ------------------- Photon
    BLK ------------------- GND
    WHT ------------------- PWM Output (A4)
  ESC----------------- 3-wire DC Servomotor
    Any three to any three
  DPST switch-------------Photon
    GND-------------------D4
  DPST switch
    HI--------------------3.3V
    LO--------------------10K to GND
  Push button-------------Photon
      R---------------------D2
    Push button
      HI--------------------3.3V
      LO--------------------10K to GND
  F2V---------------------Photon
    V5/V10----------------Analog In A2
    GND-------------------GND
  POT---------------------Photon
    VHI ------------------3.3v
    VLO ------------------GND
    WIPE -----------------Analog In A0
  LED to indicate frequency response----Digital Output (D7)
  Hardware Platform:
    Microcontroller:  Particle Photon
    ESC:Turnigy Plush 25A, 2-4S LiPo
    Power Supply:  BINZET AC 100-240V to DC 12V 10A
    Potentiometer:  Mouser 314-1410F-10K-3  10K Linear Pot
    Motor:  Hobby King 50mm Alloy EDF 4800 Kv (3s Version)
    F2V:  Texas Instruments 926-LM2907N/NOPB (14 pin, no Zener)


Connections for Arduino:
  ESC ------------------- Arduino
    BLK ------------------- GND
    WHT ------------------- PWM Output (5)
  ESC----------------- 3-wire DC Servomotor
    Any three to any three
  F2V-----------------Arduino
    V5/V10----------------Analog In A2
    GND-------------------GND
  DPST switch-------------Arduino
    CTR-------------------4
  DPST switch
    HI--------------------3.3V
    LO--------------------10K to GND
  Push button-------------Arduino
    R---------------------2
  Push button
    HI--------------------3.3V
    LO--------------------10K to GND
  POT---------------------Arduino
    VHI ------------------5V
    VLO ------------------GND
    WIPE -----------------Analog In A0
  BUTTON ----------------Ardunio  D2
    see https://www.arduino.cc/en/Tutorial/Button
  LED to indicate frequency response----Digital Output (7)
  JUMPER---------------4 to GND for Closed Loop
  Hardware Platform:
    Microcontroller:  Arduino Uno R3
    ESC:Turnigy Plush 25A, 2-4S LiPo
    Power Supply:  BINZET AC 100-240V to DC 12V 10A
    Potentiometer:  Mouser 314-1410F-10K-3  10K Linear Pot
    Motor:  Hobby King 50mm Alloy EDF 4800 Kv (3s Version)
    F2V:  Texas Instruments 926-LM2907N/NOPB (14 pin, no Zener)

  Reaquirements:
  Prime:
  1.  Inputs:  fan speed, pushbutton, slide switch, potentiometer
  2.  Outputs: ESC command using TTL/PWM servo control library
  3.  Manually sweep ESC command from min to max using pot.
  4.  Limit ESC command for safety using configurable parameters.
  5.  Turn on to minimum ESC input level.  Calibration of the ESC is done elsewhere.
  6.  Switch from open to closed loop and vice versa with small, <5 deg throttle
      bump, in response to switch change.
  7.  Perform frequency response analysis in response to button push.
  8.  Update closed loop algorithms, sense inputs, and produce outputs at rate
      at least as fast as 0.015 second update rate.
  Secondary:
  1.  Repeated Pushbutton toggles frequency response in progress.   When
      restarting, it begins completely fresh.
  2.  For non-Arduino, may also use a software Pushbutton - send string on serial.
  3.  Embed a plant model for dry code checkouts.
  4.  Filter pot noise.

  Tasks TODO:
  1.  Photon frequency response with button

  Revision history:
    31-Aug-2016   DA Gutz   Created
    13-Sep-2016   DA Gutz   Initial analyzing
    30-Sep-2016   DA Gutz   Arduino added
    10-Oct-2016   DA Gutz   First frequency response completion
    30-Oct-2016   DA Gutz   Gain scheduling
    16-Nov-2016   DA Gutz   Retune again again

  Distributed as-is; no warranty is given.
*/

// Test features usually commented
//

// Disable flags if needed.  Usually commented
//#define DISTURB_CONTROL                       // Use disturbance rejection gains in CLAW
//#define USE_FIXED_LL                          // Use the fixed lead lag compensator

// Constants always defined
// #define CONSTANT
#ifdef ARDUINO
#define BUTTON_PIN 2                                           // Button 3-way input momentary 3.3V, steady GND (D2)
#define PWM_PIN 5                                              // PWM output (PD5)
#define POT_PIN A0                                             // Potentiometer input pin on Arduino (PC0)
#define F2V_PIN A2                                             // Fan speed back-emf input pin on Arduino (PC2)
#define CL_PIN 4                                               // Closed loop 3-way switch 5V or GND (D4 to GND)
#define CLOCK_TCK 16UL                                         // Clock tick resolution, micros
#define INSCALE 1023.0                                         // Input full range from OS
const double vpotHalfDB = 0.0;                                 // Half deadband sliding deadband filter, volts
const double POT_MAX = 5.0;                                    // Maximum POT value, vdc
const double F2V_MAX = 5.0;                                    // Maximum F2V value, vdc
const double POT_BIA = 0.36 + vpotHalfDB;                      // Pot adder, vdc
const double POT_SCL = (4.6 - vpotHalfDB - POT_BIA) / POT_MAX; // Pot scalar, vdc
#else                                                          // Photon
#define BUTTON_PIN D2                                          // Button 3-way input momentary 3.3V, steady GND (D2)
#define PWM_PIN A4                                             // PWM output (A4)
#define POT_PIN A0                                             // Potentiometer input pin on Photon (A0)
#define F2V_PIN A2                                             // Fan speed back-emf input pin on Photon (A2)
#define CL_PIN D0                                              // Closed loop 3-way switch 3.3V or GND  (D0)
#define CLOCK_TCK 8UL                                          // Clock tick resolution, micros
#define INSCALE 4096.0                                         // Input full range from OS
const double vpotHalfDB = 0.0;                    // Half deadband sliding deadband filter, volts
const double POT_MAX = 3.3;                       // Maximum POT value, vdc
const double F2V_MAX = 3.45;                      // Maximum F2V value, vdc
const double POT_BIA = 0.0;                       // Pot adder, vdc
const double POT_SCL = (3.3 - POT_BIA) / POT_MAX; // Pot scalar, vdc
#endif
//********constants for all*******************
#ifdef CALIBRATING
#define PUBLISH_DELAY 150000UL // Time between cloud updates (), micros
#else
#define PUBLISH_DELAY 15000UL // Time between cloud updates (), micros
#endif
#define CONTROL_DELAY 15000UL // Control law wait (), micros
#define RAND_DELAY 1500000UL // Control law wait (), micros
#define FR_DELAY 4000000UL    // Time to start FR, micros
const double F2V_MIN = 0.0;   // Minimum F2V value, vdc
const double POT_MIN = 0;     // Minimum POT value, vdc
const double DENS_SI = 1.225; // Air density, kg/m^3

// Test
bool freqResp = false;             // Perform frequency response test status
//bool vectoring = false;            // Perform vector test status
const int nsigFn = 4;              // Length of fn
const int ntfFn = 2;               // Number of transfer  functions to calculate <= length(ix)
double fn[4] = {0, 0, 0, 0};       // Functions to analyze
const int ix[2] = {0, 0};          // Indeces of fn to excitations
const int iy[2] = {1, 2};          // Indeces of fn to responses
const double freqRespScalar = 1e8; // Use 40 for +/-3 deg, 20 for +/-6 deg, 13 for +/-10 at 50% Nf
const double freqRespAdder = 6;    // +/- deg

//
// Dependent includes
#include "myCLAW.h"
#include "myFilters.h"

// Global variables
double throttle = -5; // Servo value, 0-179 degrees
char buffer[256];
LagTustin *throttleFilter; // Tustin lag noise filter
FRAnalyzer *analyzer;      // Frequency response analyzer
Servo myservo;             // create servo object to control dc motor
ControlLaw *CLAW;          // Control Law

#ifndef ARDUINO
String inputString = "";        // a string to hold incoming data
boolean stringComplete = false; // whether the string is complete
#endif

#if TTYPE==2 // VECT
// Test vector setup (functions at bottom of this file)
bool Vcomplete(void);
double Vcalculate(double);
void Vcomplete(bool);
const double Vtv_[] =  {0,  8,  16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 96, 104,  112,  120,  128,  136, 144, 152, 156}; // Time, s
const double Vvv_[] =  {10, 20, 25, 30, 42, 48, 56, 62, 77, 90, 96, 90, 77, 62,   56,   48,   42,   30,  25,  20,  10};  // Excitation
const unsigned int Vnv_ = sizeof(Vtv_)/sizeof(double);  // Length of vector
double Voutput_ = 0;        // Excitation value
double Vtime_ = 0;          // Time into vector, s
double VtnowStart_ = 0;     // now time of vector start reference, s
bool Vcomplete_ = false;    // Status of vector, T=underway
unsigned int Viv_ = 0;      // Index of present time in vector
#endif


#if TTYPE==3 // RAMP
// Ramp vector setup (functions at bottom of this file)
bool Rcomplete(void);
double Rcalculate(double);
void Rcomplete(bool);
const double Rtv_[] =  {0,  8,  68, 78, 138, 148}; // Time, s
const double Rvv_[] =  {10, 10, 96, 96, 10,  10};  // Excitation
const unsigned int Rnv_ = sizeof(Rtv_)/sizeof(double);  // Length of vector
double Routput_ = 0;        // Excitation value
double Rtime_ = 0;          // Time into vector, s
double RtnowStart_ = 0;     // now time of vector start reference, s
bool Rcomplete_ = false;    // Status of vector, T=underway
unsigned int Riv_ = 0;      // Index of present time in vector
#endif

#if TTYPE==4 // RAND
// Test vector setup (functions at bottom of this file)
bool RandComplete(void);
double RandCalculate(double);
void RandComplete(bool);
double Voutput_ = 0;        // Excitation value
double Vtime_ = 0;          // Time into vector, s
double VtnowStart_ = 0;     // now time of vector start reference, s
bool Vcomplete_ = false;    // Status of vector, T=underway
#endif



void setup()
{
#ifndef ARDUINO
  WiFi.disconnect();
#endif
  pinMode(BUTTON_PIN, INPUT);
  Serial.begin(230400);
  //Serial.begin(250000);
  myservo.attach(PWM_PIN, 1000, 2000); // attaches the servo.  Only supported on pins that have PWM
  pinMode(POT_PIN, INPUT);
  pinMode(F2V_PIN, INPUT);
  pinMode(CL_PIN, INPUT);

  // Lag filter
  double T = float(CONTROL_DELAY) / 1000000.0;
  throttleFilter = new LagTustin(T, tau, -0.1, 0.1);

#if TTYPE==1
  // Frequency Response
  //                        wmin    wmax  dw    minCy numCySc  iniCy  wSlow
  analyzer = new FRAnalyzer(-0.8,   1.4,  0.1,  2,    1.0,     6,     1 / tauG,
                            double(CONTROL_DELAY / 1e6), ix, iy, nsigFn, ntfFn, "t,ref,exc,thr,mod,nf,T"); // 15 ms any
#endif

  myservo.write(throttle);

// Serial headers used by plotting programs
// Header for analyzer.  TODO:  should be done in analyzer.cpp so done when needed
#ifndef ARDUINO
  delay(5000); // Allow time to start serial monitor for Photon.  Ardino auto-starts
#endif
  /*
    analyzer->publish();
    Serial.println("");
    */
  // Header for time data
  if (verbose > 0)
  {
    //******************************************************************************************************************************
#ifdef CALIBRATING
    sprintf(buffer, "\ntime,mode,vf2v,  pcntref,pcntSense,pcntSenseM,  err,state,thr, modPcng,T\n");
#else
    sprintf(buffer, "\ntime,mode,vpot,  pcntref,pcntSense,pcntSenseM,  err,state,thr, modPcng,T\n");
#endif
    Serial.print(buffer);
  }

#ifndef ARDUINO
  // Serial Event to allow switches to be passed in by user Serial transmit
  inputString.reserve(200); // Reserve 200 bytes for inputString Serial Event
#endif

  // Instatiate gain scheduling tables
  CLAW = new ControlLaw(T, DENS_SI);

#ifdef ARDUINO
  delay(100);
#else
  delay(1000);
  WiFi.off();
  delay(1000);
#endif
}

void loop()
{
  static bool vectoring = false;            // Perform vector test status
  int buttonState = 0;                    // Pushbutton
  static bool closingLoop = false;        // Persisted closing loop by pin cmd, T/F
  static bool closingLoopLast = false;    // Last closing loop by pin cmd, T/F
  static bool closingLoopPast = false;   // Past closing loop by pin cmd, T/F
  static bool stepping = false;           // Step by Photon send String
  bool control;                           // Control sequence, T/F
  bool randomizer;                        // Random sequence, T/F
  bool publish;                           // Publish, T/F
  static bool analyzing;                         // Begin analyzing, T/F
  unsigned long now = micros();           // Keep track of time
  static unsigned long start = 0UL;       // Time to start looping, micros
  double elapsedTime;                     // elapsed time, micros
  static double updateTime = 0.0;         // Control law update time, sec
  static unsigned long lastControl = 0UL; // Last control law time, micros
  static unsigned long lastPublish = 0UL; // Last publish time, micros
  static unsigned long lastRand = 0UL;    // Last publish time, micros
#ifdef ARDUINO
  static unsigned long lastButton = 0UL;  // Last button push time, micros
  static unsigned long lastFR = 0UL;      // Last analyzing, micros
#endif
  static int mode = 0;                    // Mode of operation First digit: closingLoop, Second digit: testOnButton, Third digit:  analyzing
  static int RESET = 1;                   // Dynamic reset
  const double RESEThold = 5;             // RESET hold, s
  static double exciter = 0;              // Frequency response excitation, fraction
                                          ////////////////////////////////////////////////////////////////////////////////////
  static double vf2v = 0;                 // Converted sensed back emf LM2907 circuit measure, volts
  static double vpot_filt = 0;            // Pot value, volts
  static double vpotDead = 0;             // Sliding deadband value, volts
  static double vpot = 0;                 // Pot value, volts
  static int f2vValue = INSCALE / 4;      // Dial raw value
  static int potValue = INSCALE / 3;      // Dial raw value

  // Executive
  if (start == 0UL) start = now;
  elapsedTime = double(now - start) * 1e-6;
  publish = ((now - lastPublish) >= PUBLISH_DELAY - CLOCK_TCK / 2);
  if (publish)
  {
    lastPublish = now;
  }
  unsigned long deltaTick = now - lastControl;
  control = (deltaTick >= CONTROL_DELAY - CLOCK_TCK / 2);
  if (control)
  {
    updateTime = float(deltaTick) / 1000000.0;
    lastControl = now;
    unsigned long deltaRand = now - lastRand;
    randomizer = (deltaRand >= RAND_DELAY - CLOCK_TCK / 2);
    if (randomizer)
    {
      lastRand = now;
    }
  }
  if (bare)
  {
#ifdef ARDUINO
    closingLoop = true;
#endif
  }
  else
  {
    if ( control ) // Debounce three updates
    {
      bool closingLoopSwitch = (digitalRead(CL_PIN) == HIGH);
      if (!closingLoop)
        closingLoop = closingLoopLast && closingLoopPast && closingLoopSwitch;
      else
        closingLoop = !(!closingLoopLast && !closingLoopPast && !closingLoopSwitch);
      closingLoopPast = closingLoopLast;
      closingLoopLast = closingLoopSwitch;
    }
  }
  buttonState = digitalRead(BUTTON_PIN);
#ifdef ARDUINO
  if (buttonState == HIGH && (now - lastButton > 200000UL))
  {
    lastButton = now;
    switch ( testOnButton )
    {
      case FREQ:
      {
#if TTYPE==1 // FREQ
        analyzer->complete(freqResp); // reset if doing freqResp
#endif      
        freqResp = !freqResp;
        break;
      }
      case STEP:
      {
        stepping  = true;
        stepVal  = -stepVal;
        break;
      }
      case VECT:
      {
#if TTYPE==2 // VECT
        Vcomplete(vectoring); // reset if doing vector
#endif
        vectoring = !vectoring;
        break;
      }
      case RAMP:
      {
#if TTYPE==3 // RAMP
        Rcomplete(vectoring); // reset if doing vector
#endif
        vectoring = !vectoring;
        break;
      }
      case RAND:
      {
#if TTYPE==4 // RAND
        RandComplete(vectoring); // reset if doing vector
#endif
        vectoring = !vectoring;
  sprintf(buffer, "%s\n", String(vectoring).c_str()); Serial.print(buffer);
        break;
      }
    }
  }
#endif
#if TTYPE==1 // FREQ
  if ( freqResp)
    analyzing = ( ((now - lastFR) >= FR_DELAY && !analyzer->complete()) );
#elif TTYPE==2 // VECT
  if ( vectoring )
    analyzing = !Vcomplete();
#elif TTYPE==3 // RAMP
  if ( vectoring )
    analyzing = !Rcomplete();
#elif TTYPE==4 // RAND
  if ( randomizer && vectoring  )
    analyzing = !RandComplete();
#endif
  else
    analyzing = false;
  mode = closingLoop*1000 + test*100 + testOnButton*10 + analyzing;


#ifndef ARDUINO
  // Serial event  (terminate Send String data with 0A using CoolTerm)
  if (stringComplete)
  {
    String doFR = "f\n";
    if (inputString == doFR)
    {
#if TTYPE==1  // FREQ
      analyzer->complete(freqResp); // reset if doing freqResp
#endif
      freqResp = !freqResp;
    }
    String doV = "V\n";
    if (inputString == doV)
    {
#if TTYPE==2  // VECT
      Vcomplete(vectoring); // reset if doing vector
#endif
      vectoring = !vectoring;
    }
    String doR = "R\n";
    if (inputString == doR)
    {
#if TTYPE==3  // RAMP
      Rcomplete(vectoring); // reset if doing vector
#endif
      vectoring = !vectoring;
    }
    String doZ = "Z\n";
    if (inputString == doZ)
    {
#if TTYPE==4  // RAND
      RandComplete(vectoring); // reset if doing vector
#endif
      vectoring = !vectoring;
    }
    String dobare = "b\n";
    if (inputString == dobare)
    {
      bare = !bare;
    }
    String dotest = "t\n";
    if (inputString == dotest)
    {
      test = !test;
    }
    String doCL = "c\n";
    if (inputString == doCL)
    {
      closingLoop = !closingLoop;
    }
    String doStep = "s\n";
    if (inputString == doStep)
    {
      stepping = true;
      stepVal = -stepVal;
    }
    if (inputString.charAt(0) == 'v')
    {
      int vcheck = atoi(inputString.substring(1));
      if (vcheck>=0 && vcheck<10) verbose = vcheck;
    }
    if (inputString.charAt(0) == 'P')
    {
      double potThrottleX = atof(inputString.substring(1));
      if (potThrottleX>=THTL_MIN && potThrottleX<=THTL_MAX)  // ignore otherwise
      {
          double vpotX = potThrottleX * POT_MAX / THTL_MAX;
          potValue  = (vpotX*POT_SCL + POT_BIA)/POT_MAX*INSCALE;
      } 
    }
    inputString = "";
    stringComplete = false;
  }
#endif



  // Interrogate inputs
  if (control)
  {
    if (!bare)
    {
      potValue = analogRead(POT_PIN);
      f2vValue = analogRead(F2V_PIN);
    }
    vf2v = double(f2vValue) / INSCALE * F2V_MAX;
    vpot = fmin(fmax((double(potValue) / INSCALE * POT_MAX - POT_BIA) / POT_SCL, POT_MIN), POT_MAX);
  }

  // Control law
  if (control)
  {
    vpotDead = fmax(fmin(vpotDead, vpot + vpotHalfDB), vpot - vpotHalfDB);
    if (!freqResp)
      vpot_filt = throttleFilter->calculate(vpotDead, RESET); // Freeze pot for FR
    double potThrottle = vpot_filt * THTL_MAX / POT_MAX;      // deg
    double dNdT = P_LTALL_NG[1] / fmax(potThrottle, 1) / RPM_P;  // Rate normalizer, %Ng/deg
    potThrottle += stepping * stepVal / dNdT;
    throttle = CLAW->calculate(RESET, updateTime, closingLoop, analyzing, freqResp, vectoring, exciter, freqRespScalar, freqRespAdder, potThrottle, vf2v);
    if (elapsedTime > RESEThold)
      RESET = 0;
  }

  // Commands to Hardware
  if (control && !test && !bare)
  {
    myservo.write(throttle); // sets the servo position according to the scaled value
  }

  // Calculate frequency response
  if (control)
  {
    fn[0] = throttle;
    fn[1] = CLAW->modelTS();
    fn[2] = CLAW->pcnt();
    fn[3] = CLAW->pcntRef();
    if (analyzing)
    {
#if TTYPE==1  // FREQ
      if ( freqResp ) exciter = analyzer->calculate(fn, nsigFn); // use previous exciter for everything
#elif TTYPE==2 // VECT
      if ( vectoring ) exciter = Vcalculate(elapsedTime);
#elif TTYPE==3 // RAMP
      if ( vectoring ) exciter = Rcalculate(elapsedTime);
#elif TTYPE==4 // RAND
      if ( randomizer && vectoring ) exciter = RandCalculate(elapsedTime);
#endif
    }
  }

  // Publish results to serial bus
  if (publish)
  {
    if (freqResp)
    {
      if (verbose > 1 || (testOnButton==STEP  && verbose>0) )
      {
        sprintf(buffer, "%s,%s,%s,%s,%s,%s,%s,",
                String(elapsedTime, 6).c_str(), String(CLAW->pcntRef()).c_str(),
                String(exciter).c_str(), String(throttle).c_str(),
                String(CLAW->modelTS()).c_str(), String(CLAW->pcnt()).c_str(),
                String(updateTime, 6).c_str());
        Serial.print(buffer);
        if (!analyzer->complete())
        {
#if TTYPE==1 // FREQ
          analyzer->publish();
#endif
        }
        Serial.println("");
      }
    } // freqResp
    else
    {
    sprintf(buffer, "\ntime,mode,vpot,  pcntref,pcntSense,pcntSenseM,  err,state,thr, modPcng,T\n");
      if (verbose > 0)
      {
        sprintf(buffer, "%s,", String(elapsedTime, 6).c_str()); Serial.print(buffer);
        sprintf(buffer, "%s, ", String(mode).c_str()); Serial.print(buffer);
#ifdef CALIBRATING
        sprintf(buffer, "%s,  ", String(vf2v, 3).c_str()); Serial.print(buffer);
#else
        sprintf(buffer, "%s,  ", String(vpot, 3).c_str()); Serial.print(buffer);
#endif
        sprintf(buffer, "%s,", String(CLAW->pcntRef()).c_str()); Serial.print(buffer);
        sprintf(buffer, "%s,", String(CLAW->pcnt()).c_str()); Serial.print(buffer);
        sprintf(buffer, "%s,  ", String(CLAW->modelTS()).c_str()); Serial.print(buffer);
        sprintf(buffer, "%s,", String(CLAW->e()).c_str()); Serial.print(buffer);
        sprintf(buffer, "%s,", String(CLAW->intState()).c_str()); Serial.print(buffer);
        sprintf(buffer, "%s,  ", String(throttle, 0).c_str()); Serial.print(buffer);
        sprintf(buffer, "%s,", String(CLAW->modelG()).c_str()); Serial.print(buffer);
        sprintf(buffer, "%s,\n", String(updateTime, 6).c_str()); Serial.print(buffer);
      }
    }
  } // publish
#if TTYPE==1 // FREQ
  if (analyzer->complete()) freqResp = false;
#elif TTYPE==2 // VECT
  if (Vcomplete()) vectoring = false;
#elif TTYPE==3 // RAMP
  if (Rcomplete()) vectoring = false;
#elif TTYPE==4 // RAND
  if (randomizer && RandComplete()) vectoring = false;
#endif
}



#ifndef ARDUINO
/*
  Special handler that uses built-in callback.
  SerialEvent occurs whenever a new data comes in the
  hardware serial RX.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
  response.  Multiple bytes of data may be available.
 */
void serialEvent()
{
  while (Serial.available())
  {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n')
    {
      stringComplete = true;
    }
  }
}
#endif


// Vector calculator
#if TTYPE==2  // VECT
double Vcalculate(const double tnow)
{
  if ( VtnowStart_ == 0 )
  {
    VtnowStart_ = tnow;  // First call sets time
    Vcomplete_ = false;
    Viv_ = 0;
  }
  // Find location in vector
  Vtime_ = tnow-VtnowStart_;
  while ( Vtv_[Viv_]<Vtime_ && Viv_<Vnv_ ) Viv_++;
  // Output
  if ( Viv_ == Vnv_ ) Vcomplete_ = true;
  unsigned int iv = Viv_;
  if ( iv==0 ) iv = 1;
  Voutput_ = Vvv_[iv-1];
  /*
          sprintf_P(buffer, PSTR("time=%s"), String(time_).c_str());        Serial.print(buffer);
          sprintf_P(buffer, PSTR(",iv=%s"), String(iv_).c_str());        Serial.print(buffer);
          sprintf_P(buffer, PSTR(",tv[iv]=%s"), String(tv_[iv_]).c_str());        Serial.print(buffer);
          sprintf_P(buffer, PSTR(",output=%s\n"), String(Voutput_).c_str());        Serial.print(buffer);
*/
  return ( Voutput_ );
};

bool Vcomplete(void) { return (Vcomplete_); };

// Restart vector
void Vcomplete(const bool set)
{
  Viv_ = 0;
  Vcomplete_ = false;
  Vtime_ = 0;
  VtnowStart_ = 0;
};
#endif



// Ramp calculator
#if TTYPE==3  // RAMP
double Rcalculate(const double tnow)
{
  if ( RtnowStart_ == 0 )
  {
    RtnowStart_ = tnow;  // First call sets time
    Rcomplete_ = false;
    Riv_ = 0;
  }
  // Find location in vector
  Rtime_ = tnow-RtnowStart_;
  while ( Rtv_[Riv_]<Rtime_ && Riv_<Rnv_ ) Riv_++;   // iv is location past now
  // Output
  if ( Riv_ == Rnv_ ) Rcomplete_ = true;
  unsigned int ir = Riv_;
  if ( ir==0 ) ir = 1;
  Routput_ = (Rtime_-Rtv_[ir-1]) / (Rtv_[ir]-Rtv_[ir-1]) * (Rvv_[ir]-Rvv_[ir-1])  +  Rvv_[ir-1];
  /*
          sprintf_P(buffer, PSTR("time=%s"), String(time_).c_str());        Serial.print(buffer);
          sprintf_P(buffer, PSTR(",ir=%s"), String(ir_).c_str());        Serial.print(buffer);
          sprintf_P(buffer, PSTR(",tr[ir]=%s"), String(tr_[ir_]).c_str());        Serial.print(buffer);
          sprintf_P(buffer, PSTR(",output=%s\n"), String(Routput_).c_str());        Serial.print(buffer);
*/
  return ( Routput_ );
};

bool Rcomplete(void) { return (Rcomplete_); };

// Restart ramp
void Rcomplete(const bool set)
{
  Riv_ = 0;
  Rcomplete_ = false;
  Rtime_ = 0;
  RtnowStart_ = 0;
};
#endif

// Random calculator
#if TTYPE==4  // RAND
double RandCalculate(const double tnow)
{
  if ( VtnowStart_ == 0 )
  {
    VtnowStart_ = tnow;  // First call sets time
    Vcomplete_ = false;
  }
  // Find location in vector
  Vtime_ = tnow-VtnowStart_;
  Voutput_ = random(80);
  /*
          sprintf_P(buffer, PSTR("time=%s"), String(time_).c_str());        Serial.print(buffer);
          sprintf_P(buffer, PSTR(",iv=%s"), String(iv_).c_str());        Serial.print(buffer);
          sprintf_P(buffer, PSTR(",tv[iv]=%s"), String(tv_[iv_]).c_str());        Serial.print(buffer);
          sprintf_P(buffer, PSTR(",output=%s\n"), String(Voutput_).c_str());        Serial.print(buffer);
*/
  return ( Voutput_ );
};

bool RandComplete(void) { return (Vcomplete_); };

// Restart vector
void RandComplete(const bool set)
{
  Vcomplete_ = false;
  Vtime_ = 0;
  VtnowStart_ = 0;
};
#endif

