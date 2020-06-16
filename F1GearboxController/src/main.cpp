#include <Arduino.h>
#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>
#include <EEPROM.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>
#include <VehicleSimulation.h>

//Define all the pins
const int stepperLPin = 23;      //Step pin
const int stepperLDirPin = 22;   // Direction Pin
const int stepperLSleepPin = 2;  // Sleep driver pin
const int stepperRPin = 21;      //Step pin
const int stepperRDirPin = 19;   // Direction Pin
const int stepperRSleepPin = 15; // Sleep driver pin
const int potRPin = 35;          //Right side pot pin
const int potLPin = 34;          //Left side pot pin
const int escPin = 12;           //Speed controller pin
const int mainRPMPin = 5;        //Pin for interupt of the mainshaft photo sensor
const int layRPMPin = 18;        //Pin for interupt of the layshaft photo sensor
const int acceleratorPin = 0;    //Pin for input from a 10k pot representing accelerator

// Initialiize the Stepper motors
AccelStepper stepperR(1, stepperRPin, stepperRDirPin);
AccelStepper stepperL(1, stepperLPin, stepperLDirPin);

//
VehicleSimulation vehicle;

//Direction Definitions for Steppers
const int CW = 1;
const int CCW = -1;

//States for RPM Sensors
#define BLOCKED false
#define NOT_BLOCKED true
#define WATCH_SENSOR true
#define IGNORE_SENSOR false
#define LEFT 0
#define RIGHT 1

//PWM Speed Control Variables
const int freq = 20000;
const int resolution = 8;
const int pwmChannel = 0;

//RPM Variables
unsigned int mainRpm = 0;
unsigned int mainRpmFiltered = 0;
unsigned long mainPulseCount = 0;
bool mainPreviousRPMSensorState = NOT_BLOCKED;
bool mainBounceState = IGNORE_SENSOR;
unsigned long mainPreviousMicros = 0;

unsigned int layRpm = 0;
unsigned int layRpmFiltered = 0;
unsigned long layPulseCount = 0;
bool layPreviousRPMSensorState = NOT_BLOCKED;
bool layBounceState = IGNORE_SENSOR;
unsigned long layPreviousMicros = 0;

//Motor Speed Control
int speedControlPWM = 0;
float previousError = 0;
float integral = 0;
unsigned long previousMotorSpeedControlMicros = 0;
const float Kp = 5.5f;
const float Ki = 3.0f; //0.15
const float Kd =1.5f; //.15
unsigned long previousMicros = 0;
bool engineRunning = true;

const int rpmDebounceInterval = 100;
const int startingShiftSpeed = 350;
const long rpmUpdateInterval = 50000;
unsigned long rpmUpdatePreviousMicros = 0;
int targetRPM = 0;
float percentThrottle = 0;

//Communication Variables
// REPLACE WITH THE MAC Address of your receiver
uint8_t broadcastAddress[] = {0x3C, 0x71, 0xBF, 0x64, 0x55, 0x04};

// Define variables to store incoming readings
float incomingPercentThrottle;
int incomingAction;
int incomingCurrentGear; //Used only for learning purposes

//Define variables to control how often data is sent or incoming data processed
unsigned long previousUpdateRemoteDataMicros = 0;
unsigned long previousProcessIncomingRemoteMicros = 0;
const long updateRemoteDataInterval = 50000;
const long processIncomingRemoteDataInterval = 10000;

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message
{
  int rpmMain;
  int rpmLay;
  float percentThrottle;
  int currentGear;
  int action;
} struct_message;

bool actionFlag = false; //Used to stop loading data recieved until the action that was sent is processed

//Values for Action variable in messages
const int NoAction = 0;
const int Upshift = 1;
const int Downshift = 2;
const int BeginLearnMode = 3;
const int StoreValues = 4;
const int EndLearnMode = 5;
const int StartStopEngine = 6;
const int Error = 9;

// Create a struct_message called message to hold sensor readings
struct_message message;

// Create a struct_message to hold incoming sensor readings
struct_message incomingMessage;

//Gear and pot values
const int gearValueThreshold = 35; //Threshold range for pot values to confirm shift occurred
int currentGear = 0;               //Variable to store the current selected gear

//Array to save the pot values for the gear positions
// {Pot Val Left, Pot Val Right, ShiftBarrelofGear(L=0 R =1) that moves first for an upshift, Gear number}
// Values must be reassigned based on learning which will be loaded from eeprom
int gearSettings[8][4] =
    {
        {4000, 4000, 1, 0}, //Neutral
        {3500, 3500, 1, 1}, //1st
        {3000, 3000, 1, 2}, //2nd
        {2500, 2500, 0, 3}, //3rd
        {2000, 2000, 1, 4}, //4th
        {1500, 1500, 0, 5}, //5th
        {1000, 1000, 1, 6}, //6th
        {500, 500, 0, 7}    //7th
};

int pwmLookupByRPM[18][2];

//Function Declarations
int ShiftGears(int, int);
bool moveBarrel(AccelStepper, int, int, int);
void LearnGear(int);
void handleWeb();
void initializeWifi();
void LoadGearSettings();
void StoreGearSettings();
void Diagnostics();
bool CountPulses(int pin, bool &bounceState, bool &previousSensorState, unsigned long &previousMicros, unsigned long &pulseCount);
bool ComputeRPMOnInterval(unsigned long interval);
int FilterValue(int, int);
void ControlMotorSpeed(int);
void SendRemoteData();
void ProcessIncomingRemoteData();
void ProcessIncomingRemoteDataOnInterval();
void EnableDisableSteppers(bool state);
void RevMatch(int currentGear, int newGear);
void PopulatePWMLookup();

//**********************************************************************
// Callback when data is sent
//**********************************************************************
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0)
  {
    success = "Delivery Success :)";
  }
  else
  {
    success = "Delivery Fail :(";
  }
}

//**********************************************************************
// Callback when data is received
//**********************************************************************
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));
  //Serial.print("Bytes received: ");
  //Serial.println(len);

  //Flag that an action has occured and must be processed, stop loading data from remote until action has been handled
  if (!actionFlag)
  {
    incomingPercentThrottle = incomingMessage.percentThrottle;
    incomingCurrentGear = incomingMessage.currentGear;
    incomingAction = incomingMessage.action;
    if (incomingAction != NoAction)
    {
      actionFlag = true;
    }
  }
}

//**********************************************************************
// Setup
//**********************************************************************
void setup()
{
  Serial.begin(115200); // set up Serial library at 115200 bps

  //Initialize Wifi
  initializeWifi();

  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(escPin, pwmChannel);

  //Set stepper sleep pin mode to output
  pinMode(stepperRSleepPin, OUTPUT);
  pinMode(stepperLSleepPin, OUTPUT);

  stepperR.setMaxSpeed(600);
  stepperL.setMaxSpeed(600);

  //Disable the steppers until they are needed
  EnableDisableSteppers(false);

  //Initialize pins for rpm photosensors
  pinMode(mainRPMPin, INPUT_PULLUP); // Set as an input
  pinMode(layRPMPin, INPUT_PULLUP);

  // initialize EEPROM with predefined size, required for ESP32
  EEPROM.begin(__SIZEOF_INT__ * 16);

  //Initialize Photo sensor states based on if they are currently blocked/unblocked
  mainPreviousRPMSensorState = digitalRead(mainRPMPin);
  layPreviousRPMSensorState = digitalRead(layRPMPin);

  //Load the gear position settings from Eeprom
  LoadGearSettings();

  //Check shift drums and pots for any issues
  Diagnostics();

  PopulatePWMLookup();
}
//**********************************************************************
//Loop
//**********************************************************************
void loop()
{

  //Don't take in remote data until the action that was recieved is executed
  if (!actionFlag)
  {
    //Process any incoming data from the remote
    ProcessIncomingRemoteDataOnInterval();
  }
  else
  {
    //If there is an action to process, immediately skip over timer and process the action
    ProcessIncomingRemoteData();
  }
  //Update Remote Data
  SendRemoteData();

  //Count Pulses with Debouncing
  CountPulses(mainRPMPin, mainBounceState, mainPreviousRPMSensorState, mainPreviousMicros, mainPulseCount);
  CountPulses(layRPMPin, layBounceState, layPreviousRPMSensorState, layPreviousMicros, layPulseCount);

  if (ComputeRPMOnInterval(rpmUpdateInterval)) //If the RPM value has been refreshed, control the motor's speed based on new value
  {
    if (engineRunning)
    {
      targetRPM = vehicle.Simulate(incomingPercentThrottle, layRpmFiltered, mainRpmFiltered, currentGear);
    }
    else
    {
      targetRPM = 0;
    }

    ControlMotorSpeed(targetRPM);
  }
}
//**********************************************************************
//  UpdateRemoteData
//**********************************************************************
void SendRemoteData()
{
  if (micros() - previousUpdateRemoteDataMicros > updateRemoteDataInterval)
  {
    message.currentGear = currentGear;
    message.rpmMain = mainRpmFiltered;
    message.rpmLay = layRpmFiltered;

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&message, sizeof(message));

    if (result == ESP_OK)
    {
      //Serial.println("Sent with success");
    }
    else
    {
      // Serial.println("Error sending the data");
    }

    previousUpdateRemoteDataMicros = micros();
  }
}

void ProcessIncomingRemoteDataOnInterval()
{

  //Only Process incoming remote data on an interval
  if (micros() - previousProcessIncomingRemoteMicros > processIncomingRemoteDataInterval)
  {
    //Serial.println("Process Remote Data on Interval");
    ProcessIncomingRemoteData();
    previousProcessIncomingRemoteMicros = micros();
  }
}

//**********************************************************************
// ProcessIncomingRemoteData
//**********************************************************************
void ProcessIncomingRemoteData()
{
  /*
  Serial.println("INCOMING VALUES");
  Serial.print("Percent Throttle");
  Serial.println(incomingPercentThrottle);
   */
  //Serial.print("Incoming Action: ");
  //Serial.println(incomingAction);
  if (actionFlag)
  {
    switch (incomingAction)
    {
    case NoAction:
      //############################################
      percentThrottle = incomingPercentThrottle;
      //Place code here to process throttle and calculate target RPM
      //Call Vehicle Simulation
      break;
    case Upshift:
      currentGear = ShiftGears(currentGear + 1, currentGear);
      //TODO Lower revs of motor based on new gear ratio
      break;
    case Downshift:
      currentGear = ShiftGears(currentGear - 1, currentGear);
      //TODO: Up revs of motor based on new gear ratio
      break;
    case BeginLearnMode:
      //Stop the motor
      ControlMotorSpeed(0);
      break;
    case StoreValues:
      //Call learn gears to store the current gear's shift drum positions
      LearnGear(incomingCurrentGear);
      break;
    case EndLearnMode:
      //Learning is complete, store to EEPROM and reset flags and gear
      StoreGearSettings();
      break;
    case StartStopEngine:
      //If the engine isn't running start it up, otherwise turn it off
      Serial.println("Engine Start/Stop");
      engineRunning = !engineRunning;
      break;
    case Error:
      Serial.println("Error");
      break;
    }
    incomingAction = NoAction;
    incomingMessage.action = NoAction;
    actionFlag = false;
  }
}

//**********************************************************************
//  Filter a Value
//**********************************************************************
int FilterValue(int newValue, int previousValue)
{
  float EMA_a = 0.6; // EMA Alpha
  return newValue = (EMA_a * previousValue) + ((1 - EMA_a) * newValue);
}

//**********************************************************************
//  EnableDisableSteppers
//**********************************************************************
void EnableDisableSteppers(bool state)
{

  if (state)
  {
    //Enable Steppers
    digitalWrite(stepperLSleepPin, HIGH);
    digitalWrite(stepperRSleepPin, HIGH);
  }
  else
  {
    //Disable Steppers
    digitalWrite(stepperLSleepPin, LOW);
    digitalWrite(stepperRSleepPin, LOW);
  }
}

//**********************************************************************
//  Shift Gears
//**********************************************************************
int ShiftGears(int newGear, int currentGear)
{

  //Check to ensure that the new gear is valid
  if (newGear > 7 || newGear < 0 || currentGear == newGear)
  {
    Serial.println("Invalid Gear Selection");
    return currentGear;
  }

  bool shiftComplete = false;

  int filteredValueRPot = analogRead(potRPin);
  int filteredValueLPot = analogRead(potLPin);

  Serial.print("Current Value R Pot ");
  Serial.println(filteredValueRPot);
  Serial.print("Current Value L Pot ");
  Serial.println(filteredValueLPot);

  Serial.print("Destination Value R Pot ");
  Serial.println(gearSettings[newGear][RIGHT]);
  Serial.print("Destination Value L Pot ");
  Serial.println(gearSettings[newGear][LEFT]);

  int stepperRRotationDirection = 1;
  int stepperLRotationDirection = 1;

  unsigned long timer = micros();

  //Right barrel roation direction
  if (gearSettings[newGear][RIGHT] < filteredValueRPot)
  {
    stepperRRotationDirection = CCW;
  }
  else
  {
    stepperRRotationDirection = CW;
  }

  //Left barrel rotation direction
  if (gearSettings[newGear][LEFT] > filteredValueLPot)
  {
    stepperLRotationDirection = CW;
  }
  else
  {
    stepperLRotationDirection = CCW;
  }

  //Enable steppers
  EnableDisableSteppers(true);

  //Cut Throttle via to release sliders from dogs
  if (currentGear < newGear)
  {
    speedControlPWM = speedControlPWM * vehicle.ThrottleCut();
    ledcWrite(pwmChannel, speedControlPWM);
  }

  //Determine which barrel to move first

  //Upshift and left barrel is marked first
  if (gearSettings[newGear][2] == 0 && currentGear < newGear)
  {
    //Left then right barrels
    Serial.println("$$$$$$$$$$$ LEFT THEN RIGHT $$$$$$$$$$$$$$");
    if (moveBarrel(stepperL, stepperLRotationDirection, potLPin, gearSettings[newGear][LEFT]))
    {
      shiftComplete = moveBarrel(stepperR, stepperRRotationDirection, potRPin, gearSettings[newGear][RIGHT]);
      if (!shiftComplete)
      {
        Serial.println("Error with Right Barrel");
      }
    }
    else
    {
      Serial.println("Error with Left Barrel");
    }
  }
  //Upshift and right barrel is marked first
  else if (gearSettings[newGear][2] == 1 && currentGear < newGear)
  {
    Serial.println("$$$$$$$$$$$ RIGHT THEN LEFT $$$$$$$$$$$$$$");
    //Right then left barrels
    if (moveBarrel(stepperR, stepperRRotationDirection, potRPin, gearSettings[newGear][RIGHT]))
    {
      shiftComplete = moveBarrel(stepperL, stepperLRotationDirection, potLPin, gearSettings[newGear][LEFT]);
      if (!shiftComplete)
      {
        Serial.println("Error with Left Barrel");
      }
    }
    else
    {
      Serial.println("Error with Right Barrel");
    }
  }
  //Downshift and left barrel marked first (We swap for downshifts)
  else if (gearSettings[newGear][2] == 0 && currentGear > newGear)
  {
    Serial.println("$$$$$$$$$$$ RIGHT THEN LEFT $$$$$$$$$$$$$$");
    //Right then left barrels
    if (moveBarrel(stepperR, stepperRRotationDirection, potRPin, gearSettings[newGear][RIGHT]))
    {
      shiftComplete = moveBarrel(stepperL, stepperLRotationDirection, potLPin, gearSettings[newGear][LEFT]);
      if (!shiftComplete)
      {
        Serial.println("Error with Left Barrel");
      }
    }
    else
    {
      Serial.println("Error with Right Barrel");
    }
  }
  //Downshift and right barrel marked first (We swap for downshifts
  else if (gearSettings[newGear][2] == 1 && currentGear > newGear)
  {
    //Left then right barrels
    Serial.println("$$$$$$$$$$$ LEFT THEN RIGHT $$$$$$$$$$$$$$");
    if (moveBarrel(stepperL, stepperLRotationDirection, potLPin, gearSettings[newGear][LEFT]))
    {
      shiftComplete = moveBarrel(stepperR, stepperRRotationDirection, potRPin, gearSettings[newGear][RIGHT]);
      if (!shiftComplete)
      {
        Serial.println("Error with Right Barrel");
      }
    }
    else
    {
      Serial.println("Error with Left Barrel");
    }
  }

  long totalTime = micros() - timer;

  //Disable steppers
  EnableDisableSteppers(false);

  if (shiftComplete)
  {
    Serial.print("Gear shift complete, new gear is ");
    Serial.println(newGear);
    Serial.print("Shift time is ");
    Serial.println(totalTime);
    return newGear;
  }
  else
  {
    Serial.println("Shift Failed");
    engineRunning = false;
    return currentGear;
  }
}

//**********************************************************************
//  Move Barrel
//**********************************************************************
bool moveBarrel(AccelStepper stepper, int rotationDirection, int potPin, int destinationPotValue)
{
  int filteredValue = analogRead(potPin);
  int stepperBaseSpeed = startingShiftSpeed * rotationDirection;
  stepper.setSpeed(stepperBaseSpeed); //Set initial speed and direction of stepper
  float accelRate = 1.6;

  int startPosition = filteredValue;
  int extent = abs(startPosition - destinationPotValue);
  //Set the reached destination boolean, barrel maybe already in destination position
  bool reachedDestination = extent <= gearValueThreshold;

  int stepCount = 0;
  int previousExtent = 0;
  int speed = 0;
  //Run this loop until the pot value is within the threshold value for the target gear
  while (!reachedDestination)
  {
    if (stepper.runSpeed())
    {
      stepCount++;
      speed = (startingShiftSpeed + pow(accelRate, stepCount)) * rotationDirection;

      stepper.setSpeed(speed);

      filteredValue = FilterValue(analogRead(potPin), filteredValue);

      extent = abs(filteredValue - destinationPotValue);

      previousExtent = extent;

      reachedDestination = extent <= gearValueThreshold;
    }

    //Safety to prevent damage to pots which has happened
    if (filteredValue > 4090 || filteredValue < 0)
    {
      Serial.println("-------------POT VALUE EXCEEDED BOUNDS-----------------");
      Serial.print("Filtered Value that Exceeded Bounds: ");
      Serial.println(filteredValue);
      return false;
    }
  }

  /*
  Serial.println("*************Move Barrel Complete******************");
  Serial.print("Destination Pot Value: ");
  Serial.println(destinationPotValue);
  Serial.print("Filtered Pot Value: ");
  Serial.println(filteredValue);
  */
  return true;
}

//**********************************************************************
//  Rev Match the Gears Between shifts
//**********************************************************************
void RevMatch(int currentGear, int newGear)
{
  float revRatio = vehicle.RevMatch(currentGear, newGear);

  //Use the target RPM not the current RPM because of the throttle cut
  targetRPM = targetRPM * revRatio;
  int lowerBinIndex = 0;
  int upperBinIndex = 0;

  for (int i = 0; i < 24; i++)
  {
    if (pwmLookupByRPM[i][0] >= targetRPM)
    {
      lowerBinIndex = i - 1;
      upperBinIndex = i;
      //Linearly interpolate between upper and lower values of the lookup to determine the perfect PWM value to match the revs
      speedControlPWM = map(targetRPM, pwmLookupByRPM[0][lowerBinIndex], pwmLookupByRPM[0][upperBinIndex], pwmLookupByRPM[1][lowerBinIndex], pwmLookupByRPM[1][upperBinIndex]);
      break;
    }
  }
  ledcWrite(pwmChannel, speedControlPWM);

  //Wait 2/100ths of a second for speed to change
  delayMicroseconds(20000);
}

//**********************************************************************
//  Learn pot values for each gear
//**********************************************************************
void LearnGear(int gear)
{
  int filteredValueRPot = analogRead(potRPin);
  int filteredValueLPot = analogRead(potLPin);

  //Read pot value and run thru the filter function x times to eliminate jitter
  for (int i = 0; i < 3; i++)
  {
    filteredValueRPot = FilterValue(analogRead(potRPin), filteredValueRPot);
    filteredValueLPot = FilterValue(analogRead(potLPin), filteredValueLPot);
    delay(1);
  }

  //store filtered value into the array
  gearSettings[gear][RIGHT] = filteredValueRPot;
  gearSettings[gear][LEFT] = filteredValueLPot;

  Serial.println("Stored Values");
  Serial.print("R Pot");
  Serial.println(gearSettings[gear][RIGHT]);
  Serial.print("L Pot");
  Serial.println(gearSettings[gear][LEFT]);
}

//**********************************************************************
//  Set the speed of the motor
//**********************************************************************
void ControlMotorSpeed(int rpmTarget)
{
  if (rpmTarget > 0)
  {
    int error = rpmTarget - layRpmFiltered;
    float dt = (micros() - previousMotorSpeedControlMicros) / 1000000.0f;

    integral = integral + (error * dt);
    if (abs(integral) > 200)
    {
      integral = 180;
    }
    float derivative = (error - previousError) / dt;
    previousError = error;

    speedControlPWM =Kp * error + Ki * integral + Kd * derivative;

    speedControlPWM = constrain(speedControlPWM, 10, 180);
    /*
    Serial.print("Error ");
    Serial.println(error);
    Serial.print("DT ");
    Serial.println(dt);
    Serial.print("speedControlPWM ");
    Serial.println(speedControlPWM);
    Serial.print("derivative ");
    Serial.println(derivative);
    Serial.print("integral ");
    Serial.println(integral);
    Serial.print("Error ");
    Serial.println(error);
    Serial.print("DT ");
    Serial.println(dt);
    */
  }
  else
  {
    speedControlPWM = 0;
  }

  ledcWrite(pwmChannel, speedControlPWM);

  previousMotorSpeedControlMicros = micros();
}

//**********************************************************************
//  Compute RPM Count pulses and display rpm on an interval
//**********************************************************************
bool ComputeRPMOnInterval(unsigned long interval)
{
  bool rpmUpdated = false;
  //Check when the RPM reading was last updated
  unsigned long dt = micros() - rpmUpdatePreviousMicros;

  if (dt >= interval)
  {
    mainRpm = (mainPulseCount / 12.0000) / (dt / 60000000.0000);
    layRpm = (layPulseCount / 12.0000) / (dt / 60000000.0000);
    mainPulseCount = 0;
    layPulseCount = 0;

    mainRpmFiltered = FilterValue(mainRpm, mainRpmFiltered);
    layRpmFiltered = FilterValue(layRpm, layRpmFiltered);

    //Serial.println("MainRPM,LayRPM ");
    Serial.print(mainRpmFiltered);
    Serial.print(",");
    Serial.println(layRpmFiltered);

    //Serial.print("Main RPM: ");
    //Serial.println(mainRpm);

    // Serial.print("Lay RPM: ");
    // Serial.println(layRpm);

    rpmUpdatePreviousMicros = micros();
    rpmUpdated = true;
  }
  return rpmUpdated;
}

//**********************************************************************
//  CountPulse from Photosensors
//**********************************************************************
bool CountPulses(int pin, bool &bounceState, bool &previousSensorState, unsigned long &previousMicros, unsigned long &pulseCount)
{
  boolean currentSensorState = digitalRead(pin);
  boolean pulseCountUpdated = false;
  /*
Serial.print ("Previous Sensor State: ");
Serial.println(previousSensorState);
Serial.print ("Current Sensor State: ");
Serial.println(currentSensorState); */

  if (bounceState == WATCH_SENSOR)
  {
    if (previousSensorState != currentSensorState)
    {
      bounceState = IGNORE_SENSOR;
      previousMicros = micros();
    }
  }
  else
  {
    unsigned long timeElapsed = micros() - previousMicros;

    //Check if interval has expired and we've debounced long enough
    if (timeElapsed > rpmDebounceInterval)
    {
      if (currentSensorState == BLOCKED && previousSensorState != currentSensorState)
      {
        pulseCount++;
        pulseCountUpdated = true;
      }
      bounceState = WATCH_SENSOR;
      previousSensorState = currentSensorState;
    }
  }
  return pulseCountUpdated;
}

//**********************************************************************
//  Initialize Wifi
//**********************************************************************
void initializeWifi()
{
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo;

  while (esp_now_add_peer(&peerInfo) != ESP_OK)
  {

    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
      Serial.println("Failed to add peer");
      delay(1000);
    }
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

//**********************************************************************
//LoadGearSettings
// Load gear settings from eeprom
//**********************************************************************
void LoadGearSettings()
{
  //Check to see if any settings have been stored to eeprom yet.  If no, then don't try to load from eeprom.
  //int val = EEPROM.read(0);
  //if (val != 255)
  //{
  int eeAddress = 0;
  //Read gear settings to eeprom
  for (int i = 0; i <= 7; i++)
  {
    Serial.print("Read int from EEPROM: ");
    EEPROM.get(eeAddress, gearSettings[i][LEFT]);
    eeAddress += __SIZEOF_INT__;
    EEPROM.get(eeAddress, gearSettings[i][RIGHT]);
    eeAddress += __SIZEOF_INT__;
    Serial.print("Left Values");
    Serial.println(gearSettings[i][LEFT]);
    Serial.print("Right Values");
    Serial.println(gearSettings[i][RIGHT]);
  }
  //}
}

//**********************************************************************
//StoreGearSettings
// Save gear settings to eeprom
//**********************************************************************
void StoreGearSettings()
{
  int eeAddress = 0;
  //Store each gear setting into eeprom
  for (int i = 0; i <= 7; i++)
  {
    EEPROM.put(eeAddress, gearSettings[i][LEFT]);
    eeAddress += __SIZEOF_INT__;
    EEPROM.put(eeAddress, gearSettings[i][RIGHT]);
    eeAddress += __SIZEOF_INT__;
    Serial.print("Left Values");
    Serial.println(gearSettings[i][LEFT]);
    Serial.print("Right Values");
    Serial.println(gearSettings[i][RIGHT]);
  }
  if (EEPROM.commit())
  {
    Serial.println("Values Successfully Saved to EEPROM");
  }
}

//**********************************************************************
//  Diagnostics
//**********************************************************************
void Diagnostics()
{
  int initialRPotValue = analogRead(potRPin);
  int initialLPotValue = analogRead(potLPin);
  int RTestPos = 10 * CCW;
  int LTestPos = 10 * CW;
  stepperR.setSpeed(startingShiftSpeed * CCW);
  stepperR.setAcceleration(200);
  stepperL.setSpeed(startingShiftSpeed * CW);
  stepperL.setAcceleration(200);
  bool passedRTest = false;
  bool passedLTest = false;

  /*
  while(1)
  {
         Serial.print("Intial R Value: ");
      Serial.println(initialRPotValue);
      Serial.print("Intial L Value: ");
      Serial.println(initialLPotValue); 
      delay(1000);
  }
  */

  Serial.println("********************************************************************");
  Serial.println("Begin Diagnostics");
  Serial.println("********************************************************************");

  //Enable steppers
  EnableDisableSteppers(true);

  while (!passedRTest || !passedLTest)
  {
    passedRTest = false;
    passedLTest = false;

    //Check if one of the pots is reading too close to a limit to get an accurate position reading
    if (initialRPotValue > 4085 || initialRPotValue < 10 || initialLPotValue > 4085 || initialLPotValue < 10)
    {
      Serial.println("Failed: A pot value is too close to a limit of either 0 or 4095, re-index the pot to the shift barrel");
      Serial.print("Intial R Value: ");
      Serial.println(initialRPotValue);
      Serial.print("Intial L Value: ");
      Serial.println(initialLPotValue);
    }
    //****************************
    //Test Right
    stepperR.setCurrentPosition(0);
    stepperR.runToNewPosition(RTestPos); //Rotate CCW
    int currentRPotValue = analogRead(potRPin);
    stepperR.runToNewPosition(0);
    if (currentRPotValue < initialRPotValue)
    {
      Serial.println("Passed: Right Barrel is Correct");
      passedRTest = true;
    }
    else
    {
      if (abs(currentRPotValue - initialRPotValue) > gearValueThreshold)
      {
        Serial.println("Failed: Right Pot not reading correctly, or L & R are swapped");
      }
      else
      {
        Serial.println("Failed: Right Barrel Rotation Direction Backwards");
      }
    }

    Serial.print("Intial R Value: ");
    Serial.println(initialRPotValue);

    Serial.print("Test R Value: ");
    Serial.println(currentRPotValue);

    //****************************
    //Test Left
    stepperL.setCurrentPosition(0);
    stepperL.runToNewPosition(LTestPos);
    int currentLPotValue = analogRead(potLPin);
    stepperL.runToNewPosition(0);
    if (currentLPotValue > initialLPotValue) //Rotate CW
    {
      Serial.println("Passed: Left Barrel is Correct");
      passedLTest = true;
    }
    else
    {
      if (abs(currentLPotValue - initialLPotValue) > gearValueThreshold)
      {
        Serial.println("Failed:  Left Pot not reading correctly, or L & R are swapped");
      }
      else
      {
        Serial.println("Failed: Left Barrel Rotation Direction Backwards");
      }
    }
    Serial.print("Intial L Value: ");
    Serial.println(initialLPotValue);

    Serial.print("Test L Value: ");
    Serial.println(currentLPotValue);

    //Disable steppers
    digitalWrite(stepperRSleepPin, LOW);
    digitalWrite(stepperLSleepPin, LOW);

    delay(1000);
  }
}

void PopulatePWMLookup()
{
  const int rpmIncriment = 25;
  Serial.println("RPM,Gear");
  for (int i = 0; i < 18; i++)
  {
    int lookupTargetRPM = i * rpmIncriment + 350;
    Serial.print("Target RPM: ");
    Serial.println(lookupTargetRPM);

    bool recordedValue = false;
    const int numSamples = 5;
    int pwmSum = 0;
    int sampleCount = 0;

    while (!recordedValue)
    {
      //Count Pulses with Debouncing
      CountPulses(mainRPMPin, mainBounceState, mainPreviousRPMSensorState, mainPreviousMicros, mainPulseCount);
      CountPulses(layRPMPin, layBounceState, layPreviousRPMSensorState, layPreviousMicros, layPulseCount);

      if (ComputeRPMOnInterval(rpmUpdateInterval)) //If the RPM value has been refreshed, control the motor's speed based on new value
      {
        ControlMotorSpeed(lookupTargetRPM);
      }
      if (abs(layRpmFiltered - lookupTargetRPM) < 15)
      {
        pwmSum += speedControlPWM;
        sampleCount++;
        delayMicroseconds(50000);
      }
      if (sampleCount > 4)
      {
        //Store RPM
        pwmLookupByRPM[i][0] = lookupTargetRPM;
        //Store Average PWM value into table
        pwmLookupByRPM[i][1] = pwmSum / numSamples;
        recordedValue = true;
      }
    }
    Serial.print(pwmLookupByRPM[i][0]);
    Serial.print(",");
    Serial.println(pwmLookupByRPM[i][1]);
  }
}