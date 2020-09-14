#include <Arduino.h>
#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>
#include <EEPROM.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>
#include <VehicleSimulation.h>
#include <AS5600.h>

//Define all the pins
const int stepperLPin = 23;      //Step pin
const int stepperLDirPin = 17;   // Direction Pin
const int stepperLSleepPin = 2;  // Sleep driver pin
const int stepperRPin = 16;      //Step pin
const int stepperRDirPin = 19;   // Direction Pin
const int stepperRSleepPin = 15; // Sleep driver pin
const int escPin = 12;           //Speed controller pin
const int mainRPMPin = 5;        //Pin for interupt of the mainshaft photo sensor
const int layRPMPin = 18;        //Pin for interupt of the layshaft photo sensor
const int acceleratorPin = 36;   //Pin for input from a 10k pot representing accelerator
const int sda2Pin = 33;          //2nd I2C Bus
const int scl2Pin = 32;          //2nd I2C Bus

// Initialiize the Stepper motors
AccelStepper stepperR(1, stepperRPin, stepperRDirPin);
AccelStepper stepperL(1, stepperLPin, stepperLDirPin);

//
VehicleSimulation vehicle;

//
AS5600 rightEncoder;
AS5600 leftEncoder = AS5600(&Wire1, sda2Pin, scl2Pin);

//Direction Definitions for Steppers
const int CW = -1;
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
const float Kp = 0.5f; //0.6
const float Ki = 0.1f; //0.15
const float Kd = 0.02f; //.15
unsigned long previousMicros = 0;
bool engineRunning = true;

const int rpmDebounceInterval = 50;
const long rpmUpdateInterval = 50000;
unsigned long rpmUpdatePreviousMicros = 0;
int targetRPM = 0;
float percentThrottle = 0;

//Communication Variables
// REPLACE WITH THE MAC Address of your receiver
uint8_t broadcastAddress[] = {0x3C, 0x71, 0xBF, 0x64, 0x55, 0x04};

// Define variables to store incoming readings
float incomingPercentThrottle;
int incomingCurrentGear; //Used only for learning purposes

//Define variables to control how often data is sent or incoming data processed
unsigned long previousUpdateRemoteDataMicros = 0;
unsigned long previousProcessIncomingRemoteMicros = 0;
const long updateRemoteDataInterval = 50000;
const long processIncomingRemoteDataInterval = 25000;

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message
{
  int id;
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
const int StoreValues = 4;
const int StartStopEngine = 6;
const int Error = 9;

// Create a struct_message called message to hold sensor readings
struct_message message;

// Create a struct_message to hold incoming sensor readings
struct_message incomingMessage;

// Create a structure to hold the readings from each board
struct_message board1;
struct_message board2;

// Create an array with all the structures
struct_message boardsStruct[2] = {board1, board2};

//Gear and pot values
const int gearValueThreshold = 40; //Threshold range for pot values to confirm shift occurred
int currentGear = 0;               //Variable to store the current selected gear

//Array to save the pot values for the gear positions
// {Pot Val Left, Pot Val Right, ShiftBarrelofGear(L=0 R =1) that moves first for an upshift, Gear number}
// Values must be reassigned based on learning which will be loaded from eeprom
int gearSettings[8][4] =
    {
        {4000, 4000, 1, 0}, //Neutral
        {3500, 3500, 0, 1}, //1st
        {3000, 3000, 1, 2}, //2nd
        {2500, 2500, 0, 3}, //3rd
        {2000, 2000, 1, 4}, //4th
        {1500, 1500, 0, 5}, //5th
        {1000, 1000, 1, 6}, //6th
        {500, 500, 0, 7}    //7th
};

const int stepperCamPoint = 11;
const int startingStepperSpeed = 500; //Used by diagnostics function only
int pwmLookupByRPM[18][2];

//Function Declarations
int ShiftGears(int, int);
bool moveBarrel(AccelStepper stepper, int rotationDirection, AS5600 encoder, int destinationPosition, bool isEngagingGear);
void LearnGear();
void handleWeb();
void initializeWifi();
void LoadGearSettings();
void StoreGearSettings();
void Diagnostics();
bool CountPulses(int pin, bool &bounceState, bool &previousSensorState, unsigned long &previousMicros, unsigned long &pulseCount);
bool ComputeRPMOnInterval(unsigned long interval);
int FilterValue(int, int, float EMA_a);
void ControlMotorSpeed(int);
void SendRemoteData();
void ProcessIncomingRemoteData();
void ProcessIncomingRemoteDataOnInterval();
void EnableDisableSteppers(bool state);
void PopulatePWMLookup();
int FindExtent(int currentPosition, int destinationPosition);

//**********************************************************************
// Callback when data is sent
//**********************************************************************
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
  if (incomingMessage.id == 0 && !actionFlag)
  {
      //incomingCurrentGear = incomingMessage.currentGear;
      boardsStruct[incomingMessage.id].action = incomingMessage.action;

      if (boardsStruct[incomingMessage.id].action != NoAction)
      {
        actionFlag = true;
    }
  }
  else if (incomingMessage.id == 1)
  {
      boardsStruct[incomingMessage.id].percentThrottle = incomingMessage.percentThrottle;
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

  // configure PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(escPin, pwmChannel);

  //Set stepper sleep pin mode to output
  pinMode(stepperRSleepPin, OUTPUT);
  pinMode(stepperLSleepPin, OUTPUT);

  stepperR.setMaxSpeed(4500);
  stepperL.setMaxSpeed(4500);

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
  // Diagnostics();

  //PopulatePWMLookup();
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
      targetRPM = vehicle.Simulate(boardsStruct[1].percentThrottle, layRpmFiltered, currentGear);
      Serial.print("New Target RPM: ");
      Serial.println(targetRPM);
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
    message.id = 2;

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&message, sizeof(message));

    if (result == ESP_OK)
    {
      Serial.println("Success");
    }
    else if (result == ESP_ERR_ESPNOW_NOT_INIT)
    {
      // How did we get so far!!
      Serial.println("ESPNOW not Init.");
    }
    else if (result == ESP_ERR_ESPNOW_ARG)
    {
      Serial.println("Invalid Argument");
    }
    else if (result == ESP_ERR_ESPNOW_INTERNAL)
    {
      Serial.println("Internal Error");
    }
    else if (result == ESP_ERR_ESPNOW_NO_MEM)
    {
      Serial.println("ESP_ERR_ESPNOW_NO_MEM");
    }
    else if (result == ESP_ERR_ESPNOW_NOT_FOUND)
    {
      Serial.println("Peer not found.");
    }
    else
    {
      Serial.println("Not sure what happened");
    }
    delay(10);

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

  Serial.println("INCOMING VALUES");
  Serial.print("Percent Throttle");
  Serial.println(boardsStruct[1].percentThrottle);
  /*
  Serial.print("Incoming Action: ");
  Serial.println(incomingAction);
  */
  percentThrottle = boardsStruct[1].percentThrottle;

  if (actionFlag)
  {
    switch (boardsStruct[0].action)
    {
    case NoAction:
      break;
    case Upshift:
      currentGear = ShiftGears(currentGear + 1, currentGear);
      break;
    case Downshift:
      currentGear = ShiftGears(currentGear - 1, currentGear);
      break;
    case StoreValues:
      //Call learn gears to store the neutral position offset of each shift drum
      LearnGear();
      Serial.println("Learn Gear");
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
    boardsStruct[0].action = NoAction;
    actionFlag = false;
  }
}

//**********************************************************************
//  Filter a Value
//**********************************************************************
int FilterValue(int newValue, int previousValue, float EMA_a)
{
  return newValue = (EMA_a * previousValue) + ((1 - EMA_a) * newValue);
}

//**********************************************************************
//  EnableDisableSteppers
//**********************************************************************
void EnableDisableSteppers(bool state)
{
  //Enable/Disable Steppers
  digitalWrite(stepperLSleepPin, state);
  digitalWrite(stepperRSleepPin, state);
}

//**********************************************************************
//  Shift Gears
//**********************************************************************
int ShiftGears(int newGear, int currentGear)
{
  //Plan the Shift
  //Check to ensure that the new gear is valid
  if (newGear > 7 || newGear < 0)
  {
    Serial.println("Invalid Gear Selection");
    return currentGear;
  }

  bool shiftComplete = false;

  //Logging
  /*
  Serial.print("Current Value R Barrel ");
  Serial.println(rightEncoder.getPosition());
  Serial.print("Current Value L Barrel ");
  Serial.println(leftEncoder.getPosition());

  Serial.print("Destination Value R Barrel ");
  Serial.println(gearSettings[newGear][RIGHT]);
  Serial.print("Destination Value L Barrel ");
  Serial.println(gearSettings[newGear][LEFT]);
*/

  int shiftDirection = (newGear - currentGear) / abs(newGear - currentGear);
  int stepperRRotationDirection = -1 * shiftDirection;
  int stepperLRotationDirection = 1 * shiftDirection;
 
  unsigned long timer = micros();

  //Enable steppers
  EnableDisableSteppers(true);

  //Small Throttle Cut to release sliders from dogs
  if (engineRunning)
  {
    speedControlPWM = speedControlPWM * vehicle.ThrottleCut();
    ledcWrite(pwmChannel, speedControlPWM);
  }
  //Determine which barrel to move first
  if (gearSettings[newGear][2])
  {
    Serial.println("2$$$$$$$$$$$ RIGHT THEN LEFT $$$$$$$$$$$$$$");
    //Right then left barrels
    if (moveBarrel(stepperR, stepperRRotationDirection, rightEncoder, gearSettings[newGear][RIGHT], false))
    {
      //Rev Match code goes here
      vehicle.RevMatch(currentGear, newGear, layRpm);
      shiftComplete = moveBarrel(stepperL, stepperLRotationDirection, leftEncoder, gearSettings[newGear][LEFT], true);
      if (!shiftComplete)
      {
        engineRunning = false;
        Serial.println("Error with Left Barrel");
      }
    }
    else
    {
      Serial.println("Error with Right Barrel");
    }
  }
  else
  {
    //Left then right barrels
    Serial.println("1$$$$$$$$$$$ LEFT THEN RIGHT $$$$$$$$$$$$$$");
    if (moveBarrel(stepperL, stepperLRotationDirection, leftEncoder, gearSettings[newGear][LEFT], false))
    {
      //Rev Match code goes here
      vehicle.RevMatch(currentGear, newGear, layRpm);
      shiftComplete = moveBarrel(stepperR, stepperRRotationDirection, rightEncoder, gearSettings[newGear][RIGHT], true);
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
bool moveBarrel(AccelStepper stepper, int rotationDirection, AS5600 encoder, int destinationPosition, bool isEngagingGear)
{
  int currentPosition = encoder.getPosition();
  stepper.setSpeed(startingStepperSpeed); //Set initial speed and direction of stepper

  int extent = FindExtent(currentPosition, destinationPosition);
  //Set the reached destination boolean, barrel maybe already in destination position
  bool reachedDestination = abs(extent) <= gearValueThreshold;

  Serial.print("Initial Position: ");
  Serial.println(currentPosition);
  Serial.print("Destination Position: ");
  Serial.println(destinationPosition);
  Serial.print("Initial Extent: ");
  Serial.println(extent);

  // Actual true number of steps taken
  int stepCount = 0;

  stepper.setAcceleration(35000);

  //Reset Stepper motor 0 position
  stepper.setCurrentPosition(0);
  stepper.move(extent * .205 * rotationDirection);

  //Run this loop until the value is within the threshold value for the target gear
  while (!reachedDestination)
  {
    //If a step occurred, set the new stepper speeds
    if (stepper.distanceToGo() != 0)
    {
      stepper.run();
    }
    else
    {
      currentPosition = encoder.getPosition();
      //Calculate how far we have to go to the destination
      extent = FindExtent(currentPosition, destinationPosition);
      reachedDestination = abs(extent) <= gearValueThreshold;

      //Reset Stepper motor 0 position
      stepper.setCurrentPosition(0);
      stepper.move(extent * .205 * rotationDirection);
      Serial.print("Checked Extent: ");
      Serial.println(extent);
      Serial.print("Checked Position: ");
      Serial.println(currentPosition);
    }
  }

  Serial.println("**************Barrel Move Complete*****************");
  Serial.print("Final Extent: ");
  Serial.println(extent);
  Serial.print("Final Position: ");
  Serial.println(currentPosition);
  Serial.print("Step Count: ");
  Serial.println(stepCount);

  return reachedDestination;
}
//**********************************************************************
//  Find the distance between two encoder positions taking into account rollover
//  at 0 and 4095
//**********************************************************************
int FindExtent(int currentPosition, int destinationPosition)
{
  //Calculate raw distance between two points, mod 360 to remove multiple rotations, then determine the shortest angle
  int raw_dist = (destinationPosition - currentPosition);
  int mod_dist = abs(raw_dist) % 4096;
  int short_dist = mod_dist;

  if (mod_dist >= 2048)
  {
    short_dist = 4096 - mod_dist;
  }

  return short_dist;
}

//**********************************************************************
//  Learn zero offset for neutral
//**********************************************************************
void LearnGear()
{
  int rightValue = rightEncoder.getPosition();
  int leftValue = leftEncoder.getPosition();

  for (int i = 0; i <= 7; i++)
  {
    if (rightValue + (i * 455) < 4095)
    {
      gearSettings[i][RIGHT] = rightValue + (i * 455);
    }
    else
    {
      //Account for going past 4095
      gearSettings[i][RIGHT] = (rightValue + (i * 455)) - 4095;
    }
    if (i > 1)
    {
      if (leftValue + ((i - 1) * -455) > 0)
      {
        gearSettings[i][LEFT] = leftValue + ((i - 1) * -455);
      }
      else
      {
        //Account for going below 0
        gearSettings[i][leftValue] = (leftValue + ((i - 1) * -455)) + 4095;
      }
    }
    else
    {
      //Left Barrel Does not move to upshift into first gear, or to downshift into neutral
      gearSettings[1][LEFT] = leftValue;
    }
  }
  //Save values to eeprom
  StoreGearSettings();
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
    if (abs(integral) > 220)
    {
      integral = 220;
    }
    float derivative = (error - previousError) / dt;
    previousError = error;

    speedControlPWM = Kp * error + Ki * integral + Kd * derivative;

    speedControlPWM = constrain(speedControlPWM, 30, 255);
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

    mainRpmFiltered = FilterValue(mainRpm, mainRpmFiltered, 0.9f);
    layRpmFiltered = FilterValue(layRpm, layRpmFiltered, 0.9f);

    /*
    Serial.println("MainRPM,LayRPM ");
    Serial.print(mainRpmFiltered);
    Serial.print(",");
    Serial.println(layRpmFiltered);
    */

    rpmUpdatePreviousMicros = micros();
    rpmUpdated = true;
  }
  return rpmUpdated;
}

//**********************************************************************
//  CountPulses from Photosensors
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
    ESP.restart();
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 6;
  peerInfo.encrypt = false;

  // Add peer
  esp_err_t addStatus = esp_now_add_peer(&peerInfo);

  if (addStatus == ESP_OK)
  {
    // Pair success
    Serial.println("Pair success");
  }
  else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT)
  {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  }
  else if (addStatus == ESP_ERR_ESPNOW_ARG)
  {
    Serial.println("Add Peer - Invalid Argument");
  }
  else if (addStatus == ESP_ERR_ESPNOW_FULL)
  {
    Serial.println("Peer list full");
  }
  else if (addStatus == ESP_ERR_ESPNOW_NO_MEM)
  {
    Serial.println("Out of memory");
  }
  else if (addStatus == ESP_ERR_ESPNOW_EXIST)
  {
    Serial.println("Peer Exists");
  }
  else
  {
    Serial.println("Not sure what happened");
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
  /*
  while (1)
  {
    for (int i = 1; i <= 3; i++)
    {
      ShiftGears(i, i - 1);
    }
    for (int i = 2; i >= 0; i--)
    {
      ShiftGears(i, i + 1);
    }
    delay(250);
  }

  EnableDisableSteppers(false);

  while (1)
  {
    Serial.print("Right Encoder");
    Serial.println(rightEncoder.getPosition());
    Serial.print("Left Encoder");
    Serial.println(leftEncoder.getPosition());
    delay(500);
  }
  EnableDisableSteppers(true);
*/
  int initialRPositionValue = rightEncoder.getPosition();
  int initialLPositionValue = leftEncoder.getPosition();
  int RTestPos = 10 * CCW;
  int LTestPos = 10 * CW;
  stepperR.setSpeed(startingStepperSpeed * CCW);
  stepperR.setAcceleration(200);
  stepperL.setSpeed(startingStepperSpeed * CW);
  stepperL.setAcceleration(200);
  bool passedRTest = false;
  bool passedLTest = false;

  /*
  while(1)
  {
      Serial.print("Intial R Value: ");
      Serial.println(initialRPositionValue);
      Serial.print("Intial L Value: ");
      Serial.println(initialLPositionValue); 
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

    //Check if one of the Positions is reading too close to a limit to get an accurate position reading
    if (initialRPositionValue > 4085 || initialRPositionValue < 10 || initialLPositionValue > 4085 || initialLPositionValue < 10)
    {
      Serial.println("Failed: A Position value is too close to a limit of either 0 or 4095, re-index the Position to the shift barrel");
      Serial.print("Intial R Value: ");
      Serial.println(initialRPositionValue);
      Serial.print("Intial L Value: ");
      Serial.println(initialLPositionValue);
    }
    //****************************
    //Test Right
    stepperR.setCurrentPosition(0);
    stepperR.runToNewPosition(RTestPos); //Rotate CCW
    int currentRPositionValue = rightEncoder.getPosition();
    stepperR.runToNewPosition(0);
    if (currentRPositionValue > initialRPositionValue)
    {
      Serial.println("Passed: Right Barrel is Correct");
      passedRTest = true;
    }
    else
    {
      if (abs(currentRPositionValue - initialRPositionValue) > gearValueThreshold)
      {
        Serial.println("Failed: Right Position not reading correctly, or L & R are swapped");
      }
      else
      {
        Serial.println("Failed: Right Barrel Rotation Direction Backwards");
      }
    }

    Serial.print("Intial R Value: ");
    Serial.println(initialRPositionValue);

    Serial.print("Test R Value: ");
    Serial.println(currentRPositionValue);

    //****************************
    //Test Left
    stepperL.setCurrentPosition(0);
    stepperL.runToNewPosition(LTestPos);
    int currentLPositionValue = leftEncoder.getPosition();
    stepperL.runToNewPosition(0);
    if (currentLPositionValue < initialLPositionValue) //Rotate CW
    {
      Serial.println("Passed: Left Barrel is Correct");
      passedLTest = true;
    }
    else
    {
      if (abs(currentLPositionValue - initialLPositionValue) > gearValueThreshold)
      {
        Serial.println("Failed:  Left Position not reading correctly, or L & R are swapped");
      }
      else
      {
        Serial.println("Failed: Left Barrel Rotation Direction Backwards");
      }
    }
    Serial.print("Intial L Value: ");
    Serial.println(initialLPositionValue);

    Serial.print("Test L Value: ");
    Serial.println(currentLPositionValue);

    //Disable steppers
    EnableDisableSteppers(false);
  }
}
