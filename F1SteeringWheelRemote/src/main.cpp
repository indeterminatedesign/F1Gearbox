#include <Arduino.h>
#include <SparkFunSX1509.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>

//States for Button States
#define WATCH_SENSOR true
#define IGNORE_SENSOR false

//Button Pins
const int button1Pin = 34;
const int button2Pin = 33;
const int upPaddlePin = 36;
const int downPaddlePin = 35;

//Button Bounce States
int button1PreviousState = LOW;
int button2PreviousState = LOW;
int upPaddlePreviousState = LOW;
int downPaddlePreviousState = LOW;

//Button Previous States
bool button1BounceState = WATCH_SENSOR;
bool button2BounceState = WATCH_SENSOR;
bool upPaddleBounceState = WATCH_SENSOR;
bool downPaddleBounceState = WATCH_SENSOR;

//Button Previous Microseconds
unsigned long previousButton1Micros = 0;
unsigned long previousButton2Micros = 0;
unsigned long previousUpPaddleMicros = 0;
unsigned long previousDownPaddleMicros = 0;

//Interval and Micros for Processing New Gearbox Data
unsigned long previousProcessMicros = 0;
int processInterval = 50000;

//Debounce time used for all button presses
const int buttonDebounceInterval = 5000;

//Shift Indicator Pins
//green
const int g1pin = 25;
const int g2pin = 32;
const int g3pin = 26;
const int g4pin = 27;
//red
const int r1pin = 16;
const int r2pin = 12;
const int r3pin = 4;
const int r4pin = 14;
//blue
const int b1pin = 5;
const int b2pin = 17;
const int b3pin = 19;
const int b4pin = 18;

int ledArray[12] =
    {
        g1pin,
        g2pin,
        g3pin,
        g4pin,
        r1pin,
        r2pin,
        r3pin,
        r4pin,
        b1pin,
        b2pin,
        b3pin,
        b4pin};
//Shift Indicator
const int maxRPM = 1200;
const int startRPM = 800;

//SevSeg sevseg;
SX1509 sevsegLeft;
SX1509 sevsegRight;

bool learnMode = false;

float percentThrottle = 0;

// Define variables to store incoming readings
int incomingAction;
int incomingMainRPM;
int incomingLayRPM;
int incomingCurrentGear;

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

//Communication Variables
// REPLACE WITH THE MAC Address of your receiver
uint8_t broadcastAddress[] = {0x3C, 0x71, 0xBF, 0x64, 0x3B, 0x8C};

// Variable to store if sending data was successful
String success;

//Define variables to control how often data is sent or incoming data processed
unsigned long previousUpdateGearboxDataMicros = 0;
unsigned long previousProcessIncomingGearboxMicros = 0;
const long updateGearboxDataInterval = 10000;
const long processIncomingGearboxDataInterval = 10000;

void Diagnostics();
void UpdateShiftIndicatorBasedonRPM(int currentRPM);
void ShiftIndicatorDisplay(int);
void TestDisplay();
void initializeWifi();
bool PollButton(int pin, bool &bounceState, int &previousSensorState, unsigned long &previousMicros);
void SendGearboxData(int action);
void LearningMode();
void ProcessIncomingGearboxData();
void GearIndicator(int gear);
void ButtonIndicator();

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
  // Serial.print("Bytes received: ");
  //Serial.println(len);
  incomingCurrentGear = incomingMessage.currentGear;
  incomingMainRPM = incomingMessage.rpmMain;
  incomingLayRPM = incomingMessage.rpmLay;

  //Serial.println(incomingLayRPM);
  //Serial.println(incomingCurrentGear);
}

//**********************************************************************
// Setup
//**********************************************************************
void setup()
{
  Serial.begin(115200);

  //Init ESP NOW
  initializeWifi();

  // Attach the pins for the shift lights
  for (int i = 0; i < 12; i++)
  {
    pinMode(ledArray[i], OUTPUT);
    digitalWrite(ledArray[i], HIGH);
  }

  //Initialize all button pins
  pinMode(button1Pin, INPUT_PULLDOWN);
  pinMode(button2Pin, INPUT_PULLDOWN);
  pinMode(upPaddlePin, INPUT_PULLDOWN);
  pinMode(downPaddlePin, INPUT_PULLDOWN);

  //Initialize SX1509s Multiplexers
  if (!sevsegLeft.begin(0x3E))
  {
    Serial.println("Left Multiplexer Failed to begin");
  }
  if (!sevsegRight.begin(0x3F))
  {
    Serial.println("Right Multiplexer Failed to begin");
  }

  //Initialize all of the multiplexer's pins
  for (int i = 0; i < 16; i++)
  {
    sevsegLeft.pinMode(i, ANALOG_OUTPUT);
    sevsegRight.pinMode(i, ANALOG_OUTPUT);
  }

  Diagnostics();
}

//**********************************************************************
// Loop
//**********************************************************************
void loop()
{
  //Poll the Buttons
  //Button 1
  if (PollButton(button1Pin, button1BounceState, button1PreviousState, previousButton1Micros))
  {
    Serial.println("Button 1 Pressed");
    ButtonIndicator();
    LearningMode();
  }

  //Button 2
  if (PollButton(button2Pin, button2BounceState, button2PreviousState, previousButton2Micros))
  {
    ButtonIndicator();
    Serial.println("Button 2 Pressed");
    SendGearboxData(StartStopEngine);
  }

  //Up Paddle
  if (PollButton(upPaddlePin, upPaddleBounceState, upPaddlePreviousState, previousUpPaddleMicros))
  {
    Serial.println("*****Upshift*****");
    incomingCurrentGear++;
    SendGearboxData(Upshift);
  }

  //Down Paddle
  if (PollButton(downPaddlePin, downPaddleBounceState, downPaddlePreviousState, previousDownPaddleMicros))
  {
    Serial.println("*****DownShift*****");
    incomingCurrentGear--;
    SendGearboxData(Downshift);
  }

  ProcessIncomingGearboxData();
}

void ProcessIncomingGearboxData()
{
  if (micros() - previousProcessMicros > processInterval)
  {
    UpdateShiftIndicatorBasedonRPM(incomingLayRPM);
    GearIndicator(incomingCurrentGear);
  }
}

//**********************************************************************
// Learning Mode
//**********************************************************************
void LearningMode()
{
  //Tell the Gearbox we are beginning learning
  SendGearboxData(BeginLearnMode);
  //Turn on Decimal Point to indicator learning mode is active
  sevsegRight.analogWrite(15, 255);

  GearIndicator(9);
  delay(1000);
  GearIndicator(0);

  int currentGear = 0;
  while (currentGear <= 7)
  {
    //If Button 1 is pressed, store the current value for gear x
    if (PollButton(button1Pin, button1BounceState, button1PreviousState, previousButton1Micros))
    {
      message.currentGear = currentGear;
      SendGearboxData(StoreValues);
      currentGear++;
      GearIndicator(currentGear);
      ButtonIndicator();
      //TODO: Update the gear number being DISPLAYED
    }
  }
  SendGearboxData(EndLearnMode);
  ButtonIndicator();
  ButtonIndicator();
  ButtonIndicator();
  //Turn off Decimal point
  sevsegRight.analogWrite(15, 255);
}

//**********************************************************************
// Send gearbox data from remote and actions
//**********************************************************************
void SendGearboxData(int action)
{
  Serial.print("Outbound Action: ");
  Serial.println(action);
  message.action = action;
  message.percentThrottle = percentThrottle;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&message, sizeof(message));

  if (result == ESP_OK)
  {
    Serial.println("Sent with success");
  }
  else
  {
    Serial.println("Error sending the data");
  }
}

//**********************************************************************
// Poll the buttons to determine if any are pressed
//**********************************************************************
bool PollButton(int pin, bool &bounceState, int &previousButtonState, unsigned long &previousMicros)
{
  int currentButtonState = digitalRead(pin);

  bool buttonPressed = false;
  /*
  Serial.print("Previous Button State: ");
  Serial.println(previousButtonState);
  Serial.print("Current Button State: ");
  Serial.println(currentButtonState);
  */

  if (bounceState == WATCH_SENSOR)
  {
    if (previousButtonState != currentButtonState)
    {
      bounceState = IGNORE_SENSOR;
      previousMicros = micros();
    }
  }
  else
  {
    unsigned long timeElapsed = micros() - previousMicros;

    //Check if interval has expired and we've debounced long enough
    if (timeElapsed > buttonDebounceInterval)
    {
      if (currentButtonState == HIGH && previousButtonState != currentButtonState)
      {
        buttonPressed = true;
      }
      bounceState = WATCH_SENSOR;
      previousButtonState = currentButtonState;
    }
  }
  return buttonPressed;
}

//**********************************************************************
// Update the shift indicator based on an RPM
//**********************************************************************
void UpdateShiftIndicatorBasedonRPM(int rpm)
{
  if (rpm >= startRPM)
  {
    int shiftIndicatorState;
    //Take the shift indicators RPM range and divide it into 12 segements, one for each LED
    shiftIndicatorState = map(rpm, startRPM, maxRPM, 0, 12);
    ShiftIndicatorDisplay(shiftIndicatorState);
  }
  else
  {
    //Clear the shift indicator if the RPM isn't above the start point
    ShiftIndicatorDisplay(0);
  }
}

//**********************************************************************
// Turn on/off particular LEDs based on a state for the shift indicator
//**********************************************************************
void ShiftIndicatorDisplay(int state)
{
  for (int i = 0; i < 12; i++)
  {
    if (i < state)
    {
      //Turn LEDs sequentially
      digitalWrite(ledArray[i], LOW);
    }
    else
    {
      //turn off remain LEDs
      digitalWrite(ledArray[i], HIGH);
    }
  }
}

//**********************************************************************
// Update the gear indicator based on a gear value
//**********************************************************************
void GearIndicator(int gear)
{
  int SEGMENT_ON = 255;
  int SEGMENT_OFF = 0;

  switch (gear)
  {
  case 0:
    sevsegRight.analogWrite(15, SEGMENT_ON); //A
    sevsegRight.analogWrite(14, SEGMENT_ON); //B
    sevsegRight.analogWrite(6, SEGMENT_ON);  //C
    sevsegLeft.analogWrite(7, SEGMENT_ON);   //D
    sevsegLeft.analogWrite(6, SEGMENT_ON);   //E
    sevsegLeft.analogWrite(14, SEGMENT_ON);  //F
    sevsegLeft.analogWrite(15, SEGMENT_OFF); //G
    break;

  case 1:
    sevsegRight.analogWrite(15, SEGMENT_OFF);
    sevsegRight.analogWrite(14, SEGMENT_ON);
    sevsegRight.analogWrite(6, SEGMENT_ON);
    sevsegLeft.analogWrite(7, SEGMENT_OFF);
    sevsegLeft.analogWrite(6, SEGMENT_OFF);
    sevsegLeft.analogWrite(14, SEGMENT_OFF);
    sevsegLeft.analogWrite(15, SEGMENT_OFF);
    break;

  case 2:
    sevsegRight.analogWrite(15, SEGMENT_ON);
    sevsegRight.analogWrite(14, SEGMENT_ON);
    sevsegRight.analogWrite(6, SEGMENT_OFF);
    sevsegLeft.analogWrite(7, SEGMENT_ON);
    sevsegLeft.analogWrite(6, SEGMENT_ON);
    sevsegLeft.analogWrite(14, SEGMENT_OFF);
    sevsegLeft.analogWrite(15, SEGMENT_ON);
    break;

  case 3:
    sevsegRight.analogWrite(15, SEGMENT_ON);
    sevsegRight.analogWrite(14, SEGMENT_ON);
    sevsegRight.analogWrite(6, SEGMENT_ON);
    sevsegLeft.analogWrite(7, SEGMENT_ON);
    sevsegLeft.analogWrite(6, SEGMENT_OFF);
    sevsegLeft.analogWrite(14, SEGMENT_OFF);
    sevsegLeft.analogWrite(15, SEGMENT_ON);
    break;

  case 4:
    sevsegRight.analogWrite(15, SEGMENT_OFF);
    sevsegRight.analogWrite(14, SEGMENT_ON);
    sevsegRight.analogWrite(6, SEGMENT_ON);
    sevsegLeft.analogWrite(7, SEGMENT_OFF);
    sevsegLeft.analogWrite(6, SEGMENT_OFF);
    sevsegLeft.analogWrite(14, SEGMENT_ON);
    sevsegLeft.analogWrite(15, SEGMENT_ON);
    break;

  case 5:
    sevsegRight.analogWrite(15, SEGMENT_ON);
    sevsegRight.analogWrite(14, SEGMENT_OFF);
    sevsegRight.analogWrite(6, SEGMENT_ON);
    sevsegLeft.analogWrite(7, SEGMENT_ON);
    sevsegLeft.analogWrite(6, SEGMENT_OFF);
    sevsegLeft.analogWrite(14, SEGMENT_ON);
    sevsegLeft.analogWrite(15, SEGMENT_ON);
    break;

  case 6:
    sevsegRight.analogWrite(15, SEGMENT_ON);
    sevsegRight.analogWrite(14, SEGMENT_OFF);
    sevsegRight.analogWrite(6, SEGMENT_ON);
    sevsegLeft.analogWrite(7, SEGMENT_ON);
    sevsegLeft.analogWrite(6, SEGMENT_ON);
    sevsegLeft.analogWrite(14, SEGMENT_ON);
    sevsegLeft.analogWrite(15, SEGMENT_ON);
    break;

  case 7:
    sevsegRight.analogWrite(15, SEGMENT_ON);
    sevsegRight.analogWrite(14, SEGMENT_ON);
    sevsegRight.analogWrite(6, SEGMENT_ON);
    sevsegLeft.analogWrite(7, SEGMENT_OFF);
    sevsegLeft.analogWrite(6, SEGMENT_OFF);
    sevsegLeft.analogWrite(14, SEGMENT_OFF);
    sevsegLeft.analogWrite(15, SEGMENT_OFF);
    break;

  case 8:
    //Reverse
    break;

  case 9:
    //Learning "L"
    sevsegRight.analogWrite(15, SEGMENT_OFF);
    sevsegRight.analogWrite(14, SEGMENT_OFF);
    sevsegRight.analogWrite(6, SEGMENT_OFF);
    sevsegLeft.analogWrite(7, SEGMENT_ON);
    sevsegLeft.analogWrite(6, SEGMENT_ON);
    sevsegLeft.analogWrite(14, SEGMENT_ON);
    sevsegLeft.analogWrite(15, SEGMENT_OFF);
    break;

  case 10:
    //Extra
    break;
  }

  sevsegRight.analogWrite(7, SEGMENT_OFF); //DP
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
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}
//**********************************************************************
//  Flash Display when a button is pressed
//**********************************************************************
void ButtonIndicator()
{
  ShiftIndicatorDisplay(4);
  delay(100);
  ShiftIndicatorDisplay(0);
  delay(100);
}

void Diagnostics()
{
  //Test Shiftlights
  for (int i = 0; i < 13; i++)
  {
    ShiftIndicatorDisplay(i);
    delay(100);
  }
  ShiftIndicatorDisplay(0);

  //Test Gear Indicator
  for (int i = 0; i < 10; i++)
  {
    GearIndicator(i);
    delay(100);
  }
  Serial.println("Diagnostics Complete");
}