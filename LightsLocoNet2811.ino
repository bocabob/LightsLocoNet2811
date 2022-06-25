// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       LightsLocoNet.ino
    Created:    5/29/2022 
    Latest revision: 5/29/2022
    Author:     Bob Gamble and others from posted code examples

This sketch uses:
1) the LocoNet, Wire, I2CKeyPad and EEPROM libraries;
2) a LocoNet Shield (John Plocher's design or similar)
3) I2C PCF8574 IO Expansion Board Module and 4x4 Keypad
4) NeoPixel type of addressable LEDs

Pin usage:
The LocoNet Shield uses pins 7 & 8
The I2C interface uses D2 for interupt, A4 for SDA & A5 for SCL & Ground & +5V
The LEDs use pins as defined below

Improvements:
1) save states in EEPROM 
2) report feedback on LocoNet
3) keypad input for some functions 

*/
#define DEBUG_PRINT true

#include <Wire.h>
#include <LocoNet.h>
#include<EEPROM.h>
#include<I2CKeyPad.h>
#include <NeoPixelBus.h>

// NeoPixel defines

#define i_max_pixel 5
// modify as desired
const uint16_t PixelCount = i_max_pixel; 
const uint8_t PixelPin = 12;  // make sure to set this to the correct pin, ignored for Esp8266

// four element pixels, RGBW
//NeoPixelBus<NeoGrbwFeature, NeoWs2812xMethod > strip(PixelCount, PixelPin);

NeoPixelBus<NeoGrbFeature, NeoWs2811Method > strip(PixelCount, PixelPin);

#define colorSaturation 5   // brightness

RgbColor red(colorSaturation, 0, 0);
RgbColor green(0, colorSaturation, 0);
RgbColor blue(0, 0, colorSaturation);
RgbColor white(colorSaturation);
RgbColor black(0,0,0);

// Pin for interupt from I2C module
#define KeyPad_Int_Pin 2

// Define your LocoNet TX Pin below, RX is on 8
#define  TX_PIN   7

// below are for lights driven directly from pins, not NeoPixels
#define NumOfLights 2
#define Light_A 11
#define Light_B 12
  int LightPin[NumOfLights] {Light_A, Light_B};
  int LightAddr[NumOfLights] {700, 701};

int setupPin=A2;

static   lnMsg        *LnPacket;

const uint8_t KEYPAD_ADDRESS = 0x20;
I2CKeyPad keyPad(KEYPAD_ADDRESS);
char keymap[19] = "123A456B789C*0#DNF";  // N = NoKey, F = Fail (e.g. >1 keys pressed)

// two different layout styles of a nummeric keyPad
//char phone_layout[19]      = "123A456B789C*0#DNF";  // N = NoKey, F = Fail
//char calculator_layout[19] = "789A456B123C*0#DNF";  // N = NoKey, F = Fail

// volatile for IRQ var
volatile bool keyChange = false;

#define buffLen 10
  char buff[buffLen];
  uint8_t bufferIndex = 0;
  
//uint8_t lastDirection = 0xFF;


typedef struct
{
	int address;
	bool on;
	RgbColor color;
}
DCCAccessoryAddress;
DCCAccessoryAddress npLight[i_max_pixel];

void ConfigurePixels()
// memory for the Status of servo:  
//			0 = deviate position
//      1 = correct position
//      2 = Status on start sketch

{	// these are arrays for each pixel
	npLight[0].address = 800;	// DCC address for this servo
	npLight[0].on = false;	// light on flag
	npLight[0].color = white;		// color

	npLight[1].address = 801;
	npLight[1].on = false;
	npLight[1].color = white;

	npLight[2].address = 802;
	npLight[2].on = false;
	npLight[2].color = white;

	npLight[3].address = 803;
	npLight[3].on = false;
	npLight[3].color = white;

	npLight[4].address = 804;
	npLight[4].on = false;
	npLight[4].color = white;
//
//	npLight[5].address = 805;
//	npLight[5].on = false;
//	npLight[5].color = white;
//
//	npLight[6].address = 806;
//	npLight[6].on = false;
//	npLight[6].color = white;
//
//	npLight[7].address = 807;
//	npLight[7].on = false;
//	npLight[7].color = white;
//
//	npLight[8].address = 808;
//	npLight[8].on = false;
//	npLight[8].color = white;
//
//	npLight[9].address = 809;
//	npLight[9].on = false;
//	npLight[9].color = white;

/*	repeat the above construct for any additional lights implemented 	*/
}

void setupLocoNet()
{
	Serial.println(F("Setting up LocoNet node..."));

    // First initialize the LocoNet interface, specifying the TX Pin
  LocoNet.init(TX_PIN);
}

void setup()
{	
	Serial.begin(115200);
	while (!Serial);   // Wait for the USB Device to Enumerate

	Serial.println(F("Start - Accessory Lights Ready"));
  
    // this resets all the neopixels to an off state
    strip.Begin();
    strip.Show();

  for (int Light=0;Light<NumOfLights;Light++){
    pinMode(LightPin[Light], OUTPUT);
	  digitalWrite(LightPin[Light], LOW); // turn off
  }
 
  // NOTE: PCF8574 will generate an interrupt on key press and release.
  pinMode(KeyPad_Int_Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(KeyPad_Int_Pin), keyChanged, FALLING);
  keyChange = false;
  
  Wire.begin();
  Wire.setClock(100000);
  if (keyPad.begin() == false)
  {
    Serial.println("\nERROR: cannot communicate to keypad.\nPlease reboot.\n");
    while (1);
  }
  keyPad.loadKeyMap(keymap);
  
  ConfigurePixels();
  
	setupLocoNet();	
  
  if(analogRead(setupPin)>500){
    // populate default values in EEPROM
    Serial.print(F("Writing EEPROM with defaults "));
    for (int i = 0; i < (sizeof(npLight) / sizeof(DCCAccessoryAddress)); i++)
    {   
      EEPROM.put(1+i,(npLight[i].on));
    }
  }  
  else
  {
    // read values from EEPROM
    Serial.println(F("Reading EEPROM"));
    for (int i = 0; i < (sizeof(npLight) / sizeof(DCCAccessoryAddress)); i++)
    {   
      EEPROM.get((1+i),npLight[i].on);
      Serial.print(i); 
      Serial.println(npLight[i].on);
        PixelSwitch(i,npLight[i].on);
    }
  }
  for (int i = 0; i < (sizeof(npLight) / sizeof(DCCAccessoryAddress)); i++)
    {   
      LN_STATUS lnStatus = LocoNet.reportSensor(npLight[i].address,npLight[i].on);
    }  
 
    Serial.println(F("Ready"));
}

void keyChanged()
{
  keyChange = true;
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch Request messages

void notifySwitchRequest( uint16_t Addr, uint8_t OutputPower, uint8_t Direction ) {

  uint8_t InDir = (Direction > 0);

// the following prints to the serial monitor for debugging purposes
#ifdef DEBUG_PRINT
  Serial.print("notifySwitchRequestOutput: ");
  Serial.print(Addr, DEC);
  Serial.print(',');
  Serial.print(Direction, DEC);
  Serial.print(',');
  Serial.print(OutputPower, DEC);
  Serial.print(',');
  Serial.print(" InDir = ");
  Serial.println(InDir, DEC);
#endif

  if ((Addr == 700)) // hard coded address for controlling LED lights from pin Light_A
  {
    LightSwitch(0,InDir);   
  }
  if ((Addr == 701)) // hard coded address for controlling LED lights from pin Light_B
  {
    LightSwitch(1,InDir); 
  }
  for (int i = 0; i < (sizeof(npLight) / sizeof(DCCAccessoryAddress)); i++)
  {   
    if ((Addr == npLight[i].address)  && (InDir != npLight[i].on) && (OutputPower != 0))
    {
        PixelSwitch(i,InDir);
        break;
    }
  }
}
void PixelSwitch(int i, uint8_t dir)
{     if (i > (i_max_pixel - 1)) return;
      Serial.print(F("Activating Pixel : "));
      Serial.println(i, DEC);      

//      npLight[i].on = true;

//        LN_STATUS lnStatus = LocoNet.reportSensor(npLight[i].address,(dir > 0));
        
      if (dir)
      {
        strip.SetPixelColor(i, npLight[i].color);
        strip.Show();
        npLight[i].on = true;
        Serial.print(F("On : "));
        Serial.println(npLight[i].address, DEC);
        LN_STATUS lnStatus = LocoNet.reportSensor(npLight[i].address,1);
      Serial.println(LocoNet.getStatusStr(lnStatus));
      }
      else
      {    
        strip.SetPixelColor(i, black);
        strip.Show();     
        npLight[i].on = false; 
        Serial.print(F("Off : "));
        Serial.println(npLight[i].address, DEC);
        LN_STATUS lnStatus = LocoNet.reportSensor(npLight[i].address,0);
      Serial.println(LocoNet.getStatusStr(lnStatus));
      }
      EEPROM.put(1+i,(npLight[i].on));
}  
void LightSwitch(int Light, int dir)
{ if (Light > (NumOfLights - 1)) return;
  if (dir)
    {
      digitalWrite(LightPin[Light], HIGH); // turn on
    }
    else
    {
      digitalWrite(LightPin[Light], LOW); // turn off
    }
      LN_STATUS lnStatus = LocoNet.reportSensor(LightAddr[Light], dir);
#ifdef DEBUG_PRINT
      Serial.print(F("Tx: Sensor: "));
      Serial.print(LightAddr[Light]);
      Serial.print(" Status: ");
      Serial.println(LocoNet.getStatusStr(lnStatus));
#endif
}    
void keyPadCommand()
{
#ifdef DEBUG_PRINT
    Serial.print(F("BufferIndex: "));
    Serial.println(bufferIndex);
#endif    
  if (bufferIndex <= 1)
  {
    switch (buff[0]) {
  case '#':
    //
    
    break;
  case '*':
    // 
    
    break;
  case 'A':
    //
    Actions(0);
    break;
  case 'B':
    // 
    Actions(1);
    break;
  case 'C':
    //
    Actions(2);
    break;
  case 'D':
    // 
    Actions(3);
    break;
  default:
    // statements
//    keyChange = false;
//    return;
    break;
    }
    keyChange = true;
    }
  if (bufferIndex == 2)
  {
    keyChange = false;
  switch (buff[1]) {
  case '#':
    // turn on pixel
    PixelSwitch(buff[0] - 48,1);
    break;
  case '*':
    // turn off pixel
    PixelSwitch(buff[0] - 48,0);
    break;
  default:
    // statements
    break;
    }
  }   
  if (bufferIndex == 3)
  {
    keyChange = false;
  switch (buff[2]) {  //ignore first character
    case '#':
      // turn on light #
      LightSwitch((buff[1] - 48),1); 
      break;
    case '*':
      // turn off light #
      LightSwitch((buff[1] - 48),0);
      break;
    default:
      // statements
      break;
    }    
  }  
  if (bufferIndex >= 4)
  {
    keyChange = false;
    Actions(buff[2] - 48);   //ignore first two characters 
  } 
#ifdef DEBUG_PRINT  
    Serial.print(F("Buffer: "));
    Serial.println(buff);
#endif    
      for (int i = 0; i < (sizeof(buff)); i++)
      {
        buff[i]   = 0;
      }
        buff[bufferIndex]   = 0;
    bufferIndex = 0;
//              keyChange = true;  
#ifdef DEBUG_PRINT  
    Serial.println(F("Done Command"));
#endif    
}

void Actions(int i)
    {
      
      switch(i){
          case 0:
            // light toggle
//              digitalWrite(Light_A, digitalRead(Light_A) ^ 1);
//  brighten pixels       
              Serial.println(F("Brighten all pixels"));
              for (int i = 0; i < (sizeof(npLight) / sizeof(DCCAccessoryAddress)); i++)
              {                 
                RgbColor tempcolor;
                tempcolor = strip.GetPixelColor(i);
                tempcolor.Lighten(1);
                strip.SetPixelColor(i,tempcolor);
              }
            strip.Show();
            break;
          case 1:
            // light toggle
//              digitalWrite(Light_B, digitalRead(Light_B) ^ 1);
//  darken pixels       
              Serial.println(F("Darken all pixels"));
              for (int i = 0; i < (sizeof(npLight) / sizeof(DCCAccessoryAddress)); i++)
              {                 
                RgbColor tempcolor;
                tempcolor = strip.GetPixelColor(i);
                tempcolor.Darken(1);
                strip.SetPixelColor(i,tempcolor);
              }
            strip.Show();
            break;
          case 2:
            // turn on all pixels        
              Serial.println(F("Turn on all pixels"));
              for (int i = 0; i < (sizeof(npLight) / sizeof(DCCAccessoryAddress)); i++)
              {                 
                PixelSwitch(i,1);
              }
            break;
          case 3:
            // turn off all pixels           
              Serial.println(F("Turn off all pixels"));
              for (int i = 0; i < (sizeof(npLight) / sizeof(DCCAccessoryAddress)); i++)
              {                 
                PixelSwitch(i,0);
              }
            break;
          default:
            //
            break;
          }
      }   
   
    
void loop()
/* this is the loop the code continuously executes after everything is initialized. 
When there is LocoNet traffic the code is interrupted to process the message and if there is a command to a servo that respective Status array is updated. 
The changes to the Status array will cause this code loop to move the servo as commanded */
{	
  
// Check LocoNet for pending switch commands:
  LnPacket=LocoNet.receive();
  if(LnPacket){
    LocoNet.processSwitchSensorMessage(LnPacket);
  }
  
if (keyChange)
  {
    uint8_t index = keyPad.getKey();
//        delay(50);
    if (index < 16)
    {
#ifdef DEBUG_PRINT      
      Serial.print("press: ");
      Serial.println(keymap[index]);
#endif      
      buff[bufferIndex++] = keymap[index];
      if (keymap[index] == '#' || keymap[index] == '*' || keymap[index] == 'A' || keymap[index] == 'B' || keymap[index] == 'C' || keymap[index] == 'D')
      {
        keyPadCommand();
      }      
      else
      {
      keyChange = false;  
      }    
    }  
    else
    {    
      keyChange = false;  
    }  
#ifdef DEBUG_PRINT  
      Serial.println(F("Done keychange loop"));
#endif    
  }
}
// The End.

//// This call-back function is called from LocoNet.processSwitchSensorMessage
//// for all Sensor messages
//void notifySensor( uint16_t Address, uint8_t State ) {
//  Serial.print("Sensor: ");
//  Serial.print(Address, DEC);
//  Serial.print(" - ");
//  Serial.println( State ? "Active" : "Inactive" );
//}
//
//
//// This call-back function is called from LocoNet.processSwitchSensorMessage
//// for all Switch Output Report messages
//void notifySwitchOutputsReport( uint16_t Address, uint8_t ClosedOutput, uint8_t ThrownOutput) {
//  Serial.print("Switch Outputs Report: ");
//  Serial.print(Address, DEC);
//  Serial.print(": Closed - ");
//  Serial.print(ClosedOutput ? "On" : "Off");
//  Serial.print(": Thrown - ");
//  Serial.println(ThrownOutput ? "On" : "Off");
//}
//
//// This call-back function is called from LocoNet.processSwitchSensorMessage
//// for all Switch Sensor Report messages
//void notifySwitchReport( uint16_t Address, uint8_t State, uint8_t Sensor ) {
//  Serial.print("Switch Sensor Report: ");
//  Serial.print(Address, DEC);
//  Serial.print(':');
//  Serial.print(Sensor ? "Switch" : "Aux");
//  Serial.print(" - ");
//  Serial.println( State ? "Active" : "Inactive" );
//}
//
//// This call-back function is called from LocoNet.processSwitchSensorMessage
//// for all Switch State messages
//void notifySwitchState( uint16_t Address, uint8_t Output, uint8_t Direction ) {
//  Serial.print("Switch State: ");
//  Serial.print(Address, DEC);
//  Serial.print(':');
//  Serial.print(Direction ? "Closed" : "Thrown");
//  Serial.print(" - ");
//  Serial.println(Output ? "On" : "Off");
//}
