//Some examples for the AM-2916 5V, Addressable LED strips http://www.andymark.com/product-p/am-2916.htm based on the new WS2812b chipset
//We ran this demo off of our AM-2287 Arduino Ethernet http://www.andymark.com/product-p/am-2287.htm
//http://arduino.cc/en/Main/ArduinoBoardEthernet
//For convenience, everything you need can be purchased in one kit here http://www.andymark.com/product-p/am-3010.htm

//The FastLED library we use here supports multiple chipsets
//This code requires that the fastspi library be put in your arduino\libraries folder
//Arduino info on how to install software libraries http://arduino.cc/en/Guide/Libraries
//AndyMark, Inc.
//CSK 12/3/2013, 3/17/2014, 3/20/2014, 2/12/2016, 7/1/2016


//***NOTE: This strip runs off of 5V MAX!!!. Applying much more than 5V will damage/destroy you LED strip!***
//***Handling note: Don't mess with the wiring while the power is on. This can cause voltage spikes ***
//***or sneak ground paths that can damage the LED strip ***

//DO NOT try to power the whole strip (150 LEDs) off the arduino 5v regulator.  
//Use the AM-3068 10-30Vin to 5V 10A out stepdown converter http://www.andymark.com/product-p/am-3068.htm
//At full bright white, the strip can draw 4.5Amps or so. 
//This would overheat or burnout the arduino regulator if you tried to drive it from the arduino only
//The BLACK wire is ground, RED is +5V, WHITE is data
//Make sure you connect the BLACK ground from the LED strip to the Arduino ground.
//Communications to the LEDs requires a common ground to work.

//If you are using the AndyMark AM-2297 Arduino Ethernet board then make sure 
//you select Tools>Board>Arduino Ethernet from the Arduino IDE menu
//If you are new to working with Arduino a good place to start is here http://arduino.cc/en/Guide/HomePage
//Another new training resource provided by a 3rd party is here: http://www.arduinoclassroom.com/index.php/arduino-101



//CSK 3/17/2013 Libraries new location
//https://github.com/FastLED/FastLED
//https://github.com/FastLED/FastLED/wiki/Overview

#include "FastLED.h"
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#define COLOR_ORDER       GRB
#define MAX_BRIGHTNESS    255
//Tell it how many leds are in the strip. AndyMark's 2.5 meter strip has 150 leds
#define NUM_LEDS          150

// This is an array of leds. One item for each led in your strip
CRGB leds[NUM_LEDS];

//CSK 3/17/2014 I moved this to a pin that doesn't conflict with Ethernet functions in case you want to control LEDs via Ethernet
#define DATA_PIN          6 //White wire from the http://www.andymark.com/product-p/am-2917.htm power connector

//This function is used to setup things like pins, Serial ports etc.
//Here we specify which chipset our LEDs run off of by our choice of config function

int sensorPin = DAC1;    // select the input pin for the potentiometer
int sensorinPin = A1;    // select the input pin for the potentiometer


void setup()
{


Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
   // Uncomment one of the following lines for your leds arrangement.
   // FastLED.addLeds<TM1803, DATA_PIN, RGB>(leds, NUM_LEDS);
   // FastLED.addLeds<TM1804, DATA_PIN, RGB>(leds, NUM_LEDS);
   // FastLED.addLeds<TM1809, DATA_PIN, RGB>(leds, NUM_LEDS);
   //FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, NUM_LEDS);
   // FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);
   //CSK 2/12/2016 This is the correct chipset for the am-2916 LED strip
   FastLED.addLeds<WS2812B, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
   // FastLED.addLeds<UCS1903, DATA_PIN, RGB>(leds, NUM_LEDS);

   //FastLED.addLeds<WS2801, RGB>(leds, NUM_LEDS);

   // FastLED.addLeds<SM16716, RGB>(leds, NUM_LEDS);
   // FastLED.addLeds<LPD8806, RGB>(leds, NUM_LEDS);

   //***This is the chipset in the AM-2640 LED strip***
   //CSK 3/17/2013 Changed to this function to allow direct data and clock pin specification
   //FastLED.addLeds<WS2801, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);

   // FastLED.addLeds<SM16716, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
   // FastLED.addLeds<LPD8806, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
   FastLED.clear();
   FastLED.show();
   //delay(250);
   //clear() turns all LEDs off
   FastLED.clear();
   FastLED.setBrightness(MAX_BRIGHTNESS);
   fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 125, 125, 125) );
   FastLED.show();
   // start serial port at 9600 bps:
   //Serial.begin(9600);
}

void loop()
{
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  
  int dist = 4095;
  
  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    dist = measure.RangeMilliMeter;
    Serial.print("Distance (mm): "); Serial.println(dist);
  } else {
  
    Serial.println(" out of range ");
  }
  double analogval = 4095/1000 * dist;
  if (analogval>4095){
    analogval = 4095;
  }
  analogWriteResolution(12);
  analogWrite(sensorPin, analogval);

  
   //This is kind of Arduino's equivalent to Main() in a standard C program
   //This, as the name implies, loops endlessly.
   //https://code.google.com/p/fastspi/wiki/CRGBreference
   	//FastLED.clear();
   	//FastLED.show();
   //	delay(10);
   //CSK 3/20/2014 I added a rainbow function just for grins
  double LedSensor = analogRead(sensorinPin);
  
Serial.print("Voltage: ");Serial.println(LedSensor);
  if(LedSensor > 500*1023/3300 && LedSensor < 600*1023/3300){
    solidcolor(CRGB::Orange);
    FastLED.show();
  }
  else if(LedSensor > 1000*1023/3300 && LedSensor < 1200*1023/3300){
    solidcolor(CRGB::Red);
    FastLED.show();
  }   
  else if(LedSensor > 1550*1023/3300 && LedSensor < 1750*1023/3300){
    solidcolor(CRGB::White);
    FastLED.show();
  }    
  else if(LedSensor > 2100*1023/3300 && LedSensor < 2300*1023/3300){
    solidcolor(CRGB::Blue);
    FastLED.show();
  }    
  else if(LedSensor > 2650*1023/3300 && LedSensor < 2850*1023/3300){
    solidcolor(CRGB::Yellow);
    FastLED.show();
  }     
  else{
    solidcolor(CRGB::Black);
    FastLED.show();
  }
  delay(10);

    
}

void solidcolor(CRGB color){
  FastLED.setBrightness(10);
  
   
      // Make it look like one LED is moving in one direction
      for(int led_number = 0; led_number < NUM_LEDS; led_number++)
      {
         //Apply the color that was passed into the function
         leds[led_number] = color;
         //Actually turn on the LED we just set
         
      }

   
   return;
}
