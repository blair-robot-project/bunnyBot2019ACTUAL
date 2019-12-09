#include <Wire.h>
#include "Adafruit_TCS34725.h"

// for a common anode LED, connect the common pin to +5V
// for common cathode, connect the common to ground

// set to false if using a common cathode LED
#define commonAnode true

uint8_t redCached, greenCached, blueCached;

const byte DEVICE_ADDRESS = 4;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  Serial.begin(9600);
  Serial.println("Color View Test!");
  
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
  
  Wire.begin(DEVICE_ADDRESS); //Initialize communication with roboRIO
  Wire.onRequest(requestEvent);
}


void loop() {
  uint16_t clear, red, green, blue;

  tcs.setInterrupt(false);      // turn on LED

  delay(60);  // takes 50ms to read 
  
  tcs.getRawData(&red, &green, &blue, &clear);

  tcs.setInterrupt(true);  // turn off LED
  
  Serial.print("C:\t"); Serial.print(clear);
  Serial.print("\tR:\t"); Serial.print(red);
  Serial.print("\tG:\t"); Serial.print(green);
  Serial.print("\tB:\t"); Serial.print(blue);

  // Figure out some basic hex code for visualization
  uint32_t sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;
  Serial.print("\t");
  Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);
  Serial.println();

  Serial.print("Red: ");
  Serial.println(r);
  Serial.print("Green: ");
  Serial.println(g);
  Serial.print("Blue: ");
  Serial.println(b);
  Serial.println();
  
  //Serial.print((int)r ); Serial.print(" "); Serial.print((int)g);Serial.print(" ");  Serial.println((int)b );

  redCached = min((uint8_t) r,255);
  greenCached = min((uint8_t) g,255);
  blueCached = min((uint8_t) b,255);
}

void requestEvent()
{
  uint8_t rgb[3];
  rgb[0] = redCached;
  rgb[1] = greenCached;
  rgb[2] = blueCached;
  Wire.write(rgb, 3);
}
