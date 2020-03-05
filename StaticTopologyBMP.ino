#include <Sleep_n0m1.h>

/*
 Copyright (C) 2012 James Coliz, Jr. <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 
 Update 2014 - TMRh20
 */

/**
 * Simplest possible example of using RF24Network 
 *
 * TRANSMITTER NODE
 * Every 2 seconds, send a payload to the receiver node.
 */
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; // I2C Interface
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include<Wire.h>
#include <VoltageReference.h>
#include <Sleep_n0m1.h>
#include <EEPROM.h>
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define VREFCAL 1102150L
#define NODEID 35
#define SLEEPTIME_MS
VoltageReference vref;
Sleep sleep;
int measurementcounter=0;
unsigned long sleeptime=300000;
#define NumMeasurements 1

RF24 radio(10,9);                    // nRF24L01(+) radio attached using Getting Started board 

RF24Network network(radio);          // Network uses that radio

const uint16_t this_node = 031;        // Address of our node in Octal format
const uint16_t other_node = 00;       // Address of the other node in Octal format

const unsigned long interval = 10000; //ms  // How often to send 'hello world to the other unit

unsigned long last_sent;             // When did we last send?
unsigned long packets_sent;          // How many have we sent already


struct payload_t {                  // Structure of our payload
  unsigned long ms;
  unsigned long counter;
};

struct measurement{
    int16_t nodeID;
    int16_t tempc;
    int16_t humidity;
    int16_t vbatt;
    uint32_t timestamp;
  } message[NumMeasurements]={NULL};
void setup(void)
{
  Serial.begin(115200);

    if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  
  Serial.println("RF24Network/examples/helloworld_tx/");
  vref.begin(VREFCAL);
  SPI.begin();
  radio.begin();
  network.begin(/*channel*/ 90, /*node address*/ this_node);
}


uint16_t readBattery()
{
  uint16_t mv = 0;

  mv = vref.readVcc();
  //Serial.print(F("Vcc (mV): "));
  //Serial.println(mv);
  return mv;
}


void PrintData(void){ //Display message data for all recorded messages
  for(int k=0;k<=NumMeasurements-1;k++){
      Serial.print("***MEASUREMENT NUMBER ");
      Serial.print(k+1);
      Serial.println(" ***");
      Serial.print("ID: "); Serial.println(message[k].nodeID);
      Serial.print("TEMP: "); Serial.println(message[k].tempc);
      Serial.print("HUMIDITY: "); Serial.println(message[k].humidity);
      Serial.print("VOLTAGE: "); Serial.println(message[k].vbatt);
      Serial.print("TIMESTAMP: "); Serial.println(message[k].timestamp);
      Serial.println("****************************");
    }
}





void loop() {
  
  network.update();                          // Check the network regularly
  //GetData();

  
  unsigned long now = millis();              // If it's time to send a message, send it!
  //if (now - last_sent >= interval)
  {
    last_sent = now;

    Serial.println(bmp.readTemperature());
    
    
    Serial.print("Sending...");
    measurement payload = { this_node, 100*bmp.readTemperature(), 0, readBattery(), millis() };
    RF24NetworkHeader header(/*to node*/ other_node);
    bool ok = network.write(header,&payload,sizeof(payload));
    if (ok){
      Serial.println("ok.");
      //PrintData();
    }
    else
      Serial.println("failed.");
  }

  
  radio.powerDown();
  Serial.println("Good Night!");
  sleep.sleepDelay(sleeptime);
  Serial.println("Good Morning!");
  radio.powerUp();
  delay(5);
  

  
  
}


