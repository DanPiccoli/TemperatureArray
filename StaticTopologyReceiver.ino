/*
 Copyright (C) 2012 James Coliz, Jr. <maniacbug@ymail.com>
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 
 Update 2014 - TMRh20
 */
/**
 * Simplest possible example of using RF24Network,
 *
 * RECEIVER NODE
 * Listens for messages from the transmitter and prints them out.
 */
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#define NumMeasurements 1
uint32_t displayTimer = 0;
RF24 radio(10,9);                // nRF24L01(+) radio attached using Getting Started board 
RF24Network network(radio);      // Network uses that radio
const uint16_t this_node = 00;    // Address of our node in Octal format ( 04,031, etc)
const uint16_t other_node = 01;   // Address of the other node in Octal format
struct payload_t {                 // Structure of our payload
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
  Serial.println("RF24Network/examples/helloworld_rx/");
 
  SPI.begin();
  radio.begin();
  network.begin(/*channel*/ 90, /*node address*/ this_node);
}

void PrintData(measurement payload){ //Display message data for all recorded messages
  for(int k=0;k<=0;k++){
      Serial.print("***MEASUREMENT NUMBER ");
      Serial.print(k+1);
      Serial.println(" ***");
      Serial.print("ID: "); Serial.println(payload.nodeID);
      Serial.print("TEMP: "); Serial.println(payload.tempc);
      Serial.print("HUMIDITY: "); Serial.println(payload.humidity);
      Serial.print("VOLTAGE: "); Serial.println(payload.vbatt);
      Serial.print("TIMESTAMP: "); Serial.println(payload.timestamp);
      Serial.println("****************************");
    }
}


void loop(void){
  
  network.update();                  // Check the network regularly
  
  while ( network.available() ) {     // Is there anything ready for us?
    
    RF24NetworkHeader header;        // If so, grab it and print it out
    measurement payload;
    network.read(header,&payload, sizeof(payload));
    PrintData(payload);
    /*Serial.println("///////////////////////");
    Serial.print("Temp: ");
    Serial.println(payload.tempc);
    Serial.print("Node ID:");
    Serial.println(payload.nodeID);
    Serial.print("Humidity: ");
    Serial.println(payload.humidity);
    Serial.print("Timestamp: ");
    Serial.println(payload.timestamp);
    */


    
  }
}

