#include "DHT.h"
#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include "NodeData.pb.h"
#include "pb_encode.h"
#include <VoltageReference.h>
#include <Sleep_n0m1.h>
#include <SPI.h>
#include <EEPROM.h>
//#include <printf.h>
#define DHTPIN 2 //Define pin DHT is connected to
#define DHTTYPE DHT11 //Define DHT11 as the sensor being used
/**** Configure the nrf24l01 CE and CS pins ****/
RF24 radio(10, 9); //Initialize radio
DHT dht(DHTPIN, DHTTYPE); //Initialize DHT sensor
RF24Network network(radio);
RF24Mesh mesh(radio, network);
/**
 * User Configuration: nodeID - A unique identifier for each radio. Allows addressing
 * to change dynamically with physical changes to the mesh.
 *
 * In this example, configuration takes place below, prior to uploading the sketch to the device
 * A unique value from 1-255 must be configured for each node.
 * This will be stored in EEPROM on AVR devices, so remains persistent between further uploads, loss of power, etc.
 *
 **/
#define NODEID 144
#define NumMeasurements 10
#define SLEEPTIME_MS 10
#define VREFCAL 1102150L
VoltageReference vref;
Sleep sleep;

int measurementcounter=0;
uint8_t buffer[32];
uint32_t displayTimer = 0;

struct payload_t {
  unsigned long ms;
  unsigned long counter;
};


struct measurement{
    int nodeID;
    int tempc;
    float humidity;
    int vbatt;
    unsigned long timestamp;
  } message[NumMeasurements]={NULL};


void setup() {
Serial.begin(115200);
  dht.begin();
  vref.begin(VREFCAL);
  //printf_begin();
  // Set the nodeID manually
  mesh.setNodeID(NODEID);
  //Connect to the mesh
  Serial.println(F("Connecting to the mesh..."));
  mesh.begin();
  }

uint16_t readBattery()
{
  uint16_t mv = 0;

  mv = vref.readVcc();
  Serial.print(F("Vcc (mV): "));
  Serial.println(mv);
  return mv;
}

void GetData(void)
{
  message[measurementcounter].nodeID = NODEID;
  message[measurementcounter].tempc = dht.readTemperature();
  if(message[measurementcounter].tempc == NULL){
    Serial.print("TEMPERATURE READ FAILED!");
    return;
  }
  message[measurementcounter].humidity = dht.readHumidity();
  if(message[measurementcounter].humidity == NULL){
    Serial.print("HUMIDITY READ FAILED!");
    return;
  }
  message[measurementcounter].vbatt = readBattery();
  message[measurementcounter].timestamp = millis();
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
  mesh.update();
  delay(1000);
  if(measurementcounter<=NumMeasurements){
      Serial.print("Getting measurement ");
      Serial.print(measurementcounter+1);
      Serial.print(" of ");
      Serial.println(NumMeasurements);
      GetData();
      measurementcounter++;
       
  }
  if(measurementcounter==NumMeasurements){
    mesh.update();
    measurementcounter=0;
    PrintData();


    if (!mesh.write(&message[0], 'P', sizeof(message))) {

      // If a write fails, check connectivity to the mesh network
      if ( ! mesh.checkConnection() ) {
        //refresh the network address
        Serial.println("NEED TO RENEW");
        mesh.renewAddress();
        Serial.println("RENEWED");
      } else {
        Serial.println("Send fail, Test OK");
      }
    } else {
      Serial.print("Send OK: "); Serial.println(displayTimer);
    }
   
    
    while (network.available()) {
    RF24NetworkHeader header;
    payload_t payload;
    network.read(header, &payload, sizeof(payload));
    Serial.print("Received packet #");
    Serial.print(payload.counter);
    Serial.print(" at ");
    Serial.println(payload.ms);
  }
    
  }
radio.powerDown();
Serial.println("Good Night!");
sleep.sleepDelay(SLEEPTIME_MS);
Serial.println("Good Morning!");
radio.powerUp();
delay(5);
}






