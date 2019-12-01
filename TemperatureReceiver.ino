 
 
 /** RF24Mesh_Example_Master.ino by TMRh20
  * 
  *
  * This example sketch shows how to manually configure a node via RF24Mesh as a master node, which
  * will receive all data from sensor nodes.
  *
  * The nodes can change physical or logical position in the network, and reconnect through different
  * routing nodes as required. The master node manages the address assignments for the individual nodes
  * in a manner similar to DHCP.
  *
  */
  
  
#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"
#include <SPI.h>
//Include eeprom.h for AVR (Uno, Nano) etc. except ATTiny
#include <EEPROM.h>

/***** Configure the chosen CE,CS pins *****/
RF24 radio(10,9);
RF24Network network(radio);
RF24Mesh mesh(radio,network);

uint32_t displayTimer = 0;

struct measurement{
    int nodeID;
    int tempc;
    float humidity;
    int vbatt;
    unsigned long timestamp;
  } message[10];

void PrintData(void){ //Display message data for all recorded messages
  for(int k=0;k<=9;k++){
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


void setup() {
  Serial.begin(115200);

  // Set the nodeID to 0 for the master node
  mesh.setNodeID(0);
  Serial.println(mesh.getNodeID());
  // Connect to the mesh
  mesh.begin();
}


void loop() {    

  // Call mesh.update to keep the network updated
  mesh.update();
  
  // In addition, keep the 'DHCP service' running on the master node so addresses will
  // be assigned to the sensor nodes
  mesh.DHCP();
  
  
  // Check for incoming data from the sensors
  if(network.available()){
    RF24NetworkHeader header;
    network.peek(header);
    Serial.println("MESSAGE RECEIVED!");
    uint32_t dat=0;
    switch(header.type){
      case 'P':
          network.read(header,&message, sizeof(message));
          PrintData();
          break;
      default:
          Serial.println("Unrecognized Data Type");
      break;  
    }
    
    
    }
  
  if(millis() - displayTimer > 5000){
    displayTimer = millis();
    Serial.println(" ");
    Serial.println(F("********Assigned Addresses********"));
     for(int i=0; i<mesh.addrListTop; i++){
       Serial.print("NodeID: ");
       Serial.print(mesh.addrList[i].nodeID);
       Serial.print(" RF24Network Address: 0");
       Serial.println(mesh.addrList[i].address,OCT);
     }
    Serial.println(F("**********************************"));
  }
}
