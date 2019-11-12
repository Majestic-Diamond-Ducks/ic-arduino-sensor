#include <Wire.h>
#include <ArduinoJson.h>
#include <Ethernet.h>
#include "SCD30.h"
#include "sgp30.h"

#if defined(ARDUINO_ARCH_AVR)
  #pragma message("Defined architecture for ARDUINO_ARCH_AVR.")
  #define Serial Serial
#elif defined(ARDUINO_ARCH_SAM)
  #pragma message("Defined architecture for ARDUINO_ARCH_SAM.")
  #define Serial SerialUSB
#elif defined(ARDUINO_ARCH_SAMD)
  #pragma message("Defined architecture for ARDUINO_ARCH_SAMD.")  
  #define Serial SerialUSB
#elif defined(ARDUINO_ARCH_STM32F4)
  #pragma message("Defined architecture for ARDUINO_ARCH_STM32F4.")
  #define Serial SerialUSB
#else
  #pragma message("Not found any architecture.")
  #define Serial Serial
#endif

#define MEASUREMENT_DELAY_MILIS 1000

////////////////////////////
//CHANGE THESE PER ARDUINO//
////////////////////////////
#define CLIENT_NAME "TURING"
byte mac[] = { 0x2C, 0xF7, 0xF1, 0x08, 0x28, 0xC7 };  // MAC adress of the arduino
////////////////////////////

int DUST_PIN = 8;
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 2000;//sampe 2s ;
unsigned long nextTimeout = 0; 
unsigned long lowpulseoccupancy = 0;

float ratio = 0;
float concentration = 0;
float carbonReading = -1000;
float temperatureReading = -1000;
float humidityReading = -1000;
float dust = -1000;
float lumen = -1000;

byte server[] = { 158, 38, 101, 99 }; // IP adress of server

EthernetClient client;   // Ethernet client that can connect to a specified IP adress

const int LIGHT_SENSOR = 0;

void setup()
{
  starttime = millis();//get the current time
  Wire.begin();
  s16 err;
    u16 scaled_ethanol_signal, scaled_h2_signal;
    Serial.begin(115200);
    scd30.initialize();
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // no point in carrying on, so do nothing forevermore:
    for (;;)
      ;
  }
  scd30.setAutoSelfCalibration(1);
  Serial.println("MAC setup:");
  // print your local IP address:

  delay(500);
  Ethernet.begin(mac);
  Serial.println("Beginning Ethernet");
  Serial.println(Ethernet.localIP());

  delay(3000);

  Serial.print("connecting...");

  if (client.connect(server, 6789)) {
    Serial.println("connected");
  } else {
    Serial.println("connection failed");
  }
  
  pinMode(LIGHT_SENSOR, INPUT);
  pinMode(DUST_PIN,INPUT);
}
 
void loop()
{
  const size_t capacity = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(5);    // compute the required size
  DynamicJsonBuffer jsonBuffer(capacity);     // handles memory management and grows automatically
  JsonObject& root = jsonBuffer.createObject();   // create an object root¨

startTimer(10000);
  //while (client.connected())
  while (true) {

if(timerHasExpired()){
  startTimer(MEASUREMENT_DELAY_MILIS-1);

float result[3] = {0};
if(scd30.isAvailable()){
        scd30.getCarbonDioxideConcentration(result);                
    }

    if(result[0] != 0){
     carbonReading = result[0];
     temperatureReading = result[1];
     humidityReading = result[2];
    }

    float tempDust = readDustSensor();
    if(tempDust > 0 ) {
      dust = tempDust;
    }
    lumen = readLightSensor();

   
    root["T"] = temperatureReading;
    root["N"] = CLIENT_NAME;
    root["H"] = humidityReading;
    root["C"] = carbonReading;
    root["D"] = dust;
    root["L"] = lumen;

    // prints message of object we are sending in Serial monitor.
    String json;
    root.prettyPrintTo(json);
    Serial.println(json);

    client.println(json);
    client.println();
}
  }

  Serial.print("ok");
  Serial.println();
  Serial.println("disconnecting.");
  client.stop();
  for (;;)
    ;
    delay(1000);
}

int readLightSensor(){
    int sensorValue = analogRead(LIGHT_SENSOR); 
    //Serial.println("Approximated trend of the intensity of light is: ");
    //Serial.print(sensorValue);

    return sensorValue;
}

boolean timerHasExpired() {
  boolean hasExpired = false;
  if (millis() > nextTimeout){
    hasExpired = true;
    }
    else
    {
      hasExpired = false;
      }
      return hasExpired;
} 

void startTimer(unsigned long duration){
  nextTimeout = millis() + duration;
}

int readDustSensor(){
  int tempVal = -1000;
  duration = pulseIn(DUST_PIN, LOW);
  lowpulseoccupancy = lowpulseoccupancy+duration;
  if ((millis()-starttime) >= sampletime_ms) //if the sampel time = = 2s
  {
    ratio = lowpulseoccupancy/(sampletime_ms*10.0);  
    concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; 
    lowpulseoccupancy = 0;
    starttime = millis();
    if(concentration != 0.62){
      tempVal = concentration;
    }
  }
  return tempVal;
}
