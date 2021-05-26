#include <Arduino.h>

#include "BluetoothSerial.h"
#include <EEPROM.h>
#include <Wire.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_NAU7802
#include <WiFi.h>
#include <esp_now.h>

#define EEPROM_SIZE 256
#define FILTER_SIZE 1
#define LOCATION_CALIBRATION_FACTOR 0
#define LOCATION_ZERO_OFFSET 10
#define LOCATION_MAXWEIGHT 20
#define LED_PIN 2
#define BEEPER_PIN 16

String input;
int state = 0;

int weight = 0.0;
long weightFilter[FILTER_SIZE] = { 0 };
int fCount = 0;
float weightSum = 0.0;
bool spam = 1;
float maxweight = 0;

int incomingReadings = 0;
uint8_t broadcastAddress[] = {0x24, 0x0A, 0xC4, 0x5F, 0xD8, 0x8C};  // MAC Adress of crutch Nr. 1
//uint8_t broadcastAddress[] = {0x8C, 0xAA, 0xB5, 0x8A, 0xFC, 0x1C}; // MAC Adress of crutch Nr. 2

BluetoothSerial btSerial;		           // Bluetooth
NAU7802 myScale; //Create instance of the NAU7802 class

void datareceived();
void displaydata();
void setupScale(void);
bool updateAvgWeight();

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

void setup() {
  Serial.begin(115200);
  Serial.println("iUAGS Qwiic Scale Test");
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
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
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);


  EEPROM.begin(EEPROM_SIZE);

  setupScale();    // Load zeroOffset and calibrationFactor from EEPROM

  pinMode(BEEPER_PIN, OUTPUT);
  digitalWrite(BEEPER_PIN, HIGH);     // high = beeper off
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);

  btSerial.begin("iUAGS2");
}

void loop() {
  Serial.print("Raw reading: ");
  Serial.println(myScale.getReading());

  updateAvgWeight();
  //Serial.print("   Filtered Weight: ");
  //Serial.println(weight);

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &weight, sizeof(weight));
  
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  // signal overload
  if (weight > maxweight)
  {
      digitalWrite(LED_PIN, HIGH);
      digitalWrite(BEEPER_PIN, LOW);
  }
  else
  {
      digitalWrite(LED_PIN, LOW);
      digitalWrite(BEEPER_PIN, HIGH);
  }

  if (btSerial.available()) datareceived();

  delay(50);
  if (spam) {
      btSerial.print("\n\nWeight: ");
      btSerial.print(weight);
  }
}

void datareceived()
{
    input = btSerial.readString();
    switch (state)
    {
    case 0:
        if (input == String("help")) {						// help
            btSerial.print("\n\nhelp - list commands\n");
            btSerial.print("wfactor - set lin scale factor\n");
            btSerial.print("woffset - set lin scale offset\n");
            btSerial.print("maxweight - set beeper threshold\n");
            btSerial.print("data - display data\n");
            btSerial.print("spam - stream weight\n");
        }
        if (input == String("data")) displaydata();

        if (input == String("wfactor"))
        {
            btSerial.print("\nSet wfactor: ");
            state = 1;
        }
        if (input == String("woffset"))
        {
            btSerial.print("\nSet woffset: ");
            state = 2;
        }
        if (input == String("maxweight"))
        {
            btSerial.print("\nSet maxweight [g]: ");
            state = 3;
        }
        if (input == String("spam"))
        {
            spam = !spam;
        }
        break;
    case 1:     // setWeightFactor
        myScale.setCalibrationFactor(input.toFloat());
        EEPROM.put(LOCATION_CALIBRATION_FACTOR, input.toFloat());
        EEPROM.commit();
        state = 0;
        displaydata();
        break;
    case 2:     // setWeightOffset
        myScale.setZeroOffset(long(input.toFloat()));
        EEPROM.put(LOCATION_ZERO_OFFSET, long(input.toFloat()));
        EEPROM.commit();
        state = 0;
        displaydata();
        break;
    case 3:     // set maxweight
        maxweight = input.toFloat();
        EEPROM.put(LOCATION_MAXWEIGHT, input.toFloat());
        EEPROM.commit();
        state = 0;
        displaydata();
        break;
    }
}

void displaydata()
{
    btSerial.print("\n\nRaw Weight: ");
    btSerial.print(myScale.getReading());
    btSerial.print("\n\nWeight: ");
    btSerial.print(weight);
    btSerial.print("\nWeight Factor: ");
    btSerial.print(myScale.getCalibrationFactor());
    btSerial.print("\nWeight Offset: ");
    btSerial.print(myScale.getZeroOffset());
    btSerial.print("\nMaxweight: ");
    btSerial.print(maxweight);
}

//Reads the current system settings from EEPROM
void setupScale(void)
{
    long weightOffset = 0;
    float weightFactor = 0.0;

    Wire.begin();

    if (myScale.begin() == false)
    {
        Serial.println("Scale not detected. Please check wiring. Freezing...");
        while (1);
    }
    Serial.println("Scale detected!");

    myScale.setGain(NAU7802_GAIN_8);    // Gain before A/D Converter

    //read from Flash
    EEPROM.get(LOCATION_CALIBRATION_FACTOR, weightFactor);  //Value used to convert the load cell reading to lbs or kg
    EEPROM.get(LOCATION_ZERO_OFFSET, weightOffset);         //Zero value that is found when scale is tared

    //Pass these values to the library
    myScale.setCalibrationFactor(weightFactor);
    myScale.setZeroOffset(weightOffset);

    //load maxweight
    EEPROM.get(LOCATION_MAXWEIGHT, maxweight);
}

bool updateAvgWeight()      // update moving Average and store value in weight
{
    // if (myScale.available() == true)
    {
        if (fCount >= (FILTER_SIZE)) fCount = 0;
        weightSum = weightSum - weightFilter[fCount];
        weightFilter[fCount] = myScale.getWeight();
        weightSum = weightSum + weightFilter[fCount];
        weight = weightSum / (FILTER_SIZE);
        fCount++;
        return true;
    }
    /*else
    {
        Serial.print("\nError reading Scale");
        return false;
    }*/
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    Serial.println("Delivery Success :)");
  }
  else{
    Serial.println("Delivery Fail :(");
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  //var_right = incomingReadings;
}
