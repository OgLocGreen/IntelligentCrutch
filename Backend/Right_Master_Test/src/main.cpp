#include <Arduino.h>

#include "BluetoothSerial.h"
#include <EEPROM.h>
#include <Wire.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_NAU7802
#include <WiFi.h>
#include <esp_now.h>

#define EEPROM_SIZE 256
#define FILTER_SIZE 1       // 1: filter is off >1: length of filter array
#define LOCATION_CALIBRATION_FACTOR 0
#define LOCATION_ZERO_OFFSET 10
#define LOCATION_MAXWEIGHT 20
#define LOCATION_PATIENTWEIGHT 30
#define LED_PIN 2
#define BEEPER_PIN 16
#define LOOP_FREQUENCY 20   // in Hz

String input;
int state = 0;
unsigned long lastloop = 0;
unsigned long lastmsg = 0;

int weight = 0;         // in grams
int totalweight = 0;    // in grams
int footload = 0;   
long weightFilter[FILTER_SIZE] = { 0 };
int fCount = 0;
float weightSum = 0.0;  // used for moving average filter
bool spam = 1;
float maxweight = 0;
float patientweight = 0;
bool receiveflag = false;

float old_footload = 0;
float numb_steps = 0;
float numb_overload =0;
float overload = 0;
int low_threshold = 5;
bool up = false;
bool down = false;

bool step = false;
bool overload_flag = false;

float overloadArray[10];
int arrayIndex = 0;

int incomingReadings = 0;
int weightSlave = 0;        
//uint8_t broadcastAddress[] = {0x24, 0x0A, 0xC4, 0x5F, 0xD8, 0x8C};  // MAC Adress of crutch Nr. 1
uint8_t broadcastAddress[] = {0x8C, 0xAA, 0xB5, 0x8A, 0xFC, 0x1C};  // MAC Adress of crutch Nr. 2

BluetoothSerial btSerial;		           // Bluetooth
NAU7802 myScale; //Create instance of the NAU7802 class

void datareceived();
void displaydata();
void setupScale(void);
bool updateAvgWeight();
void smartdelay();

void stepCounting();
void overloadsCounting();
void overloadStrength();

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

  btSerial.begin("iUAGS");

  lastloop = millis();
}

void loop() {
    //Serial.print("Raw reading: ");
    //Serial.println(myScale.getReading());

    updateAvgWeight();
    //Serial.print("   Filtered Weight: ");
    //Serial.println(weight);

    if (spam) {
        /*btSerial.print("\n\nWeight: ");
        btSerial.print(weight);
        btSerial.print("   Slave: ");
        btSerial.print(weightSlave);*/
        
        if(receiveflag)
        {
            receiveflag = false;
            //Serial.print("\nMessage delay: ");
            //Serial.println(millis()-lastmsg);
            lastmsg = millis();
            totalweight = weight + weightSlave;
            //btSerial.print("\n\nWeightsum: ");
            //btSerial.print(totalweight / 1000.0);
            btSerial.print("\n\nFootload: ");
            if (totalweight > 2000) {
                footload = patientweight - totalweight;
            }
            else {
                footload = 0.0;
            }
            btSerial.print(footload / 1000.0);
            // Send footload via ESP-NOW
            esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &footload, sizeof(footload));
        }
    }

    // signal overload
    if (footload > maxweight)
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
    
    //smartdelay();

    stepCounting();
    overloadsCounting();
    overloadStrength();
    old_footload = footload;
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
            btSerial.print("patweight - set patient weight");
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
        if (input == String("patweight"))
        {
            btSerial.print("\nSet patweight [g]: ");
            state = 4;
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
    case 4:     // set patweight
        patientweight = input.toFloat();
        EEPROM.put(LOCATION_PATIENTWEIGHT, input.toFloat());
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
    btSerial.print("\nWeight Master: ");
    btSerial.print(weight);
    btSerial.print("\nWeight Slave: ");
    btSerial.print(weightSlave);
    btSerial.print("\nWeight Factor: ");
    btSerial.print(myScale.getCalibrationFactor());
    btSerial.print("\nWeight Offset: ");
    btSerial.print(myScale.getZeroOffset());
    btSerial.print("\nMaxweight: ");
    btSerial.print(maxweight);
    btSerial.print("\nPatientweight: ");
    btSerial.print(patientweight);
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

    //load patientweight
    EEPROM.get(LOCATION_PATIENTWEIGHT, patientweight);
    
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
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
    //Serial.print("Bytes received: ");
    //Serial.println(len);
    weightSlave = incomingReadings;
    receiveflag = true;
}

void smartdelay()
{
	int delaytime = 0;
	delaytime = 1000 / LOOP_FREQUENCY - (millis() - LOOP_FREQUENCY);
	if (delaytime < 0) delaytime = 0;
	if (delaytime > (1000 / LOOP_FREQUENCY)) delaytime = (1000 / LOOP_FREQUENCY);
    //Serial.print("\nloop period: ");
    //Serial.println(millis()-lastloop);
	delay(delaytime);
	lastloop = millis();
}

void stepCounting()
{
    if(old_footload > footload)
    {
        down = true;
        up = false;
    }
    if(old_footload < footload)
    {
        up = true;
        down = false;
    }

    if(!step && down && footload < low_threshold)
    {
        numb_steps++;
        step = true;
        overloadArray[arrayIndex] = overload;
        arrayIndex++;
        overload = 0;
        overload_flag = false;
    }

    if(step && up && footload > low_threshold )
    {
        step = false;
    }

}

void overloadsCounting()
{
    if (!overload_flag && !step && up && footload > maxweight)
    {
        overload_flag = true;
        numb_overload++;
    }
}

void overloadStrength()
{
    if(!step && old_footload < footload)
    {
        if (overload < footload - maxweight);
        overload = footload - maxweight;
    }

}