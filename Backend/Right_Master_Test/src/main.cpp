#include <Arduino.h>
#include "BluetoothSerial.h"
#include <EEPROM.h>
#include <Wire.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_NAU7802
#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>

#define EEPROM_SIZE 256
#define FILTER_SIZE 1       // 1: filter is off >1: length of filter array
#define LOCATION_MAXWEIGHT 0
#define LOCATION_PATIENTWEIGHT 1
#define LOCATION_REAL_WEIGHT 2
#define LOCATION_READING_VAL 100
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
long real_weight[15];
long raw_value[15];
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


void sendMeasurementDataOverBluetooth();
void sendAnaliticalDataOverBluetooth();
template <class T> int EEPROM_writeAnything(int ee, const T& value);
template <class T> int EEPROM_readAnything(int ee, T& value);
void getRealWeight();


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
    //getRealWeight();
    
    updateAvgWeight();
    /*

    if (spam) {
        // btSerial.print("\n\nWeight: ");
        // btSerial.print(weight);
        // btSerial.print("   Slave: ");
        // btSerial.print(weightSlave);
        
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
    */
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
        /*myScale.setCalibrationFactor(input.toFloat());
        EEPROM.put(LOCATION_CALIBRATION_FACTOR, input.toFloat());
        EEPROM.commit();
        state = 0;
        displaydata();*/
        break;
    case 2:     // setWeightOffset
        /*myScale.setZeroOffset(long(input.toFloat()));
        EEPROM.put(LOCATION_ZERO_OFFSET, long(input.toFloat()));
        EEPROM.commit();
        state = 0;
        displaydata();*/
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
    Wire.begin();

    if (myScale.begin() == false)
    {
        Serial.println("Scale not detected. Please check wiring. Freezing...");
        while (1);
    }
    Serial.println("Scale detected!");
    
    // Gain before A/D Converter
    myScale.setGain(NAU7802_GAIN_8);    

    //load maxweight
    EEPROM.get(LOCATION_MAXWEIGHT, maxweight);

    //load patientweight
    EEPROM.get(LOCATION_PATIENTWEIGHT, patientweight);

    //load real weight from calibration table
    EEPROM_readAnything(LOCATION_REAL_WEIGHT, real_weight);
    
    //load real value from calibration table
    EEPROM_readAnything(LOCATION_READING_VAL, raw_value);
}

bool updateAvgWeight()      // update moving Average and store value in weight
{
    long raw_reading = myScale.getReading();
    uint8_t lower_pos = 0;
    uint8_t upper_pos = 0;
    for (byte i = 0; i < 14; i++)
    {
        if ((raw_reading > raw_value[i]) && (raw_reading < raw_value[i + 1]))
        {
            lower_pos = i;
            upper_pos = i + 1;
            break;
        }
    }
    long calib_weight = map(raw_reading, raw_value[lower_pos], raw_value[upper_pos], real_weight[lower_pos], real_weight[upper_pos]);

    if (fCount >= (FILTER_SIZE)) fCount = 0;
    weightSum -= weightFilter[fCount];
    weightFilter[fCount] = calib_weight;        
    weightSum = weightSum + weightFilter[fCount];
    weight = weightSum / (FILTER_SIZE);
    fCount++;
    Serial.print("raw: ");
    Serial.print(raw_reading);
    Serial.print(" weight: ");
    Serial.print(weight);
    Serial.print(" weight: ");
    Serial.print(weight/1000);
    Serial.println(" KG");
    return true;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Serial.print("\r\nLast Packet Send Status:\t");
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
    if(!step && overload_flag)
    {   
        if (overload == 0 || overload < footload - maxweight)
        {
           overload = footload - maxweight;
        }
    }
}

void sendMeasurementDataOverBluetooth()
{
    StaticJsonDocument<100> measurement;
    measurement["time"] = millis(); // TODO? what time should be sent?
    measurement["raw_right"] = weight;
    measurement["raw_left"] = weightSlave;
    measurement["footload"] = footload;
    measurement["totalweight"] = totalweight;
    char buffer[100];
	size_t n = serializeJson(measurement, buffer);
    btSerial.print(buffer);
}

void sendAnaliticalDataOverBluetooth()
{
    StaticJsonDocument<100> measurement;
    measurement["steps"] = numb_steps;
    measurement["nr_overload"] = numb_overload;
    measurement["strengt_overloard"] = overload;
    char buffer[100];
	size_t n = serializeJson(measurement, buffer);
    btSerial.print(buffer);
}

void getRealWeight()
{
    static long rw_lk[15] = {0, 5000, 10000, 15000, 20000, 25000, 30000, 35000, 40000, 45000, 50000, 55000, 60000, 65000, 70000};
    // static long rs_lk[15] = {-77000, 160000, 430000, 700000, 980000, 1230000, 1360000, 1420000, 1580000, 1660000, 1750000, 1820000, 1950000, 2040000, 2300000};
    static long rs_lk[15] = {-85000, 110000, 370000, 630000, 810000, 900000, 1090000, 1200000, 1260000, 1310000, 1390000, 1470000, 1540000, 1610000, 1780000};
    long reading = myScale.getReading();
    Serial.print("gR: ");
    Serial.print(reading);
    byte lower_pos = 0;
    byte upper_pos = 0;
    for (byte i = 0; i < 14; i++)
    {
        if ((reading > rs_lk[i]) && (reading < rs_lk[i + 1]))
        {
            lower_pos = i;
            upper_pos = i + 1;
            break;
        }
    }
    Serial.print(" in range : ");
    Serial.print(rw_lk[lower_pos]);
    Serial.print(" - ");
    Serial.print(rw_lk[upper_pos]);

    long real_weight = map(reading, rs_lk[lower_pos], rs_lk[upper_pos], rw_lk[lower_pos], rw_lk[upper_pos]);
    Serial.print(" rw: ");
    Serial.print(real_weight);
    Serial.print(" fil: ");
    static int fC = 0;
    static int wSum = 0;
    static int wFil[FILTER_SIZE] = {0};
    if (fC >= (FILTER_SIZE)) fC = 0;
    wSum -= wFil[fC];
    wFil[fC] = real_weight;
    wSum += wFil[fC];
    int weight = wSum / FILTER_SIZE;
    Serial.print(weight);
    Serial.print(" gw: ");
    Serial.print(real_weight/1000);

    Serial.println();

}

template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++){
        EEPROM.write(ee++, *p++);
    }
    return i;
}


template <class T> int EEPROM_readAnything(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
    {
        *p++ = EEPROM.read(ee++);
    }
    return i;
}