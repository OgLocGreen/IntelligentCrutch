#include <Arduino.h>
#include "BluetoothSerial.h"
#include <EEPROM.h>
#include <Wire.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_NAU7802
#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>

#define EEPROM_SIZE 512
#define FILTER_SIZE 100
#define LOCATION_MAXWEIGHT 0
#define LOCATION_PATIENTWEIGHT 10
#define LOCATION_SAVED_STEPS 30
#define LOCATION_REAL_WEIGHT 40
#define LOCATION_READING_VAL 140
#define LED_PIN 2
#define BEEPER_PIN 16
#define LOOP_FREQUENCY 50   // in Hz

String input;
int delaytime = 0;
int state = 0;
unsigned long lastloop = 0;
long raw_reading;
int weight = 0.0;
long weightFilter[FILTER_SIZE] = { 0 };
int fCount = 0;
float weightSum = 0.0;  // used for moving average filter
bool spam = 1;
float maxweight = 0;
float footload = 0;
long real_weight[15];
long raw_value[15];

int incomingReadings = 0;
uint8_t broadcastAddress[] = {0x24, 0x0A, 0xC4, 0x5F, 0xD8, 0x8C};  // MAC Adress of crutch Nr. 1
//uint8_t broadcastAddress[] = {0x8C, 0xAA, 0xB5, 0x8A, 0xFC, 0x1C}; // MAC Adress of crutch Nr. 2

BluetoothSerial btSerial;		           // Bluetooth
NAU7802 myScale; //Create instance of the NAU7802 class

//void datareceived();
void displaydata();
void setupScale(void);
bool updateAvgWeight();
void smartdelay();

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
template <class T> int EEPROM_writeAnything(int ee, const T& value);
template <class T> int EEPROM_readAnything(int ee, T& value);
void BluetoothCommandHandler();

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

  lastloop = millis();
}

void loop() {
    updateAvgWeight();
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &weight, sizeof(weight));

    if (spam) {
        StaticJsonDocument<150> measurement;
        measurement["dt"] = delaytime;
        measurement["raw_reading"] = raw_reading;
        measurement["crutch_l"] = weight;
        measurement["sending"] = (result == ESP_OK);
        char buffer[150];
        size_t n = serializeJson(measurement, buffer);
        btSerial.println(buffer);
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

    //if (btSerial.available()) datareceived();
    BluetoothCommandHandler();
    //delay(10);
    smartdelay();
}

void BluetoothCommandHandler()  // This is a task.
{
    static bool write_eeprom = false;
    while (btSerial.available())
    {
        String raw_cmd = btSerial.readStringUntil('\n');
        StaticJsonDocument<150> receivedMsg;
        deserializeJson(receivedMsg, raw_cmd.c_str());
        String cmd = receivedMsg["cmd"];
        if (cmd.equals("spam"))
        {
            spam = !spam;
        }
        else if (cmd.equals("set_calib"))
        {
            // "{ "cmd" : "set_calib", "row": 12, "real_weight": 70000, "reading_val" : 112000 }""
            uint8_t row = receivedMsg["row"];
            long new_real_weight = receivedMsg["real_weight"];
            long new_raw_value = receivedMsg["reading_val"];
            if (new_real_weight != real_weight[row]){
                real_weight[row] = new_real_weight;
                EEPROM_writeAnything(LOCATION_REAL_WEIGHT, real_weight);
                write_eeprom = true;
            }
            if (new_raw_value != raw_value[row])
            {
                raw_value[row] = new_raw_value;
                EEPROM_writeAnything(LOCATION_READING_VAL, raw_value);
                write_eeprom = true;
            }
        }
        else if (cmd.equals("get_calib"))
        {
            long real, raw;
            //load real weight from calibration table
            EEPROM_readAnything(LOCATION_REAL_WEIGHT, real);
            //load real value from calibration table
            EEPROM_readAnything(LOCATION_READING_VAL, raw);
            for (size_t i = 0; i < 15; i++)
            {
                btSerial.print(i);
                btSerial.print(" | ");
                btSerial.print(real);
                btSerial.print(" | ");
                btSerial.println(raw);   
            }
        }
        else
        {

        }
    }
    if (write_eeprom)
    {
        EEPROM.commit();
        write_eeprom = false;
    }
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
    
    myScale.setGain(NAU7802_GAIN_8);    
    EEPROM.get(LOCATION_MAXWEIGHT, maxweight);
    EEPROM_readAnything(LOCATION_REAL_WEIGHT, real_weight);
    EEPROM_readAnything(LOCATION_READING_VAL, raw_value);
}

bool updateAvgWeight()      // update moving Average and store value in weight
{
    static int raw_count = 0;
    static long raw_filter[200] = {0};
    static long raw_sum = 0;
    static long raw_not_filterd;

    raw_not_filterd = myScale.getReading();
    if (raw_count >= 200) { raw_count = 0; }
    raw_not_filterd = myScale.getReading();
    raw_sum -= raw_filter[raw_count];
    raw_filter[raw_count] = raw_not_filterd;
    raw_sum += raw_filter[raw_count];
    raw_reading = raw_sum/200;
    raw_count++;
    
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
    
    if (calib_weight <= 2000) calib_weight = 0;
    if (fCount >= (FILTER_SIZE)) fCount = 0;
    
    weightSum -= weightFilter[fCount];
    weightFilter[fCount] = calib_weight;        
    weightSum = weightSum + weightFilter[fCount];
    weight = weightSum / (FILTER_SIZE);
    fCount++;
    return true;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//   Serial.print("Last Packet Send Status:\t");
//   Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  //Serial.print("Bytes received: ");
  //Serial.println(len);
  footload = incomingReadings;
}

void smartdelay()
{
	delaytime = 0;
	delaytime = 1000 / LOOP_FREQUENCY - (millis() - lastloop);
	if (delaytime < 0) delaytime = 0;
	if (delaytime > (1000 / LOOP_FREQUENCY)) delaytime = (1000 / LOOP_FREQUENCY);
	delay(delaytime);
	lastloop = millis();
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