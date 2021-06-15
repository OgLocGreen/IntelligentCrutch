#include <Arduino.h>
#include "BluetoothSerial.h"
#include <EEPROM.h>
#include <Wire.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_NAU7802
#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>

#define EEPROM_SIZE 512
#define FILTER_SIZE 1
#define LOCATION_MAXWEIGHT 0
#define LOCATION_PATIENTWEIGHT 10
#define LOCATION_SAVED_STEPS 30
#define LOCATION_REAL_WEIGHT 40
#define LOCATION_READING_VAL 140
#define LED_PIN 2
#define BEEPER_PIN 16
#define LOOP_FREQUENCY 50   // in Hz

int delaytime = 0;
int state = 0;
unsigned long lastloop = 0;
long raw_reading;
long weight = 0.0;
long weightFilter[FILTER_SIZE] = { 0 };
int fCount = 0;
long weightSum = 0.0;  // used for moving average filter
bool spam = 1;
long maxweight = 0;
long patientweight = 0;
long footload = 0;
long real_weight[15];
long raw_value[15];
unsigned long start;

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
        measurement["raw_reading"] = raw_reading;
        measurement["crutch_l"] = weight;
        measurement["sending"] = (result == ESP_OK);
        char buffer[150];
        serializeJson(measurement, buffer);
        btSerial.println(buffer);
    }
    // signal overload
    // if (footload > maxweight)
    // {
    //     digitalWrite(LED_PIN, HIGH);
    //     digitalWrite(BEEPER_PIN, LOW);
    // }
    // else
    // {
    //     digitalWrite(LED_PIN, LOW);
    //     digitalWrite(BEEPER_PIN, HIGH);
    // }
    BluetoothCommandHandler();
    smartdelay();
    if(millis() - start >= 800)
    {
        digitalWrite(BEEPER_PIN, HIGH);
        digitalWrite(LED_PIN, LOW);
    }
}

void BluetoothCommandHandler()
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
            // "{ "cmd" : "set_calib", "row": 12, "real_weight": 70000, "reading_val" : 112000 }"
            uint8_t row = receivedMsg["row"];
            long new_real_weight = receivedMsg["real_weight"];
            long new_raw_value = receivedMsg["raw_value"];
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
            StaticJsonDocument<512> doc;
            JsonArray real_weight_js = doc.createNestedArray("real_weight");
            JsonArray raw_value_js = doc.createNestedArray("raw_value");
            for (size_t i = 0; i < 15; i++)
            {
                real_weight_js.add(real_weight[i]);
                raw_value_js.add(raw_value[i]);
            }
            char buffer[512];
            serializeJson(doc, buffer);
            btSerial.println(buffer);
        }
        
        else if (cmd.equals("set_pat_weight"))
        {
            // "{ "cmd" : "set_pat_weight", "value" : 70.00 }"" // in kg and float
            float wg = receivedMsg["value"];
            if (patientweight != wg*1000)
            {
                patientweight = ((long)wg*1000);
                EEPROM_writeAnything(LOCATION_PATIENTWEIGHT, patientweight);
                write_eeprom = true;
            }
        }
        else if (cmd.equals("get_pat_weight"))
        {
            //load patientweight
            StaticJsonDocument<20> doc;
            EEPROM_readAnything(LOCATION_PATIENTWEIGHT, patientweight);
            doc["pat_weight"] = (float)patientweight/1000.00;
            char buffer[20];
            serializeJson(doc, buffer);
            btSerial.println(buffer);
        }
        else if (cmd.equals("set_max_weight"))
        {
            // "{ "cmd" : "set_max_weight", "value" : 50 }"" // in kg and float
            float mg = receivedMsg["value"];
            if (maxweight != mg*1000)
            {
                maxweight = ((long)mg*1000);
                EEPROM_writeAnything(LOCATION_MAXWEIGHT, maxweight);
                write_eeprom = true;
            }
        }
        else if (cmd.equals("get_max_weight"))
        {
            StaticJsonDocument<20> doc;
            EEPROM_readAnything(LOCATION_MAXWEIGHT, maxweight);
            doc["max_weight"] = (float)maxweight/1000.00;
            char buffer[20];
            serializeJson(doc, buffer);
            btSerial.println(buffer);
        }
        
        else
        {
            btSerial.println("command not recognised");
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
    
    // Gain before A/D Converter
    myScale.setGain(NAU7802_GAIN_8);    

    //load maxweight
    EEPROM_readAnything(LOCATION_MAXWEIGHT, maxweight);
    Serial.println(maxweight);
    //load patientweight
    EEPROM_readAnything(LOCATION_PATIENTWEIGHT, patientweight);
    Serial.println(patientweight);
    //load real weight from calibration table
    EEPROM_readAnything(LOCATION_REAL_WEIGHT, real_weight);
    
    //load real value from calibration table
    EEPROM_readAnything(LOCATION_READING_VAL, raw_value);
}


// update moving Average and store value in weight
bool updateAvgWeight()      
{
    raw_reading = myScale.getReading();
    raw_reading = (raw_reading <= raw_value[0]) ? raw_value[0] : raw_reading; // so that no negative value come as result
    uint8_t lower_pos = 0;
    uint8_t upper_pos = 0;
    for (byte i = 0; i < 14; i++)
    {
        if ((raw_reading >= raw_value[i]) && (raw_reading < raw_value[i + 1]))
        {
            lower_pos = i;
            upper_pos = i + 1;
            break;
        }
    }
    long calib_weight = map(raw_reading, raw_value[lower_pos], raw_value[upper_pos], real_weight[lower_pos], real_weight[upper_pos]);
    
    if (calib_weight <= 1000) calib_weight = 0;
    if (fCount >= (FILTER_SIZE)) fCount = 0;
    weightSum -= weightFilter[fCount];
    weightFilter[fCount] = calib_weight;        
    weightSum = weightSum + weightFilter[fCount];
    weight = weightSum / (FILTER_SIZE);
    fCount++;

    // btSerial.print("raw: ");
    // btSerial.print(raw_reading);
    // btSerial.print(" cal: ");
    // btSerial.println(weight);
    return true;
}


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Serial.print("Last Packet Send Status:\t");
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
    //Serial.print("Bytes received: ");
    //Serial.println(len);
    footload = incomingReadings;
    if (footload == 5555)
    {
        digitalWrite(BEEPER_PIN, LOW);     // high = beeper off
        digitalWrite(LED_PIN, HIGH);
        start = millis();
    }
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