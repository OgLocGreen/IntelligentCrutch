#include <Arduino.h>
#include "BluetoothSerial.h"
#include <EEPROM.h>
#include <Wire.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_NAU7802
#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>

#define EEPROM_SIZE 512
#define FILTER_SIZE 100       // 1: filter is off >1: length of filter array
#define LOCATION_MAXWEIGHT 0
#define LOCATION_PATIENTWEIGHT 10
#define LOCATION_SAVED_STEPS 30
#define LOCATION_REAL_WEIGHT 40
#define LOCATION_READING_VAL 140
#define LED_PIN 2
#define BEEPER_PIN 16
#define LOOP_FREQUENCY 20   // in Hz
#define TIME_SEND_MEASUREMENT 100 // in milliseconds

unsigned long lastloop = 0;
unsigned long lastmsg = 0;

long weight = 0;         // in grams
long totalweight = 0;    // in grams
int footload = 0;
long maxfootload = 0; 
int new_body_weight = 0;
long weightFilter[FILTER_SIZE] = { 0 };
int fCount = 0;
float weightSum = 0.0;  // used for moving average filter
bool spam = true;
bool send_analitic_data = false;
bool measuring_weight = false;
long maxweight = 0;
long patientweight = 0;
long real_weight[15];
long raw_value[15];
bool receiveflag = false;


long numb_steps = 0;
long numb_overload =0;
long maxtotalweight = 0;
long low_threshold = 15000;
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

void checkstep_overload();

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

void sendMeasurementDataOverBluetooth();
template <class T> int EEPROM_writeAnything(int ee, const T& value);
template <class T> int EEPROM_readAnything(int ee, T& value);
void getRealWeight();
void BluetoothCommandHandler();
void calculateMeasurement();

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
    updateAvgWeight();
    calculateMeasurement();
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
    checkstep_overload();
    BluetoothCommandHandler();
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
        else if (cmd.equals("analitical"))
        {
            send_analitic_data = !send_analitic_data;
        }
        else if (cmd.equals("set_calib"))
        {
            // "{ "cmd" : "set_calib", "row": 12, "real_weight": 70000, "reading_val" : 112000 }"
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
            // StaticJsonDocument<512> doc;
            // JsonArray real_weight_js = doc.createNestedArray("real_weight");
            // JsonArray raw_value_js = doc.createNestedArray("raw_value");
            // //load real weight from calibration table
            // EEPROM_readAnything(LOCATION_REAL_WEIGHT, real);
            // //load real value from calibration table
            // EEPROM_readAnything(LOCATION_READING_VAL, raw);
            // for (size_t i = 0; i < 15; i++)
            // {
            //     real_weight_js.add();
            //     raw_value_js.add();
            // }
            // char buffer[512];
            // serializeJson(doc, buffer);
            // btSerial.println(buffer);
        }
        else if (cmd.equals("set_pat_weight"))
        {
            // "{ "cmd" : "set_pat_weight", "value" : 100 }""
            float wg = receivedMsg["value"];
            if (patientweight != wg*1000)
            {
                patientweight = wg;
                EEPROM_writeAnything(LOCATION_PATIENTWEIGHT, patientweight);
                write_eeprom = true;
            }
        }
        else if (cmd.equals("set_max_weight"))
        {
            // "{ "cmd" : "set_max_weight", "value" : 100 }""
            float mg = receivedMsg["value"];
            if (maxweight != mg*1000)
            {
                maxweight = mg;
                EEPROM_writeAnything(LOCATION_MAXWEIGHT, maxweight);
                write_eeprom = true;
            }
        }
        else if (cmd.equals("measure_weight"))
        {
            measuring_weight = true;
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
    
    // Gain before A/D Converter
    myScale.setGain(NAU7802_GAIN_8);    

    //load maxweight
    EEPROM.get(LOCATION_MAXWEIGHT, maxweight);
    maxweight *= 1000; // work in grams

    //load patientweight
    EEPROM.get(LOCATION_PATIENTWEIGHT, patientweight);
    patientweight *= 1000; // work in grams

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
    
    if (calib_weight <= 2000) calib_weight = 0;
    if (fCount >= (FILTER_SIZE)) fCount = 0;
    
    weightSum -= weightFilter[fCount];
    weightFilter[fCount] = calib_weight;        
    weightSum = weightSum + weightFilter[fCount];
    weight = weightSum / (FILTER_SIZE);
    fCount++;
    /*Serial.print("raw: ");
    Serial.print(raw_reading);
    Serial.print(" weight: ");
    Serial.print(weight);
    Serial.print(" weight: ");
    Serial.print(weight/1000);
    Serial.println(" KG");*/
    return true;
}

void calculateMeasurement()
{
//    if(receiveflag)
//    {
        receiveflag = false;
        lastmsg = millis();
        totalweight = weight + weightSlave;
        if (totalweight > 2000) {
            footload = patientweight - totalweight;
            footload = footload/1000;
        }
        else { 
            footload = 0.0;
        }
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &footload, sizeof(footload));
        if (spam) sendMeasurementDataOverBluetooth();
//}
}

void measureBodyWeight()
{
    static long last_time = millis();
    static byte state = 0;
    static bool once = 0;
    static long max = 0;
    if (measuring_weight)
    {
        if (!once)
        {
            last_time = millis();
            once = true;
        }
        // preparation in 2 second
        if (state == 0)
        {
            spam = false;
            send_analitic_data = false;
            digitalWrite(BEEPER_PIN, LOW);
            if (millis() - last_time > 2000)
            {
                state = 1;
                last_time = millis();
            }
        }
        // measurement in 5 second
        if (state == 1) {
            digitalWrite(BEEPER_PIN, HIGH);
            // search the maximum value
            max = max <= totalweight ? totalweight : max;
            if (millis() - last_time > 5000)
            {
                state = 0;
                last_time = millis();
                once = false;
                spam = false;
                send_analitic_data = false;
                StaticJsonDocument<20> measurement;
                new_body_weight = max / 1000;
                measurement["measured_weight"] = new_body_weight;
                char buffer[20];
	            size_t n = serializeJson(measurement, buffer);
                btSerial.print(buffer);
                measuring_weight = false;
            }
        }       
    }
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


void checkstep_overload()
{
    if (maxtotalweight < totalweight)
    {
        maxtotalweight = totalweight; // Hier eigenentlich noch patientweight - totalweight 
    }

    if(!step && (totalweight > low_threshold))   //Schritt beginn
    {
        numb_steps++;
        step = true;
        maxfootload = 0;
    }

    if(step && (totalweight < low_threshold))    // Schritt ende
    {
        step = false;


        if (maxfootload < 69000 - maxtotalweight )
        {
            maxfootload = 69000 - maxtotalweight;
        }

        if (maxtotalweight > 50000)
        {
            numb_overload++;
        }

        maxtotalweight = 0;
    }
}


void sendMeasurementDataOverBluetooth()
{
    static int last = millis();
    if (millis() - last >= TIME_SEND_MEASUREMENT)
    {
        StaticJsonDocument<150> measurement;
        //measurement["time"] = millis(); // TODO? what time should be sent?
        //measurement["crutch_r"] = weight;
        //measurement["crutch_l"] = weightSlave;
        measurement["maxtotalweight"] = maxtotalweight;
        measurement["total"] = totalweight;
        //measurement["footload"] = footload;
        measurement["steps"] = numb_steps;
        measurement["number_ov"] = numb_overload;
        measurement["maxfootload"] = maxfootload;
        char buffer[150];
	    size_t n = serializeJson(measurement, buffer);
        btSerial.println(buffer);
        last = millis();
    }
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