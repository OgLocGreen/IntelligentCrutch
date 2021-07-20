#include <Arduino.h>
#include "BluetoothSerial.h"
#include <EEPROM.h>
#include <Wire.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_NAU7802
#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>

#include "FS.h"
#include "SPIFFS.h"

#define EEPROM_SIZE 512
#define FILTER_SIZE 1       // 1: filter is off >1: length of filter array
#define LOCATION_MAXWEIGHT 0
#define LOCATION_PATIENTWEIGHT 10
#define LOCATION_SAVED_STEPS 30
#define LOCATION_REAL_WEIGHT 40
#define LOCATION_READING_VAL 140
#define LOCATION_LOGTIME 240
#define LED_PIN 2
#define BEEPER_PIN 16
#define LOOP_FREQUENCY 50   // in Hz
//#define TIME_SEND_MEASUREMENT 50 // in milliseconds


long weight = 0;         // in grams
long totalweight = 0;    // in grams
int footload = 0;
long maxfootload = 0; 
int new_body_weight = 0;
long weightFilter[FILTER_SIZE] = { 0 };
int fCount = 0;
float weightSum = 0.0;  // used for moving average filter
bool spam = true;
bool measuring_weight = false;
long maxweight = 0;
long patientweight = 0;
long real_weight[15];
long raw_value[15];
bool receiveflag = false;
unsigned long start;
unsigned long lastloop;
int logtime = 30000;    // ms

long numb_steps = 0;
long numb_overload =0;
long maxtotalweight = 0;
long low_threshold = 8000;
bool up = false;
bool down = false;

bool step = false;
bool overload_flag = false;

float overloadArray[10];
int arrayIndex = 0;

int incomingReadings = 0;
int weightSlave = 0;     

bool loggingflag = false;
long logstartmillis = 0;

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

void appendFile(fs::FS& fs, const char* path, const char* message);
void writeFile(fs::FS& fs, const char* path, const char* message);
void readFile(fs::FS& fs, const char* path);
void logEntry();


void setup() {
    Serial.begin(115200);
    Serial.println("iUAGS Qwiic Scale Test");

    SPIFFS.begin(true);      // SPIFFS begin

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
    digitalWrite(BEEPER_PIN, HIGH);     // LOW = beeper off
    delay(200);
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BEEPER_PIN, LOW);     // LOW = beeper off
    btSerial.begin("iUAGS");

    lastloop = millis();
}


void loop() {
    updateAvgWeight();
    calculateMeasurement();
    checkstep_overload();
    BluetoothCommandHandler();
    if (loggingflag == true)
    {
        logEntry();
    }

    if(millis() - start >= 800)
    {
        digitalWrite(BEEPER_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
    }
    smartdelay();
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
        else if (cmd.equals("reset"))
        {
            maxtotalweight = 0;
            maxfootload = 0;
            numb_steps = 0;
            numb_overload = 0;
        }
        else if (cmd.equals("readlog"))
        {
            readFile(SPIFFS, "/LogFile.txt");
        }
        else if (cmd.equals("logdata"))
        {
            writeFile(SPIFFS, "/LogFile.txt", "millis;crutchR;crutchL;total;maxtotal;footload;maxfootload;steps;overloads");
            loggingflag = true;
            logstartmillis = millis();
        }
        else if (cmd.equals("logtime"))
        {
            logtime = receivedMsg["logtime"];
            logtime = 1000 * logtime;
            EEPROM.write(LOCATION_LOGTIME, logtime);
            EEPROM.commit();
            btSerial.print("New Logtime: ");
            btSerial.println(logtime);
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

    logtime = EEPROM.read(LOCATION_LOGTIME);
}


// update moving Average and store value in weight
bool updateAvgWeight()      
{
    long raw_reading = myScale.getReading();
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
    /*
    btSerial.print("\nRaw Val: ");
    btSerial.print(raw_reading);
    btSerial.print("\n   lower raw: ");
    btSerial.print(raw_value[lower_pos]);
    btSerial.print("   lower weight: ");
    btSerial.print(real_weight[lower_pos]);
    btSerial.print("\n   upper raw: ");
    btSerial.print(raw_value[upper_pos]);
    btSerial.print("   upper weight: ");
    btSerial.println(real_weight[upper_pos]);*/

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


void calculateMeasurement()
{
    totalweight = weight + weightSlave;
    if (totalweight > 4000) {
        footload = patientweight - totalweight;
        if (footload < 0.0)
            footload = 0.0;
    }
    else { 
        footload = 0.0;
    }
    if (spam) sendMeasurementDataOverBluetooth();
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
            digitalWrite(BEEPER_PIN, HIGH);
            if (millis() - last_time > 2000)
            {
                state = 1;
                last_time = millis();
            }
        }
        // measurement in 5 second
        if (state == 1) {
            digitalWrite(BEEPER_PIN, LOW);
            // search the maximum value 
            max = max <= totalweight ? totalweight : max;
            if (millis() - last_time > 5000)
            {
                state = 0;
                last_time = millis();
                once = false;
                spam = false;
                StaticJsonDocument<20> measurement;
                new_body_weight = max / 1000;
                measurement["measured_weight"] = new_body_weight;
                char buffer[20];
	            serializeJson(measurement, buffer);
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
    memcpy(&incomingReadings, incomingData, len);
    //Serial.print("Bytes received: ");
    //Serial.println(len);
    weightSlave = incomingReadings;
    receiveflag = true;
}


void checkstep_overload()
{
    if (maxtotalweight < totalweight)
    {
        maxtotalweight = totalweight;
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
        maxfootload = patientweight - maxtotalweight;
        if (maxfootload < 0) maxfootload = 0;
        if (maxweight < maxfootload)
        {
            int start_beeb = 5555;
            digitalWrite(LED_PIN, HIGH);
            digitalWrite(BEEPER_PIN, HIGH);
            esp_now_send(broadcastAddress, (uint8_t *) &start_beeb, sizeof(start_beeb));
            numb_overload++;
            start = millis();
        }
        maxtotalweight = 0;
    }
}


void sendMeasurementDataOverBluetooth()
{
    //static int last = millis();
    //if (millis() - last >= TIME_SEND_MEASUREMENT)
    //{
        StaticJsonDocument<200> measurement;
        measurement["t"] = millis(); // TODO? what time should be sent?
        measurement["cr"] = (float)weight/1000.00;
        measurement["cl"] = (float)weightSlave/1000.0;
        measurement["to"] = (float)totalweight/1000.00;
        measurement["mw"] = (float)maxtotalweight/1000.00;
        measurement["fl"] = (float)footload/1000.00;
        measurement["mf"] = (float)maxfootload/1000.00;
        measurement["s"] = numb_steps;
        measurement["n"] = numb_overload;
        
        char buffer[200];
        serializeJson(measurement, buffer);
        btSerial.println(buffer);
    //    last = millis();
    //}
}


template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++) 
    {
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

void smartdelay()
{
    int delaytime = 0;
	delaytime = 1000 / LOOP_FREQUENCY - (millis() - lastloop);
	if (delaytime < 0) delaytime = 0;
	if (delaytime > (1000 / LOOP_FREQUENCY)) delaytime = (1000 / LOOP_FREQUENCY);
	delay(delaytime);
	lastloop = millis();
}

void logEntry()         // SPIFFS
{
    if (millis() - logstartmillis > logtime) 
    {
        btSerial.print("\nLog finished\n");
        loggingflag = false;
    }
    String entry;
    entry = '\n';
    entry += String(millis());
    entry += ';';
    entry += String(weight);
    entry += ';';
    entry += String(weightSlave);
    entry += ';';
    entry += String(totalweight);
    entry += ';';
    entry += String(maxtotalweight);
    entry += ';';
    entry += String(footload);
    entry += ';';
    entry += String(maxfootload);
    entry += ';';
    entry += String(numb_steps);
    entry += ';';
    entry += String(numb_overload);
    entry += ";";
    char centry[370];
    entry.toCharArray(centry, 370);
    appendFile(SPIFFS, "/LogFile.txt", centry);
}

void appendFile(fs::FS& fs, const char* path, const char* message) {	// SPIFFS
    //btSerial.printf("\nAppending to file: %s\r\n", path);

    File file = fs.open(path, FILE_APPEND);
    if (!file) {
        btSerial.println("− failed to open file for appending");
        return;
    }
    if (file.print(message)) {
        //btSerial.println("− message appended");
    }
    else {
        btSerial.println("− append failed");
    }
}

void writeFile(fs::FS& fs, const char* path, const char* message) {		// overwrite
    btSerial.printf("\nWriting file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file) {
        btSerial.println("− failed to open file for writing");
        return;
    }
    if (file.print(message)) {
        //btSerial.println("− file written");
    }
    else {
        btSerial.println("− frite failed");
    }
}

void readFile(fs::FS& fs, const char* path)	
{
    btSerial.printf("\nReading file: %s\r\n", path);

    File file = fs.open(path);
    if (!file || file.isDirectory()) {
        btSerial.println("- failed to open file for reading");
        return;
    }

    while (file.available()) {
        btSerial.write(file.read());
    }
}

