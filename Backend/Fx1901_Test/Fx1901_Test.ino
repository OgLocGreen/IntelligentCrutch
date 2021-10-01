/*
    Name:       Fx1901_Test.ino
    Author:     JOACHIM_LAPTOP\Joachim
    Created:	04.05.2021 14:51:54
*/


#include "BluetoothSerial.h"
#include <EEPROM.h>
#include <Wire.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_NAU7802

#define EEPROM_SIZE 256
#define FILTER_SIZE 1
#define LOCATION_CALIBRATION_FACTOR 0
#define LOCATION_ZERO_OFFSET 10
#define LOCATION_MAXWEIGHT 20
#define LED_PIN 2
#define BEEPER_PIN 16

String input;
int state = 0;

float weight = 0.0;
long weightFilter[FILTER_SIZE] = { 0.0 };
int fCount = 0;
float weightSum = 0.0;
bool spam = 1;
float maxweight = 0;

BluetoothSerial btSerial;		           // Bluetooth
NAU7802 myScale; //Create instance of the NAU7802 class


void setup()
{
    Serial.begin(115200);
    Serial.println("iUAGS Qwiic Scale Test");

    EEPROM.begin(EEPROM_SIZE);

    setupScale();    // Load zeroOffset and calibrationFactor from EEPROM

    pinMode(BEEPER_PIN, OUTPUT);
    digitalWrite(BEEPER_PIN, HIGH);     // high = beeper off
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);

    btSerial.begin("iUAGS");
}

void loop()
{
    Serial.print("Raw reading: ");
    Serial.println(myScale.getReading());

    updateAvgWeight();
    //Serial.print("   Filtered Weight: ");
    //Serial.println(weight);

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
//If anything looks weird, reset setting to default value
//Return false, if actual values are default
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
