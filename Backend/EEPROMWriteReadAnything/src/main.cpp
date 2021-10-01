#include <EEPROM.h>

#define EEPROM_SIZE 512

byte max_weight;
byte patient_weight;
long real_weight[15];
long raw_value[15];

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

void clearEEPROM(){
  // write a 0 to all 512 bytes of the EEPROM
  for (int i = 0; i < 512; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.end();
}

#define LOCATION_MAXWEIGHT 0
#define LOCATION_PATIENTWEIGHT 10
#define LOCATION_SAVED_STEPS 30
#define LOCATION_REAL_WEIGHT 40
#define LOCATION_READING_VAL 140

void writeInCrutch(){
  
  delay(5000);
  Serial.begin(115200);
  while(!Serial){;}
  float mw = 20.;
  float rw = 69;
  long real_weight[15] = {0, 5000, 10000, 15000, 20000, 25000, 30000, 35000, 40000, 45000, 50000, 55000, 60000, 65000, 70000};
  long raw_value[15] =  {44500, 279000, 500000, 790000, 1000000, 1300000, 1560000, 1610000, 1695000, 1760000, 1850000, 1900000, 1964000, 2000000, 2100000};
  long raw_value2[15] = {41000, 280000, 540000, 840000, 1125000, 1280000, 1394000, 1500000, 1600000, 1670000, 1710000, 1805000, 1900000, 2000000, 2100000};
  
  EEPROM_writeAnything(LOCATION_MAXWEIGHT, mw);
  EEPROM_writeAnything(LOCATION_PATIENTWEIGHT, rw);
  EEPROM_writeAnything(LOCATION_REAL_WEIGHT, real_weight);
  EEPROM_writeAnything(LOCATION_READING_VAL, raw_value2);
  EEPROM.commit();
}

void readFromKruecke(){
  float a, b;
  EEPROM_readAnything(LOCATION_MAXWEIGHT, a);
  EEPROM_readAnything(LOCATION_PATIENTWEIGHT, b);
  Serial.print("m_w: ");
  Serial.println(a);
  Serial.print("p_w: ");
  Serial.println(b);
  long x[15];
  EEPROM_readAnything(LOCATION_REAL_WEIGHT, x);
    for(int i = 0; i<15; i++){
    Serial.print(" w: ");
    Serial.print(x[i]);
  }
  Serial.println();
  
  long y[15];
  EEPROM_readAnything(LOCATION_READING_VAL, y);
    for(int i = 0; i<15; i++){
    Serial.print(" z: ");
    Serial.print(y[i]);
  }  
  Serial.println();
}

void setup() {
  EEPROM.begin(EEPROM_SIZE);
  Serial.begin(115200);
  delay(5000);
  while(!Serial){;}
  //clearEEPROM();
  writeInCrutch();
  readFromKruecke();
}

void loop() {
  // put your main code here, to run repeatedly:
}