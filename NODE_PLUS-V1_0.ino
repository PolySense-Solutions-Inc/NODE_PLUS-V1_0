/*
 * Copyright (c) PolySense Solutions Inc.
 *
 * All rights reserved.
 *
 * Author: Matthew Gale
 * Date Created: Feb 23 2024
 * Date Modified: Feb 23 2024
 * Firmware Version 1.0
 */


#include <Arduino.h>
#include <Adafruit_SPIFlash.h>
#include <SPI.h>
#include "Adafruit_SHT4x.h"
#include <Adafruit_MCP23X08.h>
#include <Adafruit_MAX31865.h> //RTD
#include <Adafruit_MAX31856.h> //thermocouple
#include <cstring>
#include <Wire.h>
#include <RunningMedian.h>
#include <Adafruit_MPU6050.h> //acclerometer
#include "Adafruit_VEML7700.h" //lux

//~~~~~~~~~~~~~~~~~Compile Options~~~~~~~~~~~~~~~~~~~~~~~
#define FIRSTBOOT             false
#define DEBUG                 false
#define DEBUG_LED             false
#define THERMO                true //thermocouple?
#define AMBIENT               false   //? do we have an ambient sensor?  
#define RTD                   false   //? do we have RTD sensor? will be on channel 1 if true 
#define DOOR                  true   //? do we have a door sensor? 
#define ACCELa                false    //? do we have accelerometer on A
#define ACCELb                false    //? do we have accelerometer on B
#define LUX                   false    //? do we have lux sensor on channel A
#define RTDREF                4300.0  // The value of the ref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RTDNOMINAL            1000.0  //100.0 for PT100, 1000.0 for PT1000
#define RTDWIRES              2       //how many wires on RTD
#define MCP3008_MAX_SAMPLE    5       //how many samples to take from mcp adc read
#define sensorCheckInterval   30000   //how often to check sensors (30000 default)
#define MCU_SLEEP_TIME        3000    //how often are we waking up to check loop,sensors,etc
#define DEFAULT_PACKET_TIME   305000  //5 mins+5 seconds for buffer = 305000, 1 min = 60000 <-- How often to send packets if there are no flags
#define NUM_READINGS_accel    100

#if DEBUG
  #define WATCHDOG_TRIGGER    259200000//180000 // 5 minutes
#else
  #define WATCHDOG_TRIGGER    259200000//259200000 //3 days
#endif

//~~~~~~~~~~~~~~~~~constants ~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define WDG_CLOCK 32768
#define FLASH_DEEP_SLEEP_CMD  0xB9

//debug printing
#define DEBUG_SERIAL \
  if (DEBUG) Serial

//~~~~~~~~~~~~~~~~~~~~pins~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define MCP3008_CS_PIN        1    
#define REBOOT_RADIO_PIN      6   //on I/O expander
#define DOOR_ADC              0
#define THERMO_CS             0
#define RTD_POWER_PIN         2 //on I/O expander on hardware version 2
#define CH1_ADC               1
#define CH2_ADC               2
#define miso                  9
#define mosi                  10
#define sck                   8


//~~~~~~~~~~~~~~~~~~~~Global Variables~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const char appEUI[] = "AT+APPEUI=00:00:00:00:00:00:00:00\r\n";
int vBat;
int lastIndexWritten = -1; //because we increment and want to start at 0
int32_t lastSendAttemptTime;
int32_t sensorCheckTime;
int32_t packetDurationStart;
int32_t startTime;
int32_t currentTime;
//door variables
int32_t doorCheckTime;
int32_t doorOpenStart;
int32_t open_for; //door has been open for this many seconds
const size_t CH_PACK0_SIZE = 10; //5 mins is 10 items
const size_t CH_PACK1_SIZE = 8;
const size_t TEMP_PACK1_SIZE = 8;
const size_t HUMID_PACK1_SIZE = 8;
const size_t DOORVALS_SIZE = 6;
const size_t doorCountArray_SIZE = 28801; // For 24 hours of elements, 28800 + null-terminator
const size_t hexArray_SIZE = 26; // For 200 elements, 50 HEX characters + null-terminator
const size_t compressedDoorArray_SIZE = 101; // For 200 elements + null-terminator
int32_t doorVals[DOORVALS_SIZE];
int32_t ch1_pack0[CH_PACK0_SIZE];
int32_t ch2_pack0[CH_PACK0_SIZE];
int32_t ch1_pack1[CH_PACK1_SIZE];
int32_t ch2_pack1[CH_PACK1_SIZE];
int32_t temp_pack1[TEMP_PACK1_SIZE];
int32_t humid_pack1[HUMID_PACK1_SIZE];
char doorCountArray[doorCountArray_SIZE]; 
char hexArray[hexArray_SIZE];
char compressedDoorArray[compressedDoorArray_SIZE];
unsigned int doorCheckCounter = 0; //how many times have we checked the door (array counter)
int doorRead;
int join_attempt_qty;
bool doorAjar;
bool packetFull;
int packetType;


//~~~~~~~~~~~~~~~~~~~~ Instances ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_MCP23X08 mcpIO;
#if ACCELa
  Adafruit_MPU6050 mpu1;
  Adafruit_Sensor *mpu1_accel;
#endif
#if ACCELb
  Adafruit_MPU6050 mpu2;
  Adafruit_Sensor *mpu2_accel;
#endif


#if AMBIENT
  Adafruit_SHT4x sht4 = Adafruit_SHT4x(); //Temp sensor
#endif

#if LUX
  Adafruit_VEML7700 veml = Adafruit_VEML7700();
#endif

#if THERMO
  Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(THERMO_CS);
#endif


//~~~~~~~~~~~~~~~~~~~~ inits ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void init_IO_expander()
{
  DEBUG_SERIAL.println("I/O  Expander init");
  if (!mcpIO.begin_I2C(0x21)) 
  {
    DEBUG_SERIAL.println("I/O  Expander Issue");
    while (1);
  }
  DEBUG_SERIAL.println("I/O  Expander Connected");
  mcpIO.pinMode(REBOOT_RADIO_PIN, OUTPUT);
}

void initUART()
{
  #if DEBUG
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  #endif
}

void initAmbient()
{
  #if AMBIENT
    if (! sht4.begin()) {
      DEBUG_SERIAL.println("Couldn't find SHT4x");
      while (1) delay(1);
    }
    sht4.setPrecision(SHT4X_MED_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);
    DEBUG_SERIAL.println("SHT4x Sensor Initialized");
  #endif
}

void init_accelerometer()
{
  #if ACCELa
    if (!mpu1.begin(0x68)) {
      Serial.println("Failed to find MPU6050 chip");
      while (1) {
        delay(10);
      }
    }
    mpu1.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu1.setFilterBandwidth(MPU6050_BAND_44_HZ);
    mpu1_accel = mpu1.getAccelerometerSensor();
  #endif
  #if ACCELb
    if (!mpu2.begin(0x69)) {
      Serial.println("Failed to find MPU6050 chip");
      while (1) {
        delay(10);
      }
    }
    mpu2.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu2.setFilterBandwidth(MPU6050_BAND_44_HZ);
    mpu2_accel = mpu2.getAccelerometerSensor();
  #endif
}


void init_lux()
{
  #if LUX
    delay(500);
    if (!veml.begin()) {
      DEBUG_SERIAL.println("Lux sensor not found");
    }
    DEBUG_SERIAL.println("Lux sensor found");
      DEBUG_SERIAL.print(F("Gain: "));
    switch (veml.getGain()) {
      case VEML7700_GAIN_1: DEBUG_SERIAL.println("1"); break;
      case VEML7700_GAIN_2: DEBUG_SERIAL.println("2"); break;
      case VEML7700_GAIN_1_4: DEBUG_SERIAL.println("1/4"); break;
      case VEML7700_GAIN_1_8: DEBUG_SERIAL.println("1/8"); break;
    }
    veml.setIntegrationTime(VEML7700_IT_25MS, true);
    DEBUG_SERIAL.print(F("Integration Time (ms): "));
    switch (veml.getIntegrationTime()) {
      case VEML7700_IT_25MS: DEBUG_SERIAL.println("25"); break;
      case VEML7700_IT_50MS: DEBUG_SERIAL.println("50"); break;
      case VEML7700_IT_100MS: DEBUG_SERIAL.println("100"); break;
      case VEML7700_IT_200MS: DEBUG_SERIAL.println("200"); break;
      case VEML7700_IT_400MS: DEBUG_SERIAL.println("400"); break;
      case VEML7700_IT_800MS: DEBUG_SERIAL.println("800"); break;
    }
    veml.setLowThreshold(10000); //currently not used (interrupts)
    veml.setHighThreshold(20000); //currently not used (interrupts)
    veml.interruptEnable(false); 
    veml.powerSaveEnable(true);
    veml.enable(false); //put into low power mode
    
  #endif
}


void initRadio()
{
    Serial1.begin(9600);
    while (!Serial1) { 
    ;
    }
    Serial1.print("AT+AUTO_JOIN=OFF\r\n"); //set autojoin to "OFF", we need to do this manually so that we can obtain data rate
    print_reply_wait();
    Serial1.print(appEUI);
    print_reply_wait();
    Serial1.print("AT+CONFIRM=ON\r\n"); //confirm messages "ON"
    print_reply_wait();
    Serial1.print("AT+CH=8-15\r\n"); // set channel
    print_reply_wait();
    Serial1.print("AT+DEVEUI=?\r\n"); // get DEVEUI
    print_reply_wait();
    Serial1.print("AT+APPKEY=?\r\n"); // get APPKEY
    print_reply_wait();
    Serial1.print("AT+ADVI=0\r\n"); // turn off BLE advertising
    print_reply_wait();
    //If configured, join the lora network if it's the first power up
    Serial1.end();
    DEBUG_SERIAL.print("Init radio done");
}


void initRadio_factory()
{
  mcpIO.digitalWrite(REBOOT_RADIO_PIN, HIGH);
  delay(1000);
  mcpIO.digitalWrite(REBOOT_RADIO_PIN, LOW);
  if(!Serial1)
  {
    Serial1.begin(9600);
    while (!Serial1) { 
    ;
    }
  }
  Serial1.print("AT+SLEEP=OFF\r\n"); // wake up radio
  print_reply_wait();
  Serial1.print("AT+FACTORY\r\n"); //factory reboot
  print_reply_wait();
  Serial1.print("AT+AUTO_JOIN=OFF\r\n"); //set autojoin to "OFF", we need to do this manually so that we can obtain data rate
  print_reply_wait();
  Serial1.print(appEUI);
  print_reply_wait();
  Serial1.print("AT+CONFIRM=ON\r\n"); //confirm messages "ON"
  print_reply_wait();
  Serial1.print("AT+CH=8-15\r\n"); // set channel
  print_reply_wait();
  Serial1.print("AT+ADVI=0\r\n"); // turn off BLE advertising
  print_reply_wait();
  //If configured, join the lora network if it's the first power up
  Serial1.end();
  DEBUG_SERIAL.println("Factory Settings Restored");
}

void init_globals()
{
  doorAjar = false;
  doorCheckTime = millis();
  sensorCheckTime = millis();
  lastSendAttemptTime = millis();
  packetDurationStart = millis();
  startTime = millis();
  join_attempt_qty = 0;
  packetFull = false;
  packetType = 0;
}


void wdg_config(int32_t timeout) {
  /* Pause wdg when cpu sleeping */
  NRF_WDT->CONFIG = 1;
  /* Enable reload request register 1 */
  NRF_WDT->RREN = 1;
  /* Set reload counter with timeout as second */
  NRF_WDT->CRV = timeout * WDG_CLOCK;
  /* Start watchdog */
  NRF_WDT->TASKS_START = 1;
}


//~~~~~~~~~~~~~~~~~~~~ helper functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// Function to flash an LED a specific number of times
//this function can be improved later to have all 3 colors mix
//255 is off and 0 is full intensity. 
//LED options (pin options) are LED_RED, LED_GREEN, LED_BLUE
void FlashLED(uint32_t pin, int numFlashes, int delayDuration) {
  #if DEBUG_LED
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    for (int i = 0; i < numFlashes; i++) {
      analogWrite(pin, 0); // Turn LED on (0 is ON)
      delay(delayDuration); // Keep LED on for 'delayDuration' milliseconds
      
      analogWrite(pin, 255); // Turn LED off (255 is OFF)
      delay(delayDuration); // Keep LED off for 'delayDuration' milliseconds
    }
  #endif
}


//watchdog feed function
void wdg_feed(void) {
  NRF_WDT->RR[0] = 0x6E524635;
}


void print_reply()
{
  DEBUG_SERIAL.println("print reply start");
  int32_t timeoutStart = millis();
  delay(500);
  while (millis() < timeoutStart+500)
  {
    while (Serial1.available())
    {
      #if DEBUG
        DEBUG_SERIAL.print((char)Serial1.read());
      #else
        Serial1.read();
      #endif
    }
  }
  DEBUG_SERIAL.println("print reply end");
}



//this function will wait at least 5 seconds for a reply
//but will terminate after 200ms of getting a character
void print_reply_wait()
{
  DEBUG_SERIAL.println("print/wait reply start");
  int32_t timeoutStart = millis();
  int32_t timeoutDuration = 5000; 
  int32_t responseReceivedTimestamp = 0;
  bool responseReceived = false;

  while (millis() - timeoutStart < timeoutDuration)
  {
    if (Serial1.available())
    {
      if (!responseReceived)
      {
        responseReceived = true;
        responseReceivedTimestamp = millis();
      }

      #if DEBUG
        DEBUG_SERIAL.print((char)Serial1.read());
      #else
        Serial1.read();
      #endif
    }

    if (responseReceived && millis() - responseReceivedTimestamp >= 200)
    {
      break; // Break the loop if a response was received and 200ms have passed
    }
  }

  DEBUG_SERIAL.println("print/wait reply end");
}


//maybe the issue is that we are sending a radio sleep too quickly after
void radio_sleep()
{
  Serial1.begin(9600);
  while (!Serial1) {
    ;
  }
  print_reply_wait();
  Serial1.print("AT+SLEEP=ON\r\n");
  delay(5);
  print_reply_wait();
  Serial1.end();
}



uint32_t read_accel()
{
  #if ACCELa
    RunningMedian x_samples1 = RunningMedian(NUM_READINGS_accel);
    RunningMedian y_samples1 = RunningMedian(NUM_READINGS_accel);
    RunningMedian z_samples1 = RunningMedian(NUM_READINGS_accel);

    //Read the sensor NUM_READINGS times with READING_DELAY between each reading
    for (int i = 0; i < NUM_READINGS_accel; i++)
    {
      sensors_event_t accel;
      mpu1_accel->getEvent(&accel);
      x_samples1.add(accel.acceleration.x);
      y_samples1.add(accel.acceleration.y);
      z_samples1.add(accel.acceleration.z);
      delay(5);
    }
    float ampX = (((int)(1000*x_samples1.getHighest()))/1000.0) - (((int)(1000*x_samples1.getLowest()))/1000.0);
    float ampY = (((int)(1000*y_samples1.getHighest()))/1000.0) - (((int)(1000*y_samples1.getLowest()))/1000.0);
    float ampZ = (((int)(1000*z_samples1.getHighest()))/1000.0) - (((int)(1000*z_samples1.getLowest()))/1000.0);
    int accelMag = (ampX*20)+(ampY*20)+(ampZ*20);
    if (accelMag <=5) //noise floor
    {
      accelMag = 0;
    }
    DEBUG_SERIAL.print("accelMag 1 = ");
    DEBUG_SERIAL.println(accelMag);
    return accelMag;
  #else
    return 0;
  #endif
}

uint32_t read_accelb()
{
  #if ACCELb
    RunningMedian x_samples2 = RunningMedian(NUM_READINGS_accel);
    RunningMedian y_samples2 = RunningMedian(NUM_READINGS_accel);
    RunningMedian z_samples2 = RunningMedian(NUM_READINGS_accel);

    //Read the sensor NUM_READINGS times with READING_DELAY between each reading
    for (int i = 0; i < NUM_READINGS_accel; i++)
    {
      sensors_event_t accel2;
      mpu2_accel->getEvent(&accel2);
      x_samples2.add(accel2.acceleration.x);
      y_samples2.add(accel2.acceleration.y);
      z_samples2.add(accel2.acceleration.z);
      delay(10);
    }
    float ampX = (((int)(1000*x_samples2.getHighest()))/1000.0) - (((int)(1000*x_samples2.getLowest()))/1000.0);
    float ampY = (((int)(1000*y_samples2.getHighest()))/1000.0) - (((int)(1000*y_samples2.getLowest()))/1000.0);
    float ampZ = (((int)(1000*z_samples2.getHighest()))/1000.0) - (((int)(1000*z_samples2.getLowest()))/1000.0);
    int accelMag = (ampX*20)+(ampY*20)+(ampZ*20);
    if (accelMag <=5) //noise floor
    {
      accelMag = 0;
    }
    DEBUG_SERIAL.print("accelMag 2 = ");
    DEBUG_SERIAL.println(accelMag);
    return accelMag;
  #else
    return 0;
  #endif
}


uint32_t mcp3008_read(uint8_t channel) {
  uint8_t tx_buf[3];
  uint8_t rx_buf[3];
  uint32_t reading = 0;
  uint32_t adc_val = 0;
  uint8_t idx;
  delay(10);
  /* Start bit is 1 */
  tx_buf[0] = 1;
  /* single bit mode is selected */
  tx_buf[1] = 0x80 | (channel << 4);
  /* Configure CS pin as output */
  pinMode(MCP3008_CS_PIN, OUTPUT);
  /* Setup SPI with mode 0, msb first and clock as 2mhz */
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  // throw out first sample //
  digitalWrite(MCP3008_CS_PIN, LOW);
  SPI.transfer(tx_buf, rx_buf, 3);
  digitalWrite(MCP3008_CS_PIN, HIGH);
  /* Sample n times to get better result */
  for (idx = 0; idx < MCP3008_MAX_SAMPLE; idx++) {
    digitalWrite(MCP3008_CS_PIN, LOW);

    SPI.transfer(tx_buf, rx_buf, 3);

    digitalWrite(MCP3008_CS_PIN, HIGH);
    reading = ((rx_buf[1] & 0x07) << 8) | rx_buf[2];
    DEBUG_SERIAL.print("****   ");
    DEBUG_SERIAL.print(reading);
    DEBUG_SERIAL.println("   ****");
    adc_val += reading;
    delay(1);
  }
  DEBUG_SERIAL.print("new avg: ");
  DEBUG_SERIAL.println(adc_val/MCP3008_MAX_SAMPLE);
  /* Close spi to reduce power consumption */
  SPI.end();
  return (adc_val / MCP3008_MAX_SAMPLE);  
}



int read_rtd() {
  #if RTD
    mcpIO.pinMode(RTD_POWER_PIN, OUTPUT);
    mcpIO.digitalWrite(RTD_POWER_PIN, HIGH);
    delay(50);

    Adafruit_MAX31865 thermo = Adafruit_MAX31865(RTD_CS);

    if (RTDWIRES == 2)
    {
      thermo.begin(MAX31865_2WIRE);
      DEBUG_SERIAL.println("2 wire RTD setup");
    }
    if (RTDWIRES == 3)
    {
      thermo.begin(MAX31865_3WIRE);
      DEBUG_SERIAL.println("3 wire RTD setup");
    }
    float rtd_float = thermo.temperature(RTDNOMINAL, RTDREF);
    // int rtd_resistance = thermo.readRTD();
    DEBUG_SERIAL.print("float: ");
    DEBUG_SERIAL.println(rtd_float);
    int rtd = round(rtd_float);
    DEBUG_SERIAL.print("int: ");
    DEBUG_SERIAL.println(rtd);
    // DEBUG_SERIAL.print("thermo.resistance: ");
    // DEBUG_SERIAL.println(rtd_resistance);
    uint8_t fault = thermo.readFault();
    if (fault) {
      DEBUG_SERIAL.print("Fault 0x"); DEBUG_SERIAL.println(fault, HEX);
      if (fault & MAX31865_FAULT_HIGHTHRESH) {
        DEBUG_SERIAL.println("RTD High Threshold");
      }
      if (fault & MAX31865_FAULT_LOWTHRESH) {
        DEBUG_SERIAL.println("RTD Low Threshold");
      }
      if (fault & MAX31865_FAULT_REFINLOW) {
        DEBUG_SERIAL.println("REFIN- > 0.85 x Bias");
      }
      if (fault & MAX31865_FAULT_REFINHIGH) {
        DEBUG_SERIAL.println("REFIN- < 0.85 x Bias - FORCE- open");
      }
      if (fault & MAX31865_FAULT_RTDINLOW) {
        DEBUG_SERIAL.println("RTDIN- < 0.85 x Bias - FORCE- open");
      }
      if (fault & MAX31865_FAULT_OVUV) {
        DEBUG_SERIAL.println("Under/Over voltage");
      }
      thermo.clearFault();
      mcpIO.digitalWrite(RTD_POWER_PIN, LOW);
      mcpIO.pinMode(RTD_POWER_PIN, INPUT);
      SPI.end();
      pinMode(RTD_CS, INPUT_PULLUP);
      return -200; // unrealistic low temperature to flag error (broken probe, etc)
    } else {
      mcpIO.digitalWrite(RTD_POWER_PIN, LOW);
      mcpIO.pinMode(RTD_POWER_PIN, INPUT);
      SPI.end();
      pinMode(RTD_CS, INPUT);
      pinMode(MCP3008_CS_PIN, INPUT_PULLUP);
      return rtd; // Return the RTD value if there's no fault
    }
  #else
    return 0;
  #endif
}



int read_thermo() {
  #if THERMO

    DEBUG_SERIAL.print("Cold Junction Temp: ");
    DEBUG_SERIAL.println(maxthermo.readCJTemperature());
    
    DEBUG_SERIAL.print("Thermocouple Temp: ");
    DEBUG_SERIAL.println(maxthermo.readThermocoupleTemperature());
    int thermo = maxthermo.readThermocoupleTemperature();
    // Check and print any faults
    uint8_t fault = maxthermo.readFault();
    if (fault) {
      if (fault & MAX31856_FAULT_CJRANGE) DEBUG_SERIAL.println("Cold Junction Range Fault");
      if (fault & MAX31856_FAULT_TCRANGE) DEBUG_SERIAL.println("Thermocouple Range Fault");
      if (fault & MAX31856_FAULT_CJHIGH)  DEBUG_SERIAL.println("Cold Junction High Fault");
      if (fault & MAX31856_FAULT_CJLOW)   DEBUG_SERIAL.println("Cold Junction Low Fault");
      if (fault & MAX31856_FAULT_TCHIGH)  DEBUG_SERIAL.println("Thermocouple High Fault");
      if (fault & MAX31856_FAULT_TCLOW)   DEBUG_SERIAL.println("Thermocouple Low Fault");
      if (fault & MAX31856_FAULT_OVUV)    DEBUG_SERIAL.println("Over/Under Voltage Fault");
      if (fault & MAX31856_FAULT_OPEN)    DEBUG_SERIAL.println("Thermocouple Open Fault");
    }
    delay(1000);

    return thermo; // Return the RTD value if there's no fault
  
  #else
    return 0;
  #endif
}


int read_lux() {
  #if LUX
    veml.enable(true);
    delay(50);
    float rawLux = veml.readLux();
    DEBUG_SERIAL.print("raw lux: ");
    DEBUG_SERIAL.println(rawLux);
    int lux= round(rawLux/10.0);
    if (lux > 999) lux = 999;
    DEBUG_SERIAL.print("posted lux: ");
    DEBUG_SERIAL.println(lux);
    veml.powerSaveEnable(true);
    veml.enable(false);
    return lux;
  #else
    return 0;
  #endif
}


void clear_array(int32_t arr[], size_t length) {
  for (size_t i = 0; i < length; i++) {
    arr[i] = 0;
  }
}

void clear_char_array(char arr[], size_t length) {
  for (size_t i = 0; i < length; i++) {
    arr[i] = 0;
  }
}


int read_temp()
{
  DEBUG_SERIAL.println("reading temp");
  int temp_int;
  #if AMBIENT
    float avg;
    int samples = 3;
    sensors_event_t humidity, temp;
    for (int i = 0; i < samples; i++)
    {
      sht4.getEvent(&humidity, &temp);
      DEBUG_SERIAL.println(temp.temperature);
      avg += temp.temperature;
      delay(10);
    }
    float temp_float = avg/samples;
    DEBUG_SERIAL.print("pre int temp: ");
    DEBUG_SERIAL.println(temp_float);
    temp_int = static_cast<int>((temp_float >= 0 ? temp_float * 10 + 0.5 : temp_float * 10 - 0.5)); //divide by 10 later
    if (temp_int > 999) temp_int = 999;
  #endif
  DEBUG_SERIAL.print("int temp: ");
  DEBUG_SERIAL.println(temp_int);
  return temp_int;
}

int read_humid()
{
  int humid_int;
  #if AMBIENT
    float avg;
    int samples = 3;
    sensors_event_t humidity, temp;
    for (int i = 0; i < samples; i++)
    {
      sht4.getEvent(&humidity, &temp);
      avg += humidity.relative_humidity;
      delay(10);
    }
    float humid_float = avg/samples;
    humid_int = static_cast<int>(humid_float * 10 + 0.5) % 1000; //divide by 10 later
    if (humid_int > 999) humid_int = 999;
    
  #endif
  return humid_int;
}

//helper function to search for keywords from radio response
int find_substr(const char str[], const char substr[]) {
  int str_len = strlen(str);
  int substr_len = strlen(substr);

  if (substr_len == 0) {
    return 0;
  }

  for (int i = 0; i <= str_len - substr_len; ++i) {
    int j;
    for (j = 0; j < substr_len; ++j) {
      if (str[i + j] != substr[j]) {
        break;
      }
    }
    if (j == substr_len) {
      return i;
    }
  }

  return -1;
}

//converts binary to hex
void toHexString(char *inputArray, char *outputArray) {
  if (strlen(inputArray) < 100) {
    Serial.println("Error: Input array must have at least 200 items for processing.");
    return;
  }
  // Hexadecimal map
  char hexMap[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
  unsigned int maxIndex = 100; // Only process 200 elements
  unsigned int j = 0;

  for (unsigned int i = 0; i < maxIndex; i += 4, ++j) {
    // Convert 4-bit sequence to integer
    int val = 8 * (inputArray[i] - '0') + 4 * (inputArray[i + 1] - '0') + 2 * (inputArray[i + 2] - '0') + (inputArray[i + 3] - '0');
    // Map integer to hexadecimal character
    outputArray[j] = hexMap[val];
  }
  outputArray[j] = '\0'; // Null-terminate the output string
}




void compressArray(char *inputArray, char *outputArray, unsigned int size) {
  int compressedSize = compressedDoorArray_SIZE-1;
  int step = size / compressedSize;  // Changed from 200 to 50
  for (unsigned int i = 0; i < compressedSize; ++i) {  // Changed from 200 to 50
    outputArray[i] = '0';
    for (unsigned int j = 0; j < step; ++j) {
      if (inputArray[i * step + j] == '1') {
        outputArray[i] = '1';
        break;
      }
    }
  }
  outputArray[compressedSize+1] = '\0';  // Null-terminate within array bounds
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Setup ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setup()
{
  // If reset reason is wdg, print debug message
  if ((readResetReason() & POWER_RESETREAS_DOG_Msk) == 
        POWER_RESETREAS_DOG_Msk) {
    DEBUG_SERIAL.print("watchdog Reset");
  }
  /* Configure wdg to reset after 30s */
  wdg_config(30);
  flashTransport.begin();
  flashTransport.runCommand(FLASH_DEEP_SLEEP_CMD); //put flash memory into deep sleep
  flashTransport.end();
  initUART();
  init_IO_expander();
  #if AMBIENT
    initAmbient();
  #endif
  #if FIRSTBOOT
    initRadio();
    delay(1000000);
  #endif
  #if ACCELa
    init_accelerometer();
  #endif
  #if LUX
    init_lux();
  #endif
  #if THERMO
    maxthermo.begin();
    maxthermo.setThermocoupleType(MAX31856_TCTYPE_T);
  #endif
  delay(5000);
  radio_sleep();
  init_globals();
  SPI.end();
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Loop ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void loop()
{
  currentTime = millis();
  DEBUG_SERIAL.print("Loop Start - Running Time: ");
  DEBUG_SERIAL.print((currentTime - startTime) / 1000);
  DEBUG_SERIAL.println(" seconds");
  #if DOOR
    if (doorCheckCounter < 28800)
    {
      doorRead = mcp3008_read(DOOR_ADC);
      if (doorRead <= 500)
      {
        doorCountArray[doorCheckCounter] = '1';
      } else {
        doorCountArray[doorCheckCounter] = '0';
      }
    }
    doorCheckCounter++;
  #endif
  if (millis() - sensorCheckTime >= sensorCheckInterval)
  {
    DEBUG_SERIAL.println("reading sensors");
    sensor_read();
  }
  if (packetType == 0 && packetFull == true)
  {
    DEBUG_SERIAL.println("attempt send for pack 0 triggered");
    DEBUG_SERIAL.print("now - lastSent ==  ");
    DEBUG_SERIAL.print((millis() - lastSendAttemptTime) / 1000);
    DEBUG_SERIAL.println(" seconds");
    DEBUG_SERIAL.print("packet Full =  ");
    DEBUG_SERIAL.println(packetFull);
    FlashLED(LED_GREEN, 1, 500);
    attempt_send();
  }

  if (packetType == 1 && (millis() - lastSendAttemptTime > DEFAULT_PACKET_TIME))
  {
    DEBUG_SERIAL.println("attempt send for pack 1 triggered");
    DEBUG_SERIAL.print("now - lastSent ==  ");
    DEBUG_SERIAL.print((millis() - lastSendAttemptTime) / 1000);
    DEBUG_SERIAL.println(" seconds");
    DEBUG_SERIAL.print("packet Full =  ");
    DEBUG_SERIAL.println(packetFull);
    FlashLED(LED_GREEN, 1, 500);
    attempt_send();
  }

  wdg_feed(); //feed watchdog
  DEBUG_SERIAL.print("loop done, sleeping for: ");
  DEBUG_SERIAL.print(MCU_SLEEP_TIME);
  DEBUG_SERIAL.println(" milliseconds");
  FlashLED(LED_BLUE, 1, 500);
  delay(MCU_SLEEP_TIME);
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  LOGIC FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


void sensor_read()
{
  sensorCheckTime = millis();
  bool rtdERR = false;
  #if RTD
    int ch1 = read_rtd();
    if (ch1 == -200)
    {
      rtdERR = true;
      DEBUG_SERIAL.print("RTD ERROR");
    }
    DEBUG_SERIAL.print("RTD Raw reading: ");
    DEBUG_SERIAL.println(ch1);
    if (ch1 < -200) ch1 = -200;
    ch1 = ch1+200; //add 200 for encoding
    int ch2 = 0;
  #elif ACCELa
      int ch1 = read_accel();
    #if ACCELb
      int ch2 = read_accelb();
    #else
      int ch2 = 0;
    #endif
  #elif LUX
    int ch1 = read_lux();
    int ch2 = 0;
  #elif THERMO
    DEBUG_SERIAL.println("Reading thermo");
    int ch1 = read_thermo();
    if (ch1 < -200) ch1 = -200;
    ch1 = ch1+200;
    int ch2 = 0;
  #else
    int ch1 = mcp3008_read(CH1_ADC);
    int ch2 = mcp3008_read(CH2_ADC);
  #endif
  DEBUG_SERIAL.print("ch1 ");
  DEBUG_SERIAL.println(ch1);
  DEBUG_SERIAL.print("ch2 ");
  DEBUG_SERIAL.println(ch2);
  //validate to ensure vals are not larger than 999
  if (ch1 > 999) ch1 = 999;
  if (ch1 < -999) ch1 = -999;
  if (ch2 > 999) ch2 = 999;
  if (ch2 < -999) ch2 = -999;
  //store sensor data
  if (packetType == 0)
  {
    DEBUG_SERIAL.print("last index written was: ");
    DEBUG_SERIAL.println(lastIndexWritten);
    //-2 because we start with index 0, and we want this to set the packet full flag before it writes to the last index
    //If we want 20 values sent, we want this to flag at 20-2. Because it starts at 0, so that is 19 values, and we are writing the 20th now.
    if (lastIndexWritten == CH_PACK0_SIZE-2)
    {
      packetFull = true;
      DEBUG_SERIAL.println("packet 0 is full");
    }
    if (lastIndexWritten == CH_PACK0_SIZE-1)
    {
      //failsafe to ensure we don't overfill the packet
      DEBUG_SERIAL.println("packet 0 is full, can't write any more values");
    } else { //otherwise write to packet as usual
      ch1_pack0[lastIndexWritten + 1] = ch1;
      ch2_pack0[lastIndexWritten + 1] = ch2;
      lastIndexWritten += 1;
    }
  } else { //packet 1
    float avgAsFloat;
    int avgAsInt;
    //min,max,average,sampleSize, first sample, last sample,minIndex,maxIndex
    //channel 1
    if (ch1 < ch1_pack1[0])
    {
      ch1_pack1[0] = ch1;
      ch1_pack1[6] = ch1_pack1[3]+1; //where did min occur? (previous sample size +1)
      if (ch1_pack1[6] > 99999) ch1_pack1[6] = 99999; //ensure it does not overflow past 99999
    }
    if (ch1 > ch1_pack1[1])
    {
      ch1_pack1[1] = ch1;
      ch1_pack1[7] = ch1_pack1[3]+1;
      if (ch1_pack1[7] > 99999) ch1_pack1[7] = 99999;
    }
    //convert average to an int, and make sure it does not overflow past 9999
    avgAsFloat = ch1_pack1[2]+(((ch1*10)-ch1_pack1[2])/(ch1_pack1[3]+1));
    avgAsInt = static_cast<int>((avgAsFloat >= 0 ? avgAsFloat + 0.5 : avgAsFloat - 0.5)); // divide by 10 later
    if (avgAsInt > 9999) avgAsInt = 9999;    
    if (avgAsInt < -9999) avgAsInt = -9999;
    ch1_pack1[2] = avgAsInt;
    ch1_pack1[3] = ch1_pack1[3]+1;
    if (ch1_pack1[3] > 99999) ch1_pack1[3] = 99999;
    ch1_pack1[5] = ch1;
    //channel 2
    if (ch2 < ch2_pack1[0])
    {
      ch2_pack1[0] = ch2;
      ch2_pack1[6] = ch2_pack1[3]+1;
      if (ch2_pack1[6] > 99999) ch2_pack1[6] = 99999;
    }
    if (ch2 > ch2_pack1[1])
    {
      ch2_pack1[1] = ch2;
      ch2_pack1[7] = ch2_pack1[3]+1;
      if (ch2_pack1[7] > 99999) ch2_pack1[7] = 99999;
    }
    avgAsFloat = ch2_pack1[2]+(((ch2*10)-ch2_pack1[2])/(ch2_pack1[3]+1));
    avgAsInt = static_cast<int>((avgAsFloat >= 0 ? avgAsFloat + 0.5 : avgAsFloat - 0.5)); // divide by 10 later
    if (avgAsInt > 9999) avgAsInt = 9999;
    ch2_pack1[2] = avgAsInt;
    ch2_pack1[3] = ch2_pack1[3]+1;
    if (ch2_pack1[3] > 99999) ch2_pack1[3] = 99999;
    ch2_pack1[5] = ch2;
    #if DEBUG
      DEBUG_SERIAL.println("ch1 packet is:");
      for (int i = 0; i < CH_PACK1_SIZE; i++) 
      {
        Serial.print(ch1_pack1[i]);
        if (i < CH_PACK1_SIZE - 1) {
          Serial.print(", ");
        } else {
          Serial.println();
        }
      }
    #endif
  }
}



void update_pack1_ambient()
{
    float avgAsFloat;
    int avgAsInt;
    int sensor_temperature = read_temp();
    int sensor_humidity = read_humid();
    DEBUG_SERIAL.print("sensor_temperature ");
    DEBUG_SERIAL.println(sensor_temperature);
    DEBUG_SERIAL.print("sensor_humidity ");
    DEBUG_SERIAL.println(sensor_humidity);
    //min,max,average,sampleSize, first sample, last sample
    //temperature
    if (sensor_temperature < temp_pack1[0])
    {
      temp_pack1[0] = sensor_temperature;
      temp_pack1[6] = temp_pack1[3]+1; //update where the min measurement occurred
      if (temp_pack1[6] > 99999) temp_pack1[6] = 99999; //ensure it does not overflow
    }
    if (sensor_temperature > temp_pack1[1])
    {
      temp_pack1[1] = sensor_temperature;
      temp_pack1[7] = temp_pack1[3]+1; //update where the max measurement occurred
      if (temp_pack1[7] > 99999) temp_pack1[7] = 99999;
    }
    //convert average to an int, and make sure it does not overflow past 9999
    avgAsFloat = temp_pack1[2]+((sensor_temperature-temp_pack1[2])/(temp_pack1[3]+1));
    avgAsInt = static_cast<int>((avgAsFloat >= 0 ? avgAsFloat + 0.5 : avgAsFloat - 0.5)); //divide by 10 later
    if (avgAsInt > 9999) avgAsInt = 9999;
    temp_pack1[2] = avgAsInt;
    temp_pack1[3] = temp_pack1[3]+1;
    if (temp_pack1[3] > 99999) temp_pack1[3] = 99999;
    temp_pack1[5] = sensor_temperature;
    //humidity
    if (sensor_humidity < humid_pack1[0])
    {
      humid_pack1[0] = sensor_humidity;
      humid_pack1[6] = humid_pack1[3]+1; //update where the min measurement occurred
      if (humid_pack1[6] > 99999) humid_pack1[6] = 99999;
    }
    if (sensor_humidity > humid_pack1[1])
    {
      humid_pack1[1] = sensor_humidity;
      humid_pack1[7] = humid_pack1[3]+1;//update where the max measurement occurred
      if (humid_pack1[7] > 99999) humid_pack1[7] = 99999;
    }
    //convert average to an int, and make sure it does not overflow past 9999
    avgAsFloat = humid_pack1[2]+((sensor_humidity-humid_pack1[2])/(humid_pack1[3]+1));
    avgAsInt = static_cast<int>((avgAsFloat >= 0 ? avgAsFloat + 0.5 : avgAsFloat - 0.5)); //divide by 10 later
    if (avgAsInt > 9999) avgAsInt = 9999;
    humid_pack1[2] = avgAsInt;
    humid_pack1[3] = humid_pack1[3]+1;
    if (humid_pack1[3] > 99999) humid_pack1[3] = 99999;
    humid_pack1[5] = sensor_humidity;
}


void attempt_send()
{
  bool preFlight = false;
  int counter = 0;
  bool packetSuccess;
  Serial1.begin(9600);
  while (!Serial1) {
    ;
  }
  delay(10);
  Serial1.print("AT+SLEEP=OFF\r\n");
  //delay(1000); //allow radio to wake
  print_reply_wait();
  while (true) {
    if (packet_preflight())
    {
      DEBUG_SERIAL.println("packet pre-flight passed");
      preFlight = true;
      break;
    } else {
      if (counter >= 3)
      {
        DEBUG_SERIAL.println("packet pre-flight failed");
        break;
      } 
    }
    counter += 1;
    wdg_feed(); //feed watchdog
  }
  if (preFlight)
  {
    //preflight passed
    
    wdg_feed(); //feed watchdog
    if (packetType == 0)
    {
      DEBUG_SERIAL.println("packet pre-flight passed - sending packet 0");
      packetSuccess = send_packet_0();
      DEBUG_SERIAL.print("packet success 1: ");DEBUG_SERIAL.println(packetSuccess);
      if (packetSuccess == false)
      {
        DEBUG_SERIAL.println("packet 0 ACK failed, sending again");
        delay(100);
        wdg_feed(); //feed watchdog
        packetSuccess = send_packet_0();
        DEBUG_SERIAL.print("packet success 2: ");DEBUG_SERIAL.println(packetSuccess);
      }
    } else {
      DEBUG_SERIAL.println("packet pre-flight passed - sending packet 1");
      packetSuccess = send_packet_1();
    }
    DEBUG_SERIAL.print("packet success 4: ");DEBUG_SERIAL.println(packetSuccess);
    if (packetSuccess)
    {
      DEBUG_SERIAL.println("packet sent, putting radio to sleep");
      radio_sleep();
      clear_array(doorVals, DOORVALS_SIZE);
      clear_array(ch1_pack0, CH_PACK0_SIZE);
      clear_array(ch2_pack0, CH_PACK0_SIZE);
      clear_array(ch1_pack1, CH_PACK1_SIZE);
      clear_array(ch2_pack1, CH_PACK1_SIZE);
      clear_array(temp_pack1, TEMP_PACK1_SIZE);
      clear_array(humid_pack1, HUMID_PACK1_SIZE);
      clear_char_array(doorCountArray, doorCountArray_SIZE);
      clear_char_array(hexArray, hexArray_SIZE);
      clear_char_array(compressedDoorArray, compressedDoorArray_SIZE);
      doorCheckCounter = 0;
      packetFull = false;
      packetType = 0;
      lastIndexWritten = -1;
      packetDurationStart = millis();
      sensorCheckTime = millis();
    } else {
      DEBUG_SERIAL.println("ack failed, putting radio to sleep");
      radio_sleep();
      if (packetType == 0)
      {
        DEBUG_SERIAL.println("starting packet 1 due to ack failure");
        packetFull = false;
        start_pack_1();
      }
    }
  } else {
    //preflight failed
    DEBUG_SERIAL.println("preflight failed, putting radio to sleep");
    radio_sleep();
    if (packetType == 0)
    {
      DEBUG_SERIAL.println("starting packet 1 due to ack failure");
      packetFull = false;
      start_pack_1();
    }
  }
  lastSendAttemptTime = millis();
}


bool packet_preflight()
{
  DEBUG_SERIAL.println("running packet pre-flight");
  if (join_state())
  {
    if (obtain_data_rate()>200)
    {
      join_attempt_qty = 0;
      return true;
    } else {
      if (join_attempt_qty>10)
      {
        DEBUG_SERIAL.println("exceeded join attempts or DR+ attempts");
        join_attempt_qty = 0;
        initRadio_factory();//reboot radio
        DEBUG_SERIAL.println("rebooted radio, reset to factory settings");
        DEBUG_SERIAL.println("attempting to join now");
        Serial1.print("AT+JOINING\r\n");
        //delay(4000);
        print_reply_wait(); //we get two responses quickly, so get both
        print_reply_wait();
        return false;
      } else {
        join_attempt_qty += 1;
        DEBUG_SERIAL.print("incrementing join attempt count, it is now: ");
        DEBUG_SERIAL.println(join_attempt_qty);
        DEBUG_SERIAL.println("sending blank packet");
        Serial1.print("AT+LCR=?\r\n");
        delay(4000);
        print_reply_wait();
        return false;
      }
    }
  } else {
    if (join_attempt_qty>10)
    {
      join_attempt_qty = 0;
      DEBUG_SERIAL.println("exceeded join attempts or DR+ attempts");
      initRadio_factory();//reboot radio
      DEBUG_SERIAL.println("rebooted radio, reset to factory settings");
      DEBUG_SERIAL.println("attempting to join now");
      Serial1.print("AT+JOINING\r\n");
      print_reply_wait(); //we get two responses quickly, so get both
      print_reply_wait();
      return false;
    } else {
      join_attempt_qty += 1;
      DEBUG_SERIAL.print("incrementing join attempt count, it is now: ");
      DEBUG_SERIAL.println(join_attempt_qty);
      DEBUG_SERIAL.println("attempting to join now");
      Serial1.print("AT+JOINING\r\n");
      print_reply_wait(); //we get two responses quickly, so get both
      print_reply_wait();
      return false;
    }
  }
}


bool join_state() {
  DEBUG_SERIAL.println("running join-state()");
  Serial1.print("AT+JOIN_STD=?\r\n"); //get join status
  delay(200); //allow radio to process request
  bool joinedStatus = false;

  const size_t bufSize = 256; // Choose an appropriate buffer size
  char radioResponse[bufSize];
  radioResponse[0] = '\0'; // Initialize radioResponse with an empty string

  while (Serial1.available()) {
    delay(100); //allow serial buffer to fill up //was 500
    DEBUG_SERIAL.println("getting join status response");

    size_t idx = 0;
    while (Serial1.available() && idx < bufSize - 1) {
      radioResponse[idx++] = static_cast<char>(Serial1.read());
    }
    radioResponse[idx] = '\0'; // Null-terminate the string
  }

  if (find_substr(radioResponse, "JOINED") != -1) {
      joinedStatus = true;
      DEBUG_SERIAL.println("Joined!");
  } else if (find_substr(radioResponse, "JOIN FAIL") != -1) {
      DEBUG_SERIAL.println("Join failed!");
      DEBUG_SERIAL.println("/radio response start/");
      DEBUG_SERIAL.print(radioResponse);
      DEBUG_SERIAL.println("/radio response end/");
  } else {
      DEBUG_SERIAL.println("Did not get Join response. we got:");
      DEBUG_SERIAL.println("/radio response start/");
      DEBUG_SERIAL.print(radioResponse);
      DEBUG_SERIAL.println("/radio response end/");
  }

  DEBUG_SERIAL.print("joined status is: ");
  DEBUG_SERIAL.println(joinedStatus);
  return joinedStatus;
}


int obtain_data_rate()
{
  DEBUG_SERIAL.println("Getting data rate...");
  Serial1.print("AT+TX_LEN=?\r\n");
  delay(200); //wait for radio to process request

  const size_t bufSize = 256; // Choose an appropriate buffer size
  char radioResponse[bufSize];
  radioResponse[0] = '\0'; // Initialize radioResponse with an empty string

  while (Serial1.available())
  {
    delay(100); //ensure we have waited long enough for serial buffer to fill //was 500

    size_t idx = 0;
    while (Serial1.available() && idx < bufSize - 1) {
      radioResponse[idx++] = static_cast<char>(Serial1.read());
    }
    radioResponse[idx] = '\0'; // Null-terminate the string
    DEBUG_SERIAL.println(radioResponse);
  }

  int data_rate = 0;
  char *startPtr = radioResponse;
  char *endPtr = nullptr;

  for (size_t i = 0; i < strlen(radioResponse); ++i) {
    if (isdigit(radioResponse[i])) {
      startPtr = &radioResponse[i];
      break;
    }
  }

  data_rate = strtol(startPtr, &endPtr, 10);

  DEBUG_SERIAL.println("Data rate is: ");
  DEBUG_SERIAL.println(data_rate);
  return data_rate;
}


bool send_packet_0()
{
  char payload[200]; // Choose an appropriate buffer size to hold the payload string plus some extra room
  char tempBuff[200]; // Temporary buffer for converting integers to strings
  //calculate duration of packet
  int32_t packetDuration = millis() - packetDurationStart;
  DEBUG_SERIAL.print("millis - packet duration is: ");
  DEBUG_SERIAL.println(packetDuration);

  float roundedValue = round(packetDuration / 1000.0); // Intermediate rounded floating-point value
  int32_t nearestSecond = (int32_t)roundedValue; // Casting to int32_t

  DEBUG_SERIAL.print("nearest second is: ");
  DEBUG_SERIAL.println(nearestSecond);
  if (nearestSecond > 99999) nearestSecond = 99999;
  vBat = 0; 

  #if AMBIENT
    int sensor_temperature = read_temp();
    int sensor_humidity = read_humid();
  #endif
  strcpy(payload, "AT+SEND=7:12:0,"); //add AT command with pack 0 identifier
  //add vBat to payload
  snprintf(tempBuff, sizeof(tempBuff), "%d", vBat);
  strcat(payload, tempBuff);
  strcat(payload, ",");
  //add duration to payload
  snprintf(tempBuff, sizeof(tempBuff), "%d", nearestSecond);
  strcat(payload, tempBuff);
  strcat(payload, ";");
  //add ch1 vals
  for (int i = 0; i < CH_PACK0_SIZE; i++) {
    if(i <= lastIndexWritten && i >= 0)
    {
      snprintf(tempBuff, sizeof(tempBuff), "%03d", ch1_pack0[i]); //%03d adds leading zeros 5 = 005
      strcat(payload, tempBuff);
    }
  }
  strcat(payload, ";");
  //add ch2 vals
  for (int i = 0; i < CH_PACK0_SIZE; i++) {
    if(i <= lastIndexWritten && i >= 0)
    {
      snprintf(tempBuff, sizeof(tempBuff), "%03d", ch2_pack0[i]); //%03d adds leading zeros 5 = 005
      strcat(payload, tempBuff);
    }
  }
  strcat(payload, ";");
  #if AMBIENT
    snprintf(tempBuff, sizeof(tempBuff), "%d", sensor_temperature);
    strcat(payload, tempBuff);
    strcat(payload, ";");
    snprintf(tempBuff, sizeof(tempBuff), "%d", sensor_humidity);
    strcat(payload, tempBuff);
    strcat(payload, ";");
  #else
    strcat(payload, "0;0;");//no ambient, just send zeros
  #endif
  #if DOOR
    //add door vals
    DEBUG_SERIAL.print("doorCountArray: ");
    DEBUG_SERIAL.println(doorCountArray);
    DEBUG_SERIAL.print("doorCheckCounter: ");
    DEBUG_SERIAL.println(doorCheckCounter);
    toHexString(doorCountArray, hexArray); //convert binary states to hex
    DEBUG_SERIAL.print("hexArray: ");
    DEBUG_SERIAL.println(hexArray);
    strcat(payload, hexArray);
    strcat(payload, ";");
    clear_char_array(hexArray, hexArray_SIZE);
  #else
    strcat(payload, "00000000000000000000000000000000000000000000000000"); //hex for all zeros
    strcat(payload, ";");
  #endif
  DEBUG_SERIAL.print("payload pre-send: ");
  DEBUG_SERIAL.println(payload);
  //add payload terminator
  strcat(payload, "\r\n");
  Serial1.print(payload); //send payload
  return recv_ack();
}


bool send_packet_1()
{
  char payload[200]; // Choose an appropriate buffer size to hold the payload string plus some extra space
  char tempBuff[200]; // Temporary buffer for converting integers to strings
  int sensor_temperature;
  int sensor_humidity;
  //calculate duration of packet
  
  int32_t packetDuration = millis() - packetDurationStart;
  int32_t nearestSecond = round(packetDuration / 1000.0);
  if (nearestSecond > 99999) nearestSecond = 99999;
  //validate value size
  ch1_pack1[0] = (ch1_pack1[0] < -999) ? -999 : (ch1_pack1[0] > 999) ? 999 : ch1_pack1[0];
  ch1_pack1[1] = (ch1_pack1[1] < -999) ? -999 : (ch1_pack1[1] > 999) ? 999 : ch1_pack1[1];
  ch1_pack1[2] = (ch1_pack1[2] < -9999) ? -9999 : (ch1_pack1[2] > 9999) ? 9999 : ch1_pack1[2];
  ch1_pack1[3] = (ch1_pack1[3] < -99999) ? -99999 : (ch1_pack1[3] > 99999) ? 99999 : ch1_pack1[3];
  ch1_pack1[4] = (ch1_pack1[4] < -999) ? -999 : (ch1_pack1[4] > 999) ? 999 : ch1_pack1[4];
  ch1_pack1[5] = (ch1_pack1[5] < -999) ? -999 : (ch1_pack1[5] > 999) ? 999 : ch1_pack1[5];
  ch1_pack1[6] = (ch1_pack1[6] < -99999) ? -99999 : (ch1_pack1[6] > 99999) ? 99999 : ch1_pack1[6];
  ch1_pack1[7] = (ch1_pack1[7] < -99999) ? -99999 : (ch1_pack1[7] > 99999) ? 99999 : ch1_pack1[7];
  
  ch2_pack1[0] = (ch2_pack1[0] < -999) ? -999 : (ch2_pack1[0] > 999) ? 999 : ch2_pack1[0];
  ch2_pack1[1] = (ch2_pack1[1] < -999) ? -999 : (ch2_pack1[1] > 999) ? 999 : ch2_pack1[1];
  ch2_pack1[2] = (ch2_pack1[2] < -9999) ? -9999 : (ch2_pack1[2] > 9999) ? 9999 : ch2_pack1[2];
  ch2_pack1[3] = (ch2_pack1[3] < -99999) ? -99999 : (ch2_pack1[3] > 99999) ? 99999 : ch2_pack1[3];
  ch2_pack1[4] = (ch2_pack1[4] < -999) ? -999 : (ch2_pack1[4] > 999) ? 999 : ch2_pack1[4];
  ch2_pack1[5] = (ch2_pack1[5] < -999) ? -999 : (ch2_pack1[5] > 999) ? 999 : ch2_pack1[5];
  ch2_pack1[6] = (ch2_pack1[6] < -99999) ? -99999 : (ch2_pack1[6] > 99999) ? 99999 : ch2_pack1[6];
  ch2_pack1[7] = (ch2_pack1[7] < -99999) ? -99999 : (ch2_pack1[7] > 99999) ? 99999 : ch2_pack1[7];
  vBat = 0; 
  // #if AMBIENT
  //   sensor_temperature = read_temp();
  //   sensor_humidity = read_humid();
  // #endif

  strcpy(payload, "AT+SEND=7:12:1,"); //add AT command with pack 1 identifier
  //add vBat to payload
  snprintf(tempBuff, sizeof(tempBuff), "%d", vBat);
  strcat(payload, tempBuff);
  strcat(payload, ",");
  //add duration to payload
  snprintf(tempBuff, sizeof(tempBuff), "%d", nearestSecond);
  strcat(payload, tempBuff);
  strcat(payload, ";");
  //add ch1 vals
  for (int i = 0; i < CH_PACK1_SIZE; i++) {
    snprintf(tempBuff, sizeof(tempBuff), "%d", ch1_pack1[i]);
    strcat(payload, tempBuff);
    // Add a comma separator between values, except for the last item
    if (i != CH_PACK1_SIZE - 1) {
      strcat(payload, ",");
    }
  }
  strcat(payload, ";"); // Add a semi-colon separator between packet arrays as this is packet 1
  //add ch2 vals
  for (int i = 0; i < CH_PACK1_SIZE; i++) {
    snprintf(tempBuff, sizeof(tempBuff), "%d", ch2_pack1[i]);
    strcat(payload, tempBuff);
      if (i != CH_PACK1_SIZE - 1) {
    strcat(payload, ",");
    }
  }
  strcat(payload, ";");
  #if AMBIENT
  //get latest ambient values
    update_pack1_ambient();
    //add to payload
    for (int i = 0; i < TEMP_PACK1_SIZE; i++) {
      snprintf(tempBuff, sizeof(tempBuff), "%d", temp_pack1[i]);
      strcat(payload, tempBuff);
      if (i != TEMP_PACK1_SIZE - 1) {
        strcat(payload, ",");
      }
    }
    strcat(payload, ";");
    for (int i = 0; i < HUMID_PACK1_SIZE; i++) {
      snprintf(tempBuff, sizeof(tempBuff), "%d", humid_pack1[i]);
      strcat(payload, tempBuff);
      if (i != HUMID_PACK1_SIZE - 1) {
        strcat(payload, ",");
      }
    }
  #else
    strcat(payload, "0,0,0,0,0,0;0,0,0,0,0,0");//no ambient, just send 0
  #endif
  strcat(payload, ";");
  #if DOOR
    //add door vals
    compressArray(doorCountArray, compressedDoorArray, doorCheckCounter);
    toHexString(compressedDoorArray, hexArray); //convert binary states to hex
    strcat(payload, hexArray);
    strcat(payload, ";");
    clear_char_array(hexArray, hexArray_SIZE);
    clear_char_array(compressedDoorArray, compressedDoorArray_SIZE);
  #else
    strcat(payload, "00000000000000000000000000000000000000000000000000"); //hex for all zeros
    strcat(payload, ";");
  #endif
  //add payload terminator
  strcat(payload, "\r\n");
  DEBUG_SERIAL.print("payload pre-send: ");
  DEBUG_SERIAL.println(payload);
  Serial1.print(payload); //send payload
  return recv_ack();
}


bool recv_ack() {
  DEBUG_SERIAL.println("checking ACK response");
  bool errRec = true; // default to error until proven otherwise
  bool response = false; // need to know if we received ERROR or +RECV, or this function won't end.
  int32_t recv_timer = millis();

  const size_t bufSize = 500; // Choose an appropriate buffer size
  char radioResponse[bufSize];
  //delay(500); //! try removing - It appeared to make no difference during 48 hour test
  while (!response && (millis() - recv_timer) <= 20000) { // check for at most 20 seconds for response (was 10 for battery devices, changed to test on OIC vibe sensors)
    radioResponse[0] = '\0'; // Initialize radioResponse with an empty string
    delay(10); //! try removing - It appeared to make no difference during 48 hour test
    if (Serial1.available()) {
      delay(10); //! try changing to 10 (was 300) - It appeared to make no difference during 48 hour test
      size_t idx = 0;
      int32_t read_timeout = millis();
      while (Serial1.available() && (millis() - read_timeout) <= 1000) { //! try increasing to 1000 (was 500) - It appeared to make no difference during 48 hour test
      //while (Serial1.available() && (millis() - read_timeout) <= 1000) {
          if (idx < bufSize - 1)
          {
            radioResponse[idx++] = static_cast<char>(Serial1.read());
          } else {
            DEBUG_SERIAL.println("############## BUFFER OVERFLOW!");
          }
      }
      radioResponse[idx] = '\0'; // Null-terminate the string

      if (find_substr(radioResponse, "RECV") != -1) {
        DEBUG_SERIAL.println("Packet Received!");
        errRec = false;
        response = true;
      } else if (find_substr(radioResponse, "ERROR") != -1) {
        DEBUG_SERIAL.println("Error Received!");
        DEBUG_SERIAL.println("/radio response start/");
        DEBUG_SERIAL.print(radioResponse);
        DEBUG_SERIAL.println("/radio response end/");
        errRec = true;
        response = false;
      }
    }
  }
  wdg_feed(); //feed watchdog
  if (response) {
    DEBUG_SERIAL.print("time taken to get response: ");
    DEBUG_SERIAL.println(millis() - recv_timer);
  } else {
    DEBUG_SERIAL.println("Did not get a response in allotted time");
    DEBUG_SERIAL.println("########################################");
    DEBUG_SERIAL.println("########################################");
    DEBUG_SERIAL.println("########################################");
    DEBUG_SERIAL.println("########################################");
    DEBUG_SERIAL.println("########################################");
    DEBUG_SERIAL.println("########################################");
  }

  if (errRec) {
    DEBUG_SERIAL.println("error sending packet");
    DEBUG_SERIAL.println("########################################");
    DEBUG_SERIAL.println("########################################");
    DEBUG_SERIAL.println("########################################");
    DEBUG_SERIAL.println("########################################");
    DEBUG_SERIAL.println("########################################");
    DEBUG_SERIAL.println("########################################");

  }
  DEBUG_SERIAL.print("packet success 3 (response): ");DEBUG_SERIAL.println(response);
  return response;
}



void start_pack_1()
{
  packetType = 1;
  packetFull = false;
  int32_t ch1_min = 0;
  int32_t ch1_max = 0;
  int32_t ch1_minIndex = 0;
  int32_t ch1_maxIndex = 0;
  float ch1_avg = 0;
  int32_t ch2_min = 0;
  int32_t ch2_max = 0;
  int32_t ch2_minIndex = 0;
  int32_t ch2_maxIndex = 0;
  float ch2_avg = 0;
  int32_t temp_min = 0;
  int32_t temp_max = 0;
  float temp_avg = 0;
  int32_t humid_min = 0;
  int32_t humid_max = 0;
  float humid_avg = 0;
  //convert packet 0 to packet 1, channels 1 to 2
  //min,max,average,sampleSize, first sample, last sample
  for (int i = 0; i < CH_PACK0_SIZE; i++) {
    if(i == 0)
    {
      ch1_min = ch1_pack0[i];
      ch1_max = ch1_pack0[i];
      ch1_avg = ch1_pack0[i]*10;
      ch2_min = ch2_pack0[i];
      ch2_max = ch2_pack0[i];
      ch2_avg = ch2_pack0[i]*10;
    }
    if(i <= lastIndexWritten && i > 0)
    {
      //channel 1
      if (ch1_pack0[i] < ch1_min) {
        ch1_min = ch1_pack0[i];
        ch1_minIndex = i;
      }
      if (ch1_pack0[i] > ch1_max) {
        ch1_max = ch1_pack0[i];
        ch1_maxIndex = i;
      }
      ch1_avg = ch1_avg+(((ch1_pack0[i]*10)-ch1_avg)/(i+1));
      //channel 2
      if (ch2_pack0[i] < ch2_min) {
        ch2_min = ch2_pack0[i];
        ch2_minIndex = i;
      }
      if (ch2_pack0[i] > ch2_max) {
        ch2_max = ch2_pack0[i];
        ch2_maxIndex = i;
      }
      ch2_avg = ch2_avg+(((ch2_pack0[i]*10)-ch2_avg)/(i+1));
    }
  }
  //now write to packet 1 arrays
  //channel 1
  ch1_pack1[0] = ch1_min;
  ch1_pack1[1] = ch1_max;
  //condition ? expression_if_true : expression_if_false;
  //this conditional rounds the float to an int for us.
  ch1_pack1[2] = (ch1_avg > 0) ? static_cast<int32_t>(ch1_avg + 0.5) : static_cast<int32_t>(ch1_avg - 0.5);
  ch1_pack1[3] = lastIndexWritten+1;
  ch1_pack1[4] = ch1_pack0[0];
  ch1_pack1[5] = ch1_pack0[lastIndexWritten];
  ch1_pack1[6] = ch1_minIndex;
  ch1_pack1[7] = ch1_maxIndex;
  //channel 2
  ch2_pack1[0] = ch2_min;
  ch2_pack1[1] = ch2_max;
  ch2_pack1[2] = (ch2_avg > 0) ? static_cast<int32_t>(ch2_avg + 0.5) : static_cast<int32_t>(ch2_avg - 0.5);
  ch2_pack1[3] = lastIndexWritten+1;
  ch2_pack1[4] = ch2_pack0[0];
  ch2_pack1[5] = ch2_pack0[lastIndexWritten];
  ch2_pack1[6] = ch2_minIndex;
  ch2_pack1[7] = ch2_maxIndex;
  #if DEBUG
    DEBUG_SERIAL.println("ch1 packet is:");
    for (int i = 0; i < CH_PACK1_SIZE; i++) {
      Serial.print(ch1_pack1[i]);
      if (i < CH_PACK1_SIZE - 1) {
        Serial.print(", ");
      } else {
        Serial.println();
      }
    }
  #endif
  //start temperature & humidity arrays
  #if AMBIENT
    int sensor_humidity = read_humid();
    int sensor_temperature = read_temp();
    for (int i = 0; i < TEMP_PACK1_SIZE; i++) { 
      temp_pack1[i] = sensor_temperature;
      humid_pack1[i] = sensor_humidity;
    }
    //sample size
    temp_pack1[3] = 1;
    humid_pack1[3] = 1;
    //min/max indexes
    temp_pack1[6] = 1;
    temp_pack1[7] = 1;
    humid_pack1[6] = 1;
    humid_pack1[7] = 1;
  #endif
  //if door sensor exists, the packet stays the same for pack 0 and pack 1
}
