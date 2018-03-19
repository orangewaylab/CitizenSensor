
/*******************************************************************************
   Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
             (c) 2017 Tom Vijlbrief
             (c) 2017,18 Hartmut John

   TTN Stuttgart Version - Hardware Küche #3, 17.05.2017
   works with stm32duiono.com bootloader for generic boards with led on PC13
   aka 'blue pill' stm32f103c8t6
   Importat: USB Serial is not working together with sleep mode

   install adapted STM32 Arduino integration from here 
   https://github.com/tomtor/Arduino_STM32

   optional: install adapted power saving lmic version from here
   https://github.com/tomtor/arduino-lmic
   otherwise you have to comment out skipRX 

   non arduino mbed.org version can be found here
   https://developer.mbed.org/users/orangeway/code/STM32F103C8T6_LoRaWAN-lmic-app/

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   This example sends a valid LoRaWAN packet with static payload,
   using frequency and encryption settings matching those of
   the (early prototype version of) The Things Network.

   Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1, 0.1% in g2).

   ToDo before use:
   - set NWKSKEY (value from console.thethingsnetwork.com)
   - set APPKSKEY (value from console.thethingsnetwork.com)
   - set DEVADDR (value from console.thethingsnetwork.com)
   - optionally comment #define DEBUG
   - optionally comment #define SLEEP


   --- Replace the default on ttn console with this payload decoder 
   --- serving Cayenne integration and generic MQTT ---
   
function Decoder(bytes, port) {
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
  var decoded = {};

  if (port === 1) { // Citizen Sensor first generation
    decoded.temp = (bytes[1]*256+bytes[0])/100.0;
    decoded.pres = (bytes[3]*256+bytes[2])/10.0;
    decoded.humi = (bytes[5]*256+bytes[4])/100.0;
    decoded.power = bytes[6];
    decoded.rate = bytes[7];
  } else if (port === 2) { // Citizen Sensor with static Cayenne support
    decoded.temp = (bytes[2]*256+bytes[3])/10.0;
    decoded.pres = (bytes[9]*256+bytes[10])/10.0;
    decoded.humi = (bytes[6]/2.0);
    decoded.power = 100; // not used
    decoded.rate = 0; // not used
  } 

  return decoded;
}

 *******************************************************************************/

#include <libmaple/iwdg.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define USE_CAYENNE

//PCB Version 2.0 - undefine for PCB Version 4.0 and higher
//#define PCB_VER 20 

// use BME280 Sensor on I2C Pins 
#define USE_SENSOR

// swap ic2 pins SCL/SDA for compatibility with some bme280 breakout boards - not required on PCB >=V4.0 
//#define SWAPE_I2C

// send to single channel gateway? Stay on 868.1MHz but use different SF
#define SINGLE_CHN 1

// show debug statements; comment next line to disable debug statements
#define DEBUG

// Blink a led
#define BLINK

// Enable OTAA? - work in progress
//#define OTAA

// Enable down link - required for OTAA
//#define RECEIVE 1

// use low power sleep - serial debug not working
#define SLEEP
// blue pill board with power led removed + RFM95 + BME280 consumes 370 uA during deep sleep
// RAM is lost and reboots on wakeup.
// We safe some data in the RTC backup ram which survives DeepSleep. 
#define DEEP_SLEEP  true

#ifdef SLEEP
// TODO: should be dynamic - after successfull OTAA deep sleep should be an option
#if DEEP_SLEEP
#undef OTAA
#endif
#endif

#ifndef OTAA
// LoRaWAN NwkSKey, your network session key, 16 bytes (from console.thethingsnetwork.org)
static unsigned char NWKSKEY[16] = { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 };

// LoRaWAN AppSKey, application session key, 16 bytes  (from console.thethingsnetwork.org)
static unsigned char APPSKEY[16] = { 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0 };

// LoRaWAN end-device address (DevAddr), ie 0xABB481F1  (from console.thethingsnetwork.org)

//#include "deviceids.h" - for bulk configuration
#ifdef DEVICEID
static const u4_t DEVADDR = DEVICEID ; // id from config include
#else
static const u4_t DEVADDR = 0x2601FFFF ; // placeholder <-- Change this address for every node!
#endif

#else // if use OTAA (over the air activation) - work in progress
// reversed (LSB) 8 bytes of AppEUI registered with console.thethingsnetwork.org
static const u1_t APPEUI[8] = { 0xEE, 0x32, 0x00, 0xF0, 0x7E, 0xD5, 0xB3, 0x70 }; 
// MSB 16 bytes of the APPKEY used when registering a device with ttnctl register DevEUI AppKey
static const unsigned char APPKEY[16] = { 0x92, 0x5B, 0xD0, 0x03, 0xBC, 0x78, 0x3C, 0xE2, 0x53, 0xA1, 0x17, 0xD0, 0x08, 0x47, 0x83, 0xCF }; 
#endif


// ##### ----------------- board definitions ----------------------- ####

#define led       LED_BUILTIN

// port for RFM95 LoRa Radio
#define USE_SPI   1

//Global Adafruit sensor object
Adafruit_BME280 bme280; 

static const uint16 MAGICNB = 0xbeaf;

// STM32 Unique Chip IDs - used as device id in OTAA scenario
#define STM32_ID	((u1_t *) 0x1FFFF7E8)

SPIClass mySPI(USE_SPI);

extern SPIClass *SPIp;

void blinkN2(int n, int d, int t);

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations). Note that the LED flashing takes some time
int txInterval = 60*5;

#define RATE        DR_SF9

// generic payload object
struct {
  unsigned short temp;
  unsigned short pres;
  unsigned short humi;
  byte power;
  byte rate2;
  
} payload;

// Cayenne Low Power Payload (LPP) object
byte cayenne_lpp[11];  

void initBME280Sparkfun(void);

// Defined for power and sleep functions pwr.h and scb.h
#include <libmaple/pwr.h>
#include <libmaple/scb.h>
#include <libmaple/rcc.h>

#include <RTClock.h>

//RTClock rt(RTCSEL_LSI, 1000); // 10 milli second alarm
RTClock rt(RTCSEL_LSI,33); // 10 milli second alarm
//RTClock rt(RTCSEL_LSE,33); // 10 milli second alarm

// Define the Base address of the RTC registers (battery backed up CMOS Ram), so we can use them for config of touch screen or whatever.
// See http://stm32duino.com/viewtopic.php?f=15&t=132&hilit=rtc&start=40 for a more details about the RTC NVRam
// 10x 16 bit registers are available on the STM32F103CXXX more on the higher density device.
#define BKP_REG_BASE   ((uint32_t *)(0x40006C00 +0x04))

void iwdg_feed_debug(){
  //iwdg_feed();
  blinkN2(1,20,0);
}

void storeBR(int i, uint32_t v) {
  BKP_REG_BASE[2 * i] = (v << 16);
  BKP_REG_BASE[2 * i + 1] = (v & 0xFFFF);
}

uint32_t readBR(int i) {
  return ((BKP_REG_BASE[2 * i] & 0xFFFF) >> 16) | (BKP_REG_BASE[2 * i + 1] & 0xFFFF);
}

bool next = false;

void enterSleepMode(bool deepSleepFlag)
{
  // Clear PDDS and LPDS bits and wakeup pin and set Clear WUF flag (required per datasheet):
  PWR_BASE->CR &= PWR_CR_LPDS | PWR_CR_PDDS | PWR_CR_CWUF |  PWR_CSR_EWUP;



  // magic https://community.particle.io/t/how-to-put-spark-core-to-sleep-and-wakeup-on-interrupt-signal-on-a-pin/5947/56
  // Enable wakeup pin bit.
  PWR_BASE->CSR |= PWR_CSR_EWUP; 
  PWR_BASE->CR  |= PWR_CR_CWUF;
  SCB_BASE->SCR |= SCB_SCR_SLEEPDEEP;

  // System Control Register Bits. See...
  // http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0497a/Cihhjgdh.html
  if (deepSleepFlag) { // standby mode
    PWR_BASE->CR |= PWR_CR_CWUF; // FIX to save power on subsequent runs too
    // Set PDDS and LPDS bits for standby,
    // Set Power down deepsleep bit.
    PWR_BASE->CR |= PWR_CR_PDDS;
    // Unset Low-power deepsleep.
    PWR_BASE->CR &= ~PWR_CR_LPDS;
  } else {
    adc_disable(ADC1);
    adc_disable(ADC2);
#if STM32_HAVE_DAC
    dac_disable_channel(DAC, 1);
    dac_disable_channel(DAC, 2);
#endif
    //  Unset Power down deepsleep bit.
    PWR_BASE->CR &= ~PWR_CR_PDDS;
    // set Low-power deepsleep.
    PWR_BASE->CR |= PWR_CR_LPDS;
  }

  // Now go into stop mode, wake up on interrupt
  asm("    wfi");

  // Clear SLEEPDEEP bit so we can use SLEEP mode
  SCB_BASE->SCR &= ~SCB_SCR_SLEEPDEEP;
}

uint32 sleepTime;

void AlarmFunction () {
  // We always wake up with the 8Mhz HSI clock!
  // So adjust the clock if needed...

#if F_CPU == 8000000UL
  // nothing to do, using about 8 mA
#elif F_CPU == 16000000UL
  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_2);
#elif F_CPU == 48000000UL
  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_6);
#elif F_CPU == 72000000UL
  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_9);
#else
#error "Unknown F_CPU!?"
#endif

  extern volatile uint32 systick_uptime_millis;
  systick_uptime_millis += sleepTime;

}


void blinkN2(int n, int d = 400, int t = 800)
{
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_BUILTIN, 0);
    delay(5);
    digitalWrite(LED_BUILTIN, 1);
    delay(d);
  }
  pinMode(LED_BUILTIN, INPUT_ANALOG);
  delay(t);
}


#ifndef OTAA
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

#else // OTAA

void os_getArtEui (u1_t* buf) {
  memcpy(buf, APPEUI, 8);
}

void os_getDevKey (u1_t* buf) {
  memcpy(buf, APPKEY, 16);

#if 0 // use chip id - not used yet
  u1_t* p = STM32_ID;
  buf[0] = (p[0] & 0x7) + 1;
  buf[1] = (p[1] & 0x7) + 1;
  buf[2] = (p[2] & 0x7) + 1;
  buf[3] = (p[3] & 0x7) + 1;
  buf[4] = (p[4] & 0x7) + 1;
  buf[5] = (p[5] & 0x7) + 1;
  buf[6] = (p[6] & 0x7) + 1;
  buf[7] = (p[7] & 0x7) + 1;
#endif
}

//static const u1_t DEVEUI[8]={}; // for OTAA - reversed 8 bytes of DevEUI registered with console.thethingsnetwork.org 
void os_getDevEui (u1_t* buf) {
  // use chip ID:
  memcpy(buf, &STM32_ID[1], 8);
  // Make locally registered:
  buf[0] = buf[0] & ~0x3 | 0x1;
}
#endif

static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
#if USE_SPI == 1
  .nss = PA4,
  .rxtx = LMIC_UNUSED_PIN,
#if PCB_VER == 20
.rst = PB1,
  .dio = {PB3, PB4, LMIC_UNUSED_PIN} // different IOs on pcb version 2.0
#else // PCB version >2.0
  .rst = PB0,
  .dio = {PA3, PB5, LMIC_UNUSED_PIN} // we dont use dio2
#endif
#else // USE_SPI == 2
  .nss = PB12,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = PA8,
  .dio = {PB1, PB10, PB11}
#endif
};


bool TX_done = false;
bool joined = false;

void onEvent (ev_t ev) {
#ifdef DEBUG
  Serial.println(F("Enter onEvent"));
#endif

  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      joined = true;
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      TX_done = true;
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.dataLen) {
        // data received in rx slot after tx
        Serial.print(F("Data Received: "));
        Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
        Serial.println();
        payload.rate2 = (LMIC.frame + LMIC.dataBeg)[0];
        txInterval = (1 << payload.rate2);
        if (LMIC.dataLen > 1) {
          Serial.print(F("...change SF to: "));
          Serial.println((LMIC.frame + LMIC.dataBeg)[1]);
          switch ((LMIC.frame + LMIC.dataBeg)[1]) {
            case 7: LMIC_setDrTxpow(DR_SF7, 14); break;
            case 8: LMIC_setDrTxpow(DR_SF8, 14); break;
            case 9: LMIC_setDrTxpow(DR_SF9, 14); break;
            case 10: LMIC_setDrTxpow(DR_SF10, 14); break;
            case 11: LMIC_setDrTxpow(DR_SF11, 14); break;
            case 12: LMIC_setDrTxpow(DR_SF12, 14); break;
          }
        }
      }
      // Schedule next transmission
#ifndef SLEEP
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(txInterval), do_send);
#else
      next = true; // handling of going to deep sleep in loop()
#endif

      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
#ifdef DEBUG
  Serial.println(F("Leave onEvent"));
#endif

}

void do_send(osjob_t* j) {

#ifdef DEBUG
  Serial.println(F("Enter do_send"));
#endif

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    readData();
#ifdef SLEEP
    // Disable link check validation
    LMIC_setLinkCheckMode(0);
#endif
    // Prepare upstream data transmission at the next possible time.
#ifdef USE_CAYENNE
    Serial.println(F("cayenne payload queued"));
    LMIC_setTxData2(2, (unsigned char *)&cayenne_lpp, sizeof(cayenne_lpp), 0);
#else
    Serial.println(F("generic payload queued"));
    LMIC_setTxData2(1, (unsigned char *)&payload, sizeof(payload), 0);
#endif
  }
  // Next TX is scheduled after TX_COMPLETE event.
#ifdef DEBUG
  Serial.println(F("Leave do_send"));
#endif
  TX_done = false;

}

void readData()
{
  adc_enable(ADC1);

  adc_reg_map *regs = ADC1->regs;
  regs->CR2 |= ADC_CR2_TSVREFE; // enable VREFINT and temp sensor
  regs->SMPR1 = (ADC_SMPR1_SMP17 /* | ADC_SMPR1_SMP16 */); // sample rate for VREFINT ADC channel

  delay(50);

  int vref = 1200 * 4096 / adc_read(ADC1, 17); // ADC sample to millivolts
  regs->CR2 &= ~ADC_CR2_TSVREFE; // disable VREFINT and temp sensor

  adc_disable(ADC1);

  vref += 5;

  /**
  if (vref < 2000 || vref >= 3000)
    blinkN(vref / 1000);
  
  blinkN(vref % 1000 / 100);
  blinkN(vref % 100 / 10);
  **/
  
  payload.power = (vref / 10) - 200;


#ifdef USE_SENSOR

  delay(100); // give BME some time FIXME

  bme280.takeForcedMeasurement();
  payload.temp= int(bme280.readTemperature() * 100.00F);
  payload.pres= int(bme280.readPressure() / 10.0F);
  payload.humi= int(bme280.readHumidity() * 100.00F);
   
  cayenne_lpp[0] = 0x01;  // channel
  cayenne_lpp[1] = 0x67;  // Chayenne Type "Temp"
  cayenne_lpp[2] = ((int((payload.temp / 10))) >> 8) & 255;
  cayenne_lpp[3] = (int((bme280.readTemperature() * 10))) & 255;
  cayenne_lpp[4] = 0x02;  // channel
  cayenne_lpp[5] = 0x68;  // Cayenne Type "Humidity"         
  cayenne_lpp[6] = (int(round(payload.humi / 50))) & 255;
  cayenne_lpp[7] = 0x03;  // Channel
  cayenne_lpp[8] = 0x73;  // Cayenne Type "Pressure"
  cayenne_lpp[9] = ((int(payload.pres)) >> 8) & 255;
  cayenne_lpp[10] = (int(payload.pres)) & 255;

  pinMode(PB6, INPUT_ANALOG); //SCL save energy
  pinMode(PB7, INPUT_ANALOG); //SDA save energy

#else // simulate sensor
  payload.temp= (19.15) * 100.00;  
  payload.humi= (52.45) * 100.00;
  payload.pres= (1096) * 1;
  cayenne_lpp[0] = 0x01;  // channel
  cayenne_lpp[1] = 0x67;  // Chayenne Type "Temp"
  cayenne_lpp[2] = 0x00;
  cayenne_lpp[3] = 0xFF;
  cayenne_lpp[4] = 0x02;  // channel
  cayenne_lpp[5] = 0x68;  // Cayenne Type "Humidity"        
  cayenne_lpp[6] = 0xFF;
  cayenne_lpp[7] = 0x03;  // Channel
  cayenne_lpp[8] = 0x73;  // Cayenne Type "Pressure"
  cayenne_lpp[9] = 0x00;
  cayenne_lpp[10] = 0xFF;
#endif

#ifdef DEBUG
  Serial.print("temp: ");
  Serial.println(payload.temp);
  Serial.print("humi: ");
  Serial.println(payload.humi); // Serial.println(mySensor.readFloatHumidity());
  Serial.print("pres: ");
  Serial.println(payload.pres);  // Serial.println(  Serial.println(payload.pres));
  Serial.println();
#endif

}

void allInput()
{
  adc_disable(ADC1);
  adc_disable(ADC2);

  pinMode(PA0, INPUT_ANALOG);
  pinMode(PA1, INPUT_ANALOG);
  pinMode(PA2, INPUT_ANALOG);
  pinMode(PA3, INPUT_ANALOG);
  pinMode(PA4, INPUT_ANALOG);
  pinMode(PA5, INPUT_ANALOG);
  pinMode(PA6, INPUT_ANALOG);
  pinMode(PA7, INPUT_ANALOG);
  pinMode(PA8, INPUT_ANALOG);
  pinMode(PA9, INPUT_ANALOG);
  pinMode(PA10, INPUT_ANALOG);
  
  pinMode(PA11, INPUT_ANALOG);
  pinMode(PA12, INPUT_ANALOG);
  pinMode(PA13, INPUT_ANALOG);
  pinMode(PA14, INPUT_ANALOG);
  pinMode(PA15, INPUT_ANALOG);

  pinMode(PB0, INPUT_ANALOG);
  pinMode(PB1, INPUT_ANALOG);
  pinMode(PB2, INPUT_ANALOG);
  pinMode(PB3, INPUT_ANALOG);
  pinMode(PB4, INPUT_ANALOG);
  pinMode(PB5, INPUT_ANALOG);
  pinMode(PB6, INPUT_ANALOG);
  pinMode(PB7, INPUT_ANALOG);
  pinMode(PB8, INPUT_ANALOG);
  pinMode(PB9, INPUT_ANALOG);
  pinMode(PB10, INPUT_ANALOG);
  pinMode(PB11, INPUT_ANALOG);
  pinMode(PB12, INPUT_ANALOG);
  pinMode(PB13, INPUT_ANALOG);
  pinMode(PB14, INPUT_ANALOG);
  pinMode(PB15, INPUT_ANALOG);
}

void scanI2C() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  Wire.begin();

  nDevices = 0;
  for(address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.

    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.println(address, HEX);

      if (address == 1) {
        Serial.print("Hint: unknown device - missing pullups on i2c bus?");
      }

      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.println(address, HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("done");
}


/***** SETUP
 *  
 */
void setup() {

// as soon as possible
rt.createAlarm(&AlarmFunction, rt.getTime() + txInterval*1000);

// some bme280 breakouts need software patch for hardware difference
#ifdef SWAPE_I2C
  Wire.scl_pin=PB7;
  Wire.sda_pin=PB6;
#endif

  SPIp = &mySPI;

  pinMode(led, OUTPUT);
#ifdef DEBUG
  digitalWrite(led, LOW);
  delay(20);
  digitalWrite(led, HIGH);
#endif

  Serial.begin(115200);
  Serial.println(F("Wait 1sec"));
  delay(1000); // give usb port the chance to be regognized by host driver 

  allInput(); // save energy

  int cnt = 0; 
  Serial.println(F("wait 3sec for USB before going to sleep."));
  while (cnt++ < 3) {
    Serial.print(F("."));
    digitalWrite(led, LOW);
    delay(10);
    digitalWrite(led, HIGH);
    delay(290);
  }
  Serial.println(F("go"));
  delay(100);

#ifdef BLINK
    digitalWrite(led, HIGH);
    pinMode(led, INPUT_ANALOG);
 #endif

 #ifdef DEBUG
    Serial.print("LMIC.seqnoUp: ");
    Serial.println(LMIC.seqnoUp);
 #endif

    if (DEEP_SLEEP) {
       if (readBR(2) != MAGICNB) {
         LMIC.seqnoUp = 0;
         storeBR(0, LMIC.seqnoUp);
         storeBR(2, MAGICNB);
       }
    }

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

// only to test power mode - not used
#ifdef TESTSLEEP
while(true) {
    SPIp->end();

    pinMode(PB6, INPUT_ANALOG); //SCL
    pinMode(PB7, INPUT_ANALOG); //SDA

    digitalWrite(PA5, LOW); // SCK
    pinMode(PA5, OUTPUT);

    digitalWrite(PA7, LOW); // MOSI
    pinMode(PA7, OUTPUT);

    pinMode(PA6, INPUT_ANALOG); // MISO

    digitalWrite(lmic_pins.nss, LOW); // NSS
    pinMode(lmic_pins.nss, OUTPUT);

    // DIO Inputs
    pinMode(lmic_pins.dio[0], INPUT_ANALOG);
    pinMode(lmic_pins.dio[1], INPUT_ANALOG);
    pinMode(lmic_pins.dio[2], INPUT_ANALOG);

    pinMode(lmic_pins.rst, INPUT_ANALOG);

    // Serial
    pinMode(PA9, INPUT_ANALOG);
    pinMode(PA10, INPUT_ANALOG);
    
    enterSleepMode(DEEP_SLEEP);
}
#endif

#ifndef OTAA
  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#ifdef SINGLE_CHN
  // only to support non standard single channel geteways 
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_disableChannel(1);
  LMIC_disableChannel(2);
  LMIC_disableChannel(3);
  LMIC_disableChannel(4);
  LMIC_disableChannel(5);
  LMIC_disableChannel(6);
  LMIC_disableChannel(7);
  LMIC_disableChannel(8);
#else 
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#endif

#if F_CPU == 8000000UL
  // HSI is less accurate
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
#endif

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;
  
  Serial.print("LMIC.dn2Freq: ");
  Serial.println(LMIC.dn2Freq/1000000.0);
  
  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(RATE, 14);

  if (DEEP_SLEEP)
    LMIC.seqnoUp = readBR(0);

#ifdef USE_SENSOR
  bme280.begin(0x77);
  initBME280Adafruit();
#endif

#ifndef RECEIVE
    LMIC.skipRX = 1; // Do NOT wait for downstream data!
#endif

  // Start job
  do_send(&sendjob);

#ifdef DEBUG
  Serial.println(F("Leave setup"));
#endif

}


void loop() {

  if (next == false) { // still in LMIC state maschine eg. wait for sending, RX ...
    
 #ifdef BLINK
    pinMode(led, OUTPUT);
    digitalWrite(led, LOW);
 #endif

    iwdg_feed_debug();
    os_runloop_once();

  } else { // LMIC cycle fnished including receive

 #ifdef BLINK
    digitalWrite(led, HIGH);
    pinMode(led, INPUT_ANALOG);
 #endif

 #ifdef DEBUG
    Serial.print("LMIC.seqnoUp:");
    Serial.println(LMIC.seqnoUp);
 #endif
 
    if (DEEP_SLEEP) // keep seqNo in RTC ram
      storeBR(0, LMIC.seqnoUp);

    SPIp->end();

    pinMode(PB6, INPUT_ANALOG); //SCL
    pinMode(PB7, INPUT_ANALOG); //SDA

    digitalWrite(PA5, LOW); // SCK
    pinMode(PA5, OUTPUT);

    digitalWrite(PA7, LOW); // MOSI
    pinMode(PA7, OUTPUT);

    pinMode(PA6, INPUT_ANALOG); // MISO

    digitalWrite(lmic_pins.nss, LOW); // NSS
    pinMode(lmic_pins.nss, OUTPUT);

    // DIO Inputs
    pinMode(lmic_pins.dio[0], INPUT_ANALOG);
    pinMode(lmic_pins.dio[1], INPUT_ANALOG);
    pinMode(lmic_pins.dio[2], INPUT_ANALOG);

    pinMode(lmic_pins.rst, INPUT_ANALOG);

    // Serial
    pinMode(PA9, INPUT_ANALOG);
    pinMode(PA10, INPUT_ANALOG);

    enterSleepMode(DEEP_SLEEP);

    // should not be reached, board reboots on wakeup
    blinkN2(8,300,100);
  }
}


void initBME280Adafruit(void) {
  
  bme280.setSampling(Adafruit_BME280::MODE_FORCED,
    Adafruit_BME280::SAMPLING_X1, // temperature
    Adafruit_BME280::SAMPLING_X1, // pressure
    Adafruit_BME280::SAMPLING_X1, // humidity
    Adafruit_BME280::FILTER_OFF   );
}

void sleepBME280Adafruit(void) {
  
  bme280.setSampling(Adafruit_BME280::MODE_FORCED,
    Adafruit_BME280::SAMPLING_NONE, // temperature
    Adafruit_BME280::SAMPLING_NONE, // pressure
    Adafruit_BME280::SAMPLING_NONE, // humidity
    Adafruit_BME280::FILTER_OFF   );
}

void PrintHex83(uint8_t *data, uint8_t length) // prints 8-bit data in hex
{
 char tmp[length*2+1];
 byte first ;
 int j=0;
 for (uint8_t i=0; i<length; i++) 
 {
   first = (data[i] >> 4) | 48;
   if (first > 57) tmp[j] = first + (byte)39;
   else tmp[j] = first ;
   j++;

   first = (data[i] & 0x0F) | 48;
   if (first > 57) tmp[j] = first + (byte)39; 
   else tmp[j] = first;
   j++;
 }
 tmp[length*2] = 0;
 Serial.println(tmp);
}

  

