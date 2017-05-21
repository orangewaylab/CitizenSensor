/*******************************************************************************
   Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
             (c) 2017 Tom Vijlbrief
             (c) 2017 Hartmut John

   TTN Stuttgart Version - Hardware Küche #3, 17.05.2017
   works with stm32duiono.com bootloader for generic boards with led on PC13
   aka 'blue pill' stm32f103c8t6
   Importat: USB Serial is not working together with sleep mode

   install adapted STM32 Arduino integration from here 
   https://github.com/tomtor/Arduino_STM32

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


   --- TTN Payload Decoder ---
    function Decoder(bytes, port) {
      // Decode an uplink message from a buffer
      // (array) of bytes to an object of fields.
      var decoded = {};
    
      if (port === 1) {
        decoded.temp = (bytes[1]*256+bytes[0])/100.0;
        decoded.pres = (bytes[3]*256+bytes[2])/100.0;
        decoded.humi = (bytes[5]*256+bytes[4])/1.0;
        decoded.power = bytes[6];
        decoded.rate = bytes[7];
      }
      return decoded;
    }

 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <Wire.h>

#include "SparkFunBME280.h"

//Global sensor object
BME280 mySensor;

// use BME280 Sensor on I2C Pins ?
#define USE_SENSOR

// send to single channel gateway?
//#define SINGLE_CHN 1

// show debug statements; comment next line to disable debug statements
#define DEBUG

// Enable OTA? - work in progress
//#define OTA

// use low power sleep: 500-700 uA - disable if you need serial debug working
//#define SLEEP

#ifdef SLEEP
// or DeepSleep: 50uA, but RAM is lost and reboots on wakeup.
// We safe some data in the RTC backup ram which survives DeepSleep
#define DEEP_SLEEP  false

#if DEEP_SLEEP
#undef OTA
#endif
#endif

#define led       LED_BUILTIN

// port for RFM95 LoRa Radio
#define USE_SPI   1

#ifndef OTA
// LoRaWAN NwkSKey, your network session key, 16 bytes (from console.thethingsnetwork.org)
static unsigned char NWKSKEY[16] = { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 };

// LoRaWAN AppSKey, application session key, 16 bytes  (from console.thethingsnetwork.org)
static unsigned char APPSKEY[16] = { 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0 };

// LoRaWAN end-device address (DevAddr), ie 0xABB481F1  (from console.thethingsnetwork.org)
static const u4_t DEVADDR = 0x26011F20 ; // <-- Change this address for every node!

#else // if use OTAA (over the air activaton)
// reversed (LSB) 8 bytes of AppEUI registered with console.thethingsnetwork.org
static const u1_t APPEUI[8] = { }; 
// MSB 16 bytes of the APPKEY used when registering a device with ttnctl register DevEUI AppKey
static const unsigned char APPKEY[16] = { }; 
#endif

// STM32 Unique Chip IDs - used as device id in OTAA scenario
#define STM32_ID	((u1_t *) 0x1FFFF7E8)

SPIClass mySPI(USE_SPI);

extern SPIClass *SPIp;

// Blink a led
#define BLINK

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
#ifdef SLEEP
int txInterval = (DEEP_SLEEP ? 300 : 60); // Note that the LED flashing takes some time
#else
int txInterval = 60;
#endif

#define RATE        DR_SF9

struct {
  unsigned short temp;
  unsigned short pres;
  unsigned short humi;
  byte power;
  byte rate2;
  
} mydata;

void initBME280(void);

#ifdef SLEEP

// Defined for power and sleep functions pwr.h and scb.h
#include <libmaple/pwr.h>
#include <libmaple/scb.h>

#include <RTClock.h>

RTClock rt(RTCSEL_LSI, 399); // 10 milli second alarm

// Define the Base address of the RTC registers (battery backed up CMOS Ram), so we can use them for config of touch screen or whatever.
// See http://stm32duino.com/viewtopic.php?f=15&t=132&hilit=rtc&start=40 for a more details about the RTC NVRam
// 10x 16 bit registers are available on the STM32F103CXXX more on the higher density device.
#define BKP_REG_BASE   ((uint32_t *)(0x40006C00 +0x04))

void storeBR(int i, uint32_t v) {
  BKP_REG_BASE[2 * i] = (v << 16);
  BKP_REG_BASE[2 * i + 1] = (v & 0xFFFF);
}

uint32_t readBR(int i) {
  return ((BKP_REG_BASE[2 * i] & 0xFFFF) >> 16) | (BKP_REG_BASE[2 * i + 1] & 0xFFFF);
}

bool next = false;

void sleepMode(bool deepSleepFlag)
{
  // Clear PDDS and LPDS bits
  PWR_BASE->CR &= PWR_CR_LPDS | PWR_CR_PDDS | PWR_CR_CWUF;

  // Set PDDS and LPDS bits for standby mode, and set Clear WUF flag (required per datasheet):
  PWR_BASE->CR |= PWR_CR_CWUF;
  // Enable wakeup pin bit.
  PWR_BASE->CR |=  PWR_CSR_EWUP;

  SCB_BASE->SCR |= SCB_SCR_SLEEPDEEP;

  // System Control Register Bits. See...
  // http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0497a/Cihhjgdh.html
  if (deepSleepFlag) {
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

void mdelay(int n, bool mode = false)
{
  sleepTime = n;
  time_t nextAlarm = (rt.getTime() + n / 10); // Calculate from time now.
  rt.createAlarm(&AlarmFunction, nextAlarm);
  sleepMode(mode);
}

void msleep(uint32_t ms)
{
  uint32_t start = rt.getTime();

  while (rt.getTime() - start < ms) {
    asm("    wfi");
  }
}
#else
void mdelay(int n, bool mode = false)
{
  delay(n);
}
#endif

void blinkN(int n, int d = 400, int t = 800)
{
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_BUILTIN, 0);
    mdelay(5);
    digitalWrite(LED_BUILTIN, 1);
    mdelay(d);
  }
  pinMode(LED_BUILTIN, INPUT_ANALOG);
  mdelay(t);
}


#ifndef OTA
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
#else
void os_getArtEui (u1_t* buf) {
  memcpy(buf, APPEUI, 8);
}

void os_getDevKey (u1_t* buf) {
  memcpy(buf, APPKEY, 16);
#if 0
  // Use human friendly format:
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
  .rst = PB0,
  .dio = {PA3, PB5, LMIC_UNUSED_PIN} // we dont use dio2
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
        mydata.rate2 = (LMIC.frame + LMIC.dataBeg)[0];
        txInterval = (1 << mydata.rate2);
        if (LMIC.dataLen > 1) {
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
      next = true;
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
    LMIC_setTxData2(1, (unsigned char *)&mydata, sizeof(mydata), 0);
    Serial.println(F("Packet queued"));
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

  int vref = 1200 * 4096 / adc_read(ADC1, 17); // ADC sample to millivolts
  regs->CR2 &= ~ADC_CR2_TSVREFE; // disable VREFINT and temp sensor

  adc_disable(ADC1);

  vref += 5;
  if (vref < 2000 || vref >= 3000)
    blinkN(vref / 1000);
  blinkN(vref % 1000 / 100);
  blinkN(vref % 100 / 10);

  mydata.power = (vref / 10) - 200;

#ifdef USE_SENSOR
#ifndef SLEEP
  Serial.println(mySensor.begin(), HEX);
  delay(10); // wait some time
#endif
  mydata.temp= (mySensor.readTempC() * 100.00);  
  mydata.humi= (mySensor.readFloatHumidity() * 100.00);
  mydata.pres= (mySensor.readFloatPressure() * 1);
#ifndef SLEEP
   pinMode(PB6, INPUT_ANALOG); //SCL save energy
   pinMode(PB7, INPUT_ANALOG); //SDA save energy
#endif
#else
  mydata.temp= (21.25) * 100.00;  
  mydata.humi= (43.85) * 100.00;
  mydata.pres= (1096) * 1;
#endif

#ifdef DEBUG
  //Serial.println(v);

  Serial.print("temp: ");
  Serial.println(mydata.temp);
  Serial.print("humi: ");
  Serial.println(mydata.humi);
  Serial.print("pres: ");
  Serial.println(mydata.pres);
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

void setup() {
  allInput();

  SPIp = &mySPI;

  pinMode(led, OUTPUT);
#ifdef DEBUG
  digitalWrite(led, LOW);
  delay(20);
  digitalWrite(led, HIGH);
#endif

  Serial.begin(115200);
  Serial.println(F("Wait 5sec"));
  delay(5000);
  Serial.println(F("go"));

#if 0
  // Show ID in human friendly format (digits 1..8)
  u1_t* p = STM32_ID;
  blinkN((p[0] & 0x7) + 1);
  blinkN((p[1] & 0x7) + 1);
  blinkN((p[2] & 0x7) + 1);
  blinkN((p[3] & 0x7) + 1);
  blinkN((p[4] & 0x7) + 1);
  blinkN((p[5] & 0x7) + 1);
  blinkN((p[6] & 0x7) + 1);
  blinkN((p[7] & 0x7) + 1);
#endif

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

#ifndef OTA
  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
#ifdef SINGLE_CHN
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

#ifndef OTA
  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(RATE, 14);
#endif

#ifdef SLEEP
  if (DEEP_SLEEP)
    LMIC.seqnoUp = readBR(0);

#if defined(OTA) && DEEP_SLEEP
#error "DEEP_SLEEP and OTA cannot be combined!"
#endif
#endif

#ifdef USE_SENSOR
  initBME280();
#endif

  // Start job
  do_send(&sendjob);

#ifdef DEBUG
  Serial.println(F("Leave setup"));
#endif
}


void loop() {

#ifndef SLEEP

#ifdef BLINK
  static int count;
  digitalWrite(led,
               ! ((++count < 1000) || !TX_done)
              );
#endif
  os_runloop_once();

#else

#ifdef OTA
  if (!joined) {
    os_runloop_once();
    return;
  }
#endif

  if (next == false) {
    digitalWrite(led, LOW);
    //if (DEEP_SLEEP)
    LMIC.skipRX = 1; // Do NOT wait for downstream data!
    os_runloop_once();

  } else {

#ifdef BLINK
    digitalWrite(led, HIGH);
#endif

#ifdef DEBUG
    Serial.println(LMIC.seqnoUp);
#endif

    if (DEEP_SLEEP)
      storeBR(0, LMIC.seqnoUp);

    SPIp->end();
    //mySensor.end();
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

    mdelay(txInterval * 1000, DEEP_SLEEP);

    Serial.begin(115200);

    extern void hal_io_init();
    digitalWrite(lmic_pins.rst, 1); // prevent reset
    hal_io_init();

    SPIp->begin();
    Serial.println(mySensor.begin(), HEX);

#ifdef DEBUG
    Serial.println(F("Sleep complete"));
#endif
    next = false;
    // Start job
    do_send(&sendjob);
  }
#endif
}

/*
 * WIP: not power optimized
 */
void initBME280(void) {
  
  //*** BME280 Driver settings********************************//
  //specify I2C address.  Can be 0x77(default) or 0x76
  mySensor.settings.commInterface = I2C_MODE;
  mySensor.settings.I2CAddress = 0x76;
    
  //***Operation settings*****************************//
  
  //renMode can be:
  //  0, Sleep mode
  //  1 or 2, Forced mode
  //  3, Normal mode
  mySensor.settings.runMode = 3; //Normal mode
  
  //tStandby can be:
  //  0, 0.5ms
  //  1, 62.5ms
  //  2, 125ms
  //  3, 250ms
  //  4, 500ms
  //  5, 1000ms
  //  6, 10ms
  //  7, 20ms
  mySensor.settings.tStandby = 0;
  
  //filter can be off or number of FIR coefficients to use:
  //  0, filter off
  //  1, coefficients = 2
  //  2, coefficients = 4
  //  3, coefficients = 8
  //  4, coefficients = 16
  mySensor.settings.filter = 0;
  
  //tempOverSample can be:
  //  0, skipped
  //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  mySensor.settings.tempOverSample = 1;

  //pressOverSample can be:
  //  0, skipped
  //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    mySensor.settings.pressOverSample = 1;
  
  //humidOverSample can be:
  //  0, skipped
  //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  mySensor.settings.humidOverSample = 3;

  delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
  Serial.println(mySensor.begin(), HEX);

  Serial.print("Displaying BME280 ID, reset and ctrl regs\n");
  
  Serial.print("ID(0xD0): 0x");
  Serial.println(mySensor.readRegister(BME280_CHIP_ID_REG), HEX);
  Serial.print("Reset register(0xE0): 0x");
  Serial.println(mySensor.readRegister(BME280_RST_REG), HEX);
  Serial.print("ctrl_meas(0xF4): 0x");
  Serial.println(mySensor.readRegister(BME280_CTRL_MEAS_REG), HEX);
  Serial.print("ctrl_hum(0xF2): 0x");
  Serial.println(mySensor.readRegister(BME280_CTRL_HUMIDITY_REG), HEX);

  //randomSeed(analogRead(0));
  
  Serial.println("BME 280 init done");
}

