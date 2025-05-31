#ifndef rx5808_h
#define rx5808_h

#include <SPI.h>
#include "driver/spi_common.h"
#include "driver/spi_master.h"

#define rssi_pin  35  //RSSI input

#define SSP       15  //receiver SPI Select pin
#define MOSI_pin  17
#define SCK_pin   2

#define  MOSI_on  gpio_set_level(GPIO_NUM_17, 1) //digitalWrite(MOSI_pin, HIGH)
#define  MOSI_off gpio_set_level(GPIO_NUM_17, 0) //digitalWrite(MOSI_pin, LOW)
#define  SCK_on   gpio_set_level(GPIO_NUM_2, 1) //digitalWrite(SCK_pin, HIGH)
#define  SCK_off  gpio_set_level(GPIO_NUM_2, 0) //digitalWrite(SCK_pin, LOW)
#define  SSP_on   gpio_set_level(GPIO_NUM_15, 1) //digitalWrite(SCK_pin, HIGH)
#define  SSP_off  gpio_set_level(GPIO_NUM_15, 0) //digitalWrite(SCK_pin, LOW)

//calibration stored values
#define EEPROM_ADR_RSSI_MIN_L 2
#define EEPROM_ADR_RSSI_MIN_H 3
#define EEPROM_ADR_RSSI_MAX_L 4
#define EEPROM_ADR_RSSI_MAX_H 5

#define BIN_H 20

#define CHANNEL_MAX_5G 40
#define CHANNEL_MIN_5G 0

#define ADC_MAXIMUM 4095

using pvfCallback = void (*)(void);

// All Channels of the above List ordered by Mhz
const uint8_t channelList[] = {19, 32, 18, 17, 33, 16, 7, 34, 8, 24, 6, 9, 25, 5, 35, 10, 26, 4, 11, 27, 3, 36, 12, 28, 2, 13, 29, 37, 1, 14, 30, 0, 15, 31, 38, 20, 21, 39, 22, 23};

// Channels with their Mhz Values
const uint16_t channelFreqTable[] = {
  // Channel 1 - 8
  5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725, // Band A
  5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866, // Band B
  5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945, // Band E
  5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880, // Band F / Airwave
  5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917  // Raceband
};

const uint8_t channelNames[] = {
  0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8,
  0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8,
  0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8,
  0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8,
  0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8
};

class RX5808{
  public:
    RX5808(SPIClass *vspi_in);
    void init();
    void setFreq(uint32_t freq);
    uint16_t getRssi(uint16_t channel);

    uint16_t scanVec_5G[CHANNEL_MAX_5G];
    uint16_t rssiPin;
    
  private:
    uint16_t _readRSSI(uint8_t measures);
    void RX_write(uint8_t command);
    uint8_t _stop_scan;

byte data0 = 0;
byte data1 = 0;
byte data2 = 0;
byte data3 = 0;

uint16_t rssi_min = 0;
uint16_t rssi_max = 0;

SPISettings Fsettings;
SPIClass * vspi = NULL;
};

#endif
