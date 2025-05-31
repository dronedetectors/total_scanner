
#include "rx5808.h"

#define RSSI_THRESH (scanVec_5G[getMaxPos()]-5) //strong channels are near to the global max

RX5808::RX5808(SPIClass *vspi_in) {
  rssiPin = rssi_pin;
  _stop_scan = 0;

  pinMode(SSP, OUTPUT);
  pinMode(MOSI_pin, OUTPUT);
  pinMode(SCK_pin, OUTPUT);
  analogSetPinAttenuation(rssi_pin, ADC_0db);
}

uint16_t RX5808::getRssi(uint16_t channel) {return scanVec_5G[channel];}

void RX5808::RX_write(uint8_t command){
  uint8_t bit_mask = 0x01u;
  for(uint8_t i=0; i<8; i++){
    if (command & bit_mask){MOSI_on;} else{MOSI_off;}
    SCK_on; SCK_off;
    bit_mask <<= 1u;
  }
}

void RX5808::init() {
  rssi_min = 0;
  rssi_max = 1024;
  //delay(100);
  SSP_off;
  RX_write(0x10);
  RX_write(0x01);
  RX_write(0x00);
  RX_write(0x00);
  SSP_on;
  //delay(100);
}

//set a certain frequency for the RX module
void RX5808::setFreq(uint32_t freq) {
  uint32_t Delitel = (freq - 479) / 2;

  byte DelitelH = Delitel >> 5;
  byte DelitelL = Delitel & 0x1F;

  data0 = DelitelL * 32 + 17;
  data1 = DelitelH * 16 + DelitelL / 8;
  data2 = DelitelH / 16;
  data3 = 0;

  SSP_off;
  RX_write(data0);
  RX_write(data1);
  RX_write(data2);
  RX_write(data3);
  SSP_on;
}

uint16_t RX5808::_readRSSI(uint8_t measures) {
  uint32_t  sum = 0;
  for (uint8_t i = 0; i < measures; i++)
  {
    delay(20);
    sum += analogRead(rssiPin);
  }
  //Serial.print(sum / 10); Serial.print(" ");
  return sum / measures; // average
}
