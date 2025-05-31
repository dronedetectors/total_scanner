
#include "nrf24l01.h"

//#define RSSI_THRESH (scanVec[getMaxPos()]-5) //strong CHANNELS_2G are near to the global max

// nRF24 Register map

nrf24l01::nrf24l01(SPIClass *vspi_in) {
  pinMode(NMOSI_pin, OUTPUT);
  pinMode(NSCK_pin, OUTPUT);
  pinMode(NCS_pin, OUTPUT);
  pinMode(NCE_pin, OUTPUT);
  pinMode(NMISO_pin, INPUT);
  NCS_on; //CS_on;
  Nsettings = SPISettings(5000000, MSBFIRST, SPI_MODE0);
  vspi = vspi_in;
  //vspi = new SPIClass(VSPI);
  //vspi->begin(NSCK_pin, NMISO_pin, NMOSI_pin, NCS_pin);
}

void nrf24l01::init() {
  delay(70);
  NCS_off; //CS_off;
  CE_off;
  MOSI_off;
  SCK_off;
  delay(100);
  NCS_on; //CS_on;
  delay(10);

  NRF24L01_Reset();
  delay(100);

  NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0b01110011); //CRC off
  NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);     // switch off Shockburst mode
  NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, 0b01010110); //250kbps 0dbm
  //NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, 0b00001110); //2mbps 0dbm
  //NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x00); //illegal addres
  
  for(uint8_t i = 0; i < 6; ++i) {NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0+i, 32);} //setPayloadSize
  NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x00); //setChannel
  _spi_write_address_double(NRF24L01_0B_RX_ADDR_P1 | W_REGISTER, 0xAA, 0x00);
  NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0b00000011);
  

  NRF24L01_SetTxRxMode(RX_EN);
}

void nrf24l01::_spi_write_address(uint8_t address, uint8_t data){
  NCS_off; //CS_off;
  vspi->beginTransaction(Nsettings);
  vspi->transfer(address);
  vspi->transfer(data);
  vspi->endTransaction();
  NCS_on; //CS_on;
}

void nrf24l01::_spi_write_address_double(uint8_t address, uint8_t dataA, uint8_t dataB){
  NCS_off; //CS_off;
  vspi->beginTransaction(Nsettings);
  vspi->transfer(address);
  vspi->transfer(dataA);
  vspi->transfer(dataB);
  vspi->endTransaction();
  NCS_on; //CS_on;
}

uint8_t nrf24l01::_spi_read_address(uint8_t address){
  uint8_t result;
  NCS_off; //CS_off;
  vspi->beginTransaction(Nsettings);
  vspi->transfer(address);
  result = vspi->transfer(0xFF);
  vspi->endTransaction();
  NCS_on; //CS_on;
  return (result);
}

uint8_t nrf24l01::Strobe(uint8_t state){
  uint8_t result;
  NCS_off; //CS_off;
  vspi->beginTransaction(Nsettings);
  result = vspi->transfer(state);
  vspi->endTransaction();
  NCS_on; //CS_on;
  return result;
}

uint8_t nrf24l01::NRF24L01_WriteReg(uint8_t address, uint8_t data){
  _spi_write_address(address | W_REGISTER, data); 
  return 1;
  }

uint8_t nrf24l01::NRF24L01_FlushTx(){return Strobe(FLUSH_TX);}
uint8_t nrf24l01::NRF24L01_FlushRx(){return Strobe(FLUSH_RX);}

uint8_t nrf24l01::NRF24L01_ReadReg(uint8_t reg){return _spi_read_address(reg);}

void nrf24l01::NRF24L01_SetTxRxMode(uint8_t mode){
  if (mode == TX_EN) {
    CE_off;
    NRF24L01_WriteReg(NRF24L01_07_STATUS,
                      (1 << NRF24L01_07_RX_DR)    // reset the flag(s)
                      | (1 << NRF24L01_07_TX_DS)
                      | (1 << NRF24L01_07_MAX_RT));
    NRF24L01_WriteReg(NRF24L01_00_CONFIG,
                      (1 << NRF24L01_00_EN_CRC)   // switch to TX mode
                      | (1 << NRF24L01_00_CRCO)
                      | (1 << NRF24L01_00_PWR_UP));
    delayMicroseconds(130);
    CE_on;
  } else if (mode == RX_EN) {
    CE_off;
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);        // reset the flag(s)
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0b00000011);        // switch to RX mode
    NRF24L01_WriteReg(NRF24L01_07_STATUS,
                      (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                      | (1 << NRF24L01_07_TX_DS)
                      | (1 << NRF24L01_07_MAX_RT));
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG,(1 << NRF24L01_00_PWR_UP) | (1 << NRF24L01_00_PRIM_RX));
    delayMicroseconds(130);
    CE_on;
  } else {
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)); // PowerDown
    CE_off;
  }
}

uint8_t nrf24l01::NRF24L01_Reset(){
  NRF24L01_FlushTx();
  NRF24L01_FlushRx();
  uint8_t status1 = Strobe(0xFF); // NOP
  uint8_t status2 = NRF24L01_ReadReg(0x07);
  //NRF24L01_SetTxRxMode(TXRX_OFF);
  return (status1 == status2 && (status1 & 0x0f) == 0x0e);
}

void nrf24l01::scan(uint16_t step_itterate){

for(uint8_t MHz=0; MHz<CHANNELS_2G; MHz++){
  NRF24L01_WriteReg(NRF24L01_05_RF_CH, MHz);                    // Set new freqency for scan
  signalStrength_2G[MHz] = 0;  
  for(int i=0; i<step_itterate; i++){
  CE_on;   // start receiving
  delayMicroseconds(150);
  CE_off;                                                       // stop receiving - one bit is now set if received power was > -64 dBm at that instant
  if (NRF24L01_ReadReg(NRF24L01_09_CD)) {signalStrength_2G[MHz]++;}
  }
}

}

void nrf24l01::scan(void){
uint16_t nrfMAX = 0;
bool sw_delay = false;

for(MHz=0; MHz<CHANNELS_2G; MHz++){
  NRF24L01_WriteReg(NRF24L01_05_RF_CH, MHz);                    // Set new freqency for scan
  signalStrength_2G[MHz] = 0;  
  CE_on;   // start receiving
  delayMicroseconds(150);
  CE_off;                                                       // stop receiving - one bit is now set if received power was > -64 dBm at that instant
  if (NRF24L01_ReadReg(NRF24L01_09_CD)) {signalStrength_2G[MHz]++;}
}

//CE_off;        // stop receiving
//return 1; //nrfMAX;
}

uint16_t nrf24l01::getStrength(uint16_t num){return signalStrength_2G[num];} //strength
