
// si4432

#ifndef si4432_h
#define si4432_h

#include <SPI.h>
#include "driver/spi_common.h"
#include "driver/spi_master.h"

// SPI definitions and macros
#define SI_SDN_pin   2
#define SI_CS_pin    4
#define SI_MOSI_pin  23
#define SI_MISO_pin  19
#define SI_SCK_pin   18

#define  SICS_on    gpio_set_level(GPIO_NUM_4, 1)
#define  SICS_off   gpio_set_level(GPIO_NUM_4, 0)

#define MAX_CHAN_QTY     128  // max number of channel for spacing 405.456543MHz
#define MAX_RSSI_SI4432  250

typedef enum SI4432_REGISTERS{
		REG_DEV_TYPE = 0x00,
		REG_DEV_VERSION = 0x01,
		REG_DEV_STATUS = 0x02,

		REG_INT_STATUS1 = 0x03,
		REG_INT_STATUS2 = 0x04,
		REG_INT_ENABLE1 = 0x05,
		REG_INT_ENABLE2 = 0x06,
		REG_STATE = 0x07,
		REG_OPERATION_CONTROL = 0x08,

		REG_GPIO0_CONF = 0x0B,
		REG_GPIO1_CONF = 0x0C,
		REG_GPIO2_CONF = 0x0D,
		REG_IOPORT_CONF = 0x0E,

		REG_IF_FILTER_BW = 0x1C,
		REG_AFC_LOOP_GEARSHIFT_OVERRIDE = 0x1D,
		REG_AFC_TIMING_CONTROL = 0x1E,
		REG_CLOCK_RECOVERY_GEARSHIFT = 0x1F,
		REG_CLOCK_RECOVERY_OVERSAMPLING = 0x20,
		REG_CLOCK_RECOVERY_OFFSET2 = 0x21,
		REG_CLOCK_RECOVERY_OFFSET1 = 0x22,
		REG_CLOCK_RECOVERY_OFFSET0 = 0x23,
		REG_CLOCK_RECOVERY_TIMING_GAIN1 = 0x24,
		REG_CLOCK_RECOVERY_TIMING_GAIN0 = 0x25,
		REG_RSSI = 0x26,
		REG_RSSI_THRESHOLD = 0x27,

		REG_AFC_LIMITER = 0x2A,
		REG_AFC_CORRECTION_READ = 0x2B,

		REG_DATAACCESS_CONTROL = 0x30,
		REG_EZMAC_STATUS = 0x31,
		REG_HEADER_CONTROL1 = 0x32,
		REG_HEADER_CONTROL2 = 0x33,
		REG_PREAMBLE_LENGTH = 0x34,
		REG_PREAMBLE_DETECTION = 0x35,
		REG_SYNC_WORD3 = 0x36,
		REG_SYNC_WORD2 = 0x37,
		REG_SYNC_WORD1 = 0x38,
		REG_SYNC_WORD0 = 0x39,
		REG_TRANSMIT_HEADER3 = 0x3A,
		REG_TRANSMIT_HEADER2 = 0x3B,
		REG_TRANSMIT_HEADER1 = 0x3C,
		REG_TRANSMIT_HEADER0 = 0x3D,

		REG_PKG_LEN = 0x3E,

		REG_CHECK_HEADER3 = 0x3F,
		REG_CHECK_HEADER2 = 0x40,
		REG_CHECK_HEADER1 = 0x41,
		REG_CHECK_HEADER0 = 0x42,

		REG_RECEIVED_HEADER3 = 0x47,
		REG_RECEIVED_HEADER2 = 0x48,
		REG_RECEIVED_HEADER1 = 0x49,
		REG_RECEIVED_HEADER0 = 0x4A,

		REG_RECEIVED_LENGTH = 0x4B,

		REG_CHARGEPUMP_OVERRIDE = 0x58,
		REG_DIVIDER_CURRENT_TRIM = 0x59,
		REG_VCO_CURRENT_TRIM = 0x5A,

		REG_AGC_OVERRIDE = 0x69,

		REG_TX_POWER = 0x6D,
		REG_TX_DATARATE1 = 0x6E,
		REG_TX_DATARATE0 = 0x6F,

		REG_MODULATION_MODE1 = 0x70,
		REG_MODULATION_MODE2 = 0x71,

		REG_FREQ_DEVIATION = 0x72,
		REG_FREQ_OFFSET1 = 0x73,
		REG_FREQ_OFFSET2 = 0x74,
		REG_FREQBAND = 0x75,
		REG_FREQCARRIER_H = 0x76,
		REG_FREQCARRIER_L = 0x77,

		REG_FREQCHANNEL = 0x79,
		REG_CHANNEL_STEPSIZE = 0x7A,

		REG_FIFO = 0x7F,

	};

typedef enum AntennaMode {
		RXMode = 0x04, TXMode = 0x08, Ready = 0x01, TuneMode = 0x02
	};
	
/*
#define delayMicroseconds(microsec) esp_rom_delay_us(microsec)
#define delay(millsec) esp_rom_delay_us(millsec*1000)
#define millis() xTaskGetTickCount()*portTICK_PERIOD_MS
*/

//************************************* class **************************************************//
class si4432
{
public:
  si4432(SPIClass *vspi_in);
  void init(void);
  void BurstWrite(uint8_t startReg, const byte value[], uint8_t length);
  void BurstRead(uint8_t startReg, byte value[], uint8_t length);
  byte ReadRegister(uint8_t reg);
  void ChangeRegister(uint8_t reg, byte value);
  
  void Reset();
  
  void setFrequency(unsigned long baseFrequencyMhz);
  void setBaudRate(uint16_t kbps);
  void setChannel(byte channel);
  void setCommsSignature(uint16_t signature);
  void switchMode(byte mode);
  void startListening();
  void clearTxFIFO();
  void clearRxFIFO();
  void readAll();
  
  void scan(SemaphoreHandle_t mutex, uint16_t ms);
  void scan(uint16_t ms);
  void init_scan(float sfmin, float sfmax, float ssteps);
  void setStrength(uint16_t num, int vall);
  int16_t getStrength(uint16_t num);

/////////////////////// SCANNER SETUP ///////////////////////

float steps_cnt; //scanner steps

private:
int16_t si4432rssiARR[MAX_CHAN_QTY];
float scnFmin; //300.0; //378.0; //779.0; //900.00
float scnFmax; //348.0; //480.0; //899.0; //928.00
float stepVal; //scanner step value

uint64_t _freqCarrier = 433;
uint8_t _freqChannel = 0;
uint16_t _kbps = 70;
uint16_t _packageSign;

SPISettings CCsettings;
SPIClass *vspi;
};

#endif
