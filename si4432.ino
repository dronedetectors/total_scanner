#include "si4432.h"

//values here are kept in khz x 10 format (for not to deal with decimals) - look at AN440 page 26 for whole table
const uint16_t IFFilterTable[][2] = { { 322, 0x26 }, { 3355, 0x88 }, { 3618, 0x89 }, { 4202, 0x8A }, { 4684, 0x8B }, {
		5188, 0x8C }, { 5770, 0x8D }, { 6207, 0x8E } };
		
si4432::si4432(SPIClass *vspi_in){
CCsettings = SPISettings(5000000, MSBFIRST, SPI_MODE0);

pinMode(SI_SDN_pin, OUTPUT); gpio_set_level(GPIO_NUM_2, 0); //SDN ON

  pinMode(SI_SCK_pin, OUTPUT);
  pinMode(SI_MISO_pin, INPUT);
  pinMode(SI_MOSI_pin, OUTPUT);
  pinMode(SI_CS_pin, OUTPUT);
  
  vspi = vspi_in;
  SICS_on;
}

void si4432::Reset(){
  int count = 0;
  // always perform a system reset (don't send 0x87) 
  ChangeRegister(REG_STATE, 0x80); //SI4432_Write_Byte( 0x07, 0x80);
  /*
  	byte regVal = (byte) startReg | 0x80; // set MSB
	SICS_off;
	vspi->beginTransaction(CCsettings);
	vspi->transfer(REG_STATE);
	vspi->transfer(value[i]);
	vspi->endTransaction();
	SICS_on;
	*/
  delay(10);
  // wait for chiprdy bit
  while (( ReadRegister(REG_INT_STATUS2) & 0x02 ) == 0){
    delay(100);
    Serial.print(ReadRegister(REG_INT_STATUS2));
    Serial.print(" - ");
    Serial.println("Waiting for SI4432 ");
  }
}

void si4432::init(void){

	Reset();

	byte currentFix[] = { 0x80, 0x40, 0x7F };
	BurstWrite(REG_CHARGEPUMP_OVERRIDE, currentFix, 3); // refer to AN440 for reasons
/*
	ChangeRegister(REG_GPIO0_CONF, 0x0F); // tx/rx data clk pin
	ChangeRegister(REG_GPIO1_CONF, 0x00); // POR inverted pin
	ChangeRegister(REG_GPIO2_CONF, 0x1C); // clear channel pin
	*/

	ChangeRegister(REG_AFC_TIMING_CONTROL, 0x02); // refer to AN440 for reasons
	ChangeRegister(REG_AFC_LIMITER, 0xFF); // write max value - excel file did that.
	ChangeRegister(REG_AGC_OVERRIDE, 0x60); // max gain control
	ChangeRegister(REG_AFC_LOOP_GEARSHIFT_OVERRIDE, 0x3C); // turn off AFC

	//ChangeRegister(REG_TX_POWER, 0x1F); // max power
	ChangeRegister(REG_CHANNEL_STEPSIZE, 0x64); // each channel is of 1 Mhz interval

	setFrequency(_freqCarrier); // default freq
	setBaudRate(_kbps); // default baud rate is 100kpbs
	setChannel(_freqChannel); // default channel is 0
	setCommsSignature(_packageSign); // default signature

	switchMode(Ready);
  
}

void si4432::ChangeRegister(uint8_t reg, byte value) {
	BurstWrite(reg, &value, 1);

	// reg = 0x07 and value = 0x80 is Software Register Reset Bit.
	// This bit will be automatically cleared.
	//if (reg == 0x07 && value == 0x80) return;
	//byte _value;
	//BurstRead(reg, &_value, 1);
	/*
	if (value != _value) {
		ESP_LOGE(TAG, "ChangeRegister Fail");
		ESP_LOGE(TAG,"reg=0x%x value=0x%x", reg, value);
	}
	*/
}

void si4432::BurstWrite(uint8_t startReg, const byte value[], uint8_t length) {
	byte regVal = (byte) startReg | 0x80; // set MSB
	SICS_off;
	vspi->beginTransaction(CCsettings);
	vspi->transfer(regVal);
	for (byte i = 0; i < length; ++i) {vspi->transfer(value[i]);}
	vspi->endTransaction();
	SICS_on;
}

void si4432::BurstRead(uint8_t startReg, byte value[], uint8_t length) {
	//byte regVal = (byte) startReg | 0x7F; // set MSB
	byte regVal = (byte) startReg; // | 0x7F; // set MSB
	SICS_off;
	vspi->beginTransaction(CCsettings);
	vspi->transfer(regVal);
	for (byte i = 0; i < length; ++i) {value[i] = vspi->transfer(0xFF);}
	vspi->endTransaction();
	SICS_on;
}

byte si4432::ReadRegister(uint8_t reg) {
	//byte val=0;
	byte val[1];
	BurstRead(reg, val, 1);
	return val[0];
	/*
	byte regVal = (byte) reg | 0x7F; // set MSB
	SICS_off;
	vspi->beginTransaction(CCsettings);
	vspi->transfer(regVal);
	val = vspi->transfer(0xFF);
	vspi->endTransaction();
	SICS_on;
	*/
	//return val;
}

void si4432::setFrequency(unsigned long baseFrequencyMhz) {
	if((baseFrequencyMhz < 240) || (baseFrequencyMhz > 960)) {return;} // invalid frequency

	_freqCarrier = baseFrequencyMhz;
	byte highBand = 0;
	if (baseFrequencyMhz >= 480) {highBand = 1;}

	double fPart = ((double)baseFrequencyMhz / (10 * (highBand + 1))) - 24;
	uint8_t freqband = (uint8_t) fPart; // truncate the int
	uint16_t freqcarrier = (fPart - freqband) * 64000;
	byte vals[3] = { 0x40 | (highBand << 5) | (freqband & 0x3F), freqcarrier >> 8, freqcarrier & 0xFF };	// sideband is always on (0x40) :
	BurstWrite(REG_FREQBAND, vals, 3);
}

void si4432::setBaudRate(uint16_t kbps){
	if ((kbps > 256) || (kbps < 1)){return;}	// chip normally supports very low bps values, but they are cumbersome to implement - so I just didn't implement lower bps values
	_kbps = kbps;

	byte freqDev = kbps <= 10 ? 15 : 150;		// 15khz / 150 khz
	//byte modulationValue = _kbps < 30 ? 0x4c : 0x0c;		// use FIFO Mode, GFSK, low baud mode on / off
	byte modulationValue = _kbps < 30 ? 0x2c : 0x0c;		// use FIFO Mode, GFSK, low baud mode on / off
	byte modulationVals[] = { modulationValue, 0x23, round((freqDev * 1000.0) / 625.0) }; // msb of the kpbs to 3rd bit of register
	BurstWrite(REG_MODULATION_MODE1, modulationVals, 3); // 0x70

	// set data rate
	uint16_t bpsRegVal = round((kbps * (kbps < 30 ? 2097152 : 65536.0)) / 1000.0);
	byte datarateVals[] = { bpsRegVal >> 8, bpsRegVal & 0xFF };
	BurstWrite(REG_TX_DATARATE1, datarateVals, 2); // 0x6E

	//now set the timings
	uint16_t minBandwidth = (2 * (uint32_t) freqDev) + kbps;
	//ESP_LOGD(TAG,"min Bandwidth value: 0x%x", minBandwidth);
	byte IFValue = 0xff;
	//since the table is ordered (from low to high), just find the 'minimum bandwidth which is greater than required'
	for (byte i = 0; i < 8; ++i) {if (IFFilterTable[i][0] >= (minBandwidth * 10)) {IFValue = IFFilterTable[i][1]; break;}}
	//ESP_LOGD(TAG,"Selected IF value: 0x%x", IFValue);

	ChangeRegister(REG_IF_FILTER_BW, IFValue);

	byte dwn3_bypass = (IFValue & 0x80) ? 1 : 0; // if msb is set
	byte ndec_exp = (IFValue >> 4) & 0x07; // only 3 bits
	uint16_t rxOversampling = round((500.0 * (1 + 2 * dwn3_bypass)) / ((pow(2, ndec_exp - 3)) * (double ) kbps));
	uint32_t ncOffset = ceil(((double) kbps * (pow(2, ndec_exp + 20))) / (500.0 * (1 + 2 * dwn3_bypass)));

	uint16_t crGain = 2 + ((65535 * (int64_t) kbps) / ((int64_t) rxOversampling * freqDev));
	byte crMultiplier = 0x00;
	if (crGain > 0x7FF) {crGain = 0x7FF;}
	/*
	ESP_LOGD(TAG,"dwn3_bypass value: 0x%x", dwn3_bypass);
	ESP_LOGD(TAG,"ndec_exp value: 0x%x", ndec_exp);
	ESP_LOGD(TAG,"rxOversampling value: 0x%x", rxOversampling);
	ESP_LOGD(TAG,"ncOffset value: 0x%"PRIu32, ncOffset);
	ESP_LOGD(TAG,"crGain value: 0x%x", crGain);
	ESP_LOGD(TAG,"crMultiplier value: 0x%x", crMultiplier);
	*/

	byte timingVals[] = { rxOversampling & 0x00FF, ((rxOversampling & 0x0700) >> 3) | ((ncOffset >> 16) & 0x0F),(ncOffset >> 8) & 0xFF, ncOffset & 0xFF, ((crGain & 0x0700) >> 8) | crMultiplier, crGain & 0xFF };

	BurstWrite(REG_CLOCK_RECOVERY_OVERSAMPLING, timingVals, 6);

}

void si4432::setChannel(byte channel){ChangeRegister(REG_FREQCHANNEL, channel);}

void si4432::setCommsSignature(uint16_t signature) {
	_packageSign = signature;
	ChangeRegister(REG_TRANSMIT_HEADER3, _packageSign >> 8); // header (signature) byte 3 val
	ChangeRegister(REG_TRANSMIT_HEADER2, (_packageSign & 0xFF)); // header (signature) byte 2 val
	ChangeRegister(REG_CHECK_HEADER3, _packageSign >> 8); // header (signature) byte 3 val for receive checks
	ChangeRegister(REG_CHECK_HEADER2, (_packageSign & 0xFF)); // header (signature) byte 2 val for receive checks
}

void si4432::switchMode(byte mode){ChangeRegister(REG_STATE, mode);}

void si4432::startListening() {
	clearRxFIFO(); // clear first, so it doesn't overflow if packet is big
	ChangeRegister(REG_INT_ENABLE1, 0x03); // set interrupts on for package received and CRC error
	ChangeRegister(REG_INT_ENABLE2, 0x00); // set other interrupts off

	//read interrupt registers to clean them
	ReadRegister(REG_INT_STATUS1);
	ReadRegister(REG_INT_STATUS2);

	switchMode(RXMode | Ready);
}

void si4432::clearTxFIFO() {
	ChangeRegister(REG_OPERATION_CONTROL, 0x01);
	ChangeRegister(REG_OPERATION_CONTROL, 0x00);
}

void si4432::clearRxFIFO() {
	ChangeRegister(REG_OPERATION_CONTROL, 0x02);
	ChangeRegister(REG_OPERATION_CONTROL, 0x00);
}

void si4432::readAll() {
	byte allValues[0x7F];
	BurstRead(REG_DEV_TYPE, allValues, 0x7F);
}

void si4432::init_scan(float sfmin, float sfmax, float ssteps){
scnFmin = sfmin; //240
scnFmax = sfmax; //960
steps_cnt = ssteps; //scanner steps

  stepVal = (scnFmax-scnFmin)/steps_cnt;  //scanner step value
  for(int i=0; i<ssteps; i++){si4432rssiARR[i]=0;}
}

void si4432::scan(SemaphoreHandle_t mutex, uint16_t ms){
for(uint8_t i=0; i<(int)steps_cnt; i++){
  if(stepVal <= 1){
  xSemaphoreTake(mutex, SEM_TIMEOUT);
	setFrequency(scnFmin+(i*stepVal));
  xSemaphoreGive(mutex);
	delayMicroseconds(ms);
  xSemaphoreTake(mutex, SEM_TIMEOUT);
  si4432rssiARR[i] = ReadRegister(REG_RSSI);
  xSemaphoreGive(mutex);
  }else{
    int16_t buffRssi;
    float bfrq_start = scnFmin+(i*stepVal);
    float bfrq_stop = scnFmin+((i+1)*stepVal);
    uint8_t steps_cnt = (uint8_t)(bfrq_stop-bfrq_start)+1;
    float bfrq_step = (bfrq_start-bfrq_stop)/steps_cnt;
    for(uint8_t k=0; k<steps_cnt; k++){
      xSemaphoreTake(mutex, SEM_TIMEOUT);
      setFrequency(bfrq_start+(k*bfrq_step));
      xSemaphoreGive(mutex);
      delayMicroseconds(ms);
      xSemaphoreTake(mutex, SEM_TIMEOUT);
      buffRssi = ReadRegister(REG_RSSI);
      if(buffRssi > si4432rssiARR[i]){si4432rssiARR[i]=buffRssi;}
      xSemaphoreGive(mutex);
      }
  }
}
}

void si4432::scan(uint16_t ms){
for(uint8_t i=0; i<(int)steps_cnt; i++){
  setFrequency(scnFmin+(i*stepVal));
  delayMicroseconds(ms);
  si4432rssiARR[i] = ReadRegister(REG_RSSI);
}
}

int16_t si4432::getStrength(uint16_t num){return si4432rssiARR[num];} //strength
void si4432::setStrength(uint16_t num, int vall){si4432rssiARR[num] = vall;}

//si4432 si4432;
