#include <DNSServer.h>

/*
 * 80 MHz CPU
 * NO OTA 2+2MB
 * голосовая озвучка - https://texttospeech.ru/ (Оксана)
 * сообщение 0: сигнал
 * сообщение 1: "Обнаружен дрон камик+адзе!"
 * сообщение 2: "Обнаружен телеметрический канал!"
 * сообщение 3: "Обнаружены помехи сетевого оборудования!"
 * сообщение 4: "два гигагерца" два <phoneme alphabet="ipa" ph="ɡʲɨɡaɡʲˈert͡sə">гигагерца</phoneme>
 * сообщение 5: "пять гигагерц" пять <phoneme alphabet="ipa" ph="ɡʲɨɡaɡʲˈert͡s">гигагерц</phoneme>
 * сообщение 6: громкая сирена
 * сообщение 7: звук включения
 * сообщение 8:"Низкий заряд батареи!"
 * сообщение 9:"Внимание!"
 * сообщение 10:"девятьсот мегагерц"
 * сообщение 11:"уведомдение"
 * сообщение 12:"РЭБ активирован"
 */

#include <Wire.h>
#include <EEPROM.h>

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#include <rom/rtc.h>

#include "scanner.h"
//#include "arduinoFFT.h"

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

SPIClass *vspi = new SPIClass(VSPI);

static RX5808   rx5808(vspi);  //5.8 GHz scanner
static nrf24l01 nrf24l01(vspi);       //2.4 GHz scanner
static si4432   DRVsi4432(vspi);

uint32_t loop_tick = 0;
uint32_t loop_tick_time_start = 0;

String fname_old;

uint8_t auto_snd_divA = 20;
uint8_t auto_snd_volA = 0;

uint8_t auto_snd_divB = 30;
uint8_t auto_snd_volB = 0;

uint8_t auto_snd_divC = 40;
uint8_t auto_snd_volC = 0;

uint8_t button_state = 0;
bool sens_handle = false;
bool sens_handle_prev = false;
bool menu_handle = false;
bool menu_handle_prev = false;
bool locked_chip = false;

bool battery_low = false;

bool audio_car = false;
bool audio_car_once = false;

uint8_t blink_50ms = 0;

uint32_t b_left_millis = 0;
uint32_t b_right_millis = 0;
uint32_t b_all_millis = 0;

uint32_t display_millis = 0;

SemaphoreHandle_t SPImutex;

volatile bool scan_done_5G = false;
volatile bool scan_done_2G = false;
volatile bool scan_done_SUB1G = false;

bool frq_sw = false;

String chip_SERIAL = "";

/////////// FFT ///////////
/*
const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 128000;
double vReal[samples+1];
double vImag[samples+1];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);
*/
/////////// FFT ///////////

///////////////////////// SETTINGS /////////////////////////
static SETTING dev_settings; //device settings

const char *mainMenu_str[]   =  {"громкость",    //1 (0-100)
                                 "звук повтор",  //2 (0-120)
                                 "АКБ мин",      //3 (3000-4500)
                                 "АКБ макс",     //4 (3000-4500)
                                 "яркость",      //5 (0-100)
                                 "дисп выкл",    //6 (0-120)
                                 "чувств. <1G",    //8 (0-100)
                                 "чувств. 2.4",    //9 (0-100)
                                 "чувств. 5.8",    //10 (0-100)
                                 "длит. <1G",    //12 (0-15)
                                 "длит. 2.4",    //13 (0-15)
                                 "длит. 5.8",    //14 (0-15)
                                 "1G диап MIN",
                                 "1G диап MAX",
                                 "РЭБ период",   //15 (0-60)
                                 "РЭБ длит.",     //16 (0-60)
                                 "сброс настроек"     //16 (0-60)
                                 };

const char *mainMenu_val[]   =  {"%",    //1 (0-100)
                                 "s",  //2 (0-120)
                                 "m",      //3 (3000-4500)
                                 "m",     //4 (3000-4500)
                                 "%",      //5 (0-100)
                                 "s",    //6 (0-120)
                                 "k",    //чувствительность 8 (0-100)
                                 "k",    //чувствительность 9 (0-100)
                                 "k",    //чувствительность 10 (0-100)
                                 "n",    //12 (0-15)
                                 "n",    //13 (0-15)
                                 "n",    //14 (0-15)
                                 "M",
                                 "M",
                                 "s",   //15 (0-60)
                                 "s",     //16 (0-60)
                                 "" //сброс настроек
                                 };

const uint16_t mainMenu_minimum[] = { //maximum val
                                 0,  //"громкость",    //1 (0-100)
                                 0,  //"звук повтор",  //2 (0-120)
                                 3000, //"АКБ мин",      //3 (3000-4500)
                                 3000, //"АКБ макс",     //4 (3000-4500)
                                 0,  //"яркость",      //5 (0-100)
                                 0,  //"дисп выкл",    //6 (0-300)
                                 1,  //"чувствительность <1G",    //8 (0-100)
                                 1,  //"чувствительность 2.4",    //9 (0-100)
                                 1,  //"чувствительность 5.8",    //10 (0-100)
                                 0,   //"длит. <1G",    //12 (0-15)
                                 0,   //"длит. 2.4",    //13 (0-15)
                                 0,   //"длит. 5.8",    //14 (0-15)
                                 250,
                                 260,
                                 0,   //"РЭБ период",   //15 (0-60)
                                 0,    //"РЭБ длит."     //16 (0-60)
                                 0
                                 };
                                 
const uint16_t mainMenu_maximum[]   =  { //maximum val
                                 100,  //"громкость",    //1 (0-100)
                                 150,  //"звук повтор",  //2 (0-120)
                                 4500, //"АКБ мин",      //3 (3000-4500)
                                 4500, //"АКБ макс",     //4 (3000-4500)
                                 100,  //"яркость",      //5 (0-100)
                                 300,  //"дисп выкл",    //6 (0-300)
                                 200,  //"чувствительность <1G",    //8 (0-100)
                                 200,  //"чувствительность 2.4",    //9 (0-100)
                                 200,  //"чувствительность 5.8",    //10 (0-100)
                                 16,   //"длит. <1G",    //12 (0-15)
                                 16,   //"длит. 2.4",    //13 (0-15)
                                 16,   //"длит. 5.8",    //14 (0-15)
                                 950,
                                 960,
                                 60,   //"РЭБ период",   //15 (0-60)
                                 60,    //"РЭБ длит."     //16 (0-60)
                                 1
                                 };

const uint16_t mainMenu_step[]   =  { //maximum val
                                 5,  //"громкость",    //1 (0-100)
                                 5,  //"звук повтор",  //2 (0-120)
                                 100, //"АКБ мин",      //3 (3000-4500)
                                 100, //"АКБ макс",     //4 (3000-4500)
                                 5,  //"яркость",      //5 (0-100)
                                 5,  //"дисп выкл",    //6 (0-300)
                                 1,  //"чувствительность <1G",    //8 (0-100)
                                 1,  //"чувствительность 2.4",    //9 (0-100)
                                 1,  //"чувствительность 5.8",    //10 (0-100)
                                 1,   //"длит. <1G",    //12 (0-15)
                                 1,   //"длит. 2.4",    //13 (0-15)
                                 1,   //"длит. 5.8",    //14 (0-15)
                                 10,
                                 10,
                                 1,   //"РЭБ период",   //15 (0-60)
                                 1,    //"РЭБ длит."     //16 (0-60)
                                 1
                                 };
                                 
uint16_t *mainMenu_par[]= {&dev_settings.sound_volume,  //0
                           &dev_settings.sound_period,  //1
                           &dev_settings.battery_min,   //2
                           &dev_settings.battery_max,   //3
                           &dev_settings.disp_bright,   //4
                           &dev_settings.disp_timeout,  //5
                           &dev_settings.sens_SUB1G, //7
                           &dev_settings.sens_2G,  //8
                           &dev_settings.sens_5G,  //9
                           &dev_settings.duration_SUB1G,  //11
                           &dev_settings.duration_2G,   //12
                           &dev_settings.duration_5G,   //13
                           &dev_settings.SUB1G_scnFmin,
                           &dev_settings.SUB1G_scnFmax,
                           &dev_settings.warfare_sleep_period, //14
                           &dev_settings.warfare_active, //15
                           &dev_settings.reset_settings //16
                           };

#define MENU_SIZE   17

volatile uint8_t waterfall_2G[WTFAL_SIZE];
volatile uint8_t waterfall_5G[WTFAL_SIZE];
volatile uint8_t waterfall_SUB1G[WTFAL_SIZE];

float triggers[4] = {0,0,0,0}; //for trigger function

//////////////////////////////////////////// expRunningAverage
//2G, 5G, 900M, 400M
float filVal[130] = {10, 20, 20, 20, 20}; //init max values

float expRunningAverage(int num, float newVal, float k) {   //digital filter
  filVal[num] += (newVal - filVal[num]) * k;
  return filVal[num];
}

void waterfall_shiftUP(volatile uint8_t *waterfall){ //shift byte array
  for(uint16_t j=WTFALL_H; j>0; j--){
  for(uint16_t i=0; i<WTFALL_L; i++){waterfall[((j-1)*WTFALL_L)+i+WTFALL_L] = waterfall[((j-1)*WTFALL_L)+i];}  //(16-32) = (0-16)
  }
}

void display_center(int y, String s){
  int16_t x1, y1;
  uint16_t w, h;
  
  display.getTextBounds(const_cast<char*>(s.c_str()), 0, 0, &x1, &y1, &w, &h);
  display.setCursor((display.width()-w)/2, y);
  display.print(s);
}

void display_right(int y, String s, uint8_t offsett){
  int16_t x1, y1;
  uint16_t w, h;
  
  display.getTextBounds(const_cast<char*>(s.c_str()), 0, 0, &x1, &y1, &w, &h);
  display.setCursor((display.width()-w-offsett), y);
  display.print(s);
}

void display_left(int y, String s, uint8_t offsett){
  display.setCursor(offsett, y);
  display.print(s);
}

///////////////////// VOICE /////////////////////
hw_timer_t *sound_timer = NULL;
uint8_t *f_arr;
uint32_t f_stack = 0, f_len = 0;
uint8_t group = 0, timer = 0, tchannel = 0;
volatile bool audioPlaying = false;
uint32_t last_playingTime = 0;

uint16_t buzzerSIG = BUZZER_OFF;
uint32_t batt_voltage = 0;

////////////////// INTERRUPTS //////////////////

void IRAM_ATTR soundTimer(){ //wav player timer
if(audioPlaying){
  if(f_len < f_stack){
    LEDC_CHAN(group, tchannel).duty.duty = f_arr[f_len] * (16*dev_settings.sound_volume)/100; //*45;
    LEDC_CHAN(group, tchannel).conf0.sig_out_en = 1;
    LEDC_CHAN(group, tchannel).conf1.duty_start = 1;
    LEDC_CHAN(group, tchannel).conf0.clk_en = 1;
    f_len++;
    }
  else{
    audioPlaying = false; f_len=0; f_stack=0;
    LEDC_CHAN(group, tchannel).duty.duty = 0;
    LEDC_CHAN(group, tchannel).conf0.sig_out_en = 1;
    LEDC_CHAN(group, tchannel).conf1.duty_start = 1;
    LEDC_CHAN(group, tchannel).conf0.clk_en = 1;
    }
}
}

void IRAM_ATTR toggleBUTTON(){ //display timeout reset
  display_millis = millis(); //reset display screensaver timer
}

////////////////// FUNCTIONS //////////////////

void init_sound(){ //init wav player
sound_timer = timerBegin(1, 80, true); //1 MHz timer -> (16kHz)
timerAttachInterrupt(sound_timer, &soundTimer, true);
timerAlarmWrite(sound_timer, 64, true); //125
timerAlarmEnable(sound_timer);

ledcAttachPin(BUZZER_pin, PWM_CH);
ledcSetup(PWM_CH, 44000, 8);
ledcWrite(PWM_CH, 0);
group=(PWM_CH/8);
timer=((PWM_CH/2)%4);
tchannel=(PWM_CH%8);
}

void play_spiffs(String fname){ //wav play
  static String fname_old;
  static uint32_t stack_old;
  uint32_t fcount = 0;

if(!audioPlaying){ //if not play file
  File audiofile;
  delete [] f_arr;
  audiofile = SPIFFS.open(fname, "r");
  if(audiofile.available()){ //if file existing
    f_stack = 0, f_len = 0;
    f_arr = new uint8_t[audiofile.size()];

    while(audiofile.available()){
      f_arr[f_stack] = audiofile.read(); 
      if(fcount > 0x5E){f_stack++;}
      fcount++;
      } //load file
    audiofile.close();
    f_stack--;
    f_len = 0x5E;
    audioPlaying = true; //start player
    stack_old = f_stack;
}
    
fname_old = fname;
last_playingTime = millis();
display_millis = millis();
}
}

void playSpiffsDouble(String fname1, String fname2){ //wav play double files
  static uint32_t stack_old;
  String totalName = fname1+fname2;

if(!audioPlaying){ //if not play file
if(totalName != fname_old){
delete [] f_arr; //delete old array
File audiofileOne, audiofileTwo;

audiofileOne = SPIFFS.open(fname1, "r");
audiofileTwo = SPIFFS.open(fname2, "r");

uint32_t total_len = audiofileOne.size() + audiofileTwo.size();
uint32_t fcount = 0;
f_stack = 0, f_len = 0;
f_arr = new uint8_t[total_len];

fcount = 0;
while(audiofileOne.available()){
  f_arr[f_stack] = audiofileOne.read(); 
  if(fcount > 0x5E){f_stack++;}
  fcount++;
  } //load file 1
audiofileOne.close();

fcount = 0;
while(audiofileTwo.available()){
  f_arr[f_stack] = audiofileTwo.read(); 
  if(fcount > 0x5E){f_stack++;}
  fcount++;
  } //load file 1
audiofileTwo.close();

f_stack--;
f_len = 0;
audioPlaying = true; //start player
fname_old = totalName;
stack_old = f_stack;

} else { //old data playing
  f_len = 0;
  f_stack = stack_old;
  audioPlaying = true; //start player
}
last_playingTime = millis();
display_millis = millis();
}
}

void play_tone(uint16_t longer, uint8_t divv, uint8_t volum){
      for(uint16_t i=0; i<longer; i++){((i%divv)<(divv/2))?f_arr[i]=volum:f_arr[i]=0;}
      f_len=0; f_stack=longer; audioPlaying = true;
      while(audioPlaying){TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE; TIMERG0.wdt_feed=1; TIMERG0.wdt_wprotect=0;}
}

void play_silent(uint16_t longer){
      for(uint16_t i=0; i<longer; i++){f_arr[i]=0;}
      f_len=0; f_stack=longer; audioPlaying = true;
      while(audioPlaying){TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE; TIMERG0.wdt_feed=1; TIMERG0.wdt_wprotect=0;}
}

void storeStruct(void *data_source, size_t size){ //storeStruct(&dev_settings, sizeof(dev_settings));
  EEPROM.begin(size * 2);
  for(size_t i = 0; i < size; i++)
  {
    char data = ((char *)data_source)[i];
    EEPROM.write(i, data);
  }
  EEPROM.commit();
}

void loadStruct(void *data_dest, size_t size){
    EEPROM.begin(size * 2);
    for(size_t i = 0; i < size; i++)
    {
        char data = EEPROM.read(i);
        ((char *)data_dest)[i] = data;
    }
}

float simpleKalman(float newVal, float _err_measure, float _q) { //input, input error, k
  float _kalman_gain, _current_estimate;
  static float _err_estimate = _err_measure;
  static float _last_estimate;
  _kalman_gain = (float)_err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + (float)_kalman_gain * (newVal - _last_estimate);
  _err_estimate =  (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
  _last_estimate = _current_estimate;
  return _current_estimate;
}

void power_off(){ //power off device
      display.clearDisplay();
      display.setFont(&FreeMono6pt8b); 
      display.setTextSize(1);
      display_center(32, "ВЫКЛЮЧЕНИЕ");
      display.display();
      delay(100);
      while(!digitalRead(B_RIGHT_pin));
      while(audioPlaying){delay(10);} //wait sound play

      pinMode (B_LEFT_pin, OUTPUT); digitalWrite(B_LEFT_pin, LOW);
      pinMode (B_RIGHT_pin, OUTPUT); digitalWrite(B_RIGHT_pin, LOW);
      MAIN_SUPPLY_OFF;
      //esp_deep_sleep(2 * 200000);
      delay(2000);
      MAIN_SUPPLY_OFF;
}

void switch_SensScheme(uint8_t scheme){ //apply sens scheme
switch(scheme){
      case SENS_CITY: //CITY
      dev_settings.sens_SUB1G = dev_settings.sensC_SUB1G;
      dev_settings.sens_2G = dev_settings.sensC_2G;
      dev_settings.sens_5G = dev_settings.sensC_5G;
      break;

      case SENS_SUBURB: //SUBURB
      dev_settings.sens_SUB1G = dev_settings.sensS_SUB1G;
      dev_settings.sens_2G = dev_settings.sensS_2G;
      dev_settings.sens_5G = dev_settings.sensS_5G;
      break;

      case SENS_BRIM: //BRIM
      dev_settings.sens_SUB1G = dev_settings.sensB_SUB1G;
      dev_settings.sens_2G = dev_settings.sensB_2G;
      dev_settings.sens_5G = dev_settings.sensB_5G;
      break;

      case SENS_CAR: //CAR
      dev_settings.sens_SUB1G = dev_settings.sensR_SUB1G;
      dev_settings.sens_2G = dev_settings.sensR_2G;
      dev_settings.sens_5G = dev_settings.sensR_5G;
      break;
     }
}

void save_SensScheme(){ //save sens scheme
switch(dev_settings.sens_scheme){
      case 0: //CITY
      dev_settings.sensC_SUB1G = dev_settings.sens_SUB1G;
      dev_settings.sensC_2G = dev_settings.sens_2G;
      dev_settings.sensC_5G = dev_settings.sens_5G;
      break;

      case 1: //SUBURB
      dev_settings.sensS_SUB1G = dev_settings.sens_SUB1G;
      dev_settings.sensS_2G = dev_settings.sens_2G;
      dev_settings.sensS_5G = dev_settings.sens_5G;
      break;

      case 2: //BRIM
      dev_settings.sensB_SUB1G = dev_settings.sens_SUB1G;
      dev_settings.sensB_2G = dev_settings.sens_2G;
      dev_settings.sensB_5G = dev_settings.sens_5G;
      break;
     }
}

void save_settings(bool save_current_sense){ //save settings
if(dev_settings.reset_settings != 0){
      dev_settings.power_button_trig=1000; //power on/off trigger time 1000 = 1 Sec
      dev_settings.menu_button_trig=2000;
      dev_settings.sound_volume=100;     //%
      dev_settings.sound_period=10;   //S
      dev_settings.battery_min=BAT_MIN; //mV
      dev_settings.battery_max=BAT_MAX; //mV
      dev_settings.disp_timeout=60; //Seconds
      dev_settings.disp_bright=0;  //percent
      dev_settings.warfare_sleep_period=20; //electronic warfare sleep period alarms (20 alarms * sound_period(5S) = 100 seconds) 0-function OFF
      dev_settings.warfare_active=5;    //warfare active time
      dev_settings.duration_2G=14;  //scan tick period 1-16 (1=high, 14-low)
      dev_settings.duration_5G=5;  //scan tick period 1-16
      dev_settings.duration_SUB1G=2; //scan tick period
      dev_settings.reset_settings=0; //reset settings to default
      dev_settings.SUB1G_scnFmin = 250;
      dev_settings.SUB1G_scnFmax = 940;

      dev_settings.sens_scheme = 1;  //SUBURB sensivity
      
      dev_settings.sens_SUB1G = 6;   //SUBURB default
      dev_settings.sens_2G = 10;     //SUBURB default
      dev_settings.sens_5G = 8;      //SUBURB default
      
      dev_settings.sensC_SUB1G = 4;   //CITY
      dev_settings.sensC_2G = 5;      //CITY
      dev_settings.sensC_5G = 3;      //CITY
      
      dev_settings.sensS_SUB1G = 5;   //SUBURB
      dev_settings.sensS_2G = 10;     //SUBURB
      dev_settings.sensS_5G = 8;      //SUBURB
      
      dev_settings.sensB_SUB1G = 6;   //BRIM
      dev_settings.sensB_2G = 30;     //BRIM
      dev_settings.sensB_5G = 20;     //BRIM

      dev_settings.sensR_SUB1G = 5;   //CAR
      dev_settings.sensR_2G = 15;     //CAR
      dev_settings.sensR_5G = 3;     //CAR
    } else{
      if(save_current_sense){save_SensScheme();}
      } //SAVE SENS
   //check freq
   if(dev_settings.SUB1G_scnFmin < 250){dev_settings.SUB1G_scnFmin=250;}
   if(dev_settings.SUB1G_scnFmax > 960){dev_settings.SUB1G_scnFmax=960;}
   if(dev_settings.SUB1G_scnFmin >= dev_settings.SUB1G_scnFmax){dev_settings.SUB1G_scnFmin = 250; dev_settings.SUB1G_scnFmax = 940;}
   storeStruct(&dev_settings, sizeof(dev_settings));
}

void print_reset_reason(RESET_REASON reason){
  switch (reason){
    case 1 : Serial.println ("POWERON_RESET");break;        /**<1, Vbat power on reset*/      
    case 3 : Serial.println ("SW_RESET");break;       /**<3, Software reset digital core*/            
    case 4 : Serial.println ("OWDT_RESET");break;        /**<4, Legacy watch dog reset digital core*/         
    case 5 : Serial.println ("DEEPSLEEP_RESET");break;    /**<5, Deep Sleep reset digital core*/        
    case 6 : Serial.println ("SDIO_RESET");break;        /**<6, Reset by SLC module, reset digital core*/         
    case 7 : Serial.println ("TG0WDT_SYS_RESET");break;     /**<7, Timer Group0 Watch dog reset digital core*/      
    case 8 : Serial.println ("TG1WDT_SYS_RESET");break;      /**<8, Timer Group1 Watch dog reset digital core*/     
    case 9 : Serial.println ("RTCWDT_SYS_RESET");break;       /**<9, RTC Watch dog Reset digital core*/    
    case 10 : Serial.println ("INTRUSION_RESET");break;        /**<10, Instrusion tested to reset CPU*/   
    case 11 : Serial.println ("TGWDT_CPU_RESET");break;        /**<11, Time Group reset CPU*/   
    case 12 : Serial.println ("SW_CPU_RESET");break;       /**<12, Software reset CPU*/       
    case 13 : Serial.println ("RTCWDT_CPU_RESET");break;        /**<13, RTC Watch dog Reset CPU*/  
    case 14 : Serial.println ("EXT_CPU_RESET");break;       /**<14, for APP CPU, reseted by PRO CPU*/      
    case 15 : Serial.println ("RTCWDT_BROWN_OUT_RESET");break;    /**<15, Reset when the vdd voltage is not stable*/
    case 16 : Serial.println ("RTCWDT_RTC_RESET");break;         /**<16, RTC Watch dog reset digital core and rtc module*/ 
    default : Serial.println ("NO_MEAN");
  }
}

////////////////// MONITORS //////////////////
void waterfall_print(String label_w, volatile uint8_t *waterfall){
#ifndef DEBUG
if(label_w == "5G:"){
  Serial.print(label_w);
  uint8_t channelIndex;
  for (uint16_t chan = CHANNEL_MIN_5G; chan < CHANNEL_MAX_5G-1; chan++) {
    channelIndex = channelList[chan];
    Serial.print(rx5808.scanVec_5G[channelIndex], HEX); Serial.print(",");
    }
    channelIndex = channelList[CHANNEL_MAX_5G-1];
    Serial.println(rx5808.scanVec_5G[channelIndex], HEX);
} else 
if(label_w == "2G:"){
  Serial.print(label_w);
  uint16_t MHz = 0;
  for (MHz = 0; MHz < CHANNELS_2G-1; MHz++) {
    Serial.print(nrf24l01.signalStrength_2G[MHz], HEX); Serial.print(",");
    }
    MHz=CHANNELS_2G-1;
    Serial.println(nrf24l01.signalStrength_2G[MHz], HEX);
} else 
if(label_w == "1G:"){
  Serial.print(label_w);
  uint16_t MHz = 0;
  for (MHz = 0; MHz < DRVsi4432.steps_cnt; MHz++) {
    Serial.print(DRVsi4432.getStrength(MHz), HEX); Serial.print(",");
    //Serial.print(0xAE, HEX); Serial.print(",");
    }
    MHz=DRVsi4432.steps_cnt-1;
    Serial.println(DRVsi4432.getStrength(MHz), HEX);
    //Serial.println(0xAF, HEX);
}
#endif
}

void rx5808_WaterfallMonitor(uint8_t y_offset, uint8_t tick_divider){ //57-1, 25-2
  uint16_t min_f = 0xFFFF;
  uint16_t max_f = 0;

  display.setTextSize(1);
// draw scale
for(int i=1; i<display.width(); i++){
if((i%4) == 0){display.drawFastVLine(i, y_offset, 1, SH110X_WHITE);}
if((i%32) == 0){display.drawFastVLine(i, y_offset, 2, SH110X_WHITE);}
}
  display_left(y_offset+SCALE_TEXTOFFSET, "5.7", 0);
  display_center(y_offset+SCALE_TEXTOFFSET, "5.8");
  display_right(y_offset+SCALE_TEXTOFFSET, "5.9", 0);
  
if(tick_divider != 0){display.drawBitmap(0, y_offset-WTFALL_H, (const uint8_t *)waterfall_5G, SCREEN_WIDTH, WTFALL_H, 1);} //show waterfall
else{display_center(y_offset-SCALE_TEXTOFFSET, "OFF");}
display.drawFastHLine(0, y_offset-1, SCREEN_WIDTH, 1);

if(scan_done_5G){
    for (int i = CHANNEL_MIN_5G; i < CHANNEL_MAX_5G; i++) {
      uint8_t channelIndex = channelList[i];
      if(rx5808.getRssi(channelIndex) > max_f){max_f = rx5808.getRssi(channelIndex);}
      if(rx5808.getRssi(channelIndex) < min_f){min_f = rx5808.getRssi(channelIndex);}
    }
if(max_f > min_f){
for(int MHz=0; MHz<SCREEN_WIDTH; MHz++){
    uint8_t ch = MHz/((float)SCREEN_WIDTH/CHANNEL_MAX_5G);
    uint8_t channelIndex = channelList[ch];
    uint16_t centerline = min_f+((ADC_MAXIMUM-min_f)*2/dev_settings.sens_5G); // 2/20 - 2/3
  uint16_t scanVector = rx5808.getRssi(channelIndex);
    if(scanVector > centerline){waterfall_5G[MHz/8] |= (1<<(7-(MHz%8)));}
    else{waterfall_5G[MHz/8] &= ~(1<<(7-(MHz%8)));}
}
waterfall_shiftUP(waterfall_5G);
}
waterfall_print("5G:", waterfall_5G);
trig_handler(waterfall_5G, TRIGGER_5G, BUZZER_5G, 0, false);
scan_done_5G = false;
}
}

void nrf24l01_WaterfallMonitor(uint8_t y_offset, uint8_t tick_divider){ //25-1, 25-2
uint8_t MHz = 0;

uint32_t mill_start = millis();
uint16_t val_th = 0;

// draw scale
display.setTextSize(1);
for(int i=1; i<display.width(); i++){
if((i%4) == 0){display.drawFastVLine(i, y_offset, 1, SH110X_WHITE);}
if((i%32) == 0){display.drawFastVLine(i, y_offset, 2, SH110X_WHITE);}
}
  display_left(y_offset+SCALE_TEXTOFFSET, "2.40", 0);
  display_center(y_offset+SCALE_TEXTOFFSET, "2.46");
  display_right(y_offset+SCALE_TEXTOFFSET, "2.52", 0);

if(tick_divider != 0){display.drawBitmap(0, y_offset-WTFALL_H, (const uint8_t *)waterfall_2G, SCREEN_WIDTH, WTFALL_H, 1);} //draw waterfall
else{display_center(y_offset-SCALE_TEXTOFFSET, "OFF");}
display.drawFastHLine(0, y_offset-1, SCREEN_WIDTH, 1);

/////////////////////////////////////////////////////////////////
if(scan_done_2G){
uint16_t sig_min = 0xFFFF;
uint16_t sig_max = 0x00;
for(int x=0; x<CHANNELS_2G; x++){if(nrf24l01.signalStrength_2G[x]<sig_min){sig_min=nrf24l01.signalStrength_2G[x];}}
  
for(uint8_t MHz=0; MHz<CHANNELS_2G; MHz++){
 if(nrf24l01.signalStrength_2G[MHz] > (sig_min+((65535-sig_min)*2/dev_settings.sens_2G))){waterfall_2G[MHz/8] |= (1<<(7-(MHz%8)));}
 else{waterfall_2G[MHz/8] &= ~(1<<(7-(MHz%8)));}
}
waterfall_shiftUP(waterfall_2G);
waterfall_print("2G:", waterfall_2G);
  trig_handler(waterfall_2G, TRIGGER_2G, BUZZER_2G, 0, false);
  scan_done_2G = false;
  } //trigger handle
}

void WSUB1G_WaterfallMonitor(uint8_t y_offset, uint8_t tick_divider){ //57-1, 25-2
  uint16_t min_f = MAX_RSSI_SI4432;

if(y_offset != 0){
// draw scale
display.setTextSize(1);
for(int i=1; i<display.width(); i++){
if((i%4) == 0){display.drawFastVLine(i, y_offset, 1, SH110X_WHITE);}
if((i%32) == 0){display.drawFastVLine(i, y_offset, 2, SH110X_WHITE);}
}

  display_left(y_offset+SCALE_TEXTOFFSET, String(dev_settings.SUB1G_scnFmin), 0);
  display_center(y_offset+SCALE_TEXTOFFSET, String((dev_settings.SUB1G_scnFmax+dev_settings.SUB1G_scnFmin)/2));
  display_right(y_offset+SCALE_TEXTOFFSET, String(dev_settings.SUB1G_scnFmax), 0);
  
if(tick_divider != 0){display.drawBitmap(0, y_offset-WTFALL_H, (const uint8_t *)waterfall_SUB1G, SCREEN_WIDTH, WTFALL_H, 1);} //show waterfall
else{display_center(y_offset-SCALE_TEXTOFFSET, "OFF");}

display.drawFastHLine(0, y_offset-1, SCREEN_WIDTH, 1);
}

if(scan_done_SUB1G){

   for(uint8_t i = 0; i < SCREEN_WIDTH; i++) {
   if(DRVsi4432.getStrength(i) < min_f){min_f = DRVsi4432.getStrength(i);}}
   
   int16_t centerline = (min_f+((MAX_RSSI_SI4432-min_f)*2/dev_settings.sens_SUB1G));
   
for(int i=1; i<SCREEN_WIDTH; i++){
     if(DRVsi4432.getStrength(i) > centerline){waterfall_SUB1G[i/8] |= (1<<(7-(i%8)));}
     else{waterfall_SUB1G[i/8] &= ~(1<<(7-(i%8)));}
}
waterfall_shiftUP(waterfall_SUB1G);
waterfall_print("1G:", waterfall_SUB1G);
  trig_handler(waterfall_SUB1G, TRIGGER_SUB1G, BUZZER_SUB1G, 0, false);
  scan_done_SUB1G = false;
  DRVsi4432.init_scan(dev_settings.SUB1G_scnFmin, dev_settings.SUB1G_scnFmax, SCREEN_WIDTH);
  } //trigger handle
}

////////////////// HANDLERS //////////////////

void trig_handler(volatile uint8_t *waterfall, TRIGGERS_N trig, BUZZER_StateE buz, float k_gain, bool log_t){ //trigger handle
  
#define SAMPLES 128             //Must be a power of 2 WTFALL_H
int16_t inTRbuf[SAMPLES];
for(int i=0; i<SAMPLES; i++){inTRbuf[i]=0;} //reset result array

//find vertical traces in waterfall
for(uint16_t j=0; j<(WTFALL_L+1); j++){ //w 16*8=128
for(uint16_t i=0; i<(WTFALL_H+1); i++){ //h 15+1
  uint16_t byte_nym = j+(i*WTFALL_L); //col scan
  for(int b=0; b<8; b++){ //bit scanner
  uint16_t bit_num = 7-b;
  if(waterfall[byte_nym] & (1<<bit_num)){
    if(log_t){inTRbuf[(j*8)+b]+=16-i;} //LOG
    else{inTRbuf[(j*8)+b]+=1;} //linear
    }
  } //bit
  }
}

//draw chart & find maximum
int16_t maximum=0, position_m=0;
for(uint16_t i=1; i<SAMPLES-1; i++){if(inTRbuf[i] > maximum){maximum=inTRbuf[i]; position_m=i;}}
if(maximum < 0){maximum=0;}
/*
#ifdef DEBUG
if(trig == D_VIEWTRIG){
  Serial.print("max:");
  Serial.print(maximum);
  //Serial.print(", filt:");
  //Serial.print(filtered);
}
#endif
*/

if(trig == TRIGGER_SUB1G){ //dev_settings.duration_5G - 20 maximum
      if(dev_settings.duration_SUB1G > 0){
      triggers[trig]=0;
      for(uint16_t i=0; i<SAMPLES; i++){
        if(inTRbuf[i] <= 10){
          if(inTRbuf[i] >= dev_settings.duration_SUB1G){buzzerSIG |= (1<<buz); triggers[trig]++;}
        }
      }
      auto_snd_volC = (maximum*15);
      }
    }
if(trig == TRIGGER_2G){ //dev_settings.duration_5G - 20 maximum
      if(dev_settings.duration_2G > 0){
      triggers[trig]=0;
      for(uint16_t i=0; i<SAMPLES; i++){
          if(inTRbuf[i] >= dev_settings.duration_2G){buzzerSIG |= (1<<buz); triggers[trig]++;}
      }
      auto_snd_volB = (maximum*15);
      }
    }
if(trig == TRIGGER_5G){ //dev_settings.duration_5G - 20 maximum
      if(dev_settings.duration_5G > 0){
      triggers[trig]=0;
      for(uint16_t i=0; i<SAMPLES; i++){
          if(inTRbuf[i] >= dev_settings.duration_5G){buzzerSIG |= (1<<buz); triggers[trig]++;}
      }
      //if(triggers[trig] >= dev_settings.duration_5G){buzzerSIG |= (1<<buz);}
      auto_snd_volA = (maximum*15);
      }
    }
/*
#ifdef DEBUG
if(trig == D_VIEWTRIG){
  Serial.print(", trig:");
  Serial.println(triggers[trig]);
  Serial.println("");
}
#endif
*/
}

void sound_batt(void){
if(batt_voltage < dev_settings.battery_min){
  //Serial.print("batt_voltage - "); Serial.println(batt_voltage);
  //Serial.print("dev_settings.battery_min - "); Serial.println(dev_settings.battery_min);
  playSpiffsDouble("/9.wav", "/8.wav");
  while(audioPlaying){delay(10);} //wait sound play
  //power_off();
  battery_low = true;
  } //if battery low
}

void sound_handler(){
if(((millis() - last_playingTime)/1000) > dev_settings.sound_period){ //voice alarm period

////////////////////// detect buzzer SIG //////////////////////
//Serial.print("buzzerSIG: "); Serial.println(buzzerSIG);
if(buzzerSIG == ((1<<BUZZER_2G)|(1<<BUZZER_5G))){buzzerSIG = BUZZER_DRONE_FPV;}
else if(buzzerSIG == ((1<<BUZZER_SUB1G)|(1<<BUZZER_5G))){buzzerSIG = BUZZER_DRONE_FPV;}
else{
  if(buzzerSIG == (1<<BUZZER_2G)){buzzerSIG = BUZZER_2G;}
  else if(buzzerSIG == (1<<BUZZER_5G)){buzzerSIG = BUZZER_5G;}
  else if(buzzerSIG == (1<<BUZZER_SUB1G)){buzzerSIG = BUZZER_SUB1G;}
  }
////////////////////// detect buzzer SIG //////////////////////
if(dev_settings.sound_volume == 0){buzzerSIG = BUZZER_OFF;}
switch(buzzerSIG){   // buzzer signal
  case BUZZER_OFF:
  ;
  break;

  case BUZZER_NOISE:
   playSpiffsDouble("/9.wav", "/3.wav");
  break;
  
  case BUZZER_2G:
   playSpiffsDouble("/2.wav", "/4.wav");
  break;
  
  case BUZZER_5G:
   playSpiffsDouble("/2.wav", "/5.wav"); 
  break;

  case BUZZER_SUB1G:
   playSpiffsDouble("/2.wav", "/10.wav"); 
  break;

  case BUZZER_DRONE_FPV:
   playSpiffsDouble("/6.wav", "/1.wav");
  break;

  case BUZZER_DRONE_MAVIC:
   playSpiffsDouble("/9.wav", "/1.wav");
  break;

  default:
  buzzerSIG = BUZZER_OFF;
  break;
}
buzzerSIG = BUZZER_OFF;
sound_batt();
} //else{buzzerSIG = BUZZER_OFF;}
}

void battery_handler(){  // draw battery
uint16_t batt_w = 18;
uint16_t batt_h = 4;
uint16_t batt_val = 0; //0-100
float filt_k = 0.1;
static uint16_t bat_filt=0;
static bool  first_start = true;

display.drawRect(display.width()-batt_w-1, 0, batt_w, batt_h, SH110X_WHITE); //x0, y0, w, h, color
display.drawRect(display.width()-1, 1, 1, batt_h/2, SH110X_WHITE); //x0, y0, w, h, color

int32_t tmpp = 0;
for(int i=0; i<50; i++){tmpp += analogReadMilliVolts(BATTERY_PIN);}
batt_voltage += ((float)((float)tmpp/25.0) - (float)batt_voltage) * filt_k; //filter ADD
if(first_start){batt_voltage = ((float)tmpp/25.0)+200.0; first_start=false;}

//Serial.print("batt: "); Serial.println(batt_voltage); //debug

uint16_t  bat_temp = batt_voltage;
if(bat_temp < dev_settings.battery_min){bat_temp = dev_settings.battery_min;}
if(bat_temp > dev_settings.battery_max){bat_temp = dev_settings.battery_max;}

display.setFont(&Picopixel);
display.setCursor(display.width()-batt_w-7, 4);
if(!digitalRead(BAT_STD_PIN)){display.print('F');} //complete
else if(!digitalRead(BAT_CHG_PIN)){display.print('C');} //charge

batt_val = map(bat_temp, dev_settings.battery_min, dev_settings.battery_max, 0, batt_w-2);
                 display.drawRect(display.width()-batt_w, 1, batt_w-2, batt_h-2, SH110X_BLACK);  //reset batt bar
if(batt_val > 0){display.drawRect(display.width()-batt_w, 1, batt_val, batt_h-2, SH110X_WHITE);} //batt bar
}

void button_handler(){ //calc button time
static bool menu_wait_release = false;
button_state = 0; //reset state
if(!digitalRead(B_RIGHT_pin)){button_state |=B_RIGHT;}
if(!digitalRead(B_LEFT_pin)){button_state |=B_LEFT;}

switch(button_state){ //button calc press time
  case 0:
     if(!menu_handle){
     if(b_left_millis < dev_settings.menu_button_trig){
     if(b_left_millis > (dev_settings.menu_button_trig/20)){sens_handle = true;}} //switch sensivity
     }
     
     b_left_millis = 0;  //reset button time
     b_right_millis = 0;
     b_all_millis = 0;
     menu_wait_release = true;
  break; //button none
  
  case B_RIGHT: 
   b_right_millis += millis()-loop_tick_time_start;
   b_all_millis = 0;
   if(b_right_millis > dev_settings.power_button_trig){power_off();} //power off trigger
  break; //power button
  
  case B_LEFT: 
   b_left_millis += millis()-loop_tick_time_start;
   b_all_millis = 0;
   //menu_handle = true;
   if(menu_wait_release){
   if(b_left_millis > dev_settings.menu_button_trig){
    menu_handle = !menu_handle; 
    menu_wait_release = false;
    } //menu trigger
   }
  break; //menu button
  
  case B_ALL: 
   b_right_millis = 0;
   b_left_millis = 0;
   b_all_millis += millis()-loop_tick_time_start;
  break; //all buttons pressed
}
}

void mainMENU_handler(){ //основное меню
static uint8_t m_pos = MENU_SIZE*4;
static int8_t scroll_offset = 0;
static bool button_release = false;
static bool parameter_selected = false;
static uint8_t switch_item = false; //
uint8_t m_st[3] = {0,1,2};
display.clearDisplay();
display.setFont(&FreeMono6pt8b); 

if(b_left_millis > 100){
  if(button_release){switch_item = 1;}
  button_release = false;
  }
else if(b_right_millis > 100){
  if(button_release){switch_item = 2;}
  button_release = false;
  }
else if(b_all_millis > 100){
  if(button_release){parameter_selected = !parameter_selected; switch_item=0;}
  button_release = false;
}
else{
  button_release = true;
  if(parameter_selected == false){
    if(switch_item == 1){m_pos++; scroll_offset=15;}  //left
    if(switch_item == 2){m_pos--; scroll_offset=-15;} //right
    }
  else{
    m_st[1] = ((uint8_t)m_pos+1)%MENU_SIZE;
    if(switch_item == 1){if(*mainMenu_par[m_st[1]] > mainMenu_minimum[m_st[1]]){*mainMenu_par[m_st[1]]-=mainMenu_step[m_st[1]];}}
    if(switch_item == 2){if(*mainMenu_par[m_st[1]] < mainMenu_maximum[m_st[1]]){*mainMenu_par[m_st[1]]+=mainMenu_step[m_st[1]];}}
    }
  switch_item = 0;
  }

if(scroll_offset > 0){scroll_offset--;}
else if(scroll_offset < 0){scroll_offset++;}

m_st[0] = m_pos%MENU_SIZE;
m_st[1] = (m_pos+1)%MENU_SIZE;
m_st[2] = (m_pos+2)%MENU_SIZE;

display_left(31+scroll_offset, mainMenu_str[m_st[0]], 1);
display_left(46+scroll_offset, mainMenu_str[m_st[1]], 1);
display_left(62+scroll_offset, mainMenu_str[m_st[2]], 1);

display_right(31+scroll_offset, String((uint16_t)*mainMenu_par[m_st[0]])+*mainMenu_val[m_st[0]], 3);
display_right(46+scroll_offset, String((uint16_t)*mainMenu_par[m_st[1]])+*mainMenu_val[m_st[1]], 3); //selected parameter
display_right(62+scroll_offset, String((uint16_t)*mainMenu_par[m_st[2]])+*mainMenu_val[m_st[2]], 3);

display.fillRect(0,0,SCREEN_WIDTH,15, 0); //fill menu box
display.drawFastHLine(0, 15, SCREEN_WIDTH, 1);
display.drawRect(0,37,SCREEN_WIDTH,13, 1); //change box
if(parameter_selected){display_center(13, "РЕДАКТИРОВАНИЕ");} //if parameter selected - view name
else{display_center(13, "ГЛАВНОЕ МЕНЮ");}

//set parameters
display.setContrast((dev_settings.disp_bright*255)/100);
}

////////////////// MAIN //////////////////
void setup() {
  //pinMode (MAIN_SUPPLY_PIN, INPUT);

  pinMode (MAIN_SUPPLY_PIN, OUTPUT); MAIN_SUPPLY_OFF;
  delay(300);
  pinMode (MAIN_SUPPLY_PIN, OUTPUT); MAIN_SUPPLY_ON;
  pinMode (B_LEFT_pin, INPUT_PULLUP);
  pinMode (B_RIGHT_pin, INPUT_PULLUP);
  pinMode(BATTERY_PIN, INPUT);
  pinMode(BAT_CHG_PIN, INPUT);
  pinMode(BAT_STD_PIN, INPUT);

  //vspi = new SPIClass(VSPI);
  pinMode(SI_CS_pin, OUTPUT); digitalWrite(SI_CS_pin, HIGH); //SI4432 CS
  vspi->begin(SI_SCK_pin, SI_MISO_pin, SI_MOSI_pin, SI_CS_pin);
  SPImutex = xSemaphoreCreateMutex();

#ifndef DEBUG  //debug menu
  SETTING tmp_settings; //LOAD settings
  loadStruct(&tmp_settings, sizeof(tmp_settings));
  if(tmp_settings.crc_control == 666){
    dev_settings = tmp_settings;
    switch_SensScheme(dev_settings.sens_scheme); //apply sens scheme
    } //check CRC. else not checked - set default settings
#else
storeStruct(&dev_settings, sizeof(dev_settings));
#endif

  if(dev_settings.serial_speed > 1000){Serial.begin(dev_settings.serial_speed);} 
  else{Serial.begin(USB_SPEED_BAUD);} //default speed

print_reset_reason(rtc_get_reset_reason(0));
if(rtc_get_reset_reason(0) != 1){power_off();} //power BUG reset

uint64_t chipid; 
chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes). 
chip_SERIAL = String((uint16_t)(chipid>>32), HEX);
chip_SERIAL += String((uint32_t)(chipid), HEX);
chip_SERIAL.toUpperCase();

  #ifdef DEBUG  //debug menu
  Serial.print("dev_settings.power_button_trig - "); Serial.println(dev_settings.power_button_trig);
  Serial.print("dev_settings.sound_volume - "); Serial.println(dev_settings.sound_volume);
  Serial.print("dev_settings.sound_period - "); Serial.println(dev_settings.sound_period);
  Serial.print("dev_settings.battery_min - "); Serial.println(dev_settings.battery_min);
  Serial.print("dev_settings.battery_max - "); Serial.println(dev_settings.battery_max);
  Serial.print("dev_settings.serial_speed - "); Serial.println(dev_settings.serial_speed);
  Serial.print("dev_settings.crc_control - "); Serial.println(dev_settings.crc_control);
  #endif
  
  Serial.println("device start " + chip_SERIAL); //print chip serial

  //MAIN_SUPPLY_ON;
  attachInterrupt(B_LEFT_pin, toggleBUTTON, FALLING);
  attachInterrupt(B_RIGHT_pin, toggleBUTTON, FALLING);

  adcAttachPin(BATTERY_PIN);
  analogSetPinAttenuation(BATTERY_PIN, ADC_11db);

  Wire.setClock(2000000UL);
  display.begin(i2c_Address, true);
  Wire.setClock(2000000UL);

  display.clearDisplay();
  display.setContrast((dev_settings.disp_bright*255)/100); //brightness 0-127
  display.setTextColor(SH110X_WHITE);
  display.setFont(&FreeMono6pt8b); 

  display.setTextSize(1);
  display_center(15, "детектор БПЛА");
  display.setTextSize(3);
  display_center(38, "ТЕНЬ");
  display.setTextSize(1);
  display_center(50, "v0.0.1");
  display_center(60, "S/N:"+chip_SERIAL);
  display.display();
  display_millis = millis();

#ifndef DEBUG
  //power button ready? //uncomment if programm release
  uint32_t button_timer = millis();
  while(!digitalRead(B_RIGHT_pin)){
    if((millis() - button_timer) >= dev_settings.power_button_trig){
      /*display.clearDisplay();
      display.setFont(&FreeMono6pt8b); 
      display.setTextSize(1);
      display_center(32, "ВКЛЮЧЕНИЕ");
      display.display();*/
      break;
      }
    } //wait power button
  if((millis() - button_timer) < dev_settings.power_button_trig){power_off();} //if power ON
#endif

  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){Serial.println("SPIFFS Mount Failed"); return;}
  init_sound(); //sound player init
  play_spiffs("/7.wav"); //greeting song
  
  while(!digitalRead(B_RIGHT_pin)){;} //wait power button
  
  if(!locked_chip){
  rx5808.init();
  Serial.println("5.8GHz init");
  nrf24l01.init();
  Serial.println("2.4GHz init");
  DRVsi4432.init();
  DRVsi4432.setBaudRate(250);
  DRVsi4432.setFrequency(868);
  DRVsi4432.readAll();
  DRVsi4432.startListening();
  Serial.println("SUB 1G init");
  }

  loop_tick_time_start = millis(); //loop start time get

    xTaskCreatePinnedToCore(sound_Task, "sound_Task", 100000, SPImutex, 0, NULL, 1);
    xTaskCreatePinnedToCore(blink_Task, "blink_Task", 10000, SPImutex, 0, NULL, 1);
    
    xTaskCreatePinnedToCore(scan_5G_Task, "scan_5G_Task", 10000, SPImutex, 2, NULL, 0);
    xTaskCreatePinnedToCore(scan_2G_Task, "scan_2G_Task", 10000, SPImutex, 2, NULL, 0);
    xTaskCreatePinnedToCore(scan_SUB1G_Task, "scan_SUB1G_Task", 10000, SPImutex, 2, NULL, 0);
}

void loop() {
button_handler(); //button read & calc pressing time

loop_tick++; //main loop cycle
loop_tick_time_start = millis();

if(menu_handle){ //if menu entred
    mainMENU_handler();
} else {
    display.setFont(&Picopixel); 
    if(!locked_chip){
    rx5808_WaterfallMonitor(AREA_A_OFFSET, dev_settings.duration_5G);
    nrf24l01_WaterfallMonitor(AREA_B_OFFSET, dev_settings.duration_2G);
    WSUB1G_WaterfallMonitor(AREA_C_OFFSET, dev_settings.duration_SUB1G);
    }
    battery_handler();

    if(sens_handle){ //sense switch
     display.clearDisplay();
     display.setFont(&FreeMono6pt8b); 
     display.setTextSize(1);
     display_center(31, "ЧУВСТВИТЕЛЬНОСТЬ");

     if(dev_settings.sens_scheme>=3){dev_settings.sens_scheme=0;} //switch
     else{dev_settings.sens_scheme++;}

     switch_SensScheme(dev_settings.sens_scheme); //apply sens scheme
     
     switch(dev_settings.sens_scheme){
      case SENS_CITY: display_center(41, "ГОРОД"); break;
      case SENS_SUBURB: display_center(41, "ОБЛАСТЬ"); break;
      case SENS_BRIM: display_center(41, "ПОЛЕ"); break;
      case SENS_CAR: display_center(41, "АВТО"); break;
     }
     save_settings(false); //struct save
     display.display();
     delay(1000);
     sens_handle = false;
    }

}

//EEPROM handler
if(menu_handle_prev != menu_handle){ //if exit main menu - write settings
   if(!menu_handle){ //if menu exit - write settings
    save_settings(true);
   #ifdef DEBUG
   Serial.println("settings write");
   #endif
   }
   menu_handle_prev = menu_handle;
  }

if(battery_low){power_off();}

//display handler
#ifndef DEBUG
if(dev_settings.sens_scheme != SENS_CAR){
if(dev_settings.disp_timeout !=0){
if(((millis()-display_millis)/1000) > dev_settings.disp_timeout){

display.clearDisplay(); 

if(blink_50ms < 5){
display.setFont(&FreeMono6pt8b); 
display.setTextSize(1);
display_center(32, "...");
}else if(blink_50ms > 10){blink_50ms=0;}

}
} //if timeout == 0 - not disable screen
}
#endif

display.display();
display.clearDisplay(); //update display buffer
}

////////////////// RTOS TASKS //////////////////

void blink_Task(void* arg){
  SemaphoreHandle_t mutex = (SemaphoreHandle_t) arg;
  for(;;){
    TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed=1;
    TIMERG0.wdt_wprotect=0;

    blink_50ms++;
    delay(50);
  }
  vTaskDelete(NULL);
}

void sound_Task(void* arg){ //sound play task
  for(;;){
    TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed=1;
    TIMERG0.wdt_wprotect=0;
    if(dev_settings.sens_scheme != SENS_CAR){audio_car_once = true; audio_car = false; sound_handler();}
    else{
      if(audio_car_once){delete [] f_arr; f_stack = 900; f_arr = new uint8_t[f_stack]; f_len=0; audio_car_once=false; fname_old="";}
      if(auto_snd_volA!=0){play_tone(900, auto_snd_divA, auto_snd_volA); play_silent(600);}
      if(auto_snd_volB!=0){play_tone(900, auto_snd_divB, auto_snd_volB); play_silent(600);}
      if(auto_snd_volC!=0){play_tone(900, auto_snd_divC, auto_snd_volC); play_silent(600);}
      delay(100);
      play_silent(300);
      delay(100);
      sound_batt();
      }
    }
  vTaskDelete(NULL);
}

void scan_5G_Task(void* arg){
  SemaphoreHandle_t mutex = (SemaphoreHandle_t) arg;
  for(;;){
    TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed=1;
    TIMERG0.wdt_wprotect=0;
    if(!scan_done_5G){
    for (uint16_t chan = 0; chan < CHANNEL_MAX_5G; chan++) {
    uint32_t freq = channelFreqTable[chan];
    rx5808.setFreq(freq);
    delay(20);
    rx5808.scanVec_5G[chan] = analogRead(rx5808.rssiPin); //_readRSSI(1);
    }
    scan_done_5G = true;
    }
  }
  vTaskDelete(NULL);
}

void scan_2G_Task(void* arg){
TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
TIMERG0.wdt_feed=1;
TIMERG0.wdt_wprotect=0;
  SemaphoreHandle_t mutex = (SemaphoreHandle_t) arg;
  for(;;){
    TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed=1;
    TIMERG0.wdt_wprotect=0;
if(!scan_done_2G){
for(uint8_t MHz=0; MHz<CHANNELS_2G; MHz++){
  xSemaphoreTake(mutex, SEM_TIMEOUT);
  nrf24l01.NRF24L01_WriteReg(NRF24L01_05_RF_CH, MHz);                    // Set new freqency for scan
  xSemaphoreGive(mutex);
  //nrf24l01.signalStrength_2G[MHz] = 0; 
  CE_on;   // start receiving
  delayMicroseconds(300);
  CE_off;  // stop receiving - one bit is now set if received power was > -64 dBm at that instant
  xSemaphoreTake(mutex, SEM_TIMEOUT);
  if (nrf24l01.NRF24L01_ReadReg(NRF24L01_09_CD)) {
    nrf24l01.signalStrength_2G[MHz] += (0x7FFF - nrf24l01.signalStrength_2G[MHz]) >> 2;
    } else { //RSSI input
    nrf24l01.signalStrength_2G[MHz] -= nrf24l01.signalStrength_2G[MHz] >> 2;
    }
  xSemaphoreGive(mutex);
}

scan_done_2G = true;
} else{delay(20);} //xSemaphoreGive(mutex); 
  }
  vTaskDelete(NULL);
}

void scan_SUB1G_Task(void* arg){
  SemaphoreHandle_t mutex = (SemaphoreHandle_t) arg;
  for(;;){
    TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed=1;
    TIMERG0.wdt_wprotect=0;

    if(!scan_done_SUB1G){
      DRVsi4432.scan(mutex, 200); //cc_task_scan(mutex, 810, 938, steps_cnt);
      scan_done_SUB1G = true;
    }
  }
  vTaskDelete(NULL);
}
