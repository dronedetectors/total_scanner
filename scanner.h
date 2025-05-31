#ifndef scanner_h
#define scanner_h

#include "rx5808.h"
#include "nrf24l01.h"
#include "si4432.h"

#include <Arduino.h>
#include <SPI.h>
#include "driver/spi_common.h"
#include "driver/spi_master.h"

#include "FS.h"
#include "SPIFFS.h"
#include "soc/ledc_reg.h"
#include "soc/ledc_struct.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#include <Fonts/Picopixel.h>
#include "FreeMono6.h"

//#define DEBUG            1
#define CHIP_FACTORY_ID  0xAABBCCDDEEFF
#define D_VIEWTRIG       TRIGGER_5G //TRIGGER_SUB1G //TRIGGER_2G //TRIGGER_5G

#define SEM_TIMEOUT  (5000*160000) //semaphore timeout

///////////// OLED DISPLAY /////////////
#define i2c_Address 0x3D //initialize with the I2C addr 0x3D Typically Adafruit OLED's
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 //   QT-PY / XIAO

#define AREA_SHADOW_OFFSET   0

#define AREA_A_OFFSET   15
#define AREA_B_OFFSET   36
#define AREA_C_OFFSET   57

#define AREA_D_OFFSET   24
#define AREA_E_OFFSET   56

///////////// VOICE /////////////
#define FORMAT_SPIFFS_IF_FAILED true
#define LEDC_TIMER(g,t) LEDC.timer_group[(g)].timer[(t)]
#define LEDC_CHAN(g,c)  LEDC.channel_group[(g)].channel[(c)]
#define PWM_CH          1

///////////// PIN /////////////
#define B_LEFT_pin    32
#define B_RIGHT_pin   33
#define BUZZER_pin    27

#define MAIN_SUPPLY_PIN   25
#define MAIN_SUPPLY_ON    digitalWrite(MAIN_SUPPLY_PIN, HIGH)
#define MAIN_SUPPLY_OFF   digitalWrite(MAIN_SUPPLY_PIN, LOW)

#define BATTERY_PIN   34
#define BAT_CHG_PIN   39
#define BAT_STD_PIN   36

///////////// SETTINGS /////////////
#define USB_SPEED_BAUD       115200
#define POWEROFF_PERIOD      30      //button 3S off period
#define SOUND_REPLAY_PERIOD  5000    // 5Sec

#define BAT_MIN           3600 //3.6v
#define BAT_MAX           4050 //4.05v

#define B_LEFT    (1<<1)
#define B_RIGHT   (1<<2)
#define B_ALL     (B_LEFT|B_RIGHT)

#define SCALE_TEXTOFFSET   6

#define WTFALL_H   15

#define WTFALL_L   16 //byte 8*16 = 128
#define WTFAL_SIZE (WTFALL_L*(WTFALL_H+1)) //256

  #define SENS_CITY   0
  #define SENS_SUBURB 1
  #define SENS_BRIM   2
  #define SENS_CAR    3

typedef struct SETTING{ // settings structure
  uint32_t serial_speed{115200}; //baud
  uint16_t power_button_trig{1000}; //power on/off trigger time 1000 = 1 Sec
  uint16_t menu_button_trig{2000};
  
// SOUND
  uint16_t sound_volume{100};     //%
  uint16_t sound_period{10};   //S
  
// BATTERY & POWER
  uint16_t battery_min{BAT_MIN}; //mV
  uint16_t battery_max{BAT_MAX}; //mV
  
// DISPLAY
  uint16_t disp_timeout{60}; //Seconds
  uint16_t disp_bright{0};  //percent

// ELECTRONIC WARFARE 
  uint16_t warfare_sleep_period{20}; //electronic warfare sleep period alarms (20 alarms * sound_period(5S) = 100 seconds) 0-function OFF
  uint16_t warfare_active{5};    //warfare active time
  
// CHANNEL SENSIVITY
  uint16_t sens_SUB1G{5}; //alarm threshold
  uint16_t sens_2G{5};    //alarm threshold
  uint16_t sens_5G{8};    //alarm threshold

  uint16_t sens_scheme{SENS_SUBURB};    //sense scheme (SUBURB)

  //CITY DEFAULT
  uint16_t sensC_SUB1G{4}; //alarm threshold
  uint16_t sensC_2G{5};    //alarm threshold+
  uint16_t sensC_5G{3};    //alarm threshold+

  //SUBURB DEFAULT
  uint16_t sensS_SUB1G{5}; //alarm threshold
  uint16_t sensS_2G{10};    //alarm threshold+
  uint16_t sensS_5G{8};    //alarm threshold+

  //BRIM DEFAULT
  uint16_t sensB_SUB1G{6}; //alarm threshold
  uint16_t sensB_2G{30};    //alarm threshold+
  uint16_t sensB_5G{20};    //alarm threshold+

  //CAR DEFAULT
  uint16_t sensR_SUB1G{5}; //alarm threshold
  uint16_t sensR_2G{15};    //alarm threshold+
  uint16_t sensR_5G{3};    //alarm threshold+
  
// FRQ DURATION
  uint16_t duration_2G{4};  //scan tick period 1-16 (1=high, 14-low)
  uint16_t duration_5G{4};  //scan tick period 1-16
  uint16_t duration_SUB1G{2}; //scan tick period

  uint16_t SUB1G_scnFmin{250};
  uint16_t SUB1G_scnFmax{940};

  uint16_t reset_settings{0}; //reset settings to default
  
// CRC
  uint32_t crc_control{666}; //crc settings check
}SETTING __attribute__ ((packed));

typedef enum TRIGGERS_N {
  TRIGGER_2G,
  TRIGGER_5G,
  TRIGGER_SUB1G
}TRIGGERS_N;

typedef enum BUZZER_StateE {
  BUZZER_OFF,         //0
  BUZZER_NOISE,       //1
  BUZZER_2G,          //2
  BUZZER_5G,          //3
  BUZZER_SUB1G,        //5
  BUZZER_DRONE_FPV,   //6
  BUZZER_DRONE_MAVIC  //7
}BUZZER_StateE;

#endif
