/**
 * Arduino Mega with RAMPS v1.3 v1.4 pin assignments
 * Arduino Mega与RAMPSV1.3 V1.4引脚分配
 * Applies to the following boards:适用于下列主板
 *
 *  RAMPS_13_EFB (Extruder, Fan, Bed)
 *  RAMPS_13_EEB (Extruder, Extruder, Bed)
 *  RAMPS_13_EFF (Extruder, Fan, Fan)
 *  RAMPS_13_EEF (Extruder, Extruder, Fan)
 *  RAMPS_13_SF  (Spindle, Controller Fan)
 * 
 *  RAMPS_14_EFB (Extruder, Fan, Bed)
 *  RAMPS_14_EEB (Extruder, Extruder, Bed)
 *  RAMPS_14_EFF (Extruder, Fan, Fan)
 *  RAMPS_14_EEF (Extruder, Extruder, Fan)
 *  RAMPS_14_SF  (Spindle, Controller Fan)
 *
 *  Other pins_MYBOARD.h files may override these defaults
 *  其他pins_MYBOARD.h文件可能会重写这些默认值。
 *  Differences between   之间的差异
 *  RAMPS_13 | RAMPS_14
 *         7 | 11
 **/

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif
//如果！定义（ATmega1280主板）或者定义（atmega2560主板）#报警错误，要确保你的Arduino的选取“工具”菜单->板。

#define LARGE_FLASH true    //定义大Flash真

#ifdef IS_RAMPS_14
  #define SERVO0_PIN       -1//11  // 定义伺服0引脚
#else
  #define SERVO0_PIN        7 // RAMPS_13 // Will conflict with BTN_EN2 on LCD_I2C_VIKI
                                          //将与BTN En2在LCD I2C ViKi上发生冲突
#endif
#define CHECK_MATWEIAL		2                   //定义检查matweial
#define SERVO1_PIN          6                 //定义伺服1引脚
#define SERVO2_PIN          5                 //定义伺服2引脚
#define SERVO3_PIN          4                 //定义伺服3引脚

#define X_STEP_PIN         54                 //定义X步骤引脚
#define X_DIR_PIN          55                 //定义X DIR引脚
#define X_ENABLE_PIN       38                 //定义X使能引脚
#define X_MIN_PIN          3//22// 3          //定义X min引脚
#define X_MAX_PIN          -1//2//32// 2      //定义X max引脚

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14//42//14
#define Y_MAX_PIN          -1//15//43//15

#define IN_PUT				19//15//43               //投入中的定义
#define OUT_PUT				12//19//40               //定义输出

#define LED           11//65                   //定义发光二极管

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18//39//18
#define Z_MAX_PIN          -1//19//40//19


#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24

#define E1_STEP_PIN        36
#define E1_DIR_PIN         34
#define E1_ENABLE_PIN      30

#define SDPOWER            -1                 //定义SD功率
#define SDSS               53                 //定义SDSS
#define LED_PIN            11                 //定义LED引脚

#if ENABLED(FILAMENT_SENSOR)  // FMM added for Filament Extruder 如果启用（灯丝传感器）/FMM用于长丝挤出机
  // define analog pin for the filament width sensor input       为长丝宽度传感器输入定义模拟引脚
  // Use the RAMPS 1.4 Analog input 5 on the AUX2 connector      使用AUX2连接器上的RAMPS1.4模拟输入5
  #define FILWIDTH_PIN      5                  //定义线宽引脚
#endif

#if ENABLED(Z_MIN_PROBE_ENDSTOP)               //如果启用（Z分钟探针终结站）
  // Define a pin to use as the signal pin on Arduino for the Z_PROBE endstop.
  //定义一个引脚作为信号探针在Arduino上用于Z探头端止器。
  #define Z_MIN_PROBE_PIN  32                  //定义Z-min探针引脚32
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)            //如果启用（灯丝跳动传感器）
  // define digital pin 4 for the filament runout sensor. Use the RAMPS 1.4 digital input 4 on the servos connector
  //定义用于灯丝跳动传感器的数字引脚4。在伺服连接器上使用RAMPS1.4数字输入4
  #define FILRUNOUT_PIN     4                  //定义FIL跳动引脚4
#endif

#if MB(RAMPS_13_EFF) || ENABLED(IS_RAMPS_EFB)         //如果MB（RAMPS13 EFF）启用（是RAMPS  EFB）
  #define FAN_PIN          9 // (Sprinter config)     //定义风扇引脚9 / /（短跑配置）
  #if MB(RAMPS_13_EFF)
    #define CONTROLLERFAN_PIN  -1 // Pin used for the fan to cool controller
    //定义控制器风扇引脚1         //用于风扇冷却控制器的PIN
  #endif
#elif MB(RAMPS_13_EEF) || MB(RAMPS_13_SF)        //否则 MB（RAMPS13 EEF） MB（RAMPS13 SF）
  #define FAN_PIN           8                    //定义风扇引脚
#else
  #define FAN_PIN           4 // IO pin. Buffer needed 定义风扇引脚4 //IO引脚。需要缓冲区
#endif
 
#define PS_ON_PIN          12                    //在引脚12上定义PS

#if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER) || ENABLED(G3D_PANEL)
//如果启用（RePrAPP折扣智能控制器）启用（G3D面板）
  #define KILL_PIN         41                    //定义停止引脚41
#else
  #define KILL_PIN         -1
#endif

#if MB(RAMPS_13_EFF)
  #define HEATER_0_PIN      8                    //定义加热器0针
#else
  #define HEATER_0_PIN     10   // EXTRUDER 1    //定义加热器0引脚为10  挤出机1
#endif

#if MB(RAMPS_13_SF) || ENABLED(IS_RAMPS_EFB)
  #define HEATER_1_PIN   -1// 7// -1             //定义加热器1引脚1 / / 7 / / 1
#else
  #define HEATER_1_PIN      9   // EXTRUDER 2 (FAN On Sprinter)   //定义加热器1针9 / /挤出机2（风扇在短跑上）
#endif

#define HEATER_2_PIN       -1                    //定义加热器2针1


 
#define TEMP_0_PIN         13   // ANALOG NUMBERING        //定义临时0针   模拟编号
#define TEMP_1_PIN        15   // 14//// ANALOG NUMBERING  //定义临时1针15／/ 14／／／模拟编号
#define TEMP_2_PIN         -1   // ANALOG NUMBERING

#if MB(RAMPS_13_EFF) || MB(RAMPS_13_EEF) || MB(RAMPS_13_SF)
  #define HEATER_BED_PIN   -1    // NO BED      //定义加热器床脚- 1 /否
#else
  #define HEATER_BED_PIN    8    // BED          //定义加热器床脚8 / /床
#endif

#define TEMP_BED_PIN        14//15// 14   // ANALOG NUMBERING  定义临时床针14 / / 15 / / 14 /模拟编号
 
#if ENABLED(Z_PROBE_SLED)           //// 如果启用（Z探头雪橇）
  #define SLED_PIN           -1     //定义雪橇销

#endif

#if ENABLED(ULTRA_LCD)              //如果启用（超LCD）

  #if ENABLED(NEWPANEL)
    #if ENABLED(PANEL_ONE)
      #define LCD_PINS_RS 40
      #define LCD_PINS_ENABLE 42
      #define LCD_PINS_D4 65
      #define LCD_PINS_D5 66
      #define LCD_PINS_D6 44
      #define LCD_PINS_D7 64
    #else
      #define LCD_PINS_RS 16
      #define LCD_PINS_ENABLE 17
      #define LCD_PINS_D4 23
      #define LCD_PINS_D5 25
      #define LCD_PINS_D6 27
      #define LCD_PINS_D7 29
    #endif

    #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
      #define BEEPER_PIN 37

      #define BTN_EN1 31
      #define BTN_EN2 33
      #define BTN_ENC 35

      #define SD_DETECT_PIN 49
    #elif ENABLED(LCD_I2C_PANELOLU2)
      #define BTN_EN1 47  // reverse if the encoder turns the wrong way.
      #define BTN_EN2 43
      #define BTN_ENC 32
      #define LCD_SDSS 53
      #define SD_DETECT_PIN -1
      #define KILL_PIN 41
    #elif ENABLED(LCD_I2C_VIKI)
      #define BTN_EN1 22  // reverse if the encoder turns the wrong way.
      #define BTN_EN2 7   // http://files.panucatt.com/datasheets/viki_wiring_diagram.pdf
                          // tells about 40/42.
                          // 22/7 are unused on RAMPS_14. 22 is unused and 7 the SERVO0_PIN on RAMPS_13.
      #define BTN_ENC -1
      #define LCD_SDSS 53
      #define SD_DETECT_PIN 49
    #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
      #define BTN_EN1 35  // reverse if the encoder turns the wrong way.如果编码器转错了方向，则反转。
      #define BTN_EN2 37
      #define BTN_ENC 31
      #define SD_DETECT_PIN 49
      #define LCD_SDSS 53
      #define KILL_PIN 41
      #define BEEPER_PIN 23
      #define DOGLCD_CS 29
      #define DOGLCD_A0 27
      #define LCD_PIN_BL 33
    #elif ENABLED(MINIPANEL)
      #define BEEPER_PIN 42
      // Pins for DOGM SPI LCD Support用于DOGM SPI液晶显示器的引脚
      #define DOGLCD_A0  44
      #define DOGLCD_CS  66
      #define LCD_PIN_BL 65 // backlight LED on A11/D65
      #define SDSS   53

      #define KILL_PIN 64
      // GLCD features
      //#define LCD_CONTRAST 190
      // Uncomment screen orientation
      //#define LCD_SCREEN_ROT_90
      //#define LCD_SCREEN_ROT_180
      //#define LCD_SCREEN_ROT_270  定义LCD屏幕Road 270
      //The encoder and click button编码器和点击按钮
      #define BTN_EN1 40
      #define BTN_EN2 63
      #define BTN_ENC 59  //the click switch点击开关
      //not connected to a pin  未连接到引脚
      #define SD_DETECT_PIN 49   //定义SD检测引脚49

    #else

      #define BEEPER_PIN 33  // Beeper on AUX-4
      //AUX-定义AUX-4上的寻呼机PIN 33 / /蜂鸣器

      // buttons are directly attached using AUX-2  使用AUX-2直接连接按钮
      #if ENABLED(REPRAPWORLD_KEYPAD)     //如果启用（RePrPracWord小键盘）
        #define BTN_EN1 64 // encoder
        #define BTN_EN2 59 // encoder编码器
        #define BTN_ENC 63 // enter button输入按钮
        #define SHIFT_OUT 40 // shift register
        #define SHIFT_CLK 44 // shift register
        #define SHIFT_LD 42 // shift register移位寄存器
      #elif ENABLED(PANEL_ONE)
        #define BTN_EN1 59 // AUX2 PIN 3
        #define BTN_EN2 63 // AUX2 PIN 4
        #define BTN_ENC 49 // AUX3 PIN 7
      #else
        #define BTN_EN1 37
        #define BTN_EN2 35
        #define BTN_ENC 31  // the click
      #endif

      #if ENABLED(G3D_PANEL)      //如果启用（G3D面板）
        #define SD_DETECT_PIN 49  //定义SD检测引脚49
      #else
        #define SD_DETECT_PIN -1  // Ramps doesn't use this   Ramps不使用这个
        //定义SD检测引脚1
      #endif

    #endif
  #else // !NEWPANEL (Old-style panel with shift register)带移位寄存器的旧式面板

    #define BEEPER_PIN 33   // No Beeper added  蜂鸣器引脚为33

    // Buttons are attached to a shift register 按钮附加在移位寄存器上。
    // Not wired yet
    //#define SHIFT_CLK 38
    //#define SHIFT_LD 42
    //#define SHIFT_OUT 40
    //#define SHIFT_EN 17

    #define LCD_PINS_RS 16                 //定义液晶显示引脚RS 16
    #define LCD_PINS_ENABLE 17             //定义液晶显示引脚启用17
    #define LCD_PINS_D4 23                 //定义液晶显示引脚D4 23
    #define LCD_PINS_D5 25                 //定义液晶显示引脚D5 25
    #define LCD_PINS_D6 27                 //定义液晶显示引脚D6 27
    #define LCD_PINS_D7 29                 //定义液晶显示引脚D7 29
    
  #endif // !NEWPANEL

#endif // ULTRA_LCD  超LCD

// SPI for Max6675 Thermocouple   用于MAX66 75热电偶的SPI
#if DISABLED(SDSUPPORT)
  #define MAX6675_SS       66 // Do not use pin 53 if there is even the remote possibility of using Display/SD card
  //定义MX5675 SS 66         //不使用PIN 53，即使有可能使用显示/ SD卡
#else
  #define MAX6675_SS       66 // Do not use pin 49 as this is tied to the switch inside the SD card socket to detect if there is an SD card present
  //定义MX5675 SS          66 //不要使用PIN 49，因为它被绑定到SD卡插槽内的开关，以检测是否存在SD卡。
#endif

#if DISABLED(SDSUPPORT)
  // these pins are defined in the SD library if building with SD support
  #define SCK_PIN          52
  #define MISO_PIN         50
  #define MOSI_PIN         51//这些引脚在SD库中定义，如果用SD支持构建
//定义SkkPIN 52定义MISOOPIN 定50义MysiPIN 51
  //如果禁用（SDFAIL）

//这些引脚在SD库中定义，如果用SD支持构建
定义SkkPIN 52
定义MISOOPIN 50
定义MysiPIN 51
第二节
#endif
