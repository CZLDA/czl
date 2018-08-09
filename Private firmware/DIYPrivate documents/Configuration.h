#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "boards.h"
#include "macros.h"

//===========================================================================
//============================= Getting Started入门指南 =============================
//===========================================================================
/*
Here are some standard links for getting your machine calibrated:以下是一些用于校准机器的标准链接
 * http://reprap.org/wiki/Calibration       校准
 * http://youtu.be/wAL9d7FgInk
 * http://calculator.josefprusa.cz          步进电机计算器
 * http://reprap.org/wiki/Triffid_Hunter%27s_Calibration_Guide  Triffid Hunter的校准指南
 * http://www.thingiverse.com/thing:5573    基本校准集
 * https://sites.google.com/site/repraplogphase/calibration-of-your-reprap
 * http://www.thingiverse.com/thing:298812  XY 20毫米校准盒
*/

// This configuration file contains the basic settings. //此配置文件包含基本设置。
// Advanced settings can be found in Configuration_adv.h  //高级设置可在配置中找到
// BASIC SETTINGS: select your board type, temperature sensor type, axis scaling, and endstop configuration
//基本设置：选择板卡类型，温度传感器类型，轴缩放和终点挡板配置

//===========================================================================
//============================= DELTA Printer三角洲打印机 =====================
//===========================================================================
// For a Delta printer replace the configuration files with the files in the 对于DELTA打印机，请使用中文的文件替换配置文件
// example_configurations/delta directory.示例_配置/增量目录
//

//===========================================================================
//============================= SCARA Printer机械臂类型打印机 ===============================
//===========================================================================
// For a Scara printer replace the configuration files with the files in the example_configurations/SCARA directory.
//对于机械手打印机来说，使用文件中的文件替换配置文件
// example_configurations/SCARA directory.
// example_configurations / SCARA目录。

// @section info

#if ENABLED(USE_AUTOMATIC_VERSIONING)  //如果启用（使用自动版本控制）
  #include "_Version.h"
#else
  #include "Default_Version.h"
#endif

// User-specified version info of this build to display in [Pronterface, etc] terminal window during
// startup. Implementation of an idea by Prof Braino to inform user that any changes made to this
// build by the user have been successfully uploaded into firmware.
//在启动过程中，此构建的用户指定版本信息可在【转面等】终端窗口中显示。实现Prof Braino的一个想法，告知用户，
//用户对这个构建所做的任何更改都已成功上传到固件中
#define STRING_CONFIG_H_AUTHOR "(CR-3040)" // Who made the changes.//定义字符串_配置_作者
#define SHOW_BOOTSCREEN    //定义显示 启动屏幕
#define STRING_SPLASH_LINE1 SHORT_BUILD_VERSION // will be shown during bootup in line 1
//定义 字符串启动时，第1行将显示简短版本第1行
//#define STRING_SPLASH_LINE2 STRING_DISTRIBUTION_DATE // will be shown during bootup in line 2
//定义字符串初始行2字符串分发日期//将在第2行启动期间显示
// @section machine

// SERIAL_PORT selects which serial port should be used for communication with the host.
//串行端口选择应使用哪个串行端口与主机通信
// This allows the connection of wireless adapters (for instance) to non-default port pins.
//这允许将无线适配器（例如）连接到非默认端口引脚
// Serial port 0 is still used by the Arduino bootloader regardless of this setting.
//无论此设置如何，Arduino引导加载程序仍使用串行端口0
// :[0,1,2,3,4,5,6,7]
#define SERIAL_PORT 0   //选择用于和上位机通讯的串口，该值保持不动。


// This determines the communication speed of the printer这决定了打印机的通信速度
// :[2400,9600,19200,38400,57600,115200,250000]
#define BAUDRATE 115200 //波特率定义

// Enable the Bluetooth serial interface on AT90USB devices
//在AT90USB设备上启用蓝牙串行接口
//#define BLUETOOTH  //定义蓝牙

// The following define selects which electronics board you have.以下定义用于选择您拥有的电子板
// Please choose the name from boards.h that matches your setup请选择与您的设置相匹配的boards.h中的名称
#ifndef MOTHERBOARD //主板
  #define MOTHERBOARD BOARD_RAMPS_13_EFB//主板定义
#endif

// Optional custom name for your RepStrap or other custom machine
//可选的自定义名称，用于重新绑定或其他自定义计算机
// Displayed in the LCD "Ready" message      //显示在LCD中的消息为“Ready”
#define CUSTOM_MACHINE_NAME "3D Printer"   //自定义打印机名字可以加入网址，同时language.h里MACHINE_NAME“3D Printer”
//也需要更改为网址（英语的MACHINE_NAME "ready需要注视掉"

// Define this to set a unique identifier for this printer, (Used by some programs to differentiate between machines)
//定义此选项以设置此打印机的唯一标识符（由某些程序用来区分机器）
// You can use an online service to generate a random UUID. (eg http://www.uuidgenerator.net/version4)
//您可以使用联机服务生成随机UUID。（例如：//www.uuidgenerator.net/版本4）
//#define MACHINE_UUID "00000000-0000-0000-0000-000000000000"

// This defines the number of extruders  这定义了挤出机的数量
// :[1,2,3,4]
#define EXTRUDERS 1//定义挤出机数量

// Offset of the extruders (uncomment if using more than one and relying on firmware to position when changing).
//挤出机的偏移（如果使用多个，并且在更改时依赖固件定位，则取消注释
// The offset has to be X=0, Y=0 for the extruder 0 hotend (default extruder).
//对挤出机0 喷嘴（默认挤出机），偏移量必须为X = 0 ，Y = 0. 
// For the other hotends it is their distance from the extruder 0 hotend.
//对于其他喷嘴，它是它们与挤出机0喷嘴的距离
//#define EXTRUDER_OFFSET_X {0.0, 20.00} // (in mm) for each extruder, offset of the hotend on the X axis
//定义挤出机偏移X { 0，20 } //（mm）各挤出机，在X轴上的喷头偏移
//#define EXTRUDER_OFFSET_Y {0.0, 5.00}  // (in mm) for each extruder, offset of the hotend on the Y axis
//定义挤出机偏移Y { 0，5.00 } //（mm）各挤出机，在Y轴上的喷头偏移
// The following define selects which power supply you have. Please choose the one that matches your setup
//以下定义用于选择您拥有的电源。请选择与您的设置相匹配的选项
// 1 = ATX
// 2 = X-Box 360 203Watts (the blue wire connected to PS_ON and the red wire to VCC)
//2 = X - Box 360 203瓦（蓝色导线连接到PS _ON，红色导线连接到VCC）
// :{1:'ATX',2:'X-Box 360'}

#define POWER_SUPPLY 1    //定义电源供应

// Define this to have the electronics keep the power supply off on startup. If you don't know what this is leave it.
//定义此选项以使电子设备在启动时保持电源关闭。如果你不知道这是什么就别管它
//#define PS_DEFAULT_OFF  定义PS _默认_关闭

// @section temperature

//===========================================================================
//============================= Thermal Settings 温度设置============================
//===========================================================================
//
//--NORMAL IS 4.7kohm PULLUP!-- 1kohm pullup can be used on hotend sensor, using correct resistor and table
//正常的是47000欧姆上拉1000欧姆上拉可以用在升压传感器上，使用正确的电阻和表
// Temperature sensor settings:温度传感器设置
// -2 is thermocouple with MAX6675 (only for sensor 0)
// -1 is thermocouple热电偶 with AD595
// 0 is not used
// 1 is 100k thermistor - best choice for EPCOS 100k (4.7k pullup) 1为100K热敏电阻-爱普科斯100k最佳选择
// 2 is 200k thermistor - ATC Semitec 204GT-2 (4.7k pullup)
// 3 is Mendel-parts thermistor (4.7k pullup)
// 4 is 10k thermistor热敏电阻 !! do not use it for a hotend. It gives bad resolution at high temp. !!
// 5 is 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (4.7k pullup)
// 6 is 100k EPCOS - Not as accurate as table 1 (created using a fluke thermocouple) (4.7k pullup)
// 7 is 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup)
// 71 is 100k Honeywell thermistor 135-104LAF-J01 (4.7k pullup)
// 8 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
// 9 is 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
// 10 is 100k RS thermistor 198-961 (4.7k pullup)
// 11 is 100k beta 3950 1% thermistor (4.7k pullup)
// 12 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup) (calibrated for Makibox hot bed)
// 13 is 100k Hisens 3950  1% up to 300°C for hotend "Simple ONE " & "Hotend "All In ONE"
// 20 is the PT100 circuit found in the Ultimainboard V2.x
// 60 is 100k Maker's Tool Works Kapton Bed Thermistor beta=3950
//
//    1k ohm pullup tables - This is not normal, you would have to have changed out your 4.7k for 1k
//                          (but gives greater accuracy and more stable PID)
// 51 is 100k thermistor - EPCOS (1k pullup)
// 52 is 200k thermistor - ATC Semitec 204GT-2 (1k pullup)
// 55 is 100k thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (1k pullup)
//
// 1047 is Pt1000 with 4k7 pullup
// 1010 is Pt1000 with 1k pullup (non standard)
// 147 is Pt100 with 4k7 pullup
// 110 is Pt100 with 1k pullup (non standard)
// 998 and 999 are Dummy Tables. They will ALWAYS read 25°C or the temperature defined below.
// 998和999是虚表。虚拟热敏电阻恒温读数
// Use it for Testing or Development purposes. NEVER for production machine.用于测试或开发目的。从来没有为生产机器。
//#define DUMMY_THERMISTOR_998_VALUE 25      //定义虚拟热敏电阻998值25
//#define DUMMY_THERMISTOR_999_VALUE 100     //定义虚拟热敏电阻999值100
/*:{ '0': "Not used", '4': "10k !! do not use for a hotend. Bad resolution at high temp. !!", '1':
"100k / 4.7k - EPCOS", '51': "100k / 1k - EPCOS", '6': "100k / 4.7k EPCOS - Not as accurate as Table 1",
'5': "100K / 4.7k - ATC Semitec 104GT-2 (Used in ParCan & J-Head)", '7': "100k / 4.7k Honeywell 135-104LAG-J01", '71':
"100k / 4.7k Honeywell 135-104LAF-J01", '8': "100k / 4.7k 0603 SMD Vishay NTCS0603E3104FXT", '9': 
"100k / 4.7k GE Sensing AL03006-58.2K-97-G1", '10': "100k / 4.7k RS 198-961", '11': "100k / 4.7k beta 3950 1%", '12':
"100k / 4.7k 0603 SMD Vishay NTCS0603E3104FXT (calibrated for Makibox hot bed)", '13': "100k Hisens 3950  1% up to 300°C
for hotend 'Simple ONE ' & hotend 'All In ONE'", '60': "100k Maker's Tool Works Kapton Bed Thermistor beta=3950", '55':
"100k / 1k - ATC Semitec 104GT-2 (Used in ParCan & J-Head)", '2': "200k / 4.7k - ATC Semitec 204GT-2", '52': 
"200k / 1k - ATC Semitec 204GT-2", '-2': "Thermocouple + MAX6675 (only for sensor 0)", '-1': "Thermocouple + AD595", '3':
"Mendel-parts / 4.7k", '1047': "Pt1000 / 4.7k", '1010': "Pt1000 / 1k (non standard)", '20': "PT100 (Ultimainboard V2.x)", '147':
"Pt100 / 4.7k", '110': "Pt100 / 1k (non-standard)", '998': "Dummy 1", '999': "Dummy 2" }*/
#define TEMP_SENSOR_0 1
#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_3 0
#define TEMP_SENSOR_BED 1

// This makes temp sensor 1 a redundant sensor for sensor 0.
//这使得温度传感器1为传感器0的冗余传感器 
//If the temperatures difference between these sensors is to high the print will be aborted.
//如果这些传感器之间的温度差很高，打印就会中止。
//#define TEMP_SENSOR_1_AS_REDUNDANT         //将温度传感器1定义为冗余
#define MAX_REDUNDANT_TEMP_SENSOR_DIFF 10    //最大冗余温度传感器差异

// Actual temperature must be close to target for this long before M109 returns success
// 在m109返回成功之前，实际温度必须接近目标
#define TEMP_RESIDENCY_TIME 10  // (seconds)临时居住时间10s
#define TEMP_HYSTERESIS 3       //温度滞后3度  (degC) range of +/- temperatures considered "close" to the target one
                                //+/-被认为"接近"目标温度的范围
#define TEMP_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.

// The minimal temperature defines the temperature below which the heater will not be enabled It is used
//定义加热器不能启用的最低温度。 
// to check that the wiring to the thermistor is not broken.检查热敏电阻接线是否断开
// Otherwise this would lead to the heater being powered on all the time.否则，这将导致加热器一直通电
#define HEATER_0_MINTEMP 5
#define HEATER_1_MINTEMP 5
#define HEATER_2_MINTEMP 5
#define HEATER_3_MINTEMP 5
#define BED_MINTEMP 5

// When temperature exceeds max temp, your heater will be switched off.当温度超过最高温度时，加热器将关闭
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
//这个功能的存在是为了保护你的喷头过热，一不小心，但不要从热敏电阻短路/失败
// You should use MINTEMP for thermistor short/failure protection./你应该用最小温度热敏电阻短路/故障保护
#define HEATER_0_MAXTEMP 275
#define HEATER_1_MAXTEMP 275
#define HEATER_2_MAXTEMP 275
#define HEATER_3_MAXTEMP 275
#define BED_MAXTEMP 120

// If your bed has low resistance e.g. .6 ohm and throws the fuse you can duty cycle it to reduce the
// average current. The value should be an integer and the heat bed will be turned on for 1 interval of
// HEATER_BED_DUTY_CYCLE_DIVIDER intervals.
/*如果您的床具有低电阻，例如0.6欧姆，并抛出保险丝，您可以将其占空比降低平均电流。该值应为整数，加热床将打开1个间隔
加热器_床_占空比_分隔器间隔。定义加热器床占空比分配器4*/
//#define HEATER_BED_DUTY_CYCLE_DIVIDER 4  //定义加热器床占空比分配器4

// If you want the M105 heater power reported in watts, define the BED_WATTS, and (shared for all extruders) EXTRUDER_WATTS
//如果您希望M105加热器的功率以瓦特为单位报告，请定义床瓦特和（所有挤出机共用的）挤出机瓦特
//#define EXTRUDER_WATTS (12.0*12.0/6.7) //  P=I^2/R 定义挤出机功率
//#define BED_WATTS (12.0*12.0/1.1)      // P=I^2/R定义热床功率

//===========================================================================
//========================== PID Settings PID设置 ===========================
//===========================================================================
// PID Tuning Guide here: http://reprap.org/wiki/PID_Tuning  这里的PID调优指南

// Comment the following line to disable PID and enable bang-bang.
//决定了目前的温度控制模式使用PID模式。如果把这行注释掉，就代表不用PID，而是用简单控制模式（Bang bang）。
//简单控制模式的特点就是简单，没有控制参数，基本上加热器的工作方式就是当前温度小于目标温度就打开，反之就关闭
#define PIDTEMP
#define BANG_MAX 255 // limits current to nozzle while in bang-bang mode; 255=full current
                    //在砰砰模式下限制到喷嘴的电流;255=全电流
#define PID_MAX BANG_MAX // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
                   //pid处于活动状态时限制到喷嘴的电流(参见下面的pid_函式范围)；255=全电流
#if ENABLED(PIDTEMP)
  //#define PID_DEBUG // Sends debug data to the serial port.将调试数据发送到串行端口。
  //#define PID_OPENLOOP 1 // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX
                           //在开环的时候。m104/m140将输出功率从0设为pid_max
  //#define SLOW_PWM_HEATERS // 缓慢的加热器 PWM with very low frequency (roughly 0.125Hz=8s) and minimum state time of approximately 1s useful for heaters driven by a relay
  //甚低频率pwm（大约0.125hz=8S），最小状态时间约为1s，对继电器驱动的加热器有用
  //#define PID_PARAMS_PER_EXTRUDER // Uses separate PID parameters for each extruder (useful for mismatched extruders)
                                    //对每个挤出机使用单独的pid参数（对不匹配的挤出机有用）
                                    // Set/get with gcode: M301 E[extruder number, 0-2] 设定/得到gcode: M301 E[挤出机编号,0-2]
  #define PID_FUNCTIONAL_RANGE 10 /*If the temperature difference between the target temperature and the actual temperature
is more then PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.
如果目标温度与实际温度之间的温差超过了pid/函数范围，则PID将关闭，加热器将设置为最小/最大值*/
  #define PID_INTEGRAL_DRIVE_MAX PID_MAX  //limit for the integral term积分项的限制
  #define K1 0.95 //smoothing factor within the PID  //PID内的平滑因子

  // If you are using a pre-configured hotend then you can use one of the value sets by uncommenting it
 // 如果您使用的是预先配置好的启动程序，那么您可以通过不评论它来使用其中的一个值集
  // Ultimaker
  #define  DEFAULT_Kp 22.2
  #define  DEFAULT_Ki 1.08
  #define  DEFAULT_Kd 114

  // MakerGear
  //#define  DEFAULT_Kp 7.0
  //#define  DEFAULT_Ki 0.1
  //#define  DEFAULT_Kd 12

  // Mendel Parts V9 on 12V
  //#define  DEFAULT_Kp 63.0
  //#define  DEFAULT_Ki 2.25
  //#define  DEFAULT_Kd 440

#endif // PIDTEMP

//===========================================================================
//=================PID > Bed Temperature Control 热床温度控制 ===============
//===========================================================================
// Select PID or bang-bang with PIDTEMPBED. If bang-bang, BED_LIMIT_SWITCHING will enable hysteresis
//选择PID或“bang-bang”混合模式。如果bang-bang，热床限制开关将启用滞后
// Uncomment this to enable PID on the bed. It uses the same frequency PWM as the extruder.
//取消注释以在床上启用PID。它使用与挤出机相同的频率PWM
// If your PID_dT is the default, and correct for your hardware/configuration, that means 7.689Hz,
//如果您的PID _ dT是默认的，并且适合您的硬件/配置，则意味着7.689Hz
// which is fine for driving a square wave into a resistive load and does not significantly impact you FET heating.
//这对于将方波驱动到电阻负载中很好，并且不会显着影响FET加热
// This also works fine on a Fotek SSR-10DA Solid State Relay into a 250W heater.
//Fotek SSR - 10DA固态继电器在250瓦加热器上也能正常工作
// If your configuration is significantly different than this and you don't understand the issues involved, you probably
//如果您的配置明显不同于此，并且您不了解所涉及的问题，则可能在其他人验证硬件工作之前，不应该使用床PID。
// shouldn't use bed PID until someone else verifies your hardware works.
// If this is enabled, find your own PID constants below.如果启用此选项，请在下面找到您自己的PID常量。
//#define PIDTEMPBED  //定义临时数据库

//#define BED_LIMIT_SWITCHING  //定义热床_限制_切换

// This sets the max power delivered to the bed, and replaces the HEATER_BED_DUTY_CYCLE_DIVIDER option.
//这将设置输送到床的最大功率，并替换加热器床占空比分配器选项
// all forms of bed control obey this (PID, bang-bang, bang-bang with hysteresis)所有形式的热床控制都遵循这一点
// setting this to anything other than 255 enables a form of PWM to the bed just like HEATER_BED_DUTY_CYCLE_DIVIDER did,
//将该值设置为255以外的任何值都能够实现对床的PWM形式，就像加热器床占空比分配器那样，
// so you shouldn't use it unless you are OK with PWM on your bed.  (see the comment on enabling PIDTEMPBED)
//所以你不应该使用它，除非你对床上的PWM很满意。关于启用PIDTEMPBED的评论
#define MAX_BED_POWER 255 // limits duty cycle to bed; 255=full current定义最大床功率255




//#define PID_BED_DEBUG // Sends debug data to the serial port.将调试数据发送到串行端口。

#if ENABLED(PIDTEMPBED)  //如果启用（PID临时温度）

  #define PID_BED_INTEGRAL_DRIVE_MAX MAX_BED_POWER //limit for the integral term积分项的极限

  //120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)120伏250瓦硅胶加热器，4毫米硼硅酸盐（门德尔麦克斯1.5 +）
  //from FOPDT model - kp=.39 Tp=405 Tdead=66, Tc set to 79.2, aggressive factor of .15 (vs .1, 1, 10)
  #define  DEFAULT_bedKp 10.00
  #define  DEFAULT_bedKi .023
  #define  DEFAULT_bedKd 305.4

  //120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)
  //from pidautotune
  //#define  DEFAULT_bedKp 97.1
  //#define  DEFAULT_bedKi 1.41
  //#define  DEFAULT_bedKd 1675.16

  // FIND YOUR OWN: "M303 E-1 C8 S90" to run autotune on the bed at 90 degreesC for 8 cycles.
 //找到您自己的“M303 E - 1 C8 S90”，以90度在床上运行自动调谐8个周期。
#endif // PIDTEMPBED

// @section extruder  截面挤出机

//this prevents dangerous Extruder moves, i.e. if the temperature is under the limit can be software-disabled for whatever purposes by
//这防止了危险的挤出机移动，如果温度低于极限，无论出于什么目的，都可以通过以下方式禁用软件
#define PREVENT_DANGEROUS_EXTRUDE  //定义防止危险挤出
//if PREVENT_DANGEROUS_EXTRUDE is on, you can still disable (uncomment) very long bits of extrusion separately.
//如果“防止危险挤压”处于启用状态，则仍然可以单独禁用（取消注释）非常长的挤压位
#define PREVENT_LENGTHY_EXTRUDE   //定义防止长时间挤出

#define EXTRUDE_MINTEMP 170  //定义挤出最小温度170
#define EXTRUDE_MAXLENGTH (X_MAX_LENGTH+Y_MAX_LENGTH) //prevent extrusion of very large distances.
    //该值限制挤出的最大长度，超过该长度，挤出机不动作

//===========================================================================
//================= Thermal Runaway Protection 热失控保护 ===================
//===========================================================================

/*
 * Thermal Runaway Protection protects your printer from damage and fire if a thermistor falls out or temperature sensors fail in any way.
 * The issue: If a thermistor falls out or a temperature sensor fails,
 * Marlin can no longer sense the actual temperature. Since a disconnected thermistor reads as a low temperature, the firmware will keep the heater on.
 * The solution: Once the temperature reaches the target, start observing.
 * If the temperature stays too far below the target (hysteresis) for too long,the firmware will halt as a safety precaution. 
 *热失控保护保护您的打印机不受损坏和火灾，如果热敏电阻脱落或温度传感器以任何方式失败。             
 *问题：如果热敏电阻掉了，或者温度传感器失灵了，         
 *马林再也感受不到实际温度了。由于断开热敏电阻读为低温，固件将保持加热器上。               
 *解决办法：一旦温度达到目标，开始观察。             
 *如果温度保持低于目标（滞后）太久，固件将停止作为一个安全预防措施。 */

//#define THERMAL_PROTECTION_HOTENDS // Enable thermal protection for all extruders使所有的挤出机热保护 
//#define THERMAL_PROTECTION_BED     // Enable thermal protection for the heated bed使所有的热床保护 

//===========================================================================
//======================= Mechanical Settings 机械设置 ======================
//===========================================================================

// @section machine

// Uncomment this option to enable CoreXY kinematics //取消注释此选项以启用CoreXY运动学
//#define COREXY

// Uncomment this option to enable CoreXZ kinematics //取消注释此选项以启用CoreXZ运动学
//#define COREXZ

// Enable this option for Toshiba steppers为东芝步进器启用此选项
//#define CONFIG_STEPPERS_TOSHIBA   //定义配置步进器东芝

// @section homing

// coarse Endstop Settings  极限终点设置
#define ENDSTOPPULLUPS // Comment this out (using // at the start of the line) to disable the endstop pullup resistors
    //限位开关上拉电阻配置，如果你使用的是机械式的限位开关，请保留此部分
#if DISABLED(ENDSTOPPULLUPS)  //没有配置限位开关上拉电阻时，限位开关上拉电阻细分控制
  // fine endstop settings: Individual pullups. will be ignored if ENDSTOPPULLUPS is defined 精细终点设置：单个上​​拉如果定义了结束停止，将忽略
  //#define ENDSTOPPULLUP_XMAX
  //#define ENDSTOPPULLUP_YMAX
  //#define ENDSTOPPULLUP_ZMAX
  //#define ENDSTOPPULLUP_XMIN
  //#define ENDSTOPPULLUP_YMIN
  //#define ENDSTOPPULLUP_ZMIN
  //#define ENDSTOPPULLUP_ZMIN_PROBE
#endif

// Mechanical endstop with COM to ground and NC to Signal uses "false" here (most common setup).true
//机械终点挡板（COM接地，NC发信号）在此处使用“假”（最常见的设置）。真实的
const bool X_MIN_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.设置为真以反转终点挡板的逻辑
const bool Y_MIN_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.设置为真以反转终点挡板的逻辑
const bool Z_MIN_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.设置为真以反转终点挡板的逻辑
const bool X_MAX_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.设置为真以反转终点挡板的逻辑
const bool Y_MAX_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.设置为真以反转终点挡板的逻辑
const bool Z_MAX_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.设置为真以反转终点挡板的逻辑
const bool Z_MIN_PROBE_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.设置为真以反转终点挡板的逻辑
//#define DISABLE_MAX_ENDSTOPS  //定义禁用最大终点挡板，如果你的打印机的原点开关安装在X、Y、Z轴最大位置的话，这个就需要去掉注释符号
//#define DISABLE_MIN_ENDSTOPS

/* If you want to enable the Z probe pin, but disable its use, uncomment the line below.
This only affects a Z probe endstop if you have separate Z min endstop as well and have 
 activated Z_MIN_PROBE_ENDSTOP below. If you are using the Z Min endstop on your Z probe,this has no effect.
如果要启用Z探针，但禁用其使用，请取消注释下面的行，这只在您也有单独的Z最小终点并且有
已在下面激活Z最小探针终点停止。如果使用Z探针上的ž最小终点挡板,这没有效果。*/
#define DISABLE_Z_MIN_PROBE_ENDSTOP
// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
// :{0:'Low',1:'High'} 反相步进使引脚（活动低）使用0，非反相（活动高）使用1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // For all extruders

// Disables axis when it's not being used.当轴不被使用时，它就会被禁用。
// WARNING: When motors turn off there is a chance of losing position accuracy!
//警告：当马达关闭时，有可能丢失位置精度！
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
//通常情况下上面的代码是不改动的，所有轴都是选择false的。然而，如果你的3d打印机z轴有手动调整的部件，可以在
//#define DISABLE_Z 行改为true ，这样在打印机打印时，可以手动调整z轴

// @section extruder

#define DISABLE_E false // For all extruders
#define DISABLE_INACTIVE_EXTRUDER true //disable only inactive extruders and keep active extruder enabled
//定义禁用非活动挤出机   仅禁用非活动挤出机，并保持启用活动挤出机

// @section machine 分段机

// Invert the stepper direction. Change (or reverse the motor connector) if an axis goes the wrong way.
#define INVERT_X_DIR true//false
#define INVERT_Y_DIR true
#define INVERT_Z_DIR false//true
   //修改对应某个轴的配置（true或false）后，电机会反向
// @section extruder
// For direct drive extruder v9 set to true, for geared extruder set to false.
//用于直接驱动挤出机V9设置为真，用于齿轮挤出机设置为假。
#define INVERT_E0_DIR true//false
#define INVERT_E1_DIR false
#define INVERT_E2_DIR false
#define INVERT_E3_DIR false
//这部分是决定3d打印机各个轴的运行方向的，默认的选项不一定适合每一种打印机，所以需要测试这部分。

// @section homing   部分归航

// ENDSTOP SETTINGS: 终端设置
// Sets direction of endstops when homing; 1=MAX, -1=MIN   //回原点方向配置。如果原点位置为最小值配置为-1，如果原点位置为最大值配置为1
// :[-1,1]
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1

#define min_software_endstops true  // If true, axis won't move to coordinates less than HOME_POS.软件限位点
                                    //如果为真，轴将不会移动到小于家庭坐标
#define max_software_endstops true  // If true, axis won't move to coordinates greater than the defined lengths below.
                                    //如果为真，轴将不会移动到大于下面定义长度的坐标

// @section machine
/*这几个参数是配置打印尺寸的重要参数。这里需要说明的是坐标原点并不是打印中心，真正的打印中心一般在
[(x.max-x.min)/2,(y.max-y.min)/2]的位置。中心位置的坐标需要在后面的切片工具中使用到，打印中心坐
标应该与这里的参数配置匹配，否则很可能会打印到平台以外*/
// Travel limits after homing (units are in mm) 回零后的行程限制(单位：毫米)
#define X_MIN_POS 0
#define Y_MIN_POS 0
#define Z_MIN_POS 0
#define X_MAX_POS 270
#define Y_MAX_POS 270
#define Z_MAX_POS 430
//#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS) 
//define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS) 
//define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)

//===========================================================================
//================== Filament Runout Sensor灯丝磨光传感器 ===================
//===========================================================================
//#define FILAMENT_RUNOUT_SENSOR // Uncomment for defining a filament runout sensor such as a mechanical or opto endstop to check the existence of filament
                                 // In RAMPS uses servo pin 2. Can be changed in pins file. For other boards pin definition should be made.
                                 // It is assumed that when logic high = filament available;when logic  low = filament ran out
//定义灯丝_运行_传感器 用于定义灯丝耗尽传感器，如机械或光电元件端子，以检查灯丝的存在
//在RAMPS中使用伺服引脚2。可以在引脚文件中更改。对于其他板脚的定义应作出。
//假设当逻辑高=灯丝可用时;当逻辑低=灯丝耗尽时
#if ENABLED(FILAMENT_RUNOUT_SENSOR)  //如果启用（灯丝_运行_传感器）
  const bool FIL_RUNOUT_INVERTING = true;  // Should be uncommented and true or false should assigned 应注释应该分配真或假
  #define ENDSTOPPULLUP_FIL_RUNOUT // Uncomment to use internal pullup for filament runout pins if the sensor is defined.
      // 定义结束停止上拉_灯丝_运行  如果传感器已经定义，请不要评论是否使用内部拉升来制造灯丝外露引脚。
  #define FILAMENT_RUNOUT_SCRIPT "M600"   //定义灯丝耗尽脚本
#endif

//===========================================================================
//=========================== Manual Bed Leveling 人工基床整平 ===========================
//===========================================================================

//#define MANUAL_BED_LEVELING  // Add display menu option for bed leveling.添加热床调平显示菜单选项
//#define MESH_BED_LEVELING    // Enable mesh bed leveling.启用网孔热床找平

#if ENABLED(MANUAL_BED_LEVELING) //开启此选项的话(手动平铺)
  #define MBL_Z_STEP 0.025  // Step size while manually probing Z axis.手动探测z轴时的步长。
#endif  // MANUAL_BED_LEVELING  手动调平

#if ENABLED(MESH_BED_LEVELING)  //开启此选项的话(网眼床平整)
  #define MESH_MIN_X 10
  #define MESH_MAX_X (X_MAX_POS - MESH_MIN_X)
  #define MESH_MIN_Y 10
  #define MESH_MAX_Y (Y_MAX_POS - MESH_MIN_Y)
  #define MESH_NUM_X_POINTS 3  // Don't use more than 7 points per axis, implementation limited.
                               // 每轴使用不超过7点，实现受限。
  #define MESH_NUM_Y_POINTS 3
  #define MESH_HOME_SEARCH_Z 4  // Z after Home, bed somewhere below but above 0.0.Z轴会零之前，但要高于热床多少
#endif  // MESH_BED_LEVELING

//===========================================================================
//============================ Bed Auto Leveling自动调平 ============================
//===========================================================================

// @section bedlevel

#define AUTO_BED_LEVELING_FEATURE // Delete the comment to enable (remove // at the start of the line)自动找平功能
//#define DEBUG_LEVELING_FEATURE
 //调试的平衡功能
#define Z_MIN_PROBE_REPEATABILITY_TEST  // If not commented out, Z-Probe Repeatability test will be included if Auto Bed Leveling is Enabled.
//定义最小探针重复性试验
#if ENABLED(AUTO_BED_LEVELING_FEATURE)
//如果启用（自动床整平功能）
  // There are 2 different ways to specify probing locations:
  //有2种不同的方法来指定探测位置
  // - "grid" mode  //网格”模式
  //   Probe several points in a rectangular grid.   //探测矩形网格中的几个点。
  //   You specify the rectangle and the density of sample points.   //指定矩形和采样点的密度
  //   This mode is preferred because there are more measurements.   //此模式是首选，因为有更多的测量
  //
  // - "3-point" mode   //点”模式
  //   Probe 3 arbitrary points on the bed (that aren't colinear)    //探头3任意点在床上（不共线）
  //   You specify the XY coordinates of all 3 points.    //指定所有3个点的XY坐标

  // Enable this to sample the bed in a grid (least squares solution). //使这个可以在网格中取样（最小二乘解）。 
  // Note: this feature generates 10KB extra code size.
  #define AUTO_BED_LEVELING_GRID

  #if ENABLED(AUTO_BED_LEVELING_GRID)   //自动找平网格 

    #define LEFT_PROBE_BED_POSITION 20    //左探针床的位置
    #define RIGHT_PROBE_BED_POSITION 170   //右探针床的位置
    #define FRONT_PROBE_BED_POSITION 20    //前探针床的位置
    #define BACK_PROBE_BED_POSITION 170   //后探针床的位置

    #define MIN_PROBE_EDGE 10 // The Z probe minimum square sides can be no smaller than this.
     //定义最小探测边缘10 / Z探头最小平方面不能小于这。
    // Set the number of grid points per dimension.  //设置每个维度的网格点数 
    // You probably don't need more than 3 (squared=9).  //您可能不需要超过3（平方= 9）。              
    #define AUTO_BED_LEVELING_GRID_POINTS 2    //自动调整热床网格重点

  #else  // !AUTO_BED_LEVELING_GRID

      // Arbitrary points to probe./ /的四点探针。             
      // A simple cross-product is used to estimate the plane of the bed.  
      //一个简单的相反交错而行被用来估算的热床平面
      #define ABL_PROBE_PT_1_X 15
      #define ABL_PROBE_PT_1_Y 180
      #define ABL_PROBE_PT_2_X 15
      #define ABL_PROBE_PT_2_Y 20
      #define ABL_PROBE_PT_3_X 170
      #define ABL_PROBE_PT_3_Y 20

  #endif // AUTO_BED_LEVELING_GRID  自动找平网

  // Offsets to the Z probe relative to the nozzle tip.  //相对于喷嘴尖端到z探头的偏移量
  // X and Y offsets must be integers.   //x和y偏移量必须是整数
  #define X_PROBE_OFFSET_FROM_EXTRUDER 0     // Z probe to nozzle X offset: -left  +right
  #define Y_PROBE_OFFSET_FROM_EXTRUDER 0     // Z probe to nozzle Y offset: -front +behind
  #define Z_PROBE_OFFSET_FROM_EXTRUDER -0.2  // Z probe to nozzle Z offset: -below (always!)

  #define Z_RAISE_BEFORE_HOMING 8       // (in mm) Raise Z axis before homing (G28) for Z probe clearance.
                                        // Be sure you have this distance over your Z_MAX_POS in case.
                                        //提高Z轴归位前（G28）Z探头间隙。 //确保你有这个距离在你的z_max_pos案例

  #define XY_TRAVEL_SPEED 3000         // X and Y axis travel speed between probes, in mm/min.
  //定义XY移动速度3000                // X和Y轴行程速度之间的探针，在毫米/分钟。
  #define Z_RAISE_BEFORE_PROBING 15   // How much the Z axis will be raised before traveling to the first probing point.
   //定义Z之前提高探测15              //Z轴出行前第一个探测点需要提高15。
  #define Z_RAISE_BETWEEN_PROBINGS 5  // How much the Z axis will be raised when traveling from between next probing points.
  //定义在运行到第二个喷头探测点之间Z轴提高5     
  #define Z_RAISE_AFTER_PROBING 15    // How much the Z axis will be raised after the last probing point.
      //z轴在最后一个探测点之后会上升15。 
//#define Z_PROBE_END_SCRIPT "G1 Z10 F12000\nG1 X15 Y330\nG1 Z0.5\nG1 Z10" // These commands will be executed in the end of G29 routine.
// #定义Z探头端脚本“G1 Z10 f12000 \ NG1 X15 y330 \ NG1 z0.5 \ NG1 Z10”/这些命令将在29例行执行结束        // Useful to retract a deployable Z probe.

  //#define Z_PROBE_SLED // Turn on if you have a Z probe mounted on a sled like those designed by Charles Bell.
  //#define SLED_DOCKING_OFFSET 5 // The extra distance the X axis must travel to pickup the sled. 0 should be fine but you can push it further if you'd like.


  //If you have enabled the Bed Auto Leveling and are using the same Z Probe for Z Homing,
  //如果您已经启用了床自动找平，并使用Z z引导相同的Z探针，              
  //it is highly recommended you let this Z_SAFE_HOMING enabled!!!  //强烈建议您启用此Z安全引导

  #define Z_SAFE_HOMING   // This feature is meant to avoid Z homing with Z probe outside the bed area.
                          //这个功能是为了避免自动导向Z探针热床外的地区
                          // When defined, it will:当定义时，它将： //只允许X和Y归位，而步进驱动器仍然启用。
                          // - Allow Z homing only after X and Y homing AND stepper drivers still enabled.
                          // - If stepper drivers timeout, it will need X and Y homing again before Z homing.
                          //如果步进驱动器超时，它将需要x和y在Z引导之前再次归位。
                          // - Position the Z probe in a defined XY point before Z Homing when homing all axis (G28).
                          //位置Z探针在定义XY点之前Z归零时所有轴（G28）。 
                          // - Block Z homing only when the Z probe is outside bed area.//z仅在喷头位于床区外时才归位

  #if ENABLED(Z_SAFE_HOMING)   //取消该行注释，可将热床中心定义为X=0，Y=0 

    #define Z_SAFE_HOMING_X_POINT ((X_MIN_POS + X_MAX_POS) / 2)    // X point for Z homing when homing all axis (G28).
    #define Z_SAFE_HOMING_Y_POINT ((Y_MIN_POS + Y_MAX_POS) / 2)    // Y point for Z homing when homing all axis (G28).

  #endif

  // Support for a dedicated Z probe endstop separate from the Z min endstop.
  //支持一个专门的z探针端子，该端子与zmin端子分离
  // If you would like to use both a Z probe and a Z min endstop together,
  //如果您想同时使用z探针和zmin端点，请不要评论定义zmin探针端点并阅读下面的说明。
  // uncomment #define Z_MIN_PROBE_ENDSTOP and read the instructions below.
  // If you still want to use the Z min endstop for homing, disable Z_SAFE_HOMING above.
  // 如果您仍然想要使用Zmin端子进行归航，请禁用上面的z_safe_归航。
  // Example: To park the head outside the bed area when homing with G28.例如：与g 28同归于尽时，将头部停在床区外。
  // WARNING:
  // The Z min endstop will need to set properly as it would without a Z probe to prevent head crashes and premature stopping during a print.
  //zmin端点将需要像没有z探针那样正确设置，以防止打印过程中的头部碰撞和过早停止。
  // To use a separate Z probe endstop, you must have a Z_MIN_PROBE_PIN defined in the pins_XXXXX.h file for your control board.
  //要使用单独的z探针端子，必须在控件的pins_xxxx.h文件中定义一个z min探针引脚。
  // If you are using a servo based Z probe, you will need to enable NUM_SERVOS,Z_ENDSTOP_SERVO_NR and SERVO_ENDSTOP_ANGLES in the R/C SERVO support below.
  //如果您使用的是基于伺服的z探针，则需要在下面的/c伺服支持中启用数字伺服、z端点伺服和伺服端点角。
  // RAMPS 1.3/1.4 boards may be able to use the 5V, Ground and the D32 pin in the Aux 4 section of the RAMPS board. Use 5V for powered sensors,
  //RAMPS1.3/1.4板可以使用RAMPS的aux 4部分中的5v、地和d32引脚。在动力传感器上使用5v，
  // otherwise connect to ground and D32 for normally closed configuration and 5V and D32 for normally open configurations.
  //否则，对于通常闭合的配置，连接到地面和d32，对于正常打开的配置，连接到5v和d32。
  // Normally closed configuration is advised and assumed.建议并假设通常是封闭的配置。
  // The D32 pin in Aux 4 on RAMPS maps to the Arduino D32 pin.Z_MIN_PROBE_PIN is setting the pin to use on the Arduino.
  //在通往Arduino的斜面地图上的aux 4中的d32引脚正在设置用于Arduino的引脚。
  // Since the D32 pin on the RAMPS maps to D32 on Arduino, this works.D32 is currently selected in the RAMPS 1.3/1.4 pin file.
  //由于RAMPS主板电路图上的d32引脚到Arduino上的d32，这个作品s.d32目前被选在RAMPS1.3/1.4引脚文件中。
  // All other boards will need changes to the respective pins_XXXXX.h file.
  //所有其他的面板都需要对相应的pins_xxxx.h文件进行修改。
  // WARNING:
  // Setting the wrong pin may have unexpected and potentially disastrous outcomes. Use with caution and do your homework.
  //设置错误的引脚可能有意想不到的和潜在的灾难性的结果。小心使用，做你的作业。
  //#define Z_MIN_PROBE_ENDSTOP  //定义Zmin探针端子

#endif // AUTO_BED_LEVELING_FEATURE 自动床平整功能


// @section homing
//手动设置回原点位置，如果要使用该功能，请将下面#define MANUAL_HOME_POSITIONS前的“//”删除。
//使用该功能后，默认回原点位置将是你所设定的以下三个值的位置

// The position of the homing switches 自导开关的位置
//#define MANUAL_HOME_POSITIONS  // If defined, MANUAL_*_HOME_POS below will be used如已定义，将使用下面的手工家庭字段
//#define BED_CENTER_AT_0_0  // If defined, the center of the bed is at (X=0, Y=0)如果定义的话，床的中心在(x=0,y=0)

// Manual homing switch locations:   //手动归零开关位置
// For deltabots this means top and center of the Cartesian print volume.对于德尔塔伯特，这意味着顶部和中心的笛卡尔打印量。
#if ENABLED(MANUAL_HOME_POSITIONS)
  #define MANUAL_X_HOME_POS 0
  #define MANUAL_Y_HOME_POS 0
  #define MANUAL_Z_HOME_POS 0
  //#define MANUAL_Z_HOME_POS 402 // 
  // For delta: Distance between nozzle and print surface after homing.
  //对于三角洲：喷嘴和印刷表面之间的距离归位后
#endif

#endif

// @section movement 分段移动

/**
 * MOVEMENT SETTINGS 移动设置
 */
#define NUM_AXIS 4 // The axis order in all axis related arrays is X, Y, Z, E
 //所有与轴有关的数组中的轴序都是x,y,z,e
#define HOMING_FEEDRATE {50*60, 50*60, 4*60, 0}  // set the homing speeds (mm/min)
   //上面配置为回原点的速度，可根据实际情况做相应调整。单位是mm/min
   
// default settings 默认设置
#define DEFAULT_AXIS_STEPS_PER_UNIT   {80.00,80.00,400.00,93.00}  // default steps per unit for Ultimaker
/*这个参数是打印机打印尺寸是否正确的最重要参数，参数含义为各轴运行1mm所需要的脉冲数，分别对应x,y,z,e四轴。
多数情况下这个数字都需要自己计算才可以。同步带传动时的计算公式（X\Y轴）：步进电机每转步数
(1.8度步距角的电机为200,0.9度步距角的电机为400)*步进电机驱动细分配置（一般16细分）/同步带齿间距/同步轮齿数  
丝杠传动时的计算公式（Z轴）：步进电机每转步数*步进电机驱动细分配置/丝杠导程  
挤出机计算公式（E轴）：步进电机每转步数*步进电机驱动细分配置*挤出机齿轮传动比/挤出轮周长 */
#define DEFAULT_MAX_FEEDRATE          {300, 300, 5, 25}    // (mm/sec)
//该配置为各电机最高速度。过高的值需要更大的电流输出，这将导致电机过热，
//并且有可能使电机在打印时丢步。一般可设置在200-400 
#define DEFAULT_MAX_ACCELERATION      {1000,1000,100,5000}    // X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.
//该配置为电机最大加速度。过高的电机加速度将导致电机在打印动作时过冲，
//从而丢步，建议将X/Y最大加速度修改为1000-3000 

#define DEFAULT_ACCELERATION          500    // X, Y, Z and E max acceleration in mm/s^2 for printing moves默认打印加速度
#define DEFAULT_RETRACT_ACCELERATION  500   // X, Y, Z and E max acceleration in mm/s^2 for retracts默认回抽加速度

//#define DEFAULT_AXIS_STEPS_PER_UNIT   {80,80,407,96}  // default steps per unit for Ultimaker 每一单元的默认步骤
//#define DEFAULT_MAX_FEEDRATE          {300, 300, 5, 25}    // (mm/sec)
//#define DEFAULT_MAX_ACCELERATION      {3000,3000,100,6000}    // X, Y, Z, E maximum start speed for accelerated moves. E default values are good for Skeinforge 40+, for older versions raise them a lot.

//#define DEFAULT_ACCELERATION          3000    // X, Y, Z and E acceleration in mm/s^2 for printing moves
//#define DEFAULT_RETRACT_ACCELERATION  3000    // E acceleration in mm/s^2 for retracts
#define DEFAULT_TRAVEL_ACCELERATION 1000        // X, Y, Z acceleration in mm/s^2 for travel (non printing) moves
   //    定义默认的旅行加速                        x,y,z加速(mm/s^2)用于行程(非打印)移动

// The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
//不需要加速的速度变化（即软件可能认为它可以立即完成）
#define DEFAULT_XYJERK                20.0    // (mm/sec)
#define DEFAULT_ZJERK                 0.4     // (mm/sec)
#define DEFAULT_EJERK                 5.0    // (mm/sec)
//该配置为加速度变化率。该值过大同样也会导致电机丢步。当该值处于合理范围内的
//较小值时，打印动作将更平滑、打印机机械应力将更小、材料在换向时将有更好的附着力、
//打印噪声也将降低；当该值处于合理范围内的较大值时，打印时间将缩短。建议谨慎修改该值，最好不动


//=============================================================================
//============================= Additional Features附加功能 ===========================
//=============================================================================

// @section more

// Custom M code points  自定义m代码点
#define CUSTOM_M_CODES   //自定义M代码
#if ENABLED(CUSTOM_M_CODES)
  #if ENABLED(AUTO_BED_LEVELING_FEATURE)   //自动找平功能
    #define CUSTOM_M_CODE_SET_Z_PROBE_OFFSET 851  //自定义m代码集z探头偏移量851
    #define Z_PROBE_OFFSET_RANGE_MIN -20   //定义z探针偏移范围最小- 20
    #define Z_PROBE_OFFSET_RANGE_MAX 20   //定义z探针偏移范围最大 20
  #endif
#endif

// @section extras

// EEPROM
// The microcontroller can store settings in the EEPROM, e.g. max velocity...
// M500 - stores parameters in EEPROM   自动调平数据保存对应marlin-main6030行set_bed_leveling_enabled(true)
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
//define this to enable EEPROM support  定义此以启用EEPROM支持
//#define EEPROM_SETTINGS

#if ENABLED(EEPROM_SETTINGS)
  // To disable EEPROM Serial responses and decrease program space by ~1700 byte: comment this out:
  #define EEPROM_CHITCHAT // Please keep turned on if you can.预加热配置，无需作修改
#endif

//
// M100 Free Memory Watcher M100免费记忆观察者
//
//#define M100_FREE_MEMORY_WATCHER // uncomment to add the M100 Free Memory Watcher for debug purpose
//定义M100自由内存观察者       取消注释以添加用于调试目的的M100自由内存观察者
// @section temperature 截面温度

// Preheat Constants  预热常数
#define PLA_PREHEAT_HOTEND_TEMP 185
#define PLA_PREHEAT_HPB_TEMP 45   //HPB温度预热
#define PLA_PREHEAT_FAN_SPEED 0   // Insert Value between 0 and 255  可控风扇在0和255之间插入值

#define ABS_PREHEAT_HOTEND_TEMP 240
#define ABS_PREHEAT_HPB_TEMP 60
#define ABS_PREHEAT_FAN_SPEED 0   // Insert Value between 0 and 255

//==============================LCD and SD support=============================
//LCD和SD卡配置，请根据你的LCD板子做相应配置
 
// @section lcd

// Define your display language below. Replace (en) with your language code and uncomment.
//定义下面的显示语言。取代（EN）与你的语言代码和注释
// en, pl, fr, de, es, ru, bg, it, pt, pt-br, fi, an, nl, ca, eu, kana, kana_utf8, cn, test
// See also language.h    参见语言编辑目录 language.h
#define LANGUAGE_INCLUDE GENERATE_LANGUAGE_INCLUDE(en)  //#定义语言包括生成语言包括（CN）

// Choose ONE of these 3 charsets. This has to match your hardware. Ignored for full graphic display.
//选择这3个字符集。这必须与你的硬件相匹配。全图形显示忽略
// To find out what type you have - compile with (test) - upload - click to get the menu. You'll see two typical lines from the upper half of the charset.
//要查看您有什么类型的-编译与（测试）-上传-点击以获得菜单。你会看到两个典型的线条从字符集的上半部分。
// See also documentation/LCDLanguageFont.md  参见文档/ lcdlanguagefont.md
  #define DISPLAY_CHARSET_HD44780_JAPAN        // this is the most common hardwarehardware这是最常见的硬件。
  //#define DISPLAY_CHARSET_HD44780_WESTERN
  //#define DISPLAY_CHARSET_HD44780_CYRILLIC

//#define ULTRA_LCD  //general LCD support, also 16x2 一般lcd支援，也有16 x2
//#define DOGLCD  // Support for SPI LCD 128x64 (Controller ST7565R graphic Display Family)支持SPI LCD 128x64
#define SDSUPPORT // Enable SD Card Support in Hardware Console
// Changed behaviour! If you need SDSUPPORT uncomment it!
#define POWEROFF_SAVE_SD_FILE // If power off, after power on, the printing can be resumed.
 //////////////////#定义关机如果断电保存SD文件/打印，上电后，可以恢复。 
//#define SDSLOW // Use slower SD transfer mode (not normally needed - uncomment if you're getting volume init error)
            //////使用教慢的SD传输模式（一般是不需要的信息，如果你要量初始化错误）
//#define SDEXTRASLOW // Use even slower SD transfer mode (not normally needed - uncomment if you're getting volume init error)
//////使用更慢的SD传输模式
//#define SD_CHECK_AND_RETRY // Use CRC checks and retries on the SD communication
//#define ENCODER_PULSES_PER_STEP 1 // Increase if you have a high resolution encoder
    /////#定义编码器脉冲每1步/增加，如果你有一个高分辨率编码器
//#define ENCODER_STEPS_PER_MENU_ITEM 5 // Set according to ENCODER_PULSES_PER_STEP or your liking
//#define ULTIMAKERCONTROLLER //as available from the Ultimaker online store.
//#define ULTIPANEL  //the UltiPanel as on Thingiverse
//#define SPEAKER // The sound device is a speaker - not a buzzer. A buzzer resonates with his own frequency.
/////音响设备是扬声器，不是蜂鸣器。蜂鸣器与自己的频率共振
//#define LCD_FEEDBACK_FREQUENCY_DURATION_MS 100 // the duration the buzzer plays the UI feedback sound. ie Screen Click
//#define LCD_FEEDBACK_FREQUENCY_HZ 1000         // this is the tone frequency the buzzer plays when on UI feedback. ie Screen Click
                                                 // 0 to disable buzzer feedback. Test with M300 S<frequency Hz> P<duration ms>
                                                 //这是蜂鸣器在用户界面反馈时的音调频率。即屏幕上点击 0禁用蜂鸣器反馈。用M300的<频率/Hz> P<时间/ms>
// PanelOne from T3P3 (via RAMPS 1.4 AUX2/AUX3)
// http://reprap.org/wiki/PanelOne
//#define PANEL_ONE  面板

// The MaKr3d Makr-Panel with graphic controller and SD support
// http://reprap.org/wiki/MaKr3d_MaKrPanel
//#define MAKRPANEL

// The Panucatt Devices Viki 2.0 and mini Viki with Graphic LCD
// http://panucatt.com
// ==> REMEMBER TO INSTALL U8glib to your ARDUINO library folder: http://code.google.com/p/u8glib/wiki/u8glib
//#define VIKI2
//#define miniVIKI

// This is a new controller currently under development.  https://github.com/eboston/Adafruit-ST7565-Full-Graphic-Controller/
//这是一个新的控制器，目前正在发展
// ==> REMEMBER TO INSTALL U8glib to your ARDUINO library folder: http://code.google.com/p/u8glib/wiki/u8glib
//#define ELB_FULL_GRAPHIC_CONTROLLER  定义中完整的图形控制器
//#define SD_DETECT_INVERTED

// The RepRapDiscount Smart Controller (white PCB)
// http://reprap.org/wiki/RepRapDiscount_Smart_Controller
//#define REPRAP_DISCOUNT_SMART_CONTROLLER   //（去除这行代码前的//，即代表可使用LCD2004屏）


// The GADGETS3D G3D LCD/SD Controller (blue PCB)
// http://reprap.org/wiki/RAMPS_1.3/1.4_GADGETS3D_Shield_with_Panel
//#define G3D_PANEL

// The RepRapDiscount FULL GRAPHIC Smart Controller (quadratic white PCB)
// http://reprap.org/wiki/RepRapDiscount_Full_Graphic_Smart_Controller
//
// ==> REMEMBER TO INSTALL U8glib to your ARDUINO library folder: http://code.google.com/p/u8glib/wiki/u8glib
///（将\ArduinoAddons\Arduino_1或0.x.x\libraries里面的U8glib文件夹整个拷贝到
//arduino安装目录下\libraries的这个文件夹。 然后关闭arudino，重启一下，再编译。）
#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER   //(去除这行代码前的//，即代表可使用LCD12864屏)


// The RepRapWorld REPRAPWORLD_KEYPAD v1.1
// http://reprapworld.com/?products_details&products_id=202&cPath=1591_1626
//#define REPRAPWORLD_KEYPAD
//#define REPRAPWORLD_KEYPAD_MOVE_STEP 10.0 // how much should be moved when a key is pressed, eg 10.0 means 10mm per click

// The Elefu RA Board Control Panel
// http://www.elefu.com/index.php?route=product/product&product_id=53
// REMEMBER TO INSTALL LiquidCrystal_I2C.h in your ARDUINO library folder: https://github.com/kiyoshigawa/LiquidCrystal_I2C
//#define RA_CONTROL_PANEL

// The MakerLab Mini Panel with graphic controller and SD support
// http://reprap.org/wiki/Mini_panel
//#define MINIPANEL  //迷你板 

/**
 * I2C Panels
 */

//#define LCD_I2C_SAINSMART_YWROBOT

// PANELOLU2 LCD with status LEDs, separate encoder and click inputs
//
// This uses the LiquidTWI2 library v1.2.3 or later ( https://github.com/lincomatic/LiquidTWI2 )
// Make sure the LiquidTWI2 directory is placed in the Arduino or Sketchbook libraries subdirectory.
// (v1.2.3 no longer requires you to define PANELOLU in the LiquidTWI2.h library header file)
// Note: The PANELOLU2 encoder click input can either be directly connected to a pin
//       (if BTN_ENC defined to != -1) or read through I2C (when BTN_ENC == -1).
//#define LCD_I2C_PANELOLU2

// Panucatt VIKI LCD with status LEDs, integrated click & L/R/U/P buttons, separate encoder inputs
//#define LCD_I2C_VIKI
  
// SSD1306 OLED generic display support
// ==> REMEMBER TO INSTALL U8glib to your ARDUINO library folder: http://code.google.com/p/u8glib/wiki/u8glib
//#define U8GLIB_SSD1306

// Shift register panels
// ---------------------
// 2 wire Non-latching LCD SR from:
// https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/schematics#!shiftregister-connection
// LCD configuration: http://reprap.org/wiki/SAV_3D_LCD
//#define SAV_3DLCD

// @section extras

// Increase the FAN pwm frequency. Removes the PWM noise but increases heating in the FET/Arduino
//增加风扇脉宽调制（pwm）频率。消除脉宽调制（PWM）噪声但增加场效应管 （FET ）/ Arduino加热
//#define FAST_PWM_FAN

// Use software PWM to drive the fan, as for the heaters. This uses a very low frequency
// which is not as annoying as with the hardware PWM. On the other hand, if this frequency
// is too low, you should also increment SOFT_PWM_SCALE.
//使用软件脉宽调制（pwm）驱动风扇，如加热器。这使用的频率非常低。
//这不像硬件脉宽调制（pwm）那样令人烦恼。另一方面，如果这个频率太低，你也应该增加软件脉宽调制（pwm）比例。

//#define FAN_SOFT_PWM  软件脉宽调制（pwm）风扇

// Incrementing this by 1 will double the software PWM frequency, 增加这1将双软件PWM频率， 
// affecting heaters, and the fan if FAN_SOFT_PWM is enabled.影响加热器和风扇如果fan_soft_pwm启用
// However, control resolution will be halved for each increment;但是，每增加一次，控制分辨率将减半；
// at zero value, there are 128 effective control positions.在零值时，有128个有效的控制位置。
#define SOFT_PWM_SCALE 0

// Temperature status LEDs that display the hotend and bet temperature.温度状态LED显示hotend和BET的温度
// If all hotends and bed temperature and temperature setpoint are < 54C then the BLUE led is on.
///如果所有喷头温度和热床温度和设定温度低于54c然后蓝色LED上。
// Otherwise the RED led is on. There is 1C hysteresis.否则红色指示灯亮着。有着hysteresi 1C
//#define TEMP_STAT_LEDS    //定义暂态暂态元件

// M240  Triggers a camera by emulating a Canon RC-1 Remote
// M240触发相机，通过模拟一个佳能1远程
// Data from: http://www.doc-diy.net/photo/rc-1_hacked/
//#define PHOTOGRAPH_PIN     23   //定义相机的pin脚为

// SkeinForge sends the wrong arc g-codes when using Arc Point as fillet procedure
//一群假冒设备发出错误的弧G代码时，使用电弧点圆角程序 
//#define SF_ARC_FIX

// Support for the BariCUDA Paste Extruder.对BariCUDA挤出机的支持
//#define BARICUDA

//define BlinkM/CyzRgb Support
//#define BLINKM

/*********************************************************************\
* R/C SERVO support  R/C伺服支持
* Sponsored by TrinityLabs, Reworked by codexmas 由三大实验室赞助，由圣诞节重新制作
**********************************************************************/

// Number of servos  数字伺服系统
//
// If you select a configuration below, this will receive a default value and does not need to be set manually
// set it manually if you have more servos than extruders and wish to manually control some
// leaving it undefined or defining as 0 will disable the servo subsystem
//如果您在下面选择了一个配置，它将收到一个默认值，不需要手动设置。      
//设置手动如果你有比挤出更多的伺服和希望手动控制        
//未定义或定义为0将禁用伺服子系统
// If unsure, leave commented / disabled  如果不确定，请留言/禁用
//
//#define NUM_SERVOS 3 // Servo index starts with 0 for M280 command  M280指令的伺服指数以0开头

// Servo Endstops  停止伺服
//
// This allows for servo actuated endstops, primary usage is for the Z Axis to eliminate calibration or bed height changes.
// Use M851 to set the Z probe vertical offset from the nozzle. Store that setting with M500.
//这使得伺服endstops，主要用途为Z轴，消除校准或床层高度的变化。        
//使用851设置Z探针垂直偏移从喷嘴。商店设置M500
//
//#define X_ENDSTOP_SERVO_NR 1
//#define Y_ENDSTOP_SERVO_NR 2
//#define Z_ENDSTOP_SERVO_NR 0
//#define SERVO_ENDSTOP_ANGLES {{0,0}, {0,0}, {70,0}} // X,Y,Z Axis Extend and Retract angles x,y,z轴延长和缩缩角

// Servo deactivation伺服失灵
//
// With this option servos are powered only during movement, then turned off to prevent jitter.
//此选项仅在运动伺服驱动，然后关闭以防止抖动.
//#define DEACTIVATE_SERVOS_AFTER_MOVE   //定义伺服移动后停用

#if ENABLED(DEACTIVATE_SERVOS_AFTER_MOVE)
  // Delay (in microseconds) before turning the servo off. This depends on the servo speed.延迟（以微秒为单位）关闭伺服系统。这取决于伺服速度。
  // 300ms is a good value but you can try less delay.              300ms是一个很好的价值，但是你可以试着减少延迟
  // If the servo can't reach the requested position, increase it.   如果伺服不能达到所要求的位置然后增加它
  #define SERVO_DEACTIVATION_DELAY 300
#endif

/**********************************************************************\
 * Support for a filament diameter sensor 为灯丝直径传感器提供支持
 * Also allows adjustment of diameter at print time (vs  at slicing) 还允许在印刷时调整直径（相对于切片）
 * Single extruder only at this point (extruder 0) 仅在此时使用单一挤出机(挤出机0)
 *
 * Motherboards 主板
 * 34 - RAMPS1.4 - uses Analog input 5 on the AUX2 connector   34-RAMPS1.4-在辅助/附加接口2连接器上使用模拟输入5
 * 81 - Printrboard - Uses Analog input 2 on the Exp1 connector (version B,C,D,E) 81-打印板-在ex1连接器上使用模拟输入2(b,c,d,e版)
 * 301 - Rambo  - uses Analog input 3 使用模拟输入3
 * Note may require analog pins to be defined for different motherboards 注意可能需要为不同的主板定义模拟引脚
 **********************************************************************/
// Uncomment below to enable
//#define FILAMENT_SENSOR   灯丝传感器

#define FILAMENT_SENSOR_EXTRUDER_NUM 0  
//The number of the extruder that has the filament sensor (0,1,2)  挤出机具有灯丝传感器数目（0,1,2）
#define MEASUREMENT_DELAY_CM        14  
//measurement delay in cm.  This is the distance from filament sensor to middle of barrel
//测量时延以厘米为单位。这是从灯丝传感器到枪管中部的距离

#define DEFAULT_NOMINAL_FILAMENT_DIA 3.00  //Enter the diameter (in mm) of the filament generally used (3.0 mm or 1.75 mm) - this is then used in the slicer software.  Used for sensor reading validation
                                           //输入直径（毫米）一般使用（3毫米或1.75毫米）这是在切片机软件使用。用于传感器读数验证。
#define MEASURED_UPPER_LIMIT         3.30  //upper limit factor used for sensor reading validation in mm
                                           //用于传感器读数验证的上限系数为3.3mm 
#define MEASURED_LOWER_LIMIT         1.90  //lower limit factor for sensor reading validation in mm.
                                           //传感器读数验证的下限系数为1.9mm 
#define MAX_MEASUREMENT_DELAY       20     //delay buffer size in bytes (1 byte = 1cm)- limits maximum measurement delay allowable (must be larger than MEASUREMENT_DELAY_CM  and lower number saves RAM)
                                           //在字节缓冲区的大小（1字节延迟= 1cm）-限制最大测量延迟允许（必须大于measurement_delay_cm下数节省内存） 

//defines used in the code
#define DEFAULT_MEASURED_FILAMENT_DIA  DEFAULT_NOMINAL_FILAMENT_DIA  //set measured to nominal initially最初设置为标称值

//When using an LCD, uncomment the line below to display the Filament sensor data on the last line instead of status.  Status will appear for 5 sec.
//使用LCD时，取消该线下方显示丝传感器数据在最后一行，而不是地位。状态将出现5秒.
//#define FILAMENT_LCD_DISPLAY   //灯丝液晶显示器

#include "Configuration_adv.h"
#include "thermistortables.h"

#endif //CONFIGURATION_H
