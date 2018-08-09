#ifndef CONFIGURATION_ADV_H
#define CONFIGURATION_ADV_H

#include "Conditionals.h"

// @section temperature 部分温度

//===========================================================================
//=========================Thermal Settings 热设置  =========================
//===========================================================================

#if ENABLED(BED_LIMIT_SWITCHING)  //如果启用（床限制切换）
  #define BED_HYSTERESIS 2 //only disable heating if T>target+BED_HYSTERESIS and enable heating if T>target-BED_HYSTERESIS
                           //只有在T>显示热床温度上滞后时才会停止加热，并在T>显示热床温上滞后时能够进行加热
#endif
#define BED_CHECK_INTERVAL 5000 //ms between checks in bang-bang control 在bang-bang控制中的检查之间的毫秒
//定义床检查间隔
/**
 * Thermal Protection parameters 热保护参数
 */
#if ENABLED(THERMAL_PROTECTION_HOTENDS)       //如果已启用（热保护）
  #define THERMAL_PROTECTION_PERIOD 40        // Seconds 定义热保护周期
  #define THERMAL_PROTECTION_HYSTERESIS 4     // Degrees Celsius 定义热保护迟滞 摄氏度

  /**
   * Whenever an M104 or M109 increases the target temperature the firmware will wait for the
   * WATCH_TEMP_PERIOD to expire, and if the temperature hasn't increased by WATCH_TEMP_INCREASE
   * degrees, the machine is halted, requiring a hard reset. This test restarts with any M104/M109,
   * but only if the current temperature is far enough below the target for a reliable test.
   * 每当m104或m109增加目标温度时，固件就会等待手表的温度周期到期，如果温度没有随着手表温度的升高而升高，机器就
   * 会停止工作，需要进行硬重置。此测试以任何m104/m 109重新开始，但前提是目前的温度远低于目标以进行可靠的测试。
   */
  #define WATCH_TEMP_PERIOD 16                // Seconds 定义手表的临时周期
  #define WATCH_TEMP_INCREASE 4               // Degrees Celsius
#endif

#if ENABLED(THERMAL_PROTECTION_BED)           //如果已启用（热保护床）
  #define THERMAL_PROTECTION_BED_PERIOD 20    // Seconds
  #define THERMAL_PROTECTION_BED_HYSTERESIS 2 // Degrees Celsius
#endif

#if ENABLED(PIDTEMP)
  // this adds an experimental additional term to the heating power, proportional to the extrusion speed.
  //这在加热功率上增加了一个实验性的附加项，与挤压速度成正比。
  // if Kc is chosen well, the additional required power due to increased melting should be compensated.
  //如果选择了kc，则应补偿因熔化增加而产生的额外所需功率。
  #define PID_ADD_EXTRUSION_RATE            //定义pid添加挤出率
  #if ENABLED(PID_ADD_EXTRUSION_RATE)
    #define DEFAULT_Kc (100) //heating power=Kc*(e_speed)加热功率=kc*(_速度)
    #define LPQ_MAX_LEN 50
  #endif
#endif

/**
 * Automatic Temperature:
 * The hotend target temperature is calculated by all the buffered lines of gcode.
 * The maximum buffered steps/sec of the extruder motor is called "se".
 * Start autotemp mode with M109 S<mintemp> B<maxtemp> F<factor>
 * The target temperature is set to mintemp+factor*se[steps/sec] and is limited by
 * mintemp and maxtemp. Turn this off by excuting M109 without F*
 * Also, if the temperature is set to a value below mintemp, it will not be changed by autotemp.
 * On an Ultimaker, some initial testing worked with M109 S215 B260 F1 in the start.gcode
 * 自动温度：
*升压目标温度由gcode的所有缓冲线计算。
*挤出机电机的最大缓冲步数/秒称为"se"。
*以M109S（最小温度）B（最大温度）F（系数）启动自动温度模式
*目标温度设置为最小温度+系数*本身[步数/秒]，并受以下条件限制：
*最低温度和最高温度。把这个关掉，把M109去掉，没有f*
*此外，如果温度被设定为一个低于最小温度的值，它将不会被自动温度所改变。
*在一个多倍速制造商上，一些初始测试在开始时使用了M109 S215 B260 F1。
 */
#define AUTOTEMP
#if ENABLED(AUTOTEMP)
  #define AUTOTEMP_OLDWEIGHT 0.98
#endif

//Show Temperature ADC value 显示温度的正负负值
//The M105 command return, besides traditional information, the ADC value read from temperature sensors.
//M105指令返回，除了传统的信息，从温度传感器读取的adc值
//#define SHOW_TEMP_ADC_VALUES

// @section extruder

//  extruder run-out prevention. 挤出机的预防。
//if the machine is idle, and the temperature over MINTEMP, every couple of SECONDS some filament is extruded
//如果机器处于闲置状态，温度超过最低温度，每隔几秒钟就会有一些细丝被挤出
//#define EXTRUDER_RUNOUT_PREVENT  定义挤出机出料防止
#define EXTRUDER_RUNOUT_MINTEMP 190   //定义挤出机的输出温度
#define EXTRUDER_RUNOUT_SECONDS 30.   //定义挤出机运行秒
#define EXTRUDER_RUNOUT_ESTEPS 14. //mm filament 定义挤出机的运行情况。
#define EXTRUDER_RUNOUT_SPEED 1500.  //extrusion speed 定义挤出机输出速度1500。
#define EXTRUDER_RUNOUT_EXTRUDE 100   //定义挤出机输出挤出100

// @section temperature

//These defines help to calibrate the AD595 sensor in case you get wrong temperature measurements.
//这些定义帮助校准ad595传感器以防你得到错误的温度测量。
//The measured temperature is defined as "actualTemp = (measuredTemp * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET"
//测量温度的定义为“实际温度=（测量温度*温度传感器ad595增益）+温度传感器ad595偏移”
#define TEMP_SENSOR_AD595_OFFSET 0.0
#define TEMP_SENSOR_AD595_GAIN   1.0

//This is for controlling a fan to cool down the stepper drivers
//it will turn on when any driver is enabled and turn off after the set amount of seconds from last driver being disabled again
//这是为了控制风扇冷却步行者的驱动程序，当任何驱动程序被启用时，风扇就会打开，并在最后一个驱动程序被再次禁用的规定秒数后关闭
#define CONTROLLERFAN_PIN -1 //Pin used for the fan to cool controller (-1 to disable)风扇冷却控制器用的引脚（-1关闭）
#define CONTROLLERFAN_SECS 60 //How many seconds, after all motors were disabled, the fan should run
                              //多少秒，当所有的发动机都被关闭后，风扇应该运行
#define CONTROLLERFAN_SPEED 255  // == full speed 全速

// When first starting the main fan, run it at full speed for the 当第一次启动主风扇时，以全速运行
// given number of milliseconds.  This gets the fan spinning reliably 给定的毫秒数。这使风扇可靠地旋转
// before setting a PWM value. (Does not work with software PWM for fan on Sanguinololu)
//在设置pwm值之前。（不适用于软件pwm，适用于雪地诺鲁的风扇
//#define FAN_KICKSTART_TIME 100   //定义风扇启动时间

// This defines the minimal speed for the main fan, run in PWM mode  这定义了在pwm模式下运行的主风扇的最小速度
// to enable uncomment and set minimal PWM speed for reliable running (1-255) 为可靠运行设置最小pwm速度（1-255）
// if fan speed is [1 - (FAN_MIN_PWM-1)] it is set to FAN_MIN_PWM 如果风扇转速为[1-(风扇最小功率1)]，则设定为风扇最小功率
//#define FAN_MIN_PWM 50

// @section extruder


// Extruder cooling fans   挤出机冷却风扇
// Configure fan pin outputs to automatically turn on/off when the associated 配置扇引脚输出在关联时自动关闭/关闭
// extruder temperature is above/below EXTRUDER_AUTO_FAN_TEMPERATURE. 挤出机的温度高于/低于挤出机的温度。
// Multiple extruders can be assigned to the same pin in which case  多个挤出器可以被分配到同一个引脚，在这种情况下
// the fan will turn on when any selected extruder is above the threshold. 当任何选定的挤出机高于阈值时，风扇将打开。
#define EXTRUDER_0_AUTO_FAN_PIN -1
#define EXTRUDER_1_AUTO_FAN_PIN -1
#define EXTRUDER_2_AUTO_FAN_PIN -1
#define EXTRUDER_3_AUTO_FAN_PIN -1
#define EXTRUDER_AUTO_FAN_TEMPERATURE 50    //定义挤出机自动风扇温度
#define EXTRUDER_AUTO_FAN_SPEED   255  // == full speed


//===========================================================================
//=======================Mechanical Settings 机械设置========================
//===========================================================================

// @section homing

#define ENDSTOPS_ONLY_FOR_HOMING // If defined the endstops will only be used for homing 如果已定义，则端子将仅用于归航

// @section extras

//#define Z_LATE_ENABLE // Enable Z the last moment. Needed if your Z driver overheats.
//定义z延迟启用   启用z的最后一刻。如果你的z驱动程序过热，你需要。

// A single Z stepper driver is usually used to drive 2 stepper motors.
// Uncomment this define to utilize a separate stepper driver for each Z axis motor.
// Only a few motherboards support this, like RAMPS, which have dual extruder support (the 2nd, often unused, extruder driver is used
// to control the 2nd Z axis stepper motor). The pins are currently only defined for a RAMPS motherboards.
// On a RAMPS (or other 5 driver) motherboard, using this feature will limit you to using 1 extruder.
/*一个z步进电机驱动程序通常用于驱动2个步进电机。
不评论此定义用于为每个z轴电机使用一个单独的步进驱动程序。
只有少数主板支持这一点，比如RAMPS，它有双挤出机的支持（使用的是第二种，通常是未使用的挤出机驱动程序）
以控制第二z轴步进马达）。所述引脚目前仅为RAMPS主板所定义。
在RAMPS（或其他5个驱动程序）主板上，使用此特性将限制您使用1个挤出机。*/
//#define Z_DUAL_STEPPER_DRIVERS  //定义Z轴双步进驱动程序

#if ENABLED(Z_DUAL_STEPPER_DRIVERS)   //如果启用（z双步进驱动程序）

  // Z_DUAL_ENDSTOPS is a feature to enable the use of 2 endstops for both Z steppers - Let's call them Z stepper and Z2 stepper.
  // That way the machine is capable to align the bed during home, since both Z steppers are homed. 
  // There is also an implementation of M666 (software endstops adjustment) to this feature.
  // After Z homing, this adjustment is applied to just one of the steppers in order to align the bed.
  // One just need to home the Z axis and measure the distance difference between both Z axis and apply the math: Z adjust = Z - Z2.
  // If the Z stepper axis is closer to the bed, the measure Z > Z2 (yes, it is.. think about it) and the Z adjust would be positive.
  // Play a little bit with small adjustments (0.5mm) and check the behaviour.
  // The M119 (endstops report) will start reporting the Z2 Endstop as well.
/*z对偶端子是一个特性，可以为Z步进和Z2步进使用2个端子——让我们称它们为Z步进和Z2步进。
这样机器就能在家里把床对齐了，因为两个Z轴步进电机都是归航的。
对这个特性也有m666（软件端点调整）的实现。
在z归航后，这个调整只适用于其中的一个步进，以便对热床进行对齐。
只需将z轴归置，测量两个z轴之间的距离差，并应用数学：z调整=z-z2。
如果z步进轴更靠近床，则测量值z>z2（是的，它是。..考虑一下），z调整将是积极的。
稍作小调整（0.5毫米），并检查其行为。
M119（端点报告）也将开始报告z2端点。
*/
  //#define Z_DUAL_ENDSTOPS   //定义Z轴限位安装端子

  #if ENABLED(Z_DUAL_ENDSTOPS)
    #define Z2_MAX_PIN 36        //Endstop used for Z2 axis. In this case I'm using XMAX in a Rumba Board (pin 36)
                                //Z2轴使用的端子。在这种情况下，我使用Xmax在伦巴板（pin 36）             
    const bool Z2_MAX_ENDSTOP_INVERTING = false;
    #define DISABLE_XMAX_ENDSTOP       //Better to disable the XMAX to avoid conflict. Just rename "XMAX_ENDSTOP" by the endstop you are using for Z2 axis.
                                       //最好禁用Xmax以避免冲突。只要用Z2轴使用的端子重命名“Xmax端子”即可。
  #endif

#endif
// Z_DUAL_STEPPER_DRIVERS    //Z双步进驱动程序

// Same again but for Y Axis.  同样的道理，但也适用于它的轴心。
//#define Y_DUAL_STEPPER_DRIVERS    //定义Y双步进驱动程序

#if ENABLED(Y_DUAL_STEPPER_DRIVERS)
  // Define if the two Y drives need to rotate in opposite directions 定义两个y驱动器是否需要向相反方向旋转
  #define INVERT_Y2_VS_Y_DIR true   //定义逆转Y2与YDir为真
#endif

// Enable this for dual x-carriage printers.  启用此功能，用于双X-trab打印机。
// A dual x-carriage design has the advantage that the inactive extruder can be parked which prevents hot-end 
// ooze contaminating the print. It also reduces the weight of each x-carriage allowing faster printing speeds.
// 双X架设计的优点是，不活动的挤出机可以停放在防止热端渗出物污染打印。它还减轻了每节车厢的重量。允许更快的打印速度。
//#define DUAL_X_CARRIAGE  定义双X轴传动
#if ENABLED(DUAL_X_CARRIAGE)
  // Configuration for second X-carriage 配置为第二个X传动
  // Note: the first x-carriage is defined as the x-carriage which homes to the minimum endstop;
  //注：第一个X轴传动的定义是把X轴传动放置在最小的终点；
  // the second x-carriage always homes to the maximum endstop.第二个X轴电机总能到达最大的终点。
  #define X2_MIN_POS 80     // set minimum to ensure second x-carriage doesn't hit the parked first X-carriage
                            //设定最小值，以确保第二个X轴电机没有撞上停在那里的第一个X轴电机
  #define X2_MAX_POS 353    // set maximum to the distance between toolheads when both heads are homed
                            //当两个头都被定位时，最大设置为工具头之间的距离
  #define X2_HOME_DIR 1     // the second X-carriage always homes to the maximum endstop position
                            //第二个X轴电机总是把喷头停在最大的终点位置
  #define X2_HOME_POS X2_MAX_POS // default home position is the maximum carriage position 默认的主位置是最大的移动位置
      // However: In this mode the EXTRUDER_OFFSET_X value for the second extruder provides a software
      // override for X2_HOME_POS. This also allow recalibration of the distance between the two endstops
      // without modifying the firmware (through the "M218 T1 X???" command).
      // Remember: you should set the second extruder x-offset to 0 in your slicer.
      //然而：在这种模式下，第二个挤出机的挤出机_offset_x值提供了一个软件
      //重置为x2_home_pos。这也允许重新校准两个端点之间的距离。
      //不修改固件（通过"m218 t1 x???"）命令）。
      //请记住：您应该将您的Sclicer中的第二个挤出机x偏移量设置为0。

  // Pins for second x-carriage stepper driver (defined here to avoid further complicating pins.h)
  //用于第二个X轴电机驱动的引脚（此处定义是为了避免使引脚进一步复杂化）
  #define X2_ENABLE_PIN 29
  #define X2_STEP_PIN 25
  #define X2_DIR_PIN 23

  // There are a few selectable movement modes for dual x-carriages using M605 S<mode>
  //使用m605的双X车厢有几种可选择的移动模式
  //    Mode 0: Full control. The slicer has full control over both x-carriages and can achieve optimal travel results
  //                           as long as it supports dual x-carriages. (M605 S0)
  //模式0：完全控制。Sclicer完全控制了两个X车厢，并能达到最佳的旅行效果只要它支持双X车厢。(m605 s0)
  //    Mode 1: Auto-park mode. The firmware will automatically park and unpark the x-carriages on tool changes so
  //                           that additional slicer support is not required. (M605 S1)
  //模式1：自动停车模式。固件将自动停车和卸载的工具变化的X车厢，所以不需要额外的Sclicer支持。(m605 1)
  //    Mode 2: Duplication mode. The firmware will transparently make the second x-carriage and extruder copy all
  //                           actions of the first x-carriage. This allows the printer to print 2 arbitrary items at
  //                           once. (2nd extruder x offset and temp offset are set using: M605 S2 [Xnnn] [Rmmm])
  //模式2：复制模式。该固件将透明地使第二个x架和挤出机全部复制第一辆x型马车的动作。
  //这允许打印机打印两个任意项目一次。（第2挤出机X偏置和温度偏置采用：m605 s2[xnnn][rmm]）

  // This is the default power-up mode which can be later using M605.这是默认的启动模式，可以在以后使用m605。
  #define DEFAULT_DUAL_X_CARRIAGE_MODE 0   //定义默认的对偶x运输模式

  // Default settings in "Auto-park Mode"  自动停车模式”中的默认设置
  #define TOOLCHANGE_PARK_ZLIFT   0.2      // the distance to raise Z axis when parking an extruder 挤出机停车时提高z轴的距离
  #define TOOLCHANGE_UNPARK_ZLIFT 1        // the distance to raise Z axis when unparking an extruder 卸载挤出机时提高z轴的距离

  // Default x offset in duplication mode (typically set to half print bed width)
  //重复模式中默认的X偏移量(通常设置为打印床宽度的一半)
  #define DEFAULT_DUPLICATION_X_OFFSET 100

#endif //DUAL_X_CARRIAGE            双X轴传动

// @section homing

//homing hits the endstop, then retracts by this distance, before it tries to slowly bump again:
//回零击中终点，然后按这个距离向后缩，在它试图再次缓慢撞击之前：
#define X_HOME_BUMP_MM 5
#define Y_HOME_BUMP_MM 5
#define Z_HOME_BUMP_MM 2
#define HOMING_BUMP_DIVISOR {2, 2, 4}  // Re-Bump Speed Divisor (Divides the Homing Feedrate)重凸速度除数（将自导进速率分开）
//#define QUICK_HOME  //if this is defined, if both x and y are to be homed, a diagonal move will be performed initially.
//什么叫快点回家  如果定义了这一点，如果X和Y都归为homed，则最初将执行对角线移动。

// When G28 is called, this option will make Y home before X   当G28被调用时，此选项将使y在x之前返回。
//#define HOME_Y_BEFORE_X    //在x轴之前定义Y轴回零

// @section machine

#define AXIS_RELATIVE_MODES {false, false, false, false}

// @section machine

//By default pololu step drivers require an active high signal. However, some high power drivers require an active low signal as step.
//默认情况下，Pololu步进驱动程序需要一个活动的高信号。然而，一些大功率驱动程序需要一个主动低信号作为步骤。
#define INVERT_X_STEP_PIN false
#define INVERT_Y_STEP_PIN false
#define INVERT_Z_STEP_PIN false
#define INVERT_E_STEP_PIN false

// Default stepper release if idle. Set to 0 to deactivate.如果空闲，默认步进程序释放。设置为0以停用。
#define DEFAULT_STEPPER_DEACTIVE_TIME 60    //定义默认步进程序停用时间

#define DEFAULT_MINIMUMFEEDRATE       0.0     // minimum feedrate 定义默认的最小输入率
#define DEFAULT_MINTRAVELFEEDRATE     0.0     //定义默认的最小行程速率

// @section lcd

#if ENABLED(ULTIPANEL)   //启用(多面板)
  #define MANUAL_FEEDRATE {50*60, 50*60, 4*60, 60} // Feedrates for manual moves along X, Y, Z, E from panel从面板沿x,y,z,e手动移动的进料
  #define ULTIPANEL_FEEDMULTIPLY  // Comment to disable setting feedrate multiplier via encoder通过编码器禁用设置输入率乘数的注释
#endif

// @section extras

// minimum time in microseconds that a movement needs to take if the buffer is emptied.
//如果缓冲区被清空，移动所需的最短时间（以微秒为单位）
#define DEFAULT_MINSEGMENTTIME        20000  //定义默认的片段时间

// If defined the movements slow down when the look ahead buffer is only half full
//如果定义了，当向前看的缓冲区只有一半满的时候，移动速度会减慢
#define SLOWDOWN   //减速

// Frequency limit  频率限制
// See nophead's blog for more info  详见诺飞的博客以获取更多信息
// Not working O     不起作用
//#define XY_FREQUENCY_LIMIT  15   //定义xy频率限制

// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
/*最小计划连接速度。设定计划人员在最后所计划的预设最小速度
的缓冲区和所有停止。这不应该比零大得多，只应该改变
如果在用户的机器上以非常慢的速度运行时观察到了不需要的行为。*/
#define MINIMUM_PLANNER_SPEED 0.05// (mm/sec)  定义最小计划速度

// Microstep setting (Only functional when stepper driver microstep pins are connected to MCU.
//微步设置（仅当步进驱动程序微步引脚连接到mcu时才起作用。
#define MICROSTEP_MODES {16,16,16,16,16} // [1,2,4,8,16]

// Motor Current setting (Only functional when motor driver current ref pins are connected to a digital trimpot on supported boards)
//电机电流设置（仅当电机驱动电流参考引脚连接到支持板上的数字修剪器时才起作用
#define DIGIPOT_MOTOR_CURRENT {135,135,135,135,135} // Values 0-255 (RAMBO 135 = ~0.75A, 185 = ~1A)
//定义数位马达电流{135,135,135,135}/值0-255(rambo 135=~0.75a,185=~1a)

// uncomment to enable an I2C based DIGIPOT like on the Azteeg X3 Pro 不评论，以启用基于i2c的数字盒，如在azteeg x3 pro上
//#define DIGIPOT_I2C
// Number of channels available for I2C digipot, For Azteeg X3 Pro we have 8
//可供i2c数码锅使用的频道数量，如果是Zazteeg x3 Pro，我们有8个
#define DIGIPOT_I2C_NUM_CHANNELS 8
// actual motor currents in Amps, need as many here as DIGIPOT_I2C_NUM_CHANNELS 实际的电流是安培的，这里需要的是digipot i2cnum通道
#define DIGIPOT_I2C_MOTOR_CURRENTS {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}

//===========================================================================
//========================Additional Features 附加特性=======================
//===========================================================================

#define ENCODER_RATE_MULTIPLIER         // If defined, certain menu edit operations automatically multiply the steps when the encoder is moved quickly
                                        //如果定义了，当快速移动编码器时，某些菜单编辑操作会自动将步骤相乘
#define ENCODER_10X_STEPS_PER_SEC 75    // If the encoder steps per sec exceeds this value, multiply steps moved x10 to quickly advance the value
                                        //如果每个SEC的编码器步骤超过此值，则乘以移动的步骤X10以快速推进该值
#define ENCODER_100X_STEPS_PER_SEC 160  // If the encoder steps per sec exceeds this value, multiply steps moved x100 to really quickly advance the value
                                        //如果每个SEC的编码器步骤超过此值，则乘以移动的步骤X100可以真正快速地推进该值

//#define CHDK 4        //Pin for triggering CHDK to take a picture see how to use it here http://captain-slow.dk/2014/03/09/3d-printing-timelapses/
//佳能相机上运行的应用软件    用于触发chdk拍摄照片的别针，请在此查看如何使用
#define CHDK_DELAY 50 //How long in ms the pin should stay HIGH before going LOW again  这个别针要多长时间才能再变低

// @section lcd

#if ENABLED(SDSUPPORT)  //如果已启用（sd支持）

/* Some RAMPS and other boards don't detect when an SD card is inserted. You can work
   around this by connecting a push button or single throw switch to the pin defined
   as SD_DETECT_PIN in your board's pins definitions.
   This setting should be disabled unless you are using a push button, pulling the pin to ground.
   Note: This is always disabled for ULTIPANEL (except ELB_FULL_GRAPHIC_CONTROLLER).
   有些斜面和其他板无法检测sd卡是什么时候插入的。你可以工作
   通过将一个按钮或单掷开关与定义的引脚连接在一起
   在你的板子的别针定义中是sd_restor_pin。
   这个设置应该被禁用，除非您正在使用一个按按钮，将引脚拉向地面。
   注意：对于多面板（除了elb全图形控制器），这总是被禁用的。*/
  #define SD_DETECT_INVERTED

  #define SD_FINISHED_STEPPERRELEASE true  //if sd support and the file is finished: disable steppers? 如果SD支持并且文件已经完成：禁用步进程序？
  #define SD_FINISHED_RELEASECOMMAND "M84 X Y Z E" // You might want to keep the z enabled so your bed stays in place.
                                                   //你可能想保持Z启用，所以你的床保持在适当的位置。

  #define SDCARD_RATHERRECENTFIRST  //reverse file order of sd card menu display. Its sorted practically after the file system block order.
  //反文件顺序的SD卡菜单显示.它实际上是在文件系统的阻塞顺序之后进行排序的。
  // if a file is deleted, it frees a block. hence, the order is not purely chronological. To still have auto0.g accessible, there is again the option to do that.
  //如果文件被删除，它会释放一个块。因此，顺序并不纯粹是按时间顺序排列。如果您仍然可以使用Auto0.g，也可以选择这样做。
  // using:
  //#define MENU_ADDAUTOSTART  //定义菜单_加载启动

  // Show a progress bar on HD44780 LCDs for SD printing  显示用于sd印刷的hd44780lcd的进度条
  //#define LCD_PROGRESS_BAR         //定义lcd进度栏

  #if ENABLED(LCD_PROGRESS_BAR)
    // Amount of time (ms) to show the bar  显示标杆的时间（MS）
    #define PROGRESS_BAR_BAR_TIME 2000
    // Amount of time (ms) to show the status message  显示状态消息的时间(MS)
    #define PROGRESS_BAR_MSG_TIME 3000
    // Amount of time (ms) to retain the status message (0=forever)  状态消息的保留时间（0=永久）
    #define PROGRESS_MSG_EXPIRE   0
    // Enable this to show messages for MSG_TIME then hide them  启用此功能可显示msg-time的消息，然后将其隐藏
    //#define PROGRESS_MSG_ONCE
  #endif

  // This allows hosts to request long names for files and folders with M33  这允许主机为带有M33的文件和文件夹请求长名称
  //#define LONG_FILENAME_HOST_SUPPORT      //定义长文件名主机支持

  // This option allows you to abort SD printing when any endstop is triggered.  此选项允许您在触发任何终端时中止sd打印。
  // This feature must be enabled with "M540 S1" or from the LCD menu.    此功能必须使用“M540 S1”或从lcd菜单中启用。
  // To have any effect, endstops must be enabled during SD printing.     要产生任何效果，必须在sd打印过程中启用端子。
  // With ENDSTOPS_ONLY_FOR_HOMING you must send "M120" to enable endstops.  您必须发送“M120”以启用端点。
  //#define ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED        //定义已启用的终止点击功能时的中止

#endif // SDSUPPORT

// for dogm lcd displays you can choose some additional fonts: 对于dogm LCD显示器，您可以选择一些额外的字体：
#if ENABLED(DOGLCD)
  // save 3120 bytes of PROGMEM by commenting out #define USE_BIG_EDIT_FONT  通过注释定义使用大编辑字体来保存3120字节的程序
  // we don't have a big font for Cyrillic, Kana 我们没有一个大字体的西里尔，卡纳
  //#define USE_BIG_EDIT_FONT    //定义使用大编辑字体
 
  // If you have spare 2300Byte of progmem and want to use a   如果您有备用的2300byte的孕酮，并且想要使用
  // smaller font on the Info-screen uncomment the next line.  信息屏幕上较小的字体取消对下一行的注释。
  //#define USE_SMALL_INFOFONT             //定义使用小信息字体
#endif // DOGLCD

// @section more

// The hardware watchdog should reset the microcontroller disabling all outputs, in case the firmware gets stuck and doesn't do temperature regulation.
//硬件监测器应该重新设置使所有输出失效的微控制器，以防固件卡住而不进行温度调节
//#define USE_WATCHDOG    //定义使用加密狗
  
#if ENABLED(USE_WATCHDOG)
// If you have a watchdog reboot in an ArduinoMega2560 then the device will hang forever, as a watchdog reset will leave the watchdog on.
// The "WATCHDOG_RESET_MANUAL" goes around this by not using the hardware reset.
//  However, THIS FEATURE IS UNSAFE!, as it will only work if interrupts are disabled. And the code could hang in an interrupt routine with interrupts disabled.
/*如果你有一只看门狗在Arduinomega2560重新启动，那么这个装置将永远悬挂，就像看门狗重置时会离开看门狗一样。
“监视原则重新部署手册”通过不使用硬件重置来绕过这个问题。
然而，这个特性是不安全的！，因为只有在中断被禁用时，它才会工作。并且该代码可以挂在中断程序中，中断被禁用。*/
//#define WATCHDOG_RESET_MANUAL         //定义加密狗重置手册
#endif

// @section lcd

// Babystepping enables the user to control the axis in tiny amounts, independently from the normal printing process
//小步电机进使用户能够控制轴的少量，独立于正常的印刷过程
// it can e.g. be used to change z-positions in the print startup phase in real-time 例如，它可以在打印启动阶段实时更改z-位置
// does not respect endstops!不遵守暂停
//#define BABYSTEPPING
#if ENABLED(BABYSTEPPING)
  #define BABYSTEP_XY  //not only z, but also XY in the menu. more clutter, more functions不仅是Z，还有菜单上的xy。更多的混乱，更多的功能
                       //not implemented for CoreXY and deltabots!没有实现的Core XY结构和德塔巴！
  #define BABYSTEP_INVERT_Z false  //true for inverse movements in Z  对z中的逆运动是成立的
  #define BABYSTEP_MULTIPLICATOR 1 //faster movements   更快的动作
#endif

// @section extruder

// extruder advance constant (s2/mm3)  挤出机前进常数(s2/mm3)
//
// advance (steps) = STEPS_PER_CUBIC_MM_E * EXTRUDER_ADVANCE_K * cubic mm per second ^ 2
//前进（步数）=每立方毫米e*挤出机前进k*立方毫米每秒^2
// Hooke's law says:    force = k * distance  胡克定律说：力=K*距离
// Bernoulli's principle says:  v ^ 2 / 2 + g . h + pressure / density = constant  伯努利的原理是：V^2/2+g。H+压力/密度=常数
// so: v ^ 2 is proportional to number of steps we advance the extruder 因此：V^2与我们推进挤出机的步骤数成正比
//#define ADVANCE     定义预付款

#if ENABLED(ADVANCE)                 //如果启用（预付款）
  #define EXTRUDER_ADVANCE_K .0
  #define D_FILAMENT 2.85
  #define STEPS_MM_E 836
#endif

// @section extras

// Arc interpretation settings:   弧线判读设定：
#define MM_PER_ARC_SEGMENT 1         //定义每弧线段1的毫米
#define N_ARC_CORRECTION 25          //定义n弧校正25

const unsigned int dropsegments=5; //everything with less than this number of steps will be ignored as move and joined with the next movement
//所有少于这个步骤的动作都会被忽略，并加入到下一个动作中

// @section temperature
// Control heater 0 and heater 1 in parallel. //并行控制加热器0和加热器1。
//#define HEATERS_PARALLEL

//===========================================================================
//============================= Buffers缓冲器 ===============================
//===========================================================================

// @section hidden

// The number of linear motions that can be in the plan at any give time.  在任何给定的时间内可以在计划中的线性运动的数目。
// THE BLOCK_BUFFER_SIZE NEEDS TO BE A POWER OF 2, i.g. 8,16,32 because shifts and ors are used to do the ring-buffering.
//块缓冲大小需要2的功率，即8,16,32，因为轮班和ORS是用来做环形缓冲的。
#if ENABLED(SDSUPPORT)
  #define BLOCK_BUFFER_SIZE 16   // SD,LCD,Buttons take more memory, block buffer needs to be smaller 
   //sd，lcd，按钮占用更多内存，块缓冲区需要更小
#else
  #define BLOCK_BUFFER_SIZE 16 // maximize block buffer  最大程度地使用块缓冲区
#endif

// @section more

//The ASCII buffer for receiving from the serial:   用于从串行接收的ascii缓冲区：
#define MAX_CMD_SIZE 96   //定义最大cmd大小
#define BUFSIZE 4

// Bad Serial-connections can miss a received command by sending an 'ok'
// Therefore some clients abort after 30 seconds in a timeout.
// Some other clients start sending commands while receiving a 'wait'.
// This "wait" is only sent when the buffer is empty. 1 second is a good value here.
/*错误的序列化连接通过发送“确定”会错过接收命令
因此有些客户端在超时30秒后中止。
其他一些客户端在接收“等待”时开始发送命令。
此“等待”仅在缓冲区为空时发送。1秒钟是一个很好的价值在这里。*/
//#define NO_TIMEOUTS 1000 // Milliseconds  定义无超时1000/毫秒

// Some clients will have this feature soon. This could make the NO_TIMEOUTS unnecessary.
//有些客户很快就会拥有这个功能。这可能使无超时成为不必要的。
//#define ADVANCED_OK

// @section fwretract

// Firmware based and LCD controlled retract  基于固件和lcd控制的缩回
// M207 and M208 can be used to define parameters for the retraction.  m207和m208可以用来定义缩回的参数。
// The retraction can be called by the slicer using G10 and G11   这个缩回可以用G10和G11的方法来称呼。
// until then, intended retractions can be detected by moves that only extrude and the direction.
//在此之前，只有挤压和方向的移动才能检测到预期的收缩。
// the moves are than replaced by the firmware controlled ones. 这些动作被固件控制的动作所代替。

//#define FWRETRACT  //ONLY PARTIALLY TESTED  只进行了部分测试
#if ENABLED(FWRETRACT)
  #define MIN_RETRACT 0.1                //minimum extruded mm to accept a automatic gcode retraction attempt接受自动gcode撤回尝试的最小挤出毫米
  #define RETRACT_LENGTH 3               //default retract length (positive mm) 默认缩回长度(正毫米)
  #define RETRACT_LENGTH_SWAP 13         //default swap retract length (positive mm), for extruder change 挤出机更改的默认交换缩回长度(正毫米)
  #define RETRACT_FEEDRATE 45            //default feedrate for retracting (mm/s)  (m/s)回收的默认输入率
  #define RETRACT_ZLIFT 0                //default retract Z-lift  默认撤回z-升降机
  #define RETRACT_RECOVER_LENGTH 0       //default additional recover length (mm, added to retract length when recovering) 默认附加恢复长度（毫米，在恢复时添加到缩回长度）
  #define RETRACT_RECOVER_LENGTH_SWAP 0  //default additional swap recover length (mm, added to retract length when recovering from extruder change)
                                         //默认的附加交换恢复长度（毫米，从挤出机更改中恢复时添加到缩回长度）
  #define RETRACT_RECOVER_FEEDRATE 8     //default feedrate for recovering from retraction (mm/s) 从收回中恢复的默认输入率(毫米/秒)
#endif

// Add support for experimental filament exchange support M600; requires display 增加对实验灯丝的支持交换支持M600;需要显示
#if ENABLED(ULTIPANEL)
  //#define FILAMENTCHANGEENABLE 定义已启用的文件更改
  #if ENABLED(FILAMENTCHANGEENABLE)
    #define FILAMENTCHANGE_XPOS 3
    #define FILAMENTCHANGE_YPOS 3
    #define FILAMENTCHANGE_ZADD 10
    #define FILAMENTCHANGE_FIRSTRETRACT -2
    #define FILAMENTCHANGE_FINALRETRACT -100
    #define AUTO_FILAMENT_CHANGE                //This extrude filament until you press the button on LCD 这个挤出的灯丝，直到你按下lcd上的按钮
    #define AUTO_FILAMENT_CHANGE_LENGTH 0.04    //Extrusion length on automatic extrusion loop  自动挤压回路上的挤压长度
    #define AUTO_FILAMENT_CHANGE_FEEDRATE 300   //Extrusion feedrate (mm/min) on automatic extrusion loop  自动挤压回路中的挤压进给率(mm/min)
  #endif
#endif

/******************************************************************************\
 * enable this section if you have TMC26X motor drivers.   如果您有tmc 26x电机驱动程序，请启用此部分。
 * you need to import the TMC26XStepper library into the arduino IDE for this  为此，您需要将tmc 26xsteper库导入到Arduino代码集中
 ******************************************************************************/

// @section tmc

//#define HAVE_TMCDRIVER  //定义有tmc驱动程式
#if ENABLED(HAVE_TMCDRIVER)

//#define X_IS_TMC
  #define X_MAX_CURRENT 1000  //in mA                  定义x最大电流
  #define X_SENSE_RESISTOR 91 //in mOhms               定义x感电阻
  #define X_MICROSTEPS 16     //number of microsteps   微步数
  
//#define X2_IS_TMC
  #define X2_MAX_CURRENT 1000  //in mA
  #define X2_SENSE_RESISTOR 91 //in mOhms
  #define X2_MICROSTEPS 16     //number of microsteps
  
//#define Y_IS_TMC
  #define Y_MAX_CURRENT 1000  //in mA
  #define Y_SENSE_RESISTOR 91 //in mOhms
  #define Y_MICROSTEPS 16     //number of microsteps
  
//#define Y2_IS_TMC
  #define Y2_MAX_CURRENT 1000  //in mA
  #define Y2_SENSE_RESISTOR 91 //in mOhms
  #define Y2_MICROSTEPS 16     //number of microsteps 
  
//#define Z_IS_TMC
  #define Z_MAX_CURRENT 1000  //in mA
  #define Z_SENSE_RESISTOR 91 //in mOhms
  #define Z_MICROSTEPS 16     //number of microsteps
  
//#define Z2_IS_TMC
  #define Z2_MAX_CURRENT 1000  //in mA
  #define Z2_SENSE_RESISTOR 91 //in mOhms
  #define Z2_MICROSTEPS 16     //number of microsteps
  
//#define E0_IS_TMC
  #define E0_MAX_CURRENT 1000  //in mA
  #define E0_SENSE_RESISTOR 91 //in mOhms
  #define E0_MICROSTEPS 16     //number of microsteps
  
//#define E1_IS_TMC
  #define E1_MAX_CURRENT 1000  //in mA
  #define E1_SENSE_RESISTOR 91 //in mOhms
  #define E1_MICROSTEPS 16     //number of microsteps 
  
//#define E2_IS_TMC
  #define E2_MAX_CURRENT 1000  //in mA
  #define E2_SENSE_RESISTOR 91 //in mOhms
  #define E2_MICROSTEPS 16     //number of microsteps 
  
//#define E3_IS_TMC
  #define E3_MAX_CURRENT 1000  //in mA
  #define E3_SENSE_RESISTOR 91 //in mOhms
  #define E3_MICROSTEPS 16     //number of microsteps   

#endif

/******************************************************************************\
 * enable this section if you have L6470  motor drivers.   如果您有l6470电机驱动程序，请启用此部分。
 * you need to import the L6470 library into the arduino IDE for this   为此您需要将l6470库导入到Arduino代码集中
 ******************************************************************************/

// @section l6470

//#define HAVE_L6470DRIVER
#if ENABLED(HAVE_L6470DRIVER)   //如果已启用（已有l6470驱动程序）

//#define X_IS_L6470
  #define X_MICROSTEPS 16     //number of microsteps     微步数
  #define X_K_VAL 50          // 0 - 255, Higher values, are higher power. Be carefull not to go too high     0-255,数值越高,功率越高.小心不要走得太高
  #define X_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
                              //在我看来，这是一种巨大的压力。如果电流超过这个值，驱动程序就会关闭
  #define X_STALLCURRENT 1500 //current in mA where the driver will detect a stall   MA的电流，在那里司机会侦测到一个失速
  
//#define X2_IS_L6470
  #define X2_MICROSTEPS 16     //number of microsteps
  #define X2_K_VAL 50          // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
  #define X2_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define X2_STALLCURRENT 1500 //current in mA where the driver will detect a stall
  
//#define Y_IS_L6470
  #define Y_MICROSTEPS 16     //number of microsteps
  #define Y_K_VAL 50          // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
  #define Y_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define Y_STALLCURRENT 1500 //current in mA where the driver will detect a stall
  
//#define Y2_IS_L6470
  #define Y2_MICROSTEPS 16     //number of microsteps 
  #define Y2_K_VAL 50          // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
  #define Y2_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define Y2_STALLCURRENT 1500 //current in mA where the driver will detect a stall 
  
//#define Z_IS_L6470
  #define Z_MICROSTEPS 16     //number of microsteps
  #define Z_K_VAL 50          // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
  #define Z_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define Z_STALLCURRENT 1500 //current in mA where the driver will detect a stall
  
//#define Z2_IS_L6470
  #define Z2_MICROSTEPS 16     //number of microsteps
  #define Z2_K_VAL 50          // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
  #define Z2_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define Z2_STALLCURRENT 1500 //current in mA where the driver will detect a stall
  
//#define E0_IS_L6470
  #define E0_MICROSTEPS 16     //number of microsteps
  #define E0_K_VAL 50          // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
  #define E0_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define E0_STALLCURRENT 1500 //current in mA where the driver will detect a stall
  
//#define E1_IS_L6470
  #define E1_MICROSTEPS 16     //number of microsteps 
  #define E1_MICROSTEPS 16     //number of microsteps
  #define E1_K_VAL 50          // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
  #define E1_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define E1_STALLCURRENT 1500 //current in mA where the driver will detect a stall
  
//#define E2_IS_L6470
  #define E2_MICROSTEPS 16     //number of microsteps 
  #define E2_MICROSTEPS 16     //number of microsteps
  #define E2_K_VAL 50          // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
  #define E2_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define E2_STALLCURRENT 1500 //current in mA where the driver will detect a stall
  
//#define E3_IS_L6470
  #define E3_MICROSTEPS 16     //number of microsteps   
  #define E3_MICROSTEPS 16     //number of microsteps
  #define E3_K_VAL 50          // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
  #define E3_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define E3_STALLCURRENT 1500 //current in mA where the driver will detect a stall
  
#endif

#include "Conditionals.h"
#include "SanityCheck.h"

#endif //CONFIGURATION_ADV_H
