/**
 * Marlin Firmware
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * About Marlin
 *
 * This firmware is a mashup between Sprinter and grbl.
 *  - https://github.com/kliment/Sprinter
 *  - https://github.com/simen/grbl/tree
 *
 * It has preliminary support for Matthew Roberts advance algorithm
 *  - http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */

#include "Marlin.h"

#if ENABLED(AUTO_BED_LEVELING_FEATURE) //启用（自动床调平功能）
  #include "vector_3.h"
  #if ENABLED(AUTO_BED_LEVELING_GRID)//如果启用（自动床水准测量网格）
    #include "qr_solve.h"
  #endif
#endif // AUTO_BED_LEVELING_FEATURE

#if ENABLED(MESH_BED_LEVELING) //如果启用（网床整平）
  #include "mesh_bed_leveling.h"
#endif

#include "ultralcd.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "cardreader.h"
#include "configuration_store.h"
#include "language.h"
#include "pins_arduino.h"
#include "math.h"
#include "buzzer.h"

#if ENABLED(USE_WATCHDOG) //如果启用（使用看门狗）
  #include "watchdog.h"
#endif

#if ENABLED(BLINKM)   //如果启用（BLink）
  #include "blinkm.h"
  #include "Wire.h"
#endif

#if HAS_SERVOS   //有伺服系统
  #include "servo.h"
#endif

#if HAS_DIGIPOTSS
  #include <SPI.h>
#endif

/**
 * Look here for descriptions of G-codes:这里看G代码的描述：
 *  - http://linuxcnc.org/handbook/gcode/g-code.html
 *  - http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes
 *
 * Help us document these G-codes online:帮助我们在线记录这些G代码：
 *  - http://marlinfirmware.org/index.php/G-Code
 *  - http://reprap.org/wiki/G-code
 *
 * -----------------
 * Implemented Codes  实现代码
 * -----------------
 *
 * "G" Codes
 *
 * G0  -> G1
 * G1  - Coordinated Movement X Y Z E 协调运动
 * G2  - CW ARC  连续电弧
 * G3  - CCW ARC  圆弧电弧
 * G4  - Dwell S<seconds> or P<milliseconds> CW W弧秒s＞秒＞P＜毫秒＞
 * G10 - retract filament according to settings of M207根据M207的设置收回灯丝
 * G11 - retract recover filament according to settings of M208 根据M208的设置缩回回收灯丝
 * G28 - Home one or more axes 回零
 * G29 - Detailed Z probe, probes the bed at 3 or more points.  Will fail if you haven't homed yet.
 *        详细的Z探头，在3点或更多点探测床。如果你还没回家，就会失败。
 * G30 - Single Z probe, probes bed at current XY location.单Z探头，探头在当前XY位置。
 * G31 - Dock sled (Z_PROBE_SLED only)
 * G32 - Undock sled (Z_PROBE_SLED only)
 * G90 - Use Absolute Coordinates使用绝对坐标
 * G91 - Use Relative Coordinates使用相对坐标
 * G92 - Set current position to coordinates given将当前位置设置为坐标
 *
 * "M" Codes
 *
 * M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
         无条件停止等待用户按下LCD上的按钮（仅当启用ULYLCD LCD）
 * M1   - Same as M0  与M0相同
 * M17  - Enable/Power all stepper motors 启用/动力所有步进电机
 * M18  - Disable all stepper motors; same as M84 禁用所有步进电机；与M84M相同
 * M20  - List SD card  列表SD卡
 * M21  - Init SD card  init SD卡
 * M22  - Release SD card 释放SD卡
 * M23  - Select SD file (M23 filename.g) 选择SD文件（m23文件名.g）
 * M24  - Start/resume SD print  启动/恢复SD打印
 * M25  - Pause SD print 暂停SD打印
 * M26  - Set SD position in bytes (M26 S12345)以字节为单位设置SD位置（M26 S12345）
 * M27  - Report SD print status 报告SD打印状态
 * M28  - Start SD write (M28 filename.g)启动SD写入（M28文件名G）
 * M29  - Stop SD write  停止SD写入
 * M30  - Delete file from SD (M30 filename.g) 从SD中删除文件（M30文件名G）
 * M31  - Output time since last M109 or SD card start to serial 自上次M109或SD卡开始到串行的输出时间
 * M32  - Select file and start SD print (Can be used _while_ printing from SD card files):
 *        syntax "M32 /path/filename#", or "M32 S<startpos bytes> !filename#"
 *        Call gcode file : "M32 P !filename#" and return to caller file after finishing (similar to #include).
 *        The '#' is necessary when calling from within sd files, as it stops buffer prereading
 *        选择文件并启动SD打印（可用于SD卡文件的打印）：语法“M32 /路径/文件名”，或“M32 s< StestPoS字节”！文件名<
          调用GCODE文件：“M32 P！文件名>”并在完成后返回到调用方文件（类似于“包含”）。在从SD文件中调用时，必须使用'y'，因为它停止缓冲预处理。
 * M33  - Get the longname version of a path  获取路径的长名称版本
 * M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
 *       通过GCODE改变引脚状态使用M42 PX SY将引脚X设置为Y值，省略PX时，将使用车载LED
 * M48  - Measure Z_Probe repeatability. M48 [P # of points] [X position] [Y position] [V_erboseness #] [E_ngage Probe] [L # of legs of travel]
 *        测量Z-探头重复性。M48 [点p的] [x位置] [y位置] [VyeBurthisty] [Eng- NeangGe] [Lo[n]的旅行腿]
 * M80  - Turn on Power Supply 接通电源
 * M81  - Turn off Power Supply 关掉电源
 * M82  - Set E codes absolute (default) 设置E代码绝对值（默认值）
 * M83  - Set E codes relative while in Absolute Coordinates (G90) mode 在绝对坐标（G90）模式下设置相对E码
 * M84  - Disable steppers until next move,禁用步进器直到下一步移动，
 *        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
 *        或使用S<秒>指定不活动超时，之后将禁用步进器。S0禁用超时。
 * M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 *        设置不活动停机计时器，参数S＜秒＞。禁用SET零（默认）
 * M92  - Set axis_steps_per_unit - same syntax as G92 设置AxISStupSpPiSUng-与G92相同的语法
 * M104 - Set extruder target temp 设定挤压机目标温度
 * M105 - Read current temp 读取当前温度
 * M106 - Fan on 扇开
 * M107 - Fan off 扇关
 * M109 - Sxxx Wait for extruder current temp to reach target temp. Waits only when heating
 *        Rxxx Wait for extruder current temp to reach target temp. Waits when heating and cooling
 *        IF AUTOTEMP is enabled, S<mintemp> B<maxtemp> F<factor>. Exit autotemp by any M109 without F
 *        Sxxx等待挤出机当前温度达到目标温度。仅在加热时等待 Rxxx等待挤出机当前温度达到目标温度。等待加热和冷却
 *        如果启用了AutoTimp，则S＜MimTyp> B< Max MTEP > F<因子>。没有MF的任何M109退出AutoTyp
 * M110 - Set the current line number 设置当前行数
 * M111 - Set debug flags with S<mask>. See flag bits defined in Marlin.h. 设置调试标志，使用S <掩码>。参见马林H中定义的标志位。
 * M112 - Emergency stop 紧急停车
 * M114 - Output current position to serial port 输出端口到串行端口的位置
 * M115 - Capabilities string 能力字符串
 * M117 - Display a message on the controller screen 在控制器屏幕上显示消息
 * M119 - Output Endstop status to serial port 输出端停止状态到串行端口
 * M120 - Enable endstop detection 启用端点检测
 * M121 - Disable endstop detection 禁用端点检测
 * M126 - Solenoid Air Valve Open (BariCUDA support by jmil)  电磁气阀开启（由JMIL支持的Brutuda）
 * M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil) 电磁阀关闭（BrimuDA排气至大气压力JMIL）
 * M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil) ETOP开（BariCUDA EtoP＝JMIL电空压机）
 * M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil) EtoP关闭（BariCUDA EtoP＝JMIL电至气压传感器）
 * M140 - Set bed target temp 设定床目标温度
 * M145 - Set the heatup state H<hotend> B<bed> F<fan speed> for S<material> (0=PLA, 1=ABS)设定热态H Hooad＞B＞床＞F＞风扇速度＞S＞材料＞（0＝PLA，1＝ABS）
 * M150 - Set BlinkM Color Output R: Red<0-255> U(!): Green<0-255> B: Blue<0-255> over i2c, G for green does not work.
 *        设置BLink的颜色输出R：Red <0～255> U（！）Green <0～255> B：蓝色<0～255>超过I2C，G为绿色不起作用。
 * M190 - Sxxx Wait for bed current temp to reach target temp. Waits only when heating  Sxxx等待床电流温度达到目标温度。仅在加热时等待
 *        Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling   Rxxx等待床电流温度达到目标温度。等待加热和冷却
 * M200 - set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).:D<millimeters>-
 *        设定灯丝直径并将E轴单位设置为立方毫米（使用S0设置为毫米）：D＜毫米＞
 * M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000) 为打印移动设置单位/s^ 2的最大加速度（M201 x1000 Y1000）
 * M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!! 设置最大加速度单位/s^ 2的旅行移动（M202 x1000 Y1000）未在马林使用！
 * M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec 设置最大速度，您的机器可以维持（M203 X200 Y200 Z300 E10000）毫米/秒
 * M204 - Set default acceleration: P for Printing moves, R for Retract only (no X, Y, Z) moves and T for Travel (non printing) moves (ex. M204 P800 T3000 R9000) in mm/sec^2
 *        设置默认加速：P为打印移动，R为仅缩回（NO x，y，z）移动，t为行进（非打印）移动（例如，M204 P800 T3000 R9000）为毫米/秒^ 2。
 * M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
 *        高级设置：最小行进速度S＝打印T＝仅行进，B＝最小段时间X＝最大XY重击，Z＝最大Z重击，E＝最大E重击
 * M206 - Set additional homing offset 设置附加寻的偏移
 * M207 - Set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop], stays in mm regardless of M200 setting 
 *        设置收缩长度S [正mm ] F[进给速度Mm/min ] z [附加ZHPL/HOP ]，保持在毫米，无论M200设置
 * M208 - Set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/min] 设置恢复=未撤回长度S [正MM盈余到M207 S*] F[进给量MM/min ]
 * M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
 *        S＝1＝true / 0＝FALSE >如果切片机不支持G10/ 11，则启用自动缩回检测：每个正常的仅挤压的移动将被归类为取决于方向的收缩。
 * M218 - Set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y> 设定偏移距（mm）：T＜挤出物数＞x＜OffSETION OXX> y＜OffStIsIn Oy y >
 * M220 - Set speed factor override percentage: S<factor in percent> 设置速度因子超越百分比：%>
 * M221 - Set extrude factor override percentage: S<factor in percent> 设置挤出因子超越百分比：%>
 * M226 - Wait until the specified pin reaches the state required: P<pin number> S<pin state> 等待指定引脚达到所需状态：P< PIN号> S“引脚状态>
 * M240 - Trigger a camera to take a photograph  Trigger拍摄相机
 * M250 - Set LCD contrast C<contrast value> (value 0..63)  设置LCD对比度C<对比值>（值0…63）
 * M280 - Set servo position absolute. P: servo index, S: angle or microseconds  设定伺服位置绝对值。P:伺服指数，S：角度或微秒
 * M300 - Play beep sound S<frequency Hz> P<duration ms> 播放哔哔声s频率HZ> P<持续时间ms >
 * M301 - Set PID parameters P I and D 设定PID参数P和D
 * M302 - Allow cold extrudes, or set the minimum extrude S<temperature>. 允许冷挤出，或设定最小挤出温度< >。
 * M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C) PID继电器自动调谐温度>设定目标温度。（默认目标温度＝150℃）
 * M304 - Set bed PID parameters P I and D 设定床PID参数P和D
 * M380 - Activate solenoid on active extruder 在主动挤出机上启动螺线管
 * M381 - Disable all solenoids  禁用所有螺线管
 * M400 - Finish all moves 完成所有动作
 * M401 - Lower Z probe if present 下Z探头是否存在
 * M402 - Raise Z probe if present 提高Z探头是否存在
 * M404 - N<dia in mm> Enter the nominal filament width (3mm, 1.75mm ) or will display nominal filament width without parameters
 *        N＜DIMA＞输入标称丝宽（3mm，1.75 mm）或将显示标称丝宽而无参数。
 * M405 - Turn on Filament Sensor extrusion control.  Optional D<delay in cm> to set delay in centimeters between sensor and extruder
 *        开启灯丝传感器挤压控制。在传感器和挤出机之间设置厘米延迟的可选D＜厘米＞延迟
 * M406 - Turn off Filament Sensor extrusion control 关断灯丝传感器挤压控制
 * M407 - Display measured filament diameter 显示测得的细丝直径
 * M410 - Quickstop. Abort all the planned moves Quickstop。中止所有计划的行动
 * M420 - Enable/Disable Mesh Leveling (with current values) S1=enable S0=disable 启用/禁用网格水准（具有当前值）S1=启用S0=禁用
 * M421 - Set a single Z coordinate in the Mesh Leveling grid. X<mm> Y<mm> Z<mm> 在网格水准格网中设置一个z坐标。x<mm＞y＜mm＞z＜mm
 * M428 - Set the home_offset logically based on the current_position 基于CurrnType位置逻辑地设置HOMYOBPLOND
 * M500 - Store parameters in EEPROM  EEPROM中的存储参数
 * M501 - Read parameters from EEPROM (if you need reset them after you changed them temporarily).从EEPROM读取参数（如果您在临时更改它们之后需要重新设置它们）。
 * M502 - Revert to the default "factory settings". You still need to store them in EEPROM afterwards if you want to. 还原为默认的“工厂设置”。如果需要的话，你还需要把它们存储在EEPROM中
 * M503 - Print the current settings (from memory not from EEPROM). Use S0 to leave off headings. 打印当前设置（来自内存而不是EEPROM）。使用S0来删除标题。
 * M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
 *        使用S [ 0×1 ]启用或禁用停止SD卡打印在EndoStter命中（需要Apple）
 * M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal] 
 *        暂停长丝变化x[PoS] y[PoS] z [相对提升] E [初始缩回] L [稍后撤回距离的去除]
 * M665 - Set delta configurations: L<diagonal rod> R<delta radius> S<segments/s> 设置Delta配置：L＜对角线杆＞R＜δ半径＞S段/s>
 * M666 - Set delta endstop adjustment 设置三角洲终点站调整
 * M605 - Set dual x-carriage movement mode: S<mode> [ X<duplication x-offset> R<duplication temp offset> ] 设置双X-托架移动模式：S<模式> [X重复X偏移量> R重复温度偏移>
 * M907 - Set digital trimpot motor current using axis codes. 使用轴码设置数字TrimPar电机电流。
 * M908 - Control digital trimpot directly. 直接控制数字TrimPar。
 * M350 - Set microstepping mode. 设置微步模式。
 * M351 - Toggle MS1 MS2 pins directly. 直接切换MS1 MS2引脚。
 * 
 * ************ SCARA Specific - This can change to suit future G-code regulations 
 * M360 - SCARA calibration: Move to cal-position ThetaA (0 deg calibration) SCARA校准：移动到CAL位置θ（0度校准）
 * M361 - SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree) SCARA校准：移动到CAL位置TeTAB（90度校准-每个步骤的步骤）
 * M362 - SCARA calibration: Move to cal-position PsiA (0 deg calibration) SCARA校准：移动到CAL位置PsiA（0度校准
 * M363 - SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree) 移动到CAL位置PSIB（90度校准-每个步骤的步骤）
 * M364 - SCARA calibration: Move to cal-position PSIC (90 deg to Theta calibration position) SCARA校准：移动到CAL位置PSIC（90度到θ校准位置）
 * M365 - SCARA calibration: Scaling factor, X, Y, Z axis  SCARA校准：比例因子x，y，z轴
 * ************* SCARA End ***************
 *
 * ************ Custom codes - This can change to suit future G-code regulations 定制代码-这可以改变以适应未来的G代码规则
 * M100 - Watch Free Memory (For Debugging Only)  无表内存（仅用于调试）
 * M851 - Set Z probe's Z offset (mm above extruder -- The value will always be negative) 设置Z探头的Z偏移量（在挤出机上的mm值总是为负值）


 * M928 - Start SD logging (M928 filename.g) - ended by M29 启动MD日志记录（M928文件名，G）-由M29结束
 * M999 - Restart after being stopped by error 在错误停止后重新启动
 *
 * "T" Codes
 *
 * T0-T3 - Select a tool by index (usually an extruder) [ F<mm/min> ] T0 - T3 -通过索引（通常是挤出机）选择工具[f<mm /min > ]
 *
 */

#if ENABLED(M100_FREE_MEMORY_WATCHER)  //启用（M100免费内存观察员）
  void gcode_M100();
#endif

#if ENABLED(SDSUPPORT)  //启用（SD支持）
  CardReader card;  //读卡器
#endif

bool Running = true;

uint8_t marlin_debug_flags = DEBUG_INFO | DEBUG_ERRORS;//MARLIN调试标志=调试信息>调试错误；

static float feedrate = 1500.0, saved_feedrate; //静态浮子速度＝1500，节省进给速度；
float current_position[NUM_AXIS] = { 0.0 };
static float destination[NUM_AXIS] = { 0.0 };
bool axis_known_position[3] = { false };

static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

static char* current_command, *current_command_args;
static int cmd_queue_index_r = 0;
static int cmd_queue_index_w = 0;
static int commands_in_queue = 0;
static char command_queue[BUFSIZE][MAX_CMD_SIZE];

const float homing_feedrate[] = HOMING_FEEDRATE; //惯性浮子归航进给[]=寻的进给速度；
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedrate_multiplier = 100; //100->1 200->2
int saved_feedrate_multiplier;
int extruder_multiplier[EXTRUDERS] = ARRAY_BY_EXTRUDERS1(100);
bool volumetric_enabled = false;
float filament_size[EXTRUDERS] = ARRAY_BY_EXTRUDERS1(DEFAULT_NOMINAL_FILAMENT_DIA); //浮子丝尺寸[挤出机] =挤出机阵列1（默认标称丝直径）；
float volumetric_multiplier[EXTRUDERS] = ARRAY_BY_EXTRUDERS1(1.0);  //浮式容积倍增器[挤压机]＝挤出机阵列（1）；
float home_offset[3] = { 0 };    //HOMD偏移量〔3〕＝{ 0 }；
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };

uint8_t active_extruder = 0;  //活性挤出机＝0；
int fanSpeed = 0;  //扇形速度
bool cancel_heatup = false;  //取消加热=假

const char errormagic[] PROGMEM = "Error:";
const char echomagic[] PROGMEM = "echo:";
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};

static bool relative_mode = false;  //Determines Absolute or Relative Coordinates 确定绝对坐标或相对坐标
static char serial_char;
static int serial_count = 0;
static boolean comment_mode = false;
static char* seen_pointer; ///< A pointer to find chars in the command string (X, Y, Z, E, etc.) 在命令字符串（x，y，z，e等）中查找字符的指针
const char* queued_commands_P = NULL; /* pointer to the current line in the active sequence of commands, or NULL when none 
指针在命令的活动序列中的当前行，或者当没有 NULL时为空*/
const int sensitive_pins[] = SENSITIVE_PINS; ///< Sensitive pin list for M42 M42敏感PIN表
// Inactivity shutdown  闲置停机
millis_t previous_cmd_ms = 0;
static millis_t max_inactive_time = 0;
static millis_t stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME * 1000L; //T步进不活跃时间＝默认步进器停用时间
millis_t print_job_start_ms = 0; ///< Print job start time  打印作业开始时间
millis_t print_job_stop_ms = 0;  ///< Print job stop time   打印作业停止时间
static uint8_t target_extruder;   ///目标挤出机；
bool no_wait_for_cooling = true;   ///没有等待冷却=真；
bool target_direction;      //目标方向；

#if ENABLED(AUTO_BED_LEVELING_FEATURE)  //启用（自动床调平功能）
  int xy_travel_speed = XY_TRAVEL_SPEED;
  float zprobe_zoffset = Z_PROBE_OFFSET_FROM_EXTRUDER;  //Z探头Z偏移= Z探头从挤出机偏移；
#endif

#if ENABLED(Z_DUAL_ENDSTOPS) && DISABLED(DELTA)   //启用（Z双端停止）和禁用（增量）
  float z_endstop_adj = 0;
#endif

// Extruder offsets  挤出机偏移
#if EXTRUDERS > 1
  #ifndef EXTRUDER_OFFSET_X      //挤出机偏移量X
    #define EXTRUDER_OFFSET_X { 0 }
  #endif
  #ifndef EXTRUDER_OFFSET_Y     //挤出机偏移量Y
    #define EXTRUDER_OFFSET_Y { 0 }
  #endif
  float extruder_offset[][EXTRUDERS] = {
    EXTRUDER_OFFSET_X,
    EXTRUDER_OFFSET_Y
    #if ENABLED(DUAL_X_CARRIAGE)
      , { 0 } // supports offsets in XYZ plane  支持XYZ平面偏移
    #endif
  };
#endif

#if HAS_SERVO_ENDSTOPS    //有伺服终点站
  const int servo_endstop_id[] = SERVO_ENDSTOP_IDS;
  const int servo_endstop_angle[][2] = SERVO_ENDSTOP_ANGLES;
#endif

#if ENABLED(BARICUDA)
  int ValvePressure = 0;
  int EtoPPressure = 0;
#endif

#if ENABLED(FWRETRACT)   //启用（FWRECACTACT）

  bool autoretract_enabled = false;  //自动反应使能
  bool retracted[EXTRUDERS] = { false };
  bool retracted_swap[EXTRUDERS] = { false };

  float retract_length = RETRACT_LENGTH;                           //浮筒缩回长度=缩回长度；
  float retract_length_swap = RETRACT_LENGTH_SWAP;                 //浮动缩回长度互换=缩回长度交换；
  float retract_feedrate = RETRACT_FEEDRATE;                       //浮筒回缩进给=回缩进给速度；
  float retract_zlift = RETRACT_ZLIFT;                             //浮筒缩回Z举升=缩回Z举升；
  float retract_recover_length = RETRACT_RECOVER_LENGTH;           //浮筒缩回恢复长度=缩回恢复长度；
  float retract_recover_length_swap = RETRACT_RECOVER_LENGTH_SWAP; //浮点缩回恢复长度交换=缩回恢复长度交换；
  float retract_recover_feedrate = RETRACT_RECOVER_FEEDRATE;       //浮筒回缩恢复速度＝缩回恢复速度；

#endif // FWRETRACT

#if ENABLED(ULTIPANEL) && HAS_POWER_SWITCH     //启用（ULTIPLAND）&具有电源开关
  bool powersupply =
    #if ENABLED(PS_DEFAULT_OFF)    //电源=如果启用（PS默认关闭）
      false
    #else
      true
    #endif
  ;
#endif

#if ENABLED(DELTA)

  #define TOWER_1 X_AXIS
  #define TOWER_2 Y_AXIS
  #define TOWER_3 Z_AXIS

  float delta[3] = { 0 };
  #define SIN_60 0.8660254037844386
  #define COS_60 0.5
  float endstop_adj[3] = { 0 };
  // these are the default values, can be overriden with M665  这些是默认值，可以用M665来重写。
  float delta_radius = DELTA_RADIUS;
  float delta_tower1_x = -SIN_60 * (delta_radius + DELTA_RADIUS_TRIM_TOWER_1); // front left tower
  float delta_tower1_y = -COS_60 * (delta_radius + DELTA_RADIUS_TRIM_TOWER_1);
  float delta_tower2_x =  SIN_60 * (delta_radius + DELTA_RADIUS_TRIM_TOWER_2); // front right tower
  float delta_tower2_y = -COS_60 * (delta_radius + DELTA_RADIUS_TRIM_TOWER_2);
  float delta_tower3_x = 0;                                                    // back middle tower
  float delta_tower3_y = (delta_radius + DELTA_RADIUS_TRIM_TOWER_3);
  float delta_diagonal_rod = DELTA_DIAGONAL_ROD;
  float delta_diagonal_rod_trim_tower_1 = DELTA_DIAGONAL_ROD_TRIM_TOWER_1;
  float delta_diagonal_rod_trim_tower_2 = DELTA_DIAGONAL_ROD_TRIM_TOWER_2;
  float delta_diagonal_rod_trim_tower_3 = DELTA_DIAGONAL_ROD_TRIM_TOWER_3;
  float delta_diagonal_rod_2_tower_1 = sq(delta_diagonal_rod + delta_diagonal_rod_trim_tower_1);
  float delta_diagonal_rod_2_tower_2 = sq(delta_diagonal_rod + delta_diagonal_rod_trim_tower_2);
  float delta_diagonal_rod_2_tower_3 = sq(delta_diagonal_rod + delta_diagonal_rod_trim_tower_3);
  //float delta_diagonal_rod_2 = sq(delta_diagonal_rod);
  float delta_segments_per_second = DELTA_SEGMENTS_PER_SECOND;
  #if ENABLED(AUTO_BED_LEVELING_FEATURE)  //如果启用（自动床调平特性）
    int delta_grid_spacing[2] = { 0, 0 };
    float bed_level[AUTO_BED_LEVELING_GRID_POINTS][AUTO_BED_LEVELING_GRID_POINTS];  //浮床水平[自动床调平网格点] [自动床调平网格点]；
  #endif
#else
  static bool home_all_axis = true;
#endif

#if ENABLED(SCARA)
  float delta_segments_per_second = SCARA_SEGMENTS_PER_SECOND;
  static float delta[3] = { 0 };
  float axis_scaling[3] = { 1, 1, 1 };    // Build size scaling, default to 1  Buffe大小缩放，默认为1
#endif

#if ENABLED(FILAMENT_SENSOR)   //如果启用（丝线传感器）     ///断料检测
  //Variables for Filament Sensor input  用于灯丝传感器输入的变量
  float filament_width_nominal = DEFAULT_NOMINAL_FILAMENT_DIA;  //Set nominal filament width, can be changed with M404  设定标称灯丝宽度，可随M404改变。
  bool filament_sensor = false;  //M405 turns on filament_sensor control, M406 turns it off   M405打开丝状感应器控制，M406关闭它
  float filament_width_meas = DEFAULT_MEASURED_FILAMENT_DIA; //Stores the measured filament diameter  储存测得的细丝直径
  signed char measurement_delay[MAX_MEASUREMENT_DELAY + 1]; //ring buffer to delay measurement  store  extruder factor after subtracting 100 环形缓冲区减去测量减去100后的挤出因子
  int delay_index1 = 0;  //index into ring buffer  环缓冲区索引
  int delay_index2 = -1;  //index into ring buffer - set to -1 on startup to indicate ring buffer needs to be initialized  索引到环形缓冲区-启动时设置为- 1以指示环缓冲区需要初始化
  float delay_dist = 0; //delay distance counter  延迟距离计数器
  int meas_delay_cm = MEASUREMENT_DELAY_CM;  //distance delay setting  距离延迟设置
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)   //启用（灯丝跳动传感器）
  static bool filrunoutEnqueued = false;
#endif

#if ENABLED(SDSUPPORT)     /// //启用（SD支持）    ///断电续打
  static bool fromsd[BUFSIZE];
  #if ENABLED(POWEROFF_SAVE_SD_FILE)         //如果启用（断电保存SD文件）
    #define SAVE_INFO_INTERVAL (1000 * 10)   //定义保存信息间隔（1000×10） 
    #define APPEND_CMD_COUNT 5               //定义附加的CMD计数5
    //#define SAVE_EACH_CMD_MODE             //定义保存每个CMD模式
    struct power_off_info_t power_off_info;  //关机信息关闭电源关闭信息；
    static char power_off_commands[BUFSIZE + APPEND_CMD_COUNT][MAX_CMD_SIZE];
    int power_off_commands_count = 0;        //关闭命令计数＝0；
    int power_off_type_yes = 0;
    static int power_off_commands_index = 0;  //静态int关闭命令＝0；
  #endif   //
#endif

#if HAS_SERVOS                         //如果有伺服系统
  Servo servo[NUM_SERVOS];            //伺服伺服[数控伺服]；
#endif

#ifdef CHDK
  unsigned long chdkHigh = 0;
  boolean chdkActive = false;
#endif

#if ENABLED(PID_ADD_EXTRUSION_RATE)
  int lpq_len = 20;
#endif

//===========================================================================
//================================ Functions功能 ============================
//===========================================================================

void process_next_command();   //进程下命令

void plan_arc(float target[NUM_AXIS], float* offset, uint8_t clockwise);

bool setTargetedHotend(int code);

void serial_echopair_P(const char* s_P, int v)           { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, long v)          { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, float v)         { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, double v)        { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, unsigned long v) { serialprintPGM(s_P); SERIAL_ECHO(v); }

#if ENABLED(PREVENT_DANGEROUS_EXTRUDE)    //启用（防止危险挤出）
  float extrude_min_temp = EXTRUDE_MINTEMP;  //实际挤出最小温度=设置挤出最小温度；
#endif

#if ENABLED(SDSUPPORT)
  #include "SdFatUtil.h"
  int freeMemory() { return SdFatUtil::FreeRam(); }
#else
extern "C" {
  extern unsigned int __bss_end;
  extern unsigned int __heap_start;
  extern void* __brkval;

  int freeMemory() {
    int free_memory;
    if ((int)__brkval == 0)
      free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
      free_memory = ((int)&free_memory) - ((int)__brkval);
    return free_memory;
  }
}
#endif //!SDSUPPORT

/**
 * Inject the next command from the command queue, when possible  如果可能的话，从命令队列中注入下一个命令
 * Return false only if no command was pending    仅在没有命令挂起时返回false。
 */
static bool drain_queued_commands_P() {           //排出队列命令p（）{
  if (!queued_commands_P) return false;           //如果（！）排队命令p）返回错误

  // Get the next 30 chars from the sequence of gcodes to run  从GCODE序列中获取下一个30字符
  char cmd[30];
  strncpy_P(cmd, queued_commands_P, sizeof(cmd) - 1);
  cmd[sizeof(cmd) - 1] = '\0';

  // Look for the end of line, or the end of sequence   查找行的末尾，或序列的末尾
  size_t i = 0;
  char c;
  while ((c = cmd[i]) && c != '\n') i++; // find the end of this gcode command  找到这个GCODE命令的末尾
  cmd[i] = '\0';
  if (enqueuecommand(cmd)) {      // buffer was not full (else we will retry later)  缓冲区未满（稍后我们将重试）
    if (c)
      queued_commands_P += i + 1; // move to next command   移动到下一个命令
    else 
      queued_commands_P = NULL;   // will have no more commands in the sequence  将不再有序列中的命令
  }
  return true;
}

/**
 * Record one or many commands to run from program memory.记录从程序内存运行的一个或多个命令。
 * Aborts the current queue, if any.                   中止当前队列（如果有的话）
 * Note: drain_queued_commands_P() must be called repeatedly to drain the commands afterwards 注意：必须重复调用排出队列命令p（），以便以后删除命令。
 */
void enqueuecommands_P(const char* pgcode) {
  queued_commands_P = pgcode;  // 排队命令P＝PGCODE；
  drain_queued_commands_P(); // first command executed asap (when possible)  第一个命令执行ASAP（如果可能的话）
}

/**
 * Copy a command directly into the main command buffer, from RAM.直接从RAM复制命令到主命令缓冲区。
 *
 * This is done in a non-safe way and needs a rework someday. 这是一个不安全的方式，需要有一天的返工。
 * Returns false if it doesn't add any command  如果不添加任何命令，则返回false
 */
bool enqueuecommand(const char* cmd) {      /////断料检测
  if (*cmd == ';' || commands_in_queue >= BUFSIZE) return false;

  // This is dangerous if a mixing of serial and this happens  这是危险的，如果混合的串行和这种情况发生。
  char* command = command_queue[cmd_queue_index_w];
  strcpy(command, cmd);
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_Enqueueing);
  SERIAL_ECHO(command);
  SERIAL_ECHOLNPGM("\"");
  cmd_queue_index_w = (cmd_queue_index_w + 1) % BUFSIZE;
  commands_in_queue++;
  return true;
}

void setup_killpin() {
  #if HAS_KILL
    SET_INPUT(KILL_PIN);
    WRITE(KILL_PIN, HIGH);
  #endif
}

void setup_filrunoutpin() {                     //设置文件输出文件（）
  #if HAS_FILRUNOUT                             //如果耗材已经耗尽
    pinMode(FILRUNOUT_PIN, INPUT);              //输入模式（引脚,输入）；
    #if ENABLED(ENDSTOPPULLUP_FIL_RUNOUT)       //如果启用（终端停止FIL跳动）
      WRITE(FILRUNOUT_PIN, HIGH);               //写入（文件输出引脚，高）
    #endif
  #endif
}

// Set home pin   设置回零引脚
void setup_homepin(void) {    
  #if HAS_HOME                       //如果有零点
    SET_INPUT(HOME_PIN);             //设置输入（回零引脚）
    WRITE(HOME_PIN, HIGH);           //写（回零输入，高电平）
  #endif
}


void setup_photpin() {                 //设置照相
  #if HAS_PHOTOGRAPH                   //如果有照片
    OUT_WRITE(PHOTOGRAPH_PIN, LOW);    //输出（照相引脚，低电平）
  #endif
}

void setup_powerhold() {
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, HIGH);
  #endif 
  #if HAS_POWER_SWITCH                     //如果有电源开关
    #if ENABLED(PS_DEFAULT_OFF)            //如果启用（PS默认关闭）
      OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);  //输出（PS上的引脚，PS在睡眠）
    #else
      OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE);   //输出写入（PS上的引脚，PS唤醒）
    #endif
  #endif
}

void suicide() {
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, LOW);
  #endif
}

void servo_init() {
  #if NUM_SERVOS >= 1 && HAS_SERVO_0
    servo[0].attach(SERVO0_PIN);
    servo[0].detach(); // Just set up the pin. We don't have a position yet. Don't move to a random position.只需安装PIN。我们还没有职位。不要移动到一个随机的位置。
  #endif
  #if NUM_SERVOS >= 2 && HAS_SERVO_1
    servo[1].attach(SERVO1_PIN);
    servo[1].detach();
  #endif
  #if NUM_SERVOS >= 3 && HAS_SERVO_2
    servo[2].attach(SERVO2_PIN);
    servo[2].detach();
  #endif
  #if NUM_SERVOS >= 4 && HAS_SERVO_3
    servo[3].attach(SERVO3_PIN);
    servo[3].detach();
  #endif

  // Set position of Servo Endstops that are defined  被定义的伺服端子的设置位置
  #if HAS_SERVO_ENDSTOPS
    for (int i = 0; i < 3; i++)
      if (servo_endstop_id[i] >= 0)
        servo[servo_endstop_id[i]].move(servo_endstop_angle[i][1]);
  #endif

}

/**
 * Stepper Reset (RigidBoard, et.al.)   步进重置(硬板等)
 */
#if HAS_STEPPER_RESET                        //已重置步进程序
  void disableStepperDrivers() {
    pinMode(STEPPER_RESET_PIN, OUTPUT);
    digitalWrite(STEPPER_RESET_PIN, LOW);  // drive it down to hold in reset motor driver chips  把它开下来装上复位的汽车驱动芯片
  }
  void enableStepperDrivers() { pinMode(STEPPER_RESET_PIN, INPUT); }  // set to input, which allows it to be pulled high by pullups设置为输入，这样可以通过上拉将其拉高
#endif

/**
 * Marlin entry-point: Set up before the program loop  在程序循环之前设置
 *  - Set up the kill pin, filament runout, power hold  设置杀菌针，灯丝脱落，电源保持
 *  - Start the serial port   启动串行端口
 *  - Print startup messages and diagnostics   打印启动消息和诊断
 *  - Get EEPROM or default settings   输入eeprom或默认设置
 *  - Initialize managers for:   为以下目的初始化管理人员：
 *    • temperature     温度
 *    • planner         策划
 *    • watchdog        监督机构
 *    • stepper         步行者
 *    • photo pin       相机引脚
 *    • servos          伺服系统
 *    • LCD controller  lcd控制器
 *    • Digipot I2C     数字式i2c
 *    • Z probe sled    z探针雪橇
 *    • status LEDs     身份证件
 */ 
  int zhongduan = 4;
  //volatile int state = LOW ;     //挥发性INT态
  void statechange()
  {
    digitalWrite(OUT_PUT,HIGH);
  }
#if ENABLED(SDSUPPORT) && ENABLED(POWEROFF_SAVE_SD_FILE)   //启用(sd支持)(M)&启用(Poweroff保存sd文件)////断电续打
void init_power_off_info () {                       //电源关闭信息()
	int i = 0;
	memset(&power_off_info, 0, sizeof(power_off_info));
	memset(power_off_commands, 0, sizeof(power_off_commands));
	if (!card.cardOK){
		card.initsd();
	}
	if (card.cardOK) {
		SERIAL_PROTOCOLLN("Init power off infomation.");   // 关掉电源
		SERIAL_PROTOCOLLN("size: ");
		SERIAL_PROTOCOLLN(sizeof(power_off_info));
		strncpy_P(power_off_info.power_off_filename, PSTR("bin"), sizeof(power_off_info.power_off_filename) - 1);
		if (card.existPowerOffFile(power_off_info.power_off_filename)) {
			card.openPowerOffFile(power_off_info.power_off_filename, O_READ);
			card.getPowerOffInfo(&power_off_info, sizeof(power_off_info));
			card.closePowerOffFile();
     //   card.removePowerOffFile();   
			SERIAL_PROTOCOLLN("init valid: ");
			SERIAL_PROTOCOLLN((unsigned long)power_off_info.valid_head);
      SERIAL_PROTOCOLLN((unsigned long)power_off_info.valid_foot);
      if ((power_off_info.valid_head != 0) && (power_off_info.valid_head == power_off_info.valid_foot)) { 
  			/* --------------------------------------------------------------------- */
  			enable_z();
        SERIAL_PROTOCOLLN("current_position(X,Y,Z,E,F,T1..T4,B): ");
  			for (i = 0; i < NUM_AXIS; i++) {
  				//current_position[i] = power_off_info.current_position[i];      //当前位置，位置我=电源关闭信息。
  				SERIAL_PROTOCOLLN(power_off_info.current_position[i]);           //串联原胶（电源关闭，当前位置[I]）；
  			}
  			//feedrate = power_off_info.feedrate;     馈电=电源中断
  			SERIAL_PROTOCOLLN(power_off_info.feedrate);     ///串行原胶（电源断开，给料速度）
  			for (i = 0; i < 4; i++) {
  				//target_temperature[i] = power_off_info.target_temperature[i];   // targe温度=电源关闭信息。
  				SERIAL_PROTOCOLLN(power_off_info.target_temperature[i]);          //串联原胶（切断信息电源，目标温度[I]）
  			}
  			SERIAL_PROTOCOLLN(power_off_info.target_temperature_bed);           //串联原生质体（信息的能量。目标温床）；
  			/* --------------------------------------------------------------------- */
  			SERIAL_PROTOCOLLN("cmd_queue(R,W,C,Q): ");
  			//cmd_queue_index_r = power_off_info.cmd_queue_index_r;
  			SERIAL_PROTOCOLLN(power_off_info.cmd_queue_index_r);     //Cmd队列索引R=电源关闭信息.cmd_队列索引R
  			//cmd_queue_index_w = power_off_info.cmd_queue_index_w;
  			SERIAL_PROTOCOLLN(power_off_info.cmd_queue_index_w);
  			//commands_in_queue = power_off_info.commands_in_queue;    //队列中的命令=关闭信息。队列中的命令；
  			SERIAL_PROTOCOLLN(power_off_info.commands_in_queue);
  			//memcpy(command_queue, power_off_info.command_queue, sizeof(command_queue));
  			for (i = 0; i < BUFSIZE; i++) {
  				SERIAL_PROTOCOLLN(power_off_info.command_queue[i]);
  			}
        char str_Z[16];
        char str_E[16];
        char str_Z_up[16];
        memset(str_Z, 0, sizeof(str_Z));
        memset(str_E, 0, sizeof(str_E));
        memset(str_Z_up, 0, sizeof(str_Z_up));
        dtostrf(power_off_info.current_position[2], 1, 3, str_Z);
        dtostrf(power_off_info.current_position[2] + 5, 1, 3, str_Z_up);
        #if ENABLED(SAVE_EACH_CMD_MODE)             //启用(保存每个cmd模式)
        dtostrf(power_off_info.current_position[3] - 5, 1, 3, str_E);
        #else
        dtostrf(power_off_info.current_position[3], 1, 3, str_E);
        #endif
  			//sprintf_P(power_off_commands[0], PSTR("G0 Z%s"), tmp);
        sprintf_P(power_off_commands[0], PSTR("G92 Z%s E%s"), str_Z, str_E);
        sprintf_P(power_off_commands[1], PSTR("G0 Z%s"), str_Z_up);
        sprintf_P(power_off_commands[2], PSTR("G28 X0 Y0"));
        sprintf_P(power_off_commands[3], PSTR("G0 Z%s"), str_Z);
        sprintf_P(power_off_commands[4], PSTR("M117 Printing..."));
  			power_off_commands_count = APPEND_CMD_COUNT;
  			i = APPEND_CMD_COUNT;
  			while (power_off_info.commands_in_queue > 0) {
  				strcpy(power_off_commands[i++], power_off_info.command_queue[power_off_info.cmd_queue_index_r]);
  				power_off_commands_count++;
  				power_off_info.commands_in_queue--;
  				power_off_info.cmd_queue_index_r = (power_off_info.cmd_queue_index_r + 1) % BUFSIZE;
  			}
  			for (i = 0; i < power_off_commands_count; i++) {
  				SERIAL_PROTOCOLLN(power_off_commands[i]);
  			}
  			/* --------------------------------------------------------------------- */
  			SERIAL_PROTOCOLLN("sd file(start_time,file_name,sd_pos): ");
  			SERIAL_PROTOCOLLN(power_off_info.print_job_start_ms);
  			SERIAL_PROTOCOLLN(power_off_info.sd_filename);
  			SERIAL_PROTOCOLLN(power_off_info.sdpos);
    		print_job_start_ms = power_off_info.print_job_start_ms;
    		card.openFile(power_off_info.sd_filename, true);
    		card.setIndex(power_off_info.sdpos);
  			/* --------------------------------------------------------------------- */
      }
      else {
        if ((power_off_info.valid_head != 0) && (power_off_info.valid_head != power_off_info.valid_foot)) {
        //  如：（停电了）。=0)&(电源关闭)。有效头部！=电源关闭信息。有效的脚)
          enqueuecommands_P(PSTR("M117 INVALID DATA."));    //M117无效数据
        }
        memset(&power_off_info, 0, sizeof(power_off_info));
        strncpy_P(power_off_info.power_off_filename, PSTR("bin"), sizeof(power_off_info.power_off_filename) - 1);
      }
		}
	}
}

bool drain_power_off_commands () {                  //删除命令
	if (power_off_commands_count > 0) {               //如果(断电命令数>0)
		if(enqueuecommand(power_off_commands[power_off_commands_index])) {   //如果(排队时(关闭命令[关闭命令索引])){
			power_off_commands_index++;     //关闭命令索引++的电源；
			power_off_commands_count--;     //断电命令计数--；
		}
		return true;
	}
	else {
		return false;
	}
}

void save_power_off_info () {       //储存电源关闭资讯()
	int i = 0;
  //static millis_t pre_time = millis();
  //static millis_t cur_time = millis();
	if (card.cardOK && card.sdprinting) {
    //cur_time = millis();
		if (
      #if ENABLED(SAVE_EACH_CMD_MODE)       //启用(保存每个cmd模式)
      true
      #else
      power_off_info.saved_z != current_position[2]
      #endif 
        //|| ((cur_time - pre_time) > SAVE_INFO_INTERVAL)
      ) {
      //pre_time = cur_time;
      //SERIAL_PROTOCOLLN("Z : ");
      //SERIAL_PROTOCOLLN(current_position[2]);
      //SERIAL_PROTOCOLLN(power_off_info.saved_z);
      power_off_info.valid_head = random(1,256);
      power_off_info.valid_foot = power_off_info.valid_head;
			//SERIAL_PROTOCOLLN("save valid: ");
			//SERIAL_PROTOCOLLN((unsigned long)power_off_info.valid_head);
      //SERIAL_PROTOCOLLN((unsigned long)power_off_info.valid_foot);
			/* --------------------------------------------------------------------- */
			//SERIAL_PROTOCOLLN("current_position(X,Y,Z,SZ,E,F,T1..T4,B): ");
			for (i = 0; i < NUM_AXIS; i++) {
				power_off_info.current_position[i] = current_position[i];
				//SERIAL_PROTOCOLLN(current_position[i]);
			}
			power_off_info.saved_z = current_position[2];
			//SERIAL_PROTOCOLLN(power_off_info.saved_z);
			power_off_info.feedrate = feedrate;
			//SERIAL_PROTOCOLLN(power_off_info.feedrate);
			for (i = 0; i < 4; i++) {
				power_off_info.target_temperature[i] = target_temperature[i];
				//SERIAL_PROTOCOLLN(target_temperature[i]);
			}
			power_off_info.target_temperature_bed = target_temperature_bed;
			//SERIAL_PROTOCOLLN(power_off_info.target_temperature_bed);
			/* --------------------------------------------------------------------- */
			//SERIAL_PROTOCOLLN("cmd_queue(R,W,C,Q): ");
			power_off_info.cmd_queue_index_r = cmd_queue_index_r;
			//SERIAL_PROTOCOLLN(power_off_info.cmd_queue_index_r);
			power_off_info.cmd_queue_index_w = cmd_queue_index_w;
			//SERIAL_PROTOCOLLN(power_off_info.cmd_queue_index_w);
			power_off_info.commands_in_queue = commands_in_queue;
			//SERIAL_PROTOCOLLN(power_off_info.commands_in_queue);
			memcpy(power_off_info.command_queue, command_queue, sizeof(power_off_info.command_queue));
			//for (i = 0; i < BUFSIZE; i++) {
			//	SERIAL_PROTOCOLLN(power_off_info.command_queue[i]);
			//}
			/* --------------------------------------------------------------------- */
			//SERIAL_PROTOCOLLN("sd file(start_time,file_name,sd_pos): ");
      power_off_info.print_job_start_ms = print_job_start_ms;
      //SERIAL_PROTOCOLLN(power_off_info.print_job_start_ms);
      //strcpy(power_off_info.sd_filename, 
      card.getAbsFilename(power_off_info.sd_filename);
      //SERIAL_PROTOCOLLN(power_off_info.sd_filename);
			power_off_info.sdpos = card.getIndex();
			//SERIAL_PROTOCOLLN(power_off_info.sdpos);
			/* --------------------------------------------------------------------- */
			card.openPowerOffFile(power_off_info.power_off_filename, O_CREAT | O_WRITE | O_TRUNC | O_SYNC);
			if (card.savePowerOffInfo(&power_off_info, sizeof(power_off_info)) == -1){
				SERIAL_PROTOCOLLN("Write power off file failed.");
			}
		}
	}
}
#endif   /////////断电续打
 
void setup() {
  pinMode(IN_PUT,INPUT);
  pinMode(OUT_PUT,OUTPUT);
  WRITE(IN_PUT,HIGH);
  WRITE(OUT_PUT,LOW);

  attachInterrupt(zhongduan,statechange,FALLING);
 
  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);
  
  setup_killpin();
  setup_filrunoutpin();
  setup_powerhold();

  #if HAS_STEPPER_RESET
    disableStepperDrivers();
  #endif

  MYSERIAL.begin(BAUDRATE);
  SERIAL_PROTOCOLLNPGM("start");
  SERIAL_ECHO_START;         //串行回波启动；

  // Check startup - does nothing if bootloader sets MCUSR to 0    检查启动-如果引导加载程序设置为0，则不执行任何操作
  byte mcu = MCUSR;
  if (mcu & 1) SERIAL_ECHOLNPGM(MSG_POWERUP);
  if (mcu & 2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
  if (mcu & 4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
  if (mcu & 8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
  if (mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
  MCUSR = 0;

  SERIAL_ECHOPGM(MSG_MARLIN);
  SERIAL_ECHOLNPGM(" " SHORT_BUILD_VERSION);

  #ifdef STRING_DISTRIBUTION_DATE
    #ifdef STRING_CONFIG_H_AUTHOR
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
      SERIAL_ECHOPGM(STRING_DISTRIBUTION_DATE);
      SERIAL_ECHOPGM(MSG_AUTHOR);
      SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
      SERIAL_ECHOPGM("Compiled: ");
      SERIAL_ECHOLNPGM(__DATE__);
    #endif // STRING_CONFIG_H_AUTHOR
  #endif // STRING_DISTRIBUTION_DATE

  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_FREE_MEMORY);
  SERIAL_ECHO(freeMemory());
  SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
  SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);

  #if ENABLED(SDSUPPORT)
    for (int8_t i = 0; i < BUFSIZE; i++) fromsd[i] = false;
  #endif

  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  //如果有其他可用的数据，则使用默认值（并重置步进加速率）
  Config_RetrieveSettings();   //配置检索()；

  lcd_init();

  tp_init();    // Initialize temperature loop  Tp输入();//初始化温度循环
  plan_init();  // Initialize planner;         计划()；//初始化计划；

  #if ENABLED(USE_WATCHDOG)   //如果启用（使用看门狗)
    watchdog_init();           //看门狗在里面()；
  #endif      

  st_init();    // Initialize stepper, this enables interrupts!   //初始化步进程序，这将允许中断
  setup_photpin();        //安装光子()；
  servo_init();           //伺服在内()；

  #if HAS_CONTROLLERFAN      //如果有控制风扇
    SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan    设定输出(控制键);//设定键用于驱动风扇
  #endif

  #if HAS_STEPPER_RESET        //如果有步骤重置
    enableStepperDrivers();    //可辅助步行者()；
  #endif

  #if ENABLED(DIGIPOT_I2C)
    digipot_i2c_init();
  #endif

  #if ENABLED(Z_PROBE_SLED)     //如果启用（z探针雪橇）
    pinMode(SLED_PIN, OUTPUT);
    digitalWrite(SLED_PIN, LOW); // turn it off  把它关掉
  #endif // Z_PROBE_SLED  z探针雪橇

  setup_homepin();   //安装首页()；

  #ifdef STAT_LED_RED                 // 如果你是红色的
    pinMode(STAT_LED_RED, OUTPUT);    //针模（统计引导红色，输出）；
    digitalWrite(STAT_LED_RED, LOW); // turn it off  数字书写（数字引导的红色，低
  #endif

  #ifdef STAT_LED_BLUE               //如果你的头发是蓝色的
    pinMode(STAT_LED_BLUE, OUTPUT);  //针模（统计引导蓝色，输出）
    digitalWrite(STAT_LED_BLUE, LOW); // turn it off  数字书写（统计为蓝色，低
  #endif
	//设置检测材料的引脚
	pinMode(CHECK_MATWEIAL, INPUT);    //标准模式（检查参数，输入）；
    digitalWrite(CHECK_MATWEIAL, HIGH); // turn it off  数字书写（请检查数据，高）

	//设置检测材料的引脚
//	pinMode(65, OUTPUT);
//    digitalWrite(65, LOW); // turn it off

	//设置电源开关引脚
/*	pinMode(IN_PUT, INPUT);
    digitalWrite(IN_PUT, HIGH); // turn it off
	//设置电源开关引脚
	pinMode(OUT_PUT, OUTPUT);
   digitalWrite(OUT_PUT, LOW); // turn it off*/
  #if ENABLED(SDSUPPORT) && ENABLED(POWEROFF_SAVE_SD_FILE)    ///断电续打
   init_power_off_info();
  #endif
}

/**
 * The main Marlin program loop   主要的马林程序循环
 *
 *  - Save or log commands to SD   保存或记录到sd的命令
 *  - Process available commands (if not saving)  处理可用的命令（如果不保存）
 *  - Call heater manager 打电话给加热器经理
 *  - Call inactivity manager  调用不活动管理器
 *  - Call endstop manager     呼叫终端管理器
 *  - Call LCD update          呼叫液晶更新
 */
void loop() {
  if (commands_in_queue < BUFSIZE - 1) get_command();

  #if ENABLED(SDSUPPORT)
    card.checkautostart(false);
  #endif

  if (commands_in_queue) {

    #if ENABLED(SDSUPPORT)

      if (card.saving) {
        char* command = command_queue[cmd_queue_index_r];
        if (strstr_P(command, PSTR("M29"))) {
          // M29 closes the file   m29关闭文件
          card.closefile();        //已结案的案件()；
          SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED);   //串行原型文件(msg文件已保存)；
        }
        else {
          // Write the string from the read buffer to SD   将读取的缓冲区中的字符串写入sd
          card.write_command(command);
          if (card.logging)
            process_next_command(); // The card is saving because it's logging  卡片在保存，因为它在记录
          else
            SERIAL_PROTOCOLLNPGM(MSG_OK);
        }
      }
      else {
        process_next_command();              //////处理下一个命令()； 
        #if ENABLED(SDSUPPORT) && ENABLED(POWEROFF_SAVE_SD_FILE)   /// //如果已启用(sdsids)，则已启用(poweroff保存sd文件)(F)
        save_power_off_info();    //储存电源关闭资讯()；   //////断电续打
        #endif
      }

    #else

      process_next_command();

    #endif // SDSUPPORT  SD支助

    commands_in_queue--;
    cmd_queue_index_r = (cmd_queue_index_r + 1) % BUFSIZE;
  }
  checkHitEndstops();
  idle();
}

void gcode_line_error(const char* err, bool doFlush = true) {
  SERIAL_ERROR_START;
  serialprintPGM(err);
  SERIAL_ERRORLN(gcode_LastN);     //系列错误（gcode lastn）；
  //Serial.println(gcode_N);       //序列.印刷(gcode n)；
  if (doFlush) FlushSerialRequestResend();   //如果(同调)拖放序列化请求()
  serial_count = 0;
}

/**
 * Add to the circular command queue the next command from:  将下一个命令添加到循环命令队列中：
 *  - The command-injection queue (queued_commands_P)命令注入队列（队列命令p）
 *  - The active serial input (usually USB)  活动串行输入(通常为usb)
 *  - The SD card file being actively printed  正积极印制sd卡档案
 */
void get_command() {

  if (drain_queued_commands_P()) return; // priority is given to non-serial commands  优先处理非串行命令  /////断电续打

  #if ENABLED(NO_TIMEOUTS)
    static millis_t last_command_time = 0;
    millis_t ms = millis();

    if (!MYSERIAL.available() && commands_in_queue == 0 && ms - last_command_time > NO_TIMEOUTS) {
      SERIAL_ECHOLNPGM(MSG_WAIT);    //系列电子产品（msg等）
      last_command_time = ms;       //最后命令时间=MS；
    }
  #endif

  //
  // Loop while serial characters are incoming and the queue is not full  当串行字符传入且队列未满时循环
  //
  while (commands_in_queue < BUFSIZE && MYSERIAL.available() > 0) {     //同时(队列中的命令0){

    #if ENABLED(NO_TIMEOUTS)
      last_command_time = ms;
    #endif

    serial_char = MYSERIAL.read();

    //
    // If the character ends the line, or the line is full...  如果字符结束行，或行满了...
    //
    if (serial_char == '\n' || serial_char == '\r' || serial_count >= MAX_CMD_SIZE - 1) {

      // end of line == end of comment   行尾=注释尾
      comment_mode = false;

      if (!serial_count) return; // empty lines just exit   如果(!序列_计数)返回;//空行仅退出

      char* command = command_queue[cmd_queue_index_w];
      command[serial_count] = 0; // terminate string  终止字串

      // this item in the queue is not from sd  队列中的此项目不是来自SD
      #if ENABLED(SDSUPPORT)
        fromsd[cmd_queue_index_w] = false;
      #endif

      while (*command == ' ') command++; // skip any leading spaces  跳过任何领先空间
      char* npos = (*command == 'N') ? command : NULL; // Require the N parameter to start the line  需要n个参数来开始行
      char* apos = strchr(command, '*');

      if (npos) {

        boolean M110 = strstr_P(command, PSTR("M110")) != NULL;

        if (M110) {
          char* n2pos = strchr(command + 4, 'N');
          if (n2pos) npos = n2pos;
        }

        gcode_N = strtol(npos + 1, NULL, 10);

        if (gcode_N != gcode_LastN + 1 && !M110) {
          gcode_line_error(PSTR(MSG_ERR_LINE_NO));
          return;
        }

        if (apos) {
          byte checksum = 0, count = 0;
          while (command[count] != '*') checksum ^= command[count++];

          if (strtol(apos + 1, NULL, 10) != checksum) {
            gcode_line_error(PSTR(MSG_ERR_CHECKSUM_MISMATCH));
            return;
          }
          // if no errors, continue parsing  如果没有错误，继续分析
        }
        else if (npos == command) {
          gcode_line_error(PSTR(MSG_ERR_NO_CHECKSUM));
          return;
        }

        gcode_LastN = gcode_N;
        // if no errors, continue parsing
      }
      else if (apos) { // No '*' without 'N'
        gcode_line_error(PSTR(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM), false);
        //gcode行错误（pstr（msg错误，没有带有校验和的列号），错误）；
        return;
      }

      // Movement commands alert when stopped  停止时的移动命令警报
      if (IsStopped()) {
        char* gpos = strchr(command, 'G');
        if (gpos) {
          int codenum = strtol(gpos + 1, NULL, 10);
          switch (codenum) {
            case 0:
            case 1:
            case 2:
            case 3:
              SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
              LCD_MESSAGEPGM(MSG_STOPPED);
              break;
          }
        }
      }

      // If command was e-stop process now  如果命令现在是电子停止进程
      if (strcmp(command, "M112") == 0) kill(PSTR(MSG_KILLED));

      cmd_queue_index_w = (cmd_queue_index_w + 1) % BUFSIZE;
      commands_in_queue += 1;

      serial_count = 0; //clear buffer
    }
    else if (serial_char == '\\') {  // Handle escapes
      if (MYSERIAL.available() > 0 && commands_in_queue < BUFSIZE) {
        // if we have one more character, copy it over  如果我们还有一个字，就抄过来
        serial_char = MYSERIAL.read();
        command_queue[cmd_queue_index_w][serial_count++] = serial_char;
      }
      // otherwise do nothing  否则什么也不做
    } 
    else { // its not a newline, carriage return or escape char   这不是新线，不是回车，也不是逃车
      if (serial_char == ';') comment_mode = true;
      if (!comment_mode) command_queue[cmd_queue_index_w][serial_count++] = serial_char;
    }
  }

  #if ENABLED(SDSUPPORT)   //如果已启用（sd支持）

    if (!card.sdprinting || serial_count) return;  //如果(!打印||序列数)返回；

    #if ENABLED(SDSUPPORT) && ENABLED(POWEROFF_SAVE_SD_FILE)   //如果已启用(sdsend)，则启用(已关闭保存sd文件的电源)(F)
    /* 优先读取断电保存的命令。 */     ////断电续打
    if (drain_power_off_commands()) return;   //如果(耗尽电源关闭命令())返回
    #endif

    // '#' stops reading from SD to the buffer prematurely, so procedural macro calls are possible
    // if it occurs, stop_buffering is triggered and the buffer is ran dry.
    // this character _can_ occur in serial com, due to checksums. however, no checksums are used in SD printing
   /*
   过早地停止从sd读取到缓冲区，因此程序宏调用是可能的，如果发生这种情况，则触发停止缓冲并运行该缓冲区。
   由于校验和，这个字符__可以在串行com中发生。但是，在sd打印中没有使用校验和*/
    static bool stop_buffering = false;     //静态布尔停止缓冲=假；
    if (commands_in_queue == 0) stop_buffering = false;

    while (!card.eof() && commands_in_queue < BUFSIZE && !stop_buffering) {
      int16_t n = card.get();
      serial_char = (char)n;
      if (serial_char == '\n' || serial_char == '\r' ||
          ((serial_char == '#' || serial_char == ':') && !comment_mode) ||
          serial_count >= (MAX_CMD_SIZE - 1) || n == -1
      ) {
        if (card.eof()) {
          SERIAL_PROTOCOLLNPGM(MSG_FILE_PRINTED);
          print_job_stop_ms = millis();
          char time[30];
          millis_t t = (print_job_stop_ms - print_job_start_ms) / 1000;
          int hours = t / 60 / 60, minutes = (t / 60) % 60;
          sprintf_P(time, PSTR("%i " MSG_END_HOUR " %i " MSG_END_MINUTE), hours, minutes);
          SERIAL_ECHO_START;
          SERIAL_ECHOLN(time);
          lcd_setstatus(time, true);
          card.printingHasFinished();
          card.checkautostart(true);
        }
        if (serial_char == '#') stop_buffering = true;

        if (!serial_count) {
          comment_mode = false; //for new command
          return; //if empty line
        }
        command_queue[cmd_queue_index_w][serial_count] = 0; //terminate string
        // if (!comment_mode) {
        fromsd[cmd_queue_index_w] = true;
        commands_in_queue += 1;
        cmd_queue_index_w = (cmd_queue_index_w + 1) % BUFSIZE;
        // }
        comment_mode = false; //for new command
        serial_count = 0; //clear buffer
      }
      else {
        if (serial_char == ';') comment_mode = true;
        if (!comment_mode) command_queue[cmd_queue_index_w][serial_count++] = serial_char;
      }
    }

  #endif // SDSUPPORT
}

bool code_has_value() {
  int i = 1;
  char c = seen_pointer[i];
  if (c == '-' || c == '+') c = seen_pointer[++i];
  if (c == '.') c = seen_pointer[++i];
  return (c >= '0' && c <= '9');
}

float code_value() {
  float ret;
  char* e = strchr(seen_pointer, 'E');
  if (e) {
    *e = 0;
    ret = strtod(seen_pointer + 1, NULL);
    *e = 'E';
  }
  else
    ret = strtod(seen_pointer + 1, NULL);
  return ret;
}

long code_value_long() { return strtol(seen_pointer + 1, NULL, 10); }

int16_t code_value_short() { return (int16_t)strtol(seen_pointer + 1, NULL, 10); }

bool code_seen(char code) {
  seen_pointer = strchr(current_command_args, code);
  return (seen_pointer != NULL); // Return TRUE if the code-letter was found  如果找到了代码字母，返回真
}

#define DEFINE_PGM_READ_ANY(type, reader)       \
  static inline type pgm_read_any(const type *p)  \
  { return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
  static const PROGMEM type array##_P[3] =        \
      { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
  static inline type array(int axis)          \
  { return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,   MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,   MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,  HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,     MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_bump_mm,   HOME_BUMP_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir, HOME_DIR);

#if ENABLED(DUAL_X_CARRIAGE)

  #define DXC_FULL_CONTROL_MODE 0
  #define DXC_AUTO_PARK_MODE    1
  #define DXC_DUPLICATION_MODE  2

  static int dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;

  static float x_home_pos(int extruder) {
    if (extruder == 0)
      return base_home_pos(X_AXIS) + home_offset[X_AXIS];
    else
      // In dual carriage mode the extruder offset provides an override of the
      // second X-carriage offset when homed - otherwise X2_HOME_POS is used.
      // This allow soft recalibration of the second extruder offset position without firmware reflash
      // (through the M218 command).
      /*在双托架模式下，挤出机偏置提供了在归航时的第二个x-trab偏移量-否则使用x2自动机pos。
      这样就可以对第二挤出机的偏置位置进行软校正，而不需要固件的反射（通过m218命令）。*/
      return (extruder_offset[X_AXIS][1] > 0) ? extruder_offset[X_AXIS][1] : X2_HOME_POS;
  }

  static int x_home_dir(int extruder) {
    return (extruder == 0) ? X_HOME_DIR : X2_HOME_DIR;
  }

  static float inactive_extruder_x_pos = X2_MAX_POS; // used in mode 0 & 1
  static bool active_extruder_parked = false; // used in mode 1 & 2
  static float raised_parked_position[NUM_AXIS]; // used in mode 1
  static millis_t delayed_move_time = 0; // used in mode 1
  static float duplicate_extruder_x_offset = DEFAULT_DUPLICATION_X_OFFSET; // used in mode 2
  static float duplicate_extruder_temp_offset = 0; // used in mode 2
  bool extruder_duplication_enabled = false; // used in mode 2

#endif //DUAL_X_CARRIAGE   双x运输

#if ENABLED(DEBUG_LEVELING_FEATURE)   //如果已启用（调试级特性）
  void print_xyz(const char* prefix, const float x, const float y, const float z) {
    SERIAL_ECHO(prefix);  //串列回波(前缀)；
    SERIAL_ECHOPAIR(": (", x);
    SERIAL_ECHOPAIR(", ", y);
    SERIAL_ECHOPAIR(", ", z);
    SERIAL_ECHOLNPGM(")");
  }
  void print_xyz(const char* prefix, const float xyz[]) {
    print_xyz(prefix, xyz[X_AXIS], xyz[Y_AXIS], xyz[Z_AXIS]);
  }
#endif

static void set_axis_is_at_home(AxisEnum axis) {   //设轴在家中（轴经轴）

  #if ENABLED(DUAL_X_CARRIAGE)                                                   //如果启用（对偶x运输）
    if (axis == X_AXIS) {                                                        //如果(轴==x轴){
      if (active_extruder != 0) {                                                //如果（主动挤出机！=0)
        current_position[X_AXIS] = x_home_pos(active_extruder);                  //当前位置【x轴】=X家POS（主动挤出机）
                 min_pos[X_AXIS] = X2_MIN_POS;                                   //最小POS【x轴】=x2分pos；
                 max_pos[X_AXIS] = max(extruder_offset[X_AXIS][1], X2_MAX_POS);  //最大POS[x轴]=最大(挤出机偏移量[x轴][1],最大x2);
        return;
      }
      else if (dual_x_carriage_mode == DXC_DUPLICATION_MODE) {                   //否则如果(双X传输模式==dxc复制模式)
        float xoff = home_offset[X_AXIS];                                         //浮点数xoff=主偏移量[x轴]；
        current_position[X_AXIS] = base_home_pos(X_AXIS) + xoff;                  //当前位置【x轴】=基地POS(x轴)+xoff；
                 min_pos[X_AXIS] = base_min_pos(X_AXIS) + xoff;
                 max_pos[X_AXIS] = min(base_max_pos(X_AXIS) + xoff, max(extruder_offset[X_AXIS][1], X2_MAX_POS) - duplicate_extruder_x_offset);
        return;
      }
    }
  #endif

  #if ENABLED(SCARA)

    if (axis == X_AXIS || axis == Y_AXIS) {

      float homeposition[3];
      for (int i = 0; i < 3; i++) homeposition[i] = base_home_pos(i);

      // SERIAL_ECHOPGM("homeposition[x]= "); SERIAL_ECHO(homeposition[0]);
      // SERIAL_ECHOPGM("homeposition[y]= "); SERIAL_ECHOLN(homeposition[1]);
      // Works out real Homeposition angles using inverse kinematics,
      // and calculates homing offset using forward kinematics
      calculate_delta(homeposition);

      // SERIAL_ECHOPGM("base Theta= "); SERIAL_ECHO(delta[X_AXIS]);
      // SERIAL_ECHOPGM(" base Psi+Theta="); SERIAL_ECHOLN(delta[Y_AXIS]);

      for (int i = 0; i < 2; i++) delta[i] -= home_offset[i];

      // SERIAL_ECHOPGM("addhome X="); SERIAL_ECHO(home_offset[X_AXIS]);
      // SERIAL_ECHOPGM(" addhome Y="); SERIAL_ECHO(home_offset[Y_AXIS]);
      // SERIAL_ECHOPGM(" addhome Theta="); SERIAL_ECHO(delta[X_AXIS]);
      // SERIAL_ECHOPGM(" addhome Psi+Theta="); SERIAL_ECHOLN(delta[Y_AXIS]);

      calculate_SCARA_forward_Transform(delta);

      // SERIAL_ECHOPGM("Delta X="); SERIAL_ECHO(delta[X_AXIS]);
      // SERIAL_ECHOPGM(" Delta Y="); SERIAL_ECHOLN(delta[Y_AXIS]);

      current_position[axis] = delta[axis];

      // SCARA home positions are based on configuration since the actual limits are determined by the
      // inverse kinematic transform.斯卡拉家庭职位是基于配置的，因为实际的限制是由逆运动学变换。
      min_pos[axis] = base_min_pos(axis); // + (delta[axis] - base_home_pos(axis));
      max_pos[axis] = base_max_pos(axis); // + (delta[axis] - base_home_pos(axis));
    }
    else
  #endif
  {
    current_position[axis] = base_home_pos(axis) + home_offset[axis];
    min_pos[axis] = base_min_pos(axis) + home_offset[axis];
    max_pos[axis] = base_max_pos(axis) + home_offset[axis];

    #if ENABLED(AUTO_BED_LEVELING_FEATURE) && Z_HOME_DIR < 0
      if (axis == Z_AXIS) current_position[Z_AXIS] -= zprobe_zoffset;
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        SERIAL_ECHOPAIR("set_axis_is_at_home ", (unsigned long)axis);
        SERIAL_ECHOPAIR(" > (home_offset[axis]==", home_offset[axis]);
        print_xyz(") > current_position", current_position);
      }
    #endif
  }
}

/**
 * Some planner shorthand inline functions  一些简要的内联函数
 */
inline void set_homing_bump_feedrate(AxisEnum axis) {
  const int homing_bump_divisor[] = HOMING_BUMP_DIVISOR;
  int hbd = homing_bump_divisor[axis];
  if (hbd < 1) {
    hbd = 10;
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Warning: Homing Bump Divisor < 1");  //串行消长("警告：归巢凹凸除数<1")
  }
  feedrate = homing_feedrate[axis] / hbd;
}
inline void line_to_current_position() {
  plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate / 60, active_extruder);
}
inline void line_to_z(float zPosition) {
  plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate / 60, active_extruder);
}
inline void line_to_destination(float mm_m) {
  plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], mm_m / 60, active_extruder);
}
inline void line_to_destination() {
  line_to_destination(feedrate);
}
inline void sync_plan_position() {
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}
#if ENABLED(DELTA) || ENABLED(SCARA)
  inline void sync_plan_position_delta() {
    calculate_delta(current_position);
    plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
  }
#endif
inline void set_current_to_destination() { memcpy(current_position, destination, sizeof(current_position)); }
inline void set_destination_to_current() { memcpy(destination, current_position, sizeof(destination)); }

static void setup_for_endstop_move() {     //停止移动的静态空设置()
  saved_feedrate = feedrate;               //节省的给料=给料；
  saved_feedrate_multiplier = feedrate_multiplier;   //节省_给料率乘数=给料率乘数；
  feedrate_multiplier = 100;               //馈率乘数=100
  refresh_cmd_timeout();
  #if ENABLED(DEBUG_LEVELING_FEATURE)              //启用(调试级别功能)
    if (marlin_debug_flags & DEBUG_LEVELING) {     //如果(马林调试标志和调试级别)
      SERIAL_ECHOLNPGM("setup_for_endstop_move > enable_endstops(true)");  //串列的echolnpgm（“终端移动设置”启用终端（真））；
    }
  #endif
  enable_endstops(true);
}

#if ENABLED(AUTO_BED_LEVELING_FEATURE)   //已启用（自动床平整功能）

  #if ENABLED(DELTA)
    /**
     * Calculate delta, start a line, and set current_position to destination  计算Delta，启动一行，并将当前位置设置为目标
     */
    void prepare_move_raw() {                                       //空准备未加工的移动(
      #if ENABLED(DEBUG_LEVELING_FEATURE)                           //如果启用（调试级别功能）
        if (marlin_debug_flags & DEBUG_LEVELING) {                  //如果(马林调试标志和调试级别)
          print_xyz("prepare_move_raw > destination", destination); //打印XYZ（“准备移动原始>目标”，目标）；
        }
      #endif
      refresh_cmd_timeout();                        //刷新CMD超时()
      calculate_delta(destination);                 //计算三角洲（目的地）
      plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], destination[E_AXIS], (feedrate / 60) * (feedrate_multiplier / 100.0), active_extruder);
     //规划缓冲线(德尔塔[x轴]、德尔塔[y轴]、德尔塔[z轴]、目的地[e轴]、(进料率/60)*(进料率乘数/100.0)、有源挤出机)；
      set_current_to_destination();         //设定电流到目的地()；
    }
  #endif

  #if ENABLED(AUTO_BED_LEVELING_GRID)   //如果启用（自动平铺网格）

    #if DISABLED(DELTA)   //如果已禁用(三角形)

      static void set_bed_level_equation_lsq(double* plane_equation_coefficients) {    //静空设置床级方程式lsq(双*平面方程式系数)
        vector_3 planeNormal = vector_3(-plane_equation_coefficients[0], -plane_equation_coefficients[1], 1);
        planeNormal.debug("planeNormal");                                  //调试（“平面”）；
//        plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);   //计划床层矩阵=矩阵3x3：：创建看（平面正态）；
        //bedLevel.debug("bedLevel");                                 //床层。调试

        //plan_bed_level_matrix.debug("bed level before");            //平面级矩阵。除错（“床前水平”）
        //vector_3 uncorrected_position = plan_get_position_mm();     //向量_3不正确的位置=平面位置_mm()；
        //uncorrected_position.debug("position before");              //不正确的位置。除错（“位置前”）

        vector_3 corrected_position = plan_get_position();            //矢量3修正位置=平面位置()；
        //corrected_position.debug("position after");                 //修正位置。除错（“后置”）
        current_position[X_AXIS] = corrected_position.x;              //当前位置[x_轴]=修正位置。X
        current_position[Y_AXIS] = corrected_position.y;              //当前位置[y_轴]=修正位置。Y
        current_position[Z_AXIS] = corrected_position.z;              //当前位置[z_轴]=修正位置。Z
 
        #if ENABLED(DEBUG_LEVELING_FEATURE)                           //如果已启用（调试级/功能）
          if (marlin_debug_flags & DEBUG_LEVELING) {                  //如果(马林调试标志和调试级别)
            print_xyz("set_bed_level_equation_lsq > current_position", current_position);//打印XYZ（“设置床级方程式lsq>当前位置”，当前位置
          }
        #endif

        sync_plan_position();   //同步计划位置()
      }

    #endif // !DELTA   //三角洲

  #else // !AUTO_BED_LEVELING_GRID      //自动床平整网格

    static void set_bed_level_equation_3pts(float z_at_pt_1, float z_at_pt_2, float z_at_pt_3) {

      plan_bed_level_matrix.set_to_identity();

      vector_3 pt1 = vector_3(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, z_at_pt_1);
      vector_3 pt2 = vector_3(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, z_at_pt_2);
      vector_3 pt3 = vector_3(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, z_at_pt_3);
      vector_3 planeNormal = vector_3::cross(pt1 - pt2, pt3 - pt2).get_normal();

      if (planeNormal.z < 0) {
        planeNormal.x = -planeNormal.x;
        planeNormal.y = -planeNormal.y;
        planeNormal.z = -planeNormal.z;
      }

      plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);  //计划床层矩阵=矩阵3x3：：创建看（平面正态）

      vector_3 corrected_position = plan_get_position();
      current_position[X_AXIS] = corrected_position.x;
      current_position[Y_AXIS] = corrected_position.y;
      current_position[Z_AXIS] = corrected_position.z;

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
          print_xyz("set_bed_level_equation_3pts > current_position", current_position);
        }
      #endif

      sync_plan_position();
    }

  #endif // !AUTO_BED_LEVELING_GRID   //自动床平整网格

  static void run_z_probe() {   //运行Z探针

    #if ENABLED(DELTA)       //如果已启用(三角洲)

      float start_z = current_position[Z_AXIS];    //浮启动Z=当前位置【z轴】
      long start_steps = st_get_position(Z_AXIS);  //长起点步骤=St get位置（z轴）

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
          SERIAL_ECHOLNPGM("run_z_probe (DELTA) 1");  //串列电子脉冲（"运行Z探针(Delta)1"）
        }
      #endif

      // move down slowly until you find the bed   慢慢往下走，直到找到床
      feedrate = homing_feedrate[Z_AXIS] / 4;  //进料率=同源进料率[z_轴]/4
      destination[Z_AXIS] = -10;         //目的【z_轴】=-10；
      prepare_move_raw(); // this will also set_current_to_destination  这也将设置到目的地的电流
      st_synchronize();
      endstops_hit_on_purpose(); // clear endstop hit flags  清除终端命中标志

      // we have to let the planner know where we are right now as it is not where we said to go.
      //我们必须让计划者知道我们现在在哪里，因为这不是我们说好要去的地方。
//      long stop_steps = st_get_position(Z_AXIS);   /长停步骤=St get位置（z轴）；
      float mm = start_z - float(start_steps - stop_steps) / axis_steps_per_unit[Z_AXIS];
      current_position[Z_AXIS] = mm;

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
          print_xyz("run_z_probe (DELTA) 2 > current_position", current_position);
        }
      #endif

      sync_plan_position_delta();  //同步计划位置三角洲

    #else // !DELTA

      plan_bed_level_matrix.set_to_identity();  //计划床级矩阵。设置为身份()
      feedrate = homing_feedrate[Z_AXIS];

      // Move down until the Z probe (or endstop?) is triggered  向下移动，直到z探测器（或终端）。已触发
      float zPosition = -(Z_MAX_LENGTH + 10);
      line_to_z(zPosition);
      st_synchronize();

      // Tell the planner where we ended up - Get this from the stepper handler告诉策划人我们在哪里结束-从踏踏员那里得到这个
      zPosition = st_get_position_mm(Z_AXIS);
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS]);

      // move up the retract distance   向上移动缩回距离
      zPosition += home_bump_mm(Z_AXIS);
      line_to_z(zPosition);
      st_synchronize();
      endstops_hit_on_purpose(); // clear endstop hit flags  清除终端命中标志

      // move back down slowly to find bed  向下移动寻找热床
      set_homing_bump_feedrate(Z_AXIS);    //设定自导冲击进料速率（z轴）

      zPosition -= home_bump_mm(Z_AXIS) * 2;
      line_to_z(zPosition);
      st_synchronize();
      endstops_hit_on_purpose(); // clear endstop hit flags

      // Get the current stepper position after bumping an endstop  在撞击终端后得到当前步进程序的位置
      current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);   //当前位置【z轴】=St get位置mm（z轴）；
      sync_plan_position();

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
          print_xyz("run_z_probe > current_position", current_position);//（“运行Z探针>电流位置”，电流位置）
        }
      #endif

    #endif // !DELTA
  }

  /**
   *  Plan a move to (X, Y, Z) and set the current_position  计划移动到(x,y,z)并设置当前位置
   *  The final current_position may not be the one that was requested  最终汇率可能不是所要求的
   */
  static void do_blocking_move_to(float x, float y, float z) {
    float oldFeedRate = feedrate;

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        print_xyz("do_blocking_move_to", x, y, z);
      }
    #endif

    #if ENABLED(DELTA)

      feedrate = XY_TRAVEL_SPEED;

      destination[X_AXIS] = x;
      destination[Y_AXIS] = y;
      destination[Z_AXIS] = z;
      prepare_move_raw(); // this will also set_current_to_destination这也将设置为“当前”到“目标”
      st_synchronize();

    #else

      feedrate = homing_feedrate[Z_AXIS];

      current_position[Z_AXIS] = z;
      line_to_current_position();
      st_synchronize();

      feedrate = xy_travel_speed;

      current_position[X_AXIS] = x;
      current_position[Y_AXIS] = y;
      line_to_current_position();
      st_synchronize();

    #endif

    feedrate = oldFeedRate;
  }

  inline void do_blocking_move_to_xy(float x, float y) { do_blocking_move_to(x, y, current_position[Z_AXIS]); }
  inline void do_blocking_move_to_x(float x) { do_blocking_move_to(x, current_position[Y_AXIS], current_position[Z_AXIS]); }
  inline void do_blocking_move_to_z(float z) { do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], z); }
  inline void raise_z_after_probing() { do_blocking_move_to_z(current_position[Z_AXIS] + Z_RAISE_AFTER_PROBING); }

  static void clean_up_after_endstop_move() {
    #if ENABLED(ENDSTOPS_ONLY_FOR_HOMING)
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
          SERIAL_ECHOLNPGM("clean_up_after_endstop_move > ENDSTOPS_ONLY_FOR_HOMING > enable_endstops(false)");
        }
      #endif
      enable_endstops(false);
    #endif
    feedrate = saved_feedrate;
    feedrate_multiplier = saved_feedrate_multiplier;
    refresh_cmd_timeout();
  }

  static void deploy_z_probe() {

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        print_xyz("deploy_z_probe > current_position", current_position);
      }
    #endif

    #if HAS_SERVO_ENDSTOPS     //如果有伺服端子

      // Engage Z Servo endstop if enabled    启用时启动z伺服端子
      if (servo_endstop_id[Z_AXIS] >= 0) servo[servo_endstop_id[Z_AXIS]].move(servo_endstop_angle[Z_AXIS][0]);
     // 如果(伺服_端点方向[z_轴]>=0)伺服[伺服_端点方向[z_轴]]。移动（伺服_端点角[z_轴][0]）；

    #elif ENABLED(Z_PROBE_ALLEN_KEY)
      feedrate = Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE;

      // If endstop is already false, the Z probe is deployed  如果端点已经是假的，则部署z探针
      #if ENABLED(Z_MIN_PROBE_ENDSTOP)
        bool z_probe_endstop = (READ(Z_MIN_PROBE_PIN) != Z_MIN_PROBE_ENDSTOP_INVERTING);
        if (z_probe_endstop)
      #else
        bool z_min_endstop = (READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING);
        if (z_min_endstop)
      #endif
        {

          // Move to the start position to initiate deployment移动到起始位置以启动部署
          destination[X_AXIS] = Z_PROBE_ALLEN_KEY_DEPLOY_1_X;
          destination[Y_AXIS] = Z_PROBE_ALLEN_KEY_DEPLOY_1_Y;
          destination[Z_AXIS] = Z_PROBE_ALLEN_KEY_DEPLOY_1_Z;
          prepare_move_raw(); // this will also set_current_to_destination这也将设置为“当前”到“目标”

          // Move to engage deployment移动到参与部署
          if (Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE != Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE)
            feedrate = Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE;
          if (Z_PROBE_ALLEN_KEY_DEPLOY_2_X != Z_PROBE_ALLEN_KEY_DEPLOY_1_X)
            destination[X_AXIS] = Z_PROBE_ALLEN_KEY_DEPLOY_2_X;
          if (Z_PROBE_ALLEN_KEY_DEPLOY_2_Y != Z_PROBE_ALLEN_KEY_DEPLOY_1_Y)
            destination[Y_AXIS] = Z_PROBE_ALLEN_KEY_DEPLOY_2_Y;
          if (Z_PROBE_ALLEN_KEY_DEPLOY_2_Z != Z_PROBE_ALLEN_KEY_DEPLOY_1_Z)
            destination[Z_AXIS] = Z_PROBE_ALLEN_KEY_DEPLOY_2_Z;
          prepare_move_raw();

          #ifdef Z_PROBE_ALLEN_KEY_DEPLOY_3_X
            if (Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE != Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE)
              feedrate = Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE;

          // Move to trigger deployment
          if (Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE != Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE)
            feedrate = Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE;
          if (Z_PROBE_ALLEN_KEY_DEPLOY_3_X != Z_PROBE_ALLEN_KEY_DEPLOY_2_X)
            destination[X_AXIS] = Z_PROBE_ALLEN_KEY_DEPLOY_3_X;
          if (Z_PROBE_ALLEN_KEY_DEPLOY_3_Y != Z_PROBE_ALLEN_KEY_DEPLOY_2_Y)
            destination[Y_AXIS] = Z_PROBE_ALLEN_KEY_DEPLOY_3_Y;
          if (Z_PROBE_ALLEN_KEY_DEPLOY_3_Z != Z_PROBE_ALLEN_KEY_DEPLOY_2_Z)
            destination[Z_AXIS] = Z_PROBE_ALLEN_KEY_DEPLOY_3_Z;

          prepare_move_raw();

        #endif
      }
 
      // Partially Home X,Y for safety  为安全起见，部分返回x,y
      destination[X_AXIS] = destination[X_AXIS] * 0.75;
      destination[Y_AXIS] = destination[Y_AXIS] * 0.75;
      prepare_move_raw(); // this will also set_current_to_destination

      st_synchronize();

      #if ENABLED(Z_MIN_PROBE_ENDSTOP)
        z_probe_endstop = (READ(Z_MIN_PROBE_PIN) != Z_MIN_PROBE_ENDSTOP_INVERTING);
        if (z_probe_endstop)
      #else
        z_min_endstop = (READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING);
        if (z_min_endstop)
      #endif
        {
          if (IsRunning()) {              //如果(正在运行())
            SERIAL_ERROR_START;           //序列错误开始；
            SERIAL_ERRORLNPGM("Z-Probe failed to engage!");   //串行错误（“z-探针未能启动！”）
            LCD_ALERTMESSAGEPGM("Err: ZPROBE");
          }
          Stop();
        }

    #endif // Z_PROBE_ALLEN_KEY

  }

  static void stow_z_probe(bool doRaise = true) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        print_xyz("stow_z_probe > current_position", current_position);
      }
    #endif

    #if HAS_SERVO_ENDSTOPS   //有伺服端子

      // Retract Z Servo endstop if enabled  //如果启用，收回z伺服端子
      if (servo_endstop_id[Z_AXIS] >= 0) {

        #if Z_RAISE_AFTER_PROBING > 0
          if (doRaise) {
            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (marlin_debug_flags & DEBUG_LEVELING) {
                SERIAL_ECHOPAIR("Raise Z (after) by ", (float)Z_RAISE_AFTER_PROBING);
                SERIAL_EOL;
                SERIAL_ECHO("> SERVO_ENDSTOPS > raise_z_after_probing()");
                SERIAL_EOL;
              }
            #endif
            raise_z_after_probing(); // this also updates current_position  这也更新了当前的位置
            st_synchronize();
          }
        #endif

        // Change the Z servo angle  改变z伺服角度
        servo[servo_endstop_id[Z_AXIS]].move(servo_endstop_angle[Z_AXIS][1]);
      }

    #elif ENABLED(Z_PROBE_ALLEN_KEY)

      // Move up for safety  为了安全起见
      feedrate = Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE;   //进料率=z探针艾伦键积1进料率

      #if Z_RAISE_AFTER_PROBING > 0
        destination[Z_AXIS] = current_position[Z_AXIS] + Z_RAISE_AFTER_PROBING;
        prepare_move_raw(); // this will also set_current_to_destination
      #endif

      // Move to the start position to initiate retraction  移动到起始位置以启动撤回
      destination[X_AXIS] = Z_PROBE_ALLEN_KEY_STOW_1_X;
      destination[Y_AXIS] = Z_PROBE_ALLEN_KEY_STOW_1_Y;
      destination[Z_AXIS] = Z_PROBE_ALLEN_KEY_STOW_1_Z;
      prepare_move_raw();

      // Move the nozzle down to push the Z probe into retracted position  将喷嘴向下移动，将z探针推入缩回位置
      if (Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE != Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE)
        feedrate = Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE;
      if (Z_PROBE_ALLEN_KEY_STOW_2_X != Z_PROBE_ALLEN_KEY_STOW_1_X)
        destination[X_AXIS] = Z_PROBE_ALLEN_KEY_STOW_2_X;
      if (Z_PROBE_ALLEN_KEY_STOW_2_Y != Z_PROBE_ALLEN_KEY_STOW_1_Y)
        destination[Y_AXIS] = Z_PROBE_ALLEN_KEY_STOW_2_Y;
      destination[Z_AXIS] = Z_PROBE_ALLEN_KEY_STOW_2_Z;
      prepare_move_raw();

      // Move up for safety
      if (Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE != Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE)
        feedrate = Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE;
      if (Z_PROBE_ALLEN_KEY_STOW_3_X != Z_PROBE_ALLEN_KEY_STOW_2_X)
        destination[X_AXIS] = Z_PROBE_ALLEN_KEY_STOW_3_X;
      if (Z_PROBE_ALLEN_KEY_STOW_3_Y != Z_PROBE_ALLEN_KEY_STOW_2_Y)
        destination[Y_AXIS] = Z_PROBE_ALLEN_KEY_STOW_3_Y;
      destination[Z_AXIS] = Z_PROBE_ALLEN_KEY_STOW_3_Z;
      prepare_move_raw();

      // Home XY for safety
      feedrate = homing_feedrate[X_AXIS] / 2;  //进料率=自导进料率[x_轴]/2；
      destination[X_AXIS] = 0;   //目的【x_轴】=0；
      destination[Y_AXIS] = 0;
      prepare_move_raw(); // this will also set_current_to_destination

      st_synchronize();

      #if ENABLED(Z_MIN_PROBE_ENDSTOP)
        bool z_probe_endstop = (READ(Z_MIN_PROBE_PIN) != Z_MIN_PROBE_ENDSTOP_INVERTING);
        if (!z_probe_endstop)
      #else
        bool z_min_endstop = (READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING);
        if (!z_min_endstop)
      #endif
        {
          if (IsRunning()) {
            SERIAL_ERROR_START;
            SERIAL_ERRORLNPGM("Z-Probe failed to retract!");
            LCD_ALERTMESSAGEPGM("Err: ZPROBE");
          }
          Stop();
        }
    #endif // Z_PROBE_ALLEN_KEY
  }

  enum ProbeAction {
    ProbeStay          = 0,
    ProbeDeploy        = BIT(0),
    ProbeStow          = BIT(1),
    ProbeDeployAndStow = (ProbeDeploy | ProbeStow)
  };

  // Probe bed height at position (x,y), returns the measured z value  在位置(X,Y)处的探针床高度，返回测量到的Z值
  static float probe_pt(float x, float y, float z_before, ProbeAction probe_action = ProbeDeployAndStow, int verbose_level = 1) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        SERIAL_ECHOLNPGM("probe_pt >>>");
        SERIAL_ECHOPAIR("> ProbeAction:", (unsigned long)probe_action);
        SERIAL_EOL;
        print_xyz("> current_position", current_position);
      }
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        SERIAL_ECHOPAIR("Z Raise to z_before ", z_before);
        SERIAL_EOL;
        SERIAL_ECHOPAIR("> do_blocking_move_to_z ", z_before);
        SERIAL_EOL;
      }
    #endif

    // Move Z up to the z_before height, then move the Z probe to the given XY /将z移至高度前的z_，然后将z探针移至给定的xy
    do_blocking_move_to_z(z_before); // this also updates current_position 停止移动到Z(z_之前)

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        SERIAL_ECHOPAIR("> do_blocking_move_to_xy ", x - X_PROBE_OFFSET_FROM_EXTRUDER);
        SERIAL_ECHOPAIR(", ", y - Y_PROBE_OFFSET_FROM_EXTRUDER);
        SERIAL_EOL;
      }
    #endif

    do_blocking_move_to_xy(x - X_PROBE_OFFSET_FROM_EXTRUDER, y - Y_PROBE_OFFSET_FROM_EXTRUDER); // this also updates current_position

    #if DISABLED(Z_PROBE_SLED) && DISABLED(Z_PROBE_ALLEN_KEY)
      if (probe_action & ProbeDeploy) {
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            SERIAL_ECHOLNPGM("> ProbeDeploy");
          }
        #endif
        deploy_z_probe();
      }
    #endif

    run_z_probe();
    float measured_z = current_position[Z_AXIS];

    #if DISABLED(Z_PROBE_SLED) && DISABLED(Z_PROBE_ALLEN_KEY)
      if (probe_action & ProbeStow) {
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            SERIAL_ECHOLNPGM("> ProbeStow (stow_z_probe will do Z Raise)");
          }
        #endif
        stow_z_probe();
      }
    #endif

    if (verbose_level > 2) {
      SERIAL_PROTOCOLPGM("Bed X: ");
      SERIAL_PROTOCOL_F(x, 3);
      SERIAL_PROTOCOLPGM(" Y: ");
      SERIAL_PROTOCOL_F(y, 3);
      SERIAL_PROTOCOLPGM(" Z: ");
      SERIAL_PROTOCOL_F(measured_z, 3);
      SERIAL_EOL;
    }

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        SERIAL_ECHOLNPGM("<<< probe_pt");
      }
    #endif

    return measured_z;
  }

  #if ENABLED(DELTA)

    /**
     * All DELTA leveling in the Marlin uses NONLINEAR_BED_LEVELING 马林体内的所有三角洲水平均采用非线性床水平
     */

    static void extrapolate_one_point(int x, int y, int xdir, int ydir) {
      if (bed_level[x][y] != 0.0) {
        return;  // Don't overwrite good values.
      }
      float a = 2 * bed_level[x + xdir][y] - bed_level[x + xdir * 2][y]; // Left to right.
      float b = 2 * bed_level[x][y + ydir] - bed_level[x][y + ydir * 2]; // Front to back.
      float c = 2 * bed_level[x + xdir][y + ydir] - bed_level[x + xdir * 2][y + ydir * 2]; // Diagonal.
      float median = c;  // Median is robust (ignores outliers).
      if (a < b) {
        if (b < c) median = b;
        if (c < a) median = a;
      }
      else {  // b <= a
        if (c < b) median = b;
        if (a < c) median = a;
      }
      bed_level[x][y] = median;
    }

    // Fill in the unprobed points (corners of circular print surface)填写未检查的点（圆形打印表面的角）
    // using linear extrapolation, away from the center.//使用线性外推法，远离中心。
    static void extrapolate_unprobed_bed_level() {
      int half = (AUTO_BED_LEVELING_GRID_POINTS - 1) / 2;
      for (int y = 0; y <= half; y++) {
        for (int x = 0; x <= half; x++) {
          if (x + y < 3) continue;
          extrapolate_one_point(half - x, half - y, x > 1 ? +1 : 0, y > 1 ? +1 : 0);
          extrapolate_one_point(half + x, half - y, x > 1 ? -1 : 0, y > 1 ? +1 : 0);
          extrapolate_one_point(half - x, half + y, x > 1 ? +1 : 0, y > 1 ? -1 : 0);
          extrapolate_one_point(half + x, half + y, x > 1 ? -1 : 0, y > 1 ? -1 : 0);
        }
      }
    }

    // Print calibration results for plotting or manual frame adjustment. 打印绘图或手动调整框架的校准结果。
    static void print_bed_level() {
      for (int y = 0; y < AUTO_BED_LEVELING_GRID_POINTS; y++) {
        for (int x = 0; x < AUTO_BED_LEVELING_GRID_POINTS; x++) {
          SERIAL_PROTOCOL_F(bed_level[x][y], 2);
          SERIAL_PROTOCOLCHAR(' ');
        }
        SERIAL_EOL;
      }
    }

    // Reset calibration results to zero.  将校准结果重置为零
    void reset_bed_level() {                    //重置床层()
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
//          SERIAL_ECHOLNPGM("reset_bed_level");   /串连式电床（“重置床层”）；
        }
      #endif
      for (int y = 0; y < AUTO_BED_LEVELING_GRID_POINTS; y++) {
        for (int x = 0; x < AUTO_BED_LEVELING_GRID_POINTS; x++) {
          bed_level[x][y] = 0.0;
        }
      }
    }

  #endif // DELTA

  #if HAS_SERVO_ENDSTOPS && DISABLED(Z_PROBE_SLED)  //有伺服端子&禁用（z探针雪橇

    void raise_z_for_servo() {
      float zpos = current_position[Z_AXIS], z_dest = Z_RAISE_BEFORE_PROBING;
      z_dest += axis_known_position[Z_AXIS] ? zprobe_zoffset : zpos;
      if (zpos < z_dest) do_blocking_move_to_z(z_dest); // also updates current_position
    }

  #endif

#endif // AUTO_BED_LEVELING_FEATURE   自动床平整功能


#if ENABLED(Z_PROBE_SLED)

  #ifndef SLED_DOCKING_OFFSET       ///雪橇对接偏移
    #define SLED_DOCKING_OFFSET 0   //定义雪橇对接偏移量0
  #endif

  /**
   * Method to dock/undock a sled designed by Charles Bell.  Charles Bell设计的雪橇停靠/卸载方法。
   * 
   * dock[in]     If true, move to MAX_X and engage the electromagnet  停靠[中]如果为真，移动到max并与电磁铁接合
   * offset[in]   The additional distance to move to adjust docking location  抵消为调整对接位置而移动的额外距离
   */
  static void dock_sled(bool dock, int offset = 0) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        SERIAL_ECHOPAIR("dock_sled", dock);
        SERIAL_EOL;
      }
    #endif
    if (!axis_known_position[X_AXIS] || !axis_known_position[Y_AXIS]) {
      LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
      return;
    }

    float oldXpos = current_position[X_AXIS]; // save x position  保存X位置
    if (dock) {
      #if Z_RAISE_AFTER_PROBING > 0
        raise_z_after_probing(); // raise Z  升起z
      #endif
      do_blocking_move_to_x(X_MAX_POS + SLED_DOCKING_OFFSET + offset - 1);  // Dock sled a bit closer to ensure proper capturing 船坞雪橇靠近一点，以确保适当的捕捉
      digitalWrite(SLED_PIN, LOW); // turn off magnet关掉磁铁
    }
    else {
      float z_loc = current_position[Z_AXIS];
      if (z_loc < Z_RAISE_BEFORE_PROBING + 5) z_loc = Z_RAISE_BEFORE_PROBING;
      do_blocking_move_to(X_MAX_POS + SLED_DOCKING_OFFSET + offset, current_position[Y_AXIS], z_loc); // this also updates current_position
      digitalWrite(SLED_PIN, HIGH); // turn on magnet
    }
    do_blocking_move_to_x(oldXpos); // return to position before docking停靠前返回位置
  }

#endif // Z_PROBE_SLED



/**
 * Home an individual axis   独立轴的回零
 */

#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

static void homeaxis(AxisEnum axis) {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (marlin_debug_flags & DEBUG_LEVELING) {
      SERIAL_ECHOPAIR(">>> homeaxis(", (unsigned long)axis);
      SERIAL_CHAR(')');
      SERIAL_EOL;
    }
  #endif
  #define HOMEAXIS_DO(LETTER) \
    ((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))

  if (axis == X_AXIS ? HOMEAXIS_DO(X) : axis == Y_AXIS ? HOMEAXIS_DO(Y) : axis == Z_AXIS ? HOMEAXIS_DO(Z) : 0) {

    int axis_home_dir =
      #if ENABLED(DUAL_X_CARRIAGE)
        (axis == X_AXIS) ? x_home_dir(active_extruder) :
      #endif
      home_dir(axis);

    // Set the axis position as setup for the move  将轴位置设置为移动的设置
    current_position[axis] = 0;
    sync_plan_position();

    #if ENABLED(Z_PROBE_SLED)
      // Get Probe
      if (axis == Z_AXIS) {
        if (axis_home_dir < 0) dock_sled(false);
      }
    #endif

    #if SERVO_LEVELING && DISABLED(Z_PROBE_SLED)

      // Deploy a Z probe if there is one, and homing towards the bed  如果有探测器，则部署一个z探测器，并返回到床上
      if (axis == Z_AXIS) {
        if (axis_home_dir < 0) deploy_z_probe();
      }

    #endif

    #if HAS_SERVO_ENDSTOPS
      // Engage Servo endstop if enabled  启用时启动伺服端子
      if (axis != Z_AXIS && servo_endstop_id[axis] >= 0)
        servo[servo_endstop_id[axis]].move(servo_endstop_angle[axis][0]);
    #endif

    // Set a flag for Z motor locking   为z电机锁定设置标志
    #if ENABLED(Z_DUAL_ENDSTOPS)
      if (axis == Z_AXIS) In_Homing_Process(true);
    #endif

    // Move towards the endstop until an endstop is triggered  向终点移动，直到终点被触发
    destination[axis] = 1.5 * max_length(axis) * axis_home_dir;
    feedrate = homing_feedrate[axis];
    line_to_destination();
    st_synchronize();

    // Set the axis position as setup for the move  将轴位置设置为移动的设置
    current_position[axis] = 0;
    sync_plan_position();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        SERIAL_ECHOLNPGM("> enable_endstops(false)");
      }
    #endif
    enable_endstops(false); // Disable endstops while moving away   移动时禁用端子

    // Move away from the endstop by the axis HOME_BUMP_MM   从端子上的轴向外移动
    destination[axis] = -home_bump_mm(axis) * axis_home_dir;
    line_to_destination();
    st_synchronize();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        SERIAL_ECHOLNPGM("> enable_endstops(true)");
      }
    #endif
    enable_endstops(true); // Enable endstops for next homing move  启用下一步归航的端子

    // Slow down the feedrate for the next move  放慢下一步的输入速度
    set_homing_bump_feedrate(axis);

    // Move slowly towards the endstop until triggered  向终点缓慢移动直到触发
    destination[axis] = 2 * home_bump_mm(axis) * axis_home_dir;
    line_to_destination();
    st_synchronize();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        print_xyz("> TRIGGER ENDSTOP > current_position", current_position);
      }
    #endif

    #if ENABLED(Z_DUAL_ENDSTOPS)
      if (axis == Z_AXIS) {
        float adj = fabs(z_endstop_adj);
        bool lockZ1;
        if (axis_home_dir > 0) {
          adj = -adj;
          lockZ1 = (z_endstop_adj > 0);
        }
        else
          lockZ1 = (z_endstop_adj < 0);

        if (lockZ1) Lock_z_motor(true); else Lock_z2_motor(true);
        sync_plan_position();

        // Move to the adjusted endstop height
        feedrate = homing_feedrate[axis];
        destination[Z_AXIS] = adj;
        line_to_destination();
        st_synchronize();

        if (lockZ1) Lock_z_motor(false); else Lock_z2_motor(false);
        In_Homing_Process(false);
      } // Z_AXIS
    #endif

    #if ENABLED(DELTA)
      // retrace by the amount specified in endstop_adj  按终止_adj中规定的数额重新计算
      if (endstop_adj[axis] * axis_home_dir < 0) {
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            SERIAL_ECHOLNPGM("> enable_endstops(false)");
          }
        #endif
        enable_endstops(false); // Disable endstops while moving away  移动时禁用端子
        sync_plan_position();
        destination[axis] = endstop_adj[axis];
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            SERIAL_ECHOPAIR("> endstop_adj = ", endstop_adj[axis]);
            print_xyz(" > destination", destination);
          }
        #endif
        line_to_destination();
        st_synchronize();
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            SERIAL_ECHOLNPGM("> enable_endstops(true)");
          }
        #endif
        enable_endstops(true); // Enable endstops for next homing move
      }
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        else {
          if (marlin_debug_flags & DEBUG_LEVELING) {
            SERIAL_ECHOPAIR("> endstop_adj * axis_home_dir = ", endstop_adj[axis] * axis_home_dir);
            SERIAL_EOL;
          }
        }
      #endif
    #endif

    // Set the axis position to its home position (plus home offsets) 将轴位置设置为其家位置（加上家抵消）
    set_axis_is_at_home(axis);
    sync_plan_position();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        print_xyz("> AFTER set_axis_is_at_home > current_position", current_position);
      }
    #endif

    destination[axis] = current_position[axis];
    feedrate = 0.0;
    endstops_hit_on_purpose(); // clear endstop hit flags  清除终端命中标志
    axis_known_position[axis] = true;

    #if ENABLED(Z_PROBE_SLED)
      // bring Z probe back
      if (axis == Z_AXIS) {
        if (axis_home_dir < 0) dock_sled(true);
      }
    #endif

    #if SERVO_LEVELING && DISABLED(Z_PROBE_SLED)

      // Deploy a Z probe if there is one, and homing towards the bed如果有探测器，则部署一个z探测器，并返回到床上
      if (axis == Z_AXIS) {
        if (axis_home_dir < 0) {
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (marlin_debug_flags & DEBUG_LEVELING) {
              SERIAL_ECHOLNPGM("> SERVO_LEVELING > stow_z_probe");
            }
          #endif
          stow_z_probe();
        }
      }
      else

    #endif

    {
      #if HAS_SERVO_ENDSTOPS
        // Retract Servo endstop if enabled  如果启用，收回伺服端子
        if (servo_endstop_id[axis] >= 0) {
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (marlin_debug_flags & DEBUG_LEVELING) {
              SERIAL_ECHOLNPGM("> SERVO_ENDSTOPS > Stow with servo.move()");
            }
          #endif
          servo[servo_endstop_id[axis]].move(servo_endstop_angle[axis][1]);
        }
      #endif
    }

  }

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (marlin_debug_flags & DEBUG_LEVELING) {
      SERIAL_ECHOPAIR("<<< homeaxis(", (unsigned long)axis);
      SERIAL_CHAR(')');
      SERIAL_EOL;
    }
  #endif
}

#if ENABLED(FWRETRACT)

  void retract(bool retracting, bool swapping = false) {

    if (retracting == retracted[active_extruder]) return;

    float oldFeedrate = feedrate;

    set_destination_to_current();

    if (retracting) {

      feedrate = retract_feedrate * 60;
      current_position[E_AXIS] += (swapping ? retract_length_swap : retract_length) / volumetric_multiplier[active_extruder];
      plan_set_e_position(current_position[E_AXIS]);
      prepare_move();

      if (retract_zlift > 0.01) {
        current_position[Z_AXIS] -= retract_zlift;
        #if ENABLED(DELTA)
          sync_plan_position_delta();
        #else
          sync_plan_position();
        #endif
        prepare_move();
      }
    }
    else {

      if (retract_zlift > 0.01) {
        current_position[Z_AXIS] += retract_zlift;
        #if ENABLED(DELTA)
          sync_plan_position_delta();
        #else
          sync_plan_position();
        #endif
        //prepare_move();
      }

      feedrate = retract_recover_feedrate * 60;
      float move_e = swapping ? retract_length_swap + retract_recover_length_swap : retract_length + retract_recover_length;
      current_position[E_AXIS] -= move_e / volumetric_multiplier[active_extruder];
      plan_set_e_position(current_position[E_AXIS]);
      prepare_move();
    }

    feedrate = oldFeedrate;
    retracted[active_extruder] = retracting;

  } // retract()

#endif // FWRETRACT

/**
 *
 * G-Code Handler functions  *g-代码处理函数
 *
 */

/**
 * Set XYZE destination and feedrate from the current GCode command  从当前gcode命令中设置木质化目标和输入速率
 *
 *  - Set destination from included axis codes  从包含的轴码设定目的地
 *  - Set to current for missing axis codes  为丢失的轴码设置为当前
 *  - Set the feedrate, if included  如果包括在内，设定输入率
 */
void gcode_get_destination() {
  for (int i = 0; i < NUM_AXIS; i++) {
    if (code_seen(axis_codes[i]))
      destination[i] = code_value() + (axis_relative_modes[i] || relative_mode ? current_position[i] : 0);
    else
      destination[i] = current_position[i];
  }
  if (code_seen('F')) {
    float next_feedrate = code_value();
    if (next_feedrate > 0.0) feedrate = next_feedrate;
  }
}

void unknown_command_error() {
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
  SERIAL_ECHO(current_command);
  SERIAL_ECHOPGM("\"\n");
}

/**
 * G0, G1: Coordinated movement of X Y Z E axes
 */
inline void gcode_G0_G1() {
  if (IsRunning()) {
    gcode_get_destination(); // For X Y Z E F

    #if ENABLED(FWRETRACT)

      if (autoretract_enabled && !(code_seen('X') || code_seen('Y') || code_seen('Z')) && code_seen('E')) {
        float echange = destination[E_AXIS] - current_position[E_AXIS];
        // Is this move an attempt to retract or recover?  这是试图收回或恢复？
        if ((echange < -MIN_RETRACT && !retracted[active_extruder]) || (echange > MIN_RETRACT && retracted[active_extruder])) {
          current_position[E_AXIS] = destination[E_AXIS]; // hide the slicer-generated retract/recover from calculations  隐藏切片生成的缩回/从计算中恢复
          plan_set_e_position(current_position[E_AXIS]);  // AND from the planner 从策划者那里
          retract(!retracted[active_extruder]);
          return;
        }
      }

    #endif //FWRETRACT

    prepare_move();
  }
}

/**
 * G2: Clockwise Arc  顺时针弧线
 * G3: Counterclockwise Arc  逆时针弧线
 */
inline void gcode_G2_G3(bool clockwise) {
  if (IsRunning()) {

    #if ENABLED(SF_ARC_FIX)
      bool relative_mode_backup = relative_mode;
      relative_mode = true;
    #endif

    gcode_get_destination();

    #if ENABLED(SF_ARC_FIX)
      relative_mode = relative_mode_backup;
    #endif

    // Center of arc as offset from current_position  圆心，从当前位置偏移
    float arc_offset[2] = {
      code_seen('I') ? code_value() : 0,
      code_seen('J') ? code_value() : 0
    };

    // Send an arc to the planner  向策划方发送一个弧线
    plan_arc(destination, arc_offset, clockwise);

    refresh_cmd_timeout();
  }
}

/**
 * G4: Dwell S<seconds> or P<milliseconds>
 */
inline void gcode_G4() {
  millis_t codenum = 0;

  if (code_seen('P')) codenum = code_value_long(); // milliseconds to wait  要等待的毫秒
  if (code_seen('S')) codenum = code_value() * 1000; // seconds to wait

  st_synchronize();
  refresh_cmd_timeout();
  codenum += previous_cmd_ms;  // keep track of when we started waiting  记录下我们什么时候开始等待

  if (!lcd_hasstatus()) LCD_MESSAGEPGM(MSG_DWELL);

  while (millis() < codenum) idle();
}

#if ENABLED(FWRETRACT)

  /**
   * G10 - Retract filament according to settings of M207  根据m207的设置缩回灯丝
   * G11 - Recover filament according to settings of M208  根据m208的设定回收灯丝
   */
  inline void gcode_G10_G11(bool doRetract=false) {
    #if EXTRUDERS > 1
      if (doRetract) {
        retracted_swap[active_extruder] = (code_seen('S') && code_value_short() == 1); // checks for swap retract argument 检查交换缩回参数
      }
    #endif
    retract(doRetract
     #if EXTRUDERS > 1
      , retracted_swap[active_extruder]
     #endif
    );
  }

#endif //FWRETRACT

/**
 * G28: Home all axes according to settings  根据设置置回所有轴
 *
 * Parameters
 *
 *  None  Home to all axes with no parameters.
 *        With QUICK_HOME enabled XY will home together, then Z.
 *
 * Cartesian parameters
 *
 *  X   Home to the X endstop
 *  Y   Home to the Y endstop
 *  Z   Home to the Z endstop
 *
 */
inline void gcode_G28() {

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (marlin_debug_flags & DEBUG_LEVELING) {
      SERIAL_ECHOLNPGM("gcode_G28 >>>");
    }
  #endif

  // Wait for planner moves to finish!  等待计划者的动作完成
  st_synchronize();

  // For auto bed leveling, clear the level matrix  为自动床平整，清除水平矩阵
  #if ENABLED(AUTO_BED_LEVELING_FEATURE)
    plan_bed_level_matrix.set_to_identity();
    #if ENABLED(DELTA)
      reset_bed_level();
    #endif
  #endif

  // For manual bed leveling deactivate the matrix temporarily  对于手动床平整暂时停用矩阵
  #if ENABLED(MESH_BED_LEVELING)
    uint8_t mbl_was_active = mbl.active;
    mbl.active = 0;
  #endif

  setup_for_endstop_move();

  set_destination_to_current();

  feedrate = 0.0;

  #if ENABLED(DELTA)
    // A delta can only safely home all axis at the same time   a德尔塔只能同时安全地返回所有轴
    // all axis have to home at the same time                    所有轴必须同时回家

    // Pretend the current position is 0,0,0  假设当前位置为0,0
    for (int i = X_AXIS; i <= Z_AXIS; i++) current_position[i] = 0;
    sync_plan_position();

    // Move all carriages up together until the first endstop is hit.  将所有车厢一起移动，直到第一个终点被击中。
    for (int i = X_AXIS; i <= Z_AXIS; i++) destination[i] = 3 * Z_MAX_LENGTH;
    feedrate = 1.732 * homing_feedrate[X_AXIS];
    line_to_destination();
    st_synchronize();
    endstops_hit_on_purpose(); // clear endstop hit flags  清除终端命中标志
 
    // Destination reached
    for (int i = X_AXIS; i <= Z_AXIS; i++) current_position[i] = destination[i];

    // take care of back off and rehome now we are all at the top  照顾好后退，回家，现在我们都是最棒的
    HOMEAXIS(X);
    HOMEAXIS(Y);
    HOMEAXIS(Z);

    sync_plan_position_delta();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        print_xyz("(DELTA) > current_position", current_position);
      }
    #endif

  #else // NOT DELTA

    bool  homeX = code_seen(axis_codes[X_AXIS]),
          homeY = code_seen(axis_codes[Y_AXIS]),
          homeZ = code_seen(axis_codes[Z_AXIS]);

    home_all_axis = (!homeX && !homeY && !homeZ) || (homeX && homeY && homeZ);

    if (home_all_axis || homeZ) {

      #if Z_HOME_DIR > 0  // If homing away from BED do Z first 如果离家出走先做

        HOMEAXIS(Z);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            print_xyz("> HOMEAXIS(Z) > current_position", current_position);
          }
        #endif

      #elif DISABLED(Z_SAFE_HOMING) && defined(Z_RAISE_BEFORE_HOMING) && Z_RAISE_BEFORE_HOMING > 0

        // Raise Z before homing any other axes   在找到任何其它轴之前提高z
        // (Does this need to be "negative home direction?" Why not just use Z_RAISE_BEFORE_HOMING?)
        //（这需要是“负的回家方向吗？”为什么不在归航之前先使用z_rev_？
        destination[Z_AXIS] = -Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            SERIAL_ECHOPAIR("Raise Z (before homing) by ", (float)Z_RAISE_BEFORE_HOMING);
            SERIAL_EOL;
            print_xyz("> (home_all_axis || homeZ) > destination", destination);
          }
        #endif
        feedrate = max_feedrate[Z_AXIS] * 60;
        line_to_destination();
        st_synchronize();

      #endif

    } // home_all_axis || homeZ

    #if ENABLED(QUICK_HOME)

      if (home_all_axis || (homeX && homeY)) {  // First diagonal move 第一次对角线移动

        current_position[X_AXIS] = current_position[Y_AXIS] = 0;

        #if ENABLED(DUAL_X_CARRIAGE)
          int x_axis_home_dir = x_home_dir(active_extruder);
          extruder_duplication_enabled = false;
        #else
          int x_axis_home_dir = home_dir(X_AXIS);
        #endif

        sync_plan_position();

        float mlx = max_length(X_AXIS), mly = max_length(Y_AXIS),
              mlratio = mlx > mly ? mly / mlx : mlx / mly;

        destination[X_AXIS] = 1.5 * mlx * x_axis_home_dir;
        destination[Y_AXIS] = 1.5 * mly * home_dir(Y_AXIS);
        feedrate = min(homing_feedrate[X_AXIS], homing_feedrate[Y_AXIS]) * sqrt(mlratio * mlratio + 1);
        line_to_destination();
        st_synchronize();

        set_axis_is_at_home(X_AXIS);
        set_axis_is_at_home(Y_AXIS);
        sync_plan_position();

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            print_xyz("> QUICK_HOME > current_position 1", current_position);
          }
        #endif

        destination[X_AXIS] = current_position[X_AXIS];
        destination[Y_AXIS] = current_position[Y_AXIS];
        line_to_destination();
        feedrate = 0.0;
        st_synchronize();
        endstops_hit_on_purpose(); // clear endstop hit flags

        current_position[X_AXIS] = destination[X_AXIS];
        current_position[Y_AXIS] = destination[Y_AXIS];
        #if DISABLED(SCARA)
          current_position[Z_AXIS] = destination[Z_AXIS];
        #endif

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            print_xyz("> QUICK_HOME > current_position 2", current_position);
          }
        #endif
      }

    #endif // QUICK_HOME

    #if ENABLED(HOME_Y_BEFORE_X)
      // Home Y
      if (home_all_axis || homeY) HOMEAXIS(Y);
    #endif

    // Home X
    if (home_all_axis || homeX) {
      #if ENABLED(DUAL_X_CARRIAGE)
        int tmp_extruder = active_extruder;
        extruder_duplication_enabled = false;
        active_extruder = !active_extruder;
        HOMEAXIS(X);
        inactive_extruder_x_pos = current_position[X_AXIS];
        active_extruder = tmp_extruder;
        HOMEAXIS(X);
        // reset state used by the different modes  不同模式使用的重置状态
        memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
        delayed_move_time = 0;
        active_extruder_parked = true;
      #else
        HOMEAXIS(X);
      #endif
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
          print_xyz("> homeX", current_position);
        }
      #endif
    }

    #if DISABLED(HOME_Y_BEFORE_X)
      // Home Y
      if (home_all_axis || homeY) {
        HOMEAXIS(Y);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            print_xyz("> homeY", current_position);
          }
        #endif
      }
    #endif

    // Home Z last if homing towards the bed  如果回到床上
    #if Z_HOME_DIR < 0

      if (home_all_axis || homeZ) {

        #if ENABLED(Z_SAFE_HOMING)

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (marlin_debug_flags & DEBUG_LEVELING) {
              SERIAL_ECHOLNPGM("> Z_SAFE_HOMING >>>");
            }
          #endif

          if (home_all_axis) {

            current_position[Z_AXIS] = 0;
            sync_plan_position();

            //
            // Set the Z probe (or just the nozzle) destination to the safe homing point
            //将z探针（或仅仅是喷嘴）的目标设置为安全的归航点
            // NOTE: If current_position[X_AXIS] or current_position[Y_AXIS] were set above
            // then this may not work as expected.注意：如果当前位置[x_轴]或当前位置[y_轴]设置在上面
             //那么这可能不像预期的那样有效
            destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT - X_PROBE_OFFSET_FROM_EXTRUDER);
            destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT - Y_PROBE_OFFSET_FROM_EXTRUDER);
            destination[Z_AXIS] = -Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS);    // Set destination away from bed远离床上的目的地
            feedrate = XY_TRAVEL_SPEED;

            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (marlin_debug_flags & DEBUG_LEVELING) {
                SERIAL_ECHOPAIR("Raise Z (before homing) by ", (float)Z_RAISE_BEFORE_HOMING);
                SERIAL_EOL;
                print_xyz("> home_all_axis > current_position", current_position);
                print_xyz("> home_all_axis > destination", destination);
              }
            #endif

            // This could potentially move X, Y, Z all together  有可能把x,y,z一起移动
            line_to_destination();
            st_synchronize();

            // Set current X, Y is the Z_SAFE_HOMING_POINT minus PROBE_OFFSET_FROM_EXTRUDER
            current_position[X_AXIS] = destination[X_AXIS];
            current_position[Y_AXIS] = destination[Y_AXIS];

            // Home the Z axis
            HOMEAXIS(Z);
          }

          else if (homeZ) { // Don't need to Home Z twice  不需要回家两次

            // Let's see if X and Y are homed
            if (axis_known_position[X_AXIS] && axis_known_position[Y_AXIS]) {

              // Make sure the Z probe is within the physical limits  确保z探针在物理限度内
              // NOTE: This doesn't necessarily ensure the Z probe is also within the bed!注意：这并不一定确保z探测器也在床内！
              float cpx = current_position[X_AXIS], cpy = current_position[Y_AXIS];
              if (   cpx >= X_MIN_POS - X_PROBE_OFFSET_FROM_EXTRUDER
                  && cpx <= X_MAX_POS - X_PROBE_OFFSET_FROM_EXTRUDER
                  && cpy >= Y_MIN_POS - Y_PROBE_OFFSET_FROM_EXTRUDER
                  && cpy <= Y_MAX_POS - Y_PROBE_OFFSET_FROM_EXTRUDER) {
                // Set the plan current position to X, Y, 0  将计划当前位置设为x,y,0
                current_position[Z_AXIS] = 0;
                plan_set_position(cpx, cpy, 0, current_position[E_AXIS]); // = sync_plan_position

                // Set Z destination away from bed and raise the axis 将目标从床上移开并将轴抬高
                // NOTE: This should always just be Z_RAISE_BEFORE_HOMING unless...??? 注意：除非…？？？否则在归航之前应该一直是z_葡萄干
                destination[Z_AXIS] = -Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS);
                feedrate = max_feedrate[Z_AXIS] * 60;  // feedrate (mm/m) = max_feedrate (mm/s)

                #if ENABLED(DEBUG_LEVELING_FEATURE)
                  if (marlin_debug_flags & DEBUG_LEVELING) {
                    SERIAL_ECHOPAIR("Raise Z (before homing) by ", (float)Z_RAISE_BEFORE_HOMING);
                    SERIAL_EOL;
                    print_xyz("> homeZ > current_position", current_position);
                    print_xyz("> homeZ > destination", destination);
                  }
                #endif

                line_to_destination();
                st_synchronize();

                // Home the Z axis置Z轴为家
                HOMEAXIS(Z);
              }
              else {
                LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
                SERIAL_ECHO_START;
                SERIAL_ECHOLNPGM(MSG_ZPROBE_OUT);
              }
            }
            else {
              LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
              SERIAL_ECHO_START;
              SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
            }

          } // !home_all_axes && homeZ

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (marlin_debug_flags & DEBUG_LEVELING) {
              SERIAL_ECHOLNPGM("<<< Z_SAFE_HOMING");
            }
          #endif

        #else // !Z_SAFE_HOMING

          HOMEAXIS(Z);

        #endif // !Z_SAFE_HOMING

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            print_xyz("> (home_all_axis || homeZ) > final", current_position);
          }
        #endif

      } // home_all_axis || homeZ

    #endif // Z_HOME_DIR < 0

    sync_plan_position();

  #endif // else DELTA

  #if ENABLED(SCARA)
    sync_plan_position_delta();
  #endif

  #if ENABLED(ENDSTOPS_ONLY_FOR_HOMING)
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        SERIAL_ECHOLNPGM("ENDSTOPS_ONLY_FOR_HOMING enable_endstops(false)");
      }
    #endif
    enable_endstops(false);
  #endif

  // For manual leveling move back to 0,0  用于手动水平移动到0
  #if ENABLED(MESH_BED_LEVELING)
    if (mbl_was_active) {
      current_position[X_AXIS] = mbl.get_x(0);
      current_position[Y_AXIS] = mbl.get_y(0);
      set_destination_to_current();
      feedrate = homing_feedrate[X_AXIS];
      line_to_destination();
      st_synchronize();
      current_position[Z_AXIS] = MESH_HOME_SEARCH_Z;
      sync_plan_position();
      mbl.active = 1;
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
          print_xyz("mbl_was_active > current_position", current_position);
        }
      #endif
    }
  #endif

  feedrate = saved_feedrate;
  feedrate_multiplier = saved_feedrate_multiplier;
  refresh_cmd_timeout();
  endstops_hit_on_purpose(); // clear endstop hit flags

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (marlin_debug_flags & DEBUG_LEVELING) {
      SERIAL_ECHOLNPGM("<<< gcode_G28");
    }
  #endif

}

#if ENABLED(MESH_BED_LEVELING)

  enum MeshLevelingState { MeshReport, MeshStart, MeshNext, MeshSet };

  /**
   * G29: Mesh-based Z probe, probes a grid and produces a  基于网格的z探测器，探测网格并产生
   *      mesh to compensate for variable bed height  用于补偿可变床层高度的网格
   *
   * Parameters With MESH_BED_LEVELING:  *具有网格级别的参数：
   *
   *  S0              Produce a mesh report        s0生成网格报告
   *  S1              Start probing mesh points    开始探测网格点
   *  S2              Probe the next mesh point    s2探测下一个网格点
   *  S3 Xn Yn Zn.nn  Manually modify a single point
   *
   * The S0 report the points as below    S0报告以下各点
   *
   *  +----> X-axis
   *  |
   *  |
   *  v Y-axis
   *
   */
  inline void gcode_G29() {

    static int probe_point = -1;
    MeshLevelingState state = code_seen('S') ? (MeshLevelingState)code_value_short() : MeshReport;
    if (state < 0 || state > 3) {
      SERIAL_PROTOCOLLNPGM("S out of range (0-3).");
      return;
    }

    int ix, iy;
    float z;

    switch (state) {
      case MeshReport:
        if (mbl.active) {
          SERIAL_PROTOCOLPGM("Num X,Y: ");
          SERIAL_PROTOCOL(MESH_NUM_X_POINTS);
          SERIAL_PROTOCOLCHAR(',');
          SERIAL_PROTOCOL(MESH_NUM_Y_POINTS);
          SERIAL_PROTOCOLPGM("\nZ search height: ");
          SERIAL_PROTOCOL(MESH_HOME_SEARCH_Z);
          SERIAL_PROTOCOLLNPGM("\nMeasured points:");
          for (int y = 0; y < MESH_NUM_Y_POINTS; y++) {
            for (int x = 0; x < MESH_NUM_X_POINTS; x++) {
              SERIAL_PROTOCOLPGM("  ");
              SERIAL_PROTOCOL_F(mbl.z_values[y][x], 5);
            }
            SERIAL_EOL;
          }
        }
        else
          SERIAL_PROTOCOLLNPGM("Mesh bed leveling not active.");   //连续原型（“网床平整不活动”）
        break;

      case MeshStart:
        mbl.reset();
        probe_point = 0;
        enqueuecommands_P(PSTR("G28\nG29 S2"));
        break;

      case MeshNext:
        if (probe_point < 0) {
          SERIAL_PROTOCOLLNPGM("Start mesh probing with \"G29 S1\" first.");
          return;
        }
        if (probe_point == 0) {
          // Set Z to a positive value before recording the first Z.  在记录第一个z之前，将z设置为正值。
          current_position[Z_AXIS] = MESH_HOME_SEARCH_Z;
          sync_plan_position();
        }
        else {
          // For others, save the Z of the previous point, then raise Z again.  对其他人来说，保存上一点的z，然后再提高z。
          ix = (probe_point - 1) % MESH_NUM_X_POINTS;
          iy = (probe_point - 1) / MESH_NUM_X_POINTS;
          if (iy & 1) ix = (MESH_NUM_X_POINTS - 1) - ix; // zig-zag
          mbl.set_z(ix, iy, current_position[Z_AXIS]);
          current_position[Z_AXIS] = MESH_HOME_SEARCH_Z;
          plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[X_AXIS] / 60, active_extruder);
          st_synchronize();
        }
        // Is there another point to sample? Move there.
        if (probe_point < MESH_NUM_X_POINTS * MESH_NUM_Y_POINTS) {
          ix = probe_point % MESH_NUM_X_POINTS;
          iy = probe_point / MESH_NUM_X_POINTS;
          if (iy & 1) ix = (MESH_NUM_X_POINTS - 1) - ix; // zig-zag
          current_position[X_AXIS] = mbl.get_x(ix);
          current_position[Y_AXIS] = mbl.get_y(iy);
          plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[X_AXIS] / 60, active_extruder);
          st_synchronize();
          probe_point++;
        }
        else {
          // After recording the last point, activate the mbl and home  在记录了最后一点后，激活MBL并返回
          SERIAL_PROTOCOLLNPGM("Mesh probing done.");
          probe_point = -1;
          mbl.active = 1;
          enqueuecommands_P(PSTR("G28"));
        }
        break;

      case MeshSet:
        if (code_seen('X')) {
          ix = code_value_long() - 1;
          if (ix < 0 || ix >= MESH_NUM_X_POINTS) {
            SERIAL_PROTOCOLPGM("X out of range (1-" STRINGIFY(MESH_NUM_X_POINTS) ").\n");
            return;
          }
        }
        else {
          SERIAL_PROTOCOLPGM("X not entered.\n");
          return;
        }
        if (code_seen('Y')) {
          iy = code_value_long() - 1;
          if (iy < 0 || iy >= MESH_NUM_Y_POINTS) {
            SERIAL_PROTOCOLPGM("Y out of range (1-" STRINGIFY(MESH_NUM_Y_POINTS) ").\n");
            return;
          }
        }
        else {
          SERIAL_PROTOCOLPGM("Y not entered.\n");
          return;
        }
        if (code_seen('Z')) {
          z = code_value();
        }
        else {
          SERIAL_PROTOCOLPGM("Z not entered.\n");
          return;
        }
        mbl.z_values[iy][ix] = z;

    } // switch(state)
  }

#elif ENABLED(AUTO_BED_LEVELING_FEATURE)

  void out_of_range_error(const char* p_edge) {
    SERIAL_PROTOCOLPGM("?Probe ");
    serialprintPGM(p_edge);
    SERIAL_PROTOCOLLNPGM(" position out of range.");
  }

  /**
   * G29: Detailed Z probe, probes the bed at 3 or more points.
   *      Will fail if the printer has not been homed with G28.
   *详细的z探针，在3个或更多点探测床。如果打印机没有被g28定位，则会失败。
   * Enhanced G29 Auto Bed Leveling Probe Routine 增强的g 29自动平底探针程序
   *
   * Parameters With AUTO_BED_LEVELING_GRID:  参数为自定义网格
   *
   *  P  Set the size of the grid that will be probed (P x P points).p设定要探测的网格大小（p X p点）
   *     Not supported by non-linear delta printer bed leveling.  不支持非线性德尔塔打印机床水平。
   *     Example: "G29 P4"
   *
   *  S  Set the XY travel speed between probe points (in mm/min)  设探测点之间的xy移动速度
   *
   *  D  Dry-Run mode. Just evaluate the bed Topology - Don't apply  模拟模式。只是评估床的拓扑结构-不适用
   *     or clean the rotation Matrix. Useful to check the topology 或清洁旋转矩阵。用于检查拓扑
   *     after a first run of G29.  在g 29的第一次运行之后。
   *
   *  V  Set the verbose level (0-4). Example: "G29 V3"
   *
   *  T  Generate a Bed Topology Report. Example: "G29 P5 T" for a detailed report.不会生成床拓扑报告。例如：“g 29 p5t”作为详细报告
   *     This is useful for manual bed leveling and finding flaws in the bed (to 这对于手工平整床和发现床上的缺陷很有用。
   *     assist with part placement).协助零件摆放）
   *     Not supported by non-linear delta printer bed leveling. 不支持非线性德尔塔打印机床水平。
   *
   *  F  Set the Front limit of the probing grid  设置探测网格的前缘
   *  B  Set the Back limit of the probing grid   b设置探测网格的背限
   *  L  Set the Left limit of the probing grid   我设定了探测网格的左限
   *  R  Set the Right limit of the probing grid  为探测网格设置正确的限制
   *
   * Global Parameters: 全球参数：
   *
   * E/e By default G29 will engage the Z probe, test the bed, then disengage.默认情况下，g 29将与z探针接合，测试床，然后分离。
   *     Include "E" to engage/disengage the Z probe for each sample.  包括"e"来装载/拆卸每个样品的z探针。
   *     There's no extra effect if you have a fixed Z probe. 如果你有一个固定的z探针，就不会有额外的影响。
   *     Usage: "G29 E" or "G29 e"
   *
   */
  inline void gcode_G29() {

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        SERIAL_ECHOLNPGM("gcode_G29 >>>");
      }
    #endif

    // Don't allow auto-leveling without homing first  不允许在没有先归航的情况下自动调平
    if (!axis_known_position[X_AXIS] || !axis_known_position[Y_AXIS]) {
      LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
      return;
    }

    int verbose_level = code_seen('V') ? code_value_short() : 1;
    if (verbose_level < 0 || verbose_level > 4) {
      SERIAL_ECHOLNPGM("?(V)erbose Level is implausible (0-4).");
      return;
    }

    bool dryrun = code_seen('D'),
         deploy_probe_for_each_reading = code_seen('E');

    #if ENABLED(AUTO_BED_LEVELING_GRID)

      #if DISABLED(DELTA)
        bool do_topography_map = verbose_level > 2 || code_seen('T');
      #endif

      if (verbose_level > 0) {
        SERIAL_PROTOCOLPGM("G29 Auto Bed Leveling\n");          //g 29自动床水平\N")；
        if (dryrun) SERIAL_ECHOLNPGM("Running in DRY-RUN mode");
      }

      int auto_bed_leveling_grid_points = AUTO_BED_LEVELING_GRID_POINTS;

      #if DISABLED(DELTA)
        if (code_seen('P')) auto_bed_leveling_grid_points = code_value_short();
        if (auto_bed_leveling_grid_points < 2) {
          SERIAL_PROTOCOLPGM("?Number of probed (P)oints is implausible (2 minimum).\n");
          return;
        }
      #endif

      xy_travel_speed = code_seen('S') ? code_value_short() : XY_TRAVEL_SPEED;

      int left_probe_bed_position = code_seen('L') ? code_value_short() : LEFT_PROBE_BED_POSITION,
          right_probe_bed_position = code_seen('R') ? code_value_short() : RIGHT_PROBE_BED_POSITION,
          front_probe_bed_position = code_seen('F') ? code_value_short() : FRONT_PROBE_BED_POSITION,
          back_probe_bed_position = code_seen('B') ? code_value_short() : BACK_PROBE_BED_POSITION;

      bool left_out_l = left_probe_bed_position < MIN_PROBE_X,
           left_out = left_out_l || left_probe_bed_position > right_probe_bed_position - MIN_PROBE_EDGE,
           right_out_r = right_probe_bed_position > MAX_PROBE_X,
           right_out = right_out_r || right_probe_bed_position < left_probe_bed_position + MIN_PROBE_EDGE,
           front_out_f = front_probe_bed_position < MIN_PROBE_Y,
           front_out = front_out_f || front_probe_bed_position > back_probe_bed_position - MIN_PROBE_EDGE,
           back_out_b = back_probe_bed_position > MAX_PROBE_Y,
           back_out = back_out_b || back_probe_bed_position < front_probe_bed_position + MIN_PROBE_EDGE;

      if (left_out || right_out || front_out || back_out) {
        if (left_out) {
          out_of_range_error(PSTR("(L)eft"));
          left_probe_bed_position = left_out_l ? MIN_PROBE_X : right_probe_bed_position - MIN_PROBE_EDGE;
        }
        if (right_out) {
          out_of_range_error(PSTR("(R)ight"));
          right_probe_bed_position = right_out_r ? MAX_PROBE_X : left_probe_bed_position + MIN_PROBE_EDGE;
        }
        if (front_out) {
          out_of_range_error(PSTR("(F)ront"));
          front_probe_bed_position = front_out_f ? MIN_PROBE_Y : back_probe_bed_position - MIN_PROBE_EDGE;
        }
        if (back_out) {
          out_of_range_error(PSTR("(B)ack"));
          back_probe_bed_position = back_out_b ? MAX_PROBE_Y : front_probe_bed_position + MIN_PROBE_EDGE;
        }
        return;
      }

    #endif // AUTO_BED_LEVELING_GRID

    #if ENABLED(Z_PROBE_SLED)
      dock_sled(false); // engage (un-dock) the Z probe
    #elif ENABLED(Z_PROBE_ALLEN_KEY) //|| SERVO_LEVELING  伺服整平
      deploy_z_probe();
    #endif

    st_synchronize();

    if (!dryrun) {
      // make sure the bed_level_rotation_matrix is identity or the planner will get it wrong
     //确保床层旋转矩阵是恒等的，否则计划者会把它弄错
      plan_bed_level_matrix.set_to_identity();

      #if ENABLED(DELTA)
        reset_bed_level();
      #else //!DELTA
        //vector_3 corrected_position = plan_get_position_mm();
        //corrected_position.debug("position before G29");
        vector_3 uncorrected_position = plan_get_position();
        //uncorrected_position.debug("position during G29");
        current_position[X_AXIS] = uncorrected_position.x;
        current_position[Y_AXIS] = uncorrected_position.y;
        current_position[Z_AXIS] = uncorrected_position.z;
        sync_plan_position();
      #endif // !DELTA
    }

    setup_for_endstop_move();

    feedrate = homing_feedrate[Z_AXIS];

    #if ENABLED(AUTO_BED_LEVELING_GRID)

      // probe at the points of a lattice grid
      const int xGridSpacing = (right_probe_bed_position - left_probe_bed_position) / (auto_bed_leveling_grid_points - 1),
                yGridSpacing = (back_probe_bed_position - front_probe_bed_position) / (auto_bed_leveling_grid_points - 1);

      #if ENABLED(DELTA)
        delta_grid_spacing[0] = xGridSpacing;
        delta_grid_spacing[1] = yGridSpacing;
        float z_offset = zprobe_zoffset;
        if (code_seen(axis_codes[Z_AXIS])) z_offset += code_value();
      #else // !DELTA
        // solve the plane equation ax + by + d = z
        // A is the matrix with rows [x y 1] for all the probed points
        // B is the vector of the Z positions
        // the normal vector to the plane is formed by the coefficients of the plane equation in the standard form, which is Vx*x+Vy*y+Vz*z+d = 0
        // so Vx = -a Vy = -b Vz = 1 (we want the vector facing towards positive Z

        int abl2 = auto_bed_leveling_grid_points * auto_bed_leveling_grid_points;

        double eqnAMatrix[abl2 * 3], // "A" matrix of the linear system of equations线性方程组的"a"矩阵
               eqnBVector[abl2],     // "B" vector of Z points
               mean = 0.0;
        int8_t indexIntoAB[auto_bed_leveling_grid_points][auto_bed_leveling_grid_points];
      #endif // !DELTA

      int probePointCounter = 0;
      bool zig = (auto_bed_leveling_grid_points & 1) ? true : false; //always end at [RIGHT_PROBE_BED_POSITION, BACK_PROBE_BED_POSITION]

      for (int yCount = 0; yCount < auto_bed_leveling_grid_points; yCount++) {
        double yProbe = front_probe_bed_position + yGridSpacing * yCount;
        int xStart, xStop, xInc;

        if (zig) {
          xStart = 0;
          xStop = auto_bed_leveling_grid_points;
          xInc = 1;
        }
        else {
          xStart = auto_bed_leveling_grid_points - 1;
          xStop = -1;
          xInc = -1;
        }

        zig = !zig;

        for (int xCount = xStart; xCount != xStop; xCount += xInc) {
          double xProbe = left_probe_bed_position + xGridSpacing * xCount;

          // raise extruder
          float measured_z,
                z_before = probePointCounter ? Z_RAISE_BETWEEN_PROBINGS + current_position[Z_AXIS] : Z_RAISE_BEFORE_PROBING;

          if (probePointCounter) {
            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (marlin_debug_flags & DEBUG_LEVELING) {
                SERIAL_ECHOPAIR("z_before = (between) ", (float)(Z_RAISE_BETWEEN_PROBINGS + current_position[Z_AXIS]));
                SERIAL_EOL;
              }
            #endif
          }
          else {
            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (marlin_debug_flags & DEBUG_LEVELING) {
                SERIAL_ECHOPAIR("z_before = (before) ", (float)Z_RAISE_BEFORE_PROBING);
                SERIAL_EOL;
              }
            #endif
          }

          #if ENABLED(DELTA)
            // Avoid probing the corners (outside the round or hexagon print surface) on a delta printer.
            //避免在Delta打印机上探测角（圆形或六角打印面外）
            float distance_from_center = sqrt(xProbe * xProbe + yProbe * yProbe);
            if (distance_from_center > DELTA_PROBABLE_RADIUS) continue;
          #endif //DELTA

          ProbeAction act;
          if (deploy_probe_for_each_reading) // G29 E - Stow between probes  G29探针之间的电子积载
            act = ProbeDeployAndStow;
          else if (yCount == 0 && xCount == xStart)
            act = ProbeDeploy;
          else if (yCount == auto_bed_leveling_grid_points - 1 && xCount == xStop - xInc)
            act = ProbeStow;
          else
            act = ProbeStay;

          measured_z = probe_pt(xProbe, yProbe, z_before, act, verbose_level);

          #if DISABLED(DELTA)
            mean += measured_z;

            eqnBVector[probePointCounter] = measured_z;
            eqnAMatrix[probePointCounter + 0 * abl2] = xProbe;
            eqnAMatrix[probePointCounter + 1 * abl2] = yProbe;
            eqnAMatrix[probePointCounter + 2 * abl2] = 1;
            indexIntoAB[xCount][yCount] = probePointCounter;
          #else
            bed_level[xCount][yCount] = measured_z + z_offset;
          #endif

          probePointCounter++;

          idle();

        } //xProbe
      } //yProbe

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
          print_xyz("> probing complete > current_position", current_position);
        }
      #endif

      clean_up_after_endstop_move();

      #if ENABLED(DELTA)

        if (!dryrun) extrapolate_unprobed_bed_level();
        print_bed_level();

      #else // !DELTA

        // solve lsq problem
        double plane_equation_coefficients[3];
        qr_solve(plane_equation_coefficients, abl2, 3, eqnAMatrix, eqnBVector);

        mean /= abl2;

        if (verbose_level) {
          SERIAL_PROTOCOLPGM("Eqn coefficients: a: ");
          SERIAL_PROTOCOL_F(plane_equation_coefficients[0], 8);
          SERIAL_PROTOCOLPGM(" b: ");
          SERIAL_PROTOCOL_F(plane_equation_coefficients[1], 8);
          SERIAL_PROTOCOLPGM(" d: ");
          SERIAL_PROTOCOL_F(plane_equation_coefficients[2], 8);
          SERIAL_EOL;
          if (verbose_level > 2) {
            SERIAL_PROTOCOLPGM("Mean of sampled points: ");
            SERIAL_PROTOCOL_F(mean, 8);
            SERIAL_EOL;
          }
        }

        if (!dryrun) set_bed_level_equation_lsq(plane_equation_coefficients);

        // Show the Topography map if enabled 启用后显示地形图
        if (do_topography_map) {

          SERIAL_PROTOCOLPGM(" \nBed Height Topography: \n");
          SERIAL_PROTOCOLPGM("+-----------+\n");
          SERIAL_PROTOCOLPGM("|...Back....|\n");
          SERIAL_PROTOCOLPGM("|Left..Right|\n");
          SERIAL_PROTOCOLPGM("|...Front...|\n");
          SERIAL_PROTOCOLPGM("+-----------+\n");

          float min_diff = 999;

          for (int yy = auto_bed_leveling_grid_points - 1; yy >= 0; yy--) {
            for (int xx = 0; xx < auto_bed_leveling_grid_points; xx++) {
              int ind = indexIntoAB[xx][yy];
              float diff = eqnBVector[ind] - mean;

              float x_tmp = eqnAMatrix[ind + 0 * abl2],
                    y_tmp = eqnAMatrix[ind + 1 * abl2],
                    z_tmp = 0;

              apply_rotation_xyz(plan_bed_level_matrix, x_tmp, y_tmp, z_tmp);

              if (eqnBVector[ind] - z_tmp < min_diff)
                min_diff = eqnBVector[ind] - z_tmp;

              if (diff >= 0.0)
                SERIAL_PROTOCOLPGM(" +");   // Include + for column alignment
              else
                SERIAL_PROTOCOLCHAR(' ');
              SERIAL_PROTOCOL_F(diff, 5);
            } // xx
            SERIAL_EOL;
          } // yy
          SERIAL_EOL;
          if (verbose_level > 3) {
            SERIAL_PROTOCOLPGM(" \nCorrected Bed Height vs. Bed Topology: \n");

            for (int yy = auto_bed_leveling_grid_points - 1; yy >= 0; yy--) {
              for (int xx = 0; xx < auto_bed_leveling_grid_points; xx++) {
                int ind = indexIntoAB[xx][yy];
                float x_tmp = eqnAMatrix[ind + 0 * abl2],
                      y_tmp = eqnAMatrix[ind + 1 * abl2],
                      z_tmp = 0;

                apply_rotation_xyz(plan_bed_level_matrix, x_tmp, y_tmp, z_tmp);

                float diff = eqnBVector[ind] - z_tmp - min_diff;
                if (diff >= 0.0)
                  SERIAL_PROTOCOLPGM(" +");
                // Include + for column alignment
                else
                  SERIAL_PROTOCOLCHAR(' ');
                SERIAL_PROTOCOL_F(diff, 5);
              } // xx
              SERIAL_EOL;
            } // yy
            SERIAL_EOL;
          }
        } //do_topography_map
      #endif //!DELTA

    #else // !AUTO_BED_LEVELING_GRID

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
          SERIAL_ECHOLNPGM("> 3-point Leveling");
        }
      #endif

      // Actions for each probe
      ProbeAction p1, p2, p3;
      if (deploy_probe_for_each_reading)
        p1 = p2 = p3 = ProbeDeployAndStow;
      else
        p1 = ProbeDeploy, p2 = ProbeStay, p3 = ProbeStow;

      // Probe at 3 arbitrary points
      float z_at_pt_1 = probe_pt(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, Z_RAISE_BEFORE_PROBING, p1, verbose_level),
            z_at_pt_2 = probe_pt(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS, p2, verbose_level),
            z_at_pt_3 = probe_pt(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS, p3, verbose_level);
      clean_up_after_endstop_move();
      if (!dryrun) set_bed_level_equation_3pts(z_at_pt_1, z_at_pt_2, z_at_pt_3);

    #endif // !AUTO_BED_LEVELING_GRID

    #if ENABLED(DELTA)
      // Allen Key Probe for Delta
      #if ENABLED(Z_PROBE_ALLEN_KEY)
        stow_z_probe();
      #elif Z_RAISE_AFTER_PROBING > 0
        raise_z_after_probing();
      #endif
    #else // !DELTA
      if (verbose_level > 0)
        plan_bed_level_matrix.debug(" \n\nBed Level Correction Matrix:");

      if (!dryrun) {
        // Correct the Z height difference from Z probe position and nozzle tip position.修正了z探针位置和喷嘴尖端位置的z高度差。
        // The Z height on homing is measured by Z probe, but the Z probe is quite far from the nozzle.归航时的z高度是用z探针测量的，但z探针距离喷嘴相当远。
        // When the bed is uneven, this height must be corrected. 当床不平时，这个高度必须修正。
        float x_tmp = current_position[X_AXIS] + X_PROBE_OFFSET_FROM_EXTRUDER,
              y_tmp = current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_EXTRUDER,
              z_tmp = current_position[Z_AXIS],
              real_z = st_get_position_mm(Z_AXIS);  //get the real Z (since plan_get_position is now correcting the plane)得到真正的z（因为平面位置正在修正平面）

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            SERIAL_ECHOPAIR("> BEFORE apply_rotation_xyz > z_tmp  = ", z_tmp);
            SERIAL_EOL;
            SERIAL_ECHOPAIR("> BEFORE apply_rotation_xyz > real_z = ", real_z);
            SERIAL_EOL;
          }
        #endif

        apply_rotation_xyz(plan_bed_level_matrix, x_tmp, y_tmp, z_tmp); // Apply the correction sending the Z probe offset

        // Get the current Z position and send it to the planner.
        //
        // >> (z_tmp - real_z) : The rotated current Z minus the uncorrected Z (most recent plan_set_position/sync_plan_position)
        //
        // >> zprobe_zoffset : Z distance from nozzle to Z probe (set by default, M851, EEPROM, or Menu)
        //
        // >> Z_RAISE_AFTER_PROBING : The distance the Z probe will have lifted after the last probe
        //
        // >> Should home_offset[Z_AXIS] be included?
        //
        //      Discussion: home_offset[Z_AXIS] was applied in G28 to set the starting Z.
        //      If Z is not tweaked in G29 -and- the Z probe in G29 is not actually "homing" Z...
        //      then perhaps it should not be included here. The purpose of home_offset[] is to
        //      adjust for inaccurate endstops, not for reasonably accurate probes. If it were
        //      added here, it could be seen as a compensating factor for the Z probe.
       /*应用发送z探针偏移量的校正得到当前的z位置并发送给计划者。(z_tmp-ree_z)：旋转的电流z减去未修正的z(最近的平面_set_位置/同步_平面_位置)
         zprobe_zplac:从喷嘴到z探针的z距离（默认设置为m851、eEME或菜单）z_r_r_trait：在最后一个探针之后，z探针将提升的距离
         是否应该包含主偏移[z_轴]？讨论：在g 28中应用了home_fact[z_轴]来设置起始z。如果在g 29中没有调整z，那么在g 29中的z探测器实际上并不是“归航”z。..
          那么也许它不应该包括在这里。家用偏移量的目的是调整不准确的端子，而不是合理准确的探针。如果是的话在此补充，它可以被视为z探测器的补偿因素。*/
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            SERIAL_ECHOPAIR("> AFTER apply_rotation_xyz > z_tmp  = ", z_tmp);
            SERIAL_EOL;
          }
        #endif

        current_position[Z_AXIS] = -zprobe_zoffset + (z_tmp - real_z)
          #if HAS_SERVO_ENDSTOPS || ENABLED(Z_PROBE_ALLEN_KEY) || ENABLED(Z_PROBE_SLED)
             + Z_RAISE_AFTER_PROBING
          #endif
          ;
        // current_position[Z_AXIS] += home_offset[Z_AXIS]; // The Z probe determines Z=0, not "Z home"
        sync_plan_position();

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            print_xyz("> corrected Z in G29", current_position);
          }
        #endif
      }

      // Sled assembly for Cartesian bots  笛卡尔机器人的雪橇组件
      #if ENABLED(Z_PROBE_SLED)
        dock_sled(true); // dock the sled  把雪橇停靠起来
      #endif

    #endif // !DELTA

    #ifdef Z_PROBE_END_SCRIPT
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
          SERIAL_ECHO("Z Probe End Script: ");
          SERIAL_ECHOLNPGM(Z_PROBE_END_SCRIPT);
        }
      #endif
      enqueuecommands_P(PSTR(Z_PROBE_END_SCRIPT));
      st_synchronize();
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        SERIAL_ECHOLNPGM("<<< gcode_G29");
      }
    #endif

  }

  #if DISABLED(Z_PROBE_SLED)

    /**
     * G30: Do a single Z probe at the current XY  在当前xy上做一个单独的z探针
     */
    inline void gcode_G30() {
      #if HAS_SERVO_ENDSTOPS
        raise_z_for_servo();
      #endif
      deploy_z_probe(); // Engage Z Servo endstop if available  如果有，启动z伺服端子

      st_synchronize();
      // TODO: clear the leveling matrix or the planner will be set incorrectly  完成：清除级别矩阵，否则计划者将被错误地设置
      setup_for_endstop_move();

      feedrate = homing_feedrate[Z_AXIS];

      run_z_probe();
      SERIAL_PROTOCOLPGM("Bed X: ");
      SERIAL_PROTOCOL(current_position[X_AXIS] + 0.0001);
      SERIAL_PROTOCOLPGM(" Y: ");
      SERIAL_PROTOCOL(current_position[Y_AXIS] + 0.0001);
      SERIAL_PROTOCOLPGM(" Z: ");
      SERIAL_PROTOCOL(current_position[Z_AXIS] + 0.0001);
      SERIAL_EOL;

      clean_up_after_endstop_move();

      #if HAS_SERVO_ENDSTOPS
        raise_z_for_servo();
      #endif
      stow_z_probe(false); // Retract Z Servo endstop if available 如果有，请收回z伺服端子
    }

  #endif //!Z_PROBE_SLED

#endif //AUTO_BED_LEVELING_FEATURE

/**
 * G92: Set current position to given X Y Z E  将当前位置设为x-y-z
 */
inline void gcode_G92() {
  if (!code_seen(axis_codes[E_AXIS]))
    st_synchronize();

  bool didXYZ = false;
  for (int i = 0; i < NUM_AXIS; i++) {
    if (code_seen(axis_codes[i])) {
      float v = current_position[i] = code_value();
      if (i == E_AXIS)
        plan_set_e_position(v);
      else {
        didXYZ = true;
        if (i == Z_AXIS) {
          axis_known_position[Z_AXIS] = true;
        }
      }
    }
  }
  if (didXYZ) {
    #if ENABLED(DELTA) || ENABLED(SCARA)
      sync_plan_position_delta();
    #else
      sync_plan_position();
    #endif
  }
}

#if ENABLED(ULTIPANEL)

  /**
   * M0: // M0 - Unconditional stop - Wait for user button press on LCD
   * M1: // M1 - Conditional stop - Wait for user button press on LCD     m1-有条件的停止-等待lcd上的用户按钮
   */
  inline void gcode_M0_M1() {
    char* args = current_command_args;

    millis_t codenum = 0;
    bool hasP = false, hasS = false;
    if (code_seen('P')) {
      codenum = code_value_short(); // milliseconds to wait
      hasP = codenum > 0;
    }
    if (code_seen('S')) {
      codenum = code_value() * 1000; // seconds to wait
      hasS = codenum > 0;
    }

    if (!hasP && !hasS && *args != '\0')
      lcd_setstatus(args, true);
    else {
      LCD_MESSAGEPGM(MSG_USERWAIT);
      #if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
        dontExpireStatus();
      #endif
    }

    lcd_ignore_click();
    st_synchronize();
    refresh_cmd_timeout();
    if (codenum > 0) {
      codenum += previous_cmd_ms;  // wait until this time for a click等到这个时候再点击
      while (millis() < codenum && !lcd_clicked()) idle();
      lcd_ignore_click(false);
    }
    else {
      if (!lcd_detected()) return;
      while (!lcd_clicked()) idle();
    }
    if (IS_SD_PRINTING)
      LCD_MESSAGEPGM(MSG_RESUMING);
    else
      LCD_MESSAGEPGM(WELCOME_MSG);
  }

#endif // ULTIPANEL

/**
 * M17: Enable power on all stepper motors  m17：使所有步进电机都能供电
 */
inline void gcode_M17() {
  LCD_MESSAGEPGM(MSG_NO_MOVE);
  enable_all_steppers();
}

#if ENABLED(SDSUPPORT)

  /**
   * M20: List SD card to serial output   m20：列出sd卡到串行输出
   */
  inline void gcode_M20() {
    SERIAL_PROTOCOLLNPGM(MSG_BEGIN_FILE_LIST);
    card.ls();
    SERIAL_PROTOCOLLNPGM(MSG_END_FILE_LIST);
  }

  /**
   * M21: Init SD Card
   */
  inline void gcode_M21() {
    card.initsd();
    #if ENABLED(SDSUPPORT) && ENABLED(POWEROFF_SAVE_SD_FILE)////断电续打
     init_power_off_info();
   #endif
  }

  /**
   * M22: Release SD Card  释放sd卡
   */
  inline void gcode_M22() {
    card.release();
  }

  /**
   * M23: Select a file
   */
  inline void gcode_M23() {
    card.openFile(current_command_args, true);
  }

  /**
   * M24: Start SD Print
   */
  inline void gcode_M24() {
    card.startFileprint();
    print_job_start_ms = millis();
  }

  /**
   * M25: Pause SD Print
   */
  inline void gcode_M25() {
    card.pauseSDPrint();
  }

  /**
   * M26: Set SD Card file index
   */
  inline void gcode_M26() {
    if (card.cardOK && code_seen('S'))
      card.setIndex(code_value_short());
  }

  /**
   * M27: Get SD Card status
   */
  inline void gcode_M27() {
    card.getStatus();
  }

  /**
   * M28: Start SD Write
   */
  inline void gcode_M28() {
    card.openFile(current_command_args, false);
  }

  /**
   * M29: Stop SD Write     M29：停止sd写作以书面形式处理至档案程序
   * Processed in write to file routine above
   */
  inline void gcode_M29() {
    // card.saving = false;
  }

  /**
   * M30 <filename>: Delete SD Card file
   */
  inline void gcode_M30() {
    if (card.cardOK) {
      card.closefile();
      card.removeFile(current_command_args);
    }
  }

#endif //SDSUPPORT

/**
 * M31: Get the time since the start of SD Print (or last M109)  m31：从sd打印（或上一次的m109）开始计算时间
 */
inline void gcode_M31() {
  print_job_stop_ms = millis();
  millis_t t = (print_job_stop_ms - print_job_start_ms) / 1000;
  int min = t / 60, sec = t % 60;
  char time[30];
  sprintf_P(time, PSTR("%i min, %i sec"), min, sec);
  SERIAL_ECHO_START;
  SERIAL_ECHOLN(time);
  lcd_setstatus(time);
  autotempShutdown();
}

#if ENABLED(SDSUPPORT)

  /**
   * M32: Select file and start SD Print
   */
  inline void gcode_M32() {
    if (card.sdprinting)
      st_synchronize();

    char* namestartpos = strchr(current_command_args, '!');  // Find ! to indicate filename string start.
    if (!namestartpos)
      namestartpos = current_command_args; // Default name position, 4 letters after the M
    else
      namestartpos++; //to skip the '!'

    bool call_procedure = code_seen('P') && (seen_pointer < namestartpos);

    if (card.cardOK) {
      card.openFile(namestartpos, true, !call_procedure);

      if (code_seen('S') && seen_pointer < namestartpos) // "S" (must occur _before_ the filename!)
        card.setIndex(code_value_short());

      card.startFileprint();
      if (!call_procedure)
        print_job_start_ms = millis(); //procedure calls count as normal print time.
    }
  }

  #if ENABLED(LONG_FILENAME_HOST_SUPPORT)

    /**
     * M33: Get the long full path of a file or folder
     *
     * Parameters:
     *   <dospath> Case-insensitive DOS-style path to a file or folder
     *
     * Example:
     *   M33 miscel~1/armchair/armcha~1.gco
     *
     * Output:
     *   /Miscellaneous/Armchair/Armchair.gcode
     */
    inline void gcode_M33() {
      card.printLongPath(current_command_args);
    }

  #endif

  /**
   * M928: Start SD Write  开始sd编写
   */
  inline void gcode_M928() {
    card.openLogFile(current_command_args);
  }

#endif // SDSUPPORT

/**
 * M42: Change pin status via GCode  m42：通过gcode改变引脚状态
 */
inline void gcode_M42() {
  if (code_seen('S')) {
    int pin_status = code_value_short(),
        pin_number = LED_PIN;

    if (code_seen('P') && pin_status >= 0 && pin_status <= 255)
      pin_number = code_value_short();

    for (uint8_t i = 0; i < COUNT(sensitive_pins); i++) {
      if (sensitive_pins[i] == pin_number) {
        pin_number = -1;
        break;
      }
    }

    #if HAS_FAN
      if (pin_number == FAN_PIN) fanSpeed = pin_status;
    #endif

    if (pin_number > -1) {
      pinMode(pin_number, OUTPUT);
      digitalWrite(pin_number, pin_status);
      analogWrite(pin_number, pin_status);
    }
  } // code_seen('S')
}

#if ENABLED(AUTO_BED_LEVELING_FEATURE) && ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)

  // This is redundant since the SanityCheck.h already checks for a valid Z_MIN_PROBE_PIN, but here for clarity.
  //这是多余的，因为卫生检查已经检查了一个有效的zmin探针针，但在这里为了清晰
  #if ENABLED(Z_MIN_PROBE_ENDSTOP)
    #if !HAS_Z_PROBE
      #error You must define Z_MIN_PROBE_PIN to enable Z probe repeatability calculation.
    #endif
  #elif !HAS_Z_MIN
    #error You must define Z_MIN_PIN to enable Z probe repeatability calculation.
  #endif

  /**
   * M48: Z probe repeatability measurement function.  z探针的可重复性测量函数
   *
   * Usage:
   *   M48 <P#> <X#> <Y#> <V#> <E> <L#>
   *     P = Number of sampled points (4-50, default 10)
   *     X = Sample X position
   *     Y = Sample Y position
   *     V = Verbose level (0-4, default=1)  详细级别(0-4，默认=1)
   *     E = Engage Z probe for each reading   每行读入z探针
   *     L = Number of legs of movement before probe  探测前的移动腿数
   *
   * This function assumes the bed has been homed.  Specifically, that a G28 command               本功能假定床已归位。特别是，g28命令
   * as been issued prior to invoking the M48 Z probe repeatability measurement function.          在调用m 48z探针可重复性测量函数之前发出的
   * Any information generated by a prior G29 Bed leveling command will be lost and need to be     任何由先前的g 29床整平指令所产生的资讯将会遗失，并需要
   * regenerated.
   */
  inline void gcode_M48() {

    double sum = 0.0, mean = 0.0, sigma = 0.0, sample_set[50];
    uint8_t verbose_level = 1, n_samples = 10, n_legs = 0;

    if (code_seen('V')) {
      verbose_level = code_value_short();
      if (verbose_level < 0 || verbose_level > 4) {
        SERIAL_PROTOCOLPGM("?Verbose Level not plausible (0-4).\n");
        return;
      }
    }

    if (verbose_level > 0)
      SERIAL_PROTOCOLPGM("M48 Z-Probe Repeatability test\n");

    if (code_seen('P')) {
      n_samples = code_value_short();
      if (n_samples < 4 || n_samples > 50) {
        SERIAL_PROTOCOLPGM("?Sample size not plausible (4-50).\n");
        return;
      }
    }

    double X_current = st_get_position_mm(X_AXIS),
           Y_current = st_get_position_mm(Y_AXIS),
           Z_current = st_get_position_mm(Z_AXIS),
           E_current = st_get_position_mm(E_AXIS),
           X_probe_location = X_current, Y_probe_location = Y_current,
           Z_start_location = Z_current + Z_RAISE_BEFORE_PROBING;

    bool deploy_probe_for_each_reading = code_seen('E');

    if (code_seen('X')) {
      X_probe_location = code_value() - X_PROBE_OFFSET_FROM_EXTRUDER;
      if (X_probe_location < X_MIN_POS || X_probe_location > X_MAX_POS) {
        out_of_range_error(PSTR("X"));
        return;
      }
    }

    if (code_seen('Y')) {
      Y_probe_location = code_value() -  Y_PROBE_OFFSET_FROM_EXTRUDER;
      if (Y_probe_location < Y_MIN_POS || Y_probe_location > Y_MAX_POS) {
        out_of_range_error(PSTR("Y"));
        return;
      }
    }

    if (code_seen('L')) {
      n_legs = code_value_short();
      if (n_legs == 1) n_legs = 2;
      if (n_legs < 0 || n_legs > 15) {
        SERIAL_PROTOCOLPGM("?Number of legs in movement not plausible (0-15).\n");
        return;
      }
    }

    //
    // Do all the preliminary setup work.   First raise the Z probe.  完成所有的初步安装工作。首先举起z探测器
    //

    st_synchronize();
    plan_bed_level_matrix.set_to_identity();
    plan_buffer_line(X_current, Y_current, Z_start_location, E_current, homing_feedrate[Z_AXIS] / 60, active_extruder);
    st_synchronize();

    //
    // Now get everything to the specified probe point So we can safely do a probe to 现在把所有东西都送到指定的探测点，这样我们就可以安全地探测到
    // get us close to the bed.  If the Z-Axis is far from the bed, we don't want to  让我们靠近床。如果Z轴离床很远，我们不想
    // use that as a starting point for each probe.  用它作为每个探针的起点。
    //
    if (verbose_level > 2)
      SERIAL_PROTOCOLPGM("Positioning the probe...\n");

    plan_buffer_line(X_probe_location, Y_probe_location, Z_start_location,
                     E_current,
                     homing_feedrate[X_AXIS] / 60,
                     active_extruder);
    st_synchronize();

    current_position[X_AXIS] = X_current = st_get_position_mm(X_AXIS);
    current_position[Y_AXIS] = Y_current = st_get_position_mm(Y_AXIS);
    current_position[Z_AXIS] = Z_current = st_get_position_mm(Z_AXIS);
    current_position[E_AXIS] = E_current = st_get_position_mm(E_AXIS);

    //
    // OK, do the initial probe to get us close to the bed.  好吧，做初始探针让我们靠近床
    // Then retrace the right amount and use that in subsequent probes  然后追溯正确的数量，并在随后的探针中使用
    //

    deploy_z_probe();

    setup_for_endstop_move();
    run_z_probe();

    current_position[Z_AXIS] = Z_current = st_get_position_mm(Z_AXIS);
    Z_start_location = st_get_position_mm(Z_AXIS) + Z_RAISE_BEFORE_PROBING;

    plan_buffer_line(X_probe_location, Y_probe_location, Z_start_location,
                     E_current,
                     homing_feedrate[X_AXIS] / 60,
                     active_extruder);
    st_synchronize();
    current_position[Z_AXIS] = Z_current = st_get_position_mm(Z_AXIS);

    if (deploy_probe_for_each_reading) stow_z_probe();

    for (uint8_t n = 0; n < n_samples; n++) {
      // Make sure we are at the probe location  确保我们在探测器的位置
      do_blocking_move_to(X_probe_location, Y_probe_location, Z_start_location); // this also updates current_position

      if (n_legs) {
        millis_t ms = millis();
        double radius = ms % (X_MAX_LENGTH / 4),       // limit how far out to go  限制前进的距离
               theta = RADIANS(ms % 360L);
        float dir = (ms & 0x0001) ? 1 : -1;            // clockwise or counter clockwise 顺时针或逆时针

        //SERIAL_ECHOPAIR("starting radius: ",radius);
        //SERIAL_ECHOPAIR("   theta: ",theta);
        //SERIAL_ECHOPAIR("   direction: ",dir);
        //SERIAL_EOL;

        for (uint8_t l = 0; l < n_legs - 1; l++) {
          ms = millis();
          theta += RADIANS(dir * (ms % 20L));
          radius += (ms % 10L) - 5L;
          if (radius < 0.0) radius = -radius;

          X_current = X_probe_location + cos(theta) * radius;
          X_current = constrain(X_current, X_MIN_POS, X_MAX_POS);
          Y_current = Y_probe_location + sin(theta) * radius;
          Y_current = constrain(Y_current, Y_MIN_POS, Y_MAX_POS);

          if (verbose_level > 3) {
            SERIAL_ECHOPAIR("x: ", X_current);
            SERIAL_ECHOPAIR("y: ", Y_current);
            SERIAL_EOL;
          }

          do_blocking_move_to(X_current, Y_current, Z_current); // this also updates current_position

        } // n_legs loop

        // Go back to the probe location  回到探测地点
        do_blocking_move_to(X_probe_location, Y_probe_location, Z_start_location); // this also updates current_position

      } // n_legs

      if (deploy_probe_for_each_reading)  {
        deploy_z_probe();
        delay(1000);
      }

      setup_for_endstop_move();
      run_z_probe();

      sample_set[n] = current_position[Z_AXIS];

      //
      // Get the current mean for the data points we have so far  得到我们目前所掌握的数据点的平均值
      //
      sum = 0.0;
      for (uint8_t j = 0; j <= n; j++) sum += sample_set[j];
      mean = sum / (n + 1);

      //
      // Now, use that mean to calculate the standard deviation for the 现在，用这个方法来计算
      // data points we have so far    到目前为止我们拥有的数据点
      //
      sum = 0.0;
      for (uint8_t j = 0; j <= n; j++) {
        float ss = sample_set[j] - mean;
        sum += ss * ss;
      }
      sigma = sqrt(sum / (n + 1));

      if (verbose_level > 1) {
        SERIAL_PROTOCOL(n + 1);
        SERIAL_PROTOCOLPGM(" of ");
        SERIAL_PROTOCOL((int)n_samples);
        SERIAL_PROTOCOLPGM("   z: ");
        SERIAL_PROTOCOL_F(current_position[Z_AXIS], 6);
        if (verbose_level > 2) {
          SERIAL_PROTOCOLPGM(" mean: ");
          SERIAL_PROTOCOL_F(mean, 6);
          SERIAL_PROTOCOLPGM("   sigma: ");
          SERIAL_PROTOCOL_F(sigma, 6);
        }
      }

      if (verbose_level > 0) SERIAL_EOL;

      plan_buffer_line(X_probe_location, Y_probe_location, Z_start_location, current_position[E_AXIS], homing_feedrate[Z_AXIS] / 60, active_extruder);
      st_synchronize();

      // Stow between
      if (deploy_probe_for_each_reading) {
        stow_z_probe();
        delay(1000);
      }
    }

    // Stow after
    if (!deploy_probe_for_each_reading) {
      stow_z_probe();
      delay(1000);
    }

    clean_up_after_endstop_move();

    if (verbose_level > 0) {
      SERIAL_PROTOCOLPGM("Mean: ");
      SERIAL_PROTOCOL_F(mean, 6);
      SERIAL_EOL;
    }

    SERIAL_PROTOCOLPGM("Standard Deviation: ");
    SERIAL_PROTOCOL_F(sigma, 6);
    SERIAL_EOL; SERIAL_EOL;
  }

#endif // AUTO_BED_LEVELING_FEATURE && Z_MIN_PROBE_REPEATABILITY_TEST   自动床平整特性&zmin探针可重复性试验

/**
 * M104: Set hot end temperature
 */
inline void gcode_M104() {
  if (setTargetedHotend(104)) return;
  if (marlin_debug_flags & DEBUG_DRYRUN) return;

  if (code_seen('S')) {
    float temp = code_value();
    setTargetHotend(temp, target_extruder);
    #if ENABLED(DUAL_X_CARRIAGE)
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && target_extruder == 0)
        setTargetHotend1(temp == 0.0 ? 0.0 : temp + duplicate_extruder_temp_offset);
    #endif
  }
}

/**
 * M105: Read hot end and bed temperature  读取热端和床温度
 */
inline void gcode_M105() {
  if (setTargetedHotend(105)) return;

  #if HAS_TEMP_0 || HAS_TEMP_BED || ENABLED(HEATER_0_USES_MAX6675)
    SERIAL_PROTOCOLPGM(MSG_OK);
    #if HAS_TEMP_0 || ENABLED(HEATER_0_USES_MAX6675)
      SERIAL_PROTOCOLPGM(" T:");
      SERIAL_PROTOCOL_F(degHotend(target_extruder), 1);
      SERIAL_PROTOCOLPGM(" /");
      SERIAL_PROTOCOL_F(degTargetHotend(target_extruder), 1);
    #endif
    #if HAS_TEMP_BED
      SERIAL_PROTOCOLPGM(" B:");
      SERIAL_PROTOCOL_F(degBed(), 1);
      SERIAL_PROTOCOLPGM(" /");
      SERIAL_PROTOCOL_F(degTargetBed(), 1);
    #endif
    for (int8_t e = 0; e < EXTRUDERS; ++e) {
      SERIAL_PROTOCOLPGM(" T");
      SERIAL_PROTOCOL(e);
      SERIAL_PROTOCOLCHAR(':');
      SERIAL_PROTOCOL_F(degHotend(e), 1);
      SERIAL_PROTOCOLPGM(" /");
      SERIAL_PROTOCOL_F(degTargetHotend(e), 1);
    }
  #else // !HAS_TEMP_0 && !HAS_TEMP_BED
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_NO_THERMISTORS);
  #endif

  SERIAL_PROTOCOLPGM(" @:");
  #ifdef EXTRUDER_WATTS
    SERIAL_PROTOCOL((EXTRUDER_WATTS * getHeaterPower(target_extruder)) / 127);
    SERIAL_PROTOCOLCHAR('W');
  #else
    SERIAL_PROTOCOL(getHeaterPower(target_extruder));
  #endif

  SERIAL_PROTOCOLPGM(" B@:");
  #ifdef BED_WATTS
    SERIAL_PROTOCOL((BED_WATTS * getHeaterPower(-1)) / 127);
    SERIAL_PROTOCOLCHAR('W');
  #else
    SERIAL_PROTOCOL(getHeaterPower(-1));
  #endif

  #if ENABLED(SHOW_TEMP_ADC_VALUES)
    #if HAS_TEMP_BED
      SERIAL_PROTOCOLPGM("    ADC B:");
      SERIAL_PROTOCOL_F(degBed(), 1);
      SERIAL_PROTOCOLPGM("C->");
      SERIAL_PROTOCOL_F(rawBedTemp() / OVERSAMPLENR, 0);
    #endif
    for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
      SERIAL_PROTOCOLPGM("  T");
      SERIAL_PROTOCOL(cur_extruder);
      SERIAL_PROTOCOLCHAR(':');
      SERIAL_PROTOCOL_F(degHotend(cur_extruder), 1);
      SERIAL_PROTOCOLPGM("C->");
      SERIAL_PROTOCOL_F(rawHotendTemp(cur_extruder) / OVERSAMPLENR, 0);
    }
  #endif

  SERIAL_EOL;
}

#if HAS_FAN

  /**
   * M106: Set Fan Speed
   */
  inline void gcode_M106() { fanSpeed = code_seen('S') ? constrain(code_value_short(), 0, 255) : 255; }

  /**
   * M107: Fan Off
   */
  inline void gcode_M107() { fanSpeed = 0; }

#endif // HAS_FAN

/**
 * M109: Wait for extruder(s) to reach temperature  等待挤出机达到温度
 */
inline void gcode_M109() {
  if (setTargetedHotend(109)) return;
  if (marlin_debug_flags & DEBUG_DRYRUN) return;

  LCD_MESSAGEPGM(MSG_HEATING);

  no_wait_for_cooling = code_seen('S');
  if (no_wait_for_cooling || code_seen('R')) {
    float temp = code_value();
    setTargetHotend(temp, target_extruder);
    #if ENABLED(DUAL_X_CARRIAGE)
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && target_extruder == 0)
        setTargetHotend1(temp == 0.0 ? 0.0 : temp + duplicate_extruder_temp_offset);
    #endif
  }

  #if ENABLED(AUTOTEMP)
    autotemp_enabled = code_seen('F');
    if (autotemp_enabled) autotemp_factor = code_value();
    if (code_seen('S')) autotemp_min = code_value();
    if (code_seen('B')) autotemp_max = code_value();
  #endif

  millis_t temp_ms = millis();

  /* See if we are heating up or cooling down   看看我们是在加热还是在冷却 */  
  target_direction = isHeatingHotend(target_extruder); // true if heating, false if cooling  如果加热是正确的，如果冷却是错误的

  cancel_heatup = false;

  #ifdef TEMP_RESIDENCY_TIME
    long residency_start_ms = -1;
    /* continue to loop until we have reached the target temp
      _and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it继续循环直到我们到达目标温度直到我们到达那里后，时间还没有过去 */
    while ((!cancel_heatup) && ((residency_start_ms == -1) ||
                                (residency_start_ms >= 0 && (((unsigned int)(millis() - residency_start_ms)) < (TEMP_RESIDENCY_TIME * 1000UL)))))
  #else
    while (target_direction ? (isHeatingHotend(target_extruder)) : (isCoolingHotend(target_extruder) && (no_wait_for_cooling == false)))
  #endif //TEMP_RESIDENCY_TIME

  { // while loop
    if (millis() > temp_ms + 1000UL) { //Print temp & remaining time every 1s while waiting  等待时每1打印一个临时时间(S)
      SERIAL_PROTOCOLPGM("T:");
      SERIAL_PROTOCOL_F(degHotend(target_extruder), 1);
      SERIAL_PROTOCOLPGM(" E:");
      SERIAL_PROTOCOL((int)target_extruder);
      #ifdef TEMP_RESIDENCY_TIME
        SERIAL_PROTOCOLPGM(" W:");
        if (residency_start_ms > -1) {
          temp_ms = ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residency_start_ms)) / 1000UL;
          SERIAL_PROTOCOLLN(temp_ms);
        }
        else {
          SERIAL_PROTOCOLLNPGM("?");
        }
      #else
        SERIAL_EOL;
      #endif
      temp_ms = millis();
    }

    idle();

    #ifdef TEMP_RESIDENCY_TIME
      // start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time  每当我们第一次到达目标温度时，就会启动/重新启动定时器
      // or when current temp falls outside the hysteresis after target temp was reached  或当达到目标温度后，电流温度超出滞后期
      if ((residency_start_ms == -1 &&  target_direction && (degHotend(target_extruder) >= (degTargetHotend(target_extruder) - TEMP_WINDOW))) ||
          (residency_start_ms == -1 && !target_direction && (degHotend(target_extruder) <= (degTargetHotend(target_extruder) + TEMP_WINDOW))) ||
          (residency_start_ms > -1 && labs(degHotend(target_extruder) - degTargetHotend(target_extruder)) > TEMP_HYSTERESIS) )
      {
        residency_start_ms = millis();
      }
    #endif //TEMP_RESIDENCY_TIME
  }

  LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
  refresh_cmd_timeout();
  print_job_start_ms = previous_cmd_ms;
}

#if HAS_TEMP_BED

  /**
   * M190: Sxxx Wait for bed current temp to reach target temp. Waits only when heating  等待床电流温度达到目标温度。只在加热时等待
   *       Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling  等待床电流温度达到目标温度。加热和冷却时等待
   */
  inline void gcode_M190() {
    if (marlin_debug_flags & DEBUG_DRYRUN) return;

    LCD_MESSAGEPGM(MSG_BED_HEATING);
    no_wait_for_cooling = code_seen('S');
    if (no_wait_for_cooling || code_seen('R'))
      setTargetBed(code_value());

    millis_t temp_ms = millis();

    cancel_heatup = false;
    target_direction = isHeatingBed(); // true if heating, false if cooling

    while ((target_direction && !cancel_heatup) ? isHeatingBed() : isCoolingBed() && !no_wait_for_cooling) {
      millis_t ms = millis();
      if (ms > temp_ms + 1000UL) { //Print Temp Reading every 1 second while heating up.
        temp_ms = ms;
        float tt = degHotend(active_extruder);
        SERIAL_PROTOCOLPGM("T:");
        SERIAL_PROTOCOL(tt);
        SERIAL_PROTOCOLPGM(" E:");
        SERIAL_PROTOCOL((int)active_extruder);
        SERIAL_PROTOCOLPGM(" B:");
        SERIAL_PROTOCOL_F(degBed(), 1);
        SERIAL_EOL;
      }
      idle();
    }
    LCD_MESSAGEPGM(MSG_BED_DONE);
    refresh_cmd_timeout();
  }

#endif // HAS_TEMP_BED

/**
 * M111: Set the debug level
 */
inline void gcode_M111() {
  marlin_debug_flags = code_seen('S') ? code_value_short() : DEBUG_INFO | DEBUG_COMMUNICATION;

  if (marlin_debug_flags & DEBUG_ECHO) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM(MSG_DEBUG_ECHO);
  }
  // FOR MOMENT NOT ACTIVE
  //if (marlin_debug_flags & DEBUG_INFO) SERIAL_ECHOLNPGM(MSG_DEBUG_INFO);
  //if (marlin_debug_flags & DEBUG_ERRORS) SERIAL_ECHOLNPGM(MSG_DEBUG_ERRORS);
  if (marlin_debug_flags & DEBUG_DRYRUN) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM(MSG_DEBUG_DRYRUN);
    disable_all_heaters();
  }

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (marlin_debug_flags & DEBUG_LEVELING) {
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM(MSG_DEBUG_LEVELING);
    }
  #endif
}

/**
 * M112: Emergency Stop  紧急停车
 */
inline void gcode_M112() { kill(PSTR(MSG_KILLED)); }

#if ENABLED(BARICUDA)

  #if HAS_HEATER_1
    /**
     * M126: Heater 1 valve open  加热器1阀开启
     */
    inline void gcode_M126() { ValvePressure = code_seen('S') ? constrain(code_value(), 0, 255) : 255; }
    /**
     * M127: Heater 1 valve close
     */
    inline void gcode_M127() { ValvePressure = 0; }
  #endif

  #if HAS_HEATER_2
    /**
     * M128: Heater 2 valve open
     */
    inline void gcode_M128() { EtoPPressure = code_seen('S') ? constrain(code_value(), 0, 255) : 255; }
    /**
     * M129: Heater 2 valve close
     */
    inline void gcode_M129() { EtoPPressure = 0; }
  #endif

#endif //BARICUDA

/**
 * M140: Set bed temperature  设定的床温度
 */
inline void gcode_M140() {
  if (marlin_debug_flags & DEBUG_DRYRUN) return;
  if (code_seen('S')) setTargetBed(code_value());
}

#if ENABLED(ULTIPANEL)

  /**
   * M145: Set the heatup state for a material in the LCD menu  为lcd菜单中的材料设置加热状态
   *   S<material> (0=PLA, 1=ABS)
   *   H<hotend temp>
   *   B<bed temp>
   *   F<fan speed>
   */
  inline void gcode_M145() {
    uint8_t material = code_seen('S') ? code_value_short() : 0;
    if (material < 0 || material > 1) {
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM(MSG_ERR_MATERIAL_INDEX);
    }
    else {
      int v;
      switch (material) {
        case 0:
          if (code_seen('H')) {
            v = code_value_short();
            plaPreheatHotendTemp = constrain(v, EXTRUDE_MINTEMP, HEATER_0_MAXTEMP - 15);
          }
          if (code_seen('F')) {
            v = code_value_short();
            plaPreheatFanSpeed = constrain(v, 0, 255);
          }
          #if TEMP_SENSOR_BED != 0
            if (code_seen('B')) {
              v = code_value_short();
              plaPreheatHPBTemp = constrain(v, BED_MINTEMP, BED_MAXTEMP - 15);
            }
          #endif
          break;
        case 1:
          if (code_seen('H')) {
            v = code_value_short();
            absPreheatHotendTemp = constrain(v, EXTRUDE_MINTEMP, HEATER_0_MAXTEMP - 15);
          }
          if (code_seen('F')) {
            v = code_value_short();
            absPreheatFanSpeed = constrain(v, 0, 255);
          }
          #if TEMP_SENSOR_BED != 0
            if (code_seen('B')) {
              v = code_value_short();
              absPreheatHPBTemp = constrain(v, BED_MINTEMP, BED_MAXTEMP - 15);
            }
          #endif
          break;
      }
    }
  }

#endif

#if HAS_POWER_SWITCH

  /**
   * M80: Turn on Power Supply   打开电源
   */
  inline void gcode_M80() {
    OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE); //GND

    // If you have a switch on suicide pin, this is useful             如果你换了自杀针，这个很有用
    // if you want to start another print with suicide feature after   如果你想在之后再印一张带有自杀功能的照片
    // a print without suicide...  没有自杀的指纹
    #if HAS_SUICIDE
      OUT_WRITE(SUICIDE_PIN, HIGH);
    #endif

    #if ENABLED(ULTIPANEL)
      powersupply = true;
      LCD_MESSAGEPGM(WELCOME_MSG);
      lcd_update();
    #endif
  }

#endif // HAS_POWER_SWITCH

/**
 * M81: Turn off Power, including Power Supply, if there is one.如果有电源，就关掉电源，包括电源。
 *
 *      This code should ALWAYS be available for EMERGENCY SHUTDOWN!  此代码应始终可用的紧急关闭！
 */
inline void gcode_M81() {
  disable_all_heaters();
  finishAndDisableSteppers();
  fanSpeed = 0;
  delay(1000); // Wait 1 second before switching off  在关闭前等1秒钟

  WRITE(OUT_PUT,HIGH);  //关闭电源
  delay(1000);

  #if HAS_SUICIDE
    st_synchronize();
    suicide();
  #elif HAS_POWER_SWITCH
    OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
  #endif
  #if ENABLED(ULTIPANEL)
    #if HAS_POWER_SWITCH
      powersupply = false;
    #endif
    LCD_MESSAGEPGM(MACHINE_NAME " " MSG_OFF ".");   //lcd messagepgm（机器名称“”“msg关闭”）
    lcd_update();
  #endif
}


/**
 * M82: Set E codes absolute (default)
 */
inline void gcode_M82() { axis_relative_modes[E_AXIS] = false; }

/**
 * M83: Set E codes relative while in Absolute Coordinates (G90) mode m83：在绝对坐标（g 90）模式下设置相对e编码
 */
inline void gcode_M83() { axis_relative_modes[E_AXIS] = true; }

/**
 * M18, M84: Disable all stepper motors
 */
inline void gcode_M18_M84() {
  if (code_seen('S')) {
    stepper_inactive_time = code_value() * 1000;
  }
  else {
    bool all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS])) || (code_seen(axis_codes[E_AXIS])));
    if (all_axis) {
      //finishAndDisableSteppers();
      disable_x();
      disable_y();
      
    }
    else {
      st_synchronize();
      if (code_seen('X')) disable_x();
      if (code_seen('Y')) disable_y();
      if (code_seen('Z')) disable_z();
      #if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
        if (code_seen('E')) {
          disable_e0();
          disable_e1();
          disable_e2();
          disable_e3();
        }
      #endif
    }
  }
}

/**
 * M85: Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 */
inline void gcode_M85() {
  if (code_seen('S')) max_inactive_time = code_value() * 1000;
}

/**
 * M92: Set axis steps-per-unit for one or more axes, X, Y, Z, and E.
 *      (Follows the same syntax as G92)
 */
inline void gcode_M92() {
  for (int8_t i = 0; i < NUM_AXIS; i++) {
    if (code_seen(axis_codes[i])) {
      if (i == E_AXIS) {
        float value = code_value();
        if (value < 20.0) {
          float factor = axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
          max_e_jerk *= factor;
          max_feedrate[i] *= factor;
          axis_steps_per_sqr_second[i] *= factor;
        }
        axis_steps_per_unit[i] = value;
      }
      else {
        axis_steps_per_unit[i] = code_value();
      }
    }
  }
}

/**
 * M114: Output current position to serial port  m114：输出当前位置到串行端口
 */
inline void gcode_M114() {
  SERIAL_PROTOCOLPGM("X:");
  SERIAL_PROTOCOL(current_position[X_AXIS]);
  SERIAL_PROTOCOLPGM(" Y:");
  SERIAL_PROTOCOL(current_position[Y_AXIS]);
  SERIAL_PROTOCOLPGM(" Z:");
  SERIAL_PROTOCOL(current_position[Z_AXIS]);
  SERIAL_PROTOCOLPGM(" E:");
  SERIAL_PROTOCOL(current_position[E_AXIS]);

  SERIAL_PROTOCOLPGM(MSG_COUNT_X);
  SERIAL_PROTOCOL(st_get_position_mm(X_AXIS));
  SERIAL_PROTOCOLPGM(" Y:");
  SERIAL_PROTOCOL(st_get_position_mm(Y_AXIS));
  SERIAL_PROTOCOLPGM(" Z:");
  SERIAL_PROTOCOL(st_get_position_mm(Z_AXIS));

  SERIAL_EOL;

  #if ENABLED(SCARA)
    SERIAL_PROTOCOLPGM("SCARA Theta:");
    SERIAL_PROTOCOL(delta[X_AXIS]);
    SERIAL_PROTOCOLPGM("   Psi+Theta:");
    SERIAL_PROTOCOL(delta[Y_AXIS]);
    SERIAL_EOL;

    SERIAL_PROTOCOLPGM("SCARA Cal - Theta:");
    SERIAL_PROTOCOL(delta[X_AXIS] + home_offset[X_AXIS]);
    SERIAL_PROTOCOLPGM("   Psi+Theta (90):");
    SERIAL_PROTOCOL(delta[Y_AXIS] - delta[X_AXIS] - 90 + home_offset[Y_AXIS]);
    SERIAL_EOL;

    SERIAL_PROTOCOLPGM("SCARA step Cal - Theta:");
    SERIAL_PROTOCOL(delta[X_AXIS] / 90 * axis_steps_per_unit[X_AXIS]);
    SERIAL_PROTOCOLPGM("   Psi+Theta:");
    SERIAL_PROTOCOL((delta[Y_AXIS] - delta[X_AXIS]) / 90 * axis_steps_per_unit[Y_AXIS]);
    SERIAL_EOL; SERIAL_EOL;
  #endif
}

/**
 * M115: Capabilities string
 */
inline void gcode_M115() {
  SERIAL_PROTOCOLPGM(MSG_M115_REPORT);
}

/**
 * M117: Set LCD Status Message  设定lcd状态讯息
 */
inline void gcode_M117() {
  lcd_setstatus(current_command_args);
}

/**
 * M119: Output endstop states to serial output  输出终端国家到串行输出
 */
inline void gcode_M119() {
  SERIAL_PROTOCOLLN(MSG_M119_REPORT);
  #if HAS_X_MIN
    SERIAL_PROTOCOLPGM(MSG_X_MIN);
    SERIAL_PROTOCOLLN(((READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_X_MAX
    SERIAL_PROTOCOLPGM(MSG_X_MAX);
    SERIAL_PROTOCOLLN(((READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Y_MIN
    SERIAL_PROTOCOLPGM(MSG_Y_MIN);
    SERIAL_PROTOCOLLN(((READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Y_MAX
    SERIAL_PROTOCOLPGM(MSG_Y_MAX);
    SERIAL_PROTOCOLLN(((READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z_MIN
    SERIAL_PROTOCOLPGM(MSG_Z_MIN);
    SERIAL_PROTOCOLLN(((READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z_MAX
    SERIAL_PROTOCOLPGM(MSG_Z_MAX);
    SERIAL_PROTOCOLLN(((READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z2_MAX
    SERIAL_PROTOCOLPGM(MSG_Z2_MAX);
    SERIAL_PROTOCOLLN(((READ(Z2_MAX_PIN)^Z2_MAX_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z_PROBE
    SERIAL_PROTOCOLPGM(MSG_Z_PROBE);
    SERIAL_PROTOCOLLN(((READ(Z_MIN_PROBE_PIN)^Z_MIN_PROBE_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
}

/**
 * M120: Enable endstops
 */
inline void gcode_M120() { enable_endstops(true); }

/**
 * M121: Disable endstops
 */
inline void gcode_M121() { enable_endstops(false); }

#if ENABLED(BLINKM)

  /**
   * M150: Set Status LED Color - Use R-U-B for R-G-B
   */
  inline void gcode_M150() {
    SendColors(
      code_seen('R') ? (byte)code_value_short() : 0,
      code_seen('U') ? (byte)code_value_short() : 0,
      code_seen('B') ? (byte)code_value_short() : 0
    );
  }

#endif // BLINKM

/**
 * M200: Set filament diameter and set E axis units to cubic millimeters   设定灯丝直径，并设定e轴单位为立方毫米
 *
 *    T<extruder> - Optional extruder number. Current extruder if omitted.   可选挤出机编号。如果省略当前挤出机。
 *    D<mm> - Diameter of the filament. Use "D0" to set units back to millimeters.  灯丝直径。使用"d 0"将单位设置为毫米。
 */
inline void gcode_M200() {

  if (setTargetedHotend(200)) return;

  if (code_seen('D')) {
    float diameter = code_value();
    // setting any extruder filament size disables volumetric on the assumption that    设定任何挤出机的灯丝尺寸使体积失效，前提是
    // slicers either generate in extruder values as cubic mm or as as filament feeds    切削器或在挤出机中生成立方毫米的数值，或作为长丝供料
    // for all extruders  适用于所有的入侵者
    volumetric_enabled = (diameter != 0.0);
    if (volumetric_enabled) {
      filament_size[target_extruder] = diameter;
      // make sure all extruders have some sane value for the filament size   一定要确保所有的挤出者对灯丝尺寸有合理的价值
      for (int i = 0; i < EXTRUDERS; i++)
        if (! filament_size[i]) filament_size[i] = DEFAULT_NOMINAL_FILAMENT_DIA;
    }
  }
  else {
    //reserved for setting filament diameter via UFID or filament measuring device  用于通过ufid或灯丝测量装置设置灯丝直径
    return;
  }
  calculate_volumetric_multipliers();
}

/**
 * M201: Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)  为打印移动设定最大加速度单位/秒^2（m201 x1000y1000）
 */
inline void gcode_M201() {
  for (int8_t i = 0; i < NUM_AXIS; i++) {
    if (code_seen(axis_codes[i])) {
      max_acceleration_units_per_sq_second[i] = code_value();
    }
  }
  // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
  //每平方秒的步骤需要更新，以与每平方秒的单位一致（因为它们是在计划中使用的）
  reset_acceleration_rates();
}

#if 0 // Not used for Sprinter/grbl gen6
  inline void gcode_M202() {
    for (int8_t i = 0; i < NUM_AXIS; i++) {
      if (code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
    }
  }
#endif


/**
 * M203: Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
 * 设定你的机器能在毫米/秒内维持的最大进料率(m 203 x200 y200 200 z300 e10000)
 */
inline void gcode_M203() {
  for (int8_t i = 0; i < NUM_AXIS; i++) {
    if (code_seen(axis_codes[i])) {
      max_feedrate[i] = code_value();
    }
  }
}

/**
 * M204: Set Accelerations in mm/sec^2 (M204 P1200 R3000 T3000)
 *
 *    P = Printing moves
 *    R = Retract only (no X, Y, Z) moves
 *    T = Travel (non printing) moves
 *
 *  Also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum feedrate
 *  还规定最低段时间在MS(b200000)，以防止缓冲区运行不足和m20最低输入率
 */
inline void gcode_M204() {
  if (code_seen('S')) {  // Kept for legacy compatibility. Should NOT BE USED for new developments.
    travel_acceleration = acceleration = code_value();
    SERIAL_ECHOPAIR("Setting Print and Travel Acceleration: ", acceleration);
    SERIAL_EOL;
  }
  if (code_seen('P')) {
    acceleration = code_value();
    SERIAL_ECHOPAIR("Setting Print Acceleration: ", acceleration);
    SERIAL_EOL;
  }
  if (code_seen('R')) {
    retract_acceleration = code_value();
    SERIAL_ECHOPAIR("Setting Retract Acceleration: ", retract_acceleration);
    SERIAL_EOL;
  }
  if (code_seen('T')) {
    travel_acceleration = code_value();
    SERIAL_ECHOPAIR("Setting Travel Acceleration: ", travel_acceleration);
    SERIAL_EOL;
  }
}

/**
 * M205: Set Advanced Settings
 *
 *    S = Min Feed Rate (mm/s)
 *    T = Min Travel Feed Rate (mm/s)
 *    B = Min Segment Time (µs)
 *    X = Max XY Jerk (mm/s/s)
 *    Z = Max Z Jerk (mm/s/s)
 *    E = Max E Jerk (mm/s/s)
 */
inline void gcode_M205() {
  if (code_seen('S')) minimumfeedrate = code_value();
  if (code_seen('T')) mintravelfeedrate = code_value();
  if (code_seen('B')) minsegmenttime = code_value();
  if (code_seen('X')) max_xy_jerk = code_value();
  if (code_seen('Z')) max_z_jerk = code_value();
  if (code_seen('E')) max_e_jerk = code_value();
}

/**
 * M206: Set Additional Homing Offset (X Y Z). SCARA aliases T=X, P=Y
 */
inline void gcode_M206() {
  for (int8_t i = X_AXIS; i <= Z_AXIS; i++) {
    if (code_seen(axis_codes[i])) {
      home_offset[i] = code_value();
    }
  }
  #if ENABLED(SCARA)
    if (code_seen('T')) home_offset[X_AXIS] = code_value(); // Theta
    if (code_seen('P')) home_offset[Y_AXIS] = code_value(); // Psi
  #endif
}

#if ENABLED(DELTA)
  /**
   * M665: Set delta configurations
   *
   *    L = diagonal rod
   *    R = delta radius
   *    S = segments per second
   *    A = Alpha (Tower 1) diagonal rod trim
   *    B = Beta (Tower 2) diagonal rod trim
   *    C = Gamma (Tower 3) diagonal rod trim
   */
  inline void gcode_M665() {
    if (code_seen('L')) delta_diagonal_rod = code_value();
    if (code_seen('R')) delta_radius = code_value();
    if (code_seen('S')) delta_segments_per_second = code_value();
    if (code_seen('A')) delta_diagonal_rod_trim_tower_1 = code_value();
    if (code_seen('B')) delta_diagonal_rod_trim_tower_2 = code_value();
    if (code_seen('C')) delta_diagonal_rod_trim_tower_3 = code_value();
    recalc_delta_settings(delta_radius, delta_diagonal_rod);
  }
  /**
   * M666: Set delta endstop adjustment  设置Delta终端调整
   */
  inline void gcode_M666() {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        SERIAL_ECHOLNPGM(">>> gcode_M666");
      }
    #endif
    for (int8_t i = X_AXIS; i <= Z_AXIS; i++) {
      if (code_seen(axis_codes[i])) {
        endstop_adj[i] = code_value();
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            SERIAL_ECHOPGM("endstop_adj[");
            SERIAL_ECHO(axis_codes[i]);
            SERIAL_ECHOPAIR("] = ", endstop_adj[i]);
            SERIAL_EOL;
          }
        #endif
      }
    }
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        SERIAL_ECHOLNPGM("<<< gcode_M666");
      }
    #endif
  }

#elif ENABLED(Z_DUAL_ENDSTOPS) // !DELTA && ENABLED(Z_DUAL_ENDSTOPS)

  /**
   * M666: For Z Dual Endstop setup, set z axis offset to the z2 axis.
   */
  inline void gcode_M666() {
    if (code_seen('Z')) z_endstop_adj = code_value();
    SERIAL_ECHOPAIR("Z Endstop Adjustment set to (mm):", z_endstop_adj);
    SERIAL_EOL;
  }

#endif // !DELTA && Z_DUAL_ENDSTOPS

#if ENABLED(FWRETRACT)

  /**
   * M207: Set firmware retraction values
   *
   *   S[+mm]    retract_length
   *   W[+mm]    retract_length_swap (multi-extruder)
   *   F[mm/min] retract_feedrate
   *   Z[mm]     retract_zlift
   */
  inline void gcode_M207() {
    if (code_seen('S')) retract_length = code_value();
    if (code_seen('F')) retract_feedrate = code_value() / 60;
    if (code_seen('Z')) retract_zlift = code_value();
    #if EXTRUDERS > 1
      if (code_seen('W')) retract_length_swap = code_value();
    #endif
  }

  /**
   * M208: Set firmware un-retraction values
   *
   *   S[+mm]    retract_recover_length (in addition to M207 S*)
   *   W[+mm]    retract_recover_length_swap (multi-extruder)
   *   F[mm/min] retract_recover_feedrate
   */
  inline void gcode_M208() {
    if (code_seen('S')) retract_recover_length = code_value();
    if (code_seen('F')) retract_recover_feedrate = code_value() / 60;
    #if EXTRUDERS > 1
      if (code_seen('W')) retract_recover_length_swap = code_value();
    #endif
  }

  /**
   * M209: Enable automatic retract (M209 S1)
   *       detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
   */
  inline void gcode_M209() {
    if (code_seen('S')) {
      int t = code_value_short();
      switch (t) {
        case 0:
          autoretract_enabled = false;
          break;
        case 1:
          autoretract_enabled = true;
          break;
        default:
          unknown_command_error();
          return;
      }
      for (int i = 0; i < EXTRUDERS; i++) retracted[i] = false;
    }
  }

#endif // FWRETRACT

#if EXTRUDERS > 1

  /**
   * M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
   */
  inline void gcode_M218() {
    if (setTargetedHotend(218)) return;

    if (code_seen('X')) extruder_offset[X_AXIS][target_extruder] = code_value();
    if (code_seen('Y')) extruder_offset[Y_AXIS][target_extruder] = code_value();

    #if ENABLED(DUAL_X_CARRIAGE)
      if (code_seen('Z')) extruder_offset[Z_AXIS][target_extruder] = code_value();
    #endif

    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
    for (int e = 0; e < EXTRUDERS; e++) {
      SERIAL_CHAR(' ');
      SERIAL_ECHO(extruder_offset[X_AXIS][e]);
      SERIAL_CHAR(',');
      SERIAL_ECHO(extruder_offset[Y_AXIS][e]);
      #if ENABLED(DUAL_X_CARRIAGE)
        SERIAL_CHAR(',');
        SERIAL_ECHO(extruder_offset[Z_AXIS][e]);
      #endif
    }
    SERIAL_EOL;
  }

#endif // EXTRUDERS > 1

/**
 * M220: Set speed percentage factor, aka "Feed Rate" (M220 S95)
 */
inline void gcode_M220() {
  if (code_seen('S')) feedrate_multiplier = code_value();
}

/**
 * M221: Set extrusion percentage (M221 T0 S95)
 */
inline void gcode_M221() {
  if (code_seen('S')) {
    int sval = code_value();
    if (setTargetedHotend(221)) return;
    extruder_multiplier[target_extruder] = sval;
  }
}

/**
 * M226: Wait until the specified pin reaches the state required (M226 P<pin> S<state>)
 */
inline void gcode_M226() {
  if (code_seen('P')) {
    int pin_number = code_value();

    int pin_state = code_seen('S') ? code_value() : -1; // required pin state - default is inverted

    if (pin_state >= -1 && pin_state <= 1) {

      for (uint8_t i = 0; i < COUNT(sensitive_pins); i++) {
        if (sensitive_pins[i] == pin_number) {
          pin_number = -1;
          break;
        }
      }

      if (pin_number > -1) {
        int target = LOW;

        st_synchronize();

        pinMode(pin_number, INPUT);

        switch (pin_state) {
          case 1:
            target = HIGH;
            break;

          case 0:
            target = LOW;
            break;

          case -1:
            target = !digitalRead(pin_number);
            break;
        }

        while (digitalRead(pin_number) != target) idle();

      } // pin_number > -1
    } // pin_state -1 0 1
  } // code_seen('P')
}

#if HAS_SERVOS

  /**
   * M280: Get or set servo position. P<index> S<angle>
   */
  inline void gcode_M280() {
    int servo_index = code_seen('P') ? code_value_short() : -1;
    int servo_position = 0;
    if (code_seen('S')) {
      servo_position = code_value_short();
      if (servo_index >= 0 && servo_index < NUM_SERVOS)
        servo[servo_index].move(servo_position);
      else {
        SERIAL_ECHO_START;
        SERIAL_ECHO("Servo ");
        SERIAL_ECHO(servo_index);
        SERIAL_ECHOLN(" out of range");
      }
    }
    else if (servo_index >= 0) {
      SERIAL_PROTOCOL(MSG_OK);
      SERIAL_PROTOCOL(" Servo ");
      SERIAL_PROTOCOL(servo_index);
      SERIAL_PROTOCOL(": ");
      SERIAL_PROTOCOL(servo[servo_index].read());
      SERIAL_EOL;
    }
  }

#endif // HAS_SERVOS

#if HAS_BUZZER

  /**
   * M300: Play beep sound S<frequency Hz> P<duration ms>
   */
  inline void gcode_M300() {
    uint16_t beepS = code_seen('S') ? code_value_short() : 110;
    uint32_t beepP = code_seen('P') ? code_value_long() : 1000;
    if (beepP > 5000) beepP = 5000; // limit to 5 seconds
    buzz(beepP, beepS);
  }

#endif // HAS_BUZZER

#if ENABLED(PIDTEMP)

  /**
   * M301: Set PID parameters P I D (and optionally C, L)
   *
   *   P[float] Kp term
   *   I[float] Ki term (unscaled)
   *   D[float] Kd term (unscaled)
   *
   * With PID_ADD_EXTRUSION_RATE:
   *
   *   C[float] Kc term
   *   L[float] LPQ length
   */
  inline void gcode_M301() {

    // multi-extruder PID patch: M301 updates or prints a single extruder's PID values
    // default behaviour (omitting E parameter) is to update for extruder 0 only
    int e = code_seen('E') ? code_value() : 0; // extruder being updated

    if (e < EXTRUDERS) { // catch bad input value
      if (code_seen('P')) PID_PARAM(Kp, e) = code_value();
      if (code_seen('I')) PID_PARAM(Ki, e) = scalePID_i(code_value());
      if (code_seen('D')) PID_PARAM(Kd, e) = scalePID_d(code_value());
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        if (code_seen('C')) PID_PARAM(Kc, e) = code_value();
        if (code_seen('L')) lpq_len = code_value();
        NOMORE(lpq_len, LPQ_MAX_LEN);
      #endif

      updatePID();
      SERIAL_PROTOCOL(MSG_OK);
      #if ENABLED(PID_PARAMS_PER_EXTRUDER)
        SERIAL_PROTOCOL(" e:"); // specify extruder in serial output   在串行输出中指定挤出机
        SERIAL_PROTOCOL(e);
      #endif // PID_PARAMS_PER_EXTRUDER
      SERIAL_PROTOCOL(" p:");
      SERIAL_PROTOCOL(PID_PARAM(Kp, e));
      SERIAL_PROTOCOL(" i:");
      SERIAL_PROTOCOL(unscalePID_i(PID_PARAM(Ki, e)));
      SERIAL_PROTOCOL(" d:");
      SERIAL_PROTOCOL(unscalePID_d(PID_PARAM(Kd, e)));
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        SERIAL_PROTOCOL(" c:");
        //Kc does not have scaling applied above, or in resetting defaults   kc没有以上应用的缩放，或在重置默认值中应用
        SERIAL_PROTOCOL(PID_PARAM(Kc, e));
      #endif
      SERIAL_EOL;
    }
    else {
      SERIAL_ECHO_START;
      SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
    }
  }

#endif // PIDTEMP

#if ENABLED(PIDTEMPBED)

  inline void gcode_M304() {
    if (code_seen('P')) bedKp = code_value();
    if (code_seen('I')) bedKi = scalePID_i(code_value());
    if (code_seen('D')) bedKd = scalePID_d(code_value());

    updatePID();
    SERIAL_PROTOCOL(MSG_OK);
    SERIAL_PROTOCOL(" p:");
    SERIAL_PROTOCOL(bedKp);
    SERIAL_PROTOCOL(" i:");
    SERIAL_PROTOCOL(unscalePID_i(bedKi));
    SERIAL_PROTOCOL(" d:");
    SERIAL_PROTOCOL(unscalePID_d(bedKd));
    SERIAL_EOL;
  }

#endif // PIDTEMPBED

#if defined(CHDK) || HAS_PHOTOGRAPH

  /**
   * M240: Trigger a camera by emulating a Canon RC-1
   *       See http://www.doc-diy.net/photo/rc-1_hacked/
   */
  inline void gcode_M240() {
    #ifdef CHDK

      OUT_WRITE(CHDK, HIGH);
      chdkHigh = millis();
      chdkActive = true;

    #elif HAS_PHOTOGRAPH

      const uint8_t NUM_PULSES = 16;
      const float PULSE_LENGTH = 0.01524;
      for (int i = 0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        _delay_ms(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        _delay_ms(PULSE_LENGTH);
      }
      delay(7.33);
      for (int i = 0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        _delay_ms(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        _delay_ms(PULSE_LENGTH);
      }

    #endif // !CHDK && HAS_PHOTOGRAPH
  }

#endif // CHDK || PHOTOGRAPH_PIN

#if ENABLED(HAS_LCD_CONTRAST)

  /**
   * M250: Read and optionally set the LCD contrast  读取并可选择设置lcd对比度
   */
  inline void gcode_M250() {
    if (code_seen('C')) lcd_setcontrast(code_value_short() & 0x3F);
    SERIAL_PROTOCOLPGM("lcd contrast value: ");
    SERIAL_PROTOCOL(lcd_contrast);
    SERIAL_EOL;
  }

#endif // HAS_LCD_CONTRAST

#if ENABLED(PREVENT_DANGEROUS_EXTRUDE)

  void set_extrude_min_temp(float temp) { extrude_min_temp = temp; }

  /**
   * M302: Allow cold extrudes, or set the minimum extrude S<temperature>.  允许冷挤出，或设置最小挤出量。
   */
  inline void gcode_M302() {
    set_extrude_min_temp(code_seen('S') ? code_value() : 0);
  }

#endif // PREVENT_DANGEROUS_EXTRUDE

/**
 * M303: PID relay autotune
 *       S<temperature> sets the target temperature. (default target temperature = 150C)
 *       E<extruder> (-1 for the bed)
 *       C<cycles>
 */
inline void gcode_M303() {
  int e = code_seen('E') ? code_value_short() : 0;
  int c = code_seen('C') ? code_value_short() : 5;
  float temp = code_seen('S') ? code_value() : (e < 0 ? 70.0 : 150.0);
  PID_autotune(temp, e, c);
}

#if ENABLED(SCARA)
  bool SCARA_move_to_cal(uint8_t delta_x, uint8_t delta_y) {
    //SoftEndsEnabled = false;              // Ignore soft endstops during calibration
    //SERIAL_ECHOLN(" Soft endstops disabled ");
    if (IsRunning()) {
      //gcode_get_destination(); // For X Y Z E F
      delta[X_AXIS] = delta_x;
      delta[Y_AXIS] = delta_y;
      calculate_SCARA_forward_Transform(delta);
      destination[X_AXIS] = delta[X_AXIS] / axis_scaling[X_AXIS];
      destination[Y_AXIS] = delta[Y_AXIS] / axis_scaling[Y_AXIS];
      prepare_move();
      //ok_to_send();
      return true;
    }
    return false;
  }

  /**
   * M360: SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
   */
  inline bool gcode_M360() {
    SERIAL_ECHOLN(" Cal: Theta 0 ");
    return SCARA_move_to_cal(0, 120);
  }

  /**
   * M361: SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
   */
  inline bool gcode_M361() {
    SERIAL_ECHOLN(" Cal: Theta 90 ");
    return SCARA_move_to_cal(90, 130);
  }

  /**
   * M362: SCARA calibration: Move to cal-position PsiA (0 deg calibration)
   */
  inline bool gcode_M362() {
    SERIAL_ECHOLN(" Cal: Psi 0 ");
    return SCARA_move_to_cal(60, 180);
  }

  /**
   * M363: SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
   */
  inline bool gcode_M363() {
    SERIAL_ECHOLN(" Cal: Psi 90 ");
    return SCARA_move_to_cal(50, 90);
  }

  /**
   * M364: SCARA calibration: Move to cal-position PSIC (90 deg to Theta calibration position)
   */
  inline bool gcode_M364() {
    SERIAL_ECHOLN(" Cal: Theta-Psi 90 ");
    return SCARA_move_to_cal(45, 135);
  }

  /**
   * M365: SCARA calibration: Scaling factor, X, Y, Z axis
   */
  inline void gcode_M365() {
    for (int8_t i = X_AXIS; i <= Z_AXIS; i++) {
      if (code_seen(axis_codes[i])) {
        axis_scaling[i] = code_value();
      }
    }
  }

#endif // SCARA

#if ENABLED(EXT_SOLENOID)

  void enable_solenoid(uint8_t num) {
    switch (num) {
      case 0:
        OUT_WRITE(SOL0_PIN, HIGH);
        break;
        #if HAS_SOLENOID_1
          case 1:
            OUT_WRITE(SOL1_PIN, HIGH);
            break;
        #endif
        #if HAS_SOLENOID_2
          case 2:
            OUT_WRITE(SOL2_PIN, HIGH);
            break;
        #endif
        #if HAS_SOLENOID_3
          case 3:
            OUT_WRITE(SOL3_PIN, HIGH);
            break;
        #endif
      default:
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM(MSG_INVALID_SOLENOID);
        break;
    }
  }

  void enable_solenoid_on_active_extruder() { enable_solenoid(active_extruder); }

  void disable_all_solenoids() {
    OUT_WRITE(SOL0_PIN, LOW);
    OUT_WRITE(SOL1_PIN, LOW);
    OUT_WRITE(SOL2_PIN, LOW);
    OUT_WRITE(SOL3_PIN, LOW);
  }

  /**
   * M380: Enable solenoid on the active extruder   启用活动挤出机上的螺线管
   */
  inline void gcode_M380() { enable_solenoid_on_active_extruder(); }

  /**
   * M381: Disable all solenoids
   */
  inline void gcode_M381() { disable_all_solenoids(); }

#endif // EXT_SOLENOID

/**
 * M400: Finish all moves
 */
inline void gcode_M400() { st_synchronize(); }

#if ENABLED(AUTO_BED_LEVELING_FEATURE) && DISABLED(Z_PROBE_SLED) && (HAS_SERVO_ENDSTOPS || ENABLED(Z_PROBE_ALLEN_KEY))

  /**
   * M401: Engage Z Servo endstop if available
   */
  inline void gcode_M401() {
    #if HAS_SERVO_ENDSTOPS
      raise_z_for_servo();
    #endif
    deploy_z_probe();
  }

  /**
   * M402: Retract Z Servo endstop if enabled
   */
  inline void gcode_M402() {
    #if HAS_SERVO_ENDSTOPS
      raise_z_for_servo();
    #endif
    stow_z_probe(false);
  }

#endif // AUTO_BED_LEVELING_FEATURE && (HAS_SERVO_ENDSTOPS || Z_PROBE_ALLEN_KEY) && !Z_PROBE_SLED

#if ENABLED(FILAMENT_SENSOR)

  /**
   * M404: Display or set the nominal filament width (3mm, 1.75mm ) W<3.0>
   */
  inline void gcode_M404() {
    #if HAS_FILWIDTH
      if (code_seen('W')) {
        filament_width_nominal = code_value();
      }
      else {
        SERIAL_PROTOCOLPGM("Filament dia (nominal mm):");
        SERIAL_PROTOCOLLN(filament_width_nominal);
      }
    #endif
  }

  /**
   * M405: Turn on filament sensor for control
   */
  inline void gcode_M405() {
    if (code_seen('D')) meas_delay_cm = code_value();
    if (meas_delay_cm > MAX_MEASUREMENT_DELAY) meas_delay_cm = MAX_MEASUREMENT_DELAY;

    if (delay_index2 == -1) { //initialize the ring buffer if it has not been done since startup
      int temp_ratio = widthFil_to_size_ratio();

      for (delay_index1 = 0; delay_index1 < MAX_MEASUREMENT_DELAY + 1; ++delay_index1)
        measurement_delay[delay_index1] = temp_ratio - 100;  //subtract 100 to scale within a signed byte

      delay_index1 = delay_index2 = 0;
    }

    filament_sensor = true;

    //SERIAL_PROTOCOLPGM("Filament dia (measured mm):");
    //SERIAL_PROTOCOL(filament_width_meas);
    //SERIAL_PROTOCOLPGM("Extrusion ratio(%):");
    //SERIAL_PROTOCOL(extruder_multiplier[active_extruder]);
  }

  /**
   * M406: Turn off filament sensor for control
   */
  inline void gcode_M406() { filament_sensor = false; }

  /**
   * M407: Get measured filament diameter on serial output
   */
  inline void gcode_M407() {
    SERIAL_PROTOCOLPGM("Filament dia (measured mm):");
    SERIAL_PROTOCOLLN(filament_width_meas);
  }

#endif // FILAMENT_SENSOR

/**
 * M410: Quickstop - Abort all planned moves
 *
 * This will stop the carriages mid-move, so most likely they
 * will be out of sync with the stepper position after this.
 */
inline void gcode_M410() { quickStop(); }


#if ENABLED(MESH_BED_LEVELING)

  /**
   * M420: Enable/Disable Mesh Bed Leveling
   */
  inline void gcode_M420() { if (code_seen('S') && code_has_value()) mbl.active = !!code_value_short(); }

  /**
   * M421: Set a single Mesh Bed Leveling Z coordinate
   */
  inline void gcode_M421() {
    float x, y, z;
    bool err = false, hasX, hasY, hasZ;
    if ((hasX = code_seen('X'))) x = code_value();
    if ((hasY = code_seen('Y'))) y = code_value();
    if ((hasZ = code_seen('Z'))) z = code_value();

    if (!hasX || !hasY || !hasZ) {
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM(MSG_ERR_M421_REQUIRES_XYZ);
      err = true;
    }

    if (x >= MESH_NUM_X_POINTS || y >= MESH_NUM_Y_POINTS) {
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM(MSG_ERR_MESH_INDEX_OOB);
      err = true;
    }

    if (!err) mbl.set_z(mbl.select_x_index(x), mbl.select_y_index(y), z);
  }

#endif

/**
 * M428: Set home_offset based on the distance between the
 *       current_position and the nearest "reference point."
 *       If an axis is past center its endstop position
 *       is the reference-point. Otherwise it uses 0. This allows
 *       the Z offset to be set near the bed when using a max endstop.
 *
 *       M428 can't be used more than 2cm away from 0 or an endstop.
 *
 *       Use M206 to set these values directly.
 */
inline void gcode_M428() {
  bool err = false;
  float new_offs[3], new_pos[3];
  memcpy(new_pos, current_position, sizeof(new_pos));
  memcpy(new_offs, home_offset, sizeof(new_offs));
  for (int8_t i = X_AXIS; i <= Z_AXIS; i++) {
    if (axis_known_position[i]) {
      float base = (new_pos[i] > (min_pos[i] + max_pos[i]) / 2) ? base_home_pos(i) : 0,
            diff = new_pos[i] - base;
      if (diff > -20 && diff < 20) {
        new_offs[i] -= diff;
        new_pos[i] = base;
      }
      else {
        SERIAL_ERROR_START;
        SERIAL_ERRORLNPGM(MSG_ERR_M428_TOO_FAR);
        LCD_ALERTMESSAGEPGM("Err: Too far!");
        #if HAS_BUZZER
          enqueuecommands_P(PSTR("M300 S40 P200"));
        #endif
        err = true;
        break;
      }
    }
  }

  if (!err) {
    memcpy(current_position, new_pos, sizeof(new_pos));
    memcpy(home_offset, new_offs, sizeof(new_offs));
    sync_plan_position();
    LCD_ALERTMESSAGEPGM("Offset applied.");
    #if HAS_BUZZER
      enqueuecommands_P(PSTR("M300 S659 P200\nM300 S698 P200"));
    #endif
  }
}

/**
 * M500: Store settings in EEPROM
 */
inline void gcode_M500() {
  Config_StoreSettings();
}

/**
 * M501: Read settings from EEPROM
 */
inline void gcode_M501() {
  Config_RetrieveSettings();
}

/**
 * M502: Revert to default settings
 */
inline void gcode_M502() {
  Config_ResetDefault();
}

/**
 * M503: print settings currently in memory
 */
inline void gcode_M503() {
  Config_PrintSettings(code_seen('S') && code_value() == 0);
}

#if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)

  /**
   * M540: Set whether SD card print should abort on endstop hit (M540 S<0|1>)  设定sd卡打印是否应在终端命中时中止（m540s）
   */
  inline void gcode_M540() {
    if (code_seen('S')) abort_on_endstop_hit = (code_value() > 0);
  }

#endif // ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED

#ifdef CUSTOM_M_CODE_SET_Z_PROBE_OFFSET

  inline void gcode_SET_Z_PROBE_OFFSET() {

    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_ZPROBE_ZOFFSET);
    SERIAL_CHAR(' ');

    if (code_seen('Z')) {
      float value = code_value();
      if (Z_PROBE_OFFSET_RANGE_MIN <= value && value <= Z_PROBE_OFFSET_RANGE_MAX) {
        zprobe_zoffset = value;
        SERIAL_ECHOPGM(MSG_OK);
      }
      else {
        SERIAL_ECHOPGM(MSG_Z_MIN);
        SERIAL_ECHO(Z_PROBE_OFFSET_RANGE_MIN);
        SERIAL_ECHOPGM(MSG_Z_MAX);
        SERIAL_ECHO(Z_PROBE_OFFSET_RANGE_MAX);
      }
    }
    else {
      SERIAL_ECHOPAIR(": ", zprobe_zoffset);
    }

    SERIAL_EOL;
  }
 
#endif // CUSTOM_M_CODE_SET_Z_PROBE_OFFSET   自定义m码集z探针偏移量

#if ENABLED(FILAMENTCHANGEENABLE)

  /**
   * M600: Pause for filament change
   *
   *  E[distance] - Retract the filament this far (negative value)
   *  Z[distance] - Move the Z axis by this distance
   *  X[position] - Move to this X position, with Y
   *  Y[position] - Move to this Y position, with X
   *  L[distance] - Retract distance for removal (manual reload)
   *
   *  Default values are used for omitted arguments.
   *
   */
  inline void gcode_M600() {

    if (degHotend(active_extruder) < extrude_min_temp) {
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM(MSG_TOO_COLD_FOR_M600);
      return;
    }

    float lastpos[NUM_AXIS], fr60 = feedrate / 60;

    for (int i = 0; i < NUM_AXIS; i++)
      lastpos[i] = destination[i] = current_position[i];

    #if ENABLED(DELTA)
      #define RUNPLAN calculate_delta(destination); \
                      plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], destination[E_AXIS], fr60, active_extruder);
    #else
      #define RUNPLAN line_to_destination();
    #endif

    //retract by E
    if (code_seen('E')) destination[E_AXIS] += code_value();
    #ifdef FILAMENTCHANGE_FIRSTRETRACT
      else destination[E_AXIS] += FILAMENTCHANGE_FIRSTRETRACT;
    #endif

    RUNPLAN;

    //lift Z
    if (code_seen('Z')) destination[Z_AXIS] += code_value();
    #ifdef FILAMENTCHANGE_ZADD
      else destination[Z_AXIS] += FILAMENTCHANGE_ZADD;
    #endif

    RUNPLAN;

    //move xy
    if (code_seen('X')) destination[X_AXIS] = code_value();
    #ifdef FILAMENTCHANGE_XPOS
      else destination[X_AXIS] = FILAMENTCHANGE_XPOS;
    #endif

    if (code_seen('Y')) destination[Y_AXIS] = code_value();
    #ifdef FILAMENTCHANGE_YPOS
      else destination[Y_AXIS] = FILAMENTCHANGE_YPOS;
    #endif

    RUNPLAN;

    if (code_seen('L')) destination[E_AXIS] += code_value();
    #ifdef FILAMENTCHANGE_FINALRETRACT
      else destination[E_AXIS] += FILAMENTCHANGE_FINALRETRACT;
    #endif

    RUNPLAN;

    //finish moves
    st_synchronize();
    //disable extruder steppers so filament can be removed
    disable_e0();
    disable_e1();
    disable_e2();
    disable_e3();
    delay(100);
    LCD_ALERTMESSAGEPGM(MSG_FILAMENTCHANGE);
    millis_t next_tick = 0;
    while (!lcd_clicked()) {
      #if DISABLED(AUTO_FILAMENT_CHANGE)
        millis_t ms = millis();
        if (ms >= next_tick) {
          lcd_quick_feedback();
          next_tick = ms + 2500; // feedback every 2.5s while waiting
        }
        manage_heater();
        manage_inactivity(true);
        lcd_update();
      #else
        current_position[E_AXIS] += AUTO_FILAMENT_CHANGE_LENGTH;
        destination[E_AXIS] = current_position[E_AXIS];
        line_to_destination(AUTO_FILAMENT_CHANGE_FEEDRATE);
        st_synchronize();
      #endif
    } // while(!lcd_clicked)
    lcd_quick_feedback(); // click sound feedback

    #if ENABLED(AUTO_FILAMENT_CHANGE)
      current_position[E_AXIS] = 0;
      st_synchronize();
    #endif

    //return to normal
    if (code_seen('L')) destination[E_AXIS] -= code_value();
    #ifdef FILAMENTCHANGE_FINALRETRACT
      else destination[E_AXIS] -= FILAMENTCHANGE_FINALRETRACT;
    #endif

    current_position[E_AXIS] = destination[E_AXIS]; //the long retract of L is compensated by manual filament feeding L的长缩回是通过人工喂养长丝来补偿的
    plan_set_e_position(current_position[E_AXIS]);

    RUNPLAN; //should do nothing  什么都不该做

    lcd_reset_alert_level();

    #if ENABLED(DELTA)
      // Move XYZ to starting position, then E  将xyz移动到起始位置，然后
      calculate_delta(lastpos);
      plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], destination[E_AXIS], fr60, active_extruder);
      plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], lastpos[E_AXIS], fr60, active_extruder);
    #else
      // Move XY to starting position, then Z, then E  将xy移动到起始位置，然后z，然后e
      destination[X_AXIS] = lastpos[X_AXIS];
      destination[Y_AXIS] = lastpos[Y_AXIS];
      line_to_destination();
      destination[Z_AXIS] = lastpos[Z_AXIS];
      line_to_destination();
      destination[E_AXIS] = lastpos[E_AXIS];
      line_to_destination();
    #endif

    #if ENABLED(FILAMENT_RUNOUT_SENSOR)
      filrunoutEnqueued = false;
    #endif

  }

#endif // FILAMENTCHANGEENABLE

#if ENABLED(DUAL_X_CARRIAGE)

  /** 
   * M605: Set dual x-carriage movement mode   设置双x-car移动模式
   *
   *    M605 S0: Full control mode. The slicer has full control over x-carriage movement
   *    M605 S1: Auto-park mode. The inactive head will auto park/unpark without slicer involvement
   *    M605 S2 [Xnnn] [Rmmm]: Duplication mode. The second extruder will duplicate the first with nnn
   *                         millimeters x-offset and an optional differential hotend temperature of
   *                         mmm degrees. E.g., with "M605 S2 X100 R2" the second extruder will duplicate
   *                         the first with a spacing of 100mm in the x direction and 2 degrees hotter.
   *
   *    Note: the X axis should be homed after changing dual x-carriage mode.  第一个在X方向的间距为100mm，较热2度。
   */
  inline void gcode_M605() {
    st_synchronize();
    if (code_seen('S')) dual_x_carriage_mode = code_value();
    switch (dual_x_carriage_mode) {
      case DXC_DUPLICATION_MODE:
        if (code_seen('X')) duplicate_extruder_x_offset = max(code_value(), X2_MIN_POS - x_home_pos(0));
        if (code_seen('R')) duplicate_extruder_temp_offset = code_value();
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
        SERIAL_CHAR(' ');
        SERIAL_ECHO(extruder_offset[X_AXIS][0]);
        SERIAL_CHAR(',');
        SERIAL_ECHO(extruder_offset[Y_AXIS][0]);
        SERIAL_CHAR(' ');
        SERIAL_ECHO(duplicate_extruder_x_offset);
        SERIAL_CHAR(',');
        SERIAL_ECHOLN(extruder_offset[Y_AXIS][1]);
        break;
      case DXC_FULL_CONTROL_MODE:
      case DXC_AUTO_PARK_MODE:
        break;
      default:
        dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
        break;
    }
    active_extruder_parked = false;
    extruder_duplication_enabled = false;
    delayed_move_time = 0;
  }

#endif // DUAL_X_CARRIAGE

/**
 * M907: Set digital trimpot motor current using axis codes X, Y, Z, E, B, S
 */
inline void gcode_M907() {
  #if HAS_DIGIPOTSS
    for (int i = 0; i < NUM_AXIS; i++)
      if (code_seen(axis_codes[i])) digipot_current(i, code_value());
    if (code_seen('B')) digipot_current(4, code_value());
    if (code_seen('S')) for (int i = 0; i <= 4; i++) digipot_current(i, code_value());
  #endif
  #ifdef MOTOR_CURRENT_PWM_XY_PIN
    if (code_seen('X')) digipot_current(0, code_value());
  #endif
  #ifdef MOTOR_CURRENT_PWM_Z_PIN
    if (code_seen('Z')) digipot_current(1, code_value());
  #endif
  #ifdef MOTOR_CURRENT_PWM_E_PIN
    if (code_seen('E')) digipot_current(2, code_value());
  #endif
  #if ENABLED(DIGIPOT_I2C)
    // this one uses actual amps in floating point
    for (int i = 0; i < NUM_AXIS; i++) if (code_seen(axis_codes[i])) digipot_i2c_set_current(i, code_value());
    // for each additional extruder (named B,C,D,E..., channels 4,5,6,7...)
    for (int i = NUM_AXIS; i < DIGIPOT_I2C_NUM_CHANNELS; i++) if (code_seen('B' + i - NUM_AXIS)) digipot_i2c_set_current(i, code_value());
  #endif
}

#if HAS_DIGIPOTSS

  /**
   * M908: Control digital trimpot directly (M908 P<pin> S<current>)
   */
  inline void gcode_M908() {
    digitalPotWrite(
      code_seen('P') ? code_value() : 0,
      code_seen('S') ? code_value() : 0
    );
  }

#endif // HAS_DIGIPOTSS

#if HAS_MICROSTEPS

  // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
  //m350设置微步进模式。警告：单位步数不变。s的代码为所有驱动程序设置步进模式。
  inline void gcode_M350() {
    if (code_seen('S')) for (int i = 0; i <= 4; i++) microstep_mode(i, code_value());
    for (int i = 0; i < NUM_AXIS; i++) if (code_seen(axis_codes[i])) microstep_mode(i, (uint8_t)code_value());
    if (code_seen('B')) microstep_mode(4, code_value());
    microstep_readings();
  }

  /**
   * M351: Toggle MS1 MS2 pins directly with axis codes X Y Z E B
   *       S# determines MS1 or MS2, X# sets the pin high/low.
   */
  inline void gcode_M351() {
    if (code_seen('S')) switch (code_value_short()) {
      case 1:
        for (int i = 0; i < NUM_AXIS; i++) if (code_seen(axis_codes[i])) microstep_ms(i, code_value(), -1);
        if (code_seen('B')) microstep_ms(4, code_value(), -1);
        break;
      case 2:
        for (int i = 0; i < NUM_AXIS; i++) if (code_seen(axis_codes[i])) microstep_ms(i, -1, code_value());
        if (code_seen('B')) microstep_ms(4, -1, code_value());
        break;
    }
    microstep_readings();
  }

#endif // HAS_MICROSTEPS

/**
 * M999: Restart after being stopped  停止后重新启动
 */
inline void gcode_M999() {
  Running = true;
  lcd_reset_alert_level();
  // gcode_LastN = Stopped_gcode_LastN;
  FlushSerialRequestResend();
}

/**
 * T0-T3: Switch tool, usually switching extruders  开关工具，通常是开关推进器
 *
 *   F[mm/min] Set the movement feedrate  设置运动输入率
 */
inline void gcode_T(uint8_t tmp_extruder) {
  if (tmp_extruder >= EXTRUDERS) {
    SERIAL_ECHO_START;
    SERIAL_CHAR('T');
    SERIAL_PROTOCOL_F(tmp_extruder, DEC);
    SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
  }
  else {
    target_extruder = tmp_extruder;

    #if EXTRUDERS > 1
      bool make_move = false;
    #endif

    if (code_seen('F')) {

      #if EXTRUDERS > 1
        make_move = true;
      #endif

      float next_feedrate = code_value();
      if (next_feedrate > 0.0) feedrate = next_feedrate;
    }
    #if EXTRUDERS > 1
      if (tmp_extruder != active_extruder) {
        // Save current position to return to after applying extruder offset  在使用挤出机偏置后保存当前位置以返回
        set_destination_to_current();
        #if ENABLED(DUAL_X_CARRIAGE)
          if (dual_x_carriage_mode == DXC_AUTO_PARK_MODE && IsRunning() &&
              (delayed_move_time != 0 || current_position[X_AXIS] != x_home_pos(active_extruder))) {
            // Park old head: 1) raise 2) move to park position 3) lower
            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT,
                             current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
            plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT,
                             current_position[E_AXIS], max_feedrate[X_AXIS], active_extruder);
            plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS],
                             current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
            st_synchronize();
          }

          // apply Y & Z extruder offset (x offset is already used in determining home pos) 应用y&z挤出机偏移量（X偏移量已用于确定家用POS)
          current_position[Y_AXIS] = current_position[Y_AXIS] -
                                     extruder_offset[Y_AXIS][active_extruder] +
                                     extruder_offset[Y_AXIS][tmp_extruder];
          current_position[Z_AXIS] = current_position[Z_AXIS] -
                                     extruder_offset[Z_AXIS][active_extruder] +
                                     extruder_offset[Z_AXIS][tmp_extruder];
          active_extruder = tmp_extruder;

          // This function resets the max/min values - the current position may be overwritten below.此函数重置最大值/最小值-当前位置可能在下面被覆盖。
          set_axis_is_at_home(X_AXIS);

          if (dual_x_carriage_mode == DXC_FULL_CONTROL_MODE) {
            current_position[X_AXIS] = inactive_extruder_x_pos;
            inactive_extruder_x_pos = destination[X_AXIS];
          }
          else if (dual_x_carriage_mode == DXC_DUPLICATION_MODE) {
            active_extruder_parked = (active_extruder == 0); // this triggers the second extruder to move into the duplication position 这会触发第二个挤出机移动到复制位置
            if (active_extruder == 0 || active_extruder_parked)
              current_position[X_AXIS] = inactive_extruder_x_pos;
            else
              current_position[X_AXIS] = destination[X_AXIS] + duplicate_extruder_x_offset;
            inactive_extruder_x_pos = destination[X_AXIS];
            extruder_duplication_enabled = false;
          }
          else {
            // record raised toolhead position for use by unpark  记录已提升的工具头位置，以供unpark使用
            memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
            raised_parked_position[Z_AXIS] += TOOLCHANGE_UNPARK_ZLIFT;
            active_extruder_parked = true;
            delayed_move_time = 0;
          }
        #else // !DUAL_X_CARRIAGE  双x运输
          // Offset extruder (only by XY)
          for (int i = X_AXIS; i <= Y_AXIS; i++)
            current_position[i] += extruder_offset[i][tmp_extruder] - extruder_offset[i][active_extruder];
          // Set the new active extruder and position  设置新的活动挤出机和位置
          active_extruder = tmp_extruder;
        #endif // !DUAL_X_CARRIAGE
        #if ENABLED(DELTA)
          sync_plan_position_delta();
        #else
          sync_plan_position();
        #endif
        // Move to the old position if 'F' was in the parameters  如果“f”在参数中，则移动到旧位置
        if (make_move && IsRunning()) prepare_move();
      }

      #if ENABLED(EXT_SOLENOID)
        st_synchronize();
        disable_all_solenoids();
        enable_solenoid_on_active_extruder();
      #endif // EXT_SOLENOID

    #endif // EXTRUDERS > 1
    SERIAL_ECHO_START;
    SERIAL_ECHO(MSG_ACTIVE_EXTRUDER);
    SERIAL_PROTOCOLLN((int)active_extruder);
  }
}

/**
 * Process a single command and dispatch it to its handler 处理单个命令并将其发送给其处理程序
 * This is called from the main loop()   *这是从主循环()调用的
 */
void process_next_command() {
  current_command = command_queue[cmd_queue_index_r];

  if ((marlin_debug_flags & DEBUG_ECHO)) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLN(current_command);
  }

  // Sanitize the current command: 对当前命令进行消毒：
  //  - Skip leading spaces
  //  - Bypass N[0-9][0-9]*[ ]*
  //  - Overwrite * with nul to mark the end
  while (*current_command == ' ') ++current_command;
  if (*current_command == 'N' && ((current_command[1] >= '0' && current_command[1] <= '9') || current_command[1] == '-')) {
    current_command += 2; // skip N[-0-9]
    while (*current_command >= '0' && *current_command <= '9') ++current_command; // skip [0-9]*
    while (*current_command == ' ') ++current_command; // skip [ ]*
  }
  char* starpos = strchr(current_command, '*');  // * should always be the last parameter 他总是最后一个参数
  if (starpos) while (*starpos == ' ' || *starpos == '*') *starpos-- = '\0'; // nullify '*' and ' '

  // Get the command code, which must be G, M, or T  得到命令代码，它必须是g、m或t
  char command_code = *current_command;

  // The code must have a numeric value 代码必须具有数值
  bool code_is_good = (current_command[1] >= '0' && current_command[1] <= '9');

  int codenum; // define ahead of goto  定义优先于政府

  // Bail early if there's no code  如果没有密码就提前保释
  if (!code_is_good) goto ExitUnknownCommand;

  // Args pointer optimizes code_seen, especially those taking XYZEF   args指针优化了代码显示，特别是那些使用XYZEF的
  // This wastes a little cpu on commands that expect no arguments.  这会在不需要参数的命令上浪费一点CPU。
  current_command_args = current_command;
  while (*current_command_args && *current_command_args != ' ') ++current_command_args;
  while (*current_command_args == ' ') ++current_command_args;

  // Interpret the code int  解释代码INT
  seen_pointer = current_command;
  codenum = code_value_short();

  // Handle a known G, M, or T  处理已知的g、m或t
  switch (command_code) {
    case 'G': switch (codenum) {

      // G0, G1
      case 0:
      case 1:
        gcode_G0_G1();
        break;

      // G2, G3
      #if DISABLED(SCARA)
        case 2: // G2  - CW ARC
        case 3: // G3  - CCW ARC
          gcode_G2_G3(codenum == 2);
          break;
      #endif

      // G4 Dwell
      case 4:
        gcode_G4();
        break;
       

      #if ENABLED(FWRETRACT)

        case 10: // G10: retract
        case 11: // G11: retract_recover
          gcode_G10_G11(codenum == 10);
          break;
        

      #endif //FWRETRACT

      case 28: // G28: Home all axes, one at a time  G28：家庭所有轴，一次一个
        gcode_G28(false);
        set_bed_leveling_enabled(true);    ////***//保存自动调平数据
        break;

      #if ENABLED(AUTO_BED_LEVELING_FEATURE) || ENABLED(MESH_BED_LEVELING)
        case 29: // G29 Detailed Z probe, probes the bed at 3 or more points.  G29详细的z探针，在3个或更多点探测床。
          gcode_G29();
          break;
      #endif

      #if ENABLED(AUTO_BED_LEVELING_FEATURE)

        #if DISABLED(Z_PROBE_SLED)

          case 30: // G30 Single Z probe
            gcode_G30();
            break;

        #else // Z_PROBE_SLED

            case 31: // G31: dock the sled
            case 32: // G32: undock the sled
              dock_sled(codenum == 31);
              break;

        #endif // Z_PROBE_SLED

      #endif // AUTO_BED_LEVELING_FEATURE  自动床平整功能

      case 90: // G90
        relative_mode = false;
        break;
      case 91: // G91
        relative_mode = true;
        break;

      case 92: // G92
        gcode_G92();
        break;
    }
    break;

    case 'M': switch (codenum) {
      #if ENABLED(ULTIPANEL)
        case 0: // M0 - Unconditional stop - Wait for user button press on LCD
        case 1: // M1 - Conditional stop - Wait for user button press on LCD
          gcode_M0_M1();
          break;
      #endif // ULTIPANEL

      case 17:
        gcode_M17();
     /*   break;                             ////控制灯模块代码
        
      case 180://M180 LED_PIN  on 
     digitalWrite(LED_PIN,HIGH);
     delay(1000);
          break;
     
      case 181://M180 LED_PIN  off 
     digitalWrite(LED_PIN,LOW);*/
          break;

      #if ENABLED(SDSUPPORT)

        case 20: // M20 - list SD card
          gcode_M20(); break;
        case 21: // M21 - init SD card
          gcode_M21(); break;
        case 22: //M22 - release SD card
          gcode_M22(); break;
        case 23: //M23 - Select file
          gcode_M23(); break;
        case 24: //M24 - Start SD print
          gcode_M24(); break;
        case 25: //M25 - Pause SD print
          gcode_M25(); break;
        case 26: //M26 - Set SD index
          gcode_M26(); break;
        case 27: //M27 - Get SD status
          gcode_M27(); break;
        case 28: //M28 - Start SD write
          gcode_M28(); break;
        case 29: //M29 - Stop SD write
          gcode_M29(); break;
        case 30: //M30 <filename> Delete File
          gcode_M30(); break;
        case 32: //M32 - Select file and start SD print  选择文件并启动sd打印
          gcode_M32(); break;

        #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
          case 33: //M33 - Get the long full path to a file or folder  获取文件或文件夹的长完整路径
            gcode_M33(); break;
        #endif // LONG_FILENAME_HOST_SUPPORT

        case 928: //M928 - Start SD write
          gcode_M928(); break;

      #endif //SDSUPPORT

      case 31: //M31 take time since the start of the SD print or an M109 command  m31自sd打印或m109命令开始后需要时间
        gcode_M31();
        break;

      case 42: //M42 -Change pin status via gcode
        gcode_M42();
        break;

      #if ENABLED(AUTO_BED_LEVELING_FEATURE) && ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)
        case 48: // M48 Z probe repeatability   m 48z探针的可重复性
          gcode_M48();
          break;
      #endif // AUTO_BED_LEVELING_FEATURE && Z_MIN_PROBE_REPEATABILITY_TEST   //自动床平整特性&z_min探针可重复性试验

      #if ENABLED(M100_FREE_MEMORY_WATCHER)
        case 100:
          gcode_M100();
          break;
      #endif

      case 104: // M104
        gcode_M104();
        break;

      case 111: // M111: Set debug level
        gcode_M111();
        break;

      case 112: // M112: Emergency Stop
        gcode_M112();
        break;

      case 140: // M140: Set bed temp
        gcode_M140();
        break;

      case 105: // M105: Read current temperature
        gcode_M105();
        return; // "ok" already printed

      case 109: // M109: Wait for temperature
        gcode_M109();
        break;

      #if HAS_TEMP_BED
        case 190: // M190: Wait for bed heater to reach target
          gcode_M190();
          break;
      #endif // HAS_TEMP_BED

      #if HAS_FAN
        case 106: // M106: Fan On
          gcode_M106();
          break;
        case 107: // M107: Fan Off
          gcode_M107();
          break;
      #endif // HAS_FAN

      #if ENABLED(BARICUDA)
        // PWM for HEATER_1_PIN
        #if HAS_HEATER_1
          case 126: // M126: valve open
            gcode_M126();
            break;
          case 127: // M127: valve closed
            gcode_M127();
            break;
        #endif // HAS_HEATER_1

        // PWM for HEATER_2_PIN
        #if HAS_HEATER_2
          case 128: // M128: valve open
            gcode_M128();
            break;
          case 129: // M129: valve closed
            gcode_M129();
            break;
        #endif // HAS_HEATER_2
      #endif // BARICUDA

      #if HAS_POWER_SWITCH

        case 80: // M80: Turn on Power Supply
          gcode_M80();
          break;

      #endif // HAS_POWER_SWITCH

      case 81: // M81: Turn off Power, including Power Supply, if possible
        gcode_M81();
        break;

      case 82:
        gcode_M82();
        break;
      case 83:
        gcode_M83();
        break;
      case 18: // (for compatibility)
      case 84: // M84
        gcode_M18_M84();
        break;
      case 85: // M85
        gcode_M85();
        break;
      case 92: // M92: Set the steps-per-unit for one or more axes
        gcode_M92();
        break;
      case 115: // M115: Report capabilities
        gcode_M115();
        break;
      case 117: // M117: Set LCD message text, if possible
        gcode_M117();
        break;
      case 114: // M114: Report current position
        gcode_M114();
        break;
      case 120: // M120: Enable endstops
        gcode_M120();
        break;
      case 121: // M121: Disable endstops
        gcode_M121();
        break;
      case 119: // M119: Report endstop states
        gcode_M119();
        break;

      #if ENABLED(ULTIPANEL)

        case 145: // M145: Set material heatup parameters
          gcode_M145();
          break;

      #endif

      #if ENABLED(BLINKM)

        case 150: // M150
          gcode_M150();
          break;

      #endif //BLINKM

      case 200: // M200 D<millimeters> set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).
                //m200d设置灯丝直径，并设置e轴单位为立方毫米（使用s0设置为毫米）
        gcode_M200();
        break;
      case 201: // M201
        gcode_M201();
        break;
      #if 0 // Not used for Sprinter/grbl gen6
        case 202: // M202
          gcode_M202();
          break;
      #endif
      case 203: // M203 max feedrate mm/sec
        gcode_M203();
        break;
      case 204: // M204 acclereration S normal moves T filmanent only moves
        gcode_M204();
        break;
      case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
                //m 205高级设置：最小行程速度s=打印时仅t=行程，b=最小段时间x=最大XY挺举，z=最大z挺举
        gcode_M205();
        break;
      case 206: // M206 additional homing offset
        gcode_M206();
        break;

      #if ENABLED(DELTA)
        case 665: // M665 set delta configurations L<diagonal_rod> R<delta_radius> S<segments_per_sec>
          gcode_M665();
          break;
      #endif

      #if ENABLED(DELTA) || ENABLED(Z_DUAL_ENDSTOPS)
        case 666: // M666 set delta / dual endstop adjustment
          gcode_M666();
          break;
      #endif

      #if ENABLED(FWRETRACT)
        case 207: //M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop]
          gcode_M207();
          break;
        case 208: // M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/min]
          gcode_M208();
          break;
        case 209: // M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
          gcode_M209();
          break;
      #endif // FWRETRACT

      #if EXTRUDERS > 1
        case 218: // M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
          gcode_M218();
          break;
      #endif

      case 220: // M220 S<factor in percent>- set speed factor override percentage
        gcode_M220();
        break;

      case 221: // M221 S<factor in percent>- set extrude factor override percentage
        gcode_M221();
        break;

      case 226: // M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
        gcode_M226();
        break;

      #if HAS_SERVOS
        case 280: // M280 - set servo position absolute. P: servo index, S: angle or microseconds
          gcode_M280();
          break;
      #endif // HAS_SERVOS

      #if HAS_BUZZER
        case 300: // M300 - Play beep tone
          gcode_M300();
          break;
      #endif // HAS_BUZZER

      #if ENABLED(PIDTEMP)
        case 301: // M301
          gcode_M301();
          break;
      #endif // PIDTEMP

      #if ENABLED(PIDTEMPBED)
        case 304: // M304
          gcode_M304();
          break;
      #endif // PIDTEMPBED

      #if defined(CHDK) || HAS_PHOTOGRAPH
        case 240: // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
          gcode_M240();
          break;
      #endif // CHDK || PHOTOGRAPH_PIN

      #if ENABLED(HAS_LCD_CONTRAST)
        case 250: // M250  Set LCD contrast value: C<value> (value 0..63)
          gcode_M250();
          break;
      #endif // HAS_LCD_CONTRAST

      #if ENABLED(PREVENT_DANGEROUS_EXTRUDE)
        case 302: // allow cold extrudes, or set the minimum extrude temperature
          gcode_M302();
          break;
      #endif // PREVENT_DANGEROUS_EXTRUDE

      case 303: // M303 PID autotune
        gcode_M303();
        break;

      #if ENABLED(SCARA)
        case 360:  // M360 SCARA Theta pos1
          if (gcode_M360()) return;
          break;
        case 361:  // M361 SCARA Theta pos2
          if (gcode_M361()) return;
          break;
        case 362:  // M362 SCARA Psi pos1
          if (gcode_M362()) return;
          break;
        case 363:  // M363 SCARA Psi pos2
          if (gcode_M363()) return;
          break;
        case 364:  // M364 SCARA Psi pos3 (90 deg to Theta)
          if (gcode_M364()) return;
          break;
        case 365: // M365 Set SCARA scaling for X Y Z
          gcode_M365();
          break;
      #endif // SCARA

      case 400: // M400 finish all moves
        gcode_M400();
        break;

      #if ENABLED(AUTO_BED_LEVELING_FEATURE) && (HAS_SERVO_ENDSTOPS || ENABLED(Z_PROBE_ALLEN_KEY)) && DISABLED(Z_PROBE_SLED)
        case 401:
          gcode_M401();
          break;
        case 402:
          gcode_M402();
          break;
      #endif // AUTO_BED_LEVELING_FEATURE && (HAS_SERVO_ENDSTOPS || Z_PROBE_ALLEN_KEY) && !Z_PROBE_SLED

      #if ENABLED(FILAMENT_SENSOR)
        case 404:  //M404 Enter the nominal filament width (3mm, 1.75mm ) N<3.0> or display nominal filament width
          gcode_M404();
          break;
        case 405:  //M405 Turn on filament sensor for control
          gcode_M405();
          break;
        case 406:  //M406 Turn off filament sensor for control
          gcode_M406();
          break;
        case 407:   //M407 Display measured filament diameter
          gcode_M407();
          break;
      #endif // FILAMENT_SENSOR

      case 410: // M410 quickstop - Abort all the planned moves.
        gcode_M410();
        break;

      #if ENABLED(MESH_BED_LEVELING)
        case 420: // M420 Enable/Disable Mesh Bed Leveling
          gcode_M420();
          break;
        case 421: // M421 Set a Mesh Bed Leveling Z coordinate
          gcode_M421();
          break;
      #endif

      case 428: // M428 Apply current_position to home_offset
        gcode_M428();
        break;

      case 500: // M500 Store settings in EEPROM
        gcode_M500();
        break;
      case 501: // M501 Read settings from EEPROM
        gcode_M501();
        break;
      case 502: // M502 Revert to default settings
        gcode_M502();
        break;
      case 503: // M503 print settings currently in memory
        gcode_M503();
        break;

      #if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
        case 540:
          gcode_M540();
          break;
      #endif

      #ifdef CUSTOM_M_CODE_SET_Z_PROBE_OFFSET
        case CUSTOM_M_CODE_SET_Z_PROBE_OFFSET:
          gcode_SET_Z_PROBE_OFFSET();
          break;
      #endif // CUSTOM_M_CODE_SET_Z_PROBE_OFFSET

      #if ENABLED(FILAMENTCHANGEENABLE)
        case 600: //Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
          gcode_M600();
          break;
      #endif // FILAMENTCHANGEENABLE

      #if ENABLED(DUAL_X_CARRIAGE)
        case 605:
          gcode_M605();
          break;
      #endif // DUAL_X_CARRIAGE

      case 907: // M907 Set digital trimpot motor current using axis codes.
        gcode_M907();
        break;

      #if HAS_DIGIPOTSS
        case 908: // M908 Control digital trimpot directly.
          gcode_M908();
          break;
      #endif // HAS_DIGIPOTSS

      #if HAS_MICROSTEPS

        case 350: // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
          gcode_M350();
          break;

        case 351: // M351 Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
          gcode_M351();
          break;

      #endif // HAS_MICROSTEPS

      case 999: // M999: Restart after being Stopped
        gcode_M999();
        break;
    }
    break;

    case 'T':
      gcode_T(codenum);
      break;

    default: code_is_good = false;
  }

ExitUnknownCommand:

  // Still unknown command? Throw an error
  if (!code_is_good) unknown_command_error();

  ok_to_send();
}

void FlushSerialRequestResend() {
  //char command_queue[cmd_queue_index_r][100]="Resend:";
  MYSERIAL.flush();
  SERIAL_PROTOCOLPGM(MSG_RESEND);
  SERIAL_PROTOCOLLN(gcode_LastN + 1);
  ok_to_send();
}

void ok_to_send() {
  refresh_cmd_timeout();
  #if ENABLED(SDSUPPORT)
    if (fromsd[cmd_queue_index_r]) return;
  #endif
  SERIAL_PROTOCOLPGM(MSG_OK);
  #if ENABLED(ADVANCED_OK)
    SERIAL_PROTOCOLPGM(" N"); SERIAL_PROTOCOL(gcode_LastN);
    SERIAL_PROTOCOLPGM(" P"); SERIAL_PROTOCOL(int(BLOCK_BUFFER_SIZE - movesplanned() - 1));
    SERIAL_PROTOCOLPGM(" B"); SERIAL_PROTOCOL(BUFSIZE - commands_in_queue);
  #endif
  SERIAL_EOL;
}

void clamp_to_software_endstops(float target[3]) {
  if (min_software_endstops) {
    NOLESS(target[X_AXIS], min_pos[X_AXIS]);
    NOLESS(target[Y_AXIS], min_pos[Y_AXIS]);

    float negative_z_offset = 0;
    #if ENABLED(AUTO_BED_LEVELING_FEATURE)
      if (zprobe_zoffset < 0) negative_z_offset += zprobe_zoffset;
      if (home_offset[Z_AXIS] < 0) {
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            SERIAL_ECHOPAIR("> clamp_to_software_endstops > Add home_offset[Z_AXIS]:", home_offset[Z_AXIS]);
            SERIAL_EOL;
          }
        #endif
        negative_z_offset += home_offset[Z_AXIS];
      }
    #endif
    NOLESS(target[Z_AXIS], min_pos[Z_AXIS] + negative_z_offset);
  }

  if (max_software_endstops) {
    NOMORE(target[X_AXIS], max_pos[X_AXIS]);
    NOMORE(target[Y_AXIS], max_pos[Y_AXIS]);
    NOMORE(target[Z_AXIS], max_pos[Z_AXIS]);
  }
}

#if ENABLED(DELTA)

  void recalc_delta_settings(float radius, float diagonal_rod) {
    delta_tower1_x = -SIN_60 * (radius + DELTA_RADIUS_TRIM_TOWER_1);  // front left tower
    delta_tower1_y = -COS_60 * (radius + DELTA_RADIUS_TRIM_TOWER_1);
    delta_tower2_x =  SIN_60 * (radius + DELTA_RADIUS_TRIM_TOWER_2);  // front right tower
    delta_tower2_y = -COS_60 * (radius + DELTA_RADIUS_TRIM_TOWER_2);
    delta_tower3_x = 0.0;                                             // back middle tower
    delta_tower3_y = (radius + DELTA_RADIUS_TRIM_TOWER_3);
    delta_diagonal_rod_2_tower_1 = sq(delta_diagonal_rod + delta_diagonal_rod_trim_tower_1);
    delta_diagonal_rod_2_tower_2 = sq(delta_diagonal_rod + delta_diagonal_rod_trim_tower_2);
    delta_diagonal_rod_2_tower_3 = sq(delta_diagonal_rod + delta_diagonal_rod_trim_tower_3);
  }

  void calculate_delta(float cartesian[3]) {

    delta[TOWER_1] = sqrt(delta_diagonal_rod_2_tower_1
                          - sq(delta_tower1_x - cartesian[X_AXIS])
                          - sq(delta_tower1_y - cartesian[Y_AXIS])
                         ) + cartesian[Z_AXIS];
    delta[TOWER_2] = sqrt(delta_diagonal_rod_2_tower_2
                          - sq(delta_tower2_x - cartesian[X_AXIS])
                          - sq(delta_tower2_y - cartesian[Y_AXIS])
                         ) + cartesian[Z_AXIS];
    delta[TOWER_3] = sqrt(delta_diagonal_rod_2_tower_3
                          - sq(delta_tower3_x - cartesian[X_AXIS])
                          - sq(delta_tower3_y - cartesian[Y_AXIS])
                         ) + cartesian[Z_AXIS];
    /*
    SERIAL_ECHOPGM("cartesian x="); SERIAL_ECHO(cartesian[X_AXIS]);
    SERIAL_ECHOPGM(" y="); SERIAL_ECHO(cartesian[Y_AXIS]);
    SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(cartesian[Z_AXIS]);

    SERIAL_ECHOPGM("delta a="); SERIAL_ECHO(delta[TOWER_1]);
    SERIAL_ECHOPGM(" b="); SERIAL_ECHO(delta[TOWER_2]);
    SERIAL_ECHOPGM(" c="); SERIAL_ECHOLN(delta[TOWER_3]);
    */
  }

  #if ENABLED(AUTO_BED_LEVELING_FEATURE)   // 自动床平整功能

    // Adjust print surface height by linear interpolation over the bed_level array.  在床级阵列上通过线性插值调整打印表面高度。
    void adjust_delta(float cartesian[3]) {
      if (delta_grid_spacing[0] == 0 || delta_grid_spacing[1] == 0) return; // G29 not done!  G29还没完成！

      int half = (AUTO_BED_LEVELING_GRID_POINTS - 1) / 2;
      float h1 = 0.001 - half, h2 = half - 0.001,
            grid_x = max(h1, min(h2, cartesian[X_AXIS] / delta_grid_spacing[0])),
            grid_y = max(h1, min(h2, cartesian[Y_AXIS] / delta_grid_spacing[1]));
      int floor_x = floor(grid_x), floor_y = floor(grid_y);
      float ratio_x = grid_x - floor_x, ratio_y = grid_y - floor_y,
            z1 = bed_level[floor_x + half][floor_y + half],
            z2 = bed_level[floor_x + half][floor_y + half + 1],
            z3 = bed_level[floor_x + half + 1][floor_y + half],
            z4 = bed_level[floor_x + half + 1][floor_y + half + 1],
            left = (1 - ratio_y) * z1 + ratio_y * z2,
            right = (1 - ratio_y) * z3 + ratio_y * z4,
            offset = (1 - ratio_x) * left + ratio_x * right;

      delta[X_AXIS] += offset;
      delta[Y_AXIS] += offset;
      delta[Z_AXIS] += offset;

      /*
      SERIAL_ECHOPGM("grid_x="); SERIAL_ECHO(grid_x);
      SERIAL_ECHOPGM(" grid_y="); SERIAL_ECHO(grid_y);
      SERIAL_ECHOPGM(" floor_x="); SERIAL_ECHO(floor_x);
      SERIAL_ECHOPGM(" floor_y="); SERIAL_ECHO(floor_y);
      SERIAL_ECHOPGM(" ratio_x="); SERIAL_ECHO(ratio_x);
      SERIAL_ECHOPGM(" ratio_y="); SERIAL_ECHO(ratio_y);
      SERIAL_ECHOPGM(" z1="); SERIAL_ECHO(z1);
      SERIAL_ECHOPGM(" z2="); SERIAL_ECHO(z2);
      SERIAL_ECHOPGM(" z3="); SERIAL_ECHO(z3);
      SERIAL_ECHOPGM(" z4="); SERIAL_ECHO(z4);
      SERIAL_ECHOPGM(" left="); SERIAL_ECHO(left);
      SERIAL_ECHOPGM(" right="); SERIAL_ECHO(right);
      SERIAL_ECHOPGM(" offset="); SERIAL_ECHOLN(offset);
      */
    }
  #endif // AUTO_BED_LEVELING_FEATURE  自动床平整功能

#endif // DELTA

#if ENABLED(MESH_BED_LEVELING)   //启用(网床平整)

// This function is used to split lines on mesh borders so each segment is only part of one mesh area
void mesh_plan_buffer_line(float x, float y, float z, const float e, float feed_rate, const uint8_t& extruder, uint8_t x_splits = 0xff, uint8_t y_splits = 0xff) {
  if (!mbl.active) {
    plan_buffer_line(x, y, z, e, feed_rate, extruder);
    set_current_to_destination();
    return;
  }
  int pix = mbl.select_x_index(current_position[X_AXIS]);
  int piy = mbl.select_y_index(current_position[Y_AXIS]);
  int ix = mbl.select_x_index(x);
  int iy = mbl.select_y_index(y);
  pix = min(pix, MESH_NUM_X_POINTS - 2);
  piy = min(piy, MESH_NUM_Y_POINTS - 2);
  ix = min(ix, MESH_NUM_X_POINTS - 2);
  iy = min(iy, MESH_NUM_Y_POINTS - 2);
  if (pix == ix && piy == iy) {
    // Start and end on same mesh square  开始和结束在同一个网格方
    plan_buffer_line(x, y, z, e, feed_rate, extruder);
    set_current_to_destination();
    return;
  }
  float nx, ny, ne, normalized_dist;
  if (ix > pix && (x_splits) & BIT(ix)) {
    nx = mbl.get_x(ix);
    normalized_dist = (nx - current_position[X_AXIS]) / (x - current_position[X_AXIS]);
    ny = current_position[Y_AXIS] + (y - current_position[Y_AXIS]) * normalized_dist;
    ne = current_position[E_AXIS] + (e - current_position[E_AXIS]) * normalized_dist;
    x_splits ^= BIT(ix);
  }
  else if (ix < pix && (x_splits) & BIT(pix)) {
    nx = mbl.get_x(pix);
    normalized_dist = (nx - current_position[X_AXIS]) / (x - current_position[X_AXIS]);
    ny = current_position[Y_AXIS] + (y - current_position[Y_AXIS]) * normalized_dist;
    ne = current_position[E_AXIS] + (e - current_position[E_AXIS]) * normalized_dist;
    x_splits ^= BIT(pix);
  }
  else if (iy > piy && (y_splits) & BIT(iy)) {
    ny = mbl.get_y(iy);
    normalized_dist = (ny - current_position[Y_AXIS]) / (y - current_position[Y_AXIS]);
    nx = current_position[X_AXIS] + (x - current_position[X_AXIS]) * normalized_dist;
    ne = current_position[E_AXIS] + (e - current_position[E_AXIS]) * normalized_dist;
    y_splits ^= BIT(iy);
  }
  else if (iy < piy && (y_splits) & BIT(piy)) {
    ny = mbl.get_y(piy);
    normalized_dist = (ny - current_position[Y_AXIS]) / (y - current_position[Y_AXIS]);
    nx = current_position[X_AXIS] + (x - current_position[X_AXIS]) * normalized_dist;
    ne = current_position[E_AXIS] + (e - current_position[E_AXIS]) * normalized_dist;
    y_splits ^= BIT(piy);
  }
  else {
    // Already split on a border  已经在边界上分裂了
    plan_buffer_line(x, y, z, e, feed_rate, extruder);
    set_current_to_destination();
    return;
  }
  // Do the split and look for more borders  进行拆分，寻找更多的边框
  destination[X_AXIS] = nx;
  destination[Y_AXIS] = ny;
  destination[E_AXIS] = ne;
  mesh_plan_buffer_line(nx, ny, z, ne, feed_rate, extruder, x_splits, y_splits);
  destination[X_AXIS] = x;
  destination[Y_AXIS] = y;
  destination[E_AXIS] = e;
  mesh_plan_buffer_line(x, y, z, e, feed_rate, extruder, x_splits, y_splits);
}
#endif  // MESH_BED_LEVELING

#if ENABLED(PREVENT_DANGEROUS_EXTRUDE)

  inline void prevent_dangerous_extrude(float& curr_e, float& dest_e) {
    if (marlin_debug_flags & DEBUG_DRYRUN) return;
    float de = dest_e - curr_e;
    if (de) {
      if (degHotend(active_extruder) < extrude_min_temp) {
        curr_e = dest_e; // Behave as if the move really took place, but ignore E part 表现得好像搬家是真的，但忽略了这一部分
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM(MSG_ERR_COLD_EXTRUDE_STOP);
      }
      #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
        if (labs(de) > EXTRUDE_MAXLENGTH) {
          curr_e = dest_e; // Behave as if the move really took place, but ignore E part
          SERIAL_ECHO_START;
          SERIAL_ECHOLNPGM(MSG_ERR_LONG_EXTRUDE_STOP);
        }
      #endif
    }
  }

#endif // PREVENT_DANGEROUS_EXTRUDE

#if ENABLED(DELTA) || ENABLED(SCARA)   //启用(三角洲)||启用(斯卡拉)

  inline bool prepare_move_delta(float target[NUM_AXIS]) {
    float difference[NUM_AXIS];
    for (int8_t i = 0; i < NUM_AXIS; i++) difference[i] = target[i] - current_position[i];

    float cartesian_mm = sqrt(sq(difference[X_AXIS]) + sq(difference[Y_AXIS]) + sq(difference[Z_AXIS]));
    if (cartesian_mm < 0.000001) cartesian_mm = abs(difference[E_AXIS]);
    if (cartesian_mm < 0.000001) return false;
    float seconds = 6000 * cartesian_mm / feedrate / feedrate_multiplier;
    int steps = max(1, int(delta_segments_per_second * seconds));

    // SERIAL_ECHOPGM("mm="); SERIAL_ECHO(cartesian_mm);
    // SERIAL_ECHOPGM(" seconds="); SERIAL_ECHO(seconds);
    // SERIAL_ECHOPGM(" steps="); SERIAL_ECHOLN(steps);

    for (int s = 1; s <= steps; s++) {

      float fraction = float(s) / float(steps);

      for (int8_t i = 0; i < NUM_AXIS; i++)
        target[i] = current_position[i] + difference[i] * fraction;

      calculate_delta(target);

      #if ENABLED(AUTO_BED_LEVELING_FEATURE)
        adjust_delta(target);
      #endif

      //SERIAL_ECHOPGM("target[X_AXIS]="); SERIAL_ECHOLN(target[X_AXIS]);
      //SERIAL_ECHOPGM("target[Y_AXIS]="); SERIAL_ECHOLN(target[Y_AXIS]);
      //SERIAL_ECHOPGM("target[Z_AXIS]="); SERIAL_ECHOLN(target[Z_AXIS]);
      //SERIAL_ECHOPGM("delta[X_AXIS]="); SERIAL_ECHOLN(delta[X_AXIS]);
      //SERIAL_ECHOPGM("delta[Y_AXIS]="); SERIAL_ECHOLN(delta[Y_AXIS]);
      //SERIAL_ECHOPGM("delta[Z_AXIS]="); SERIAL_ECHOLN(delta[Z_AXIS]);

      plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], target[E_AXIS], feedrate / 60 * feedrate_multiplier / 100.0, active_extruder);
    }
    return true;
  }

#endif // DELTA || SCARA

#if ENABLED(SCARA)
  inline bool prepare_move_scara(float target[NUM_AXIS]) { return prepare_move_delta(target); }
#endif

#if ENABLED(DUAL_X_CARRIAGE)

  inline bool prepare_move_dual_x_carriage() {
    if (active_extruder_parked) {
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && active_extruder == 0) {
        // move duplicate extruder into correct duplication position.
        plan_set_position(inactive_extruder_x_pos, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        plan_buffer_line(current_position[X_AXIS] + duplicate_extruder_x_offset,
                         current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS], 1);
        sync_plan_position();
        st_synchronize();
        extruder_duplication_enabled = true;
        active_extruder_parked = false;
      }
      else if (dual_x_carriage_mode == DXC_AUTO_PARK_MODE) { // handle unparking of head  处理头的卸载
        if (current_position[E_AXIS] == destination[E_AXIS]) {
          // This is a travel move (with no extrusion)  这是一个移动（没有挤压）
          // Skip it, but keep track of the current position  跳过它，但要跟踪当前位置
          // (so it can be used as the start of the next non-travel move)  （因此可以作为下次非旅行移动的开始）
          if (delayed_move_time != 0xFFFFFFFFUL) {
            set_current_to_destination();
            NOLESS(raised_parked_position[Z_AXIS], destination[Z_AXIS]);
            delayed_move_time = millis();
            return false;
          }
        }
        delayed_move_time = 0;
        // unpark extruder: 1) raise, 2) move into starting XY position, 3) lower  开泊挤出机：1）升起，2）移至起动xy位置，3）较低
        plan_buffer_line(raised_parked_position[X_AXIS], raised_parked_position[Y_AXIS], raised_parked_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], raised_parked_position[Z_AXIS], current_position[E_AXIS], min(max_feedrate[X_AXIS], max_feedrate[Y_AXIS]), active_extruder);
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
        active_extruder_parked = false;
      }
    }
    return true;
  }

#endif // DUAL_X_CARRIAGE

#if DISABLED(DELTA) && DISABLED(SCARA)

  inline bool prepare_move_cartesian() {
    // Do not use feedrate_multiplier for E or Z only moves  不要只对e或z移动使用馈率乘数
    if (current_position[X_AXIS] == destination[X_AXIS] && current_position[Y_AXIS] == destination[Y_AXIS]) {
      line_to_destination();
    }
    else {
      #if ENABLED(MESH_BED_LEVELING)
        mesh_plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], (feedrate / 60) * (feedrate_multiplier / 100.0), active_extruder);
        return false;
      #else
        line_to_destination(feedrate * feedrate_multiplier / 100.0);
      #endif
    }
    return true;
  }

#endif // !DELTA && !SCARA

/**
 * Prepare a single move and get ready for the next one 准备一个步骤，为下一个步骤做好准备
 *
 * (This may call plan_buffer_line several times to put 这可能需要规划缓冲线多次放置
 *  smaller moves into the planner for DELTA or SCARA.)*更小的移动进入三角洲或斯卡拉的计划
 */
void prepare_move() {
  clamp_to_software_endstops(destination);
  refresh_cmd_timeout();

  #if ENABLED(PREVENT_DANGEROUS_EXTRUDE)
    prevent_dangerous_extrude(current_position[E_AXIS], destination[E_AXIS]);
  #endif

  #if ENABLED(SCARA)
    if (!prepare_move_scara(destination)) return;
  #elif ENABLED(DELTA)
    if (!prepare_move_delta(destination)) return;
  #endif

  #if ENABLED(DUAL_X_CARRIAGE)
    if (!prepare_move_dual_x_carriage()) return;
  #endif

  #if DISABLED(DELTA) && DISABLED(SCARA)
    if (!prepare_move_cartesian()) return;
  #endif

  set_current_to_destination();
}

/**
 * Plan an arc in 2 dimensions
 *
 * The arc is approximated by generating many small linear segments.
 * The length of each segment is configured in MM_PER_ARC_SEGMENT (Default 1mm)
 * Arcs should only be made relatively large (over 5mm), as larger arcs with
 * larger segments will tend to be more efficient. Your slicer should have
 * options for G2/G3 arc generation. In future these options may be GCode tunable.
 */
void plan_arc(
  float target[NUM_AXIS], // Destination position
  float* offset,          // Center of rotation relative to current_position
  uint8_t clockwise       // Clockwise?
) {

  float radius = hypot(offset[X_AXIS], offset[Y_AXIS]),
        center_axis0 = current_position[X_AXIS] + offset[X_AXIS],
        center_axis1 = current_position[Y_AXIS] + offset[Y_AXIS],
        linear_travel = target[Z_AXIS] - current_position[Z_AXIS],
        extruder_travel = target[E_AXIS] - current_position[E_AXIS],
        r_axis0 = -offset[X_AXIS],  // Radius vector from center to current location
        r_axis1 = -offset[Y_AXIS],
        rt_axis0 = target[X_AXIS] - center_axis0,
        rt_axis1 = target[Y_AXIS] - center_axis1;

  // CCW angle of rotation between position and target from the circle center. Only one atan2() trig computation required.
  //从圆心的位置和目标之间的cccw旋转角度。只需一个atan2()三角计算。
  float angular_travel = atan2(r_axis0 * rt_axis1 - r_axis1 * rt_axis0, r_axis0 * rt_axis0 + r_axis1 * rt_axis1);
  if (angular_travel < 0)  angular_travel += RADIANS(360);
  if (clockwise)  angular_travel -= RADIANS(360);

  // Make a circle if the angular rotation is 0  如果角旋转为0，则做一个圆
  if (current_position[X_AXIS] == target[X_AXIS] && current_position[Y_AXIS] == target[Y_AXIS] && angular_travel == 0)
    angular_travel += RADIANS(360);

  float mm_of_travel = hypot(angular_travel * radius, fabs(linear_travel));
  if (mm_of_travel < 0.001)  return;
  uint16_t segments = floor(mm_of_travel / MM_PER_ARC_SEGMENT);
  if (segments == 0) segments = 1;

  float theta_per_segment = angular_travel / segments;
  float linear_per_segment = linear_travel / segments;
  float extruder_per_segment = extruder_travel / segments;

  /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
     and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
         r_T = [cos(phi) -sin(phi);
                sin(phi)  cos(phi] * r ;

     For arc generation, the center of the circle is the axis of rotation and the radius vector is
     defined from the circle center to the initial position. Each line segment is formed by successive
     vector rotations. This requires only two cos() and sin() computations to form the rotation
     matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
     all double numbers are single precision on the Arduino. (True double precision will not have
     round off issues for CNC applications.) Single precision error can accumulate to be greater than
     tool precision in some cases. Therefore, arc path correction is implemented.

     Small angle approximation may be used to reduce computation overhead further. This approximation
     holds for everything, but very small circles and large MM_PER_ARC_SEGMENT values. In other words,
     theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
     to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
     numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
     issue for CNC machines with the single precision Arduino calculations.

     This approximation also allows plan_arc to immediately insert a line segment into the planner
     without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
     a correction, the planner should have caught up to the lag caused by the initial plan_arc overhead.
     This is important when there are successive arc motions.
  *//*向量通过变换矩阵旋转：R是原来的向量，r_t是旋转的向量，
和Phi是旋转的角度。根据Jens Geisler的解决方案。
r_t=[因为(PHI)-sin(PHI);
罪（PHI）因为（Phi）*R；
对于弧生成，圆的中心是自转轴，半径向量为
定义从圆心到初始位置。每个线段都是由连续的
向量旋转.这只需要两个Cos()和sin()计算就可以形成旋转
整个弧线持续时间的矩阵。误差可能因数值舍入而累积，因为
所有双数都是Arduino上的单精度。（真正的双重精度不会有
用于cnc应用的循环问题。)单精度误差可以累积到大于
在某些情况下,工具的精度.因此，实现了弧径校正。
小角度近似可以用来进一步降低计算开销。这个近似值
可以容纳任何东西，但是只有非常小的圆和大的mm_perar_sece值。换句话说，
这部分需要大于0.1 Rad，而n_arc的校正则需要很大
引起可察觉的漂移误差。n_arc_纠正~=25小到足以纠正
数值漂移误差.在错误变为
单精度阿杜伊诺计算的数控机床问题。
这个近似值还允许平面弧立即将线段插入计划中
没有计算的初始开销，Cos()或sin()。到需要应用弧线的时候
修正后，计划人员应该已经赶上了最初的计划所造成的延迟。
当连续的弧线运动时，这是很重要的。
*/
  // Vector rotation matrix values  向量旋转矩阵值
  float cos_T = 1 - 0.5 * theta_per_segment * theta_per_segment; // Small angle approximation
  float sin_T = theta_per_segment;

  float arc_target[NUM_AXIS];
  float sin_Ti;
  float cos_Ti;
  float r_axisi;
  uint16_t i;
  int8_t count = 0;

  // Initialize the linear axis  初始化线性轴
  arc_target[Z_AXIS] = current_position[Z_AXIS];

  // Initialize the extruder axis
  arc_target[E_AXIS] = current_position[E_AXIS];

  float feed_rate = feedrate * feedrate_multiplier / 60 / 100.0;

  for (i = 1; i < segments; i++) { // Increment (segments-1)

    if (count < N_ARC_CORRECTION) {
      // Apply vector rotation matrix to previous r_axis0 / 1
      r_axisi = r_axis0 * sin_T + r_axis1 * cos_T;
      r_axis0 = r_axis0 * cos_T - r_axis1 * sin_T;
      r_axis1 = r_axisi;
      count++;
    }
    else {
      // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
      // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
      cos_Ti = cos(i * theta_per_segment);
      sin_Ti = sin(i * theta_per_segment);
      r_axis0 = -offset[X_AXIS] * cos_Ti + offset[Y_AXIS] * sin_Ti;
      r_axis1 = -offset[X_AXIS] * sin_Ti - offset[Y_AXIS] * cos_Ti;
      count = 0;
    }

    // Update arc_target location
    arc_target[X_AXIS] = center_axis0 + r_axis0;
    arc_target[Y_AXIS] = center_axis1 + r_axis1;
    arc_target[Z_AXIS] += linear_per_segment;
    arc_target[E_AXIS] += extruder_per_segment;

    clamp_to_software_endstops(arc_target);

    #if ENABLED(DELTA) || ENABLED(SCARA)
      calculate_delta(arc_target);
      #if ENABLED(AUTO_BED_LEVELING_FEATURE)
        adjust_delta(arc_target);
      #endif
      plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], arc_target[E_AXIS], feed_rate, active_extruder);
    #else
      plan_buffer_line(arc_target[X_AXIS], arc_target[Y_AXIS], arc_target[Z_AXIS], arc_target[E_AXIS], feed_rate, active_extruder);
    #endif
  }

  // Ensure last segment arrives at target location.  确保最后一段到达目标位置。
  #if ENABLED(DELTA) || ENABLED(SCARA)
    calculate_delta(target);
    #if ENABLED(AUTO_BED_LEVELING_FEATURE)
      adjust_delta(target);
    #endif
    plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], target[E_AXIS], feed_rate, active_extruder);
  #else
    plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feed_rate, active_extruder);
  #endif

  // As far as the parser is concerned, the position is now == target. In reality the     就解析器而言，位置现在是==目标。在现实中
  // motion control system might still be processing the action and the real tool position  运动控制系统可能还在处理动作和实际工具的位置
  // in any intermediate location. 在任何中间位置。
  set_current_to_destination();
}

#if HAS_CONTROLLERFAN

  void controllerFan() {
    static millis_t lastMotor = 0;      // Last time a motor was turned on 上次有台马达被打开
    static millis_t lastMotorCheck = 0; // Last time the state was checked  上次检查的时候
    millis_t ms = millis();
    if (ms >= lastMotorCheck + 2500) { // Not a time critical function, so we only check every 2500ms  不是时间的关键功能，所以我们只检查每2500m
      lastMotorCheck = ms;
      if (X_ENABLE_READ == X_ENABLE_ON || Y_ENABLE_READ == Y_ENABLE_ON || Z_ENABLE_READ == Z_ENABLE_ON || soft_pwm_bed > 0
          || E0_ENABLE_READ == E_ENABLE_ON // If any of the drivers are enabled...
          #if EXTRUDERS > 1
            || E1_ENABLE_READ == E_ENABLE_ON
            #if HAS_X2_ENABLE
              || X2_ENABLE_READ == X_ENABLE_ON
            #endif
            #if EXTRUDERS > 2
              || E2_ENABLE_READ == E_ENABLE_ON
              #if EXTRUDERS > 3
                || E3_ENABLE_READ == E_ENABLE_ON
              #endif
            #endif
          #endif
      ) {
        lastMotor = ms; //... set time to NOW so the fan will turn on 把时间定在现在，这样风扇就会打开
      }
      uint8_t speed = (lastMotor == 0 || ms >= lastMotor + (CONTROLLERFAN_SECS * 1000UL)) ? 0 : CONTROLLERFAN_SPEED;
      // allows digital or PWM fan output to be used (see M42 handling) 允许使用数字或pwm风扇输出（见m 42处理）
      digitalWrite(CONTROLLERFAN_PIN, speed);
      analogWrite(CONTROLLERFAN_PIN, speed);
    }
  }

#endif // HAS_CONTROLLERFAN

#if ENABLED(SCARA)

  void calculate_SCARA_forward_Transform(float f_scara[3]) {
    // Perform forward kinematics, and place results in delta[3]
    // The maths and first version has been done by QHARLEY . Integrated into masterbranch 06/2014 and slightly restructured by Joachim Cerny in June 2014

    float x_sin, x_cos, y_sin, y_cos;

    //SERIAL_ECHOPGM("f_delta x="); SERIAL_ECHO(f_scara[X_AXIS]);
    //SERIAL_ECHOPGM(" y="); SERIAL_ECHO(f_scara[Y_AXIS]);

    x_sin = sin(f_scara[X_AXIS] / SCARA_RAD2DEG) * Linkage_1;
    x_cos = cos(f_scara[X_AXIS] / SCARA_RAD2DEG) * Linkage_1;
    y_sin = sin(f_scara[Y_AXIS] / SCARA_RAD2DEG) * Linkage_2;
    y_cos = cos(f_scara[Y_AXIS] / SCARA_RAD2DEG) * Linkage_2;

    //SERIAL_ECHOPGM(" x_sin="); SERIAL_ECHO(x_sin);
    //SERIAL_ECHOPGM(" x_cos="); SERIAL_ECHO(x_cos);
    //SERIAL_ECHOPGM(" y_sin="); SERIAL_ECHO(y_sin);
    //SERIAL_ECHOPGM(" y_cos="); SERIAL_ECHOLN(y_cos);

    delta[X_AXIS] = x_cos + y_cos + SCARA_offset_x;  //theta
    delta[Y_AXIS] = x_sin + y_sin + SCARA_offset_y;  //theta+phi

    //SERIAL_ECHOPGM(" delta[X_AXIS]="); SERIAL_ECHO(delta[X_AXIS]);
    //SERIAL_ECHOPGM(" delta[Y_AXIS]="); SERIAL_ECHOLN(delta[Y_AXIS]);
  }

  void calculate_delta(float cartesian[3]) {
    //reverse kinematics.
    // Perform reversed kinematics, and place results in delta[3]
    // The maths and first version has been done by QHARLEY . Integrated into masterbranch 06/2014 and slightly restructured by Joachim Cerny in June 2014

    float SCARA_pos[2];
    static float SCARA_C2, SCARA_S2, SCARA_K1, SCARA_K2, SCARA_theta, SCARA_psi;

    SCARA_pos[X_AXIS] = cartesian[X_AXIS] * axis_scaling[X_AXIS] - SCARA_offset_x;  //Translate SCARA to standard X Y
    SCARA_pos[Y_AXIS] = cartesian[Y_AXIS] * axis_scaling[Y_AXIS] - SCARA_offset_y;  // With scaling factor.

    #if (Linkage_1 == Linkage_2)
      SCARA_C2 = ((sq(SCARA_pos[X_AXIS]) + sq(SCARA_pos[Y_AXIS])) / (2 * (float)L1_2)) - 1;
    #else
      SCARA_C2 = (sq(SCARA_pos[X_AXIS]) + sq(SCARA_pos[Y_AXIS]) - (float)L1_2 - (float)L2_2) / 45000;
    #endif

    SCARA_S2 = sqrt(1 - sq(SCARA_C2));

    SCARA_K1 = Linkage_1 + Linkage_2 * SCARA_C2;
    SCARA_K2 = Linkage_2 * SCARA_S2;

    SCARA_theta = (atan2(SCARA_pos[X_AXIS], SCARA_pos[Y_AXIS]) - atan2(SCARA_K1, SCARA_K2)) * -1;
    SCARA_psi = atan2(SCARA_S2, SCARA_C2);

    delta[X_AXIS] = SCARA_theta * SCARA_RAD2DEG;  // Multiply by 180/Pi  -  theta is support arm angle   希塔是支撑手臂的角度
    delta[Y_AXIS] = (SCARA_theta + SCARA_psi) * SCARA_RAD2DEG;  //       -  equal to sub arm angle (inverted motor) 等于分臂角度(倒马达)
    delta[Z_AXIS] = cartesian[Z_AXIS];

    /*
    SERIAL_ECHOPGM("cartesian x="); SERIAL_ECHO(cartesian[X_AXIS]);
    SERIAL_ECHOPGM(" y="); SERIAL_ECHO(cartesian[Y_AXIS]);
    SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(cartesian[Z_AXIS]);

    SERIAL_ECHOPGM("scara x="); SERIAL_ECHO(SCARA_pos[X_AXIS]);
    SERIAL_ECHOPGM(" y="); SERIAL_ECHOLN(SCARA_pos[Y_AXIS]);

    SERIAL_ECHOPGM("delta x="); SERIAL_ECHO(delta[X_AXIS]);
    SERIAL_ECHOPGM(" y="); SERIAL_ECHO(delta[Y_AXIS]);
    SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(delta[Z_AXIS]);

    SERIAL_ECHOPGM("C2="); SERIAL_ECHO(SCARA_C2);
    SERIAL_ECHOPGM(" S2="); SERIAL_ECHO(SCARA_S2);
    SERIAL_ECHOPGM(" Theta="); SERIAL_ECHO(SCARA_theta);
    SERIAL_ECHOPGM(" Psi="); SERIAL_ECHOLN(SCARA_psi);
    SERIAL_EOL;
    */
  }

#endif // SCARA

#if ENABLED(TEMP_STAT_LEDS)

  static bool red_led = false;
  static millis_t next_status_led_update_ms = 0;

  void handle_status_leds(void) {
    float max_temp = 0.0;
    if (millis() > next_status_led_update_ms) {
      next_status_led_update_ms += 500; // Update every 0.5s
      for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder)
        max_temp = max(max(max_temp, degHotend(cur_extruder)), degTargetHotend(cur_extruder));
      #if HAS_TEMP_BED
        max_temp = max(max(max_temp, degTargetBed()), degBed());
      #endif
      bool new_led = (max_temp > 55.0) ? true : (max_temp < 54.0) ? false : red_led;
      if (new_led != red_led) {
        red_led = new_led;
        digitalWrite(STAT_LED_RED, new_led ? HIGH : LOW);
        digitalWrite(STAT_LED_BLUE, new_led ? LOW : HIGH);
      }
    }
  }

#endif

void enable_all_steppers() {
  enable_x();
  enable_y();
  enable_z();
  enable_e0();
  enable_e1();
  enable_e2();
  enable_e3();
}

void disable_all_steppers() {
  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();
  disable_e3();
}

/**
 * Standard idle routine keeps the machine alive
 */
void idle() {
  manage_heater();
  manage_inactivity();
  lcd_update();
}

/**
 * Manage several activities:
 *  - Check for Filament Runout
 *  - Keep the command buffer full
 *  - Check for maximum inactive time between commands
 *  - Check for maximum inactive time between stepper commands
 *  - Check if pin CHDK needs to go LOW
 *  - Check for KILL button held down
 *  - Check for HOME button held down
 *  - Check if cooling fan needs to be switched on
 *  - Check if an idle but hot extruder needs filament extruded (EXTRUDER_RUNOUT_PREVENT)
 */
void manage_inactivity(bool ignore_stepper_queue/*=false*/) {

  #if HAS_FILRUNOUT
    if (IS_SD_PRINTING && !(READ(FILRUNOUT_PIN) ^ FIL_RUNOUT_INVERTING))
      filrunout();
  #endif

  if (commands_in_queue < BUFSIZE - 1) get_command();

  millis_t ms = millis();

  if (max_inactive_time && ms > previous_cmd_ms + max_inactive_time) kill(PSTR(MSG_KILLED));

  if (stepper_inactive_time && ms > previous_cmd_ms + stepper_inactive_time
      && !ignore_stepper_queue && !blocks_queued()) {
    #if DISABLE_X == true
      disable_x();
    #endif
    #if DISABLE_Y == true
      disable_y();
    #endif
    #if DISABLE_Z == true
      disable_z();
    #endif
    #if DISABLE_E == true
      disable_e0();
      disable_e1();
      disable_e2();
      disable_e3();
    #endif
  }

  #ifdef CHDK // Check if pin should be set to LOW after M240 set it to HIGH
    if (chdkActive && ms > chdkHigh + CHDK_DELAY) {
      chdkActive = false;
      WRITE(CHDK, LOW);
    }
  #endif

  #if HAS_KILL

    // Check if the kill button was pressed and wait just in case it was an accidental
    // key kill key press 检查是否按下了终止按钮，然后等待，以防是意外键杀死键按

    // -------------------------------------------------------------------------------
    static int killCount = 0;   // make the inactivity button a bit less responsive  使不活动按钮的响应性降低一点
    const int KILL_DELAY = 750;
    if (!READ(KILL_PIN))
      killCount++;
    else if (killCount > 0)
      killCount--;

    // Exceeded threshold and we can confirm that it was not accidental
    // KILL the machine
    // ----------------------------------------------------------------
    if (killCount >= KILL_DELAY) kill(PSTR(MSG_KILLED));
  #endif

  #if HAS_HOME
    // Check to see if we have to home, use poor man's debouncer  看看我们是否要回家，用穷人的电话
    // ---------------------------------------------------------
    static int homeDebounceCount = 0;   // poor man's debouncing count  可怜人的弃数
    const int HOME_DEBOUNCE_DELAY = 2500;
    if (!READ(HOME_PIN)) {
      if (!homeDebounceCount) {
        enqueuecommands_P(PSTR("G28"));
        LCD_MESSAGEPGM(MSG_AUTO_HOME);
      }
      if (homeDebounceCount < HOME_DEBOUNCE_DELAY)
        homeDebounceCount++;
      else
        homeDebounceCount = 0;
    }
  #endif

  #if HAS_CONTROLLERFAN
    controllerFan(); // Check if fan should be turned on to cool stepper drivers down  检查风扇是否应打开以冷却步行者司机下来
  #endif

  #if ENABLED(EXTRUDER_RUNOUT_PREVENT)
    if (ms > previous_cmd_ms + EXTRUDER_RUNOUT_SECONDS * 1000)
      if (degHotend(active_extruder) > EXTRUDER_RUNOUT_MINTEMP) {
        bool oldstatus;
        switch (active_extruder) {
          case 0:
            oldstatus = E0_ENABLE_READ;
            enable_e0();
            break;
          #if EXTRUDERS > 1
            case 1:
              oldstatus = E1_ENABLE_READ;
              enable_e1();
              break;
            #if EXTRUDERS > 2
              case 2:
                oldstatus = E2_ENABLE_READ;
                enable_e2();
                break;
              #if EXTRUDERS > 3
                case 3:
                  oldstatus = E3_ENABLE_READ;
                  enable_e3();
                  break;
              #endif
            #endif
          #endif
        }
        float oldepos = current_position[E_AXIS], oldedes = destination[E_AXIS];
        plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS],
                         destination[E_AXIS] + EXTRUDER_RUNOUT_EXTRUDE * EXTRUDER_RUNOUT_ESTEPS / axis_steps_per_unit[E_AXIS],
                         EXTRUDER_RUNOUT_SPEED / 60. * EXTRUDER_RUNOUT_ESTEPS / axis_steps_per_unit[E_AXIS], active_extruder);
      current_position[E_AXIS] = oldepos;
      destination[E_AXIS] = oldedes;
      plan_set_e_position(oldepos);
      previous_cmd_ms = ms; // refresh_cmd_timeout()
      st_synchronize();
      switch (active_extruder) {
        case 0:
          E0_ENABLE_WRITE(oldstatus);
          break;
        #if EXTRUDERS > 1
          case 1:
            E1_ENABLE_WRITE(oldstatus);
            break;
          #if EXTRUDERS > 2
            case 2:
              E2_ENABLE_WRITE(oldstatus);
              break;
            #if EXTRUDERS > 3
              case 3:
                E3_ENABLE_WRITE(oldstatus);
                break;
            #endif
          #endif
        #endif
      }
    }
  #endif

  #if ENABLED(DUAL_X_CARRIAGE)
    // handle delayed move timeout
    if (delayed_move_time && ms > delayed_move_time + 1000 && IsRunning()) {
      // travel moves have been received so enact them
      delayed_move_time = 0xFFFFFFFFUL; // force moves to be done
      set_destination_to_current();
      prepare_move();
    }
  #endif

  #if ENABLED(TEMP_STAT_LEDS)
    handle_status_leds();
  #endif

  check_axes_activity();
}

void kill(const char* lcd_msg) {
  #if ENABLED(ULTRA_LCD)
    lcd_setalertstatuspgm(lcd_msg);
  #else
    UNUSED(lcd_msg);
  #endif

  cli(); // Stop interrupts
  disable_all_heaters();
  disable_all_steppers();

  #if HAS_POWER_SWITCH
    pinMode(PS_ON_PIN, INPUT);
  #endif

  SERIAL_ERROR_START;
  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);

  // FMC small patch to update the LCD before ending  fmc小补丁在结束前更新lcd
  sei();   // enable interrupts
  for (int i = 5; i--; lcd_update()) delay(200); // Wait a short time  稍等片刻
  cli();   // disable interrupts
  suicide();
  while (1) { /* Intentionally left empty */ } // Wait for reset
}

#if ENABLED(FILAMENT_RUNOUT_SENSOR)

  void filrunout() {
    if (!filrunoutEnqueued) {
      filrunoutEnqueued = true;
      enqueuecommands_P(PSTR(FILAMENT_RUNOUT_SCRIPT));
      st_synchronize();
    }
  }

#endif // FILAMENT_RUNOUT_SENSOR

#if ENABLED(FAST_PWM_FAN)

  void setPwmFrequency(uint8_t pin, int val) {
    val &= 0x07;
    switch (digitalPinToTimer(pin)) {
      #if defined(TCCR0A)
        case TIMER0A:
        case TIMER0B:
          // TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
          // TCCR0B |= val;
          break;
      #endif
      #if defined(TCCR1A)
        case TIMER1A:
        case TIMER1B:
          // TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
          // TCCR1B |= val;
          break;
      #endif
      #if defined(TCCR2)
        case TIMER2:
        case TIMER2:
          TCCR2 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
          TCCR2 |= val;
          break;
      #endif
      #if defined(TCCR2A)
        case TIMER2A:
        case TIMER2B:
          TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
          TCCR2B |= val;
          break;
      #endif
      #if defined(TCCR3A)
        case TIMER3A:
        case TIMER3B:
        case TIMER3C:
          TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
          TCCR3B |= val;
          break;
      #endif
      #if defined(TCCR4A)
        case TIMER4A:
        case TIMER4B:
        case TIMER4C:
          TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
          TCCR4B |= val;
          break;
      #endif
      #if defined(TCCR5A)
        case TIMER5A:
        case TIMER5B:
        case TIMER5C:
          TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
          TCCR5B |= val;
          break;
      #endif
    }
  }
#endif // FAST_PWM_FAN

void Stop() {
  disable_all_heaters();
  if (IsRunning()) {
    Running = false;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
    LCD_MESSAGEPGM(MSG_STOPPED);
  }
}

/**
 * Set target_extruder from the T parameter or the active_extruder 从t参数或动作挤出机设置目标挤出机
 *
 * Returns TRUE if the target is invalid 如果目标无效，返回真
 */
bool setTargetedHotend(int code) {
  target_extruder = active_extruder;
  if (code_seen('T')) {
    target_extruder = code_value_short();
    if (target_extruder >= EXTRUDERS) {
      SERIAL_ECHO_START;
      SERIAL_CHAR('M');
      SERIAL_ECHO(code);
      SERIAL_ECHOPGM(" " MSG_INVALID_EXTRUDER " ");
      SERIAL_ECHOLN(target_extruder);
      return true;
    }
  }
  return false;
}

float calculate_volumetric_multiplier(float diameter) {
  if (!volumetric_enabled || diameter == 0) return 1.0;
  float d2 = diameter * 0.5;
  return 1.0 / (M_PI * d2 * d2);
}

void calculate_volumetric_multipliers() {
  for (int i = 0; i < EXTRUDERS; i++)
    volumetric_multiplier[i] = calculate_volumetric_multiplier(filament_size[i]);
}

