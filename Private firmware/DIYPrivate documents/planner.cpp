/**
 * planner.cpp - Buffer movement commands and manage the acceleration profile plan  
 * 缓冲移动命令和管理加速配置文件计划
 * Part of Grbl
 *
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 *
 * Grbl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Grbl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * The ring buffer implementation gleaned from the wiring_serial library by David A. Mellis.
 *
 *
 * Reasoning behind the mathematics in this module (in the key of 'Mathematica'):
 *
 * s == speed, a == acceleration, t == time, d == distance
 *s==速度，a=加速度，T==时间，D==距离
 * Basic definitions:
 *   Speed[s_, a_, t_] := s + (a*t)  速度：=s+(a*T)
 *   Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]  旅行[s_,a_,T_]：=集成[s,a,T],T]
 *
 * Distance to reach a specific speed with a constant acceleration: 以恒定加速度达到特定速度的距离：
 *   Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
 *   d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()
 *
 * Speed after a given distance of travel with constant acceleration: 在给定的行程距离后，以恒定加速度行驶的速度
 *   Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
 *   m -> Sqrt[2 a d + s^2]
 *解决[[速度[s],a,T]==M,旅行[s,a,T]==D},M,T] *M->平方米[2 a D+s^2]
 * DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]
 *
 * When to start braking (di) to reach a specified destination speed (s2) after accelerating
 * from initial speed s1 without ever stopping at a plateau:   加速后何时开始刹车以达到指定的目的地速度
 *   Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
 *   di -> (2 a d - s1^2 + s2^2)/(4 a) --> intersection_distance()
 *
 * IntersectionDistance[s1_, s2_, a_, d_] := (2 a d - s1^2 + s2^2)/(4 a)
 *
 */

#include "Marlin.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "ultralcd.h"
#include "language.h"

#if ENABLED(MESH_BED_LEVELING)
  #include "mesh_bed_leveling.h"
#endif

//===========================================================================
//============================= public variables公共变量 ============================
//===========================================================================

millis_t minsegmenttime;
float max_feedrate[NUM_AXIS]; // Max speeds in mm per minute最大速度(毫米/分钟)
float axis_steps_per_unit[NUM_AXIS];//设置与您的硬件相匹配的值。对于Bath Masterclass Prusa Mendel的特殊铸造齿轮，这些值是  
// float axis_steps_per_unit [] = {91.42857,91.42857,3200 / 1.25,700}; 
unsigned long max_acceleration_units_per_sq_second[NUM_AXIS]; // Use M201 to override by software 使用m201以软体取代
float minimumfeedrate;
float acceleration;         // Normal acceleration mm/s^2  DEFAULT ACCELERATION for all printing moves. M204 SXXXX所有打印动作的正常加速度mm/s^2默认加速度。m 204 xxxxx
float retract_acceleration; // Retract acceleration mm/s^2 filament pull-back and push-forward while standing still in the other axes M204 TXXXX在其他轴上静止不动时，收回加速度mm/s^2长丝拉回和向前推
float travel_acceleration;  // Travel acceleration mm/s^2  DEFAULT ACCELERATION for all NON printing moves. M204 MXXXX所有非打印移动的默认加速mm/s^2。m 204 mXX
float max_xy_jerk;          // The largest speed change requiring no acceleration  不需要加速的最大速度变化
float max_z_jerk;
float max_e_jerk;
float mintravelfeedrate;
unsigned long axis_steps_per_sqr_second[NUM_AXIS];

#if ENABLED(AUTO_BED_LEVELING_FEATURE)
  // Transform required to compensate for bed level 为补偿床位水平所需的改造
  matrix_3x3 plan_bed_level_matrix = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0
  };
#endif // AUTO_BED_LEVELING_FEATURE

#if ENABLED(AUTOTEMP)
  float autotemp_max = 250;
  float autotemp_min = 210;
  float autotemp_factor = 0.1;
  bool autotemp_enabled = false;
#endif

//==============================================================================
//= semi-private variables, used in inline functions半私有变量，用于内联函数 ===
//==============================================================================

block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instfructions运动不稳定的环形缓冲器
volatile unsigned char block_buffer_head;           // Index of the next block to be pushed下一个要按的区块的索引
volatile unsigned char block_buffer_tail;           // Index of the block to process now要处理的块的索引

//===========================================================================
//====================== private variables 私有变量==========================
//===========================================================================

// The current position of the tool in absolute steps该工具当前的绝对步骤位置
long position[NUM_AXIS];               // Rescaled from extern when axis_steps_per_unit are changed by gcode当轴向被代码改变时，从外部重新命名
static float previous_speed[NUM_AXIS]; // Speed of previous path line segment  上一个路径线段的速度
static float previous_nominal_speed;   // Nominal speed of previous path line segment  上一个路径线段的名义速度

uint8_t g_uc_extruder_last_move[EXTRUDERS] = { 0 };

#ifdef XY_FREQUENCY_LIMIT
  // Used for the frequency limit 用于频率限制
  #define MAX_FREQ_TIME (1000000.0/XY_FREQUENCY_LIMIT)
  // Old direction bits. Used for speed calculations   旧方向位。用于速度计算
  static unsigned char old_direction_bits = 0;
  // Segment times (in µs). Used for speed calculations  分段时间（微米）。用于速度计算
  static long axis_segment_time[2][3] = { {MAX_FREQ_TIME + 1, 0, 0}, {MAX_FREQ_TIME + 1, 0, 0} };
#endif

#if ENABLED(FILAMENT_SENSOR)
  static char meas_sample; //temporary variable to hold filament measurement sample  持有灯丝测量样品的临时变量
#endif

#if ENABLED(DUAL_X_CARRIAGE)
  extern bool extruder_duplication_enabled;
#endif

//===========================================================================
//=============================== functions功能 =============================
//===========================================================================

// Get the next / previous index of the next block in the ring buffer 获取环缓冲区中下一个块的下一个/上一个索引
// NOTE: Using & here (not %) because BLOCK_BUFFER_SIZE is always a power of 2  注意：这里使用(不是%)，因为块大小总是2的幂
FORCE_INLINE int8_t next_block_index(int8_t block_index) { return BLOCK_MOD(block_index + 1); }
FORCE_INLINE int8_t prev_block_index(int8_t block_index) { return BLOCK_MOD(block_index - 1); }

// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the
//计算从初始速率加速到目标速率所需的距离（而不是时间）
// given acceleration:  //给定加速度
FORCE_INLINE float estimate_acceleration_distance(float initial_rate, float target_rate, float acceleration) {
  if (acceleration == 0) return 0; // acceleration was 0, set acceleration distance to 0 加速为0，加速距离设为0
  return (target_rate * target_rate - initial_rate * initial_rate) / (acceleration * 2);
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)
/* 这个函数给出了你必须开始刹车的点（以-加速度的速度），如果
//您以初始速率开始并加速到这一点，并希望在
//总路程。这可以用来计算加速度和
//在梯形没有平台的情况下减速（即永远不能达到最高速度）  */

FORCE_INLINE float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance) {
  if (acceleration == 0) return 0; // acceleration was 0, set intersection distance to 0
  return (acceleration * 2 * distance - initial_rate * initial_rate + final_rate * final_rate) / (acceleration * 4);
}

// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.

void calculate_trapezoid_for_block(block_t* block, float entry_factor, float exit_factor) {
  unsigned long initial_rate = ceil(block->nominal_rate * entry_factor); // (step/min)
  unsigned long final_rate = ceil(block->nominal_rate * exit_factor); // (step/min)

  // Limit minimal step rate (Otherwise the timer will overflow.)  限制最小步长（否则计时器会溢出）
  NOLESS(initial_rate, 120);
  NOLESS(final_rate, 120);

  long acceleration = block->acceleration_st;
  int32_t accelerate_steps = ceil(estimate_acceleration_distance(initial_rate, block->nominal_rate, acceleration));
  int32_t decelerate_steps = floor(estimate_acceleration_distance(block->nominal_rate, final_rate, -acceleration));

  // Calculate the size of Plateau of Nominal Rate.  计算标称利率的高原面积
  int32_t plateau_steps = block->step_event_count - accelerate_steps - decelerate_steps;

  // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
  // have to use intersection_distance() to calculate when to abort acceleration and start braking
  // in order to reach the final_rate exactly at the end of this block.
  if (plateau_steps < 0) {
    accelerate_steps = ceil(intersection_distance(initial_rate, final_rate, acceleration, block->step_event_count));
    accelerate_steps = max(accelerate_steps, 0); // Check limits due to numerical round-off由于数值翻转而产生的检查限制
    accelerate_steps = min((uint32_t)accelerate_steps, block->step_event_count);
    //(We can cast here to unsigned, because the above line ensures that we are above zero)我们可以在这里投给没有签名的人，因为上面一行确保我们在零以上)
    plateau_steps = 0;
  }

  #if ENABLED(ADVANCE)
    volatile long initial_advance = block->advance * entry_factor * entry_factor;
    volatile long final_advance = block->advance * exit_factor * exit_factor;
  #endif // ADVANCE

  // block->accelerate_until = accelerate_steps;块->加速=加速步骤；
  // block->decelerate_after = accelerate_steps+plateau_steps;块->后减速=加速级+平级级；
  CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section 在关键部分填充步进程序使用的变量
  if (!block->busy) { // Don't update variables if block is busy.  如果块占线，不要更新变量。
    block->accelerate_until = accelerate_steps;
    block->decelerate_after = accelerate_steps + plateau_steps;
    block->initial_rate = initial_rate;
    block->final_rate = final_rate;
    #if ENABLED(ADVANCE)
      block->initial_advance = initial_advance;
      block->final_advance = final_advance;
    #endif
  }
  CRITICAL_SECTION_END;
}

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the计算此时的最大允许速度，此时必须使用
// acceleration within the allotted distance.加速度在指定距离内。
FORCE_INLINE float max_allowable_speed(float acceleration, float target_velocity, float distance) {
  return sqrt(target_velocity * target_velocity - 2 * acceleration * distance);
}

// "Junction jerk" in this context is the immediate change in speed at the junction of two blocks.
// This method will calculate the junction jerk as the euclidean distance between the nominal
// velocities of the respective blocks.
//inline float junction_jerk(block_t *before, block_t *after) {
//  return sqrt(
//    pow((before->speed_x-after->speed_x), 2)+pow((before->speed_y-after->speed_y), 2));
//}


// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
void planner_reverse_pass_kernel(block_t* previous, block_t* current, block_t* next) {
  if (!current) return;
  UNUSED(previous);

  if (next) {
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (current->entry_speed != current->max_entry_speed) {

      // If nominal length true, max junction speed is guaranteed to be reached. Only compute果标称长度为真，则保证达到最大结速度。仅计算
      // for max allowable speed if block is decelerating and nominal length is false.对于最大允许速度，如果块是减速和名义长度是假的
      if (!current->nominal_length_flag && current->max_entry_speed > next->entry_speed) {
        current->entry_speed = min(current->max_entry_speed,
                                   max_allowable_speed(-current->acceleration, next->entry_speed, current->millimeters));
      }
      else {
        current->entry_speed = current->max_entry_speed;
      }
      current->recalculate_flag = true;

    }
  } // Skip last block. Already initialized and set for recalculation跳过最后一个街区。已初始化并设定重新计算.
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This计划_重新计算()需要重复当前计划两次。一次反向，一次向前。这个
// implements the reverse pass.反向传输.
void planner_reverse_pass() {
  uint8_t block_index = block_buffer_head;

  //Make a local copy of block_buffer_tail, because the interrupt can alter it 制作一个本地的块-缓冲区-尾部副本，因为中断会改变它
  CRITICAL_SECTION_START;
    unsigned char tail = block_buffer_tail;
  CRITICAL_SECTION_END

  if (BLOCK_MOD(block_buffer_head - tail + BLOCK_BUFFER_SIZE) > 3) { // moves queued移动队列
    block_index = BLOCK_MOD(block_buffer_head - 3);
    block_t* block[3] = { NULL, NULL, NULL };
    while (block_index != tail) {
      block_index = prev_block_index(block_index);
      block[2] = block[1];
      block[1] = block[0];
      block[0] = &block_buffer[block_index];
      planner_reverse_pass_kernel(block[0], block[1], block[2]);
    }
  }
}

// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
//当扫描计划从第一个条目到最后一个条目时，计划调用的内核_重新计算()。
void planner_forward_pass_kernel(block_t* previous, block_t* current, block_t* next) {
  if (!previous) return;
  UNUSED(next);

  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, we need to adjust the entry speed accordingly. Entry
  // speeds have already been reset, maximized, and reverse planned by reverse planner.
  // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
  if (!previous->nominal_length_flag) {
    if (previous->entry_speed < current->entry_speed) {
      double entry_speed = min(current->entry_speed,
                               max_allowable_speed(-previous->acceleration, previous->entry_speed, previous->millimeters));
      // Check for junction speed change  检查连接处的速度变化
      if (current->entry_speed != entry_speed) {
        current->entry_speed = entry_speed;
        current->recalculate_flag = true;
      }
    }
  }
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 计划_重新计算()需要重复当前计划两次。一次反向，一次向前。这个
// implements the forward pass.  实现前进通行证
void planner_forward_pass() {
  uint8_t block_index = block_buffer_tail;
  block_t* block[3] = { NULL, NULL, NULL };

  while (block_index != block_buffer_head) {
    block[0] = block[1];
    block[1] = block[2];
    block[2] = &block_buffer[block_index];
    planner_forward_pass_kernel(block[0], block[1], block[2]);
    block_index = next_block_index(block_index);
  }
  planner_forward_pass_kernel(block[1], block[2], NULL);
}

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the  重新计算计划中所有区块的梯形速度剖面图
// entry_factor for each junction. Must be called by planner_recalculate() after  每个结的入口因子。必须以计划_重新计算()调用
// updating the blocks.更新区块。
void planner_recalculate_trapezoids() {
  int8_t block_index = block_buffer_tail;
  block_t* current;
  block_t* next = NULL;

  while (block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    if (current) {
      // Recalculate if current block entry or exit junction speed has changed. 如果当前块进入或退出结速度发生变化，则重新计算。
      if (current->recalculate_flag || next->recalculate_flag) {
        // NOTE: Entry and exit factors always > 0 by all previous logic operations.注意：所有以前的逻辑运算总是>0。
        float nom = current->nominal_speed;
        calculate_trapezoid_for_block(current, current->entry_speed / nom, next->entry_speed / nom);
        current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed仅重置电流以确保计算下一个梯形
      }
    }
    block_index = next_block_index(block_index);
  }
  // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.缓冲区中的最后/最新块。退出速度以最小规划速度设定。总是重新计算。
  if (next) {
    float nom = next->nominal_speed;
    calculate_trapezoid_for_block(next, next->entry_speed / nom, MINIMUM_PLANNER_SPEED / nom);
    next->recalculate_flag = false;
  }
}

// Recalculates the motion plan according to the following algorithm:
//
//   1. Go over every block in reverse order and calculate a junction speed reduction (i.e. block_t.entry_factor)
//      so that:
//     a. The junction jerk is within the set limit
//     b. No speed reduction within one block requires faster deceleration than the one, true constant
//        acceleration.
//   2. Go over every block in chronological order and dial down junction speed reduction values if
//     a. The speed increase within one block would require faster acceleration than the one, true
//        constant acceleration.
//
// When these stages are complete all blocks have an entry_factor that will allow all speed changes to
// be performed using only the one, true constant acceleration, and where no junction jerk is jerkier than
// the set limit. Finally it will:
//
//   3. Recalculate trapezoids for all blocks.

void planner_recalculate() {
  planner_reverse_pass();
  planner_forward_pass();
  planner_recalculate_trapezoids();
}

void plan_init() {
  block_buffer_head = block_buffer_tail = 0;
  memset(position, 0, sizeof(position)); // clear position清除位置
  for (int i = 0; i < NUM_AXIS; i++) previous_speed[i] = 0.0;
  previous_nominal_speed = 0.0;
}


#if ENABLED(AUTOTEMP)
  void getHighESpeed() {
    static float oldt = 0;

    if (!autotemp_enabled) return;
    if (degTargetHotend0() + 2 < autotemp_min) return; // probably temperature set to zero. 可能温度设定为零。

    float high = 0.0;
    uint8_t block_index = block_buffer_tail;

    while (block_index != block_buffer_head) {
      block_t* block = &block_buffer[block_index];
      if (block->steps[X_AXIS] || block->steps[Y_AXIS] || block->steps[Z_AXIS]) {
        float se = (float)block->steps[E_AXIS] / block->step_event_count * block->nominal_speed; // mm/sec;
        if (se > high) high = se;
      }
      block_index = next_block_index(block_index);
    }

    float t = autotemp_min + high * autotemp_factor;
    t = constrain(t, autotemp_min, autotemp_max);
    if (oldt > t) {
      t *= (1 - AUTOTEMP_OLDWEIGHT);
      t += AUTOTEMP_OLDWEIGHT * oldt;
    }
    oldt = t;
    setTargetHotend0(t);
  }
#endif //AUTOTEMP

void check_axes_activity() {
  unsigned char axis_active[NUM_AXIS] = { 0 },
                tail_fan_speed = fanSpeed;
  #if ENABLED(BARICUDA)
    unsigned char tail_valve_pressure = ValvePressure,
                  tail_e_to_p_pressure = EtoPPressure;
  #endif

  block_t* block;

  if (blocks_queued()) {
    uint8_t block_index = block_buffer_tail;
    tail_fan_speed = block_buffer[block_index].fan_speed;
    #if ENABLED(BARICUDA)
      block = &block_buffer[block_index];
      tail_valve_pressure = block->valve_pressure;
      tail_e_to_p_pressure = block->e_to_p_pressure;
    #endif
    while (block_index != block_buffer_head) {
      block = &block_buffer[block_index];
      for (int i = 0; i < NUM_AXIS; i++) if (block->steps[i]) axis_active[i]++;
      block_index = next_block_index(block_index);
    }
  }
  if (DISABLE_X && !axis_active[X_AXIS]) disable_x();
  if (DISABLE_Y && !axis_active[Y_AXIS]) disable_y();
  if (DISABLE_Z && !axis_active[Z_AXIS]) disable_z();
  if (DISABLE_E && !axis_active[E_AXIS]) {
    disable_e0();
    disable_e1();
    disable_e2();
    disable_e3();
  }

  #if HAS_FAN
    #ifdef FAN_KICKSTART_TIME
      static millis_t fan_kick_end;
      if (tail_fan_speed) {
        millis_t ms = millis();
        if (fan_kick_end == 0) {
          // Just starting up fan - run at full power.  刚刚启动了全速的风扇。
          fan_kick_end = ms + FAN_KICKSTART_TIME;
          tail_fan_speed = 255;
        }
        else if (fan_kick_end > ms)
          // Fan still spinning up.  风扇还在转
          tail_fan_speed = 255;
        }
        else {
          fan_kick_end = 0;
        }
    #endif //FAN_KICKSTART_TIME
    #if ENABLED(FAN_MIN_PWM)
      #define CALC_FAN_SPEED (tail_fan_speed ? ( FAN_MIN_PWM + (tail_fan_speed * (255 - FAN_MIN_PWM)) / 255 ) : 0)
    #else
      #define CALC_FAN_SPEED tail_fan_speed
    #endif // FAN_MIN_PWM
    #if ENABLED(FAN_SOFT_PWM)
      fanSpeedSoftPwm = CALC_FAN_SPEED;
    #else
      analogWrite(FAN_PIN, CALC_FAN_SPEED);
    #endif // FAN_SOFT_PWM
  #endif // HAS_FAN

  #if ENABLED(AUTOTEMP)
    getHighESpeed();
  #endif

  #if ENABLED(BARICUDA)
    #if HAS_HEATER_1
      analogWrite(HEATER_1_PIN, tail_valve_pressure);
    #endif
    #if HAS_HEATER_2
      analogWrite(HEATER_2_PIN, tail_e_to_p_pressure);
    #endif
  #endif
}


float junction_deviation = 0.1;
// Add a new linear movement to the buffer. steps[X_AXIS], _y and _z is the absolute position in
// mm. Microseconds specify how many microseconds the move should take to perform. To aid acceleration
// calculation the caller must also provide the physical length of the line in millimeters.
#if ENABLED(AUTO_BED_LEVELING_FEATURE) || ENABLED(MESH_BED_LEVELING)
  void plan_buffer_line(float x, float y, float z, const float& e, float feed_rate, const uint8_t extruder)
#else
  void plan_buffer_line(const float& x, const float& y, const float& z, const float& e, float feed_rate, const uint8_t extruder)
#endif  // AUTO_BED_LEVELING_FEATURE
{
  // Calculate the buffer head after we push this byte  在我们按下这个字节后，计算缓冲头
  int next_buffer_head = next_block_index(block_buffer_head);

  // If the buffer is full: good! That means we are well ahead of the robot.如果缓冲区已经满了：很好！这意味着我们远远领先于机器人。
  // Rest here until there is room in the buffer. 在这里休息，直到有缓冲空间。
  while (block_buffer_tail == next_buffer_head) idle();

  #if ENABLED(MESH_BED_LEVELING)
    if (mbl.active) z += mbl.get_z(x, y);
  #elif ENABLED(AUTO_BED_LEVELING_FEATURE)
    apply_rotation_xyz(plan_bed_level_matrix, x, y, z);
  #endif

  // The target position of the tool in absolute steps工具的绝对步骤目标位置
  // Calculate target position in absolute steps  以绝对步数计算目标位置
  //this should be done after the wait, because otherwise a M92 code within the gcode disrupts this calculation somehow
  //这应该在等待之后完成，因为否则gcode中的m92代码会在某种程度上破坏这种计算
  long target[NUM_AXIS];
  target[X_AXIS] = lround(x * axis_steps_per_unit[X_AXIS]);
  target[Y_AXIS] = lround(y * axis_steps_per_unit[Y_AXIS]);
  target[Z_AXIS] = lround(z * axis_steps_per_unit[Z_AXIS]);
  target[E_AXIS] = lround(e * axis_steps_per_unit[E_AXIS]);

  float dx = target[X_AXIS] - position[X_AXIS],
        dy = target[Y_AXIS] - position[Y_AXIS],
        dz = target[Z_AXIS] - position[Z_AXIS];

  // DRYRUN ignores all temperature constraints and assures that the extruder is instantly satisfied干流忽略了所有的温度约束，并确保挤出机立即得到满足
  if (marlin_debug_flags & DEBUG_DRYRUN)
    position[E_AXIS] = target[E_AXIS];

  float de = target[E_AXIS] - position[E_AXIS];

  #if ENABLED(PREVENT_DANGEROUS_EXTRUDE)
    if (de) {
      if (degHotend(extruder) < extrude_min_temp) {
        position[E_AXIS] = target[E_AXIS]; // Behave as if the move really took place, but ignore E part表现得好像搬家是真的，但忽略了这一部分
        de = 0; // no difference 没有区别
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM(MSG_ERR_COLD_EXTRUDE_STOP);
      }
      #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
        if (labs(de) > axis_steps_per_unit[E_AXIS] * EXTRUDE_MAXLENGTH) {
          position[E_AXIS] = target[E_AXIS]; // Behave as if the move really took place, but ignore E part表现得好像搬家是真的，但忽略了这一部分
          de = 0; // no difference
          SERIAL_ECHO_START;
          SERIAL_ECHOLNPGM(MSG_ERR_LONG_EXTRUDE_STOP);
        }
      #endif
    }
  #endif

  // Prepare to set up new block  准备建立新的街区
  block_t* block = &block_buffer[block_buffer_head];

  // Mark block as not busy (Not executed by the stepper interrupt)  标记块为不忙（不由步行者中断执行）
  block->busy = false;

  // Number of steps for each axis  每个轴的步数
  #if ENABLED(COREXY)
    // corexy planning 合作计划
    // these equations follow the form of the dA and dB equations on http://www.corexy.com/theory.html
    block->steps[A_AXIS] = labs(dx + dy);
    block->steps[B_AXIS] = labs(dx - dy);
    block->steps[Z_AXIS] = labs(dz);
  #elif ENABLED(COREXZ)
    // corexz planning
    block->steps[A_AXIS] = labs(dx + dz);
    block->steps[Y_AXIS] = labs(dy);
    block->steps[C_AXIS] = labs(dx - dz);
  #else
    // default non-h-bot planning
    block->steps[X_AXIS] = labs(dx);
    block->steps[Y_AXIS] = labs(dy);
    block->steps[Z_AXIS] = labs(dz);
  #endif

  block->steps[E_AXIS] = labs(de);
  block->steps[E_AXIS] *= volumetric_multiplier[extruder];
  block->steps[E_AXIS] *= extruder_multiplier[extruder];
  block->steps[E_AXIS] /= 100;
  block->step_event_count = max(block->steps[X_AXIS], max(block->steps[Y_AXIS], max(block->steps[Z_AXIS], block->steps[E_AXIS])));

  // Bail if this is a zero-length block 如果这是一个零长度的障碍
  if (block->step_event_count <= dropsegments) return;

  block->fan_speed = fanSpeed;
  #if ENABLED(BARICUDA)
    block->valve_pressure = ValvePressure;
    block->e_to_p_pressure = EtoPPressure;
  #endif

  // Compute direction bits for this block 计算此块的方向位
  uint8_t db = 0;
  #if ENABLED(COREXY)
    if (dx < 0) db |= BIT(X_HEAD); // Save the real Extruder (head) direction in X Axis 在x轴上有一个真实的挤出机（头）方向
    if (dy < 0) db |= BIT(Y_HEAD); // ...and Y
    if (dz < 0) db |= BIT(Z_AXIS);
    if (dx + dy < 0) db |= BIT(A_AXIS); // Motor A direction  电机一个方向
    if (dx - dy < 0) db |= BIT(B_AXIS); // Motor B direction
  #elif ENABLED(COREXZ)
    if (dx < 0) db |= BIT(X_HEAD); // Save the real Extruder (head) direction in X Axis
    if (dy < 0) db |= BIT(Y_AXIS);
    if (dz < 0) db |= BIT(Z_HEAD); // ...and Z
    if (dx + dz < 0) db |= BIT(A_AXIS); // Motor A direction
    if (dx - dz < 0) db |= BIT(C_AXIS); // Motor B direction
  #else
    if (dx < 0) db |= BIT(X_AXIS);
    if (dy < 0) db |= BIT(Y_AXIS);
    if (dz < 0) db |= BIT(Z_AXIS);
  #endif
  if (de < 0) db |= BIT(E_AXIS);
  block->direction_bits = db;

  block->active_extruder = extruder;

  //enable active axes  启用活动轴
  #if ENABLED(COREXY)
    if (block->steps[A_AXIS] || block->steps[B_AXIS]) {
      enable_x();
      enable_y();
    }
    #if DISABLED(Z_LATE_ENABLE)
      if (block->steps[Z_AXIS]) enable_z();
    #endif
  #elif ENABLED(COREXZ)
    if (block->steps[A_AXIS] || block->steps[C_AXIS]) {
      enable_x();
      enable_z();
    }
    if (block->steps[Y_AXIS]) enable_y();
  #else
    if (block->steps[X_AXIS]) enable_x();
    if (block->steps[Y_AXIS]) enable_y();
    #if DISABLED(Z_LATE_ENABLE)
      if (block->steps[Z_AXIS]) enable_z();
    #endif
  #endif

  // Enable extruder(s)
  if (block->steps[E_AXIS]) {
    if (DISABLE_INACTIVE_EXTRUDER) { //enable only selected extruder只启用选中的挤出机

      for (int i = 0; i < EXTRUDERS; i++)
        if (g_uc_extruder_last_move[i] > 0) g_uc_extruder_last_move[i]--;

      switch(extruder) {
        case 0:
          enable_e0();
          #if ENABLED(DUAL_X_CARRIAGE)
            if (extruder_duplication_enabled) {
              enable_e1();
              g_uc_extruder_last_move[1] = BLOCK_BUFFER_SIZE * 2;
            }
          #endif
          g_uc_extruder_last_move[0] = BLOCK_BUFFER_SIZE * 2;
          #if EXTRUDERS > 1
            if (g_uc_extruder_last_move[1] == 0) disable_e1();
            #if EXTRUDERS > 2
              if (g_uc_extruder_last_move[2] == 0) disable_e2();
              #if EXTRUDERS > 3
                if (g_uc_extruder_last_move[3] == 0) disable_e3();
              #endif
            #endif
          #endif
        break;
        #if EXTRUDERS > 1
          case 1:
            enable_e1();
            g_uc_extruder_last_move[1] = BLOCK_BUFFER_SIZE * 2;
            if (g_uc_extruder_last_move[0] == 0) disable_e0();
            #if EXTRUDERS > 2
              if (g_uc_extruder_last_move[2] == 0) disable_e2();
              #if EXTRUDERS > 3
                if (g_uc_extruder_last_move[3] == 0) disable_e3();
              #endif
            #endif
          break;
          #if EXTRUDERS > 2
            case 2:
              enable_e2();
              g_uc_extruder_last_move[2] = BLOCK_BUFFER_SIZE * 2;
              if (g_uc_extruder_last_move[0] == 0) disable_e0();
              if (g_uc_extruder_last_move[1] == 0) disable_e1();
              #if EXTRUDERS > 3
                if (g_uc_extruder_last_move[3] == 0) disable_e3();
              #endif
            break;
            #if EXTRUDERS > 3
              case 3:
                enable_e3();
                g_uc_extruder_last_move[3] = BLOCK_BUFFER_SIZE * 2;
                if (g_uc_extruder_last_move[0] == 0) disable_e0();
                if (g_uc_extruder_last_move[1] == 0) disable_e1();
                if (g_uc_extruder_last_move[2] == 0) disable_e2();
              break;
            #endif // EXTRUDERS > 3
          #endif // EXTRUDERS > 2
        #endif // EXTRUDERS > 1
      }
    }
    else { // enable all
      enable_e0();
      enable_e1();
      enable_e2();
      enable_e3();
    }
  }

  if (block->steps[E_AXIS])
    NOLESS(feed_rate, minimumfeedrate);
  else
    NOLESS(feed_rate, mintravelfeedrate);

  /**
   * This part of the code calculates the total length of the movement.
   * For cartesian bots, the X_AXIS is the real X movement and same for Y_AXIS.
   * But for corexy bots, that is not true. The "X_AXIS" and "Y_AXIS" motors (that should be named to A_AXIS
   * and B_AXIS) cannot be used for X and Y length, because A=X+Y and B=X-Y.
   * So we need to create other 2 "AXIS", named X_HEAD and Y_HEAD, meaning the real displacement of the Head.
   * Having the real displacement of the head, we can calculate the total movement length and apply the desired speed.
  代码的这一部分计算移动的总长度。
*对于笛卡尔机器人，x_轴是实x运动，而对于y_轴则相同。
*但对于核心机器人，这是不正确的。"x_轴"和"y_轴"电机(应命名为a_轴)
*和b_轴）不能用于x和y长度，因为a=x+y和b=x-y。
*所以我们需要创建另外2个"轴"，命名为x_head和y_head，意思是头部的真正位移。
*有了头部的真实位移，我们可以计算出总运动长度并应用所需的速度。 */
  #if ENABLED(COREXY)
    float delta_mm[6];
    delta_mm[X_HEAD] = dx / axis_steps_per_unit[A_AXIS];
    delta_mm[Y_HEAD] = dy / axis_steps_per_unit[B_AXIS];
    delta_mm[Z_AXIS] = dz / axis_steps_per_unit[Z_AXIS];
    delta_mm[A_AXIS] = (dx + dy) / axis_steps_per_unit[A_AXIS];
    delta_mm[B_AXIS] = (dx - dy) / axis_steps_per_unit[B_AXIS];
  #elif ENABLED(COREXZ)
    float delta_mm[6];
    delta_mm[X_HEAD] = dx / axis_steps_per_unit[A_AXIS];
    delta_mm[Y_AXIS] = dy / axis_steps_per_unit[Y_AXIS];
    delta_mm[Z_HEAD] = dz / axis_steps_per_unit[C_AXIS];
    delta_mm[A_AXIS] = (dx + dz) / axis_steps_per_unit[A_AXIS];
    delta_mm[C_AXIS] = (dx - dz) / axis_steps_per_unit[C_AXIS];
  #else
    float delta_mm[4];
    delta_mm[X_AXIS] = dx / axis_steps_per_unit[X_AXIS];
    delta_mm[Y_AXIS] = dy / axis_steps_per_unit[Y_AXIS];
    delta_mm[Z_AXIS] = dz / axis_steps_per_unit[Z_AXIS];
  #endif
  delta_mm[E_AXIS] = (de / axis_steps_per_unit[E_AXIS]) * volumetric_multiplier[extruder] * extruder_multiplier[extruder] / 100.0;

  if (block->steps[X_AXIS] <= dropsegments && block->steps[Y_AXIS] <= dropsegments && block->steps[Z_AXIS] <= dropsegments) {
    block->millimeters = fabs(delta_mm[E_AXIS]);
  }
  else {
    block->millimeters = sqrt(
      #if ENABLED(COREXY)
        square(delta_mm[X_HEAD]) + square(delta_mm[Y_HEAD]) + square(delta_mm[Z_AXIS])
      #elif ENABLED(COREXZ)
        square(delta_mm[X_HEAD]) + square(delta_mm[Y_AXIS]) + square(delta_mm[Z_HEAD])
      #else
        square(delta_mm[X_AXIS]) + square(delta_mm[Y_AXIS]) + square(delta_mm[Z_AXIS])
      #endif
    );
  }
  float inverse_millimeters = 1.0 / block->millimeters;  // Inverse millimeters to remove multiple divides反毫米来移除多个鸿沟

  // Calculate speed in mm/second for each axis. No divide by zero due to previous checks.为每个轴计算速度，单位为mm/秒。由于以前的检查，没有除以零
  float inverse_second = feed_rate * inverse_millimeters;

  int moves_queued = movesplanned();

  // Slow down when the buffer starts to empty, rather than wait at the corner for a buffer refill当缓冲区开始清空时，放慢速度，而不是在角落等待缓冲区填充
  #if ENABLED(OLD_SLOWDOWN) || ENABLED(SLOWDOWN)
    bool mq = moves_queued > 1 && moves_queued < BLOCK_BUFFER_SIZE / 2;
    #if ENABLED(OLD_SLOWDOWN)
      if (mq) feed_rate *= 2.0 * moves_queued / BLOCK_BUFFER_SIZE;
    #endif
    #if ENABLED(SLOWDOWN)
      //  segment time im micro seconds  分段时间是微秒
      unsigned long segment_time = lround(1000000.0/inverse_second);
      if (mq) {
        if (segment_time < minsegmenttime) {
          // buffer is draining, add extra time.  The amount of time added increases if the buffer is still emptied more.
          //缓冲正在耗尽，增加额外的时间。如果缓冲区仍然被更多地清空，则添加的时间会增加
          inverse_second = 1000000.0 / (segment_time + lround(2 * (minsegmenttime - segment_time) / moves_queued));
          #ifdef XY_FREQUENCY_LIMIT
            segment_time = lround(1000000.0 / inverse_second);
          #endif
        }
      }
    #endif
  #endif

  block->nominal_speed = block->millimeters * inverse_second; // (mm/sec) Always > 0
  block->nominal_rate = ceil(block->step_event_count * inverse_second); // (step/sec) Always > 0

  #if ENABLED(FILAMENT_SENSOR)
    //FMM update ring buffer used for delay with filament measurementsfmm更新用于延迟测量灯丝的环形缓冲器

    if (extruder == FILAMENT_SENSOR_EXTRUDER_NUM && delay_index2 > -1) {  //only for extruder with filament sensor and if ring buffer is initialized
                              //仅适用于带有灯丝传感器的挤出机，如果环形缓冲器已初始化
      const int MMD = MAX_MEASUREMENT_DELAY + 1, MMD10 = MMD * 10;

      delay_dist += delta_mm[E_AXIS];  // increment counter with next move in e axis 下一步在E轴移动的增量计数器
      while (delay_dist >= MMD10) delay_dist -= MMD10; // loop around the buffer 在缓冲区周围循环
      while (delay_dist < 0) delay_dist += MMD10;

      delay_index1 = delay_dist / 10.0;  // calculate index  计算索引
      delay_index1 = constrain(delay_index1, 0, MAX_MEASUREMENT_DELAY); // (already constrained above)上文已有限制

      if (delay_index1 != delay_index2) { // moved index
        meas_sample = widthFil_to_size_ratio() - 100;  // Subtract 100 to reduce magnitude - to store in a signed char减去100，以降低大小-存储在一个有符号字符
        while (delay_index1 != delay_index2) {
          // Increment and loop around buffer
          if (++delay_index2 >= MMD) delay_index2 -= MMD;
          delay_index2 = constrain(delay_index2, 0, MAX_MEASUREMENT_DELAY);
          measurement_delay[delay_index2] = meas_sample;
        }
      }
    }
  #endif

  // Calculate and limit speed in mm/sec for each axis//计算和限制每个轴的速度，单位为毫米/秒
  float current_speed[NUM_AXIS];
  float speed_factor = 1.0; //factor <=1 do decrease speed
  for (int i = 0; i < NUM_AXIS; i++) {
    current_speed[i] = delta_mm[i] * inverse_second;
    float cs = fabs(current_speed[i]), mf = max_feedrate[i];
    if (cs > mf) speed_factor = min(speed_factor, mf / cs);
  }

  // Max segement time in us.
  #ifdef XY_FREQUENCY_LIMIT

    // Check and limit the xy direction change frequency  检查并限制XY方向变化频率
    unsigned char direction_change = block->direction_bits ^ old_direction_bits;
    old_direction_bits = block->direction_bits;
    segment_time = lround((float)segment_time / speed_factor);

    long xs0 = axis_segment_time[X_AXIS][0],
         xs1 = axis_segment_time[X_AXIS][1],
         xs2 = axis_segment_time[X_AXIS][2],
         ys0 = axis_segment_time[Y_AXIS][0],
         ys1 = axis_segment_time[Y_AXIS][1],
         ys2 = axis_segment_time[Y_AXIS][2];

    if ((direction_change & BIT(X_AXIS)) != 0) {
      xs2 = axis_segment_time[X_AXIS][2] = xs1;
      xs1 = axis_segment_time[X_AXIS][1] = xs0;
      xs0 = 0;
    }
    xs0 = axis_segment_time[X_AXIS][0] = xs0 + segment_time;

    if ((direction_change & BIT(Y_AXIS)) != 0) {
      ys2 = axis_segment_time[Y_AXIS][2] = axis_segment_time[Y_AXIS][1];
      ys1 = axis_segment_time[Y_AXIS][1] = axis_segment_time[Y_AXIS][0];
      ys0 = 0;
    }
    ys0 = axis_segment_time[Y_AXIS][0] = ys0 + segment_time;

    long max_x_segment_time = max(xs0, max(xs1, xs2)),
         max_y_segment_time = max(ys0, max(ys1, ys2)),
         min_xy_segment_time = min(max_x_segment_time, max_y_segment_time);
    if (min_xy_segment_time < MAX_FREQ_TIME) {
      float low_sf = speed_factor * min_xy_segment_time / MAX_FREQ_TIME;
      speed_factor = min(speed_factor, low_sf);
    }
  #endif // XY_FREQUENCY_LIMIT

  // Correct the speed
  if (speed_factor < 1.0) {
    for (unsigned char i = 0; i < NUM_AXIS; i++) current_speed[i] *= speed_factor;
    block->nominal_speed *= speed_factor;
    block->nominal_rate *= speed_factor;
  }

  // Compute and limit the acceleration rate for the trapezoid generator.  /计算并限制梯形发电机的加速速度。
  float steps_per_mm = block->step_event_count / block->millimeters;
  long bsx = block->steps[X_AXIS], bsy = block->steps[Y_AXIS], bsz = block->steps[Z_AXIS], bse = block->steps[E_AXIS];
  if (bsx == 0 && bsy == 0 && bsz == 0) {
    block->acceleration_st = ceil(retract_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2 转换为：加速步骤/秒^2
  }
  else if (bse == 0) {
    block->acceleration_st = ceil(travel_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
  }
  else {
    block->acceleration_st = ceil(acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
  }
  // Limit acceleration per axis  轴的极限加速度
  unsigned long acc_st = block->acceleration_st,
                xsteps = axis_steps_per_sqr_second[X_AXIS],
                ysteps = axis_steps_per_sqr_second[Y_AXIS],
                zsteps = axis_steps_per_sqr_second[Z_AXIS],
                esteps = axis_steps_per_sqr_second[E_AXIS];
  if ((float)acc_st * bsx / block->step_event_count > xsteps) acc_st = xsteps;
  if ((float)acc_st * bsy / block->step_event_count > ysteps) acc_st = ysteps;
  if ((float)acc_st * bsz / block->step_event_count > zsteps) acc_st = zsteps;
  if ((float)acc_st * bse / block->step_event_count > esteps) acc_st = esteps;

  block->acceleration_st = acc_st;
  block->acceleration = acc_st / steps_per_mm;
  block->acceleration_rate = (long)(acc_st * 16777216.0 / (F_CPU / 8.0));

  #if 0  // Use old jerk for now
    // Compute path unit vector
    double unit_vec[3];

    unit_vec[X_AXIS] = delta_mm[X_AXIS] * inverse_millimeters;
    unit_vec[Y_AXIS] = delta_mm[Y_AXIS] * inverse_millimeters;
    unit_vec[Z_AXIS] = delta_mm[Z_AXIS] * inverse_millimeters;

    // Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
    // Let a circle be tangent to both previous and current path line segments, where the junction
    // deviation is defined as the distance from the junction to the closest edge of the circle,
    // colinear with the circle center. The circular segment joining the two paths represents the
    // path of centripetal acceleration. Solve for max velocity based on max acceleration about the
    // radius of the circle, defined indirectly by junction deviation. This may be also viewed as
    // path width or max_jerk in the previous grbl version. This approach does not actually deviate
    // from path, but used as a robust way to compute cornering speeds, as it takes into account the
    // nonlinearities of both the junction angle and junction velocity.
    /*用向心加速度近似法计算结处的最大允许进入速度。
//设一个圆与前面的和现在的路径线段相切，其中的结
//偏差定义为从结到圆的最近边的距离，
//与圆心共线。连接两条路径的循环段表示
//向心加速度的路径.基于最大加速度的最大速度解
//圆的半径，由结偏差间接定义。这也可视为
//路径宽度或上一grbl版本中的最大挺举。这种方法实际上并没有偏离
//来自路径，但作为计算过弯速度的可靠方法，因为它考虑到了
//结合角和结速度的非线性。*/
    double vmax_junction = MINIMUM_PLANNER_SPEED; // Set default max junction speed设定预设的最大结速度

    // Skip first block or when previous_nominal_speed is used as a flag for homing and offset cycles.
    //跳过第一块，或者在前面的“名义速度”被用作引导和偏移周期的标志时。
    if ((block_buffer_head != block_buffer_tail) && (previous_nominal_speed > 0.0)) {
      // Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
      // NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
      //计算上一个路径和当前路径夹角的余弦值。（推尔·纽特·维克为阴性）
      //注意：最大结速度是通过三角半角恒等式计算的，不含sin()或ACO()。
      double cos_theta = - previous_unit_vec[X_AXIS] * unit_vec[X_AXIS]
                         - previous_unit_vec[Y_AXIS] * unit_vec[Y_AXIS]
                         - previous_unit_vec[Z_AXIS] * unit_vec[Z_AXIS] ;
      // Skip and use default max junction speed for 0 degree acute junction.
      if (cos_theta < 0.95) {
        vmax_junction = min(previous_nominal_speed, block->nominal_speed);
        // Skip and avoid divide by zero for straight junctions at 180 degrees. Limit to min() of nominal speeds.
        //对于180度的直线结，跳过并避免除以零。限制为名义速度的最小()。
        if (cos_theta > -0.95) {
          // Compute maximum junction velocity based on maximum acceleration and junction deviation//根据最大加速度和结偏差计算最大结速度
          double sin_theta_d2 = sqrt(0.5 * (1.0 - cos_theta)); // Trig half angle identity. Always positive.三角半角同一性。总是积极的。
          vmax_junction = min(vmax_junction,
                              sqrt(block->acceleration * junction_deviation * sin_theta_d2 / (1.0 - sin_theta_d2)));
        }
      }
    }
  #endif

  // Start with a safe speed  以安全的速度开始
  float vmax_junction = max_xy_jerk / 2;
  float vmax_junction_factor = 1.0;
  float mz2 = max_z_jerk / 2, me2 = max_e_jerk / 2;
  float csz = current_speed[Z_AXIS], cse = current_speed[E_AXIS];
  if (fabs(csz) > mz2) vmax_junction = min(vmax_junction, mz2);
  if (fabs(cse) > me2) vmax_junction = min(vmax_junction, me2);
  vmax_junction = min(vmax_junction, block->nominal_speed);
  float safe_speed = vmax_junction;

  if ((moves_queued > 1) && (previous_nominal_speed > 0.0001)) {
    float dx = current_speed[X_AXIS] - previous_speed[X_AXIS],
          dy = current_speed[Y_AXIS] - previous_speed[Y_AXIS],
          dz = fabs(csz - previous_speed[Z_AXIS]),
          de = fabs(cse - previous_speed[E_AXIS]),
          jerk = sqrt(dx * dx + dy * dy);

    //    if ((fabs(previous_speed[X_AXIS]) > 0.0001) || (fabs(previous_speed[Y_AXIS]) > 0.0001)) {
    vmax_junction = block->nominal_speed;
    //    }
    if (jerk > max_xy_jerk) vmax_junction_factor = max_xy_jerk / jerk;
    if (dz > max_z_jerk) vmax_junction_factor = min(vmax_junction_factor, max_z_jerk / dz);
    if (de > max_e_jerk) vmax_junction_factor = min(vmax_junction_factor, max_e_jerk / de);

    vmax_junction = min(previous_nominal_speed, vmax_junction * vmax_junction_factor); // Limit speed to max previous speed将速度限制为最大速度
  }
  block->max_entry_speed = vmax_junction;

  // Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.//初始化块条目速度。基于减速到用户定义的最小规划速度的计算。
  double v_allowable = max_allowable_speed(-block->acceleration, MINIMUM_PLANNER_SPEED, block->millimeters);
  block->entry_speed = min(vmax_junction, v_allowable);

  // Initialize planner efficiency flags
  // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
  // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
  // the current block and next block junction speeds are guaranteed to always be at their maximum
  // junction speeds in deceleration and acceleration, respectively. This is due to how the current
  // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
  // the reverse and forward planners, the corresponding block junction speed will always be at the
  // the maximum junction speed and may always be ignored for any speed reduction checks.
  /*//初始化计划效率标志
//设置标志，如果块将始终达到最大连接速度，无论进入/退出速度。
//如果块可以在块的长度内从名义速度变为零，那么
//保证当前的块结速度和下一个块结速度总是在最大范围内
//结速度分别在减速和加速中。这是由于当前如何
//阻塞标称速度限制了当前和下一个最大结速度。因此，在这两方面
//反向和向前规划者，相应的块结速度总是在
//最大结速度，在任何减速检查中都可能被忽略。*/
  block->nominal_length_flag = (block->nominal_speed <= v_allowable);
  block->recalculate_flag = true; // Always calculate trapezoid for new block总是计算新块的梯形

  // Update previous path unit_vector and nominal speed  更新上一个路径单位-向量和名义速度
  for (int i = 0; i < NUM_AXIS; i++) previous_speed[i] = current_speed[i];
  previous_nominal_speed = block->nominal_speed;

  #if ENABLED(ADVANCE)
    // Calculate advance rate   计算预付款
    if (!bse || (!bsx && !bsy && !bsz)) {
      block->advance_rate = 0;
      block->advance = 0;
    }
    else {
      long acc_dist = estimate_acceleration_distance(0, block->nominal_rate, block->acceleration_st);
      float advance = (STEPS_PER_CUBIC_MM_E * EXTRUDER_ADVANCE_K) * (cse * cse * EXTRUSION_AREA * EXTRUSION_AREA) * 256;
      block->advance = advance;
      block->advance_rate = acc_dist ? advance / (float)acc_dist : 0;
    }
    /*
      SERIAL_ECHO_START;
     SERIAL_ECHOPGM("advance :");
     SERIAL_ECHO(block->advance/256.0);
     SERIAL_ECHOPGM("advance rate :");
     SERIAL_ECHOLN(block->advance_rate/256.0);
     */
  #endif // ADVANCE

  calculate_trapezoid_for_block(block, block->entry_speed / block->nominal_speed, safe_speed / block->nominal_speed);

  // Move buffer head
  block_buffer_head = next_buffer_head;

  // Update position
  for (int i = 0; i < NUM_AXIS; i++) position[i] = target[i];

  planner_recalculate();

  st_wake_up();

} // plan_buffer_line()

#if ENABLED(AUTO_BED_LEVELING_FEATURE) && DISABLED(DELTA)
  vector_3 plan_get_position() {
    vector_3 position = vector_3(st_get_position_mm(X_AXIS), st_get_position_mm(Y_AXIS), st_get_position_mm(Z_AXIS));

    //position.debug("in plan_get position");   调试（“在计划中获得位置”）；    
    //plan_bed_level_matrix.debug("in plan_get_position");
    matrix_3x3 inverse = matrix_3x3::transpose(plan_bed_level_matrix);
    //inverse.debug("in plan_get inverse");
    position.apply_rotation(inverse);
    //position.debug("after rotation");

    return position;
  }
#endif // AUTO_BED_LEVELING_FEATURE && !DELTA   自动床平整功能&&！三角洲

#if ENABLED(AUTO_BED_LEVELING_FEATURE) || ENABLED(MESH_BED_LEVELING)
  void plan_set_position(float x, float y, float z, const float& e)
#else
  void plan_set_position(const float& x, const float& y, const float& z, const float& e)
#endif // AUTO_BED_LEVELING_FEATURE || MESH_BED_LEVELING
  {
    #if ENABLED(MESH_BED_LEVELING)
      if (mbl.active) z += mbl.get_z(x, y);
    #elif ENABLED(AUTO_BED_LEVELING_FEATURE)
      apply_rotation_xyz(plan_bed_level_matrix, x, y, z);
    #endif

    float nx = position[X_AXIS] = lround(x * axis_steps_per_unit[X_AXIS]),
          ny = position[Y_AXIS] = lround(y * axis_steps_per_unit[Y_AXIS]),
          nz = position[Z_AXIS] = lround(z * axis_steps_per_unit[Z_AXIS]),
          ne = position[E_AXIS] = lround(e * axis_steps_per_unit[E_AXIS]);
    st_set_position(nx, ny, nz, ne);
    previous_nominal_speed = 0.0; // Resets planner junction speeds. Assumes start from rest.重置计划连接速度。假设从休息开始

    for (int i = 0; i < NUM_AXIS; i++) previous_speed[i] = 0.0;
  }

void plan_set_e_position(const float& e) {
  position[E_AXIS] = lround(e * axis_steps_per_unit[E_AXIS]);
  st_set_e_position(position[E_AXIS]);
}

// Calculate the steps/s^2 acceleration rates, based on the mm/s^s/ 根据毫米/秒的加速度计算步速/秒^2
void reset_acceleration_rates() {
  for (int i = 0; i < NUM_AXIS; i++)
    axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
}
