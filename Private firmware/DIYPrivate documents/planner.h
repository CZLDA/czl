/*
  planner.h - buffers movement commands and manages the acceleration profile plan缓冲移动命令和管理加速配置文件计划
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

// This module is to be considered a sub-module of stepper.c. Please don't include  本模块被认为是步进的子模块。请不要包括在内
// this file from any other module.此文件来自任何其他模块。

#ifndef PLANNER_H
#define PLANNER_H

#include "Marlin.h"

// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in
// the source g-code and may never actually be reached if acceleration management is active.
//此结构用于缓冲每个线性运动的设置“名义值”，如
//来源G-Code，并且如果加速管理是活动的，可能永远无法真正达到。
typedef struct {
  // Fields used by the bresenham algorithm for tracing the line
  long steps[NUM_AXIS];                     // Step count along each axis  沿各轴的步数
  unsigned long step_event_count;           // The number of step events required to complete this block完成此块所需的步骤事件的数目
  long accelerate_until;                    // The index of the step event on which to stop acceleration停止加速度的步进事件的指数
  long decelerate_after;                    // The index of the step event on which to start decelerating开始减速的步骤事件的索引
  long acceleration_rate;                   // The acceleration rate used for acceleration calculation用于计算加速度的加速度
  unsigned char direction_bits;             // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)为此块设置的方向位（在配置中指*_指定位）
  unsigned char active_extruder;            // Selects the active extruder选择活动挤出机
  #if ENABLED(ADVANCE)
    long advance_rate;
    volatile long initial_advance;
    volatile long final_advance;
    float advance;
  #endif

  // Fields used by the motion planner to manage acceleration 运动规划师用于管理加速度的字段
  // float speed_x, speed_y, speed_z, speed_e;          // Nominal mm/sec for each axis 每个轴的名义毫米/秒
  float nominal_speed;                               // The nominal speed for this block in mm/sec  此区块的标称速度(mm/秒)
  float entry_speed;                                 // Entry speed at previous-current junction in mm/sec   在上一个电流结处的输入速度(毫米/秒)
  float max_entry_speed;                             // Maximum allowable junction entry speed in mm/sec  最大允许结入口速度(毫米/秒)
  float millimeters;                                 // The total travel of this block in mm 这个街区的总行程(毫米)
  float acceleration;                                // acceleration mm/sec^2  加速m/sec^2
  unsigned char recalculate_flag;                    // Planner flag to recalculate trapezoids on entry junction  规划标志重新计算入口路口的梯形
  unsigned char nominal_length_flag;                 // Planner flag for nominal speed always reached  标称速度的规划标志总是达到

  // Settings for the trapezoid generator  梯形生成器的设置
  unsigned long nominal_rate;                        // The nominal step rate for this block in step_events/sec  此块在步骤_事件/秒中的名义步长
  unsigned long initial_rate;                        // The jerk-adjusted step rate at start of block  区块开始时的齿轮调整步进率
  unsigned long final_rate;                          // The minimal rate at exit   退出时的最低利率
  unsigned long acceleration_st;                     // acceleration steps/sec^2  加速步骤/秒^2
  unsigned long fan_speed;
  #if ENABLED(BARICUDA)
    unsigned long valve_pressure;
    unsigned long e_to_p_pressure;
  #endif
  volatile char busy;
} block_t;

#define BLOCK_MOD(n) ((n)&(BLOCK_BUFFER_SIZE-1))

// Initialize the motion plan subsystem   初始化运动计划子系统
void plan_init();

void check_axes_activity();

// Get the number of buffered moves   得到缓冲移动的次数
extern volatile unsigned char block_buffer_head;
extern volatile unsigned char block_buffer_tail;
FORCE_INLINE uint8_t movesplanned() { return BLOCK_MOD(block_buffer_head - block_buffer_tail + BLOCK_BUFFER_SIZE); }

#if ENABLED(AUTO_BED_LEVELING_FEATURE) || ENABLED(MESH_BED_LEVELING)

  #if ENABLED(AUTO_BED_LEVELING_FEATURE)
    #include "vector_3.h"

    // Transform required to compensate for bed level  为补偿床位水平所需的改造
    extern matrix_3x3 plan_bed_level_matrix;

    /**
     * Get the position applying the bed level matrix  得到应用床层矩阵的位置
     */
    vector_3 plan_get_position();
  #endif  // AUTO_BED_LEVELING_FEATURE

  /**
   * Add a new linear movement to the buffer. x, y, z are the signed, absolute target position in向缓冲区添加新的线性移动。X,Y,Z是在
   * millimeters. Feed rate specifies the (target) speed of the motion.  输入速率指定运动的（目标）速度。
   */
  void plan_buffer_line(float x, float y, float z, const float& e, float feed_rate, const uint8_t extruder);

  /**
   * Set the planner positions. Used for G92 instructions.
   * Multiplies by axis_steps_per_unit[] to set stepper positions.
   * Clears previous speed values.  清除先前的速度值
   */
  void plan_set_position(float x, float y, float z, const float& e);

#else

  void plan_buffer_line(const float& x, const float& y, const float& z, const float& e, float feed_rate, const uint8_t extruder);
  void plan_set_position(const float& x, const float& y, const float& z, const float& e);

#endif // AUTO_BED_LEVELING_FEATURE || MESH_BED_LEVELING

void plan_set_e_position(const float& e);

//===========================================================================
//========================== public variables 公共变量=======================
//===========================================================================

extern millis_t minsegmenttime;
extern float max_feedrate[NUM_AXIS]; // Max speeds in mm per minute  最大速度(毫米/分钟)
extern float axis_steps_per_unit[NUM_AXIS];
extern unsigned long max_acceleration_units_per_sq_second[NUM_AXIS]; // Use M201 to override by software使用m201以软体取代
extern float minimumfeedrate;
extern float acceleration;         // Normal acceleration mm/s^2  DEFAULT ACCELERATION for all printing moves. M204 SXXXX所有打印动作的正常加速度mm/s^2默认加速度。m 204 xxxxx
extern float retract_acceleration; // Retract acceleration mm/s^2 filament pull-back and push-forward while standing still in the other axes M204 TXXXX在其他轴上静止不动时，收回加速度m/s^2长丝拉回和向前推
extern float travel_acceleration;  // Travel acceleration mm/s^2  DEFAULT ACCELERATION for all NON printing moves. M204 MXXXX
extern float max_xy_jerk;          // The largest speed change requiring no acceleration 不需要加速的最大速度变化
extern float max_z_jerk;
extern float max_e_jerk;
extern float mintravelfeedrate;
extern unsigned long axis_steps_per_sqr_second[NUM_AXIS];

#if ENABLED(AUTOTEMP)
  extern bool autotemp_enabled;
  extern float autotemp_max;
  extern float autotemp_min;
  extern float autotemp_factor;
#endif

extern block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instructions  用于运动指令的环形缓冲器
extern volatile unsigned char block_buffer_head;           // Index of the next block to be pushed  下一个要按的区块的索引
extern volatile unsigned char block_buffer_tail;

// Returns true if the buffer has a queued block, false otherwise  如果缓冲区有队列块，返回真，否则返回假
FORCE_INLINE bool blocks_queued() { return (block_buffer_head != block_buffer_tail); }

// Called when the current block is no longer needed. Discards   当不再需要当前块时调用
// the block and makes the memory available for new blocks. 并使内存可用于新块。
FORCE_INLINE void plan_discard_current_block() {
  if (blocks_queued())
    block_buffer_tail = BLOCK_MOD(block_buffer_tail + 1);
}

// Gets the current block. Returns NULL if buffer empty  得到当前块。如果缓冲区为空，则返回空
FORCE_INLINE block_t* plan_get_current_block() {
  if (blocks_queued()) {
    block_t* block = &block_buffer[block_buffer_tail];
    block->busy = true;
    return block;
  }
  else
    return NULL;
}

void reset_acceleration_rates();

#endif // PLANNER_H
