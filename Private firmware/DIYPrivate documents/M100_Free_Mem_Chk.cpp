#define M100_FREE_MEMORY_DUMPER     // Comment out to remove Dump sub-command   注释以移除转储子命令
#define M100_FREE_MEMORY_CORRUPTOR    // Comment out to remove Corrupt sub-command  注释以移除损坏的子命令


// M100 Free Memory Watcher  M100免费记忆观察者
//
// This code watches the free memory block between the bottom of the heap and the top of the stack.
//此代码监视堆底部和堆栈顶部之间的空闲内存块。
// This memory block is initialized and watched via the M100 command.这个内存块是通过m100命令初始化并监视的。
//
// M100 I Initializes the free memory block and prints vitals statistics about the area
//M100i初始化自由存储块并打印有关该区域的生命统计数据
// M100 F Identifies how much of the free memory block remains free and unused.  It also
//M100f表示自由存储块中有多少仍然是空闲的和未使用的。它也是
//    detects and reports any corruption within the free memory block that may have
//检测并报告自由存储块中可能存在的任何腐败
//    happened due to errant firmware.    出现在错误的固件上
// M100 D Does a hex display of the free memory block along with a flag for any errant
//M100d用于显示自由存储块的十六进制显示，并为任何出错的对象加上一个标志
//    data that does not match the expected value.  与预期值不匹配的数据。
// M100 C x Corrupts x locations within the free memory block.   This is useful to check the
//M100c X会损坏自由内存块中的X位置。这是很有用的检查
//    correctness of the M100 F and M100 D commands. M100f和M100d命令的正确性。
//
// Initial version by Roxy-3DPrintBoard   初始版本由roxy-3drprint板
//
//


#include "Marlin.h"

#if ENABLED(M100_FREE_MEMORY_WATCHER)    //如果启用（M100免费内存观察者）
extern void* __brkval;
extern size_t  __heap_start, __heap_end, __flp;


//
// Declare all the functions we need from Marlin_Main.cpp to do the work!
//向马林公司申报我们需要的所有功能，以便完成这项工作！
//

float code_value();
long code_value_long();
bool code_seen(char);
void serial_echopair_P(const char*, float);
void serial_echopair_P(const char*, double);
void serial_echopair_P(const char*, unsigned long);
void serial_echopair_P(const char*, int);
void serial_echopair_P(const char*, long);




//
// Utility functions used by M100 to get its work done.m100用于完成其工作的实用函数。
//

unsigned char* top_of_stack();
void prt_hex_nibble(unsigned int);
void prt_hex_byte(unsigned int);
void prt_hex_word(unsigned int);
int how_many_E5s_are_here(unsigned char*);




void gcode_M100() {
  static int m100_not_initialized = 1;
  unsigned char* sp, *ptr;
  int i, j, n;
  //
  // M100 D dumps the free memory block from __brkval to the stack pointer.
  // malloc() eats memory from the start of the block and the stack grows
  // up from the bottom of the block.    Solid 0xE5's indicate nothing has
  // used that memory yet.   There should not be anything but 0xE5's within
  // the block of 0xE5's.  If there is, that would indicate memory corruption
  // probably caused by bad pointers.  Any unexpected values will be flagged in
  // the right hand column to help spotting them.
// m100d将空闲内存块从__brkval转储到堆栈指针。
//malloc()从块开始吃内存，堆栈就会增长
//从街区的底部往上。实心0xe5表示没有任何东西
//用过那段记忆了吗？里面除了0xe5以外什么都没有
//0xe5的区块。如果有的话，那就说明你的记忆被破坏了
//可能是错误的指针造成的。任何意外值将会在
//右手栏，以帮助发现它们。
  //
#if ENABLED(M100_FREE_MEMORY_DUMPER) // Disable to remove Dump sub-command禁用以移除转储子命令
  if (code_seen('D')) {
    ptr = (unsigned char*) __brkval;
    //
    // We want to start and end the dump on a nice 16 byte boundry even though
    //我们要开始和结束在一个漂亮的16字节的边界上的倾倒，即使
    // the values we are using are not 16 byte aligned. 我们正在使用的值不是16个字节对齐的。
    //
    SERIAL_ECHOPGM("\n__brkval : ");
    prt_hex_word((unsigned int) ptr);
    ptr = (unsigned char*)((unsigned long) ptr & 0xfff0);
    sp = top_of_stack();
    SERIAL_ECHOPGM("\nStack Pointer : ");
    prt_hex_word((unsigned int) sp);
    SERIAL_ECHOPGM("\n");
    sp = (unsigned char*)((unsigned long) sp | 0x000f);
    n = sp - ptr;
    //
    // This is the main loop of the Dump command.这是转储命令的主循环。
    //
    while (ptr < sp) {
      prt_hex_word((unsigned int) ptr); // Print the address  打印地址
      SERIAL_ECHOPGM(":");
      for (i = 0; i < 16; i++) {      // and 16 data bytes  以及16个数据字节
        prt_hex_byte(*(ptr + i));
        SERIAL_ECHOPGM(" ");
        delay(2);
      }
      SERIAL_ECHO("|");         // now show where non 0xE5's are  现在显示非0xe5的位置
      for (i = 0; i < 16; i++) {
        delay(2);
        if (*(ptr + i) == 0xe5)
          SERIAL_ECHOPGM(" ");
        else
          SERIAL_ECHOPGM("?");
      }
      SERIAL_ECHO("\n");
      ptr += 16;
      delay(2);
    }
    SERIAL_ECHOLNPGM("Done.\n");
    return;
  }
#endif
  //
  // M100 F   requests the code to return the number of free bytes in the memory pool along with
  //M100f要求代码返回内存池中的自由字节数，并将
  // other vital statistics that define the memory pool. 其他定义内存池的生命统计数据。
  //
  if (code_seen('F')) {
    int max_addr = (int) __brkval;
    int max_cnt = 0;
    int block_cnt = 0;
    ptr = (unsigned char*) __brkval;
    sp = top_of_stack();
    n = sp - ptr;
    // Scan through the range looking for the biggest block of 0xE5's we can find
    //扫描范围寻找最大的0xe5的块，我们可以找到
    for (i = 0; i < n; i++) {
      if (*(ptr + i) == (unsigned char) 0xe5) {
        j = how_many_E5s_are_here((unsigned char*) ptr + i);
        if (j > 8) {
          SERIAL_ECHOPAIR("Found ", j);
          SERIAL_ECHOPGM(" bytes free at 0x");
          prt_hex_word((int) ptr + i);
          SERIAL_ECHOPGM("\n");
          i += j;
          block_cnt++;
        }
        if (j > max_cnt) {      // We don't do anything with this information yet 我们还没对这些信息做任何事情
          max_cnt  = j;     // but we do know where the biggest free memory block is.  但我们知道最大的自由存储块在哪。
          max_addr = (int) ptr + i;
        }
      }
    }
    if (block_cnt > 1)
      SERIAL_ECHOLNPGM("\nMemory Corruption detected in free memory area.\n");// 在自由内存区域检测到内存损坏。\N"）
    SERIAL_ECHO("\nDone.\n"); 
    return;
  }
  //
  // M100 C x  Corrupts x locations in the free memory pool and reports the locations of the corruption.
  // This is useful to check the correctness of the M100 D and the M100 F commands.
 //M100c X会损坏自由内存池中的X位置，并报告损坏的位置。
//这对检查M100d和m100f命令的正确性很有用。
  //
#if ENABLED(M100_FREE_MEMORY_CORRUPTOR)
  if (code_seen('C')) {
    int x;      // x gets the # of locations to corrupt within the memory pool X得到内存池中要损坏的位置的#
    x = code_value();
    SERIAL_ECHOLNPGM("Corrupting free memory block.\n");
    ptr = (unsigned char*) __brkval;
    SERIAL_ECHOPAIR("\n__brkval : ", (long) ptr);
    ptr += 8;
    sp = top_of_stack();
    SERIAL_ECHOPAIR("\nStack Pointer : ", (long) sp);
    SERIAL_ECHOLNPGM("\n");
    n = sp - ptr - 64;    // -64 just to keep us from finding interrupt activity that  -64只是为了不让我们发现
    // has altered the stack.  已经改变了堆叠。
    j = n / (x + 1);
    for (i = 1; i <= x; i++) {
      *(ptr + (i * j)) = i;
      SERIAL_ECHO("\nCorrupting address: 0x");
      prt_hex_word((unsigned int)(ptr + (i * j)));
    }
    SERIAL_ECHOLNPGM("\n");
    return;
  }
#endif
  //
  // M100 I    Initializes the free memory pool so it can be watched and prints vital
  // statistics that define the free memory pool.
  //m100i初始化自由内存池，这样它就可以被监视并打印至关重要的内容
  //统计定义了自由内存池。
  //
  if (m100_not_initialized || code_seen('I')) {       // If no sub-command is specified, the first time 如果没有指定子命令，则在第一次
    SERIAL_ECHOLNPGM("Initializing free memory block.\n");    // this happens, it will Initialize.  如果发生这种情况，它将初始化
    ptr = (unsigned char*) __brkval;        // Repeated M100 with no sub-command will not destroy the  没有子命令的重复m100将不会破坏
    SERIAL_ECHOPAIR("\n__brkval : ", (long) ptr);     // state of the initialized free memory pool.  初始化自由内存池的状态。
    ptr += 8;
    sp = top_of_stack();
    SERIAL_ECHOPAIR("\nStack Pointer : ", (long) sp);
    SERIAL_ECHOLNPGM("\n");
    n = sp - ptr - 64;    // -64 just to keep us from finding interrupt activity that -64只是为了不让我们发现中断活动
    // has altered the stack.
    SERIAL_ECHO(n);
    SERIAL_ECHOLNPGM(" bytes of memory initialized.\n");
    for (i = 0; i < n; i++)
      *(ptr + i) = (unsigned char) 0xe5;
    for (i = 0; i < n; i++) {
      if (*(ptr + i) != (unsigned char) 0xe5) {
        SERIAL_ECHOPAIR("? address : ", (unsigned long) ptr + i);
        SERIAL_ECHOPAIR("=", *(ptr + i));
        SERIAL_ECHOLNPGM("\n");
      }
    }
    m100_not_initialized = 0;
    SERIAL_ECHOLNPGM("Done.\n");
    return;
  }
  return;
}

// top_of_stack() returns the location of a variable on its stack frame.  The value returned is above
//top_of_stack()返回变量在其堆栈框架上的位置。返回的值高于
// the stack once the function returns to the caller.
//一旦函数返回到调用方，就开始堆栈。

unsigned char* top_of_stack() {
  unsigned char x;
  return &x + 1; // x is pulled on return;  X在返回时被拉动
} 

//
// 3 support routines to print hex numbers.  We can print a nibble, byte and word
//3个支持例程来打印十六进制数字。我们可以打印一个小字，字节和单词
//

void prt_hex_nibble(unsigned int n) {
  if (n <= 9)
    SERIAL_ECHO(n);
  else
    SERIAL_ECHO((char)('A' + n - 10));
  delay(2);
}

void prt_hex_byte(unsigned int b) {
  prt_hex_nibble((b & 0xf0) >> 4);
  prt_hex_nibble(b & 0x0f);
}

void prt_hex_word(unsigned int w) {
  prt_hex_byte((w & 0xff00) >> 8);
  prt_hex_byte(w & 0x0ff);
}

// how_many_E5s_are_here() is a utility function to easily find out how many 0xE5's are
//这里有多少个e5()是一个实用函数，可以很容易地找出有多少个0xe5
// at the specified location.  Having this logic as a function simplifies the search code.
//在指定的位置。将此逻辑作为一个函数，可以简化搜索代码。
//
int how_many_E5s_are_here(unsigned char* p) {
  int n;
  for (n = 0; n < 32000; n++) {
    if (*(p + n) != (unsigned char) 0xe5)
      return n - 1;
  }
  return -1;
}

#endif

