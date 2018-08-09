/*
  HardwareSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
  硬件串口库，用于布线
(c)Nicholas Zambetti。所有权利保留。
此库是免费软件；您可以重新分发它和/或
根据一般民众的条件修改
由自由软件基金会公布的许可证；
许可证的版本2.1,或(根据您的选择)任何后来的版本.
这个图书馆是为了希望有用而发行的，
但没有任何保证；甚至没有
适销性或适合某一特定目的的。看见了吗？
较少的一般公共许可证的更多细节。
你应该已经收到了一份次要大众的副本。
许可证连同这个图书馆一起；如果没有，就写信给免费软件
美国波士顿五楼51街，美国马0211-1301号
David A. Mellis于2006年11月23日修改
Mark Sproul于2010年9月28日修改
*/

#include "Marlin.h"
#include "MarlinSerial.h"

#ifndef USBCON
// this next line disables the entire HardwareSerial.cpp,  下一行将禁用整个硬页页页
// this is so I can support Attiny series and any other chip without a UART
//这是为了让我能够支持有关联的系列和任何其他没有uart的芯片
#if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H) || defined(UBRR2H) || defined(UBRR3H)

#if UART_PRESENT(SERIAL_PORT)
  ring_buffer rx_buffer  =  { { 0 }, 0, 0 };
#endif

FORCE_INLINE void store_char(unsigned char c) {
  int i = (unsigned int)(rx_buffer.head + 1) % RX_BUFFER_SIZE;

  // if we should be storing the received character into the location//如果我们应该将接收到的字符存储到位置中
  // just before the tail (meaning that the head would advance to the //就在尾巴之前（意思是头部会向前推进到
  // current location of the tail), we're about to overflow the buffer //尾部的当前位置)，我们即将溢出缓冲区
  // and so we don't write the character or advance the head.  //所以我们不写这个角色也不把它的头往前推。
  if (i != rx_buffer.tail) {
    rx_buffer.buffer[rx_buffer.head] = c;
    rx_buffer.head = i;
  }
}


//#elif defined(SIG_USART_RECV)
#if defined(M_USARTx_RX_vect)
  // fixed by Mark Sproul this is on the 644/644p
  //SIGNAL(SIG_USART_RECV)
  SIGNAL(M_USARTx_RX_vect) {
    unsigned char c  =  M_UDRx;
    store_char(c);
  }
#endif

// Constructors ////////////////////////////////////////////////////////////////

MarlinSerial::MarlinSerial() { }

// Public Methods //////////////////////////////////////////////////////////////

void MarlinSerial::begin(long baud) {
  uint16_t baud_setting;
  bool useU2X = true;

  #if F_CPU == 16000000UL && SERIAL_PORT == 0
    // hard-coded exception for compatibility with the bootloader shipped  与已发运的引导加载程序兼容性的硬编码异常
    // with the Duemilanove and previous boards and the firmware on the 8U2  在8u2上有双关板和前几块板和固件
    // on the Uno and Mega 2560.
    if (baud == 57600) {
      useU2X = false;
    }
  #endif

  if (useU2X) {
    M_UCSRxA = BIT(M_U2Xx);
    baud_setting = (F_CPU / 4 / baud - 1) / 2;
  }
  else {
    M_UCSRxA = 0;
    baud_setting = (F_CPU / 8 / baud - 1) / 2;
  }

  // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
  M_UBRRxH = baud_setting >> 8;
  M_UBRRxL = baud_setting;

  sbi(M_UCSRxB, M_RXENx);
  sbi(M_UCSRxB, M_TXENx);
  sbi(M_UCSRxB, M_RXCIEx);
}

void MarlinSerial::end() {
  cbi(M_UCSRxB, M_RXENx);
  cbi(M_UCSRxB, M_TXENx);
  cbi(M_UCSRxB, M_RXCIEx);
}


int MarlinSerial::peek(void) {
  if (rx_buffer.head == rx_buffer.tail) {
    return -1;
  }
  else {
    return rx_buffer.buffer[rx_buffer.tail];
  }
}

int MarlinSerial::read(void) {
  // if the head isn't ahead of the tail, we don't have any characters
  //如果头不在尾巴的前面，我们就没有文字。
  if (rx_buffer.head == rx_buffer.tail) {
    return -1;
  }
  else {
    unsigned char c = rx_buffer.buffer[rx_buffer.tail];
    rx_buffer.tail = (unsigned int)(rx_buffer.tail + 1) % RX_BUFFER_SIZE;
    return c;
  }
}

void MarlinSerial::flush() {
  // don't reverse this or there may be problems if the RX interrupt
  // occurs after reading the value of rx_buffer_head but before writing
  // the value to rx_buffer_tail; the previous value of rx_buffer_head
  // may be written to rx_buffer_tail, making it appear as if the buffer
  // don't reverse this or there may be problems if the RX interrupt
  // occurs after reading the value of rx_buffer_head but before writing
  // the value to rx_buffer_tail; the previous value of rx_buffer_head
  // may be written to rx_buffer_tail, making it appear as if the buffer
  // were full, not empty.
  /*不要反转，否则如果RX中断可能会出现问题。
读取RX缓冲头的值，但在写入之前发生
Rx Buffe尾部的值；Rx1 BuffelyHead的前一个值
可以写入RX缓冲区尾部，使其看起来像缓冲器。
不要反转，否则如果RX中断可能会出现问题。
读取RX缓冲头的值，但在写入之前发生
对Rx$BuffelyT尾的值；Rx1BuffelyHead的前一个值
可以写到RxxBuffelyT尾，使其看起来像缓冲器。
满是，不是空的。*/
  rx_buffer.head = rx_buffer.tail;
}


/// imports from print.h


void MarlinSerial::print(char c, int base) {
  print((long) c, base);
}

void MarlinSerial::print(unsigned char b, int base) {
  print((unsigned long) b, base);
}

void MarlinSerial::print(int n, int base) {
  print((long) n, base);
}

void MarlinSerial::print(unsigned int n, int base) {
  print((unsigned long) n, base);
}

void MarlinSerial::print(long n, int base) {
  if (base == 0) {
    write(n);
  }
  else if (base == 10) {
    if (n < 0) {
      print('-');
      n = -n;
    }
    printNumber(n, 10);
  }
  else {
    printNumber(n, base);
  }
}

void MarlinSerial::print(unsigned long n, int base) {
  if (base == 0) write(n);
  else printNumber(n, base);
}

void MarlinSerial::print(double n, int digits) {
  printFloat(n, digits);
}

void MarlinSerial::println(void) {
  print('\r');
  print('\n');
}

void MarlinSerial::println(const String& s) {
  print(s);
  println();
}

void MarlinSerial::println(const char c[]) {
  print(c);
  println();
}

void MarlinSerial::println(char c, int base) {
  print(c, base);
  println();
}

void MarlinSerial::println(unsigned char b, int base) {
  print(b, base);
  println();
}

void MarlinSerial::println(int n, int base) {
  print(n, base);
  println();
}

void MarlinSerial::println(unsigned int n, int base) {
  print(n, base);
  println();
}

void MarlinSerial::println(long n, int base) {
  print(n, base);
  println();
}

void MarlinSerial::println(unsigned long n, int base) {
  print(n, base);
  println();
}

void MarlinSerial::println(double n, int digits) {
  print(n, digits);
  println();
}

// Private Methods /////////////////////////////////////////////////////////////

void MarlinSerial::printNumber(unsigned long n, uint8_t base) {
  unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars.
  unsigned long i = 0;

  if (n == 0) {
    print('0');
    return;
  }

  while (n > 0) {
    buf[i++] = n % base;
    n /= base;
  }

  for (; i > 0; i--)
    print((char)(buf[i - 1] < 10 ?
                 '0' + buf[i - 1] :
                 'A' + buf[i - 1] - 10));
}

void MarlinSerial::printFloat(double number, uint8_t digits) {
  // Handle negative numbers
  if (number < 0.0) {
    print('-');
    number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  print(int_part);

  // Print the decimal point, but only if there are digits beyond打印小数点，但仅当有数字超出
  if (digits > 0) print('.');

  // Extract digits from the remainder one at a time从剩余的一次提取一个数字
  while (digits-- > 0) {
    remainder *= 10.0;
    int toPrint = int(remainder);
    print(toPrint);
    remainder -= toPrint;
  }
}
// Preinstantiate Objects //////////////////////////////////////////////////////
//预实例化对象

MarlinSerial customizedSerial;//马林系列定制系列；

#endif // whole file
#endif // !USBCON

// For AT90USB targets use the UART for BT interfacing对目标使用AT90USB BT的UART接口
#if defined(USBCON) && ENABLED(BLUETOOTH)
  HardwareSerial bluetoothSerial; //硬件串口蓝牙串口
#endif
