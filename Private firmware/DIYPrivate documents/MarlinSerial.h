/*
  HardwareSerial.h - Hardware serial library for Wiring
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

  Modified 28 September 2010 by Mark Sproul
  硬件布线系列硬件库
版权所有（C）2006 Nicholas Zambetti。版权所有。
这个库是免费软件，您可以重新分配它和/或
在GNU小公众的条款下修改它
自由软件基金会发布的许可证；
版本2.1的许可证，或（在你的选择）任何后来的版本。
这个图书馆是分发的，希望它有用。
但没有任何保证，甚至没有默示保证。
适销性或适合某一特定目的的适销性。看GNU
较低的一般公共许可证的更多细节。
你应该已经收到了GNU小公众的拷贝。
许可证与此库一起；如果没有，则写入免费软件
基金会，51富兰克林ST，第五楼，波士顿，美国02110-1301
Mark Sproul 2010年9月28日修正案
*/

#ifndef MarlinSerial_h
#define MarlinSerial_h
#include "Marlin.h"

#ifndef SERIAL_PORT
  #define SERIAL_PORT 0
#endif

// The presence of the UBRRH register is used to detect a UART.UBRRH寄存器的存在被用于检测UART。
#define UART_PRESENT(port) ((port == 0 && (defined(UBRRH) || defined(UBRR0H))) || \
                            (port == 1 && defined(UBRR1H)) || (port == 2 && defined(UBRR2H)) || \
                            (port == 3 && defined(UBRR3H)))

// These are macros to build serial port register names for the selected SERIAL_PORT (C preprocessor
//这些宏用于为选定的串行端口（C预处理器）构建串行端口寄存器名称。需要两个间接级别来适当地扩展宏值
// requires two levels of indirection to expand macro values properly)
#define SERIAL_REGNAME(registerbase,number,suffix) SERIAL_REGNAME_INTERNAL(registerbase,number,suffix)
#if SERIAL_PORT == 0 && (!defined(UBRR0H) || !defined(UDR0)) // use un-numbered registers if necessary必要时使用未编号寄存器
  #define SERIAL_REGNAME_INTERNAL(registerbase,number,suffix) registerbase##suffix
#else
  #define SERIAL_REGNAME_INTERNAL(registerbase,number,suffix) registerbase##number##suffix
#endif

// Registers used by MarlinSerial class (these are expanded
// depending on selected serial port MARLIN系列类使用的寄存器（这些已展开）取决于所选择的串行端口
#define M_UCSRxA SERIAL_REGNAME(UCSR,SERIAL_PORT,A) 
// defines M_UCSRxA to be UCSRnA where n is the serial port number 定义M UCSRxA为UCSRNA，其中N是串行端口号
#define M_UCSRxB SERIAL_REGNAME(UCSR,SERIAL_PORT,B)
#define M_RXENx SERIAL_REGNAME(RXEN,SERIAL_PORT,)
#define M_TXENx SERIAL_REGNAME(TXEN,SERIAL_PORT,)
#define M_RXCIEx SERIAL_REGNAME(RXCIE,SERIAL_PORT,)
#define M_UDREx SERIAL_REGNAME(UDRE,SERIAL_PORT,)
#define M_UDRx SERIAL_REGNAME(UDR,SERIAL_PORT,)
#define M_UBRRxH SERIAL_REGNAME(UBRR,SERIAL_PORT,H)
#define M_UBRRxL SERIAL_REGNAME(UBRR,SERIAL_PORT,L)
#define M_RXCx SERIAL_REGNAME(RXC,SERIAL_PORT,)
#define M_USARTx_RX_vect SERIAL_REGNAME(USART,SERIAL_PORT,_RX_vect)
#define M_U2Xx SERIAL_REGNAME(U2X,SERIAL_PORT,)


#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define BYTE 0


#ifndef USBCON
// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which rx_buffer_head is the index of the
// location to which to write the next incoming character and rx_buffer_tail
// is the index of the location from which to read.
//定义缓冲输入串行数据的常数和变量。我们是
//使用环缓冲器（我认为），其中RxxBuffelyHead是索引
//写入下一个传入字符和RXX缓冲尾的位置
//是要读取的位置的索引。
#define RX_BUFFER_SIZE 128


struct ring_buffer {
  unsigned char buffer[RX_BUFFER_SIZE];
  int head;
  int tail;
};

#if UART_PRESENT(SERIAL_PORT)
  extern ring_buffer rx_buffer;
#endif

class MarlinSerial { //: public Stream

  public:
    MarlinSerial();
    void begin(long);
    void end();
    int peek(void);
    int read(void);
    void flush(void);

    FORCE_INLINE int available(void) {
      return (unsigned int)(RX_BUFFER_SIZE + rx_buffer.head - rx_buffer.tail) % RX_BUFFER_SIZE;
    }

    FORCE_INLINE void write(uint8_t c) {
      while (!TEST(M_UCSRxA, M_UDREx))
        ;
      M_UDRx = c;
    }

    FORCE_INLINE void checkRx(void) {
      if (TEST(M_UCSRxA, M_RXCx)) {
        unsigned char c  =  M_UDRx;
        int i = (unsigned int)(rx_buffer.head + 1) % RX_BUFFER_SIZE;

        // if we should be storing the received character into the location
        // just before the tail (meaning that the head would advance to the
        // current location of the tail), we're about to overflow the buffer
        // and so we don't write the character or advance the head.
        //如果我们应该将接收到的字符存储到位置
        //就在尾巴的前面（意思是头朝前）
       //当前的尾部位置，我们即将溢出缓冲区。我们不写这个字，也不写头。
        if (i != rx_buffer.tail) {
          rx_buffer.buffer[rx_buffer.head] = c;
          rx_buffer.head = i;
        }
      }
    }

  private:
    void printNumber(unsigned long, uint8_t);
    void printFloat(double, uint8_t);

  public:
    FORCE_INLINE void write(const char* str) { while (*str) write(*str++); }
    FORCE_INLINE void write(const uint8_t* buffer, size_t size) { while (size--) write(*buffer++); }
    FORCE_INLINE void print(const String& s) { for (int i = 0; i < (int)s.length(); i++) write(s[i]); }
    FORCE_INLINE void print(const char* str) { write(str); }

    void print(char, int = BYTE);
    void print(unsigned char, int = BYTE);
    void print(int, int = DEC);
    void print(unsigned int, int = DEC);
    void print(long, int = DEC);
    void print(unsigned long, int = DEC);
    void print(double, int = 2);

    void println(const String& s);
    void println(const char[]);
    void println(char, int = BYTE);
    void println(unsigned char, int = BYTE);
    void println(int, int = DEC);
    void println(unsigned int, int = DEC);
    void println(long, int = DEC);
    void println(unsigned long, int = DEC);
    void println(double, int = 2);
    void println(void);
};

extern MarlinSerial customizedSerial;
#endif // !USBCON

// Use the UART for Bluetooth in AT90USB configurations在AT90USB配置中使用蓝牙的UART
#if defined(USBCON) && ENABLED(BLUETOOTH)
  extern HardwareSerial bluetoothSerial;
#endif

#endif
