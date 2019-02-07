/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
/* SAMD20E18
 * +------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * +  E    Board pin  |  PIN   | Label/Name      | Comments (* is for default peripheral in use)
 * +------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | Digital Low      |        |                 |
 * +------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 0 -> RX          |  PA25  | 0               | SERCOM3/PAD[3]
 * | 1 <- TX          |  PA24  | 1               | SERCOM3/PAD[2]
 * | 2                |  PA11  | 2               | GPIO
 * | 3                |  PA14  | 3               | GPIO
 * | 4                |  PA15  | 4               | GPIO
 * | 5                |  PA16  | 5               | GPIO
 * | 6                |  PA17  | 6               | GPIO
 * | 7                |  PA18  | 7               | GPIO
 * +------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | Digital High     |        |                 |
 * +------------------+--------+-----------------+-------------------------------------------------------------------------------------------------------- 
 * | 8                |  PA19  | 8               | GPIO
 * | 9                |  PA22  | 9               | GPIO
 * | 10               |  PA23  | 10              | GPIO
 * | 11               |  PA27  | 11              | GPIO
 * | 12               |  PA28  | 12              | GPIO
 * +------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | Analog Connector |        |                 |
 * +------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | A0               |  PA02  | 13/A0           | AIN[0]  DAC/VOUT
 * | A1               |  PA04  | 14/A1           | AIN[4]  
 * | A2               |  PA05  | 15/A2           | AIN[5]  
 * | A3               |  PA06  | 16/A3           | AIN[6] 
 * | A4               |  PA07  | 17/A4           | AIN[7] 
 * | A5               |  PA10  | 18/A5           | AIN[18]
 * +------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | Wire             |        |                 |
 * +------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | SDA              |  PA08  | 19/SDA          | SERCOM0/PAD[0]
 * | SCL              |  PA09  | 20/SCL          | SERCOM0/PAD[1]
 * +------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |SPI (Legacy ICSP) |        |                 |
 * +------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 1                |  PA19  | 21/MISO         | SERCOM1/PAD[3]
 * | 2                |        | 5V0             |
 * | 4                |  PA16  | 22/MOSI         | SERCOM1/PAD[0]
 * | 3                |  PA17  | 23/SCK          | SERCOM1/PAD[1]
 * | 5                |        | RESET           |
 * | 6                |        | GND             |
 * +------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |SPI 2             |        |                 |
 * +------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |                  |  PA08  | 24/MOSI         | SERCOM0/PAD[0]
 * |                  |  PA09  | 25/SCK          | SERCOM0/PAD[1]
 * |                  |  PA11  | 26/MISO         | SERCOM0/PAD[3]
 * +------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | Debug/Flashing   |        |                 |
 * +------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | RESET            | /RESET |                 |
 * | SWCLK            | PA30   |                 |
 * | SWDIO            | PA31   |                 |
 * +------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | AREF/DAC         |        |                 |
 * +------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | AREF             |  PA03  | 27              | AIN[1]
 * | DAC/VOUT         |  PA02  | 28/A0           | AIN[0]  DAC/VOUT
 * +------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |32.768KHz Crystal |        |                 |
 * +------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |                  | PA00   | XIN32           |
 * |                  | PA01   | XOUT32          |
 * +------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 */

#include "variant.h"

// Pin descriptions
const PinDescription g_APinDescription[]=
{
  // 0..12 - Digital pins
  // ----------------------
  // 0/1 - SERCOM/UART (Serial1)
  { PORTA, 25, PIO_SERCOM, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13 }, // RX: SERCOM3/PAD[3]
  { PORTA, 24, PIO_SERCOM, (PIN_ATTR_DIGITAL), No_ADC_Channel, PWM5_CH0, TC5_CH0, EXTERNAL_INT_12 }, // TX: SERCOM3/PAD[2]

  // 2..12
  // Digital Low
  { PORTA, 11, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, 
  { PORTA, 14, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, PWM3_CH0, TC3_CH0, EXTERNAL_INT_14 }, 
  { PORTA, 15, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 }, 
  { PORTA, 16, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, PWM2_CH0, TC2_CH0, EXTERNAL_INT_0 }, 
  { PORTA, 17, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, 
  { PORTA, 18, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },

  // Digital High
  { PORTA, 19, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 },
  { PORTA, 22, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, PWM4_CH0, TC4_CH0, EXTERNAL_INT_6 }, 
  { PORTA, 23, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, 
  { PORTA, 27, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },
  { PORTA, 28, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 },

  // 13..18 - Analog pins
  // --------------------
  { PORTA,  2, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_DIGITAL), ADC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // ADC/AIN[0]
  { PORTA,  4, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_DIGITAL), ADC_Channel4, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 }, // ADC/AIN[4]
  { PORTA,  5, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_DIGITAL), ADC_Channel5, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 }, // ADC/AIN[5]
  { PORTA,  6, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_DIGITAL), ADC_Channel6, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 }, // ADC/AIN[6]
  { PORTA,  7, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_DIGITAL), ADC_Channel7, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, // ADC/AIN[7]
  { PORTA, 10, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_DIGITAL), ADC_Channel18, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, // ADC/AIN[18]

  // 19..20 I2C pins (SDA/SCL)
  // ----------------------
  { PORTA,  8, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NMI }, // SDA: SERCOM0/PAD[0]
  { PORTA,  9, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 }, // SCL: SERCOM0/PAD[1]

  // 21..23 - SPI pins (ICSP:MISO,SCK,MOSI)
  // ----------------------
  { PORTA, 19, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 }, // MISO: SERCOM1/PAD[0]
  { PORTA, 16, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 }, // MOSI: SERCOM1/PAD[2]
  { PORTA, 17, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, // SCK: SERCOM1/PAD[3]

  // 24..26 - SPI2 pins
  // ----------------------
  { PORTA,  8, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NMI }, // MOSI: SERCOM0/PAD[0]
  { PORTA,  9, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },   // SCK: SERCOM0/PAD[1]
  { PORTA, 11, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },  // MISO: SERCOM0/PAD[3]

  // 27 (AREF)
  { PORTA, 3, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // DAC/VREFP

  // ----------------------
  // 28 - Alternate use of A0 (DAC output)
  { PORTA, 2, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // DAC/VOUT  
} ;

// TODO: See if we need this
const void *g_apTCInstances[TC_INST_NUM] = {TC0, TC1, TC2, TC3, TC4, TC5};

// Sercom objects
SERCOM sercom0( SERCOM0 );
SERCOM sercom1( SERCOM1 );
SERCOM sercom2( SERCOM2 );
SERCOM sercom3( SERCOM3 );

Uart Serial( &sercom3, PIN_SERIAL_RX, PIN_SERIAL_TX, PAD_SERIAL_RX, PAD_SERIAL_TX );

void SERCOM3_Handler()
{
    Serial.IrqHandler();
}

// Timer counter objects
TimerCounter Timer( TC2 );
TimerCounter Timer1( TC3 );
TimerCounter Timer2( TC4 );
TimerCounter Timer3( TC5 );

// PWM objects
PWM PWMChannel0( &Timer );
PWM PWMChannel1( &Timer1 );
PWM PWMChannel2( &Timer2 );
PWM PWMChannel3( &Timer3 );

void TC2_Handler()
{
    Timer.IrqHandler();
}

void TC3_Handler()
{
    Timer1.IrqHandler();
}

void TC4_Handler()
{
    Timer2.IrqHandler();
}

void TC5_Handler()
{
    Timer3.IrqHandler();
}

// Emulated EEPROM
EEEPROM EEPROM;