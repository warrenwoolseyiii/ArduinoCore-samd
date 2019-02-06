/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

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

#pragma once

#include <stdint.h>
#include "sam.h"

#ifdef __cplusplus
extern "C" {
#endif

// Definitions for analog channels
typedef enum _EAnalogChannel {
    No_ADC_Channel = -1,
    ADC_Channel0 = 0,
    ADC_Channel1 = 1,
    ADC_Channel2 = 2,
    ADC_Channel3 = 3,
    ADC_Channel4 = 4,
    ADC_Channel5 = 5,
    ADC_Channel6 = 6,
    ADC_Channel7 = 7,
    ADC_Channel8 = 8,
    ADC_Channel9 = 9,
    ADC_Channel10 = 10,
    ADC_Channel11 = 11,
    ADC_Channel12 = 12,
    ADC_Channel13 = 13,
    ADC_Channel14 = 14,
    ADC_Channel15 = 15,
    ADC_Channel16 = 16,
    ADC_Channel17 = 17,
    ADC_Channel18 = 18,
    ADC_Channel19 = 19,
    DAC_Channel0,
} EAnalogChannel;

// Definitions for TC channels
typedef enum _ETCChannel {
    NOT_ON_TIMER = -1,

#if defined( __SAMD20E18__ )
    TC0_CH0 = ( 0 << 8 ) | ( 0 ),
    TC0_CH1 = ( 0 << 8 ) | ( 1 ),
    TC1_CH0 = ( 1 << 8 ) | ( 0 ),
    TC1_CH1 = ( 1 << 8 ) | ( 1 ),
    TC2_CH0 = ( 2 << 8 ) | ( 0 ),
    TC2_CH1 = ( 2 << 8 ) | ( 1 ),
    TC3_CH0 = ( 3 << 8 ) | ( 0 ),
    TC3_CH1 = ( 3 << 8 ) | ( 1 ),
    TC4_CH0 = ( 4 << 8 ) | ( 0 ),
    TC4_CH1 = ( 4 << 8 ) | ( 1 ),
    TC5_CH0 = ( 5 << 8 ) | ( 0 ),
    TC5_CH1 = ( 5 << 8 ) | ( 1 ),
#endif /* __SAMD20E18__ */

    //#if defined( __SAMD21G18__ )
    // TCC0_CH0 = (0<<8)|(0),
    // TCC0_CH1 = (0<<8)|(1),
    // TCC0_CH2 = (0<<8)|(2),
    // TCC0_CH3 = (0<<8)|(3),
    // TCC0_CH4 = (0<<8)|(0), // Channel 4 is 0!
    // TCC0_CH5 = (0<<8)|(1), // Channel 5 is 1!
    // TCC0_CH6 = (0<<8)|(2), // Channel 6 is 2!
    // TCC0_CH7 = (0<<8)|(3), // Channel 7 is 3!
    // TCC1_CH0 = (1<<8)|(0),
    // TCC1_CH1 = (1<<8)|(1),
    // TCC1_CH2 = (1<<8)|(0), // Channel 2 is 0!
    // TCC1_CH3 = (1<<8)|(1), // Channel 3 is 1!
    // TCC2_CH0 = (2<<8)|(0),
    // TCC2_CH1 = (2<<8)|(1),
    // TCC2_CH2 = (2<<8)|(0), // Channel 2 is 0!
    // TCC2_CH3 = (2<<8)|(1), // Channel 3 is 1!
    //#endif /* __SAMD21G18__ */

    //#if defined( __SAMD21J18A__ )
    // TC6_CH0  = (6<<8)|(0),
    // TC6_CH1  = (6<<8)|(1),
    // TC7_CH0  = (7<<8)|(0),
    // TC7_CH1  = (7<<8)|(1),
    //#endif /* __SAMD21J18A__ */
} ETCChannel;

extern const void *g_apTCInstances[TC_INST_NUM];

#define GetTCNumber( x ) ( ( x ) >> 8 )
#define GetTCChannelNumber( x ) ( (x)&0xff )
#define GetTC( x ) ( g_apTCInstances[( x ) >> 8] )

// Definitions for PWM channels
typedef enum _EPWMChannel {
    NOT_ON_PWM = -1,

#if defined( __SAMD20E18__ )
    PWM0_CH0 = TC0_CH0,
    PWM0_CH1 = TC0_CH1,
    PWM1_CH0 = TC1_CH0,
    PWM1_CH1 = TC1_CH1,
    PWM2_CH0 = TC2_CH0,
    PWM2_CH1 = TC2_CH1,
    PWM3_CH0 = TC3_CH0,
    PWM3_CH1 = TC3_CH1,
    PWM4_CH0 = TC4_CH0,
    PWM4_CH1 = TC4_CH1,
    PWM5_CH0 = TC5_CH0,
    PWM5_CH1 = TC5_CH1,
#endif /* __SAMD20E18__ */

    //#if( defined( __SAMD21J18A__ ) || defined( __SAMD20J18__ ) )
    // PWM0_CH0 = TCC0_CH0,
    // PWM0_CH1 = TCC0_CH1,
    // PWM0_CH2 = TCC0_CH2,
    // PWM0_CH3 = TCC0_CH3,
    // PWM0_CH4 = TCC0_CH4,
    // PWM0_CH5 = TCC0_CH5,
    // PWM0_CH6 = TCC0_CH6,
    // PWM0_CH7 = TCC0_CH7,
    // PWM1_CH0 = TCC1_CH0,
    // PWM1_CH1 = TCC1_CH1,
    // PWM1_CH2 = TCC1_CH2,
    // PWM1_CH3 = TCC1_CH3,
    // PWM2_CH0 = TCC2_CH0,
    // PWM2_CH1 = TCC2_CH1,
    // PWM2_CH2 = TCC2_CH2,
    // PWM2_CH3 = TCC2_CH3,
    // PWM6_CH0 = TC6_CH0,
    // PWM6_CH1 = TC6_CH1,
    // PWM7_CH0 = TC7_CH0,
    // PWM7_CH1 = TC7_CH1,
    //#endif // __SAMD21J18A__ || __SAMD20J18__

} EPWMChannel;

typedef enum _EPortType {
    NOT_A_PORT = -1,
    PORTA = 0,
    PORTB = 1,
    PORTC = 2,
} EPortType;

typedef enum {
    EXTERNAL_INT_0 = 0,
    EXTERNAL_INT_1,
    EXTERNAL_INT_2,
    EXTERNAL_INT_3,
    EXTERNAL_INT_4,
    EXTERNAL_INT_5,
    EXTERNAL_INT_6,
    EXTERNAL_INT_7,
    EXTERNAL_INT_8,
    EXTERNAL_INT_9,
    EXTERNAL_INT_10,
    EXTERNAL_INT_11,
    EXTERNAL_INT_12,
    EXTERNAL_INT_13,
    EXTERNAL_INT_14,
    EXTERNAL_INT_15,
    EXTERNAL_INT_NMI,
    EXTERNAL_NUM_INTERRUPTS,
    NOT_AN_INTERRUPT = -1,
    EXTERNAL_INT_NONE = NOT_AN_INTERRUPT,
} EExt_Interrupts;

typedef enum _EPioType {
    PIO_NOT_A_PIN = -1,
    PIO_EXTINT = 0,
    PIO_ANALOG,
    PIO_SERCOM,
    PIO_SERCOM_ALT,
    PIO_TIMER,
    PIO_TIMER_ALT,
    PIO_COM,
    PIO_AC_CLK,
    PIO_DIGITAL,
    PIO_INPUT,
    PIO_INPUT_PULLUP,
    PIO_OUTPUT,
    PIO_PWM = PIO_TIMER,
    PIO_PWM_ALT = PIO_TIMER_ALT,
} EPioType;

#define PIN_ATTR_NONE ( 0UL << 0 )
#define PIN_ATTR_COMBO ( 1UL << 0 )
#define PIN_ATTR_ANALOG ( 1UL << 1 )
#define PIN_ATTR_DIGITAL ( 1UL << 2 )
#define PIN_ATTR_PWM ( 1UL << 3 )
#define PIN_ATTR_TIMER ( 1UL << 4 )
#define PIN_ATTR_TIMER_ALT ( 1UL << 5 )
#define PIN_ATTR_EXTINT ( 1UL << 6 )

// Pin description struct
typedef struct _PinDescription
{
    EPortType       ulPort;
    uint32_t        ulPin;
    EPioType        ulPinType;
    uint32_t        ulPinAttribute;
    EAnalogChannel  ulADCChannelNumber;
    EPWMChannel     ulPWMChannel;
    ETCChannel      ulTCChannel;
    EExt_Interrupts ulExtInt;
} PinDescription;

// Pins table to be instantiated into variant.cpp
extern const PinDescription g_APinDescription[];

#ifdef __cplusplus
} // extern "C"
#endif
