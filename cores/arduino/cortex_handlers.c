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

#include <sam.h>
#include <stdio.h>
#include <system_samd20.h>
#include "variant.h"

/* RTOS Hooks */
extern void svcHook( void );
extern void pendSVHook( void );
extern int  sysTickHook( void );

/* Default empty handler */
void Dummy_Handler( void )
{
#if defined DEBUG
    __BKPT( 3 );
#endif
    for( ;; ) {
    }
}

// Cortex M0+ handlers
void HardFault_Handler( void )
    __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
void Reset_Handler( void );
void NMI_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
void SVC_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
void PendSV_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
void SysTick_Handler( void );

// Peripheral handlers
void PM_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
void SYSCTRL_Handler( void )
    __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
void WDT_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
void RTC_Handler( void );
void EIC_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
void NVMCTRL_Handler( void )
    __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
void EVSYS_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
void SERCOM0_Handler( void )
    __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
void SERCOM1_Handler( void )
    __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
void SERCOM2_Handler( void )
    __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
void SERCOM3_Handler( void )
    __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
void TC0_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
void TC1_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
void TC2_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
void TC3_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
void TC4_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
void TC5_Handler( void ) __attribute__( ( weak ) );
void ADC_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
void AC_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
void DAC_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
void PTC_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );

//#ifndef SAMD20
//void I2S_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
//#endif // SAMD20
//
//#ifndef SAMD20
//void DMAC_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
//void USB_Handler( void ) __attribute__( ( weak ) );
//#endif // SAMD20
//
//#ifndef __SAMD20E18__
//void SERCOM4_Handler( void )
//__attribute__( ( weak, alias( "Dummy_Handler" ) ) );
//void SERCOM5_Handler( void )
//__attribute__( ( weak, alias( "Dummy_Handler" ) ) );
//#endif // __SAMD20E18__
//
//#ifndef SAMD20
//void TCC0_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
//void TCC1_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
//void TCC2_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
//#else
//#endif // SAMD20
//
//#ifndef __SAMD20E18__
//void TC6_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
//void TC7_Handler( void ) __attribute__( ( weak, alias( "Dummy_Handler" ) ) );
//#endif // __SAMD20E18__

// Initialize segments
extern uint32_t __etext;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __StackTop;

// Exception table
__attribute__( ( section( ".isr_vector" ) ) )
const DeviceVectors exception_table = {
    /* Configure Initial Stack Pointer, using linker-generated symbols */
    (void *)( &__StackTop ),

    (void *)Reset_Handler, 
	(void *)NMI_Handler, 
	(void *)HardFault_Handler,
    (void *)( 0UL ),                      
    (void *)( 0UL ),                      
    (void *)( 0UL ),                      
    (void *)( 0UL ),                      
    (void *)( 0UL ),                      
    (void *)( 0UL ),                      
    (void *)( 0UL ),                      
    (void *)SVC_Handler, 
	(void *)( 0UL ), 
    (void *)( 0UL ),                      
    (void *)PendSV_Handler, 
	(void *)SysTick_Handler,

    /* Configurable interrupts */
    (void *)PM_Handler, // 0 Power Manager
    (void *)SYSCTRL_Handler, // 1 System Control
    (void *)WDT_Handler, // 2 Watch dog timer
    (void *)RTC_Handler, // 3 Real time clock
    (void *)EIC_Handler, // 4 External interrupt controller    
    (void *)NVMCTRL_Handler, // 5 Non-volatile memory controller
    (void *)EVSYS_Handler, // 6 Event system 
    (void *)SERCOM0_Handler, // 7 Serial communications 0
    (void *)SERCOM1_Handler, // 8 Serial communications 1
    (void *)SERCOM2_Handler, // 9 Serial communications 2
    (void *)SERCOM3_Handler, // 10 Serial communications 3
    (void *)( 0UL ), // 11 Reserved
    (void *)( 0UL ), // 12 Reserved
    (void *)TC0_Handler, // 13 Timer counter 0
	(void *)TC1_Handler, // 14 Timer counter 1
	(void *)TC2_Handler, // 15 Timer counter 2 
    (void *)TC3_Handler, // 16 Timer counter 3
    (void *)TC4_Handler, // 17 Timer counter 4
    (void *)TC5_Handler, // 18 Timer counter 5
    (void *)( 0UL ), // 19 Reserved
    (void *)( 0UL ), // 20 Reserved
    (void *)ADC_Handler, // 21 Analog to digital converter
    (void *)AC_Handler,  // 22 Analog comparator
    (void *)DAC_Handler, // 23 Digital to analog converter
    (void *)PTC_Handler, // 24 peripheral touch controller
};

//#ifndef SAMD20
//(void *)DMAC_Handler,    /*  6 Direct Memory Access Controller */
//(void *)USB_Handler,     /*  7 Universal Serial Bus */
//#endif                       // SAMD20
//#ifndef SAMD20
//(void *)TCC0_Handler, /* 15 Timer Counter Control 0 */
//(void *)TCC1_Handler, /* 16 Timer Counter Control 1 */
//(void *)TCC2_Handler, /* 17 Timer Counter Control 2 */
//#else
//#ifndef __SAMD20E18__
//(void *)TC6_Handler, /* 21 Basic Timer Counter 3 */
//(void *)TC7_Handler, /* 22 Basic Timer Counter 4 */
//#else
//#ifndef SAMD20
//(void *)I2S_Handler, /* 27 Inter-IC Sound Interface */
//(void *)( 0UL ),
//#endif                   // SAMD20

extern int  main();
extern void LowPowerSysInit();
uint8_t     gRCause = 0;
uint32_t    SystemCoreClock = 1000000ul;

// This is called on processor reset to initialize the device and call main()
void Reset_Handler( void )
{
    uint32_t *pSrc, *pDest;

    // Initialize the initialized data section
    pSrc = &__etext;
    pDest = &__data_start__;

    if( ( &__data_start__ != &__data_end__ ) && ( pSrc != pDest ) ) {
        for( ; pDest < &__data_end__; pDest++, pSrc++ ) *pDest = *pSrc;
    }

    // Clear the zero section
    if( ( &__data_start__ != &__data_end__ ) && ( pSrc != pDest ) ) {
        for( pDest = &__bss_start__; pDest < &__bss_end__; pDest++ ) *pDest = 0;
    }

    // Get the reset cause flag, made available by externalizing gRCause
    gRCause = PM->RCAUSE.reg;
    LowPowerSysInit();

    main();

    while( 1 )
        ;
}

extern void SysTick_IRQHandler();
void        SysTick_Handler( void )
{
    SysTick_IRQHandler();
}

extern void RTC_IRQHandler();
void        RTC_Handler( void )
{
    RTC_IRQHandler();
}
