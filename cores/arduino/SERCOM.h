/*
  Copyright (c) 2014 Arduino.  All right reserved.

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

#ifndef SERCOM_H_
#define SERCOM_H_

#include "sam.h"

#define SERCOM_NVIC_PRIORITY ( ( 1 << __NVIC_PRIO_BITS ) - 1 )

// SPI defines for library
#define SPI_POLAR_LOW 0x0
#define SPI_POLAR_HIGH 0x1

#define SPI_LD_SAMPLE 0x0
#define SPI_LD_CHANGE 0x1

#define SPI_LSB_FST 0x0
#define SPI_MSB_FST 0x1

#define SPI_8BIT 0x0
#define SPI_9BIT 0x1

typedef enum
{
    UART_EXT_CLOCK = 0,
    UART_INT_CLOCK = 0x1u
} SercomUartMode;

typedef enum
{
    SPI_SLAVE_OPERATION = 0x2u,
    SPI_MASTER_OPERATION = 0x3u
} SercomSpiMode;

typedef enum
{
    I2C_SLAVE_OPERATION = 0x4u,
    I2C_MASTER_OPERATION = 0x5u
} SercomI2CMode;

typedef enum
{
    SERCOM_EVEN_PARITY = 0,
    SERCOM_ODD_PARITY,
    SERCOM_NO_PARITY
} SercomParityMode;

typedef enum
{
    SERCOM_STOP_BIT_1 = 0,
    SERCOM_STOP_BITS_2
} SercomNumberStopBit;

typedef enum
{
    MSB_FIRST = 0,
    LSB_FIRST
} SercomDataOrder;

typedef enum
{
    UART_CHAR_SIZE_8_BITS = 0,
    UART_CHAR_SIZE_9_BITS,
    UART_CHAR_SIZE_5_BITS = 0x5u,
    UART_CHAR_SIZE_6_BITS,
    UART_CHAR_SIZE_7_BITS
} SercomUartCharSize;

typedef enum
{
    SERCOM_RX_PAD_0 = 0,
    SERCOM_RX_PAD_1,
    SERCOM_RX_PAD_2,
    SERCOM_RX_PAD_3
} SercomRXPad;

typedef enum
{
    UART_TX_PAD_0 = 0x0ul, // Only for UART
    UART_TX_PAD_2 = 0x1ul, // Only for UART
    UART_TX_RTS_CTS_PAD_0_2_3 =
        0x2ul, // Only for UART with TX on PAD0, RTS on PAD2 and CTS on PAD3
} SercomUartTXPad;

typedef enum
{
    SAMPLE_RATE_x16 = 0x1, // Fractional
    SAMPLE_RATE_x8 = 0x3,  // Fractional
} SercomUartSampleRate;

typedef enum
{
    SPI_TX0_SCK1_SS2 = 0,
    SPI_TX2_SCK3_SS1,
    SPI_TX3_SCK1_SS2,
    SPI_TX0_SCK3_SS1
} SercomSpiTXPad;

typedef enum
{
    WIRE_UNKNOWN_STATE = 0x0ul,
    WIRE_IDLE_STATE,
    WIRE_OWNER_STATE,
    WIRE_BUSY_STATE
} SercomWireBusState;

typedef enum
{
    WIRE_WRITE_FLAG = 0x0ul,
    WIRE_READ_FLAG
} SercomWireReadWriteFlag;

typedef enum
{
    WIRE_MASTER_ACT_NO_ACTION = 0,
    WIRE_MASTER_ACT_REPEAT_START,
    WIRE_MASTER_ACT_READ,
    WIRE_MASTER_ACT_STOP
} SercomMasterCommandWire;

typedef enum
{
    WIRE_MASTER_ACK_ACTION = 0,
    WIRE_MASTER_NACK_ACTION
} SercomMasterAckActionWire;

typedef enum
{
    MODE_WIRE = 0,
    MODE_UART = 1,
    MODE_SPI = 2,
    MODE_NONE = 3
} SercomMode;

typedef struct
{
    uint32_t       baud;
    uint8_t        dOrder, dSize, pol, pha;
    SercomSpiTXPad txPad;
    SercomRXPad    rxPad;
} SercomSPISettings_t;

class SERCOM
{
  public:
    SERCOM( Sercom *s );

    /* ========== UART ========== */
    void initUART( SercomUartMode mode, uint32_t baudrate = 0 );
    void initFrame( SercomUartCharSize charSize, SercomDataOrder dataOrder,
                    SercomParityMode    parityMode,
                    SercomNumberStopBit nbStopBits );
    void initPads( SercomUartTXPad txPad, SercomRXPad rxPad );

    void    resetUART( void );
    void    endUART( void );
    void    enableUART( void );
    void    flushUART( void );
    void    clearStatusUART( void );
    bool    availableDataUART( void );
    bool    isBufferOverflowErrorUART( void );
    bool    isFrameErrorUART( void );
    bool    isParityErrorUART( void );
    bool    isDataRegisterEmptyUART( void );
    uint8_t readDataUART( void );
    int     writeDataUART( uint8_t data );
    bool    isUARTError();
    void    acknowledgeUARTError();
    void    enableDataRegisterEmptyInterruptUART();
    void    disableDataRegisterEmptyInterruptUART();

    /* ========== SPI ========== */
    void    applySPISettings( SercomSPISettings_t settings );
    void    endSPI();
    uint8_t transferDataSPI( uint8_t data );
    void    transferDataSPI( uint8_t *buff, uint16_t len );

    /* ========== WIRE ========== */
    void initSlaveWIRE( uint8_t address, bool enableGeneralCall = false );
    void initMasterWIRE( uint32_t baudrate );

    void resetWIRE( void );
    void enableWIRE( void );
    void disableWIRE( void );
    void endWire( void );
    void prepareNackBitWIRE( void );
    void prepareAckBitWIRE( void );
    void prepareCommandBitsWire( uint8_t cmd );
    bool startTransmissionWIRE( uint8_t address, SercomWireReadWriteFlag flag );
    bool sendDataMasterWIRE( uint8_t data );
    bool sendDataSlaveWIRE( uint8_t data );
    bool isMasterWIRE( void );
    bool isSlaveWIRE( void );
    bool isBusIdleWIRE( void );
    bool isBusOwnerWIRE( void );
    bool isDataReadyWIRE( void );
    bool isStopDetectedWIRE( void );
    bool isRestartDetectedWIRE( void );
    bool isAddressMatch( void );
    bool isMasterReadOperationWIRE( void );
    bool isRXNackReceivedWIRE( void );
    int  availableWIRE( void );
    uint8_t readDataWIRE( void );

  private:
    Sercom *   _sercom;
    SercomMode _mode;
    uint8_t    calculateBaudrateSynchronous( uint32_t baudrate );
    uint32_t   division( uint32_t dividend, uint32_t divisor );
    void       enableSERCOM();
    void       disableSERCOM();
    void       takeDownMode();
};

#endif /* SERCOM_H_ */
