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

#include "SERCOM.h"
#include "variant.h"
#include "Arduino.h"

#ifndef WIRE_RISE_TIME_NANOSECONDS
// Default rise time in nanoseconds, based on 4.7K ohm pull up resistors
// you can override this value in your variant if needed
#define WIRE_RISE_TIME_NANOSECONDS 125
#endif

// SPI Macros
#define SPI_RESET                                                             \
    _sercom->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_SWRST;                          \
    while( _sercom->SPI.CTRLA.bit.SWRST || _sercom->SPI.STATUS.bit.SYNCBUSY ) \
        ;

#define SPI_WAIT_SYNC                         \
    while( _sercom->SPI.STATUS.bit.SYNCBUSY ) \
        ;

SERCOM::SERCOM( Sercom *s )
{
    _sercom = s;
    _mode = MODE_NONE;
}

/* 	=========================
 *	===== Sercom UART
 *	=========================
 */
void SERCOM::initUART( SercomUartMode mode, uint32_t baudrate )
{
    if( _mode < MODE_NONE ) takeDownMode();
    _mode = MODE_UART;
    enableSERCOM();
    resetUART();

    // Setting the CTRLA register
    _sercom->USART.CTRLA.reg = SERCOM_USART_CTRLA_MODE( mode );

    // Enable the receive data interrupt
    _sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;

    if( mode == UART_INT_CLOCK ) {
        uint64_t ratio = 1048576;
        ratio *= baudrate;
        ratio /= SystemCoreClock;
        _sercom->USART.BAUD.reg = ( uint16_t )( 65536 - ratio );
    }
}

void SERCOM::initFrame( SercomUartCharSize charSize, SercomDataOrder dataOrder,
                        SercomParityMode    parityMode,
                        SercomNumberStopBit nbStopBits )
{
    // Setting the CTRLA register
    _sercom->USART.CTRLA.reg |=
        SERCOM_USART_CTRLA_FORM( ( parityMode == SERCOM_NO_PARITY ? 0 : 1 ) ) |
        dataOrder << SERCOM_USART_CTRLA_DORD_Pos;

    // Setting the CTRLB register
    _sercom->USART.CTRLB.reg |=
        SERCOM_USART_CTRLB_CHSIZE( charSize ) |
        nbStopBits << SERCOM_USART_CTRLB_SBMODE_Pos |
        ( parityMode == SERCOM_NO_PARITY ? 0 : parityMode )
            << SERCOM_USART_CTRLB_PMODE_Pos; // If no parity use default value
}

void SERCOM::initPads( SercomUartTXPad txPad, SercomRXPad rxPad )
{
    _sercom->USART.CTRLA.reg |=
        SERCOM_USART_CTRLA_TXPO | SERCOM_USART_CTRLA_RXPO( rxPad );

    // Enable Transceiver
    _sercom->USART.CTRLB.reg |=
        SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN;
}

void SERCOM::resetUART()
{
    _sercom->USART.CTRLA.bit.SWRST = 1;
    while( _sercom->USART.CTRLA.bit.SWRST ||
           _sercom->USART.STATUS.bit.SYNCBUSY )
        ;
}

void SERCOM::endUART()
{
    disableSERCOM();
}

void SERCOM::enableUART()
{
    _sercom->USART.CTRLA.bit.ENABLE = 0x1u;
    while( _sercom->USART.STATUS.bit.SYNCBUSY )
        ;
}

void SERCOM::flushUART()
{
    // Skip checking transmission completion if data register is empty
    if( isDataRegisterEmptyUART() ) return;

    // Wait for transmission to complete
    while( !_sercom->USART.INTFLAG.bit.TXC )
        ;
}

void SERCOM::clearStatusUART()
{
    // Reset (with 0) the STATUS register
    _sercom->USART.STATUS.reg = SERCOM_USART_STATUS_RESETVALUE;
}

bool SERCOM::availableDataUART()
{
    // RXC : Receive Complete
    return _sercom->USART.INTFLAG.bit.RXC;
}

bool SERCOM::isUARTError()
{
    bool rtn = _sercom->USART.STATUS.reg &
               ( SERCOM_USART_STATUS_BUFOVF | SERCOM_USART_STATUS_FERR |
                 SERCOM_USART_STATUS_PERR );
    return rtn;
}

void SERCOM::acknowledgeUARTError()
{
    if( isBufferOverflowErrorUART() )
        _sercom->USART.STATUS.reg |= SERCOM_USART_STATUS_BUFOVF;
    if( isFrameErrorUART() )
        _sercom->USART.STATUS.reg |= SERCOM_USART_STATUS_FERR;
    if( isParityErrorUART() )
        _sercom->USART.STATUS.reg |= SERCOM_USART_STATUS_PERR;
}

bool SERCOM::isBufferOverflowErrorUART()
{
    // BUFOVF : Buffer Overflow
    return _sercom->USART.STATUS.bit.BUFOVF;
}

bool SERCOM::isFrameErrorUART()
{
    // FERR : Frame Error
    return _sercom->USART.STATUS.bit.FERR;
}

bool SERCOM::isParityErrorUART()
{
    // PERR : Parity Error
    return _sercom->USART.STATUS.bit.PERR;
}

bool SERCOM::isDataRegisterEmptyUART()
{
    // DRE : Data Register Empty
    return _sercom->USART.INTFLAG.bit.DRE;
}

uint8_t SERCOM::readDataUART()
{
    return _sercom->USART.DATA.bit.DATA;
}

int SERCOM::writeDataUART( uint8_t data )
{
    // Wait for data register to be empty
    while( !isDataRegisterEmptyUART() )
        ;

    // Put data into DATA register
    _sercom->USART.DATA.reg = (uint16_t)data;
    return 1;
}

void SERCOM::enableDataRegisterEmptyInterruptUART()
{
    _sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
}

void SERCOM::disableDataRegisterEmptyInterruptUART()
{
    _sercom->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
}

/*	=========================
 *	===== Sercom SPI
 *	=========================
 */
void SERCOM::applySPISettings( SercomSPISettings_t settings )
{
    // Enable the peripheral and reset to default values
    enableSERCOM();
    SPI_RESET;

    // Determine and set the CTRLA register values, for now we are just
    // implementing master mode
    uint32_t ctrlA = SERCOM_SPI_CTRLA_MODE_SPI_MASTER;

    // Data order, polarity, and phase
    ctrlA |= ( settings.dOrder == SPI_MSB_FST ? SERCOM_SPI_CTRLA_DORD : 0 );
    ctrlA |= ( settings.pol == SPI_POLAR_HIGH ? SERCOM_SPI_CTRLA_CPOL : 0 );
    ctrlA |= ( settings.pha == SPI_LD_CHANGE ? SERCOM_SPI_CTRLA_CPHA : 0 );

    // Data input & output pins
    ctrlA |= ( SERCOM_SPI_CTRLA_DOPO( settings.txPad ) |
               SERCOM_SPI_CTRLA_DIPO( settings.rxPad ) );

    // Enable the peripheral
    ctrlA |= SERCOM_SPI_CTRLA_ENABLE;

    // Determine and set the CTRLB register values, for now we are not using
    // address mode
    uint32_t ctrlB = SERCOM_SPI_CTRLB_RXEN;

    // Data size
    ctrlB |=
        ( settings.dSize == SPI_9BIT ? SERCOM_SPI_CTRLB_CHSIZE( 0x1 ) : 0 );

    // Set the baud rate
    _sercom->SPI.BAUD.reg = calculateBaudrateSynchronous( settings.baud );

    // Apply control registers A and B
    _sercom->SPI.CTRLB.reg = ctrlB;
    _sercom->SPI.CTRLA.reg = ctrlA;
    SPI_WAIT_SYNC;
}

void SERCOM::endSPI()
{
    SPI_RESET;
    disableSERCOM();
}

uint8_t SERCOM::transferDataSPI( uint8_t data )
{
    // Write data and wait for return data
    _sercom->SPI.DATA.bit.DATA = data;
    while( _sercom->SPI.INTFLAG.bit.RXC == 0 )
        ;

    return _sercom->SPI.DATA.bit.DATA;
}

void SERCOM::transferDataSPI( uint8_t *buff, uint16_t len )
{
    for( uint16_t i = 0; i < len; i++ ) {
        // Transmit
        while( !( _sercom->SPI.INTFLAG.bit.DRE ) )
            ;
        _sercom->SPI.DATA.bit.DATA = buff[i];

        // Receive
        while( _sercom->SPI.INTFLAG.bit.RXC == 0 )
            ;
        buff[i] = _sercom->SPI.DATA.bit.DATA;
    }
}

uint8_t SERCOM::calculateBaudrateSynchronous( uint32_t baudrate )
{
    return SystemCoreClock / ( 2 * baudrate ) - 1;
}

/*	=========================
 *	===== Sercom WIRE
 *	=========================
 */
void SERCOM::resetWIRE()
{
    // I2CM OR I2CS, no matter SWRST is the same bit.
    // Setting the Software bit to 1
    _sercom->I2CM.CTRLA.bit.SWRST = 1;
    while( _sercom->I2CM.CTRLA.bit.SWRST || _sercom->I2CM.STATUS.bit.SYNCBUSY )
        ;
}

void SERCOM::enableWIRE()
{
    // I2C Master and Slave modes share the ENABLE bit function.
    // Enable the I2C master mode
    _sercom->I2CM.CTRLA.bit.ENABLE = 1;
    while( _sercom->I2CM.STATUS.bit.SYNCBUSY )
        ;

    // Setting bus idle mode
    _sercom->I2CM.STATUS.bit.BUSSTATE = 1;
    while( _sercom->I2CM.STATUS.bit.SYNCBUSY )
        ;
}

void SERCOM::disableWIRE()
{
    // I2C Master and Slave modes share the ENABLE bit function.
    // Enable the I2C master mode
    _sercom->I2CM.CTRLA.bit.ENABLE = 0;
    while( _sercom->I2CM.STATUS.bit.SYNCBUSY )
        ;
}

void SERCOM::endWire()
{
    resetWIRE();
    disableSERCOM();
}

void SERCOM::initSlaveWIRE( uint8_t ucAddress, bool enableGeneralCall )
{
    if( _mode < MODE_NONE ) takeDownMode();
    _mode = MODE_WIRE;

    enableSERCOM();
    resetWIRE();

    // Set slave mode
    _sercom->I2CS.CTRLA.bit.MODE = I2C_SLAVE_OPERATION;

    _sercom->I2CS.ADDR.reg =
        SERCOM_I2CS_ADDR_ADDR( ucAddress &
                               0x7Ful ) |    // 0x7F, select only 7 bits
        SERCOM_I2CS_ADDR_ADDRMASK( 0x00ul ); // 0x00, only match exact address
    if( enableGeneralCall ) {
        _sercom->I2CS.ADDR.reg |=
            SERCOM_I2CS_ADDR_GENCEN; // enable general call (address 0x00)
    }

    // Set the interrupt register
    _sercom->I2CS.INTENSET.reg = SERCOM_I2CS_INTENSET_PREC |   // Stop
                                 SERCOM_I2CS_INTENSET_AMATCH | // Address Match
                                 SERCOM_I2CS_INTENSET_DRDY;    // Data Ready

    while( _sercom->I2CM.STATUS.bit.SYNCBUSY )
        ;
}

void SERCOM::initMasterWIRE( uint32_t baudrate )
{
    if( _mode < MODE_NONE ) takeDownMode();
    _mode = MODE_WIRE;

    enableSERCOM();
    resetWIRE();

    // Set master mode and enable SCL Clock Stretch mode (stretch after ACK bit)
    _sercom->I2CM.CTRLA.reg =
        SERCOM_I2CM_CTRLA_MODE( I2C_MASTER_OPERATION ) /* |
                            SERCOM_I2CM_CTRLA_SCLSM*/
        ;

    // Enable Smart mode and Quick Command
    // sercom->I2CM.CTRLB.reg =  SERCOM_I2CM_CTRLB_SMEN /*|
    // SERCOM_I2CM_CTRLB_QCEN*/ ;

    // Enable all interrupts
    //  sercom->I2CM.INTENSET.reg = SERCOM_I2CM_INTENSET_MB |
    //  SERCOM_I2CM_INTENSET_SB | SERCOM_I2CM_INTENSET_ERROR ;

    // Synchronous arithmetic baudrate
    _sercom->I2CM.BAUD.bit.BAUD =
        SystemCoreClock / ( 2 * baudrate ) - 5 -
        ( ( ( SystemCoreClock / 1000000 ) * WIRE_RISE_TIME_NANOSECONDS ) /
          ( 2 * 1000 ) );
}

void SERCOM::prepareNackBitWIRE( void )
{
    if( isMasterWIRE() ) {
        // Send a NACK
        _sercom->I2CM.CTRLB.bit.ACKACT = 1;
    }
    else {
        _sercom->I2CS.CTRLB.bit.ACKACT = 1;
    }
}

void SERCOM::prepareAckBitWIRE( void )
{
    if( isMasterWIRE() ) {
        // Send an ACK
        _sercom->I2CM.CTRLB.bit.ACKACT = 0;
    }
    else {
        _sercom->I2CS.CTRLB.bit.ACKACT = 0;
    }
}

void SERCOM::prepareCommandBitsWire( uint8_t cmd )
{
    if( isMasterWIRE() ) {
        _sercom->I2CM.CTRLB.bit.CMD = cmd;
        while( _sercom->I2CM.STATUS.bit.SYNCBUSY )
            ;
    }
    else {
        _sercom->I2CS.CTRLB.bit.CMD = cmd;
    }
}

bool SERCOM::startTransmissionWIRE( uint8_t                 address,
                                    SercomWireReadWriteFlag flag )
{
    // 7-bits address + 1-bits R/W
    address = ( address << 0x1ul ) | flag;

    // Wait idle or owner bus mode
    while( !isBusIdleWIRE() && !isBusOwnerWIRE() )
        ;

    // Send start and address
    _sercom->I2CM.ADDR.bit.ADDR = address;

    // Address Transmitted
    if( flag == WIRE_WRITE_FLAG ) // Write mode
    {
        while( !_sercom->I2CM.INTFLAG.bit.MB ) {
            // Wait transmission complete
        }
    }
    else // Read mode
    {
        while( !_sercom->I2CM.INTFLAG.bit.SB ) {
            // If the slave NACKS the address, the MB bit will be set.
            // In that case, send a stop condition and return false.
            if( _sercom->I2CM.INTFLAG.bit.MB ) {
                _sercom->I2CM.CTRLB.bit.CMD = 3; // Stop condition
                return false;
            }
            // Wait transmission complete
        }

        // Clean the 'Slave on Bus' flag, for further usage.
        // sercom->I2CM.INTFLAG.bit.SB = 0x1ul;
    }

    // ACK received (0: ACK, 1: NACK)
    if( _sercom->I2CM.STATUS.bit.RXNACK ) {
        return false;
    }
    else {
        return true;
    }
}

bool SERCOM::sendDataMasterWIRE( uint8_t data )
{
    // Send data
    _sercom->I2CM.DATA.bit.DATA = data;

    // Wait transmission successful
    while( !_sercom->I2CM.INTFLAG.bit.MB ) {

        // If a bus error occurs, the MB bit may never be set.
        // Check the bus error bit and bail if it's set.
        if( _sercom->I2CM.STATUS.bit.BUSERR ) {
            return false;
        }
    }

    // Problems on line? nack received?
    if( _sercom->I2CM.STATUS.bit.RXNACK )
        return false;
    else
        return true;
}

bool SERCOM::sendDataSlaveWIRE( uint8_t data )
{
    // Send data
    _sercom->I2CS.DATA.bit.DATA = data;

    // Problems on line? nack received?
    if( !_sercom->I2CS.INTFLAG.bit.DRDY || _sercom->I2CS.STATUS.bit.RXNACK )
        return false;
    else
        return true;
}

bool SERCOM::isMasterWIRE( void )
{
    return _sercom->I2CS.CTRLA.bit.MODE == I2C_MASTER_OPERATION;
}

bool SERCOM::isSlaveWIRE( void )
{
    return _sercom->I2CS.CTRLA.bit.MODE == I2C_SLAVE_OPERATION;
}

bool SERCOM::isBusIdleWIRE( void )
{
    return _sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_IDLE_STATE;
}

bool SERCOM::isBusOwnerWIRE( void )
{
    return _sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_OWNER_STATE;
}

bool SERCOM::isDataReadyWIRE( void )
{
    return _sercom->I2CS.INTFLAG.bit.DRDY;
}

bool SERCOM::isStopDetectedWIRE( void )
{
    return _sercom->I2CS.INTFLAG.bit.PREC;
}

bool SERCOM::isRestartDetectedWIRE( void )
{
    return _sercom->I2CS.STATUS.bit.SR;
}

bool SERCOM::isAddressMatch( void )
{
    return _sercom->I2CS.INTFLAG.bit.AMATCH;
}

bool SERCOM::isMasterReadOperationWIRE( void )
{
    return _sercom->I2CS.STATUS.bit.DIR;
}

bool SERCOM::isRXNackReceivedWIRE( void )
{
    return _sercom->I2CM.STATUS.bit.RXNACK;
}

int SERCOM::availableWIRE( void )
{
    if( isMasterWIRE() )
        return _sercom->I2CM.INTFLAG.bit.SB;
    else
        return _sercom->I2CS.INTFLAG.bit.DRDY;
}

uint8_t SERCOM::readDataWIRE( void )
{
    if( isMasterWIRE() ) {
        while( _sercom->I2CM.INTFLAG.bit.SB == 0 ) {
            // Waiting complete receive
        }

        return _sercom->I2CM.DATA.bit.DATA;
    }
    else {
        return _sercom->I2CS.DATA.reg;
    }
}

void SERCOM::enableSERCOM()
{
    uint32_t id = GCLK_CLKCTRL_ID_SERCOM0_CORE_Val;
    uint32_t apbMask = PM_APBCMASK_SERCOM0;
    uint32_t irqn = SERCOM0_IRQn;
    if( _sercom == SERCOM1 ) {
        id = GCLK_CLKCTRL_ID_SERCOM1_CORE_Val;
        apbMask = PM_APBCMASK_SERCOM1;
        irqn = SERCOM1_IRQn;
    }
    if( _sercom == SERCOM2 ) {
        id = GCLK_CLKCTRL_ID_SERCOM2_CORE_Val;
        apbMask = PM_APBCMASK_SERCOM2;
        irqn = SERCOM2_IRQn;
    }
    if( _sercom == SERCOM3 ) {
        id = GCLK_CLKCTRL_ID_SERCOM3_CORE_Val;
        apbMask = PM_APBCMASK_SERCOM3;
        irqn = SERCOM3_IRQn;
    }
#if defined( SERCOM4 )
    if( _sercom == SERCOM4 ) {
        id = GCLK_CLKCTRL_ID_SERCOM4_CORE_Val;
        apbMask = PM_APBCMASK_SERCOM4;
        irqn = SERCOM4_IRQn;
    }
#endif /* SERCOM4 */
#if defined( SERCOM5 )
    if( _sercom == SERCOM5 ) {
        id = GCLK_CLKCTRL_ID_SERCOM5_CORE_Val;
        apbMask = PM_APBCMASK_SERCOM5;
        irqn = SERCOM5_IRQn;
    }
#endif /* SERCOM5 */

    // Ensure that PORT is enabled
    enableAPBBClk( PM_APBBMASK_PORT, 1 );

    initGenericClk( GCLK_CLKCTRL_GEN_GCLK0_Val, id );
    enableAPBCClk( apbMask, 1 );
    NVIC_EnableIRQ( (IRQn_Type)irqn );
}

void SERCOM::disableSERCOM()
{
    uint32_t id = GCLK_CLKCTRL_ID_SERCOM0_CORE_Val;
    uint32_t apbMask = PM_APBCMASK_SERCOM0;
    uint32_t irqn = SERCOM0_IRQn;
    if( _sercom == SERCOM1 ) {
        id = GCLK_CLKCTRL_ID_SERCOM1_CORE_Val;
        apbMask = PM_APBCMASK_SERCOM1;
        irqn = SERCOM1_IRQn;
    }
    if( _sercom == SERCOM2 ) {
        id = GCLK_CLKCTRL_ID_SERCOM2_CORE_Val;
        apbMask = PM_APBCMASK_SERCOM2;
        irqn = SERCOM2_IRQn;
    }
    if( _sercom == SERCOM3 ) {
        id = GCLK_CLKCTRL_ID_SERCOM3_CORE_Val;
        apbMask = PM_APBCMASK_SERCOM3;
        irqn = SERCOM3_IRQn;
    }
#if defined( SERCOM4 )
    if( _sercom == SERCOM4 ) {
        id = GCLK_CLKCTRL_ID_SERCOM4_CORE_Val;
        apbMask = PM_APBCMASK_SERCOM4;
        irqn = SERCOM4_IRQn;
    }
#endif /* SERCOM4 */
#if defined( SERCOM5 )
    if( _sercom == SERCOM5 ) {
        id = GCLK_CLKCTRL_ID_SERCOM5_CORE_Val;
        apbMask = PM_APBCMASK_SERCOM5;
        irqn = SERCOM5_IRQn;
    }
#endif /* SERCOM5 */

    NVIC_DisableIRQ( (IRQn_Type)irqn );
    enableAPBCClk( apbMask, 0 );
    disableGenericClk( id );
}

void SERCOM::takeDownMode()
{
    switch( _mode ) {
        case MODE_WIRE: endWire(); break;
        case MODE_UART: endUART(); break;
        case MODE_SPI: endSPI(); break;
        case MODE_NONE:
        default: break; _mode = MODE_NONE;
    }
}
