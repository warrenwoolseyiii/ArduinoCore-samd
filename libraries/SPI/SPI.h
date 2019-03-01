/*
 * SPI Master library for Arduino Zero.
 * Copyright (c) 2015 Arduino LLC
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <Arduino.h>

// SPI_HAS_TRANSACTION means SPI has
//   - beginTransaction()
//   - endTransaction()
//   - usingInterrupt()
//   - SPISetting(clock, bitOrder, dataMode)
#define SPI_HAS_TRANSACTION 1

// SPI_HAS_NOTUSINGINTERRUPT means that SPI has notUsingInterrupt() method
#define SPI_HAS_NOTUSINGINTERRUPT 1

typedef struct
{
    uint8_t SCK, MISO, MOSI, SS, padTx, padRx;
} SPIPins_t;

// SPI TX / RX characteristics that are run time programmable
typedef enum
{
    spi_polarity_low = SPI_POLAR_LOW,
    spi_polarity_high = SPI_POLAR_HIGH
} SPIPolarity_t;

typedef enum
{
    spi_lead_sample = SPI_LD_SAMPLE,
    spi_lead_change = SPI_LD_CHANGE
} SPIPhase_t;

typedef enum
{
    spi_lsb_first = SPI_LSB_FST,
    spi_msb_first = SPI_MSB_FST
} SPIDataOrder_t;

typedef enum
{
    spi_data_len_8bit = SPI_8BIT,
    spi_data_len_9bit = SPI_9BIT
} SPIDataLength_t;

class SPISettings
{
  public:
    SPISettings()
    {
        // Default format
        set( 4000000, spi_polarity_low, spi_lead_sample, spi_msb_first,
             spi_data_len_8bit );
    }

    SPISettings( uint32_t clockSpeed, SPIPolarity_t polarity, SPIPhase_t phase,
                 SPIDataOrder_t dataOrder, SPIDataLength_t dataLength )
    {
        set( clockSpeed, polarity, phase, dataOrder, dataLength );
    }

    void set( uint32_t clockSpeed, SPIPolarity_t polarity, SPIPhase_t phase,
              SPIDataOrder_t dataOrder, SPIDataLength_t dataLength )
    {
        _clockSpeed = clockSpeed;
        _polarity = polarity;
        _phase = phase;
        _dataOrder = dataOrder;
        _dataLength = dataLength;
    }

    SPISettings &operator=( const SPISettings &arg )
    {
        if( this != &arg ) {
            _clockSpeed = arg._clockSpeed;
            _polarity = arg._polarity;
            _phase = arg._phase;
            _dataOrder = arg._dataOrder;
            _dataLength = arg._dataLength;
        }

        return *this;
    }

    bool operator==( const SPISettings &arg )
    {
        if( this == &arg ) return true;
        return !( ( _clockSpeed != arg._clockSpeed ) ||
                  ( _polarity != arg._polarity ) || ( _phase != arg._phase ) ||
                  ( _dataOrder != arg._dataOrder ) ||
                  ( _dataLength != arg._dataLength ) );
    }

  private:
    uint32_t        _clockSpeed;
    SPIPolarity_t   _polarity;
    SPIPhase_t      _phase;
    SPIDataOrder_t  _dataOrder;
    SPIDataLength_t _dataLength;

    friend class SPIClass;
};

class SPIClass
{
  public:
    SPIClass( SERCOM *sercom, SPIPins_t pins );
    SPIClass( SERCOM *sercom, uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin,
              uint8_t ssPin, uint8_t padRxPin, uint8_t padTxPin );

    uint8_t  transfer( uint8_t data );
    uint16_t transfer16( uint16_t data );
    void     transfer( void *buf, size_t count );

    // Transaction Functions
    void usingInterrupt( int interruptNumber );
    void notUsingInterrupt( int interruptNumber );
    void beginTransaction( SPISettings settings );
    void endTransaction( void );

    // SPI Configuration methods
    void attachInterrupt();
    void detachInterrupt();

    void begin();
    void end();

    void setBitOrder( SPIDataOrder_t order );
    void setDataMode( SPIPolarity_t pol, SPIPhase_t pha );
    void setClockDivider( uint8_t div );

  private:
    void config( SPISettings settings, bool force = false );

    SERCOM *    _p_sercom;
    SPISettings _settings;
    SPIPins_t   _pins;

    uint8_t  _interruptMode;
    char     _interruptSave;
    uint32_t _interruptMask;
};

#if SPI_INTERFACES_COUNT > 0
extern SPIClass SPI;
#endif
#if SPI_INTERFACES_COUNT > 1
extern SPIClass SPI1;
#endif
#if SPI_INTERFACES_COUNT > 2
extern SPIClass SPI2;
#endif
#if SPI_INTERFACES_COUNT > 3
extern SPIClass SPI3;
#endif
#if SPI_INTERFACES_COUNT > 4
extern SPIClass SPI4;
#endif
#if SPI_INTERFACES_COUNT > 5
extern SPIClass SPI5;
#endif

// For compatibility with sketches designed for AVR @ 16 MHz
// New programs should use SPI.beginTransaction to set the SPI clock
#define SPI_CLOCK_DIV2 2
#define SPI_CLOCK_DIV4 4
#define SPI_CLOCK_DIV8 8
#define SPI_CLOCK_DIV16 16
#define SPI_CLOCK_DIV32 32
#define SPI_CLOCK_DIV64 64
#define SPI_CLOCK_DIV128 128

#endif /* _SPI_H_INCLUDED */
