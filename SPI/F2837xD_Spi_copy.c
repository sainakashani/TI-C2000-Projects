//###########################################################################
//
// FILE:   F2837xD_Spi.c
//
// TITLE:  F2837xD SPI Initialization & Support Functions.
//
//###########################################################################

//
// Included Files
//
#include "F2837xD_device.h"
#include "F2837xD_Examples.h"

//
// Calculate BRR: 7-bit baud rate register value
// SPI CLK freq = 5 MHz for CPU_FRQ_200MHZ
// LSPCLK freq  = CPU freq / 4  by default
// BRR          = (LSPCLK freq / SPI CLK freq) - 1
//
#if CPU_FRQ_200MHZ
#define SPI_BRR        ((200E6 / 4) / 5E6) - 1
#endif

#if CPU_FRQ_150MHZ
#define SPI_BRR        ((150E6 / 4) / 500E3) - 1
#endif

#if CPU_FRQ_120MHZ
#define SPI_BRR        ((120E6 / 4) / 500E3) - 1
#endif

//
// InitSpi - Initialize SPI-A as master
//
void InitSpi(void)
{
    //
    // Put SPI-A in reset before configuration.
    //
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;

    //
    // SPI-A configuration:
    // CPOL = 0
    // CPHA = 0
    // 16-bit word
    // external mode, no internal loopback
    //
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;
    SpiaRegs.SPICCR.bit.SPICHAR = (16 - 1);
    SpiaRegs.SPICCR.bit.SPILBK = 0;

    //
    // SPI-A = master
    //
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;
    SpiaRegs.SPICTL.bit.TALK = 1;
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0;
    SpiaRegs.SPICTL.bit.SPIINTENA = 0;

    //
    // SPI-A baud rate
    //
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = SPI_BRR;

    //
    // Continue SPI operation during debug halt.
    //
    SpiaRegs.SPIPRI.bit.FREE = 1;

    //
    // Release SPI-A from reset.
    //
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;
}

//
// InitSpiBSlave - Initialize SPI-B as slave
//
void InitSpiBSlave(void)
{
    //
    // Put SPI-B in reset before configuration.
    //
    SpibRegs.SPICCR.bit.SPISWRESET = 0;

    //
    // SPI-B configuration:
    // Same mode as SPI-A
    // CPOL = 0
    // CPHA = 0
    // 16-bit word
    //
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0;
    SpibRegs.SPICCR.bit.SPICHAR = (16 - 1);
    SpibRegs.SPICCR.bit.SPILBK = 0;

    //
    // SPI-B = slave
    //
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 0;
    SpibRegs.SPICTL.bit.TALK = 1;
    SpibRegs.SPICTL.bit.CLK_PHASE = 0;
    SpibRegs.SPICTL.bit.SPIINTENA = 0;

    //
    // BRR is mainly used in master mode.
    // Set it anyway to keep SPI-B in a known state.
    //
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = SPI_BRR;

    //
    // Continue SPI operation during debug halt.
    //
    SpibRegs.SPIPRI.bit.FREE = 1;

    //
    // Release SPI-B from reset.
    //
    SpibRegs.SPICCR.bit.SPISWRESET = 1;
}

//
// InitSpiGpio - Initialize SPI-A GPIO pins
//
void InitSpiGpio(void)
{
    InitSpiaGpio();
}

//
// GPIO58 = SPISIMOA / SPIAMOSI
// GPIO59 = SPISOMIA / SPIAMISO
// GPIO60 = SPICLKA  / SPIACLK
// GPIO61 = SPISTEA  / SPIACS
//
void InitSpiaGpio(void)
{
    EALLOW;

    //
    // Enable internal pull-up for SPI-A pins.
    //
    GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO61 = 0;

    //
    // Asynchronous qualification for SPI pins.
    //
    GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = 3;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO59 = 3;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO60 = 3;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO61 = 3;

    //
    // Configure GPIO58-GPIO61 as SPI-A.
    // For these pins:
    // GMUX = 3
    // MUX  = 3
    //
    GpioCtrlRegs.GPBGMUX2.bit.GPIO58 = 3;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO59 = 3;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO60 = 3;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO61 = 3;

    GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 3;
    GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 3;
    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 3;
    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 3;

    EDIS;
}

//
// InitSpibGpio - Initialize SPI-B GPIOs for this LaunchPad/header mapping
//
// GPIO63 = SPISIMOB / SPIBMOSI
// GPIO64 = SPISOMIB / SPIBMISO
// GPIO65 = SPICLKB  / SPIBCLK
// GPIO66 = SPISTEB  / SPIBCS
//
void InitSpibGpio(void)
{
    EALLOW;

    //
    // GPIO63 is on Port B.
    //
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3;

    GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 3;
    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 3;

    //
    // GPIO64-GPIO66 are on Port C.
    //
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO66 = 0;

    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3;
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3;
    GpioCtrlRegs.GPCQSEL1.bit.GPIO66 = 3;

    //
    // Configure GPIO64-GPIO66 as SPI-B.
    // For these pins:
    // GMUX = 3
    // MUX  = 3
    //
    GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 3;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO65 = 3;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO66 = 3;

    GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 3;
    GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 3;
    GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 3;

    EDIS;
}

//
// End of file
//
