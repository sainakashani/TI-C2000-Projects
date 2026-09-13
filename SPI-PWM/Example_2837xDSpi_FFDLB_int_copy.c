//###########################################################################
// Purpose:
// Read ADC sensor values every 100us, prepare packet in ADC ISR
//###########################################################################

#include "F28x_Project.h"

// ---------------- Configuration ----------------

#define CPU_MHZ              200.0f
#define SAMPLE_PERIOD_US     100.0f

#define H7_CS_GPIO           61U

#define PACKET_HEADER        0xAA55U
#define PACKET_END           0x55AAU

#define PACKET_WORDS         18U
#define NO_READY_BUFFER      0xFFFFU

#define ADC_SCALE_3V         (3.0f / 4096.0f)

// SPI test
#define TEST_OK_GPIO         31U
#define TEST_STRING_WORDS    6U
#define SPI_TEST_TIMEOUT     500000UL

// PWM scope test
// PWM frequency is about 20 kHz if EPWMCLK = 100 MHz.
#define PWM_SCOPE_TBPRD      5000U

// ---------------- State Machine ----------------

typedef enum
{
    STATE_IDLE = 0,
    STATE_ADC_BUSY,
    STATE_PACKET_READY,
    STATE_SPI_SENDING,
    STATE_OVERRUN
} SystemState;

volatile SystemState g_state = STATE_IDLE;

// ---------------- Global Flags ----------------

volatile Uint16 g_adcBusy = 0;
volatile Uint16 g_readyBuffer = NO_READY_BUFFER;
volatile Uint16 g_fillBuffer = 0;

volatile Uint16 g_seq = 0;
volatile Uint32 g_adcOverrunCount = 0;
volatile Uint32 g_packetDropCount = 0;
volatile Uint32 g_spiSendCount = 0;

// Double buffer for SPI packets
volatile Uint16 g_packet[2][PACKET_WORDS];

// SPI-A internal loopback variables
volatile Uint16 g_spiStringTestDone = 0;
volatile Uint16 g_spiStringTestPass = 0;
volatile Uint16 g_spiStringTestFailIndex = 0xFFFFU;
const Uint16 g_testTx[TEST_STRING_WORDS] = { 'H', 'E', 'L', 'L', 'O', 0 };
volatile Uint16 g_testRx[TEST_STRING_WORDS];

// SPI-A to SPI-B test variables
volatile Uint16 g_spiAToBTestDone = 0;
volatile Uint16 g_spiAToBTestPass = 0;
volatile Uint16 g_spiAToBTestFailIndex = 0xFFFFU;
volatile Uint16 g_spiAToBTimeout = 0;

const Uint16 g_spiAToBTx[TEST_STRING_WORDS] = { 'H', 'E', 'L', 'L', 'O', 0 };
volatile Uint16 g_spiAToBRx[TEST_STRING_WORDS];

// ---------------- PWM Scope Variables ----------------
// Duty command range: 0 to 1000
// 0    = 0%
// 500  = 50%
// 1000 = 100%

volatile Uint16 g_pwm1_duty = 100;   // GPIO0  / ePWM1A / 10%
volatile Uint16 g_pwm2_duty = 200;   // GPIO1  / ePWM1B / 20%
volatile Uint16 g_pwm3_duty = 300;   // GPIO2  / ePWM2A / 30%
volatile Uint16 g_pwm4_duty = 400;   // GPIO3  / ePWM2B / 40%
volatile Uint16 g_pwm5_duty = 500;   // GPIO4  / ePWM3A / 50%
volatile Uint16 g_pwm6_duty = 600;   // GPIO5  / ePWM3B / 60%
volatile Uint16 g_pwm7_duty = 700;   // GPIO6  / ePWM4A / 70%

// ---------------- Raw ADC Sensor Data ----------------

volatile Uint16 g_ADCA0_VPV1          = 0;
volatile Uint16 g_ADCA1_Leakage       = 0;
volatile Uint16 g_ADCA2_VPV2          = 0;
volatile Uint16 g_ADCA3_VdcLink       = 0;
volatile Uint16 g_ADCA4_Earth         = 0;
volatile Uint16 g_ADCA5_Riso          = 0;

volatile Uint16 g_ADCB2_VshiftPV      = 0;
volatile Uint16 g_ADCB3_Vgrid         = 0;
volatile Uint16 g_ADCB4_IPV1          = 0;
volatile Uint16 g_ADCB5_VshiftGrid    = 0;

volatile Uint16 g_ADCC2_RCurrentGrid  = 0;
volatile Uint16 g_ADCC3_Vc            = 0;
volatile Uint16 g_ADCC4_IPV2          = 0;
volatile Uint16 g_ADCC5_IGrid         = 0;

volatile float g_vdc_adc_voltage      = 0.0f;

// ---------------- Function Prototypes ----------------

void init_system(void);
void init_timer_100us(void);
Uint16 timer_100us_elapsed(void);

void init_adc_sensors(void);
void start_adc_conversion(void);

void spi_fifo_init(void);
void spi_b_fifo_init(void);

void spi_send_word(Uint16 data);
void spi_send_packet_buffer(Uint16 bufferIndex);

Uint16 spi_transfer_word(Uint16 data);
void spi_set_internal_loopback(Uint16 enable);
void spi_string_internal_loopback_test(void);
void spi_a_to_spib_string_test(void);

void init_pwm_scope_outputs(void);
void pwm_update_scope_outputs(void);
Uint16 pwm_duty_to_cmp(Uint16 dutyPermil);

void InitSpibGpio(void);
void InitSpiBSlave(void);

void error(void);

interrupt void adca1_isr(void);

// ---------------- Main ----------------

void main(void)
{
    Uint16 txBuffer;

    init_system();
    init_adc_sensors();

    // SPI-A = master
    spi_fifo_init();

    // SPI-B = slave
    spi_b_fifo_init();

    // 7 PWM outputs for oscilloscope test
    init_pwm_scope_outputs();

    // Do not run startup tests.
    // Test will run only inside STATE_SPI_SENDING.
    // spi_string_internal_loopback_test();
    // spi_a_to_spib_string_test();

    init_timer_100us();

    for(;;)
    {
        // 100us scheduler
        if(timer_100us_elapsed())
        {
            if(g_adcBusy == 0)
            {
                g_state = STATE_ADC_BUSY;
                start_adc_conversion();
            }
            else
            {
                g_adcOverrunCount++;
                g_state = STATE_OVERRUN;
            }
        }

        // If ADC ISR prepared a packet, enter SPI sending state.
        if(g_readyBuffer != NO_READY_BUFFER)
        {
            DINT;
            txBuffer = g_readyBuffer;
            g_readyBuffer = NO_READY_BUFFER;
            EINT;

            g_state = STATE_SPI_SENDING;

            // Send HELLO from SPI-A and receive it with SPI-B.
            // spi_send_packet_buffer(txBuffer);
            spi_a_to_spib_string_test();

            g_spiSendCount++;

            g_state = STATE_IDLE;
        }

        // Keep PWM compare values updated from Expressions.
        pwm_update_scope_outputs();
    }
}

// ---------------- System Init ----------------

void init_system(void)
{
    InitSysCtrl();

    // SPI-A pins according to your board schematic:
    // GPIO58 = SPIAMOSI / SPISIMOA
    // GPIO59 = SPIAMISO / SPISOMIA
    // GPIO60 = SPIACLK  / SPICLKA
    // GPIO61 = SPIACS   / manual CS
    InitSpiaGpio();

    // SPI-B pins according to your board schematic:
    // GPIO63 = SPIBMOSI / SPISIMOB
    // GPIO64 = SPIBMISO / SPISOMIB
    // GPIO65 = SPIBCLK  / SPICLKB
    // GPIO66 = SPIBCS   / SPISTEB
    InitSpibGpio();

    // Use GPIO61 as manual CS, not SPISTE-A.
    EALLOW;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO61 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO61 = 1;
    EDIS;

    // CS inactive high
    GpioDataRegs.GPBSET.bit.GPIO61 = 1;

    // GPIO31 is test output.
    // GPIO31 = 1 means SPI-A to SPI-B test passed.
    EALLOW;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;
    EDIS;

    GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;

    // Interrupt setup
    DINT;

    InitPieCtrl();

    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();

    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr;
    EDIS;

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    IER |= M_INT1;

    EINT;
    ERTM;
}

// ---------------- 100us Timer ----------------

void init_timer_100us(void)
{
    InitCpuTimers();

    ConfigCpuTimer(&CpuTimer0, CPU_MHZ, SAMPLE_PERIOD_US);

    // Do not use timer interrupt. Poll TIF flag in main loop.
    CpuTimer0Regs.TCR.bit.TIE = 0;

    // Reload and start timer
    CpuTimer0Regs.TCR.bit.TRB = 1;
    CpuTimer0Regs.TCR.bit.TSS = 0;
}

Uint16 timer_100us_elapsed(void)
{
    if(CpuTimer0Regs.TCR.bit.TIF == 1)
    {
        CpuTimer0Regs.TCR.bit.TIF = 1;
        return 1;
    }

    return 0;
}

// ---------------- ADC Init ----------------

void init_adc_sensors(void)
{
    EALLOW;

    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;
    AdccRegs.ADCCTL2.bit.PRESCALE = 6;

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    EDIS;

    DELAY_US(1000);

    EALLOW;

    // ADCA SOC0-SOC5: channels A0-A5
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 2;
    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 3;
    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 4;
    AdcaRegs.ADCSOC5CTL.bit.CHSEL = 5;

    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14;
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 14;
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 14;
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = 14;
    AdcaRegs.ADCSOC4CTL.bit.ACQPS = 14;
    AdcaRegs.ADCSOC5CTL.bit.ACQPS = 14;

    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0;
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 0;
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 0;
    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = 0;
    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = 0;

    // ADCB SOC0-SOC3: channels B2-B5
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2;
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 3;
    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 4;
    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 5;

    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 14;
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 14;
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = 14;
    AdcbRegs.ADCSOC3CTL.bit.ACQPS = 14;

    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 0;
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 0;
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 0;
    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = 0;

    // ADCC SOC0-SOC3: channels C2-C5
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 2;
    AdccRegs.ADCSOC1CTL.bit.CHSEL = 3;
    AdccRegs.ADCSOC2CTL.bit.CHSEL = 4;
    AdccRegs.ADCSOC3CTL.bit.CHSEL = 5;

    AdccRegs.ADCSOC0CTL.bit.ACQPS = 14;
    AdccRegs.ADCSOC1CTL.bit.ACQPS = 14;
    AdccRegs.ADCSOC2CTL.bit.ACQPS = 14;
    AdccRegs.ADCSOC3CTL.bit.ACQPS = 14;

    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 0;
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 0;
    AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 0;
    AdccRegs.ADCSOC3CTL.bit.TRIGSEL = 0;

    // ADCA1 interrupt after ADCA SOC5 conversion.
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 5;
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1;

    // B/C flags are only used for safety check, not CPU interrupt.
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 3;
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdcbRegs.ADCINTOVFCLR.bit.ADCINT1 = 1;

    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 3;
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdccRegs.ADCINTOVFCLR.bit.ADCINT1 = 1;

    EDIS;
}

// ---------------- Start ADC Conversion ----------------

void start_adc_conversion(void)
{
    g_adcBusy = 1;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1;
    AdcbRegs.ADCINTOVFCLR.bit.ADCINT1 = 1;
    AdccRegs.ADCINTOVFCLR.bit.ADCINT1 = 1;

    AdcaRegs.ADCSOCFRC1.all = 0x003F;
    AdcbRegs.ADCSOCFRC1.all = 0x000F;
    AdccRegs.ADCSOCFRC1.all = 0x000F;
}

// ---------------- ADC ISR ----------------

interrupt void adca1_isr(void)
{
    Uint16 b;
    Uint16 checksum;

    while(AdcbRegs.ADCINTFLG.bit.ADCINT1 == 0)
    {
    }

    while(AdccRegs.ADCINTFLG.bit.ADCINT1 == 0)
    {
    }

    g_ADCA0_VPV1    = AdcaResultRegs.ADCRESULT0;
    g_ADCA1_Leakage = AdcaResultRegs.ADCRESULT1;
    g_ADCA2_VPV2    = AdcaResultRegs.ADCRESULT2;
    g_ADCA3_VdcLink = AdcaResultRegs.ADCRESULT3;
    g_ADCA4_Earth   = AdcaResultRegs.ADCRESULT4;
    g_ADCA5_Riso    = AdcaResultRegs.ADCRESULT5;

    g_ADCB2_VshiftPV   = AdcbResultRegs.ADCRESULT0;
    g_ADCB3_Vgrid      = AdcbResultRegs.ADCRESULT1;
    g_ADCB4_IPV1       = AdcbResultRegs.ADCRESULT2;
    g_ADCB5_VshiftGrid = AdcbResultRegs.ADCRESULT3;

    g_ADCC2_RCurrentGrid = AdccResultRegs.ADCRESULT0;
    g_ADCC3_Vc           = AdccResultRegs.ADCRESULT1;
    g_ADCC4_IPV2         = AdccResultRegs.ADCRESULT2;
    g_ADCC5_IGrid        = AdccResultRegs.ADCRESULT3;

    g_vdc_adc_voltage = g_ADCA3_VdcLink * ADC_SCALE_3V;

    if(g_readyBuffer != NO_READY_BUFFER)
    {
        g_packetDropCount++;
    }
    else
    {
        b = g_fillBuffer;

        checksum = PACKET_HEADER;
        checksum ^= g_seq;
        checksum ^= g_ADCA0_VPV1;
        checksum ^= g_ADCA1_Leakage;
        checksum ^= g_ADCA2_VPV2;
        checksum ^= g_ADCA3_VdcLink;
        checksum ^= g_ADCA4_Earth;
        checksum ^= g_ADCA5_Riso;
        checksum ^= g_ADCB2_VshiftPV;
        checksum ^= g_ADCB3_Vgrid;
        checksum ^= g_ADCB4_IPV1;
        checksum ^= g_ADCB5_VshiftGrid;
        checksum ^= g_ADCC2_RCurrentGrid;
        checksum ^= g_ADCC3_Vc;
        checksum ^= g_ADCC4_IPV2;
        checksum ^= g_ADCC5_IGrid;

        g_packet[b][0]  = PACKET_HEADER;
        g_packet[b][1]  = g_seq;

        g_packet[b][2]  = g_ADCA0_VPV1;
        g_packet[b][3]  = g_ADCA1_Leakage;
        g_packet[b][4]  = g_ADCA2_VPV2;
        g_packet[b][5]  = g_ADCA3_VdcLink;
        g_packet[b][6]  = g_ADCA4_Earth;
        g_packet[b][7]  = g_ADCA5_Riso;

        g_packet[b][8]  = g_ADCB2_VshiftPV;
        g_packet[b][9]  = g_ADCB3_Vgrid;
        g_packet[b][10] = g_ADCB4_IPV1;
        g_packet[b][11] = g_ADCB5_VshiftGrid;

        g_packet[b][12] = g_ADCC2_RCurrentGrid;
        g_packet[b][13] = g_ADCC3_Vc;
        g_packet[b][14] = g_ADCC4_IPV2;
        g_packet[b][15] = g_ADCC5_IGrid;

        g_packet[b][16] = checksum;
        g_packet[b][17] = PACKET_END;

        g_readyBuffer = b;
        g_fillBuffer ^= 1U;
        g_seq++;

        g_state = STATE_PACKET_READY;
    }

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1;
    AdcbRegs.ADCINTOVFCLR.bit.ADCINT1 = 1;
    AdccRegs.ADCINTOVFCLR.bit.ADCINT1 = 1;

    g_adcBusy = 0;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// ---------------- SPI Init ----------------

void spi_fifo_init(void)
{
    SpiaRegs.SPIFFTX.all = 0xE040;
    SpiaRegs.SPIFFRX.all = 0x2044;
    SpiaRegs.SPIFFCT.all = 0x0000;

    InitSpi();

    while(SpiaRegs.SPIFFRX.bit.RXFFST != 0)
    {
        (void)SpiaRegs.SPIRXBUF;
    }

    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;
}

void spi_b_fifo_init(void)
{
    SpibRegs.SPIFFTX.all = 0xE040;
    SpibRegs.SPIFFRX.all = 0x2044;
    SpibRegs.SPIFFCT.all = 0x0000;

    InitSpiBSlave();

    while(SpibRegs.SPIFFRX.bit.RXFFST != 0)
    {
        (void)SpibRegs.SPIRXBUF;
    }

    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1;
}

// ---------------- SPI Send / Transfer ----------------

void spi_set_internal_loopback(Uint16 enable)
{
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;

    if(enable)
    {
        SpiaRegs.SPICCR.bit.SPILBK = 1;
    }
    else
    {
        SpiaRegs.SPICCR.bit.SPILBK = 0;
    }

    SpiaRegs.SPICCR.bit.SPISWRESET = 1;
}

Uint16 spi_transfer_word(Uint16 data)
{
    Uint16 rx;

    while(SpiaRegs.SPIFFTX.bit.TXFFST != 0)
    {
    }

    SpiaRegs.SPITXBUF = data;

    while(SpiaRegs.SPIFFRX.bit.RXFFST == 0)
    {
    }

    rx = SpiaRegs.SPIRXBUF;

    return rx;
}

void spi_string_internal_loopback_test(void)
{
    Uint16 i;
    Uint16 pass = 1;

    g_spiStringTestDone = 0;
    g_spiStringTestPass = 0;
    g_spiStringTestFailIndex = 0xFFFFU;

    GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;

    while(SpiaRegs.SPIFFRX.bit.RXFFST != 0)
    {
        (void)SpiaRegs.SPIRXBUF;
    }

    spi_set_internal_loopback(1);

    // CS low: GPIO61
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    for(i = 0; i < TEST_STRING_WORDS; i++)
    {
        g_testRx[i] = spi_transfer_word(g_testTx[i]);

        if(g_testRx[i] != g_testTx[i])
        {
            pass = 0;

            if(g_spiStringTestFailIndex == 0xFFFFU)
            {
                g_spiStringTestFailIndex = i;
            }
        }
    }

    // CS high: GPIO61
    GpioDataRegs.GPBSET.bit.GPIO61 = 1;

    spi_set_internal_loopback(0);

    if(pass == 1)
    {
        g_spiStringTestPass = 1;
        GpioDataRegs.GPASET.bit.GPIO31 = 1;
    }
    else
    {
        g_spiStringTestPass = 0;
        GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;
    }

    g_spiStringTestDone = 1;
}

void spi_a_to_spib_string_test(void)
{
    Uint16 i;
    Uint16 pass = 1;
    Uint32 timeout;

    g_spiAToBTestDone = 0;
    g_spiAToBTestPass = 0;
    g_spiAToBTestFailIndex = 0xFFFFU;
    g_spiAToBTimeout = 0;

    GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;

    while(SpibRegs.SPIFFRX.bit.RXFFST != 0)
    {
        (void)SpibRegs.SPIRXBUF;
    }

    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1;

    // CS low.
    // GPIO61 must be physically connected to GPIO66 / SPISTEB.
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    for(i = 0; i < TEST_STRING_WORDS; i++)
    {
        spi_send_word(g_spiAToBTx[i]);

        timeout = SPI_TEST_TIMEOUT;

        while((SpibRegs.SPIFFRX.bit.RXFFST == 0) && (timeout > 0UL))
        {
            timeout--;
        }

        if(timeout == 0UL)
        {
            pass = 0;
            g_spiAToBTimeout = 1;

            if(g_spiAToBTestFailIndex == 0xFFFFU)
            {
                g_spiAToBTestFailIndex = i;
            }

            break;
        }

        g_spiAToBRx[i] = SpibRegs.SPIRXBUF;

        if(g_spiAToBRx[i] != g_spiAToBTx[i])
        {
            pass = 0;

            if(g_spiAToBTestFailIndex == 0xFFFFU)
            {
                g_spiAToBTestFailIndex = i;
            }
        }
    }

    // CS high.
    GpioDataRegs.GPBSET.bit.GPIO61 = 1;

    if(pass == 1)
    {
        g_spiAToBTestPass = 1;
        GpioDataRegs.GPASET.bit.GPIO31 = 1;
    }
    else
    {
        g_spiAToBTestPass = 0;
        GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;
    }

    g_spiAToBTestDone = 1;
}

void spi_send_word(Uint16 data)
{
    while(SpiaRegs.SPIFFTX.bit.TXFFST != 0)
    {
    }

    SpiaRegs.SPITXBUF = data;

    while(SpiaRegs.SPIFFRX.bit.RXFFST == 0)
    {
    }

    (void)SpiaRegs.SPIRXBUF;
}

void spi_send_packet_buffer(Uint16 bufferIndex)
{
    Uint16 i;

    // CS low: GPIO61
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    for(i = 0; i < PACKET_WORDS; i++)
    {
        spi_send_word(g_packet[bufferIndex][i]);
    }

    // CS high: GPIO61
    GpioDataRegs.GPBSET.bit.GPIO61 = 1;
}

// ---------------- PWM Scope Outputs ----------------

Uint16 pwm_duty_to_cmp(Uint16 dutyPermil)
{
    Uint32 cmp;

    if(dutyPermil > 1000U)
    {
        dutyPermil = 1000U;
    }

    cmp = ((Uint32)PWM_SCOPE_TBPRD * (Uint32)dutyPermil) / 1000UL;

    return (Uint16)cmp;
}

void init_pwm_scope_outputs(void)
{
    EALLOW;

    // ePWM clock:
    // If SYSCLK = 200 MHz and EPWMCLKDIV = 1,
    // EPWMCLK = SYSCLK / 2 = 100 MHz.
    // With TBPRD = 5000 and up-count mode:
    // PWM frequency = 100 MHz / 5000 = 20 kHz.
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1;

    // Stop ePWM time-base clock during configuration.
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;

    // Enable ePWM module clocks.
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM4 = 1;

    // GPIO0-GPIO6 as ePWM pins.
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;

    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO4 = 0;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO5 = 0;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO6 = 0;

    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;    // ePWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;    // ePWM1B
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;    // ePWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;    // ePWM2B
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;    // ePWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;    // ePWM3B
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;    // ePWM4A

    EDIS;

    // ePWM1 setup.
    EPwm1Regs.TBPRD = PWM_SCOPE_TBPRD;
    EPwm1Regs.TBCTR = 0;
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    EPwm1Regs.AQCTLA.all = 0;
    EPwm1Regs.AQCTLB.all = 0;
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;
    EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;

    // ePWM2 setup.
    EPwm2Regs.TBPRD = PWM_SCOPE_TBPRD;
    EPwm2Regs.TBCTR = 0;
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    EPwm2Regs.AQCTLA.all = 0;
    EPwm2Regs.AQCTLB.all = 0;
    EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;
    EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;

    // ePWM3 setup.
    EPwm3Regs.TBPRD = PWM_SCOPE_TBPRD;
    EPwm3Regs.TBCTR = 0;
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    EPwm3Regs.AQCTLA.all = 0;
    EPwm3Regs.AQCTLB.all = 0;
    EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm3Regs.AQCTLB.bit.ZRO = AQ_SET;
    EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;

    // ePWM4 setup.
    // Only ePWM4A is used for PWM7.
    EPwm4Regs.TBPRD = PWM_SCOPE_TBPRD;
    EPwm4Regs.TBCTR = 0;
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

    EPwm4Regs.AQCTLA.all = 0;
    EPwm4Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;

    // Load initial duty values.
    pwm_update_scope_outputs();

    EALLOW;

    // Start ePWM time-base clock.
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

    EDIS;
}

void pwm_update_scope_outputs(void)
{
    EPwm1Regs.CMPA.bit.CMPA = pwm_duty_to_cmp(g_pwm1_duty);
    EPwm1Regs.CMPB.bit.CMPB = pwm_duty_to_cmp(g_pwm2_duty);

    EPwm2Regs.CMPA.bit.CMPA = pwm_duty_to_cmp(g_pwm3_duty);
    EPwm2Regs.CMPB.bit.CMPB = pwm_duty_to_cmp(g_pwm4_duty);

    EPwm3Regs.CMPA.bit.CMPA = pwm_duty_to_cmp(g_pwm5_duty);
    EPwm3Regs.CMPB.bit.CMPB = pwm_duty_to_cmp(g_pwm6_duty);

    EPwm4Regs.CMPA.bit.CMPA = pwm_duty_to_cmp(g_pwm7_duty);
}

// ---------------- Error ----------------

void error(void)
{
    asm(" ESTOP0");

    for(;;)
    {
    }
}
