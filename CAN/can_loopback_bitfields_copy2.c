//###########################################################################
// Purpose:
// Read ADC sensor values every 100us, prepare packet in ADC ISR,
// send ADC packet through CAN-A using 5 CAN frames,
// and optionally verify CAN transmission using CAN external loopback.
//###########################################################################

#include "F28x_Project.h"

//Configuration

#define CPU_MHZ              200.0f
#define SAMPLE_PERIOD_US     100.0f

#define PACKET_HEADER        0xAA55U
#define PACKET_END           0x55AAU
#define PACKET_WORDS         18U
#define NO_READY_BUFFER      0xFFFFU

#define ADC_SCALE_3V         (3.0f / 4096.0f)

// CAN configuration
#define CAN_SOURCE_CLOCK_HZ       200000000UL
#define CAN_BIT_RATE              500000UL

#define CAN_BASE_MSG_ID           0x120U
#define CAN_FRAME_COUNT           5U
#define CAN_BYTES_PER_FRAME       8U

#define CAN_TX_MSG_OBJ_BASE       1U
#define CAN_RX_MSG_OBJ_BASE       11U

#define CAN_RX_TIMEOUT            500000UL

// 1 = test with CAN external loopback
// 0 = real CAN bus transmit
#define CAN_ENABLE_LOOPBACK_CHECK 1U

#define CAN_MAX_BIT_DIVISOR       13U
#define CAN_MIN_BIT_DIVISOR       5U
#define CAN_MAX_PRE_DIVISOR       1024U
#define CAN_MIN_PRE_DIVISOR       1U
#define CAN_BTR_BRP_M             0x3FU
#define CAN_BTR_BRPE_M            0xF0000U
#define CAN_MSG_ID_SHIFT          18U

// State Machine

typedef enum
{
    STATE_IDLE = 0,
    STATE_ADC_BUSY,
    STATE_PACKET_READY,
    STATE_CAN_SENDING,
    STATE_OVERRUN
} SystemState;

volatile SystemState g_state = STATE_IDLE;

//Global Flags

volatile Uint16 g_adcBusy = 0;
volatile Uint16 g_readyBuffer = NO_READY_BUFFER;
volatile Uint16 g_fillBuffer = 0;

volatile Uint16 g_seq = 0;
volatile Uint32 g_adcOverrunCount = 0;
volatile Uint32 g_packetDropCount = 0;

// Double buffer for ADC/CAN packets
volatile Uint16 g_packet[2][PACKET_WORDS];

//CAN Status Variables

volatile Uint16 g_canInitOk = 0;
volatile Uint32 g_canActualBitRate = 0;

volatile Uint32 g_canPacketSendCount = 0;
volatile Uint32 g_canFrameSendCount = 0;
volatile Uint32 g_canLoopbackPassCount = 0;
volatile Uint32 g_canLoopbackFailCount = 0;
volatile Uint32 g_canTimeoutCount = 0;

volatile Uint16 g_canLastFailFrame = 0xFFFFU;
volatile Uint16 g_canLastFailByte = 0xFFFFU;
volatile Uint16 g_canLastSendPass = 0;

volatile Uint16 g_canTxFrame[CAN_FRAME_COUNT][CAN_BYTES_PER_FRAME];
volatile Uint16 g_canRxFrame[CAN_FRAME_COUNT][CAN_BYTES_PER_FRAME];

const Uint16 g_canFrameDlc[CAN_FRAME_COUNT] = {8U, 8U, 8U, 8U, 4U};

//Raw ADC Sensor Data

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

//CAN Types

typedef enum
{
    MSG_OBJ_TYPE_TRANSMIT = 0,
    MSG_OBJ_TYPE_RECEIVE  = 1
} msgObjType;

static const Uint16 canBitValues[] =
{
    0x1100U,
    0x1200U,
    0x2240U,
    0x2340U,
    0x3340U,
    0x3440U,
    0x3540U,
    0x3640U,
    0x3740U
};

//Function Prototypes

void init_system(void);
void init_timer_100us(void);
Uint16 timer_100us_elapsed(void);

void init_adc_sensors(void);
void start_adc_conversion(void);

void init_can_gpio(void);
void init_can_sender(void);
Uint32 setCANBitRate(Uint32 sourceClock, Uint32 bitRate);

void can_setup_message_object(Uint32 objID, Uint32 msgID, msgObjType msgType, Uint16 dlc);
void can_send_frame(Uint32 objID, volatile Uint16 *data, Uint16 dlc);
Uint16 can_get_frame(Uint32 objID, volatile Uint16 *data, Uint16 dlc);
Uint16 can_wait_receive_frame(Uint16 frameIndex);
Uint16 can_compare_frame(Uint16 frameIndex);
void can_pack_packet_to_frames(Uint16 bufferIndex);
void can_send_packet_buffer(Uint16 bufferIndex);

void InitCAN(void);
void error(void);

interrupt void adca1_isr(void);

//Main

void main(void)
{
    Uint16 txBuffer;

    init_system();
    init_adc_sensors();
    init_can_sender();
    init_timer_100us();

    for(;;)
    {
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

        if(g_readyBuffer != NO_READY_BUFFER)
        {
            DINT;
            txBuffer = g_readyBuffer;
            g_readyBuffer = NO_READY_BUFFER;
            EINT;

            g_state = STATE_CAN_SENDING;

            can_send_packet_buffer(txBuffer);

            g_state = STATE_IDLE;
        }
    }
}

//System Init

void init_system(void)
{
    InitSysCtrl();

    init_can_gpio();

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

// 100us Timer

void init_timer_100us(void)
{
    InitCpuTimers();

    ConfigCpuTimer(&CpuTimer0, CPU_MHZ, SAMPLE_PERIOD_US);

    CpuTimer0Regs.TCR.bit.TIE = 0;
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

//ADC Init

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

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 5;
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1;

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

//Start ADC Conversion

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

//ADC ISR

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

//CAN Init

void init_can_gpio(void)
{
    GPIO_SetupPinMux(30, GPIO_MUX_CPU1, 1);      // GPIO30 = CANRXA
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 1);      // GPIO31 = CANTXA

    GPIO_SetupPinOptions(30, GPIO_INPUT, GPIO_ASYNC);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
}

void init_can_sender(void)
{
    Uint16 i;
    Uint32 status;

    InitCAN();

    ClkCfgRegs.CLKSRCCTL2.bit.CANABCLKSEL = 0;

    status = setCANBitRate(CAN_SOURCE_CLOCK_HZ, CAN_BIT_RATE);

    if(status == 0)
    {
        g_canInitOk = 0;
        error();
    }

    g_canActualBitRate = status;
    g_canInitOk = 1;

    CanaRegs.CAN_CTL.bit.Init = 0;

#if CAN_ENABLE_LOOPBACK_CHECK
    CanaRegs.CAN_CTL.bit.Test = 1;
    CanaRegs.CAN_TEST.bit.EXL = 1;
#else
    CanaRegs.CAN_CTL.bit.Test = 0;
    CanaRegs.CAN_TEST.bit.EXL = 0;
#endif

    for(i = 0; i < CAN_FRAME_COUNT; i++)
    {
        can_setup_message_object(CAN_TX_MSG_OBJ_BASE + i,
                                 CAN_BASE_MSG_ID + i,
                                 MSG_OBJ_TYPE_TRANSMIT,
                                 g_canFrameDlc[i]);

#if CAN_ENABLE_LOOPBACK_CHECK
        can_setup_message_object(CAN_RX_MSG_OBJ_BASE + i,
                                 CAN_BASE_MSG_ID + i,
                                 MSG_OBJ_TYPE_RECEIVE,
                                 g_canFrameDlc[i]);
#endif
    }
}

//CAN Bit Rate

Uint32 setCANBitRate(Uint32 sourceClock, Uint32 bitRate)
{
    Uint32 desiredRatio;
    Uint32 canBits;
    Uint32 preDivide;
    Uint32 regValue;
    Uint16 canControlValue;

    desiredRatio = sourceClock / bitRate;

    if((sourceClock / desiredRatio) > bitRate)
    {
        desiredRatio += 1;
    }

    while(desiredRatio <= CAN_MAX_PRE_DIVISOR * CAN_MAX_BIT_DIVISOR)
    {
        for(canBits = CAN_MAX_BIT_DIVISOR;
            canBits >= CAN_MIN_BIT_DIVISOR;
            canBits--)
        {
            preDivide = desiredRatio / canBits;

            if((preDivide * canBits) == desiredRatio)
            {
                regValue = canBitValues[canBits - CAN_MIN_BIT_DIVISOR];

                canControlValue = CanaRegs.CAN_CTL.all;
                CanaRegs.CAN_CTL.bit.Init = 1;
                CanaRegs.CAN_CTL.bit.CCE = 1;

                regValue |= ((preDivide - 1U) & CAN_BTR_BRP_M) |
                            (((preDivide - 1U) << 10U) & CAN_BTR_BRPE_M);

                CanaRegs.CAN_BTR.all = regValue;

                CanaRegs.CAN_CTL.all = canControlValue;

                return(sourceClock / (preDivide * canBits));
            }
        }

        desiredRatio++;
    }

    return 0;
}

// ---------------- CAN Message Objects ----------------

void can_setup_message_object(Uint32 objID, Uint32 msgID, msgObjType msgType, Uint16 dlc)
{
    union CAN_IF1CMD_REG CAN_IF1CMD_SHADOW;

    while(CanaRegs.CAN_IF1CMD.bit.Busy)
    {
    }

    CAN_IF1CMD_SHADOW.all = 0;

    CanaRegs.CAN_IF1MSK.all = 0;
    CanaRegs.CAN_IF1ARB.all = 0;
    CanaRegs.CAN_IF1MCTL.all = 0;

    CAN_IF1CMD_SHADOW.bit.Control = 1;
    CAN_IF1CMD_SHADOW.bit.Arb = 1;
    CAN_IF1CMD_SHADOW.bit.Mask = 1;
    CAN_IF1CMD_SHADOW.bit.DIR = 1;

    if(msgType == MSG_OBJ_TYPE_TRANSMIT)
    {
        CanaRegs.CAN_IF1ARB.bit.Dir = 1;
    }
    else
    {
        CanaRegs.CAN_IF1ARB.bit.Dir = 0;
    }

    CanaRegs.CAN_IF1ARB.bit.ID = (msgID << CAN_MSG_ID_SHIFT);
    CanaRegs.CAN_IF1ARB.bit.MsgVal = 1;

    CanaRegs.CAN_IF1MCTL.bit.DLC = dlc;
    CanaRegs.CAN_IF1MCTL.bit.EoB = 1;

    CAN_IF1CMD_SHADOW.bit.MSG_NUM = objID;
    CanaRegs.CAN_IF1CMD.all = CAN_IF1CMD_SHADOW.all;
}

// ---------------- CAN Frame Send / Receive ----------------

void can_send_frame(Uint32 objID, volatile Uint16 *data, Uint16 dlc)
{
    union CAN_IF1CMD_REG CAN_IF1CMD_SHADOW;

    while(CanaRegs.CAN_IF1CMD.bit.Busy)
    {
    }

    CanaRegs.CAN_IF1DATA.bit.Data_0 = 0;
    CanaRegs.CAN_IF1DATA.bit.Data_1 = 0;
    CanaRegs.CAN_IF1DATA.bit.Data_2 = 0;
    CanaRegs.CAN_IF1DATA.bit.Data_3 = 0;
    CanaRegs.CAN_IF1DATB.bit.Data_4 = 0;
    CanaRegs.CAN_IF1DATB.bit.Data_5 = 0;
    CanaRegs.CAN_IF1DATB.bit.Data_6 = 0;
    CanaRegs.CAN_IF1DATB.bit.Data_7 = 0;

    if(dlc > 0) CanaRegs.CAN_IF1DATA.bit.Data_0 = data[0];
    if(dlc > 1) CanaRegs.CAN_IF1DATA.bit.Data_1 = data[1];
    if(dlc > 2) CanaRegs.CAN_IF1DATA.bit.Data_2 = data[2];
    if(dlc > 3) CanaRegs.CAN_IF1DATA.bit.Data_3 = data[3];
    if(dlc > 4) CanaRegs.CAN_IF1DATB.bit.Data_4 = data[4];
    if(dlc > 5) CanaRegs.CAN_IF1DATB.bit.Data_5 = data[5];
    if(dlc > 6) CanaRegs.CAN_IF1DATB.bit.Data_6 = data[6];
    if(dlc > 7) CanaRegs.CAN_IF1DATB.bit.Data_7 = data[7];

    CanaRegs.CAN_IF1MCTL.bit.DLC = dlc;
    CanaRegs.CAN_IF1MCTL.bit.EoB = 1;

    CAN_IF1CMD_SHADOW.all = 0;
    CAN_IF1CMD_SHADOW.bit.DIR = 1;
    CAN_IF1CMD_SHADOW.bit.DATA_A = 1;
    CAN_IF1CMD_SHADOW.bit.DATA_B = 1;
    CAN_IF1CMD_SHADOW.bit.Control = 1;
    CAN_IF1CMD_SHADOW.bit.TXRQST = 1;
    CAN_IF1CMD_SHADOW.bit.MSG_NUM = objID;

    CanaRegs.CAN_IF1CMD.all = CAN_IF1CMD_SHADOW.all;

    while(CanaRegs.CAN_IF1CMD.bit.Busy)
    {
    }
}

Uint16 can_get_frame(Uint32 objID, volatile Uint16 *data, Uint16 dlc)
{
    union CAN_IF2CMD_REG CAN_IF2CMD_SHADOW;

    CAN_IF2CMD_SHADOW.all = 0;
    CAN_IF2CMD_SHADOW.bit.Control = 1;
    CAN_IF2CMD_SHADOW.bit.DATA_A = 1;
    CAN_IF2CMD_SHADOW.bit.DATA_B = 1;
    CAN_IF2CMD_SHADOW.bit.MSG_NUM = objID;

    CanaRegs.CAN_IF2CMD.all = CAN_IF2CMD_SHADOW.all;

    while(CanaRegs.CAN_IF2CMD.bit.Busy)
    {
    }

    if(CanaRegs.CAN_IF2MCTL.bit.NewDat == 1)
    {
        if(dlc > 0) data[0] = CanaRegs.CAN_IF2DATA.bit.Data_0;
        if(dlc > 1) data[1] = CanaRegs.CAN_IF2DATA.bit.Data_1;
        if(dlc > 2) data[2] = CanaRegs.CAN_IF2DATA.bit.Data_2;
        if(dlc > 3) data[3] = CanaRegs.CAN_IF2DATA.bit.Data_3;
        if(dlc > 4) data[4] = CanaRegs.CAN_IF2DATB.bit.Data_4;
        if(dlc > 5) data[5] = CanaRegs.CAN_IF2DATB.bit.Data_5;
        if(dlc > 6) data[6] = CanaRegs.CAN_IF2DATB.bit.Data_6;
        if(dlc > 7) data[7] = CanaRegs.CAN_IF2DATB.bit.Data_7;

        CAN_IF2CMD_SHADOW.all = CanaRegs.CAN_IF2CMD.all;
        CAN_IF2CMD_SHADOW.bit.TxRqst = 1;
        CAN_IF2CMD_SHADOW.bit.MSG_NUM = objID;
        CanaRegs.CAN_IF2CMD.all = CAN_IF2CMD_SHADOW.all;

        return 1;
    }

    return 0;
}

Uint16 can_wait_receive_frame(Uint16 frameIndex)
{
    Uint32 timeout;
    Uint32 objID;

    objID = CAN_RX_MSG_OBJ_BASE + frameIndex;
    timeout = CAN_RX_TIMEOUT;

    while(timeout > 0UL)
    {
        if(can_get_frame(objID,
                         &g_canRxFrame[frameIndex][0],
                         g_canFrameDlc[frameIndex]))
        {
            return 1;
        }

        timeout--;
    }

    g_canTimeoutCount++;
    return 0;
}

Uint16 can_compare_frame(Uint16 frameIndex)
{
    Uint16 i;
    Uint16 dlc;

    dlc = g_canFrameDlc[frameIndex];

    for(i = 0; i < dlc; i++)
    {
        if(g_canRxFrame[frameIndex][i] != g_canTxFrame[frameIndex][i])
        {
            g_canLastFailFrame = frameIndex;
            g_canLastFailByte = i;
            return 0;
        }
    }

    return 1;
}

//Packet Pack and Send

void can_pack_packet_to_frames(Uint16 bufferIndex)
{
    Uint16 frame;
    Uint16 byteInFrame;
    Uint16 wordIndex;
    Uint16 i;
    Uint16 word;

    for(frame = 0; frame < CAN_FRAME_COUNT; frame++)
    {
        for(i = 0; i < CAN_BYTES_PER_FRAME; i++)
        {
            g_canTxFrame[frame][i] = 0;
            g_canRxFrame[frame][i] = 0;
        }
    }

    for(wordIndex = 0; wordIndex < PACKET_WORDS; wordIndex++)
    {
        word = g_packet[bufferIndex][wordIndex];

        frame = (wordIndex * 2U) / CAN_BYTES_PER_FRAME;
        byteInFrame = (wordIndex * 2U) % CAN_BYTES_PER_FRAME;

        g_canTxFrame[frame][byteInFrame] = (word & 0x00FFU);
        g_canTxFrame[frame][byteInFrame + 1U] = ((word >> 8U) & 0x00FFU);
    }
}

void can_send_packet_buffer(Uint16 bufferIndex)
{
    Uint16 frame;
    Uint16 pass;

    pass = 1;

    g_canLastSendPass = 0;
    g_canLastFailFrame = 0xFFFFU;
    g_canLastFailByte = 0xFFFFU;

    can_pack_packet_to_frames(bufferIndex);

    for(frame = 0; frame < CAN_FRAME_COUNT; frame++)
    {
        can_send_frame(CAN_TX_MSG_OBJ_BASE + frame,
                       &g_canTxFrame[frame][0],
                       g_canFrameDlc[frame]);

        g_canFrameSendCount++;

#if CAN_ENABLE_LOOPBACK_CHECK
        if(can_wait_receive_frame(frame) == 0)
        {
            pass = 0;
            g_canLastFailFrame = frame;
            break;
        }

        if(can_compare_frame(frame) == 0)
        {
            pass = 0;
            break;
        }
#endif
    }

    if(pass == 1)
    {
        g_canLastSendPass = 1;
        g_canPacketSendCount++;

#if CAN_ENABLE_LOOPBACK_CHECK
        g_canLoopbackPassCount++;
#endif
    }
    else
    {
        g_canLastSendPass = 0;
        g_canLoopbackFailCount++;
    }
}

//Error

void error(void)
{
    asm(" ESTOP0");

    for(;;)
    {
    }
}
