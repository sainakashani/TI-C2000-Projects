#include "F28x_Project.h"
#include <string.h>
#include "F021_F2837xD_C28x.h"

/*
 * C2000 Custom Flash Bootloader - CPU1
 *
 * Bootloader address:
 *   0x080000
 *
 * Application address:
 *   0x0A0000
 *
 * Application metadata address:
 *   0x0BE000
 *
 * SPI protocol:
 *   H7 / STM32H7 = SPI Master
 *   C2000        = SPIA Slave
 *   SPI mode     = Mode 0, CPOL=0, CPHA=0
 *   Word size    = 16-bit
 *
 * H7 must send 14 x 16-bit words:
 *
 *   magic low word
 *   magic high word
 *   appStartAddress low word
 *   appStartAddress high word
 *   appVersion low word
 *   appVersion high word
 *   appBuildDate low word
 *   appBuildDate high word
 *   appSize low word
 *   appSize high word
 *   appCrc low word
 *   appCrc high word
 *   validFlag low word
 *   validFlag high word
 */

#ifdef __TI_COMPILER_VERSION__
    #if __TI_COMPILER_VERSION__ >= 15009000
        #define ramFuncSection ".TI.ramfunc"
    #else
        #define ramFuncSection "ramfuncs"
    #endif
#endif

#define APP_START_ADDR              0x0A0000UL
#define APP_METADATA_ADDR           0x0BE000UL

#define APP_MAGIC_VALUE             0xA55A5AA5UL
#define APP_VALID_FLAG              0x000000A5UL

#define CPU_FREQ_MHZ                200U

#define REQUIRE_SPI_METADATA_FROM_H7    1

#define SPI_RX_TIMEOUT_FOREVER      0xFFFFFFFFUL

#define APP_METADATA_U32_COUNT      7U
#define APP_METADATA_WORD_COUNT     14U

typedef void (*app_entry_t)(void);

typedef struct
{
    Uint32 magic;
    Uint32 appStartAddress;
    Uint32 appVersion;
    Uint32 appBuildDate;
    Uint32 appSize;
    Uint32 appCrc;
    Uint32 validFlag;
} AppMetadata_t;

/*
 * These linker symbols must exist in the flash linker command file.
 * They normally exist in the standard F2837xD FLASH linker file.
 */
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadSize;
extern Uint16 RamfuncsRunStart;

/*
 * Debug-visible globals.
 * You can add these in Expressions window.
 */
volatile AppMetadata_t g_rxMetadata;
volatile Uint16 g_bootState = 0;
volatile Uint16 g_spiWordCount = 0;
volatile Uint16 g_flashWriteResult = 0;

/*
 * Flash programming buffers.
 * Aligned because Flash API programming/verification buffers should be aligned.
 */
#pragma DATA_ALIGN(g_flashProgramWords, 8)
Uint16 g_flashProgramWords[APP_METADATA_WORD_COUNT];

#pragma DATA_ALIGN(g_flashVerifyWords, 8)
uint32 g_flashVerifyWords[APP_METADATA_U32_COUNT];

/*
 * Function prototypes.
 */
void delay_loop_boot(void);

void init_boot_gpio(void);
void init_spia_gpio(void);
void init_spia_slave(void);

void bootloader_start_blink(void);

Uint16 spia_receive_word(Uint16 *data, Uint32 timeout);
Uint16 receive_metadata_from_h7_spi(AppMetadata_t *meta);

Uint32 make_u32_from_words(Uint16 lowWord, Uint16 highWord);
void metadata_to_program_words(const AppMetadata_t *meta, Uint16 *words);
void metadata_to_verify_words(const AppMetadata_t *meta, uint32 *words);

Uint16 is_metadata_ram_valid(const AppMetadata_t *meta);
Uint16 is_application_valid(void);

Uint16 write_app_metadata_to_flash(const AppMetadata_t *meta);
Uint16 flash_api_init(void);

void jump_to_application(void);

void error_no_valid_app(void);
void error_spi_timeout(void);
void error_invalid_spi_metadata(void);
void error_flash_write(void);

#pragma CODE_SECTION(write_app_metadata_to_flash, ramFuncSection);
#pragma CODE_SECTION(flash_api_init, ramFuncSection);

void main(void)
{
    AppMetadata_t rxMeta;

    /*
     * Step 1: Basic system initialization.
     */
    InitSysCtrl();

#ifdef _FLASH
    /*
     * Copy ramfuncs to RAM.
     * Flash API wrapper functions must run from RAM.
     */
    memcpy(&RamfuncsRunStart,
           &RamfuncsLoadStart,
           (size_t)&RamfuncsLoadSize);
#endif

    /*
     * Initialize Flash wait states.
     */
    InitFlash();

    /*
     * Step 2: Disable interrupts.
     */
    DINT;

    InitPieCtrl();

    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();

    /*
     * Step 3: Configure GPIO31 as bootloader indicator.
     */
    init_boot_gpio();

    /*
     * Step 4: Configure SPIA as slave.
     */
    init_spia_gpio();
    init_spia_slave();

    /*
     * Step 5: Blink GPIO31 to show bootloader started.
     */
    bootloader_start_blink();

#if REQUIRE_SPI_METADATA_FROM_H7

    /*
     * Step 6:
     * Wait for H7 to send metadata through SPI.
     */
    g_bootState = 10;

    if(receive_metadata_from_h7_spi(&rxMeta) == 0U)
    {
        error_spi_timeout();
    }

    /*
     * Store a copy in global variable for debug watch.
     */
    g_rxMetadata = rxMeta;

    /*
     * Step 7:
     * Check received metadata before writing it to Flash.
     */
    g_bootState = 20;

    if(is_metadata_ram_valid(&rxMeta) == 0U)
    {
        error_invalid_spi_metadata();
    }

    /*
     * Step 8:
     * Erase/program/verify metadata sector in Flash.
     */
    g_bootState = 30;

    if(write_app_metadata_to_flash(&rxMeta) == 0U)
    {
        error_flash_write();
    }

#endif

    /*
     * Step 9:
     * Validate application using metadata stored in Flash.
     */
    g_bootState = 40;

    if(is_application_valid())
    {
        g_bootState = 50;
        jump_to_application();
    }
    else
    {
        error_no_valid_app();
    }

    while(1)
    {
    }
}

void init_boot_gpio(void)
{
    EALLOW;

    /*
     * GPIO31 is on GPIO port A.
     * Set GPIO31 as normal GPIO output.
     */
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;

    EDIS;
}

void init_spia_gpio(void)
{
    EALLOW;

    /*
     * Enable internal pull-ups.
     * GPIO16 = SPISIMOA
     * GPIO17 = SPISOMIA
     * GPIO18 = SPICLKA
     * GPIO19 = SPISTEA
     */
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;

    /*
     * Asynchronous qualification for SPI pins.
     */
    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3;

    /*
     * Configure pins as SPIA.
     */
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1;

    EDIS;
}

void init_spia_slave(void)
{
    EALLOW;

    /*
     * Enable SPIA peripheral clock.
     */
    CpuSysRegs.PCLKCR8.bit.SPI_A = 1;

    EDIS;

    /*
     * Put SPI in reset before configuration.
     */
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;

    /*
     * SPI Mode 0:
     * CPOL = 0
     * CPHA = 0
     *
     * 16-bit character length.
     */
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;
    SpiaRegs.SPICCR.bit.SPICHAR = 15;
    SpiaRegs.SPICCR.bit.SPILBK = 0;

    /*
     * Slave mode.
     * TALK = 1 allows C2000 to drive SOMI.
     * H7 may ignore returned data.
     */
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 0;
    SpiaRegs.SPICTL.bit.TALK = 1;
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0;
    SpiaRegs.SPICTL.bit.SPIINTENA = 0;

    /*
     * SPIBRR is not used as clock source in slave mode,
     * but keep it initialized.
     */
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = 0x007F;

    /*
     * Continue SPI operation during debug halt.
     */
    SpiaRegs.SPIPRI.bit.FREE = 1;

    /*
     * Release SPI from reset.
     */
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;
}

void bootloader_start_blink(void)
{
    Uint16 i;

    for(i = 0; i < 6U; i++)
    {
        GpioDataRegs.GPATOGGLE.all = 0x80000000UL;
        delay_loop_boot();
    }
}

Uint16 spia_receive_word(Uint16 *data, Uint32 timeout)
{
    /*
     * Preload dummy data for the word that C2000 shifts out.
     * H7 can ignore MISO/SOMI in this test.
     */
    SpiaRegs.SPITXBUF = 0xA500U;

    if(timeout == SPI_RX_TIMEOUT_FOREVER)
    {
        while(SpiaRegs.SPISTS.bit.INT_FLAG == 0U)
        {
        }
    }
    else
    {
        while(SpiaRegs.SPISTS.bit.INT_FLAG == 0U)
        {
            if(timeout == 0U)
            {
                return 0U;
            }

            timeout--;
        }
    }

    *data = SpiaRegs.SPIRXBUF;

    return 1U;
}

Uint16 receive_metadata_from_h7_spi(AppMetadata_t *meta)
{
    Uint16 w[APP_METADATA_WORD_COUNT];
    Uint16 i;

    for(i = 0U; i < APP_METADATA_WORD_COUNT; i++)
    {
        if(spia_receive_word(&w[i], SPI_RX_TIMEOUT_FOREVER) == 0U)
        {
            return 0U;
        }

        g_spiWordCount = i + 1U;
    }

    /*
     * H7 sends low 16-bit word first, then high 16-bit word.
     */
    meta->magic           = make_u32_from_words(w[0],  w[1]);
    meta->appStartAddress = make_u32_from_words(w[2],  w[3]);
    meta->appVersion      = make_u32_from_words(w[4],  w[5]);
    meta->appBuildDate    = make_u32_from_words(w[6],  w[7]);
    meta->appSize         = make_u32_from_words(w[8],  w[9]);
    meta->appCrc          = make_u32_from_words(w[10], w[11]);
    meta->validFlag       = make_u32_from_words(w[12], w[13]);

    return 1U;
}

Uint32 make_u32_from_words(Uint16 lowWord, Uint16 highWord)
{
    Uint32 value;

    value = ((Uint32)highWord << 16) | ((Uint32)lowWord);

    return value;
}

void metadata_to_program_words(const AppMetadata_t *meta, Uint16 *words)
{
    words[0]  = (Uint16)(meta->magic & 0xFFFFUL);
    words[1]  = (Uint16)((meta->magic >> 16) & 0xFFFFUL);

    words[2]  = (Uint16)(meta->appStartAddress & 0xFFFFUL);
    words[3]  = (Uint16)((meta->appStartAddress >> 16) & 0xFFFFUL);

    words[4]  = (Uint16)(meta->appVersion & 0xFFFFUL);
    words[5]  = (Uint16)((meta->appVersion >> 16) & 0xFFFFUL);

    words[6]  = (Uint16)(meta->appBuildDate & 0xFFFFUL);
    words[7]  = (Uint16)((meta->appBuildDate >> 16) & 0xFFFFUL);

    words[8]  = (Uint16)(meta->appSize & 0xFFFFUL);
    words[9]  = (Uint16)((meta->appSize >> 16) & 0xFFFFUL);

    words[10] = (Uint16)(meta->appCrc & 0xFFFFUL);
    words[11] = (Uint16)((meta->appCrc >> 16) & 0xFFFFUL);

    words[12] = (Uint16)(meta->validFlag & 0xFFFFUL);
    words[13] = (Uint16)((meta->validFlag >> 16) & 0xFFFFUL);
}

void metadata_to_verify_words(const AppMetadata_t *meta, uint32 *words)
{
    words[0] = (uint32)meta->magic;
    words[1] = (uint32)meta->appStartAddress;
    words[2] = (uint32)meta->appVersion;
    words[3] = (uint32)meta->appBuildDate;
    words[4] = (uint32)meta->appSize;
    words[5] = (uint32)meta->appCrc;
    words[6] = (uint32)meta->validFlag;
}

Uint16 is_metadata_ram_valid(const AppMetadata_t *meta)
{
    if(meta->magic != APP_MAGIC_VALUE)
    {
        return 0U;
    }

    if(meta->appStartAddress != APP_START_ADDR)
    {
        return 0U;
    }

    if(meta->validFlag != APP_VALID_FLAG)
    {
        return 0U;
    }

    return 1U;
}

Uint16 is_application_valid(void)
{
    volatile const AppMetadata_t *meta;
    volatile Uint16 *appFirstWord;

    meta = (volatile const AppMetadata_t *)APP_METADATA_ADDR;
    appFirstWord = (volatile Uint16 *)APP_START_ADDR;

    /*
     * If first word of application is 0xFFFF,
     * the application flash area is erased.
     */
    if(*appFirstWord == 0xFFFFU)
    {
        return 0U;
    }

    if(meta->magic != APP_MAGIC_VALUE)
    {
        return 0U;
    }

    if(meta->appStartAddress != APP_START_ADDR)
    {
        return 0U;
    }

    if(meta->validFlag != APP_VALID_FLAG)
    {
        return 0U;
    }

    /*
     * appSize and appCrc are still not used in this step.
     * They will be used in the next phase for real firmware validation.
     */

    return 1U;
}

Uint16 flash_api_init(void)
{
    Fapi_StatusType status;

    EALLOW;

    /*
     * Initialize Flash API for 200 MHz CPU.
     */
    status = Fapi_initializeAPI(F021_CPU0_BASE_ADDRESS, CPU_FREQ_MHZ);

    if(status != Fapi_Status_Success)
    {
        EDIS;
        return 0U;
    }

    /*
     * Select Flash Bank 0.
     */
    status = Fapi_setActiveFlashBank(Fapi_FlashBank0);

    if(status != Fapi_Status_Success)
    {
        EDIS;
        return 0U;
    }

    EDIS;

    return 1U;
}

Uint16 write_app_metadata_to_flash(const AppMetadata_t *meta)
{
    Fapi_StatusType status;
    Fapi_FlashStatusWordType flashStatusWord;
    volatile Fapi_FlashStatusType flashStatus;

    metadata_to_program_words(meta, g_flashProgramWords);
    metadata_to_verify_words(meta, g_flashVerifyWords);

    if(flash_api_init() == 0U)
    {
        return 0U;
    }

    EALLOW;

    /*
     * Gain flash pump semaphore.
     */
    SeizeFlashPump();

    /*
     * Erase the sector that starts at APP_METADATA_ADDR.
     * IMPORTANT:
     * This erases the whole sector containing 0x0BE000.
     * Do not place App code or constants in this sector.
     */
    status = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
                                               (uint32 *)APP_METADATA_ADDR);

    if(status != Fapi_Status_Success)
    {
        ReleaseFlashPump();
        EDIS;
        return 0U;
    }

    while(Fapi_checkFsmForReady() != Fapi_Status_FsmReady)
    {
    }

    flashStatus = Fapi_getFsmStatus();

    if(flashStatus != 0U)
    {
        ReleaseFlashPump();
        EDIS;
        return 0U;
    }

    /*
     * Blank check only the metadata area we are going to use.
     * 7 x 32-bit words = 14 x 16-bit words.
     */
    status = Fapi_doBlankCheck((uint32 *)APP_METADATA_ADDR,
                               APP_METADATA_U32_COUNT,
                               &flashStatusWord);

    if(status != Fapi_Status_Success)
    {
        ReleaseFlashPump();
        EDIS;
        return 0U;
    }

    /*
     * Program first 8 x 16-bit words.
     */
    status = Fapi_issueProgrammingCommand((uint32 *)APP_METADATA_ADDR,
                                          &g_flashProgramWords[0],
                                          8U,
                                          0,
                                          0,
                                          Fapi_AutoEccGeneration);

    while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy)
    {
    }

    if(status != Fapi_Status_Success)
    {
        ReleaseFlashPump();
        EDIS;
        return 0U;
    }

    flashStatus = Fapi_getFsmStatus();

    if(flashStatus != 0U)
    {
        ReleaseFlashPump();
        EDIS;
        return 0U;
    }

    /*
     * Program remaining 6 x 16-bit words.
     * Address + 8 because Flash API addresses are 16-bit word addresses.
     */
    status = Fapi_issueProgrammingCommand((uint32 *)(APP_METADATA_ADDR + 8UL),
                                          &g_flashProgramWords[8],
                                          6U,
                                          0,
                                          0,
                                          Fapi_AutoEccGeneration);

    while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy)
    {
    }

    if(status != Fapi_Status_Success)
    {
        ReleaseFlashPump();
        EDIS;
        return 0U;
    }

    flashStatus = Fapi_getFsmStatus();

    if(flashStatus != 0U)
    {
        ReleaseFlashPump();
        EDIS;
        return 0U;
    }

    /*
     * Verify all 7 x 32-bit metadata words.
     */
    status = Fapi_doVerify((uint32 *)APP_METADATA_ADDR,
                           APP_METADATA_U32_COUNT,
                           g_flashVerifyWords,
                           &flashStatusWord);

    if(status != Fapi_Status_Success)
    {
        ReleaseFlashPump();
        EDIS;
        return 0U;
    }

    /*
     * Release flash pump semaphore.
     */
    ReleaseFlashPump();

    EDIS;

    g_flashWriteResult = 1U;

    return 1U;
}

void jump_to_application(void)
{
    DINT;

    IER = 0x0000;
    IFR = 0x0000;

    /*
     * Stable direct long branch to application code_start.
     */
    asm(" LB 0x0A0000");

    while(1)
    {
    }
}

void error_no_valid_app(void)
{
    g_bootState = 100U;

    while(1)
    {
        GpioDataRegs.GPATOGGLE.all = 0x80000000UL;
        delay_loop_boot();
        delay_loop_boot();
        delay_loop_boot();
    }
}

void error_spi_timeout(void)
{
    g_bootState = 110U;

    while(1)
    {
        GpioDataRegs.GPATOGGLE.all = 0x80000000UL;
        delay_loop_boot();
    }
}

void error_invalid_spi_metadata(void)
{
    g_bootState = 120U;

    while(1)
    {
        GpioDataRegs.GPATOGGLE.all = 0x80000000UL;
        delay_loop_boot();
        delay_loop_boot();
    }
}

void error_flash_write(void)
{
    g_bootState = 130U;

    while(1)
    {
        GpioDataRegs.GPATOGGLE.all = 0x80000000UL;
        delay_loop_boot();
        delay_loop_boot();
        delay_loop_boot();
        delay_loop_boot();
    }
}

void delay_loop_boot(void)
{
    volatile Uint32 i;

    for(i = 0; i < 1500000UL; i++)
    {
    }
}
