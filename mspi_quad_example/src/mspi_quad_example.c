//*****************************************************************************
//
//! @file mspi_quad_example.c
//! edit by luomengjie
//!
//! @brief Example of the MSPI operation with Quad SPI Flash.and PDM MIC
//!
//! Purpose: This example configures an MSPI connected flash device in Quad mode
//! and performs various operations - verifying the correctness of the same
//! Operations include - Erase, Write, Read, Read using XIP Apperture and XIP.
//!
//|
//!
//! GPIO 11 - PDM DATA
//! GPIO 12 - PDM CLK
//! GPIO 19 - MSPI CE0
//! GPIO 24 - MSPI CLK
//! GPIO 23 - MSPI HOLD
//! GPIO 4  - MSPI WP
//! GPIO 26 - MSPI MISO
//! GPIO 22 - MSPI MOSI
//! GPIO 29 - UART_TX
//! GPIO 28 - UART_RX
//! BOTTON0 - Short Press to start/pause;
//!           Long  Press to Reset Flash,erase all data
//! BOTTON1 - Press to transfer flash data to PC by uart

//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2019, Ambiq Micro
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
// 
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 2.2.0 of the AmbiqSuite Development Package.
//
//*****************************************************************************


#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_devices_mspi_flash.h"
#include "am_util.h"

//*****************************************************************************
//
// Macros CTIME
//
//*****************************************************************************
//
// The default is to use the LFRC as the clock source.
// Can  use the XTAL via the USE_XTAL define.
//
//#define USE_XTAL    1
#if USE_XTAL
#define BC_CLKSRC   "XTAL"
#else
#define BC_CLKSRC   "LFRC"
#endif

#ifdef AM_BSP_NUM_LEDS
#define NUM_LEDS    AM_BSP_NUM_LEDS
#else
#define NUM_LEDS    5       // Make up an arbitrary number of LEDs
#endif

//*****************************************************************************
//
// CTIME Globals parameter
//
//*****************************************************************************
volatile uint32_t g_ui32TimerCount = 0;

//**************************************
// Timer configuration.
//**************************************
am_hal_ctimer_config_t g_sTimer0 =
{
    // Don't link timers.
    0,

    // Set up Timer0A.
    (AM_HAL_CTIMER_FN_REPEAT    |
     AM_HAL_CTIMER_INT_ENABLE   |
#if USE_XTAL
     AM_HAL_CTIMER_XT_256HZ),
#else
     AM_HAL_CTIMER_LFRC_32HZ),
#endif

    // No configuration for Timer0B.
    0,
};



//*****************************************************************************
//
// Function to initialize Timer A0 to interrupt every 1/4 second.
//
//*****************************************************************************
void
timerA0_init(void)
{
    uint32_t ui32Period;

    //
    // Enable the LFRC.
    //
#if USE_XTAL
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_XTAL_START, 0);
#else
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_LFRC_START, 0);
#endif

    //
    // Set up timer A0.
    //
    am_hal_ctimer_clear(0, AM_HAL_CTIMER_TIMERA);
    am_hal_ctimer_config(0, &g_sTimer0);

    //
    // Set up timerA0 to 32Hz from LFRC divided to 1/4 second period.
    //
    ui32Period = 8;
#if USE_XTAL
    ui32Period *= 8;
#endif
    am_hal_ctimer_period_set(0, AM_HAL_CTIMER_TIMERA, ui32Period,
                             (ui32Period >> 1));

    //
    // Clear the timer Interrupt
    //
    am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);
}



//*****************************************************************************
//
// RTC parameter
//
//*****************************************************************************
//
// String arrays to index Days and Months with the values returned by the RTC.
//
char *pcWeekday[] =
{
    "Sunday",
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday",
    "Invalid day"
};
char *pcMonth[] =
{
    "January",
    "February",
    "March",
    "April",
    "May",
    "June",
    "July",
    "August",
    "September",
    "October",
    "November",
    "December",
    "Invalid month"
};

am_hal_rtc_time_t hal_time;
uint32_t    g_LastSecond = 0;
uint32_t    g_TestCount = 0;
//*****************************************************************************
//
// Support function:
// toVal() converts a string to an ASCII value.
//
//*****************************************************************************
int
toVal(char *pcAsciiStr)
{
  int iRetVal = 0;
  iRetVal += pcAsciiStr[1] - '0';
  iRetVal += pcAsciiStr[0] == ' ' ? 0 : (pcAsciiStr[0] - '0') * 10;
  return iRetVal;
}

//*****************************************************************************
//
// Support function:
// mthToIndex() converts a string indicating a month to an index value.
// The return value is a value 0-12, with 0-11 indicating the month given
// by the string, and 12 indicating that the string is not a month.
//
//*****************************************************************************
int
mthToIndex(char *pcMon)
{
  int idx;
  for (idx = 0; idx < 12; idx++)
  {
    if ( am_util_string_strnicmp(pcMonth[idx], pcMon, 3) == 0 )
    {
      return idx;
    }
  }
  return 12;
}

//*****************************************************************************
//
// UART handle.
//
//*****************************************************************************



//*****************************************************************************
//
// UART parameters.
//
//*****************************************************************************

bool botton2       = false;
bool uart_transfer = false;

	

//*****************************************************************************
//
// GPIO parameters
//
//*****************************************************************************

//
// Set up the configuration for BUTTON0.
//
const am_hal_gpio_pincfg_t g_deepsleep_button0 =
{
    .uFuncSel = 3,
    .eIntDir = AM_HAL_GPIO_PIN_INTDIR_HI2LO,
    .eGPInput = AM_HAL_GPIO_PIN_INPUT_ENABLE,
};

//
// Set up the configuration for BUTTON1.
//
const am_hal_gpio_pincfg_t g_deepsleep_button1 =
{
    .uFuncSel = 3,
    .eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
    .eGPInput = AM_HAL_GPIO_PIN_INPUT_ENABLE,
};


//*****************************************************************************
//
// PDM parameters.
//
//*****************************************************************************


#define PDM_SIZE                (528)
#define PDM_BYTES               (526 * 4) //matching for flash one page(2048+64 byte),8 byte for flag
#define PRINT_PDM_DATA           0

#define PDM_FFT_SIZE             4096
#define PDM_FFT_BYTES            (PDM_FFT_SIZE * 2)

//*****************************************************************************
//
// Global PDM variables.
//
//*****************************************************************************
volatile bool g_bPDMDataReady = false;
volatile bool buff_switch     = false;  //false:buff1;true:buff2
bool          power_on        = false;  //false:power off;true:power on
bool          power_up        = true ;  //first power on
bool          g_ui32TBottonEnd= false;

bool          botton1         = false;
uint32_t      g_ui32BottonStatus; //the botton value of GIPI(high or low)

uint32_t      count = 0 ;

#if defined(fft_test)
  uint32_t      g_ui32PDMDataBuffer1[PDM_FFT_SIZE];  //4byte *528 = 2112 byte = 1page
  uint32_t      g_ui32PDMDataBuffer2[PDM_FFT_SIZE];
  float         g_fPDMTimeDomain[PDM_FFT_SIZE * 2];
  float         g_fPDMFrequencyDomain[PDM_FFT_SIZE * 2];
  float         g_fPDMMagnitudes[PDM_FFT_SIZE * 2];
#else
  uint32_t      g_ui32PDMDataBuffer1[PDM_SIZE];  //4byte *528 = 2112 byte = 1page
  uint32_t      g_ui32PDMDataBuffer2[PDM_SIZE];
#endif
uint32_t      g_ui32SampleFreq;



//*****************************************************************************
//
// PDM configuration information.
//
//*****************************************************************************
void *PDMHandle;

am_hal_pdm_config_t g_sPdmConfig =
{
    .eClkDivider = AM_HAL_PDM_MCLKDIV_1,
    .eLeftGain   = AM_HAL_PDM_GAIN_0DB,
    .eRightGain  = AM_HAL_PDM_GAIN_0DB,
    .ui32DecimationRate = 24,
    .bHighPassEnable = 0,
    .ui32HighPassCutoff = 0xB,
    .ePDMClkSpeed = AM_HAL_PDM_CLK_750KHZ,
    .bInvertI2SBCLK = 0,
    .ePDMClkSource = AM_HAL_PDM_INTERNAL_CLK,
    .bPDMSampleDelay = 0,
    .bDataPacking = 1,
    .ePCMChannels = AM_HAL_PDM_CHANNEL_RIGHT,
    .ui32GainChangeDelay = 1,
    .bI2SEnable = 0, 
    .bSoftMute = 0,
    .bLRSwap = 0,
};


//*****************************************************************************
//
// PDM initialization.
//
//*****************************************************************************
void
pdm_init(void)
{
    //
    // Initialize, power-up, and configure the PDM.
    //
    am_hal_pdm_initialize(0, &PDMHandle);
    am_hal_pdm_power_control(PDMHandle, AM_HAL_PDM_POWER_ON, false);
    am_hal_pdm_configure(PDMHandle, &g_sPdmConfig);
    am_hal_pdm_enable(PDMHandle);

    //
    // Configure the necessary pins.
    //
    am_hal_gpio_pincfg_t sPinCfg = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    sPinCfg.uFuncSel = AM_HAL_PIN_11_PDMDATA;
    am_hal_gpio_pinconfig(11, sPinCfg);

    sPinCfg.uFuncSel = AM_HAL_PIN_12_PDMCLK;
    am_hal_gpio_pinconfig(12, sPinCfg);

//    am_hal_gpio_state_write(14, AM_HAL_GPIO_OUTPUT_CLEAR);
//    am_hal_gpio_pinconfig(14, g_AM_HAL_GPIO_OUTPUT);

    //
    // Configure and enable PDM interrupts (set up to trigger on DMA
    // completion).
    //
    am_hal_pdm_interrupt_enable(PDMHandle, (AM_HAL_PDM_INT_DERR
                                            | AM_HAL_PDM_INT_DCMP
                                            | AM_HAL_PDM_INT_UNDFL
                                            | AM_HAL_PDM_INT_OVF));

    NVIC_EnableIRQ(PDM_IRQn); 
}

//*****************************************************************************
//
// Print PDM configuration data.
//
//*****************************************************************************
void
pdm_config_print(void)
{
    uint32_t ui32PDMClk;
    uint32_t ui32MClkDiv;
//    float fFrequencyUnits;

    //
    // Read the config structure to figure out what our internal clock is set
    // to.
    //
    switch (g_sPdmConfig.eClkDivider)
    {
        case AM_HAL_PDM_MCLKDIV_4: ui32MClkDiv = 4; break;
        case AM_HAL_PDM_MCLKDIV_3: ui32MClkDiv = 3; break;
        case AM_HAL_PDM_MCLKDIV_2: ui32MClkDiv = 2; break;
        case AM_HAL_PDM_MCLKDIV_1: ui32MClkDiv = 1; break;

        default:
            ui32MClkDiv = 0;
    }

    switch (g_sPdmConfig.ePDMClkSpeed)
    {
        case AM_HAL_PDM_CLK_12MHZ:  ui32PDMClk = 12000000; break;
        case AM_HAL_PDM_CLK_6MHZ:   ui32PDMClk =  6000000; break;
        case AM_HAL_PDM_CLK_3MHZ:   ui32PDMClk =  3000000; break;
        case AM_HAL_PDM_CLK_1_5MHZ: ui32PDMClk =  1500000; break;
        case AM_HAL_PDM_CLK_750KHZ: ui32PDMClk =   750000; break;
        case AM_HAL_PDM_CLK_375KHZ: ui32PDMClk =   375000; break;
        case AM_HAL_PDM_CLK_187KHZ: ui32PDMClk =   187000; break;

        default:
            ui32PDMClk = 0;
    }

    //
    // Record the effective sample frequency. We'll need it later to print the
    // loudest frequency from the sample.
    //
    g_ui32SampleFreq = (ui32PDMClk /
                        (ui32MClkDiv * 2 * g_sPdmConfig.ui32DecimationRate));


    am_util_stdio_printf("Settings:\n");
    am_util_stdio_printf("PDM Clock (Hz):         %12d\n", ui32PDMClk);
    am_util_stdio_printf("Decimation Rate:        %12d\n", g_sPdmConfig.ui32DecimationRate);
    am_util_stdio_printf("Effective Sample Freq.: %12d\n", g_ui32SampleFreq);

}

//*****************************************************************************
//
// Start a transaction to get some number of bytes from the PDM interface.
//
//*****************************************************************************
void
pdm_data_get(void)
{
  //
  // Configure DMA and target address.
  //
  am_hal_pdm_transfer_t sTransfer;
  if(buff_switch)
  {
    sTransfer.ui32TargetAddr = (uint32_t ) g_ui32PDMDataBuffer1;
//	am_util_stdio_printf("put in pdm buffer1\n");
  }
  else
  {
    sTransfer.ui32TargetAddr = (uint32_t ) g_ui32PDMDataBuffer2;
//    am_util_stdio_printf("put in pdm buffer2\n");
  }
  
  sTransfer.ui32TotalCount = PDM_BYTES;

  //
  // Start the data transfer.
  //
  
//  am_hal_pdm_enable(PDMHandle);
//  am_util_delay_ms(15);
  am_hal_pdm_fifo_flush(PDMHandle);
  am_hal_pdm_dma_start(PDMHandle, &sTransfer);

}


//*****************************************************************************
//
// Flash parameters.
//
//*****************************************************************************


#define    AM_BSP_GPIO_GP19      19

const am_hal_gpio_pincfg_t g_AM_BSP_GPIO_GP19_CE0 =
{
  .uFuncSel            = AM_HAL_PIN_23_GPIO,
  .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_8MA,
  .ePullup             = AM_HAL_GPIO_PIN_PULLUP_WEAK,
  .eGPOutcfg           = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL,
  .eGPInput            = AM_HAL_GPIO_PIN_INPUT_NONE
};


#define MPSI_TEST_PAGE_INCR     (AM_DEVICES_MSPI_FLASH_PAGE_SIZE*3)
#define MSPI_SECTOR_INCR        (33)
#define MSPI_INT_TIMEOUT        (100)

#define MSPI_TARGET_SECTOR      (16) 
#define MSPI_TARGET_ADDRESS     0x40000 // {10bit,6bit,12bit},{block address,page address,column address}
#define MSPI_BUFFER_SIZE        (2*1024+64)  // 8k example buffer size.
//#define MSPI_BUFFER_SIZE        PDM_BYTES

#define DEFAULT_TIMEOUT         10000

#define MSPI_TEST_MODULE        0

#define MSPI_XIP_BASE_ADDRESS   0x04000000

//#define START_SPEED_INDEX       0
//#define END_SPEED_INDEX         11

uint32_t          DMATCBBuffer[2560];
uint8_t           TestBuffer[2048];
uint8_t           DummyBuffer[1024];
uint8_t           g_SectorTXBuffer[MSPI_BUFFER_SIZE];
uint8_t           g_SectorRXBuffer[MSPI_BUFFER_SIZE];
volatile uint32_t targer_address[2] = {MSPI_TARGET_ADDRESS,0};
volatile uint32_t column_address;

volatile bool     g_bMSPIDataReady = false;
volatile bool     timeflag = false;
volatile uint32_t timestamp[6];
uint32_t blocklist[20]={0};



void            *pHandle = NULL;


const am_hal_mspi_dev_config_t      MSPI_Flash_Serial_CE0_MSPIConfig =
{
  .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
  .eClockFreq           = AM_HAL_MSPI_CLK_8MHZ,
#if defined(MICRON_N25Q256A)
  .ui8TurnAround        = 3,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
#elif defined (CYPRESS_S25FS064S)
  .ui8TurnAround        = 3,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
#elif defined (MACRONIX_MX25U12835F)
  .ui8TurnAround        = 8,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
#elif defined (ADESTO_ATXP032)
  .ui8TurnAround        = 8,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE,  
#elif defined (GIGA_5F1GQ4RB91G)
	.ui8TurnAround      = 0,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_1_BYTE,  
#endif
  .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
  .eDeviceConfig        = AM_HAL_MSPI_FLASH_SERIAL_CE0,
  .bSeparateIO          = true,
  .bSendInstr           = true,
  .bSendAddr            = true,
  .bTurnaround          = true,
  .ui8ReadInstr         = AM_DEVICES_MSPI_GD5FXGQ4XC_READ_FROM_CACHE,
  .ui8WriteInstr        = AM_DEVICES_MSPI_GD5FXGQ4XC_PROGRAM_LOAD ,
  .ui32TCBSize          = (sizeof(DMATCBBuffer) / sizeof(uint32_t)),
  .pTCB                 = DMATCBBuffer,
  .scramblingStartAddr  = 0,
  .scramblingEndAddr    = 0,
};

const am_hal_mspi_dev_config_t      MSPI_Flash_Quad_CE0_MSPIConfig =
{
  .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
  .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
#if defined(MICRON_N25Q256A)
  .ui8TurnAround        = 3,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
#elif defined (CYPRESS_S25FS064S)
  .ui8TurnAround        = 3,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
#elif defined (MACRONIX_MX25U12835F)
  .ui8TurnAround        = 8,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
#elif defined (ADESTO_ATXP032)
  .ui8TurnAround        = 8,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE,
#elif defined (GIGA_5F1GQ4RB91G)
  .ui8TurnAround        = 8,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,    
#endif
  .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
  .eDeviceConfig        = AM_HAL_MSPI_FLASH_QUAD_CE0,
  .bSeparateIO          = false,
  .bSendInstr           = true,
#if defined (GIGA_5F1GQ4RB91G)
  .bSendAddr            = false,
#else
  .bSendAddr            = true,
#endif
  .bTurnaround          = true,
#if (defined(MICRON_N25Q256A) || defined(MACRONIX_MX25U12835F) || defined(ADESTO_ATXP032))
  .ui8ReadInstr         = AM_DEVICES_MSPI_FLASH_FAST_READ,
#elif defined (CYPRESS_S25FS064S)
  .ui8ReadInstr         = AM_DEVICES_MSPI_FLASH_QUAD_IO_READ,
#endif          // TODO - Flag an error if another part.
  .ui8WriteInstr        = AM_DEVICES_MSPI_FLASH_PAGE_PROGRAM,
  .ui32TCBSize          = (sizeof(DMATCBBuffer) / sizeof(uint32_t)),
  .pTCB                 = DMATCBBuffer,
  .scramblingStartAddr  = 0,
  .scramblingEndAddr    = 0,
};

static void
soft_reset(void)
{
  uint32_t ui32Status;

  
  am_hal_pdm_disable(PDMHandle);
			  
  am_devices_led_array_init(am_bsp_psLEDs, AM_BSP_NUM_LEDS);
			  
  ui32Status=am_devices_mspi_flash_reset(MSPI_TEST_MODULE);
  if (AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS != ui32Status)
  {
    am_util_stdio_printf("Failed to RESET the MSPI and Flash Device correctly!\r\n");
  }
  ui32Status=am_devices_mspi_flash_mass_erase(MSPI_TEST_MODULE);
  {
    am_util_stdio_printf("Failed to MASS ERASE the MSPI and Flash Device correctly!\r\n");
  }

  targer_address[0] = MSPI_TARGET_ADDRESS;
//  targer_address[0] = 0XBFC0000;
  targer_address[1] = 0;
			   
//  column_address = 0xFF81000;
  power_up = true;			  
  am_devices_mspi_flash_write(MSPI_TEST_MODULE,(uint8_t *)targer_address, 0xFF81000, 8); 
  am_devices_mspi_flash_read(MSPI_TEST_MODULE,(uint8_t *)targer_address, 0xFF81000, 8,true); 
	
  am_devices_mspi_flash_bad_block(MSPI_TEST_MODULE, blocklist);

  
  for (int ix = 0; ix < AM_BSP_NUM_LEDS; ix++)
  {
    am_util_delay_ms(500); 
    am_devices_led_off(am_bsp_psLEDs, ix);
  }

}



static void 
address_get (void)
{

  //
  // getting a data write address after poweron
  //

  uint32_t count = 0;  //for power on to find the next block address
  
  while(1)
  {
	if(power_up) // FISRT POWER UP,Read Register From 1022 block
	{
	  am_devices_mspi_flash_read(MSPI_TEST_MODULE,g_SectorRXBuffer, 0xFF81000+count,MSPI_BUFFER_SIZE,true);    //7FC0000,511 block for register
	}
	else // if system hold,fisrt read register from next address
	{
	  am_devices_mspi_flash_read(MSPI_TEST_MODULE,g_SectorRXBuffer, column_address,MSPI_BUFFER_SIZE,true);	   
	}
	uint32_t *current_address = (uint32_t *)g_SectorRXBuffer;
	for(uint32_t i = 2;i < MSPI_BUFFER_SIZE/8; i=i+1)
	{ 
		if(current_address[i] == 0xffffffff && current_address[i+1] == 0xffffffff)
		{
		  targer_address[0] =  current_address[i-2];
		  targer_address[1] =  current_address[i-1];
		  power_up	  = false;
		  return;
		}
	}
	count = count + 0x1000;   
  }
  
}


static void 
ConfigGP19AsGpioOutputForPullHighCE0Pin (void)
{

  //
  // Configure the pin as a push-pull GPIO output.
  //
  am_hal_gpio_pinconfig(AM_BSP_GPIO_GP19, g_AM_BSP_GPIO_GP19_CE0); 
  //
  // Disable the output driver, and set the output value to the high level
  // state.  Note that for Apollo3 GPIOs in push-pull mode, the output
  // enable, normally a tri-state control, instead functions as an enable
  // for Fast GPIO. Its state does not matter on previous chips, so for
  // normal GPIO usage on Apollo3, it must be disabled.
  //
  am_hal_gpio_state_write(AM_BSP_GPIO_GP19, AM_HAL_GPIO_OUTPUT_TRISTATE_DISABLE);
  am_hal_gpio_state_write(AM_BSP_GPIO_GP19, AM_HAL_GPIO_OUTPUT_SET);   // pull high

}

static void 
time_stamp (void)
{

  //
  // get the timestamp
  //



  am_hal_rtc_time_get(&hal_time);


  timestamp[0] = hal_time.ui32Hour;
  timestamp[1] = hal_time.ui32Minute;
  timestamp[2] = hal_time.ui32Second;
  timestamp[3] = hal_time.ui32Month+1;
  timestamp[4] = hal_time.ui32DayOfMonth;
  timestamp[5] = hal_time.ui32Year;

  

}

void
pdm2mspi(void)
{
	uint32_t ui32Status;
	uint8_t *pi8PCMData;
	uint8_t *timebuffer;
	uint32_t i_step;


  //
  // Generate data into the Sector Buffer
  //

    
  if(!buff_switch)
  {
    pi8PCMData = (uint8_t *) g_ui32PDMDataBuffer1;
//    am_util_stdio_printf("read from pdm buffer1\n");
  }
  else
  {
    pi8PCMData = (uint8_t *) g_ui32PDMDataBuffer2;
//	am_util_stdio_printf("read from pdm buffer2\n");
  }


  if(timeflag)
  {
    time_stamp();
    timebuffer = (uint8_t*) timestamp;
    g_SectorTXBuffer[0]=0x00;
    g_SectorTXBuffer[1]=0x00;
    g_SectorTXBuffer[2]=0x00;
    g_SectorTXBuffer[3]=0x00;
    g_SectorTXBuffer[4]=0x00;
    g_SectorTXBuffer[5]=0x00;
    g_SectorTXBuffer[6]=0x00;
    g_SectorTXBuffer[7]=0x00;
    for(uint32_t i = 0; i < 24; i++)
    {g_SectorTXBuffer[i+8]= timebuffer[i];}
    i_step = 24;
  }
  else
  {
    g_SectorTXBuffer[0]=0x00;
    g_SectorTXBuffer[1]=0x00;
    g_SectorTXBuffer[2]=0x01;
    g_SectorTXBuffer[3]=0x00;
    g_SectorTXBuffer[4]=0x01;
    g_SectorTXBuffer[5]=0x00;
    g_SectorTXBuffer[6]=0x00;
    g_SectorTXBuffer[7]=0x00;
    i_step = 0;
  }
  //
  // Generate data into the Sector Buffer
  //
	
  for (uint32_t i = i_step; i < PDM_BYTES; i++)
  {
    g_SectorTXBuffer[i+8] = pi8PCMData[i];

  }

  //
  //bad block and page 0 skip
  //
  for(uint32_t i=0;i<19;i++)
  {
	if((targer_address[0] >> 18) == blocklist[i]){ targer_address[0] = targer_address[0]+0x40000;}
  }
  if((targer_address[0] & 0x0003F000) == 0){targer_address[0] = targer_address[0] +0x1000;}

	
  //
  // Write the TX buffer into the target sector.
  //
  am_util_stdio_printf("Writing %d Bytes to the %d blcok ,the %d page\n", MSPI_BUFFER_SIZE, targer_address[0] >> 18,((targer_address[0] >> 12) & 0x0000003F));
  ui32Status = am_devices_mspi_flash_write(MSPI_TEST_MODULE, g_SectorTXBuffer, targer_address[0], MSPI_BUFFER_SIZE);
  if (AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS != ui32Status)
  {
    am_util_stdio_printf("Failed to write buffer to Flash Device!\n");
  }

  targer_address[0] = targer_address[0]+0x1000;
  
  g_bMSPIDataReady = true;

  timeflag = false;

}


static void 
uart_data_get(void)
{
  uint32_t read_address = MSPI_TARGET_ADDRESS;

  if(uart_transfer)
  {
    am_devices_led_on(am_bsp_psLEDs, 1);
    for(;read_address< 0xff7f000;)
    {  
      //
      // read bad block flag
      //


      for(uint32_t i=0;i<19;i++)
      {
        if((read_address >> 18) == blocklist[i]){ read_address = read_address+0x40000;}
      }
      if((read_address & 0x0003F000) == 0){read_address = read_address +0x1000;}

      //
      //getting data in flash which transfer by uart 
      //
      am_devices_mspi_flash_read(MSPI_TEST_MODULE,g_SectorRXBuffer,read_address ,MSPI_BUFFER_SIZE,true);
      if((g_SectorRXBuffer[0] == 0xff) & ((g_SectorRXBuffer[1] == 0xff) ))
      {
        am_util_stdio_printf("\r\nFlash READ OVER !\r\n");
        uart_transfer = false;
		am_util_delay_ms(500); //for flash power on timing
		am_devices_led_off(am_bsp_psLEDs, 1);
        return;
      }
      else
      {
        for(uint32_t j = 0; j<MSPI_BUFFER_SIZE; j++)
        {
          am_util_stdio_printf("%02x",g_SectorRXBuffer[j]); 	  
        }	  
      }
      read_address = read_address + 0x1000;
      if(!uart_transfer){
        am_util_stdio_printf("\r\nFlash READ Interrput !\r\n");
        break;}		
    }
  }
  am_devices_led_off(am_bsp_psLEDs, 1);
  uart_transfer = false;
  
}



//*****************************************************************************
//
// PDM interrupt handler.
//
//*****************************************************************************
void
am_pdm0_isr(void)
{
  uint32_t ui32Status;

  //
  // Read the interrupt status.
  //
  am_hal_pdm_interrupt_status_get(PDMHandle, &ui32Status, true);
  am_hal_pdm_interrupt_clear(PDMHandle, ui32Status);

  //
  // Once our DMA transaction completes, we will disable the PDM and send a
  // flag back down to the main routine. Disabling the PDM is only necessary
  // because this example only implemented a single buffer for storing FFT
  // data. More complex programs could use a system of multiple buffers to
  // allow the CPU to run the FFT in one buffer while the DMA pulls PCM data
  // into another buffer.
  //
  if (ui32Status & AM_HAL_PDM_INT_DCMP)
  {
    g_bPDMDataReady = true;
//    am_hal_pdm_disable(PDMHandle);
    buff_switch = !buff_switch;
  }

}

//*****************************************************************************
//
// GPIO ISR
//
//*****************************************************************************
void
am_gpio_isr(void)
{
  uint64_t g_IntStatus;
  am_hal_gpio_interrupt_status_get(true,&g_IntStatus);
	
  if(((g_IntStatus >> 16) & 0x1) == 1)
  { 
    botton1 = true;
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON0));
    //
    // Start timer A0
    //
    am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERA);

    g_ui32TimerCount = 0;
  }
	
  if(((g_IntStatus >> 18) & 0x1) == 1)
  {
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON1));
    uart_transfer = !uart_transfer;
    botton2 = true;
	
    //
    // Start timer A0
    //
    am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERA);

    g_ui32TimerCount = 0;
  }
}


//*****************************************************************************
//
// Timer Interrupt Service Routine (ISR)
//
//*****************************************************************************
void
am_ctimer_isr(void)
{
  //
  // Increment count and set limit based on the number of LEDs available.
  //

  am_hal_gpio_state_read(AM_BSP_GPIO_BUTTON0, AM_HAL_GPIO_INPUT_READ,&g_ui32BottonStatus);

  if (!g_ui32BottonStatus)
  {
    g_ui32TimerCount++ ;
  }
  else
  {
    //
    // Stop timer A0.
    //
    am_hal_ctimer_clear(0, AM_HAL_CTIMER_TIMERA);
    g_ui32TBottonEnd = true;
  }

  //
  // Clear TimerA0 Interrupt (write to clear).
  //
  am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);
}



//*****************************************************************************
//
// MSPI Example Main.
//
//*****************************************************************************
int
main(void)
{
	
    //*****************************************************************************
    //
    // GLOBAL Config
    //
    //*****************************************************************************

    //
    // Set the clock frequency.
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    // Configute the GPIO Pull High
    //
    ConfigGP19AsGpioOutputForPullHighCE0Pin();


    //*****************************************************************************
    //
    // Uart Inital and Config 
    //
    //*****************************************************************************

#if 1
    //
    // Initialize the printf interface for UART output.
    //
    am_bsp_uart_printf_enable();
#else
    //
    // Initialize the printf interface for ITM/SWO output.
    //
    am_bsp_itm_printf_enable();

#endif

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("Apollo3 PDM TO MSPI Example\r\n");
		

    //*****************************************************************************
    //
    // RTC Inital and Config 
    //
    //*****************************************************************************

    //
    // Set the RTC time 
    // WARNING this will destroy any time epoch currently in the RTC.
    //
#define rtc_load
#if defined rtc_load
#if defined(__GNUC__)  ||  defined(__ARMCC_VERSION)  ||  defined(__IAR_SYSTEMS_ICC__)
    //
    // The RTC is initialized from the date and time strings that are
    // obtained from the compiler at compile time.
    //
    hal_time.ui32Hour = toVal(&__TIME__[0]);
    hal_time.ui32Minute = toVal(&__TIME__[3]);
    hal_time.ui32Second = toVal(&__TIME__[6]);
    hal_time.ui32Hundredths = 00;
    hal_time.ui32Weekday = am_util_time_computeDayofWeek(2000 + toVal(&__DATE__[9]), mthToIndex(&__DATE__[0]) + 1, toVal(&__DATE__[4]) );
    hal_time.ui32DayOfMonth = toVal(&__DATE__[4]);
    hal_time.ui32Month = mthToIndex(&__DATE__[0]);
    hal_time.ui32Year = toVal(&__DATE__[9]);
    hal_time.ui32Century = 0;
#else
    //
    // The RTC is initialized from an arbitrary date.
    //
    hal_time.ui32Hour = 14;
    hal_time.ui32Minute = 24;
    hal_time.ui32Second = 33;
    hal_time.ui32Hundredths = 50;
    hal_time.ui32Weekday = 3;
    hal_time.ui32DayOfMonth = 15;
    hal_time.ui32Month = 4;
    hal_time.ui32Year = 14;
    hal_time.ui32Century = 0;
#endif
#endif
    am_hal_rtc_time_set(&hal_time);
		
    //
    // Select XT for RTC clock source
    //
    am_hal_rtc_osc_select(AM_HAL_RTC_OSC_XT);

    //
    // Enable the RTC.
    //
    am_hal_rtc_osc_enable();



    //*****************************************************************************
    //
    // GPIO Botton Config
    //
    //*****************************************************************************
#if defined(AM_BSP_NUM_BUTTONS)  &&  defined(AM_BSP_NUM_LEDS)
    //
    // Configure the button pin.
    //
    am_hal_gpio_pinconfig(AM_BSP_GPIO_BUTTON0, g_deepsleep_button0);
	
    //
    // Clear the GPIO Interrupt (write to clear).
    //
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON0));
    
    //
    // Enable the GPIO/button interrupt.
    //
    am_hal_gpio_interrupt_enable(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON0));


    //
    // Configure the button pin.
    //
    am_hal_gpio_pinconfig(AM_BSP_GPIO_BUTTON1, g_deepsleep_button1);

    //
    // Clear the GPIO Interrupt (write to clear).
    //
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON1));
    
    //
    // Enable the GPIO/button interrupt.
    //
    am_hal_gpio_interrupt_enable(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON1));
    
    //
    // Configure the LEDs.Power On Indication
    //
    am_devices_led_array_init(am_bsp_psLEDs, AM_BSP_NUM_LEDS);
		
    //
    // Eanble GIPO interrput
    //
	
    NVIC_EnableIRQ(GPIO_IRQn);
    


#endif

    //*****************************************************************************
    //
    // Flash and MSPI Config.
    //
    //*****************************************************************************

    uint32_t      ui32Status;

    am_util_delay_ms(10); //for flash power on timing
    //
    // Configure the MSPI and Flash Device.
    //
    ui32Status = am_devices_mspi_flash_init(MSPI_TEST_MODULE, (am_hal_mspi_dev_config_t *)&MSPI_Flash_Serial_CE0_MSPIConfig, &pHandle);
    if (AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to configure the MSPI and Flash Device correctly!\r\n");
    }

    ui32Status = am_devices_mspi_flash_id(MSPI_TEST_MODULE);
    if (AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Flash Device ID is uncorrectly!\r\n");
    }

    am_devices_mspi_flash_bad_block(MSPI_TEST_MODULE, blocklist);

	
    //*****************************************************************************
    //
    // PDM Config.
    //
    //*****************************************************************************

    pdm_init(); 
	
    pdm_config_print();
	
    am_hal_pdm_fifo_flush(PDMHandle);
		
		
    //*****************************************************************************
    //
    // Print RTC Time to Confire RTC is Right
    //
    //*****************************************************************************
    am_hal_rtc_time_get(&hal_time);
    am_util_stdio_printf("\r\nIt is now ");
    am_util_stdio_printf("%d : ", hal_time.ui32Hour);
    am_util_stdio_printf("%02d : ", hal_time.ui32Minute);
    am_util_stdio_printf("%02d ", hal_time.ui32Second);
    am_util_stdio_printf(pcWeekday[hal_time.ui32Weekday]);
    am_util_stdio_printf(" ");
    am_util_stdio_printf(pcMonth[hal_time.ui32Month]);
    am_util_stdio_printf(" ");
    am_util_stdio_printf("%d, ", hal_time.ui32DayOfMonth);
    am_util_stdio_printf("20%02d", hal_time.ui32Year);
		
		
    //
    // Turn the LEDs off, Initialize Finish.
    //
    for (int ix = 0; ix < AM_BSP_NUM_LEDS; ix++)
    {
      am_devices_led_off(am_bsp_psLEDs, ix);
      am_util_delay_ms(500);
    }
		
		
		
		


    //*****************************************************************************
    //
    // Timer Config.
    //
    //*****************************************************************************

    //
    // TimerA0 init.
    //
    timerA0_init();

    //
    // Enable the timer Interrupt.
    //
    am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA0);


    //
    // Enable the timer interrupt in the NVIC.
    //
    NVIC_EnableIRQ(CTIMER_IRQn);






    //*****************************************************************************
    //
    // Main Process 
    //
    //*****************************************************************************

    am_hal_interrupt_master_enable();


    //
    // Loop forever while sleeping.
    //
    while (1)
    {

      //
      //botton0 behavior
      //
			
      if(g_ui32TBottonEnd && botton1)
      {
        botton1 = false;
        g_ui32TBottonEnd=false;
        if(g_ui32TimerCount > 4) 
        {
          soft_reset();
//           am_hal_pdm_disable(PDMHandle);
					
//           am_devices_led_array_init(am_bsp_psLEDs, AM_BSP_NUM_LEDS);
					
//           ui32Status=am_devices_mspi_flash_reset(MSPI_TEST_MODULE);
//					 if (AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS != ui32Status)
//           {
//             am_util_stdio_printf("Failed to RESET the MSPI and Flash Device correctly!\r\n");
//           }
//           ui32Status=am_devices_mspi_flash_mass_erase(MSPI_TEST_MODULE);
//					 {
//             am_util_stdio_printf("Failed to MASS ERASE the MSPI and Flash Device correctly!\r\n");
//           }
//           targer_address[0] = MSPI_TARGET_ADDRESS;
					
//           targer_address[1] = 0;
					 
//           column_address = 0xFF81000;
					
//           am_devices_mspi_flash_write(MSPI_TEST_MODULE,(uint8_t *)targer_address, 0xFF81000, 8);			
							
//           for (int ix = 0; ix < AM_BSP_NUM_LEDS; ix++)
//           {
//             am_util_delay_ms(500); 
//             am_devices_led_off(am_bsp_psLEDs, ix);
//           }
        }
        else
        {
          am_devices_led_toggle(am_bsp_psLEDs, 0);	
          power_on = !power_on;
					
          if(power_on)
          {
            timeflag = true;
//            uint32_t count = 0;  //for power up to find the next block address
//            bool     address_get = false;
//            while(1)
//            {
//              if(power_up) // FISRT POWER UP,Read Register From 511 block
//              {
//                am_devices_mspi_flash_read(MSPI_TEST_MODULE,g_SectorRXBuffer, 0xFF81000+count,MSPI_BUFFER_SIZE,true);	 //511 block for register
//              }
//              else
//              {
//                am_devices_mspi_flash_read(MSPI_TEST_MODULE,g_SectorRXBuffer, column_address,MSPI_BUFFER_SIZE,true);	 
//              }
//              uint32_t *current_address = (uint32_t *)g_SectorRXBuffer;
//              for(uint32_t i = 2;i < MSPI_BUFFER_SIZE/8; i=i+1)
//              {	
//                  if(current_address[i] == 0xffffffff && current_address[i+1] == 0xffffffff)
//                  {
//             	    targer_address[0] =  current_address[i-2];
//                    targer_address[1] =  current_address[i-1];
//                    power_up    = false;
//                    address_get = true;
//                    break;
//                  }
//              }
//              if(address_get){break;}
//              count = count + 0x1000;	
//            }
            address_get();
            am_hal_pdm_enable(PDMHandle);
            am_util_delay_ms(100);
            pdm_data_get();
          }
          else
          {
            uint32_t page_offset;
            targer_address[1] = targer_address[1] + 1;
            page_offset       = targer_address[1] / 264   ; 						
            column_address    = 0xFF81000 + page_offset * 0x1000 + (targer_address[1]<<3); //8 byte per write
					

            am_devices_mspi_flash_write(MSPI_TEST_MODULE,(uint8_t *)targer_address, column_address, 8);					
            am_hal_pdm_disable(PDMHandle);
          }
        }
      }		

			
      //
      //Botton2 Behavior
      //
			
      if(botton2 && !power_on && g_ui32TBottonEnd)
      {
        g_ui32TBottonEnd=false;
        botton2 = false;
        uart_data_get();

 //       if(uart_transfer)
 //       {
 //         am_devices_led_on(am_bsp_psLEDs, 1);
 //         uint32_t target_address = MSPI_TARGET_ADDRESS;
 //         //
 //         // Print the banner.
 //         //
 //         for(uint32_t i =0;i<200;i++)
 //         {
 //           am_devices_mspi_flash_read(MSPI_TEST_MODULE,g_SectorRXBuffer,target_address ,MSPI_BUFFER_SIZE,true);
 //           for(uint32_t i =0;i<MSPI_BUFFER_SIZE;i++)
 //           {
 //             am_util_stdio_printf("%02x",g_SectorRXBuffer[i]);				
 //           }
 //           target_address = target_address + 0x1000;
 //           if(!uart_transfer)
 //           {
 //             break;
 //           }					
 //         }
 //         am_devices_led_off(am_bsp_psLEDs, 1);
 //         uart_transfer = false;
        
       }

      //
      //PDM Data to MSPI
      //
      if(power_on)
      {
        if(g_bPDMDataReady)
        {
          g_bPDMDataReady = false;
						
          pdm_data_get();
					
          pdm2mspi();
        }      
      }
      //
      // Go to Deep Sleep.
      //
      am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

    }
		


}

