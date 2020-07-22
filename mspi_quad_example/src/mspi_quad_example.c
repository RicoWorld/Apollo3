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
// GPIO parameters
//
//*****************************************************************************

//
// Set up the configuration for BUTTON0.
//
const am_hal_gpio_pincfg_t g_deepsleep_button0 =
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
#define PDM_BYTES               (PDM_SIZE * 4) //matching for flash one page(2048+64 byte)
#define PRINT_PDM_DATA              0

//*****************************************************************************
//
// Global PDM variables.
//
//*****************************************************************************
volatile bool g_bPDMDataReady = false;
volatile bool buff_switch     = false;  //false:buff1;true:buff2
volatile bool power_on        = false;  //false:power off;true:power on
uint32_t      count = 0 ;

uint32_t      g_ui32PDMDataBuffer1[PDM_SIZE];  //4byte *528 = 2112 byte = 1page
uint32_t      g_ui32PDMDataBuffer2[PDM_SIZE];
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
    .ui32DecimationRate = 64,
    .bHighPassEnable = 0,
    .ui32HighPassCutoff = 0xB,
    .ePDMClkSpeed = AM_HAL_PDM_CLK_3MHZ,
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

    am_hal_gpio_state_write(14, AM_HAL_GPIO_OUTPUT_CLEAR);
    am_hal_gpio_pinconfig(14, g_AM_HAL_GPIO_OUTPUT);

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
	if(buff_switch){
    sTransfer.ui32TargetAddr = (uint32_t ) g_ui32PDMDataBuffer1;
	}
	else{sTransfer.ui32TargetAddr = (uint32_t ) g_ui32PDMDataBuffer2;
	}
    sTransfer.ui32TotalCount = PDM_BYTES;

    //
    // Start the data transfer.
    //
    am_hal_pdm_enable(PDMHandle);
    am_util_delay_ms(100);
    am_hal_pdm_fifo_flush(PDMHandle);
    am_hal_pdm_dma_start(PDMHandle, &sTransfer);
}







//*****************************************************************************
//
// Flash parameters.
//
//*****************************************************************************


#if FIREBALL_CARD
//
// The Fireball device card multiplexes various devices including each of an SPI
// and I2C FRAM. The Fireball device driver controls access to these devices.
// If the Fireball card is not used, FRAM devices can be connected directly
// to appropriate GPIO pins.
//
#include "am_devices_fireball.h"
#endif // FIREBALL_CARD

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
#define MSPI_TARGET_ADDRESS     0x40000 // the fisrt block fisrt page for GIGA Flash,save configure data
#define MSPI_BUFFER_SIZE        (2*1024+64)  // 8k example buffer size.

#define DEFAULT_TIMEOUT         10000

#define MSPI_TEST_MODULE        0

#define MSPI_XIP_BASE_ADDRESS 0x04000000

//#define START_SPEED_INDEX       0
//#define END_SPEED_INDEX         11

uint32_t        DMATCBBuffer[2560];
uint8_t         TestBuffer[2048];
uint8_t         DummyBuffer[1024];
uint8_t         g_SectorTXBuffer[MSPI_BUFFER_SIZE];
uint8_t         g_SectorRXBuffer[MSPI_BUFFER_SIZE];
uint32_t        targer_address = MSPI_TARGET_ADDRESS + 0x1000;

volatile bool   g_bMSPIDataReady = false;

    void          *pHandle = NULL;


const am_hal_mspi_dev_config_t      MSPI_Flash_Serial_CE0_MSPIConfig =
{
  .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
  .eClockFreq           = AM_HAL_MSPI_CLK_3MHZ,
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


void
pdm2mspi(void)
{
	uint32_t ui32Status;
	uint8_t *pi8PCMData;

 
  g_bMSPIDataReady = false;

  //
  // Generate data into the Sector Buffer
  //
    
  if(buff_switch){
    pi8PCMData = (uint8_t *) g_ui32PDMDataBuffer1;
  }
  else{
    pi8PCMData = (uint8_t *) g_ui32PDMDataBuffer2;
  }
  //
  // Generate data into the Sector Buffer
  //
		
  for (uint32_t i = 0; i < MSPI_BUFFER_SIZE; i++)
  {
    g_SectorTXBuffer[i] = pi8PCMData[i];
  }
  //
  // Write the TX buffer into the target sector.
  //
  am_util_stdio_printf("Writing %d Bytes to Sector %x\n", MSPI_BUFFER_SIZE, targer_address);
  ui32Status = am_devices_mspi_flash_write(MSPI_TEST_MODULE, g_SectorTXBuffer, targer_address, MSPI_BUFFER_SIZE);
  if (AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS != ui32Status)
  {
    am_util_stdio_printf("Failed to write buffer to Flash Device!\n");
  }

  targer_address=targer_address+0x1000;
  g_bMSPIDataReady = true;

    //
    // Read the data back into the RX buffer.
    //
//    am_util_stdio_printf("Read %d Bytes from Sector %x\n", MSPI_BUFFER_SIZE, MSPI_TARGET_ADDRESS >> 12);
//    ui32Status = am_devices_mspi_flash_read(MSPI_TEST_MODULE,g_SectorRXBuffer, MSPI_TARGET_ADDRESS , MSPI_BUFFER_SIZE, true);
//    if (AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS != ui32Status)
//    {
//        am_util_stdio_printf("Failed to read buffer to Flash Device!\n");
//    }


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
//      am_hal_pdm_disable(PDMHandle);
			pdm_data_get();
      g_bPDMDataReady = true;
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
    uint32_t ui32Status;

	  //
    // Delay for debounce.
    //
//    am_util_delay_ms(30);

		//
    // Clear the GPIO Interrupt (write to clear).
    //
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON0));
    //
    // Toggle LED 0.
    //
#ifdef AM_BSP_NUM_LEDS
//    am_devices_led_toggle(am_bsp_psLEDs, 0);
#endif



	if(count > 15){
    //
		// Configure the LEDs.
		//
		am_devices_led_array_init(am_bsp_psLEDs, AM_BSP_NUM_LEDS);
	
		//
		// Turn the LEDs off, but initialize LED5 on so user will see something.
		//
		count = 0;
	}
	else {
    while(1)
			{
      if(!power_on){
		  am_devices_led_toggle(am_bsp_psLEDs, 0);

	    //
	    //Read the write address from Target Address
	    //
//	    ui32Status = am_devices_mspi_flash_read(MSPI_TEST_MODULE,g_SectorRXBuffer, MSPI_TARGET_ADDRESS , 4, true);
//        if (AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS != ui32Status)
//         {
//          am_util_stdio_printf("Failed to read buffer to Flash Device!\n");
//         }
//      targer_address  = (g_SectorRXBuffer[3]<<24);
//      targer_address |= (g_SectorRXBuffer[2]<<16);
//      targer_address |= (g_SectorRXBuffer[1]<<8 );
//      targer_address |= (g_SectorRXBuffer[0]    );

		   
      //
      // Toggle PDM 
      //
      pdm_init(); 
      pdm_config_print();
      am_hal_pdm_fifo_flush(PDMHandle);
      pdm_data_get();
	    power_on = true;
	    break;
	  }
//	  else if(g_bPDMDataReady & power_on) {
		  else{
	  	am_devices_led_toggle(am_bsp_psLEDs, 0);
	    //
	    //Stop PDM Safly
	    //
	    am_hal_pdm_disable(PDMHandle);

      //
      // Disable the  PDM interrupt 
      //
      NVIC_DisableIRQ(PDM_IRQn);

      //
      // Wait for MSPI Finish			
			// 
//        while(!g_bMSPIDataReady){} 
												
	      power_on = false;
			  am_util_stdio_printf("g_bPDMDataReady is %d \n",g_bPDMDataReady);
				am_util_stdio_printf("g_bMSPIReady is %d \n",g_bMSPIDataReady);
				g_bPDMDataReady = false;
				g_bMSPIDataReady = false;
  	    break;
   	  }
    }
	}

}


//*****************************************************************************
//
// Static function to be executed from external flash device
//
//*****************************************************************************
#if defined(__GNUC_STDC_INLINE__)
__attribute__((naked))
static void xip_test_function(void)
{
    __asm
    (
        "   nop\n"              // Just execute NOPs and return.
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   bx      lr\n"
    );
}

#elif defined(__ARMCC_VERSION)
__asm static void xip_test_function(void)
{
    nop                         // Just execute NOPs and return.
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    bx      lr
}

#elif defined(__IAR_SYSTEMS_ICC__)
__stackless static void xip_test_function(void)
{
    __asm("    nop");           // Just execute NOPs and return.
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    bx      lr");
}
#endif

#define MSPI_XIP_FUNCTION_SIZE  72
typedef void (*mspi_xip_test_function_t)(void);

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
    ConfigGP19AsGpioOutputForPullHighCE0Pin();

#if 0
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
    am_util_stdio_printf("Apollo3 PDM TO MSPI Example\n\n");
	
#ifdef AM_PART_APOLLO
		//
		// Power down all but the first SRAM banks.
		//
		am_hal_mcuctrl_sram_power_set(AM_HAL_MCUCTRL_SRAM_POWER_DOWN_1 |
									  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_2 |
									  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_3 |
									  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_4 |
									  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_5 |
									  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_6 |
									  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_7,
									  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_1 |
									  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_2 |
									  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_3 |
									  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_4 |
									  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_5 |
									  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_6 |
									  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_7);
#endif // AM_PART_APOLLO
	
#ifdef AM_PART_APOLLO2
	
		//
		// Turn OFF Flash1
		//
		PWRCTRL->MEMEN_b.FLASH1 = 0;
		while ( PWRCTRL->PWRONSTATUS_b.PD_FLAM1 != 0);
	
		//
		// Power down SRAM
		//
		PWRCTRL->SRAMPWDINSLEEP_b.SRAMSLEEPPOWERDOWN = PWRCTRL_SRAMPWDINSLEEP_SRAMSLEEPPOWERDOWN_ALLBUTLOWER8K;
	
#endif // AM_PART_APOLLO2
	
#ifdef AM_PART_APOLLO3
		//
		// Turn OFF Flash1
		//
//		if ( am_hal_pwrctrl_memory_enable(AM_HAL_PWRCTRL_MEM_FLASH_512K) )
//		{
//			while(1);
//		}
	
		//
		// Power down SRAM
		//
//		PWRCTRL->MEMPWDINSLEEP_b.SRAMPWDSLP = PWRCTRL_MEMPWDINSLEEP_SRAMPWDSLP_ALLBUTLOWER32K;
#endif // AM_PART_APOLLO3

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
		// Configure the LEDs.
		//
		am_devices_led_array_init(am_bsp_psLEDs, AM_BSP_NUM_LEDS);
	
		//
		// Turn the LEDs off, but initialize LED5 on so user will see something.
		//
		for (int ix = 0; ix < AM_BSP_NUM_LEDS-1; ix++)
		{
			am_devices_led_off(am_bsp_psLEDs, ix);
		    am_util_delay_ms(500);
		}

        //
        // Power On Indication
        //
		am_devices_led_on(am_bsp_psLEDs, 4);   



		
#endif

	//*****************************************************************************
	//
	// Flash and MSPI Config.
	//
	//*****************************************************************************

    uint32_t      ui32Status;
//    void          *pHandle = NULL;
//    uint32_t      funcAddr = ((uint32_t)&xip_test_function) & 0xFFFFFFFE;

    //
    // Cast a pointer to the begining of the sector as the test function to call.
    //
//    mspi_xip_test_function_t test_function = (mspi_xip_test_function_t)((MSPI_XIP_BASE_ADDRESS + (MSPI_TARGET_SECTOR << 16)) | 0x00000001);




#if FIREBALL_CARD
    //
    // Set the MUX for the Flash Device
    //
    uint32_t ui32Ret, ui32ID;

#if 1
    //
    // Get Fireball ID and Rev info.
    //
    ui32Ret = am_devices_fireball_control(AM_DEVICES_FIREBALL_STATE_ID_GET, &ui32ID);
    if ( ui32Ret != 0 )
    {
        am_util_stdio_printf("FAIL: am_devices_fireball_control(%d) returned 0x%X.\n",
                             AM_DEVICES_FIREBALL_STATE_ID_GET, ui32Ret);
        return -1;
    }
    else if ( ui32ID == FIREBALL_ID )
    {
        am_util_stdio_printf("Fireball found, ID is 0x%X.\n", ui32ID);
    }
    else
    {
        am_util_stdio_printf("Unknown device returned ID as 0x%X.\n", ui32ID);
    }

    ui32Ret = am_devices_fireball_control(AM_DEVICES_FIREBALL_STATE_VER_GET, &ui32ID);
    if ( ui32Ret != 0 )
    {
        am_util_stdio_printf("FAIL: am_devices_fireball_control(%d) returned 0x%X.\n",
                             AM_DEVICES_FIREBALL_STATE_VER_GET, ui32Ret);
        return -1;
    }
    else
    {
        am_util_stdio_printf("Fireball Version is 0x%X.\n", ui32ID);
    }
#endif

#if !defined(ADESTO_ATXP032)
    ui32Ret = am_devices_fireball_control(AM_DEVICES_FIREBALL_STATE_TWIN_QUAD_CE0_CE1, 0);
    if ( ui32Ret != 0 )
    {
        am_util_stdio_printf("FAIL: am_devices_fireball_control(%d) returned 0x%X.\n",
                             AM_DEVICES_FIREBALL_STATE_TWIN_QUAD_CE0_CE1, ui32Ret);
        return -1;
    }
#else
    ui32Ret = am_devices_fireball_control(AM_DEVICES_FIREBALL_STATE_OCTAL_FLASH_CE0, 0);
    if ( ui32Ret != 0 )
    {
        am_util_stdio_printf("FAIL: am_devices_fireball_control(%d) returned 0x%X.\n",
                             AM_DEVICES_FIREBALL_STATE_OCTAL_FLASH_CE0, ui32Ret);
        return -1;
    }
#endif
#endif // FIREBALL_CARD

    am_util_delay_ms(10); //for flash power on timing
    //
    // Configure the MSPI and Flash Device.
    //
    ui32Status = am_devices_mspi_flash_init(MSPI_TEST_MODULE, (am_hal_mspi_dev_config_t *)&MSPI_Flash_Serial_CE0_MSPIConfig, &pHandle);
    if (AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to configure the MSPI and Flash Device correctly!\n");
    }

	  ui32Status = am_devices_mspi_flash_id(MSPI_TEST_MODULE);
    if (AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Flash Device ID is uncorrectly!\n");
    }




	  am_devices_led_off(am_bsp_psLEDs, 4);


	//*****************************************************************************
	//
	// PDM config
	//
	//*****************************************************************************

	//
    // Turn on the PDM, set it up for our chosen recording settings, and start
    // the first DMA transaction.
    //
    //pdm_init();   // control by gpio interrupt
    //pdm_config_print();
    //am_hal_pdm_fifo_flush(PDMHandle);
    //pdm_data_get();


    //////////////////////////////////////////////
    // process start after enable GPIO interrput
    // bottom for start/end
    /////////////////////////////////////////////

    //
    // Eanble GIPO interrput
    //
		
		uint32_t ui32BottonStatus ;
	  NVIC_EnableIRQ(GPIO_IRQn);
    am_hal_interrupt_master_enable();
	
    //
    // Loop forever while sleeping.
    //
    while (1)
    {
//       am_hal_interrupt_master_disable();
//       am_hal_gpio_state_read(AM_BSP_GPIO_BUTTON0,AM_HAL_GPIO_INPUT_READ,&ui32BottonStatus);
	       while(!ui32BottonStatus){
//			 am_hal_gpio_state_read(AM_BSP_GPIO_BUTTON0,AM_HAL_GPIO_INPUT_READ,&ui32BottonStatus);
	     }	
        if (g_bPDMDataReady)
        {
            g_bPDMDataReady = false;

            am_util_stdio_printf("THE Flash Address[%x] Write Compelete\n\n",targer_address);					
			
            pdm2mspi();
			
            while (PRINT_PDM_DATA);

            //
            // Start converting the next set of PCM samples.
            //
//            pdm_data_get();
        }
			//
      // Go to Deep Sleep.
      //
      am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }


}

