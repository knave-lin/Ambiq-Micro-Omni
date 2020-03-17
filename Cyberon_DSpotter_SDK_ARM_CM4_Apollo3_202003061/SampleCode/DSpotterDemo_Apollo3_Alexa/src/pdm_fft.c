//*****************************************************************************
//
//! @file pdm_fft.c
//!
//! @brief An example to show basic PDM operation.
//!
//! This example enables the PDM interface to record audio signals from an
//! external microphone. The required pin connections are:
//!
//! GPIO 10 - PDM DATA
//! GPIO 11 - PDM CLK
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2018, Ambiq Micro
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
// This is part of revision v1.2.12-736-gdf97e703f of the AmbiqSuite Development Package.
//
//*****************************************************************************

#define ARM_MATH_CM4
#include <arm_math.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "portmacro.h"
#include "portable.h"
#include "projdefs.h"

#include "DSpotterSDKApi.h"

//*****************************************************************************
//
// Macro.
//
//*****************************************************************************
#define k_nMemSize				(64 * 1024)		//64KB
#define k_nMaxTime				(300)
#define BLOCK_SAMPLE_SIZE	(640)
#define BLOCK_DUMP_SIZE		(BLOCK_SAMPLE_SIZE * 3)

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
volatile bool g_bPDMDataReady = false;
int16_t g_i16PDMSampleMono[BLOCK_SAMPLE_SIZE];
int16_t g_i16Sample[BLOCK_SAMPLE_SIZE];
QueueHandle_t g_xSampleQueue;
volatile bool g_bDump = false;
uint8_t g_u8Buffer2[BLOCK_DUMP_SIZE];
HANDLE g_hDSpotter = NULL;
BYTE g_lpbyMemPool[k_nMemSize];
extern uint32_t u32CMDDataBegin;
extern uint32_t u32LicenseDataBegin;

//*****************************************************************************
//
// UART handle.
//
//*****************************************************************************
void *phUART;

#define CHECK_ERRORS(x)                                                       \
    if ((x) != AM_HAL_STATUS_SUCCESS)                                         \
    {                                                                         \
        error_handler(x);                                                     \
    }

volatile uint32_t ui32LastError;

//*****************************************************************************
//
// Catch HAL errors.
//
//*****************************************************************************
void
error_handler(uint32_t ui32ErrorStatus)
{
    ui32LastError = ui32ErrorStatus;

    while (1);
}

//*****************************************************************************
//
// UART buffers.
//
//*****************************************************************************
uint8_t g_pui8TxBuffer[BLOCK_DUMP_SIZE << 2];
uint8_t g_pui8RxBuffer[32];

//*****************************************************************************
//
// UART configuration.
//
//*****************************************************************************
const am_hal_uart_config_t g_sUartConfig =
{
    //
    // Standard UART settings: 921600-8-N-1
    //
    .ui32BaudRate = 921600,
    .ui32DataBits = AM_HAL_UART_DATA_BITS_8,
    .ui32Parity = AM_HAL_UART_PARITY_NONE,
    .ui32StopBits = AM_HAL_UART_ONE_STOP_BIT,
    .ui32FlowControl = AM_HAL_UART_FLOW_CTRL_NONE,

    //
    // Set TX and RX FIFOs to interrupt at half-full.
    //
    .ui32FifoLevels = (AM_HAL_UART_TX_FIFO_1_2 |
                       AM_HAL_UART_RX_FIFO_1_2),

    //
    // Buffers
    //
    .pui8TxBuffer = g_pui8TxBuffer,
    .ui32TxBufferSize = sizeof(g_pui8TxBuffer),
    .pui8RxBuffer = g_pui8RxBuffer,
    .ui32RxBufferSize = sizeof(g_pui8RxBuffer),
};

//*****************************************************************************
//
// UART0 interrupt handler.
//
//*****************************************************************************
void
am_uart_isr(void)
{
    //
    // Service the FIFOs as necessary, and clear the interrupts.
    //
    uint32_t ui32Status, ui32Idle;
    am_hal_uart_interrupt_status_get(phUART, &ui32Status, true);
    am_hal_uart_interrupt_clear(phUART, ui32Status);
    am_hal_uart_interrupt_service(phUART, ui32Status, &ui32Idle);
}

//*****************************************************************************
//
// UART print string
//
//*****************************************************************************
void
uart_print(char *pcStr)
{
    uint32_t ui32StrLen = 0;
    uint32_t ui32BytesWritten = 0;

    //
    // Measure the length of the string.
    //
    while (pcStr[ui32StrLen] != 0)
    {
        ui32StrLen++;
    }

    //
    // Print the string via the UART.
    //
    const am_hal_uart_transfer_t sUartWrite =
    {
        .ui32Direction = AM_HAL_UART_WRITE,
        .pui8Data = (uint8_t *) pcStr,
        .ui32NumBytes = ui32StrLen,
        .ui32TimeoutMs = 0,
        .pui32BytesTransferred = &ui32BytesWritten,
    };

    CHECK_ERRORS(am_hal_uart_transfer(phUART, &sUartWrite));

    if (ui32BytesWritten != ui32StrLen)
    {
        //
        // Couldn't send the whole string!!
        //
        while(1);
    }
}

//*****************************************************************************
//
// PDM configuration information.
//
//*****************************************************************************
void *PDMHandle;

am_hal_pdm_config_t g_sPdmConfig =
{
	.eClkDivider = AM_HAL_PDM_MCLKDIV_1,
	.eLeftGain = AM_HAL_PDM_GAIN_P285DB,
	.eRightGain = AM_HAL_PDM_GAIN_P285DB,
	.ui32DecimationRate = 0x18,
	.bHighPassEnable = 0,
	.ui32HighPassCutoff = 0xB,
	.ePDMClkSpeed = AM_HAL_PDM_CLK_750KHZ,
	.bInvertI2SBCLK = 0,
	.ePDMClkSource = AM_HAL_PDM_INTERNAL_CLK,
	.bPDMSampleDelay = 0,
	.bDataPacking = 1,
	.ePCMChannels = AM_HAL_PDM_CHANNEL_RIGHT,
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

	sPinCfg.uFuncSel = AM_HAL_PIN_10_PDMCLK;
	am_hal_gpio_pinconfig(10, sPinCfg);

	sPinCfg.uFuncSel = AM_HAL_PIN_11_PDMDATA;
	am_hal_gpio_pinconfig(11, sPinCfg);

	//am_hal_gpio_state_write(14, AM_HAL_GPIO_OUTPUT_CLEAR);
	//am_hal_gpio_pinconfig(14, g_AM_HAL_GPIO_OUTPUT);

	//
	// Configure and enable PDM interrupts (set up to trigger on DMA
	// completion).
	//
	am_hal_pdm_interrupt_enable(PDMHandle, (AM_HAL_PDM_INT_DERR
	                                        | AM_HAL_PDM_INT_DCMP
	                                        | AM_HAL_PDM_INT_UNDFL
	                                        | AM_HAL_PDM_INT_OVF));

#if AM_CMSIS_REGS
	NVIC_EnableIRQ(PDM_IRQn);
#else
	am_hal_interrupt_enable(AM_HAL_INTERRUPT_PDM);
#endif
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
	sTransfer.ui32TargetAddr = (uint32_t ) g_i16PDMSampleMono;
	sTransfer.ui32TotalCount = BLOCK_SAMPLE_SIZE << 1;

	//
	// Start the data transfer.
	//
	am_hal_pdm_dma_start(PDMHandle, &sTransfer);
	//am_hal_pdm_enable(PDMHandle);
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
	    g_bPDMDataReady = true;
}

//*****************************************************************************
//
// Unpack model.
//
//*****************************************************************************
INT 
UnpackBin(BYTE lpbyBin[], BYTE *lppbyModel[], INT nMaxNumModel)
{
	UINT *lpnBin = (UINT *)lpbyBin;
	INT nNumBin = lpnBin[0];
	UINT *lpnBinSize = lpnBin + 1;
	INT i;

	lppbyModel[0] = (BYTE *)(lpnBinSize + nNumBin);
	for (i = 1; i < nNumBin; i++){
		if (i >= nMaxNumModel)
			break;
		lppbyModel[i] = lppbyModel[i-1] + lpnBinSize[i-1];
	}

	return i;
}

//*****************************************************************************
//
// Initialize DSpotter SDK.
//
//*****************************************************************************
void 
DSpotterInit(void)
{
	INT nMemUsage, nErr;
	BYTE *ppbyModel[2] = {0};
	
	//
	// Unpack command data.
	//
	if(UnpackBin((BYTE *)&u32CMDDataBegin, ppbyModel, 2) < 2) {
		am_util_stdio_printf("Invalid bin\r\n");
		return;
	}

	//
	// Check the memory usage.
	//
	nMemUsage = DSpotter_GetMemoryUsage_Multi(ppbyModel[0], (BYTE **)&ppbyModel[1], 1, k_nMaxTime);
	if(nMemUsage > k_nMemSize){
		am_util_stdio_printf("need more memory, memory usage = %d\r\n", nMemUsage);
		return;
	}

	//
	// Initialize SDK.
	//
	g_hDSpotter = DSpotter_Init_Multi(ppbyModel[0], (BYTE **)&ppbyModel[1], 1, k_nMaxTime, g_lpbyMemPool, k_nMemSize, NULL, 0, &nErr, (BYTE *)&u32LicenseDataBegin);
	if(g_hDSpotter == NULL){
		am_util_stdio_printf("g_hDSpotter == NULL, nErr = %d\r\n", nErr);
		return;
	}
}

//*****************************************************************************
//
// Record PDM data.
//
//*****************************************************************************
void 
Record_Task(void *param)
{
	//
	// Turn on the PDM, set it up for our chosen recording settings, and start
	// the first DMA transaction.
	//
	pdm_init();
	am_hal_pdm_fifo_flush(PDMHandle);
	pdm_data_get();
	am_hal_pdm_enable(PDMHandle);

	//
	// Loop forever while sleeping.
	//
	while (1)
	{
		am_hal_interrupt_master_disable();

		if(g_bPDMDataReady)
		{
			g_bPDMDataReady = false;

			//
			// Push PDM data
			//
			if(xQueueSendToBack(g_xSampleQueue, g_i16PDMSampleMono, 0) == errQUEUE_FULL)
			{
				am_util_stdio_printf("Queue is full!\r\n");
			}
//			else
//			{
//				am_util_stdio_printf("send to the queue successfully\r\n");
//			}

			pdm_data_get();
		}

		//
		// Go to Deep Sleep.
		//
		am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

		am_hal_interrupt_master_enable();
	}
}

//*****************************************************************************
//
// Recognize PDM data.
//
//*****************************************************************************
void 
Recog_Task(void *param)
{
	INT nCmdID;
	int nCnt;
	bool bLightOn = false;
	uint8_t *u8Ptr;
	uint32_t ui32BytesWritten;
	
	//
	// Start recognition.
	//
	DSpotter_Reset(g_hDSpotter);
	am_util_stdio_printf("Waiting for voice trigger...\r\n");
	
	while(1)
  {
//		if(uxQueueMessagesWaiting(g_xSampleQueue) == 0)
//		{
//			am_util_stdio_printf("Queue is empty!\r\n");
//		}

		if(xQueueReceive(g_xSampleQueue, g_i16Sample, portMAX_DELAY) != pdPASS)
			am_util_stdio_printf("Could not receive from the queue\r\n");
//		else
//				am_util_stdio_printf("receive from the queue successfully.\r\n");

		if(DSpotter_AddSample(g_hDSpotter, g_i16Sample, BLOCK_SAMPLE_SIZE) == DSPOTTER_SUCCESS)
		{
			nCmdID = DSpotter_GetResult(g_hDSpotter);
//					am_util_stdio_printf( "ID: %d, %d item in queue\r\n", nCmdID, uxQueueMessagesWaiting(g_xSampleQueue));
			am_util_stdio_printf( "ID: %d\r\n", nCmdID);
			
			DSpotter_Reset(g_hDSpotter);
			
			for(int i = 0; i < AM_BSP_NUM_LEDS; i++)
						am_devices_led_on(am_bsp_psLEDs, i);
			
			nCnt = 0;
			bLightOn = true;
		}
		
		if(bLightOn)
		{
			nCnt += BLOCK_SAMPLE_SIZE;
			if(nCnt >= 16000)
			{
				bLightOn = false;
				for(int i = 0; i < AM_BSP_NUM_LEDS; i++)
						am_devices_led_off(am_bsp_psLEDs, i);
			}
		}

		if(g_bDump)
		{
			u8Ptr = (uint8_t *)g_i16Sample;
			ui32BytesWritten = 0;
			
			for(int i = 0, j = 0; i < BLOCK_DUMP_SIZE; i += 3, j += 2)
			{
				g_u8Buffer2[i] = u8Ptr[j];
				g_u8Buffer2[i + 1] = u8Ptr[j + 1];
				g_u8Buffer2[i + 2] = u8Ptr[j] ^ u8Ptr[j + 1] ^ 0xFF;
			}
		
			const am_hal_uart_transfer_t sUartWrite =
			{
				.ui32Direction = AM_HAL_UART_WRITE,
				.pui8Data = (uint8_t *)g_u8Buffer2,
				.ui32NumBytes = BLOCK_DUMP_SIZE,
				.ui32TimeoutMs = 0,
				.pui32BytesTransferred = &ui32BytesWritten,
			};
			CHECK_ERRORS(am_hal_uart_transfer(phUART, &sUartWrite));
		
			if (ui32BytesWritten != BLOCK_DUMP_SIZE)
			{
				am_util_stdio_printf("ui32BytesWritten != BLOCK_DUMP_SIZE, ui32BytesWritten = %d\r\n", ui32BytesWritten);
			}
		}
	}
}

//*****************************************************************************
//
// Dump PDM data.
//
//*****************************************************************************
void 
Dump_Task(void *param)
{
	uint8_t u8Buffer[32];
	uint32_t ui32BytesRead = 0;
  TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
	
	while(1)
	{
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
		
		const am_hal_uart_transfer_t sUartRead =
		{
			.ui32Direction = AM_HAL_UART_READ,
			.pui8Data = (uint8_t *)u8Buffer,
			.ui32NumBytes = sizeof(u8Buffer),
			.ui32TimeoutMs = 0,
			.pui32BytesTransferred = &ui32BytesRead,
		};
		CHECK_ERRORS(am_hal_uart_transfer(phUART, &sUartRead));
		
		if(ui32BytesRead > 0)
		{
			for(int i = 0; i < ui32BytesRead; i++)
			{
				if(u8Buffer[i] == 0x31)
					g_bDump = true;
				else if(u8Buffer[i] == 0x32)
					g_bDump = false;
			}
		}
	}
}

//*****************************************************************************
//
// Main
//
//*****************************************************************************
int
main(void)
{
#if ENABLE_BURST_MODE
	am_hal_burst_avail_e eBurstModeAvailable;
	am_hal_burst_mode_e eBurstMode;
#endif
	
	//
	// Perform the standard initialzation for clocks, cache settings, and
	// board-level low-power operation.
	//
	am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);
	am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
	am_hal_cachectrl_enable();
	am_bsp_low_power_init();

	//
	// Initialize the printf interface for UART output.
	//
	CHECK_ERRORS(am_hal_uart_initialize(0, &phUART));
	CHECK_ERRORS(am_hal_uart_power_control(phUART, AM_HAL_SYSCTRL_WAKE, false));
	CHECK_ERRORS(am_hal_uart_configure(phUART, &g_sUartConfig));

	//
	// Enable the UART pins.
	//
	am_hal_gpio_pinconfig(48, g_AM_BSP_GPIO_COM_UART_TX);
  am_hal_gpio_pinconfig(49, g_AM_BSP_GPIO_COM_UART_RX);

	//
	// Enable interrupts.
	//
#if AM_CMSIS_REGS
	NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + AM_BSP_UART_PRINT_INST));
#else // AM_CMSIS_REGS
	am_hal_interrupt_enable(AM_HAL_INTERRUPT_UART0);
#endif // AM_CMSIS_REGS
	am_hal_interrupt_master_enable();

#if 0
	//
	// Set the main print interface to use the UART print function we defined.
	//
	am_util_stdio_printf_init(uart_print);
#else
	//
	// Initialize the printf interface for ITM output
	//
	am_bsp_itm_printf_enable();
#endif

#if ENABLE_BURST_MODE
	//
	// Check that the Burst Feature is available.
	//
	if (AM_HAL_STATUS_SUCCESS == am_hal_burst_mode_initialize(&eBurstModeAvailable))
	{
		if (AM_HAL_BURST_AVAIL == eBurstModeAvailable)
		{
			am_util_stdio_printf("Apollo3 Burst Mode is Available\r\n");
		}
		else
		{
			am_util_stdio_printf("Apollo3 Burst Mode is Not Available\r\n");
		}
	}
	else
	{
		am_util_stdio_printf("Failed to Initialize for Burst Mode operation\r\n");
	}

	//
	// Make sure we are in "Normal" mode.
	//
	if (AM_HAL_STATUS_SUCCESS == am_hal_burst_mode_disable(&eBurstMode))
	{
		if (AM_HAL_NORMAL_MODE == eBurstMode)
		{
			am_util_stdio_printf("Apollo3 operating in Normal Mode (48MHz)\r\n");
		}
	}
	else
	{
		am_util_stdio_printf("Failed to Disable Burst Mode operation\r\n");
	}
		
	//
	// Put the MCU into "Burst" mode.
	//
	if (AM_HAL_STATUS_SUCCESS == am_hal_burst_mode_enable(&eBurstMode))
	{
		if (AM_HAL_BURST_MODE == eBurstMode)
		{
			am_util_stdio_printf("Apollo3 operating in Burst Mode (96MHz)\r\n");
		}
	}
#endif

	//
	// Initialize LEDs.
	//
	am_devices_led_array_init(am_bsp_psLEDs, AM_BSP_NUM_LEDS);
	for(int i = 0; i < AM_BSP_NUM_LEDS; i++)
		am_devices_led_off(am_bsp_psLEDs, i);
	
	//
	// Initialize DSpotter SDK.
	//
	DSpotterInit();
	if(!g_hDSpotter)
		goto ERR_MAIN;
	
	//
	// Create sample buffer.
	//
	g_xSampleQueue = xQueueCreate(5, BLOCK_SAMPLE_SIZE << 1);
	if(g_xSampleQueue == NULL)
	{
		am_util_stdio_printf("The queue could not be created\r\n");
		goto ERR_MAIN;
	}

	//
	// Create tasks and run them.
	//
	if(xTaskCreate(Record_Task, ((const char*)"Record_Task"), 512, NULL, tskIDLE_PRIORITY + 1, NULL) != pdTRUE)
		am_util_stdio_printf("xTaskCreate(Record_Task) failed\r\n");
	if(xTaskCreate(Recog_Task, ((const char*)"Recog_Task"), 2048, NULL, tskIDLE_PRIORITY + 2, NULL) != pdTRUE)
		am_util_stdio_printf("xTaskCreate(Recog_Task) failed\r\n");
	if(xTaskCreate(Dump_Task, ((const char*)"Dump_Task"), 256, NULL, tskIDLE_PRIORITY  + 1, NULL) != pdTRUE)
		am_util_stdio_printf("xTaskCreate(Dump_Task) failed\r\n");
	vTaskStartScheduler();
	
ERR_MAIN:
	while(1);
}
