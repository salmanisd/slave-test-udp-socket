//*****************************************************************************
// main.c
//
// The demo application focuses on showing the required initialization
// sequence to enable the CC3200 SPI module in full duplex 4-wire master
// and slave mode(s).
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

#include <string.h>
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_mcspi.h"
#include "hw_ints.h"
#include "spi.h"
#include "pin.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "interrupt.h"
#include "pinmux.h"
#include "uart.h"
#include "udma.h"
#include "uart_if.h"
#include "udma_if.h"




#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     50

//testing first commit
//*****************************************************************************
// Global variables
//*****************************************************************************
static unsigned char g_ucTxBuff[TR_BUFF_SIZE];
static unsigned char g_ucRxBuff[TR_BUFF_SIZE];
static unsigned char ucTxBuffNdx;
static unsigned char ucRxBuffNdx;

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif

#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//*****************************************************************************
//
//! SPI Slave Interrupt handler
//!
//! This function is invoked when SPI slave has its receive register full or
//! transmit register empty.
//!
//! \return None.
//
//*****************************************************************************
unsigned char myStr[1024]; unsigned int gi = 0;
static void SlaveIntHandler()
{
  unsigned long ulRecvData;
  unsigned long ulStatus;

  ulStatus = MAP_SPIIntStatus(GSPI_BASE,true);

  MAP_SPIIntClear(GSPI_BASE,SPI_INT_DMARX|SPI_INT_DMATX);

//#if 1
  if(ulStatus & SPI_INT_TX_EMPTY)
  {
    MAP_SPIDataPut(GSPI_BASE,g_ucTxBuff[ucTxBuffNdx%TR_BUFF_SIZE]);
    ucTxBuffNdx++;
  }

  if(ulStatus & SPI_INT_RX_FULL)
  {
    MAP_SPIDataGetNonBlocking(GSPI_BASE,&ulRecvData);
    g_ucTxBuff[ucRxBuffNdx%TR_BUFF_SIZE] = ulRecvData;
   // myStr[gi++]=(char)ulRecvData;
    //   Report("%c",ulRecvData);
    ucRxBuffNdx++;
  }
//#endif
}

//*****************************************************************************
//
//! SPI Slave mode main loop
//!
//! This function configures SPI modelue as slave and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************
void SlaveMain()
{
  unsigned long ulNdx;

  //
  // Initialize the message
  //
  for(ulNdx=0; ulNdx < TR_BUFF_SIZE; ulNdx++)
  {
    g_ucTxBuff[ulNdx] = ulNdx;
  }

  //
  // Set Tx buffer index
  //
  ucTxBuffNdx = 0;
  ucRxBuffNdx = 0;

  //
  // Reset SPI
  //
  MAP_SPIReset(GSPI_BASE);

  //
  // Initialize UDMA
  //
  UDMAInit();

  //
  // Configure SPI interface
  //
  MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_SLAVE,SPI_SUB_MODE_0,
                     (SPI_HW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVELOW |
                     SPI_WL_8));

  //
  // Register Interrupt Handler
  //
  MAP_SPIIntRegister(GSPI_BASE,SlaveIntHandler);

  SetupTransfer(UDMA_CH30_GSPI_RX,UDMA_MODE_BASIC,1000,
                UDMA_SIZE_8,UDMA_ARB_1,
                (void *)(GSPI_BASE + MCSPI_O_RX0),UDMA_SRC_INC_NONE,
                &myStr,UDMA_DST_INC_8);

  SetupTransfer(UDMA_CH31_GSPI_TX,UDMA_MODE_BASIC,TR_BUFF_SIZE,
                UDMA_SIZE_8,UDMA_ARB_1,
                &g_ucTxBuff,UDMA_SRC_INC_8,(void *)(GSPI_BASE + MCSPI_O_TX0),
                UDMA_DST_INC_NONE);

#if 0
  SPIWordCountSet(GSPI_BASE,TR_BUFF_SIZE);

  SPIFIFOLevelSet(GSPI_BASE,1,1);


  SPIFIFOEnable(GSPI_BASE, SPI_RX_FIFO|SPI_TX_FIFO);
#endif

  SPIDmaEnable(GSPI_BASE,SPI_RX_DMA|SPI_TX_DMA);


  //
  // Enable Interrupts
  //
  MAP_SPIIntEnable(GSPI_BASE,SPI_INT_DMATX|SPI_INT_DMARX);

  //
  // Enable SPI for communication
  //
  MAP_SPIEnable(GSPI_BASE);


}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
  //
  // Enable Processor
  //
  MAP_IntMasterEnable();
  MAP_IntEnable(FAULT_SYSTICK);

  PRCMCC3200MCUInit();
}

//*****************************************************************************
//
//! Main function for spi demo application
//!
//! \param none
//!
//! \return None.
//
//*****************************************************************************
void spi()
{

  //
  // Initialize Board configurations
  //
//BoardInit();

  //
  // Muxing  SPI lines.
  //
//  PinMuxConfig();

  //
  // Enable the SPI module clock
  //
  MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);



  //
  // Reset the peripheral
  //
  MAP_PRCMPeripheralReset(PRCM_GSPI);



  SlaveMain();



  while(1)
  {

  }

}

