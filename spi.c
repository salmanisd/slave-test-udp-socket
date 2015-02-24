

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
//*********************5*******************************************************
unsigned short myStrA[50];
unsigned short myStrB[50];
unsigned short myStrC[50];

unsigned int index = 0;
unsigned int index2 = 0;
unsigned int check=0;
unsigned long ulStatus;
		    unsigned long ulMode;
static void SlaveIntHandler()
{


	unsigned int index = 0;


		  //  check++;
		    //
		    // Read the interrupt status of the SPI.
		    //
		    ulStatus = MAP_SPIIntStatus(GSPI_BASE,true);
		    //
		    // Clear any pending status, even though there should be none since no SPI
		    // interrupts were enabled.
		    //
		    //	MAP_SPIIntClear(GSPI_BASE,ulStatus);
		   		//    MAP_SPIIntClear(GSPI_BASE,SPI_INT_DMARX|SPI_INT_DMATX);
		   	    MAP_SPIIntClear(GSPI_BASE,SPI_INT_DMARX);
		   		//    MAP_SPIIntClear(GSPI_BASE,SPI_INT_DMATX);
			   	//	  MAP_SPIIntDisable(GSPI_BASE,SPI_INT_DMARX);

		  // 		  MAP_SPIIntEnable(GSPI_BASE,SPI_INT_DMARX);


		    //
		    // Check the DMA control table to see if the ping-pong "A" transfer is
		    // complete.  The "A" transfer uses receive buffer "A", and the primary
		    // control structure.
		    //
		    ulMode = MAP_uDMAChannelModeGet(UDMA_CH30_GSPI_RX | UDMA_PRI_SELECT);

		    //
		    // If the primary control structure indicates stop, that means the "A"
		    // receive buffer is done.  The uDMA controller should still be receiving
		    // data into the "B" buffer.
		    //
		    if(ulMode == UDMA_MODE_STOP)
		    {

		    	for (index=0;index<50;index++)
		    	{ myStrC[index]=myStrA[index];

		    	}
		    	// BsdUdpClient(5001);
		        //
		        // Set up the next transfer for the "A" buffer, using the primary
		        // control structure.  When the ongoing receive into the "B" buffer is
		        // done, the uDMA controller will switch back to this one.
		        //
		    	  SetupTransfer(UDMA_CH30_GSPI_RX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG,
		    	              sizeof(myStrA),UDMA_SIZE_16, UDMA_ARB_4,
		    	              (void *)(GSPI_BASE + MCSPI_O_RX0), UDMA_SRC_INC_NONE,
		    	              myStrA, UDMA_DST_INC_16);

		    }

		    //
		    // Check the DMA control table to see if the ping-pong "B" transfer is
		    // complete.  The "B" transfer uses receive buffer "B", and the alternate
		    // control structure.
		    //
		    ulMode = MAP_uDMAChannelModeGet(UDMA_CH30_GSPI_RX | UDMA_ALT_SELECT);

		    //
		    // If the alternate control structure indicates stop, that means the "B"
		    // receive buffer is done.  The uDMA controller should still be receiving
		    // data into the "A" buffer.
		    //
		    if(ulMode == UDMA_MODE_STOP)
		    {
		    	for (index=0;index<50;index++)
		    			    	{ myStrC[index]=myStrB[index];

		    			    	}
		        //
		        // Set up the next transfer for the "B" buffer, using the alternate
		        // control structure.  When the ongoing receive into the "A" buffer is
		        // done, the uDMA controller will switch back to this one.
		        //
		   	  SetupTransfer(UDMA_CH30_GSPI_RX | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG,
		  		    	              sizeof(myStrB),UDMA_SIZE_16, UDMA_ARB_4,
		  		    	              (void *)(GSPI_BASE + MCSPI_O_RX0), UDMA_SRC_INC_NONE,
		  		    	              myStrB, UDMA_DST_INC_16);


		    }

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


  //
  // Reset SPI
  //
  MAP_SPIReset(GSPI_BASE);

  //
  // Configure SPI interface
  //
  MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_SLAVE,SPI_SUB_MODE_0,
                     (SPI_HW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_16));

  //
  // Register Interrupt Handler
  //
 MAP_SPIIntRegister(GSPI_BASE,SlaveIntHandler);//Disable SPI Interrupts


  SetupTransfer(UDMA_CH30_GSPI_RX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG,
              sizeof(myStrA),UDMA_SIZE_16, UDMA_ARB_4,
              (void *)(GSPI_BASE + MCSPI_O_RX0), UDMA_SRC_INC_NONE,
              myStrA, UDMA_DST_INC_16);

  SetupTransfer(UDMA_CH30_GSPI_RX | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG,
              sizeof(myStrB),UDMA_SIZE_16, UDMA_ARB_4,
              (void *)(GSPI_BASE + MCSPI_O_RX0), UDMA_SRC_INC_NONE,
              myStrB, UDMA_DST_INC_16);




  SPIDmaEnable(GSPI_BASE,SPI_RX_DMA);


  //
  // Enable Interrupts
  //
  MAP_SPIIntEnable(GSPI_BASE,SPI_INT_DMARX);

  //
  // Enable SPI for communication
  //
  MAP_SPIEnable(GSPI_BASE);


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
	//UART_PRINT("Device started as STATION \n\r");
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




}

