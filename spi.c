

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




#define SPI_IF_BIT_RATE  24000000
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


 //*********************5*******************************************************

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

unsigned short tx_dummy_strA[50];
unsigned short tx_dummy_strB[50];

unsigned long ulStatReg;
unsigned int index = 0;
unsigned int flag = 0;
unsigned short check_cmd=11;

unsigned char bufA[4];
unsigned char ulDummy[5];
unsigned char ulSend[5];


unsigned long ulStatus;
		    unsigned long ulMode;


static void SlaveIntHandler()
{

	unsigned int index = 0;


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
		 //   MAP_SPIIntClear(GSPI_BASE,SPI_INT_TX_EMPTY);

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

		    	if (myStrA[0]==11)
	    	{flag++;

	    	 MAP_SPIDataPut(GSPI_BASE,'AB');


	    		 //   	 MAP_SPIDataPut(GSPI_BASE,'CD');


	    	}

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
		    	              sizeof(myStrA),UDMA_SIZE_16, UDMA_ARB_1,
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
		    //	 MAP_SPIDataPut(GSPI_BASE,'B');

		    	//
		        // Set up the next transfer for the "B" buffer, using the alternate
		        // control structure.  When the ongoing receive into the "A" buffer is
		        // done, the uDMA controller will switch back to this one.
		        //
		   	  SetupTransfer(UDMA_CH30_GSPI_RX | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG,
		  		    	              sizeof(myStrB),UDMA_SIZE_16, UDMA_ARB_1,
		  		    	              (void *)(GSPI_BASE + MCSPI_O_RX0), UDMA_SRC_INC_NONE,
		  		    	              myStrB, UDMA_DST_INC_16);


		    }

}
//*********************5*******************************************************

static void MasterIntHandler()
		    {

		    	unsigned int index = 0;


		    		    //
		    		    // Read the interrupt status of the SPI.
		    		    //
		    		    ulStatus = MAP_SPIIntStatus(GSPI_BASE,true);


		    		   	 MAP_SPIIntClear(GSPI_BASE,SPI_INT_DMARX|SPI_INT_DMATX);

if (ulStatus & SPI_INT_DMATX)
{
		    		    ulMode = MAP_uDMAChannelModeGet(UDMA_CH31_GSPI_TX | UDMA_PRI_SELECT);

		    		    if(ulMode == UDMA_MODE_STOP)
		    		    {



		    		        SetupTransfer(UDMA_CH31_GSPI_TX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG,
		    		                      sizeof(tx_dummy_strA),UDMA_SIZE_16, UDMA_ARB_1,
		    		                      tx_dummy_strA, UDMA_SRC_INC_16,
		    		                      (void *)(GSPI_BASE + MCSPI_O_TX0), UDMA_DST_INC_NONE);

		    		    }


		    		    ulMode = MAP_uDMAChannelModeGet(UDMA_CH31_GSPI_TX | UDMA_ALT_SELECT);

		    		    if(ulMode == UDMA_MODE_STOP)
		    		    {


		    		    	 SetupTransfer(UDMA_CH31_GSPI_TX | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG,
		    		    			    sizeof(tx_dummy_strB),UDMA_SIZE_16, UDMA_ARB_1,
		    		    			    tx_dummy_strB, UDMA_SRC_INC_16,
		    		    			    (void *)(GSPI_BASE + MCSPI_O_TX0), UDMA_DST_INC_NONE);


		    		    }
}

if (ulStatus & SPI_INT_DMARX)
{
		    		    ulMode = MAP_uDMAChannelModeGet(UDMA_CH30_GSPI_RX | UDMA_PRI_SELECT);


		    		   		    if(ulMode == UDMA_MODE_STOP)
		    		   		    {



		    		   		    	for (index=0;index<50;index++)
		    		   		    			    			    	{ myStrC[index]=myStrA[index];

		    		   		    			    			    	}


		    		   		    	  SetupTransfer(UDMA_CH30_GSPI_RX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG,
		    		   		    	              sizeof(myStrA),UDMA_SIZE_16, UDMA_ARB_1,
		    		   		    	              (void *)(GSPI_BASE + MCSPI_O_RX0), UDMA_SRC_INC_NONE,
		    		   		    	              myStrA, UDMA_DST_INC_16);

		    		   		    }


		    		   		    ulMode = MAP_uDMAChannelModeGet(UDMA_CH30_GSPI_RX | UDMA_ALT_SELECT);

		    		   		    if(ulMode == UDMA_MODE_STOP)
		    		   		    {

		    		   		    	for (index=0;index<50;index++)
		    		   		    			    	{ myStrC[index]=myStrB[index];

		    		   		    			    	}

		    		   		   	  SetupTransfer(UDMA_CH30_GSPI_RX | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG,
		    		   		  		    	              sizeof(myStrB),UDMA_SIZE_16, UDMA_ARB_1,
		    		   		  		    	              (void *)(GSPI_BASE + MCSPI_O_RX0), UDMA_SRC_INC_NONE,
		    		   		  		    	              myStrB, UDMA_DST_INC_16);


		    		   		    }





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
                     SPI_CS_ACTIVELOW |
                     SPI_WL_16));

  //
  // Register Interrupt Handler
  //
 MAP_SPIIntRegister(GSPI_BASE,SlaveIntHandler);



  SetupTransfer(UDMA_CH30_GSPI_RX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG,
              sizeof(myStrA),UDMA_SIZE_16, UDMA_ARB_1,
              (void *)(GSPI_BASE + MCSPI_O_RX0), UDMA_SRC_INC_NONE,
              myStrA, UDMA_DST_INC_16);

  SetupTransfer(UDMA_CH30_GSPI_RX | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG,
              sizeof(myStrB),UDMA_SIZE_16, UDMA_ARB_1,
              (void *)(GSPI_BASE + MCSPI_O_RX0), UDMA_SRC_INC_NONE,
              myStrB, UDMA_DST_INC_16);




  SPIDmaEnable(GSPI_BASE,SPI_RX_DMA);



  //
  // Enable Interrupts
  //
  MAP_SPIIntEnable(GSPI_BASE,SPI_INT_DMARX);
 // MAP_SPIIntEnable(GSPI_BASE,SPI_INT_RX_FULL);
  //
  // Enable SPI for communication
  //
  MAP_SPIEnable(GSPI_BASE);


}
//*****************************************************************************
//
//! SPI Master mode main loop
//!
//! This function configures SPI modelue as master and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************

void MasterMain()
{
	// unsigned long ulUserData;
unsigned int i=0;
unsigned int j=0;
unsigned long recv=0;
//


for ( j=0;j<50;j++)
{
tx_dummy_strA[j]=0xFFFF;
}

for ( j=0;j<50;j++)
{
tx_dummy_strB[j]=0xFFFF;

}


    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
    		SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF|
                     SPI_CS_ACTIVELOW |
                     SPI_WL_16));

    //
      // Register Interrupt Handler
      //

     MAP_SPIIntRegister(GSPI_BASE,MasterIntHandler);

     SetupTransfer(UDMA_CH30_GSPI_RX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG,
                 sizeof(myStrA),UDMA_SIZE_16, UDMA_ARB_1,
                 (void *)(GSPI_BASE + MCSPI_O_RX0), UDMA_SRC_INC_NONE,
                 myStrA, UDMA_DST_INC_16);

     SetupTransfer(UDMA_CH30_GSPI_RX | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG,
                 sizeof(myStrB),UDMA_SIZE_16, UDMA_ARB_1,
                 (void *)(GSPI_BASE + MCSPI_O_RX0), UDMA_SRC_INC_NONE,
                 myStrB, UDMA_DST_INC_16);


     SetupTransfer(UDMA_CH31_GSPI_TX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG,
                 sizeof(tx_dummy_strA),UDMA_SIZE_16, UDMA_ARB_1,
                 tx_dummy_strA, UDMA_SRC_INC_16,
                 (void *)(GSPI_BASE + MCSPI_O_TX0), UDMA_DST_INC_NONE);

     SetupTransfer(UDMA_CH31_GSPI_TX | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG,
   		  	  sizeof(tx_dummy_strB),UDMA_SIZE_16, UDMA_ARB_1,
   		       tx_dummy_strB, UDMA_SRC_INC_16,
   		      (void *)(GSPI_BASE + MCSPI_O_TX0), UDMA_DST_INC_NONE);



      SPIDmaEnable(GSPI_BASE,SPI_RX_DMA|SPI_TX_DMA);



      //
      // Enable Interrupts
      //
      MAP_SPIIntEnable(GSPI_BASE,SPI_INT_DMARX|SPI_INT_DMATX);




    // Enable SPI for communication
            //
            MAP_SPIEnable(GSPI_BASE);

           MAP_SPICSEnable(GSPI_BASE);
/*
    //
    // Send the string to slave. Chip Select(CS) needs to be
    // asserted at start of transfer and deasserted at the end.
    //
    ulSend[0]=0xA5;
    ulSend[1]=0xF5;

    MAP_SPITransfer(GSPI_BASE,&ulSend[0],&ulDummy[0],1,SPI_CS_ENABLE|SPI_CS_DISABLE);
//    MAP_SPITransfer(GSPI_BASE,&ulSend[1],&ulDummy[1],1,SPI_CS_ENABLE|SPI_CS_DISABLE);

 */
/*

    ulDummy[0]=0;
    //
    // Enable Chip select
    //
  MAP_SPICSEnable(GSPI_BASE);

        // Push the character over SPI
        //
        MAP_SPIDataPut(GSPI_BASE,0x50);
     MAP_SPIDataGet(GSPI_BASE,&ulDummy[0]);

       MAP_SPICSDisable(GSPI_BASE);

       MAP_SPIDisable(GSPI_BASE);
*/

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
  // Enable the SPI module clock
  //
  MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);



  //
  // Reset the peripheral
  //
  MAP_PRCMPeripheralReset(PRCM_GSPI);

//  SlaveMain();


 MasterMain();



}

