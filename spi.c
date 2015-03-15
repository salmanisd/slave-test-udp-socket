
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

unsigned short myStrX[50];
unsigned short myStrY[50];

unsigned long ulStatReg;
unsigned int index = 0;
unsigned int flag = 0;
unsigned short check_cmd=11;

unsigned short ulRecvData;
unsigned short recv_buff[10];

unsigned char bufA[4];

  volatile signed int h=-1;
		    unsigned long ulMode;


		unsigned short response[2];




static void SlaveIntHandler()
				    {


	unsigned int index = 0;
	unsigned long ulStatus;



				    		    // Read the interrupt status of the SPI.
				    		    //
				    		    ulStatus = MAP_SPIIntStatus(GSPI_BASE,true);





				    		    					        MAP_SPIIntClear(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);

				    		    					        if(ulStatus & SPI_INT_TX_EMPTY)
				    		    					        {
				    		    					        	flag++;
				    		    					        	 MAP_SPIDataPutNonBlocking(GSPI_BASE,0xABCD);

				    		    					        }

				    		    					        if(ulStatus & SPI_INT_RX_FULL)
				    		    					        {

				    		    					       // 	while(!(HWREG(GSPI_BASE + MCSPI_O_CH0STAT) & MCSPI_CH0STAT_RXS));

				    		    					        	ulRecvData = HWREG(GSPI_BASE + MCSPI_O_RX0);

				    		    					        	if(ulRecvData==0xFFFF)
				    		    					        	{
				    		    					        		h=1;
				    		    					        		MAP_SPIIntDisable(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);
				    		    					        	}


				    		    					        }

		if (ulStatus & SPI_INT_DMATX)
		{

			MAP_SPIIntClear(GSPI_BASE,SPI_INT_DMATX);


				    		    ulMode = MAP_uDMAChannelModeGet(UDMA_CH31_GSPI_TX | UDMA_PRI_SELECT);

				    		    if(ulMode == UDMA_MODE_STOP)
				    		    {



				    		        SetupTransfer(UDMA_CH31_GSPI_TX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG,
				    		                      sizeof(myStrX),UDMA_SIZE_16, UDMA_ARB_1,
				    		                      myStrX, UDMA_SRC_INC_16,
				    		                      (void *)(GSPI_BASE + MCSPI_O_TX0), UDMA_DST_INC_NONE);

				    		    }


				    		    ulMode = MAP_uDMAChannelModeGet(UDMA_CH31_GSPI_TX | UDMA_ALT_SELECT);

				    		    if(ulMode == UDMA_MODE_STOP)
				    		    {


				    		    	 SetupTransfer(UDMA_CH31_GSPI_TX | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG,
				    		    			    sizeof(myStrY),UDMA_SIZE_16, UDMA_ARB_1,
				    		    			    myStrY, UDMA_SRC_INC_16,
				    		    			    (void *)(GSPI_BASE + MCSPI_O_TX0), UDMA_DST_INC_NONE);


				    		    }
		}

		if (ulStatus & SPI_INT_DMARX)
		{

		   	 MAP_SPIIntClear(GSPI_BASE,SPI_INT_DMARX);

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

	int j=0;

for(j=0;j<50;j++)
{
	myStrX[j]=0xFFFF;
}

for(j=0;j<50;j++)
{
	myStrY[j]=0xFFFF;
}

MAP_SPIDisable(GSPI_BASE);
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
/*
  SetupTransfer(UDMA_CH31_GSPI_TX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG,
              sizeof(myStrX),UDMA_SIZE_16, UDMA_ARB_1,
               myStrX , UDMA_SRC_INC_16,
              (void *)(GSPI_BASE + MCSPI_O_TX0), UDMA_DST_INC_NONE);

  SetupTransfer(UDMA_CH31_GSPI_TX | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG,
              sizeof(myStrY),UDMA_SIZE_16, UDMA_ARB_1,
               myStrY, UDMA_SRC_INC_NONE,
              (void *)(GSPI_BASE + MCSPI_O_TX0), UDMA_DST_INC_NONE);
*/

  SPIDmaEnable(GSPI_BASE,SPI_RX_DMA);



  //
  // Enable Interrupts
   MAP_SPIIntDisable(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);
  MAP_SPIIntEnable(GSPI_BASE,SPI_INT_DMARX);
 // MAP_SPIIntEnable(GSPI_BASE,SPI_INT_RX_FULL);
  //
  // Enable SPI for communication
  //
  MAP_SPIEnable(GSPI_BASE);

}


//*****************************************************************************
static void SyncIntHandler()
{
}

//*****************************************************************************
//*****************************************************************************
//
//!COmmand MODE SPI
//!
//! \param none
//!
//! \return None.
//
//*****************************************************************************
int sync_spi()
{
	signed int j=-1;
	// Enable the SPI module clock
		  //
		  MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);



		  //
		  // Reset the peripheral
		  //
		  MAP_PRCMPeripheralReset(PRCM_GSPI);

    MAP_SPIReset(GSPI_BASE);
    MAP_SPIDisable(GSPI_BASE);
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




	  MAP_SPIIntRegister(GSPI_BASE,SlaveIntHandler);

	      //
	      // Enable Interrupts
	      //
	      MAP_SPIIntEnable(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);

	      //
	      // Enable SPI for communication
	      //
	      MAP_SPIEnable(GSPI_BASE);

while(j==-1)
{
	j=h;
}
return 1;
	      /*
MAP_SPIEnable(GSPI_BASE);
do {

	  MAP_SPIDataPut(GSPI_BASE,0xABCD);
		  while(!(HWREG(GSPI_BASE + MCSPI_O_CH0STAT) & MCSPI_CH0STAT_RXS))
		   {
		   }
	//	  MAP_SPIDataGet(GSPI_BASE,&response[1]);
		  while(!(HWREG(GSPI_BASE + MCSPI_O_CH0STAT) & MCSPI_CH0STAT_RXS))
		   {
		   }

		   //
		   // Read the value
		   //
		   response[1] = HWREG(GSPI_BASE + MCSPI_O_RX0);

		//   re=response[1];
if(response[1]=='YZ')
{ flag++;}

 }while(response[1]!='YZ');



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


  SlaveMain();




}
