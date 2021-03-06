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
#include "timer.h"
#include "common.h"


// Common interface includes
#include "timer_if.h"
#include "gpio_if.h"///

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     50
#define TRUE 1
#define FALSE 0
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
void frame_sync();
void spi();

extern struct Command {
	unsigned short opcode;
	unsigned short len;
	unsigned short descriptor;
};

unsigned short myStrA[700];
unsigned short myStrB[700];



unsigned int c_full=0;

//unsigned short ping_buf_1[100];

unsigned long ulStatReg;
unsigned int index = 0;
unsigned int flag = 0;
volatile unsigned long chk_bufa_flag=0;
unsigned short check_cmd=11;

unsigned short ulRecvData;
unsigned short recv_buff[10];

unsigned char bufA[4];

int Reset_SYNC;
volatile unsigned int get_sync_cmd_resp=FALSE;
volatile unsigned int cmd_sent=FALSE;
unsigned int flow_ctrl=0;
unsigned int recv_ping_packet=0;
unsigned int recv_pong_packet=0;


unsigned long ulMode;

unsigned long cmd_ack=0;
unsigned short cmd_ackcheck=0;

unsigned short cmd_buffer[5];
volatile unsigned int cmd_index=0;

volatile unsigned long check_frame_start=0;
unsigned long pingpong_setup_t;
unsigned long framesync_t;

static void SlaveIntHandler()
{


	unsigned int index = 0;
	unsigned long ulStatus;
	unsigned long ulMode;



	// Read the interrupt status of the SPI.
	//
	ulStatus = MAP_SPIIntStatus(GSPI_BASE,true);





	MAP_SPIIntClear(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);

	if( (ulStatus & SPI_INT_TX_EMPTY)&&(get_sync_cmd_resp==FALSE) )
	{
		flag++;
		MAP_SPIDataPut(GSPI_BASE,0xFFFF);


	}

	if ( (ulStatus & SPI_INT_TX_EMPTY)&&(get_sync_cmd_resp==TRUE) )
	{
		cmd_index++;
		MAP_SPIDataPut(GSPI_BASE,cmd_buffer[cmd_index]);

		if (cmd_index==6)
		{
			MAP_SPIIntDisable(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);

			cmd_sent=TRUE;
		}
	}

	if(ulStatus & SPI_INT_RX_FULL)
	{
		//  	while(!(HWREG(GSPI_BASE + MCSPI_O_CH0STAT) & MCSPI_CH0STAT_RXS));

		ulRecvData = HWREG(GSPI_BASE + MCSPI_O_RX0);

		if(ulRecvData==0xFFFF)
		{
			get_sync_cmd_resp=TRUE;
			//		 MAP_SPIIntDisable(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);
		}


	}



	//PINGPONG SETUP TO RECEIEVE ADC VALUES

	if (ulStatus & SPI_INT_DMARX)
	{
		MAP_SPIIntClear(GSPI_BASE,SPI_INT_DMARX);

		ulMode = MAP_uDMAChannelModeGet(UDMA_CH30_GSPI_RX | UDMA_PRI_SELECT);


		if(ulMode == UDMA_MODE_STOP)
		{
			recv_ping_packet=1;
			if( (myStrA[0]==0xA5A5) && (myStrA[1]==0xA5A5) )
			{
				check_frame_start++;



			}
			else
			{

				check_frame_start=0;

				//	 frame_sync();//Reset_SYNC=reset_sync_spi();	// //check_frame_start=0;
			}



			SetupTransfer(UDMA_CH30_GSPI_RX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG,
					700,UDMA_SIZE_16, UDMA_ARB_1,
					(void *)(GSPI_BASE + MCSPI_O_RX0), UDMA_SRC_INC_NONE,
					myStrA, UDMA_DST_INC_16);

		}


		ulMode = MAP_uDMAChannelModeGet(UDMA_CH30_GSPI_RX | UDMA_ALT_SELECT);

		if(ulMode == UDMA_MODE_STOP)
		{
			recv_pong_packet=1;

			if( (myStrB[0]==0xA5A5) && (myStrB[1]==0xA5A5) )
			{


				check_frame_start++;

			}
			else
			{

				check_frame_start=0;
			}


			SetupTransfer(UDMA_CH30_GSPI_RX | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG,
					700,UDMA_SIZE_16, UDMA_ARB_1,
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
	//  ms_delay(1000);//1 msec





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
			700,UDMA_SIZE_16, UDMA_ARB_1,
			(void *)(GSPI_BASE + MCSPI_O_RX0), UDMA_SRC_INC_NONE,
			myStrA, UDMA_DST_INC_16);

	SetupTransfer(UDMA_CH30_GSPI_RX | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG,
			700,UDMA_SIZE_16, UDMA_ARB_1,
			(void *)(GSPI_BASE + MCSPI_O_RX0), UDMA_SRC_INC_NONE,
			myStrB, UDMA_DST_INC_16);
	/**
  SetupTransfer(UDMA_CH31_GSPI_TX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG,
              sizeof(myStrX),UDMA_SIZE_16, UDMA_ARB_1,
               myStrX , UDMA_SRC_INC_16,
              (void *)(GSPI_BASE + MCSPI_O_TX0), UDMA_DST_INC_NONE);
  SetupTransfer(UDMA_CH31_GSPI_TX | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG,
              sizeof(myStrY),UDMA_SIZE_16, UDMA_ARB_1,
               myStrY, UDMA_SRC_INC_NONE,
              (void *)(GSPI_BASE + MCSPI_O_TX0), UDMA_DST_INC_NONE);*/


	SPIDmaEnable(GSPI_BASE,SPI_RX_DMA);



	//
	// Enable Interrupts
	MAP_SPIIntDisable(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);
	MAP_SPIIntEnable(GSPI_BASE,SPI_INT_DMARX);

	//
	// Enable SPI for communication
	//
	MAP_SPIEnable(GSPI_BASE);




}


//*****************************************************************************
//send_cmd();
//*****************************************************************************

int send_cmd(unsigned short *opcode)
{

	cmd_sent=FALSE;
	cmd_index=0;

	cmd_buffer[0]=0xABCD;
	cmd_buffer[1]=0x5678;
	cmd_buffer[2]=*opcode;
	cmd_buffer[3]=0x7778;
	cmd_buffer[4]=0x2525;

	/*cmd_buffer[0]=0x3245;
				cmd_buffer[1]=0x8973;
				cmd_buffer[2]=0x6AC7;
				cmd_buffer[3]=0x9342;*/


	MAP_SPIDisable(GSPI_BASE);
	//
	// Reset SPI
	//
	MAP_SPIReset(GSPI_BASE);

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

	while(cmd_sent==FALSE);
	//ulRecvData = HWREG(GSPI_BASE + MCSPI_O_RX0);

	//read spi register value to chek for success signal
	SPIDataGet(GSPI_BASE,&cmd_ack);
	SPIDataGet(GSPI_BASE,&cmd_ack);

	spi();
return (cmd_ack & 0x0000FFFF);


}


//*****************************************************************************
//
//!COmmand MODE SPI
//!
//! \param none
//!
//! \return None.
//
//*****************************************************************************
int reset_sync_spi()
{


	get_sync_cmd_resp=FALSE;

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


	MAP_SPIIntEnable(GSPI_BASE,SPI_INT_TX_EMPTY|SPI_INT_RX_FULL);


	//
	// Enable SPI for communication
	//
	MAP_SPIEnable(GSPI_BASE);

	//  while(1);
	while(get_sync_cmd_resp==FALSE);
	return 1;


}
//*****************************************************************************
//
//call this function when wrong frame receieved
//
//*****************************************************************************
void frame_sync()
{	 chk_bufa_flag++;

//Disable and clear All SPI interrupts
MAP_SPIIntDisable(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY|SPI_INT_DMATX|SPI_INT_DMARX);
MAP_SPIIntClear(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY|SPI_INT_DMATX|SPI_INT_DMARX);

get_sync_cmd_resp=FALSE;


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


MAP_SPIIntEnable(GSPI_BASE,SPI_INT_TX_EMPTY|SPI_INT_RX_FULL);


//
// Enable SPI for communication
//
MAP_SPIEnable(GSPI_BASE);

//  while(1);
while(get_sync_cmd_resp==FALSE);

spi();
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
