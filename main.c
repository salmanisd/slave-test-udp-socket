#include <stdlib.h>
#include <string.h>

// simplelink includes
#include "simplelink.h"
#include "wlan.h"

// driverlib includes
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"
#include "timer.h"
#include "timer_if.h"

// common interface includes
#include "udma_if.h"
#include "common.h"
#ifndef NOTERM
#include "uart_if.h"
#endif

#include "pinmux.h"
#include "spi.h"


#define APPLICATION_NAME        "UDP Socket"
#define APPLICATION_VERSION     "1.1.0"

#define IP_ADDR      0xC0A8AD01//   0xC0A8AD01//192.168.173.1			  // 0xc0a8006E /* 192.168.0.110 */

#define PORT_NUM           5001
#define BUF_SIZE           1400
#define UDP_PACKET_COUNT   1000

// Application specific status/error codes
typedef enum{
	// Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
	UCP_CLIENT_FAILED = -0x7D0,
	UCP_SERVER_FAILED = UCP_CLIENT_FAILED - 1,
	DEVICE_NOT_IN_STATION_MODE = UCP_SERVER_FAILED - 1,

	STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;


extern struct Command {
	unsigned short opcode;
	unsigned short len;
	unsigned short descriptor;
};



//****************************************************************************
//                      Receiver Wireless Settings
//SSID Name= forticc
//password=icecream
//Connected to Wireless Network Connection 2 = 192.168.137.1
//****************************************************************************

//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
int BsdUdpClient(unsigned short usPort);
int BsdUdpServer(unsigned short usPort);
int UdpServer(unsigned short usPort,unsigned short destPort);
static long WlanConnect();
//static void DisplayBanner();
static void BoardInit();
static void InitializeAppVariables();
static long ConfigureSimpleLinkToDefaultState();
void spi();
int reset_sync_spi();
void ms_delay(int ms);
//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
unsigned long  g_ulDestinationIp = IP_ADDR;        // Client IP address
unsigned int   g_uiPortNum = PORT_NUM;
unsigned long  g_ulPacketCount = UDP_PACKET_COUNT;
unsigned char  g_ucSimplelinkstarted = 0;
unsigned long  g_ulIpAddr = 0;
unsigned short hh;

unsigned long recvfromflag;
unsigned long udploop;


extern int Reset_SYNC;
char g_cBsdBuf[BUF_SIZE];char udp_str[]="Testing UDP";unsigned int num=9578;unsigned char myStr2[];unsigned short shadowstr;

extern unsigned short myStrA[700];				//Holds string from master over SPI
extern unsigned short myStrB[700];				//Holds string from master over SPI
static unsigned short myStrC[700];
static unsigned short myStrD[700];


extern unsigned int recv_ping_packet;
extern unsigned int recv_pong_packet;

extern unsigned int check_frame_start;
unsigned int spi_ret;
extern unsigned int c_full;

unsigned short cmd_code;

unsigned short rx_udp_server[2];
#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void ms_delay(int ms) {
	while (ms-- > 0) {
		volatile int x=5971;
		while (x-- > 0)
			__asm("nop");
	}
}

void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
	if(!pWlanEvent)
	{
		return;
	}

	switch(pWlanEvent->Event)
	{
	case SL_WLAN_CONNECT_EVENT:
	{
		SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

		//
		// Information about the connected AP (like name, MAC etc) will be
		// available in 'slWlanConnectAsyncResponse_t'-Applications
		// can use it if required
		//
		//  slWlanConnectAsyncResponse_t *pEventData = NULL;
		// pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
		//

		// Copy new connection SSID and BSSID to global parameters
		memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
				STAandP2PModeWlanConnected.ssid_name,
				pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
		memcpy(g_ucConnectionBSSID,
				pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
				SL_BSSID_LENGTH);

		UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s , "
				"BSSID: %x:%x:%x:%x:%x:%x\n\r",
				g_ucConnectionSSID,g_ucConnectionBSSID[0],
				g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
				g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
				g_ucConnectionBSSID[5]);
	}
	break;

	case SL_WLAN_DISCONNECT_EVENT:
	{
		slWlanConnectAsyncResponse_t*  pEventData = NULL;

		CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
		CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

		pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

		// If the user has initiated 'Disconnect' request,
		//'reason_code' is SL_USER_INITIATED_DISCONNECTION
		if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
		{
			UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
					"BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
					g_ucConnectionSSID,g_ucConnectionBSSID[0],
					g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
					g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
					g_ucConnectionBSSID[5]);
		}
		else
		{
			UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s,"
					"BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
					g_ucConnectionSSID,g_ucConnectionBSSID[0],
					g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
					g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
					g_ucConnectionBSSID[5]);
		}
		memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
		memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
	}
	break;

	default:
	{
		UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
				pWlanEvent->Event);
	}
	break;
	}
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
	if(!pNetAppEvent)
	{
		return;
	}

	switch(pNetAppEvent->Event)
	{
	case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
	{
		SlIpV4AcquiredAsync_t *pEventData = NULL;

		SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

		//Ip Acquired Event Data
		pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

		g_ulIpAddr = pEventData->ip;

		//Gateway IP address
		g_ulGatewayIP = pEventData->gateway;

		UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
				"Gateway=%d.%d.%d.%d\n\r",

				SL_IPV4_BYTE(g_ulIpAddr,3),
				SL_IPV4_BYTE(g_ulIpAddr,2),
				SL_IPV4_BYTE(g_ulIpAddr,1),
				SL_IPV4_BYTE(g_ulIpAddr,0),
				SL_IPV4_BYTE(g_ulGatewayIP,3),
				SL_IPV4_BYTE(g_ulGatewayIP,2),
				SL_IPV4_BYTE(g_ulGatewayIP,1),
				SL_IPV4_BYTE(g_ulGatewayIP,0));
	}
	break;

	default:
	{
		UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
				pNetAppEvent->Event);
	}
	break;
	}
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
	if(!pDevEvent)
	{
		return;
	}

	//
	// Most of the general errors are not FATAL are are to be handled
	// appropriately by the application
	//
	UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
			pDevEvent->EventData.deviceEvent.status,
			pDevEvent->EventData.deviceEvent.sender);
}

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************


//*****************************************************************************
static void InitializeAppVariables()
{
	g_ulStatus = 0;
	g_ulGatewayIP = 0;
	memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
	memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
	g_ulDestinationIp = 0xC0A88901; //1921681371
	g_uiPortNum = PORT_NUM;
	g_ulPacketCount = UDP_PACKET_COUNT;
}

void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
		SlHttpServerResponse_t *pHttpResponse)
{
	// Unused in this application
}

void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
	//
	// This application doesn't work w/ socket - Events are not expected
	//

}

//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - Unregister mDNS services
//!           - Remove all filters
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
static long ConfigureSimpleLinkToDefaultState()
{
	SlVersionFull   ver = {0};
	_WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

	unsigned char ucVal = 1;
	unsigned char ucConfigOpt = 0;
	unsigned char ucConfigLen = 0;
	unsigned char ucPower = 0;

	long lRetVal = -1;
	long lMode = -1;

	lMode = sl_Start(0, 0, 0);
	ASSERT_ON_ERROR(lMode);

	// If the device is not in station-mode, try configuring it in station-mode
	if (ROLE_STA != lMode)
	{
		if (ROLE_AP == lMode)
		{
			// If the device is in AP mode, we need to wait for this event
			// before doing anything
			while(!IS_IP_ACQUIRED(g_ulStatus))
			{
#ifndef SL_PLATFORM_MULTI_THREADED
				_SlNonOsMainLoopTask();
#endif
			}
		}

		// Switch to STA role and restart
		lRetVal = sl_WlanSetMode(ROLE_STA);
		ASSERT_ON_ERROR(lRetVal);

		lRetVal = sl_Stop(0xFF);
		ASSERT_ON_ERROR(lRetVal);

		lRetVal = sl_Start(0, 0, 0);
		ASSERT_ON_ERROR(lRetVal);

		// Check if the device is in station again
		if (ROLE_STA != lRetVal)
		{
			// We don't want to proceed if the device is not coming up in STA-mode
			return DEVICE_NOT_IN_STATION_MODE;
		}
	}

	// Get the device's version-information
	ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
	ucConfigLen = sizeof(ver);
	lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt,
			&ucConfigLen, (unsigned char *)(&ver));
	ASSERT_ON_ERROR(lRetVal);

	UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
	UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
			ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
			ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
			ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
			ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
			ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

	// Set connection policy to Auto + SmartConfig
	//      (Device's default connection policy)
	lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION,
			SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
	ASSERT_ON_ERROR(lRetVal);

	// Remove all profiles
	lRetVal = sl_WlanProfileDel(0xFF);
	ASSERT_ON_ERROR(lRetVal);



	//
	// Device in station-mode. Disconnect previous connection if any
	// The function returns 0 if 'Disconnected done', negative number if already
	// disconnected Wait for 'disconnection' event if 0 is returned, Ignore
	// other return-codes
	//
	lRetVal = sl_WlanDisconnect();
	if(0 == lRetVal)
	{
		// Wait
		while(IS_CONNECTED(g_ulStatus))
		{
#ifndef SL_PLATFORM_MULTI_THREADED
			_SlNonOsMainLoopTask();
#endif
		}
	}

	//     Enable DHCP client
	   lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

	/*SlNetCfgIpV4Args_t ipV4;
	ipV4.ipV4 = (unsigned long)SL_IPV4_VAL(192,168,173,54); // unsigned long IP  address
	ipV4.ipV4Mask = (unsigned long)SL_IPV4_VAL(255,255,255,0); // unsigned long Subnet mask for this AP/P2P
	ipV4.ipV4Gateway = (unsigned long)SL_IPV4_VAL(192,168,173,0); // unsigned long Default gateway address
	ipV4.ipV4DnsServer = (unsigned long)SL_IPV4_VAL(160,85,193,100); // unsigned long DNS server address
	lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_STATIC_ENABLE, IPCONFIG_MODE_ENABLE_IPV4, sizeof(SlNetCfgIpV4Args_t), (unsigned char *) &ipV4);
	sl_Stop(0);
	sl_Start(NULL,NULL,NULL);
	ASSERT_ON_ERROR(lRetVal);*/

	// Disable scan
	ucConfigOpt = SL_SCAN_POLICY(0);
	lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
	ASSERT_ON_ERROR(lRetVal);

	// Set Tx power level for station mode
	// Number between 0-15, as dB offset from max power - 0 will set max power
	ucPower = 0;
	lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
			WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
	ASSERT_ON_ERROR(lRetVal);

	// Set PM policy to normal
	lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
	ASSERT_ON_ERROR(lRetVal);

	// Unregister mDNS services
	lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
	ASSERT_ON_ERROR(lRetVal);

	// Remove  all 64 filters (8*8)
	memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
	lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
			sizeof(_WlanRxFilterOperationCommandBuff_t));
	ASSERT_ON_ERROR(lRetVal);

	lRetVal = sl_Stop(SL_STOP_TIMEOUT);
	ASSERT_ON_ERROR(lRetVal);

	InitializeAppVariables();

	return lRetVal; // Success
}

//****************************************************************************
//
//
//****************************************************************************

unsigned int timerValue = 0;
unsigned int tempCnt = 0;
int UdpServer(unsigned short serverPort,unsigned short destPort)
{
	SlSockAddrIn_t  xAddr;
	SlSockAddrIn_t  sAddr;
	SlSockAddrIn_t  sLocalAddr;
	int             iCounter;
	int             iAddrSize;
	int             Send_SockID,Send_SockID2,Recv_SockID;

	int             iStatus,iStatus2;
	long            lLoopCount = 0;
	short           sTestBufLen;
	int 			nonBlockingValue=1;
	int   		    packet_count=0;
	//filling the UDP server socket address
	char udpstr[2];
	udpstr[0]='A';
	udpstr[1]='B';

	sLocalAddr.sin_family = SL_AF_INET;
	sLocalAddr.sin_port = sl_Htons((unsigned short) serverPort);
	sLocalAddr.sin_addr.s_addr =0;

	sAddr.sin_family = SL_AF_INET;
	sAddr.sin_port = sl_Htons((unsigned short)destPort);
	sAddr.sin_addr.s_addr =sl_Htonl(SL_IPV4_VAL(192,168,173,1));// sl_Htonl((unsigned int)g_ulDestinationIp);192,168,173,1

	xAddr.sin_family = SL_AF_INET;
	xAddr.sin_port = sl_Htons((unsigned short)5000);
	xAddr.sin_addr.s_addr =sl_Htonl(SL_IPV4_VAL(120,0,0,0));

	iAddrSize = sizeof(SlSockAddrIn_t);

	// creating a UDP socket
	Send_SockID = sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, 0);
	if (Send_SockID < 0)
	{
		// error
		ASSERT_ON_ERROR(UCP_SERVER_FAILED);
	}

	// creating a UDP socket
		Send_SockID2 = sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, 0);
		if (Send_SockID2 < 0)
		{
			// error
			ASSERT_ON_ERROR(UCP_SERVER_FAILED);
		}

	// creating a UDP socket
	Recv_SockID = sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, 0);
	if (Recv_SockID < 0)
	{
		// error
		ASSERT_ON_ERROR(UCP_SERVER_FAILED);
	}

	// binding the UDP socket to the UDP server address
	iStatus = sl_Bind(Recv_SockID, (SlSockAddr_t *) &sLocalAddr, iAddrSize);
	if (iStatus < 0)
	{
		// error
		sl_Close(Recv_SockID);
		ASSERT_ON_ERROR(UCP_SERVER_FAILED);
	}

	//NonBlocking Socket
	iStatus = sl_SetSockOpt(Send_SockID, SL_SOL_SOCKET, SL_SO_NONBLOCKING,&nonBlockingValue, sizeof(nonBlockingValue)) ;
	if (iStatus < 0)
	{
		// error
		sl_Close(Send_SockID);
		ASSERT_ON_ERROR(UCP_SERVER_FAILED);
	}

	//NonBlocking Socket
	iStatus = sl_SetSockOpt(Send_SockID2, SL_SOL_SOCKET, SL_SO_NONBLOCKING,&nonBlockingValue, sizeof(nonBlockingValue)) ;
	if (iStatus < 0)
	{
		// error
		sl_Close(Send_SockID2);
		ASSERT_ON_ERROR(UCP_SERVER_FAILED);
	}

	//NonBlocking Socket
	iStatus = sl_SetSockOpt(Recv_SockID, SL_SOL_SOCKET, SL_SO_NONBLOCKING,&nonBlockingValue, sizeof(nonBlockingValue)) ;
	if (iStatus < 0)
	{
		// error
		sl_Close(Recv_SockID);
		ASSERT_ON_ERROR(UCP_SERVER_FAILED);
	}



	unsigned short *iter=rx_udp_server;
	unsigned short cmd_code;
	udploop=0;
	struct Command CMD_01;

	CMD_01.opcode=0x4251; //1 to start transmission
	CMD_01.len=0x0040; //length 64 bits
	CMD_01.descriptor=0x6789; //fixed to 6789 for now

	struct Command CMD_02;

	CMD_02.opcode=0x4851; //2 to stop transmission
	CMD_02.len=0x0040; //length 64 bits
	CMD_02.descriptor=0x6789; //fixed to 6789 for now





	UART_PRINT("calling  sync");



		Reset_SYNC=reset_sync_spi();

		UART_PRINT("returned from sync");
		send_cmd(&CMD_01);

		unsigned int index=0;

	while (1)
	{



while(packet_count<800)
{
		if 	(recv_ping_packet==1)
		{


			iStatus2= sl_SendTo(Send_SockID, myStrA, 1400, 0,
											(SlSockAddr_t *)&sAddr, iAddrSize);

			recv_ping_packet=0;
		}

		if 	(recv_pong_packet==1)
		{


			myStrB[0]=myStrB[698];
			myStrB[1]=myStrB[699];

			iStatus2= sl_SendTo(Send_SockID, myStrB, 1400, 0,
					(SlSockAddr_t *)&sAddr, iAddrSize);

			recv_pong_packet=0;
			packet_count++;
		}
}

iStatus = sl_RecvFrom(Recv_SockID, iter, 4, 0,
		( SlSockAddr_t *)&xAddr, (SlSocklen_t*)&iAddrSize );

if( iStatus < 0 )
{
	packet_count=0;
}
else if( iStatus > 0 )
{
	recvfromflag++;


		Reset_SYNC=reset_sync_spi();
	//	cmd_code=sl_Ntohs(*iter);
		send_cmd(iter);



}



	}//while

}

//****************************************************************************
//
//!  \brief Connecting to a WLAN Accesspoint
//!
//!   This function connects to the required AP (SSID_NAME) with Security
//!   parameters specified in te form of macros at the top of this file
//!
//!   \param[in]              None
//!
//!   \return       status value
//!
//!   \warning    If the WLAN connection fails or we don't aquire an IP
//!            address, It will be stuck in this function forever.
//
//****************************************************************************
static long WlanConnect()
{
	SlSecParams_t secParams = {0};
	long lRetVal = 0;

	secParams.Key = (signed char*)SECURITY_KEY;
	secParams.KeyLen = strlen(SECURITY_KEY);
	secParams.Type = SECURITY_TYPE;

	lRetVal = sl_WlanConnect((signed char*)SSID_NAME, strlen(SSID_NAME), 0, \
			&secParams, 0);
	ASSERT_ON_ERROR(lRetVal);

	while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))
	{
		// Wait for WLAN Event
#ifndef SL_PLATFORM_MULTI_THREADED
		_SlNonOsMainLoopTask();
#endif

	}

	return SUCCESS;
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
#if defined(ccs) || defined(gcc)
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

//****************************************************************************
//                            MAIN FUNCTION
//*****************************************g***********************************
void main()
{

	long lRetVal = -1;



	//
	// Board Initialization
	//
	BoardInit();

	//
	// uDMA Initialization
	//
	UDMAInit();

	//
	// Configure the pinmux settings for the peripherals UART AND SPI
	//
	PinMuxConfig();

	//
	// Configuring UART
	//
	InitTerm();


	//
	// Display banner
	//
	//  DisplayBanner(APPLICATION_NAME);

	InitializeAppVariables();

	//Disable and clear All SPI interrupts
	MAP_SPIIntDisable(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY|SPI_INT_DMATX|SPI_INT_DMARX);
	MAP_SPIIntClear(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY|SPI_INT_DMATX|SPI_INT_DMARX);

	struct Command CMD_01;

		CMD_01.opcode=0x0001; //1 to start transmission
		CMD_01.len=0x0040; //length 64 bits
		CMD_01.descriptor=0x6789; //fixed to 6789 for now

/*		Reset_SYNC=reset_sync_spi();
			UART_PRINT("returned from sync");
			send_cmd(&CMD_01);
while(1)
{
}*/

	//
	// Following function configure the device to default state by cleaning
	// the persistent settings stored in NVMEM (viz. connection profiles &
	// policies, power policy etc)
	//
	// Applications may choose to skip this step if the developer is sure
	// that the device is in its desired state at start of applicaton
	//
	// Note that all profiles and persistent settings that were done on the
	// device will be lost
	//
	lRetVal = ConfigureSimpleLinkToDefaultState();

	if(lRetVal < 0)
	{
		if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
			UART_PRINT("Failed to configure the device in its default state \n\r");

		LOOP_FOREVER();
	}

	UART_PRINT("Device is configured in default state \n\r");

	//
	// Asumption is that the device is configured in station mode already
	// and it is in its default state
	//
	lRetVal = sl_Start(0, 0, 0);
	if (lRetVal < 0 || lRetVal != ROLE_STA)
	{
		UART_PRINT("Failed to start the device \n\r");
		LOOP_FOREVER();
	}

	UART_PRINT("Device started as STATION \n\r");

	UART_PRINT("Connecting to AP: %s ...\r\n",SSID_NAME);

	//
	//Connecting to WLAN AP
	//
	lRetVal = WlanConnect();
	if(lRetVal < 0)
	{
		UART_PRINT("Failed to establish connection w/ an AP \n\r");
		LOOP_FOREVER();
	}

	UART_PRINT("Connected to AP: %s \n\r",SSID_NAME);

	UART_PRINT("Device IP: %d.%d.%d.%d\n\r\n\r",
			SL_IPV4_BYTE(g_ulIpAddr,3),
			SL_IPV4_BYTE(g_ulIpAddr,2),
			SL_IPV4_BYTE(g_ulIpAddr,1),
			SL_IPV4_BYTE(g_ulIpAddr,0));




	/*
#ifdef USER_INPUT_ENABLE
//    lRetVal = UserInput();
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }
#else
    lRetVal = BsdUdpClient(PORT_NUM);
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }
    lRetVal = BsdUdpServer(PORT_NUM);
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }
#endif
    UART_PRINT("Exiting Application ...\n\r");
	 */






	UdpServer(5000,50001);


	// power off the network processor
	//
	lRetVal = sl_Stop(SL_STOP_TIMEOUT);


	while (1)
	{
		_SlNonOsMainLoopTask();
	}
}




////////////////////////////////////////////


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
