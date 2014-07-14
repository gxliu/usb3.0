#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "frmwexam.h"
#include "cyu3usb.h"
#include "cyu3i2c.h"
#include "cyu3gpif.h"
#include "cyu3pib.h"
#include "pib_regs.h"
#include "cyfxgpif2config.h"
#include "cyu3socket.h"
#include "cyu3gpio.h"
#include "gpio_regs.h"
#include "cyu3vic.h"
#include "cyu3utils.h"

CyBool_t g_isActive;
CyU3PThread g_rAppThread;
CyU3PDmaBuffer_t outputBuf;
CyU3PDmaChannel g_SlFifoUtoP, g_SlFifoPtoU, g_SlFifoUtoP2, g_SlFifoPtoU2, g_SlFifoUtoP3, g_SlFifoPtoU3, g_SlFifoUtoP4, g_SlFifoPtoU4, g_SlFifoUtoP5, g_SlFifoPtoU5;
uint32_t g_Timeout = 10, g_Channels = 0, g_setupData0, g_setupData1;
uint32_t ssize;
int g_mode = -1, g_mode1 = -1;
uint8_t glEp0Buffer[4096] __attribute__ ((aligned (32))), g_Stack;
uint16_t glI2cPageSize = 0x40, wTrans=0, g_Signals[8] = {0};

void ChansSet(int mode);
void FpgaReset(int mode);


void ResetBuffer(CyU3PDmaBuffer_t buf)
{
	CyU3PMemSet(buf.buffer, 0, 1024);
	buf.count = 0;
	buf.status = 0;
	buf.size = g_SlFifoPtoU.size;
}

uint16_t selectSize(void)
{
	CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();
		switch (usbSpeed)
		{
		case CY_U3P_FULL_SPEED:
			return 64;
			break;
		case CY_U3P_HIGH_SPEED:
			return 512;
			break;
		case  CY_U3P_SUPER_SPEED:
			return 1024;
			break;
		default:
			return usbSpeed;
			break;
		}
}

CyU3PReturnStatus_t CyFxUsbI2cTransfer (uint16_t byteAddress, uint8_t devAddr, uint16_t byteCount, uint8_t *buffer, CyBool_t isRead)
{
	CyU3PI2cPreamble_t preamble;
    uint16_t pageCount = (byteCount / glI2cPageSize);
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint16_t resCount = glI2cPageSize;

    if (byteCount == 0)
        return CY_U3P_SUCCESS;

    if ((byteCount % glI2cPageSize) != 0)
    {
        pageCount ++;
        resCount = byteCount % glI2cPageSize;
    }
    while (pageCount != 0)
    {
    	preamble.buffer[0] = devAddr;
		preamble.buffer[1] = (uint8_t)(byteAddress >> 8);
		preamble.buffer[2] = (uint8_t)(byteAddress & 0xFF);
        if (isRead)
        {
            preamble.length    = 4;
            preamble.buffer[3] = (devAddr | 0x01);
            preamble.ctrlMask  = 0x0004;
            status = CyU3PI2cReceiveBytes (&preamble, buffer, (pageCount == 1) ? resCount : glI2cPageSize, 0);
            if (status != CY_U3P_SUCCESS)
                return status;
        }
        else /* Write */
        {
            preamble.length    = 3;
            preamble.ctrlMask  = 0x0000;
            status = CyU3PI2cTransmitBytes (&preamble, buffer, (pageCount == 1) ? resCount : glI2cPageSize, 0);
            if (status != CY_U3P_SUCCESS)
                return status;
            preamble.length = 1;
            status = CyU3PI2cWaitForAck(&preamble, 200);
            if (status != CY_U3P_SUCCESS)
                return status;
        }
        CyU3PThreadSleep (1);
        byteAddress  += glI2cPageSize;
        buffer += glI2cPageSize;
        pageCount --;
    }
    return CY_U3P_SUCCESS;
}

void SetMessage()
{

	uint32_t regVal, regVal2, intVal;;
	intVal = CyU3PVicDisableAllInterrupts();
	regVal = (GPIO->lpp_gpio_simple[45] & ~(CY_U3P_LPP_GPIO_INTR | CY_U3P_LPP_GPIO_OUT_VALUE));
	regVal2 = (regVal | CY_U3P_LPP_GPIO_OUT_VALUE);
	GPIO->lpp_gpio_simple[45] = regVal;
	GPIO->lpp_gpio_simple[45] = regVal2;
	CyU3PVicEnableInterrupts(intVal);
}

void DataU2GCB(CyU3PDmaChannel *chHandle, CyU3PDmaCbType_t type, CyU3PDmaCBInput_t *input)
{
	//SetMessage();
	if (g_Signals[g_Stack] != 0)
		g_Stack++;
	g_Signals[g_Stack] = FLAG_DATAREADY;
}

void ApplicationStart()
{
	uint16_t size = 0, /*burstLen = 1,*/ i;
	CyU3PEpConfig_t epCfg;
	//CyU3PGpioClock_t rGpioClk;
	//CyU3PGpioSimpleConfig_t gpioConfig;

//	CyU3PMemSet((uint8_t *)&rGpioClk, 0, sizeof(rGpioClk));
//	rGpioClk.clkSrc = CY_U3P_SYS_CLK;
//	rGpioClk.fastClkDiv = 2;

	CyU3PGpioSetValue(45, CyFalse);
	size = selectSize();

	CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
	epCfg.enable = CyTrue;
	epCfg.epType = CY_U3P_USB_EP_BULK;
	epCfg.burstLen = 1;
	epCfg.streams = 0;
	epCfg.pcktSize = size;
	CyU3PSetEpConfig(EP_PROD_1, &epCfg);
	CyU3PSetEpConfig(EP_CONS_1, &epCfg);
	if (size == 1024)
		epCfg.burstLen = BULK_BURST;
	for (i = 2; i < 5; i++)
	{
		CyU3PSetEpConfig(i, &epCfg);
		CyU3PSetEpConfig(0x80 + i, &epCfg);
	}

	ChansSet(-1);
	CyU3PGpioSetValue(45, CyTrue);
	g_isActive = CyTrue;
	if (g_Signals[g_Stack] != 0)
		g_Stack++;
	g_Signals[g_Stack] = FLAG_WAITCOMM;

//	CyU3PDeviceGpioRestore(21);
//	CyU3PDeviceGpioRestore(22);
//	CyU3PDeviceGpioRestore(25);
//	CyU3PDeviceGpioRestore(26);

//	CyU3PGpioDeInit();
//	CyU3PGpioInit(&rGpioClk, NULL);
//	CyU3PDeviceGpioOverride(45, CyTrue);
//	gpioConfig.driveHighEn = CyTrue;
//	gpioConfig.driveLowEn = CyTrue;
//	gpioConfig.inputEn = CyFalse;
//	gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
//	gpioConfig.outValue = CyTrue;
//	CyU3PGpioSetSimpleConfig(45, &gpioConfig);
//	CyU3PGpifDisable(CyTrue);
//	CyU3PGpifLoad (&CyFxGpifConfig);
//	CyU3PGpifSMStart (RESET, ALPHA_RESET);

//	for (i = 8; i >= 1; i--)
//		CyU3PGpifSocketConfigure((i-1) & 3, 0x100 + (i-1), 127, ((i-1) & 1), 1);
}

void AppStopEP(CyU3PDmaChannel* chHandle, uint8_t ep)
{
	CyU3PEpConfig_t epCfg;
	CyU3PUsbFlushEp(ep);
	CyU3PDmaChannelDestroy(chHandle);
	CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
	CyU3PSetEpConfig(ep, &epCfg);
}

void ApplicationStop()
{
	g_isActive = CyFalse;
	AppStopEP(&g_SlFifoUtoP, EP_PROD_1);
	AppStopEP(&g_SlFifoPtoU, EP_CONS_1);
	AppStopEP(&g_SlFifoUtoP2, EP_PROD_2);
	AppStopEP(&g_SlFifoPtoU2, EP_CONS_2);
	AppStopEP(&g_SlFifoUtoP3, EP_PROD_3);
	AppStopEP(&g_SlFifoPtoU3, EP_CONS_3);
	AppStopEP(&g_SlFifoUtoP4, EP_PROD_4);
	AppStopEP(&g_SlFifoPtoU4, EP_CONS_4);
	AppStopEP(&g_SlFifoUtoP5, EP_PROD_5);
	AppStopEP(&g_SlFifoPtoU5, EP_CONS_5);
	if (g_Signals[g_Stack] != 0)
		g_Stack++;
	g_Signals[g_Stack] = FLAG_APPSTART;
}

void EPClearReq(uint16_t wIndex)
{
	CyU3PDmaChannel chHandle;
	switch (wIndex)
	{
	case EP_PROD_1:
		chHandle = g_SlFifoUtoP;
		break;
	case EP_CONS_1:
		chHandle = g_SlFifoPtoU;
		break;
	case EP_PROD_2:
		chHandle = g_SlFifoUtoP2;
		break;
	case EP_CONS_2:
		chHandle = g_SlFifoPtoU2;
		break;
	case EP_PROD_3:
		chHandle = g_SlFifoUtoP3;
		break;
	case EP_CONS_3:
		chHandle = g_SlFifoPtoU3;
		break;
	case EP_PROD_4:
		chHandle = g_SlFifoUtoP4;
		break;
	case EP_CONS_4:
		chHandle = g_SlFifoPtoU4;
		break;
	case EP_PROD_5:
		chHandle = g_SlFifoUtoP5;
		break;
	case EP_CONS_5:
		chHandle = g_SlFifoPtoU5;
		break;
	}
	CyU3PDmaChannelReset (&chHandle);
	CyU3PUsbStall(wIndex, CyFalse, CyTrue);
	CyU3PUsbFlushEp(wIndex);
	CyU3PUsbResetEp (wIndex);
	CyU3PDmaChannelSetXfer (&chHandle, DMA_TX_SIZE);
}

CyBool_t USBSetupCB(uint32_t setupdat0, uint32_t setupdat1)
{
	uint8_t  bRequest, bReqType, i2cAddr, bType, bTarget;
	uint16_t wValue, wIndex, wLength;
	CyBool_t isHandled = CyFalse;
	//CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

	bReqType = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
	bType    = (bReqType & CY_U3P_USB_TYPE_MASK);
	bTarget  = (bReqType & CY_U3P_USB_TARGET_MASK);
	bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
	wValue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);
	wIndex   = ((setupdat1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS);
	wLength   = ((setupdat1 & CY_U3P_USB_LENGTH_MASK)   >> CY_U3P_USB_LENGTH_POS);

	if (bType == CY_U3P_USB_VENDOR_RQT)
	{
		isHandled = CyTrue;
		i2cAddr = wValue << 1;
		switch (bRequest)
		{
		case 0xBA:
			g_setupData0 = setupdat0;
			g_setupData1 = setupdat1;
			if (g_Signals[g_Stack] != 0)
				g_Stack++;
			g_Signals[g_Stack] = FLAG_GETI2C;
//			status  = CyU3PUsbGetEP0Data(((wLength + 15) & 0xFFF0), glEp0Buffer, NULL);
//			if (status == CY_U3P_SUCCESS)
//				status = CyFxUsbI2cTransfer (wIndex, i2cAddr, wLength, glEp0Buffer, CyFalse);
			break;
		case 0xBB:
			g_setupData0 = setupdat0;
			g_setupData1 = setupdat1;
			if (g_Signals[g_Stack] != 0)
				g_Stack++;
			g_Signals[g_Stack] = FLAG_REQI2C;
//			CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
//			status = CyFxUsbI2cTransfer (wIndex, i2cAddr, wLength, glEp0Buffer, CyTrue);
//			if (status == CY_U3P_SUCCESS)
//				status = CyU3PUsbSendEP0Data(wLength, glEp0Buffer);
			break;
//		case 0xCC:
//			CyU3PDeviceReset(CyTrue);
//			break;
		default:
			isHandled = CyFalse;
			break;
		}
//		if (status != CY_U3P_SUCCESS)
//			isHandled = CyFalse;
	}
	if (bType == CY_U3P_USB_STANDARD_RQT)
	{
		if ((bTarget == CY_U3P_USB_TARGET_INTF) && ((bRequest == CY_U3P_USB_SC_SET_FEATURE)	|| (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)) && (wValue == 0))
		{
			if (g_isActive)
				CyU3PUsbAckSetup ();
			else
				CyU3PUsbStall (0, CyTrue, CyTrue);
			isHandled = CyTrue;
		}
		if ((bTarget == CY_U3P_USB_TARGET_ENDPT) && (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE) && (wValue == CY_U3P_USBX_FS_EP_HALT))
		{
//			if (g_isActive)
//			{
//				switch (wIndex)
//				{
//				case EP_PROD_1:
//					EPClearReq (&g_SlFifoUtoP, EP_PROD_1);
//					break;
//				case EP_CONS_1:
//					EPClearReq (&g_SlFifoPtoU, EP_CONS_1);
//					break;
//				case EP_PROD_2:
//					EPClearReq (&g_SlFifoUtoP2, EP_PROD_2);
//					break;
//				case EP_CONS_2:
//					EPClearReq (&g_SlFifoPtoU2, EP_CONS_2);
//					break;
//				case EP_PROD_3:
//					EPClearReq (&g_SlFifoUtoP3, EP_PROD_3);
//					break;
//				case EP_CONS_3:
//					EPClearReq (&g_SlFifoPtoU3, EP_CONS_3);
//					break;
//				case EP_PROD_4:
//					EPClearReq (&g_SlFifoUtoP4, EP_PROD_4);
//					break;
//				case EP_CONS_4:
//					EPClearReq (&g_SlFifoPtoU4, EP_CONS_4);
//					break;
//				case EP_PROD_5:
//					EPClearReq (&g_SlFifoUtoP5, EP_PROD_5);
//					break;
//				case EP_CONS_5:
//					EPClearReq (&g_SlFifoPtoU5, EP_CONS_5);
//					break;
//				}
//				CyU3PUsbStall (wIndex, CyFalse, CyTrue);
//				isHandled = CyTrue;
//			}
			g_setupData1 = setupdat1;
			if (g_Signals[g_Stack] != 0)
				g_Stack++;
			g_Signals[g_Stack] = FLAG_CLEAREP;
		}
	}
	return isHandled;
}

void USBEventCB(CyU3PUsbEventType_t evtype, uint16_t evdata)
{
	switch (evtype)
	{
	case CY_U3P_USB_EVENT_RESET:
	case CY_U3P_USB_EVENT_DISCONNECT:
	case CY_U3P_USB_EVENT_SETCONF:
		if (g_Signals[g_Stack] != 0)
			g_Stack++;
		if (g_isActive)
			g_Signals[g_Stack] = FLAG_APPSTOP;
		else
			g_Signals[g_Stack] = FLAG_APPSTART;
			//ApplicationStop();
		//ApplicationStart(-1);
		break;


//    	CyU3PDeviceReset(CyTrue);
//    	break;

//    	if (g_isActive)
//    		ApplicationStop();
//    	break;

    default:
    	break;
	}
}

CyBool_t LPMRqtCB(CyU3PUsbLinkPowerMode link_mode)
{
	return CyFalse;
}

void AppInit()
{
	CyU3PPibClock_t pibClock;
	CyU3PGpioSimpleConfig_t gpioConfig;
	uint16_t i;

	g_Stack = 0;
	g_mode = -1;

	pibClock.clkDiv = 2;
	pibClock.clkSrc = CY_U3P_SYS_CLK;
	pibClock.isHalfDiv = CyFalse;
	pibClock.isDllEnable = CyFalse;
	CyU3PPibInit(CyTrue, &pibClock);
	CyU3PGpifLoad (&CyFxGpifConfig);
	CyU3PGpifSMStart (RESET, ALPHA_RESET);

	//Reset FPGA
	CyU3PDeviceGpioOverride(45, CyTrue);

//	CyU3PDeviceGpioOverride(21, CyTrue);
//	CyU3PDeviceGpioOverride(22, CyTrue);
//	CyU3PDeviceGpioOverride(25, CyTrue);
//	CyU3PDeviceGpioOverride(26, CyTrue);

	gpioConfig.driveHighEn = CyTrue;
	gpioConfig.driveLowEn = CyTrue;
	gpioConfig.inputEn = CyFalse;
	gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
	gpioConfig.outValue = CyFalse;
	CyU3PGpioSetSimpleConfig(45, &gpioConfig);

//	gpioConfig.outValue = CyTrue;
//	CyU3PGpioSetSimpleConfig(21, &gpioConfig);
//	gpioConfig.outValue = CyFalse;
//	CyU3PGpioSetSimpleConfig(22, &gpioConfig);
//	gpioConfig.outValue = CyTrue;
//	CyU3PGpioSetSimpleConfig(25, &gpioConfig);
//	gpioConfig.outValue = CyFalse;
//	CyU3PGpioSetSimpleConfig(26, &gpioConfig);

	CyU3PGpioSetValue(45, CyFalse);
	CyU3PThreadSleep (2);

	for (i = 8; i >= 1; i--)
		CyU3PGpifSocketConfigure((i-1) & 3, 0x100 + (i-1), 127, ((i-1) & 1), 1);

//	CyU3PGpifSocketConfigure(0, CY_U3P_PIB_SOCKET_4, 127, CyTrue, 1);
//	CyU3PGpifSocketConfigure(1, CY_U3P_PIB_SOCKET_5, 127, CyFalse, 1);
//	CyU3PGpifSocketConfigure(2, CY_U3P_PIB_SOCKET_6, 127, CyTrue, 1);
//	CyU3PGpifSocketConfigure(3, CY_U3P_PIB_SOCKET_7, 127, CyFalse, 1);
//	CyU3PGpifSocketConfigure(0, CY_U3P_PIB_SOCKET_0, 127, CyFalse, 1);
//	CyU3PGpifSocketConfigure(1, CY_U3P_PIB_SOCKET_1, 127, CyTrue, 1);
//	CyU3PGpifSocketConfigure(2, CY_U3P_PIB_SOCKET_2, 2, CyFalse, 1);
//	CyU3PGpifSocketConfigure(3, CY_U3P_PIB_SOCKET_3, 2, CyTrue, 1);

	CyU3PUsbStart();
	CyU3PUsbRegisterSetupCallback(USBSetupCB, CyTrue);
	CyU3PUsbRegisterEventCallback(USBEventCB);
	CyU3PUsbRegisterLPMRequestCallback(LPMRqtCB);

	CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, NULL, (uint8_t *)USB30DeviceDscr);
	CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, NULL, (uint8_t *)USB20DeviceDscr);
	CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, NULL, (uint8_t *)USBBOSDscr);
	CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, NULL, (uint8_t *)USBDeviceQualDscr);
	CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, NULL, (uint8_t *)USBSSConfigDscr);
	CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, NULL, (uint8_t *)USBHSConfigDscr);
	CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, NULL, (uint8_t *)USBFSConfigDscr);
    CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)USBStringLangIDDscr);
    CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)USBManufactureDscr);
    CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)USBProductDscr);
    CyU3PConnectState(CyTrue, CyTrue);
}

uint8_t TranslateCommand(uint8_t* inBuf, uint8_t* outBuf)
{
	uint8_t bType = 0, bSrc;
	uint32_t buf = 0;
	ssize = PIB->pp_dma_size;
	if ((inBuf[BYTE_SIGN] != SIGN_IN) || (inBuf[BYTE_VERSION] != COMM_VER))
		return 0xFE;
	CyU3PMemSet(outBuf, 0, 1024);
	bType = (inBuf[BYTE_COMM] >> BIT_CTYPE) & 31;
	bSrc = (inBuf[BYTE_COMM] & 7);
	wTrans++;
	CyU3PMemCopy(outBuf + 6, (uint8_t*)&wTrans, 2);
	switch (bType)
	{
	case 0:
		if ((inBuf[BYTE_COMM] & 7) == 0)
			bType = 0xF1;
		CyU3PMemCopy((uint8_t *)&g_Timeout, inBuf+BYTE_HVAL, 4);
		if (g_Timeout == 0)
			g_Timeout = 10;
		g_mode = inBuf[BYTE_PARAM];
		if (g_Signals[g_Stack] != 0)
			g_Stack++;
		g_Signals[g_Stack] = FLAG_GETREPLY;
		g_Stack++;
		g_Signals[g_Stack] = FLAG_FPGARESET;
		break;
	case 1:
		switch (bSrc & 3)
		{
		case 0:
			CyU3PMemCopy(outBuf, inBuf, 4);
			outBuf[4] = ((outBuf[0] & 8) << 4) + ((inBuf[BYTE_PARAM] & 31) << 2) + (inBuf[BYTE_HVAL] & 3);
			outBuf[5] = ((inBuf[BYTE_COMM] & 4) << 5) + (inBuf[5] & 31);
			break;
		case 1:
			CyU3PMemCopy(outBuf, inBuf, 2);
			outBuf[4] = ((inBuf[BYTE_COMM] & 4) << 5) + ((inBuf[BYTE_PARAM] & 31) << 2) + (inBuf[BYTE_HVAL] & 3);
			outBuf[5] = ((inBuf[BYTE_COMM] & 4) << 5) + (1 << BIT_TYPE);
			break;
		default:
			CyU3PMemCopy(outBuf, inBuf, 4);
			outBuf[4] = inBuf[BYTE_HVAL];
			outBuf[5] = ((inBuf[BYTE_COMM] & 4) << 5) + ((bType & 3) << BIT_TYPE) + (inBuf[BYTE_HVAL+1] & 31);
			break;
		}
		if (g_Signals[g_Stack] != 0)
			g_Stack++;
		g_Signals[g_Stack] = FLAG_COMMREADY;
		break;
	case 2:
		CyU3PMemCopy(outBuf, inBuf + BYTE_HVAL, 4);
		outBuf[4] = (1 << BIT_WR) + ((inBuf[BYTE_PARAM] & 31) << 2) + 2;
		outBuf[5] = (5 << BIT_TYPE);
		wTrans++;
		buf = ((inBuf[BYTE_PARAM] & 31) << 2) + (1 << 13) + ((uint32_t)wTrans << 16);
		CyU3PMemCopy(outBuf + 12, (uint8_t *)&buf, 4);
		CyU3PMemCopy(outBuf + 16, inBuf, 2);
		wTrans++;
		buf = ((bSrc & 4) << BIT_TYPE) + ((inBuf[BYTE_PARAM] & 31) << 2) + 3 + (((bSrc & 4) + 1) << 13) + ((uint32_t)wTrans << 16);
		CyU3PMemCopy(outBuf + 20, (uint8_t *)&buf, 4);
		if (g_Signals[g_Stack] != 0)
			g_Stack++;
		g_Signals[g_Stack] = FLAG_COMMREADY;
		break;
	case 3:
		//outBuf[1] = (g_SlFifoUtoP.consSckId - 0x100) + ((g_SlFifoPtoU.prodSckId - 0x100) << 4);
		outBuf[0] = 1 << BIT_ON;//2;
		switch ((inBuf[BYTE_PARAM] >> BIT_CSCOND) & 7)
		{
		case 1:
			outBuf[0] = (1 << BIT_ON) + (1 << BIT_OUT);//10;
			break;
		case 3:
		case 5:
			outBuf[0] = (g_Channels >> (bSrc * 3 - 1)) & ((1 << BIT_ON) + (1 << BIT_OUT));//(((g_Channels >> (inBuf[BYTE_COMM] & 7)) & 4) << 1) & 250;
			break;
		case 4:
			outBuf[0] = ((g_Channels >> (bSrc * 3 - 1)) & (1 << BIT_OUT)) | ((1 << BIT_ON) + (1 << BIT_EN));//(((g_Channels >> (inBuf[BYTE_COMM] & 7)) & 4) << 1) | 6;
			break;
		case 6:
			//outBuf[0] = (g_Channels >> (bSrc - 1)) & (1 << BIT_OUT);//((g_Channels >> (inBuf[BYTE_COMM] & 7)) & 7) << 1;
		case 7:
			outBuf[0] = (g_Channels >> (bSrc * 3 - 1)) & (1 << BIT_OUT);//(((g_Channels >> (inBuf[BYTE_COMM] & 7)) & 4) << 1) & 248;
			break;
		}
		outBuf[3] = 0x10;
		outBuf[4] = 1 + (((outBuf[0] >> BIT_OUT) & 1) << BIT_WR);//((outBuf[0] >> 3) & 1) + ((inBuf[8] & 31) << 2) + 2;
		//outBuf[2] = outBuf[4];
		outBuf[5] = (1 << BIT_WR) + (bSrc << 2);
		if (g_Signals[g_Stack] != 0)
			g_Stack++;
		g_Signals[g_Stack] = FLAG_COMMREADY;
		break;
	case 7:
		ssize = PIB->pp_dma_xfer;
		ssize = PIB->pp_dma_size;
		break;
	default:
		if (g_Signals[g_Stack] != 0)
			g_Stack++;
		g_Signals[g_Stack] = FLAG_GETREPLY;
		return bType;
	}
	return bType;
}

uint32_t CheckAnswer(uint8_t* inBuf, uint8_t* sendBuf, uint8_t bType)
{
	uint32_t status = 0;
	uint8_t bSrc = (sendBuf[5] >> BIT_TYPE) & 3;
	if ((CyU3PMemCmp(inBuf+6, sendBuf+6, 2) != 0) && (bType != 0))
		status = 0x2;
	switch (bType)
	{
	case 1:
		switch (bSrc)
		{
		case 0:
			if (CyU3PMemCmp(inBuf + 3, sendBuf + 3, 3) != 0)
				status += 0x4;
			if (CyU3PMemCmp(inBuf + 2, sendBuf + 4, 1) != 0)
				status += 0x8;
			if (CyU3PMemCmp(inBuf, sendBuf, 1) != 0)
				status += 0x10;
			if (((inBuf[1] & 15) != ((g_SlFifoUtoP.activeConsIndex - 0x100)&15)) || (((inBuf[1] >> 4) & 15) != ((g_SlFifoPtoU.activeProdIndex - 0x100)&15)))
				status += 0x20;
			break;
		case 1:
		case 2:
		case 3:
			if (CyU3PMemCmp(inBuf + 4, sendBuf + 4, 2) != 0)
				status += 0x40;
			if ((((inBuf[5] >> BIT_WR) & 1) == 1) && (CyU3PMemCmp(inBuf, sendBuf, 4) != 0))
				status += 0x80;
			break;
		}
		break;
	case 2:
		if ((CyU3PMemCmp(inBuf + 4, sendBuf + 4, 2) | CyU3PMemCmp(inBuf + 12, sendBuf + 12, 2) | CyU3PMemCmp(inBuf + 20, sendBuf + 20, 2)) != 0)
			status += 0x100;
		if ((CyU3PMemCmp(inBuf + 14, sendBuf + 14, 2) | CyU3PMemCmp(inBuf + 22, sendBuf + 22, 2)) != 0)
			status += 0x200;
		if ((CyU3PMemCmp(inBuf, sendBuf, 4)) != 0)
			status += 0x400;
		if ((((inBuf[21] >> BIT_WR) & 1) == 1) && (CyU3PMemCmp(inBuf+16, sendBuf+16, 4) != 0))
			status += 0x800;
		if ((inBuf[8] & 1) != 1)
			status = 1;
		break;
	case 3:
		if (CyU3PMemCmp(inBuf + 3, sendBuf + 3, 3) != 0)
			status += 0x1000;
//		if (CyU3PMemCmp(inBuf + 2, sendBuf + 4, 1) != 0)
//			status += 0x04;
		if (CyU3PMemCmp(inBuf, sendBuf, 1) != 0)
			status += 0x2000;
//		if (((inBuf[1] & 15) != (g_SlFifoUtoP.consSckId - 0x100)) || (((inBuf[1] >> 4) & 15) != (g_SlFifoPtoU.prodSckId - 0x100)))
//			status += 0x80;
		break;
	}
	return status;
}

uint32_t GetAnswer(uint8_t* outBuf, uint8_t* recvBuf, uint8_t bType, uint32_t nResult, uint32_t nTimeout)
{
	uint32_t status = 0;
	outBuf[BYTE_SIGN] = SIGN_OUT;
	switch (bType)
	{
	case 0:
		CyU3PMemCopy(outBuf, (uint8_t *)&nTimeout, 4);
		CyU3PMemCopy(outBuf + BYTE_PARAM, (uint8_t *)&g_mode, 1);
		break;
	case 1:
		CyU3PMemCopy(outBuf, recvBuf, 4);
		if ((recvBuf[5] & 96) == 0)
			g_Channels = (g_Channels & (~(7 << (((recvBuf[5] >> BIT_STREAM) & 7) * 3)))) + (((recvBuf[0] >> 1) & 7) << (((recvBuf[5] >> BIT_STREAM) & 7) * 3));//(g_Channels & (~(7 << ((outBuf[5] & 7) >> 2)))) + ((outBuf[0] >> 1) & 7);
		break;
	case 2:
		CyU3PMemCopy(outBuf, recvBuf+16, 4);
		break;
	case 3:
		g_Channels = (g_Channels & (~(7 << ((outBuf[BYTE_COMM] & 7) * 3)))) + (((recvBuf[0] >> 1) & 7) << ((outBuf[BYTE_COMM] & 7) * 3));//(g_Channels & (~(7 << ((recvBuf[5] & 7) >> 2)))) + ((recvBuf[0] >> 1) & 7);
		break;
	case 0xf1:
		nResult = outBuf[BYTE_PARAM];
		if (nTimeout < g_Timeout)
			CyU3PThreadSleep(g_Timeout - nTimeout);
	default:
		outBuf[BYTE_COMM] = 0;
		CyU3PMemCopy(outBuf + BYTE_PARAM, (uint8_t*)&nResult, 1);//outBuf[BYTE_PARAM] = nResult;
		break;
	}
	return status;
}

void AppThreadEntry(uint32_t input)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	CyU3PDmaBuffer_t tempBuf, buffer_p;//, bufStream;
	uint8_t /*bMode,*/ bType = 0xF2, i, i2cAddr;
	uint16_t wValue, wIndex, wLength;
	uint32_t nCommands, nTimeout;
	AppInit();
	tempBuf.buffer = (uint8_t*)CyU3PDmaBufferAlloc(1024);
	buffer_p.buffer = (uint8_t*)CyU3PDmaBufferAlloc(1024);
	for (;;)
	{

		switch (g_Signals[g_Stack])
		{
		case (FLAG_APPSTART):
			g_Signals[g_Stack] = 0;
			if (g_Stack != 0)
				g_Stack--;
			ApplicationStart();
			break;
		case (FLAG_APPSTOP):
			g_Signals[g_Stack] = 0;
			if (g_Stack != 0)
				g_Stack--;
			ApplicationStop();
			break;
		case (FLAG_GETI2C):
			g_Signals[g_Stack] = 0;
			if (g_Stack != 0)
				g_Stack--;
			wValue   = ((g_setupData0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);
			wIndex   = ((g_setupData1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS);
			wLength   = ((g_setupData1 & CY_U3P_USB_LENGTH_MASK)   >> CY_U3P_USB_LENGTH_POS);
			i2cAddr = wValue << 1;
			CyU3PUsbGetEP0Data(((wLength + 15) & 0xFFF0), glEp0Buffer, NULL);
			CyFxUsbI2cTransfer(wIndex, i2cAddr, wLength, glEp0Buffer, CyFalse);
			break;
		case (FLAG_REQI2C):
			g_Signals[g_Stack] = 0;
			if (g_Stack != 0)
				g_Stack--;
			wValue   = ((g_setupData0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);
			wIndex   = ((g_setupData1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS);
			wLength   = ((g_setupData1 & CY_U3P_USB_LENGTH_MASK)   >> CY_U3P_USB_LENGTH_POS);
			i2cAddr = wValue << 1;
			CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
			CyFxUsbI2cTransfer(wIndex, i2cAddr, wLength, glEp0Buffer, CyTrue);
			CyU3PUsbSendEP0Data(wLength, glEp0Buffer);
			break;
		case (FLAG_CLEAREP):
			g_Signals[g_Stack] = 0;
			if (g_Stack != 0)
				g_Stack--;
			wIndex = ((g_setupData1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS);
			EPClearReq(wIndex);
			break;
		case (FLAG_DATAREADY):
			SetMessage();
			g_Signals[g_Stack] = 0;
			if (g_Stack != 0)
				g_Stack--;
			break;
		case (FLAG_WAITCOMM):
			if (CyU3PDmaChannelGetBuffer(&g_SlFifoUtoP, &buffer_p, CYU3P_NO_WAIT) == CY_U3P_SUCCESS)
			{
				nCommands = (buffer_p.count / 16) - (buffer_p.count % 16);
				i = 0;
				CyU3PDmaChannelDiscardBuffer(&g_SlFifoUtoP);
				g_Stack++;
				g_Signals[g_Stack] = (FLAG_GETCOMM);
			}
			break;
		case (FLAG_GETCOMM):
			g_Signals[g_Stack] = 0;
			if (g_Stack != 0)
				g_Stack--;
			nTimeout = CyU3PGetTime();
			bType = TranslateCommand(buffer_p.buffer + i*16, tempBuf.buffer);
			if ((bType == 1) || (bType == 3))
				tempBuf.count = 8;
			if (bType == 2)
				tempBuf.count = 24;
			if (bType >= 0xF0)
			{
				if (g_Signals[g_Stack] != 0)
					g_Stack++;
				g_Signals[g_Stack] = (FLAG_GETREPLY);
			}
			tempBuf.size = g_SlFifoUtoP.size;
			tempBuf.status = 0;
			break;
		case (FLAG_COMMREADY):
			CyU3PDmaChannelReset(&g_SlFifoUtoP);
			CyU3PDmaChannelSetXfer(&g_SlFifoPtoU, DMA_TX_SIZE);
			CyU3PDmaChannelSetupSendBuffer(&g_SlFifoUtoP, &tempBuf);
			SetMessage();
			g_Signals[g_Stack] = (FLAG_WAITREPLY);
			break;
		case (FLAG_WAITREPLY):
			if (CyU3PDmaChannelGetBuffer(&g_SlFifoPtoU, &outputBuf, 3) == CY_U3P_SUCCESS)
			{
				g_Signals[g_Stack] = (FLAG_GETREPLY);
				CyU3PDmaChannelDiscardBuffer(&g_SlFifoPtoU);
			}
			else
				SetMessage();
			break;
		case (FLAG_GETREPLY):
//			g_Signals[g_Stack] = 0;
//			if (g_Stack != 0)
//				g_Stack--;
			apiRetStatus = CheckAnswer(outputBuf.buffer, tempBuf.buffer, bType);
			nTimeout = CyU3PGetTime() - nTimeout;
			GetAnswer(buffer_p.buffer + (16*i), outputBuf.buffer, bType, apiRetStatus, nTimeout);
			CyU3PMemCopy(buffer_p.buffer + (16*i) + BYTE_DEBUG, &bType, 1);
			i++;
			if (i >= nCommands)
				g_Signals[g_Stack] = (FLAG_REPLYREADY);
			else
				g_Signals[g_Stack] = (FLAG_GETCOMM);
			break;
		case (FLAG_REPLYREADY):
			g_Signals[g_Stack] = 0;
			if (g_Stack != 0)
				g_Stack--;

			CyU3PDmaChannelReset(&g_SlFifoPtoU);
			CyU3PDmaChannelSetupSendBuffer(&g_SlFifoPtoU, &buffer_p);
			apiRetStatus = CyU3PDmaChannelSetXfer(&g_SlFifoUtoP, DMA_TX_SIZE);
			apiRetStatus = CyU3PDmaChannelSetXfer(&g_SlFifoPtoU, DMA_TX_SIZE);
			break;
		case (FLAG_FPGARESET):
			FpgaReset(g_mode);
			for (i = 0; i < 8; i++)
				g_Signals[i] = 0;
			g_Stack = 0;
			g_Signals[g_Stack] = (FLAG_WAITCOMM);
			g_Stack = 1;
			g_Signals[g_Stack] = (FLAG_GETREPLY);
			break;
		default:
			break;
		}
/*
		if (!g_isActive)
			continue;
//		apiRetStatus = CyU3PDmaChannelGetBuffer(&g_SlFifoPtoU2, &bufStream, CYU3P_NO_WAIT);
//		if (apiRetStatus == CY_U3P_SUCCESS)
//			apiRetStatus = CyU3PDmaChannelCommitBuffer(&g_SlFifoPtoU2, bufStream.count, bufStream.status);
//		apiRetStatus = CyU3PDmaChannelGetBuffer(&g_SlFifoUtoP2, &bufStream, CYU3P_NO_WAIT);
//		if (apiRetStatus == CY_U3P_SUCCESS)
//			apiRetStatus = CyU3PDmaChannelCommitBuffer(&g_SlFifoUtoP2, bufStream.count, bufStream.status);
		apiRetStatus = CyU3PDmaChannelGetBuffer(&g_SlFifoUtoP, &buffer_p, CYU3P_NO_WAIT);
		if (apiRetStatus != CY_U3P_SUCCESS)
			continue;
		if (buffer_p.count > 0)
		{
			CyU3PDmaChannelDiscardBuffer(&g_SlFifoUtoP);
			nCommands = (buffer_p.count / 16) - (buffer_p.count % 16);
			for (i = 0; i < nCommands; i++)
			{
				nTimeout = CyU3PGetTime();
				CyU3PMemSet(tempBuf.buffer, 0, 1024);
				bType = TranslateCommand(buffer_p.buffer + i*16, tempBuf.buffer);
				switch (bType)
				{
				case 0:
					bMode = buffer_p.buffer[8];
					FpgaReset(bMode);
					break;
				case 1:
				case 3:
					CyU3PDmaChannelReset(&g_SlFifoUtoP);
					tempBuf.count = 8;
					break;
				case 2:
					CyU3PDmaChannelReset(&g_SlFifoUtoP);
					tempBuf.count = 24;
					break;
				case (0xF1):
					break;
				default:
					break;
				}
				for (;;)
				{
					if ((bType > 0) && (bType < 0xF1) && (g_mode != -1))
					{
						apiRetStatus = CyU3PDmaChannelSetXfer(&g_SlFifoPtoU, DMA_TX_SIZE);
						tempBuf.size = g_SlFifoPtoU.size;
						tempBuf.status = 0;
						CyU3PDmaChannelSetupSendBuffer(&g_SlFifoUtoP, &tempBuf);
						CyU3PMemSet(outputBuf.buffer, 0, 1024);
						for (;;)
						{
							SetMessage();
							apiRetStatus = CyU3PDmaChannelGetBuffer(&g_SlFifoPtoU, &outputBuf, 3);
							CyU3PDmaChannelDiscardBuffer(&g_SlFifoPtoU);
							if ((apiRetStatus == CY_U3P_SUCCESS)||((CyU3PGetTime() - nTimeout) > g_Timeout))
								break;
						}
					}
					apiRetStatus = CheckAnswer(outputBuf.buffer, tempBuf.buffer, bType);
					if ((apiRetStatus != 1) || ((CyU3PGetTime() - nTimeout) > g_Timeout))
						break;
				}
				if (apiRetStatus > 0)
				{
					bType = 0xFF;
//					if (apiRetStatus != 1)
//						apiRetStatus = 3;
				}
				nTimeout = (uint32_t)(CyU3PGetTime() - nTimeout);
				if (nTimeout > g_Timeout)
				{
					apiRetStatus = 1;
					bType = 0xFF;
				}
				GetAnswer(buffer_p.buffer + (16*i), outputBuf.buffer, bType, apiRetStatus, nTimeout);
				CyU3PMemCopy(buffer_p.buffer + (16*i) + BYTE_DEBUG, &bType, 1);
			}
			apiRetStatus = CyU3PDmaChannelReset(&g_SlFifoPtoU);
			apiRetStatus = CyU3PDmaChannelSetupSendBuffer(&g_SlFifoPtoU, &buffer_p);
			//CyU3PMemSet(buffer_p.buffer, 0, 1024);
//			CyU3PThreadSleep(1);
//			CyU3PDmaChannelReset(&g_SlFifoPtoU);
			apiRetStatus = CyU3PDmaChannelSetXfer(&g_SlFifoUtoP, DMA_TX_SIZE);
			apiRetStatus = CyU3PDmaChannelSetXfer(&g_SlFifoPtoU, DMA_TX_SIZE);
		}
		else
			CyU3PDmaChannelDiscardBuffer(&g_SlFifoUtoP);
//		ResetBuffer(outputBuf);
//		ResetBuffer(buffer_p);
//		ResetBuffer(tempBuf);*/
	}

}

void
CyFxApplicationDefine (
        void)
{

	void *pThread = NULL;
	uint32_t status = CY_U3P_SUCCESS;

	pThread = CyU3PMemAlloc(THREAD_STACK);
	status = CyU3PThreadCreate(&g_rAppThread, 			//pointer to application thread
								"21:FMC406C",			//thread id and name
								AppThreadEntry,			//thread entry function
								0,						//thread entry function parameter
								pThread,				//pointer to thread stack
								THREAD_STACK,			//thread stack size
								THREAD_PRIORITY,		//thread priority
								THREAD_PRIORITY,		//min priority to preempt thread
								CYU3P_NO_TIME_SLICE,	//using of time slice
								CYU3P_AUTO_START);		//auto start the thread
//	if (status != 0)
//		ErrorHandle(status);
}

int
main (void)
{
	CyU3PSysClockConfig_t rClkCfg;
	CyU3PIoMatrixConfig_t rIOCfg;
	CyU3PGpioClock_t rGpioClk;
	rClkCfg.setSysClk400 = CyFalse; //False - clock freq = 384 MHz, True - clock freq = 403.2 MHz
	rClkCfg.cpuClkDiv = 2; //default
	rClkCfg.mmioClkDiv = 2; //default
	rClkCfg.dmaClkDiv = 2; //DMA clock must be = N*MMIO clock, N non-zero integer
	rClkCfg.useStandbyClk = CyTrue; //if 32KHz clock has been supplied on the CLKIN_32 pin
	rClkCfg.clkSrc = CY_U3P_SYS_CLK; //clock freq
	CyU3PDeviceInit(NULL);//NULL - use default params
	CyU3PDeviceCacheControl(CyTrue, CyTrue, CyTrue); //enable instructions cache, data cache, autoclean and autoflush DMA cache
	CyU3PMemSet((uint8_t *)&rIOCfg, 0, sizeof(rIOCfg));
	//rIOCfg.useUart = CyFalse;
	rIOCfg.useI2C = CyTrue;
	//rIOCfg.useI2S = CyFalse;
	//rIOCfg.useSpi = CyFalse;
	rIOCfg.isDQ32Bit = CyTrue;//GPIF data bus 32bit wide
	rIOCfg.gpioComplexEn[0] = 0;
	rIOCfg.gpioComplexEn[1] = 0;
	rIOCfg.gpioSimpleEn[0] = 0;
	rIOCfg.gpioSimpleEn[1] = 0;
	rIOCfg.lppMode = CY_U3P_IO_MATRIX_LPP_DEFAULT;
	CyU3PDeviceConfigureIOMatrix(&rIOCfg);
	CyU3PMemSet((uint8_t *)&rGpioClk, 0, sizeof(rGpioClk));
	rGpioClk.clkSrc = CY_U3P_SYS_CLK;
	rGpioClk.fastClkDiv = 2;
	//rGpioClk.halfDiv = CyFalse;
	//rGpioClk.simpleDiv = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
	//rGpioClk.slowClkDiv = 0;
	CyU3PGpioInit(&rGpioClk, NULL);
	CyU3PKernelEntry();
	return 0;
}

void AbortAndDestroy(CyU3PDmaChannel* chHandle)
{
	//CyU3PDmaChannelDiscardBuffer(chHandle);
	//CyU3PDmaChannelReset(chHandle);
	CyU3PDmaChannelAbort(chHandle);
//	CyU3PDmaChannelReset(chHandle);
	CyU3PDmaChannelDestroy(chHandle);
}

void FpgaReset(int mode)
{
	CyU3PDmaBuffer_t rBuf;
	uint16_t i, size = 64, wmark = 127;
	//CyU3PGpioSimpleConfig_t gpioConfig;
	//CyU3PGpioClock_t rGpioClk;
	//CyU3PReturnStatus_t status = 0;
//	CyU3PMemSet((uint8_t *)&rGpioClk, 0, sizeof(rGpioClk));
//	rGpioClk.clkSrc = CY_U3P_SYS_CLK;
//	rGpioClk.fastClkDiv = 2;

	AbortAndDestroy(&g_SlFifoPtoU);
	AbortAndDestroy(&g_SlFifoPtoU2);
	AbortAndDestroy(&g_SlFifoPtoU3);
	AbortAndDestroy(&g_SlFifoPtoU4);
	AbortAndDestroy(&g_SlFifoPtoU5);
	AbortAndDestroy(&g_SlFifoUtoP);
	AbortAndDestroy(&g_SlFifoUtoP2);
	AbortAndDestroy(&g_SlFifoUtoP3);
	AbortAndDestroy(&g_SlFifoUtoP4);
	AbortAndDestroy(&g_SlFifoUtoP5);

	size = selectSize();

	wmark = (size / 4) - 1;

	if (size == 1024)
		size = size * BULK_BURST * SIZE_MULT;
//	if ((mode > 0) && (size > 512))
//		size = size / 2;

	size = (size / 4) - 1;

	for (i = 6; i >= 1; i--)
		CyU3PGpifSocketConfigure((i-1) & 3, 0x100 + (i-1), size, CyTrue, 1);
//   CyU3PGpifSocketConfigure(0, CY_U3P_PIB_SOCKET_4, 2, CyTrue, 1);
//	CyU3PGpifSocketConfigure(1, CY_U3P_PIB_SOCKET_5, 2, CyTrue, 1);
//	CyU3PGpifSocketConfigure(0, CY_U3P_PIB_SOCKET_0, 2, CyTrue, 1);
//	CyU3PGpifSocketConfigure(1, CY_U3P_PIB_SOCKET_1, 2, CyTrue, 1);
//	CyU3PGpifSocketConfigure(2, CY_U3P_PIB_SOCKET_2, 2, CyTrue, 1);
//	CyU3PGpifSocketConfigure(3, CY_U3P_PIB_SOCKET_3, 2, CyTrue, 1);
	CyU3PGpioSimpleSetValue(45, CyFalse);

	//CyU3PGpifDisable(CyTrue);

	uint8_t bFlags = 0;
	switch (mode)
	{
	case -1:
	case 0:
		bFlags = 250;
		break;
	case 1:
		bFlags = 229;
		break;
	case 2:
		bFlags = 225;
		break;
	case 3:
		bFlags = 237;
		break;
	case 4:
		bFlags = 224;
		break;
	case 5:
		bFlags = 239;
		break;
	default:
		bFlags = 254;
		break;
	}

	g_Channels = 0;
	for (i = 6; i >= 1; i--)
	{
		//CyU3PGpifSocketConfigure((i-1) & 3, 0x100 + (i-1), 127, (CyBool_t)((bFlags >> (i-1)) & 1), 1);
		g_Channels += (((bFlags >> (i-1)) & 1) << ((i*3) + BIT_OUT)) + (1 << ((i*3) + BIT_ON));
	}

//	CyU3PDeviceGpioOverride(21, CyTrue);
//	CyU3PDeviceGpioOverride(22, CyTrue);
//	CyU3PDeviceGpioOverride(25, CyTrue);
//	CyU3PDeviceGpioOverride(26, CyTrue);

//	gpioConfig.driveHighEn = CyTrue;
//	gpioConfig.driveLowEn = CyTrue;
//	gpioConfig.inputEn = CyFalse;
//	gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
//	gpioConfig.outValue = CyTrue;
//	CyU3PGpioSetSimpleConfig(21, &gpioConfig);
//	gpioConfig.outValue = CyFalse;
//	CyU3PGpioSetSimpleConfig(22, &gpioConfig);
//	gpioConfig.outValue = CyTrue;
//	CyU3PGpioSetSimpleConfig(25, &gpioConfig);
//	gpioConfig.outValue = CyFalse;
//	CyU3PGpioSetSimpleConfig(26, &gpioConfig);

	if (mode > 0)
	{
		CyU3PGpifSocketConfigure(0, CY_U3P_PIB_SOCKET_4, wmark, (CyBool_t)((bFlags >> 4) & 1), 1);
		CyU3PGpifSocketConfigure(1, CY_U3P_PIB_SOCKET_5, wmark, (CyBool_t)((bFlags >> 5) & 1), 1);
		CyU3PGpifSocketConfigure(0, CY_U3P_PIB_SOCKET_0, 127, (CyBool_t)(bFlags&1), 1);
		CyU3PGpifSocketConfigure(1, CY_U3P_PIB_SOCKET_1, 127, (CyBool_t)((bFlags >> 1) & 1), 1);
	}
	else
	{
		CyU3PGpifSocketConfigure(0, CY_U3P_PIB_SOCKET_4, 127, (CyBool_t)((bFlags >> 4) & 1), 1);
		CyU3PGpifSocketConfigure(1, CY_U3P_PIB_SOCKET_5, 127, (CyBool_t)((bFlags >> 5) & 1), 1);
		CyU3PGpifSocketConfigure(0, CY_U3P_PIB_SOCKET_0, wmark, (CyBool_t)(bFlags&1), 1);
		CyU3PGpifSocketConfigure(1, CY_U3P_PIB_SOCKET_1, wmark, (CyBool_t)((bFlags >> 1) & 1), 1);
	}
	CyU3PGpifSocketConfigure(2, CY_U3P_PIB_SOCKET_2, 127, (CyBool_t)((bFlags >> 2) & 1), 1);
	CyU3PGpifSocketConfigure(3, CY_U3P_PIB_SOCKET_3, 127, (CyBool_t)((bFlags >> 3) & 1), 1);
	ChansSet(mode);

	CyU3PGpioSimpleSetValue(45, CyTrue);

    CyU3PDmaBufferFree(outputBuf.buffer);
    outputBuf.buffer = (uint8_t*)CyU3PDmaBufferAlloc(1024);
    rBuf.buffer = (uint8_t*)CyU3PDmaBufferAlloc(1024);
    rBuf.size = 1024;
    CyU3PDmaChannelReset(&g_SlFifoUtoP);
	CyU3PMemSet(outputBuf.buffer, 0, 40);
	outputBuf.buffer[5] = 0x80;
	outputBuf.buffer[14] = 0x01;
	outputBuf.buffer[21] = 0x04;
	outputBuf.buffer[22] = 0x02;
	outputBuf.buffer[29] = 0x08;
	outputBuf.buffer[30] = 0x03;
	outputBuf.buffer[37] = 0x0C;
	outputBuf.buffer[38] = 0x04;
	outputBuf.count = 40;
	outputBuf.size = 1024;
	outputBuf.status = 0;

//	CyU3PDeviceGpioRestore(21);
//	CyU3PDeviceGpioRestore(22);
//	CyU3PDeviceGpioRestore(25);
//	CyU3PDeviceGpioRestore(26);
//	CyU3PGpioDeInit();
//	CyU3PGpioInit(&rGpioClk, NULL);
////	CyU3PGpifDisable(CyTrue);
////	CyU3PGpifLoad (&CyFxGpifConfig);
////	CyU3PGpifSMStart (RESET, ALPHA_RESET);
//	CyU3PDeviceGpioOverride(45, CyTrue);
//	gpioConfig.outValue = CyTrue;
//	CyU3PGpioSetSimpleConfig(45, &gpioConfig);
//
//	CyU3PGpifSocketConfigure(0, CY_U3P_PIB_SOCKET_4, size, (CyBool_t)((bFlags >> 4) & 1), 1);
//	CyU3PGpifSocketConfigure(1, CY_U3P_PIB_SOCKET_5, size, (CyBool_t)((bFlags >> 5) & 1), 1);
//	CyU3PGpifSocketConfigure(0, CY_U3P_PIB_SOCKET_0, wmark, (CyBool_t)((bFlags >> 3) & 1), 1);
//	CyU3PGpifSocketConfigure(1, CY_U3P_PIB_SOCKET_1, wmark, (CyBool_t)((bFlags >> 3) & 1), 1);
//	CyU3PGpifSocketConfigure(2, CY_U3P_PIB_SOCKET_2, size, (CyBool_t)((bFlags >> 2) & 1), 1);
//	CyU3PGpifSocketConfigure(3, CY_U3P_PIB_SOCKET_3, size, (CyBool_t)((bFlags >> 3) & 1), 1);

	CyU3PDmaChannelSetupSendBuffer(&g_SlFifoUtoP, &outputBuf);
	SetMessage();
	CyU3PDmaChannelGetBuffer(&g_SlFifoPtoU, &rBuf, 10);
	CyU3PDmaChannelReset(&g_SlFifoUtoP);
	CyU3PDmaChannelSetXfer(&g_SlFifoUtoP, DMA_TX_SIZE);
	CyU3PDmaChannelDiscardBuffer(&g_SlFifoPtoU);

	if (g_mode1 == -1)
	{
		CyU3PDmaChannelReset(&g_SlFifoUtoP);
		CyU3PMemSet(outputBuf.buffer, 0, 32);
		CyU3PMemSet(rBuf.buffer, 0xFF, 32);
		outputBuf.count = 8;
		outputBuf.status = 0;
		outputBuf.buffer[4] = 8;
		outputBuf.buffer[5] = 0xC0;
		CyU3PDmaChannelSetupSendBuffer(&g_SlFifoUtoP, &outputBuf);
		SetMessage();
		CyU3PDmaChannelGetBuffer(&g_SlFifoPtoU, &rBuf, 10);
		CyU3PDmaChannelDiscardBuffer(&g_SlFifoPtoU);

		for (i = 0; i < 10; i++)
		{
			CyU3PMemSet(outputBuf.buffer, 0, 32);
			CyU3PMemSet(rBuf.buffer, 0xFF, 32);
			CyU3PDmaChannelReset(&g_SlFifoUtoP);
			outputBuf.buffer[0] = 1;
			outputBuf.buffer[4] = 8;
			outputBuf.buffer[5] = 0xC0;
			outputBuf.buffer[8] = 3;
			outputBuf.buffer[12] = 8;
			outputBuf.buffer[13] = 0xC0;
			outputBuf.buffer[16] = 7;
			outputBuf.buffer[20] = 8;
			outputBuf.buffer[21] = 0xC0;
	//		outputBuf.buffer[28] = 0x10;
	//		outputBuf.buffer[29] = 0x40;
			outputBuf.count = 24;
			CyU3PDmaChannelSetupSendBuffer(&g_SlFifoUtoP, &outputBuf);
	//		for (j = 0; j < 10; j++)
	//		{
				SetMessage();
				CyU3PDmaChannelGetBuffer(&g_SlFifoPtoU, &rBuf, 10);
				CyU3PDmaChannelDiscardBuffer(&g_SlFifoPtoU);
	//			if (CyU3PDmaChannelGetBuffer(&g_SlFifoPtoU, &rBuf, 10) == CY_U3P_SUCCESS)
	//				break;
	//		}
			CyU3PMemSet(outputBuf.buffer, 0, 32);
			CyU3PDmaChannelReset(&g_SlFifoUtoP);
			outputBuf.buffer[4] = 0x10;
			outputBuf.buffer[5] = 0x40;
			outputBuf.count = 8;
			CyU3PDmaChannelSetupSendBuffer(&g_SlFifoUtoP, &outputBuf);
			SetMessage();
			CyU3PDmaChannelGetBuffer(&g_SlFifoPtoU, &rBuf, 10);
			if ((rBuf.buffer[0] & 1) == 1)
				break;
			else
				CyU3PDmaChannelDiscardBuffer(&g_SlFifoPtoU);
		}
		CyU3PDmaChannelDiscardBuffer(&g_SlFifoPtoU);
		CyU3PMemSet(outputBuf.buffer, 0, 8);
		CyU3PDmaChannelDiscardBuffer(&g_SlFifoPtoU);
		outputBuf.count = 8;
		outputBuf.buffer[0] = 0x0F;
		outputBuf.buffer[4] = 8;
		outputBuf.buffer[5] = 0xC0;
		CyU3PDmaChannelReset(&g_SlFifoUtoP);
		CyU3PDmaChannelSetupSendBuffer(&g_SlFifoUtoP, &outputBuf);
	//	rBuf.status = 0;
	//	rBuf.size = 1024;
		for (i = 0; i < 10; i++)
		{
			SetMessage();
	//		CyU3PDmaChannelGetBuffer(&g_SlFifoPtoU, &rBuf, 10);
			if (CyU3PDmaChannelGetBuffer(&g_SlFifoPtoU, &rBuf, 10) == CY_U3P_SUCCESS)
				break;
			CyU3PThreadSleep(800);
			//CyU3PDmaChannelDiscardBuffer(&g_SlFifoPtoU);
		}
		CyU3PDmaChannelDiscardBuffer(&g_SlFifoPtoU);
		CyU3PDmaChannelReset(&g_SlFifoUtoP);
		CyU3PDmaChannelSetXfer(&g_SlFifoUtoP, DMA_TX_SIZE);
	}
	//CyU3PDmaBufferFree(rBuf.buffer);
	g_mode1 = g_mode;
}

void SetSockId(CyU3PDmaChannelConfig_t * dmaCfg, CyBool_t sockType, uint16_t sockId)
{
	if (sockType == CyTrue)
		dmaCfg->prodSckId = sockId;
	else
		dmaCfg->consSckId = sockId;
}

void SetDmaCfg(CyBool_t dir, uint64_t mode, uint8_t ep, uint16_t sockId1, uint16_t sockId2, uint16_t sockId3, uint16_t sockId4, CyU3PDmaChannel* dmaChan, uint16_t size, uint8_t dmaMode)
{
	CyU3PDmaChannelConfig_t dmaCfg;
	CyU3PMemSet((uint8_t *)&dmaCfg, 0, sizeof(dmaCfg));
	if (((ep == EP_CONS_2) || (ep == EP_PROD_2)) && (g_mode == -1))
		dmaCfg.count = 1;
	else
		dmaCfg.count = BUF_COUNT;
	dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
	dmaCfg.size = size;
	if ((g_mode == -1) && (ep == EP_PROD_2))
	{
		dmaCfg.cb = DataU2GCB;
		dmaCfg.notification = CY_U3P_DMA_CB_PROD_EVENT;
	}
	if (((1 << (g_mode+1)) & mode) != 0)
		SetSockId(&dmaCfg, dir, sockId1);
	else
		if (((1 << (g_mode+1)) & (mode >> 16)) != 0)
			SetSockId(&dmaCfg, dir, sockId2);
		else
			if (((1 << (g_mode+1)) & (mode >> 32)) != 0)
				SetSockId(&dmaCfg, dir, sockId3);
	SetSockId(&dmaCfg, (~dir)&1, sockId4);
	if ((((1 << (g_mode+1)) & mode) != 0) || (((1 << (g_mode+1)) & (mode >> 16)) != 0) || (((1 << (g_mode+1)) & (mode >> 32)) != 0))
	{
		CyU3PUsbSetEpNak(ep, CyFalse);
		CyU3PDmaChannelCreate (dmaChan, dmaMode, &dmaCfg);
//		CyU3PUsbStall(ep, CyFalse, CyTrue);
//		CyU3PUsbResetEp(ep);
		CyU3PUsbFlushEp(ep);
		CyU3PDmaChannelSetXfer(dmaChan, DMA_TX_SIZE);
	}
	else
		CyU3PUsbSetEpNak(ep, CyTrue);
}

void ChansSet(int mode)
{
	g_mode = mode;
	uint8_t dmaMode = 0;
	uint16_t size = 64;
	uint64_t modes = 0;

	size = selectSize();

	//command OUT
	modes = 2 + (1 << 8) + (1 << 16) + ((uint64_t)124 << 32);
	if (mode == -1)
		dmaMode = CY_U3P_DMA_TYPE_MANUAL_IN;
	else
		dmaMode = CY_U3P_DMA_TYPE_MANUAL;
	SetDmaCfg(CyFalse, modes, EP_PROD_1, CY_U3P_PIB_SOCKET_1, CY_U3P_CPU_SOCKET_CONS, CY_U3P_PIB_SOCKET_5, PROD_SOCKET_1, &g_SlFifoUtoP, size, dmaMode);

	//command IN
	if (mode == -1)
		dmaMode = CY_U3P_DMA_TYPE_MANUAL_OUT;
	else
		dmaMode = CY_U3P_DMA_TYPE_MANUAL;
	SetDmaCfg(CyTrue, modes, EP_CONS_1, CY_U3P_PIB_SOCKET_0, CY_U3P_CPU_SOCKET_PROD, CY_U3P_PIB_SOCKET_4, CONS_SOCKET_1, &g_SlFifoPtoU, size, dmaMode);

	if (size == 1024)
		size = size * BULK_BURST * SIZE_MULT;

//	if ((mode > 0) && (size > 512))
//		size = size / 2;



	// 1st data OUT
	modes = 2 + (92 << 16) + ((uint64_t)1 << 32);
	if (mode == -1)
		dmaMode = CY_U3P_DMA_TYPE_AUTO_SIGNAL;
	else
		dmaMode = CY_U3P_DMA_TYPE_AUTO;
	SetDmaCfg(CyFalse, modes, EP_PROD_2, CY_U3P_PIB_SOCKET_3, CY_U3P_PIB_SOCKET_0, CY_U3P_PIB_SOCKET_1, PROD_SOCKET_2, &g_SlFifoUtoP2, size, dmaMode);

	// 1st data IN
	modes = 2 + (28 << 16) + ((uint64_t)33 << 32);
	SetDmaCfg(CyTrue, modes, EP_CONS_2, CY_U3P_PIB_SOCKET_2, CY_U3P_PIB_SOCKET_1, CY_U3P_PIB_SOCKET_0, CONS_SOCKET_2, &g_SlFifoPtoU2, size, CY_U3P_DMA_TYPE_AUTO);

	// 2nd data OUT

	modes = 1 + (20 << 16) + ((uint64_t)64 << 32);
	SetDmaCfg(CyFalse, modes, EP_PROD_3, CY_U3P_PIB_SOCKET_3, CY_U3P_PIB_SOCKET_2, CY_U3P_PIB_SOCKET_1, PROD_SOCKET_3, &g_SlFifoUtoP3, size, CY_U3P_DMA_TYPE_AUTO);

	// 2nd data IN
	modes = 4 + (9 << 16) + ((uint64_t)32 << 32);
	SetDmaCfg(CyTrue, modes, EP_CONS_3, CY_U3P_PIB_SOCKET_3, CY_U3P_PIB_SOCKET_2, CY_U3P_PIB_SOCKET_1, CONS_SOCKET_3, &g_SlFifoPtoU3, size, CY_U3P_DMA_TYPE_AUTO);

	// 3rd data OUT
	modes = (1 << 5) + (64 << 16);
	SetDmaCfg(CyFalse, modes, EP_PROD_4, CY_U3P_PIB_SOCKET_3, CY_U3P_PIB_SOCKET_2, 0, PROD_SOCKET_4, &g_SlFifoUtoP4, size, CY_U3P_DMA_TYPE_AUTO);

	// 3rd data IN
	modes = 8 + (32 << 16);
	SetDmaCfg(CyTrue, modes, EP_CONS_4, CY_U3P_PIB_SOCKET_3, CY_U3P_PIB_SOCKET_2, 0, CONS_SOCKET_4, &g_SlFifoPtoU4, size, CY_U3P_DMA_TYPE_AUTO);

	// 4th data OUT
	modes = 64;
	SetDmaCfg(CyFalse, modes, EP_PROD_5, CY_U3P_PIB_SOCKET_5, 0, 0, PROD_SOCKET_5, &g_SlFifoUtoP5, size, CY_U3P_DMA_TYPE_AUTO);

	// 4th data IN
	modes = 32;
	SetDmaCfg(CyTrue, modes, EP_CONS_5, CY_U3P_PIB_SOCKET_3, 0, 0, CONS_SOCKET_5, &g_SlFifoPtoU5, size, CY_U3P_DMA_TYPE_AUTO);
}
