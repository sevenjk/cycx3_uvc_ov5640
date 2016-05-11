/*
## Cypress Image Sensor Routines for XXXX sensor (cyu3imagesnesor.c)
## ===========================
##
##  Copyright Cypress Semiconductor Corporation, 2013,
##  All Rights Reserved
##  UNPUBLISHED, LICENSED SOFTWARE.
##
##  CONFIDENTIAL AND PROPRIETARY INFORMATION
##  WHICH IS THE PROPERTY OF CYPRESS.
##
##  Use of this file is governed
##  by the license agreement included in the file
##
##     <install>/license/license.txt
##
##  where <install> is the Cypress software
##  installation root directory path.
##
## ===========================
*/


#include <cyu3error.h>
#include <cyu3i2c.h>
#include "cycx3_m10mo.h"
#include <cyu3utils.h>
#include "cyu3spi.h"
#include "cycx3_uvc.h"
#include "cyu3dma.h"
#include "cyu3uart.h"
#include "cyu3gpio.h"
#include "m10mo_fw.h"

/* Misc Macros; Fill the correct I2C addresses below */
#define M10MO_I2C_READ_ADDRESS         (0x3F) 
#define M10MO_I2C_WRITE_ADDRESS        (0x3E) 

#define SENSOR_DEBUG	1

CyU3PReturnStatus_t I2C_SensorRead(uint8_t *pSendData, uint8_t count, uint8_t *buf)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;
    uint8_t cnt=0;
	//CyU3PDebugPrint(2, "[I2C_SensorRead]uSendCmd [0x%x,0x%x,0x%x,0x%x]\r\n", pSendData[0], pSendData[1],pSendData[2],pSendData[3]);

    for(cnt=0; cnt<1 ; cnt++)
    {
    	preamble.length = 7;
		preamble.buffer[0] = M10MO_I2C_WRITE_ADDRESS; /* Slave address: write operation */
        preamble.buffer[1] = pSendData[0];//count+3;
        preamble.buffer[2] = pSendData[1];//0x01;
        preamble.buffer[3] = pSendData[2];//CY_U3P_GET_MSB (regAddr);	/* category*/
		preamble.buffer[4] = pSendData[3];//CY_U3P_GET_LSB (regAddr);	/* Bytes*/
		preamble.buffer[5] = count-1;
		preamble.buffer[6] = M10MO_I2C_READ_ADDRESS;
		preamble.ctrlMask = 0x0020;

        status = CyU3PI2cReceiveBytes (&preamble, buf, count,0);
        CyU3PThreadSleep(100);
        if (status == CY_U3P_SUCCESS)
        {
            break;
        }
        else
            CyU3PDebugPrint(4,"\r\nM10MOSensorRead Failed status=0x%x\r\n",status);
    }
    return status;
}

CyU3PReturnStatus_t I2C_SensorReadMem(uint8_t count, uint8_t *buf)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;
    uint8_t cnt=0;
	//CyU3PDebugPrint(2, "[I2C_SensorRead]uSendCmd [0x%x,0x%x,0x%x,0x%x]\r\n", pSendData[0], pSendData[1],pSendData[2],pSendData[3]);

    for(cnt=0; cnt<1 ; cnt++)
    {
    	preamble.length = 1;
		preamble.buffer[0] = M10MO_I2C_READ_ADDRESS; /* Slave address: write operation */
		preamble.ctrlMask = 0x0000;

        status = CyU3PI2cReceiveBytes (&preamble, buf, count,0);
        CyU3PThreadSleep(100);
        if (status == CY_U3P_SUCCESS)
        {
            break;
        }
        else
            CyU3PDebugPrint(4,"\r\nM10MOSensorReadMem Failed status=0x%x\r\n",status);
    }
    return status;
}

CyU3PReturnStatus_t I2C_SensorWrite(uint16_t regAddr, uint16_t count, uint8_t *buf)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;
    uint8_t cnt=0;

    for(cnt=0; cnt<1 ; cnt++)
    {
		preamble.buffer[0] = M10MO_I2C_WRITE_ADDRESS; /* Slave address: write operation */
        preamble.buffer[1] = buf[0];//count+4;
        preamble.buffer[2] = buf[1];	/* command id*/	
        preamble.buffer[3] = CY_U3P_GET_MSB (regAddr);//(uint8_t)buf[3];	/* category*/
		preamble.buffer[4] = CY_U3P_GET_LSB (regAddr);//(uint8_t)buf[4];	/* Bytes*/
        preamble.length = 5;
        preamble.ctrlMask = 0x0000;
	
        status = CyU3PI2cTransmitBytes (&preamble, buf+4, count,0);
//        CyU3PThreadSleep(1);
        if (status == CY_U3P_SUCCESS)
        {
            break;
        }
        else
            CyU3PDebugPrint(4,"\r\n CyU3PI2cTransmitBytes Failed status=%x\r\n",status);
#if 1	
		/* Wait for the write to complete. */
		preamble.length = 1;
		status = CyU3PI2cWaitForAck(&preamble, 200);
		if (status != CY_U3P_SUCCESS)
			CyU3PDebugPrint(4,"\r\nCyU3PI2cWaitForAck Failed status=0x%x\r\n",status);
#endif		
    }
    return status;
}

/*-------------  Read / Write M10MO Category Parameter  ----------*/
static uint8_t M10MO_read_category_parameter_ram(uint8_t category, uint8_t byte)
{
	uint8_t uSendCmd[5] = 
	{
		0x05, 
		COMMAND_ID_READ_CATEGORY_PARAMETER, 
		category, 
		byte, 
		1
	};
	uint8_t uGetData[2] = {0};

	uGetData[1] = 0xff;
	I2C_SensorRead(uSendCmd, 2, uGetData);

	return uGetData[1];
}

static uint16_t M10MO_read_category_parameter_2byte(uint8_t category, uint8_t byte)
{
	uint8_t uSendCmd[5] = 
	{
		0x05, 
		COMMAND_ID_READ_CATEGORY_PARAMETER, 
		category, 
		byte, 
		2
	};

	uint8_t uGetData[4] = {0};
	uint16_t byte2return = 0;
	uGetData[0] = 0xff;
	uGetData[1] = 0xff;
	uGetData[2] = 0xff;

	I2C_SensorRead(uSendCmd, 3, uGetData);

#ifdef SENSOR_DEBUG
		CyU3PDebugPrint (2, "[M10MO_read_category_parameter_2byte]uSendCmd [0x%x, 0x%x,0x%x,0x%x,0x%x], read resulte [0x%x, 0x%x,0x%x]\r\n", uSendCmd[0], uSendCmd[1],uSendCmd[2],uSendCmd[3],uSendCmd[4],uGetData[0],uGetData[1],uGetData[2]);
#endif
	return byte2return = (uGetData[1]<<8 | uGetData[2]);
	
}

static uint32_t M10MO_read_category_parameter_4Byte(uint8_t category, uint8_t byte)
{
	uint32_t uReturnValue = 0;
	uint8_t uSendCmd[5] = 
	{
		0x05, 
		COMMAND_ID_READ_CATEGORY_PARAMETER, 
		category, 
		byte, 
		4
	};
	uint8_t uGetData[5] = {0};

	I2C_SensorRead(uSendCmd, 5, uGetData);

	uReturnValue = ((uint32_t)uGetData[1] << 24)
				    | ((uint32_t)uGetData[2] << 16)
				    | ((uint32_t)uGetData[3] << 8)
				    | ((uint32_t)uGetData[4]) ;
	
	//CyU3PDebugPrint (4, "[M10MO_read_category_parameter_4Byte]uReturnValue = [0x%x]\r\n", uReturnValue); 
	return uReturnValue;
}

void M10MO_Read_Memory(uint32_t addr, uint16_t count, uint8_t *data)
{
	uint8_t uSendCmd[] = 
	{
		0x0, 
		COMMAND_ID_READ_MEMORY_8BIT_ACCESS, 
		(uint8_t)((addr>>24)&0xFF),
		(uint8_t)((addr>>16)&0xFF),
		(uint8_t)((addr>>8)&0xFF),
		(uint8_t)(addr&0xFF),
		(uint8_t)((count>>8)&0xFF),
		(uint8_t)(count&0xFF),
	};

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "[M10MO_Write_Memory]uSendCmd [0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x]\r\n",
			uSendCmd[0], uSendCmd[1],uSendCmd[2],uSendCmd[3],
			uSendCmd[4], uSendCmd[5],uSendCmd[6],uSendCmd[7]);
#endif

	I2C_SensorWrite((uint16_t) ((addr >> 16) & 0xffff), 5, uSendCmd);
	I2C_SensorReadMem(addr, count);
}

static void M10MO_write_category_parameter_ram(uint8_t category, uint8_t byte, uint8_t uPara)
{
	uint8_t uSendCmd[5] =
	{
		0x05, 
		COMMAND_ID_WRITE_CATEGORY_PARAMETER, 
		category, 
		byte, 
		uPara
	};
	
	uint16_t regAddr;
	regAddr = ((uint16_t)uSendCmd[2]<<8)|((uint16_t)uSendCmd[3]);

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "[M10MO_write_category_parameter_ram]uSendCmd [0x%x, 0x%x,0x%x,0x%x,0x%x]\r\n", uSendCmd[0], uSendCmd[1],uSendCmd[2],uSendCmd[3],uSendCmd[4]);
#endif
	I2C_SensorWrite(regAddr, 1, uSendCmd);
}

static void M10MO_write_category_parameter_2Byte(uint8_t category, uint8_t byte, uint16_t uPara)
{
	uint8_t uSendCmd[8] = 
	{
		0x06, 
		COMMAND_ID_WRITE_CATEGORY_PARAMETER, 
		category, 
		byte,
		(uint8_t)((uPara>>8)&0xFF),
		(uint8_t)(uPara&0xFF)
	};
	uint16_t regAddr;
	regAddr = ((uint16_t)uSendCmd[2]<<8)|((uint16_t)uSendCmd[3]);

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "[MS2R_write_category_parameter_2Byte]uSendCmd [0x%x, 0x%x,0x%x,0x%x,0x%x]\r\n", uSendCmd[0], uSendCmd[1],uSendCmd[2],uSendCmd[3],uPara);
#endif
	//CyU3PDebugPrint(2, "[MS2R_write_category_parameter_2Byte]regAddr=0x%x\r\n", regAddr);

	I2C_SensorWrite(regAddr, 2, uSendCmd);
}

static void M10MO_write_category_parameter_4Byte(uint8_t category, uint8_t byte, uint32_t uPara)
{
	uint8_t uSendCmd[8] = 
	{
		0x08, 
		COMMAND_ID_WRITE_CATEGORY_PARAMETER, 
		category, 
		byte,
		(uint8_t)((uPara>>24)&0xFF),
		(uint8_t)((uPara>>16)&0xFF),
		(uint8_t)((uPara>>8)&0xFF),
		(uint8_t)(uPara&0xFF)
	};
	uint16_t regAddr;
	regAddr = ((uint16_t)uSendCmd[2]<<8)|((uint16_t)uSendCmd[3]);

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "[MS2R_write_category_parameter_4Byte]uSendCmd [0x%x, 0x%x,0x%x,0x%x,0x%x]\r\n", uSendCmd[0], uSendCmd[1],uSendCmd[2],uSendCmd[3],uPara);
#endif
	//CyU3PDebugPrint(2, "[MS2R_write_category_parameter_4Byte]regAddr=0x%x\r\n", regAddr);

	I2C_SensorWrite(regAddr, 4, uSendCmd);
}

static void M10MO_Write_Memory(uint32_t addr, uint16_t count, uint8_t data)
{
	uint8_t uSendCmd[9] = 
	{
		0x0, 
		COMMAND_ID_WRITE_MEMORY_8BIT_ACCESS, 
		(uint8_t)((addr>>24)&0xFF),
		(uint8_t)((addr>>16)&0xFF),
		(uint8_t)((addr>>8)&0xFF),
		(uint8_t)(addr&0xFF),
		(uint8_t)((count>>8)&0xFF),
		(uint8_t)(count&0xFF),
		data
	};

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "[M10MO_Write_Memory]uSendCmd [0x%x, 0x%x,0x%x,0x%x,0x%x]\r\n", uSendCmd[0], uSendCmd[1],uSendCmd[2],uSendCmd[3],data);
#endif

	I2C_SensorWrite((uint16_t) ((addr >> 16) & 0xffff), 5, uSendCmd);
}

/* Application critical error handler */
void
CyCx3_M10MO_ErrorHandler (
        CyU3PReturnStatus_t status        /* API return status */
        )
{
    /* Application failed with the error code status */

    /* Add custom debug or recovery actions here */
	CyU3PDebugPrint(4,"\r\nEntering Error Handler, code will get stuck here..status = 0x%x",status);
    /* Loop indefinitely */
    for (;;)
    {
        /* Thread sleep : 100 ms */
        CyU3PThreadSleep (100);
    }
}

static void M10MO_SetAddress(uint32_t Addr)
{
#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(4, "*******Enter M10MO_SetAddress Function \r\n");
#endif

	M10MO_write_category_parameter_4Byte(0x0F, 0x00, Addr);

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(4, "*******Exist M10MO_SetAddress Function\r\n");
#endif
}

static void M10MO_SetProgramSize()
{
#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(4, "*******Enter M10MO_SetProgramSize Function \r\n");
#endif

	M10MO_write_category_parameter_2Byte(0x0F, 0x04, 0x0000);

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(4, "*******Exist M10MO_SetProgramSize Function\r\n");
#endif
}

static void M10MO_SetProgramSize_s(int size)
{
#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(4, "*******Enter M10MO_SetProgramSize Function \r\n");
#endif

	M10MO_write_category_parameter_2Byte(0x0F, 0x04, size);

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(4, "*******Exist M10MO_SetProgramSize Function\r\n");
#endif
}

static void M10MO_SetSIOMode(void)
{
#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(4, "*******Enter M10MO_SetSIOMode Function \r\n");
#endif

	M10MO_write_category_parameter_ram(0x0F, 0x4B, 0x44);

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(4, "*******Exist M10MO_SetSIOMode Function\r\n");
#endif
}

static void M10MO_SetPll(void)
{
	uint32_t PllValue = 0;
#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(4, "*******Enter M10MO Setting Pll clock\r\n");
#endif

	M10MO_write_category_parameter_4Byte(0x0F, 0x1C, 0x00170141);
#if 0
	PllValue = M10MO_read_category_parameter_4Byte(0x0F, 0x1c);
	CyU3PDebugPrint (4, "M10MO_SetPll read pll value = 0x%x\r\n",
		PllValue);
#endif

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(4, "*******Exist M10MO Setting Pll clock\r\n");
#endif
}

static void M10MO_SartRAM(void)
{
	uint8_t RamStatus = 0;
#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(4, "*******Enter M10MO_StartRAM Function \r\n");
#endif

	M10MO_write_category_parameter_ram(0x0F, 0x4A, 0x01);
#if 0
	RamStatus = M10MO_read_category_parameter_ram(0x0F, 0x4A);
	CyU3PDebugPrint (4, "M10MO_SartRAM read RAM Status = 0x%x\r\n",
		RamStatus);
#endif

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(4, "*******Exist M10MO_StartRAM Function\r\n");
#endif
}

CyU3PReturnStatus_t Set_SSNLineValue(uint8_t gpio, CyBool_t gpioState)
{
	//CyBool_t gpioState;
	CyU3PReturnStatus_t status;
    //gpioState = 0;
    status = CyU3PGpioSetValue (gpio, gpioState);
    if (status != CY_U3P_SUCCESS)
    {
        /* Error handling */
        CyU3PDebugPrint (4, "CyU3PGpioSetValue failed, error code = %d\n\r",
        		status);
        CyCx3_M10MO_ErrorHandler(status);
    }

    return status;
}

CyU3PReturnStatus_t M10MO_SIO_Transfer()
{
	CyU3PReturnStatus_t status;
	uint8_t temp[1024];
	uint8_t location[4];
	uint32_t ISPFirmwareSize = M10MO_FW_SIZE, readlen = 0, transferlen = 1024;


    location[0] = 0x03; /* Read command */
    uint32_t DestAddress = 0x20000000;

    location[1] = (DestAddress >> 16) & 0xFF;       /* MS byte */
    location[2] = (DestAddress >> 8) & 0xFF;
    location[3] = DestAddress & 0xFF;               /* LS byte */

    CyU3PDebugPrint(4,"\n\rISPFirmwareSize left %d", ISPFirmwareSize);
    while (ISPFirmwareSize > 0)
    {
		location[1] = (DestAddress >> 16) & 0xFF;       /* MS byte */
		location[2] = (DestAddress >> 8) & 0xFF;
		location[3] = DestAddress & 0xFF;               /* LS byte */

    	if(ISPFirmwareSize > transferlen)
    	{

    		readlen = transferlen;
    		CyU3PDebugPrint(4,"\n\rreadlen %d", readlen);
    	}
    	else
    	{
    		readlen = ISPFirmwareSize;
    		CyU3PDebugPrint(4,"\n\rreadlen %d", readlen);
    	}

		CyU3PSpiSetSsnLine (CyFalse);

		CyU3PDebugPrint(4,"\n\rSending read command 0x%X 0x%X 0x%X 0x%X ", location[0], location[1], location[2], location[3]);
		status = CyU3PSpiTransmitWords (location, 4);
        if (status != CY_U3P_SUCCESS)
        {
            CyU3PDebugPrint (2, "\r\nSPI Write command failed");
            CyU3PSpiSetSsnLine (CyTrue);
            break;
        }
        CyU3PDebugPrint(4,"\n\rreading SPI Flash ");

		status = CyU3PSpiReceiveWords (temp,readlen);
		if (status != CY_U3P_SUCCESS)
		{
			CyU3PDebugPrint (2, "\n\rSPI Read failed 0x%X",status);
			break;
		}
		else
		{
			ISPFirmwareSize -= readlen;
			DestAddress += readlen;


			CyU3PSpiSetSsnLine (CyTrue);

			Set_SSNLineValue(54,CyFalse);
			CyU3PBusyWait(10);

			status = CyU3PSpiTransmitWords (temp, readlen);
			if (status != CY_U3P_SUCCESS)
			{
				CyU3PDebugPrint (2, "\n\rSPI Write failed 0x%X",status);
				break;
			}
			CyU3PDebugPrint(4,"\n\rISPFirmwareSize left %d", ISPFirmwareSize);


			CyU3PBusyWait(10);
			Set_SSNLineValue(54,CyTrue);
		}

    }

    CyU3PDebugPrint(4,"\n\rFinished ISP Firmware download");

    CyU3PSpiSetSsnLine (CyTrue);
    Set_SSNLineValue(54,CyTrue);

    CyU3PThreadSleep(150);

    //CyCx3_M10MO_SetJPEGQFactor(95);
//    CyCx3_M10MO_Flip(0x00);
    //CyCx3_M10MO_LED(0x02, 50);
    //CyCx3_M10MO_SetColorMode(0x00);
    //CyCx3_M10MO_SetAEFrameRateControl(1);
    //CyCx3_M10MO_SetPresetExposure(0);
    //CyCx3_M10MO_BlackWhiteThreshold(0x0);

    return status;
}

static void M10MO_ChipErase(void)
{
#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "*******Enter M10MO_ChipErase Function \r\n");
#endif

	M10MO_write_category_parameter_ram(0x0F, 0x06, 0x02);

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "*******Exist M10MO_ChipErase Function\r\n");
#endif
}

static void M10MO_CheckEraseStatus(void)
{
	uint8_t erase_status = 0;
	uint8_t poll_cnt = 0;

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "*******Enter M10MO_CheckEraseStatus Function \r\n");
#endif

	do{
	  CyU3PThreadSleep(100);
	  erase_status = M10MO_read_category_parameter_ram(0x0F, 0x06);
	  poll_cnt++;
#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "Erase Staus = [0x%x] poll_cnt = %d;\r\n", erase_status, poll_cnt);
#endif
	}while(erase_status != 0x00);

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "*******Exist M10MO_CheckEraseStatus Function\r\n");
#endif
}

static CyU3PReturnStatus_t M10MO_SendFirmware(uint8_t *buf, int len)
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	CyU3PI2cPreamble_t preamble;
	uint8_t cnt=0;

	for(cnt=0; cnt<1 ; cnt++)
	{
		preamble.buffer[0] = M10MO_I2C_WRITE_ADDRESS; /* Slave address: write operation */
		preamble.buffer[1] = 0x00;
		preamble.buffer[2] = 0x04;
		preamble.buffer[3] = 0x01;
		preamble.buffer[4] = 0x10;
		preamble.buffer[5] = 0x00;
		preamble.buffer[6] = 0x00;
		preamble.length = 7;
		preamble.ctrlMask = 0x0000;
	
		status = CyU3PI2cTransmitBytes (&preamble, buf, len, 0);
		if (status == CY_U3P_SUCCESS)
		{
			break;
		}
		else
			CyU3PDebugPrint(4,"\r\n CyU3PI2cTransmitBytes Failed status=%x\r\n",status);

		/* Wait for the write to complete. */
		preamble.length = 1;
		status = CyU3PI2cWaitForAck(&preamble, 200);
		if (status != CY_U3P_SUCCESS)
			CyU3PDebugPrint(4,"\r\nCyU3PI2cWaitForAck Failed status=0x%x\r\n",status);
	}
	return status;
}

static void M10MO_DoProgram(void)
{
#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "*******Enter M10MO_DoProgram Function \r\n");
#endif

	M10MO_write_category_parameter_ram(0x0F, 0x07, 0x01);

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "*******Exist M10MO_DoProgram Function\r\n");
#endif
}

static void M10MO_CheckPGMStatus(void)
{
	uint8_t Program_status = 0;
	uint8_t poll_cnt = 0;

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "*******Enter M10MO_CheckPGMStatus Function \r\n");
#endif

	do{
	  CyU3PThreadSleep(100);
	  Program_status = M10MO_read_category_parameter_ram(0x0F, 0x07);
	  poll_cnt++;
#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "Program Staus = [%x] poll_cnt = %d;\r\n", Program_status, poll_cnt);
#endif
	}while(Program_status != 0x00);

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "*******Exist M10MO_CheckPGMStatus Function\r\n");
#endif
}

static void M10MO_SetFlashCtrl(void)
{
#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "\r\n*******Enter M10MO_SetFlashCtrl\r\n");
#endif

	M10MO_Write_Memory(0x13000005, 0x0001, 0x7F);

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "*******Exist M10MO_SetFlashCtrl\r\n");
#endif
}

static void M10MO_CalCheckSum(void)
{
#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "*******Enter M10MO_CalCheckSum Function \r\n");
#endif

	M10MO_write_category_parameter_ram(0x0F, 0x09, 0x04);

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "*******Exist M10MO_CalCheckSum Function\r\n");
#endif
}

static void M10MO_CheckSumStatus(void)
{
	uint8_t checksum_status = 0;
	uint8_t poll_cnt = 0;

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "*******Enter M10MO_CheckSumStatus Function \r\n");
#endif

	do{
	  CyU3PThreadSleep(100);
	  checksum_status = M10MO_read_category_parameter_ram(0x0F, 0x09);
	  poll_cnt++;
#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "checksum_status = [%x] poll_cnt = %d;\r\n", checksum_status, poll_cnt);
#endif
	}while(checksum_status != 0x00);

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "*******Exist M10MO_CheckSumStatus Function\r\n");
#endif
}

static uint16_t M10MO_GetCheckSum(void)
{
	uint16_t checksum_value = 0;
	uint8_t poll_cnt = 0;

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "*******Enter M10MO_GetCheckSum Function \r\n");
#endif
    checksum_value = M10MO_read_category_parameter_2byte(0x0F, 0x0A);
#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "checksum_value = [%x] poll_cnt = %d;\r\n", checksum_value, poll_cnt);
#endif
#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "*******Exist M10MO_GetCheckSum Function\r\n");
#endif

	return (checksum_value);
}

CyU3PReturnStatus_t CyCx3_M10MO_PowerOn()
{
	CyU3PReturnStatus_t status;
    CyU3PGpioSimpleConfig_t gpioConfig;

    /* power enable */
    status = CyU3PDeviceGpioOverride (22, CyTrue);
    if (status != CY_U3P_SUCCESS)
    {
        /* Error handling */
        CyU3PDebugPrint (4, "GPIO Override Failed, code=%d\r\n", status);
		return status;
    }

    gpioConfig.outValue    = CyTrue;
    gpioConfig.inputEn     = CyFalse;
    gpioConfig.driveLowEn  = CyTrue;
    gpioConfig.driveHighEn = CyTrue;
    gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
    status = CyU3PGpioSetSimpleConfig(22, &gpioConfig);
	if (status != CY_U3P_SUCCESS)
    {
        /* Error handling */
        CyU3PDebugPrint (4, "GPIO Override Failed, code=%d\r\n", status);
		return status;
    }
	
    status = CyU3PDeviceGpioOverride (26, CyTrue);
    if (status != CY_U3P_SUCCESS)
    {
        /* Error handling */
        CyU3PDebugPrint (4, "GPIO Override Failed, code=%d\r\n", status);
		return status;
    }

    gpioConfig.outValue    = CyTrue;
    gpioConfig.inputEn     = CyFalse;
    gpioConfig.driveLowEn  = CyTrue;
    gpioConfig.driveHighEn = CyTrue;
    gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
    status = CyU3PGpioSetSimpleConfig(26, &gpioConfig);
	if (status != CY_U3P_SUCCESS)
    {
        /* Error handling */
        CyU3PDebugPrint (4, "GPIO Override Failed, code=%d\r\n", status);
    }

	return status;
}

void CyCx3_M10MO_PowerOff()
{
    CyU3PReturnStatus_t RetStatus = CY_U3P_SUCCESS;

	RetStatus = CyU3PGpioSetValue (CYCX3_GPIO_M10MO_VDD1_2, CyFalse);
	if (RetStatus != CY_U3P_SUCCESS)
	{
		/* Error handling */
		CyU3PDebugPrint (4, "CyU3PGpioSetValue1 failed, error code = %d\r\n",
			RetStatus);
		CyCx3_M10MO_ErrorHandler(RetStatus);
	}
	CyU3PThreadSleep(50);
	RetStatus = CyU3PGpioSetValue (CYCX3_GPIO_M10MO_VDD1_8, CyFalse);
	if (RetStatus != CY_U3P_SUCCESS)
	{
		/* Error handling */
		CyU3PDebugPrint (4, "CyU3PGpioSetValue2 failed, error code = %d\r\n",
			RetStatus);
		CyCx3_M10MO_ErrorHandler(RetStatus);
	}
			/* Wait for two seconds */
	CyU3PThreadSleep(50);

}

CyU3PReturnStatus_t CyCx3_M10MO_SIOInit()
{
	CyU3PReturnStatus_t status;
//	g_M10MO_YUV_Mode=SYS_MODE_PRE_INITIALIZATION;

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "Start to Download M10MO Firmware Through SPI\r\n");
#endif

	CyU3PThreadSleep(10);
	M10MO_SetAddress(0x20000000);
	M10MO_SetProgramSize();
	M10MO_SetSIOMode();
	M10MO_SetPll();
	M10MO_SartRAM();
	CyU3PThreadSleep(10);

	status = M10MO_SIO_Transfer();

	M10MO_SetAddress(0x00000000);
	M10MO_ChipErase();
	M10MO_CheckEraseStatus();
	M10MO_DoProgram();
	M10MO_CheckPGMStatus();
	M10MO_SetFlashCtrl();
	M10MO_CalCheckSum();
	M10MO_CheckSumStatus();
	M10MO_GetCheckSum();

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "CyCx3_M10MO_SIOInit End\n\r");
#endif
//	g_M10MO_YUV_Mode=SYS_MODE_INITIALIZATION;

	return status;

}

CyU3PReturnStatus_t CyCx3_M10MO_I2CInit()
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	uint8_t PrgmCnt = 0;
	uint32_t PrgmAddr = 0x00000000;

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "Start to Download M10MO Firmware Through I2C\r\n");
#endif
	M10MO_SetPll();
	M10MO_SartRAM();
	CyU3PThreadSleep(3);
	M10MO_SetAddress(0x00000000); 
	M10MO_ChipErase();
	M10MO_CheckEraseStatus();
	for(PrgmCnt=0; PrgmCnt<32; PrgmCnt++)
	{
		M10MO_SetAddress(PrgmAddr);
		M10MO_SetProgramSize();
//		M10MO_SendFirmware();
		PrgmAddr = PrgmAddr + 0x00010000; 
		M10MO_DoProgram();
		M10MO_CheckPGMStatus();
	}
	M10MO_SetFlashCtrl();
	M10MO_CalCheckSum();
	M10MO_CheckSumStatus();
	M10MO_GetCheckSum();

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "M10MO_DownloadFW End\r\n");
#endif
	CyCx3_M10MO_PowerOff();

	return status;

}

void M10MO_ColorBar_Output(void)
{
	uint8_t resp[2];

	CyU3PDebugPrint(2, "\r\n*******Enter M10MO_ColorBar_Output Function\r\n");
	
	CyU3PDebugPrint(4, "Grace###########################prepare output colorbar\r\n");
//	CyU3PThreadSleep(500);
	M10MO_write_category_parameter_ram(0x01, 0x01, 0x17);
	M10MO_write_category_parameter_ram(0x0D, 0xEE, 0x01);

	resp[1] = M10MO_read_category_parameter_ram (0x00, 0x0b);
	CyU3PDebugPrint(2, "status check after color bar resp[1] = 0x[%x]\r\n", resp[1]);

	//Stop ISP Test pattern
	CyU3PThreadSleep(1000);
	M10MO_write_category_parameter_ram(0x0D, 0xEE, 0x00);
#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "*******Exist M10MO_ColorBar_Output Function\r\n");
#endif
}

static void M10MO_StartFW(void)
{
	uint16_t m10mo_status = 0;
	uint8_t poll_cnt = 0;

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(4, "*******Enter M10MO_StartFW Function \r\n");
#endif

	M10MO_write_category_parameter_ram(0x0F, 0x12, 0x01);
#if 1
	do{
	  CyU3PThreadSleep(50);
	  m10mo_status = M10MO_read_category_parameter_ram (0x00, 0x1C);
	  poll_cnt++;
	  CyU3PDebugPrint(4, "%s INT_FACTOR= [%x] poll_cnt = %d;\r\n", __func__,m10mo_status,poll_cnt);
	}while((m10mo_status & 0xff)!=0x00);
#endif

#ifdef SENSOR_DEBUG
	CyU3PDebugPrint(2, "*******Exist M10MO_StartFW Function\r\n");
#endif
}

void M10MO_prep()
{
	M10MO_SetPll();
	M10MO_SartRAM();
	CyU3PThreadSleep(3);
	M10MO_SetAddress(0x00000000); 
	M10MO_ChipErase();
	M10MO_CheckEraseStatus();
}

void M10MO_program(int block, uint8_t *buf)
{
	M10MO_SetAddress(block * M10MO_FLASH_BLOCK_SIZE);
	M10MO_SetProgramSize_s(M10MO_FLASH_BLOCK_SIZE);
	M10MO_SendFirmware(buf, M10MO_FLASH_BLOCK_SIZE + 2);
	M10MO_DoProgram();
	M10MO_CheckPGMStatus();
}

int M10MO_finalize()
{
	M10MO_SetFlashCtrl();
	M10MO_CalCheckSum();
	M10MO_CheckSumStatus();
	return M10MO_GetCheckSum();
}

void CyCx3_M10MO_StartFW()
{
	CyCx3_M10MO_PowerOn();
	M10MO_SetFlashCtrl();
	M10MO_StartFW();
}
#if 0
CyU3PReturnStatus_t Wait_for_CameraModeTransition()
{
	uint8_t RDBuff[2];
	uint8_t count = 100;
	CyU3PReturnStatus_t status;

	status = I2C_SensorRead(0xF002, 1 , RDBuff);

	while ((RDBuff[0] != 0x81)&(count>0) )
	{
		CyU3PDebugPrint(4,"\n\r0xF002: status 0x%X retry %d", RDBuff[0], count);
		RDBuff[0] = 0x00;
		CyU3PThreadSleep(500);
		status = I2C_SensorRead(0xF002, 1 , RDBuff);
		count--;

	}
	CyU3PDebugPrint(4,"\n\r0xF002: status 0x%X", RDBuff[0]);

	RDBuff[0] = 0x00;
	count = 100;
	status = I2C_SensorRead(0xF011, 1 , RDBuff);

	while ((RDBuff[0] != 0x01)&(count>0) )
	{
		CyU3PDebugPrint(4,"\n\r0xF011: status 0x%X retry %d", RDBuff[0], count);
		RDBuff[0] = 0x00;
		CyU3PThreadSleep(500);
		status = I2C_SensorRead(0xF011, 1 , RDBuff);
		count--;

	}
	CyU3PDebugPrint(4,"\n\r0xF011: status 0x%X", RDBuff[0]);

	return status;
}
#endif

CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_VGA_USB3()
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	
	M10MO_write_category_parameter_ram (0x01, 0x01, 0x17);

	return status;
}

CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_720p_USB3()
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	
	M10MO_write_category_parameter_ram (0x01, 0x01, 0x21);

	return status;

}

CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_720p60_USB3()
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	
	M10MO_write_category_parameter_ram (0x01, 0x01, 0x25);

	return status;

}

CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_1080p_USB3()
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	
	M10MO_write_category_parameter_ram (0x01, 0x01, 0x28);

	return status;

}

CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_1080p60_USB3()
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	
	M10MO_write_category_parameter_ram (0x01, 0x01, 0x2B);

	return status;

}

CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_VGA_USB2()
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	
	M10MO_write_category_parameter_ram (0x01, 0x01, 0x17);

	return status;

}

CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_720p_USB2()
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	
	M10MO_write_category_parameter_ram (0x01, 0x01, 0x21);

	return status;

}

CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_1080p_USB2()
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	
	M10MO_write_category_parameter_ram (0x01, 0x01, 0x28);

	return status;
}

CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_13M_USB3()

{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	
	M10MO_write_category_parameter_ram (0x0B, 0x01, 0x39);

	return status;

}

CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_13M_USB2()
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	
	M10MO_write_category_parameter_ram (0x0B, 0x01, 0x39);

	return status;

}

CyU3PReturnStatus_t CyCx3_M10MO_Set_Mirror_Flip(uint8_t Flip)
{

	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    CyU3PDebugPrint(2, "[Enter]:CyCx3_M10MO_Set_Mirror_Flip:Flip value=%d\r\n",Flip);

    switch (Flip)
	{
	    case IMAGE_NORMAL:
			M10MO_write_category_parameter_ram(0x02, 0x05, 0x00);
			M10MO_write_category_parameter_ram(0x02, 0x06, 0x00);
			M10MO_write_category_parameter_ram(0x02, 0x07, 0x00);
			M10MO_write_category_parameter_ram(0x02, 0x08, 0x00);
	        break;
		case IMAGE_V_MIRROR:
			M10MO_write_category_parameter_ram(0x02, 0x05, 0x01);
			M10MO_write_category_parameter_ram(0x02, 0x06, 0x00);
			M10MO_write_category_parameter_ram(0x02, 0x07, 0x01);
			M10MO_write_category_parameter_ram(0x02, 0x08, 0x00);
			break;
		case IMAGE_H_MIRROR:
			M10MO_write_category_parameter_ram(0x02, 0x05, 0x00);
			M10MO_write_category_parameter_ram(0x02, 0x06, 0x01);
			M10MO_write_category_parameter_ram(0x02, 0x07, 0x00);
			M10MO_write_category_parameter_ram(0x02, 0x08, 0x01);
			break;
	    case IMAGE_HV_MIRROR:
			M10MO_write_category_parameter_ram(0x02, 0x05, 0x01);
			M10MO_write_category_parameter_ram(0x02, 0x06, 0x01);
			M10MO_write_category_parameter_ram(0x02, 0x07, 0x01);
			M10MO_write_category_parameter_ram(0x02, 0x08, 0x01);
	        break;
    }

//	CyU3PDebugPrint("[Exist]:CyCx3_M10MO_Set_Mirror_Flip func End\r\n");

	return status;
}

CyU3PReturnStatus_t CyCx3_M10MO_SetColorMode(uint8_t ColorMode)
{


#ifdef SENSOR_DEBUG
//    CyU3PDebugPrint(4,"\r\n Sensor Initialization start");
#endif

    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if ((ColorMode == 0x00)|(ColorMode == 0x01)|(ColorMode == 0x03)|(ColorMode == 0x10))
    {
//    	I2C_Sensor_RegWrite(0xF051, ColorMode);
    }

return status;
}

CyU3PReturnStatus_t CyCx3_M10MO_BlackWhiteThreshold(uint8_t Threshold)
{


#ifdef SENSOR_DEBUG
//    CyU3PDebugPrint(4,"\r\n Sensor Initialization start");
#endif

    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if ((Threshold >= 0x00)&(Threshold <= 0xFF))
    {
//    	I2C_Sensor_RegWrite(0xF058, Threshold);
    }

return status;
}

uint16_t packetcount = 0;

CyU3PReturnStatus_t CyCx3_M10MO_AF_Single_Focus()
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

	CyU3PDebugPrint(2, "CYCX3_M10M0_AF_single_focus--------> IN \r\n");

	//AF normal mode
	M10MO_write_category_parameter_ram(0x0A, 0x00, 0x01);
	//AF Rang mode -0x01:INF,0x02:Normal,0x03:Bar code, 0x04:Macro position
	M10MO_write_category_parameter_ram(0x0A, 0x01, 0x02);
	//start AF
	M10MO_write_category_parameter_ram(0x0A, 0x02, 0x01);
	CyU3PDebugPrint(2, "CYCX3_M10MO_AF_single_focus--------> OUT \r\n");

	return status;
}

CyU3PReturnStatus_t CyCx3_M10MO_AF_Continuous_Focus()
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    CyU3PDebugPrint(2, "CYCX3_M10M0_AF_Continuous_focus--------> IN \r\n");

	//Continuous AF mode
	M10MO_write_category_parameter_ram(0x0A, 0x00, 0x06);
	//AF Rang mode -0x01:INF,0x02:Normal,0x03:Bar code, 0x04:Macro position
	M10MO_write_category_parameter_ram(0x0A, 0x49, 0x00);
	//start AF
	M10MO_write_category_parameter_ram(0x0A, 0x02, 0x01);
	CyU3PDebugPrint(2, "CYCX3_M10MO_AF_single_focus--------> OUT \r\n");

	return status;
}
