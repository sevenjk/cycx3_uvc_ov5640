/*
 ## Cypress FX3 Core Library Header (cycx3_m10mo.h)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2014,
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
 *  Created on: 17-Aug-2015
 *  Author: nikl
 */

/* CX3 Header file for APIs implementing Image Sensor specific functions
 */
#ifndef _INCLUDED_CYCX3_M10MO_H_
#define _INCLUDED_CYCX3_M10MO_H_

#include <cyu3types.h>

/** \file cycx3_m10mo.h
    \brief API declarations for configuring Socinext ISP M10MO, connected to CX3 over I2C bus.
    This file is applicable to CX3-Socinext ISP RDK.
 */
#define M10MO_FLASH_BLOCK_SIZE		0x2000	/* 8KB */
#define M10MO_FLASH_BLOCKS			256		/* total: 8KB * 256 = 2MB */

#define COMMAND_ID_READ_CATEGORY_PARAMETER		(0x01)
#define COMMAND_ID_WRITE_CATEGORY_PARAMETER		(0x02)
#define COMMAND_ID_READ_MEMORY_8BIT_ACCESS		(0x03)
#define COMMAND_ID_WRITE_MEMORY_8BIT_ACCESS		(0x04)

enum
{
    IMAGE_NORMAL=0,
    IMAGE_H_MIRROR,
    IMAGE_V_MIRROR,
    IMAGE_HV_MIRROR 
};

typedef enum
{
	SYS_MODE_POWEROFF,
	SYS_MODE_PRE_INITIALIZATION,
	SYS_MODE_INITIALIZATION,
	SYS_MODE_PARAMETER_SETTING,
	SYS_MODE_MONITOR,
	SYS_MODE_PRE_CAPTURE,
	SYS_MODE_SINGLE_CAPTURE,
	SYS_MODE_AUTO_FOCUS,
	SYS_MODE_VIDEO,
	SYS_MODE_MAX

}M10MO_SYS_MODE;

#define CYCX3_GPIO_M10MO_VDD1_8	22
#define CYCX3_GPIO_M10MO_VDD1_2	26

//volatile M10MO_SYS_MODE g_M10MO_YUV_Mode=SYS_MODE_PARAMETER_SETTING;

/******ISP State Initialization and State Control Functions******/
/* Initialize Image Sensor*/
extern CyU3PReturnStatus_t CyCx3_M10MO_Init();

/* Power on image sensor */
extern CyU3PReturnStatus_t CyCx3_M10MO_PowerOn();

/* Put Image Sensor to Sleep/Low Power Mode */
extern CyU3PReturnStatus_t CyCx3_M10MO_Sleep();

/* Wake Image Sensor from Sleep/Low Power Mode to Active Mode */
extern CyU3PReturnStatus_t CyCx3_M10MO_Wakeup();

/************************THine ISP Video Streaming Functions******************/
/* Configure Sensor for UYVY_3M (2048x1536) in USB 3.0*/
/*Description: API to set 3M UYVY in USB3 mode. Binning/Resize are supported for this resolution
 *Parameter: Pass binning_3M_UYVY as: 1 for binning, 0 for resize*/
//extern CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_3M_USB3(CyBool_t binning_3M_UYVY);

/* Configure Sensor for UYVY_13M (4160x3120) in USB 3.0*/
extern CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_13M_USB3();

/* Configure Sensor for UYVY_VGA (640x480) in USB 3.0*/
extern CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_VGA_USB3();

/* Configure Sensor for UYVY_720p (1280x720) at 30 FPS in USB 3.0*/
extern CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_720p_USB3();

/* Configure Sensor for UYVY_720p (1280x720) at 60FPS in USB 3.0*/
extern CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_720p60_USB3();

/* Configure Sensor for UYVY_1080p (1920x1080) at 30 FPS in USB 3.0*/
/*Description: API to set 1080p UYVY at 30 FPS in USB3 mode. Binning/Resize are supported for this resolution
 *Parameter: Pass binning_1080p_UYVY as: 1 for binning, 0 for resize*/
//extern CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_1080p_USB3(uint8_t binning_1080p_UYVY);
extern CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_1080p_USB3();

/* Configure Sensor for UYVY_1080p (1920x1080) at 60 FPS in USB 3.0*/
extern CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_1080p60_USB3();

/* Configure Sensor for UYVY_4164x3120 at 10 FPS in USB 3.0*/
extern CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_4164x3120_USB3();

/* Configure Sensor for MJPEG_3M in USB 3.0*/
/*Description: API to set 3M UYVY at 30 FPS in USB3 mode. Binning/Resize are supported for this resolution
 *Parameter: Pass binning_3M_UYVY as: 1 for binning, 0 for resize*/
//extern CyU3PReturnStatus_t CyCx3_M10MO_Set_MJPEG_3M_USB3(CyBool_t binning_3M_MJPEG);

/* Configure Sensor for MJPEG_13M in USB 3.0*/
extern CyU3PReturnStatus_t CyCx3_M10MO_Set_MJPEG_13M_USB3();

/* Configure Sensor for MJPEG_VGA in USB 3.0*/
extern CyU3PReturnStatus_t CyCx3_M10MO_Set_MJPEG_VGA_USB3();

/* Configure Sensor for MJPEG_720p in USB 3.0*/
extern CyU3PReturnStatus_t CyCx3_M10MO_Set_MJPEG_720p_USB3();

/* Configure Sensor for MJPEG_720p60 in USB 3.0*/
extern CyU3PReturnStatus_t CyCx3_M10MO_Set_MJPEG_720p60_USB3();

/* Configure Sensor for MJPEG_1080p in USB 3.0*/
/*Description: API to set 1080p MJPEG at 30 FPS in USB3 mode. Binning/Resize are supported for this resolution
 *Parameter: Pass binning_1080p_MJPEG as: 1 for binning, 0 for resize*/
extern CyU3PReturnStatus_t CyCx3_M10MO_Set_MJPEG_1080p_USB3(CyBool_t binning_1080p_MJPEG);

/* Configure Sensor for UYVY_1080p60 in USB 3.0*/
extern CyU3PReturnStatus_t CyCx3_M10MO_Set_MJPEG_1080p60_USB3();

/* Configure Sensor for 1080p in USB 2.0*/
/*Description: API to set 1080p UYVY at 5 FPS in USB2 mode. Binning/Resize are supported for this resolution
 *Parameter: Pass binning_1080p_UYVY as: 1 for binning, 0 for resize*/
//extern CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_1080p_USB2(CyBool_t binning_1080p_UYVY);
extern CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_1080p_USB2();

/* Configure Sensor for 720p in USB 2.0*/
extern CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_720p_USB2();

/* Configure Sensor for UYVY_VGA in USB 2.0*/
extern CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_VGA_USB2();


/* Configure Sensor for UYVY_13M in USB 2.0*/
extern CyU3PReturnStatus_t CyCx3_M10MO_Set_UYVY_13M_USB2();


/* Configure Sensor for MJPEG_1080p (1920x1080) in USB 2.0*/
/*Description: API to set 1080p MJPEG at 30 FPS in USB3 mode. Binning/Resize are supported for this resolution
 *Parameter: Pass binning_1080p_MJPEG as: 1 for binning, 0 for resize*/
extern CyU3PReturnStatus_t CyCx3_M10MO_Set_MJPEG_1080p_USB2(CyBool_t binning_1080p_MJPEG);

/* Configure Sensor for MJPEG_VGA (640x480) in USB 2.0*/
extern CyU3PReturnStatus_t CyCx3_M10MO_Set_MJPEG_VGA_USB2();

/* Configure Sensor for MJPEG_720p (1280x720) in USB 2.0*/
extern CyU3PReturnStatus_t CyCx3_M10MO_Set_MJPEG_720p_USB2();

/* Configure Sensor for MJPEG_13M (4160x3120) in USB 2.0*/
extern CyU3PReturnStatus_t CyCx3_M10MO_Set_MJPEG_13M_USB2();


/************************THine ISP UVC Video Control APIs**************************/
/************************UVC Video Control - Camera Terminal***********************/
/*These features are supported through regular UVC Video Control Interface*/
/** \brief : Exposure Mode (Auto, Preset Manual)

    **Description**\n
		Set Exposure mode to auto or manual by using this API.

	**Parameters**\n
		AEControl:
			0 - Normal Auto Exposure is performed
			1 - Exposure Setting is locked
			2 - Exposure is set to manual mode. In this setting the exposure
			value set using CyCx3_M10MO_SetPresetExposure API takes affect.

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_AEControl(uint8_t AEControl);

/** \brief : Preset Exposure (Manual) Setting

    **Description**\n
		Apply exposure value in Manual Exposure mode.

	**Parameters**\n
		PresetExposure:
			0-11 (12 Settings, 0 for Darker scene and 11 for Brighter scene)

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_SetPresetExposure(uint8_t PresetExposure);


/** \brief : Focus Status

    **Description**\n
		To know the Focus status (either in Auto mode or in Manual mode) use this API. This API should be called after Auto Focus mode change or after Manual focus setting.

	**Parameters**\n
		None.

    **Return value**\n
		CY_U3P_SUCCESS - if the focus is successful.\n
		* CY_U3P_ERROR_TIMEOUT - if the focus process timed out.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_Focus_Status();

/** \brief : Auto Focus Enable Setting

    **Description**\n
		To enable Auto Focus use this API. After calling this API, call API CyCx3_M10MO_Focus_Status to verify Focus Status.
		Example Usage: 	CyCx3_M10MO_Autofocus_Start();
						CyCx3_M10MO_Focus_Status();

	**Parameters**\n
		None.

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.

 */
extern CyU3PReturnStatus_t CyCx3_M10MO_Autofocus_Start();


/** \brief : Auto Focus Disable Setting

    **Description**\n
		To disable Auto Focus use this API. Note that Auto Focus needs to be disabled before Manual Focus setting is applied.

	**Parameters**\n
		None.

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.

 */
extern CyU3PReturnStatus_t CyCx3_M10MO_Autofocus_Stop();



/** \brief : Manual Focus Setting

    **Description**\n
		Apply focus position in Manual Focus mode. Note that Auto Focus needs to be disabled (using CyCx3_M10MO_Autofocus_Stop API) before Manual Focus setting is applied.
		Example Usage: 	CyCx3_M10MO_Autofocus_Stop;
						CyCx3_M10MO_ManualFocusControl(30);
						CyCx3_M10MO_Focus_Status();

	**Parameters**\n
		ManualFocusControl:
			100-3000 (mm unit for Manual Focus Position)

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.

 */
extern CyU3PReturnStatus_t CyCx3_M10MO_ManualFocusControl(uint16_t ManualFocusControl);


/** \brief : Aperture Piority Control (Low light compensation)

    **Description**\n
		To set the Aperture Priority use this API.

	**Parameters**\n
		AEFrameRateControl:
			0 for Auto adjust Frame Rate (Dependent on scene light , ISP adjust the frame rate).
				  Auto adjust Frame Rate gives better Video/Images
			1 for Fix Frame Rate

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_AEFrameRateControl(uint8_t AEFrameRateControl);



/** \brief : Roll Control

    **Description**\n
		To Roll the video frame use this API. Note that this API supports Flip and Mirror options.

	**Parameters**\n
		Flip:
			bit 0: 0 for Normal, 1 for Flip
			bit 1: 0 for Normal, 1 for Mirror
			NOTE: To Roll the frame by 180 degree, the frame has to flipped and mirrored (both bits should be set)

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_Flip(uint8_t Flip);



/************************UVC Video Control - Processing Unit Terminal***********************/

/** \brief : Backlight Compensation Setting

    **Description**\n
		To implement backlight compensation use this API.
		NOTE: When Document Scanner mode is selected, backlight compensation setting is ignored.

	**Parameters**\n
		BackLightCompControl:
			0 - Auto (Backlight compensation OFF)
			1 - Backlight compensation ON in Normal mode

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_BackLightCompControl(uint8_t BackLightCompControl);


/** \brief : Brightness Control Setting

    **Description**\n
		To implement brightness control use this API.

	**Parameters**\n
		BrightnessControl:
			0 - 0x14 values supported; 21 steps (0x0A for normal, 0x00 is darkest setting, 0x14 for brightest setting)

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_BrightnessControl(uint8_t BrightnessControl);


/** \brief : Contrast Control Setting

    **Description**\n
		To implement contrast control use this API.

	**Parameters**\n
		ContrastControl:
			0 - 0x14 values supported; 21 steps (0x0A for normal, 0x00 is lowest contrast, 0x14 for highest contrast)

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_ContrastControl(uint8_t ContrastControl);


/** \brief : Hue Control Setting

    **Description**\n
		To implement Hue control use this API.

	**Parameters**\n
		HueControl:
			0 - 0x14 values supported; 11 steps of resolution 2(0x0A for 0 degree(Normal Hue), 0x00 for -180 degree, 0x02 for -144 degree, 0x14 for 180 degree)

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_HueControl(uint8_t HueControl);


/** \brief : Saturation Control Setting

    **Description**\n
		To implement Saturation control use this API.

	**Parameters**\n
		SaturationControl:
			0x05 - 0x0F values supported; 11 steps (0x0A for Normal Saturation, 0x05 for Weakest Saturation, 0x0F for Strongest Saturation)

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_SaturationControl(uint8_t SaturationControl);


/** \brief : Sharpness Control Setting

    **Description**\n
		To implement Sharpness control use this API.

	**Parameters**\n
		SharpnessControl:
			0x05 - 0x0F values supported; 11 steps (0x0A for Normal Sharpness, 0x05 for Weakest Sharpness, 0x0F for Strongest Sharpness)

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_SharpnessControl(uint8_t SharpnessControl);


/** \brief : Gamma Control Setting

    **Description**\n
		To implement Gamme control use this API.

	**Parameters**\n
		GammaControl:
			100 - 200 values supported; 11 steps of resolution 10, with 160 as default

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_GammaControl(uint8_t GammaControl);


/** \brief : Flicker Cancellation Setting

    **Description**\n
		To implement Flicker Cancellation of Power Line Frequency use this API.

	**Parameters**\n
		PLFreqControl:
			0 if Flicker Cancel Frequency is 50 Hz
			1 if Flicker Cancel Frequency is 60 Hz

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_PLFreqControl(uint8_t PLFreqControl);


/** \brief : White Balance Mode Selection

    **Description**\n
		To select White Balance Mode (Auto/Manual) use this API.

	**Parameters**\n
		AutoWBControl:
			0 for Auto
			0x10 for Manual Setting (in this mode the White Balance Temperature Manual setting is set using CyCx3_M10MO_ManualWhiteBalanceControl API)

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_AutoWhiteBalanceControl(uint8_t AutoWBControl);


/** \brief : White Balance Temperature Setting

    **Description**\n
		To set White Balance Temperature in Manual Mode use this API.

	**Parameters**\n
		ManualWBControl:
			20-80 values supported (100k Unit, i.e. 28 means 2800K)

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_ManualWhiteBalanceControl(uint8_t ManualWBControl);


/***********************THine ISP non-UVC Video Control APIs***************/
/*These features are not supported by UVC specification for Video Control.
 *Hence an additional HID interface was added to firmware to support these
 *features as UVC Extension features*/

/** \brief : Auto Exposure Compensation Control

    **Description**\n
		To set Auto Exposure Compensation Control use this API. This setting is applicable only in Auto Exposure mode.

	**Parameters**\n
		ExpCompControl:
			0x00 - 0x0C values supported; 13 steps (0x00 for (-6/3) EV, 0x0C for (+6/3) EV, default setting is 0x06 (0 EV))

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_ExposureCompensationControl(uint8_t ExpCompControl);

/** \brief : Auto Focus Mode Control

    **Description**\n
		To select the Auto Focus mode (One shot/ Continuous) use this API. Call
		Example Usage: 	CyCx3_M10MO_AutoFocusModeControl(0);
						CyCx3_M10MO_Focus_Status();


	**Parameters**\n
		AFModeControl:
			0 for One shot AF
			3 for Continuous AF for Picture

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_AutoFocusModeControl(uint8_t AFModeControl);


/** \brief : Auto Focus Area Control

    **Description**\n
		To select the Auto Focus Area (Center Weighted/ User Select/ Document Scanner) use this API. If User Select (customer area) mode is set
		CyCx3_M10MO_AutoFocusWindowControl API is used to set the AF Window first.
		Example Usage: To set the AF area as Upper-Right Corner while streaming VGA, follow the sequence below
						CyCx3_M10MO_AutoFocusWindowControl(600, 640, 1, 50);
        				CyCx3_M10MO_AutoFocusAreaControl(3);
        				CyCx3_M10MO_Focus_Status();

	**Parameters**\n
		AFAreaControl:
			0 for Center Weighted
			3 for User Select. In this mode, API CyCx3_M10MO_AutoFocusWindowControl is be used to set the AF Window.

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_AutoFocusAreaControl(uint8_t AFAreaControl);


/** \brief : Auto Focus Window Control

    **Description**\n
		To set the Auto Focus Window use this API
		NOTE: Maximum AF area is same size of current picture size.
		Example Usage: To set the AF area as Upper-Right Corner while streaming VGA, follow the sequence below
				CyCx3_M10MO_AutoFocusWindowControl(600, 640, 1, 50);
				CyCx3_M10MO_AutoFocusAreaControl(3);
				CyCx3_M10MO_Focus_Status();

	**Parameters**\n
		AFWindow_H_Start:
			Starting Horizontal Pixel position of Auto-Focus Window.
		AFWindow_H_End:
			Ending Horizontal Pixel position of Auto-Focus Window.
		AFWindow_V_Start:
			Starting Vertical Line position of Auto-Focus Window.
		AFWindow_V_End:
			Ending Vertical Line position of Auto-Focus Window.


    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_AutoFocusWindowControl(uint16_t AFWindow_H_Start,uint16_t AFWindow_H_End,uint16_t AFWindow_V_Start,uint16_t AFWindow_V_End);


/** \brief : Color Mode Selection

    **Description**\n
		To set the Color Mode (Normal/ Monochrome/ Negative/ Black and White) use this API

	**Parameters**\n
		ColorMode:
			0 for Normal
			1 for Monochrome
			3 for Negative
			10 for Black and White. While in this mode, use CyCx3_M10MO_BlackWhiteThreshold API to set the Black and White threshold

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_SetColorMode(uint8_t ColorMode);

/** \brief : Black and White Threshold Setting

    **Description**\n
		To set the Black and white threshold while in Black and White mode use this API. Note that this
		setting is applicable when the color mode is set to Black and White.

	**Parameters**\n
		Threshold:
			0 for Auto
			0x01 - 0xFF for fixing Threshold value

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_BlackWhiteThreshold(uint8_t Threshold);

/** \brief : Noise Reduction Control

    **Description**\n
		To set the Noise Reduction (Auto/Fix Value) use this API.

	**Parameters**\n
		NoiseReductionLevel:
			bit 7: 0 for Auto, 1 for Fix
			bit 6-0: 0-10 (applicable when bit 7 is 1; 0 for weak, 5 for normal, 10 for strong)

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_NoiseReductionControl(uint8_t NoiseReductionLevel);

/** \brief : Scene Mode Control

    **Description**\n
		To select the Scene mode (Normal/ Document Scanner) use this API.

	**Parameters**\n
		SceneModeControl:
			0 for Normal
			0x20 for Document Scanner

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_SceneModeControl(uint8_t SceneModeControl);

/** \brief : Max Frame Rate Control

    **Description**\n
		To select the Scene mode (Normal/ Document Scanner) use this API.

	**Parameters**\n
		MaxFrameRateControl:
			0 for Disable (Frame rate applicable as per Resolution configuration)
			3-119 for applying max frame rate (10 means 10 FPS or slower)

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_MaxFrameRateControl(uint8_t MaxFrameRateControl);

/** \brief : JPEG Q-Factor Control

    **Description**\n
		To set the JPEG Quality Factor (compression ratio) use this API.

	**Parameters**\n
		AutoControl:
			0 for Disable Auto Control by ISP. In this case QFactor setting will be applied.
			1 for Enabling Auto Control by ISP. In this case QFactor setting will be ignored.
		QFactor:
			1 - 97 value supported

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.
 */
extern CyU3PReturnStatus_t CyCx3_M10MO_SetJPEGQFactor(CyBool_t AutoControl, uint8_t QFactor);


/** \brief : Crop Resolution

    **Description**\n
		To obtain a new resolution by cropping a Full frame use this API. For e.g. 960x800 is not
		natively achieved by THine ISP. So 960x800 can be obtained by cropping 1280x720 Full frame.
		Example Usage: For obtaining UYVY 960x800 resolution from 1280x960 resolution use this API as shown below:
		 				CyCx3_M10MO_Crop_Resolution( 0x07, 960, 800, 0, 1);

	**Parameters**\n
		FullResID: This is Full Frame resolution on which the cropping is being performed
			0x03 for 640x480
			0x05 for 800x600
			0x07 for 1280x960
			0x08 for 1280x1024
			0x09 for 1600x1200
			0x0A for 1280x720
			0x0B for 1920x1080
			0x0D for 3840x2160
			0x10 for 2048x1536
			0x11 for 2592x1944
			0x12 for 3264x2448
			0x14 for 4160x3120
			0x20 for 2052x1536
			0x24 for 4164x3120
		CropResWidth: Cropping Resolution width (Horiontal Pixels).
			Conditions: 1) This value must be less than Full resolution width
						2) This value must be even number. Otherwise this is
						   rounded down to even number.
						3) If the output format is MJPEG, this value must be a multiple
						   of 16, otherwise this is rounded down to a multiple of 16.
						4) Minimum Crop width is 32
		CropResWidth: Cropping Resolution height (Verticle lines).
			Conditions: 1) This value must be less than Full resolution height
						2) If the output format is MJPEG, this value must be a multiple
						   of 8, otherwise this is rounded down to a multiple of 8.
						4) Minimum Crop height is 32

		isMJPEG: 1 if output if MJPEG format
				 0 is output is UYVY format

		isUSB3: 1 if the connection is USB 3.0 (Different MIPI clocks are used for USB 3.0 and USB 2.0. Hence appropriate connection information is required.)
				0 if the connection is USB 2.0

    **Return value**\n
		CY_U3P_SUCCESS - if the function call is successful.\n
		CY_U3P_ERROR_BAD_ARGUMENT - if a bad argument is passed.


 */
extern CyU3PReturnStatus_t CyCx3_M10MO_Crop_Resolution(
		uint8_t FullResID,
		uint16_t CropResWidth,
		uint16_t CropResHeight,
		CyBool_t isMJPEG,
		CyBool_t isUSB3);

extern void CyCx3_M10MO_StartFW();
extern CyU3PReturnStatus_t CyCx3_M10MO_SIOInit();
extern CyU3PReturnStatus_t CyCx3_M10MO_I2CInit();
extern void CyCx3_M10MO_PowerOff();
extern void M10MO_prep();
extern void M10MO_program(int block, uint8_t *buf);
extern int M10MO_finalize();
extern void M10MO_ColorBar_Output();

extern void M10MO_Read_Memory(uint32_t addr, uint16_t count, uint8_t *data);

#endif /* _INCLUDED_CYCX3_M10MO_H_ */
