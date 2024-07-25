/*
 * imu.c
 *
 *  Created on: 23.07.2020
 *      Author: AndreR
 */

#include "imu.h"
#include <stdio.h>
#include "gui/styles.h"
#include "misc/inventory.h"

static struct _handles
{
	GHandle hAcc[3];
	GHandle hGyr[3];
	GHandle hMag[3];
	GHandle hImuTemp;
	GHandle hMagTemp;
} handles;

static GHandle createLabel(coord_t x, coord_t y, coord_t w, coord_t h, GHandle parent, const char* pText)
{
	GWidgetInit deviceLabelInit = { { x, y, w, h, TRUE, parent }, pText, 0, 0, 0 };
	return gwinLabelCreate(0, &deviceLabelInit);
}

GHandle ImuCreate()
{
    font_t curFont = gwinGetDefaultFont();
    gwinSetDefaultFont(gdispOpenFont("fixed_7x14"));

	GWidgetInit topContInit = { { 0, 60, 240, 260, FALSE, 0 }, "IMU", 0, 0, 0 };
	GHandle hTop = gwinContainerCreate(0, &topContInit, 0);

    createLabel(45, 5, 30, 20, hTop, "Acc");
    createLabel(115, 5, 30, 20, hTop, "Gyr");
    createLabel(185, 5, 30, 20, hTop, "Mag");
    createLabel(5, 25, 20, 20, hTop, "X");
    createLabel(5, 45, 20, 20, hTop, "Y");
    createLabel(5, 65, 20, 20, hTop, "Z");
    createLabel(5, 85, 20, 20, hTop, "T");

    for(uint8_t i = 0; i < 3; i++)
    {
    	handles.hAcc[i] = createLabel(25, 25+i*20, 60, 20, hTop, "-");
    	handles.hGyr[i] = createLabel(95, 25+i*20, 60, 20, hTop, "-");
    	handles.hMag[i] = createLabel(165, 25+i*20, 60, 20, hTop, "-");
    }

    const InventoryImuCalibration* pImuCalib = InventoryGetImuCalibration();

    if(pImuCalib->calibrated)
    {
    	GWidgetInit calInit = { { 5, 105, 120, 20, TRUE, hTop }, "Calibrated: Yes", 0, 0, &GreenTextStyle };
    	gwinLabelCreate(0, &calInit);
    }
    else
    {
    	GWidgetInit calInit = { { 5, 105, 120, 20, TRUE, hTop }, "Calibrated: No", 0, 0, &RedTextStyle };
    	gwinLabelCreate(0, &calInit);
    }

    handles.hImuTemp = createLabel(70, 85, 100, 20, hTop, "-");
    handles.hMagTemp = createLabel(165, 85, 100, 20, hTop, "-");

	gwinSetDefaultFont(curFont);

	return hTop;
}

void ImuUpdate(const float* pAcc, const float* pGyr, const float* pMag, float imuTemp, float magTemp)
{
	const InventoryImuCalibration* pImuCalib = InventoryGetImuCalibration();

	static char sAcc[3][16];
	static char sGyr[3][16];
	static char sMag[3][16];
	static char sImuTemp[16];
	static char sMagTemp[16];

	for(uint8_t i = 0; i < 3; i++)
	{
		float tiltBias = (i < 2) ? pImuCalib->accTiltBias[i] : 0.0f;
		snprintf(sAcc[i], 16, "% .4f", pAcc[i] - pImuCalib->accBias[i] - tiltBias);
		gwinSetText(handles.hAcc[i], sAcc[i], FALSE);

		snprintf(sGyr[i], 16, "% .4f", pGyr[i] - pImuCalib->gyrBias[i]);
		gwinSetText(handles.hGyr[i], sGyr[i], FALSE);

		snprintf(sMag[i], 16, "% .4f", pMag[i] - pImuCalib->magBias[i]);
		gwinSetText(handles.hMag[i], sMag[i], FALSE);
	}

	snprintf(sImuTemp, 16, "%.2f", imuTemp);
	gwinSetText(handles.hImuTemp, sImuTemp, FALSE);

	snprintf(sMagTemp, 16, "% .2f", magTemp);
	gwinSetText(handles.hMagTemp, sMagTemp, FALSE);
}
