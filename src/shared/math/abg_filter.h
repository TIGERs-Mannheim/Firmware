/*
 * abg_filter.h
 *
 *  Created on: 05.11.2015
 *      Author: AndreR
 *
 *  Alpha-Beta-Gamma Filter according to http://www.plctalk.net/qanda/archive/index.php/t-67883.html
 *  and http://deltamotion.com/peter/pdf/AlphaBetaGammaCode.pdf
 */

#ifndef ABG_FILTER_H_
#define ABG_FILTER_H_

// TODO: hint: http://www.researchgate.net/publication/257725291_Alpha_Beta_Gamma_Filter_for_Cascaded_PID_Motor_Position_Control

#define ABG_FILTER_CRITICALLY_DAMPED 0
#define ABG_FILTER_BUTTERWORTH 1
#define ABG_FILTER_MINIMUM_IAE 2

#include <stdint.h>

typedef struct _ABGFilter
{
	float dT;
	float K[3];
	float x[3];
	float x0Limit;
} ABGFilter;

void ABGFilterInit(ABGFilter* pFilter, float updateRate, float filterFreq, uint8_t filterType);
void ABGFilterUpdate(ABGFilter* pFilter, float meas);

#endif /* ABG_FILTER_H_ */
