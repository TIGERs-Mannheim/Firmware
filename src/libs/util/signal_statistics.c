/*
 * signal_statistics.c
 *
 *  Created on: 22.07.2020
 *      Author: AndreR
 */

#include "signal_statistics.h"
#include <math.h>

void SignalStatisticsReset(SignalStatistics* pStat)
{
	pStat->sum = 0.0;
	pStat->sum2 = 0.0;
	pStat->numSamples = 0;
}

void SignalStatisticsSample(SignalStatistics* pStat, double sample)
{
	pStat->sum += sample;
	pStat->sum2 += sample*sample;
	pStat->numSamples++;
}

double SignalStatisticsGetVariance(const SignalStatistics* pStat)
{
	if(pStat->numSamples <= 1)
		return 0.0;

	double N = (double)pStat->numSamples;

	return (pStat->sum2 - pStat->sum*pStat->sum/N)/(N-1.0);
}

double SignalStatisticsGetMean(const SignalStatistics* pStat)
{
	if(pStat->numSamples <= 1)
		return 0.0;

	double N = (double)pStat->numSamples;

	return 1.0/N * pStat->sum;
}

void SignalStatisticsUpdate(SignalStatistics* pStat)
{
	pStat->mean = SignalStatisticsGetMean(pStat);
	pStat->variance = SignalStatisticsGetVariance(pStat);
	pStat->standardDeviation = sqrt(pStat->variance);
}
