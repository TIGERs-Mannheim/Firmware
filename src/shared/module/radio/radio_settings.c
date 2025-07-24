#include "radio_settings.h"
#include "sx1280_def.h"

RadioSettings radioSettings = {
	.phyTimeouts = {
		.default_us = 100,
		.bufferIo_us = 100,
		.modeChange_us = 200,
		.txWait_us = 450,
		.rxWait_us = 580,
		.rxWaitLong_us = 2500,
		.txDelay_us = 50,
	},
	.packetType = PACKET_TYPE_FLRC,
	.settingsCommon = {
		.frequency = 2300000000UL, // default frequency, overwritten by config, recommended channel 0..30 for best antenna matching
		.txPower = 16, // = -2dBm, don't go over -2dBm to respect FEM absolute maximum ratings.
		.rxGain = -1,
		.paRampTime = PA_RAMP_04_US,
		.highSensitivityMode = 1,
	},
	.settingsGfsk = {
		.bitrateAndBandwidth = GFSK_BR_2_000_BW_2_4,
		.modulationIndex = MOD_IND_0_5,
		.modulationShaping = MOD_SHAPING_BT_0_5,
		.preambleLength = PREAMBLE_LENGTH_32_BITS,
		.longPreambleCount = 4,
		.crcLength = GFSK_CRC_2_BYTES,
		.enableWhitening = WHITENING_DISABLE,
		.syncWord = 0x54696761,
		.syncWordTolerance = 8,
	},
	.settingsFlrc = {
		.bitrateAndBandwidth = FLRC_BR_1_300_BW_1_2,
		.codingRate = FLRC_CR_1_0,
		.modulationShaping = MOD_SHAPING_BT_0_5,
		.preambleLength = PREAMBLE_LENGTH_32_BITS,
		.crcLength = FLRC_CRC_2_BYTES,
		.enableWhitening = WHITENING_DISABLE,
		.syncWord = 0x54696761,
		.syncWordTolerance = 2,
	},
	.connectionTimeout_us = 1000000,
};
