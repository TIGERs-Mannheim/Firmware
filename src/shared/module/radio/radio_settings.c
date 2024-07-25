#include "radio_settings.h"
#include "sx1280_def.h"

RadioSettings radioSettings = {
	.phyTimeouts = {
		.default_us = 100,
		.bufferIo_us = 100,
		.modeChange_us = 200,
		.txWait_us = 450,
		.rxWait_us = 400,
		.rxWaitLong_us = 5000,
		.txDelay_us = 25,
	},
	.packetType = PACKET_TYPE_GFSK,
	.settingsCommon = {
		.frequency = 2300000000UL,
		.txPower = 16, // = -2dBm, don't go over -2dBm to respect FEM absolute maximum ratings;
		.paRampTime = PA_RAMP_04_US,
		.highSensitivityMode = 0,
	},
	.settingsGfsk = {
		.bitrateAndBandwidth = GFSK_BR_2_000_BW_2_4,
		.modulationIndex = MOD_IND_0_5,
		.modulationShaping = MOD_SHAPING_BT_1_0,
		.preambleLength = PREAMBLE_LENGTH_24_BITS,
		.crcLength = 0,
		.enableWhitening = WHITENING_DISABLE,
		.syncWord = 0x54696761,
		.syncWordTolerance = 2,
	},
	.settingsFlrc = {
		.bitrateAndBandwidth = FLRC_BR_1_300_BW_1_2,
		.codingRate = FLRC_CR_1_0,
		.modulationShaping = MOD_SHAPING_BT_1_0,
		.preambleLength = PREAMBLE_LENGTH_16_BITS,
		.crcLength = CRC_OFF,
		.enableWhitening = WHITENING_DISABLE,
		.syncWord = 0x54696761,
		.syncWordTolerance = 2,
	},
	.connectionTimeout_us = 1000000,
};
