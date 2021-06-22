/*
 * microphone.h
 *
 *  Created on: 18.04.2020
 *      Author: FelixW
 */

#pragma once

#include "ch.h"
#include "fatfs/ff.h"

#define MICROPHONE_EVENT_QUEUE_SIZE 10

typedef struct _Microphone
{
	mailbox_t eventQueue;
	msg_t eventQueueData[MICROPHONE_EVENT_QUEUE_SIZE];

	uint32_t totalBytes[2];

	FIL files[2];
} Microphone;

extern Microphone microphone;

void MicrophoneInit();
void MicrophoneTask(void* params);

void MicrophoneStartRecoding();
void MicrophoneStopRecording();
