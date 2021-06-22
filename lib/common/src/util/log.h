/*
 * log.h
 *
 *  Created on: 14.04.2014
 *      Author: AndreR
 */

#ifndef LOG_H_
#define LOG_H_

#include <stdint.h>

#ifdef DEBUG
#define LOG_LEVEL 3
#else
#define LOG_LEVEL 1
#endif

#ifdef STM32F30X
#define LOG_NUM_ENTRIES 32
#else
#define LOG_NUM_ENTRIES 128
#endif

#define LOG_NUM_LEVELS LOG_LEVEL+1

#define LOG_DEBUG		3
#define LOG_INFO		2
#define LOG_WARNING		1
#define LOG_ERROR		0

typedef struct _LogEntry
{
	const char* pFilename;
	const char* pMsg;
	uint16_t line;
	uint32_t code;
	uint32_t time;
	uint32_t repeats;
} LogEntry;

typedef struct _log_t
{
	struct _level
	{
		LogEntry entries[LOG_NUM_ENTRIES];
		volatile uint16_t lastEntry;
	} level[LOG_NUM_LEVELS];
} log_t;

extern log_t _log;

#ifndef __BASE_FILE__
#define __BASE_FILE__ "X"	// this is just to make eclipse happy
#endif

#if LOG_LEVEL >= 3
#define LogDebug(msg) 			LogDebugDo(__BASE_FILE__, __LINE__, 0, msg)
#define LogDebugC(msg, code)	LogDebugDo(__BASE_FILE__, __LINE__, code, msg)
#else
#define LogDebug(msg)
#define LogDebugC(msg, code)
#endif

#if LOG_LEVEL >= 2
#define LogInfo(msg) 			LogInfoDo( __BASE_FILE__, __LINE__, 0, msg)
#define LogInfoC(msg, code) 	LogInfoDo( __BASE_FILE__, __LINE__, code, msg)
#else
#define LogInfo(msg)
#define LogInfoC(msg, code)
#endif

#if LOG_LEVEL >= 1
#define LogWarn(msg) 			LogWarnDo( __BASE_FILE__, __LINE__, 0, msg)
#define LogWarnC(msg, code) 	LogWarnDo( __BASE_FILE__, __LINE__, code, msg)
#else
#define LogWarn(msg)
#define LogWarnC(msg, code)
#endif

#if LOG_LEVEL >= 0
#define LogError(msg) 			LogErrDo( __BASE_FILE__, __LINE__, 0, msg)
#define LogErrorC(msg, code) 	LogErrDo( __BASE_FILE__, __LINE__, code, msg)
#else
#define LogError(msg)
#define LogErrorC(msg, code)
#endif

void LogDebugDo(const char* pFilename, uint16_t line, uint32_t code, const char* pMsg);
void LogInfoDo(const char* pFilename, uint16_t line, uint32_t code, const char* pMsg);
void LogWarnDo(const char* pFilename, uint16_t line, uint32_t code, const char* pMsg);
void LogErrDo(const char* pFilename, uint16_t line, uint32_t code, const char* pMsg);
void LogPrintAllRecent(uint16_t numEntries);

#endif /* LOG_H_ */
