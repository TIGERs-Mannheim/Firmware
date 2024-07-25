#include "log.h"
#include "hal/sys_time.h"

#include <string.h>
#include <stdio.h>

log_t _log;

#if LOG_LEVEL >= 3
void LogDebugDo(const char* pFilename, uint16_t line, uint32_t code, const char* pMsg)
{
	uint16_t entry = _log.level[LOG_DEBUG].lastEntry;
	entry %= LOG_NUM_ENTRIES;
	LogEntry* pEntry = &_log.level[LOG_DEBUG].entries[entry];

	if(pEntry->pFilename == pFilename && pEntry->line == line)
	{
		pEntry->time = SysTimeUSec();
		pEntry->code = code;
		pEntry->pMsg = pMsg;
		pEntry->repeats++;
	}
	else
	{
		entry = ++_log.level[LOG_DEBUG].lastEntry;
		entry %= LOG_NUM_ENTRIES;
		pEntry = &_log.level[LOG_DEBUG].entries[entry];
		pEntry->pFilename = pFilename;
		pEntry->line = line;
		pEntry->time = SysTimeUSec();
		pEntry->code = code;
		pEntry->pMsg = pMsg;
		pEntry->repeats = 0;
	}
}
#endif

#if LOG_LEVEL >= 2
void LogInfoDo(const char* pFilename, uint16_t line, uint32_t code, const char* pMsg)
{
	uint16_t entry = _log.level[LOG_INFO].lastEntry;
	entry %= LOG_NUM_ENTRIES;
	LogEntry* pEntry = &_log.level[LOG_INFO].entries[entry];

	if(pEntry->pFilename == pFilename && pEntry->line == line)
	{
		pEntry->time = SysTimeUSec();
		pEntry->code = code;
		pEntry->pMsg = pMsg;
		pEntry->repeats++;
	}
	else
	{
		entry = ++_log.level[LOG_INFO].lastEntry;
		entry %= LOG_NUM_ENTRIES;
		pEntry = &_log.level[LOG_INFO].entries[entry];
		pEntry->pFilename = pFilename;
		pEntry->line = line;
		pEntry->time = SysTimeUSec();
		pEntry->code = code;
		pEntry->pMsg = pMsg;
		pEntry->repeats = 0;
	}
}
#endif

#if LOG_LEVEL >= 1
void LogWarnDo(const char* pFilename, uint16_t line, uint32_t code, const char* pMsg)
{
	uint16_t entry = _log.level[LOG_WARNING].lastEntry;
	entry %= LOG_NUM_ENTRIES;
	LogEntry* pEntry = &_log.level[LOG_WARNING].entries[entry];

	if(pEntry->pFilename == pFilename && pEntry->line == line)
	{
		pEntry->time = SysTimeUSec();
		pEntry->code = code;
		pEntry->pMsg = pMsg;
		pEntry->repeats++;
	}
	else
	{
		entry = ++_log.level[LOG_WARNING].lastEntry;
		entry %= LOG_NUM_ENTRIES;
		pEntry = &_log.level[LOG_WARNING].entries[entry];
		pEntry->pFilename = pFilename;
		pEntry->line = line;
		pEntry->time = SysTimeUSec();
		pEntry->code = code;
		pEntry->pMsg = pMsg;
		pEntry->repeats = 0;
	}
}
#endif

#if LOG_LEVEL >= 0
void LogErrDo(const char* pFilename, uint16_t line, uint32_t code, const char* pMsg)
{
	uint16_t entry = _log.level[LOG_ERROR].lastEntry;
	entry %= LOG_NUM_ENTRIES;
	LogEntry* pEntry = &_log.level[LOG_ERROR].entries[entry];

	if(pEntry->pFilename == pFilename && pEntry->line == line)
	{
		pEntry->time = SysTimeUSec();
		pEntry->code = code;
		pEntry->pMsg = pMsg;
		pEntry->repeats++;
	}
	else
	{
		entry = ++_log.level[LOG_ERROR].lastEntry;
		entry %= LOG_NUM_ENTRIES;
		pEntry = &_log.level[LOG_ERROR].entries[entry];
		pEntry->pFilename = pFilename;
		pEntry->line = line;
		pEntry->time = SysTimeUSec();
		pEntry->code = code;
		pEntry->pMsg = pMsg;
		pEntry->repeats = 0;
	}
}
#endif

void LogPrintAllRecent(uint16_t numEntries)
{
	for(uint8_t level = 0; level < LOG_NUM_LEVELS; level++)
	{
		printf("--- Level %hu ---\r\n", (uint16_t)level);

		uint16_t entry = _log.level[level].lastEntry%LOG_NUM_ENTRIES;

		for(uint16_t i = 0; i < numEntries; i++)
		{
			LogEntry* pEntry = &_log.level[level].entries[entry];
			if(pEntry->time == 0)
				continue;
			printf("%10u %s:%hu\t\t0x%08X\t(%u)\t%s\r\n", pEntry->time,
					pEntry->pFilename, pEntry->line, pEntry->code, pEntry->repeats, pEntry->pMsg);
			--entry;
			entry %= LOG_NUM_ENTRIES;
		}
	}
}
