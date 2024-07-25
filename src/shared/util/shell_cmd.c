#include "shell_cmd.h"
#include "ch.h"
#include <string.h>
#include <stdio.h>

// TODO: debug
#include "util/log.h"
#include "hal/sys_time.h"

SHELL_CMD(help, "List all commands, or give details about a specific command");

SHELL_CMD_IMPL(help)
{
	ShellCmdHandler* pHandler = (ShellCmdHandler*)pUser;

	if(argc <= 1)
	{
		// List all commands briefly
		size_t commandWidth = 0;

		for(size_t i = 0; i < pHandler->commandsUsed; i++)
		{
			size_t length = strlen(pHandler->commands[i]->pName);

			if(length > commandWidth)
				commandWidth = length;
		}

		printf("Available commands:\r\n");

		for(size_t i = 0; i < pHandler->commandsUsed; i++)
		{
			const ShellCommand* pCmd = pHandler->commands[i];

			printf("  %*s - %s\r\n", commandWidth, pCmd->pName, pCmd->pDesc);
		}
	}
	else
	{
		// Give details of a specific command
		for(size_t i = 0; i < pHandler->commandsUsed; i++)
		{
			const ShellCommand* pCmd = pHandler->commands[i];

			if(strcmp(pCmd->pName, argv[1]))
				continue;

			printf("%s\r\n", pCmd->pDesc);
			printf("Usage: %s ", pCmd->pName);

			for(size_t j = 0; j < pCmd->numArgs; j++)
			{
				printf("%s ", pCmd->pArgs[j].name);
			}

			printf("\r\n");

			size_t argWidth = 0;

			for(size_t j = 0; j < pCmd->numArgs; j++)
			{
				size_t len = strlen(pCmd->pArgs[j].name);

				if(len > argWidth)
					argWidth = len;
			}

			for(size_t j = 0; j < pCmd->numArgs; j++)
			{
				printf("  %*s - %s\r\n", argWidth, pCmd->pArgs[j].name, pCmd->pArgs[j].desc);
			}
		}
	}
}

void ShellCmdHandlerInit(ShellCmdHandler* pHandler, void* pUser)
{
	pHandler->pUser = pUser;

	ShellCmdAdd(pHandler, help_command);
}

void ShellCmdAdd(ShellCmdHandler* pHandler, const ShellCommand* pCmd)
{
	if(pHandler->commandsUsed >= SHELL_CMD_MAX_CMDS)
	{
		chSysHalt("Shell command handler capacity exhausted.");
	}

	pHandler->commands[pHandler->commandsUsed++] = pCmd;
}

#define STATE_IN_WHITESPACE 0
#define STATE_IN_CMD 1
#define STATE_IN_TEXT 2

void ShellCmdExecute(ShellCmdHandler* pHandler, char* pCmdText)
{
	const char* pArgs[SHELL_CMD_MAX_ARGS];
	memset(pArgs, 0, sizeof(pArgs));

	int argsFound = 0;

	size_t cmdLen = strlen(pCmdText);

	uint16_t state = STATE_IN_WHITESPACE;

	for(size_t i = 0; i < cmdLen; i++)
	{
		char c = pCmdText[i];

		switch(state)
		{
			case STATE_IN_WHITESPACE:
			{
				if(argsFound == SHELL_CMD_MAX_ARGS)
				{
					i = cmdLen;
					break;
				}
				else if(c == '"')
				{
					state = STATE_IN_TEXT;
					pArgs[argsFound] = &pCmdText[i+1];
				}
				else if(c != ' ' && c != '\t')
				{
					state = STATE_IN_CMD;
					pArgs[argsFound] = &pCmdText[i];
				}
			}
			break;
			case STATE_IN_CMD:
			{
				if(c == ' ' || c == '\t')
				{
					pCmdText[i] = 0;
					argsFound++;
					state = STATE_IN_WHITESPACE;
				}
			}
			break;
			case STATE_IN_TEXT:
			{
				if(c == '"')
				{
					pCmdText[i] = 0;
					argsFound++;
					state = STATE_IN_WHITESPACE;
				}
			}
			break;
		}
	}

	if(state != STATE_IN_WHITESPACE)
		argsFound++;

	if(argsFound == 0)
		return;

	ShellCmdForward(pHandler, argsFound, pArgs);
}

void ShellCmdForward(ShellCmdHandler* pHandler, int argc, const char** argv)
{
	const ShellCommand* pDisplayHelpForCmd = 0;

	for(size_t i = 0; i < pHandler->commandsUsed; i++)
	{
		const ShellCommand* pCmd = pHandler->commands[i];

		if(strcmp(pCmd->pName, argv[0]) == 0)
		{
			if(argc > pCmd->numArgs)
			{
				if(strcmp(pCmd->pName, "help") == 0)
				{
					(*pCmd->handler)(pHandler, argc, argv);
				}
				else
				{
					(*pCmd->handler)(pHandler->pUser, argc, argv);
				}

				return;
			}
			else
			{
				pDisplayHelpForCmd = pCmd;
				break;
			}
		}
	}

	// At this point the command was either not found or had an invalid number of parameters
	if(pDisplayHelpForCmd)
	{
		fprintf(stderr, "Missing parameters\r\n");
	}
	else
	{
		fprintf(stderr, "Unknown command\r\n");
	}
}
