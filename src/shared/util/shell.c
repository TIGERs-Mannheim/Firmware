/*
 * Useful links:
 * https://en.wikipedia.org/wiki/ANSI_escape_code
 * http://ascii-table.com/ansi-escape-sequences-vt-100.php
 * http://www.asciitable.com/
 */

#include "shell.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define EVENT_MASK_UART			EVENT_MASK(0)

#define ESC_STATE_NONE 0 // not in escape mode
#define ESC_STATE_TYPE 1 // parsing type byte of escape sequence
#define ESC_STATE_CSI 2 // Control Sequence Introducer

void ShellInit(Shell* pShell, ShellWriteFunc writeFunc)
{
	pShell->writeFunc = writeFunc;

	pShell->pHistoryCur = pShell->history;
	pShell->pHistoryLast = pShell->history;

	ShellCmdHandlerInit(&pShell->cmdHandler, 0);
}

static void shellWrite(Shell* pShell, const char* pData, size_t dataLength)
{
	(*pShell->writeFunc)(pData, dataLength);
}

static void clearAndOutputCurrentEntry(Shell* pShell)
{
	const char* clearLine = "\e[2K\r";
	shellWrite(pShell, clearLine, strlen(clearLine));
	shellWrite(pShell, pShell->pHistoryCur->dataCur, pShell->pHistoryCur->lengthCur);
}

static void moveCursorRight(Shell* pShell, uint16_t change)
{
	if(change == 0)
		return;

	char moveCursorBuf[8];
	uint32_t bytesWritten = snprintf(moveCursorBuf, 8, "\e[%huC", change);
	shellWrite(pShell, moveCursorBuf, bytesWritten);
}

static void moveCursorLeft(Shell* pShell, uint16_t change)
{
	if(change == 0)
		return;

	char moveCursorBuf[8];
	uint32_t bytesWritten = snprintf(moveCursorBuf, 8, "\e[%huD", change);
	shellWrite(pShell, moveCursorBuf, bytesWritten);
}

static void executeHome(Shell* pShell)
{
	if(pShell->cursorPos > 0)
	{
		moveCursorLeft(pShell, pShell->cursorPos);
		pShell->cursorPos = 0;
	}
}

static void executeEnd(Shell* pShell)
{
	if(pShell->cursorPos < pShell->pHistoryCur->lengthCur)
	{
		moveCursorRight(pShell, pShell->pHistoryCur->lengthCur - pShell->cursorPos);
		pShell->cursorPos = pShell->pHistoryCur->lengthCur;
	}
}

void ShellParse(Shell* pShell, char data)
{
	switch(pShell->escapeMode)
	{
		case ESC_STATE_NONE:
		{
			if(data == 27)	// escape character?
			{
				pShell->escapeMode = ESC_STATE_TYPE;
			}
			else
			{
				// normal character processing

				if(data > 31 && data < 127)	// readable ASCII char?
				{
					if(pShell->pHistoryCur->lengthCur < SHELL_INPUT_BUFFER_SIZE)
					{
						if(pShell->pHistoryCur->lengthCur == pShell->cursorPos)
						{
							// insert at end
							pShell->pHistoryCur->dataCur[pShell->pHistoryCur->lengthCur++] = data;
							pShell->cursorPos++;
							shellWrite(pShell, &data, 1);
						}
						else
						{
							// insert within text, need to move
							memmove(&pShell->pHistoryCur->dataCur[pShell->cursorPos+1],
									&pShell->pHistoryCur->dataCur[pShell->cursorPos],
									SHELL_INPUT_BUFFER_SIZE - pShell->cursorPos - 1);
							pShell->pHistoryCur->dataCur[pShell->cursorPos] = data;
							pShell->pHistoryCur->lengthCur++;
							pShell->cursorPos++;

							clearAndOutputCurrentEntry(pShell);
							moveCursorLeft(pShell, pShell->pHistoryCur->lengthCur - pShell->cursorPos);
						}
					}
				}

				if(data == 9) // Tab
				{
					// no tab completion implemented :D
				}

				if(data == 127)	// backspace
				{
					if(pShell->pHistoryCur->lengthCur > 0 && pShell->cursorPos > 0)
					{
						memmove(&pShell->pHistoryCur->dataCur[pShell->cursorPos-1],
								&pShell->pHistoryCur->dataCur[pShell->cursorPos],
								SHELL_INPUT_BUFFER_SIZE - pShell->cursorPos);
						pShell->pHistoryCur->lengthCur--;
						pShell->cursorPos--;

						clearAndOutputCurrentEntry(pShell);
						moveCursorLeft(pShell, pShell->pHistoryCur->lengthCur - pShell->cursorPos);
					}
				}

				if(data == '\n' || data == '\r')
				{
					const char* nextLine = "\r\n";
					shellWrite(pShell, nextLine, strlen(nextLine));

					if(pShell->pHistoryCur->lengthCur > 0)
					{
						// command complete, process
						memcpy(pShell->activeCmdData, pShell->pHistoryCur->dataCur, pShell->pHistoryCur->lengthCur);
						pShell->activeCmdData[pShell->pHistoryCur->lengthCur] = 0;

						ShellCmdExecute(&pShell->cmdHandler, pShell->activeCmdData);

						if(pShell->pHistoryCur == pShell->pHistoryLast)
						{
							pShell->pHistoryCur->lengthOrig = pShell->pHistoryCur->lengthCur;
							memcpy(pShell->pHistoryCur->dataOrig, pShell->pHistoryCur->dataCur, pShell->pHistoryCur->lengthCur);
						}
						else
						{
							// copy modified command from history to latest entry
							pShell->pHistoryLast->lengthCur = pShell->pHistoryCur->lengthCur;
							pShell->pHistoryLast->lengthOrig = pShell->pHistoryCur->lengthCur;
							memcpy(pShell->pHistoryLast->dataCur, pShell->pHistoryCur->dataCur, pShell->pHistoryCur->lengthCur);
							memcpy(pShell->pHistoryLast->dataOrig, pShell->pHistoryCur->dataCur, pShell->pHistoryCur->lengthCur);

							// reset command in history to its original form
							pShell->pHistoryCur->lengthCur = pShell->pHistoryCur->lengthOrig;
							memcpy(pShell->pHistoryCur->dataCur, pShell->pHistoryCur->dataOrig, pShell->pHistoryCur->lengthOrig);
						}

						ShellHistoryEntry* pPrev = pShell->pHistoryLast - 1;
						if(pPrev < pShell->history)
							pPrev = &pShell->history[SHELL_HISTORY_SIZE-1];

						if(pPrev->lengthOrig == pShell->pHistoryLast->lengthOrig &&
								memcmp(pPrev->dataOrig, pShell->pHistoryLast->dataOrig, pPrev->lengthOrig) == 0)
						{
							// same command as previous one => discard from history
						}
						else
						{
							// different commmand => save in history
							++pShell->pHistoryLast;
							if(pShell->pHistoryLast >= &pShell->history[SHELL_HISTORY_SIZE])
								pShell->pHistoryLast = pShell->history;
						}

						pShell->pHistoryCur = pShell->pHistoryLast;

						pShell->pHistoryCur->lengthCur = 0;
						pShell->pHistoryCur->lengthOrig = 0;
						memset(pShell->pHistoryCur->dataOrig, 0, SHELL_INPUT_BUFFER_SIZE);
						memset(pShell->pHistoryCur->dataCur, 0, SHELL_INPUT_BUFFER_SIZE);

						pShell->cursorPos = 0;
					}
					else
					{
						// empty line entered
						// reset command in history to its original form
						if(pShell->pHistoryCur != pShell->pHistoryLast)
						{
							pShell->pHistoryCur->lengthCur = pShell->pHistoryCur->lengthOrig;
							memcpy(pShell->pHistoryCur->dataCur, pShell->pHistoryCur->dataOrig, pShell->pHistoryCur->lengthOrig);
						}

						pShell->pHistoryCur = pShell->pHistoryLast;

						pShell->pHistoryCur->lengthCur = 0;
						pShell->pHistoryCur->lengthOrig = 0;
						memset(pShell->pHistoryCur->dataOrig, 0, SHELL_INPUT_BUFFER_SIZE);
						memset(pShell->pHistoryCur->dataCur, 0, SHELL_INPUT_BUFFER_SIZE);
					}
				}
			}
		}
		break;
		case ESC_STATE_TYPE:
		{
			if(data == '[')
			{
				pShell->escapeMode = ESC_STATE_CSI;
				pShell->esc.paramPos = 0;
				pShell->esc.interPos = 0;
			}
			else
				pShell->escapeMode = ESC_STATE_NONE;
		}
		break;
		case ESC_STATE_CSI:
		{
			if(data >= 0x30 && data <= 0x3F)
			{
				// "parameter bytes" in the range 0x30�0x3F (ASCII 0�9:;<=>?)
				if(pShell->esc.paramPos < sizeof(pShell->esc.param))
					pShell->esc.param[pShell->esc.paramPos++] = data;
			}
			else if(data >= 0x20 && data <= 0x2F)
			{
				// "intermediate bytes" in the range 0x20�0x2F (ASCII space and !"#$%&'()*+,-./)
				if(pShell->esc.interPos < sizeof(pShell->esc.inter))
					pShell->esc.inter[pShell->esc.interPos++] = data;
			}
			else if(data >= 0x40 && data <= 0x7E)
			{
				// "final byte" in the range 0x40�0x7E (ASCII @A�Z[\]^_`a�z{|}~)
				pShell->esc.final = data;

				switch(pShell->esc.final)
				{
					case 'D': // left arrow
					{
						if(pShell->cursorPos > 0)
						{
							moveCursorLeft(pShell, 1);
							pShell->cursorPos--;
						}
					}
					break;
					case 'C': // right arrow
					{
						if(pShell->cursorPos < pShell->pHistoryCur->lengthCur)
						{
							moveCursorRight(pShell, 1);
							pShell->cursorPos++;
						}
					}
					break;
					case 'A': // up arrow
					{
						ShellHistoryEntry* pLastValid = pShell->pHistoryLast + 1;
						if(pLastValid >= &pShell->history[SHELL_HISTORY_SIZE])
							pLastValid = pShell->history;

						ShellHistoryEntry* pPrev = pShell->pHistoryCur - 1;
						if(pPrev < pShell->history)
							pPrev = &pShell->history[SHELL_HISTORY_SIZE-1];

						if(pPrev != pLastValid && pPrev->lengthCur > 0)
						{
							// valid entry
							pShell->pHistoryCur = pPrev;

							clearAndOutputCurrentEntry(pShell);
							pShell->cursorPos = pShell->pHistoryCur->lengthCur;
						}
					}
					break;
					case 'B': // down arrow
					{
						ShellHistoryEntry* pNext = pShell->pHistoryCur + 1;
						if(pNext >= &pShell->history[SHELL_HISTORY_SIZE])
							pNext = pShell->history;

						if(pShell->pHistoryCur != pShell->pHistoryLast)
						{
							pShell->pHistoryCur = pNext;

							clearAndOutputCurrentEntry(pShell);
							pShell->cursorPos = pShell->pHistoryCur->lengthCur;
						}
					}
					break;
					case 'F': // end (xterm)
					{
						executeEnd(pShell);
					}
					break;
					case 'H': // home (xterm)
					{
						executeHome(pShell);
					}
					break;
					case '~': // "private key": home, end, insert, delete
					{
						switch(pShell->esc.param[0])
						{
							case '1': // home
							{
								executeHome(pShell);
							}
							break;
							case '2': // insert
							{
								// not used
							}
							break;
							case '3': // delete
							{
								if(pShell->pHistoryCur->lengthCur > 0)
								{
									if(pShell->pHistoryCur->lengthCur == pShell->cursorPos)
									{
										// no effect at end of text
									}
									else
									{
										// delete character under cursor
										memmove(&pShell->pHistoryCur->dataCur[pShell->cursorPos],
												&pShell->pHistoryCur->dataCur[pShell->cursorPos+1],
												SHELL_INPUT_BUFFER_SIZE - pShell->cursorPos);
										pShell->pHistoryCur->lengthCur--;

										clearAndOutputCurrentEntry(pShell);

										if(pShell->pHistoryCur->lengthCur != pShell->cursorPos)
											moveCursorLeft(pShell, pShell->pHistoryCur->lengthCur - pShell->cursorPos);
									}
								}
							}
							break;
							case '4': // end
							{
								executeEnd(pShell);
							}
							break;
						}
					}
					break;
				}

				pShell->escapeMode = ESC_STATE_NONE;
			}
			else
			{
				pShell->escapeMode = ESC_STATE_NONE;
			}
		}
		break;
	}
}
