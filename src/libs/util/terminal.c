/*
 * terminal.c
 *
 *  Created on: 26.10.2014
 *      Author: AndreR
 *
 * Useful links:
 * https://en.wikipedia.org/wiki/ANSI_escape_code
 * http://ascii-table.com/ansi-escape-sequences-vt-100.php
 * http://www.asciitable.com/
 */

#include "terminal.h"

#include "util/console.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define ESC_STATE_NONE 0 // not in escape mode
#define ESC_STATE_TYPE 1 // parsing type byte of escape sequence
#define ESC_STATE_CSI 2 // Control Sequence Introducer

Terminal terminal;

void TerminalInit(input_queue_t* pInQueue, output_queue_t* pOutQueue)
{
	terminal.pInQueue = pInQueue;
	terminal.pOutQueue = pOutQueue;

	terminal.pHistoryCur = terminal.history;
	terminal.pHistoryLast = terminal.history;
}

static void clearAndOutputCurrentEntry()
{
	const char* clearLine = "\e[2K\r";
	TerminalWrite((uint8_t*)clearLine, strlen(clearLine));
	TerminalWrite((uint8_t*)terminal.pHistoryCur->dataCur, terminal.pHistoryCur->lengthCur);
}

static void moveCursorRight(uint16_t change)
{
	if(change == 0)
		return;

	char moveCursorBuf[8];
	uint32_t bytesWritten = snprintf(moveCursorBuf, 8, "\e[%huC", change);
	TerminalWrite((uint8_t*)moveCursorBuf, bytesWritten);
}

static void moveCursorLeft(uint16_t change)
{
	if(change == 0)
		return;

	char moveCursorBuf[8];
	uint32_t bytesWritten = snprintf(moveCursorBuf, 8, "\e[%huD", change);
	TerminalWrite((uint8_t*)moveCursorBuf, bytesWritten);
}

static void executeHome()
{
	if(terminal.cursorPos > 0)
	{
		moveCursorLeft(terminal.cursorPos);
		terminal.cursorPos = 0;
	}
}

static void executeEnd()
{
	if(terminal.cursorPos < terminal.pHistoryCur->lengthCur)
	{
		moveCursorRight(terminal.pHistoryCur->lengthCur - terminal.cursorPos);
		terminal.cursorPos = terminal.pHistoryCur->lengthCur;
	}
}

void TerminalTask(void* params)
{
	(void)params;

	msg_t data;
	uint8_t escapeMode = 0;

	chRegSetThreadName("Terminal");

#if defined(STM32F4XX) || defined(STM32F30X)
	chThdSleepMilliseconds(4000);
#endif

	while(1)
	{
		data = chIQGetTimeout(terminal.pInQueue, TIME_INFINITE);
		if(data < 0)
			continue;

		switch(escapeMode)
		{
			case ESC_STATE_NONE:
			{
				if(data == 27)	// escape character?
				{
					escapeMode = ESC_STATE_TYPE;
				}
				else
				{
					// normal character processing

					if(data > 31 && data < 127)	// readable ASCII char?
					{
						if(terminal.pHistoryCur->lengthCur < TERMINAL_INPUT_BUFFER_SIZE)
						{
							if(terminal.pHistoryCur->lengthCur == terminal.cursorPos)
							{
								// insert at end
								terminal.pHistoryCur->dataCur[terminal.pHistoryCur->lengthCur++] = data;
								terminal.cursorPos++;
								chOQPutTimeout(terminal.pOutQueue, data, TIME_IMMEDIATE);
							}
							else
							{
								// insert within text, need to move
								memmove(&terminal.pHistoryCur->dataCur[terminal.cursorPos+1],
										&terminal.pHistoryCur->dataCur[terminal.cursorPos],
										TERMINAL_INPUT_BUFFER_SIZE - terminal.cursorPos - 1);
								terminal.pHistoryCur->dataCur[terminal.cursorPos] = data;
								terminal.pHistoryCur->lengthCur++;
								terminal.cursorPos++;

								clearAndOutputCurrentEntry();
								moveCursorLeft(terminal.pHistoryCur->lengthCur - terminal.cursorPos);
							}
						}
					}

					if(data == 9) // Tab
					{
						// no tab completion implemented :D
					}

					if(data == 127)	// backspace
					{
						if(terminal.pHistoryCur->lengthCur > 0 && terminal.cursorPos > 0)
						{
							memmove(&terminal.pHistoryCur->dataCur[terminal.cursorPos-1],
									&terminal.pHistoryCur->dataCur[terminal.cursorPos],
									TERMINAL_INPUT_BUFFER_SIZE - terminal.cursorPos);
							terminal.pHistoryCur->lengthCur--;
							terminal.cursorPos--;

							clearAndOutputCurrentEntry();
							moveCursorLeft(terminal.pHistoryCur->lengthCur - terminal.cursorPos);
						}
					}

					if(data == '\n' || data == '\r')
					{
						const char* nextLine = "\r\n";
						TerminalWrite((uint8_t*)nextLine, strlen(nextLine));

						if(terminal.pHistoryCur->lengthCur > 0)
						{
							// command complete, process
							if(terminal.cmdCb)
								(*terminal.cmdCb)(terminal.pHistoryCur->dataCur, terminal.pHistoryCur->lengthCur);

							if(terminal.pHistoryCur == terminal.pHistoryLast)
							{
								terminal.pHistoryCur->lengthOrig = terminal.pHistoryCur->lengthCur;
								memcpy(terminal.pHistoryCur->dataOrig, terminal.pHistoryCur->dataCur, terminal.pHistoryCur->lengthCur);
							}
							else
							{
								// copy modified command from history to latest entry
								terminal.pHistoryLast->lengthCur = terminal.pHistoryCur->lengthCur;
								terminal.pHistoryLast->lengthOrig = terminal.pHistoryCur->lengthCur;
								memcpy(terminal.pHistoryLast->dataCur, terminal.pHistoryCur->dataCur, terminal.pHistoryCur->lengthCur);
								memcpy(terminal.pHistoryLast->dataOrig, terminal.pHistoryCur->dataCur, terminal.pHistoryCur->lengthCur);

								// reset command in history to its original form
								terminal.pHistoryCur->lengthCur = terminal.pHistoryCur->lengthOrig;
								memcpy(terminal.pHistoryCur->dataCur, terminal.pHistoryCur->dataOrig, terminal.pHistoryCur->lengthOrig);
							}

							TerminalHistoryEntry* pPrev = terminal.pHistoryLast - 1;
							if(pPrev < terminal.history)
								pPrev = &terminal.history[TERMINAL_HISTORY_SIZE-1];

							if(pPrev->lengthOrig == terminal.pHistoryLast->lengthOrig &&
									memcmp(pPrev->dataOrig, terminal.pHistoryLast->dataOrig, pPrev->lengthOrig) == 0)
							{
								// same command as previous one => discard from history
							}
							else
							{
								// different commmand => save in history
								++terminal.pHistoryLast;
								if(terminal.pHistoryLast >= &terminal.history[TERMINAL_HISTORY_SIZE])
									terminal.pHistoryLast = terminal.history;
							}

							terminal.pHistoryCur = terminal.pHistoryLast;

							terminal.pHistoryCur->lengthCur = 0;
							terminal.pHistoryCur->lengthOrig = 0;
							memset(terminal.pHistoryCur->dataOrig, 0, TERMINAL_INPUT_BUFFER_SIZE);
							memset(terminal.pHistoryCur->dataCur, 0, TERMINAL_INPUT_BUFFER_SIZE);

							terminal.cursorPos = 0;
						}
						else
						{
							// empty line entered
							// reset command in history to its original form
							if(terminal.pHistoryCur != terminal.pHistoryLast)
							{
								terminal.pHistoryCur->lengthCur = terminal.pHistoryCur->lengthOrig;
								memcpy(terminal.pHistoryCur->dataCur, terminal.pHistoryCur->dataOrig, terminal.pHistoryCur->lengthOrig);
							}

							terminal.pHistoryCur = terminal.pHistoryLast;

							terminal.pHistoryCur->lengthCur = 0;
							terminal.pHistoryCur->lengthOrig = 0;
							memset(terminal.pHistoryCur->dataOrig, 0, TERMINAL_INPUT_BUFFER_SIZE);
							memset(terminal.pHistoryCur->dataCur, 0, TERMINAL_INPUT_BUFFER_SIZE);
						}
					}
				}
			}
			break;
			case ESC_STATE_TYPE:
			{
				if(data == '[')
				{
					escapeMode = ESC_STATE_CSI;
					terminal.esc.paramPos = 0;
					terminal.esc.interPos = 0;
				}
				else
					escapeMode = ESC_STATE_NONE;
			}
			break;
			case ESC_STATE_CSI:
			{
				if(data >= 0x30 && data <= 0x3F)
				{
					// "parameter bytes" in the range 0x30–0x3F (ASCII 0–9:;<=>?)
					if(terminal.esc.paramPos < sizeof(terminal.esc.param))
						terminal.esc.param[terminal.esc.paramPos++] = data;
				}
				else if(data >= 0x20 && data <= 0x2F)
				{
					// "intermediate bytes" in the range 0x20–0x2F (ASCII space and !"#$%&'()*+,-./)
					if(terminal.esc.interPos < sizeof(terminal.esc.inter))
						terminal.esc.inter[terminal.esc.interPos++] = data;
				}
				else if(data >= 0x40 && data <= 0x7E)
				{
					// "final byte" in the range 0x40–0x7E (ASCII @A–Z[\]^_`a–z{|}~)
					terminal.esc.final = data;

					switch(terminal.esc.final)
					{
						case 'D': // left arrow
						{
							if(terminal.cursorPos > 0)
							{
								moveCursorLeft(1);
								terminal.cursorPos--;
							}
						}
						break;
						case 'C': // right arrow
						{
							if(terminal.cursorPos < terminal.pHistoryCur->lengthCur)
							{
								moveCursorRight(1);
								terminal.cursorPos++;
							}
						}
						break;
						case 'A': // up arrow
						{
							TerminalHistoryEntry* pLastValid = terminal.pHistoryLast + 1;
							if(pLastValid >= &terminal.history[TERMINAL_HISTORY_SIZE])
								pLastValid = terminal.history;

							TerminalHistoryEntry* pPrev = terminal.pHistoryCur - 1;
							if(pPrev < terminal.history)
								pPrev = &terminal.history[TERMINAL_HISTORY_SIZE-1];

							if(pPrev != pLastValid && pPrev->lengthCur > 0)
							{
								// valid entry
								terminal.pHistoryCur = pPrev;

								clearAndOutputCurrentEntry();
								terminal.cursorPos = terminal.pHistoryCur->lengthCur;
							}
						}
						break;
						case 'B': // down arrow
						{
							TerminalHistoryEntry* pNext = terminal.pHistoryCur + 1;
							if(pNext >= &terminal.history[TERMINAL_HISTORY_SIZE])
								pNext = terminal.history;

							if(terminal.pHistoryCur != terminal.pHistoryLast)
							{
								terminal.pHistoryCur = pNext;

								clearAndOutputCurrentEntry();
								terminal.cursorPos = terminal.pHistoryCur->lengthCur;
							}
						}
						break;
						case 'F': // end (xterm)
						{
							executeEnd();
						}
						break;
						case 'H': // home (xterm)
						{
							executeHome();
						}
						break;
						case '~': // "private key": home, end, insert, delete
						{
							switch(terminal.esc.param[0])
							{
								case '1': // home
								{
									executeHome();
								}
								break;
								case '2': // insert
								{
									// not used
								}
								break;
								case '3': // delete
								{
									if(terminal.pHistoryCur->lengthCur > 0)
									{
										if(terminal.pHistoryCur->lengthCur == terminal.cursorPos)
										{
											// no effect at end of text
										}
										else
										{
											// delete character under cursor
											memmove(&terminal.pHistoryCur->dataCur[terminal.cursorPos],
													&terminal.pHistoryCur->dataCur[terminal.cursorPos+1],
													TERMINAL_INPUT_BUFFER_SIZE - terminal.cursorPos);
											terminal.pHistoryCur->lengthCur--;

											clearAndOutputCurrentEntry();

											if(terminal.pHistoryCur->lengthCur != terminal.cursorPos)
												moveCursorLeft(terminal.pHistoryCur->lengthCur - terminal.cursorPos);
										}
									}
								}
								break;
								case '4': // end
								{
									executeEnd();
								}
								break;
							}
						}
						break;
					}

					escapeMode = ESC_STATE_NONE;
				}
				else
				{
					escapeMode = ESC_STATE_NONE;
				}
			}
			break;
		}
	}
}

void TerminalWrite(const uint8_t* pData, uint32_t dataLength)
{
	for(uint32_t i = 0; i < dataLength; i++)
	{
		chOQPutTimeout(terminal.pOutQueue, pData[i], TIME_IMMEDIATE);
	}
}
