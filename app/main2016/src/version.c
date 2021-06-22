/*
 * version.c
 *
 *  Created on: 04.02.2018
 *      Author: AndreR
 */

#include "main/version.h"

#define VERSION_MAJOR 11
#define VERSION_MINOR 12

#define xstr(s) str(s)
#define str(s) #s

#define VERSION_STRING "Firmware Version: " xstr(VERSION_MAJOR) "." xstr(VERSION_MINOR) "\r\n"

uint32_t VersionGet()
{
	return (VERSION_MAJOR << 16) | VERSION_MINOR;
}

const char* VersionGetString()
{
	return VERSION_STRING;
}
