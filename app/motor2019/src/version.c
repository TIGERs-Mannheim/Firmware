/*
 * version.c
 *
 *  Created on: 01.07.2019
 *      Author: AndreR
 */

#define VERSION_MAJOR 2
#define VERSION_MINOR 1

#define xstr(s) str(s)
#define str(s) #s

#define VERSION_STRING "Firmware Version: " xstr(VERSION_MAJOR) "." xstr(VERSION_MINOR) "\r\n"

static const char version[] __attribute__((used)) = VERSION_STRING;
