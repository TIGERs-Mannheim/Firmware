#pragma once

// Based on: https://github.com/rideskip/anchor/tree/master/console

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#if defined(__GNUC__)

// The _SHELL_NUM_ARGS(...) macro returns the number of arguments passed to it (0-10)
#define _SHELL_NUM_ARGS(...) _SHELL_NUM_ARGS_HELPER1(_, ##__VA_ARGS__, _SHELL_NUM_ARGS_SEQ())
#define _SHELL_NUM_ARGS_HELPER1(...) _SHELL_NUM_ARGS_HELPER2(__VA_ARGS__)
#define _SHELL_NUM_ARGS_HELPER2(_1,_2,_3,_4,_5,_6,_7,_8,_9,_10,N,...) N
#define _SHELL_NUM_ARGS_SEQ() 9,8,7,6,5,4,3,2,1,0
_Static_assert(_SHELL_NUM_ARGS() == 0, "_SHELL_NUM_ARGS() != 0");
_Static_assert(_SHELL_NUM_ARGS(1) == 1, "_SHELL_NUM_ARGS(1) != 1");
_Static_assert(_SHELL_NUM_ARGS(1,2,3,4,5,6,7,8,9,10) == 10, "_SHELL_NUM_ARGS(<10_args>) != 10");

// General helper macro for concatentating two tokens
#define _SHELL_CONCAT(A, B) _SHELL_CONCAT2(A, B)
#define _SHELL_CONCAT2(A, B) A ## B

// Helper macro for calling a given macro for each of 0-10 arguments
#define _SHELL_MAP(X, ...) _SHELL_MAP_HELPER(_SHELL_CONCAT(_SHELL_MAP_, _SHELL_NUM_ARGS(__VA_ARGS__)), X, ##__VA_ARGS__)
#define _SHELL_MAP_HELPER(C, X, ...) C(X, ##__VA_ARGS__)
#define _SHELL_MAP_0(X)
#define _SHELL_MAP_1(X,_1) X(_1)
#define _SHELL_MAP_2(X,_1,_2) X(_1) X(_2)
#define _SHELL_MAP_3(X,_1,_2,_3) X(_1) X(_2) X(_3)
#define _SHELL_MAP_4(X,_1,_2,_3,_4) X(_1) X(_2) X(_3) X(_4)
#define _SHELL_MAP_5(X,_1,_2,_3,_4,_5) X(_1) X(_2) X(_3) X(_4) X(_5)
#define _SHELL_MAP_6(X,_1,_2,_3,_4,_5,_6) X(_1) X(_2) X(_3) X(_4) X(_5) X(_6)
#define _SHELL_MAP_7(X,_1,_2,_3,_4,_5,_6,_7) X(_1) X(_2) X(_3) X(_4) X(_5) X(_6) X(_7)
#define _SHELL_MAP_8(X,_1,_2,_3,_4,_5,_6,_7,_8) X(_1) X(_2) X(_3) X(_4) X(_5) X(_6) X(_7) X(_8)
#define _SHELL_MAP_9(X,_1,_2,_3,_4,_5,_6,_7,_8,_9) X(_1) X(_2) X(_3) X(_4) X(_5) X(_6) X(_7) X(_8) X(_9)
#define _SHELL_MAP_10(X,_1,_2,_3,_4,_5,_6,_7,_8,_9,_10) X(_1) X(_2) X(_3) X(_4) X(_5) X(_6) X(_7) X(_8) X(_9) X(_10)

// Helper macros for defining shell argument instances.
#define _SHELL_ARG_HELPER(...) _SHELL_ARG_HELPER2 __VA_ARGS__
#define _SHELL_ARG_HELPER2(NAME, DESC) \
    { .name = #NAME, .desc = DESC },

#else // !defined(__GNUC__)
#error "Unsupported toolchain"
#endif

// command handler type
typedef void(*ShellCmdFunc)(void*, int, const char**);

typedef struct {
    // The name of the argument
    const char* name;
    // A description of the argument for display by the "help" command
    const char* desc;
} ShellArg;

typedef struct {
    // The name of the command (must be a valid C symbol name, consist of only lower-case alphanumeric characters, and be unique)
    const char* pName;
    // A description of the command for display by the "help" command
    const char* pDesc;
    // The command handler
    ShellCmdFunc handler;
    // List of argument definitions
    const ShellArg* pArgs;
    // The number of arguments
    uint32_t numArgs;
} ShellCommand;

#define SHELL_CMD(CMD, DESC, ...) \
    static void CMD##_command_handler(void* pUser, int argc, const char** argv); \
    static const ShellArg _##CMD##_ARGS_DEF[] = { \
        _SHELL_MAP(_SHELL_ARG_HELPER, ##__VA_ARGS__) \
    }; \
    static const ShellCommand _##CMD##_DEF = { \
        .pName = #CMD, \
        .pDesc = DESC, \
        .handler = (ShellCmdFunc)CMD##_command_handler, \
        .pArgs = _##CMD##_ARGS_DEF, \
        .numArgs = sizeof(_##CMD##_ARGS_DEF) / sizeof(ShellArg), \
    }; \
    static const ShellCommand* const CMD##_command = &_##CMD##_DEF

#define SHELL_CMD_IMPL(CMD) static void CMD##_command_handler(void* pUser, int argc, const char** argv)

#define SHELL_ARG(NAME, DESC) (NAME, DESC)

#define SHELL_CMD_MAX_CMDS 50
#define SHELL_CMD_MAX_ARGS 10

typedef struct _ShellCmdHandler
{
	const ShellCommand* commands[SHELL_CMD_MAX_CMDS];
	size_t commandsUsed;

	void* pUser;
} ShellCmdHandler;

void ShellCmdHandlerInit(ShellCmdHandler* pHandler, void* pUser);
void ShellCmdAdd(ShellCmdHandler* pHandler, const ShellCommand* pCmd);
void ShellCmdExecute(ShellCmdHandler* pHandler, char* pCmdText);
void ShellCmdForward(ShellCmdHandler* pHandler, int argc, const char** argv);
