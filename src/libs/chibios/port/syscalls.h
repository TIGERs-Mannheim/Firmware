#pragma once

typedef int(*SyscallWriteFunc)(const char* pData, int length);

extern SyscallWriteFunc syscallWriteFunc;
