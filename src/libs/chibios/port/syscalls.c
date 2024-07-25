#include "syscalls.h"

#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>
#include <string.h>

#include "ch.h"

SyscallWriteFunc syscallWriteFunc;

#undef errno
extern int errno;

/*
 environ
 A pointer to a list of environment variables and their values.
 For a minimal environment, this empty list is adequate:
 */
//char *__env[1] = { 0 };
//char **environ = __env;

int _write(int file, char *ptr, int len);

void _exit(int status)
{
	(void)status;

	chSysHalt("exit called");

	while(1);
}

int _close(int file)
{
	(void)file;

	return -1;
}
/*
 execve
 Transfer control to a new process. Minimal implementation (for a system without processes):
 */
int _execve(char *name, char **argv, char **env)
{
	(void)name;
	(void)argv;
	(void)env;

	errno = ENOMEM;
	return -1;
}
/*
 fork
 Create a new process. Minimal implementation (for a system without processes):
 */

int _fork()
{
	errno = EAGAIN;
	return -1;
}
/*
 fstat
 Status of an open file. For consistency with other minimal implementations in these examples,
 all files are regarded as character special devices.
 The `sys/stat.h' header file required is distributed in the `include' subdirectory for this C library.
 */
int _fstat(int file, struct stat *st)
{
	(void)file;

	st->st_mode = S_IFCHR;
	return 0;
}

/*
 getpid
 Process-ID; this is sometimes used to generate strings unlikely to conflict with other processes. Minimal implementation, for a system without processes:
 */

int _getpid(void)
{
	return 1;
}

/*
 isatty
 Query whether output stream is a terminal. For consistency with the other minimal implementations,
 */
int _isatty(int file)
{
	(void)file;

	return 1;
}

/*
 kill
 Send a signal. Minimal implementation:
 */
int _kill(int pid, int sig)
{
	(void)pid;
	(void)sig;

	errno = EINVAL;
	return (-1);
}

/*
 link
 Establish a new name for an existing file. Minimal implementation:
 */

int _link(char *old, char *new)
{
	(void)old;
	(void)new;

	errno = EMLINK;
	return -1;
}

/*
 lseek
 Set position in a file. Minimal implementation:
 */
int _lseek(int file, int ptr, int dir)
{
	(void)file;
	(void)ptr;
	(void)dir;

	return 0;
}

/*
 sbrk
 Increase program data space.
 Malloc and related functions depend on this
 */
caddr_t _sbrk(int incr)
{
#if CH_CFG_USE_MEMCORE
	void *p;

	if(incr == 0)
		return (caddr_t)0;

	p = chCoreAlloc((size_t)incr);
	if (p == NULL)
	{
		errno = ENOMEM;
		chSysHalt("Out of memory");
		return (caddr_t)-1;
	}
	return (caddr_t)p;
#else
	(void)incr;
	errno = ENOMEM;
	return (caddr_t) -1;
#endif
}

/*
 read
 Read a character to a file. `libc' subroutines will use this system routine for input from all files, including stdin
 Returns -1 on error or blocks until the number of characters have been read.
 */

int _read(int file, char *ptr, int len)
{
	(void)file;
	(void)ptr;
	(void)len;

	errno = EBADF;
	return -1;
}

/*
 stat
 Status of a file (by name). Minimal implementation:
 int    _EXFUN(stat,( const char *__path, struct stat *__sbuf ));
 */
int _stat(const char *filepath, struct stat *st)
{
	(void) filepath;

	memset(st, 0, sizeof(*st));
	st->st_mode = S_IFCHR;
	return 0;
}

/*
 times
 Timing information for current process. Minimal implementation:
 */
clock_t _times(struct tms *buf)
{
	(void)buf;

	return -1;
}

/*
 unlink
 Remove a file's directory entry. Minimal implementation:
 */
int _unlink(char *name)
{
	(void)name;

	errno = ENOENT;
	return -1;
}

/*
 wait
 Wait for a child process. Minimal implementation:
 */
int _wait(int *status)
{
	(void)status;

	errno = ECHILD;
	return -1;
}

/*
 write
 Write a character to a file. `libc' subroutines will use this system routine for output to all files, including stdout
 Returns -1 on error or number of bytes sent
 */
int _write(int file, char *ptr, int len)
{
	if(syscallWriteFunc)
	{
		if(file == STDOUT_FILENO)
		{
			return (*syscallWriteFunc)(ptr, len);
		}
		else if(file == STDERR_FILENO)
		{
			const char* pYellow = "\e[33m";
			const char* pReset = "\e[0m";

			(*syscallWriteFunc)(pYellow, 5);
			int written = (*syscallWriteFunc)(ptr, len);
			(*syscallWriteFunc)(pReset, 4);
			return written;
		}
	}

	errno = EBADF;
	return -1;
}
