/*
 * syscalls.c
 * newlib-nano system call stubs for the GCC build.
 *
 * _sbrk() (heap) is provided by Core/Src/sysmem.c (CubeMX generated),
 * which allocates from _end up to _estack - _Min_Stack_Size (see
 * CMake/stm32h723vgtx.ld).
 *
 * printf/vsnprintf etc. from nano-libc do not need real I/O; _write() is a
 * no-op that swallows output. Redirect it to a UART if you want printf to go
 * out over the debug serial port.
 */

#include <errno.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/times.h>
#include <sys/unistd.h>

int _close(int file)
{
  (void)file;
  return -1;
}

int _fstat(int file, struct stat *st)
{
  (void)file;
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file)
{
  (void)file;
  return 1;
}

int _lseek(int file, int ptr, int dir)
{
  (void)file;
  (void)ptr;
  (void)dir;
  return 0;
}

int _read(int file, char *ptr, int len)
{
  (void)file;
  (void)ptr;
  (void)len;
  return 0;
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  (void)ptr;
  return len;
}

int _getpid(void)
{
  return 1;
}

int _kill(int pid, int sig)
{
  (void)pid;
  (void)sig;
  errno = EINVAL;
  return -1;
}

void _exit(int status)
{
  (void)status;
  for (;;)
  {
    __asm volatile("nop");
  }
}

/* __gettimeofday needed by some newlib configurations. */
int _gettimeofday(struct timeval *tv, void *tz)
{
  (void)tv;
  (void)tz;
  return 0;
}

/* Called by assert()/abort() if they ever fire. */
void _abort(void)
{
  for (;;)
  {
    __asm volatile("nop");
  }
}
