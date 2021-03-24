/**
 * References: https://interrupt.memfault.com/blog/boostrapping-libc-with-newlib#enabling-newlib
 **/
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>

register char* stack_ptr asm("sp");

int _write (int file, char * ptr, int len) {
  return -1;
}

int _read (int file, char * ptr, int len) {
  return 0;
}

void* _sbrk(int incr) {
  return NULL;
}

int _open(const char *path, int flags, ...) {
  return 0;
}

int _close(int file) {
  return -1;
}

int _fstat(int file, struct stat *st) {
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file) {
  return 1;
}

int _lseek(int file, int ptr, int dir) {
  return 0;
}

void _exit(int status) {
  while(1);
}

void _kill(int pid, int sig) {
  return;
}

int _getpid(void) {
  return -1;
}