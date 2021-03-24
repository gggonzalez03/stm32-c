/**
 * References: https://interrupt.memfault.com/blog/boostrapping-libc-with-newlib#enabling-newlib
 **/
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>
#include "stm32f411xe.h"

register void* stack_ptr asm("sp");

/**
 * Note: https://stackoverflow.com/questions/1716296/why-does-printf-not-flush-after-the-call-unless-a-newline-is-in-the-format-strin
 * printf does not flush the buffer unless there is "\n" in the string.
 * So printf's have to have "\n" at the end. e.g. printf("Hello World\n")
 * Alternatively, fprintf(stderr, "Hello World") can be used (will work without "\n")
 **/ 
int _write (int file, char * ptr, int len) {
  GPIOC->ODR ^= (1UL << 13);
  return 0;
}

int _read (int file, char * ptr, int len) {
  return 0;
}

void* _sbrk(int requested_byte_count) {
  extern uint32_t __heap_start__;

  static void *next_free_heap = (void *)&__heap_start__;
  void *memory_to_return = next_free_heap;

  if (next_free_heap + requested_byte_count > stack_ptr) {
    memory_to_return = NULL;
  }

  if (memory_to_return != NULL) {
    next_free_heap += requested_byte_count;
  }

  return memory_to_return;
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