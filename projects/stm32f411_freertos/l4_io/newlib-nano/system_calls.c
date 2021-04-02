/**
 * References: https://interrupt.memfault.com/blog/boostrapping-libc-with-newlib#enabling-newlib
 **/
#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>

#include "usart.h"

static const usart_e system_calls__uart_type = USART__1;

static void system_calls__polled_transmit(const char *ptr, int len) {
  for (int i = 0; i < len; i++) {
    usart__polled_transmit(system_calls__uart_type, ptr[i]);
  }
}

/**
 * NOTE: https://stackoverflow.com/questions/1716296/why-does-printf-not-flush-after-the-call-unless-a-newline-is-in-the-format-strin
 * printf does not flush the buffer unless there is "\n" in the string.
 * So printf's have to have "\n" at the end. e.g. printf("Hello World\n")
 * Alternatively, fprintf(stderr, "Hello World") can be used (will work without "\n")
 **/ 
int _write (int file, char * ptr, int len) {
  system_calls__polled_transmit(ptr, len);
  return len;
}

int _read (int file, char * ptr, int len) {
  return 0;
}

void* _sbrk(int requested_byte_count) {
  extern uint32_t __heap_start__;
  extern uint32_t __heap_end__;

  static void *next_free_heap = (void *)&__heap_start__;
  void *memory_to_return = next_free_heap;

  if (next_free_heap + requested_byte_count > (void *)&__heap_end__) {
    errno = ENOMEM;
    return (void *) -1;
  }

  next_free_heap += requested_byte_count;

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