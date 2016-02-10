/*
 * libc_stubs.c
 *
 *  Created on: Feb 10, 2016
 *      Author: narmstrong
 */
#include <sys/stat.h>
#include <usart.h>

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

int _open(const char *name, int flags, int mode) {
	return 1;
}

int _read(int file, char *ptr, int len) {
	return (HAL_UART_Receive(&huart3, ptr, len, HAL_MAX_DELAY) == HAL_OK ? 0 : -1);
}

char *heap_end = 0;
caddr_t _sbrk(int incr) {
	extern char heap_low; /* Defined by the linker */
	extern char heap_top; /* Defined by the linker */
	char *prev_heap_end;

	if (heap_end == 0) {
		heap_end = &heap_low;
	}
	prev_heap_end = heap_end;

	if (heap_end + incr > &heap_top) {
		/* Heap and stack collision */
		return (caddr_t) 0;
	}

	heap_end += incr;
	return (caddr_t) prev_heap_end;
}

int _write(int file, char *ptr, int len) {
	return (HAL_UART_Transmit(&huart3, ptr, len, HAL_MAX_DELAY) == HAL_OK ? 0 : -1);
}

void _init(void) {
	/* Nothing to do */
}
