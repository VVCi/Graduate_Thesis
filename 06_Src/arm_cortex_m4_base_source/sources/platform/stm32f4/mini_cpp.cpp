#include <cstdlib>
#include <sys/types.h>
#include "../../sys/sys_dbg.h"

/*
 * The default pulls in 70K of garbage
 */

namespace __gnu_cxx {

void __verbose_terminate_handler() {
	for(;;);
}

}


/*
 * The default pulls in about 12K of garbage
 */

extern "C" void __cxa_pure_virtual() {
	FATAL("C++", 0x01);
}

/*
 * Implement C++ new/delete operators using the heap
 */

void *operator new(size_t size) {
	return malloc(size);
}

void *operator new[](size_t size) {
	return malloc(size);
}

void operator delete(void *p) {
	free(p);
}

void operator delete[](void *p) {
	free(p);
}


/*
 * sbrk function for getting space for malloc and friends
 */

/* end of stack region */
extern int  _ebss;

extern "C" {
caddr_t _sbrk ( int incr ) {

	static unsigned char *heap = NULL;
	unsigned char *prev_heap;

	if (heap == NULL) {
		heap = (unsigned char *)&_ebss;
	}
	prev_heap = heap;
	/* check removed to show basic approach */

	heap += incr;

	return (caddr_t) prev_heap;
}
}
