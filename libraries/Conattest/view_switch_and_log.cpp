#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/mman.h>

#define handle_error(msg) \
	do { perror(msg); exit(EXIT_FAILURE); } while (0)

// #define PROT_READ        0x1                /* Page can be read.  */
// #define PROT_WRITE       0x2                /* Page can be written.  */
// #define PROT_EXEC        0x4                /* Page can be executed.  */
// #define PROT_NONE        0x0                /* Page can not be accessed.  */

extern "C" { // C naming instead of C++ mangling
	
	void view_switch_to_rd_and_log()
	{
		// if (mprotect((void *)0x600000, 0x200000, PROT_READ ) == -1)
		//   handle_error("mprotect");
		printf("view_switch_to_rd_and_log\n");	
	}


	void view_switch_to_text_and_log()
	{
		// if (mprotect((void *)0x400000, 0x200000, PROT_READ) == -1)
		//   handle_error("mprotect");
		printf("view_switch_to_text_and_log\n");	
	}

}