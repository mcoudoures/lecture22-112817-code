#include <stdio.h>
#include <sys/stat.h> 
#include <fcntl.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <assert.h>
#include <string.h>

#ifndef NODE1SOC
#define _GNU_SOURCE
#include <sys/types.h>
#include <sys/syscall.h>
#include <sys/mman.h>
#include <linux/perf_event.h>
#include "hps_0.h"
#include "socal/hps.h"

static int fddev = -1;
__attribute__((constructor)) static void init(void) {
        static struct perf_event_attr attr;
        attr.type = PERF_TYPE_HARDWARE;
        attr.config = PERF_COUNT_HW_CPU_CYCLES;
        fddev = syscall(__NR_perf_event_open, &attr, 0, -1, -1, 0);
}

__attribute__((destructor)) static void fini(void) {
  close(fddev);
}

static inline long long cpucycles(void) {
  long long result = 0;
  if (read(fddev, &result, sizeof(result)) < sizeof(result)) return 0;
  return result;
}
#endif

typedef signed char           sample_t;
typedef volatile signed char *sample_hptr;
typedef unsigned              index_t;

#ifdef NODE1SOC
// bulk storage in main memory
sample_t    bulk[N1*Q1*S1];
#else
// bulk storage in FPGA on AXI bus
sample_hptr bulk; // [N1*Q1*S1];
#endif

int main() {

#ifndef NODE1SOC  
  int memfd;  
  if( ( memfd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
    printf( "ERROR: could not open \"/dev/mem\"...\n" );
    return( 1 );
  }
  
  bulk = mmap( NULL, 
	       ONCHIP_MEMORY2_0_SPAN, 
	       ( PROT_READ | PROT_WRITE ), 
	       MAP_SHARED, 
	       memfd, 
	       0xC0000000 + ONCHIP_MEMORY2_0_BASE );
  
  if( bulk == MAP_FAILED ) {
    printf( "ERROR: mmap() failed...\n" );
    close( memfd );
    return(1);
  }
#endif

  unsigned *v = (unsigned *) bulk;
  unsigned i;

  for (i=0; i<16; i++)
    v[i] = ~i;

  for (i=0; i<64; i++)
    printf("%3x %3x\n", i, bulk[i]);
  

  
  return 0;
}
