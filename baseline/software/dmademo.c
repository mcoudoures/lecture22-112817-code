#include <stdio.h>
#include <sys/stat.h> 
#include <fcntl.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <assert.h>
#include <string.h>

#define _GNU_SOURCE
#include <sys/types.h>
#include <sys/syscall.h>
#include <sys/mman.h>
#include <linux/perf_event.h>
#include "hps_0.h"
#include "socal/hps.h"

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

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

typedef signed char           sample_t;
typedef volatile signed char *sample_hptr;
typedef unsigned              index_t;

#define DMA_REG_STATUS  0
#define DMA_REG_READ    1
#define DMA_REG_WRITE   2
#define DMA_REG_LENGTH  3
#define DMA_REG_CONTROL 6

#define DMA_STATUS_DONE 0x1
#define DMA_STATUS_BUSY 0x2

#define DMA_CONTROL_BYTE   0x1
#define DMA_CONTROL_WORD   0x3
#define DMA_CONTROL_START  0x8
#define DMA_CONTROL_ENDLEN 0x80
#define DMA_CONTROL_RCONST 0x100
#define DMA_CONTROL_WCONST 0x200
#define DMA_CONTROL_RESET  0x1000

int dmacomplete(volatile unsigned *dmabase) {
  if ((dmabase[DMA_REG_STATUS] & 0x1) == 0x1) {
    // clear the start bit
    dmabase[DMA_REG_CONTROL] &= ~(DMA_CONTROL_START);
    // clear the done bit
    dmabase[DMA_REG_STATUS]  = 0;
    return 1;
  }
  return 0;
}

void dmareset(volatile unsigned *dmabase) {
  dmabase[DMA_REG_CONTROL] = DMA_CONTROL_RESET;
}

void dmacopy(volatile unsigned *dmabase,
	     unsigned source,
	     unsigned destination,
	     unsigned nbytes) {
  // clear done bit
  dmabase[DMA_REG_STATUS]  = 0;
  // program the transaction
  dmabase[DMA_REG_LENGTH]  = nbytes;
  dmabase[DMA_REG_READ]    = source;
  dmabase[DMA_REG_WRITE]   = destination;
  // start the transaction
  dmabase[DMA_REG_CONTROL] =
    DMA_CONTROL_BYTE |
    DMA_CONTROL_ENDLEN |
    DMA_CONTROL_START;
}

// bulk storage in FPGA on AXI bus
sample_hptr mem1;
sample_hptr mem2; 
volatile unsigned *dmabase;
volatile unsigned *ledpio;

void *peripherals;
sample_hptr        onchipmem;

int main() {
  long long c1;
  
  int memfd;  
  if( ( memfd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
    printf( "ERROR: could not open \"/dev/mem\"...\n" );
    return( 1 );
  }

  peripherals = mmap( NULL,
		      HW_REGS_SPAN,
		      ( PROT_READ | PROT_WRITE ),
		      MAP_SHARED,
		      memfd,
		      HW_REGS_BASE );
  if (peripherals == MAP_FAILED) {
    printf("ERROR: mmap() failed ...\n");
    return 1;
  }

  printf("HW_REGS_BASE %x\n", HW_REGS_BASE);
  printf("ALT_LWFPGASLVS_OFST %x\n", ALT_LWFPGASLVS_OFST);
  
  dmabase = peripherals +
    ( ( unsigned )( ALT_LWFPGASLVS_OFST + DMA_0_BASE) &
      ( unsigned )( HW_REGS_MASK ) );
  ledpio = peripherals +
    ( ( unsigned )( ALT_LWFPGASLVS_OFST + LED_PIO_0_BASE) &
      ( unsigned )( HW_REGS_MASK ) );

  // mem1 points to ONCHIP_MEMORY2_0, 128Kbyte source memory
  mem1 = mmap( NULL,
	       ONCHIP_MEMORY2_0_SPAN,
	       ( PROT_READ | PROT_WRITE ),
	       MAP_SHARED,
	       memfd,
	       0xC0000000 + ONCHIP_MEMORY2_0_BASE);
  if (mem1 == MAP_FAILED) {
    printf("ERROR: mmap() failed ...\n");
    return 1;
  }
  
  // mem1 points to ONCHIP_MEMORY2_0, 128Kbyte destination memory
  mem2 = mmap( NULL,
	       ONCHIP_MEMORY2_1_SPAN,
	       ( PROT_READ | PROT_WRITE ),
	       MAP_SHARED,
	       memfd,
	       0xC0000000 + ONCHIP_MEMORY2_1_BASE);
  if (mem2 == MAP_FAILED) {
    printf("ERROR: mmap() failed ...\n");
    return 1;
  }

  printf("mem1: %x\n", (unsigned) mem1);
  printf("mem2: %x\n", (unsigned) mem2);
  printf("dma:  %x\n", (unsigned) dmabase);
  printf("led:  %x\n", (unsigned) ledpio);
  
  // increment every location of source memory
  unsigned i;
  for (i=0; i<128*1024; i++)
    mem1[i] = mem1[i] + 1;

  printf("dmareset\n");
  dmareset(dmabase);
  
  printf("dmacopy\n");
  c1 = cpucycles();
  // DMA transaction from source memory to destination memory
  //
  // the source address is determined by the address space
  // of the DMA read master (check QSYS memory space)
  //
  // mem1 - 0x20000 - 3FFFF
  // mem2 - 0x00000 - 1FFFF
  //
  // the destination address is determined by the address
  // space of the DMA write master
  //
  // mem2 - 0x00000 - 1FFFF
  
  dmacopy (dmabase,
    	   0x20000  /* source      */,
    	   0x00000  /* destination */,
    	   1024*128 /* bytes       */
    	   );
  while (! dmacomplete(dmabase)) ;
  c1 = cpucycles() - c1;

  printf("Done. DMA copy takes %lld cycles\n", c1);

  c1 = cpucycles();
  for (i=0; i<2*65536; i++)
    mem2[i] = mem1[i];
  c1 = cpucycles() - c1;
  
  printf("Done. CPU copy takes %lld cycles\n", c1);

  //  for (i=0; i<64; i++)
  //    printf("%3x %3x %3x\n", i, mem1[i], mem2[i]);
  
  close(memfd);
  
  return 0;
}
