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

void dmacopyconst(volatile unsigned *dmabase,
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
    DMA_CONTROL_WCONST |
    DMA_CONTROL_START;
}

//------------------------------------------------------------------
void clearsum(volatile unsigned *sumbase) {
  sumbase[1] = 0;
}

void accsumbyte(volatile unsigned *sumbase, unsigned char a1) {
  ((volatile unsigned char *)sumbase)[0] = a1;
}

void accsumword(volatile unsigned *sumbase, unsigned a1) {
  sumbase[0] = a1;
}

int sumout(volatile unsigned *sumbase) {
  return sumbase[0];
}
//------------------------------------------------------------------


// bulk storage in FPGA on AXI bus
sample_hptr mem1;
sample_hptr mem2; 
volatile unsigned *dmabase;
volatile unsigned *sumbase;
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

  sumbase = mmap( NULL,
		  MYSUM_0_SPAN,
		  ( PROT_READ | PROT_WRITE ),
		  MAP_SHARED,
		  memfd,
		  0xC0000000 + MYSUM_0_BASE);
  if (sumbase == MAP_FAILED) {
    printf("ERROR: mmap() failed ...\n");
    return 1;
  }

  printf("mem1:    %x\n", (unsigned) mem1   );
  printf("mem2:    %x\n", (unsigned) mem2   );
  printf("sumbase: %x\n", (unsigned) sumbase);
  printf("dma:     %x\n", (unsigned) dmabase);
  printf("led:     %x\n", (unsigned) ledpio );
  
  clearsum(sumbase);
  accsumword(sumbase, 0x01020304);
  accsumword(sumbase, 0x05060708);
  printf("Sumout %d\n", sumout(sumbase));
  accsumbyte(sumbase, 1);
  accsumbyte(sumbase, 2);
  accsumbyte(sumbase, 3);
  printf("Sumout %d\n", sumout(sumbase));
  clearsum(sumbase);
  accsumword(sumbase, 0x01020304);
  accsumword(sumbase, 0x05060708);
  printf("Sumout %d\n", sumout(sumbase));

  clearsum(sumbase);

  // fill mem1 with bytes
  unsigned i;
  for (i=0; i<1024*128; i++) {
    mem1[i] = i;
  }

  // sum it
  c1 = cpucycles();
  unsigned w=0;
  for (i=0; i<128*1024; i++)
    w += ((unsigned char) mem1[i]);
  c1 = cpucycles() - c1;
  
  printf("Done. Summing by CPU total %x cycles %lld\n", w, c1);

  
  // sum it by dma
  c1 = cpucycles();
  dmareset(dmabase);
  dmacopyconst(dmabase,
	       DMA_0_READ_MASTER_ONCHIP_MEMORY2_0_BASE,
	       DMA_0_WRITE_MASTER_MYSUM_0_BASE,
	       1024*128
	       );
  while (! dmacomplete(dmabase)) ;
  c1 = cpucycles() - c1;
  
  printf("Done. Summing by DMA (bytes) total %x cycles %lld\n", sumout(sumbase), c1);

  close(memfd);
  
  return 0;
}
