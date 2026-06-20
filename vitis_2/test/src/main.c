#include <stdio.h>
#include <string.h>

#include "platform.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xil_cache.h"
#include "xaxidma.h"
#include "xstatus.h"

#include "netif/xadapter.h"

struct netif *echo_netif;

#define DMA_DEV_ID   XPAR_AXIDMA_0_DEVICE_ID

#define RX_WORDS     8192
#define RX_BYTES     (RX_WORDS * 8)

#define RX_REPEAT    10
#define DUMMY_REPEAT 1

#define DMA_TIMEOUT  10000000U

#define INIT_PATTERN 0xDEADBEEFDEADBEEFULL

static XAxiDma AxiDma;

static u64 RxBuffer[RX_REPEAT][RX_WORDS]
__attribute__ ((aligned(64)));

static u64 DummyBuffer[RX_WORDS]
__attribute__ ((aligned(64)));

static void print_s2mm_status(const char *tag)
{
    u32 cr  = XAxiDma_ReadReg(AxiDma.RegBase,
                              XAXIDMA_RX_OFFSET + XAXIDMA_CR_OFFSET);

    u32 sr  = XAxiDma_ReadReg(AxiDma.RegBase,
                              XAXIDMA_RX_OFFSET + XAXIDMA_SR_OFFSET);

    u32 len = XAxiDma_ReadReg(AxiDma.RegBase,
                              XAXIDMA_RX_OFFSET + XAXIDMA_BUFFLEN_OFFSET);

    xil_printf("[%s]\r\n", tag);
    xil_printf("  S2MM CR  = 0x%08lx\r\n", cr);
    xil_printf("  S2MM SR  = 0x%08lx\r\n", sr);
    xil_printf("  S2MM LEN = 0x%08lx\r\n", len);

    xil_printf("  Halted    = %lu\r\n", (sr >> 0) & 1);
    xil_printf("  Idle      = %lu\r\n", (sr >> 1) & 1);
    xil_printf("  DMAIntErr = %lu\r\n", (sr >> 4) & 1);
    xil_printf("  DMASlvErr = %lu\r\n", (sr >> 5) & 1);
    xil_printf("  DMADecErr = %lu\r\n", (sr >> 6) & 1);
    xil_printf("\r\n");
}

static int init_dma(void)
{
    XAxiDma_Config *CfgPtr;
    int Status;

    CfgPtr = XAxiDma_LookupConfig(DMA_DEV_ID);

    if (!CfgPtr) {
        xil_printf("No DMA config found\r\n");
        return XST_FAILURE;
    }

    Status = XAxiDma_CfgInitialize(&AxiDma, CfgPtr);

    if (Status != XST_SUCCESS) {
        xil_printf("DMA init failed: %d\r\n", Status);
        return XST_FAILURE;
    }

    if (XAxiDma_HasSg(&AxiDma)) {
        xil_printf("DMA is SG mode. Use Simple DMA mode.\r\n");
        return XST_FAILURE;
    }

    XAxiDma_Reset(&AxiDma);
    while (!XAxiDma_ResetIsDone(&AxiDma));

    print_s2mm_status("After DMA reset");

    xil_printf("DMA init OK\r\n");

    return XST_SUCCESS;
}

static void init_buffer(u64 *buf, u32 words)
{
    for (u32 i = 0; i < words; i++) {
        buf[i] = INIT_PATTERN;
    }

    Xil_DCacheFlushRange(
        (UINTPTR)buf,
        words * 8);
}

static int start_and_wait_s2mm(u64 *buf, u32 words)
{
    int Status;
    u32 timeout;
    u32 bytes = words * 8;

    Status = XAxiDma_SimpleTransfer(
        &AxiDma,
        (UINTPTR)buf,
        bytes,
        XAXIDMA_DEVICE_TO_DMA);

    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    timeout = DMA_TIMEOUT;

    while (XAxiDma_Busy(&AxiDma, XAXIDMA_DEVICE_TO_DMA)) {
        if (--timeout == 0) {
            return XST_FAILURE;
        }
    }

    return XST_SUCCESS;
}

int main()
{
    int Status;

    init_platform();

    xil_printf("\r\n==== AD9627 AXI DMA S2MM MULTI CAPTURE TEST ====\r\n");
    xil_printf("RX_WORDS  = %d\r\n", RX_WORDS);
    xil_printf("RX_BYTES  = %d\r\n", RX_BYTES);
    xil_printf("RX_REPEAT = %d\r\n", RX_REPEAT);
    xil_printf("FIFO DEPTH assumed = 32768 words\r\n\r\n");

    Status = init_dma();

    if (Status != XST_SUCCESS) {
        xil_printf("DMA init error\r\n");
        while (1);
    }

    for (int d = 0; d < DUMMY_REPEAT; d++) {

        xil_printf("\r\n==== DUMMY TRANSFER %d START ====\r\n", d + 1);

        init_buffer(DummyBuffer, RX_WORDS);

        Status = start_and_wait_s2mm(DummyBuffer, RX_WORDS);

        Xil_DCacheInvalidateRange(
            (UINTPTR)DummyBuffer,
            RX_BYTES);

        if (Status != XST_SUCCESS) {
            xil_printf("Dummy transfer failed\r\n");
            print_s2mm_status("Dummy failed");
            while (1);
        }

        xil_printf("Dummy transfer %d done, discarded\r\n", d + 1);
    }

    xil_printf("\r\n==== REAL DMA CAPTURES START ====\r\n");

    for (int run = 0; run < RX_REPEAT; run++) {
        init_buffer(RxBuffer[run], RX_WORDS);
    }

    for (int run = 0; run < RX_REPEAT; run++) {

        Status = start_and_wait_s2mm(RxBuffer[run], RX_WORDS);

        if (Status != XST_SUCCESS) {
            xil_printf("DMA capture %d failed\r\n", run + 1);
            print_s2mm_status("DMA failed");
            while (1);
        }
    }

    for (int run = 0; run < RX_REPEAT; run++) {
        Xil_DCacheInvalidateRange(
            (UINTPTR)RxBuffer[run],
            RX_BYTES);
    }

    xil_printf("All real DMA captures done\r\n");

    print_s2mm_status("After all DMA captures");

    xil_printf("\r\n==== START UART OUTPUT ====\r\n");

    xil_printf("CAPTURE_BEGIN,1\r\n");

    for (int run = 0; run < RX_REPEAT; run++) {
        for (int i = 0; i < RX_WORDS; i++) {

            int global_index = run * RX_WORDS + i;

            xil_printf("%d,%016llX\r\n",
                       global_index,
                       RxBuffer[run][i]);
        }
    }

    xil_printf("CAPTURE_END,1\r\n");

    xil_printf("\r\n==== ALL DMA TEST DONE ====\r\n");

    while (1);

    cleanup_platform();

    return 0;
}
