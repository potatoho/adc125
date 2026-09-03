#define _GNU_SOURCE
#define _FILE_OFFSET_BITS 64

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

/*
 * PetaLinux/Linux userspace version of the original Vitis bare-metal
 * application.
 *
 * Hardware access model:
 *   - adc_ctrl AXI4-Lite registers are accessed through /dev/mem.
 *   - AXI DMA S2MM simple-mode registers are accessed through /dev/mem.
 *   - RX buffers are mapped through /dev/mem.
 *
 * Important:
 *   The RX buffer physical range must be reserved in the device tree and must
 *   not be used by Linux. The AXI DMA kernel driver must not be bound to the
 *   same DMA instance while this program controls its registers.
 */

/* ============================================================
 * Status
 * ============================================================ */

#define APP_SUCCESS                     0
#define APP_FAILURE                     (-1)

/* ============================================================
 * DMA/data configuration
 * ============================================================ */

#define MAX_FRAME_WORDS                 131072U
#define MIN_FRAME_WORDS                 1U
#define PL_DEFAULT_FRAME_WORDS          131072U
#define PL_DEFAULT_PRETRIGGER_WORDS     16384U
#define PL_FORCE_FOOTER                 1U

#define PL_FOOTER_WORDS                 17U
#define PL_FOOTER_MAGIC                 0x00000000FEE70001ULL
#define MAX_DMA_WORDS                   (MAX_FRAME_WORDS + PL_FOOTER_WORDS)
#define MAX_DMA_BYTES                   (MAX_DMA_WORDS * 8U)

#define DEFAULT_ADC_CTRL_PHYS           UINT64_C(0xA0010000)
#define DEFAULT_DMA_PHYS                UINT64_C(0xA0000000)
#define DEFAULT_RX_BUFFER0_PHYS         UINT64_C(0x10000000)
#define DEFAULT_RX_BUFFER1_PHYS         UINT64_C(0x10200000)

/* AXI DMA register map, S2MM/simple mode. */
#define AXIDMA_MAP_BYTES                0x1000U
#define AXIDMA_S2MM_OFFSET              0x30U
#define AXIDMA_CR_OFFSET                0x00U
#define AXIDMA_SR_OFFSET                0x04U
#define AXIDMA_DSTADDR_OFFSET           0x18U
#define AXIDMA_DSTADDR_MSB_OFFSET       0x1CU
#define AXIDMA_LENGTH_OFFSET            0x28U

#define AXIDMA_CR_RUNSTOP               0x00000001U
#define AXIDMA_CR_RESET                 0x00000004U
#define AXIDMA_CR_IOC_IRQEN             0x00001000U
#define AXIDMA_CR_ERR_IRQEN             0x00004000U

#define AXIDMA_SR_HALTED                0x00000001U
#define AXIDMA_SR_IDLE                  0x00000002U
#define AXIDMA_SR_ERR_ALL               0x00000770U
#define AXIDMA_SR_IRQ_ALL               0x00007000U
#define AXIDMA_SR_IOC_IRQ               0x00001000U

/* ============================================================
 * Ethernet/UDP configuration
 * ============================================================ */

#define DEFAULT_PC_IP                   "192.168.1.71"
#define DEFAULT_PC_UDP_PORT             5000U
#define DEFAULT_CONTROL_UDP_PORT        5001U

#define DEFAULT_UDP_ADC_PAYLOAD_BYTES   8192U
#define MIN_UDP_ADC_PAYLOAD_BYTES       512U
#define MAX_UDP_ADC_PAYLOAD_BYTES       8952U

#define UDP_TX_RETRY_COUNT              10000U
#define UDP_TX_RETRY_DELAY_US           100U
#define UDP_PACKET_GAP_US               0U
#define UDP_SOCKET_BUFFER_BYTES         (4U * 1024U * 1024U)
#define UDP_TX_FAILURE_DRAIN_MS         50U
#define UDP_TX_FAILURE_BACKOFF_STEP_MS  25U
#define UDP_TX_FAILURE_BACKOFF_MAX_MS   250U

#define UDP_MAGIC                       0xADC64096U

#define CTRL_CMD_MAGIC                  0x4354524CU  /* CTRL */
#define CTRL_REPLY_MAGIC                0x43545252U  /* CTRR */

#define CTRL_OP_READ                    1U
#define CTRL_OP_WRITE                   2U
#define CTRL_OP_DUMP                    3U

#define CTRL_STATUS_OK                  0U
#define CTRL_STATUS_BAD_OP              1U
#define CTRL_STATUS_BAD_REG             2U
#define CTRL_STATUS_BAD_LEN             3U

#define DMA_WAIT_RECONFIGURE            2
#define DMA_WAIT_SOFTWARE_RESET         3

/* ============================================================
 * adc_ctrl register map
 * ============================================================ */

#define ADC_CTRL_MAP_BYTES              0x1000U

#define ADC_CTRL_REG_CONTROL            0U  /* byte offset 0x00: control */
#define ADC_CTRL_REG_STATUS             1U  /* byte offset 0x04: read-only PL status */
#define ADC_CTRL_REG_FRAME_SIZE         2U
#define ADC_CTRL_REG_PRETRIGGER_SIZE    3U
#define ADC_CTRL_REG_SAMPLE_DECIMATION  4U
#define ADC_CTRL_REG_TRIGGER_CFG        5U
#define ADC_CTRL_REG_SELF_THRESHOLD     6U
#define ADC_CTRL_REG_CHANNEL_MASK       7U
#define ADC_CTRL_REG_OUTPUT_CFG         8U
#define ADC_CTRL_REG_BASELINE_SHIFT     9U

#define ADC_CTRL_MAX_REG                ADC_CTRL_REG_BASELINE_SHIFT

#define DEFAULT_FRAME_WORDS             PL_DEFAULT_FRAME_WORDS
#define DEFAULT_PRETRIGGER_WORDS        PL_DEFAULT_PRETRIGGER_WORDS
#define DEFAULT_SAMPLE_DECIMATION       1U
#define DEFAULT_TRIGGER_CFG             0x00000001U
#define DEFAULT_SELF_THRESHOLD          1680U
#define DEFAULT_CHANNEL_MASK            0x00000001U

#define OUTPUT_CFG_BASELINE_CORRECT     0x00000001U
#define OUTPUT_CFG_APPEND_FOOTER        0x00000002U
#define OUTPUT_CFG_MASK_OUTPUT_CHANNEL  0x00000004U
#define OUTPUT_CFG_PEAK_SIGNED_MAX      0x00000008U
#define OUTPUT_CFG_FOOTER_ONLY          0x00000010U
#define OUTPUT_CFG_RAW_EVERY_N          0x00000040U
#define OUTPUT_CFG_RAW_PERIOD_SHIFT     8U
#define OUTPUT_CFG_RAW_PERIOD_MASK      0x00FFFFFFU

#define DEFAULT_OUTPUT_CFG              OUTPUT_CFG_APPEND_FOOTER
#define DEFAULT_BASELINE_SHIFT          10U

#define CONTROL_SOFT_TRIGGER            0x00000002U
#define CONTROL_ACQUISITION_ARM         0x00000004U
#define CONTROL_FIFO_ALARM_CLEAR        0x00000008U
#define CONTROL_SOFTWARE_RESET          0x00000010U
#define CONTROL_ACQUISITION_ENABLE      0x00000020U

#define PL_STATUS_ADC1_FIFO_FULL        0x00000001U  /* adc_ctrl[0x04] bit0 */
#define PL_STATUS_ADC2_FIFO_FULL        0x00000002U  /* adc_ctrl[0x04] bit1 */
#define PL_STATUS_ANY_FIFO_FULL         0x00000004U  /* adc_ctrl[0x04] bit2 */
#define PL_STATUS_CAPTURE_OVERRUN       0x00000008U  /* adc_ctrl[0x04] bit3 */

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint32_t frame_id;
    uint16_t packet_index;
    uint16_t total_packets;
    uint16_t payload_bytes;
    uint16_t reserved;
    uint32_t frame_bytes;
} UdpAdcHeader;

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint32_t seq;
    uint16_t op;
    uint16_t reg;
    uint32_t value;
} ControlCommand;

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint32_t seq;
    uint16_t status;
    uint16_t reg;
    uint32_t value;

    uint32_t control;
    uint32_t frame_size;
    uint32_t pretrigger_size;
    uint32_t sample_decimation;
    uint32_t trigger_cfg;
    uint32_t self_threshold;
    uint32_t channel_mask;
    uint32_t output_cfg;
    uint32_t baseline_shift;
} ControlReply;

#define UDP_HEADER_BYTES                ((uint32_t)sizeof(UdpAdcHeader))
#define UDP_PACKET_BYTES                (UDP_HEADER_BYTES + MAX_UDP_ADC_PAYLOAD_BYTES)

typedef struct
{
    uint64_t adc_ctrl_phys;
    uint64_t dma_phys;
    uint64_t rx_phys[2];
    const char *pc_ip;
    uint16_t data_port;
    uint16_t control_port;
    uint16_t udp_payload_bytes;
    int init_adc_ctrl;
} AppConfig;

typedef struct
{
    int fd;
    void *map_base;
    size_t map_size;
    off_t page_base;
    off_t page_offset;
} MmioMap;

/* ============================================================
 * Global objects
 * ============================================================ */

static AppConfig Config =
{
    .adc_ctrl_phys = DEFAULT_ADC_CTRL_PHYS,
    .dma_phys = DEFAULT_DMA_PHYS,
    .rx_phys =
    {
        DEFAULT_RX_BUFFER0_PHYS,
        DEFAULT_RX_BUFFER1_PHYS
    },
    .pc_ip = DEFAULT_PC_IP,
    .data_port = DEFAULT_PC_UDP_PORT,
    .control_port = DEFAULT_CONTROL_UDP_PORT,
    .udp_payload_bytes = DEFAULT_UDP_ADC_PAYLOAD_BYTES,
    .init_adc_ctrl = 1
};

static MmioMap AdcCtrlMap = { .fd = -1 };
static MmioMap DmaMap = { .fd = -1 };
static MmioMap RxMap[2] = { { .fd = -1 }, { .fd = -1 } };

static volatile uint8_t *AdcCtrlRegs;
static volatile uint8_t *DmaRegs;
static uint64_t *RxBuffer[2];

static int DataSock = -1;
static int ControlSock = -1;
static struct sockaddr_in PcAddr;

static uint32_t FrameId = 0U;
static uint32_t ConsecutiveUdpFailures = 0U;
static uint64_t UdpPayloadBytesSent = 0U;
static uint32_t UdpFramesSent = 0U;
static uint32_t UdpRawFramesSent = 0U;
static uint32_t UdpFooterOnlyFramesSent = 0U;

static volatile int PendingFrameSizeValid = 0;
static volatile uint32_t PendingFrameSizeWords = DEFAULT_FRAME_WORDS;

static volatile int PendingPretriggerSizeValid = 0;
static volatile uint32_t PendingPretriggerSizeWords = DEFAULT_PRETRIGGER_WORDS;

static volatile int PendingOutputCfgValid = 0;
static volatile uint32_t PendingOutputCfg = DEFAULT_OUTPUT_CFG;

static volatile int PendingSampleDecimationValid = 0;
static volatile uint32_t PendingSampleDecimation = DEFAULT_SAMPLE_DECIMATION;

static volatile int ReconfigureRequested = 0;
static volatile int SoftwareResetRequested = 0;
static volatile sig_atomic_t StopRequested = 0;

static uint8_t UdpPacketBuffer[UDP_PACKET_BYTES];

/* ============================================================
 * Utility
 * ============================================================ */

static void on_signal(int signo)
{
    (void)signo;
    StopRequested = 1;
}

static void sleep_us(uint32_t duration_us)
{
    struct timespec req;

    req.tv_sec = duration_us / 1000000U;
    req.tv_nsec = (long)(duration_us % 1000000U) * 1000L;

    while (nanosleep(&req, &req) != 0 && errno == EINTR)
    {
        if (StopRequested)
        {
            break;
        }
    }
}

static int parse_u64(const char *text, uint64_t *value)
{
    char *end = NULL;
    unsigned long long parsed;

    errno = 0;
    parsed = strtoull(text, &end, 0);

    if (errno != 0 || end == text || *end != '\0')
    {
        return APP_FAILURE;
    }

    *value = (uint64_t)parsed;
    return APP_SUCCESS;
}

static int parse_u16(const char *text, uint16_t *value)
{
    uint64_t parsed;

    if (parse_u64(text, &parsed) != APP_SUCCESS || parsed > 65535U)
    {
        return APP_FAILURE;
    }

    *value = (uint16_t)parsed;
    return APP_SUCCESS;
}

static int parse_udp_payload(const char *text, uint16_t *value)
{
    uint16_t parsed;

    if (parse_u16(text, &parsed) != APP_SUCCESS ||
        parsed < MIN_UDP_ADC_PAYLOAD_BYTES ||
        parsed > MAX_UDP_ADC_PAYLOAD_BYTES)
    {
        return APP_FAILURE;
    }

    *value = parsed;
    return APP_SUCCESS;
}

static void print_usage(const char *argv0)
{
    printf(
        "Usage: %s [options]\n"
        "\n"
        "Defaults are from the supplied xparameters.h:\n"
        "  adc_ctrl AXI4-Lite base = 0x%08" PRIx64 "\n"
        "  AXI DMA base            = 0x%08" PRIx64 "\n"
        "\n"
        "Options:\n"
        "  --adc-base <phys>       Override adc_ctrl AXI4-Lite physical base\n"
        "  --dma-base <phys>       Override AXI DMA physical base\n"
        "  --rx0 <phys>            RX buffer 0 physical address (default 0x%08" PRIx64 ")\n"
        "  --rx1 <phys>            RX buffer 1 physical address (default 0x%08" PRIx64 ")\n"
        "  --pc-ip <addr>          UDP destination PC IP (default %s)\n"
        "  --data-port <port>      ADC frame UDP destination port (default %u)\n"
        "  --control-port <port>   UDP control listen port (default %u)\n"
        "  --udp-payload <bytes>   ADC bytes per UDP packet, default %u, range %u..%u\n"
        "                           Use 1400 for MTU 1500, 8192 for jumbo MTU 9000\n"
        "  --no-adc-init           Do not overwrite adc_ctrl registers at startup\n"
        "  --help                  Show this help\n",
        argv0,
        DEFAULT_ADC_CTRL_PHYS,
        DEFAULT_DMA_PHYS,
        DEFAULT_RX_BUFFER0_PHYS,
        DEFAULT_RX_BUFFER1_PHYS,
        DEFAULT_PC_IP,
        DEFAULT_PC_UDP_PORT,
        DEFAULT_CONTROL_UDP_PORT,
        DEFAULT_UDP_ADC_PAYLOAD_BYTES,
        MIN_UDP_ADC_PAYLOAD_BYTES,
        MAX_UDP_ADC_PAYLOAD_BYTES
    );
}

static int parse_args(int argc, char **argv)
{
    int i;

    for (i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--adc-base") == 0 && i + 1 < argc)
        {
            if (parse_u64(argv[++i], &Config.adc_ctrl_phys) != APP_SUCCESS)
            {
                fprintf(stderr, "Invalid --adc-base value\n");
                return APP_FAILURE;
            }
        }
        else if (strcmp(argv[i], "--dma-base") == 0 && i + 1 < argc)
        {
            if (parse_u64(argv[++i], &Config.dma_phys) != APP_SUCCESS)
            {
                fprintf(stderr, "Invalid --dma-base value\n");
                return APP_FAILURE;
            }
        }
        else if (strcmp(argv[i], "--rx0") == 0 && i + 1 < argc)
        {
            if (parse_u64(argv[++i], &Config.rx_phys[0]) != APP_SUCCESS)
            {
                fprintf(stderr, "Invalid --rx0 value\n");
                return APP_FAILURE;
            }
        }
        else if (strcmp(argv[i], "--rx1") == 0 && i + 1 < argc)
        {
            if (parse_u64(argv[++i], &Config.rx_phys[1]) != APP_SUCCESS)
            {
                fprintf(stderr, "Invalid --rx1 value\n");
                return APP_FAILURE;
            }
        }
        else if (strcmp(argv[i], "--pc-ip") == 0 && i + 1 < argc)
        {
            Config.pc_ip = argv[++i];
        }
        else if (strcmp(argv[i], "--data-port") == 0 && i + 1 < argc)
        {
            if (parse_u16(argv[++i], &Config.data_port) != APP_SUCCESS)
            {
                fprintf(stderr, "Invalid --data-port value\n");
                return APP_FAILURE;
            }
        }
        else if (strcmp(argv[i], "--control-port") == 0 && i + 1 < argc)
        {
            if (parse_u16(argv[++i], &Config.control_port) != APP_SUCCESS)
            {
                fprintf(stderr, "Invalid --control-port value\n");
                return APP_FAILURE;
            }
        }
        else if (strcmp(argv[i], "--udp-payload") == 0 && i + 1 < argc)
        {
            if (parse_udp_payload(argv[++i], &Config.udp_payload_bytes) != APP_SUCCESS)
            {
                fprintf(
                    stderr,
                    "Invalid --udp-payload value; use %u..%u bytes\n",
                    MIN_UDP_ADC_PAYLOAD_BYTES,
                    MAX_UDP_ADC_PAYLOAD_BYTES
                );
                return APP_FAILURE;
            }
        }
        else if (strcmp(argv[i], "--no-adc-init") == 0)
        {
            Config.init_adc_ctrl = 0;
        }
        else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0)
        {
            print_usage(argv[0]);
            exit(0);
        }
        else
        {
            fprintf(stderr, "Unknown or incomplete option: %s\n", argv[i]);
            return APP_FAILURE;
        }
    }

    return APP_SUCCESS;
}

static int map_physical(uint64_t phys, size_t size, MmioMap *map, void **virt)
{
    long page_size;
    uint64_t page_mask;
    uint64_t page_base;
    uint64_t page_offset;
    size_t map_size;
    void *mapped;

    page_size = sysconf(_SC_PAGESIZE);

    if (page_size <= 0)
    {
        perror("sysconf(_SC_PAGESIZE)");
        return APP_FAILURE;
    }

    page_mask = (uint64_t)page_size - 1U;
    page_base = phys & ~page_mask;
    page_offset = phys - page_base;
    map_size = (size_t)page_offset + size;

    map->fd = open("/dev/mem", O_RDWR | O_SYNC);

    if (map->fd < 0)
    {
        perror("open(/dev/mem)");
        return APP_FAILURE;
    }

    mapped = mmap(NULL, map_size, PROT_READ | PROT_WRITE, MAP_SHARED, map->fd, (off_t)page_base);

    if (mapped == MAP_FAILED)
    {
        perror("mmap(/dev/mem)");
        close(map->fd);
        map->fd = -1;
        return APP_FAILURE;
    }

    map->map_base = mapped;
    map->map_size = map_size;
    map->page_base = (off_t)page_base;
    map->page_offset = (off_t)page_offset;
    *virt = (uint8_t *)mapped + page_offset;

    return APP_SUCCESS;
}

static void unmap_physical(MmioMap *map)
{
    if (map->map_base != NULL)
    {
        munmap(map->map_base, map->map_size);
        map->map_base = NULL;
    }

    if (map->fd >= 0)
    {
        close(map->fd);
        map->fd = -1;
    }
}

static uint32_t mmio_read32(volatile uint8_t *base, uint32_t offset)
{
    return *(volatile uint32_t *)(base + offset);
}

static void mmio_write32(volatile uint8_t *base, uint32_t offset, uint32_t value)
{
    *(volatile uint32_t *)(base + offset) = value;
    __sync_synchronize();
}

/* ============================================================
 * Network service
 * ============================================================ */

static void handle_control_packet(const uint8_t *data, ssize_t len, const struct sockaddr_in *remote);

static void service_network(void)
{
    struct pollfd pfd;
    uint8_t buf[256];
    struct sockaddr_in remote;
    socklen_t remote_len;
    ssize_t len;

    if (ControlSock < 0)
    {
        return;
    }

    pfd.fd = ControlSock;
    pfd.events = POLLIN;
    pfd.revents = 0;

    while (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN) != 0)
    {
        remote_len = sizeof(remote);
        len = recvfrom(ControlSock, buf, sizeof(buf), 0, (struct sockaddr *)&remote, &remote_len);

        if (len < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
            {
                break;
            }

            perror("recvfrom(control)");
            break;
        }

        handle_control_packet(buf, len, &remote);
    }
}

static void drain_network_for_us(uint32_t duration_us)
{
    uint32_t elapsed_us = 0U;
    uint32_t step_us;

    while (elapsed_us < duration_us && !StopRequested)
    {
        step_us = duration_us - elapsed_us;

        if (step_us > 1000U)
        {
            step_us = 1000U;
        }

        service_network();
        sleep_us(step_us);
        elapsed_us += step_us;
    }
}

static uint32_t udp_failure_recovery_delay_us(void)
{
    uint32_t delay_ms;

    delay_ms = UDP_TX_FAILURE_DRAIN_MS +
        (ConsecutiveUdpFailures * UDP_TX_FAILURE_BACKOFF_STEP_MS);

    if (delay_ms > UDP_TX_FAILURE_BACKOFF_MAX_MS)
    {
        delay_ms = UDP_TX_FAILURE_BACKOFF_MAX_MS;
    }

    return delay_ms * 1000U;
}

/* ============================================================
 * adc_ctrl AXI4-Lite access
 * ============================================================ */

static uint32_t adc_ctrl_offset(uint16_t reg)
{
    return ((uint32_t)reg) * 4U;
}

static int adc_ctrl_reg_valid(uint16_t reg)
{
    return (reg <= ADC_CTRL_MAX_REG);
}

static uint32_t adc_ctrl_read(uint16_t reg)
{
    return mmio_read32(AdcCtrlRegs, adc_ctrl_offset(reg));
}

static void adc_ctrl_write(uint16_t reg, uint32_t value)
{
    mmio_write32(AdcCtrlRegs, adc_ctrl_offset(reg), value);
}

static uint32_t sanitize_frame_words(uint32_t words)
{
    if (words == 0U)
    {
        words = PL_DEFAULT_FRAME_WORDS;
    }

    if (words < MIN_FRAME_WORDS)
    {
        words = MIN_FRAME_WORDS;
    }

    if (words > MAX_FRAME_WORDS)
    {
        words = MAX_FRAME_WORDS;
    }

    return words;
}

static uint32_t sanitize_pretrigger_words(uint32_t words)
{
    uint32_t frame_words = sanitize_frame_words(adc_ctrl_read(ADC_CTRL_REG_FRAME_SIZE));

    if (PendingFrameSizeValid)
    {
        frame_words = sanitize_frame_words(PendingFrameSizeWords);
    }

    /*
     * RTL treats cfg_pretrigger_size=0 as DEFAULT_PRE_SIZE, then clamps it
     * against the effective frame size.
     */
    if (words == 0U)
    {
        words = PL_DEFAULT_PRETRIGGER_WORDS;
    }

    if (words > frame_words)
    {
        words = frame_words;
    }

    return words;
}

static uint32_t get_configured_frame_words(void)
{
    return sanitize_frame_words(adc_ctrl_read(ADC_CTRL_REG_FRAME_SIZE));
}

static uint32_t get_configured_pretrigger_words(void)
{
    return sanitize_pretrigger_words(adc_ctrl_read(ADC_CTRL_REG_PRETRIGGER_SIZE));
}

static uint32_t get_reported_frame_words(void)
{
    if (PendingFrameSizeValid)
    {
        return sanitize_frame_words(PendingFrameSizeWords);
    }

    return get_configured_frame_words();
}

static uint32_t get_reported_pretrigger_words(void)
{
    if (PendingPretriggerSizeValid)
    {
        return sanitize_pretrigger_words(PendingPretriggerSizeWords);
    }

    return get_configured_pretrigger_words();
}

static uint32_t sanitize_output_cfg(uint32_t output_cfg)
{
    output_cfg &= 0xFFFFFF5FU;

    if (PL_FORCE_FOOTER != 0U)
    {
        output_cfg |= OUTPUT_CFG_APPEND_FOOTER;
    }

    return output_cfg;
}

static uint32_t get_configured_output_cfg(void)
{
    return sanitize_output_cfg(adc_ctrl_read(ADC_CTRL_REG_OUTPUT_CFG));
}

static uint32_t get_reported_output_cfg(void)
{
    if (PendingOutputCfgValid)
    {
        return sanitize_output_cfg(PendingOutputCfg);
    }

    return get_configured_output_cfg();
}

static uint32_t sanitize_sample_decimation(uint32_t value)
{
    return (value == 0U) ? 1U : value;
}

static uint32_t get_reported_sample_decimation(void)
{
    if (PendingSampleDecimationValid)
    {
        return sanitize_sample_decimation(PendingSampleDecimation);
    }

    return sanitize_sample_decimation(adc_ctrl_read(ADC_CTRL_REG_SAMPLE_DECIMATION));
}

static void request_sample_decimation_change(uint32_t value)
{
    uint32_t requested = sanitize_sample_decimation(value);
    uint32_t current = get_reported_sample_decimation();

    if (requested == current)
    {
        printf("Sample decimation unchanged: %u\n", requested);
        return;
    }

    PendingSampleDecimation = requested;
    PendingSampleDecimationValid = 1;
    ReconfigureRequested = 1;
    SoftwareResetRequested = 1;

    printf("Sample decimation change queued: %u\n", requested);
}

static void request_frame_size_change(uint32_t frame_words)
{
    uint32_t requested_words = sanitize_frame_words(frame_words);
    uint32_t current_words = get_reported_frame_words();

    if (requested_words == current_words)
    {
        printf("Frame size unchanged: %u words\n", requested_words);
        return;
    }

    PendingFrameSizeWords = requested_words;
    PendingFrameSizeValid = 1;
    ReconfigureRequested = 1;
    SoftwareResetRequested = 1;

    printf("Frame size change queued: %u words\n", PendingFrameSizeWords);
}

static void request_pretrigger_size_change(uint32_t pretrigger_words)
{
    uint32_t requested_words = sanitize_pretrigger_words(pretrigger_words);
    uint32_t current_words = get_reported_pretrigger_words();

    if (requested_words == current_words)
    {
        printf("Pretrigger size unchanged: %u words\n", requested_words);
        return;
    }

    PendingPretriggerSizeWords = requested_words;
    PendingPretriggerSizeValid = 1;
    ReconfigureRequested = 1;
    SoftwareResetRequested = 1;

    printf("Pretrigger size change queued: %u words\n", PendingPretriggerSizeWords);
}

static void request_output_cfg_change(uint32_t output_cfg)
{
    uint32_t current_cfg = get_reported_output_cfg();

    output_cfg = sanitize_output_cfg(output_cfg);

    if (output_cfg == current_cfg)
    {
        printf("Output cfg unchanged: 0x%08x\n", output_cfg);
        return;
    }

    PendingOutputCfg = output_cfg;
    PendingOutputCfgValid = 1;
    ReconfigureRequested = 1;
    SoftwareResetRequested = 1;

    printf("Output cfg change queued: 0x%08x\n", PendingOutputCfg);
}

static void apply_pending_length_related_config(void)
{
    if (PendingSampleDecimationValid)
    {
        adc_ctrl_write(ADC_CTRL_REG_SAMPLE_DECIMATION, sanitize_sample_decimation(PendingSampleDecimation));
        printf("Sample decimation change applied: %u\n", sanitize_sample_decimation(PendingSampleDecimation));
        PendingSampleDecimationValid = 0;
    }

    if (PendingFrameSizeValid)
    {
        adc_ctrl_write(ADC_CTRL_REG_FRAME_SIZE, sanitize_frame_words(PendingFrameSizeWords));
        printf("Frame size change applied: %u words\n", sanitize_frame_words(PendingFrameSizeWords));
        PendingFrameSizeValid = 0;
    }

    if (PendingPretriggerSizeValid)
    {
        adc_ctrl_write(ADC_CTRL_REG_PRETRIGGER_SIZE, sanitize_pretrigger_words(PendingPretriggerSizeWords));
        printf("Pretrigger size change applied: %u words\n", sanitize_pretrigger_words(PendingPretriggerSizeWords));
        PendingPretriggerSizeValid = 0;
    }

    if (PendingOutputCfgValid)
    {
        adc_ctrl_write(ADC_CTRL_REG_OUTPUT_CFG, sanitize_output_cfg(PendingOutputCfg));
        printf("Output cfg change applied: 0x%08x\n", sanitize_output_cfg(PendingOutputCfg));
        PendingOutputCfgValid = 0;
    }
}

static uint32_t frame_words_to_dma_bytes(uint32_t frame_words)
{
    uint32_t dma_words = sanitize_frame_words(frame_words);

    /* RTL FORCE_FOOTER=1: every frame includes the 17-word footer. */
    if (PL_FORCE_FOOTER != 0U)
    {
        dma_words += PL_FOOTER_WORDS;
    }

    return dma_words * 8U;
}

static uint16_t frame_bytes_to_udp_packets(uint32_t frame_bytes)
{
    return (uint16_t)(
        (frame_bytes + Config.udp_payload_bytes - 1U) /
        Config.udp_payload_bytes
    );
}

static void initialize_adc_control_ip(void)
{
    uint32_t pl_status;

    /*
     * Acquisition starts disabled. CONTROL[5] is owned by the PC GUI:
     * it is set when the GUI starts (and re-set after a GUI-requested
     * software reset) and cleared when the GUI exits. Leaving it clear
     * here means the PL->DDR path stays idle until a GUI is actually
     * listening, instead of streaming frames to nobody.
     *
     * All later CONTROL accesses in this program are read-modify-write
     * pulses (arm / fifo clear / software reset), so they preserve
     * whatever CONTROL[5] state the GUI has selected.
     */
    adc_ctrl_write(ADC_CTRL_REG_CONTROL, 0U);
    adc_ctrl_write(ADC_CTRL_REG_FRAME_SIZE, DEFAULT_FRAME_WORDS);
    adc_ctrl_write(ADC_CTRL_REG_PRETRIGGER_SIZE, DEFAULT_PRETRIGGER_WORDS);
    adc_ctrl_write(ADC_CTRL_REG_SAMPLE_DECIMATION, DEFAULT_SAMPLE_DECIMATION);
    adc_ctrl_write(ADC_CTRL_REG_TRIGGER_CFG, DEFAULT_TRIGGER_CFG);
    adc_ctrl_write(ADC_CTRL_REG_SELF_THRESHOLD, DEFAULT_SELF_THRESHOLD);
    adc_ctrl_write(ADC_CTRL_REG_CHANNEL_MASK, DEFAULT_CHANNEL_MASK);
    adc_ctrl_write(ADC_CTRL_REG_OUTPUT_CFG, sanitize_output_cfg(DEFAULT_OUTPUT_CFG));
    adc_ctrl_write(ADC_CTRL_REG_BASELINE_SHIFT, DEFAULT_BASELINE_SHIFT);

    PendingFrameSizeValid = 0;
    PendingPretriggerSizeValid = 0;
    PendingOutputCfgValid = 0;
    PendingSampleDecimationValid = 0;
    ReconfigureRequested = 0;
    SoftwareResetRequested = 0;

    printf("adc_ctrl initialized at 0x%08" PRIx64 "\n", Config.adc_ctrl_phys);
    printf("frame_size      = %u words\n", DEFAULT_FRAME_WORDS);
    printf("pretrigger_size = %u words\n", DEFAULT_PRETRIGGER_WORDS);
    printf("trigger_cfg     = 0x%08x\n", DEFAULT_TRIGGER_CFG);
    printf("channel_mask    = 0x%08x\n", DEFAULT_CHANNEL_MASK);
    printf("output_cfg      = 0x%08x\n", sanitize_output_cfg(DEFAULT_OUTPUT_CFG));
    printf("control         = 0x%08x\n", adc_ctrl_read(ADC_CTRL_REG_CONTROL));
    printf("baseline_shift  = %u\n", DEFAULT_BASELINE_SHIFT);
    pl_status = adc_ctrl_read(ADC_CTRL_REG_STATUS);
    printf(
        "status          = 0x%08x "
        "(adc1_full=%u, adc2_full=%u, any_full=%u, overrun=%u)\n",
        pl_status,
        (unsigned int)((pl_status & PL_STATUS_ADC1_FIFO_FULL) != 0U),
        (unsigned int)((pl_status & PL_STATUS_ADC2_FIFO_FULL) != 0U),
        (unsigned int)((pl_status & PL_STATUS_ANY_FIFO_FULL) != 0U),
        (unsigned int)((pl_status & PL_STATUS_CAPTURE_OVERRUN) != 0U)
    );
}

/* ============================================================
 * Control UDP
 * ============================================================ */

static int send_control_reply(
    const struct sockaddr_in *addr,
    uint32_t seq,
    uint16_t status,
    uint16_t reg,
    uint32_t value
)
{
    ControlReply reply;
    ssize_t sent;

    reply.magic = htonl(CTRL_REPLY_MAGIC);
    reply.seq = htonl(seq);
    reply.status = htons(status);
    reply.reg = htons(reg);
    reply.value = htonl(value);

    reply.control = htonl(adc_ctrl_read(ADC_CTRL_REG_CONTROL));
    reply.frame_size = htonl(get_reported_frame_words());
    reply.pretrigger_size = htonl(get_reported_pretrigger_words());
    reply.sample_decimation = htonl(get_reported_sample_decimation());
    reply.trigger_cfg = htonl(adc_ctrl_read(ADC_CTRL_REG_TRIGGER_CFG));
    reply.self_threshold = htonl(adc_ctrl_read(ADC_CTRL_REG_SELF_THRESHOLD));
    reply.channel_mask = htonl(adc_ctrl_read(ADC_CTRL_REG_CHANNEL_MASK));
    reply.output_cfg = htonl(get_reported_output_cfg());
    reply.baseline_shift = htonl(adc_ctrl_read(ADC_CTRL_REG_BASELINE_SHIFT));

    sent = sendto(
        ControlSock,
        &reply,
        sizeof(reply),
        0,
        (const struct sockaddr *)addr,
        sizeof(*addr)
    );

    return (sent == (ssize_t)sizeof(reply)) ? APP_SUCCESS : APP_FAILURE;
}

static uint32_t read_reg_for_reply(uint16_t reg)
{
    if (reg == ADC_CTRL_REG_FRAME_SIZE)
    {
        return get_reported_frame_words();
    }

    if (reg == ADC_CTRL_REG_OUTPUT_CFG)
    {
        return get_reported_output_cfg();
    }

    if (reg == ADC_CTRL_REG_PRETRIGGER_SIZE)
    {
        return get_reported_pretrigger_words();
    }

    if (reg == ADC_CTRL_REG_SAMPLE_DECIMATION)
    {
        return get_reported_sample_decimation();
    }

    return adc_ctrl_read(reg);
}

static void write_control_reg(uint16_t reg, uint32_t value)
{
    if (reg == ADC_CTRL_REG_CONTROL)
    {
        if ((value & CONTROL_SOFTWARE_RESET) != 0U)
        {
            SoftwareResetRequested = 1;
        }

        adc_ctrl_write(ADC_CTRL_REG_CONTROL, value & ~CONTROL_SOFTWARE_RESET);
        return;
    }

    if (reg == ADC_CTRL_REG_FRAME_SIZE)
    {
        request_frame_size_change(value);
        return;
    }

    if (reg == ADC_CTRL_REG_OUTPUT_CFG)
    {
        request_output_cfg_change(value);
        return;
    }

    if (reg == ADC_CTRL_REG_SAMPLE_DECIMATION)
    {
        request_sample_decimation_change(value);
        return;
    }

    if (reg == ADC_CTRL_REG_PRETRIGGER_SIZE)
    {
        request_pretrigger_size_change(value);
        return;
    }

    adc_ctrl_write(reg, value);
}

static void handle_control_packet(const uint8_t *data, ssize_t len, const struct sockaddr_in *remote)
{
    ControlCommand cmd;
    uint32_t magic;
    uint32_t seq = 0U;
    uint16_t op = 0U;
    uint16_t reg = 0U;
    uint32_t value = 0U;
    uint16_t status = CTRL_STATUS_OK;

    memset(&cmd, 0, sizeof(cmd));

    if (len >= 8)
    {
        memcpy(&cmd, data, (size_t)((len < (ssize_t)sizeof(cmd)) ? len : (ssize_t)sizeof(cmd)));
        seq = ntohl(cmd.seq);
    }

    if (len != (ssize_t)sizeof(ControlCommand))
    {
        send_control_reply(remote, seq, CTRL_STATUS_BAD_LEN, reg, value);
        return;
    }

    memcpy(&cmd, data, sizeof(cmd));

    magic = ntohl(cmd.magic);
    seq = ntohl(cmd.seq);
    op = ntohs(cmd.op);
    reg = ntohs(cmd.reg);
    value = ntohl(cmd.value);

    if (magic != CTRL_CMD_MAGIC)
    {
        return;
    }

    if (op == CTRL_OP_DUMP)
    {
        send_control_reply(remote, seq, CTRL_STATUS_OK, reg, value);
        return;
    }

    if (!adc_ctrl_reg_valid(reg))
    {
        send_control_reply(remote, seq, CTRL_STATUS_BAD_REG, reg, value);
        return;
    }

    if (op == CTRL_OP_READ)
    {
        value = read_reg_for_reply(reg);
    }
    else if (op == CTRL_OP_WRITE)
    {
        if (reg == ADC_CTRL_REG_STATUS)
        {
            status = CTRL_STATUS_BAD_OP;
            value = read_reg_for_reply(reg);
        }
        else
        {
            write_control_reg(reg, value);
            value = read_reg_for_reply(reg);
        }
    }
    else
    {
        status = CTRL_STATUS_BAD_OP;
    }

    if (send_control_reply(remote, seq, status, reg, value) != APP_SUCCESS)
    {
        perror("sendto(control reply)");
    }
}

static void pulse_pl_software_reset(void)
{
    uint32_t control_value;

    control_value = adc_ctrl_read(ADC_CTRL_REG_CONTROL);
    control_value &= ~CONTROL_SOFTWARE_RESET;
    adc_ctrl_write(ADC_CTRL_REG_CONTROL, control_value);

    control_value |= CONTROL_SOFTWARE_RESET;
    adc_ctrl_write(ADC_CTRL_REG_CONTROL, control_value);
    sleep_us(1U);

    control_value &= ~CONTROL_SOFTWARE_RESET;
    adc_ctrl_write(ADC_CTRL_REG_CONTROL, control_value);
    sleep_us(1U);
}

static void pulse_fifo_alarm_clear(void)
{
    uint32_t control_value;

    control_value = adc_ctrl_read(ADC_CTRL_REG_CONTROL);
    control_value &= ~CONTROL_FIFO_ALARM_CLEAR;
    adc_ctrl_write(ADC_CTRL_REG_CONTROL, control_value);

    control_value |= CONTROL_FIFO_ALARM_CLEAR;
    adc_ctrl_write(ADC_CTRL_REG_CONTROL, control_value);
    sleep_us(1U);

    control_value &= ~CONTROL_FIFO_ALARM_CLEAR;
    adc_ctrl_write(ADC_CTRL_REG_CONTROL, control_value);
    sleep_us(1U);
}

static void set_pl_acquisition_enable(int enabled)
{
    uint32_t control_value;

    if (AdcCtrlRegs == NULL)
    {
        return;
    }

    control_value = adc_ctrl_read(ADC_CTRL_REG_CONTROL);

    if (enabled)
    {
        control_value |= CONTROL_ACQUISITION_ENABLE;
    }
    else
    {
        control_value &= ~CONTROL_ACQUISITION_ENABLE;
    }

    adc_ctrl_write(ADC_CTRL_REG_CONTROL, control_value);
}

static int get_pl_acquisition_enable(void)
{
    if (AdcCtrlRegs == NULL)
    {
        return 0;
    }

    return ((adc_ctrl_read(ADC_CTRL_REG_CONTROL) & CONTROL_ACQUISITION_ENABLE) != 0U) ? 1 : 0;
}

static void hold_pl_acquisition_idle(void)
{
    uint32_t control_value;

    if (AdcCtrlRegs == NULL)
    {
        return;
    }

    control_value = adc_ctrl_read(ADC_CTRL_REG_CONTROL);
    control_value &= ~(CONTROL_ACQUISITION_ENABLE | CONTROL_ACQUISITION_ARM | CONTROL_SOFT_TRIGGER);
    adc_ctrl_write(ADC_CTRL_REG_CONTROL, control_value);
    sleep_us(2U);
}

static void pulse_acquisition_arm(void)
{
    uint32_t control_value;

    control_value = adc_ctrl_read(ADC_CTRL_REG_CONTROL);

    control_value &= ~CONTROL_ACQUISITION_ARM;
    adc_ctrl_write(ADC_CTRL_REG_CONTROL, control_value);
    sleep_us(1U);

    control_value |= CONTROL_ACQUISITION_ARM;
    adc_ctrl_write(ADC_CTRL_REG_CONTROL, control_value);
    sleep_us(1U);

    control_value &= ~CONTROL_ACQUISITION_ARM;
    adc_ctrl_write(ADC_CTRL_REG_CONTROL, control_value);
}

/* ============================================================
 * DMA helpers
 * ============================================================ */

static uint32_t dma_reg_offset(uint32_t reg)
{
    return AXIDMA_S2MM_OFFSET + reg;
}

static uint32_t read_dma_status(void)
{
    return mmio_read32(DmaRegs, dma_reg_offset(AXIDMA_SR_OFFSET));
}

static void print_dma_status(void)
{
    printf("S2MM_DMASR = 0x%08x\n", read_dma_status());
}

static int dma_has_error(void)
{
    uint32_t status = read_dma_status();

    return ((status & AXIDMA_SR_ERR_ALL) != 0U) ? 1 : 0;
}

static int reset_dma_engine(void)
{
    uint32_t timeout;

    mmio_write32(DmaRegs, dma_reg_offset(AXIDMA_CR_OFFSET), AXIDMA_CR_RESET);

    timeout = 1000000U;

    while ((mmio_read32(DmaRegs, dma_reg_offset(AXIDMA_CR_OFFSET)) & AXIDMA_CR_RESET) != 0U)
    {
        if (timeout == 0U)
        {
            printf("ERROR: DMA reset timeout\n");
            return APP_FAILURE;
        }

        timeout--;
        sleep_us(1U);
    }

    mmio_write32(DmaRegs, dma_reg_offset(AXIDMA_SR_OFFSET), AXIDMA_SR_IRQ_ALL | AXIDMA_SR_ERR_ALL);

    printf("DMA reset complete\n");

    return APP_SUCCESS;
}

static int initialize_dma(void)
{
    uint32_t status;

    if (reset_dma_engine() != APP_SUCCESS)
    {
        return APP_FAILURE;
    }

    status = read_dma_status();

    if ((status & AXIDMA_SR_HALTED) == 0U)
    {
        printf("WARNING: DMA is not halted after reset\n");
        print_dma_status();
    }

    printf("DMA initialized at 0x%08" PRIx64 "\n", Config.dma_phys);
    printf("MAX_FRAME_WORDS = %u\n", MAX_FRAME_WORDS);
    printf("MAX_DMA_BYTES   = %u\n", MAX_DMA_BYTES);
    printf("RX buffer 0     = 0x%08" PRIx64 "\n", Config.rx_phys[0]);
    printf("RX buffer 1     = 0x%08" PRIx64 "\n", Config.rx_phys[1]);

    return APP_SUCCESS;
}

static int start_dma_receive(int buffer_index, uint32_t frame_bytes)
{
    uint64_t phys = Config.rx_phys[buffer_index];
    uint32_t cr;

    if (frame_bytes > MAX_DMA_BYTES)
    {
        printf("ERROR: requested DMA length is too large: %u\n", frame_bytes);
        return APP_FAILURE;
    }

    mmio_write32(DmaRegs, dma_reg_offset(AXIDMA_SR_OFFSET), AXIDMA_SR_IRQ_ALL | AXIDMA_SR_ERR_ALL);

    cr = mmio_read32(DmaRegs, dma_reg_offset(AXIDMA_CR_OFFSET));
    cr |= AXIDMA_CR_RUNSTOP;
    mmio_write32(DmaRegs, dma_reg_offset(AXIDMA_CR_OFFSET), cr);

    mmio_write32(DmaRegs, dma_reg_offset(AXIDMA_DSTADDR_OFFSET), (uint32_t)(phys & 0xffffffffU));
    mmio_write32(DmaRegs, dma_reg_offset(AXIDMA_DSTADDR_MSB_OFFSET), (uint32_t)(phys >> 32));
    mmio_write32(DmaRegs, dma_reg_offset(AXIDMA_LENGTH_OFFSET), frame_bytes);

    sleep_us(1U);

    if (dma_has_error())
    {
        printf("ERROR: DMA transfer start failed\n");
        print_dma_status();
        return APP_FAILURE;
    }

    return APP_SUCCESS;
}

static int dma_receive_complete(void)
{
    uint32_t status = read_dma_status();

    if ((status & AXIDMA_SR_IOC_IRQ) != 0U)
    {
        return 1;
    }

    if (((status & AXIDMA_SR_IDLE) != 0U) && ((status & AXIDMA_SR_HALTED) == 0U))
    {
        return 1;
    }

    return 0;
}

static int wait_for_dma_complete(void)
{
    while (!StopRequested && !dma_receive_complete())
    {
        if (SoftwareResetRequested)
        {
            return DMA_WAIT_SOFTWARE_RESET;
        }

        service_network();
        sleep_us(50U);
    }

    if (StopRequested)
    {
        return APP_FAILURE;
    }

    if (dma_has_error())
    {
        printf("ERROR: DMA transfer completed with error\n");
        print_dma_status();
        return APP_FAILURE;
    }

    return APP_SUCCESS;
}

/* ============================================================
 * UDP setup and transmission
 * ============================================================ */

static int make_socket_nonblocking(int fd)
{
    int flags;

    flags = fcntl(fd, F_GETFL, 0);

    if (flags < 0)
    {
        perror("fcntl(F_GETFL)");
        return APP_FAILURE;
    }

    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0)
    {
        perror("fcntl(F_SETFL)");
        return APP_FAILURE;
    }

    return APP_SUCCESS;
}

static int initialize_udp(void)
{
    int sockbuf = (int)UDP_SOCKET_BUFFER_BYTES;

    DataSock = socket(AF_INET, SOCK_DGRAM, 0);

    if (DataSock < 0)
    {
        perror("socket(data)");
        return APP_FAILURE;
    }

    (void)setsockopt(DataSock, SOL_SOCKET, SO_SNDBUF, &sockbuf, sizeof(sockbuf));

    memset(&PcAddr, 0, sizeof(PcAddr));
    PcAddr.sin_family = AF_INET;
    PcAddr.sin_port = htons(Config.data_port);

    if (inet_pton(AF_INET, Config.pc_ip, &PcAddr.sin_addr) != 1)
    {
        fprintf(stderr, "Invalid PC IP address: %s\n", Config.pc_ip);
        return APP_FAILURE;
    }

    if (connect(DataSock, (const struct sockaddr *)&PcAddr, sizeof(PcAddr)) < 0)
    {
        perror("connect(data UDP)");
        return APP_FAILURE;
    }

    if (make_socket_nonblocking(DataSock) != APP_SUCCESS)
    {
        return APP_FAILURE;
    }

    printf("UDP destination = %s:%u\n", Config.pc_ip, Config.data_port);
    printf("UDP payload bytes = %u\n", Config.udp_payload_bytes);
    if (Config.udp_payload_bytes > 1400U)
    {
        printf(
            "NOTE: UDP payload > 1400 needs jumbo MTU on board/PC/switch "
            "to avoid IP fragmentation\n"
        );
    }
    printf("UDP max packets per frame = %u\n", frame_bytes_to_udp_packets(MAX_DMA_BYTES));

    return APP_SUCCESS;
}

static int initialize_control_udp(void)
{
    struct sockaddr_in listen_addr;
    int reuse = 1;

    ControlSock = socket(AF_INET, SOCK_DGRAM, 0);

    if (ControlSock < 0)
    {
        perror("socket(control)");
        return APP_FAILURE;
    }

    setsockopt(ControlSock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    memset(&listen_addr, 0, sizeof(listen_addr));
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    listen_addr.sin_port = htons(Config.control_port);

    if (bind(ControlSock, (const struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0)
    {
        perror("bind(control UDP)");
        return APP_FAILURE;
    }

    if (make_socket_nonblocking(ControlSock) != APP_SUCCESS)
    {
        return APP_FAILURE;
    }

    printf("Control UDP listening on port %u\n", Config.control_port);

    return APP_SUCCESS;
}

static void prepare_pc_arp(void)
{
    /*
     * Linux owns ARP. Send a tiny warm-up datagram so the neighbor entry is
     * created before the first large frame burst.
     */
    uint8_t warmup = 0U;

    (void)send(DataSock, &warmup, sizeof(warmup), 0);
    sleep_us(100000U);
}

static int send_udp_packet(
    uint32_t frame_id,
    uint16_t packet_index,
    uint16_t total_packets,
    uint32_t frame_bytes,
    const uint8_t *payload,
    uint16_t payload_bytes
)
{
    UdpAdcHeader header;
    uint32_t retry_count;
    uint16_t packet_bytes;
    ssize_t sent;

    header.magic = htonl(UDP_MAGIC);
    header.frame_id = htonl(frame_id);
    header.packet_index = htons(packet_index);
    header.total_packets = htons(total_packets);
    header.payload_bytes = htons(payload_bytes);
    header.reserved = htons(0U);
    header.frame_bytes = htonl(frame_bytes);

    memcpy(UdpPacketBuffer, &header, sizeof(header));
    memcpy(UdpPacketBuffer + sizeof(header), payload, payload_bytes);

    packet_bytes = (uint16_t)(sizeof(header) + payload_bytes);

    for (retry_count = 0U; retry_count < UDP_TX_RETRY_COUNT && !StopRequested; retry_count++)
    {
        sent = send(DataSock, UdpPacketBuffer, packet_bytes, 0);

        if (sent == (ssize_t)packet_bytes)
        {
            return APP_SUCCESS;
        }

        if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS || errno == EINTR))
        {
            service_network();
            sleep_us(UDP_TX_RETRY_DELAY_US);
            continue;
        }

        perror("send(data UDP)");
        return APP_FAILURE;
    }

    printf(
        "ERROR: UDP send timeout at packet %u after %u retries\n",
        packet_index,
        UDP_TX_RETRY_COUNT
    );

    return APP_FAILURE;
}

static void select_udp_payload(
    uint64_t *buffer,
    uint32_t full_frame_bytes,
    const uint8_t **payload,
    uint32_t *payload_bytes
)
{
    const uint32_t footer_bytes = PL_FOOTER_WORDS * 8U;

    *payload = (const uint8_t *)buffer;
    *payload_bytes = full_frame_bytes;

    if (full_frame_bytes <= footer_bytes)
    {
        return;
    }

    if (buffer[0] == PL_FOOTER_MAGIC)
    {
        *payload = (const uint8_t *)buffer;
        *payload_bytes = footer_bytes;
    }
}

static int send_adc_frame(uint64_t *buffer, uint32_t frame_id, uint32_t frame_bytes)
{
    const uint8_t *frame_data;
    uint32_t udp_frame_bytes;
    uint32_t offset = 0U;
    uint32_t remaining;
    uint16_t packet_index;
    uint16_t payload_bytes;
    uint16_t total_packets;

    __sync_synchronize();
    select_udp_payload(buffer, frame_bytes, &frame_data, &udp_frame_bytes);
    total_packets = frame_bytes_to_udp_packets(udp_frame_bytes);

    for (packet_index = 0U; packet_index < total_packets && !StopRequested; packet_index++)
    {
        remaining = udp_frame_bytes - offset;

        if (remaining > Config.udp_payload_bytes)
        {
            payload_bytes = Config.udp_payload_bytes;
        }
        else
        {
            payload_bytes = (uint16_t)remaining;
        }

        if (send_udp_packet(
                frame_id,
                packet_index,
                total_packets,
                udp_frame_bytes,
                frame_data + offset,
                payload_bytes
            ) != APP_SUCCESS)
        {
            printf("ERROR: UDP packet %u transmission failed\n", packet_index);
            return APP_FAILURE;
        }

        offset += payload_bytes;

#if UDP_PACKET_GAP_US > 0U
        drain_network_for_us(UDP_PACKET_GAP_US);
#endif
        service_network();
    }

    if (!StopRequested)
    {
        UdpFramesSent++;
        UdpPayloadBytesSent += udp_frame_bytes;
        if (udp_frame_bytes <= (PL_FOOTER_WORDS * 8U))
        {
            UdpFooterOnlyFramesSent++;
        }
        else
        {
            UdpRawFramesSent++;
        }
    }

    return StopRequested ? APP_FAILURE : APP_SUCCESS;
}

/* ============================================================
 * Initialization/cleanup
 * ============================================================ */

static int initialize_hardware_maps(void)
{
    void *virt;

    if (map_physical(Config.adc_ctrl_phys, ADC_CTRL_MAP_BYTES, &AdcCtrlMap, &virt) != APP_SUCCESS)
    {
        return APP_FAILURE;
    }

    AdcCtrlRegs = (volatile uint8_t *)virt;

    if (map_physical(Config.dma_phys, AXIDMA_MAP_BYTES, &DmaMap, &virt) != APP_SUCCESS)
    {
        return APP_FAILURE;
    }

    DmaRegs = (volatile uint8_t *)virt;

    if (map_physical(Config.rx_phys[0], MAX_DMA_BYTES, &RxMap[0], &virt) != APP_SUCCESS)
    {
        return APP_FAILURE;
    }

    RxBuffer[0] = (uint64_t *)virt;

    if (map_physical(Config.rx_phys[1], MAX_DMA_BYTES, &RxMap[1], &virt) != APP_SUCCESS)
    {
        return APP_FAILURE;
    }

    RxBuffer[1] = (uint64_t *)virt;

    return APP_SUCCESS;
}

static void cleanup(void)
{
    /*
     * Drop CONTROL[5] so the PL does not keep accepting triggers after
     * this program exits. An in-flight post-trigger capture finishes
     * normally in the PL; only new triggers are blocked.
     */
    set_pl_acquisition_enable(0);

    if (DataSock >= 0)
    {
        close(DataSock);
        DataSock = -1;
    }

    if (ControlSock >= 0)
    {
        close(ControlSock);
        ControlSock = -1;
    }

    unmap_physical(&RxMap[1]);
    unmap_physical(&RxMap[0]);
    unmap_physical(&DmaMap);
    unmap_physical(&AdcCtrlMap);
}

/* ============================================================
 * Main
 * ============================================================ */

int main(int argc, char **argv)
{
    int status;
    int active_rx_buffer;
    int completed_buffer;
    uint32_t active_frame_words;
    uint32_t active_frame_bytes;
    uint32_t completed_frame_bytes;
    uint32_t completed_frame_id;
    int restore_acquisition_enable = 0;

    if (parse_args(argc, argv) != APP_SUCCESS)
    {
        print_usage(argv[0]);
        return 1;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    printf("\n");
    printf("==============================================\n");
    printf(" ADC Trigger DMA UDP Control Application Linux\n");
    printf("==============================================\n");

    if (initialize_hardware_maps() != APP_SUCCESS)
    {
        cleanup();
        return 1;
    }

    status = initialize_udp();

    if (status != APP_SUCCESS)
    {
        cleanup();
        return 1;
    }

    status = initialize_control_udp();

    if (status != APP_SUCCESS)
    {
        cleanup();
        return 1;
    }

    if (Config.init_adc_ctrl)
    {
        initialize_adc_control_ip();
    }

    status = initialize_dma();

    if (status != APP_SUCCESS)
    {
        cleanup();
        return 1;
    }

    prepare_pc_arp();

    printf("Resetting PL capture path before first DMA arm\n");
    pulse_pl_software_reset();
    pulse_fifo_alarm_clear();

    active_rx_buffer = 0;
    active_frame_words = get_configured_frame_words();
    active_frame_bytes = frame_words_to_dma_bytes(active_frame_words);

    status = start_dma_receive(active_rx_buffer, active_frame_bytes);

    if (status != APP_SUCCESS)
    {
        printf("First DMA start failed\n");
        cleanup();
        return 1;
    }

    pulse_acquisition_arm();

    printf(
        "DMA armed on buffer %d for %u bytes; "
        "acquisition disabled, waiting for GUI to set CONTROL[5]...\n",
        active_rx_buffer,
        active_frame_bytes
    );

    while (!StopRequested)
    {
        status = wait_for_dma_complete();

        if (status == DMA_WAIT_SOFTWARE_RESET)
        {
            printf("Software reset requested; resetting DMA and PL capture path\n");

            SoftwareResetRequested = 0;
            restore_acquisition_enable = get_pl_acquisition_enable();
            hold_pl_acquisition_idle();

            if (reset_dma_engine() != APP_SUCCESS)
            {
                break;
            }

            pulse_pl_software_reset();
            pulse_fifo_alarm_clear();

            if (ReconfigureRequested)
            {
                apply_pending_length_related_config();
                ReconfigureRequested = 0;
                SoftwareResetRequested = 0;
            }

            active_rx_buffer ^= 1;
            active_frame_words = get_configured_frame_words();
            active_frame_bytes = frame_words_to_dma_bytes(active_frame_words);

            if (start_dma_receive(active_rx_buffer, active_frame_bytes) != APP_SUCCESS)
            {
                printf("ERROR: DMA re-arm failed after software reset\n");
                break;
            }

            if (restore_acquisition_enable)
            {
                set_pl_acquisition_enable(1);
                restore_acquisition_enable = 0;
            }

            pulse_acquisition_arm();

            printf(
                "DMA and PL re-armed on buffer %d for %u bytes after software reset\n",
                active_rx_buffer,
                active_frame_bytes
            );

            continue;
        }

        if (status != APP_SUCCESS)
        {
            if (StopRequested)
            {
                break;
            }

            printf("WARNING: DMA receive failed; resetting DMA and arming next buffer\n");

            restore_acquisition_enable = get_pl_acquisition_enable();
            hold_pl_acquisition_idle();

            if (reset_dma_engine() != APP_SUCCESS)
            {
                break;
            }

            pulse_pl_software_reset();
            pulse_fifo_alarm_clear();

            active_rx_buffer ^= 1;

            if (ReconfigureRequested)
            {
                apply_pending_length_related_config();
                ReconfigureRequested = 0;
                SoftwareResetRequested = 0;
            }

            active_frame_words = get_configured_frame_words();
            active_frame_bytes = frame_words_to_dma_bytes(active_frame_words);

            if (start_dma_receive(active_rx_buffer, active_frame_bytes) != APP_SUCCESS)
            {
                printf("WARNING: DMA restart failed; resetting and retrying once\n");

                if (reset_dma_engine() != APP_SUCCESS)
                {
                    printf("ERROR: DMA restart retry failed\n");
                    break;
                }

                pulse_pl_software_reset();
                pulse_fifo_alarm_clear();

                if (start_dma_receive(active_rx_buffer, active_frame_bytes) != APP_SUCCESS)
                {
                    printf("ERROR: DMA restart retry failed\n");
                    break;
                }
            }

            if (restore_acquisition_enable)
            {
                set_pl_acquisition_enable(1);
                restore_acquisition_enable = 0;
            }

            pulse_acquisition_arm();

            printf(
                "DMA and PL re-armed on buffer %d for %u bytes after DMA error\n",
                active_rx_buffer,
                active_frame_bytes
            );

            continue;
        }

        completed_buffer = active_rx_buffer;
        completed_frame_bytes = active_frame_bytes;

        __sync_synchronize();

        FrameId++;
        completed_frame_id = FrameId;

        /*
         * Re-arm DMA before spending time on UDP transmission. This overlaps
         * the next PL playback/DMA write with userspace packetization of the
         * completed buffer and reduces trigger dead time.
         */
        mmio_write32(DmaRegs, dma_reg_offset(AXIDMA_SR_OFFSET), AXIDMA_SR_IRQ_ALL);

        active_rx_buffer ^= 1;

        if (ReconfigureRequested)
        {
            printf("Reconfiguration pending; resetting DMA and PL before next arm\n");

            restore_acquisition_enable = get_pl_acquisition_enable();
            hold_pl_acquisition_idle();

            if (reset_dma_engine() != APP_SUCCESS)
            {
                break;
            }

            pulse_pl_software_reset();
            pulse_fifo_alarm_clear();

            apply_pending_length_related_config();
            ReconfigureRequested = 0;
            SoftwareResetRequested = 0;
        }

        active_frame_words = get_configured_frame_words();
        active_frame_bytes = frame_words_to_dma_bytes(active_frame_words);

        status = start_dma_receive(active_rx_buffer, active_frame_bytes);

        if (status != APP_SUCCESS)
        {
            printf("WARNING: next DMA start failed; resetting and retrying once\n");

            if (!restore_acquisition_enable)
            {
                restore_acquisition_enable = get_pl_acquisition_enable();
                hold_pl_acquisition_idle();
            }

            if (reset_dma_engine() != APP_SUCCESS)
            {
                printf("ERROR: next DMA start retry failed\n");
                break;
            }

            pulse_pl_software_reset();
            pulse_fifo_alarm_clear();

            if (start_dma_receive(active_rx_buffer, active_frame_bytes) != APP_SUCCESS)
            {
                printf("ERROR: next DMA start retry failed\n");
                break;
            }
        }

        if (restore_acquisition_enable)
        {
            set_pl_acquisition_enable(1);
            restore_acquisition_enable = 0;
        }

        pulse_acquisition_arm();

        status = send_adc_frame(
            RxBuffer[completed_buffer],
            completed_frame_id,
            completed_frame_bytes
        );

        if (status != APP_SUCCESS)
        {
            if (StopRequested)
            {
                break;
            }

            printf(
                "ERROR: frame %u UDP transmission failed; recovering and re-arming\n",
                completed_frame_id
            );

            ConsecutiveUdpFailures++;
            drain_network_for_us(udp_failure_recovery_delay_us());

            if (reset_dma_engine() != APP_SUCCESS)
            {
                break;
            }

            /*
             * UDP congestion is outside the PL capture path. Keep the PL
             * prebuffer intact and only re-arm DMA for the next transfer.
             */
            active_rx_buffer ^= 1;

            if (ReconfigureRequested)
            {
                printf("Applying pending configuration before UDP recovery re-arm\n");

                restore_acquisition_enable = get_pl_acquisition_enable();
                hold_pl_acquisition_idle();
                pulse_pl_software_reset();
                pulse_fifo_alarm_clear();

                apply_pending_length_related_config();
                ReconfigureRequested = 0;
                SoftwareResetRequested = 0;
            }

            active_frame_words = get_configured_frame_words();
            active_frame_bytes = frame_words_to_dma_bytes(active_frame_words);

            if (start_dma_receive(active_rx_buffer, active_frame_bytes) != APP_SUCCESS)
            {
                printf("WARNING: DMA re-arm failed after UDP failure; resetting and retrying once\n");

                if (reset_dma_engine() != APP_SUCCESS ||
                    start_dma_receive(active_rx_buffer, active_frame_bytes) != APP_SUCCESS)
                {
                    printf("ERROR: DMA re-arm retry failed after UDP failure\n");
                    break;
                }
            }

            if (restore_acquisition_enable)
            {
                set_pl_acquisition_enable(1);
                restore_acquisition_enable = 0;
            }

            pulse_acquisition_arm();

            printf(
                "DMA re-armed on buffer %d for %u bytes after UDP failure; dropped frame %u\n",
                active_rx_buffer,
                active_frame_bytes,
                completed_frame_id
            );

            continue;
        }

        ConsecutiveUdpFailures = 0U;

        if ((completed_frame_id % 100U) == 0U)
        {
            printf(
                "Frame %u handled (dma=%u bytes); UDP frames=%u raw=%u footer=%u payload=%" PRIu64 " bytes; DMA armed on buffer %d for %u bytes\n",
                completed_frame_id,
                completed_frame_bytes,
                UdpFramesSent,
                UdpRawFramesSent,
                UdpFooterOnlyFramesSent,
                UdpPayloadBytesSent,
                active_rx_buffer,
                active_frame_bytes
            );
        }

        service_network();
    }

    printf("Stopping\n");
    cleanup();

    return 0;
}
