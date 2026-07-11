#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "xparameters.h"
#include "xstatus.h"
#include "xaxidma.h"
#include "xil_cache.h"
#include "xil_printf.h"
#include "sleep.h"

#include "platform.h"
#include "platform_config.h"

#include "netif/xadapter.h"

#include "lwip/init.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"
#include "lwip/inet.h"
#include "lwip/etharp.h"


/* ============================================================
 * DMA 설정
 * ============================================================ */

#define DMA_DEV_ID             XPAR_AXIDMA_0_DEVICE_ID

#define FRAME_WORDS            131072U
#define FRAME_BYTES            (FRAME_WORDS * 8U)

/* 전송 완료 후 다음 DMA를 arm하기 전까지의 트리거 무시 시간 */
#define TRIGGER_HOLDOFF_SECONDS 10U

/*
 * 두 수신 버퍼는 충분히 떨어진 DDR 주소에 배치
 *
 * Buffer 0: 0x10000000 ~ 0x1007FFFF
 * Buffer 1: 0x10100000 ~ 0x1017FFFF
 */
#define RX_BUFFER0_BASE        0x10000000U
#define RX_BUFFER1_BASE        0x10100000U


/* ============================================================
 * 보드 Ethernet 설정
 * ============================================================ */

#define BOARD_IP0              10
#define BOARD_IP1              0
#define BOARD_IP2              147
#define BOARD_IP3              27

#define BOARD_NETMASK0         255
#define BOARD_NETMASK1         255
#define BOARD_NETMASK2         255
#define BOARD_NETMASK3         0

#define BOARD_GATEWAY0         10
#define BOARD_GATEWAY1         0
#define BOARD_GATEWAY2         147
#define BOARD_GATEWAY3         99


/* ============================================================
 * PC UDP 목적지 설정
 *
 * 실제 PC IP에 맞게 수정
 * ============================================================ */

#define PC_IP0                 10
#define PC_IP1                 0
#define PC_IP2                 147
#define PC_IP3                 72

#define PC_UDP_PORT            5000U


/* ============================================================
 * UDP 패킷 설정
 * ============================================================ */

/*
 * Ethernet MTU = 일반적으로 1500 bytes
 *
 * Custom header : 20 bytes
 * ADC payload   : 최대 1400 bytes
 * UDP header    : 8 bytes
 * IPv4 header   : 20 bytes
 *
 * 총 IP packet:
 * 20 + 8 + 20 + 1400 = 1448 bytes
 */
#define UDP_ADC_PAYLOAD_BYTES  1400U

/* 64K 프레임 연속 송신 시 GEM/lwIP TX 큐 고갈 방지 */
#define UDP_TX_RETRY_COUNT     1000U
#define UDP_TX_RETRY_DELAY_US  100U
#define UDP_PACKET_GAP_US      50U

#define UDP_MAGIC              0xADC64096U

#define TOTAL_UDP_PACKETS      \
    ((FRAME_BYTES + UDP_ADC_PAYLOAD_BYTES - 1U) / \
     UDP_ADC_PAYLOAD_BYTES)


/* ============================================================
 * UDP 사용자 헤더
 *
 * 모든 헤더 값은 Network byte order로 전송
 * ADC payload는 DMA 메모리의 little-endian 원본 그대로 전송
 * ============================================================ */

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


#define UDP_HEADER_BYTES       ((uint32_t)sizeof(UdpAdcHeader))

#define UDP_PACKET_BYTES       \
    (UDP_HEADER_BYTES + UDP_ADC_PAYLOAD_BYTES)


/* ============================================================
 * 전역 객체
 * ============================================================ */

static XAxiDma AxiDma;

static uint64_t *RxBuffer[2] =
{
    (uint64_t *)RX_BUFFER0_BASE,
    (uint64_t *)RX_BUFFER1_BASE
};

static struct netif ServerNetif;
struct netif *echo_netif;
static struct udp_pcb *UdpPcb;
static ip_addr_t PcIpAddr;

static uint32_t FrameId = 0U;

/*
 * UDP 패킷 임시 조립 버퍼
 *
 * 함수 내부 stack 사용을 피하기 위해 전역으로 선언
 */
static uint8_t UdpPacketBuffer[UDP_PACKET_BYTES];


/* ============================================================
 * Xilinx platform timer flag
 * ============================================================ */

extern volatile int TcpFastTmrFlag;
extern volatile int TcpSlowTmrFlag;


/* ============================================================
 * 네트워크 처리
 *
 * DMA 대기 중과 UDP 전송 중 모두 반복 호출
 * ============================================================ */

static void service_network(void)
{
    if (TcpFastTmrFlag)
    {
        tcp_fasttmr();
        TcpFastTmrFlag = 0;
    }

    if (TcpSlowTmrFlag)
    {
        tcp_slowtmr();
        TcpSlowTmrFlag = 0;
    }

    xemacif_input(echo_netif);
}


/* ============================================================
 * 트리거 holdoff 대기
 *
 * 이 함수가 실행되는 동안에는 다음 DMA를 시작하지 않는다.
 * Ethernet 처리가 멈추지 않도록 1 ms 단위로 나누어 대기한다.
 * ============================================================ */

static void wait_trigger_holdoff(void)
{
    uint32_t elapsed_ms;
    const uint32_t holdoff_ms =
        TRIGGER_HOLDOFF_SECONDS * 1000U;

    for (elapsed_ms = 0U;
         elapsed_ms < holdoff_ms;
         elapsed_ms++)
    {
        service_network();
        usleep(1000U);
    }
}


/* ============================================================
 * IP 주소 출력
 * ============================================================ */

static void print_ip_address(
    const char *message,
    const ip_addr_t *ip
)
{
    xil_printf(
        "%s%d.%d.%d.%d\r\n",
        message,
        ip4_addr1(ip),
        ip4_addr2(ip),
        ip4_addr3(ip),
        ip4_addr4(ip)
    );
}


/* ============================================================
 * DMA 상태 읽기
 * ============================================================ */

static u32 read_dma_status(void)
{
    return XAxiDma_ReadReg(
        AxiDma.RegBase,
        XAXIDMA_RX_OFFSET + XAXIDMA_SR_OFFSET
    );
}


static void print_dma_status(void)
{
    u32 status = read_dma_status();

    xil_printf(
        "S2MM_DMASR = 0x%08lx\r\n",
        (unsigned long)status
    );
}


/* ============================================================
 * DMA error 확인
 *
 * DMASR:
 * bit 4 = DMAIntErr
 * bit 5 = DMASlvErr
 * bit 6 = DMADecErr
 * ============================================================ */

static int dma_has_error(void)
{
    u32 status = read_dma_status();

    if ((status & 0x00000070U) != 0U)
    {
        return 1;
    }

    return 0;
}


/* ============================================================
 * DMA 초기화
 * ============================================================ */

static int initialize_dma(void)
{
    XAxiDma_Config *cfg;
    int status;

    cfg = XAxiDma_LookupConfig(DMA_DEV_ID);

    if (cfg == NULL)
    {
        xil_printf(
            "ERROR: DMA configuration not found\r\n"
        );

        return XST_FAILURE;
    }

    status = XAxiDma_CfgInitialize(
        &AxiDma,
        cfg
    );

    if (status != XST_SUCCESS)
    {
        xil_printf(
            "ERROR: DMA initialization failed\r\n"
        );

        return XST_FAILURE;
    }

    if (XAxiDma_HasSg(&AxiDma))
    {
        xil_printf(
            "ERROR: AXI DMA is Scatter-Gather mode\r\n"
        );

        return XST_FAILURE;
    }

    XAxiDma_IntrDisable(
        &AxiDma,
        XAXIDMA_IRQ_ALL_MASK,
        XAXIDMA_DEVICE_TO_DMA
    );

    xil_printf("DMA initialized\r\n");

    xil_printf(
        "FRAME_WORDS = %lu\r\n",
        (unsigned long)FRAME_WORDS
    );

    xil_printf(
        "FRAME_BYTES = %lu\r\n",
        (unsigned long)FRAME_BYTES
    );

    return XST_SUCCESS;
}


/* ============================================================
 * 지정된 버퍼에 DMA 수신 시작
 * ============================================================ */

static int start_dma_receive(uint64_t *buffer)
{
    int status;

    /*
     * DMA가 쓸 메모리에 남아 있을 수 있는 cache line 무효화
     */
    Xil_DCacheInvalidateRange(
        (UINTPTR)buffer,
        FRAME_BYTES
    );

    status = XAxiDma_SimpleTransfer(
        &AxiDma,
        (UINTPTR)buffer,
        FRAME_BYTES,
        XAXIDMA_DEVICE_TO_DMA
    );

    if (status != XST_SUCCESS)
    {
        xil_printf(
            "ERROR: DMA transfer start failed\r\n"
        );

        print_dma_status();

        return XST_FAILURE;
    }

    return XST_SUCCESS;
}


/* ============================================================
 * DMA 완료 여부
 * ============================================================ */

static int dma_receive_complete(void)
{
    if (XAxiDma_Busy(
            &AxiDma,
            XAXIDMA_DEVICE_TO_DMA
        ))
    {
        return 0;
    }

    return 1;
}


/* ============================================================
 * DMA 완료 대기
 *
 * 대기 중에도 Ethernet RX와 lwIP timer 처리
 * ============================================================ */

static int wait_for_dma_complete(void)
{
    while (!dma_receive_complete())
    {
        service_network();
    }

    if (dma_has_error())
    {
        xil_printf(
            "ERROR: DMA transfer completed with error\r\n"
        );

        print_dma_status();

        return XST_FAILURE;
    }

    return XST_SUCCESS;
}


/* ============================================================
 * UDP 초기화
 * ============================================================ */

static int initialize_udp(void)
{
    err_t err;

    UdpPcb = udp_new();

    if (UdpPcb == NULL)
    {
        xil_printf(
            "ERROR: udp_new failed\r\n"
        );

        return XST_FAILURE;
    }

    IP4_ADDR(
        &PcIpAddr,
        PC_IP0,
        PC_IP1,
        PC_IP2,
        PC_IP3
    );

    err = udp_connect(
        UdpPcb,
        &PcIpAddr,
        PC_UDP_PORT
    );

    if (err != ERR_OK)
    {
        xil_printf(
            "ERROR: udp_connect failed: %d\r\n",
            err
        );

        udp_remove(UdpPcb);
        UdpPcb = NULL;

        return XST_FAILURE;
    }

    xil_printf(
        "UDP destination = %d.%d.%d.%d:%lu\r\n",
        PC_IP0,
        PC_IP1,
        PC_IP2,
        PC_IP3,
        (unsigned long)PC_UDP_PORT
    );

    xil_printf(
        "UDP packets per frame = %lu\r\n",
        (unsigned long)TOTAL_UDP_PACKETS
    );

    return XST_SUCCESS;
}


/* ============================================================
 * 첫 ADC 프레임 전송 전 PC의 ARP 정보 준비
 *
 * 첫 udp_send()에서 ARP 해석이 시작되면 앞쪽 UDP 패킷이
 * 유실될 수 있으므로 DMA를 arm하기 전에 ARP 요청을 보낸다.
 * ============================================================ */

static void prepare_pc_arp(void)
{
    uint32_t elapsed_ms;
    err_t err;

    xil_printf("Resolving PC MAC address...\r\n");

    err = etharp_request(
        echo_netif,
        ip_2_ip4(&PcIpAddr)
    );

    if (err != ERR_OK)
    {
        xil_printf(
            "WARNING: ARP request failed: %d\r\n",
            err
        );
    }

    /* ARP 응답을 받을 수 있도록 1초 동안 Ethernet 처리 */
    for (elapsed_ms = 0U; elapsed_ms < 1000U; elapsed_ms++)
    {
        service_network();
        usleep(1000U);
    }

    xil_printf("ARP preparation complete\r\n");
}


/* ============================================================
 * UDP 패킷 한 개 전송
 * ============================================================ */

static int send_udp_packet(
    uint32_t frame_id,
    uint16_t packet_index,
    uint16_t total_packets,
    const uint8_t *payload,
    uint16_t payload_bytes
)
{
    UdpAdcHeader header;

    struct pbuf *p;
    err_t err;
    uint32_t retry_count;

    uint16_t packet_bytes;

    /*
     * 헤더는 network byte order
     */
    header.magic =
        htonl(UDP_MAGIC);

    header.frame_id =
        htonl(frame_id);

    header.packet_index =
        htons(packet_index);

    header.total_packets =
        htons(total_packets);

    header.payload_bytes =
        htons(payload_bytes);

    header.reserved =
        htons(0U);

    header.frame_bytes =
        htonl(FRAME_BYTES);

    memcpy(
        UdpPacketBuffer,
        &header,
        sizeof(header)
    );

    memcpy(
        UdpPacketBuffer + sizeof(header),
        payload,
        payload_bytes
    );

    packet_bytes =
        (uint16_t)(sizeof(header) + payload_bytes);

    p = pbuf_alloc(
        PBUF_TRANSPORT,
        packet_bytes,
        PBUF_RAM
    );

    if (p == NULL)
    {
        xil_printf(
            "ERROR: pbuf_alloc failed\r\n"
        );

        return XST_FAILURE;
    }

    err = pbuf_take(
        p,
        UdpPacketBuffer,
        packet_bytes
    );

    if (err != ERR_OK)
    {
        xil_printf(
            "ERROR: pbuf_take failed: %d\r\n",
            err
        );

        pbuf_free(p);

        return XST_FAILURE;
    }

    retry_count = 0U;

    do
    {
        err = udp_send(
            UdpPcb,
            p
        );

        if (err != ERR_MEM)
            break;

        /* TX descriptor가 반환될 때까지 Ethernet 처리 후 재시도 */
        service_network();
        usleep(UDP_TX_RETRY_DELAY_US);
        retry_count++;

    } while (retry_count < UDP_TX_RETRY_COUNT);

    pbuf_free(p);

    if (err != ERR_OK)
    {
        xil_printf(
            "ERROR: udp_send failed: %d\r\n",
            err
        );

        return XST_FAILURE;
    }

    return XST_SUCCESS;
}


/* ============================================================
 * 한 DMA 프레임을 UDP 여러 패킷으로 전송
 * ============================================================ */

static int send_adc_frame(
    uint64_t *buffer,
    uint32_t frame_id
)
{
    const uint8_t *frame_data =
        (const uint8_t *)buffer;

    uint32_t offset = 0U;
    uint32_t remaining;

    uint16_t packet_index;
    uint16_t payload_bytes;

    const uint16_t total_packets =
        (uint16_t)TOTAL_UDP_PACKETS;

    for (
        packet_index = 0U;
        packet_index < total_packets;
        packet_index++
    )
    {
        remaining =
            FRAME_BYTES - offset;

        if (remaining > UDP_ADC_PAYLOAD_BYTES)
        {
            payload_bytes =
                (uint16_t)UDP_ADC_PAYLOAD_BYTES;
        }
        else
        {
            payload_bytes =
                (uint16_t)remaining;
        }

        if (send_udp_packet(
                frame_id,
                packet_index,
                total_packets,
                frame_data + offset,
                payload_bytes
            ) != XST_SUCCESS)
        {
            xil_printf(
                "ERROR: UDP packet %u transmission failed\r\n",
                packet_index
            );

            return XST_FAILURE;
        }

        offset += payload_bytes;

        /*
         * Ethernet RX 및 lwIP timer 지속 처리
         */
        service_network();
        usleep(UDP_PACKET_GAP_US);
    }

    return XST_SUCCESS;
}


/* ============================================================
 * Ethernet 초기화
 * ============================================================ */

static int initialize_ethernet(void)
{
    ip_addr_t ip_address;
    ip_addr_t netmask;
    ip_addr_t gateway;

    unsigned char mac_address[6] =
    {
        0x00,
        0x0A,
        0x35,
        0x00,
        0x01,
        0x02
    };

    echo_netif =
        &ServerNetif;

    IP4_ADDR(
        &ip_address,
        BOARD_IP0,
        BOARD_IP1,
        BOARD_IP2,
        BOARD_IP3
    );

    IP4_ADDR(
        &netmask,
        BOARD_NETMASK0,
        BOARD_NETMASK1,
        BOARD_NETMASK2,
        BOARD_NETMASK3
    );

    IP4_ADDR(
        &gateway,
        BOARD_GATEWAY0,
        BOARD_GATEWAY1,
        BOARD_GATEWAY2,
        BOARD_GATEWAY3
    );

    lwip_init();

    if (!xemac_add(
    		echo_netif,
            &ip_address,
            &netmask,
            &gateway,
            mac_address,
            PLATFORM_EMAC_BASEADDR
        ))
    {
        xil_printf(
            "ERROR: adding Ethernet interface failed\r\n"
        );

        return XST_FAILURE;
    }

    netif_set_default(
    		echo_netif
    );

    platform_enable_interrupts();

    netif_set_up(
    		echo_netif
    );

    print_ip_address(
        "Board IP : ",
        &ip_address
    );

    print_ip_address(
        "Netmask  : ",
        &netmask
    );

    print_ip_address(
        "Gateway  : ",
        &gateway
    );

    return XST_SUCCESS;
}


/* ============================================================
 * main
 * ============================================================ */

int main(void)
{
    int status;

    int active_rx_buffer;
    int completed_buffer;

    uint32_t completed_frame_id;

    init_platform();

    xil_printf(
        "\r\n"
        "========================================\r\n"
        " ADC Trigger DMA UDP Ping-Pong\r\n"
        "========================================\r\n"
    );

    status = initialize_ethernet();

    if (status != XST_SUCCESS)
    {
        xil_printf(
            "Ethernet initialization failed\r\n"
        );

        cleanup_platform();

        return -1;
    }

    status = initialize_udp();

    if (status != XST_SUCCESS)
    {
        xil_printf(
            "UDP initialization failed\r\n"
        );

        cleanup_platform();

        return -1;
    }

    status = initialize_dma();

    if (status != XST_SUCCESS)
    {
        xil_printf(
            "DMA initialization failed\r\n"
        );

        udp_remove(UdpPcb);

        cleanup_platform();

        return -1;
    }

    /* 첫 프레임 UDP 패킷 유실 방지를 위해 DMA보다 먼저 실행 */
    prepare_pc_arp();

    /*
     * 첫 번째 DMA는 Buffer 0에서 시작
     */
    active_rx_buffer = 0;

    status = start_dma_receive(
        RxBuffer[active_rx_buffer]
    );

    if (status != XST_SUCCESS)
    {
        xil_printf(
            "First DMA start failed\r\n"
        );

        udp_remove(UdpPcb);

        cleanup_platform();

        return -1;
    }

    xil_printf(
        "\r\nWaiting for first trigger frame...\r\n"
    );

    while (1)
    {
        /*
         * 현재 DMA가 한 프레임을 수신할 때까지 대기
         */
        status = wait_for_dma_complete();

        if (status != XST_SUCCESS)
        {
            xil_printf(
                "DMA receive failed\r\n"
            );

            break;
        }

        /*
         * 방금 완료된 버퍼 번호 보존
         */
        completed_buffer =
            active_rx_buffer;

        /*
         * DMA가 쓴 메모리를 CPU가 읽기 전에 invalidate
         */
        Xil_DCacheInvalidateRange(
            (UINTPTR)RxBuffer[completed_buffer],
            FRAME_BYTES
        );

        /*
         * 완료 프레임 번호 증가
         */
        FrameId++;

        completed_frame_id =
            FrameId;

        /*
         * 완료된 프레임 하나만 UDP로 전송
         * 이 시점에는 다음 DMA를 시작하지 않았으므로 holdoff 상태
         */
        status = send_adc_frame(
            RxBuffer[completed_buffer],
            completed_frame_id
        );

        if (status != XST_SUCCESS)
        {
            xil_printf(
                "ERROR: frame UDP transmission failed\r\n"
            );

            break;
        }

        xil_printf(
            "Frame %lu sent, ignoring triggers for %lu seconds...\r\n",
            (unsigned long)completed_frame_id,
            (unsigned long)TRIGGER_HOLDOFF_SECONDS
        );

        wait_trigger_holdoff();

        /* holdoff 종료 후 반대쪽 버퍼에 다음 한 프레임 수신 arm */
        active_rx_buffer ^= 1;

        status = start_dma_receive(
            RxBuffer[active_rx_buffer]
        );

        if (status != XST_SUCCESS)
        {
            xil_printf(
                "ERROR: next DMA start failed\r\n"
            );

            break;
        }

        xil_printf(
            "DMA armed on buffer %d, waiting for next trigger...\r\n",
            active_rx_buffer
        );
    }

    if (UdpPcb != NULL)
    {
        udp_remove(
            UdpPcb
        );
    }

    cleanup_platform();

    return 0;
}
