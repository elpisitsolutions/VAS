#if !defined LWIPOPTS_H
#define LWIPOPTS_H

#define NO_SYS                          1
#define NO_SYS_NO_TIMERS                1
#define FREERTOS_LWIP                   0

#define LWIP_MPU_COMPATIBLE             0
#define LWIP_TCPIP_CORE_LOCKING         0
#define LWIP_TCPIP_CORE_LOCKING_INPUT   0
#define SYS_LIGHTWEIGHT_PROT            0

#define MEM_ALIGNMENT                   4

#define LWIP_ARP                        1

#define LWIP_ETHERNET                   LWIP_ARP

#define LWIP_IPV4                       1
#define IP_REASSEMBLY                   0
#define IP_FRAG                         0

#define LWIP_ICMP                       1

#define LWIP_RAW                        0

#define LWIP_DHCP                       0

#define LWIP_AUTOIP                     0

#define LWIP_IGMP                       0

#define LWIP_DNS                        0

#define LWIP_UDP                        1

#define LWIP_TCP                        1

#define LWIP_NETIF_EXT_STATUS_CALLBACK  1
#define LWIP_EVENT_API                  0
#define LWIP_CALLBACK_API               1
#define LWIP_NETIF_LINK_CALLBACK        1
#define LWIP_NETCONN                    0
#define LWIP_SOCKET                     0
#define LWIP_STATS                      1
#define LWIP_IPV6                       0
#define LWIP_PERF                       0
#define LWIP_WND_SCALE                  1
#define TCP_RCV_SCALE                   7
#define LWIP_DISABLE_TCP_SANITY_CHECKS  1  

#define TCP_MSS                         960
#define MEMP_NUM_PBUF                   32   // Reduce the number of pbufs
#define MEMP_NUM_TCP_SEG                32   // Reduce the number of TCP segments
#define PBUF_POOL_SIZE                  32   // Reduce the number of pbufs in the pool

// #define LWIP_DEBUG
// #define TCP_DEBUG LWIP_DBG_ON
// #define ETHARP_DEBUG LWIP_DBG_ON
// #define PBUF_DEBUG LWIP_DBG_ON
// #define API_LIB_DEBUG LWIP_DBG_ON
// #define API_MSG_DEBUG LWIP_DBG_ON
// #define SOCKETS_DEBUG LWIP_DBG_ON
// #define ICMP_DEBUG LWIP_DBG_ON
// #define INET_DEBUG LWIP_DBG_ON
// #define IP_DEBUG LWIP_DBG_ON
// #define IP_REASS_DEBUG LWIP_DBG_ON
// #define RAW_DEBUG LWIP_DBG_ON
// #define MEM_DEBUG LWIP_DBG_ON
// #define MEMP_DEBUG LWIP_DBG_ON
// #define SYS_DEBUG LWIP_DBG_ON
// #define TIMERS_DEBUG LWIP_DBG_ON
// #define TCP_INPUT_DEBUG LWIP_DBG_ON
// #define TCP_OUTPUT_DEBUG LWIP_DBG_ON
// #define LWIP_DEBUG
// #define NETIF_DEBUG LWIP_DBG_ON
// #define ETHARP_DEBUG LWIP_DBG_ON
// #define PBUF_DEBUG LWIP_DBG_ON
// #define MEMP_DEBUG LWIP_DBG_ON
#endif
