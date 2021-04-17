/* Digital I/O
*
* CE Chip Enable Activates RX or TX mode
* CSN SPI Chip Select
* SCK SPI Clock
* MOSI SPI Slave Data Input
* MISO SPI Slave Data Output, with tri-state option
* IRQ Maskable interrupt pin. Active low
*
*/

/*
* nrf24 operations
*/

#ifndef __NRF24L01_DEFS_H__
#define __NRF24L01_DEFS_H__

#include <stdint.h>

#define R_REGISTER                  0b00000000           /* Read command and status registers  UINT8_C(5LSB - register addr) */
#define W_REGISTER                  0b00100000           /* Write command and status registers  UINT8_C(5LSB - register addr) */
#define R_RX_PAYLOAD                0b01100001           /* Read RX-payload: 1 - 32 bytes */
#define W_TX_PAYLOAD                0b10100000           /* Write TX-payload */
#define FLUSH_TX                    0b11100001           /* Flush TX FIFO, used in TX mode */
#define FLUSH_RX                    0b11100010           /* Flush RX FIFO, used in RX mode */
#define REUSE_TX_PL                 0b11100011           /* Reuse last transmitted payload */
#define R_RX_PL_WID                 0b01100000           /* Read RX payload width for the top */
#define W_ACK_PAYLOAD               0b10101000           /* Write Payload to be transmitted together with ACK packet on PIPE 3LZB - PPP */
#define W_TX_PAYLOAD_NO_ACK         0b10110000           /* Disables AUTOACK on this specific packet. */
#define NOP                         0b11111111           /* No Operation */
 

/*
 * nrf24 registers
 */


#define CONFIG                 UINT8_C(0x00)                  /* Configuration Register */
#define MASK_RX_DR             UINT8_C(1<<6)
#define MASK_TX_DS             UINT8_C(1<<5)
#define MASK_MAX_RT            UINT8_C(1<<4)
#define EN_CRC                 UINT8_C(1<<3)
#define CRCO                   UINT8_C(1<<2)
#define PWR_UP                 UINT8_C(1<<1)
#define PRIM_RX                UINT8_C(1<<0)

#define EN_AA                  UINT8_C(0x01)                /* Enable ‘Auto Acknowledgment’ */
#define ENAA_P5                UINT8_C(1<<5)
#define ENAA_P4                UINT8_C(1<<4)
#define ENAA_P3                UINT8_C(1<<3)
#define ENAA_P2                UINT8_C(1<<2)
#define ENAA_P1                UINT8_C(1<<1)
#define ENAA_P0                UINT8_C(1<<0)

#define EN_RXADDR              UINT8_C(0x02)                /* Enabled RX Addresses */
#define ERX_P5                 UINT8_C(1<<5)
#define ERX_P4                 UINT8_C(1<<4)
#define ERX_P3                 UINT8_C(1<<3)
#define ERX_P2                 UINT8_C(1<<2)
#define ERX_P1                 UINT8_C(1<<1)
#define ERX_P0                 UINT8_C(1<<0)

#define SETUP_AW               UINT8_C(0x03)               /* Setup of Address Widths 3-5 */
#define AW_LEN3 			   UINT8_C(0x01)
#define AW_LEN4 			   UINT8_C(0x02)
#define AW_LEN5 			   UINT8_C(0x03)

#define SETUP_RETR             UINT8_C(0x04)               /* Setup of automatic retransmission */
#define ARD                    UINT8_C(0xF0)
#define ARC                    UINT8_C(0x0F)

#define RF_CH                  UINT8_C(0x05)               /* RF Channel 6:0 */

#define RF_SETUP               UINT8_C(0x06)               /* RF Setup Register */
#define CONT_WAVE              UINT8_C(1<<7)
#define RF_DR_LOW              UINT8_C(1<<5)
#define PLL_LOCK               UINT8_C(1<<4)
#define RF_DR_HIGH             UINT8_C(1<<3)
#define RF_PWR                 UINT8_C(0x06)

#define DR_1MBS				   UINT8_C(0x00)
#define DR_2MBS				   UINT8_C(0x04)
#define DR_250KBS			   UINT8_C(0x10)

#define PWR_m18dBm 		       UINT8_C(0x00)
#define PWR_m12dBm 		       UINT8_C(0x01)
#define PWR_m6dBm 		       UINT8_C(0x02)
#define PWR_0dBm 		       UINT8_C(0x03)

#define STATUS                 UINT8_C(0x07)             /* Status Register */
#define RX_DR                  UINT8_C(1<<6)
#define TX_DS                  UINT8_C(1<<5)
#define MAX_RT                 UINT8_C(1<<4)
#define RX_P_NO                UINT8_C(0x0D)
#define TX_FULL                UINT8_C(1<<0)

#define OBSERVE_TX             UINT8_C(0x08)              /* Transmit observe register */
#define PLOS_CNT               UINT8_C(0xF0)
#define ARC_CNT                UINT8_C(0x0F)

#define RPD                    UINT8_C(0x09)             /* Received Power Detecto */

#define RX_ADDR_P0             UINT8_C(0x0A)             /* Receive address data pipe 0 3-5 bytes */
#define RX_ADDR_P1             UINT8_C(0x0B)             /* Receive address data pipe 1 3-5 bytes */
#define RX_ADDR_P2             UINT8_C(0x0C)             /* Receive address data pipe 2 LSB only */
#define RX_ADDR_P3             UINT8_C(0x0D)             /* Receive address data pipe 3 LSB only */
#define RX_ADDR_P4             UINT8_C(0x0E)             /* Receive address data pipe 4 LSB only */
#define RX_ADDR_P5             UINT8_C(0x0F)             /* Receive address data pipe 5 LSB only */

#define TX_ADDR                UINT8_C(0x10)             /* Transmit address Used for a PTX device only */

#define RX_PW_P0               UINT8_C(0x11)             /* Number of bytes in RX payload in data pipe 0 */
#define RX_PW_P1               UINT8_C(0x12)             /* Number of bytes in RX payload in data pipe 1 */
#define RX_PW_P2               UINT8_C(0x13)             /* Number of bytes in RX payload in data pipe 2 */
#define RX_PW_P3               UINT8_C(0x14)             /* Number of bytes in RX payload in data pipe 3 */
#define RX_PW_P4               UINT8_C(0x15)             /* Number of bytes in RX payload in data pipe 4 */
#define RX_PW_P5               UINT8_C(0x16)             /* Number of bytes in RX payload in data pipe 5 */

#define FIFO_STATUS            UINT8_C(0x17)             /* FIFO Status Register */
#define FIFO_TX_REUSE          UINT8_C(1<<6)
#define FIFO_TX_FULL           UINT8_C(1<<5)
#define FIFO_TX_EMPTY          UINT8_C(1<<4)
#define FIFO_RX_FULL           UINT8_C(1<<1)
#define FIFO_RX_EMPTY          UINT8_C(1<<0)

#define DYNPD                  UINT8_C(0x1C)             /* Enable dynamic payload length */
#define DPL_P5                 UINT8_C(1<<5)
#define DPL_P4                 UINT8_C(1<<4)
#define DPL_P3                 UINT8_C(1<<3)
#define DPL_P2                 UINT8_C(1<<2)
#define DPL_P1                 UINT8_C(1<<1)
#define DPL_P0                 UINT8_C(1<<0)

#define FEATURE                UINT8_C(0x1D)          /* Feature Register */
#define EN_DPL                 UINT8_C(1<<2)
#define EN_ACK_PAY             UINT8_C(1<<1)
#define EN_DYN_ACK             UINT8_C(1<<0)

#endif
