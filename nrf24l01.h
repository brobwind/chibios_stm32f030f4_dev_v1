/*
 * Copyright (C) 2016 https://www.brobwind.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#include "hal.h"


#if !HAL_USE_SPI || !HAL_USE_EXT
#error "NRF24L01 requires HAL_USE_SPI and HAL_USE_EXT"
#endif


/* nRF24L01 Instruction Definitions */
#define NRF24_CMD_WRITE_REG         0x20  /**< Register write command */
#define NRF24_CMD_RD_RX_PLOAD_WID   0x60  /**< Read RX payload command */
#define NRF24_CMD_RD_RX_PLOAD       0x61  /**< Read RX payload command */
#define NRF24_CMD_WR_TX_PLOAD       0xA0  /**< Write TX payload command */
#define NRF24_CMD_WR_ACK_PLOAD      0xA8  /**< Write ACK payload command */
#define NRF24_CMD_WR_NACK_TX_PLOAD  0xB0  /**< Write ACK payload command */
#define NRF24_CMD_FLUSH_TX          0xE1  /**< Flush TX register command */
#define NRF24_CMD_FLUSH_RX          0xE2  /**< Flush RX register command */
#define NRF24_CMD_REUSE_TX_PL       0xE3  /**< Reuse TX payload command */
#define NRF24_CMD_ACTIVATE          0x50  /**< Activate command */
#define NRF24_CMD_NOP               0xFF  /**< No Operation command, used for reading status register */

/* nRF24L01 * Register Definitions * */
#define NRF24_CONFIG                0x00  /**< nRF24L01 config register */
#define NRF24_MASK_RX_DR            (1 << 6)
#define NRF24_MASK_TX_DS            (1 << 5)
#define NRF24_MAX_RT                (1 << 4)
#define NRF24_EN_CRC                (1 << 3)
#define NRF24_CRCO                  (1 << 2)
#define NRF24_PWR_UP                (1 << 1)
#define NRF24_PRIM_RX               (1 << 0)

#define NRF24_EN_AA                  0x01  /**< nRF24L01 enable Auto-Acknowledge register */
#define NRF24_ENAA_P5                (1 << 5)
#define NRF24_ENAA_P4                (1 << 4)
#define NRF24_ENAA_P3                (1 << 3)
#define NRF24_ENAA_P2                (1 << 2)
#define NRF24_ENAA_P1                (1 << 1)
#define NRF24_ENAA_P0                (1 << 0)

#define NRF24_EN_RXADDR              0x02  /**< nRF24L01 enable RX addresses register */
#define NRF24_ERX_P5                 (1 << 5)
#define NRF24_ERX_P4                 (1 << 4)
#define NRF24_ERX_P3                 (1 << 3)
#define NRF24_ERX_P2                 (1 << 2)
#define NRF24_ERX_P1                 (1 << 1)
#define NRF24_ERX_P0                 (1 << 0)

#define NRF24_SETUP_AW               0x03  /**< nRF24L01 setup of address width register */
#define NRF24_AW_3BYTES              0x01
#define NRF24_AW_4BYTES              0x02
#define NRF24_AW_5BYTES              0x03

#define NRF24_SETUP_RETR             0x04  /**< nRF24L01 setup of automatic retransmission register */
#define NRF24_ARD_250US              0x00
#define NRF24_ARD_500US              0x10
#define NRF24_ARD_750US              0x20
#define NRF24_ARD_1000US             0x30
#define NRF24_ARD_1250US             0x40
#define NRF24_ARD_1500US             0x50
#define NRF24_ARD_1750US             0x60
#define NRF24_ARD_2000US             0x70
#define NRF24_ARD_2250US             0x80
#define NRF24_ARD_2500US             0x90
#define NRF24_ARD_2750US             0xA0
#define NRF24_ARD_3000US             0xB0
#define NRF24_ARD_3250US             0xC0
#define NRF24_ARD_3500US             0xD0
#define NRF24_ARD_3750US             0xE0
#define NRF24_ARD_4000US             0xF0
#define NRF24_ARC_0RT                0x00
#define NRF24_ARC_1RT                0x01
#define NRF24_ARC_2RT                0x02
#define NRF24_ARC_3RT                0x03
#define NRF24_ARC_4RT                0x04
#define NRF24_ARC_5RT                0x05
#define NRF24_ARC_6RT                0x06
#define NRF24_ARC_7RT                0x07
#define NRF24_ARC_8RT                0x08
#define NRF24_ARC_9RT                0x09
#define NRF24_ARC_10RT               0x0A
#define NRF24_ARC_11RT               0x0B
#define NRF24_ARC_12RT               0x0C
#define NRF24_ARC_13RT               0x0D
#define NRF24_ARC_14RT               0x0E
#define NRF24_ARC_15RT               0x0F

#define NRF24_RF_CH                  0x05  /**< nRF24L01 RF channel register */

#define NRF24_RF_SETUP               0x06  /**< nRF24L01 RF setup register */
#define NRF24_CONT_WAVE              (1 << 7)
#define NRF24_RF_DR_LOW              (1 << 5)
#define NRF24_PLL_LOCK               (1 << 4)
#define NRF24_RF_DR_HIGH             (1 << 3)
#define NRF24_RF_PWR_18DBM           (0x00 << 1)
#define NRF24_RF_PWR_12DBM           (0x01 << 1)
#define NRF24_RF_PWR_6DBM            (0x02 << 1)
#define NRF24_RF_PWR_0DBM            (0x03 << 1)
#define NRF24_LNA_HCURR              (1 << 0)

#define NRF24_STATUS                 0x07  /**< nRF24L01 status register */
#define NRF24_RX_DR                  (1 << 6)
#define NRF24_TX_DS                  (1 << 5)
#define NRF24_MAX_RT                 (1 << 4)
#define NRF24_RX_P_NO_MASK           (0x07 << 1)
#define NRF24_TX_FULL                (1 << 0)

#define NRF24_OBSERVE_TX             0x08  /**< nRF24L01 transmit observe register */
#define NRF24_PLOS_CNT_MASK          (0x0F << 4)
#define NRF24_ARC_CNT_MASK           (0x0F << 0)

#define NRF24_CD                     0x09  /**< nRF24L01 carrier detect register */
#define NRF24_RX_ADDR_P0             0x0A  /**< nRF24L01 receive address data pipe0 */
#define NRF24_RX_ADDR_P1             0x0B  /**< nRF24L01 receive address data pipe1 */
#define NRF24_RX_ADDR_P2             0x0C  /**< nRF24L01 receive address data pipe2 */
#define NRF24_RX_ADDR_P3             0x0D  /**< nRF24L01 receive address data pipe3 */
#define NRF24_RX_ADDR_P4             0x0E  /**< nRF24L01 receive address data pipe4 */
#define NRF24_RX_ADDR_P5             0x0F  /**< nRF24L01 receive address data pipe5 */
#define NRF24_TX_ADDR                0x10  /**< nRF24L01 transmit address */
#define NRF24_RX_PW_P0               0x11  /**< nRF24L01 \# of bytes in rx payload for pipe0 */
#define NRF24_RX_PW_P1               0x12  /**< nRF24L01 \# of bytes in rx payload for pipe1 */
#define NRF24_RX_PW_P2               0x13  /**< nRF24L01 \# of bytes in rx payload for pipe2 */
#define NRF24_RX_PW_P3               0x14  /**< nRF24L01 \# of bytes in rx payload for pipe3 */
#define NRF24_RX_PW_P4               0x15  /**< nRF24L01 \# of bytes in rx payload for pipe4 */
#define NRF24_RX_PW_P5               0x16  /**< nRF24L01 \# of bytes in rx payload for pipe5 */

#define NRF24_FIFO_STATUS            0x17  /**< nRF24L01 FIFO status register */
#define NRF24_FIFO_TX_REUSE          (1 << 6)
#define NRF24_FIFO_TX_FULL           (1 << 5)
#define NRF24_FIFO_TX_EMPTY          (1 << 4)
#define NRF24_FIFO_RX_FULL           (1 << 1)
#define NRF24_FIFO_RX_EMPTY          (0 << 1)

#define NRF24_DYNPD                  0x1C  /**< nRF24L01 Dynamic payload setup */
#define NRF24_DPL_P5                 (1 << 5)
#define NRF24_DPL_P4                 (1 << 4)
#define NRF24_DPL_P3                 (1 << 3)
#define NRF24_DPL_P2                 (1 << 2)
#define NRF24_DPL_P1                 (1 << 1)
#define NRF24_DPL_P0                 (1 << 0)

#define NRF24_FEATURE                0x1D  /**< nRF24L01 Exclusive feature setup */
#define NRF24_EN_DPL                 (1 << 2)
#define NRF24_EN_ACK_PAY             (1 << 1)
#define NRF24_EN_DYN_ACK             (1 << 0)

#define NRF24L01_MAX_ADDR_LEN        5
#define NRF24L01_MAX_PL_LEN          32
#define NRF24L01_GENERIC_IRQ         1

#define NRF24L01_PKT_BEG             0x00 // 0b0000 0000
#define NRF24L01_PKT_MID             0x40 // 0b0100 0000
#define NRF24L01_PKT_END             0xE0 // 0b1110 0000
#define NRF24L01_PKT_ACK             0xA0 // 0b1010 0000

#if 0
/* An enum describing the radio's irq sources */
typedef enum {
    NRF24_MAX_RT = 4,                /**< Max retries interrupt */
    NRF24_TX_DS,                     /**< TX data sent interrupt */
    NRF24_RX_DR                      /**< RX data received interrupt */
} nrf24_irq_source_t;

/* An enum describing the radio's power mode */
typedef enum {
    NRF24_PTX,                        /**< Primary TX operation */
    NRF24_PRX                         /**< Primary RX operation */
} nrf24_operation_mode_t;

/* An enum describing the radio's power mode */
typedef enum {
    NRF24_PWR_DOWN,                   /**< Device power-down */
    NRF24_PWR_UP                      /**< Device power-up */
} nrf24_pwr_mode_t;

/* An enum describing the radio's output power mode */
typedef enum {
    NRF24_18DBM,                      /**< Output power set to -18dBm */
    NRF24_12DBM,                      /**< Output power set to -12dBm */
    NRF24_6DBM,                       /**< Output power set to -6dBm */
    NRF24_0DBM                        /**< Output power set to 0dBm */
} nrf24_output_power_t;

/* An enum describing the radio's on-air datarate */
typedef enum {
    NRF24_1MBPS,                      /**< Datarate set to 1Mbps */
    NRF24_2MBPS                       /**< Datarate set to 2Mbps */
} nrf24_datarate_t;

/* An enum describing the radio's PLL mode */
typedef enum {
    NRF24_PLL_UNLOCK,                 /**< PLL unlocked, normal operation */
    NRF24_PLL_LOCK                    /**< PLL locked, test mode */
} nrf24_pll_mode_t;

/* An enum describing the radio's LNA mode */
typedef enum {
    NRF24_LNA_LCURR,                  /**< LNA set to low current mode */
    NRF24_LNA_HCURR                   /**< LNA set to high current mode */
} nrf24_lna_mode_t;

/* An enum describing the radio's CRC mode */
typedef enum {
    NRF24_CRC_OFF,                    /**< CRC check disabled */
    NRF24_CRC_8BIT = 2,               /**< CRC check set to 8-bit */
    NRF24_CRC_16BIT                   /**< CRC check set to 16-bit */
} nrf24_crc_mode_t;

/* An enum describing the read/write payload command */
typedef enum {
    NRF24_TX_PLOAD = 7,               /**< TX payload definition */
    NRF24_RX_PLOAD,                   /**< RX payload definition */
    NRF24_ACK_PLOAD
} nrf24_pload_comamnd_t;

/* An enum describing the nRF24L01 pipe addresses and TX address */
typedef enum {
    NRF24_PIPE0,                      /**< Select pipe0 */
    NRF24_PIPE1,                      /**< Select pipe1 */
    NRF24_PIPE2,                      /**< Select pipe2 */
    NRF24_PIPE3,                      /**< Select pipe3 */
    NRF24_PIPE4,                      /**< Select pipe4 */
    NRF24_PIPE5,                      /**< Select pipe5 */
    NRF24_TX,                         /**< Refer to TX address */
    NRF24_ALL = 0xff                  /**< Close or opern all pipes */
} nrf24_address_t;

/* An enum describing the radio's adress width */
typedef enum {
    NRF24_AW_3BYTES = 3,              /**< Set address width to 3 bytes */
    NRF24_AW_4BYTES,                  /**< Set address width to 4 bytes */
    NRF24_AW_5BYTES                   /**< Set address width to 5 bytes */
} nrf24_address_width_t;
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

typedef enum {
    NRF24_STATE_UNINIT,
    NRF24_STATE_STOP,
    NRF24_STATE_READY,
} nrf24_state_t;

typedef struct {
    ioportid_t ceport;                // Chip enable line port
    uint16_t cepad;                   // Chip enable line pad

    ioportid_t irqport;               // The interrupt line port
    uint16_t irqpad;                  // The interrupt line pad

    SPIDriver *spip;                  // SPI driver associated to this RF
    const SPIConfig *spicfg;          // SPI configuration

    uint8_t addrlen;
} NRF24Config;

#define _nrf24_methods \
    int16_t (*rcvPkt)(void *ip, systime_t rxtime, uint8_t *buf, uint16_t len); \
    int16_t (*sndPkt)(void *ip, systime_t rxtime, uint8_t *buf, uint16_t len);

struct NRF24VMT {
    _nrf24_methods
};

#define _nrf24_data \
    nrf24_state_t state; \
    const NRF24Config *config; \
    event_source_t irq_event; \
    event_listener_t el; \
    mutex_t mutex; \
    uint8_t txaddr[NRF24L01_MAX_ADDR_LEN]; \
    uint8_t rxaddr[NRF24L01_MAX_ADDR_LEN];

typedef struct NRF24Driver {
    const struct NRF24VMT *vmt;
    _nrf24_data;
} NRF24Driver;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

#define nrf24RcvPkt(ip, rxtime, buf, len) \
    (ip)->vmt->rcvPkt(ip, rxtime, buf, len)

#define nrf24SndPkt(ip, rxtime, buf, len) \
    (ip)->vmt->sndPkt(ip, rxtime, buf, len)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

void nrf24ObjectInit(NRF24Driver *devp, const uint8_t *address);
void nrf24Start(NRF24Driver *devp, const NRF24Config *config);
void nrf24Stop(NRF24Driver *devp);

#ifdef __cplusplus
}
#endif

#endif /* __NRF24L01_H__ */
