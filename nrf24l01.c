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

#include <string.h>

#include "hal.h"

#include "nrf24l01.h"


#define NRF24L01_WITH_PA            0

#define MIN(a, b) ((a) < (b) ? (a) : (b))


/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static uint8_t nrf24Exchange(NRF24Driver *drvp, uint8_t *txbuf, uint8_t *rxbuf, uint8_t len) {
    SPIDriver *spip = drvp->config->spip;

    chMtxLock(&drvp->mutex);
    spiSelect(spip);
    spiExchange(spip, len, txbuf, rxbuf);
    spiUnselect(spip);
    chMtxUnlock(&drvp->mutex);

    return rxbuf[0];
}

static uint8_t nrf24RegRead(NRF24Driver *drvp, uint8_t reg, uint8_t *rxbuf, uint8_t len) {
    uint8_t status;
    SPIDriver *spip = drvp->config->spip;

    chMtxLock(&drvp->mutex);
    spiSelect(spip);
    spiExchange(spip, 1, &reg, &status);
    spiReceive(spip, len, rxbuf);
    spiUnselect(spip);
    chMtxUnlock(&drvp->mutex);

    return status;
}

static uint8_t nrf24RegWrite(NRF24Driver *drvp, uint8_t reg, uint8_t *txbuf, uint8_t len) {
    uint8_t status;
    SPIDriver *spip = drvp->config->spip;

    chMtxLock(&drvp->mutex);
    spiSelect(spip);
    spiExchange(spip, 1, &reg, &status);
    spiSend(spip, len, txbuf);
    spiUnselect(spip);
    chMtxUnlock(&drvp->mutex);

    return status;
}

// ----- Command -----

/**
 * R_REGISTER
 * Read command and status registers.
 */
static uint8_t __attribute__ ((unused)) nrf24CmdRd(NRF24Driver *drvp, uint8_t reg, uint8_t *buf, uint8_t len) {
    return nrf24RegRead(drvp, reg, buf, len);
}

static uint8_t nrf24CmdRdOb(NRF24Driver *drvp, uint8_t reg, uint8_t *val) {
    return nrf24RegRead(drvp, reg, val, 1);
}

/**
 * W_REGISTER
 * Write command and status registers.
 * Executable in power down or standby modes only.
 */
static uint8_t nrf24CmdWr(NRF24Driver *drvp, uint8_t reg, uint8_t *buf, uint8_t len) {
    return nrf24RegWrite(drvp, NRF24_CMD_WRITE_REG | reg , buf, len);
}

static uint8_t nrf24CmdWrOb(NRF24Driver *drvp, uint8_t reg, uint8_t val) {
    return nrf24RegWrite(drvp, NRF24_CMD_WRITE_REG | reg, &val, 1);
}

/**
 * R_RX_PAYLOAD
 * Read RX-payload: 1 – 32 bytes. A read operation always starts at byte 0.
 * Payload is deleted from FIFO after it is read. Used in RX mode.
 */
static uint8_t nrf24CmdGetRxPl(NRF24Driver *drvp, uint8_t *rxbuf, uint8_t len) {
    return nrf24RegRead(drvp, NRF24_CMD_RD_RX_PLOAD, rxbuf, len);
}

/**
 * W_TX_PAYLOAD
 * Write TX-payload: 1 – 32 bytes. A write operation always starts at byte 0 used in TX payload.
 */
static uint8_t nrf24CmdSetTxPl(NRF24Driver *drvp, uint8_t *txbuf, uint8_t len) {
    return nrf24RegWrite(drvp, NRF24_CMD_WR_TX_PLOAD, txbuf, len);
}

/**
 * FLUSH_TX
 * Flush TX FIFO, used in TX mode
 */
static uint8_t nrf24CmdFlushTx(NRF24Driver *drvp) {
    uint8_t txbuf[1] = {
        NRF24_CMD_FLUSH_TX
    };
    uint8_t rxbuf[1];

    return nrf24Exchange(drvp, txbuf, rxbuf, 1);
}

/**
 * FLUSH_RX
 * Flush RX FIFO, used in RX mode
 * Should not be executed during transmission of acknowledge, that is,
 * acknowledge package will not be completed.
 */
static uint8_t nrf24CmdFlushRx(NRF24Driver *drvp) {
    uint8_t txbuf[1] = {
        NRF24_CMD_FLUSH_RX
    };
    uint8_t rxbuf[1];

    return nrf24Exchange(drvp, txbuf, rxbuf, 1);
}

/**
 * REUSE_TX_PL
 * Used for a PTX device
 * Reuse last transmitted payload. Packets are repeatedly retransmitted as long as CE is high.
 * TX payload reuse is active until W_TX_PAYLOAD or FLUSH TX is executed. TX payload reuse must
 * not be activated or deactivated during package transmission
 */
static uint8_t __attribute__ ((unused)) nrf24CmdReuseTxPl(NRF24Driver *drvp) {
    uint8_t txbuf[1] = {
        NRF24_CMD_REUSE_TX_PL
    };
    uint8_t rxbuf[1];

    return nrf24Exchange(drvp, txbuf, rxbuf, 1);
}

/**
 * ACTIVATE
 * This write command followed by data 0x73 acti- vates the following features:
 *  - R_RX_PL_WID
 *  - W_ACK_PAYLOAD
 *  - W_TX_PAYLOAD_NOACK
 * A new ACTIVATE command with the same data deactivates them again. This is executable
 * in power down or stand by modes only.
 * The R_RX_PL_WID, W_ACK_PAYLOAD, and W_TX_PAYLOAD_NOACK features registers are initially in
 * a deactivated state; a write has no effect, a read only results in zeros on MISO. To
 * activate these registers, use the ACTIVATE command followed by data 0x73. Then they can be
 * accessed as any other register in nRF24L01. Use the same command and data to deactivate the
 * registers again.
 */
static uint8_t __attribute__ ((unused)) nrf24CmdActivate(NRF24Driver *drvp) {
    uint8_t txbuf[2] = {
        NRF24_CMD_ACTIVATE, 0x73
    };
    uint8_t rxbuf[2];

    return nrf24Exchange(drvp, txbuf, rxbuf, 1);
}

/**
 * R_RX_PL_WID
 * Read RX-payload width for the top R_RX_PAYLOAD in the RX FIFO.
 */
static uint8_t nrf24CmdReadRxPlWid(NRF24Driver *drvp, uint8_t *len) {
    uint8_t txbuf[2] = {
        NRF24_CMD_RD_RX_PLOAD_WID, 0xff
    };
    uint8_t rxbuf[2];

    nrf24Exchange(drvp, txbuf, rxbuf, 2);
    *len = rxbuf[1];
    return rxbuf[0];
}

/**
 * W_ACK_PAYLOAD
 * Used in RX mode.
 * Write Payload to be transmitted together with ACK packet on PIPE PPP. (PPP valid in the
 * range from 000 to 101). Maximum three ACK packet payloads can be pending. Payloads with
 * same PPP are handled using first in - first out principle.
 * Write payload: 1– 32 bytes. A write operation always starts at byte 0.
 */
static uint8_t __attribute__ ((unused)) nrf24CmdSetAckPl(NRF24Driver *drvp, uint8_t idx, uint8_t *val, uint8_t len) {
    return nrf24RegWrite(drvp, NRF24_CMD_WR_ACK_PLOAD | idx, val, len);
}

/**
 * W_TX_PAYLOAD_NOACK
 * Used in TX mode. Disables AUTOACK on this specific packet.
 */
static uint8_t __attribute__ ((unused)) nrf24CmdSetTxPlNoAck(NRF24Driver *drvp, uint8_t *val, uint8_t len) {
    return nrf24RegWrite(drvp, NRF24_CMD_WR_NACK_TX_PLOAD, val, len);
}

/**
 * NOP
 * No Operation. Might be used to read the STATUS register
 */
static uint8_t nrf24CmdGetStatus(NRF24Driver *drvp) {
    uint8_t txbuf[1] = {
        NRF24_CMD_NOP
    };
    uint8_t rxbuf[1];

    return nrf24Exchange(drvp, txbuf, rxbuf, 1);
}

// ----- Command end -------

/*===========================================================================*/
/* Driver height level functions.                                            */
/*===========================================================================*/

uint8_t txIsEmpty(NRF24Driver *drvp) {
    uint8_t status;
    nrf24CmdRdOb(drvp, NRF24_FIFO_STATUS, &status);
    return !(status & NRF24_TX_FULL);
}

uint8_t rxIsEmpty(NRF24Driver *drvp) {
    uint8_t status;
    nrf24CmdRdOb(drvp, NRF24_FIFO_STATUS, &status);
    return status & NRF24_FIFO_RX_EMPTY;
}

static msg_t rcvPktOp(NRF24Driver *drvp, systime_t time, uint8_t *rxbuf, uint8_t len) {
    uint8_t status, pllen;

    nrf24CmdWrOb(drvp, NRF24_CONFIG, NRF24_EN_CRC | NRF24_CRCO | NRF24_PWR_UP | NRF24_PRIM_RX);

    nrf24CmdFlushRx(drvp);

    // Clear interrupt flags
    nrf24CmdWrOb(drvp, NRF24_STATUS, NRF24_RX_DR | NRF24_TX_DS | NRF24_MAX_RT);

    // Prepare to receive packet
    palSetPad(drvp->config->ceport, drvp->config->cepad);

    if (chEvtWaitOneTimeout(ALL_EVENTS, time) == 0) {
        palClearPad(drvp->config->ceport, drvp->config->cepad);
        return MSG_TIMEOUT;
    }

    palClearPad(drvp->config->ceport, drvp->config->cepad);

    status = nrf24CmdGetStatus(drvp);
    if (!(status & (NRF24_RX_DR | NRF24_TX_DS)) || rxIsEmpty(drvp)) {
        return MSG_RESET;
    }

    nrf24CmdReadRxPlWid(drvp, &pllen);
    if (pllen > NRF24L01_MAX_PL_LEN || pllen > len) return MSG_RESET;

    nrf24CmdGetRxPl(drvp, rxbuf, pllen);

    return MSG_OK;
}

static msg_t sndPktOp(NRF24Driver *drvp, uint8_t *txbuf, uint8_t len) {
    uint8_t status;

    nrf24CmdWrOb(drvp, NRF24_CONFIG, NRF24_EN_CRC | NRF24_CRCO | NRF24_PWR_UP);

    nrf24CmdFlushTx(drvp);

    // Clear interrupt flags
    nrf24CmdWrOb(drvp, NRF24_STATUS, NRF24_RX_DR | NRF24_TX_DS | NRF24_MAX_RT);

    nrf24CmdSetTxPl(drvp, txbuf, len);

    // From https://www.nordicsemi.com/eng/nordic/content_download/2726/34069/file/nRF24L01P_Product_Specification_1_0.pdf
    // P23: 6.1.5 TX mode:
    //  - The TX mode is an active mode for transmitting packets. ... a high pulse on the CE for more than 10μs.
    //  - It is important never to keep the nRF24L01+ in TX mode for more than 4ms at a time.
    //  - If the Enhanced ShockBurst(TM) features are enabled, nRF24L01+ is never in TX mode longer than 4ms.
#if NRF24L01_WITH_PA
    palSetPad(drvp->config->ceport, drvp->config->cepad);

    if (chEvtWaitOneTimeout(ALL_EVENTS, TIME_INFINITE) == 0) {
        return MSG_TIMEOUT;
    }

    palClearPad(drvp->config->ceport, drvp->config->cepad);
#else
    {
        // 0x100 -> 70μs, 0x40 -> 19μs
        volatile uint16_t counter = 0x24;

        chSysLock();
        palSetPad(drvp->config->ceport, drvp->config->cepad);
        while (counter--) __NOP(); // At least 10us
        palClearPad(drvp->config->ceport, drvp->config->cepad);
        chSysUnlock();
    }

    if (chEvtWaitOneTimeout(ALL_EVENTS, TIME_INFINITE) == 0) {
        return MSG_TIMEOUT;
    }
#endif

    status = nrf24CmdGetStatus(drvp);
    if ((status & NRF24_MAX_RT) != 0) {
        return MSG_RESET;
    }

    return MSG_OK;
}

int16_t rcvPkt(void *ip, systime_t rxtime, uint8_t *rxbuf, uint16_t len) {
    uint8_t buf[NRF24L01_MAX_PL_LEN], id, bl;
    uint16_t size, count;

    chDbgAssert((((NRF24Driver *)ip)->state == NRF24_STATE_READY), "rcvPkt(), invalid state");

    for (count = 0, size = len; count < size; count += bl) {
        msg_t msg = rcvPktOp((NRF24Driver *)ip, rxtime, buf, sizeof(buf));
        if (msg != MSG_OK) return -1;
        id                = buf[0] & 0xE0;
        bl                = buf[0] & 0x1F;
        switch (id) {
        case NRF24L01_PKT_BEG:
            size = buf[1] << 8 | buf[2];
            if (count != 0 || size > len) return -2;
            memcpy(rxbuf, buf + 3, bl);
            break;
        case NRF24L01_PKT_MID:
            if (count == 0 || bl != NRF24L01_MAX_PL_LEN - 1) return -3;
            memcpy(rxbuf + count, buf + 1, bl);
            break;
        case NRF24L01_PKT_END:
            if (count == 0 || count + bl != size) return -4;
            memcpy(rxbuf + count, buf + 1, bl);
            break;
        default:
            return -5;
        }

        buf[0] = NRF24L01_PKT_ACK | 0x00;
        if (sndPktOp((NRF24Driver *)ip, buf, 1) != MSG_OK) return -6;
    }

    return size;
}

int16_t sndPkt(void *ip, systime_t rxtime, uint8_t *txbuf, uint16_t len) {
    uint8_t buf[NRF24L01_MAX_PL_LEN], id, bl;
    uint16_t count;

    chDbgAssert((((NRF24Driver *)ip)->state == NRF24_STATE_READY), "sndPkt(), invalid state");

    for (count = 0; count < len; count += bl) {
        if (count == 0) {
            bl = MIN(len, NRF24L01_MAX_PL_LEN - 3);
            buf[0] = NRF24L01_PKT_BEG | bl;
            buf[1] = (len >> 8) & 0xFF;
            buf[2] = (len >> 0) & 0xFF;

            memcpy(buf + 3, txbuf, bl);

            if (sndPktOp((NRF24Driver *)ip, buf, bl + 3) != MSG_OK) {
                return -1;
            }
        } else {
            bl = MIN(len - count, NRF24L01_MAX_PL_LEN - 1);
            if (bl == NRF24L01_MAX_PL_LEN - 1 && bl != len - count) {
                id = NRF24L01_PKT_MID;
            } else {
                id = NRF24L01_PKT_END;
            }
            buf[0] = id | bl;
            memcpy(buf + 1, txbuf + count, bl);
            if (sndPktOp((NRF24Driver *)ip, buf, bl + 1) != MSG_OK) {
                return -2;
            }
        }

        if (rcvPktOp((NRF24Driver *)ip, rxtime, buf, sizeof(buf)) != MSG_OK) {
            return -3;
        }
        if (buf[0] != (NRF24L01_PKT_ACK | 0x00)) return -4;
    }

    return count;
}

static const struct NRF24VMT vmt_nrf24 = {
    rcvPkt, sndPkt
};

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

void nrf24ObjectInit(NRF24Driver *devp, const uint8_t *address) {
    devp->vmt = &vmt_nrf24;
    devp->state = NRF24_STATE_STOP;
    devp->config = NULL;

    chEvtObjectInit(&devp->irq_event);
    chMtxObjectInit(&devp->mutex);

    memcpy(devp->txaddr, address, sizeof(devp->txaddr));
    memcpy(devp->rxaddr, address, sizeof(devp->rxaddr));
}

void nrf24Start(NRF24Driver *devp, const NRF24Config *config) {
    chDbgAssert((devp->state == NRF24_STATE_STOP) || (devp->state == NRF24_STATE_READY),
                "nrf24Start(), invalid state");

    devp->config = config;

    palClearPad(devp->config->ceport, devp->config->cepad);

    chEvtRegister(&devp->irq_event, &devp->el, 0);
    spiStart(devp->config->spip, devp->config->spicfg);

    nrf24CmdWrOb(devp, NRF24_STATUS, NRF24_RX_DR | NRF24_TX_DS | NRF24_MAX_RT);
    // Power up
    nrf24CmdWrOb(devp, NRF24_CONFIG, NRF24_PWR_UP | NRF24_EN_CRC);

    // Wait 1.5ms to enter standby mode
    chThdSleepMilliseconds(2);

    // Enable auto ackknowledgement in all data pipes
    nrf24CmdWrOb(devp, NRF24_EN_AA, NRF24_ENAA_P0 | NRF24_ENAA_P1 |
            NRF24_ENAA_P2 | NRF24_ENAA_P3 | NRF24_ENAA_P4 | NRF24_ENAA_P5);
    // Enable data pipes
    nrf24CmdWrOb(devp, NRF24_EN_RXADDR, NRF24_ERX_P0 | NRF24_ERX_P1 |
            NRF24_ERX_P2 | NRF24_ERX_P3 | NRF24_ERX_P4 | NRF24_ERX_P5);

    // Sets the frequency channel
    nrf24CmdWrOb(devp, NRF24_RF_CH, 0x50);

    // The ARD time must never be shorter than the sum of the startup time and the time on-air for the ACK packet:
    // - For 2Mbps data rate and 5 byte address; 15 byte is maximum ACK packet payload length for ARD=250μs (reset value).
    // - For 1Mbps data rate and 5 byte address; 5 byte is maximum ACK packet payload length for ARD=250μs (reset value).
    // - ARD=500μs is long enough for any ACK payload length in 1 or 2Mbps mode.
    nrf24CmdWrOb(devp, NRF24_SETUP_RETR, NRF24_ARD_500US | NRF24_ARC_3RT);

    // Setup of address widths
    switch (devp->config->addrlen) {
        case 3:
            nrf24CmdWrOb(devp, NRF24_SETUP_AW, NRF24_AW_3BYTES);
            break;
        case 4:
            nrf24CmdWrOb(devp, NRF24_SETUP_AW, NRF24_AW_4BYTES);
            break;
        default:
        case 5:
            nrf24CmdWrOb(devp, NRF24_SETUP_AW, NRF24_AW_5BYTES);
            break;
    }

    // Data rate: 2Mbps, PWR 0dBm, LNA gain hcurr
    nrf24CmdWrOb(devp, NRF24_RF_SETUP, NRF24_RF_DR_HIGH | NRF24_LNA_HCURR | NRF24_RF_PWR_0DBM);

    // Setup features: enable dynamic payload length
    nrf24CmdWrOb(devp, NRF24_FEATURE, NRF24_EN_DPL);
    // Dynamic payload length in all data pipes
    nrf24CmdWrOb(devp, NRF24_DYNPD, NRF24_DPL_P0 | NRF24_DPL_P1 |
            NRF24_DPL_P2 | NRF24_DPL_P3 | NRF24_DPL_P4 | NRF24_DPL_P5);

#if 0
    nrf24CmdActivate(devp); // NRF24L01P doesn't have this command
#endif

    // Setup Tx and Rx address
    nrf24CmdWr(devp, NRF24_TX_ADDR, (uint8_t *)devp->txaddr, devp->config->addrlen);
    nrf24CmdWr(devp, NRF24_RX_ADDR_P0, (uint8_t *)devp->rxaddr, devp->config->addrlen);

    devp->state = NRF24_STATE_READY;
}

void nrf24Stop(NRF24Driver *devp) {
    chDbgAssert((devp->state == NRF24_STATE_STOP) || (devp->state == NRF24_STATE_READY),
                "nrf24Stop(), invalid state");

    if (devp->state == NRF24_STATE_READY) {
        chEvtUnregister(&devp->irq_event, &devp->el);

        // Power down
        nrf24CmdWrOb(devp, NRF24_CONFIG, 0);

        spiStop(devp->config->spip);
    }

    devp->state = NRF24_STATE_STOP;
}
