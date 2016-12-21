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

#include "ch.h"
#include "hal.h"

#include "chprintf.h"

#include "pcf8574.h"
#include "lcdiic.h"
#include "nrf24l01.h"


#define NRF24L01_TARGET_ADDRESS    ((const uint8_t *)"NRF24")


static struct {
  uint8_t ptx; /* ptx or prx */
  uint8_t ready;
  uint32_t tx;
  uint32_t rx;
  uint32_t txerr;
  uint32_t rxerr;
} TrCtx;

static mutex_t logMtx;
int broprintf(const char *fmt, ...) {
  va_list ap;
  int formatted_bytes;

  chMtxLock(&logMtx);
  va_start(ap, fmt);
  formatted_bytes = chvprintf((BaseSequentialStream *)&SD1, fmt, ap);
  va_end(ap);
  chMtxUnlock(&logMtx);

  return formatted_bytes;
}

static void keyExtCallback(EXTDriver *extp, expchannel_t channel);
static void nrf24ExtCallback(EXTDriver *extp, expchannel_t channel);

static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA,
      keyExtCallback},    /* 0 */
    {EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOB,
      nrf24ExtCallback},    /* IRQ line connected to PB1 */
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},    /* 5 */
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},    /* 10 */
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},    /* 15 */
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}
  }
};

/*===========================================================================*/
/* NRF24L01                                                                  */
/*===========================================================================*/

#define  GPIOA_RF_CE                       1
#define  GPIOB_RF_IRQ                      1
#define  GPIOA_RF_SPID1_CS                 4
#define  GPIOA_RF_SPID1_SCK                5
#define  GPIOA_RF_SPID1_MISO               6
#define  GPIOA_RF_SPID1_MOSI               7


static const SPIConfig spicfg = {
  NULL,
  GPIOA,                                     /*   port of CS  */
  GPIOA_RF_SPID1_CS,                         /*   pin of CS   */
  SPI_CR1_BR_1,                              /*   CR1 register: 6MHz */
  SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0 /*   CR2 register: 8-bit */
};

static const NRF24Config nrf24cfg = {
  GPIOA, GPIOA_RF_CE,
  GPIOB, GPIOB_RF_IRQ,
  &SPID1, &spicfg,
  5, /* address length: 3 ~ 5 */
};

static NRF24Driver NRF24D1;

static void nrf24ExtCallback(EXTDriver *extp, expchannel_t channel) {
  (void)extp;

  chSysLockFromISR();
  if (channel == NRF24D1.config->irqpad) {
    chEvtBroadcastFlagsI(&NRF24D1.irq_event, NRF24L01_GENERIC_IRQ);
  }
  chSysUnlockFromISR();
}

static THD_WORKING_AREA(waNrf24, 512);
static __attribute__((noreturn)) THD_FUNCTION(Nrf24, arg) {
  (void)arg;

  chRegSetThreadName("Nrf24");

  nrf24ObjectInit(&NRF24D1, NRF24L01_TARGET_ADDRESS);
  nrf24Start(&NRF24D1, &nrf24cfg);

  while (!TrCtx.ready) { chThdSleepMilliseconds(100); }

  while (TRUE) {
    int16_t len;
    uint8_t buf[] = "Hello, world! - https://www.brobwind.com";

    if (TrCtx.ptx) {
      len = nrf24SndPkt(&NRF24D1, MS2ST(500), buf, sizeof(buf));
      if (len > 0) {
        broprintf("=> Message sent: %d - %lu\r\n", len, chVTGetSystemTime());
        TrCtx.tx += len;
      } else  {
        broprintf("Message not sent: ret=%d - %lu\r\n", len, chVTGetSystemTime());
        TrCtx.txerr++;
        continue;
      }

      len = nrf24RcvPkt(&NRF24D1, MS2ST(500), buf, sizeof(buf));
      if (len > 0) {
        broprintf("=> Message received: len=%d, buf=%s - %lu\r\n", len, buf, chVTGetSystemTime());
        TrCtx.rx += len;
      } else {
        broprintf("Message not received: ret=%d - %lu\r\n", len, chVTGetSystemTime());
        TrCtx.rxerr++;
      }
    } else {
      len = nrf24RcvPkt(&NRF24D1, TIME_INFINITE, buf, sizeof(buf));
      if (len > 0) {
        broprintf("=> Message received: len=%d, buf=%s - %lu\r\n", len, buf, chVTGetSystemTime());
        TrCtx.rx += len;
      } else {
        broprintf("Message not received: ret=%d - %lu\r\n", len, chVTGetSystemTime());
        TrCtx.rxerr++;
        continue;
      }

      strcpy((char *)buf, "OKAY");
      len = nrf24SndPkt(&NRF24D1, MS2ST(500), buf, strlen((char *)buf) + 1);
      if (len > 0) {
        broprintf("=> Message sent: len=%d, buf=%s - %lu\r\n", len, buf, chVTGetSystemTime());
        TrCtx.tx += len;
      } else  {
        broprintf("Message not sent: ret=%d - %lu\r\n", len, chVTGetSystemTime());
        TrCtx.txerr++;
      }
    }
  }

  nrf24Stop(&NRF24D1);
}

/*===========================================================================*/
/* LCD display                                                               */
/*===========================================================================*/

static const I2CConfig i2ccfg = { // I2CCLK=48MHz, SCL=~100kHz
  STM32_TIMINGR_PRESC(0x0B)  |
  STM32_TIMINGR_SCLDEL(0x04) | STM32_TIMINGR_SDADEL(0x02) |
  STM32_TIMINGR_SCLH(0x0F)   | STM32_TIMINGR_SCLL(0x13),
  0,
  0
};

/* Primary LCD display configuration */
static const PCF8574Config pcf8574cfg = {
  &I2CD1,
  &i2ccfg,
  PCF8574A_SAD_0X3F,
  0x00,
  0x00,
};

static PCF8574Driver PCF8574D1;

static const LCDIICConfig lcdiiccfg = {
  &PCF8574D1,
  &pcf8574cfg,
};

static LCDIICDriver LCDIICD1;

/* Delay function, used in control LCD display */
static void delayUs(uint32_t val) {
  (void)val;
}

static void delayMs(uint32_t val) {
  chThdSleepMilliseconds(val);
}

/* Primary LCD display thread */
static THD_WORKING_AREA(waLcdDisplay, 256);
static __attribute__((noreturn)) THD_FUNCTION(LcdDisplay, arg) {
  (void)arg;
  int8_t idx;
  char buf[32];

  chRegSetThreadName("LcdDisplay");

  pcf8574ObjectInit(&PCF8574D1);
  pcf8574Start(&PCF8574D1, &pcf8574cfg);

  lcdiicObjectInit(&LCDIICD1, &delayUs, &delayMs);
  lcdiicStart(&LCDIICD1, &lcdiiccfg);

  {
    const uint8_t SYMBOL[][8] = {
      { 0x04, 0x0e, 0x0e, 0x0e, 0x1f, 0x00, 0x04, 0x00 }, // bell
      { 0x02, 0x03, 0x02, 0x0e, 0x1e, 0x0c, 0x00, 0x00 }, // note
      { 0x00, 0x0e, 0x15, 0x17, 0x11, 0x0e, 0x00, 0x00 }, // clock
      { 0x00, 0x0a, 0x1f, 0x1f, 0x0e, 0x04, 0x00, 0x00 }, // heart
      { 0x00, 0x0c, 0x1d, 0x0f, 0x0f, 0x06, 0x00, 0x00 }, // duck
      { 0x00, 0x01, 0x03, 0x16, 0x1c, 0x08, 0x00, 0x00 }, // check
      { 0x00, 0x1b, 0x0e, 0x04, 0x0e, 0x1b, 0x00, 0x00 }, // cross
      { 0x01, 0x01, 0x05, 0x09, 0x1f, 0x08, 0x04, 0x00 }, // retarrow
    };

    for (idx = 0; (uint8_t)idx < sizeof(SYMBOL) / sizeof(SYMBOL[0]); idx++) {
      lcdiicUpdatePattern(&LCDIICD1, idx, SYMBOL[idx]);
    }

    for (idx = 0; (uint8_t)idx < sizeof(SYMBOL) / sizeof(SYMBOL[0][0]); idx++) {
      uint8_t tmp;
      lcdiicReadData(&LCDIICD1, 0, idx, &tmp);

      if (idx % 8 == 0) {
        broprintf("---- dump pattern: %d ----\r\n", idx / 8);
      }

      broprintf("%02x%s", tmp, idx % 8 == 7 ? "\r\n" : " ");
    }
  }

  for (idx = 30; idx >= 0; idx--) {
    chsnprintf(buf, sizeof(buf), "Select mode:%4d", idx * 100);
    lcdiicDrawText(&LCDIICD1, 0, 0, buf, strlen(buf));

    if (idx == 1) extChannelDisable(&EXTD1, 0);

    if (TrCtx.ptx) {
      chsnprintf(buf, sizeof(buf), "> Transmitter");
    } else {
      chsnprintf(buf, sizeof(buf), "> Receiver   ");
    }
    lcdiicDrawText(&LCDIICD1, 1, 0, buf, strlen(buf));

    chThdSleepMilliseconds(100);
  }

  lcdiicClearScreen(&LCDIICD1);

  chsnprintf(buf, sizeof(buf), "Mode selected:", idx * 100);
  lcdiicDrawText(&LCDIICD1, 0, 0, buf, strlen(buf));
  if (TrCtx.ptx) {
    chsnprintf(buf, sizeof(buf), "> Transmitter");
  } else {
    chsnprintf(buf, sizeof(buf), "> Receiver");
  }
  lcdiicDrawText(&LCDIICD1, 1, 0, buf, strlen(buf));

  chThdSleepMilliseconds(2000);
  TrCtx.ready = 1;

  lcdiicClearScreen(&LCDIICD1);

#define REPEAT_TOTAL                  10
  while (TRUE) {
    systime_t systicks, seconds;

    for (idx = 0; idx < REPEAT_TOTAL * 3; idx++) {
      if (idx < REPEAT_TOTAL * 1) {
        /* First line: NRF24L01 mode */
        if (TrCtx.ptx) {
          chsnprintf(buf, sizeof(buf), "> Transmitter   ");
        } else {
          chsnprintf(buf, sizeof(buf), "> Receiver      ");
        }
        lcdiicDrawText(&LCDIICD1, 0, 0, buf, strlen(buf));

        /* Second line: system time */
        systicks = chVTGetSystemTime();
        seconds = ST2S(systicks); /* ST2MS() may overflow */
#if CH_CFG_ST_FREQUENCY == 1000
        chsnprintf(buf, sizeof(buf), "%6d:%02d:%02d.%03d", seconds / 3600, (seconds / 60) % 60,
            seconds % 60, systicks % 1000);
#else
        chsnprintf(buf, sizeof(buf), "%6d:%02d:%02d", seconds / 3600, (seconds / 60) % 60, seconds % 60);
#endif
        lcdiicDrawText(&LCDIICD1, 1, 0, buf, strlen(buf));
      } else if (idx < REPEAT_TOTAL * 2) {
        /* First line: transmit bytes count */
        chsnprintf(buf, sizeof(buf), "Tx: %-12d", TrCtx.tx);
        lcdiicDrawText(&LCDIICD1, 0, 0, buf, strlen(buf));
        /* Second line: transmit retry count */
        chsnprintf(buf, sizeof(buf), "TxErr: %-9d", TrCtx.txerr);
        lcdiicDrawText(&LCDIICD1, 1, 0, buf, strlen(buf));
      } else {
        /* First line: receive bytes count */
        chsnprintf(buf, sizeof(buf), "Rx: %-12d", TrCtx.rx);
        lcdiicDrawText(&LCDIICD1, 0, 0, buf, strlen(buf));
        /* Second line: receive retry count */
        chsnprintf(buf, sizeof(buf), "RxErr: %-9d", TrCtx.rxerr);
        lcdiicDrawText(&LCDIICD1, 1, 0, buf, strlen(buf));
      }
      chThdSleepMilliseconds(200);
    }
  }

  lcdiicStop(&LCDIICD1);
  pcf8574Stop(&PCF8574D1);
}

/*===========================================================================*/
/* LED blinker                                                               */
/*===========================================================================*/

static THD_WORKING_AREA(waLedBlinker, 24);
static THD_FUNCTION(LedBlinker, arg) {
  (void)arg;
  chRegSetThreadName("LedBlinker");

  while (TRUE) {
  /* LED on */
  if (chMtxTryLock(&NRF24D1.mutex)) {
    palClearPad(GPIOA, GPIOA_LED_GREEN);
    chMtxUnlock(&NRF24D1.mutex);
  }
  chThdSleepMilliseconds(1);

  /* LED off */
  if (chMtxTryLock(&NRF24D1.mutex)) {
    palSetPad(GPIOA, GPIOA_LED_GREEN);
    chMtxUnlock(&NRF24D1.mutex);
  }
  chThdSleepMilliseconds(999);
  }
}

/*===========================================================================*/
/* User key                                                                  */
/*===========================================================================*/

static void keyExtCallback(EXTDriver *extp, expchannel_t channel) {
  (void)extp, (void)channel;
  if (!TrCtx.ready) {
    TrCtx.ptx = !TrCtx.ptx;
  }
}

/*===========================================================================*/
/* Main                                                                      */
/*===========================================================================*/

/*
 * Application entry point.
 */
int __attribute__((noreturn)) main(void)
{
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * SPID1 I/O pins setup.(It bypasses board.h configurations)
   */
  palSetPadMode(GPIOA, GPIOA_RF_SPID1_SCK,
                 PAL_MODE_ALTERNATE(0) | PAL_STM32_OSPEED_HIGHEST);   /* SPID1 SCK */
  palSetPadMode(GPIOA, GPIOA_RF_SPID1_MISO,
                 PAL_MODE_ALTERNATE(0) | PAL_STM32_OSPEED_HIGHEST);   /* SPID1 MISO*/
  palSetPadMode(GPIOA, GPIOA_RF_SPID1_MOSI,
                 PAL_MODE_ALTERNATE(0) | PAL_STM32_OSPEED_HIGHEST);   /* SPID1 MOSI*/
  palSetPadMode(GPIOA, GPIOA_RF_SPID1_CS,
                 PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);/* SPID1 CS  */
  /*
   * CE and IRQ pins setup.
   */

  palSetPadMode(GPIOA, GPIOA_RF_CE,
                 PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);/* SPID1 CE  */
  palSetPadMode(GPIOB, GPIOB_RF_IRQ,
                 PAL_MODE_INPUT | PAL_STM32_OSPEED_HIGHEST);          /* SPID1 IRQ  */

  /*
   * I2CD1 I/O pins setup.(It bypasses board.h configurations)
   */
  palSetPadMode(GPIOA, GPIOA_PIN9, PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGHEST);   /* SCL */
  palSetPadMode(GPIOA, GPIOA_PIN10, PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGHEST);  /* SDA */

  chMtxObjectInit(&logMtx);

  /*
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);

  /*
   * Activates the EXT driver 1.
   */
  extStart(&EXTD1, &extcfg);

  /*
   * Creates the LED blinker thread.
   */
  chThdCreateStatic(waLedBlinker, sizeof(waLedBlinker), NORMALPRIO, LedBlinker, NULL);

  /*
   * Creates the NRF24L01 thread.
   */
  chThdCreateStatic(waNrf24, sizeof(waNrf24), NORMALPRIO + 1, Nrf24, NULL);

  /*
   * Creates the LCD display thread.
   */
  chThdCreateStatic(waLcdDisplay, sizeof(waLcdDisplay), NORMALPRIO, LcdDisplay, NULL);

  while(TRUE){
    chThdSleepMilliseconds(500);
  }

  extStop(&EXTD1);
}
