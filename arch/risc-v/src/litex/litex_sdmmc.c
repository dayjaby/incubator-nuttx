/****************************************************************************
 * arch/arm/src/litexf7/litex_sdmmc.c
 *
 *   Copyright (C) 2009, 2011-2018,2019 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david.sidrane @nscdg.com>
 *            Bob Feretich <bob.feretich@rafresearch.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/mmcsd.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "litex_sdmmc.h"

#ifndef SDCARD_CLK_FREQ_INIT
#define SDCARD_CLK_FREQ_INIT 400000
#endif

#define CLKGEN_STATUS_BUSY		0x1
#define CLKGEN_STATUS_PROGDONE	0x2
#define CLKGEN_STATUS_LOCKED	0x4

#define SD_CMD_RESPONSE_SIZE 16

#define SD_OK         0
#define SD_CRCERROR   1
#define SD_TIMEOUT    2
#define SD_WRITEERROR 3

#define SD_SWITCH_CHECK  0
#define SD_SWITCH_SWITCH 1

#define SD_SPEED_SDR12  0
#define SD_SPEED_SDR25  1
#define SD_SPEED_SDR50  2
#define SD_SPEED_SDR104 3
#define SD_SPEED_DDR50  4

#define SD_DRIVER_STRENGTH_B 0
#define SD_DRIVER_STRENGTH_A 1
#define SD_DRIVER_STRENGTH_C 2
#define SD_DRIVER_STRENGTH_D 3

#define SD_GROUP_ACCESSMODE     0
#define SD_GROUP_COMMANDSYSTEM  1
#define SD_GROUP_DRIVERSTRENGTH 2
#define SD_GROUP_POWERLIMIT     3

#define SDCARD_STREAM_STATUS_OK           0b000
#define SDCARD_STREAM_STATUS_TIMEOUT      0b001
#define SDCARD_STREAM_STATUS_DATAACCEPTED 0b010
#define SDCARD_STREAM_STATUS_CRCERROR     0b101
#define SDCARD_STREAM_STATUS_WRITEERROR   0b110

#define SDCARD_CTRL_DATA_TRANSFER_NONE  0
#define SDCARD_CTRL_DATA_TRANSFER_READ  1
#define SDCARD_CTRL_DATA_TRANSFER_WRITE 2

#define SDCARD_CTRL_RESPONSE_NONE  0
#define SDCARD_CTRL_RESPONSE_SHORT 1
#define SDCARD_CTRL_RESPONSE_LONG  2

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure defines the state of the LiteX SDMMC interface */

struct litex_dev_s
{
  struct sdio_dev_s  dev;             /* Standard, base SDIO interface */

  /* LiteX-specific extensions */

  uint32_t          phy_base;
  uint32_t          base;
  int               nirq;
#ifdef CONFIG_LITEX_SDMMC_DMA
  uint32_t          dmapri;
#endif

  /* Event support */

  sem_t              waitsem;         /* Implements event waiting */
  sdio_eventset_t    waitevents;      /* Set of events to be waited for */
  uint32_t           waitmask;        /* Interrupt enables for event waiting */
  volatile sdio_eventset_t wkupevent; /* The event that caused the wakeup */
  struct wdog_s      waitwdog;        /* Watchdog that handles event timeouts */

  /* Callback support */

  sdio_statset_t     cdstatus;        /* Card status */
  sdio_eventset_t    cbevents;        /* Set of events to be cause callbacks */
  worker_t           callback;        /* Registered callback function */
  void              *cbarg;           /* Registered callback argument */
  struct work_s      cbwork;          /* Callback work queue structure */

  /* Interrupt mode data transfer support */

  uint32_t          *buffer;          /* Address of current R/W buffer */
  size_t             remaining;       /* Number of bytes remaining in the transfer */
  uint32_t           xfrmask;         /* Interrupt enables for data transfer */

  /* DMA data transfer support */

  bool               widebus;         /* Required for DMA support */
  bool               onebit;          /* true: Only 1-bit transfers are supported */
#ifdef CONFIG_LITEX_SDMMC_DMA
  volatile uint8_t   xfrflags;        /* Used to synchronize SDMMC and DMA completion events */
  bool               dmamode;         /* true: DMA mode transfer */
  DMA_HANDLE         dma;             /* Handle for DMA channel */
#endif

  #ifdef HAVE_SDMMC_SDIO_MODE
  /* Interrupt at SDIO_D1 pin, only for SDIO cards */

  uint32_t           sdiointmask;            /* LITEX SDIO register mask */
  int               (*do_sdio_card)(void *); /* SDIO card ISR */
  void               *do_sdio_arg;           /* arg for SDIO card ISR */
  bool               sdiomode;               /* True: in SDIO mode */
#endif

  /* Misc */

  uint32_t           blocksize;       /* Current block size */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers ********************************************************/

/* round up to closest power-of-two */
static inline uint32_t pow2_round_up(uint32_t r);

static inline void sdcard_set_clk_freq(uint32_t clk_freq);

static int sdcard_wait_cmd_done(void);

static int sdcard_wait_data_done(void);

static int sdcard_send_ext_csd(void);

static int sdcard_app_cmd(uint16_t rca);

static int sdcard_app_send_op_cond(int hcs);

static int sdcard_all_send_cid(void);

static int sdcard_set_relative_address(void);

static int sdcard_send_cid(uint16_t rca);

static int sdcard_send_csd(uint16_t rca);

static int sdcard_select_card(uint16_t rca);

static int sdcard_app_set_bus_width(void);

static int sdcard_switch(unsigned int mode, unsigned int group, unsigned int value);

static int sdcard_app_send_scr(void);

static int sdcard_app_set_blocklen(unsigned int blocklen);

static int sdcard_write_single_block(unsigned int blockaddr);

static int sdcard_write_multiple_block(unsigned int blockaddr, unsigned int blockcnt);

static int sdcard_read_single_block(unsigned int blockaddr);

static int sdcard_read_multiple_block(unsigned int blockaddr, unsigned int blockcnt);

static int sdcard_stop_transmission(void);

static int sdcard_send_status(uint16_t rca);

static int sdcard_set_block_count(unsigned int blockcnt);

static uint16_t sdcard_decode_rca(void);

static void sdcard_decode_cid(void);

static void sdcard_decode_csd(void);

static inline int sdcard_send_command(uint32_t arg, uint8_t cmd, uint8_t rsp);

static int sdcard_go_idle(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_LITEX_SDMMC1
struct litex_dev_s g_sdmmcdev1 =
{
  .dev =
  {
#ifdef CONFIG_SDIO_MUXBUS
    .lock             = litex_lock,
#endif
    .reset            = litex_reset,
    .capabilities     = litex_capabilities,
    .status           = litex_status,
    .widebus          = litex_widebus,
    .clock            = litex_clock,
    .attach           = litex_attach,
    .sendcmd          = litex_sendcmd,
    .blocksetup       = litex_blocksetup,
    .recvsetup        = litex_recvsetup,
    .sendsetup        = litex_sendsetup,
    .cancel           = litex_cancel,
    .waitresponse     = litex_waitresponse,
    .recv_r1          = litex_recvshortcrc,
    .recv_r2          = litex_recvlong,
    .recv_r3          = litex_recvshort,
    .recv_r4          = litex_recvshort,
    .recv_r5          = litex_recvshortcrc,
    .recv_r6          = litex_recvshortcrc,
    .recv_r7          = litex_recvshort,
    .waitenable       = litex_waitenable,
    .eventwait        = litex_eventwait,
    .callbackenable   = litex_callbackenable,
    .registercallback = litex_registercallback,
#ifdef CONFIG_SDIO_DMA
#ifdef CONFIG_LITEX_SDMMC_DMA
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
    .dmapreflight     = litex_dmapreflight,
#endif
    .dmarecvsetup     = litex_dmarecvsetup,
    .dmasendsetup     = litex_dmasendsetup,
#ifdef CONFIG_ARCH_HAVE_SDIO_DELAYED_INVLDT
    .dmadelydinvldt   = litex_dmadelydinvldt,
#endif
#else
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
    .dmapreflight     = NULL,
#endif
    .dmarecvsetup     = litex_recvsetup,
    .dmasendsetup     = litex_sendsetup,
#endif /* CONFIG_LITEX_SDMMC_DMA */
#endif /* CONFIG_SDIO_DMA*/
  },
  .phy_base          = CSR_SDPHY_BASE,
  .base              = CSR_SDCORE_BASE,
  .nirq              = CSR_SDIRQ_BASE,
#ifdef CONFIG_LITEX_SDMMC1_DMAPRIO
  .dmapri            = CONFIG_LITEX_SDMMC1_DMAPRIO,
#endif

#ifdef HAVE_SDMMC_SDIO_MODE
#ifdef CONFIG_SDMMC1_SDIO_MODE
  .sdiomode          = true,
#else
  .sdiomode          = false,
#endif
  .do_sdio_card      = NULL,
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t pow2_round_up(uint32_t r) {
	r--;
	r |= r >>  1;
	r |= r >>  2;
	r |= r >>  4;
	r |= r >>  8;
	r |= r >> 16;
	r++;
	return r;
}

static inline void sdcard_set_clk_freq(uint32_t clk_freq) {
	uint32_t divider;
	divider = clk_freq ? CONFIG_CLOCK_FREQUENCY/clk_freq : 256;
	divider = pow2_round_up(divider);
	divider <<= 1; /* NOTE: workaround for occasional sdcardboot failure */
	divider = min(max(divider, 2), 256);
	putreg32(divider, CSR_SDPHY_CLOCKER_DIVIDER_ADDR);
}

int sdcard_wait_cmd_done(void) {
	uint32_t event;
	for (;;) {
		event = getreg32(CSR_SDCORE_CMD_EVENT_ADDR);
		usleep(10);
		if (event & 0x1)
			break;
	}
	if (event & 0x4)
		return SD_TIMEOUT;
	if (event & 0x8)
		return SD_CRCERROR;
	return SD_OK;
}

int sdcard_wait_data_done(void) {
	uint32_t event;
	for (;;) {
		event = getreg32(CSR_SDCORE_DATA_EVENT_ADDR);
		if (event & 0x1)
			break;
		usleep(10);
	}
	if (event & 0x4)
		return SD_TIMEOUT;
	else if (event & 0x8)
		return SD_CRCERROR;
	return SD_OK;
}

static inline int sdcard_send_command(uint32_t arg, uint8_t cmd, uint8_t rsp) {
	putreg32(arg, CSR_SDCORE_CMD_ARGUMENT_ADDR);
	putreg32((cmd << 8) | rsp, CSR_SDCORE_CMD_COMMAND_ADDR);
	putreg8(1, CSR_SDCORE_CMD_SEND_ADDR);
	return sdcard_wait_cmd_done();
}

static int sdcard_go_idle(void) {
	return sdcard_send_command(0, 0, SDCARD_CTRL_RESPONSE_NONE);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sdio_initialize
 *
 * Description:
 *   Initialize SDIO for operation.
 *
 * Input Parameters:
 *   slotno - Not used.
 *
 * Returned Value:
 *   A reference to an SDIO interface structure.  NULL is returned on
 *   failures.
 *
 ****************************************************************************/

FAR struct sdio_dev_s *sdio_initialize(int slotno)
{
  struct litex_dev_s *priv = NULL;
  if (slotno == 0)
    {
      /* Select SDMMC 1 */

      priv = &g_sdmmcdev1;
      sdcard_set_clk_freq(SDCARD_CLK_FREQ_INIT);
      for (uint32_t timeout=1000; timeout>0; timeout--) 
        {
          /* Set SDCard in SPI Mode (generate 80 dummy clocks) */
          sdphy_init_initialize_write(1);
          usleep(1000); // 1ms
          
          /* Set SDCard in Idle state */
          if (sdcard_go_idle() == SD_OK)
            break;
          usleep(1000); // 1ms
        }
    }
  return &priv->dev;
}

