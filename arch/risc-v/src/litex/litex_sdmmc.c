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
  .base              = STM32_SDMMC1_BASE,
  .nirq              = STM32_IRQ_SDMMC1,
#ifdef CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE
  .d0_gpio           = SDMMC1_SDIO_PULL(GPIO_SDMMC1_D0),
#endif
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
    }
  return &priv->dev;
}

