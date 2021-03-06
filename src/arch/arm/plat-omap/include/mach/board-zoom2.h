/*
 * arch/arm/plat-omap/include/mach/board-zoom2.h
 *
 * Hardware definitions for TI OMAP3 LDP.
 *
 * Copyright (C) 2008 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __ASM_ARCH_OMAP_ZOOM2_H
#define __ASM_ARCH_OMAP_ZOOM2_H

extern void ldp_flash_init(void);
extern void twl4030_bci_battery_init(void);
extern unsigned get_last_off_on_transaction_id(struct device *dev);

#define TWL4030_IRQNUM		INT_34XX_SYS_NIRQ
#define LDP3430_NAND_CS		0 

/* LAN chip details */
#define LDP_SMC911X_CS		7
#define LDP_SMC911X_GPIO	158
#define DEBUG_BASE		0x08000000
#define OMAP34XX_ETHR_START	DEBUG_BASE

#define OMAP3_WAKEUP (PRCM_WAKEUP_T2_KEYPAD |\
			PRCM_WAKEUP_TOUCHSCREEN | PRCM_WAKEUP_UART)

#endif /* __ASM_ARCH_OMAP_ZOOM2_H */
