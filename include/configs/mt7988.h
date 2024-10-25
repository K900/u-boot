/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Configuration for MediaTek MT7988 SoC
 *
 * Copyright (C) 2022 MediaTek Inc.
 * Author: Sam Shih <sam.shih@mediatek.com>
 */

#ifndef __MT7988_H
#define __MT7988_H

#define CFG_MAX_MEM_MAPPED		0xC0000000

// Start at 1.25G, reserve 256M for the kernel, and 16M for the device tree. 
// We have RAM, it's fine (probably).
#define CFG_EXTRA_ENV_SETTINGS \
	"kernel_addr_r=0x50000000\0" \
    "fdt_addr_r=0x60000000\0" \
    "ramdisk_addr_r=0x61000000\0" \
    "fdtfile=" CONFIG_DEFAULT_FDT_FILE "\0"

#endif
