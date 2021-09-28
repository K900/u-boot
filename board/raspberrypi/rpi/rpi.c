// SPDX-License-Identifier: GPL-2.0
/*
 * (C) Copyright 2012-2016 Stephen Warren
 */

#include <common.h>
#include <config.h>
#include <dm.h>
#include <env.h>
#include <efi_loader.h>
#include <fdt_support.h>
#include <fdt_simplefb.h>
#include <init.h>
#include <lcd.h>
#include <memalign.h>
#include <mmc.h>
#include <asm/gpio.h>
#include <asm/arch/mbox.h>
#include <asm/arch/msg.h>
#include <asm/arch/sdhci.h>
#include <asm/global_data.h>
#include <dm/platform_data/serial_bcm283x_mu.h>
#ifdef CONFIG_ARM64
#include <asm/armv8/mmu.h>
#endif
#include <watchdog.h>
#include <dm/pinctrl.h>

DECLARE_GLOBAL_DATA_PTR;

/* Assigned in lowlevel_init.S
 * Push the variable into the .data section so that it
 * does not get cleared later.
 */
unsigned long __section(".data") fw_dtb_pointer;

/* TODO(sjg@chromium.org): Move these to the msg.c file */
struct msg_get_arm_mem {
	struct bcm2835_mbox_hdr hdr;
	struct bcm2835_mbox_tag_get_arm_mem get_arm_mem;
	u32 end_tag;
};

struct msg_get_board_rev {
	struct bcm2835_mbox_hdr hdr;
	struct bcm2835_mbox_tag_get_board_rev get_board_rev;
	u32 end_tag;
};

struct msg_get_board_serial {
	struct bcm2835_mbox_hdr hdr;
	struct bcm2835_mbox_tag_get_board_serial get_board_serial;
	u32 end_tag;
};

struct msg_get_mac_address {
	struct bcm2835_mbox_hdr hdr;
	struct bcm2835_mbox_tag_get_mac_address get_mac_address;
	u32 end_tag;
};

struct msg_get_clock_rate {
	struct bcm2835_mbox_hdr hdr;
	struct bcm2835_mbox_tag_get_clock_rate get_clock_rate;
	u32 end_tag;
};

#ifdef CONFIG_ARM64
#define DTB_DIR "broadcom/"
#else
#define DTB_DIR ""
#endif

/*
 * https://www.raspberrypi.org/documentation/hardware/raspberrypi/revision-codes/README.md
 */
struct rpi_model {
	const char *name;
	const char *fdtfile;
	bool has_onboard_eth;
};

static const struct rpi_model rpi_model_unknown = {
	"Unknown model",
	DTB_DIR "bcm283x-rpi-other.dtb",
	false,
};

static const struct rpi_model rpi_models_new_scheme[] = {
	[0x0] = {
		"Model A",
		DTB_DIR "bcm2835-rpi-a.dtb",
		false,
	},
	[0x1] = {
		"Model B",
		DTB_DIR "bcm2835-rpi-b.dtb",
		true,
	},
	[0x2] = {
		"Model A+",
		DTB_DIR "bcm2835-rpi-a-plus.dtb",
		false,
	},
	[0x3] = {
		"Model B+",
		DTB_DIR "bcm2835-rpi-b-plus.dtb",
		true,
	},
	[0x4] = {
		"2 Model B",
		DTB_DIR "bcm2836-rpi-2-b.dtb",
		true,
	},
	[0x6] = {
		"Compute Module",
		DTB_DIR "bcm2835-rpi-cm.dtb",
		false,
	},
	[0x8] = {
		"3 Model B",
		DTB_DIR "bcm2837-rpi-3-b.dtb",
		true,
	},
	[0x9] = {
		"Zero",
		DTB_DIR "bcm2835-rpi-zero.dtb",
		false,
	},
	[0xA] = {
		"Compute Module 3",
		DTB_DIR "bcm2837-rpi-cm3.dtb",
		false,
	},
	[0xC] = {
		"Zero W",
		DTB_DIR "bcm2835-rpi-zero-w.dtb",
		false,
	},
	[0xD] = {
		"3 Model B+",
		DTB_DIR "bcm2837-rpi-3-b-plus.dtb",
		true,
	},
	[0xE] = {
		"3 Model A+",
		DTB_DIR "bcm2837-rpi-3-a-plus.dtb",
		false,
	},
	[0x10] = {
		"Compute Module 3+",
		DTB_DIR "bcm2837-rpi-cm3.dtb",
		false,
	},
	[0x11] = {
		"4 Model B",
		DTB_DIR "bcm2711-rpi-4-b.dtb",
		true,
	},
	[0x13] = {
		"400",
		DTB_DIR "bcm2711-rpi-400.dtb",
		true,
	},
	[0x14] = {
		"Compute Module 4",
		DTB_DIR "bcm2711-rpi-cm4.dtb",
		true,
	},
};

static const struct rpi_model rpi_models_old_scheme[] = {
	[0x2] = {
		"Model B",
		DTB_DIR "bcm2835-rpi-b.dtb",
		true,
	},
	[0x3] = {
		"Model B",
		DTB_DIR "bcm2835-rpi-b.dtb",
		true,
	},
	[0x4] = {
		"Model B rev2",
		DTB_DIR "bcm2835-rpi-b-rev2.dtb",
		true,
	},
	[0x5] = {
		"Model B rev2",
		DTB_DIR "bcm2835-rpi-b-rev2.dtb",
		true,
	},
	[0x6] = {
		"Model B rev2",
		DTB_DIR "bcm2835-rpi-b-rev2.dtb",
		true,
	},
	[0x7] = {
		"Model A",
		DTB_DIR "bcm2835-rpi-a.dtb",
		false,
	},
	[0x8] = {
		"Model A",
		DTB_DIR "bcm2835-rpi-a.dtb",
		false,
	},
	[0x9] = {
		"Model A",
		DTB_DIR "bcm2835-rpi-a.dtb",
		false,
	},
	[0xd] = {
		"Model B rev2",
		DTB_DIR "bcm2835-rpi-b-rev2.dtb",
		true,
	},
	[0xe] = {
		"Model B rev2",
		DTB_DIR "bcm2835-rpi-b-rev2.dtb",
		true,
	},
	[0xf] = {
		"Model B rev2",
		DTB_DIR "bcm2835-rpi-b-rev2.dtb",
		true,
	},
	[0x10] = {
		"Model B+",
		DTB_DIR "bcm2835-rpi-b-plus.dtb",
		true,
	},
	[0x11] = {
		"Compute Module",
		DTB_DIR "bcm2835-rpi-cm.dtb",
		false,
	},
	[0x12] = {
		"Model A+",
		DTB_DIR "bcm2835-rpi-a-plus.dtb",
		false,
	},
	[0x13] = {
		"Model B+",
		DTB_DIR "bcm2835-rpi-b-plus.dtb",
		true,
	},
	[0x14] = {
		"Compute Module",
		DTB_DIR "bcm2835-rpi-cm.dtb",
		false,
	},
	[0x15] = {
		"Model A+",
		DTB_DIR "bcm2835-rpi-a-plus.dtb",
		false,
	},
};

static uint32_t revision;
static uint32_t rev_scheme;
static uint32_t rev_type;
static const struct rpi_model *model;

int dram_init(void)
{
	ALLOC_CACHE_ALIGN_BUFFER(struct msg_get_arm_mem, msg, 1);
	int ret;

	BCM2835_MBOX_INIT_HDR(msg);
	BCM2835_MBOX_INIT_TAG(&msg->get_arm_mem, GET_ARM_MEMORY);

	ret = bcm2835_mbox_call_prop(BCM2835_MBOX_PROP_CHAN, &msg->hdr);
	if (ret) {
		printf("bcm2835: Could not query ARM memory size\n");
		return -1;
	}

	gd->ram_size = msg->get_arm_mem.body.resp.mem_size;

	/*
	 * In some configurations the memory size returned by VideoCore
	 * is not aligned to the section size, what is mandatory for
	 * the u-boot's memory setup.
	 */
	gd->ram_size &= ~MMU_SECTION_SIZE;

	return 0;
}

#ifdef CONFIG_OF_BOARD
int dram_init_banksize(void)
{
	int ret;

	ret = fdtdec_setup_memory_banksize();
	if (ret)
		return ret;

	return fdtdec_setup_mem_size_base();
}
#endif

static void set_fdtfile(void)
{
	const char *fdtfile;

	if (env_get("fdtfile"))
		return;

	fdtfile = model->fdtfile;
	env_set("fdtfile", fdtfile);
}

/*
 * If the firmware provided a valid FDT at boot time, let's expose it in
 * ${fdt_addr} so it may be passed unmodified to the kernel.
 */
static void set_fdt_addr(void)
{
	if (env_get("fdt_addr"))
		return;

	if (fdt_magic(fw_dtb_pointer) != FDT_MAGIC)
		return;

	env_set_hex("fdt_addr", fw_dtb_pointer);
}

/*
 * Prevent relocation from stomping on a firmware provided FDT blob.
 */
unsigned long board_get_usable_ram_top(unsigned long total_size)
{
	if ((gd->ram_top - fw_dtb_pointer) > SZ_64M)
		return gd->ram_top;
	return fw_dtb_pointer & ~0xffff;
}

static void set_usbethaddr(void)
{
	ALLOC_CACHE_ALIGN_BUFFER(struct msg_get_mac_address, msg, 1);
	int ret;

	if (!model->has_onboard_eth)
		return;

	if (env_get("usbethaddr"))
		return;

	BCM2835_MBOX_INIT_HDR(msg);
	BCM2835_MBOX_INIT_TAG(&msg->get_mac_address, GET_MAC_ADDRESS);

	ret = bcm2835_mbox_call_prop(BCM2835_MBOX_PROP_CHAN, &msg->hdr);
	if (ret) {
		printf("bcm2835: Could not query MAC address\n");
		/* Ignore error; not critical */
		return;
	}

	eth_env_set_enetaddr("usbethaddr", msg->get_mac_address.body.resp.mac);

	if (!env_get("ethaddr"))
		env_set("ethaddr", env_get("usbethaddr"));

	return;
}

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
static void set_board_info(void)
{
	char s[11];

	snprintf(s, sizeof(s), "0x%X", revision);
	env_set("board_revision", s);
	snprintf(s, sizeof(s), "%d", rev_scheme);
	env_set("board_rev_scheme", s);
	/* Can't rename this to board_rev_type since it's an ABI for scripts */
	snprintf(s, sizeof(s), "0x%X", rev_type);
	env_set("board_rev", s);
	env_set("board_name", model->name);
}
#endif /* CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG */

static void set_serial_number(void)
{
	ALLOC_CACHE_ALIGN_BUFFER(struct msg_get_board_serial, msg, 1);
	int ret;
	char serial_string[17] = { 0 };

	if (env_get("serial#"))
		return;

	BCM2835_MBOX_INIT_HDR(msg);
	BCM2835_MBOX_INIT_TAG_NO_REQ(&msg->get_board_serial, GET_BOARD_SERIAL);

	ret = bcm2835_mbox_call_prop(BCM2835_MBOX_PROP_CHAN, &msg->hdr);
	if (ret) {
		printf("bcm2835: Could not query board serial\n");
		/* Ignore error; not critical */
		return;
	}

	snprintf(serial_string, sizeof(serial_string), "%016llx",
		 msg->get_board_serial.body.resp.serial);
	env_set("serial#", serial_string);
}

int misc_init_r(void)
{
	set_fdt_addr();
	set_fdtfile();
	set_usbethaddr();
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	set_board_info();
#endif
	set_serial_number();

	return 0;
}

static void get_board_rev(void)
{
	ALLOC_CACHE_ALIGN_BUFFER(struct msg_get_board_rev, msg, 1);
	int ret;
	const struct rpi_model *models;
	uint32_t models_count;

	BCM2835_MBOX_INIT_HDR(msg);
	BCM2835_MBOX_INIT_TAG(&msg->get_board_rev, GET_BOARD_REV);

	ret = bcm2835_mbox_call_prop(BCM2835_MBOX_PROP_CHAN, &msg->hdr);
	if (ret) {
		printf("bcm2835: Could not query board revision\n");
		/* Ignore error; not critical */
		return;
	}

	/*
	 * For details of old-vs-new scheme, see:
	 * https://github.com/pimoroni/RPi.version/blob/master/RPi/version.py
	 * http://www.raspberrypi.org/forums/viewtopic.php?f=63&t=99293&p=690282
	 * (a few posts down)
	 *
	 * For the RPi 1, bit 24 is the "warranty bit", so we mask off just the
	 * lower byte to use as the board rev:
	 * http://www.raspberrypi.org/forums/viewtopic.php?f=63&t=98367&start=250
	 * http://www.raspberrypi.org/forums/viewtopic.php?f=31&t=20594
	 */
	revision = msg->get_board_rev.body.resp.rev;
	if (revision & 0x800000) {
		rev_scheme = 1;
		rev_type = (revision >> 4) & 0xff;
		models = rpi_models_new_scheme;
		models_count = ARRAY_SIZE(rpi_models_new_scheme);
	} else {
		rev_scheme = 0;
		rev_type = revision & 0xff;
		models = rpi_models_old_scheme;
		models_count = ARRAY_SIZE(rpi_models_old_scheme);
	}
	if (rev_type >= models_count) {
		printf("RPI: Board rev 0x%x outside known range\n", rev_type);
		model = &rpi_model_unknown;
	} else if (!models[rev_type].name) {
		printf("RPI: Board rev 0x%x unknown\n", rev_type);
		model = &rpi_model_unknown;
	} else {
		model = &models[rev_type];
	}

	printf("RPI %s (0x%x)\n", model->name, revision);
}

int board_init(void)
{
#ifdef CONFIG_HW_WATCHDOG
	hw_watchdog_init();
#endif

	get_board_rev();

	gd->bd->bi_boot_params = 0x100;

	return bcm2835_power_on_module(BCM2835_MBOX_POWER_DEVID_USB_HCD);
}

/*
 * If the firmware passed a device tree use it for U-Boot.
 */
void *board_fdt_blob_setup(void)
{
	if (fdt_magic(fw_dtb_pointer) != FDT_MAGIC)
		return NULL;
	return (void *)fw_dtb_pointer;
}

void copy_emmc_config(void *our_fdt)
{
	/*
	 * As of 2021-09-28, the Pi 4 has two different revisions, one using a
	 * B0 stepping of the BCM2711 SoC, and one using a C0 stepping.
	 *
	 * The two SoC versions have different, incompatible DMA mappings for
	 * the on-board eMMC controller, which would normally make them require
	 * two different DTs.
	 *
	 * Unfortunately for us, the different revisions don't actually _use_
	 * different DTs - instead, the proprietary stage0 bootloader reads the DT,
	 * patches it in-memory, then passes the corrected DT to the OS.
	 *
	 * In our case, the OS is actually U-Boot, and U-Boot can choose to
	 * completely disregard the firmware-supplied DT and load a custom one
	 * instead, which is used by, e.g., NixOS.
	 *
	 * When that happens, the DT patches applied by the firmware are also
	 * thrown out, which leads to BCM2711C0 boards being unable to boot
	 * due to them trying to use the hardcoded DMA mappings in the DT
	 * (which are for the B0 revision).
	 *
	 * Work around that by manually copying the DMA region setup from the
	 * firmware-provided DT into whatever DT we're actually being asked
	 * to load.
	 */
	void *fw_fdt = (void *)fw_dtb_pointer;
	int fw_emmc_node;
	int our_emmc_node;
	int length;
	const void *fw_value;
	int result;

	fw_emmc_node = fdt_path_offset(fw_fdt, "emmc2bus");
	if (fw_emmc_node < 0) {
		printf("RPi: Failed to find EMMC config in FW DT: %d\n", fw_emmc_node);
		return;
	}

	our_emmc_node = fdt_path_offset(our_fdt, "emmc2bus");
	if (our_emmc_node < 0) {
		printf("RPi: Failed to find EMMC config in our DT: %d\n", our_emmc_node);
		return;
	}

	*fw_value = fdt_getprop(fw_fdt, fw_emmc_node, "dma-ranges", &length);
	if (!fw_value) {
		printf("RPi: Failed to get EMMC DMA ranges property from FW DT: %d\n", length);
		return;
	}

	result = fdt_setprop(our_fdt, our_emmc_node, "dma-ranges", fw_value, length);
	if (result != 0) {
		printf("RPi: Failed to set EMMC DMA ranges property in our DT: %d\n", result);
		return;
	}

	printf("RPi: successfully copied FW DT EMMC configuration to our DT!\n");
}

int ft_board_setup(void *blob, struct bd_info *bd)
{
	int node;

	node = fdt_node_offset_by_compatible(blob, -1, "simple-framebuffer");
	if (node < 0)
		lcd_dt_simplefb_add_node(blob);

#ifdef CONFIG_EFI_LOADER
	/* Reserve the spin table */
	efi_add_memory_map(0, CONFIG_RPI_EFI_NR_SPIN_PAGES << EFI_PAGE_SHIFT,
			   EFI_RESERVED_MEMORY_TYPE);
#endif

	copy_emmc_config(blob);

	return 0;
}
