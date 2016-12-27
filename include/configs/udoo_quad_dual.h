/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2015 UDOO Team
 *
 * Configuration settings for UDOO Quad/Dual board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"

#define CONFIG_SPL_ENV_SUPPORT
#include "imx6_spl.h"

#define MACH_TYPE_UDOO		4800
#define CONFIG_MACH_TYPE	MACH_TYPE_UDOO

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(2 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART2_BASE

/* SATA Configs */

#define CONFIG_CMD_SATA
#ifdef CONFIG_CMD_SATA
#define CONFIG_DWC_AHSATA
#define CONFIG_SYS_SATA_MAX_DEVICE	1
#define CONFIG_DWC_AHSATA_PORT_ID	0
#define CONFIG_DWC_AHSATA_BASE_ADDR	SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#define CONFIG_LIBATA
#endif

/* Network support */

#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE                    ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE             RGMII
#define CONFIG_ETHPRIME                 "FEC"
#define CONFIG_FEC_MXC_PHYADDR          6
#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
#define CONFIG_PHY_MICREL_KSZ9031

/* Command definition */
#define CONFIG_CMD_BMODE

#define CONFIG_SYS_MEMTEST_START	0x10000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 500 * SZ_1M)

/* MMC Configuration */
#define CONFIG_SYS_FSL_ESDHC_ADDR	0
#define CONFIG_SYS_MAX_FLASH_BANKS 1


#ifdef CONFIG_SATABOOT
	#define UBOOT_DEVICE	"sata"
	#define CONFIG_MMCROOT	"/dev/sda2"
	#define SATA_INIT		"sata init;"
#else
	#define UBOOT_DEVICE	"mmc"
	#define CONFIG_MMCROOT	"/dev/mmcblk0p2"
	#define SATA_INIT		""
#endif

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

#define CONFIG_EXTRA_ENV_SETTINGS \
	"script=uEnv.txt\0" \
	"image=/boot/zImage\0" \
	"console=ttymxc1\0" \
	"splashpos=m,m\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"fdt_file=undefined\0" \
	"fdt_addr=0x18000000\0" \
	"boot_fdt=try\0" \
	"ip_dyn=yes\0" \
	"mmcdev=0\0" \
	"mmcpart=2\0" \
	"mmcrootfstype=ext4\0" \
	"mmcroot=" CONFIG_MMCROOT " rootwait rw\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} ${video} ${memory} " \
		"root=${mmcroot} rootfstype=${mmcrootfstype} ahci_imx.hotplug=1\0" \
	"loadbootscript=" \
		"fatload " UBOOT_DEVICE " ${mmcdev}:1 ${loadaddr} ${script};\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"env import -t ${loadaddr} ${filesize};\0" \
	"loadimage=ext2load " UBOOT_DEVICE " ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"loadfdt=ext2load " UBOOT_DEVICE " ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0" \
	"netargs=setenv bootargs console=${console},${baudrate} " \
		"root=/dev/nfs " \
	"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
		"netboot=echo Booting from net ...; " \
		"run netargs; " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${image}; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if ${get_cmd} ${fdt_addr} ${fdt_file}; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0"

#define CONFIG_BOOTCOMMAND \
		"mmc dev ${mmcdev}; " SATA_INIT \
		"if mmc rescan; then " \
			"if run loadbootscript; then " \
				"run bootscript; " \
			"fi; " \
			"udooinit; " \
			"if run loadimage; then " \
				"run mmcboot; " \
			"else run netboot; " \
			"fi; " \
		"else run netboot; fi"

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* Environment organization */
#define CONFIG_ENV_SIZE			(8 * 1024)

#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_OFFSET		(6 * 64 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV		0

#endif			       /* __CONFIG_H * */
