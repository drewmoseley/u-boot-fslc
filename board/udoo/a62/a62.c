/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include "../common/udooinit.h"

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <malloc.h>
#include <asm/arch/mx6-pins.h>
#include <linux/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/sata.h>
#include <asm/imx-common/spi.h>
#include <asm/imx-common/mxc_i2c.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <i2c.h>
#include <spi_flash.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |                   \
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
		PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL   (PAD_CTL_PUS_100K_UP |                  \
		PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |   \
		PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define WDT_EN		IMX_GPIO_NR(5, 4)
#define WDT_TRG		IMX_GPIO_NR(3, 19)

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

static iomux_v3_cfg_t const uart2_pads[] = {
	IOMUX_PADS(PAD_EIM_D26__UART2_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D27__UART2_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

/* ON BOARD uSD	*/
iomux_v3_cfg_t const usdhc3_pads[] = {
	IOMUX_PADS(PAD_SD3_CLK__SD3_CLK     | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_CMD__SD3_CMD     | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT0__SD3_DATA0  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT1__SD3_DATA1  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT2__SD3_DATA2  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT3__SD3_DATA3  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT5__GPIO7_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL)),     // CD
	IOMUX_PADS(PAD_NANDF_D5__GPIO2_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL)),     // PWR
};

/* eMMC */
iomux_v3_cfg_t const usdhc4_pads[] = {
	IOMUX_PADS(PAD_SD4_CLK__SD4_CLK     | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_CMD__SD4_CMD     | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT0__SD4_DATA0  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT1__SD4_DATA1  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT2__SD4_DATA2  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT3__SD4_DATA3  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT4__SD4_DATA4  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT5__SD4_DATA5  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT6__SD4_DATA6  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT7__SD4_DATA7  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	IOMUX_PADS(PAD_EIM_A24__GPIO5_IO04 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D19__GPIO3_IO19),
};

static iomux_v3_cfg_t const ecspi1_pads[] = {
	IOMUX_PADS(PAD_EIM_D17__ECSPI1_MISO  | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D18__ECSPI1_MOSI  | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D16__ECSPI1_SCLK  | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_EB2__GPIO2_IO30   | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

/*  I2C2 - EEPROM, HDMI  */
struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode  = MX6Q_PAD_KEY_COL3__I2C2_SCL   | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_KEY_COL3__GPIO4_IO12 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp        = IMX_GPIO_NR(4, 12),
	},
	.sda = {
		.i2c_mode  = MX6Q_PAD_KEY_ROW3__I2C2_SDA   | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_KEY_ROW3__GPIO4_IO13 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp        = IMX_GPIO_NR(4, 13),
	},
};


/*  I2C3 - H29, CN11  */
struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode  = MX6Q_PAD_GPIO_5__I2C3_SCL
				| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_GPIO_5__GPIO1_IO05
				| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp        = IMX_GPIO_NR(1, 5),
	},
	.sda = {
		.i2c_mode  = MX6Q_PAD_GPIO_6__I2C3_SDA
				| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_GPIO_6__GPIO1_IO06
				| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp        = IMX_GPIO_NR(7, 11),
	},
};
int mx6_rgmii_rework(struct phy_device *phydev)
{
	/*
	 * Bug: Apparently uDoo does not works with Gigabit switches...
	 * Limiting speed to 10/100Mbps, and setting master mode, seems to
	 * be the only way to have a successfull PHY auto negotiation.
	 * How to fix: Understand why Linux kernel do not have this issue.
	 */
	phy_write(phydev, MDIO_DEVAD_NONE, MII_CTRL1000, 0x1c00);

	/* control data pad skew - devaddr = 0x02, register = 0x04 */
	ksz9031_phy_extended_write(phydev, 0x02,
				   MII_KSZ9031_EXT_RGMII_CTRL_SIG_SKEW,
				   MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x0000);
	/* rx data pad skew - devaddr = 0x02, register = 0x05 */
	ksz9031_phy_extended_write(phydev, 0x02,
				   MII_KSZ9031_EXT_RGMII_RX_DATA_SKEW,
				   MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x0000);
	/* tx data pad skew - devaddr = 0x02, register = 0x05 */
	ksz9031_phy_extended_write(phydev, 0x02,
				   MII_KSZ9031_EXT_RGMII_TX_DATA_SKEW,
				   MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x0000);
	/* gtx and rx clock pad skew - devaddr = 0x02, register = 0x08 */
	ksz9031_phy_extended_write(phydev, 0x02,
				   MII_KSZ9031_EXT_RGMII_CLOCK_SKEW,
				   MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x03FF);
	return 0;
}

static iomux_v3_cfg_t const enet_pads1[] = {
	IOMUX_PADS(PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TXC__RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD0__RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD1__RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD2__RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD3__RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RXC__RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	/* RGMII reset */
	IOMUX_PADS(PAD_EIM_D23__GPIO3_IO23		| MUX_PAD_CTRL(NO_PAD_CTRL)),
	/* Ethernet power supply */
	IOMUX_PADS(PAD_KEY_COL2__GPIO4_IO10		| MUX_PAD_CTRL(NO_PAD_CTRL)),
	/* pin 32 - 1 - (MODE0) all */
	IOMUX_PADS(PAD_RGMII_RD0__GPIO6_IO25		| MUX_PAD_CTRL(NO_PAD_CTRL)),
	/* pin 31 - 1 - (MODE1) all */
	IOMUX_PADS(PAD_RGMII_RD1__GPIO6_IO27		| MUX_PAD_CTRL(NO_PAD_CTRL)),
	/* pin 28 - 1 - (MODE2) all */
	IOMUX_PADS(PAD_RGMII_RD2__GPIO6_IO28		| MUX_PAD_CTRL(NO_PAD_CTRL)),
	/* pin 27 - 1 - (MODE3) all */
	IOMUX_PADS(PAD_RGMII_RD3__GPIO6_IO29		| MUX_PAD_CTRL(NO_PAD_CTRL)),
	/* pin 33 - 1 - (CLK125_EN) 125Mhz clockout enabled */
	IOMUX_PADS(PAD_RGMII_RX_CTL__GPIO6_IO24	| MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static iomux_v3_cfg_t const enet_pads2[] = {
	IOMUX_PADS(PAD_RGMII_RD0__RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD1__RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD2__RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD3__RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
};

static void setup_a62_i2c(void)
{
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);

}

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
		return (bus == 0 && cs == 0) ? (IMX_GPIO_NR(2, 30)) : -1;
}

static void setup_spinor(void)
{
	gpio_request(IMX_GPIO_NR(2, 30), "spi_cs");
	gpio_direction_output(IMX_GPIO_NR(2, 30), 1);
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
}

static void setup_iomux_enet(void)
{
	SETUP_IOMUX_PADS(enet_pads1);
	udelay(20);
	gpio_direction_output(IMX_GPIO_NR(4, 10), 1); /* Power supply on */
	gpio_set_value (IMX_GPIO_NR(4, 10), 1);

	gpio_direction_output(IMX_GPIO_NR(3, 23), 0); /* assert PHY rst */

	gpio_direction_output(IMX_GPIO_NR(6, 24), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 25), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 27), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 28), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 29), 1);
	udelay(1000);

	gpio_set_value(IMX_GPIO_NR(3, 23), 1); /* deassert PHY rst */

	/* Need 100ms delay to exit from reset. */
	udelay(1000 * 100);

	gpio_free(IMX_GPIO_NR(6, 24));
	gpio_free(IMX_GPIO_NR(6, 25));
	gpio_free(IMX_GPIO_NR(6, 27));
	gpio_free(IMX_GPIO_NR(6, 28));
	gpio_free(IMX_GPIO_NR(6, 29));

	SETUP_IOMUX_PADS(enet_pads2);
}

static void setup_iomux_uart(void)
{
	SETUP_IOMUX_PADS(uart2_pads);
}

static void setup_iomux_wdog(void)
{
	SETUP_IOMUX_PADS(wdog_pads);
	gpio_direction_output(WDT_TRG, 0);
	gpio_direction_output(WDT_EN, 1);
	gpio_direction_input(WDT_TRG);
}

/*  __________________________________________________________________________
 * |                                                                          |
 * |                                   USDHC                                  |
 * |__________________________________________________________________________|
 */
#ifdef CONFIG_FSL_ESDHC

#define USDHC3_CD_GPIO	IMX_GPIO_NR(7, 0)
#define USDHC3_PWR_GPIO IMX_GPIO_NR(2, 5)

/* USDHC map:
 * 	USDHC4  -->  eMMC  -->  FSL_SDHC: 0
 * 	USDHC3  -->  uSD   -->  FSL_SDHC: 1
 */

struct fsl_esdhc_cfg usdhc_cfg[CONFIG_SYS_FSL_USDHC_NUM] = {
	{USDHC3_BASE_ADDR, 0, 4}, // FSL_SDHC: 0, uSD the primary boot medium when J27 is closed
	{USDHC4_BASE_ADDR, 0, 8}, // FSL_SDHC: 1, eMMC (optional) the secondary boot medium when J27 is open
};

enum mxc_clock usdhc_clk[CONFIG_SYS_FSL_USDHC_NUM] = {
	MXC_ESDHC4_CLK,
	MXC_ESDHC3_CLK,
};

/* map the usdhc controller id to the devno given to the board device */
int usdhc_devno[4] = { -1, -1, 1, 0};

int board_mmc_getcd (struct mmc *mmc) {
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
		case USDHC3_BASE_ADDR:
			ret = !gpio_get_value(USDHC3_CD_GPIO);
			break;
		case USDHC4_BASE_ADDR:
			ret = 1; /* eMMC/uSDHC4 is always present */
			break;
	}

	return ret;
}

int check_mmc_autodetect (void) {
	char *autodetect_str = getenv ("mmcautodetect");

	if ( (autodetect_str != NULL) &&
		(strcmp(autodetect_str, "yes") == 0) ) {
		return 1;
	}

	return 0;
}

int check_emmc_selected(void) {

	struct src *psrc = (struct src *)SRC_BASE_ADDR;
	unsigned int reg_smbr1 = readl(&psrc->sbmr1);
	unsigned int reg = readl(&psrc->sbmr1) >> 11;
	unsigned int port = reg & 0x3;
	int ret = 0;
	if ((reg_smbr1 & 0x0820) == 0x0820)
	{
		ret = 1;
		printf("[SPL] Selected eMMC as boot medium; port %u \n", port);
	}
	else
	{
		printf("[SPL] Selected uSD as boot medium; port %u \n", port);
	}
	return ret;
}

int board_mmc_init(bd_t *bis){
#ifndef CONFIG_SPL_BUILD
	int ret;
	int i;
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			SETUP_IOMUX_PADS(usdhc4_pads);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			break;
		case 1:
			SETUP_IOMUX_PADS(usdhc3_pads);
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			gpio_direction_input(USDHC3_CD_GPIO);
			gpio_direction_output(USDHC3_PWR_GPIO, 1);
			break;

		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       i + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret)
			return ret;
	}

	return 0;
#else

	if (check_emmc_selected()) {
	   /* Setup eMMC */
	   SETUP_IOMUX_PADS(usdhc4_pads);
	   usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
	   return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
	} else {
	   /* Setup uSD */
	   SETUP_IOMUX_PADS(usdhc3_pads);
	   usdhc_cfg[1].esdhc_base = USDHC3_BASE_ADDR;
	   usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
	   gpio_direction_input(USDHC3_CD_GPIO);
	   gpio_direction_output(USDHC3_PWR_GPIO, 1);
	   gd->arch.sdhc_clk = usdhc_cfg[1].sdhc_clk;
	   return fsl_esdhc_initialize(bis, &usdhc_cfg[1]);
	}
#endif
}
#endif /*  CONFIG_FSL_ESDHC  */

int board_eth_init(bd_t *bis)
{
	uint32_t base = IMX_FEC_BASE;
	struct mii_dev *bus = NULL;
	struct phy_device *phydev = NULL;
	int ret;

	setup_iomux_enet();

#ifdef CONFIG_FEC_MXC
	bus = fec_get_miibus(base, -1);
	if (!bus)
		return -EINVAL;
	/* scan phy 4,5,6,7 */
	phydev = phy_find_by_mask(bus, (0xf << 4), PHY_INTERFACE_MODE_RGMII);

	if (!phydev) {
		ret = -EINVAL;
		goto free_bus;
	}
	printf("using phy at %d\n", phydev->addr);
	ret  = fec_probe(bis, -1, base, bus, phydev);
	if (ret)
		goto free_phydev;
#endif
	return 0;

free_phydev:
	free(phydev);
free_bus:
	free(bus);
	return ret;
}

int board_early_init_f(void)
{
	setup_iomux_wdog();
	setup_iomux_uart();

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	mx6_rgmii_rework(phydev);
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
/*
#ifdef CONFIG_CMD_I2C
	setup_a62_i2c();
#endif
#ifdef CONFIG_CMD_SF
	setup_spinor();
#endif
#ifdef CONFIG_CMD_SATA
	if (is_cpu_type(MXC_CPU_MX6Q))
		setup_sata();
#endif
*/
	return 0;
}

int board_late_init(void)
{
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	if (is_cpu_type(MXC_CPU_MX6Q))
		setenv("board_rev", "MX6Q");
	else if (is_cpu_type(MXC_CPU_MX6DL))
		setenv("board_rev", "MX6DL");
	else
		setenv("board_rev", "MX6SOLO");
#endif

	if (check_emmc_selected())
	{ /* Boot from eMMC */
		printf("Booting from eMMC!\n");
		setenv("mmcdev","1");
		setenv("mmcroot","/dev/mmcblk1p1 rootwait rw");
	}


	return 0;
}

int checkboard(void)
{
	if (is_cpu_type(MXC_CPU_MX6Q))
		puts("Board: Seco SBC-A62-J-QUAD\n");
	else if (is_cpu_type(MXC_CPU_MX6DL))
		 puts("Board: Seco SBC-A62-J-LITE\n");
	else
		puts("Board: Seco SBC-A62-J-SOLO\n");

	return 0;
}

/**
 * After loading uEnv.txt, we autodetect which fdt file we need to load.
 * uEnv.txt can contain:
 *  - video_output=hdmi|lvds7|lvds15
 *    any other value (or if the variable is not specified) will default to "hdmi"
 *  - use_custom_dtb=true|false
 *    any other value (or if the variable is not specified) will default to "false"
 *
 * Despite the signature, this command does not accept any argument.
 */
int do_udooinit(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char* modelfdt;

	if (is_cpu_type(MXC_CPU_MX6Q)) {
		modelfdt = "imx6q-seco_A62";
	} else {
		modelfdt = "imx6dl-seco_A62";
	}

	return udooinit_setenv(modelfdt);
}

U_BOOT_CMD(
	udooinit,	1,	1,	do_udooinit,
	"(UDOO) determine the device tree to load", ""
);
