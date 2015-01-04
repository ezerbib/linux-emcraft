/*
 * linux/arch/arm/mach-lpc178x/spi.c
 *
 * Copyright (C) 2013 Vladimir Khusainov, Emcraft Systems
 * Copyright (C) 2013 Andreas Haas, ah114088@gmx.de
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/spi.h>
#include <linux/amba/bus.h>
#include <linux/amba/pl022.h>
#include <linux/spi/flash.h>
#include <linux/spi/mmc_spi.h>
#include <linux/mmc/host.h>
#include <mach/lpc178x.h>
#include <mach/platform.h>
#include <mach/clock.h>
#include <mach/iomux.h>
#include <mach/spi.h>

/* 
 * Size of the SSP/SPI controller register area 
 */
#define SPI_LPC178X_REGS_SIZE	0xFFF

/*
 * SSP/SPI_0
 */
#if defined(CONFIG_LPC178X_SPI0)

#define SPI_LPC178X_DEV0_IRQ	14
#define SPI_LPC178X_DEV0_REGS	0x40088000

static struct pl022_ssp_controller lpc178x_spi0_data = {
	.bus_id                 = 0,
	.num_chipselect         = 8,
	.enable_dma             = 0,
};

static struct amba_device lpc178x_spi0_dev = {
	.dev                            = {
 		.coherent_dma_mask      = ~0,
		.init_name              = "dev:ssp0",
		.platform_data          = &lpc178x_spi0_data,
	},
	.res                            = {
 		.start                  = SPI_LPC178X_DEV0_REGS,
		.end                    = SPI_LPC178X_DEV0_REGS + 
						SPI_LPC178X_REGS_SIZE,
		.flags                  = IORESOURCE_MEM,
 	},
	.dma_mask                       = ~0,
	.irq                            = {SPI_LPC178X_DEV0_IRQ, NO_IRQ},
};

#endif

#if defined(CONFIG_LPC178X_SPI0) && defined(CONFIG_MMC_SPI)

/*
 * Chip Select control for the SPI Flash on SPI0 of LPC-LNX eval board
 */
#define SPI_FLASH_CS_GRP__LPC178X_EVAL_AUDIO	0
#define SPI_FLASH_CS_PIN__LPC178X_EVAL_AUDIO	16

static void spi_lpc178x_flash_cs__lpc178x_eval(u32 control)
{
	// AUDIO SPI CS
	lpc178x_gpio_out(SPI_FLASH_CS_GRP__LPC178X_EVAL_AUDIO,
				SPI_FLASH_CS_PIN__LPC178X_EVAL_AUDIO, control);
}

#endif

/*
 * SSP/SPI_1
 */
#if defined(CONFIG_LPC178X_SPI1)

#define SPI_LPC178X_DEV1_IRQ	15
#define SPI_LPC178X_DEV1_REGS	0x40030000

static struct pl022_ssp_controller lpc178x_spi1_data = {
	.bus_id                 = 1,
	.num_chipselect         = 8,
	.enable_dma             = 0,
};

static struct amba_device lpc178x_spi1_dev = {
	.dev                            = {
 		.coherent_dma_mask      = ~0,
		.init_name              = "dev:ssp1",
		.platform_data          = &lpc178x_spi1_data,
	},
	.res                            = {
 		.start                  = SPI_LPC178X_DEV1_REGS,
		.end                    = SPI_LPC178X_DEV1_REGS +
						SPI_LPC178X_REGS_SIZE,
		.flags                  = IORESOURCE_MEM,
 	},
	.dma_mask                       = ~0,
	.irq                            = {SPI_LPC178X_DEV1_IRQ, NO_IRQ},
};

#endif


#if defined(CONFIG_LPC178X_SPI1)
/*
 * Chip Select control for the SPI Flash on SPI1 of kramer board
 */
#define SPI_FLASH_CS_GRP__LPC178X_EVAL_FLASH	0
#define SPI_FLASH_CS_PIN__LPC178X_EVAL_FLASH	5

static void spi_lpc178x_flash_cs__lpc178x_eval_ssp1_flash(u32 control)
{
	// FLASH SPI CS
	lpc178x_gpio_out(SPI_FLASH_CS_GRP__LPC178X_EVAL_FLASH,
				SPI_FLASH_CS_PIN__LPC178X_EVAL_FLASH, control);
}

/*
 * Chip Select control for the SPI Flash on SPI1 of of kramer board
 */
#define SPI_FLASH_CS_GRP__LPC178X_EVAL_EEPROM	0
#define SPI_FLASH_CS_PIN__LPC178X_EVAL_EEPROM	4

static void spi_lpc178x_flash_cs__lpc178x_eval_ssp1_audio(u32 control)
{
	// EEPROM SPI CS
	lpc178x_gpio_out(SPI_FLASH_CS_GRP__LPC178X_EVAL_EEPROM,
				SPI_FLASH_CS_PIN__LPC178X_EVAL_EEPROM, control);
}

#endif
/*
 * Register the LPC178X specific SPI controllers and devices with the kernel.
 */
//__init
void  lpc178x_spi_init(void)
{
	int p = lpc178x_platform_get();

#if defined(CONFIG_LPC178X_SPI0)
	amba_device_register(&lpc178x_spi0_dev, &iomem_resource);
#endif

#if defined(CONFIG_LPC178X_SPI1)
	amba_device_register(&lpc178x_spi1_dev, &iomem_resource);
#endif

	if (p == PLATFORM_LPC178X_LNX_EVB || p == PLATFORM_LPC178X_EA_LPC1788) {

		/* This assumes that the code is running on
		 * the Emcraft LPC-LNX-EVB board, which
		 * has an SD Card hand-wired on SSP/SPI0.
		 */
#if defined(CONFIG_LPC178X_SPI0) && defined(CONFIG_MMC_SPI)
		/*
 		 * SPI slave
 		 */
		static struct pl022_config_chip spi0_slave_audio = {
 			.com_mode = INTERRUPT_TRANSFER,
			.iface = SSP_INTERFACE_MOTOROLA_SPI,
			.hierarchy = SSP_MASTER,
			.slave_tx_disable = 0,
			.rx_lev_trig = SSP_RX_4_OR_MORE_ELEM,
			.tx_lev_trig = SSP_TX_4_OR_MORE_EMPTY_LOC,
			.ctrl_len = SSP_BITS_8,
			.data_size = SSP_DATA_BITS_8,
			.wait_state = SSP_MWIRE_WAIT_ZERO,
			.duplex = SSP_MICROWIRE_CHANNEL_FULL_DUPLEX,
			.cs_control = spi_lpc178x_flash_cs__lpc178x_eval,
		};

		static struct mmc_spi_platform_data mmc_pdata = {
				.detect_delay = 100,
				.powerup_msecs = 100,
				.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
		}; 

		static struct spi_board_info spi0_board_info = {
			.modalias = "spidev",
			.max_speed_hz = 10000000, //10Mhz
			.bus_num = 0,
			.chip_select = 0,
			.controller_data = &spi0_slave_audio,
			.mode = SPI_MODE_0,
			.platform_data = &mmc_pdata, 
		};

		/*
		 * Set up the Chip Select GPIO for the SPI Flash
		 */
		lpc178x_gpio_dir(SPI_FLASH_CS_GRP__LPC178X_EVAL_AUDIO,
			SPI_FLASH_CS_PIN__LPC178X_EVAL_AUDIO, 1);
		lpc178x_gpio_out(SPI_FLASH_CS_GRP__LPC178X_EVAL_AUDIO,
				SPI_FLASH_CS_PIN__LPC178X_EVAL_AUDIO, 1);

		/*
		 * Register SPI slaves
		 */
		spi_register_board_info(&spi0_board_info,
			sizeof(spi0_board_info) / 
			sizeof(struct spi_board_info));
#endif

#if defined(CONFIG_LPC178X_SPI1)

//#if ! defined(CONFIG_SPI_SPIDEV)
				/*
				 * SPI Flash partitioning:
				 */
		// 128K
		#define FLASH_IMAGE_OFFSET__LPC18XX_EVAL	0x20000
		// 2.5 Mb
		#define FLASH_JFFS2_OFFSET__LPC18XX_EVAL	(2*1024*1024+500*1024) + FLASH_IMAGE_OFFSET__LPC18XX_EVAL
				static struct mtd_partition
					spi_lpc178x_flash_partitions__lpc178x_eval[] = {
					{
						.name	= "flash_uboot",
						.offset = 0,
						.size	= FLASH_IMAGE_OFFSET__LPC18XX_EVAL,
					},
					{
						.name	= "flash_linux_image",
						.offset = FLASH_IMAGE_OFFSET__LPC18XX_EVAL,
						.size	= (FLASH_JFFS2_OFFSET__LPC18XX_EVAL
							 - FLASH_IMAGE_OFFSET__LPC18XX_EVAL),
					},
					{
						.name	= "flash_jffs2",
						.offset = FLASH_JFFS2_OFFSET__LPC18XX_EVAL,
					},
				};

				/*
				 * SPI Flash
		 		 */
				static struct flash_platform_data
					spi_lpc18xx_flash_data__lpc18xx_eval = {
					.name = "sst25vf032b",
					.parts =  spi_lpc178x_flash_partitions__lpc178x_eval,
					.nr_parts =
					ARRAY_SIZE(spi_lpc178x_flash_partitions__lpc178x_eval),
					.type = "sst25vf032b",
				};
				
				static struct mtd_partition
					spi_lpc178x_eeprom_partitions__lpc178x_eval[] = {
					{
						.name	= "nvdata",
						.offset = 0,
						.size	= 32768,
					},
				};
				
				/*
				 * SPI EEPROM
		 		 */
				static struct flash_platform_data
					spi_lpc18xx_eeprom_data__lpc18xx_eval = {
					.name = "m95256wm",
					.parts =  spi_lpc178x_eeprom_partitions__lpc178x_eval,
					.nr_parts =
					ARRAY_SIZE(spi_lpc178x_eeprom_partitions__lpc178x_eval),
					.type = "m95256wm",
				};
//#endif
		/*
 		 * SPI slave
 		 */
		static struct pl022_config_chip spi1_slave_flash = {
 			.com_mode = INTERRUPT_TRANSFER,
			.iface = SSP_INTERFACE_MOTOROLA_SPI,
			.hierarchy = SSP_MASTER,
			.slave_tx_disable = 0,
			.rx_lev_trig = SSP_RX_4_OR_MORE_ELEM,
			.tx_lev_trig = SSP_TX_4_OR_MORE_EMPTY_LOC,
			.ctrl_len = SSP_BITS_8,
			.data_size = SSP_DATA_BITS_8,
			.wait_state = SSP_MWIRE_WAIT_ZERO,
			.duplex = SSP_MICROWIRE_CHANNEL_FULL_DUPLEX,
			.cs_control = spi_lpc178x_flash_cs__lpc178x_eval_ssp1_flash,
		};

		static struct pl022_config_chip spi1_slave_eeprom = {
 			.com_mode = INTERRUPT_TRANSFER,
			.iface = SSP_INTERFACE_MOTOROLA_SPI,
			.hierarchy = SSP_MASTER,
			.slave_tx_disable = 0,
			.rx_lev_trig = SSP_RX_4_OR_MORE_ELEM,
			.tx_lev_trig = SSP_TX_4_OR_MORE_EMPTY_LOC,
			.ctrl_len = SSP_BITS_8,
			.data_size = SSP_DATA_BITS_8,
			.wait_state = SSP_MWIRE_WAIT_ZERO,
			.duplex = SSP_MICROWIRE_CHANNEL_FULL_DUPLEX,
			.cs_control = spi_lpc178x_flash_cs__lpc178x_eval_ssp1_audio,
		};


		/*
		 * Chip disable
		 */
		lpc178x_gpio_out(SPI_FLASH_CS_GRP__LPC178X_EVAL_FLASH,
				SPI_FLASH_CS_PIN__LPC178X_EVAL_FLASH, 1);
		lpc178x_gpio_out(SPI_FLASH_CS_GRP__LPC178X_EVAL_EEPROM,
				SPI_FLASH_CS_PIN__LPC178X_EVAL_EEPROM, 1);

		/*
		 * Set up the Chip Select GPIO for the SPI Flash
		 */
		lpc178x_gpio_dir(SPI_FLASH_CS_GRP__LPC178X_EVAL_FLASH,
			SPI_FLASH_CS_PIN__LPC178X_EVAL_FLASH, 1);
		lpc178x_gpio_dir(SPI_FLASH_CS_GRP__LPC178X_EVAL_EEPROM,
			SPI_FLASH_CS_PIN__LPC178X_EVAL_EEPROM, 1);

		/*
		 * I'm not sure about theses structures ?
		 */
		

		static struct spi_board_info spi1_board_info[] = {
				{
					.modalias = "m25p80",
					.max_speed_hz = 24000000,  //24MHZ - 25MHz in datasheet for READ
					//.max_speed_hz = 10000000,  //10MHZ -
					.bus_num = 1,
					.chip_select = 0,
					.controller_data = &spi1_slave_flash,
					.mode = SPI_MODE_0,
					.platform_data = &spi_lpc18xx_flash_data__lpc18xx_eval,
				},
				{
					/*
					 * SPIDEV
					 */
					.modalias = "m95m01_r",
					.max_speed_hz = 10000000,  //10MHZ
					.bus_num = 1,
					.chip_select = 1,
					.controller_data = &spi1_slave_eeprom,
					.mode = SPI_MODE_0,
					.platform_data = &spi_lpc18xx_eeprom_data__lpc18xx_eval,
				}};
		/*
		 * Register SPI slaves
		 */
		spi_register_board_info(&spi1_board_info,
			sizeof(spi1_board_info) /
			sizeof(struct spi_board_info));
#endif
	}
}
