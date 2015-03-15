/*
 * MTD SPI driver for ST M25Pxx (and similar) serial flash chips
 *
 * Author: Mike Lavender, mike@steroidmicros.com
 *
 * Copyright (c) 2005, Intec Automation Inc.
 *
 * Some parts are based on lart.c by Abraham Van Der Merwe
 *
 * Cleaned up and generalized based on mtd_dataflash.c
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/math64.h>
#include <linux/sched.h>
#include <linux/mod_devicetable.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

/* Flash opcodes. */
#define	OPCODE_WREN		0x06	/* Write enable */
#define	OPCODE_RDSR		0x05	/* Read status register */
#define	OPCODE_WRSR		0x01	/* Write status register 1 byte */
#define	OPCODE_NORM_READ	0x03	/* Read data bytes (low frequency) */
#define	OPCODE_FAST_READ	0x0b	/* Read data bytes (high frequency) */
#define	OPCODE_PP		0x02	/* Page program (up to 256 bytes) */
#define	OPCODE_BE_4K		0x20	/* Erase 4KiB block */
#define	OPCODE_BE_32K		0x52	/* Erase 32KiB block */
#define	OPCODE_CHIP_ERASE	0xc7	/* Erase whole flash chip */
#define	OPCODE_SE		0xd8	/* Sector erase (usually 64KiB) */
#define	OPCODE_RDID		0x9f	/* Read JEDEC ID */

/* Used for SST flashes only. */
#define	OPCODE_BP		0x02	/* Byte program */
#define	OPCODE_WRDI		0x04	/* Write disable */
#define	OPCODE_AAI_WP		0xad	/* Auto address increment word program */

/* Status Register bits. */
#define	SR_WIP			1	/* Write in progress */
#define	SR_WEL			2	/* Write enable latch */
/* meaning of other SR_* bits may differ between vendors */
#define	SR_BP0			4	/* Block protect 0 */
#define	SR_BP1			8	/* Block protect 1 */
#define	SR_BP2			0x10	/* Block protect 2 */
#define	SR_SRWD			0x80	/* SR write protect */

/* Define max times to check status register before we give up. */
#define	MAX_READY_WAIT_JIFFIES	(40 * HZ)	/* M25P16 specs 40s max chip erase */
#define	MAX_CMD_SIZE		4

#ifdef CONFIG_M25PXX_USE_FAST_READ
#define OPCODE_READ 	OPCODE_FAST_READ
#define FAST_READ_DUMMY_BYTE 1
#else
#define OPCODE_READ 	OPCODE_NORM_READ
#define FAST_READ_DUMMY_BYTE 0
#endif

/****************************************************************************/

struct m25p {
	struct spi_device	*spi;
	struct mutex		lock;
	struct mtd_info		mtd;
	unsigned		partitioned:1;
	u16			page_size;
	u16			addr_width;
	u8			erase_opcode;
	u8			*command;
};

static inline struct m25p *mtd_to_m25p(struct mtd_info *mtd)
{
	return container_of(mtd, struct m25p, mtd);
}

/****************************************************************************/

/*
 * Internal helper functions
 */

/*
 * Read the status register, returning its value in the location
 * Return the status register value.
 * Returns negative if error occurred.
 */
static int read_sr(struct m25p *flash)
{
	ssize_t retval;
	u8 code = OPCODE_RDSR;
	u8 val;

	retval = spi_write_then_read(flash->spi, &code, 1, &val, 1);

	if (retval < 0) {
		dev_err(&flash->spi->dev, "error %d reading SR\n",
				(int) retval);
		return retval;
	}

	return val;
}

/*
 * Write status register 1 byte
 * Returns negative if error occurred.
 */
static int write_sr(struct m25p *flash, u8 val)
{
	flash->command[0] = OPCODE_WRSR;
	flash->command[1] = val;

	return spi_write(flash->spi, flash->command, 2);
}

/*
 * Set write enable latch with Write Enable command.
 * Returns negative if error occurred.
 */
static inline int write_enable(struct m25p *flash)
{
	u8	code = OPCODE_WREN;

	return spi_write_then_read(flash->spi, &code, 1, NULL, 0);
}

/*
 * Send write disble instruction to the chip.
 */
static inline int write_disable(struct m25p *flash)
{
	u8	code = OPCODE_WRDI;

	return spi_write_then_read(flash->spi, &code, 1, NULL, 0);
}

/*
 * Service routine to read status register until ready, or timeout occurs.
 * Returns non-zero if error.
 */
static int wait_till_ready(struct m25p *flash)
{
	unsigned long deadline;
	int sr;

	deadline = jiffies + MAX_READY_WAIT_JIFFIES;

	do {
		if ((sr = read_sr(flash)) < 0)
			break;
		else if (!(sr & SR_WIP))
			return 0;

		cond_resched();

	} while (!time_after_eq(jiffies, deadline));

	return 1;
}

/*
 * Erase the whole flash memory
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_chip(struct m25p *flash)
{
	DEBUG(MTD_DEBUG_LEVEL3, "%s: %s %lldKiB\n",
	      dev_name(&flash->spi->dev), __func__,
	      (long long)(flash->mtd.size >> 10));

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash))
		return 1;

	/* Send write enable, then erase commands. */
	write_enable(flash);

	/* Set up command buffer. */
	flash->command[0] = OPCODE_CHIP_ERASE;

	spi_write(flash->spi, flash->command, 1);

	return 0;
}

static void m25p_addr2cmd(struct m25p *flash, unsigned int addr, u8 *cmd)
{
	/* opcode is in cmd[0] */
	cmd[1] = addr >> (flash->addr_width * 8 -  8);
	cmd[2] = addr >> (flash->addr_width * 8 - 16);
	cmd[3] = addr >> (flash->addr_width * 8 - 24);
}

static int m25p_cmdsz(struct m25p *flash)
{
	return 1 + flash->addr_width;
}

/*
 * Erase one sector of flash memory at offset ``offset'' which is any
 * address within the sector which should be erased.
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_sector(struct m25p *flash, u32 offset)
{
	DEBUG(MTD_DEBUG_LEVEL3, "%s: %s %dKiB at 0x%08x\n",
			dev_name(&flash->spi->dev), __func__,
			flash->mtd.erasesize / 1024, offset);

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash))
		return 1;

	/* Send write enable, then erase commands. */
	write_enable(flash);

	/* Set up command buffer. */
	flash->command[0] = flash->erase_opcode;
	m25p_addr2cmd(flash, offset, flash->command);

	spi_write(flash->spi, flash->command, m25p_cmdsz(flash));

	return 0;
}

/****************************************************************************/

/*
 * MTD implementation
 */

/*
 * Erase an address range on the flash chip.  The address range may extend
 * one or more erase sectors.  Return an error is there is a problem erasing.
 */
static int m25p80_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct m25p *flash = mtd_to_m25p(mtd);
	u32 addr,len;
	uint32_t rem;

	DEBUG(MTD_DEBUG_LEVEL2, "%s: %s %s 0x%llx, len %lld\n",
	      dev_name(&flash->spi->dev), __func__, "at",
	      (long long)instr->addr, (long long)instr->len);

	/* sanity checks */
	if (instr->addr + instr->len > flash->mtd.size)
		return -EINVAL;
	div_u64_rem(instr->len, mtd->erasesize, &rem);
	if (rem)
		return -EINVAL;

	addr = instr->addr;
	len = instr->len;

	mutex_lock(&flash->lock);

	/* whole-chip erase? */
	if (len == flash->mtd.size) {
		if (erase_chip(flash)) {
			instr->state = MTD_ERASE_FAILED;
			mutex_unlock(&flash->lock);
			return -EIO;
		}

	/* REVISIT in some cases we could speed up erasing large regions
	 * by using OPCODE_SE instead of OPCODE_BE_4K.  We may have set up
	 * to use "small sector erase", but that's not always optimal.
	 */

	/* "sector"-at-a-time erase */
	} else {
		while (len) {
			if (erase_sector(flash, addr)) {
				instr->state = MTD_ERASE_FAILED;
				mutex_unlock(&flash->lock);
				return -EIO;
			}

			addr += mtd->erasesize;
			len -= mtd->erasesize;
		}
	}

	mutex_unlock(&flash->lock);

	instr->state = MTD_ERASE_DONE;
	mtd_erase_callback(instr);

	return 0;
}

/*
 * Read an address range from the flash chip.  The address range
 * may be any size provided it is within the physical boundaries.
 */
static int m25p80_read(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen, u_char *buf)
{
	struct m25p *flash = mtd_to_m25p(mtd);
	struct spi_transfer t[2];
	struct spi_message m;

	DEBUG(MTD_DEBUG_LEVEL2, "%s: %s %s 0x%08x, len %zd\n",
			dev_name(&flash->spi->dev), __func__, "from",
			(u32)from, len);

	/* sanity checks */
	if (!len)
		return 0;

	if (from + len > flash->mtd.size)
		return -EINVAL;

	spi_message_init(&m);
	memset(t, 0, (sizeof t));

	/* NOTE:
	 * OPCODE_FAST_READ (if available) is faster.
	 * Should add 1 byte DUMMY_BYTE.
	 */
	t[0].tx_buf = flash->command;
	t[0].len = m25p_cmdsz(flash) + FAST_READ_DUMMY_BYTE;
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = buf;
	t[1].len = len;
	spi_message_add_tail(&t[1], &m);

	/* Byte count starts at zero. */
	if (retlen)
		*retlen = 0;

	mutex_lock(&flash->lock);

	/* Wait till previous write/erase is done. */
	if (wait_till_ready(flash)) {
		/* REVISIT status return?? */
		mutex_unlock(&flash->lock);
		return 1;
	}

	/* FIXME switch to OPCODE_FAST_READ.  It's required for higher
	 * clocks; and at this writing, every chip this driver handles
	 * supports that opcode.
	 */

	/* Set up the write data buffer. */
	flash->command[0] = OPCODE_READ;
	m25p_addr2cmd(flash, from, flash->command);

	spi_sync(flash->spi, &m);

	*retlen = m.actual_length - m25p_cmdsz(flash) - FAST_READ_DUMMY_BYTE;

	mutex_unlock(&flash->lock);

	return 0;
}

/*
 * Write an address range to the flash chip.  Data must be written in
 * FLASH_PAGESIZE chunks.  The address range may be any size provided
 * it is within the physical boundaries.
 */
static int m25p80_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const u_char *buf)
{
	struct m25p *flash = mtd_to_m25p(mtd);
	u32 page_offset, page_size;
	struct spi_transfer t[2];
	struct spi_message m;

	DEBUG(MTD_DEBUG_LEVEL2, "%s: %s %s 0x%08x, len %zd\n",
			dev_name(&flash->spi->dev), __func__, "to",
			(u32)to, len);

	if (retlen)
		*retlen = 0;

	/* sanity checks */
	if (!len)
		return(0);

	if (to + len > flash->mtd.size)
		return -EINVAL;

	spi_message_init(&m);
	memset(t, 0, (sizeof t));

	t[0].tx_buf = flash->command;
	t[0].len = m25p_cmdsz(flash);
	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = buf;
	spi_message_add_tail(&t[1], &m);

	mutex_lock(&flash->lock);

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash)) {
		mutex_unlock(&flash->lock);
		return 1;
	}

	write_enable(flash);

	/* Set up the opcode in the write buffer. */
	flash->command[0] = OPCODE_PP;
	m25p_addr2cmd(flash, to, flash->command);

	page_offset = to & (flash->page_size - 1);

	/* do all the bytes fit onto one page? */
	if (page_offset + len <= flash->page_size) {
		t[1].len = len;

		spi_sync(flash->spi, &m);

		*retlen = m.actual_length - m25p_cmdsz(flash);
	} else {
		u32 i;

		/* the size of data remaining on the first page */
		page_size = flash->page_size - page_offset;

		t[1].len = page_size;
		spi_sync(flash->spi, &m);

		*retlen = m.actual_length - m25p_cmdsz(flash);

		/* write everything in flash->page_size chunks */
		for (i = page_size; i < len; i += page_size) {
			page_size = len - i;
			if (page_size > flash->page_size)
				page_size = flash->page_size;

			/* write the next page to flash */
			m25p_addr2cmd(flash, to + i, flash->command);

			t[1].tx_buf = buf + i;
			t[1].len = page_size;

			wait_till_ready(flash);

			write_enable(flash);

			spi_sync(flash->spi, &m);

			if (retlen)
				*retlen += m.actual_length - m25p_cmdsz(flash);
		}
	}

	mutex_unlock(&flash->lock);

	return 0;
}
//#define UBOOT_SPI_BOOST
#ifdef UBOOT_SPI_BOOST
/*==========================================================================================*/
/* try to import u-boot code source  */
/*==========================================================================================*/
#include <mach/lpc178x.h>
#include <mach/platform.h>
#include <mach/clock.h>
#include <mach/iomux.h>
#include <mach/spi.h>
#include <linux/io.h>

#define CMD_SST_WREN		0x06	/* Write Enable */
#define CMD_SST_WRDI		0x04	/* Write Disable */
#define CMD_SST_RDSR		0x05	/* Read Status Register */
#define CMD_SST_WRSR		0x01	/* Write Status Register */
#define CMD_SST_READ		0x03	/* Read Data Bytes */
#define CMD_SST_FAST_READ	0x0b	/* Read Data Bytes at Higher Speed */
#define CMD_SST_BP		0x02	/* Byte Program */
#define CMD_SST_AAI_WP		0xAD	/* Auto Address Increment Word Program */
#define CMD_SST_SE		0x20	/* Sector Erase */

#define SST_SR_WIP		(1 << 0)	/* Write-in-Progress */
#define SST_SR_WEL		(1 << 1)	/* Write enable */
#define SST_SR_BP0		(1 << 2)	/* Block Protection 0 */
#define SST_SR_BP1		(1 << 3)	/* Block Protection 1 */
#define SST_SR_BP2		(1 << 4)	/* Block Protection 2 */
#define SST_SR_AAI		(1 << 6)	/* Addressing mode */
#define SST_SR_BPL		(1 << 7)	/* BP bits lock */
/* SPI mode flags */
#define	SPI_CPHA	0x01			/* clock phase */
#define	SPI_CPOL	0x02			/* clock polarity */
#define	SPI_MODE_0	(0|0)			/* (original MicroWire) */
#define	SPI_MODE_1	(0|SPI_CPHA)
#define	SPI_MODE_2	(SPI_CPOL|0)
#define	SPI_MODE_3	(SPI_CPOL|SPI_CPHA)
#define	SPI_CS_HIGH	0x04			/* CS active high */
#define	SPI_LSB_FIRST	0x08			/* per-word bits-on-wire */
#define	SPI_3WIRE	0x10			/* SI/SO signals shared */
#define	SPI_LOOP	0x20			/* loopback mode */

/* SPI transfer flags */
#define SPI_XFER_BEGIN	0x01			/* Assert CS before transfer */
#define SPI_XFER_END	0x02			/* Deassert CS after transfer */
/* SSP registers mapping */
struct pl022 {
       u32     ssp_cr0;        /* 0x000 */
       u32     ssp_cr1;        /* 0x004 */
       u32     ssp_dr;         /* 0x008 */
       u32     ssp_sr;         /* 0x00c */
       u32     ssp_cpsr;       /* 0x010 */
       u32     ssp_imsc;       /* 0x014 */
       u32     ssp_ris;        /* 0x018 */
       u32     ssp_mis;        /* 0x01c */
       u32     ssp_icr;        /* 0x020 */
       u32     ssp_dmacr;      /* 0x024 */
       u8      reserved_1[0x080 - 0x028];
       u32     ssp_itcr;       /* 0x080 */
       u32     ssp_itip;       /* 0x084 */
       u32     ssp_itop;       /* 0x088 */
       u32     ssp_tdr;        /* 0x08c */
       u8      reserved_2[0xFE0 - 0x090];
       u32     ssp_pid0;       /* 0xfe0 */
       u32     ssp_pid1;       /* 0xfe4 */
       u32     ssp_pid2;       /* 0xfe8 */
       u32     ssp_pid3;       /* 0xfec */
       u32     ssp_cid0;       /* 0xff0 */
       u32     ssp_cid1;       /* 0xff4 */
       u32     ssp_cid2;       /* 0xff8 */
       u32     ssp_cid3;       /* 0xffc */
};

/* SSP Control Register 0  - SSP_CR0 */
#define SSP_CR0_SPO            (0x1 << 6)
#define SSP_CR0_SPH            (0x1 << 7)
#define SSP_CR0_8BIT_MODE      (0x07)
#define SSP_SCR_MAX            (0xFF)
#define SSP_SCR_SHFT           8

/* SSP Control Register 0  - SSP_CR1 */
#define SSP_CR1_MASK_SSE       (0x1 << 1)

#define SSP_CPSR_MAX           (0xFE)

/* SSP Status Register - SSP_SR */
#define SSP_SR_MASK_TFE                (0x1 << 0) /* Transmit FIFO empty */
#define SSP_SR_MASK_TNF                (0x1 << 1) /* Transmit FIFO not full */
#define SSP_SR_MASK_RNE                (0x1 << 2) /* Receive FIFO not empty */
#define SSP_SR_MASK_RFF                (0x1 << 3) /* Receive FIFO full */
#define SSP_SR_MASK_BSY                (0x1 << 4) /* Busy Flag */

struct spi_slave {
	unsigned int	bus;
	unsigned int	cs;
};

struct spi_flash {
	struct spi_slave *spi;

	const char	*name;

	//u32		size;
/*
	int		(*read)(struct spi_flash *flash, u32 offset,
				size_t len, void *buf);
	int		(*write)(struct spi_flash *flash, u32 offset,
				size_t len, const void *buf);
	int		(*erase)(struct spi_flash *flash, u32 offset,
				size_t len);
*/
};

struct pl022_spi_slave {
       struct spi_slave slave;
       void __iomem	 *regs;
       unsigned int freq;
};

#define debug(x,...)


/**
 * container_of - cast a member of a structure out to the containing structure
 * @ptr:	the pointer to the member.
 * @type:	the type of the container struct this is embedded in.
 * @member:	the name of the member within the struct.
 *
 */
#define uboot_container_of(ptr, type, member) ({			\
	const typeof( ((type *)0)->member ) *__mptr = (ptr);	\
	(type *)( (char *)__mptr - offsetof(type,member) );})

static inline struct pl022_spi_slave *to_pl022_spi(struct spi_slave *slave)
{
       return uboot_container_of(slave, struct pl022_spi_slave, slave);
}

struct pl022_spi_slave *spi_alloc_slave(unsigned int bus, unsigned int cs)
{
	struct pl022_spi_slave *s;
	s =kmalloc(sizeof(struct pl022_spi_slave), GFP_KERNEL);
	if (!s) {
		return NULL;
	}
	s->slave.bus=bus;
	s->slave.cs=cs;
	return s;
}

#define SPI_FLASH_CS_GRP__LPC178X_EVAL_FLASH	0
#define SPI_FLASH_CS_PIN__LPC178X_EVAL_FLASH	5
void spi_cs_activate(struct spi_slave *slave)
{
	// FLASH SPI CS
		lpc178x_gpio_out(SPI_FLASH_CS_GRP__LPC178X_EVAL_FLASH,
					SPI_FLASH_CS_PIN__LPC178X_EVAL_FLASH, 0);
}

void spi_cs_deactivate(struct spi_slave *slave)
{
	// FLASH SPI CS
		lpc178x_gpio_out(SPI_FLASH_CS_GRP__LPC178X_EVAL_FLASH,
					SPI_FLASH_CS_PIN__LPC178X_EVAL_FLASH, 1);
}

int spi_cs_is_valid(unsigned int bus, unsigned int cs)
{
       return 1;
}

#define SSP_CR1(r)	(r + 0x004)
int spi_claim_bus(struct spi_slave *slave)
{
       struct pl022_spi_slave *ps = to_pl022_spi(slave);
       struct pl022 __iomem *pl022 = (struct pl022 *)ps->regs;
       int cr1;

       /* Enable the SPI hardware */
       //setbits_le32(&pl022->ssp_cr1, SSP_CR1_MASK_SSE);
      // pl022 = ioremap((struct pl022 *)ps->regs, 0xFFF);
       //release_mem_region(pl022, 0xFFF);
       //request_mem_region(0x40030000, 0xFFF,"m25p80" );

/*
       cr1 = readl(&pl022->ssp_cr1);
       cr1 |= SSP_CR1_MASK_SSE;
       writel(cr1, &pl022->ssp_cr1);
*/
       writew((readw(SSP_CR1(ps->regs)) &
       			(~SSP_CR1_MASK_SSE)), SSP_CR1(ps->regs));
//pr_err("after spi_claim_bus");
       return 0;
}

void spi_release_bus(struct spi_slave *slave)
{
       struct pl022_spi_slave *ps = to_pl022_spi(slave);
       struct pl022 *pl022 = (struct pl022 *)ps->regs;

       /* Disable the SPI hardware */
       writel(0x0, &pl022->ssp_cr1);
}

int spi_xfer(struct spi_slave *slave, unsigned int bitlen,
               const void *dout, void *din, unsigned long flags)
{
       struct pl022_spi_slave *ps = to_pl022_spi(slave);
       struct pl022 *pl022 = (struct pl022 *)ps->regs;
       u32             len_tx = 0, len_rx = 0, len;
       u32             ret = 0;
       const u8        *txp = dout;
       u8              *rxp = din, value;

       if (bitlen == 0)
               /* Finish any previously submitted transfers */
               goto out;

       /*
        * TODO: The controller can do non-multiple-of-8 bit
        * transfers, but this driver currently doesn't support it.
        *
        * It's also not clear how such transfers are supposed to be
        * represented as a stream of bytes...this is a limitation of
        * the current SPI interface.
        */
       if (bitlen % 8) {
               ret = -1;

               /* Errors always terminate an ongoing transfer */
               flags |= SPI_XFER_END;
               goto out;
       }

       len = bitlen / 8;

       if (flags & SPI_XFER_BEGIN)
               spi_cs_activate(slave);

       while (len_tx < len) {
               if (readl(&pl022->ssp_sr) & SSP_SR_MASK_TNF) {
                       value = (txp != NULL) ? *txp++ : 0;
                       //printf("spi write=%02X\n",value);
                       writel(value, &pl022->ssp_dr);
                       len_tx++;
               }

               if (readl(&pl022->ssp_sr) & SSP_SR_MASK_RNE) {
                       value = readl(&pl022->ssp_dr);
                       //printf("spi write&read=%02X\n",value);
                       if (rxp)
                               *rxp++ = value;
                       len_rx++;
               }
       }

       unsigned long timeout = jiffies + msecs_to_jiffies(500);
       while (len_rx < len_tx) {
    	   	   if (time_after(jiffies, timeout)) {
    	   				goto out;
    	   			}
               if (readl(&pl022->ssp_sr) & SSP_SR_MASK_RNE) {
                       value = readl(&pl022->ssp_dr);
                       //printf("spi read=%02X\n",value);
                       if (rxp)
                               *rxp++ = value;
                       len_rx++;
               }
       }

out:
       if (flags & SPI_XFER_END)
               spi_cs_deactivate(slave);

       return ret;
}

int spi_flash_cmd(struct spi_slave *spi, u8 cmd, void *response, size_t len)
{
	unsigned long flags = SPI_XFER_BEGIN;
	int ret;

	if (len == 0)
		flags |= SPI_XFER_END;

	ret = spi_xfer(spi, 8, &cmd, NULL, flags);
	if (ret) {
		debug("SF: Failed to send command %02x: %d\n", cmd, ret);
		return ret;
	}

	if (len) {
		ret = spi_xfer(spi, len * 8, NULL, response, SPI_XFER_END);
		if (ret)
			debug("SF: Failed to read response (%zu bytes): %d\n",
					len, ret);
	}

	return ret;
}

static int
sst_disable_writing(struct spi_flash *flash)
{
	int ret = spi_flash_cmd(flash->spi, CMD_SST_WRDI, NULL, 0);
	if (ret)
		debug("SF: Disabling Write failed\n");
	return ret;
}

static int
sst_enable_writing(struct spi_flash *flash)
{
	int ret = spi_flash_cmd(flash->spi, CMD_SST_WREN, NULL, 0);
	if (ret)
		debug("SF: Enabling Write failed\n");
	return ret;
}



#define CONFIG_SYS_SPI_BASE 0x40030000
#define CONFIG_SYS_SPI_CLK  120000000
#define CONFIG_SYS_HZ			1000
#define SPI_FLASH_PROG_TIMEOUT		(2 * CONFIG_SYS_HZ)

int b_spi_init=0;
struct spi_flash myflash;

void spi_init(void)
{
	b_spi_init=1;
}

/*
 * ARM PL022 exists in different 'flavors'.
 * This drivers currently support the standard variant (0x00041022), that has a
 * 16bit wide and 8 locations deep TX/RX FIFO.
 */
static int pl022_is_supported(struct pl022_spi_slave *ps)
{
	return 1;
}

struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
                       unsigned int max_hz, unsigned int mode)
{
       struct pl022_spi_slave *ps;
       struct pl022 *pl022;
       u16 scr = 1, prescaler, cr0 = 0, cpsr = 0;

       //printf("spi_setup_slave(bus=%d, cs=%d,hz=%d,mode=%d)\n",bus,cs,max_hz,mode);

       if (!b_spi_init)
    	   spi_init();

       if (!spi_cs_is_valid(bus, cs))
               return NULL;

       ps = spi_alloc_slave(bus, cs);
       if (!ps)
               return NULL;

       ps->freq = max_hz;

       switch (bus) {
       case 0:
               ps->regs = (void *)CONFIG_SYS_SPI_BASE;
               break;
       default:
               kfree(ps);
               return NULL;
       }

       pl022 = (struct pl022 *)ps->regs;
       /* Check the PL022 version */
       if (!pl022_is_supported(ps)) {
               kfree(ps);
               return NULL;
       }

       /* Set requested polarity and 8bit mode */
       cr0 = SSP_CR0_8BIT_MODE;
       cr0 |= (mode & SPI_CPHA) ? SSP_CR0_SPH : 0;
       cr0 |= (mode & SPI_CPOL) ? SSP_CR0_SPO : 0;

       writel(cr0, &pl022->ssp_cr0);

       /* Program the SSPClk frequency */
       prescaler = CONFIG_SYS_SPI_CLK / ps->freq;
       pr_err("prescaler=(%d/%d) => %d\n",CONFIG_SYS_SPI_CLK , ps->freq,prescaler);
       if (prescaler <= 0xFF) {
               cpsr = prescaler;
       } else {
               for (scr = 1; scr <= SSP_SCR_MAX; scr++) {
                       if (!(prescaler % scr)) {
                               cpsr = prescaler / scr;
                               if (cpsr <= SSP_CPSR_MAX)
                                       break;
                       }
               }

               if (scr > SSP_SCR_MAX) {
                       scr = SSP_SCR_MAX;
                       cpsr = prescaler / scr;
                       cpsr &= SSP_CPSR_MAX;
               }
       }

       if (cpsr & 0x1)
               cpsr++;
       pr_err("SPI CPSR=%d scr=%d\n",cpsr,scr);
       writel(cpsr, &pl022->ssp_cpsr);
       cr0 = readl(&pl022->ssp_cr0);
       writel(cr0 | (scr - 1) << SSP_SCR_SHFT, &pl022->ssp_cr0);
       cr0 = readl(&pl022->ssp_cr0);
       pr_err("SPI CR0=0x%X \n",cr0);

       request_mem_region(0x40030000, 0xFFF,"m25p80" );
       return &ps->slave;
}

static void
uboot_init()
{
	struct spi_slave *s;
	s=spi_setup_slave(0,0,24000000, 3);
	myflash.spi=s;
	return;
}

static int
sst_wait_ready(struct spi_flash *flash, unsigned long timeout)
{
	struct spi_slave *spi = flash->spi;
	unsigned long timebase;
	int ret;
	u8 byte = CMD_SST_RDSR;

	ret = spi_xfer(spi, sizeof(byte) * 8, &byte, NULL, SPI_XFER_BEGIN);
	if (ret) {
		debug("SF: Failed to send command %02x: %d\n", byte, ret);
		return ret;
	}

	timebase = jiffies;
	do {
		ret = spi_xfer(spi, sizeof(byte) * 8, NULL, &byte, 0);
		if (ret)
			break;

		if ((byte & SST_SR_WIP) == 0)
			break;

	} while (jiffies< timebase+ msecs_to_jiffies(timeout));

	spi_xfer(spi, 0, NULL, NULL, SPI_XFER_END);

	if (!ret && (byte & SST_SR_WIP) != 0)
		ret = -1;

	if (ret)
		debug("SF: sst wait for ready timed out\n");
	return ret;
}

int spi_flash_cmd_write(struct spi_slave *spi, const u8 *cmd, size_t cmd_len,
		const void *data, size_t data_len)
{
	unsigned long flags = SPI_XFER_BEGIN;
	int ret;

	if (data_len == 0)
		flags |= SPI_XFER_END;

	ret = spi_xfer(spi, cmd_len * 8, cmd, NULL, flags);
	if (ret) {
		debug("SF: Failed to send read command (%zu bytes): %d\n",
				cmd_len, ret);
	} else if (data_len != 0) {
		ret = spi_xfer(spi, data_len * 8, data, NULL, SPI_XFER_END);
		if (ret)
			debug("SF: Failed to read %zu bytes of data: %d\n",
					data_len, ret);
	}

	return ret;
}


static int
sst_byte_write(struct spi_flash *flash, u32 offset, const void *buf)
{
	int ret;
	u8 cmd[4] = {
		CMD_SST_BP,
		offset >> 16,
		offset >> 8,
		offset,
	};

	debug("BP[%02x]: 0x%p => cmd = { 0x%02x 0x%06x }\n",
		spi_w8r8(flash->spi, CMD_SST_RDSR), buf, cmd[0], offset);

	ret = sst_enable_writing(flash);
	if (ret)
		return ret;

	ret = spi_flash_cmd_write(flash->spi, cmd, sizeof(cmd), buf, 1);
	if (ret)
		return ret;

	return sst_wait_ready(flash, SPI_FLASH_PROG_TIMEOUT);
}


static int
uboot_sst_write(struct spi_flash *flash, u32 offset, size_t len, const void *buf)
{
	size_t actual, cmd_len;
	int ret;
	u8 cmd[4];

	//printf("#");

	ret = spi_claim_bus(flash->spi);
	if (ret) {
		debug("SF: Unable to claim SPI bus\n");
		return ret;
	}

	/* If the data is not word aligned, write out leading single byte */
	actual = offset % 2;
	if (actual) {
		ret = sst_byte_write(flash, offset, buf);
		if (ret)
			goto done;
	}
	offset += actual;

	ret = sst_enable_writing(flash);
	if (ret)
		goto done;

	cmd_len = 4;
	cmd[0] = CMD_SST_AAI_WP;
	cmd[1] = offset >> 16;
	cmd[2] = offset >> 8;
	cmd[3] = offset;

	for (; actual < len - 1; actual += 2) {
		debug("WP[%02x]: 0x%p => cmd = { 0x%02x 0x%06x }\n",
		     spi_w8r8(flash->spi, CMD_SST_RDSR), buf + actual, cmd[0],
		     offset);

		/*if (actual%20480==0)
		{
			printf("#");
		}*/


		ret = spi_flash_cmd_write(flash->spi, cmd, cmd_len,
		                          buf + actual, 2);
		if (ret) {
			debug("SF: sst word program failed\n");
			break;
		}

		ret = sst_wait_ready(flash, SPI_FLASH_PROG_TIMEOUT);
		if (ret)
			break;

		cmd_len = 1;
		offset += 2;

	}

	//printf("\n");

	if (!ret)
		ret = sst_disable_writing(flash);

	/* If there is a single trailing byte, write it out */
	if (!ret && actual != len)
		ret = sst_byte_write(flash, offset, buf + actual);

 done:
	debug("SF: sst: program %s %zu bytes @ 0x%zx\n",
	      ret ? "failure" : "success", len, offset - actual);

	spi_release_bus(flash->spi);
	return ret;
}

static int sst_write(struct mtd_info *mtd, loff_t to, size_t len,
		size_t *retlen, const u_char *buf)
{
	struct m25p *flash = mtd_to_m25p(mtd);
	int ret;

	if (retlen)
		*retlen = 0;

	/* sanity checks */
	if (!len)
		return 0;

	if (to + len > flash->mtd.size)
		return -EINVAL;

	mutex_lock(&flash->lock);
	/* Wait until finished previous write command. */
	ret = wait_till_ready(flash);
	if (ret)
		goto time_out;

	//write_enable(flash);

	ret=uboot_sst_write(&myflash, to, len, buf);
	*retlen=(ret==0)?len:0;

	//write_disable(flash);
	//ret = wait_till_ready(flash);
	//if (ret)
	//	goto time_out;

time_out:
	mutex_unlock(&flash->lock);
	return ret;
}

#else
static int sst_write(struct mtd_info *mtd, loff_t to, size_t len,
		size_t *retlen, const u_char *buf)
{
	struct m25p *flash = mtd_to_m25p(mtd);
	struct spi_transfer t[8];
	struct spi_message m;
	size_t actual;
	int cmd_sz, ret/*,l*/;

	if (retlen)
		*retlen = 0;

	/* sanity checks */
	if (!len)
		return 0;

	if (to + len > flash->mtd.size)
		return -EINVAL;

	spi_message_init(&m);
	memset(t, 0, (sizeof t));

	t[0].tx_buf = flash->command;
	t[0].len = m25p_cmdsz(flash);
	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = buf;
	spi_message_add_tail(&t[1], &m);

	mutex_lock(&flash->lock);

	/* Wait until finished previous write command. */
	ret = wait_till_ready(flash);
	if (ret)
		goto time_out;

	write_enable(flash);

	actual = to % 2;
	/* Start write from odd address. */
	if (actual) {
		flash->command[0] = OPCODE_BP;
		m25p_addr2cmd(flash, to, flash->command);

		/* write one byte. */
		t[1].len = 1;
		spi_sync(flash->spi, &m);
		ret = wait_till_ready(flash);
		if (ret)
			goto time_out;
		*retlen += m.actual_length - m25p_cmdsz(flash);
	}
	to += actual;

	flash->command[0] = OPCODE_AAI_WP;
	m25p_addr2cmd(flash, to, flash->command);
	/* Write out most of the data here. */
	cmd_sz = m25p_cmdsz(flash);

#if 0
	l=len-*retlen;
	if (l/8>=1)
	{
		//pr_err("x8- len=%d to=%ld\n",len,to);
		struct spi_message m2;
		int j;

		spi_message_init(&m2);

		spi_message_add_tail(&t[0], &m2);
		spi_message_add_tail(&t[1], &m2);
		spi_message_add_tail(&t[2], &m2);
		spi_message_add_tail(&t[3], &m2);
		spi_message_add_tail(&t[4], &m2);
		spi_message_add_tail(&t[5], &m2);
		spi_message_add_tail(&t[6], &m2);
		spi_message_add_tail(&t[7], &m2);

		t[1].len = 2;

		t[2].len = 1;
		t[2].tx_buf = flash->command;

		t[3].len = 2;

		t[4].len = 1;
		t[4].tx_buf = flash->command;

		t[5].len = 2;

		t[6].len = 1;
		t[6].tx_buf = flash->command;

		t[7].len = 2;

		//original code
		for (j=0; j<l/8; j++)
		{
			t[0].len = cmd_sz;
			/* write two bytes. */
			t[1].tx_buf = buf + actual;
			/* write two bytes. */

			t[3].tx_buf = buf + actual + 2;

			t[5].tx_buf = buf + actual + 4;

			t[7].tx_buf = buf + actual + 6;

			spi_sync(flash->spi, &m2);
			*retlen += m2.actual_length - cmd_sz - 3;
			cmd_sz = 1;
			to += 8;
			actual += 8;
		}
	}
	l=len-*retlen;
	if (l/4>=1)
	{
		//pr_err("x4- len=%d to=%ld\n",len,to);
		struct spi_message m2;
		int j=0;
		spi_message_init(&m2);

		spi_message_add_tail(&t[0], &m2);
		spi_message_add_tail(&t[1], &m2);
		spi_message_add_tail(&t[2], &m2);
		t[1].len = 2;

		t[2].len = 1;
		t[2].tx_buf = flash->command;

		t[3].len = 2;

		spi_message_add_tail(&t[3], &m2);
		//original code
		for (; j<l/4;j++) {
			t[0].len = cmd_sz;
			/* write two bytes. */
			t[1].tx_buf = buf + actual;
			/* write two bytes. */

			t[3].tx_buf = buf + actual + 2;

			spi_sync(flash->spi, &m2);
			*retlen += m2.actual_length - cmd_sz - 1;
			cmd_sz = 1;
			to += 4;
			actual += 4;
		}
	}
	l=len-*retlen;
	if (l/2>=1)
	{
		int j=0;
		//original code
		for (; j < l/2; j++)
		{
			t[0].len = cmd_sz;
			/* write two bytes. */
			t[1].len = 2;
			t[1].tx_buf = buf + actual;

			spi_sync(flash->spi, &m);
	#if 0
			// wait 200ms per call but the previous spi_sync is also 190ms
			// so we don't need to wait it while the spec of the flash is only 10 micro sec
			ret = wait_till_ready(flash);
			if (ret)
				goto time_out;
	#endif
			*retlen += m.actual_length - cmd_sz;
			cmd_sz = 1;
			to += 2;
			actual += 2;
		}
	}
#endif

	for (; actual < len - 1; actual += 2) {
		t[0].len = cmd_sz;
		/* write two bytes. */
		t[1].len = 2;
		t[1].tx_buf = buf + actual;

		spi_sync(flash->spi, &m);
		//spi_write(flash->spi,t[0].tx_buf,t[0].len);
		//spi_write(flash->spi,t[1].tx_buf,t[1].len);
		//m.actual_length=t[0].len+t[1].len;
#if 0
		// wait 200ms per call but the previous spi_sync is also 190ms
		// so we don't need to wait it while the spec of the flash is only 10 micro sec
		ret = wait_till_ready(flash);
		if (ret)
			goto time_out;
#endif
		*retlen += m.actual_length - cmd_sz;
		cmd_sz = 1;
		to += 2;
	}
	write_disable(flash);
#if 0
	// wait 200ms per call but the previous spi_sync is also 190ms
	// so we don't need to wait it while the spec of the flash is only 10 micro sec
	ret = wait_till_ready(flash);
	if (ret)
		goto time_out;
#endif

	/* Write out trailing byte if it exists. */
	if (actual != len) {
		write_enable(flash);
		flash->command[0] = OPCODE_BP;
		m25p_addr2cmd(flash, to, flash->command);
		t[0].len = m25p_cmdsz(flash);
		t[1].len = 1;
		t[1].tx_buf = buf + actual;

		spi_sync(flash->spi, &m);
#if 0
		// wait 200ms per call but the previous spi_sync is also 190ms
		// so we don't need to wait it while the spec of the flash is only 10 micro sec
		ret = wait_till_ready(flash);
		if (ret)
			goto time_out;
#endif
		*retlen += m.actual_length - m25p_cmdsz(flash);
		write_disable(flash);
	}

time_out:
	mutex_unlock(&flash->lock);
	return ret;
}
#endif

/****************************************************************************/

/*
 * SPI device driver setup and teardown
 */

struct flash_info {
	/* JEDEC id zero means "no ID" (most older chips); otherwise it has
	 * a high byte of zero plus three data bytes: the manufacturer id,
	 * then a two byte device id.
	 */
	u32		jedec_id;
	u16             ext_id;

	/* The size listed here is what works with OPCODE_SE, which isn't
	 * necessarily called a "sector" by the vendor.
	 */
	unsigned	sector_size;
	u16		n_sectors;

	u16		page_size;
	u16		addr_width;

	u16		flags;
#define	SECT_4K		0x01		/* OPCODE_BE_4K works uniformly */
#define	M25P_NO_ERASE	0x02		/* No erase command needed */
};

#define INFO(_jedec_id, _ext_id, _sector_size, _n_sectors, _flags)	\
	((kernel_ulong_t)&(struct flash_info) {				\
		.jedec_id = (_jedec_id),				\
		.ext_id = (_ext_id),					\
		.sector_size = (_sector_size),				\
		.n_sectors = (_n_sectors),				\
		.page_size = 256,					\
		.addr_width = 3,					\
		.flags = (_flags),					\
	})

#define CAT25_INFO(_sector_size, _n_sectors, _page_size, _addr_width)	\
	((kernel_ulong_t)&(struct flash_info) {				\
		.sector_size = (_sector_size),				\
		.n_sectors = (_n_sectors),				\
		.page_size = (_page_size),				\
		.addr_width = (_addr_width),				\
		.flags = M25P_NO_ERASE,					\
	})

/* NOTE: double check command sets and memory organization when you add
 * more flash chips.  This current list focusses on newer chips, which
 * have been converging on command sets which including JEDEC ID.
 */
static const struct spi_device_id m25p_ids[] = {
	/* Atmel -- some are (confusingly) marketed as "DataFlash" */
	{ "at25fs010",  INFO(0x1f6601, 0, 32 * 1024,   4, SECT_4K) },
	{ "at25fs040",  INFO(0x1f6604, 0, 64 * 1024,   8, SECT_4K) },

	{ "at25df041a", INFO(0x1f4401, 0, 64 * 1024,   8, SECT_4K) },
	{ "at25df641",  INFO(0x1f4800, 0, 64 * 1024, 128, SECT_4K) },

	{ "at26f004",   INFO(0x1f0400, 0, 64 * 1024,  8, SECT_4K) },
	{ "at26df081a", INFO(0x1f4501, 0, 64 * 1024, 16, SECT_4K) },
	{ "at26df161a", INFO(0x1f4601, 0, 64 * 1024, 32, SECT_4K) },
	{ "at26df321",  INFO(0x1f4701, 0, 64 * 1024, 64, SECT_4K) },

	/* Macronix */
	{ "mx25l4005a",  INFO(0xc22013, 0, 64 * 1024,   8, SECT_4K) },
	{ "mx25l3205d",  INFO(0xc22016, 0, 64 * 1024,  64, 0) },
	{ "mx25l6405d",  INFO(0xc22017, 0, 64 * 1024, 128, 0) },
	{ "mx25l12805d", INFO(0xc22018, 0, 64 * 1024, 256, 0) },
	{ "mx25l12855e", INFO(0xc22618, 0, 64 * 1024, 256, 0) },

	/* Spansion -- single (large) sector size only, at least
	 * for the chips listed here (without boot sectors).
	 */
	{ "s25sl004a",  INFO(0x010212,      0,  64 * 1024,   8, 0) },
	{ "s25sl008a",  INFO(0x010213,      0,  64 * 1024,  16, 0) },
	{ "s25sl016a",  INFO(0x010214,      0,  64 * 1024,  32, 0) },
	{ "s25sl032a",  INFO(0x010215,      0,  64 * 1024,  64, 0) },
	{ "s25sl064a",  INFO(0x010216,      0,  64 * 1024, 128, 0) },
	{ "s25sl12800", INFO(0x012018, 0x0300, 256 * 1024,  64, 0) },
	{ "s25sl12801", INFO(0x012018, 0x0301,  64 * 1024, 256, 0) },
	{ "s25fl129p0", INFO(0x012018, 0x4d00, 256 * 1024,  64, 0) },
	{ "s25fl129p1", INFO(0x012018, 0x4d01,  64 * 1024, 256, 0) },

	/* SST -- large erase sizes are "overlays", "sectors" are 4K */
	{ "sst25vf040b", INFO(0xbf258d, 0, 64 * 1024,  8, SECT_4K) },
	{ "sst25vf080b", INFO(0xbf258e, 0, 64 * 1024, 16, SECT_4K) },
	{ "sst25vf016b", INFO(0xbf2541, 0, 64 * 1024, 32, SECT_4K) },
	{ "sst25vf032b", INFO(0xbf254a, 0, 64 * 1024, 64, SECT_4K) },
	{ "sst25wf512",  INFO(0xbf2501, 0, 64 * 1024,  1, SECT_4K) },
	{ "sst25wf010",  INFO(0xbf2502, 0, 64 * 1024,  2, SECT_4K) },
	{ "sst25wf020",  INFO(0xbf2503, 0, 64 * 1024,  4, SECT_4K) },
	{ "sst25wf040",  INFO(0xbf2504, 0, 64 * 1024,  8, SECT_4K) },
	{ "sst26vf032b", INFO(0xbf2602, 0, 64 * 1024, 64, SECT_4K) },

	/* ST Microelectronics -- newer production may have feature updates */
	{ "m25p05",  INFO(0x202010,  0,  32 * 1024,   2, 0) },
	{ "m25p10",  INFO(0x202011,  0,  32 * 1024,   4, 0) },
	{ "m25p20",  INFO(0x202012,  0,  64 * 1024,   4, 0) },
	{ "m25p40",  INFO(0x202013,  0,  64 * 1024,   8, 0) },
	{ "m25p80",  INFO(0x202014,  0,  64 * 1024,  16, 0) },
	{ "m25p16",  INFO(0x202015,  0,  64 * 1024,  32, 0) },
	{ "m25p32",  INFO(0x202016,  0,  64 * 1024,  64, 0) },
	{ "m25p64",  INFO(0x202017,  0,  64 * 1024, 128, 0) },
	{ "m25p128", INFO(0x202018,  0, 256 * 1024,  64, 0) },

	{ "m45pe10", INFO(0x204011,  0, 64 * 1024,    2, 0) },
	{ "m45pe80", INFO(0x204014,  0, 64 * 1024,   16, 0) },
	{ "m45pe16", INFO(0x204015,  0, 64 * 1024,   32, 0) },

	{ "m25pe80", INFO(0x208014,  0, 64 * 1024, 16,       0) },
	{ "m25pe16", INFO(0x208015,  0, 64 * 1024, 32, SECT_4K) },

	/* Winbond -- w25x "blocks" are 64K, "sectors" are 4KiB */
	{ "w25x10", INFO(0xef3011, 0, 64 * 1024,  2,  SECT_4K) },
	{ "w25x20", INFO(0xef3012, 0, 64 * 1024,  4,  SECT_4K) },
	{ "w25x40", INFO(0xef3013, 0, 64 * 1024,  8,  SECT_4K) },
	{ "w25x80", INFO(0xef3014, 0, 64 * 1024,  16, SECT_4K) },
	{ "w25x16", INFO(0xef3015, 0, 64 * 1024,  32, SECT_4K) },
	{ "w25x32", INFO(0xef3016, 0, 64 * 1024,  64, SECT_4K) },
	{ "w25x64", INFO(0xef3017, 0, 64 * 1024, 128, SECT_4K) },

	/* Catalyst / On Semiconductor -- non-JEDEC */
	{ "cat25c11", CAT25_INFO(  16, 8, 16, 1) },
	{ "cat25c03", CAT25_INFO(  32, 8, 16, 2) },
	{ "cat25c09", CAT25_INFO( 128, 8, 32, 2) },
	{ "cat25c17", CAT25_INFO( 256, 8, 32, 2) },
	{ "cat25128", CAT25_INFO(2048, 8, 64, 2) },
	{ },
};
MODULE_DEVICE_TABLE(spi, m25p_ids);

static const struct spi_device_id *__devinit jedec_probe(struct spi_device *spi)
{
	int			tmp;
	u8			code = OPCODE_RDID;
	u8			id[5];
	u32			jedec;
	u16                     ext_jedec;
	struct flash_info	*info;

	/* JEDEC also defines an optional "extended device information"
	 * string for after vendor-specific data, after the three bytes
	 * we use here.  Supporting some chips might require using it.
	 */
	tmp = spi_write_then_read(spi, &code, 1, id, 5);
	if (tmp < 0) {
		DEBUG(MTD_DEBUG_LEVEL0, "%s: error %d reading JEDEC ID\n",
			dev_name(&spi->dev), tmp);
		return NULL;
	}
	jedec = id[0];
	jedec = jedec << 8;
	jedec |= id[1];
	jedec = jedec << 8;
	jedec |= id[2];

	/*
	 * Some chips (like Numonyx M25P80) have JEDEC and non-JEDEC variants,
	 * which depend on technology process. Officially RDID command doesn't
	 * exist for non-JEDEC chips, but for compatibility they return ID 0.
	 */
	if (jedec == 0)
		return NULL;

	ext_jedec = id[3] << 8 | id[4];

	for (tmp = 0; tmp < ARRAY_SIZE(m25p_ids) - 1; tmp++) {
		info = (void *)m25p_ids[tmp].driver_data;
		if (info->jedec_id == jedec) {
			if (info->ext_id != 0 && info->ext_id != ext_jedec)
				continue;
			return &m25p_ids[tmp];
		}
	}
	return NULL;
}


/*
 * board specific setup should have ensured the SPI clock used here
 * matches what the READ command supports, at least until this driver
 * understands FAST_READ (for clocks over 25 MHz).
 */
static int __devinit m25p_probe(struct spi_device *spi)
{
	const struct spi_device_id	*id = spi_get_device_id(spi);
	struct flash_platform_data	*data;
	struct m25p			*flash;
	struct flash_info		*info;
	unsigned			i;

	/* Platform data helps sort out which chip type we have, as
	 * well as how this board partitions it.  If we don't have
	 * a chip ID, try the JEDEC id commands; they'll work for most
	 * newer chips, even if we don't recognize the particular chip.
	 */
	data = spi->dev.platform_data;
	if (data && data->type) {
		const struct spi_device_id *plat_id;

		for (i = 0; i < ARRAY_SIZE(m25p_ids) - 1; i++) {
			plat_id = &m25p_ids[i];
			if (strcmp(data->type, plat_id->name))
				continue;
			break;
		}

		if (plat_id)
			id = plat_id;
		else
			dev_warn(&spi->dev, "unrecognized id %s\n", data->type);
	}

	info = (void *)id->driver_data;

	if (info->jedec_id) {
		const struct spi_device_id *jid;

		jid = jedec_probe(spi);
		if (!jid) {
			dev_info(&spi->dev, "non-JEDEC variant of %s\n",
				 id->name);
		} else if (jid != id) {
			/*
			 * JEDEC knows better, so overwrite platform ID. We
			 * can't trust partitions any longer, but we'll let
			 * mtd apply them anyway, since some partitions may be
			 * marked read-only, and we don't want to lose that
			 * information, even if it's not 100% accurate.
			 */
			dev_warn(&spi->dev, "found %s, expected %s\n",
				 jid->name, id->name);
			id = jid;
			info = (void *)jid->driver_data;
		}
	}

	flash = kzalloc(sizeof *flash, GFP_KERNEL);
	if (!flash)
		return -ENOMEM;
	flash->command = kmalloc(MAX_CMD_SIZE + FAST_READ_DUMMY_BYTE, GFP_KERNEL);
	if (!flash->command) {
		kfree(flash);
		return -ENOMEM;
	}

	flash->spi = spi;
	mutex_init(&flash->lock);
	dev_set_drvdata(&spi->dev, flash);

	/*
	 * Atmel and SST serial flash tend to power
	 * up with the software protection bits set
	 */

	if (info->jedec_id >> 16 == 0x1f ||
	    info->jedec_id >> 16 == 0xbf) {
		write_enable(flash);
		write_sr(flash, 0);
	}

	if (data && data->name)
		flash->mtd.name = data->name;
	else
		flash->mtd.name = dev_name(&spi->dev);

	flash->mtd.type = MTD_NORFLASH;
	flash->mtd.writesize = 1;
	flash->mtd.flags = MTD_CAP_NORFLASH;
	flash->mtd.size = info->sector_size * info->n_sectors;
	flash->mtd.erase = m25p80_erase;
	flash->mtd.read = m25p80_read;

	/* sst flash chips use AAI word program */
	if (info->jedec_id >> 16 == 0xbf)
		flash->mtd.write = sst_write;
	else
		flash->mtd.write = m25p80_write;

	/* prefer "small sector" erase if possible */
	if (info->flags & SECT_4K) {
		flash->erase_opcode = OPCODE_BE_4K;
		flash->mtd.erasesize = 4096;
	} else {
		flash->erase_opcode = OPCODE_SE;
		flash->mtd.erasesize = info->sector_size;
	}

	if (info->flags & M25P_NO_ERASE)
		flash->mtd.flags |= MTD_NO_ERASE;

	flash->mtd.dev.parent = &spi->dev;
	flash->page_size = info->page_size;
	flash->addr_width = info->addr_width;

	dev_info(&spi->dev, "%s (%lld Kbytes)\n", id->name,
			(long long)flash->mtd.size >> 10);

	DEBUG(MTD_DEBUG_LEVEL2,
		"mtd .name = %s, .size = 0x%llx (%lldMiB) "
			".erasesize = 0x%.8x (%uKiB) .numeraseregions = %d\n",
		flash->mtd.name,
		(long long)flash->mtd.size, (long long)(flash->mtd.size >> 20),
		flash->mtd.erasesize, flash->mtd.erasesize / 1024,
		flash->mtd.numeraseregions);

	if (flash->mtd.numeraseregions)
		for (i = 0; i < flash->mtd.numeraseregions; i++)
			DEBUG(MTD_DEBUG_LEVEL2,
				"mtd.eraseregions[%d] = { .offset = 0x%llx, "
				".erasesize = 0x%.8x (%uKiB), "
				".numblocks = %d }\n",
				i, (long long)flash->mtd.eraseregions[i].offset,
				flash->mtd.eraseregions[i].erasesize,
				flash->mtd.eraseregions[i].erasesize / 1024,
				flash->mtd.eraseregions[i].numblocks);


	/* partitions should match sector boundaries; and it may be good to
	 * use readonly partitions for writeprotected sectors (BP2..BP0).
	 */
	if (mtd_has_partitions()) {
		struct mtd_partition	*parts = NULL;
		int			nr_parts = 0;

		if (mtd_has_cmdlinepart()) {
			static const char *part_probes[]
					= { "cmdlinepart", NULL, };

			nr_parts = parse_mtd_partitions(&flash->mtd,
					part_probes, &parts, 0);
		}

		if (nr_parts <= 0 && data && data->parts) {
			parts = data->parts;
			nr_parts = data->nr_parts;
		}

		if (nr_parts > 0) {
			for (i = 0; i < nr_parts; i++) {
				DEBUG(MTD_DEBUG_LEVEL2, "partitions[%d] = "
					"{.name = %s, .offset = 0x%llx, "
						".size = 0x%llx (%lldKiB) }\n",
					i, parts[i].name,
					(long long)parts[i].offset,
					(long long)parts[i].size,
					(long long)(parts[i].size >> 10));
			}
			flash->partitioned = 1;
			return add_mtd_partitions(&flash->mtd, parts, nr_parts);
		}
	} else if (data && data->nr_parts)
		dev_warn(&spi->dev, "ignoring %d default partitions on %s\n",
				data->nr_parts, data->name);


	return add_mtd_device(&flash->mtd) == 1 ? -ENODEV : 0;
}


static int __devexit m25p_remove(struct spi_device *spi)
{
	struct m25p	*flash = dev_get_drvdata(&spi->dev);
	int		status;

	/* Clean up MTD stuff. */
	if (mtd_has_partitions() && flash->partitioned)
		status = del_mtd_partitions(&flash->mtd);
	else
		status = del_mtd_device(&flash->mtd);
	if (status == 0) {
		kfree(flash->command);
		kfree(flash);
	}
	return 0;
}


static struct spi_driver m25p80_driver = {
	.driver = {
		.name	= "m25p80",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.id_table	= m25p_ids,
	.probe	= m25p_probe,
	.remove	= __devexit_p(m25p_remove),

	/* REVISIT: many of these chips have deep power-down modes, which
	 * should clearly be entered on suspend() to minimize power use.
	 * And also when they're otherwise idle...
	 */
};


static int __init m25p80_init(void)
{
#ifdef UBOOT_SPI_BOOST
	uboot_init();
#endif
	return spi_register_driver(&m25p80_driver);
}


static void __exit m25p80_exit(void)
{
	spi_unregister_driver(&m25p80_driver);
}


module_init(m25p80_init);
module_exit(m25p80_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mike Lavender");
MODULE_DESCRIPTION("MTD SPI driver for ST M25Pxx flash chips");
