/*
 * m95m01_r.c
 *
 * Driver for M95M01_R SPI Flash chips
 *
 * Author: Antonino Calderone <antonino.calderone@ericsson.com>
 *
 * Based on m25p80.c
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/sched.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

/* Erases can take up to 3 seconds! */
#define MAX_READY_WAIT_JIFFIES   msecs_to_jiffies(3000)

#define M95M01_R_CMD_WRSR      0x01   /* Write status register */
#define M95M01_R_CMD_WRDI      0x04   /* Write disable */
#define M95M01_R_CMD_RDSR      0x05   /* Read status register */
#define M95M01_R_CMD_WREN      0x06   /* Write enable */
#define M95M01_R_CMD_READ      0x03   /* High speed read */
#define M95M01_R_CMD_WRITE     0x02   /* High speed read */

#define M95M01_R_STATUS_BUSY  (1 << 0)   /* Chip is busy */
#define M95M01_R_STATUS_WREN  (1 << 1)   /* Write enabled */
#define M95M01_R_STATUS_BP0   (1 << 2)   /* Block protection 0 */
#define M95M01_R_STATUS_BP1   (1 << 3)   /* Block protection 1 */


#ifdef DEBUG 
  #define DBGPRINT(string, args...) \
    printk(KERN_INFO "%s: " string, __FUNCTION__, ##args) 
#else
  #define DBGPRINT(string, args...)
#endif

struct m95m01_r_flash {
   struct spi_device *spi;
   struct mutex      lock;
   struct mtd_info   mtd;
   int    partitioned;
};

struct flash_info {
   const char    *name;
   unsigned      page_size;
   unsigned      nr_pages;
};

#define to_m95m01_r_flash(x) container_of(x, struct m95m01_r_flash, mtd)

static struct flash_info __initdata m95m01_r_flash_info[] = {
   {"m95m01_r", 64, 512},  //256 Kbit with 64Byte per pages = 512 pages
};

static int m95m01_r_status(struct m95m01_r_flash *flash, int *status)
{
   struct spi_message m;
   struct spi_transfer t;
   unsigned char cmd_resp[2];
   int err;

   spi_message_init(&m);
   memset(&t, 0, sizeof(struct spi_transfer));

   cmd_resp[0] = M95M01_R_CMD_RDSR;
   cmd_resp[1] = 0xff;
   t.tx_buf = cmd_resp;
   t.rx_buf = cmd_resp;
   t.len = sizeof(cmd_resp);
   spi_message_add_tail(&t, &m);
   err = spi_sync(flash->spi, &m);

   if (err < 0)
   {
      DBGPRINT("error=%i\n", err );
      return err;
   }

   *status = cmd_resp[1];

   return 0;
}

static int m95m01_r_write_enable(struct m95m01_r_flash *flash, int enable)
{
   unsigned char command[2];
   int status, err;

   command[0] = enable ? M95M01_R_CMD_WREN : M95M01_R_CMD_WRDI;

   DBGPRINT("+: %s\n", enable ? "enable":"disable" );

   err = spi_write(flash->spi, command, 1);
   if (err)
   {
      DBGPRINT("-: %s -> ERROR\n", enable ? "enable":"disable" );
      return err;
   }

   DBGPRINT("-: %s -> OK\n", enable ? "enable":"disable" );

   return 0;
}

static int m95m01_r_wait_till_ready(struct m95m01_r_flash *flash)
{
   unsigned long deadline;
   int status, err;

   deadline = jiffies + MAX_READY_WAIT_JIFFIES;
   do {
      err = m95m01_r_status(flash, &status);
      if (err)
         return err;
      if (!(status & M95M01_R_STATUS_BUSY))
         return 0;

      cond_resched();
   } while (!time_after_eq(jiffies, deadline));

   DBGPRINT("TIMEOUT\n");

   return -ETIMEDOUT;
}

static int m95m01_r_erase(struct mtd_info *mtd, struct erase_info *instr)
{
   struct m95m01_r_flash *flash = to_m95m01_r_flash(mtd);

   /* Sanity checks */
   if (! flash)
   {
     DBGPRINT("container_of(...) returns NULL\n");
     return -EINVAL;
   }

   if (instr->addr + instr->len > flash->mtd.size)
      return -EINVAL;

   instr->state = MTD_ERASE_DONE;
   mtd_erase_callback(instr);

   DBGPRINT("OK\n");
   return 0;
}

static int m95m01_r_read(struct mtd_info *mtd, loff_t from, size_t len,
             size_t *retlen, unsigned char *buf)
{
   struct m95m01_r_flash *flash = to_m95m01_r_flash(mtd);
   struct spi_transfer transfer[2];
   struct spi_message message;
   unsigned char command[4];
   int ret;

   DBGPRINT("+\n");

   /* Sanity checking */

   if (! flash)
   {
     DBGPRINT("container_of(...) returns NULL\n");
     return -EINVAL;
   }

   if (len == 0)
      return 0;

   if (from + len > flash->mtd.size)
      return -EINVAL;

   if (retlen)
      *retlen = 0;

   spi_message_init(&message);
   memset(&transfer, 0, sizeof(transfer));

   command[0] = M95M01_R_CMD_READ;
   command[1] = from >> 16;
   command[2] = from >> 8;
   command[3] = from;

   transfer[0].tx_buf = command;
   transfer[0].len = sizeof(command);
   spi_message_add_tail(&transfer[0], &message);

   transfer[1].rx_buf = buf;
   transfer[1].len = len;
   spi_message_add_tail(&transfer[1], &message);

   mutex_lock(&flash->lock);

   /* Wait for previous write/erase to complete */
   ret = m95m01_r_wait_till_ready(flash);
   if (ret) {
      mutex_unlock(&flash->lock);
      return ret;
   }

   spi_sync(flash->spi, &message);

   if (retlen && message.actual_length > sizeof(command))
      *retlen += message.actual_length - sizeof(command);

   mutex_unlock(&flash->lock);

   DBGPRINT("OK\n");
   return 0;
}

static int m95m01_r_write(struct mtd_info *mtd, loff_t to, size_t len,
         size_t *retlen, const unsigned char *buf)
{
   struct m95m01_r_flash *flash = to_m95m01_r_flash(mtd);
   int i, ret, copied = 0;
   static unsigned char command[ 4096 ];

   DBGPRINT("+\n");

   /* Sanity checks */
   if (! flash)
   {
     DBGPRINT("container_of(...) returns NULL\n");
     return -EINVAL;
   }

   if (!len)
      return 0;

   if (to + len > flash->mtd.size)
      return -EINVAL;

   if (mtd->writesize > sizeof(command))
      return -EINVAL;

   if ((uint32_t)to % mtd->writesize)
      return -EINVAL;

   mutex_lock(&flash->lock);

   for (i = 0; i < len; i += mtd->writesize) 
   {
      ret = m95m01_r_wait_till_ready(flash);
      if (ret)
         goto out;

      ret = m95m01_r_write_enable(flash, 1);
      if (ret)
         goto out;

      /* Write the page */
      command[0] = M95M01_R_CMD_WRITE;
      command[1] = (to + i) >> 16;
      command[2] = (to + i) >> 8;
      command[3] = (to + i);
      memcpy ( &command[4], &buf[i], mtd->writesize );

      ret = spi_write(flash->spi, command, mtd->writesize + 4);

      if (ret < 0)
         goto out;

      copied += mtd->writesize;
   }

out:
   ret = m95m01_r_write_enable(flash, 0);

   if (retlen)
      *retlen = copied;

   mutex_unlock(&flash->lock);

   DBGPRINT("ret = %i\n", ret);
   return ret;
}

static int __devinit m95m01_r_probe(struct spi_device *spi)
{
   struct flash_info *flash_info;
   struct m95m01_r_flash *flash;
   struct flash_platform_data *data;
   int ret, i;

   DBGPRINT("+\n");

   flash_info = &m95m01_r_flash_info[0];
   if (!flash_info)
      return -ENODEV;

   flash = kzalloc(sizeof(struct m95m01_r_flash), GFP_KERNEL);
   if (!flash)
      return -ENOMEM;

   flash->spi = spi;
   mutex_init(&flash->lock);
   dev_set_drvdata(&spi->dev, flash);

   data = spi->dev.platform_data;
   if (data && data->name)
      flash->mtd.name = data->name;
   else
      flash->mtd.name = dev_name(&spi->dev);

   flash->mtd.type        = MTD_NORFLASH;
   flash->mtd.flags       = MTD_CAP_NORFLASH;
   flash->mtd.erasesize   = flash_info->page_size;
   flash->mtd.writesize   = flash_info->page_size;
   flash->mtd.size        = flash_info->page_size * flash_info->nr_pages;
   flash->mtd.erase       = m95m01_r_erase;
   flash->mtd.read        = m95m01_r_read;
   flash->mtd.write       = m95m01_r_write;

   dev_info(&spi->dev, "%s (%lld KiB)\n", flash_info->name,
       (long long)flash->mtd.size >> 10);

   DBGPRINT(
         "mtd .name = %s, .size = 0x%llx (%lldMiB) "
         ".erasesize = 0x%.8x .numeraseregions = %d\n",
         flash->mtd.name,
         (long long)flash->mtd.size, (long long)(flash->mtd.size >> 20),
         flash_info->page_size,
         flash->mtd.numeraseregions);

   if (mtd_has_partitions()) {
      struct mtd_partition *parts = NULL;
      int nr_parts = 0;

      if (mtd_has_cmdlinepart()) {
         static const char *part_probes[] =
            {"cmdlinepart", NULL};

         nr_parts = parse_mtd_partitions(&flash->mtd,
                     part_probes,
                     &parts, 0);
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
         return add_mtd_partitions(&flash->mtd,
                    parts, nr_parts);
      }

   } else if (data && data->nr_parts) {
      dev_warn(&spi->dev, "ignoring %d default partitions on %s\n",
          data->nr_parts, data->name);
   }

   ret = add_mtd_device(&flash->mtd);
   if (ret == 1) {
      kfree(flash);
      dev_set_drvdata(&spi->dev, NULL);
      return -ENODEV;
   }

   DBGPRINT("OK\n");

   return 0;
}

static int __exit m95m01_r_remove(struct spi_device *spi)
{
   struct m95m01_r_flash *flash = dev_get_drvdata(&spi->dev);
   int ret;

   if (mtd_has_partitions() && flash->partitioned)
      ret = del_mtd_partitions(&flash->mtd);
   else
      ret = del_mtd_device(&flash->mtd);
   if (ret == 0)
      kfree(flash);

   DBGPRINT("ret = %i\n", ret);
   return ret;
}

static struct spi_driver m95m01_r_driver = {
   .driver = {
      .name  = "m95m01_r",
      .bus   = &spi_bus_type,
      .owner = THIS_MODULE,
   },
   .probe    = m95m01_r_probe,
   .remove   = __exit_p(m95m01_r_remove),
};

static int __init m95m01_r_init(void)
{
   DBGPRINT("\n");
   return spi_register_driver(&m95m01_r_driver);
}

static void __exit m95m01_r_exit(void)
{
   DBGPRINT("\n");
   spi_unregister_driver(&m95m01_r_driver);
}

module_init(m95m01_r_init);
module_exit(m95m01_r_exit);

MODULE_DESCRIPTION("MTD SPI driver for M95M01_R Flash chips");
MODULE_AUTHOR("Antonino Calderone <antonino.calderone@ericsson.com>");
MODULE_LICENSE("GPL");
