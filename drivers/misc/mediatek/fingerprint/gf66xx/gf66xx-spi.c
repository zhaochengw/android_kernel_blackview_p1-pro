/*
 *Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/poll.h>
#include <linux/delay.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <mach/mt_gpio.h>
#include <mach/mt_spi.h>
#include <cust_gpio_usage.h>

#include <asm/uaccess.h>
#include <linux/ktime.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/completion.h>
#include "gf66xx-spi.h"

#define GF66XX_PID              "GF66XX"
#define SPI_DEV_NAME            "gf66xx"
#define GF66XX_INPUT_DEV_NAME   "gf66xx_input_key"
/*device name after register in charater*/
#define DEV_NAME "gf66xx-spi"

#define CHRD_DRIVER_NAME        "gf66xx"
#define CLASS_NAME              "gf66xx-spi"
#define SPIDEV_MAJOR			0//155	///* assigned */
#define N_SPI_MINORS			32	///* ... up to 256 */
#define FW_LENGTH               (42*1024)
#define CFG_UPATE               0
#define FW_UPDATE               0
#define ESD_PROTECT             1

#if FW_UPDATE
static unsigned char  GF66XX_FW[] =
{
        #include "gf66xx_fw.i"
};
#endif

static DECLARE_BITMAP(minors, N_SPI_MINORS);

/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY)

#define     MTK_SPI_ALIGN_BITS  (9)
#define     MTK_SPI_ALIGN_MASK  ((0x1 << MTK_SPI_ALIGN_BITS) - 1)
/**************************debug******************************/
#define GF66XX_DEBUG   1
#undef GF66XX_DEBUG

#ifdef GF66XX_DEBUG
#define   gf66xx_dbg(fmt, args...) do{ \
					pr_warn("gf66xx:" fmt, ##args);\
				}while(0)
#define FUNC_ENTRY()  pr_warn("gf66xx:%s, entry\n", __func__)
#define FUNC_EXIT()  pr_warn("gf66xx:%s, exit\n", __func__)
#else
#define gf66xx_dbg(fmt, args...)
#define FUNC_ENTRY()
#define FUNC_EXIT()
#endif

/***********************************************************
gf66xx config for GF66XX_2030(6BC2).BIN 
************************************************************/
u8 gf66xx_config[] = {
    0x42,0xe6,0xe6,0xe4,0x0c,0x90,0x4c,0x04,0x00,0x32,0x28,0xc8,0xc8,0xe4,0x0c,0x90,
    0x4d,0x02,0x80,0x05,0x00,0xa0,0x0d,0x00,0x14,0x19,0x0f,0x0f,0x0f,0xb3,0x3f,0xb3,
    0x33,0x00,0x90,0x01,0x20,0xc0,0x03,0x2d,0x96,0x0f,0x14,0x14,0x40,0x47,0x4e,0x56,
    0x60,0x6b,0x1e,0x14,0x14,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x10,0x00,0xa0,0x0f,0xd0,0x07,0x20,
    0x03,0x62,0x0e,0x3a,0x15,0x22,0xd0,0x00,0x00,0x60,0x03,0x00,0x00,0x02,0x04,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,
    0x00,0xff,0x0f,0x00,0x00,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x72,0x01
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static struct gf66xx_dev gf66xx;
static struct task_struct *gf66xx_irq_thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int suspend_flag = 0;
static int irq_flag = 0;
static int thread_flag = 0;
static int major_gf66xx = 0;

/*************************data stream***********************
*	FRAME NO  | RING_H  | RING_L  |  DATA STREAM | CHECKSUM |
*     1B      |   1B    |  1B     |    2048B     |  2B      |
************************************************************/
static unsigned bufsiz = 4*(2048+5);
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

static struct mt_chip_conf spi_conf_mt65xx = {
	.setuptime = 60,
	.holdtime = 60,
	.high_time = 15, //15--3m   25--2m   50--1m  [ 100--0.5m]
	.low_time = 15,
	.cs_idletime = 8,
	.ulthgh_thrsh = 0,
	
	.cpol = 0,
	.cpha = 0,
	
	.rx_mlsb = 1,
	.tx_mlsb = 1,
	.tx_endian = 0,
	.rx_endian = 0,
	//.com_mod = FIFO_TRANSFER,
	.com_mod = DMA_TRANSFER,
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};

typedef enum {
	SPEED_500KHZ=0,
	SPEED_1MHZ,
	SPEED_2MHZ,
	SPEED_3MHZ,
	SPEED_4MHZ,
	SPEED_6MHZ,
	SPEED_8MHZ,
	SPEED_KEEP,
	SPEED_UNSUPPORTED
}SPI_SPEED;

static void gf66xx_spi_set_mode(struct spi_device *spi, SPI_SPEED speed, int flag)
{
	struct mt_chip_conf *mcc = &spi_conf_mt65xx;
	if(flag == 0) {
		mcc->com_mod = FIFO_TRANSFER;
	} else {
		mcc->com_mod = DMA_TRANSFER;
	}

	switch(speed)
	{
		case SPEED_500KHZ:
			mcc->high_time = 120;
			mcc->low_time = 120;
			break;
		case SPEED_1MHZ:
			mcc->high_time = 60;
			mcc->low_time = 60;
			break;
		case SPEED_2MHZ:
			mcc->high_time = 30;
			mcc->low_time = 30;
			break;
		case SPEED_3MHZ:
			mcc->high_time = 20;
			mcc->low_time = 20;
			break;
		case SPEED_4MHZ:
			mcc->high_time = 15;
			mcc->low_time = 15;
			break;

		case SPEED_6MHZ:
			mcc->high_time = 10;
			mcc->low_time = 10;
			break;
		case SPEED_8MHZ:
		    mcc->high_time = 8;
			mcc->low_time = 8;
			break;
		case SPEED_KEEP:
		case SPEED_UNSUPPORTED:
			break;
	}
	if(spi_setup(spi) < 0){
		pr_warn("gf66xx:Failed to set spi.\n");
	}
}

/**********************************************************
*Message format:
*	write cmd   |  ADDR_H |ADDR_L  |  data stream  |
*    1B         |   1B    |  1B    |  length       |
*
* write buffer length should be 1 + 1 + 1 + data_length
***********************************************************/
int gf66xx_spi_write_bytes(struct gf66xx_dev *gf66xx_dev,
				u16 addr, u32 data_len, u8 *tx_buf)
{
	struct spi_message msg;
	struct spi_transfer *xfer;
	u32  package_num = (data_len + GF66XX_WDATA_OFFSET)>>MTK_SPI_ALIGN_BITS;
	u32  reminder = (data_len + GF66XX_WDATA_OFFSET) & MTK_SPI_ALIGN_MASK;
	u8 *reminder_buf = NULL;
	u8   twice = 0;
	int ret = 0;

	/*set spi mode.*/
	if((data_len + GF66XX_WDATA_OFFSET) > 32) {
		gf66xx_spi_set_mode(gf66xx_dev->spi, SPEED_KEEP, 1); //DMA
	} else {
		gf66xx_spi_set_mode(gf66xx_dev->spi, SPEED_KEEP, 0); //FIFO
	}

	if((package_num > 0) && (reminder != 0)) {
		twice = 1;
		/*copy the reminder data to temporarity buffer.*/
		reminder_buf = kzalloc(reminder + GF66XX_WDATA_OFFSET, GFP_KERNEL);
		if(reminder_buf == NULL ) {
			pr_err("gf66xx:No memory for exter data.\n");
			return -ENOMEM;
		}
		memcpy(reminder_buf + GF66XX_WDATA_OFFSET, tx_buf + GF66XX_WDATA_OFFSET+data_len - reminder, reminder);
        gf66xx_dbg("gf66xx:w-reminder:0x%x-0x%x,0x%x\n", reminder_buf[GF66XX_WDATA_OFFSET],reminder_buf[GF66XX_WDATA_OFFSET+1],
                reminder_buf[GF66XX_WDATA_OFFSET + 2]);
		xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
	} else {
		twice = 0;
		xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
	}
	if( xfer == NULL){
		gf66xx_dbg("gf66xx:No memory for command.\n");
		if(reminder_buf != NULL)
			kfree(reminder_buf);
		return -ENOMEM;
	}

	//gf66xx_dbg("gf66xx:write twice = %d. data_len = %d, package_num = %d, reminder = %d\n", twice, data_len, package_num, reminder);
	/*if the length is not align with 1024. Need 2 transfer at least.*/
	spi_message_init(&msg);
	tx_buf[0] = GF66XX_W;
	tx_buf[1] = (u8)((addr >> 8)&0xFF);
	tx_buf[2] = (u8)(addr & 0xFF);
	xfer[0].tx_buf = tx_buf;
//    xfer[0].delay_usecs = 5;
	if(twice == 1) {
		xfer[0].len = package_num << 10;
		spi_message_add_tail(&xfer[0], &msg);
		
		addr += data_len - reminder;
		reminder_buf[0] = GF66XX_W;
		reminder_buf[1] = (u8)((addr >> 8)&0xFF);
		reminder_buf[2] = (u8)(addr & 0xFF);
		xfer[1].tx_buf = reminder_buf;
		xfer[1].len = reminder + GF66XX_WDATA_OFFSET;
//        xfer[1].delay_usecs = 5;
		spi_message_add_tail(&xfer[1], &msg);
	} else {
		xfer[0].len = data_len + GF66XX_WDATA_OFFSET;
		spi_message_add_tail(&xfer[0], &msg);
	}

	ret = spi_sync(gf66xx_dev->spi, &msg);
	if(ret == 0) {
		if(twice == 1)
			ret = msg.actual_length - 2*GF66XX_WDATA_OFFSET;
		else
			ret = msg.actual_length - GF66XX_WDATA_OFFSET;
	} else 	{
		pr_warn("gf66xx:write async failed. ret = %d\n", ret);
	}

	if(xfer != NULL) {
		kfree(xfer);
		xfer = NULL;
	}
	if(reminder_buf != NULL) {
		kfree(reminder_buf);
		reminder_buf = NULL;
	}
	
	return ret;
}

/*************************************************************
*First message:
*	write cmd   |  ADDR_H |ADDR_L  |
*    1B         |   1B    |  1B    |
*Second message:
*	read cmd   |  data stream  |
*    1B        |   length    |
*
* read buffer length should be 1 + 1 + 1 + 1 + data_length
**************************************************************/
int gf66xx_spi_read_bytes(struct gf66xx_dev *gf66xx_dev,
				u16 addr, u32 data_len, u8 *rx_buf)
{
	struct spi_message msg;
	struct spi_transfer *xfer;
	u32  package_num = (data_len + 1)>>MTK_SPI_ALIGN_BITS;
	u32  reminder = (data_len + 1) & MTK_SPI_ALIGN_MASK;
	u8 *reminder_buf = NULL;
	u8   twice = 0;
	int ret = 0;
	
//	printk("--lantao-- gf66xx_spi_read_bytes . 20150410\n ");
	if((package_num > 0) && (reminder != 0)) {
		twice = 1;
		reminder_buf = kzalloc(reminder + 2*GF66XX_WDATA_OFFSET, GFP_KERNEL);
		if(reminder_buf == NULL ) {
			pr_err("No memory for exter data.\n");
//			printk("--lantao-- reminder_buf = NULL. 20150410\n ");
			return -ENOMEM;
		}
		xfer = kzalloc(sizeof(*xfer)*4, GFP_KERNEL);
	} else {
		twice = 0;
		xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
	}
	if( xfer == NULL){
//		printk("--lantao-- xfer = NULL. 20150410\n ");
		gf66xx_dbg("No memory for command.\n");
		if(reminder_buf != NULL)
			kfree(reminder_buf);
		return -ENOMEM;
	}
	/*set spi mode.*/
//	printk("--lantao-- data_len + GF66XX_RDATA_OFFSET = %d. 20150410\n ",data_len + GF66XX_RDATA_OFFSET);	
	if((data_len + GF66XX_RDATA_OFFSET) > 32) {
		gf66xx_spi_set_mode(gf66xx_dev->spi, SPEED_KEEP, 1); //DMA
	} else {
		gf66xx_spi_set_mode(gf66xx_dev->spi, SPEED_KEEP, 0); //FIFO
	}
	spi_message_init(&msg);
    /*send GF66XX command to device.*/
	rx_buf[0] = GF66XX_W;
	rx_buf[1] = (u8)((addr >> 8)&0xFF);
	rx_buf[2] = (u8)(addr & 0xFF);
	xfer[0].tx_buf = rx_buf;
	xfer[0].len = 3;
//	printk("--lantao-- rx_buf[0]=%d,rx_buf[1]=%d,rx_buf[2]=%d. 20150410\n ",rx_buf[0], rx_buf[1],rx_buf[2]);	
	spi_message_add_tail(&xfer[0], &msg);
    spi_sync(gf66xx_dev->spi, &msg);
    spi_message_init(&msg);

	/*if wanted to read data from GF66XX. 
	 *Should write Read command to device
	 *before read any data from device.
	 */
		//memset(rx_buf, 0xff, data_len);
	rx_buf[4] = GF66XX_R;
	xfer[1].tx_buf = &rx_buf[4];
	xfer[1].rx_buf = &rx_buf[4];
	if(twice == 1)
		xfer[1].len = (package_num << 10);
	else
		xfer[1].len = data_len + 1;
	spi_message_add_tail(&xfer[1], &msg);

	if(twice == 1) {
		addr += data_len - reminder;
		reminder_buf[0] = GF66XX_W;
		reminder_buf[1] = (u8)((addr >> 8)&0xFF);
		reminder_buf[2] = (u8)(addr & 0xFF);
		xfer[2].tx_buf = reminder_buf;
		xfer[2].len = 3;
		spi_message_add_tail(&xfer[2], &msg);
        
        spi_sync(gf66xx_dev->spi, &msg);
        spi_message_init(&msg);
		reminder_buf[4] = GF66XX_R;
		xfer[3].tx_buf = &reminder_buf[4];
		xfer[3].rx_buf = &reminder_buf[4];
		xfer[3].len = reminder + 1;
		spi_message_add_tail(&xfer[3], &msg);
	}
/*
    if(twice == 1) {
        xfer[3].delay_usecs = 5;
    } else {
        xfer[1].delay_usecs = 5;
    }
*/
	ret = spi_sync(gf66xx_dev->spi, &msg);
	if(ret == 0) {
		if(twice == 1) {
            gf66xx_dbg("gf66xx:reminder:0x%x:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n", reminder_buf[0], reminder_buf[1], 
                    reminder_buf[2], reminder_buf[3],reminder_buf[4],reminder_buf[5],reminder_buf[6],reminder_buf[7]);
			memcpy(rx_buf + GF66XX_RDATA_OFFSET + data_len - reminder, reminder_buf + GF66XX_RDATA_OFFSET, reminder);
			ret = data_len;//msg.actual_length - 1; //8 
		} else {
			ret = data_len;//msg.actual_length - 1; //4
		}
	}else {
        pr_warn("gf66xx: read failed. ret = %d\n", ret);
    }

	kfree(xfer);
	if(xfer != NULL)
		xfer = NULL;
	
	//gf66xx_dbg("gf66xx:read twice = %d, data_len = %d, package_num = %d, reminder = %d\n",twice, data_len, package_num, reminder);
	//gf66xx_dbg("gf66xx:data_len = %d, msg.actual_length = %d, ret = %d\n", data_len, msg.actual_length, ret);
	if(reminder_buf != NULL) { 
		kfree(reminder_buf); 
		reminder_buf = NULL; 
	}
	return ret;
}

static int gf66xx_spi_read_byte(struct gf66xx_dev *gf66xx_dev, u16 addr, u8 *value)
{
	int status = 0;
	mutex_lock(&gf66xx_dev->buf_lock);
	
	status = gf66xx_spi_read_bytes(gf66xx_dev, addr, 1, gf66xx_dev->buffer);
	*value = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
	//gf66xx_dbg("status = %d, value = 0x%x, buffer[4] = 0x%x, buffer[5] = %d\n",status, *value, 
	//		gf66xx_dev->buffer[4], gf66xx_dev->buffer[5]);
	mutex_unlock(&gf66xx_dev->buf_lock);
	return status;
}
static int gf66xx_spi_write_byte(struct gf66xx_dev *gf66xx_dev, u16 addr, u8 value)
{
	int status = 0;
	mutex_lock(&gf66xx_dev->buf_lock);
	gf66xx_dev->buffer[GF66XX_WDATA_OFFSET] = value;
	status = gf66xx_spi_write_bytes(gf66xx_dev, addr, 1, gf66xx_dev->buffer);
	mutex_unlock(&gf66xx_dev->buf_lock);
	return status;
}

/*-------------------------------------------------------------------------*/
/* Read-only message with current device setup */
static ssize_t
gf66xx_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct gf66xx_dev *gf66xx_dev = filp->private_data;
	ssize_t			status = 0;
	long int t1, t2;
	long int t3, t4;
	FUNC_ENTRY();
	if(count > bufsiz) {
		pr_warn("gf66xx:Max size for write buffer is %d\n", bufsiz);
		return -EMSGSIZE;
	}FUNC_EXIT();

	gf66xx_dev = filp->private_data;
	mutex_lock(&gf66xx_dev->buf_lock);
	gf66xx_spi_set_mode(gf66xx_dev->spi, SPEED_4MHZ, 0);
	t1 = ktime_to_us(ktime_get());
	status = gf66xx_spi_read_bytes(gf66xx_dev, GF66XX_BUFFER_DATA, count, gf66xx_dev->buffer);
	t2 = ktime_to_us(ktime_get());
	if(status > 0) {
		unsigned long missing = 0;
		t3 = ktime_to_us(ktime_get());
		missing = copy_to_user(buf, gf66xx_dev->buffer + GF66XX_RDATA_OFFSET, status);
		t4 = ktime_to_us(ktime_get());
//		gf66xx_dbg("gf66xx:spi use : %ld us, copy use : %ld us, status = %d, count = %d\n", (t2-t1), (t4-t3), status, count);
		if(missing == status)
			status = -EFAULT;
	} else {
		pr_err("Failed to read data from SPI device.\n");
		status = -EFAULT;
	}

	mutex_unlock(&gf66xx_dev->buf_lock);
	return status;
}

/* Write-only message with current device setup */
static ssize_t
gf66xx_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct gf66xx_dev *gf66xx_dev = filp->private_data;
	ssize_t			status = 0;
	FUNC_ENTRY();
	if(count > bufsiz) {
		pr_warn("Max size for write buffer is %d\n", bufsiz);
		return -EMSGSIZE;
	} 

	mutex_lock(&gf66xx_dev->buf_lock);
	gf66xx_spi_set_mode(gf66xx_dev->spi, SPEED_4MHZ, 0);
	status = copy_from_user(gf66xx_dev->buffer + GF66XX_WDATA_OFFSET, buf, count);
	if(status == 0) {
		status = gf66xx_spi_write_bytes(gf66xx_dev, 0x8000, count, gf66xx_dev->buffer);
	} else {
		pr_err("Failed to xfer data through SPI bus.\n");
		status = -EFAULT;
	}
	mutex_unlock(&gf66xx_dev->buf_lock);
	FUNC_EXIT();
	return status;
}

static long
gf66xx_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf66xx_dev *gf66xx_dev = NULL;
	struct gf66xx_ioc_transfer *ioc = NULL;
	int			err = 0;
	int 		retval = 0;
//	unsigned int cmd_32 = cmd & 0xffffffff;
	
	FUNC_ENTRY();
	if (_IOC_TYPE(cmd) != GF66XX_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	*/
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
						(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
						(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;
	
	//gf66xx_dev = (struct gf66xx_dev *)filp->private_data;
	gf66xx_dev = filp->private_data;
//	unsigned int cmd_32 = cmd & 0xffffffff;
	switch(cmd) {
	case GF66XX_IOC_CMD:
		ioc = kzalloc(sizeof(*ioc), GFP_KERNEL);
        if(ioc == NULL) {
            pr_err("Failed to allocate memory.\n");
            retval = -ENOMEM;
            break;
        }
		/*copy command data from user to kernel.*/
		if(copy_from_user(ioc, (struct gf66xx_ioc_transfer*)arg, sizeof(*ioc))){
			pr_err("Failed to copy command from user to kernel.\n");
			retval = -EFAULT;
            kfree(ioc);
			break;
		}
	    mutex_lock(&gf66xx_dev->buf_lock);
	    gf66xx_spi_set_mode(gf66xx_dev->spi, SPEED_1MHZ, 0);
		if(ioc->cmd == GF66XX_R) {
			/*if want to read data from hardware.*/
//					printk("gf66xx GF66XX_R ioc->cmd =0x%02X  ***2015032701.-lantao-\n", ioc->cmd);

			//gf66xx_dbg("Read data from 0x%02X, len = 0x%02X buf = 0x%08X\n", ioc->addr, ioc->len, (void __user*)(unsigned long)(ioc->buf));
			gf66xx_spi_read_bytes(gf66xx_dev, ioc->addr, ioc->len, gf66xx_dev->buffer);
			gf66xx_dbg("Read buffer[4]=0x%02X, buffer[5] = 0x%02X buffer[6] = 0x%02X\n", gf66xx_dev->buffer[4],
					gf66xx_dev->buffer[5], gf66xx_dev->buffer[6]);
	//		printk("Read ioc->addr =0x%02X  buffer[4]=0x%02X, buffer[5] = 0x%02X buffer[6] = 0x%02X\n", ioc->addr, gf66xx_dev->buffer[4],
	//				gf66xx_dev->buffer[5], gf66xx_dev->buffer[6]);
	/*bernard20150330*/		if(copy_to_user((void __user*)(unsigned long)(ioc->buf), gf66xx_dev->buffer + GF66XX_RDATA_OFFSET, ioc->len)) {
				pr_err("Failed to copy data from kernel to user.\n");
				retval = -EFAULT;
                kfree(ioc);
	            mutex_unlock(&gf66xx_dev->buf_lock);
				break;
			}
		} else if (ioc->cmd == GF66XX_W) {
			/*if want to read data from hardware.*/
//			gf66xx_dbg("gf66xx:Write data from 0x%x, len = 0x%x, ioc_buf:0x%x,0x%x,0x%x\n", ioc->addr, ioc->len,
//                    ioc->buf[ioc->len - 3], ioc->buf[ioc->len - 2], ioc->buf[ioc->len - 1]);
//					printk("gf66xx GF66XX_W ioc->cmd =0x%02X  ***2015032701.-lantao-\n", ioc->cmd);
			if(copy_from_user(gf66xx_dev->buffer + GF66XX_WDATA_OFFSET, (void __user*)(unsigned long)(ioc->buf), ioc->len)){
				pr_err("Failed to copy data from user to kernel.\n");
				retval = -EFAULT;
                kfree(ioc);
	            mutex_unlock(&gf66xx_dev->buf_lock);
				break;
			}
//					printk("gf66xx GF66XX_W ioc->addr =0x%04X, ioc->len =%d, gf66xx_dev->buffer[3] =0x%02X  ***2015032701.-lantao-\n",ioc->addr, 
//						ioc->len,gf66xx_dev->buffer[3]);

			gf66xx_spi_write_bytes(gf66xx_dev, ioc->addr, ioc->len, gf66xx_dev->buffer);
		} else {
			pr_warn("Error command for GF66XX.\n");
		}
		if(ioc != NULL) {
			kfree(ioc);
			ioc = NULL;
		}
	    mutex_unlock(&gf66xx_dev->buf_lock);
		break;
	case GF66XX_IOC_REINIT:
		gf66xx_irq_close(gf66xx_dev->spi->irq);
        gf66xx_hw_reset(gf66xx_dev, 60);
		gf66xx_irq_open(gf66xx_dev->spi->irq);
		break;
    case GF66XX_IOC_STOPTIMER:
#if ESD_PROTECT
        del_timer_sync(&gf66xx_dev->gf66xx_timer);
#endif
        break;
    case GF66XX_IOC_STARTTIMER:
#if ESD_PROTECT
        gf66xx_dev->gf66xx_timer.expires = jiffies + 2*HZ;
        add_timer(&gf66xx_dev->gf66xx_timer);
#endif
        break;
	default:
		pr_warn("gf66xx doesn't support this command(%d)\n", cmd);
		break;
	}
	FUNC_EXIT();
	return retval;
}

static unsigned int gf66xx_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct gf66xx_dev *gf66xx_dev = filp->private_data;
	u8 status = 0;
	gf66xx_spi_read_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, &status);
	if((status & GF66XX_BUF_STA_MASK) == GF66XX_BUF_STA_READY) {
		return (POLLIN|POLLRDNORM);
	} else {
		gf66xx_dbg("Poll no data.\n");
	}
	return 0;
}

static void gf66xx_irq(void)
{
#if 0
	struct gf66xx_dev *gf66xx_dev = &gf66xx;
			if(gf66xx_dev->async) {
				kill_fasync(&gf66xx_dev->async, SIGIO, POLL_IN);
			}
#else
    irq_flag = 1;
 //   pr_info("gf66xx: in irq, thread_flag = %d, suspend_flag = %d\n", thread_flag, suspend_flag);
    wake_up_interruptible(&waiter);
#endif
}

static int gf66xx_irq_work(void *data)
{
	struct gf66xx_dev* gf66xx_dev = &gf66xx;
    struct sched_param param = {.sched_priority = REG_RT_PRIO(1)};
	unsigned char status;
	unsigned char mode = 0x80;

    sched_setscheduler(current, SCHED_RR, &param);
    thread_flag = 0;
    do{

        gf66xx_dbg("gf66xx: wait event.\n");
        wait_event_interruptible(waiter, ((irq_flag!=0)&&(suspend_flag != 1)));
       
        gf66xx_irq_close(gf66xx_dev->spi->irq);
        irq_flag = 0;

    gf66xx_spi_read_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, &status);
	gf66xx_dbg("[bernard]gf66xx:read [0x%04X]status = 0x%02X\n", GF66XX_BUFFER_STATUS, status );

    if(!(status & GF66XX_BUF_STA_MASK)) {
        pr_info("gf66xx:BAD IRQ = 0x%x\n", status);
        gf66xx_irq_open(gf66xx_dev->spi->irq);
        continue;
    }

    if(!(status & (GF66XX_IMAGE_MASK | GF66XX_KEY_MASK))) {
        gf66xx_spi_write_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, (status & 0x7F));
        pr_info("gf66xx:Invalid IRQ = 0x%x\n", status);
        gf66xx_irq_open(gf66xx_dev->spi->irq);
        continue;
    }
	gf66xx_spi_read_byte(gf66xx_dev, GF66XX_MODE_STATUS, &mode);
	gf66xx_dbg("[bernard]gf66xx:status = 0x%02X,read [0x%04X] mode = 0x%02X.\n", status, GF66XX_MODE_STATUS, mode);
	switch(mode)
		{
		case GF66XX_FF_MODE:
            if((status & GF66XX_KEY_MASK) && (status & GF66XX_KEY_STA)){
                pr_info("gf66xx: wake device.\n");
			    gf66xx_spi_write_byte(gf66xx_dev, GF66XX_MODE_STATUS, 0x00);
			    input_report_key(gf66xx_dev->input, KEY_POWER, 1);
			    input_sync(gf66xx_dev->input);			
			    input_report_key(gf66xx_dev->input, KEY_POWER, 0);
			    input_sync(gf66xx_dev->input);
            } else {
                break;
            }
        
		case GF66XX_IMAGE_MODE:
			#ifdef GF66XX_FASYNC
			if(gf66xx_dev->async) {
	//			pr_info("gf66xx: GF66XX_IMAGE_MODE.\n");
				kill_fasync(&gf66xx_dev->async, SIGIO, POLL_IN);
			}
			#endif
			break;
		case GF66XX_KEY_MODE:
			gf66xx_dbg("gf66xx:Key mode: status = 0x%x\n", status);
			if  ((status & GF66XX_KEY_MASK) && (status & GF66XX_BUF_STA_MASK)) {
				input_report_key(gf66xx_dev->input, KEY_HOME, (status & GF66XX_KEY_STA)>>4);
				input_sync(gf66xx_dev->input);
			}
			gf66xx_spi_write_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, (status & 0x7F));
			break;
		case GF66XX_SLEEP_MODE:
			pr_warn("gf66xx:Should not happen in sleep mode.\n");
			break;
        case GF66XX_DEBUG_MODE:
            #ifdef GF66XX_FASYNC
			if(gf66xx_dev->async) {
				kill_fasync(&gf66xx_dev->async, SIGIO, POLL_IN);
			}
			#endif
            break;
		default:
			pr_warn("gf66xx:Unknown mode. mode = 0x%x\n", mode);
			break;
			
		}
            
        gf66xx_irq_open(gf66xx_dev->spi->irq);
    } while(!kthread_should_stop());
    thread_flag = 1;
    pr_info("gf66xx:thread finished.\n");
}

static long gf66xx_compat_ioctl(struct file *filp,unsigned int cmd,unsigned long arg)
{
	//compat_ptr() convert the 64 bit unsigned long type to 32 bit address.
	return gf66xx_ioctl(filp,cmd,(unsigned long)compat_ptr(arg));
}

static int gf66xx_open(struct inode *inode, struct file *filp)
{
	struct gf66xx_dev *gf66xx_dev;
	int			status = -ENXIO;

	FUNC_ENTRY();
	mutex_lock(&device_list_lock);

	list_for_each_entry(gf66xx_dev, &device_list, device_entry) {
		if(gf66xx_dev->devt == inode->i_rdev) {
			pr_info("gf66xx:device Found\n");
			status = 0;
			break;
		}
	}

	if(status == 0){
		mutex_lock(&gf66xx_dev->buf_lock);
		if( gf66xx_dev->buffer == NULL) {
			gf66xx_dev->buffer = (u8 *)__get_free_pages(GFP_KERNEL, get_order(bufsiz+2*GF66XX_RDATA_OFFSET));
			//kzalloc(bufsiz + 2*GF66XX_RDATA_OFFSET, GFP_KERNEL);
			if(gf66xx_dev->buffer == NULL) {
				dev_dbg(&gf66xx_dev->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
            pr_info("gf66xx: alloc memory.\n");
		}
		mutex_unlock(&gf66xx_dev->buf_lock);
		if(status == 0) {
			gf66xx_dev->users++;
			filp->private_data = gf66xx_dev;
			nonseekable_open(inode, filp);
			pr_info("gf66xx:Succeed to open device. irq = %d\n", gf66xx_dev->spi->irq);
            if(gf66xx_dev->users == 1)
                gf66xx_irq_open(gf66xx_dev->spi->irq);
		}
	} else {
		pr_info("gf66xx:No device for minor %d\n", iminor(inode));
	}

	mutex_unlock(&device_list_lock);
	FUNC_EXIT();
	return status;
}

#ifdef GF66XX_FASYNC
static int gf66xx_fasync(int fd, struct file *filp, int mode)
{
	struct gf66xx_dev *gf66xx_dev = filp->private_data;
	int ret;

	FUNC_ENTRY();
	ret = fasync_helper(fd, filp, mode, &gf66xx_dev->async);
	FUNC_EXIT();
	gf66xx_dbg("ret = %d\n", ret);
	return ret;
}
#endif

static int gf66xx_release(struct inode *inode, struct file *filp)
{
	struct gf66xx_dev *gf66xx_dev;
	int			status = 0;

	FUNC_ENTRY();
	mutex_lock(&device_list_lock);
	gf66xx_dev = filp->private_data;
	filp->private_data = NULL;

	/*last close??*/
	gf66xx_dev->users --;
	if(!gf66xx_dev->users) {
		
		gf66xx_dbg("disble_irq. irq = %d\n", gf66xx_dev->spi->irq);
		gf66xx_irq_close(gf66xx_dev->spi->irq);

        pr_info("gf66xx:release.\n");
//		kfree(gf66xx_dev->buffer);
//		gf66xx_dev->buffer = NULL;
	}
	mutex_unlock(&device_list_lock);
	FUNC_EXIT();
	return status;
}

static const struct file_operations gf66xx_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.write =	gf66xx_write,
	.read =		gf66xx_read,
	.unlocked_ioctl = gf66xx_ioctl,
	.compat_ioctl = gf66xx_compat_ioctl,
	.open =		gf66xx_open,
	.release =	gf66xx_release,
	.poll   = gf66xx_poll,
#ifdef GF66XX_FASYNC
	.fasync = gf66xx_fasync,
#endif
};

#if FW_UPDATE
static int isUpdate(struct gf66xx_dev *gf66xx_dev)
{
	unsigned char version[16];
	unsigned short ver_fw = 0;
	unsigned char* fw = GF66XX_FW;
	unsigned char fw_running = 0;
	unsigned short ver_file = 0;

	gf66xx_spi_read_bytes(gf66xx_dev, 0x41e4, 1, gf66xx_dev->buffer);
	fw_running = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
	if(fw_running == 0xbe) {
		/*firmware running*/
		ver_file = (*(fw+12))<<8 | (*(fw+13)); //get the fw version in the i file;
		/*In case we want to upgrade to a special firmware. Such as debug firmware.*/
		if(ver_file != 0x5a5a) {
			gf66xx_spi_read_bytes(gf66xx_dev,0x8000,16,gf66xx_dev->buffer);
			memcpy(version, gf66xx_dev->buffer + GF66XX_RDATA_OFFSET, 16);
            if(memcmp(version, GF66XX_PID, 6)) {
                //pr_info("version: 0x%x-0x%x-0x%x-0x%x-0x%x-0x%x\n", version[0], version[1],
                //     version[2], version[3], version[4], version[5]);
                    return 1;
            }
            if((version[7]>9) || (((version[8]&0xF0)>>4)>9) || ((version[8]&0x0F)>9)) {
                //pr_info("version: 7-0x%x; 8-0x%x\n", version[7], version[8]);
                return 1;
            }
			ver_fw = ((version[7]<<4)<<8) | (version[8]<<4); //get the current fw version
			if(ver_fw >= ver_file){
				/*If the running firmware is or ahead of the file's firmware. No need to do upgrade.*/
				return 0;
			}
		}
		pr_info("gf66xx:Current Ver: 0x%x, Upgrade to Ver: 0x%x\n", ver_fw, ver_file);
	}else {
		/*no firmware.*/
		pr_info("gf66xx:No running firmware. Value = 0x%x\n", fw_running);
	}
	return 1;
}

static int gf66xx_fw_update_init(struct gf66xx_dev *gf66xx_dev)
{
	u8 retry_cnt = 5;
	u8 value[2];
	while(retry_cnt--)
	{
        gf66xx_hw_reset(gf66xx_dev, 5);
		/*4.Hold SS51 and DSP(0x4180 == 0x0C)*/
		gf66xx_dev->buffer[GF66XX_WDATA_OFFSET] = 0x0c;
		gf66xx_spi_write_bytes(gf66xx_dev, 0x4180, 1, gf66xx_dev->buffer);
		gf66xx_spi_read_bytes(gf66xx_dev, 0x4180,1, gf66xx_dev->buffer);
		value[0] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
		gf66xx_spi_read_bytes(gf66xx_dev, 0x4030,1, gf66xx_dev->buffer);
		value[1] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
		pr_info("[info] %s hold SS51 and DSP,0x4180=0x%x,0x4030=0x%x,retry_cnt=%d !\n",__func__,value[0] ,value[1],retry_cnt);
		if (value[0] == 0x0C)/* && value[1] == 0*/
		{
			pr_info("[info] %s hold SS51 and DSP successfully!\n",__func__);
			break;
		}
	}
	pr_info("Hold retry_cnt=%d\n",retry_cnt);
	/*5.enable DSP and MCU power(0x4010 == 0x00)*/
	gf66xx_dev->buffer[GF66XX_WDATA_OFFSET] = 0x0;
	gf66xx_spi_write_bytes(gf66xx_dev, 0x4010, 1, gf66xx_dev->buffer);
	return 1;
}
#endif

#if ESD_PROTECT
static void gf66xx_timer_work(struct work_struct *work)
{
	unsigned char value[3];
	int ret = 0;
	struct gf66xx_dev *gf66xx_dev;
#if FW_UPDATE
	unsigned char* p_fw = GF66XX_FW;
#endif
	if(work == NULL)
	{
		pr_info("[info] %s wrong work\n",__func__);
		return;
	}
	gf66xx_dev = container_of(work, struct gf66xx_dev, spi_work);
	mutex_lock(&gf66xx_dev->buf_lock);
	gf66xx_dev->spi->max_speed_hz= 1000*1000;
	spi_setup(gf66xx_dev->spi);
	gf66xx_spi_read_bytes(gf66xx_dev, 0x8040,1, gf66xx_dev->buffer);
	value[0] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
	gf66xx_spi_read_bytes(gf66xx_dev, 0x8000, 1, gf66xx_dev->buffer);
	value[1] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
	gf66xx_spi_read_bytes(gf66xx_dev, 0x8046, 1, gf66xx_dev->buffer); //&& value[1] == 0x47 && value[2] == 0x56
	value[2] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
	if(value[0] == 0xC6&& value[1] == 0x47){
		//printk("######Jason no need to kick dog 111!\n");
		gf66xx_dev->buffer[GF66XX_WDATA_OFFSET] = 0xAA;
		gf66xx_spi_write_bytes(gf66xx_dev, 0x8040, 1, gf66xx_dev->buffer);
	}else{
        unsigned char version[16] = {0};
        pr_info("hardware works abnormal, do reset! 0x8040 = 0x%x  0x8000 = 0x%x 0x8046 = 0x%x \n",value[0],value[1],value[2]);
        gf66xx_irq_close(gf66xx_dev->spi->irq);
        gf66xx_hw_reset(gf66xx_dev, 360);

        gf66xx_spi_read_bytes(gf66xx_dev, 0x41e4, 1, gf66xx_dev->buffer);
        value[0] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
        gf66xx_spi_read_bytes(gf66xx_dev, 0x8000, 16, gf66xx_dev->buffer);
        memcpy(version, gf66xx_dev->buffer + GF66XX_RDATA_OFFSET, 16);
        pr_info("[info] %s read 0x41e4 finish value = 0x%x, version[0]=%x\n", __func__,value[0], version[0]);
        if((value[0] != 0xbe) || memcmp(version, GF66XX_PID, 6)) {
            gf66xx_power_off(gf66xx_dev);
            mdelay(10);
            ret = gf66xx_power_on(gf66xx_dev);
            if(ret)
                pr_info("[info] %s power on fail\n", __func__);
        }
#if FW_UPDATE
        if((value[0] != 0xbe) || memcmp(version, GF66XX_PID, 6)) {
            gf66xx_spi_read_bytes(gf66xx_dev, 0x41e4, 1, gf66xx_dev->buffer);
            value[0] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
            if(value[0] != 0xbe) {
                /***********************************firmware update*********************************/
                pr_info("[info] %s firmware update start\n", __func__);
                del_timer_sync(&gf66xx_dev->gf66xx_timer);
                gf66xx_fw_update_init(gf66xx_dev);
                ret = gf66xx_fw_update(gf66xx_dev, p_fw, FW_LENGTH);
                gf66xx_hw_reset(gf66xx_dev, 60);
                gf66xx_dev->gf66xx_timer.expires = jiffies + 2 * HZ;
                add_timer(&gf66xx_dev->gf66xx_timer);
            }
        }
#endif
#if CFG_UPDATE 
        /***************************************update config********************************/
        pr_info("[info] %s write config \n", __func__);
        gf66xx_dev->buffer[GF66XX_WDATA_OFFSET] = 0xAA;
        ret = gf66xx_spi_write_bytes(gf66xx_dev, 0x8040, 1, gf66xx_dev->buffer);
        if(!ret)
            pr_info("[info] %s write 0x8040 fail\n", __func__);
        memcpy(gf66xx_dev->buffer + GF66XX_WDATA_OFFSET, gf66xx_config, GF66XX_CFG_LEN);
        ret = gf66xx_spi_write_bytes(gf66xx_dev, GF66XX_CFG_ADDR, GF66XX_CFG_LEN, gf66xx_dev->buffer);
        if(ret <= 0)
            pr_info("[info] %s write config fail\n", __func__);
#endif
        gf66xx_irq_open(gf66xx_dev->spi->irq);
    }
    mutex_unlock(&gf66xx_dev->buf_lock);
}

static void gf66xx_timer_func(unsigned long arg)
{
	struct gf66xx_dev *gf66xx_dev = (struct gf66xx_dev*)arg;
	if(gf66xx_dev == NULL)
	{
		pr_info("[info] %s can't get the gf66xx_dev\n",__func__);
		return;
	}
	schedule_work(&gf66xx_dev->spi_work);
	mod_timer(&gf66xx_dev->gf66xx_timer, jiffies + 2*HZ);
}
#endif


/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *gf66xx_spi_class;
/*-------------------------------------------------------------------------*/

static int gf66xx_probe(struct spi_device *spi)
{
	struct gf66xx_dev	*gf66xx_dev = &gf66xx;
	int			status = -EINVAL;
	unsigned long		minor;
    unsigned char       version = 0;
	FUNC_ENTRY();

	
	/* Initialize the driver data */
	gf66xx_dev->spi = spi;
	spin_lock_init(&gf66xx_dev->spi_lock);
	mutex_init(&gf66xx_dev->buf_lock);
	INIT_LIST_HEAD(&gf66xx_dev->device_entry);

    gf66xx_dev->irq_gpio = -EINVAL;
    gf66xx_dev->reset_gpio = -EINVAL;
    gf66xx_dev->power_gpio = -EINVAL;
 
	hwPowerOn(MT6328_POWER_LDO_VGP1, VOL_2800, "gf66xx");
 //   if(gf66xx_parse_dts(gf66xx_dev) || gf66xx_power_on(gf66xx_dev)) {
	if(gf66xx_parse_dts(gf66xx_dev)) {
        pr_info("gf66xx:Failed to parse dts.\n");
        goto error;
    }
	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */	 
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
//	printk("gf66xx minor = %d\n",minor);
	if (minor < N_SPI_MINORS) {
		struct device *dev;
		gf66xx_dev->devt = MKDEV(major_gf66xx, minor);
		dev = device_create(gf66xx_spi_class, &spi->dev, gf66xx_dev->devt,
				    gf66xx_dev, DEV_NAME);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
		gf66xx_dbg("gf66xx status = %d",status);
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&gf66xx_dev->device_entry, &device_list);
		
	} else {
        gf66xx_dev->devt = 0;
    }
	mutex_unlock(&device_list_lock);
	
	if (status == 0){
		gf66xx_dev->buffer = (u8 *)__get_free_pages(GFP_KERNEL, get_order(bufsiz+2*GF66XX_RDATA_OFFSET));
		gf66xx_dbg("gf66xx dev->buffer");
		//kzalloc(bufsiz + 2*GF66XX_RDATA_OFFSET, GFP_KERNEL);
		if(gf66xx_dev->buffer == NULL) {
			gf66xx_dbg("Failed to allocate data buffer.\n");
			status = -ENOMEM;
			goto error;
		}
		spi_set_drvdata(spi, gf66xx_dev);
	
		/*register device within input system.*/
		gf66xx_dev->input = input_allocate_device();
		gf66xx_dbg("gf66xx dev->input");
		if(gf66xx_dev->input == NULL) {
			gf66xx_dbg("Failed to allocate input device.\n");
			status = -ENOMEM;
			goto error;
		}

		__set_bit(EV_KEY, gf66xx_dev->input->evbit);
		__set_bit(KEY_POWER, gf66xx_dev->input->keybit);

		gf66xx_dev->input->name = GF66XX_INPUT_DEV_NAME;
		if(input_register_device(gf66xx_dev->input)) {
			gf66xx_dbg("Failed to register input device.\n");
		}

        suspend_flag = 0;
        irq_flag = 0;

        pr_info("gf66xx:chip reset.\n");
        /*SPI Configuration*/
		gf66xx_spi_pins_config();
		/*SPI parameters.*/
		spi->irq = gf66xx_irq_num(gf66xx_dev); //gpio_to_irq(GF66XX_IRQ_PIN);//
		spi->bits_per_word = 8; //?
		spi->controller_data  = (void*)&spi_conf_mt65xx;
		gf66xx_spi_set_mode(gf66xx_dev->spi, SPEED_1MHZ, 0);

        /*GF66XX hardware initialize.*/
		gf66xx_hw_reset(gf66xx_dev, 360);
#if FW_UPDATE
		if(isUpdate(gf66xx_dev)) {
			unsigned char* fw = GF66XX_FW;
			/*Do upgrade action.*/
			gf66xx_fw_update_init(gf66xx_dev);
			gf66xx_fw_update(gf66xx_dev, fw, FW_LENGTH);
			gf66xx_hw_reset(gf66xx_dev, 60);
		}
#endif

#if CFG_UPDATE 
		/*write config*/
		memcpy(gf66xx_dev->buffer + GF66XX_WDATA_OFFSET, gf66xx_config, GF66XX_CFG_LEN);
		ret = gf66xx_spi_write_bytes(gf66xx_dev, GF66XX_CFG_ADDR, GF66XX_CFG_LEN, gf66xx_dev->buffer);
		if(ret <= 0)
			pr_info("[info] %s write config fail\n", __func__);
#endif
        /*device detect.*/ 	
		gf66xx_spi_read_byte(gf66xx_dev,0x8000,&version);
		msleep(1000);
		gf66xx_dbg("gf66xx version = %x",version);
		
		if(version != 0x47)
		{
			pr_err("gf66xx:[ERR] %s device detect error!!\n", __func__);
			status = -ENODEV;	
			goto error;
		}

        /*install irq handler.*/    
        gf66xx_irq_thread = kthread_run(gf66xx_irq_work, 0, "gf66xx");
        if(IS_ERR(gf66xx_irq_thread)) {
            pr_info("gf66xx: Failed to create kernel thread: %ld\n", PTR_ERR(gf66xx_irq_thread));
        }
//		gf66xx_dbg("gf66xx gf66xx_irq_setup = %x");
		gf66xx_irq_setup(gf66xx_dev, gf66xx_irq);

#if ESD_PROTECT
		INIT_WORK(&gf66xx_dev->spi_work, gf66xx_timer_work);
		init_timer(&gf66xx_dev->gf66xx_timer);
		gf66xx_dev->gf66xx_timer.function = gf66xx_timer_func;
		gf66xx_dev->gf66xx_timer.expires = jiffies + 20*HZ;
		gf66xx_dev->gf66xx_timer.data = (unsigned long)gf66xx_dev;
		add_timer(&gf66xx_dev->gf66xx_timer);
#endif	

	}
	gf66xx_dbg("GF66XX status = %x",status);
    return status;
error:
    if(gf66xx_dev->devt != 0) {
        pr_info("gf66xx:Err: status = %d\n", status);
        mutex_lock(&device_list_lock);
        //sysfs_remove_group(&spi->dev.kobj, &gf66xx_debug_attr_group);
        list_del(&gf66xx_dev->device_entry);
        device_destroy(gf66xx_spi_class, gf66xx_dev->devt);
        clear_bit(MINOR(gf66xx_dev->devt), minors);
        mutex_unlock(&device_list_lock);
    }
    if(gf66xx_dev->input != NULL)
        input_unregister_device(gf66xx_dev->input);

    if(gf66xx_dev->buffer != NULL) {
        //kfree(gf66xx_dev->buffer);
        free_pages((unsigned long)gf66xx_dev->buffer, get_order(bufsiz + GF66XX_RDATA_OFFSET));
    }
 
	FUNC_EXIT();
	return status;
}

static int gf66xx_remove(struct spi_device *spi)
{
	struct gf66xx_dev	*gf66xx_dev = spi_get_drvdata(spi);
	FUNC_ENTRY();

	/* make sure ops on existing fds can abort cleanly */
	if(gf66xx_dev->spi->irq) {
		gf66xx_irq_release(gf66xx_dev);
	}
    
    if(gf66xx_irq_thread != NULL) {
        irq_flag = 1;
        suspend_flag = 0;
        kthread_stop(gf66xx_irq_thread);
        gf66xx_irq_thread = NULL;
    }

	spin_lock_irq(&gf66xx_dev->spi_lock);
	gf66xx_dev->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&gf66xx_dev->spi_lock);
#if ESD_PROTECT
    del_timer_sync(&gf66xx_dev->gf66xx_timer);
#endif

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&gf66xx_dev->device_entry);
	device_destroy(gf66xx_spi_class, gf66xx_dev->devt);
	clear_bit(MINOR(gf66xx_dev->devt), minors);
    pr_info("gf66xx:remove. users = %d\n", gf66xx_dev->users);
	if (gf66xx_dev->users == 0) {
		if(gf66xx_dev->input != NULL) 
			input_unregister_device(gf66xx_dev->input);

		if(gf66xx_dev->buffer != NULL)
//			kfree(gf66xx_dev->buffer);
			free_pages((unsigned long)gf66xx_dev->buffer,
			   get_order(bufsiz+2*GF66XX_RDATA_OFFSET));
        memset(gf66xx_dev, 0, sizeof(struct gf66xx_dev));
	}
	mutex_unlock(&device_list_lock);

	FUNC_EXIT();
	return 0;
}

static int gf66xx_suspend(struct spi_device *spi, pm_message_t msg)
{
    struct gf66xx_dev *gf66xx_dev = spi_get_drvdata(spi);
    if(gf66xx_dev == NULL) {
        pr_warn("gf66xx: NULL pointer in suspend.\n");
        return -ENODEV;
    }

    pr_info("gf66xx:gf66xx_suspend\n");
    return 0;
}

static int gf66xx_resume(struct spi_device *spi)
{
     struct gf66xx_dev *gf66xx_dev = spi_get_drvdata(spi);
    if(gf66xx_dev == NULL) {
        pr_warn("gf66xx: NULL pointer in resume.\n");
        return -ENODEV;
    }

    pr_info("gf66xx: gf66xx_resume.\n");
    return 0;
}

struct spi_device_id spi_id_table_01 = {"gf66xx", 0};

static struct spi_driver gf66xx_spi_driver = {
	.driver = {
		.name =		SPI_DEV_NAME,
		.owner =	THIS_MODULE,
	},
	.probe =	gf66xx_probe,
//	.remove =	__devexit_p(gf66xx_remove),
	.remove = gf66xx_remove,
    .suspend = gf66xx_suspend,
    .resume = gf66xx_resume,
    .id_table = &spi_id_table_01,

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

static struct spi_board_info spi_board_devs[] __initdata = {
	[0] = {
        .modalias="gf66xx",
		.platform_data = NULL,
		.max_speed_hz = 5 * 100 * 1000,
		.bus_num = 0,
		.chip_select=0,
		.mode = SPI_MODE_0,
	},
};

/*-------------------------------------------------------------------------*/
static int __init gf66xx_init(void)
{
	int status;
	FUNC_ENTRY();

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, "gf66xx", &gf66xx_fops);
	major_gf66xx = status;
	if (status < 0){
		gf66xx_dbg("Failed to register char device!\n");
		FUNC_EXIT();
		return status;
	}
	spi_register_board_info(spi_board_devs, ARRAY_SIZE(spi_board_devs));	
	gf66xx_spi_class = class_create(THIS_MODULE,CLASS_NAME);
	if (IS_ERR(gf66xx_spi_class)) {
		unregister_chrdev(SPIDEV_MAJOR, gf66xx_spi_driver.driver.name);
		gf66xx_dbg("Failed to create class.\n");
		FUNC_EXIT();
		return PTR_ERR(gf66xx_spi_class);
	}	
	status = spi_register_driver(&gf66xx_spi_driver);
	if (status < 0) {
		class_destroy(gf66xx_spi_class);
		unregister_chrdev(SPIDEV_MAJOR, gf66xx_spi_driver.driver.name);
		gf66xx_dbg("Failed to register SPI driver.\n");
	}		
	FUNC_EXIT();
	return status;
}
module_init(gf66xx_init);

static void __exit gf66xx_exit(void)
{
    FUNC_ENTRY();
	spi_unregister_driver(&gf66xx_spi_driver);
	class_destroy(gf66xx_spi_class);
	unregister_chrdev(SPIDEV_MAJOR, gf66xx_spi_driver.driver.name);
	FUNC_EXIT();
}
module_exit(gf66xx_exit);

MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("spi:gf66xx-spi");

