#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <mach/irqs.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <mach/mt_gpio.h>
#include <linux/delay.h>
#include <mach/eint.h>
#include <cust_gpio_usage.h>
#include <cust_eint.h>
#include <linux/io.h>          
#include "gf66xx-spi.h"
#include <linux/gpio.h>
#include <mach/mt_pm_ldo.h>

int gf66xx_parse_dts(struct gf66xx_dev *gf66xx_dev)
{
    int retval = 0;
    if(gf66xx_dev == NULL) {
        pr_info("%s. input buffer is NULL.\n", __func__);
        return -1;
    }
//	printk("gf66xx_parse_dts gf66xx_dev= %x",*gf66xx_dev);
    gf66xx_dev->irq_gpio =  GPIO_INT_PIN;
    gf66xx_dev->reset_gpio =  GPIO_RST_PIN;
    gf66xx_dev->power_gpio =  GPIO_PWR_PIN;
    /*power pin*/
/*    retval =  mt_set_gpio_mode(gf66xx_dev->power_gpio, GPIO_PWR_PIN_M_GPIO);
    if(retval < 0) {
        pr_info("Failed to set power gpio. retval = %d\n", retval);
        return retval;
    } else {
        retval = mt_set_gpio_dir(gf66xx_dev->power_gpio, GPIO_DIR_OUT);
		printk("gf66xx retval = %d",retval);
        if(retval < 0) {
            pr_info("Failed to set power gpio as output. retval = %d\n", retval);
            return retval;
        }
    }*/
	
    /*reset pin as output GPIO with pull-up*/ 
	retval = mt_set_gpio_mode(gf66xx_dev->reset_gpio, GPIO_SPI_RESET_PIN_M_GPIO);
//	printk("-lantao- mt_set_gpio_mode retval = %d\n",retval);
    if(retval < 0) {
	    pr_info("Failed to set reset gpio. retval = %d\n", retval);
        return retval;
    }
    retval = mt_set_gpio_dir(gf66xx_dev->reset_gpio, GPIO_DIR_OUT);
//	printk("-lantao- mt_set_gpio_dir retval = %d\n",retval);
    if(retval < 0) {
	    pr_info("Failed to set reset gpio as output. retval = %d\n", retval);
        return retval;
    }
    retval = mt_set_gpio_pull_enable(gf66xx_dev->reset_gpio, GPIO_PULL_ENABLE);
//	printk("-lantao- mt_set_gpio_pull_enable retval = %d\n",retval);
    if(retval < 0) {
	    pr_info("Failed to set reset gpio pull. retval = %d\n", retval);
        return retval;
    }
    retval = mt_set_gpio_pull_select(gf66xx_dev->reset_gpio, GPIO_PULL_UP);
//	printk("-lantao- mt_set_gpio_pull_select retval = %d\n",retval);
    if(retval < 0) {
	    pr_info("Failed to set reset gpio pull-up. retval = %d\n", retval);
        return retval;
    }
    return 0;
}

int gf66xx_power_on(struct gf66xx_dev *gf66xx_dev)
{
   /*power enable*/
    int retval = 0;
	printk("-lantao- gf66xx  power_on  start\n");
    if(gf66xx_dev == NULL) {
        pr_info("%s. input buffer is NULL.\n", __func__);
        return -1;
    }
 //   retval =  mt_set_gpio_out(gf66xx_dev->power_gpio, GPIO_OUT_ONE);
	hwPowerOn(MT6328_POWER_LDO_VGP1, VOL_2800, "gf66xx");
 /*   if(retval < 0) {
        pr_info("Failed to output power(enable). retval = %d\n", retval);
    } else {
        msleep(10);
    }
   return retval;  */
   printk("-lantao- gf66xx hwpoweron end\n");
   return 0;
}

int gf66xx_power_off(struct gf66xx_dev *gf66xx_dev)
{
    /*power disable*/
    int retval = 0;
    if(gf66xx_dev == NULL) {
        pr_info("%s. input buffer is NULL.\n", __func__);
        return -1;
    }

	hwPowerDown(MT6328_POWER_LDO_VGP1, "gf66xx");
/*    retval = mt_set_gpio_out(gf66xx_dev->power_gpio, GPIO_OUT_ZERO);
    if(retval < 0) {
        pr_info("Failed to output power(disable). retval = %d\n", retval);
    }
    return retval;*/
	return 0;
}

int gf66xx_irq_setup(struct gf66xx_dev *gf66xx_dev, void (*irq_handler)(void )) //need change yfpan
{
    /*IRQ pin as input gpio. no pull-down/pull-up.*/
    int retval = 0;
	printk("gf66xx gf66xx_irq_setup start.\n");
    if(gf66xx_dev == NULL) {
        printk("%s. input buffer is NULL.\n", __func__);
        return -1;
    }

	retval = mt_set_gpio_mode(gf66xx_dev->irq_gpio, GPIO_SPI_EINT_PIN_M_EINT); //set to eint MODE for enable eint function
    if(retval < 0) {
        printk("Failed to set IRQ pin as INT. retval = %d\n", retval);
        return retval;
    }
	retval = mt_set_gpio_dir(gf66xx_dev->irq_gpio, GPIO_DIR_IN); 
    if(retval < 0) {
        printk("Failed to set IRQ as input. retval = %d\n", retval);
        return retval;
    }
	retval = mt_set_gpio_pull_enable(gf66xx_dev->irq_gpio, GPIO_PULL_DISABLE);
    if(retval < 0) {
        printk("Failed to set pull for IRQ pin. retval = %d\n", retval);
        return retval;
    }
//	printk("gf66xx:SPI GPIO EINT PIN mode:num:%d, %d, dir:%d,pullen:%d,pullup%d\n",gf66xx_dev->irq_gpio,
//				mt_get_gpio_mode(gf66xx_dev->irq_gpio), mt_get_gpio_dir(gf66xx_dev->irq_gpio),
//				mt_get_gpio_pull_enable(gf66xx_dev->irq_gpio),mt_get_gpio_pull_select(gf66xx_dev->irq_gpio));    
	
    mt_eint_registration(gf66xx_dev->spi->irq, EINTF_TRIGGER_RISING, irq_handler, 1);         // 0:auto mask is no
	mt_eint_mask(gf66xx_dev->spi->irq);   
	return 0;
}

void gf66xx_spi_pins_config(void)
{
#if 1
	/*cs*/
    mt_set_gpio_mode(GPIO_SPI_CS_PIN, GPIO_SPI_CS_PIN_M_CS);
    mt_set_gpio_pull_enable(GPIO_SPI_CS_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_SPI_CS_PIN, GPIO_PULL_UP);
	/*sck*/
    mt_set_gpio_mode(GPIO_SPI_SCK_PIN, GPIO_SPI_SCK_PIN_M_SCK);
    mt_set_gpio_pull_enable(GPIO_SPI_SCK_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_SPI_SCK_PIN, GPIO_PULL_DOWN);
	/*miso*/
    mt_set_gpio_mode(GPIO_SPI_MISO_PIN, GPIO_SPI_MISO_PIN_M_MISO);
    mt_set_gpio_pull_enable(GPIO_SPI_MISO_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_SPI_MISO_PIN, GPIO_PULL_UP);//  GPIO_PULL_DOWN
	/*mosi*/
    mt_set_gpio_mode(GPIO_SPI_MOSI_PIN, GPIO_SPI_MOSI_PIN_M_MOSI);
    mt_set_gpio_pull_enable(GPIO_SPI_MOSI_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_SPI_MOSI_PIN, GPIO_PULL_UP);
#endif
    msleep(1);
}

int gf66xx_irq_num(struct gf66xx_dev *gf66xx_dev)
{
	return GPIO_IRQ_NUM;
}
int gf66xx_irq_open(int irq_no)
{
	mt_eint_unmask(irq_no);   
        return 0;
}
int gf66xx_irq_close(int irq_no)
{
	mt_eint_mask(irq_no);  
	return 0;
}

int gf66xx_irq_release(struct gf66xx_dev *gf66xx_dev)
{
    int retval = 0;
    if(gf66xx_dev == NULL) {
        pr_info("%s. input buffer is NULL.\n", __func__);
        return -1;
    }

//	mt_eint_mask(gf66xx_dev->spi->irq);   
	mt_set_gpio_pull_enable(gf66xx_dev->irq_gpio, 0);
	mt_set_gpio_pull_select(gf66xx_dev->irq_gpio,  0);
	retval = mt_set_gpio_mode(gf66xx_dev->irq_gpio, GPIO_SPI_EINT_PIN_M_GPIO); //set to eint MODE for enable eint function
    if(retval < 0) {
        pr_info("Failed to set IRQ pin as GPIO. retval = %d\n", retval);
    } else {
	    pr_info("SPI GPIO EINT PIN mode:num:%lx, %d, dir:%d,pullen:%d,pullup%d\n",gf66xx_dev->irq_gpio,
				mt_get_gpio_mode(gf66xx_dev->irq_gpio),mt_get_gpio_dir(gf66xx_dev->irq_gpio),
				mt_get_gpio_pull_enable(gf66xx_dev->irq_gpio),mt_get_gpio_pull_select(gf66xx_dev->irq_gpio));    
    }
    return retval;
}
int gf66xx_hw_reset(struct gf66xx_dev *gf66xx_dev, unsigned int delay_ms) 
{
    if(gf66xx_dev == NULL) {
        pr_info("%s. input buffer is NULL.\n", __func__);
        return -1;
    }
	mt_set_gpio_out(gf66xx_dev->reset_gpio, GPIO_OUT_ZERO); 	
	msleep(5);  //delay for power to reset  typical:10ms max:50ms
	mt_set_gpio_out(gf66xx_dev->reset_gpio, GPIO_OUT_ONE);
    msleep(delay_ms);
    return 0;
}

