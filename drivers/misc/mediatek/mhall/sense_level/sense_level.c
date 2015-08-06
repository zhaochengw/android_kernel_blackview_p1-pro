#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/kthread.h>
#include <cust_eint.h>
#include <cust_gpio_usage.h>
#include <mach/mt_gpio.h>
#include <mach/eint.h>
#include <linux/input.h>
#include <linux/sensors_io.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#define KEY_MHALL_COVER_OPEN			KEY_F1
#define KEY_MHALL_COVER_CLOSE			KEY_F2
#ifdef RGK_MHALL_AK8789_SUPPORT
#define MHALL_DEVICE_NAME				"AK8789"
#elif defined(RGK_MHALL_OCH165_SUPPORT)
#define MHALL_DEVICE_NAME				"OCH165"
#else
#define MHALL_DEVICE_NAME				"UNKNOWN"
#endif

#define MHALL_DEBUG
#ifdef MHALL_DEBUG
#define MHALL_LOG(fmt, args...)			printk("[MHALL-" MHALL_DEVICE_NAME "] " fmt, ##args)
#else
#define MHALL_LOG(fmt, args...)
#endif

#define MHALL_NAME						"mhall"
//add by xuhongming for holl sense start
#ifdef RGK_TOUCHPANEL_HALL_MODE
extern int g_tpd_opencover_fun(void);
extern int g_tpd_closecover_fun(void);
#endif
//add by xuhongming for holl sense end
static bool mhall = 0;
core_param(mhall, mhall, bool, 0444);
static bool pull_u = 0;
core_param(pull_u, pull_u, bool, 0444);
static bool pull_d = 0;
core_param(pull_d, pull_d, bool, 0444);
bool mhall_flag = 0;
core_param(mhall_flag, mhall_flag, bool, 0444);

static struct input_dev *mhall_input_dev;
static DECLARE_WAIT_QUEUE_HEAD(mhall_thread_wq);
static atomic_t mhall_wakeup_flag = ATOMIC_INIT(0);
static int mhall_cover_open_gpio_value = 1;

static int mhall_open(struct inode *inode, struct file *file)
{
	return 0; 
}

static int mhall_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long mhall_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	MHALL_LOG("mhall_ioctl\n");
    switch (cmd) {
		case MHALL_IOCTL_GET_STATUS:              
			if (copy_to_user((void __user *)arg, &mhall_flag, sizeof(mhall_flag))) {
				MHALL_LOG("mhall_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
		break;
		default:
		break;
	}
	return 0;
}

static struct file_operations mhall_fops = {
	.owner = THIS_MODULE,
	.open = mhall_open,
	.release = mhall_release,
	.unlocked_ioctl = mhall_ioctl,
};

static struct miscdevice mhall_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mhall",
	.fops = &mhall_fops,
};

static ssize_t mhall_show_status(struct device* dev, struct device_attribute *attr, char *buf)
{
	ssize_t res;

	res = snprintf(buf, PAGE_SIZE, "%d\n", mhall_flag);
	return res;    
}

static DEVICE_ATTR(status, S_IWUSR | S_IRUGO, mhall_show_status, NULL);

void mhall_eint_interrupt_handler(void)
{
	MHALL_LOG("mhall eint interrupt\n");
    atomic_set(&mhall_wakeup_flag, 1);
	wake_up_interruptible(&mhall_thread_wq);
}

static int mhall_thread_kthread(void *x)
{
	while(1)
	{
		unsigned int mhall_value;

		wait_event_interruptible(mhall_thread_wq, atomic_read(&mhall_wakeup_flag));
		atomic_set(&mhall_wakeup_flag, 0);
		
		mhall_value = mt_get_gpio_in(GPIO_MHALL_EINT_PIN);
		MHALL_LOG("gpio value=%d\n", mhall_value);
		if(mhall_value == mhall_cover_open_gpio_value) {
			input_report_key(mhall_input_dev, KEY_MHALL_COVER_OPEN, 1);
			input_sync(mhall_input_dev);
			msleep(10);
			input_report_key(mhall_input_dev, KEY_MHALL_COVER_OPEN, 0);
			input_sync(mhall_input_dev);
			
			mt_eint_set_polarity(CUST_EINT_MHALL_NUM, !mhall_cover_open_gpio_value);
			mt_eint_unmask(CUST_EINT_MHALL_NUM);
			mhall_flag = 0;
			//add by xuhongming for holl sense start
			#ifdef RGK_TOUCHPANEL_HALL_MODE
			g_tpd_opencover_fun();
			#endif
			//add by xuhongming for holl sense end
			MHALL_LOG("cover open\n");
		} else {
			input_report_key(mhall_input_dev, KEY_MHALL_COVER_CLOSE, 1);
			input_sync(mhall_input_dev);
			msleep(10);
			input_report_key(mhall_input_dev, KEY_MHALL_COVER_CLOSE, 0);
			input_sync(mhall_input_dev);
			
			mt_eint_set_polarity(CUST_EINT_MHALL_NUM, mhall_cover_open_gpio_value);
			mt_eint_unmask(CUST_EINT_MHALL_NUM);
			mhall_flag = 1;
			//add by xuhongming for holl sense start
			#ifdef RGK_TOUCHPANEL_HALL_MODE
			g_tpd_opencover_fun();
			#endif
			//add by xuhongming for holl sense end
			MHALL_LOG("cover close\n");
		}
	}
	return 0;
}

static int mhall_setup_eint(void)
{	
	mt_set_gpio_mode(GPIO_MHALL_EINT_PIN, GPIO_MHALL_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_MHALL_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_DISABLE); 
	mt_eint_set_hw_debounce(CUST_EINT_MHALL_NUM, CUST_EINT_MHALL_DEBOUNCE_EN);
	mt_eint_registration(CUST_EINT_MHALL_NUM, CUST_EINT_MHALL_TYPE, mhall_eint_interrupt_handler, 0);	
	mt_eint_unmask(CUST_EINT_MHALL_NUM);  

    return 0;
}

static int __init mhall_init(void)
{
	int ret;
	
	if(CUST_EINT_MHALL_TYPE == CUST_EINTF_TRIGGER_HIGH)
		mhall_cover_open_gpio_value = 0;
	else if(CUST_EINT_MHALL_TYPE == CUST_EINTF_TRIGGER_LOW)
		mhall_cover_open_gpio_value = 1;
	else {
		MHALL_LOG("gpio int configure error\n");
		return -1;
	}

//	mt_set_gpio_mode(GPIO_MHALL_EINT_PIN, GPIO_MHALL_EINT_PIN_M_EINT);
//	mt_set_gpio_dir(GPIO_MHALL_EINT_PIN, GPIO_DIR_IN);
//	mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_DISABLE);
//	mt_set_gpio_pull_select(GPIO_MHALL_EINT_PIN, GPIO_PULL_DOWN);
//	mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_ENABLE);
//	if((pull_d = mt_get_gpio_in(GPIO_MHALL_EINT_PIN)) != 0)
//		mhall = 1;
//
//	mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_DISABLE);
//	mt_set_gpio_pull_select(GPIO_MHALL_EINT_PIN, GPIO_PULL_UP);
//	mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_ENABLE);
//	if((pull_u = mt_get_gpio_in(GPIO_MHALL_EINT_PIN)) == 0)
//		mhall = 1;
//
//	if (mhall == 0) {
//		MHALL_LOG("detecetd hall failed\n");
//		return -ENODEV;
//	} 
	MHALL_LOG("detecetd hall success\n");

	mhall_input_dev = input_allocate_device();
	if (!mhall_input_dev) {
		MHALL_LOG("alloc input device failed\n");
		return -ENOMEM;
	}
	
	set_bit(EV_KEY, mhall_input_dev->evbit);
	set_bit(KEY_MHALL_COVER_OPEN, mhall_input_dev->keybit);
	set_bit(KEY_MHALL_COVER_CLOSE, mhall_input_dev->keybit);
	
	mhall_input_dev->id.bustype = BUS_HOST;
	mhall_input_dev->name = MHALL_NAME;
	if(ret = input_register_device(mhall_input_dev)) {
		MHALL_LOG("input device register failed\n");
		return ret;
	}
	MHALL_LOG("input register success\n");

	if(ret = misc_register(&mhall_misc_device)) {
		MHALL_LOG("misc device register failed\n");
		return ret;
	}
	MHALL_LOG("misc device register success\n");
	
	if(ret = device_create_file(mhall_misc_device.this_device,&dev_attr_status)) {
		MHALL_LOG("dev attr status register failed\n");
		return ret;
	}
	MHALL_LOG("dev attr status register success\n");
    
	if (IS_ERR(kthread_run(mhall_thread_kthread, NULL, "mhall_thread_kthread"))) {
		MHALL_LOG("kthread create failed\n");
		return -1;
	}
	MHALL_LOG("kthread create success\n");

	mhall_setup_eint();
	MHALL_LOG("setup eint success\n");

	ret = mt_get_gpio_in(GPIO_MHALL_EINT_PIN);
	if(ret == mhall_cover_open_gpio_value) {
		mhall_flag = 0;
		MHALL_LOG("cover open\n");
	} else {
		mhall_flag = 1;
		MHALL_LOG("cover close\n");
	}

	return 0;
}

static void __exit mhall_exit(void)
{
	return ;
}

module_init(mhall_init);
module_exit(mhall_exit);

MODULE_AUTHOR("wanwei.jiang@ragentek.com");
MODULE_DESCRIPTION("mhall driver for level sense chip");
MODULE_LICENSE("GPL");
MODULE_ALIAS("mhall sense level");
