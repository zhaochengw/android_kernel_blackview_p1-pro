#define pr_fmt(fmt) "["KBUILD_MODNAME"] " fmt
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/kfifo.h>

#include <linux/firmware.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/printk.h>

#include <mach/mt_reg_base.h>
#include <mach/mt_typedefs.h>
#include <asm/setup.h>
#include <mach/mt_chip_common.h>

struct mt_chip_drv g_chip_drv = {
    .info_bit_mask = CHIP_INFO_BIT(CHIP_INFO_ALL)
};

struct mt_chip_drv* get_mt_chip_drv(void)
{
    return &g_chip_drv;
}

struct chip_inf_entry
{
    const char*     name;
    chip_info_t     id;
    int             (*to_str)(char* buf, size_t len, int val);
};

static int hex2str(char* buf, size_t len, int val)
{
    return snprintf(buf, len, "%04X", val);
}

static int dec2str(char* buf, size_t len, int val)
{
    return snprintf(buf, len, "%04d", val);    
}

static int date2str(char* buf, size_t len, int val)
{
    unsigned int year = ((val & 0x3C0) >> 6) + 2012;
    unsigned int week = (val & 0x03F);
    return snprintf(buf, len, "%04d%02d", year, week);
}

#define __chip_info(id) ((g_chip_drv.get_chip_info) ? (g_chip_drv.get_chip_info(id)) : (0x0000))

static struct proc_dir_entry *chip_proc = NULL;
static struct chip_inf_entry chip_ent[] = 
{
    {"hw_code",     CHIP_INFO_HW_CODE,      hex2str},
    {"hw_subcode",  CHIP_INFO_HW_SUBCODE,   hex2str},
    {"hw_ver",      CHIP_INFO_HW_VER,       hex2str},
    {"sw_ver",      CHIP_INFO_SW_VER,       hex2str},
    {"code_func",   CHIP_INFO_FUNCTION_CODE,hex2str},
    {"code_date",   CHIP_INFO_DATE_CODE,    date2str},
    {"code_proj",   CHIP_INFO_PROJECT_CODE, dec2str},
    {"code_fab",    CHIP_INFO_FAB_CODE,     hex2str},
    {"wafer_big_ver", CHIP_INFO_WAFER_BIG_VER, hex2str},
    {"info",        CHIP_INFO_ALL,          NULL}
};

static int chip_proc_show(struct seq_file* s, void* v)
{
    struct chip_inf_entry *ent = s->private;
    if ((ent->id > CHIP_INFO_NONE) && (ent->id < CHIP_INFO_MAX)) {
        seq_printf(s, "%04X\n", __chip_info(ent->id));
    } else {
        int idx = 0;
        char buf[16];
        
        for (idx = 0; idx < sizeof(chip_ent)/sizeof(chip_ent[0]); idx++) {    
            struct chip_inf_entry *ent = &chip_ent[idx];
            unsigned int val = __chip_info(ent->id);
        
            if (!CHIP_INFO_SUP(g_chip_drv.info_bit_mask, ent->id))
                continue;
            else if (!ent->to_str)    
                continue;            
            else if (0 < ent->to_str(buf, sizeof(buf), val))
                seq_printf(s, "%-16s : %s (%04x)\n", ent->name, buf, val);
            else
                seq_printf(s, "%-16s : %s (%04x)\n", ent->name, "NULL", val); 
        }
        seq_printf(s, "%-16s : %04X %04X %04X %04X\n", "reg", 
                      __chip_info(CHIP_INFO_REG_HW_CODE),
                      __chip_info(CHIP_INFO_REG_HW_SUBCODE),
                      __chip_info(CHIP_INFO_REG_HW_VER),
                      __chip_info(CHIP_INFO_REG_SW_VER)); 
    }      
    return 0;
}

static int chip_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, chip_proc_show, PDE_DATA(file_inode(file)));
}

static const struct file_operations chip_proc_fops = { 
    .open           = chip_proc_open,
    .read           = seq_read,
    .llseek         = seq_lseek,
    .release        = single_release,
};

extern struct proc_dir_entry proc_root;
static void __init create_procfs(void)
{
    int idx;
    
    chip_proc = proc_mkdir_data("chip", 0, &proc_root, NULL);
    if (NULL == chip_proc) {
        pr_err("create /proc/chip fails\n");
        return;
    } else {            
        pr_alert("create /proc/chip(%x)\n", g_chip_drv.info_bit_mask);
    }
    
    for (idx = 0; idx < sizeof(chip_ent)/sizeof(chip_ent[0]); idx++) {
        struct chip_inf_entry *ent = &chip_ent[idx];
        if (!CHIP_INFO_SUP(g_chip_drv.info_bit_mask, ent->id))
            continue;
        if (NULL == proc_create_data(ent->name, S_IRUGO, chip_proc, &chip_proc_fops, ent)) {
            pr_err("create /proc/chip/%s fail\n", ent->name);
            return;
        }
    }
}

#if 1//lisong 2014-12-15 [BUGID:NULL][add for *#*#9375#*#*]start
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

char rgk_mem_name[100]= {0};
static int __init init_rgk_mem_name(char *str)
{
       strncpy(rgk_mem_name, str, sizeof(rgk_mem_name)-1);
       rgk_mem_name[sizeof(rgk_mem_name)-1] = '\0';
       return 1;
}
__setup("rgk_memname=", init_rgk_mem_name);

static int rgk_mem_info_show(struct seq_file *m, void *v)
{
       seq_printf(m, "%s\n", rgk_mem_name);    
       return 0;
}

static int rgk_mem_info_open(struct inode *inode, struct file *file)
{
       return single_open(file, rgk_mem_info_show, NULL);
}

static const struct file_operations rgk_mem_info_proc_fops = { 
       .owner      = THIS_MODULE,
       .open       = rgk_mem_info_open,
       .read       = seq_read,
       .llseek     = seq_lseek,
       .release    = single_release,
};

int rgk_creat_proc_mem_info(void)
{
       struct proc_dir_entry *mem_info_entry = proc_create("rgk_memInfo", 0444, NULL, &rgk_mem_info_proc_fops);
       if (mem_info_entry == NULL)
               printk("create /proc/mem_info_entry fail\n");
       
       return 0;
}

//pcb info
#include <mach/mt_gpio.h>
#include "custom_pcb_info.h"

static int rgk_pcb_info = 0xff;
static int rgk_pcb_info_show(struct seq_file *m, void *v)
{
#if defined(GPIO_PCB_ID1_PIN) && defined(GPIO_PCB_ID2_PIN)
    {
        s32 id1_pin,id2_pin;

        mt_set_gpio_mode(GPIO_PCB_ID1_PIN, GPIO_PCB_ID1_PIN_M_GPIO);
        mt_set_gpio_dir(GPIO_PCB_ID1_PIN, GPIO_DIR_IN);
        mt_set_gpio_pull_enable(GPIO_PCB_ID1_PIN, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(GPIO_PCB_ID1_PIN, GPIO_PULL_DOWN);

        mt_set_gpio_mode(GPIO_PCB_ID2_PIN, GPIO_PCB_ID2_PIN_M_GPIO);
        mt_set_gpio_dir(GPIO_PCB_ID2_PIN, GPIO_DIR_IN);
        mt_set_gpio_pull_enable(GPIO_PCB_ID2_PIN, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(GPIO_PCB_ID2_PIN, GPIO_PULL_DOWN);

        id1_pin = mt_get_gpio_in(GPIO_PCB_ID1_PIN);
        id2_pin = mt_get_gpio_in(GPIO_PCB_ID2_PIN);
        if(id1_pin == 0 && id2_pin == 0)
        {rgk_pcb_info = PCB_INFO_0;}
        else if(id1_pin == 1 && id2_pin == 0)
        {rgk_pcb_info = PCB_INFO_1;}
        else if(id1_pin == 0 && id2_pin == 1)
        {rgk_pcb_info = PCB_INFO_2;}
        else
        {rgk_pcb_info = PCB_INFO_3;}         
    }
#endif  
    seq_printf(m, "%d\n", rgk_pcb_info);    
    return 0;
}

static int rgk_pcb_info_open(struct inode *inode, struct file *file)
{
       return single_open(file, rgk_pcb_info_show, NULL);
}

static const struct file_operations rgk_pcb_info_proc_fops = { 
       .owner      = THIS_MODULE,
       .open       = rgk_pcb_info_open,
       .read       = seq_read,
       .llseek     = seq_lseek,
       .release    = single_release,
};

int rgk_creat_proc_pcb_info(void)
{
       struct proc_dir_entry *pcb_info_entry = proc_create("rgk_pcbInfo", 0444, NULL, &rgk_pcb_info_proc_fops);
       if (pcb_info_entry == NULL)
               printk("create /proc/pcb_info_entry fail\n");
       
       return 0;
}


#endif//lisong 2014-12-15 [BUGID:NULL][add for *#*#9375#*#*]end

int __init chip_common_init(void)
{
    create_procfs();
#if 1//lisong 2014-12-15 [BUGID:NULL][add for *#*#9375#*#*]start
	rgk_creat_proc_mem_info();
    rgk_creat_proc_pcb_info();
#endif//lisong 2014-12-15 [BUGID:NULL][add for *#*#9375#*#*]end
    return 0;
}

arch_initcall(chip_common_init);
MODULE_DESCRIPTION("MTK Chip Common");
MODULE_LICENSE("GPL");

