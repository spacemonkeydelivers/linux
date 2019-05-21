#ifndef SK_FPGA_DRIVER_HEADER
#define SK_FPGA_DRIVER_HEADER

// TODO: I don't think we need so many includes

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <generated/utsrelease.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/wait.h>

#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/dmaengine.h>
#include <linux/rtc.h>
#include <linux/ioctl.h>
#include <linux/platform_data/atmel.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/suspend.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>

#if 0
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>

#include <asm/siginfo.h>    //siginfo
#include <linux/rcupdate.h> //rcu_read_lock
#include <linux/sched.h>    //find_task_by_pid_type
#include <linux/sched/signal.h>    //find_task_by_pid_type

#include <linux/module.h>  // Needed by all modules
#include <linux/kernel.h>  // Needed for KERN_INFO
#include <linux/fs.h>      // Needed by filp
#include <asm/uaccess.h>   // Needed by segment descriptors
#endif

#include "sk_at91_xc6slx_ioctl.h"
#include <linux/regmap.h>
#include <linux/mfd/syscon/atmel-smc.h>


#ifdef DEBUG
# define _DBG(fmt, args...) printk(KERN_ALERT "%s: " fmt "\n", __FUNCTION__, ##args)
#else
# define _DBG(fmt, args...) do { } while(0);
#endif

#define DEBUG

#define TMP_BUF_SIZE 4096
#define DMA_BUF_SIZE 65536
#define MAX_WAIT_COUNTER 8*2048

#if 0
enum dma_dir
{
    DMA_ARM_TO_FPGA,
    DMA_FPGA_TO_ARM,
    DMA_LAST,
};

struct sk_fpga_dma_transaction
{
    uint32_t addr;
    uint32_t len;
    uint8_t  dir;
    uint8_t  sync;
};
#endif

struct sk_fpga_pins
{
    uint8_t fpga_cclk;                // pin to run cclk on fpga
    uint8_t fpga_din;                 // pin to set data to fpga
    uint8_t fpga_done;                // pin to read status done from fpga
    uint8_t fpga_prog;                // pin to set mode to prog on fpga
    uint8_t fpga_reset;               // pin to reset fpga internal state
    uint8_t fpga_irq;                 // pin to trigger irq on arm side
    uint8_t host_irq;                 // pin to trigger irq on fpga side
};

struct sk_fpga
{
    struct platform_device *pdev;
    // be aware that real window size is limited by 25 address lines
    uint32_t fpga_mem_window_size;    // phys mem size on any cs pin
    uint32_t fpga_mem_phys_start_cs0; // phys mapped addr of fpga mem on cs0
    uint32_t fpga_mem_phys_start_cs1; // phys mapped addr of fpga mem on cs1
    uint16_t __iomem* fpga_mem_virt_start_cs0;// virt mapped addr of fpga mem on cs0
    uint16_t __iomem* fpga_mem_virt_start_cs1;// virt mapped addr of fpga mem on cs1
    uint8_t opened;                   // fpga opened times
    struct sk_fpga_smc_timings smc_timings; // holds timings for ebi
    struct atmel_smc_cs_conf atmel_smc_conf;
    struct sk_fpga_pins        fpga_pins; // pins to be used to programm fpga or interact with it
    uint8_t* fpga_prog_buffer; // tmp buffer to hold fpga firmware
    struct clk* fpga_clk;
    uint32_t    fpga_freq;
    enum fpga_addr_selector fpga_addr_sel;
    uint8_t programming_started;

    const char* ebi_regmap_name;
    struct regmap* fpga_ebi_regmap;
    const char* matrix_regmap_name;
    struct regmap* fpga_matrix_regmap;

#if 0

    struct dma_chan* fpga_dma_chan;
    dma_addr_t  dma_addr_buf;
    void*       dma_buf;
    int pid;
    int irq_num;
#endif
};

// Maybe we want to hide some of these functions
static int     sk_fpga_remove (struct platform_device *pdev);
static int     sk_fpga_probe  (struct platform_device *pdev);
static int     sk_fpga_close  (struct inode *inodep, struct file *filp);
static int     sk_fpga_open   (struct inode *inode, struct file *file);
static ssize_t sk_fpga_write  (struct file *file, const char __user *buf,
                               size_t len, loff_t *ppos);
#if 0
static ssize_t sk_fpga_read   (struct file *file, char __user *buf,
                               size_t len, loff_t *ppos);
#endif
static long    sk_fpga_ioctl  (struct file *f, unsigned int cmd, unsigned long arg);
int            sk_fpga_setup_ebicsa (void);
// TODO: add description
int sk_fpga_prepare_to_program (void);
int sk_fpga_programming_done   (void);
void sk_fpga_program (const uint8_t* buff, uint32_t bufLen);
#if 0
int sk_fpga_prog(char* fName);
static int sk_fpga_mmap (struct file *file, struct vm_area_struct * vma);
int sk_fpga_setup_dma (struct platform_device *pdev);
int sk_fpga_dma_config_slave (void);
int sk_fpga_do_dma_transfer (struct sk_fpga_dma_transaction* tran);
void sk_fpga_dma_callback (void);
int sk_fpga_unregister_irq (void);
int sk_fpga_register_irq (void);
irqreturn_t sk_fpga_irq_handler (int irq, void *dev_id);
#endif

#endif
