#include <linux/fpga/sk_at91_xc6slx.h>
#include <linux/fpga/sk_at91_xc6slx_ioctl.h>

#define DEBUG


#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>

struct sk_fpga fpga;

static const struct file_operations fpga_fops = {
        .owner          = THIS_MODULE,
        .open           = sk_fpga_open,
        .release        = sk_fpga_close,
        .write          = sk_fpga_write,
        .read           = sk_fpga_read,
        .unlocked_ioctl = sk_fpga_ioctl,
        .mmap           = sk_fpga_mmap,
};

static struct miscdevice sk_fpga_dev = {
        MISC_DYNAMIC_MINOR,
        "fpga",
        &fpga_fops
};

// FIXME: is it optimal way to calculate the pointer?
uint16_t* sk_fpga_ptr_by_addr (uint32_t addr)
{
    uint16_t* mem_start = NULL;
    BUG_ON(addr >= fpga.fpga_mem_window_size);
    BUG_ON(!(fpga.fpga_addr_sel == FPGA_ADDR_CS0) && !(fpga.fpga_addr_sel == FPGA_ADDR_CS1));
    mem_start = (fpga.fpga_addr_sel == FPGA_ADDR_CS0) ? fpga.fpga_mem_virt_start_cs0 : fpga.fpga_mem_virt_start_cs1;
    BUG_ON(mem_start == NULL);
    BUG_ON(addr & 0x1);
    return (mem_start + addr/sizeof(uint16_t));
}

static int sk_fpga_open (struct inode *inode, struct file *file)
{
    if (fpga.opened) 
    {
        return -EBUSY;
    } 
    else 
    {
        fpga.opened++;
    }
    return 0;
}

static int sk_fpga_close (struct inode *inode, struct file *file)
{
    if (fpga.opened) 
    {
        fpga.opened--;
    } 
    else 
    {
        return -ENODEV;
    }
    return 0;
}

// FIXME: sk_fpga_ptr_by_addr doesn't check for window crossing
static ssize_t sk_fpga_read (struct file *file, char __user *buf,
                    size_t len, loff_t *ppos)
{
    int i = 0;
    int res = 0;
    uint16_t bytes_to_read = (TMP_BUF_SIZE < len) ? TMP_BUF_SIZE : len;
    uint16_t* start = sk_fpga_ptr_by_addr(fpga.address);
    // 2 since byte vs short
    BUG_ON(bytes_to_read & 0x1);
    BUG_ON((bytes_to_read + fpga.address) > fpga.fpga_mem_window_size);
    for (; i < bytes_to_read; i += sizeof(uint16_t))
    {
        fpga.fpga_prog_buffer[i] = ioread16(start + i);
    }
    res = copy_to_user(buf, fpga.fpga_prog_buffer, bytes_to_read);
    return (bytes_to_read - res);
}

// Write data to FPGA if FPGA is not being programmed
static ssize_t sk_fpga_write(struct file *file, const char __user *buf,
                             size_t len, loff_t *ppos)
{
    int i = 0;
    uint16_t* start = NULL;
    uint16_t bytes_to_copy = (TMP_BUF_SIZE < len) ? TMP_BUF_SIZE : len;
    int res = copy_from_user(fpga.fpga_prog_buffer, buf, bytes_to_copy);
    start = sk_fpga_ptr_by_addr(fpga.address);
    BUG_ON((bytes_to_copy + fpga.address) > fpga.fpga_mem_window_size);
    // 2 since byte vs short
    BUG_ON(bytes_to_copy & 0x1);
    for (; i < bytes_to_copy; i += sizeof(uint16_t))
    {
        iowrite16(fpga.fpga_prog_buffer[i], start + i);
    }
    return (bytes_to_copy - res);
}

static long sk_fpga_ioctl (struct file *f, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    struct sk_fpga_data data = {0};
    uint8_t value = 0;
    char fName[256] = {0};
    struct sk_fpga_dma_transaction dma_tran = {0};
    int pid = 0;

    switch (cmd)
    {
    // set current fpga ebi timings
    case SKFPGA_IOSSMCTIMINGS:
        if (copy_from_user(&fpga.smc_timings, (int __user *)arg, sizeof(struct sk_fpga_smc_timings)))
            return -EFAULT;
        if (sk_fpga_setup_smc())
            return -EFAULT;
        break;

    // Get current fpga ebi timings
    case SKFPGA_IOGSMCTIMINGS:
        if (sk_fpga_read_smc())
            return -EFAULT;
        if (copy_to_user((int __user *)arg, &fpga.smc_timings, sizeof(struct sk_fpga_smc_timings)))
            return -EFAULT;
        break;

    // write short to FPGA
    case SKFPGA_IOSDATA:
        if (copy_from_user(&data, (int __user *)arg, sizeof(struct sk_fpga_data)))
            return -EFAULT;
        BUG_ON(data.address + sizeof(uint16_t) > fpga.fpga_mem_window_size);
        iowrite16(data.data, sk_fpga_ptr_by_addr(data.address));
        break;

    // read short from FPGA
    case SKFPGA_IOGDATA:
        if (copy_from_user(&data, (int __user *)arg, sizeof(struct sk_fpga_data)))
            return -EFAULT;
        BUG_ON(data.address + sizeof(uint16_t) > fpga.fpga_mem_window_size);
        data.data = ioread16(sk_fpga_ptr_by_addr(data.address));
        if (copy_to_user((int __user *)arg, &data, sizeof(struct sk_fpga_data)))
            return -EFAULT;
        break;

    case SKFPGA_IOSPROG:
        if (copy_from_user(fName, (int __user *)arg, sizeof(char)*PROG_FILE_NAME_LEN))
            return -EFAULT;
        if (sk_fpga_prog(fName))
            return -EFAULT;
        break;

    // toggle reset ping
    case SKFPGA_IOSRESET:
        if (copy_from_user(&value, (int __user *)arg, sizeof(uint8_t)))
            return -EFAULT;
        gpio_set_value(fpga.fpga_pins.fpga_reset, (value) ? 1 : 0);
        break;

    case SKFPGA_IOGRESET:
        value = gpio_get_value(fpga.fpga_pins.fpga_reset);
        if (copy_to_user((int __user *)arg, &value, sizeof(uint8_t)))
            return -EFAULT;
        break;

    case SKFPGA_IOSHOSTIRQ:
        if (copy_from_user(&value, (int __user *)arg, sizeof(uint8_t)))
            return -EFAULT;
        gpio_set_value(fpga.fpga_pins.host_irq, (value) ? 1 : 0);
        break;
    
    case SKFPGA_IOGHOSTIRQ:
        value = gpio_get_value(fpga.fpga_pins.host_irq);
        if (copy_to_user((int __user *)arg, &value, sizeof(uint8_t)))
            return -EFAULT;
        break;

    case SKFPGA_IOSFPGAIRQ:
        if (copy_from_user(&value, (int __user *)arg, sizeof(uint8_t)))
            return -EFAULT;
        if (value)
        {
            if (sk_fpga_register_irq())
                return -EFAULT;
        }
        else
        {
            if (sk_fpga_unregister_irq())
                return -EFAULT;
        }
        break;

    case SKFPGA_IOSADDRSEL:
        if (copy_from_user(&value, (int __user *)arg, sizeof(uint8_t)))
            return -EFAULT;
        BUG_ON(!(value == FPGA_ADDR_CS0) && !(value == FPGA_ADDR_CS1) && !(value == FPGA_ADDR_DMA));
        fpga.fpga_addr_sel = value;
        break;
    
    case SKFPGA_IOGADDRSEL:
        if (copy_to_user((int __user *)arg, &fpga.fpga_addr_sel, sizeof(uint8_t)))
            return -EFAULT;
        break;

    case SKFPGA_IOSDMA:
        if (copy_from_user(&dma_tran, (int __user *)arg, sizeof(struct sk_fpga_dma_transaction)))
            return -EFAULT;
        BUG_ON(dma_tran.addr & 0x1);
        if (sk_fpga_dma_config_slave())
            return -EFAULT;
        if (sk_fpga_do_dma_transfer(&dma_tran))
            return -EIO;
        break;

    case SKFPGA_IOSPID:
        if (copy_from_user(&pid, (int __user *)arg, sizeof(int)))
            return -EFAULT;
        fpga.pid = pid;
        break;

    default:
        return -ENOTTY;
    }
    return ret;
}

int sk_fpga_do_dma_transfer (struct sk_fpga_dma_transaction* tran)
{
    int err = 0;
    enum dma_status status;
    struct dma_async_tx_descriptor* dma_desc;
    dma_cookie_t        dma_cookie;
    
    dma_desc = dmaengine_prep_dma_memcpy(fpga.fpga_dma_chan,
                                         ((enum dma_dir)tran->dir == DMA_ARM_TO_FPGA) ? tran->addr : fpga.dma_addr_buf,
                                         ((enum dma_dir)tran->dir == DMA_ARM_TO_FPGA) ? fpga.dma_addr_buf : tran->addr,
                                         tran->len,
                                         DMA_PREP_INTERRUPT | DMA_CTRL_ACK);

    // register callback only in case of async transfer?
    dma_desc->callback = (void*)sk_fpga_dma_callback;
    dma_cookie = dmaengine_submit(dma_desc);
    if (dma_submit_error(dma_cookie))
    {
        printk(KERN_ALERT"Failed to submit dma transfer");
        BUG_ON(1);
    }
    dma_async_issue_pending(fpga.fpga_dma_chan);
    if (tran->sync)
    {
        status = dma_wait_for_async_tx(dma_desc);
        if (status != DMA_COMPLETE)
        {
            printk(KERN_ALERT"DMA tranfer failed with status %d", status);
            dmaengine_terminate_async(fpga.fpga_dma_chan);
        }
    }
    return err;
}

// TODO: fix dtb to avoid such hacks
int sk_fpga_setup_ebicsa (void)
{
    int ret = -EIO;
    uint32_t __iomem* matrix = NULL;
    uint32_t __iomem* ebicsa = NULL;
    uint32_t cur_ebicsa = 0;

    if (!request_mem_region(MATRIX_ADDRESS, MATRIX_ADDRESS_WINDOW, "sk_fpga_matrix")) 
    {
        printk(KERN_ALERT"Failed to request mem region for matrix\n");
        return ret;
    }
    matrix = ioremap(MATRIX_ADDRESS, MATRIX_ADDRESS_WINDOW);
    if (!matrix) 
    {
        printk(KERN_ALERT"Failed to ioremap mem region for matrix\n");
        release_mem_region(MATRIX_ADDRESS, MATRIX_ADDRESS_WINDOW);
        return ret;
    }
    ebicsa = matrix + EBICSA_OFFSET/sizeof(uint32_t);
    cur_ebicsa = ioread32(ebicsa);
    cur_ebicsa &= ~EBICSA_CS1_MASK;
    iowrite32(cur_ebicsa, ebicsa);

    iounmap(matrix);
    release_mem_region(MATRIX_ADDRESS, MATRIX_ADDRESS_WINDOW);

    return 0;    
}

// TODO: reuse existing atmel ebi interfaces and data from dtb
int sk_fpga_setup_smc (void)
{
    int ret = -EIO;
    uint32_t __iomem* smc = NULL;

    if (!request_mem_region(SMC_ADDRESS, SMC_ADDRESS_WINDOW, "sk_fpga_smc0")) 
    {
        printk(KERN_ALERT"Failed to request mem region for smc\n");
        return ret;
    }
    smc = ioremap(SMC_ADDRESS, SMC_ADDRESS_WINDOW);
    if (!smc) 
    {
        printk(KERN_ALERT"Failed to ioremap mem region for smc\n");
        release_mem_region(SMC_ADDRESS, SMC_ADDRESS_WINDOW);
        return ret;
    }

    iowrite32(fpga.smc_timings.setup, SMC_SETUP(smc, fpga.smc_timings.num));
    iowrite32(fpga.smc_timings.pulse, SMC_PULSE(smc, fpga.smc_timings.num));
    iowrite32(fpga.smc_timings.cycle, SMC_CYCLE(smc, fpga.smc_timings.num));
    iowrite32(fpga.smc_timings.mode, SMC_MODE(smc, fpga.smc_timings.num));

    iounmap(smc);
    release_mem_region(SMC_ADDRESS, SMC_ADDRESS_WINDOW);
   
    return 0;    
}

int sk_fpga_register_irq (void)
{
    int ret = 0;
    fpga.irq_num = gpio_to_irq(fpga.fpga_pins.fpga_irq);
    if (!fpga.irq_num)
    {
        printk(KERN_ALERT"Failed to obtain irq number");
        return -EFAULT;
    }
    ret = request_irq(fpga.irq_num,
                      (irq_handler_t)sk_fpga_irq_handler,
                      IRQ_TYPE_EDGE_BOTH,
                      "sk_fpga_irq",
                      NULL);
    if (ret)
    {
        printk(KERN_ALERT"Failed to register irq");
        fpga.irq_num = 0;
        return ret;
    }
    return ret;
}

int sk_fpga_unregister_irq (void)
{
    free_irq(fpga.irq_num, NULL);
    fpga.irq_num = 0;
    return 0;
}

irqreturn_t sk_fpga_irq_handler (int irq, void *dev_id)
{
    int ret = 0;
    uint8_t curSel = 0;

    struct task_struct* current_task = NULL;
    struct kernel_siginfo info;

    // for some reason irq happens right after registering
    if (!gpio_get_value(fpga.fpga_pins.fpga_irq))
        return IRQ_HANDLED;

    // clear irq pin!!
    curSel = fpga.fpga_addr_sel;
    fpga.fpga_addr_sel = FPGA_ADDR_CS0;
    iowrite16(0, sk_fpga_ptr_by_addr(0));
    fpga.fpga_addr_sel = curSel;

    memset(&info, 0, sizeof(struct kernel_siginfo));
    info.si_signo = SIGUSR2;
    info.si_code = 0;
    info.si_int = 0;
    if (current_task == NULL)
    {
        rcu_read_lock();
        current_task = pid_task(find_vpid(fpga.pid), PIDTYPE_PID);
        rcu_read_unlock();
    }
    ret = send_sig_info(SIGUSR2, &info, current_task);
    if (ret < 0) 
    {
        printk(KERN_ALERT"Failed to send a signal");
        BUG_ON(1);
    }
    return IRQ_HANDLED;
}

void sk_fpga_dma_callback (void)
{
    int ret = 0;
    struct task_struct* current_task = NULL;
    struct kernel_siginfo info;
    memset(&info, 0, sizeof(struct kernel_siginfo));
    info.si_signo = SIGUSR1;
    info.si_code = 0;
    info.si_int = 0;
    if (current_task == NULL)
    {
        rcu_read_lock();
        current_task = pid_task(find_vpid(fpga.pid), PIDTYPE_PID);
        rcu_read_unlock();
    }
    ret = send_sig_info(SIGUSR1, &info, current_task);
    if (ret < 0) 
    {
        printk(KERN_ALERT"Failed to send a signal");
        BUG_ON(1);
    }
}

// TODO: reuse existing atmel ebi interfaces and data from dtb
int sk_fpga_read_smc (void)
{
    int ret = -EIO;
    uint32_t __iomem* smc = NULL;

    if (!request_mem_region(SMC_ADDRESS, SMC_ADDRESS_WINDOW, "sk_fpga_smc0")) 
    {
        printk(KERN_ALERT"Failed to request mem region for smc\n");
        return ret;
    }

    smc = ioremap(SMC_ADDRESS, SMC_ADDRESS_WINDOW);
    if (!smc) 
    {
        printk(KERN_ALERT"Failed to ioremap mem region for smc\n");
        release_mem_region(SMC_ADDRESS, SMC_ADDRESS_WINDOW);
        return ret;
    }

    fpga.smc_timings.setup = ioread32(SMC_SETUP(smc, fpga.smc_timings.num));
    fpga.smc_timings.pulse = ioread32(SMC_PULSE(smc, fpga.smc_timings.num));
    fpga.smc_timings.cycle = ioread32(SMC_CYCLE(smc, fpga.smc_timings.num));
    fpga.smc_timings.mode  = ioread32(SMC_MODE(smc, fpga.smc_timings.num));

    iounmap(smc);
    release_mem_region(SMC_ADDRESS, SMC_ADDRESS_WINDOW);
    return 0;    
}

// TODO: read smc settings from dtb and initialize timings with them
int sk_fpga_fill_structure(struct platform_device *pdev)
{
    int ret = -EIO;

    // get FPGA clk source
    fpga.fpga_clk = devm_clk_get(&pdev->dev, "mclk");
    if (IS_ERR(fpga.fpga_clk)) 
    {
        dev_err(&pdev->dev, "Failed to get clk source for fpga from dtb\n");
        return ret;
    }

    // get fpga reset gpio
    fpga.fpga_pins.fpga_reset = of_get_named_gpio(pdev->dev.of_node, "fpga-reset-gpio", 0);
    if (!fpga.fpga_pins.fpga_reset) {
        dev_err(&pdev->dev, "Failed to obtain fpga reset pin\n");
        return ret;
    }

    // get fpga done gpio
    fpga.fpga_pins.fpga_done = of_get_named_gpio(pdev->dev.of_node, "fpga-program-done", 0);
    if (!fpga.fpga_pins.fpga_done) {
        dev_err(&pdev->dev, "Failed to obtain fpga done pin\n");
        return ret;
    }

    // get fpga cclk gpio
    fpga.fpga_pins.fpga_cclk = of_get_named_gpio(pdev->dev.of_node, "fpga-program-cclk", 0);
    if (!fpga.fpga_pins.fpga_cclk) {
        dev_err(&pdev->dev, "Failed to obtain fpga cclk pin\n");
        return ret;
    }

    // get fpga din gpio
    fpga.fpga_pins.fpga_din = of_get_named_gpio(pdev->dev.of_node, "fpga-program-din", 0);
    if (!fpga.fpga_pins.fpga_din) {
        dev_err(&pdev->dev, "Failed to obtain fpga din pin\n");
        return ret;
    }

    // get fpga prog gpio
    fpga.fpga_pins.fpga_prog = of_get_named_gpio(pdev->dev.of_node, "fpga-program-prog", 0);
    if (!fpga.fpga_pins.fpga_prog) {
        dev_err(&pdev->dev, "Failed to obtain fpga prog pin\n");
        return ret;
    }
    
    // get fpga reset pin
    fpga.fpga_pins.fpga_reset = of_get_named_gpio(pdev->dev.of_node, "fpga-reset-gpio", 0);
    if (!fpga.fpga_pins.fpga_reset) {
        dev_err(&pdev->dev, "Failed to obtain fpga reset pin\n");
        return ret;
    }

    // get fpga irq pin
    fpga.fpga_pins.fpga_irq = of_get_named_gpio(pdev->dev.of_node, "fpga-irq-gpio", 0);
    if (!fpga.fpga_pins.fpga_irq) {
        dev_err(&pdev->dev, "Failed to obtain fpga irq pin\n");
        return ret;
    }
    // get host irq pin
    fpga.fpga_pins.host_irq = of_get_named_gpio(pdev->dev.of_node, "fpga-host-irq-gpio", 0);
    if (!fpga.fpga_pins.host_irq) {
        dev_err(&pdev->dev, "Failed to obtain host irq pin\n");
        return ret;
    }

    // get fpga mem sizes
    ret = of_property_read_u32(pdev->dev.of_node, "fpga-memory-window-size", &fpga.fpga_mem_window_size);
    if (ret)
    {
        printk(KERN_ALERT"Failed to obtain fpga cs memory window size from dtb\n");
        return -ENOMEM;
    }

    // get FPGA phys start address for cs0
    ret = of_property_read_u32(pdev->dev.of_node, "fpga-memory-start-address-cs0", &fpga.fpga_mem_phys_start_cs0);
    if (ret) {
        printk(KERN_ALERT"Failed to obtain start phys mem start address from dtb\n");
        return -ENOMEM;
    }

    // get FPGA phys start address for cs1
    ret = of_property_read_u32(pdev->dev.of_node, "fpga-memory-start-address-cs1", &fpga.fpga_mem_phys_start_cs1);
    if (ret) {
        printk(KERN_ALERT"Failed to obtain start phys mem start address from dtb\n");
        return -ENOMEM;
    }

    // get FPGA frequency
    ret = of_property_read_u32(pdev->dev.of_node, "fpga-frequency", &fpga.fpga_freq);
    if (ret) {
        printk(KERN_ALERT"Failed to obtain start phys mem start address from dtb\n");
        return -ENOMEM;
    }
    
    fpga.fpga_mem_virt_start_cs0 = NULL;
    fpga.fpga_mem_virt_start_cs1 = NULL;

    return 0;
}

static int sk_fpga_probe (struct platform_device *pdev)
{
    int ret = -EIO;
    memset(&fpga, 0, sizeof(fpga));
    fpga.pdev = pdev;

    printk(KERN_ALERT"Loading FPGA driver for SK-AT91SAM9M10G45EK-XC6SLX\n");

    // register misc device
    ret = misc_register(&sk_fpga_dev);
    if (ret) {
        printk(KERN_ALERT"Unable to register \"fpga\" misc device\n");
        return -ENOMEM;
    }

    // fill structure by dtb info
    ret = sk_fpga_fill_structure(fpga.pdev);
    if (ret) {
        printk(KERN_ALERT"Failed to fill fpga structure out of dts\n");
        ret = -EINVAL;
        goto misc_dereg;
    }
    
    // allocate tmp buffer
    fpga.fpga_prog_buffer = kmalloc(TMP_BUF_SIZE, GFP_KERNEL);
    if (!fpga.fpga_prog_buffer) {
        printk(KERN_ALERT"Failed to allocate memory for tmp buffer");
        ret = -ENOMEM;
        goto misc_dereg;
    }

    // map phys 2 virt for both windows
    if (!request_mem_region(fpga.fpga_mem_phys_start_cs0, fpga.fpga_mem_window_size, "sk_fpga_mem_window_cs0")) {
        printk(KERN_ALERT"Failed to request mem region for sk_fpga_mem_window_cs0\n");
        ret = -ENOMEM;
        goto free_buf;
    }

    fpga.fpga_mem_virt_start_cs0 = ioremap(fpga.fpga_mem_phys_start_cs0, fpga.fpga_mem_window_size);
    if (!fpga.fpga_mem_virt_start_cs0) {
        printk(KERN_ALERT"Failed to ioremap mem region for sk_fpga_mem_window_cs0\n");
        ret = -ENOMEM;
        goto release_window_cs0;
    }

    if (!request_mem_region(fpga.fpga_mem_phys_start_cs1, fpga.fpga_mem_window_size, "sk_fpga_mem_window_cs1")) {
        printk(KERN_ALERT"Failed to request mem region for sk_fpga_mem_window_cs1\n");
        ret = -ENOMEM;
        goto unmap_window_cs0;
    }

    fpga.fpga_mem_virt_start_cs1 = ioremap(fpga.fpga_mem_phys_start_cs1, fpga.fpga_mem_window_size);
    if (!fpga.fpga_mem_virt_start_cs1) {
        printk(KERN_ALERT"Failed to ioremap mem region for sk_fpga_mem_window_cs1\n");
        ret = -ENOMEM;
        goto release_window_cs1;
    }

    ret = clk_set_rate(fpga.fpga_clk, fpga.fpga_freq);
    if (ret)
    {
        printk(KERN_ALERT"Failed to set clk rate for FPGA to %d", fpga.fpga_freq);
        ret = -EIO;
        goto unmap_window_cs1;
    }
    
    ret = clk_prepare_enable(fpga.fpga_clk);
    if (ret)
    {
        dev_err(&pdev->dev, "Couldn't enable FPGA clock\n");
        printk(KERN_ALERT"PREPARE  STATUS: %d\n", ret);
        goto unmap_window_cs1;
    }

    ret = gpio_request(fpga.fpga_pins.fpga_reset, "sk_fpga_reset_pin");
    if (ret)
    {
        printk(KERN_ALERT"Failed to acqiure reset pin");
        ret = -EIO;
        goto unmap_window_cs1;
    }

    ret = gpio_direction_output(fpga.fpga_pins.fpga_reset, 1);
    if (ret)
    {
        printk(KERN_ALERT"Failed to set reset pin as output");
        ret = -EIO;
        goto release_reset_pin;
    }

    ret = gpio_request(fpga.fpga_pins.fpga_irq, "sk_fpga_irq_pin");
    if (ret)
    {
        printk(KERN_ALERT"Failed to acqiure fpga irq pin");
        ret = -EIO;
        goto release_reset_pin;
    }

    ret = gpio_direction_input(fpga.fpga_pins.fpga_irq);
    if (ret)
    {
        printk(KERN_ALERT"Failed to set fpga irq pin as input");
        ret = -EIO;
        goto release_irq_pin;
    }

    ret = gpio_request(fpga.fpga_pins.host_irq, "sk_host_irq_pin");
    if (ret)
    {
        printk(KERN_ALERT"Failed to acqiure host irq pin");
        ret = -EIO;
        goto release_irq_pin;
    }

    ret = gpio_direction_output(fpga.fpga_pins.host_irq, 0);
    if (ret)
    {
        printk(KERN_ALERT"Failed to set host irq pin as output");
        ret = -EIO;
        goto release_host_irq_pin;
    }

    ret = sk_fpga_setup_ebicsa();
    if (ret)
    {
        printk(KERN_ALERT"Failed to setup bux matrix");
        ret = -EIO;
        goto release_host_irq_pin;
    }

    // device is not yet opened
    fpga.opened = 0;
    fpga.fpga_addr_sel = FPGA_ADDR_UNDEFINED;

    ret = sk_fpga_setup_dma(pdev);
    if (ret)
    {
        goto release_host_irq_pin;
    }
    
    return ret;

release_host_irq_pin:
    gpio_free(fpga.fpga_pins.host_irq);
release_irq_pin:
    gpio_free(fpga.fpga_pins.fpga_irq);
release_reset_pin:
    gpio_free(fpga.fpga_pins.fpga_reset);
unmap_window_cs1:
    iounmap(fpga.fpga_mem_virt_start_cs1);
release_window_cs1:
    release_mem_region(fpga.fpga_mem_phys_start_cs1, fpga.fpga_mem_window_size);
unmap_window_cs0:
    iounmap(fpga.fpga_mem_virt_start_cs0);
release_window_cs0:
    release_mem_region(fpga.fpga_mem_phys_start_cs0, fpga.fpga_mem_window_size);
free_buf:
    kfree(fpga.fpga_prog_buffer);
misc_dereg:
    misc_deregister(&sk_fpga_dev);
    return ret;
}

int sk_fpga_setup_dma (struct platform_device *pdev)
{
    struct dma_slave_config	slave_config;
    struct device *dev = &pdev->dev;
    int err = 0;

    dma_cap_mask_t mask;
    dma_cap_zero(mask);
    dma_cap_set(DMA_SLAVE, mask);
    
    fpga.fpga_dma_chan = dma_request_slave_channel_reason(dev, "tx - rx");
    if (IS_ERR(fpga.fpga_dma_chan)) 
    {
        err = PTR_ERR(fpga.fpga_dma_chan);
        if (err == -EPROBE_DEFER) 
        {
            dev_warn(dev, "no DMA channel available at the moment\n");
            goto release_chan;
        }
        dev_err(dev, "DMA TX - RX channel not available, SPI unable to use DMA\n");
        err = -EBUSY;
        goto release_chan;
    }

    slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
    slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
    slave_config.dst_addr = 0;
    slave_config.src_addr = 0;
    slave_config.src_maxburst = 1;
    slave_config.dst_maxburst = 1;
    slave_config.device_fc = false;

    slave_config.direction = DMA_DEV_TO_MEM;
    if (dmaengine_slave_config(fpga.fpga_dma_chan, &slave_config)) 
    {
        dev_err(&pdev->dev, "failed to configure dma channel\n");
        err = -EINVAL;
        goto release_chan;
    }

    // allocate buffers
    fpga.dma_buf = dma_alloc_coherent(&pdev->dev, DMA_BUF_SIZE, &fpga.dma_addr_buf, GFP_KERNEL | GFP_DMA);
    if (!fpga.dma_buf) 
    {
        dma_free_coherent(&pdev->dev, DMA_BUF_SIZE, fpga.dma_buf, fpga.dma_addr_buf);
        err = -ENOMEM;
        goto release_chan;
    }
    
    return err;

release_chan:
    if (!IS_ERR(fpga.fpga_dma_chan))
    {
        dma_release_channel(fpga.fpga_dma_chan);
    }
    fpga.fpga_dma_chan = NULL;
    return err;
}

int sk_fpga_dma_config_slave ()
{
    int err = 0;
    struct dma_slave_config	slave_config;
    struct device *dev = &fpga.pdev->dev;
    
    slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
    slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;

    slave_config.dst_addr = 0;
    slave_config.src_addr = 0;
    slave_config.src_maxburst = 1;
    slave_config.dst_maxburst = 1;
    slave_config.device_fc = false;

    slave_config.direction = DMA_DEV_TO_MEM;
    if (dmaengine_slave_config(fpga.fpga_dma_chan, &slave_config)) 
    {
        dev_err(dev, "failed to configure tx - rx dma channel\n");
        err = -EINVAL;
    }
    return err;
}

static int sk_fpga_remove (struct platform_device *pdev)
{
    printk(KERN_ALERT"Removing FPGA driver for SK-AT91SAM9M10G45EK-XC6SLX\n");
    misc_deregister(&sk_fpga_dev);
    kfree(fpga.fpga_prog_buffer);
    iounmap(fpga.fpga_mem_virt_start_cs0);
    release_mem_region(fpga.fpga_mem_phys_start_cs0, fpga.fpga_mem_window_size);
    iounmap(fpga.fpga_mem_virt_start_cs1);
    release_mem_region(fpga.fpga_mem_phys_start_cs1, fpga.fpga_mem_window_size);
    gpio_free(fpga.fpga_pins.fpga_reset);
    gpio_free(fpga.fpga_pins.fpga_irq);
    gpio_free(fpga.fpga_pins.host_irq);
    dma_free_coherent(&pdev->dev, DMA_BUF_SIZE, fpga.dma_buf, fpga.dma_addr_buf);
    dma_release_channel(fpga.fpga_dma_chan);
    return 0;
}

// TODO: try to adopt existing FPGA spi programming code in kernel
int sk_fpga_prepare_to_program (void)
{
    int ret = 0;
    printk(KERN_ALERT"FPGA programming is started");
    // acquire pins to program FPGA
    gpio_free(fpga.fpga_pins.fpga_done);
    gpio_free(fpga.fpga_pins.fpga_din);
    gpio_free(fpga.fpga_pins.fpga_cclk);
    gpio_free(fpga.fpga_pins.fpga_prog);
    ret = gpio_request(fpga.fpga_pins.fpga_prog, "sk_fpga_prog_pin");
    if (ret) {
        printk(KERN_ALERT"Failed to allocate fpga prog pin");
        goto release_prog_pin;
    }
    gpio_direction_output(fpga.fpga_pins.fpga_prog, 1);
    ret = gpio_request(fpga.fpga_pins.fpga_cclk, "sk_fpga_cclk_pin");
    if (ret) {
        printk(KERN_ALERT"Failed to allocate fpga cclk pin");
        goto release_cclk_pin;
    }
    gpio_direction_output(fpga.fpga_pins.fpga_cclk, 1);
    ret = gpio_request(fpga.fpga_pins.fpga_din, "sk_fpga_din_pin");
    if (ret) {
        printk(KERN_ALERT"Failed to allocate fpga din pin");
        goto release_din_pin;
    }
    gpio_direction_output(fpga.fpga_pins.fpga_din, 1);
    ret = gpio_request(fpga.fpga_pins.fpga_done, "sk_fpga_done_pin");
    if (ret) {
        printk(KERN_ALERT"Failed to allocate fpga done pin");
        goto release_done_pin;
    }
    gpio_direction_input(fpga.fpga_pins.fpga_done);

    // perform sort of firmware reset on fpga
    gpio_set_value(fpga.fpga_pins.fpga_prog, 0);
    gpio_set_value(fpga.fpga_pins.fpga_prog, 1);
    return 0;

release_done_pin:
    gpio_free(fpga.fpga_pins.fpga_done);
release_din_pin:
    gpio_free(fpga.fpga_pins.fpga_din);
release_cclk_pin:
    gpio_free(fpga.fpga_pins.fpga_cclk);
release_prog_pin:
    gpio_free(fpga.fpga_pins.fpga_prog);
    return -ENODEV;
}

// TODO: refactoring needed
void sk_fpga_program (const uint8_t* buff, uint32_t bufLen)
{
    int i, j;
    unsigned char byte;
    unsigned char bit;
    for (i = 0; i < bufLen; i++) {
        byte = buff[i];
        for (j = 7; j >= 0; j--) {
            bit = (1 << j) & byte;
            gpio_set_value(fpga.fpga_pins.fpga_din, bit ? 1 : 0);
            gpio_set_value(fpga.fpga_pins.fpga_cclk, 1);
            gpio_set_value(fpga.fpga_pins.fpga_cclk, 0);
        }
    }
}

// TODO: refactoring needed
int sk_fpga_programming_done (void)
{
    int counter, i, done = 0;
    int ret = 0;
    gpio_set_value(fpga.fpga_pins.fpga_din, 1);
    done = gpio_get_value(fpga.fpga_pins.fpga_done);
    counter = 0;
    // toggle fpga clock while done signal appears
    while (!done) 
    {
        gpio_set_value(fpga.fpga_pins.fpga_cclk, 1);
        gpio_set_value(fpga.fpga_pins.fpga_cclk, 0);
        done = gpio_get_value(fpga.fpga_pins.fpga_done);
        counter++;
        if (counter > MAX_WAIT_COUNTER) 
        {
            printk(KERN_ALERT"Failed to get FPGA done pin as high");
            ret = -EIO;
            // might want to set it to undefined
            goto finish;
        }
    }
    // toggle clock a little bit just to ensure nothing's wrong
    for (i = 0; i < 10; i++) 
    {
        gpio_set_value(fpga.fpga_pins.fpga_cclk, 1);
        gpio_set_value(fpga.fpga_pins.fpga_cclk, 0);
    }

finish:
    if (!ret)
        printk(KERN_ALERT"FPGA programming is done");
    // release program pins
    gpio_free(fpga.fpga_pins.fpga_done);
    gpio_free(fpga.fpga_pins.fpga_din);
    gpio_free(fpga.fpga_pins.fpga_cclk);
    gpio_free(fpga.fpga_pins.fpga_prog);
    return ret;
}

int sk_fpga_prog (char* fName)
{
    int ret = 0;
    struct file *f;
    mm_segment_t fs;
    ssize_t len = 0;
    loff_t off = 0;

    if (sk_fpga_prepare_to_program())
    {
        return -ENODEV;
    }

    f = filp_open(fName, O_RDONLY, 0);
    if(f == NULL)
    {
        printk(KERN_ALERT "Failed to open file: %s", fName);
        return -ENOMEM;
    }
    else
    {
        fs = get_fs();
        set_fs(KERNEL_DS);
        do
        {
            len = kernel_read(f, fpga.fpga_prog_buffer, TMP_BUF_SIZE, &off);
            sk_fpga_program(fpga.fpga_prog_buffer, len);
        }
        while (len);
        set_fs(fs);
        filp_close(f, NULL);
    }
    if (sk_fpga_programming_done())
    {
        return -ENODEV;
    }

    return ret;
}

static int sk_fpga_mmap (struct file *file, struct vm_area_struct * vma)
{
    //NOTE: we should really protect these by mutexes and stuff...
    // Ignoring pgoff to determine start of mmap
    int ret = 0;
    unsigned long start = 0;
    unsigned long len   = 0;
    switch (fpga.fpga_addr_sel)
    {
    case FPGA_ADDR_CS0:
        start = (fpga.fpga_mem_phys_start_cs0 >> PAGE_SHIFT);
        break;
    case FPGA_ADDR_CS1:
        start = (fpga.fpga_mem_phys_start_cs1 >> PAGE_SHIFT);
        break;
    case FPGA_ADDR_DMA:
        BUG_ON(fpga.dma_addr_buf & (PAGE_SIZE - 1));
        start = (fpga.dma_addr_buf >> PAGE_SHIFT);
        break;
    default:
        printk(KERN_ALERT"Wrong address space selector");
        BUG_ON(1);
        break;
    }
    // (vm_end - vm_start) should be equal to window size
    len = (vma->vm_end - vma->vm_start);
    BUG_ON(vma->vm_pgoff);
    if (fpga.fpga_addr_sel == FPGA_ADDR_DMA)
    {
        BUG_ON(DMA_BUF_SIZE != (vma->vm_end - vma->vm_start));
    }
    else
    {
        BUG_ON(fpga.fpga_mem_window_size != (vma->vm_end - vma->vm_start));
    }

    // mark these pages as IO
    vma->vm_page_prot = vm_get_page_prot(vma->vm_flags | VM_IO);

    //io_remap_pfn_range call...
    ret = io_remap_pfn_range(vma, vma->vm_start, start, len, vma->vm_page_prot);
    if (ret) 
    {
        printk(KERN_ALERT"fpga mmap failed :(\n");
        return ret;
    }
    return ret;
}

static const struct of_device_id sk_fpga_of_match_table[] = {
    { .compatible = "sk,at91-xc6slx", },
    { /* end of list */ }
};
MODULE_DEVICE_TABLE(of, sk_fpga_of_match_table);

static struct platform_driver sk_fpga_driver = {
    .probe         = sk_fpga_probe,
    .remove        = sk_fpga_remove,
    .driver        = {
        .owner          = THIS_MODULE,
        .name           = "fpga",
        .of_match_table = of_match_ptr(sk_fpga_of_match_table),
    },
};

module_platform_driver(sk_fpga_driver);
MODULE_AUTHOR("Alexey Baturo <baturo.alexey@gmail.com>");
MODULE_DESCRIPTION("Driver for Xilinx Spartan6 xc6slx16 fpga for StarterKit AT91SAM9M10G45EK-XC6SLX board");
MODULE_LICENSE("GPL v2");
