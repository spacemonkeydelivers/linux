#ifndef SK_FPGA_IOCTL_HEADER
#define SK_FPGA_IOCTL_HEADER

#include <linux/ioctl.h>

struct sk_fpga_data
{
    uint32_t address;
    uint16_t data;
};

struct sk_fpga_smc_timings
{
    uint32_t setup; // setup ebi timings
    uint32_t pulse; // pulse ebi timings
    uint32_t cycle; // cycle ebi timings
    uint32_t mode;  // ebi mode
    uint8_t  cs;    // chip select
};

enum fpga_addr_selector
{
    FPGA_ADDR_UNDEFINED = 0,
    FPGA_ADDR_CS0,
    FPGA_ADDR_CS1,
    FPGA_ADDR_DMA,
    FPGA_ADDR_LAST,
};

#define SKFP_IOC_MAGIC 0x81

// ioctl to write data to FPGA
#define SKFPGA_SHORT_WRITE _IOW(SKFP_IOC_MAGIC, 1, struct sk_fpga_data)
// ioctl to read data from FPGA
#define SKFPGA_SHORT_READ _IOR(SKFP_IOC_MAGIC, 2, struct sk_fpga_data)

// ioctl to set SMC timings
#define SKFPGA_SMC_TIMINGS_WRITE _IOW(SKFP_IOC_MAGIC, 3, struct sk_fpga_smc_timings)
// ioctl to request SMC timings
#define SKFPGA_SMC_TIMINGS_READ _IOR(SKFP_IOC_MAGIC, 4, struct sk_fpga_smc_timings)

// ioctl to start programming FPGA
#define SKFPGA_PROG_START _IO(SKFP_IOC_MAGIC, 5)
// ioctl to finish programming FPGA
#define SKFPGA_PROG_FINISH _IOR(SKFP_IOC_MAGIC, 7, uint8_t)

// ioctl to set reset pin level
#define SKFPGA_RESET_WRITE _IOW(SKFP_IOC_MAGIC, 8, uint8_t)
// ioctl to get reset pin level
#define SKFPGA_RESET_READ _IOR(SKFP_IOC_MAGIC, 9, uint8_t)

// ioctl to set arm-to-fpga pin level
#define SKFPGA_FPGA_IRQ_WRITE _IOW(SKFP_IOC_MAGIC, 10, uint8_t)
// ioctl to get arm-to-fpga pin level
#define SKFPGA_FPGA_IRQ_READ _IOR(SKFP_IOC_MAGIC, 11, uint8_t)

// ioctl to set proper address space
#define SKFPGA_ADDR_SEL_WRITE _IOW(SKFP_IOC_MAGIC, 12, uint32_t)
// ioctl to get current address space
#define SKFPGA_ADDR_SEL_READ _IOR(SKFP_IOC_MAGIC, 13, uint32_t)

// TODO: implement later
// ioctl to set fpga-to-arm as irq
//#define SKFPGA_IOSFPGAIRQ _IOR(SKFP_IOC_MAGIC, 10, uint8_t)
// ioctl to set address space selector
//#define SKFPGA_IOSADDRSEL _IOR(SKFP_IOC_MAGIC, 12, uint8_t)
// ioctl to get address space selector
//#define SKFPGA_IOGADDRSEL _IOR(SKFP_IOC_MAGIC, 13, uint8_t)
// ioctl to start DMA transaction
//#define SKFPGA_IOSDMA _IOR(SKFP_IOC_MAGIC, 14, struct sk_fpga_dma_transaction)
// ioctl to set pid
//#define SKFPGA_IOSPID _IOR(SKFP_IOC_MAGIC, 15, int)

// ioctl to set the current mode for the FPGA
//#define SKFPGA_IOSMODE _IOR(SKFP_IOC_MAGIC, 3, int)
// ioctl to get the current mode for the FPGA
//#define SKFPGA_IOQMODE _IOW(SKFP_IOC_MAGIC, 4, int)
// ioctl to set the current mode for the FPGA
//#define SKFPGA_IOSPROG_DONE _IOR(SKFP_IOC_MAGIC, 5, int)
// ioctl to get the current mode for the FPGA
//#define SKFPGA_IOQPROG_DONE _IOW(SKFP_IOC_MAGIC, 6, int)

#endif
