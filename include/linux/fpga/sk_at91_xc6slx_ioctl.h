#ifndef SK_FPGA_IOCTL_HEADER
#define SK_FPGA_IOCTL_HEADER

#include <linux/ioctl.h>

#define SKFP_IOC_MAGIC 0x81
// ioctl to write data to FPGA
#define SKFPGA_IOSDATA _IOW(SKFP_IOC_MAGIC, 1, struct sk_fpga_data)
// ioctl to read data from FPGA
#define SKFPGA_IOGDATA _IOW(SKFP_IOC_MAGIC, 2, struct sk_fpga_data)
// ioctl to set SMC timings
#define SKFPGA_IOSSMCTIMINGS _IOW(SKFP_IOC_MAGIC, 3, struct sk_fpga_smc_timings)
// ioctl to request SMC timings
#define SKFPGA_IOGSMCTIMINGS _IOR(SKFP_IOC_MAGIC, 4, struct sk_fpga_smc_timings)
// ioctl to programm FPGA
#define SKFPGA_IOSPROG _IOR(SKFP_IOC_MAGIC, 5, char[PROG_FILE_NAME_LEN])
// ioctl to set reset pin level
#define SKFPGA_IOSRESET _IOR(SKFP_IOC_MAGIC, 6, uint8_t)
// ioctl to get reset pin level
#define SKFPGA_IOGRESET _IOR(SKFP_IOC_MAGIC, 7, uint8_t)
// ioctl to set arm-to-fpga pin level
#define SKFPGA_IOSHOSTIRQ _IOR(SKFP_IOC_MAGIC, 8, uint8_t)
// ioctl to get arm-to-fpga pin level
#define SKFPGA_IOGHOSTIRQ _IOR(SKFP_IOC_MAGIC, 9, uint8_t)
// TODO: implement later
// ioctl to set fpga-to-arm as irq
#define SKFPGA_IOSFPGAIRQ _IOR(SKFP_IOC_MAGIC, 10, uint8_t)
// ioctl to set address space selector
#define SKFPGA_IOSADDRSEL _IOR(SKFP_IOC_MAGIC, 12, uint8_t)
// ioctl to get address space selector
#define SKFPGA_IOGADDRSEL _IOR(SKFP_IOC_MAGIC, 13, uint8_t)
// ioctl to start DMA transaction
#define SKFPGA_IOSDMA _IOR(SKFP_IOC_MAGIC, 14, struct sk_fpga_dma_transaction)
// ioctl to set pid
#define SKFPGA_IOSPID _IOR(SKFP_IOC_MAGIC, 15, int)

// ioctl to set the current mode for the FPGA
//#define SKFPGA_IOSMODE _IOR(SKFP_IOC_MAGIC, 3, int)
// ioctl to get the current mode for the FPGA
//#define SKFPGA_IOQMODE _IOW(SKFP_IOC_MAGIC, 4, int)
// ioctl to set the current mode for the FPGA
//#define SKFPGA_IOSPROG_DONE _IOR(SKFP_IOC_MAGIC, 5, int)
// ioctl to get the current mode for the FPGA
//#define SKFPGA_IOQPROG_DONE _IOW(SKFP_IOC_MAGIC, 6, int)

#endif
