/*******************************************************************************
 * Copyright 2019-2021 Microchip FPGA Embedded Systems Solutions.
 *
 * SPDX-License-Identifier: MIT
 *
 * @file hw_pmp_hart2.h
 * @author Microchip-FPGA Embedded Systems Solutions
 *
 *
 * Note 1: This file should not be edited. If you need to modify a parameter
 * without going through regenerating using the MSS Configurator Libero flow 
 * or editing the associated xml file
 * the following method is recommended: 

 * 1. edit the following file 
 * boards/your_board/platform_config/mpfs_hal_config/mss_sw_config.h

 * 2. define the value you want to override there.
 * (Note: There is a commented example in the platform directory)

 * Note 2: The definition in mss_sw_config.h takes precedence, as
 * mss_sw_config.h is included prior to the generated header files located in
 * boards/your_board/fpga_design_config
 *
 */

#ifndef HW_PMP_HART2_H_
#define HW_PMP_HART2_H_


#ifdef __cplusplus
extern  "C" {
#endif

#if !defined (LIBERO_SETTING_HART2_CSR_PMPCFG0)
/*PMP configuration for 8 adress regions, bit 0 read, bit 1 write, bit 2
execute, bit 7 disable, bits 3,4 address format (0x18 => NAPOT) */
#define LIBERO_SETTING_HART2_CSR_PMPCFG0    0x9B9B9B9B00000000ULL
    /* PMP0CFG                           [0:8]   RW value= 0x00 */
    /* PMP1CFG                           [8:8]   RW value= 0x00 */
    /* PMP2CFG                           [16:8]  RW value= 0x00 */
    /* PMP3CFG                           [24:8]  RW value= 0x00 */
    /* PMP4CFG                           [32:8]  RW value= 0x9B */
    /* PMP5CFG                           [40:8]  RW value= 0x9B */
    /* PMP6CFG                           [48:8]  RW value= 0x9B */
    /* PMP7CFG                           [56:8]  RW value= 0x9B */
#endif
#if !defined (LIBERO_SETTING_HART2_CSR_PMPCFG2)
/*PMP configuration for 8 address regions, bit 0 read, bit 1 write, bit 2
execute, bit 7 disable, bits 3,4 address format (0x18 => NAPOT) */
#define LIBERO_SETTING_HART2_CSR_PMPCFG2    0x00989B9B9F9B9B9BULL
    /* PMP8CFG                           [0:8]   RW value= 0x9B */
    /* PMP9CFG                           [8:8]   RW value= 0x9B */
    /* PMP10CFG                          [16:8]  RW value= 0x9B */
    /* PMP11CFG                          [24:8]  RW value= 0x9F */
    /* PMP12CFG                          [32:8]  RW value= 0x9B */
    /* PMP13CFG                          [40:8]  RW value= 0x9B */
    /* PMP14CFG                          [48:8]  RW value= 0x98 */
    /* PMP15CFG                          [56:8]  RW value= 0x0 */
#endif
#if !defined (LIBERO_SETTING_HART2_CSR_PMPADDR0)
/*PMP ADRESS and size, format determined from bit 3 and 4 of configuration byte
in CSR_PMPCFGx */
#define LIBERO_SETTING_HART2_CSR_PMPADDR0    0x0000000000000000ULL
    /* CSR_PMPADDR0                      [0:64]  RW value= 0x00 */
#endif
#if !defined (LIBERO_SETTING_HART2_CSR_PMPADDR1)
/*PMP ADRESS and size, format determined from bit 3 and 4 of configuration byte
in CSR_PMPCFGx */
#define LIBERO_SETTING_HART2_CSR_PMPADDR1    0x0000000000000000ULL
    /* CSR_PMPADDR1                      [0:64]  RW value= 0x00 */
#endif
#if !defined (LIBERO_SETTING_HART2_CSR_PMPADDR2)
/*PMP ADRESS and size, format determined from bit 3 and 4 of configuration byte
in CSR_PMPCFGx */
#define LIBERO_SETTING_HART2_CSR_PMPADDR2    0x0000000000000000ULL
    /* CSR_PMPADDR2                      [0:64]  RW value= 0x00 */
#endif
#if !defined (LIBERO_SETTING_HART2_CSR_PMPADDR3)
/*PMP ADRESS and size, format determined from bit 3 and 4 of configuration byte
in CSR_PMPCFGx */
#define LIBERO_SETTING_HART2_CSR_PMPADDR3    0x0000000000000000ULL
    /* CSR_PMPADDR3                      [0:64]  RW value= 0x00 */
#endif
#if !defined (LIBERO_SETTING_HART2_CSR_PMPADDR4)
/*PMP ADRESS and size, format determined from bit 3 and 4 of configuration byte
in CSR_PMPCFGx */
#define LIBERO_SETTING_HART2_CSR_PMPADDR4    0x0000000008047FFFULL
    /* CSR_PMPADDR4                      [0:64]  RW value= 0x8047FFF */
#endif
#if !defined (LIBERO_SETTING_HART2_CSR_PMPADDR5)
/*PMP ADRESS and size, format determined from bit 3 and 4 of configuration byte
in CSR_PMPCFGx */
#define LIBERO_SETTING_HART2_CSR_PMPADDR5    0x000000000307FFFFULL
    /* CSR_PMPADDR5                      [0:64]  RW value= 0x307FFFF */
#endif
#if !defined (LIBERO_SETTING_HART2_CSR_PMPADDR6)
/*PMP ADRESS and size, format determined from bit 3 and 4 of configuration byte
in CSR_PMPCFGx */
#define LIBERO_SETTING_HART2_CSR_PMPADDR6    0x00000000080009FFULL
    /* CSR_PMPADDR6                      [0:64]  RW value= 0x80009FF */
#endif
#if !defined (LIBERO_SETTING_HART2_CSR_PMPADDR7)
/*PMP ADRESS and size, format determined from bit 3 and 4 of configuration byte
in CSR_PMPCFGx */
#define LIBERO_SETTING_HART2_CSR_PMPADDR7    0x00000000080021FFULL
    /* CSR_PMPADDR7                      [0:64]  RW value= 0x80021FF */
#endif
#if !defined (LIBERO_SETTING_HART2_CSR_PMPADDR8)
/*PMP ADRESS and size, format determined from bit 3 and 4 of configuration byte
in CSR_PMPCFGx */
#define LIBERO_SETTING_HART2_CSR_PMPADDR8    0x00000000080805FFULL
    /* CSR_PMPADDR8                      [0:64]  RW value= 0x80805FF */
#endif
#if !defined (LIBERO_SETTING_HART2_CSR_PMPADDR9)
/*PMP ADRESS and size, format determined from bit 3 and 4 of configuration byte
in CSR_PMPCFGx */
#define LIBERO_SETTING_HART2_CSR_PMPADDR9    0x00000000085FFFFFULL
    /* CSR_PMPADDR9                      [0:64]  RW value= 0x85FFFFF */
#endif
#if !defined (LIBERO_SETTING_HART2_CSR_PMPADDR10)
/*PMP ADRESS and size, format determined from bit 3 and 4 of configuration byte
in CSR_PMPCFGx */
#define LIBERO_SETTING_HART2_CSR_PMPADDR10    0x0000000008803FFFULL
    /* CSR_PMPADDR10                     [0:64]  RW value= 0x8803FFF */
#endif
#if !defined (LIBERO_SETTING_HART2_CSR_PMPADDR11)
/*PMP ADRESS and size, format determined from bit 3 and 4 of configuration byte
in CSR_PMPCFGx */
#define LIBERO_SETTING_HART2_CSR_PMPADDR11    0x000000002003FFFFULL
    /* CSR_PMPADDR11                     [0:64]  RW value= 0x2003FFFF */
#endif
#if !defined (LIBERO_SETTING_HART2_CSR_PMPADDR12)
/*PMP ADRESS and size, format determined from bit 3 and 4 of configuration byte
in CSR_PMPCFGx */
#define LIBERO_SETTING_HART2_CSR_PMPADDR12    0x00000000008041FFULL
    /* CSR_PMPADDR12                     [0:64]  RW value= 0x8041FF */
#endif
#if !defined (LIBERO_SETTING_HART2_CSR_PMPADDR13)
/*PMP ADRESS and size, format determined from bit 3 and 4 of configuration byte
in CSR_PMPCFGx */
#define LIBERO_SETTING_HART2_CSR_PMPADDR13    0x0000000000801FFFULL
    /* CSR_PMPADDR13                     [0:64]  RW value= 0x801FFF */
#endif
#if !defined (LIBERO_SETTING_HART2_CSR_PMPADDR14)
/*PMP ADRESS and size, format determined from bit 3 and 4 of configuration byte
in CSR_PMPCFGx */
#define LIBERO_SETTING_HART2_CSR_PMPADDR14    0xFFFFFFFFFFFFFFFFULL
    /* CSR_PMPADDR14                     [0:64]  RW value= 0xFFFFFFFFFFFFFFFF */
#endif
#if !defined (LIBERO_SETTING_HART2_CSR_PMPADDR15)
/*PMP ADRESS and size, format determined from bit 3 and 4 of configuration byte
in CSR_PMPCFGx */
#define LIBERO_SETTING_HART2_CSR_PMPADDR15    0x0000000000000000ULL
    /* CSR_PMPADDR15                     [0:64]  RW value= 0x0 */
#endif

#ifdef __cplusplus
}
#endif


#endif /* #ifdef HW_PMP_HART2_H_ */

