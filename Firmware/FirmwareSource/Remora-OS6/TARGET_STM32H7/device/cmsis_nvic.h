/* mbed Microcontroller Library
 * SPDX-License-Identifier: BSD-3-Clause
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2016-2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
*/

#ifndef MBED_CMSIS_NVIC_H
#define MBED_CMSIS_NVIC_H

#if !defined(MBED_ROM_START)
#define MBED_ROM_START  0x8000000
#endif

#if !defined(MBED_APP_SIZE)
  #define MBED_APP_SIZE   0x100000      ; 1MB Flash for H723
  #define MBED_APP_SIZE   0x40000
#endif

#if !defined(MBED_RAM_SIZE)
#define MBED_RAM_SIZE  0x20000  // 128 KB
#endif

#if !defined(MBED_RAM1_START)
#define MBED_RAM1_START  0x24000000
#endif

#ifndef MBED_RAM1_SIZE
#define MBED_RAM1_SIZE  0x50000  // 320 KB
#endif

#if !defined(MBED_CONF_TARGET_BOOT_STACK_SIZE)
/* This value is normally defined by the tools to 0x1000 for bare metal and 0x400 for RTOS */
#if defined(MBED_BOOT_STACK_SIZE)
#define MBED_CONF_TARGET_BOOT_STACK_SIZE MBED_BOOT_STACK_SIZE
#else
#define MBED_CONF_TARGET_BOOT_STACK_SIZE 0x200
#endif
#endif

#if !defined(MBED_RAM_START)
#define MBED_RAM_START  0x20000000
#endif


// 0x20000000 - 0x2001FFFF 128K DTCM
// 0x24000000 - 0x2404FFFF 320K AXI SRAM
// 0x30000000 - 0x30003FFF 16K SRAM1
// 0x30004000 - 0x30007FFF 16K SRAM2
// 0x38000000 - 0x38003FFF 16K SRAM4

#define NVIC_NUM_VECTORS        180
#define NVIC_RAM_VECTOR_ADDRESS MBED_RAM_START

/* Round up VECTORS_SIZE to 8 bytes */
#define VECTORS_SIZE  (((NVIC_NUM_VECTORS * 4) + 7) AND ~7)

#define MBED_CRASH_REPORT_RAM_START (NVIC_RAM_VECTOR_ADDRESS + VECTORS_SIZE)
#define MBED_CRASH_REPORT_RAM_SIZE  0x100


#endif
