/***************************************************************************
 *     Copyright (c) 1999-2009, Broadcom Corporation
 *     All Rights Reserved
 *     Confidential Property of Broadcom Corporation
 *
 *
 * THIS SOFTWARE MAY ONLY BE USED SUBJECT TO AN EXECUTED SOFTWARE LICENSE
 * AGREEMENT  BETWEEN THE USER AND BROADCOM.  YOU HAVE NO RIGHT TO USE OR
 * EXPLOIT THIS MATERIAL EXCEPT SUBJECT TO THE TERMS OF SUCH AN AGREEMENT.
 *
 * $brcm_Workfile: bchp_armcr4_bridge.h $
 * $brcm_Revision: Hydra_Software_Devel/1 $
 * $brcm_Date: 7/17/09 8:28p $
 *
 * Module Description:
 *                     DO NOT EDIT THIS FILE DIRECTLY
 *
 * This module was generated magically with RDB from a source description
 * file. You must edit the source file for changes to be made to this file.
 *
 *
 * Date:           Generated on         Fri Jul 17 19:42:56 2009
 *                 MD5 Checksum         2914699efc3fb3edefca5cb4f4f38b34
 *
 * Compiled with:  RDB Utility          combo_header.pl
 *                 RDB Parser           3.0
 *                 unknown              unknown
 *                 Perl Interpreter     5.008008
 *                 Operating System     linux
 *
 * Revision History:
 *
 * $brcm_Log: /magnum/basemodules/chp/70015/rdb/a0/bchp_armcr4_bridge.h $
 * 
 * Hydra_Software_Devel/1   7/17/09 8:28p albertl
 * PR56880: Initial revision.
 *
 ***************************************************************************/

#ifndef BCHP_ARMCR4_BRIDGE_H__
#define BCHP_ARMCR4_BRIDGE_H__

/***************************************************************************
 *ARMCR4_BRIDGE - ARM Cortex R4 Bridge control registers
 ***************************************************************************/
#define BCHP_ARMCR4_BRIDGE_REG_CORE_REV_ID       0x000e0000 /* ARM Cortex R4 bridge revision ID */
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL        0x000e0004 /* Bridge interface and buffer configuration */
#define BCHP_ARMCR4_BRIDGE_REG_ARM_CTL           0x000e0008 /* ARM core configuration */
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_STS        0x000e0014 /* Bridge interface and buffer status */
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_PCI1         0x000e0018 /* PCI mailbox #1 */
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_ARM1         0x000e001c /* ARM mailbox #1 */
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_PCI2         0x000e0020 /* PCI mailbox #2 */
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_ARM2         0x000e0024 /* ARM mailbox #2 */
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_PCI3         0x000e0028 /* PCI mailbox #3 */
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_ARM3         0x000e002c /* ARM mailbox #3 */
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_PCI4         0x000e0030 /* PCI mailbox #4 */
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_ARM4         0x000e0034 /* ARM mailbox #4 */
#define BCHP_ARMCR4_BRIDGE_REG_SEMAPHORE_1       0x000e0038 /* CPU semaphore #1 */
#define BCHP_ARMCR4_BRIDGE_REG_SEMAPHORE_2       0x000e003c /* CPU semaphore #2 */
#define BCHP_ARMCR4_BRIDGE_REG_SEMAPHORE_3       0x000e0040 /* CPU semaphore #3 */
#define BCHP_ARMCR4_BRIDGE_REG_SEMAPHORE_4       0x000e0044 /* CPU semaphore #4 */
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_1         0x000e0048 /* CPU scratchpad #1 */
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_2         0x000e004c /* CPU scratchpad #2 */
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_3         0x000e0050 /* CPU scratchpad #3 */
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_4         0x000e0054 /* CPU scratchpad #4 */
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_5         0x000e0058 /* CPU scratchpad #5 */
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_6         0x000e005c /* CPU scratchpad #6 */
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_7         0x000e0060 /* CPU scratchpad #7 */
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_8         0x000e0064 /* CPU scratchpad #8 */
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_9         0x000e0068 /* CPU scratchpad #9 */
#define BCHP_ARMCR4_BRIDGE_REG_PERF_CONFIG       0x000e006c /* Performance monitor configuration */
#define BCHP_ARMCR4_BRIDGE_REG_PERF_LIMIT        0x000e0070 /* Performance monitor count threshold */
#define BCHP_ARMCR4_BRIDGE_REG_PERF_WR_CNT       0x000e0074 /* Counts the number of merge buffer updates (hits + misses) */
#define BCHP_ARMCR4_BRIDGE_REG_PERF_WR_MISS      0x000e0078 /* Counts the number of merge buffer misses */
#define BCHP_ARMCR4_BRIDGE_REG_PERF_RD_CNT       0x000e007c /* Counts the number of prefetch buffer accesses (hits + misses) */
#define BCHP_ARMCR4_BRIDGE_REG_PERF_RD_MISS      0x000e0080 /* Counts the number of prefetch buffer misses */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM1        0x000e0084 /* ARM memory TM1 control register */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM2        0x000e0088 /* ARM memory TM2 control register */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM3        0x000e008c /* ARM memory TM3 control register */
#define BCHP_ARMCR4_BRIDGE_REG_FIFO_STATUS       0x000e0090 /* Fifo Status */
#define BCHP_ARMCR4_BRIDGE_REG_BORCH_STATUS      0x000e0094 /* Bridge Out-of-range Checker Status */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM4        0x000e0098 /* ARM memory TM4 control register */

/***************************************************************************
 *REG_CORE_REV_ID - ARM Cortex R4 bridge revision ID
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_CORE_REV_ID :: reserved0 [31:16] */
#define BCHP_ARMCR4_BRIDGE_REG_CORE_REV_ID_reserved0_MASK          0xffff0000
#define BCHP_ARMCR4_BRIDGE_REG_CORE_REV_ID_reserved0_SHIFT         16

/* ARMCR4_BRIDGE :: REG_CORE_REV_ID :: MAJOR [15:08] */
#define BCHP_ARMCR4_BRIDGE_REG_CORE_REV_ID_MAJOR_MASK              0x0000ff00
#define BCHP_ARMCR4_BRIDGE_REG_CORE_REV_ID_MAJOR_SHIFT             8

/* ARMCR4_BRIDGE :: REG_CORE_REV_ID :: MINOR [07:00] */
#define BCHP_ARMCR4_BRIDGE_REG_CORE_REV_ID_MINOR_MASK              0x000000ff
#define BCHP_ARMCR4_BRIDGE_REG_CORE_REV_ID_MINOR_SHIFT             0

/***************************************************************************
 *REG_BRIDGE_CTL - Bridge interface and buffer configuration
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_BRIDGE_CTL :: reserved0 [31:24] */
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL_reserved0_MASK           0xff000000
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL_reserved0_SHIFT          24

/* ARMCR4_BRIDGE :: REG_BRIDGE_CTL :: timeout [23:12] */
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL_timeout_MASK             0x00fff000
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL_timeout_SHIFT            12

/* ARMCR4_BRIDGE :: REG_BRIDGE_CTL :: reserved1 [11:09] */
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL_reserved1_MASK           0x00000e00
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL_reserved1_SHIFT          9

/* ARMCR4_BRIDGE :: REG_BRIDGE_CTL :: ccb_space_alias_en [08:08] */
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL_ccb_space_alias_en_MASK  0x00000100
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL_ccb_space_alias_en_SHIFT 8

/* ARMCR4_BRIDGE :: REG_BRIDGE_CTL :: timeout_en [07:07] */
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL_timeout_en_MASK          0x00000080
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL_timeout_en_SHIFT         7

/* ARMCR4_BRIDGE :: REG_BRIDGE_CTL :: io_sync_en [06:06] */
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL_io_sync_en_MASK          0x00000040
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL_io_sync_en_SHIFT         6

/* ARMCR4_BRIDGE :: REG_BRIDGE_CTL :: prefetch_en [05:05] */
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL_prefetch_en_MASK         0x00000020
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL_prefetch_en_SHIFT        5

/* ARMCR4_BRIDGE :: REG_BRIDGE_CTL :: gather_en [04:04] */
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL_gather_en_MASK           0x00000010
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL_gather_en_SHIFT          4

/* ARMCR4_BRIDGE :: REG_BRIDGE_CTL :: reserved2 [03:02] */
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL_reserved2_MASK           0x0000000c
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL_reserved2_SHIFT          2

/* ARMCR4_BRIDGE :: REG_BRIDGE_CTL :: arm_run_request [01:01] */
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL_arm_run_request_MASK     0x00000002
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL_arm_run_request_SHIFT    1

/* ARMCR4_BRIDGE :: REG_BRIDGE_CTL :: bridge_soft_rst [00:00] */
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL_bridge_soft_rst_MASK     0x00000001
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_CTL_bridge_soft_rst_SHIFT    0

/***************************************************************************
 *REG_ARM_CTL - ARM core configuration
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_ARM_CTL :: reserved0 [31:05] */
#define BCHP_ARMCR4_BRIDGE_REG_ARM_CTL_reserved0_MASK              0xffffffe0
#define BCHP_ARMCR4_BRIDGE_REG_ARM_CTL_reserved0_SHIFT             5

/* ARMCR4_BRIDGE :: REG_ARM_CTL :: DAP_DBGEN [04:04] */
#define BCHP_ARMCR4_BRIDGE_REG_ARM_CTL_DAP_DBGEN_MASK              0x00000010
#define BCHP_ARMCR4_BRIDGE_REG_ARM_CTL_DAP_DBGEN_SHIFT             4

/* ARMCR4_BRIDGE :: REG_ARM_CTL :: CR4_DBGEN [03:03] */
#define BCHP_ARMCR4_BRIDGE_REG_ARM_CTL_CR4_DBGEN_MASK              0x00000008
#define BCHP_ARMCR4_BRIDGE_REG_ARM_CTL_CR4_DBGEN_SHIFT             3

/* ARMCR4_BRIDGE :: REG_ARM_CTL :: CFGNMFI [02:02] */
#define BCHP_ARMCR4_BRIDGE_REG_ARM_CTL_CFGNMFI_MASK                0x00000004
#define BCHP_ARMCR4_BRIDGE_REG_ARM_CTL_CFGNMFI_SHIFT               2

/* ARMCR4_BRIDGE :: REG_ARM_CTL :: TEINIT [01:01] */
#define BCHP_ARMCR4_BRIDGE_REG_ARM_CTL_TEINIT_MASK                 0x00000002
#define BCHP_ARMCR4_BRIDGE_REG_ARM_CTL_TEINIT_SHIFT                1

/* ARMCR4_BRIDGE :: REG_ARM_CTL :: CFGEE [00:00] */
#define BCHP_ARMCR4_BRIDGE_REG_ARM_CTL_CFGEE_MASK                  0x00000001
#define BCHP_ARMCR4_BRIDGE_REG_ARM_CTL_CFGEE_SHIFT                 0

/***************************************************************************
 *REG_BRIDGE_STS - Bridge interface and buffer status
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_BRIDGE_STS :: reserved0 [31:06] */
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_STS_reserved0_MASK           0xffffffc0
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_STS_reserved0_SHIFT          6

/* ARMCR4_BRIDGE :: REG_BRIDGE_STS :: BRIDGE_RESET_DONE [05:05] */
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_STS_BRIDGE_RESET_DONE_MASK   0x00000020
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_STS_BRIDGE_RESET_DONE_SHIFT  5

/* ARMCR4_BRIDGE :: REG_BRIDGE_STS :: ARM_STANDBYWFI [04:04] */
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_STS_ARM_STANDBYWFI_MASK      0x00000010
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_STS_ARM_STANDBYWFI_SHIFT     4

/* ARMCR4_BRIDGE :: REG_BRIDGE_STS :: prefetch_buff_idle [03:03] */
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_STS_prefetch_buff_idle_MASK  0x00000008
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_STS_prefetch_buff_idle_SHIFT 3

/* ARMCR4_BRIDGE :: REG_BRIDGE_STS :: prefetch_buff_valid [02:02] */
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_STS_prefetch_buff_valid_MASK 0x00000004
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_STS_prefetch_buff_valid_SHIFT 2

/* ARMCR4_BRIDGE :: REG_BRIDGE_STS :: merge_buff_idle [01:01] */
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_STS_merge_buff_idle_MASK     0x00000002
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_STS_merge_buff_idle_SHIFT    1

/* ARMCR4_BRIDGE :: REG_BRIDGE_STS :: merge_buff_valid [00:00] */
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_STS_merge_buff_valid_MASK    0x00000001
#define BCHP_ARMCR4_BRIDGE_REG_BRIDGE_STS_merge_buff_valid_SHIFT   0

/***************************************************************************
 *REG_MBOX_PCI1 - PCI mailbox #1
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_MBOX_PCI1 :: mbox [31:00] */
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_PCI1_mbox_MASK                 0xffffffff
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_PCI1_mbox_SHIFT                0

/***************************************************************************
 *REG_MBOX_ARM1 - ARM mailbox #1
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_MBOX_ARM1 :: mbox [31:00] */
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_ARM1_mbox_MASK                 0xffffffff
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_ARM1_mbox_SHIFT                0

/***************************************************************************
 *REG_MBOX_PCI2 - PCI mailbox #2
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_MBOX_PCI2 :: mbox [31:00] */
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_PCI2_mbox_MASK                 0xffffffff
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_PCI2_mbox_SHIFT                0

/***************************************************************************
 *REG_MBOX_ARM2 - ARM mailbox #2
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_MBOX_ARM2 :: mbox [31:00] */
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_ARM2_mbox_MASK                 0xffffffff
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_ARM2_mbox_SHIFT                0

/***************************************************************************
 *REG_MBOX_PCI3 - PCI mailbox #3
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_MBOX_PCI3 :: mbox [31:00] */
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_PCI3_mbox_MASK                 0xffffffff
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_PCI3_mbox_SHIFT                0

/***************************************************************************
 *REG_MBOX_ARM3 - ARM mailbox #3
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_MBOX_ARM3 :: mbox [31:00] */
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_ARM3_mbox_MASK                 0xffffffff
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_ARM3_mbox_SHIFT                0

/***************************************************************************
 *REG_MBOX_PCI4 - PCI mailbox #4
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_MBOX_PCI4 :: mbox [31:00] */
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_PCI4_mbox_MASK                 0xffffffff
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_PCI4_mbox_SHIFT                0

/***************************************************************************
 *REG_MBOX_ARM4 - ARM mailbox #4
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_MBOX_ARM4 :: mbox [31:00] */
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_ARM4_mbox_MASK                 0xffffffff
#define BCHP_ARMCR4_BRIDGE_REG_MBOX_ARM4_mbox_SHIFT                0

/***************************************************************************
 *REG_SEMAPHORE_1 - CPU semaphore #1
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_SEMAPHORE_1 :: reserved0 [31:08] */
#define BCHP_ARMCR4_BRIDGE_REG_SEMAPHORE_1_reserved0_MASK          0xffffff00
#define BCHP_ARMCR4_BRIDGE_REG_SEMAPHORE_1_reserved0_SHIFT         8

/* ARMCR4_BRIDGE :: REG_SEMAPHORE_1 :: semaphore [07:00] */
#define BCHP_ARMCR4_BRIDGE_REG_SEMAPHORE_1_semaphore_MASK          0x000000ff
#define BCHP_ARMCR4_BRIDGE_REG_SEMAPHORE_1_semaphore_SHIFT         0

/***************************************************************************
 *REG_SEMAPHORE_2 - CPU semaphore #2
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_SEMAPHORE_2 :: reserved0 [31:08] */
#define BCHP_ARMCR4_BRIDGE_REG_SEMAPHORE_2_reserved0_MASK          0xffffff00
#define BCHP_ARMCR4_BRIDGE_REG_SEMAPHORE_2_reserved0_SHIFT         8

/* ARMCR4_BRIDGE :: REG_SEMAPHORE_2 :: semaphore [07:00] */
#define BCHP_ARMCR4_BRIDGE_REG_SEMAPHORE_2_semaphore_MASK          0x000000ff
#define BCHP_ARMCR4_BRIDGE_REG_SEMAPHORE_2_semaphore_SHIFT         0

/***************************************************************************
 *REG_SEMAPHORE_3 - CPU semaphore #3
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_SEMAPHORE_3 :: reserved0 [31:08] */
#define BCHP_ARMCR4_BRIDGE_REG_SEMAPHORE_3_reserved0_MASK          0xffffff00
#define BCHP_ARMCR4_BRIDGE_REG_SEMAPHORE_3_reserved0_SHIFT         8

/* ARMCR4_BRIDGE :: REG_SEMAPHORE_3 :: semaphore [07:00] */
#define BCHP_ARMCR4_BRIDGE_REG_SEMAPHORE_3_semaphore_MASK          0x000000ff
#define BCHP_ARMCR4_BRIDGE_REG_SEMAPHORE_3_semaphore_SHIFT         0

/***************************************************************************
 *REG_SEMAPHORE_4 - CPU semaphore #4
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_SEMAPHORE_4 :: reserved0 [31:08] */
#define BCHP_ARMCR4_BRIDGE_REG_SEMAPHORE_4_reserved0_MASK          0xffffff00
#define BCHP_ARMCR4_BRIDGE_REG_SEMAPHORE_4_reserved0_SHIFT         8

/* ARMCR4_BRIDGE :: REG_SEMAPHORE_4 :: semaphore [07:00] */
#define BCHP_ARMCR4_BRIDGE_REG_SEMAPHORE_4_semaphore_MASK          0x000000ff
#define BCHP_ARMCR4_BRIDGE_REG_SEMAPHORE_4_semaphore_SHIFT         0

/***************************************************************************
 *REG_SCRATCH_1 - CPU scratchpad #1
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_SCRATCH_1 :: data [31:00] */
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_1_data_MASK                 0xffffffff
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_1_data_SHIFT                0

/***************************************************************************
 *REG_SCRATCH_2 - CPU scratchpad #2
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_SCRATCH_2 :: data [31:00] */
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_2_data_MASK                 0xffffffff
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_2_data_SHIFT                0

/***************************************************************************
 *REG_SCRATCH_3 - CPU scratchpad #3
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_SCRATCH_3 :: data [31:00] */
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_3_data_MASK                 0xffffffff
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_3_data_SHIFT                0

/***************************************************************************
 *REG_SCRATCH_4 - CPU scratchpad #4
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_SCRATCH_4 :: data [31:00] */
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_4_data_MASK                 0xffffffff
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_4_data_SHIFT                0

/***************************************************************************
 *REG_SCRATCH_5 - CPU scratchpad #5
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_SCRATCH_5 :: data [31:00] */
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_5_data_MASK                 0xffffffff
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_5_data_SHIFT                0

/***************************************************************************
 *REG_SCRATCH_6 - CPU scratchpad #6
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_SCRATCH_6 :: data [31:00] */
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_6_data_MASK                 0xffffffff
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_6_data_SHIFT                0

/***************************************************************************
 *REG_SCRATCH_7 - CPU scratchpad #7
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_SCRATCH_7 :: data [31:00] */
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_7_data_MASK                 0xffffffff
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_7_data_SHIFT                0

/***************************************************************************
 *REG_SCRATCH_8 - CPU scratchpad #8
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_SCRATCH_8 :: data [31:00] */
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_8_data_MASK                 0xffffffff
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_8_data_SHIFT                0

/***************************************************************************
 *REG_SCRATCH_9 - CPU scratchpad #9
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_SCRATCH_9 :: data [31:00] */
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_9_data_MASK                 0xffffffff
#define BCHP_ARMCR4_BRIDGE_REG_SCRATCH_9_data_SHIFT                0

/***************************************************************************
 *REG_PERF_CONFIG - Performance monitor configuration
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_PERF_CONFIG :: reserved0 [31:01] */
#define BCHP_ARMCR4_BRIDGE_REG_PERF_CONFIG_reserved0_MASK          0xfffffffe
#define BCHP_ARMCR4_BRIDGE_REG_PERF_CONFIG_reserved0_SHIFT         1

/* ARMCR4_BRIDGE :: REG_PERF_CONFIG :: enable [00:00] */
#define BCHP_ARMCR4_BRIDGE_REG_PERF_CONFIG_enable_MASK             0x00000001
#define BCHP_ARMCR4_BRIDGE_REG_PERF_CONFIG_enable_SHIFT            0

/***************************************************************************
 *REG_PERF_LIMIT - Performance monitor count threshold
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_PERF_LIMIT :: count [31:00] */
#define BCHP_ARMCR4_BRIDGE_REG_PERF_LIMIT_count_MASK               0xffffffff
#define BCHP_ARMCR4_BRIDGE_REG_PERF_LIMIT_count_SHIFT              0

/***************************************************************************
 *REG_PERF_WR_CNT - Counts the number of merge buffer updates (hits + misses)
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_PERF_WR_CNT :: count [31:00] */
#define BCHP_ARMCR4_BRIDGE_REG_PERF_WR_CNT_count_MASK              0xffffffff
#define BCHP_ARMCR4_BRIDGE_REG_PERF_WR_CNT_count_SHIFT             0

/***************************************************************************
 *REG_PERF_WR_MISS - Counts the number of merge buffer misses
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_PERF_WR_MISS :: count [31:00] */
#define BCHP_ARMCR4_BRIDGE_REG_PERF_WR_MISS_count_MASK             0xffffffff
#define BCHP_ARMCR4_BRIDGE_REG_PERF_WR_MISS_count_SHIFT            0

/***************************************************************************
 *REG_PERF_RD_CNT - Counts the number of prefetch buffer accesses (hits + misses)
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_PERF_RD_CNT :: count [31:00] */
#define BCHP_ARMCR4_BRIDGE_REG_PERF_RD_CNT_count_MASK              0xffffffff
#define BCHP_ARMCR4_BRIDGE_REG_PERF_RD_CNT_count_SHIFT             0

/***************************************************************************
 *REG_PERF_RD_MISS - Counts the number of prefetch buffer misses
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_PERF_RD_MISS :: count [31:00] */
#define BCHP_ARMCR4_BRIDGE_REG_PERF_RD_MISS_count_MASK             0xffffffff
#define BCHP_ARMCR4_BRIDGE_REG_PERF_RD_MISS_count_SHIFT            0

/***************************************************************************
 *REG_MEMORY_TM1 - ARM memory TM1 control register
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_MEMORY_TM1 :: tm_dc7 [31:28] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM1_tm_dc7_MASK              0xf0000000
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM1_tm_dc7_SHIFT             28

/* ARMCR4_BRIDGE :: REG_MEMORY_TM1 :: tm_dc6 [27:24] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM1_tm_dc6_MASK              0x0f000000
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM1_tm_dc6_SHIFT             24

/* ARMCR4_BRIDGE :: REG_MEMORY_TM1 :: tm_dc5 [23:20] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM1_tm_dc5_MASK              0x00f00000
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM1_tm_dc5_SHIFT             20

/* ARMCR4_BRIDGE :: REG_MEMORY_TM1 :: tm_dc4 [19:16] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM1_tm_dc4_MASK              0x000f0000
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM1_tm_dc4_SHIFT             16

/* ARMCR4_BRIDGE :: REG_MEMORY_TM1 :: tm_dc3 [15:12] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM1_tm_dc3_MASK              0x0000f000
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM1_tm_dc3_SHIFT             12

/* ARMCR4_BRIDGE :: REG_MEMORY_TM1 :: tm_dc2 [11:08] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM1_tm_dc2_MASK              0x00000f00
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM1_tm_dc2_SHIFT             8

/* ARMCR4_BRIDGE :: REG_MEMORY_TM1 :: tm_dc1 [07:04] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM1_tm_dc1_MASK              0x000000f0
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM1_tm_dc1_SHIFT             4

/* ARMCR4_BRIDGE :: REG_MEMORY_TM1 :: tm_dc0 [03:00] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM1_tm_dc0_MASK              0x0000000f
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM1_tm_dc0_SHIFT             0

/***************************************************************************
 *REG_MEMORY_TM2 - ARM memory TM2 control register
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_MEMORY_TM2 :: reserved0 [31:18] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM2_reserved0_MASK           0xfffc0000
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM2_reserved0_SHIFT          18

/* ARMCR4_BRIDGE :: REG_MEMORY_TM2 :: tm_dirty [17:16] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM2_tm_dirty_MASK            0x00030000
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM2_tm_dirty_SHIFT           16

/* ARMCR4_BRIDGE :: REG_MEMORY_TM2 :: tm_ic6 [15:12] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM2_tm_ic6_MASK              0x0000f000
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM2_tm_ic6_SHIFT             12

/* ARMCR4_BRIDGE :: REG_MEMORY_TM2 :: tm_ic4 [11:08] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM2_tm_ic4_MASK              0x00000f00
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM2_tm_ic4_SHIFT             8

/* ARMCR4_BRIDGE :: REG_MEMORY_TM2 :: tm_ic2 [07:04] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM2_tm_ic2_MASK              0x000000f0
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM2_tm_ic2_SHIFT             4

/* ARMCR4_BRIDGE :: REG_MEMORY_TM2 :: tm_ic0 [03:00] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM2_tm_ic0_MASK              0x0000000f
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM2_tm_ic0_SHIFT             0

/***************************************************************************
 *REG_MEMORY_TM3 - ARM memory TM3 control register
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_MEMORY_TM3 :: reserved0 [31:16] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM3_reserved0_MASK           0xffff0000
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM3_reserved0_SHIFT          16

/* ARMCR4_BRIDGE :: REG_MEMORY_TM3 :: tm_it3 [15:14] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM3_tm_it3_MASK              0x0000c000
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM3_tm_it3_SHIFT             14

/* ARMCR4_BRIDGE :: REG_MEMORY_TM3 :: tm_it2 [13:12] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM3_tm_it2_MASK              0x00003000
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM3_tm_it2_SHIFT             12

/* ARMCR4_BRIDGE :: REG_MEMORY_TM3 :: tm_it1 [11:10] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM3_tm_it1_MASK              0x00000c00
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM3_tm_it1_SHIFT             10

/* ARMCR4_BRIDGE :: REG_MEMORY_TM3 :: tm_it0 [09:08] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM3_tm_it0_MASK              0x00000300
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM3_tm_it0_SHIFT             8

/* ARMCR4_BRIDGE :: REG_MEMORY_TM3 :: tm_dt3 [07:06] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM3_tm_dt3_MASK              0x000000c0
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM3_tm_dt3_SHIFT             6

/* ARMCR4_BRIDGE :: REG_MEMORY_TM3 :: tm_dt2 [05:04] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM3_tm_dt2_MASK              0x00000030
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM3_tm_dt2_SHIFT             4

/* ARMCR4_BRIDGE :: REG_MEMORY_TM3 :: tm_dt1 [03:02] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM3_tm_dt1_MASK              0x0000000c
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM3_tm_dt1_SHIFT             2

/* ARMCR4_BRIDGE :: REG_MEMORY_TM3 :: tm_dt0 [01:00] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM3_tm_dt0_MASK              0x00000003
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM3_tm_dt0_SHIFT             0

/***************************************************************************
 *REG_FIFO_STATUS - Fifo Status
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_FIFO_STATUS :: reserved0 [31:02] */
#define BCHP_ARMCR4_BRIDGE_REG_FIFO_STATUS_reserved0_MASK          0xfffffffc
#define BCHP_ARMCR4_BRIDGE_REG_FIFO_STATUS_reserved0_SHIFT         2

/* ARMCR4_BRIDGE :: REG_FIFO_STATUS :: CCB_RD_FULL [01:01] */
#define BCHP_ARMCR4_BRIDGE_REG_FIFO_STATUS_CCB_RD_FULL_MASK        0x00000002
#define BCHP_ARMCR4_BRIDGE_REG_FIFO_STATUS_CCB_RD_FULL_SHIFT       1

/* ARMCR4_BRIDGE :: REG_FIFO_STATUS :: CCB_WR_FULL [00:00] */
#define BCHP_ARMCR4_BRIDGE_REG_FIFO_STATUS_CCB_WR_FULL_MASK        0x00000001
#define BCHP_ARMCR4_BRIDGE_REG_FIFO_STATUS_CCB_WR_FULL_SHIFT       0

/***************************************************************************
 *REG_BORCH_STATUS - Bridge Out-of-range Checker Status
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_BORCH_STATUS :: reserved0 [31:01] */
#define BCHP_ARMCR4_BRIDGE_REG_BORCH_STATUS_reserved0_MASK         0xfffffffe
#define BCHP_ARMCR4_BRIDGE_REG_BORCH_STATUS_reserved0_SHIFT        1

/* ARMCR4_BRIDGE :: REG_BORCH_STATUS :: BORCH_ERROR_STATUS [00:00] */
#define BCHP_ARMCR4_BRIDGE_REG_BORCH_STATUS_BORCH_ERROR_STATUS_MASK 0x00000001
#define BCHP_ARMCR4_BRIDGE_REG_BORCH_STATUS_BORCH_ERROR_STATUS_SHIFT 0

/***************************************************************************
 *REG_MEMORY_TM4 - ARM memory TM4 control register
 ***************************************************************************/
/* ARMCR4_BRIDGE :: REG_MEMORY_TM4 :: reserved0 [31:02] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM4_reserved0_MASK           0xfffffffc
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM4_reserved0_SHIFT          2

/* ARMCR4_BRIDGE :: REG_MEMORY_TM4 :: tm_pref [01:00] */
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM4_tm_pref_MASK             0x00000003
#define BCHP_ARMCR4_BRIDGE_REG_MEMORY_TM4_tm_pref_SHIFT            0

#endif /* #ifndef BCHP_ARMCR4_BRIDGE_H__ */

/* End of File */