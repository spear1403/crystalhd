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
 * $brcm_Workfile: bchp_trb_top.h $
 * $brcm_Revision: Hydra_Software_Devel/1 $
 * $brcm_Date: 7/17/09 8:21p $
 *
 * Module Description:
 *                     DO NOT EDIT THIS FILE DIRECTLY
 *
 * This module was generated magically with RDB from a source description
 * file. You must edit the source file for changes to be made to this file.
 *
 *
 * Date:           Generated on         Fri Jul 17 19:42:24 2009
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
 * $brcm_Log: /magnum/basemodules/chp/70015/rdb/a0/bchp_trb_top.h $
 * 
 * Hydra_Software_Devel/1   7/17/09 8:21p albertl
 * PR56880: Initial revision.
 *
 ***************************************************************************/

#ifndef BCHP_TRB_TOP_H__
#define BCHP_TRB_TOP_H__

/***************************************************************************
 *TRB_TOP - TRB Control Registers
 ***************************************************************************/
#define BCHP_TRB_TOP_CTL                         0x000f0000 /* TRB Control */
#define BCHP_TRB_TOP_STATUS                      0x000f0004 /* TRB Status */
#define BCHP_TRB_TOP_REVISION                    0x000f0008 /* TRB REVISION */

/***************************************************************************
 *CTL - TRB Control
 ***************************************************************************/
/* TRB_TOP :: CTL :: reserved0 [31:02] */
#define BCHP_TRB_TOP_CTL_reserved0_MASK                            0xfffffffc
#define BCHP_TRB_TOP_CTL_reserved0_SHIFT                           2

/* TRB_TOP :: CTL :: TRBOver [01:01] */
#define BCHP_TRB_TOP_CTL_TRBOver_MASK                              0x00000002
#define BCHP_TRB_TOP_CTL_TRBOver_SHIFT                             1

/* TRB_TOP :: CTL :: TRBEna [00:00] */
#define BCHP_TRB_TOP_CTL_TRBEna_MASK                               0x00000001
#define BCHP_TRB_TOP_CTL_TRBEna_SHIFT                              0

/***************************************************************************
 *STATUS - TRB Status
 ***************************************************************************/
/* TRB_TOP :: STATUS :: reserved0 [31:01] */
#define BCHP_TRB_TOP_STATUS_reserved0_MASK                         0xfffffffe
#define BCHP_TRB_TOP_STATUS_reserved0_SHIFT                        1

/* TRB_TOP :: STATUS :: ErrSeen [00:00] */
#define BCHP_TRB_TOP_STATUS_ErrSeen_MASK                           0x00000001
#define BCHP_TRB_TOP_STATUS_ErrSeen_SHIFT                          0

/***************************************************************************
 *REVISION - TRB REVISION
 ***************************************************************************/
/* TRB_TOP :: REVISION :: reserved0 [31:16] */
#define BCHP_TRB_TOP_REVISION_reserved0_MASK                       0xffff0000
#define BCHP_TRB_TOP_REVISION_reserved0_SHIFT                      16

/* TRB_TOP :: REVISION :: MAJOR [15:08] */
#define BCHP_TRB_TOP_REVISION_MAJOR_MASK                           0x0000ff00
#define BCHP_TRB_TOP_REVISION_MAJOR_SHIFT                          8

/* TRB_TOP :: REVISION :: MINOR [07:00] */
#define BCHP_TRB_TOP_REVISION_MINOR_MASK                           0x000000ff
#define BCHP_TRB_TOP_REVISION_MINOR_SHIFT                          0

#endif /* #ifndef BCHP_TRB_TOP_H__ */

/* End of File */