/* SPDX-License-Identifier: BSD-2-Clause-Patent */
/* CS35L41 OTP trim unpack, map 1 (OTPID 0x01 / 0x08: the pdx203 amplifiers read 0x08), from Linux cs35l41-lib.c otp_map_1 (Sony's
 * cs35l41-tables.c is identical). The fields follow each other LSB first in OTP_MEM0..31 from bit 80; each is written into its register
 * field with a read-modify-write while the test key is unlocked. Generated from /tmp/kmdconn/cs35l41-init-spec.md. */
#ifndef PDX203_CS35L41_OTP_H
#define PDX203_CS35L41_OTP_H
#include <stdint.h>
typedef struct {uint32_t reg; uint8_t shift, size;} Cs35l41OtpField;
#define CS35L41_OTP_START_BIT 80u
static const Cs35l41OtpField kCs35l41OtpMap1[99] = {
    {0x00002030, 0, 4},   /*  0 bit 80: TRIM_OSC_FREQ_TRIM */
    {0x00002030, 7, 1},   /*  1 bit 84: TRIM_OSC_TRIM_DONE */
    {0x0000208C, 24, 6},   /*  2 bit 85: TST_DIGREG_VREF_TRIM */
    {0x00002090, 14, 4},   /*  3 bit 91: TST_REF_TRIM */
    {0x00002090, 10, 4},   /*  4 bit 95: TST_REF_TEMPCO_TRIM */
    {0x0000300C, 11, 4},   /*  5 bit 99: PLL_LDOA_TST_VREF_TRIM */
    {0x0000394C, 23, 2},   /*  6 bit 103: BST_ATEST_CM_VOFF */
    {0x00003950, 0, 7},   /*  7 bit 105: BST_ATRIM_IADC_OFFSET */
    {0x00003950, 8, 7},   /*  8 bit 112: BST_ATRIM_IADC_GAIN1 */
    {0x00003950, 16, 8},   /*  9 bit 119: BST_ATRIM_IPKCOMP_OFFSET1 */
    {0x00003950, 24, 8},   /* 10 bit 127: BST_ATRIM_IPKCOMP_GAIN1 */
    {0x00003954, 0, 7},   /* 11 bit 135: BST_ATRIM_IADC_OFFSET2 */
    {0x00003954, 8, 7},   /* 12 bit 142: BST_ATRIM_IADC_GAIN2 */
    {0x00003954, 16, 8},   /* 13 bit 149: BST_ATRIM_IPKCOMP_OFFSET2 */
    {0x00003954, 24, 8},   /* 14 bit 157: BST_ATRIM_IPKCOMP_GAIN2 */
    {0x00003958, 0, 7},   /* 15 bit 165: BST_ATRIM_IADC_OFFSET3 */
    {0x00003958, 8, 7},   /* 16 bit 172: BST_ATRIM_IADC_GAIN3 */
    {0x00003958, 16, 8},   /* 17 bit 179: BST_ATRIM_IPKCOMP_OFFSET3 */
    {0x00003958, 24, 8},   /* 18 bit 187: BST_ATRIM_IPKCOMP_GAIN3 */
    {0x0000395C, 0, 7},   /* 19 bit 195: BST_ATRIM_IADC_OFFSET4 */
    {0x0000395C, 8, 7},   /* 20 bit 202: BST_ATRIM_IADC_GAIN4 */
    {0x0000395C, 16, 8},   /* 21 bit 209: BST_ATRIM_IPKCOMP_OFFSET4 */
    {0x0000395C, 24, 8},   /* 22 bit 217: BST_ATRIM_IPKCOMP_GAIN4 */
    {0x0000416C, 0, 8},   /* 23 bit 225: VMON_GAIN_OTP_VAL */
    {0x00004160, 0, 7},   /* 24 bit 233: VMON_OFFSET_OTP_VAL */
    {0x0000416C, 8, 8},   /* 25 bit 240: IMON_GAIN_OTP_VAL */
    {0x00004160, 16, 10},   /* 26 bit 248: IMON_OFFSET_OTP_VAL */
    {0x0000416C, 16, 12},   /* 27 bit 258: VMON_CM_GAIN_OTP_VAL */
    {0x0000416C, 28, 1},   /* 28 bit 270: VMON_CM_GAIN_SIGN_OTP_VAL */
    {0x00004170, 0, 6},   /* 29 bit 271: IMON_CAL_TEMPCO_OTP_VAL */
    {0x00004170, 6, 1},   /* 30 bit 277: IMON_CAL_TEMPCO_SIGN_OTP */
    {0x00004170, 8, 6},   /* 31 bit 278: IMON_CAL_TEMPCO2_OTP_VAL */
    {0x00004170, 14, 1},   /* 32 bit 284: IMON_CAL_TEMPCO2_DN_UPB_OTP_VAL */
    {0x00004170, 16, 9},   /* 33 bit 285: IMON_CAL_TEMPCO_TBASE_OTP_VAL */
    {0x00004360, 0, 5},   /* 34 bit 294: TEMP_GAIN_OTP_VAL */
    {0x00004360, 6, 9},   /* 35 bit 299: TEMP_OFFSET_OTP_VAL */
    {0x00004448, 0, 8},   /* 36 bit 308: VP_SARADC_OFFSET */
    {0x00004448, 8, 8},   /* 37 bit 316: VP_GAIN_INDEX */
    {0x00004448, 16, 8},   /* 38 bit 324: VBST_SARADC_OFFSET */
    {0x00004448, 24, 8},   /* 39 bit 332: VBST_GAIN_INDEX */
    {0x0000444C, 0, 3},   /* 40 bit 340: ANA_SELINVREF */
    {0x00006E30, 0, 5},   /* 41 bit 343: GAIN_ERR_COEFF_0 */
    {0x00006E30, 8, 5},   /* 42 bit 348: GAIN_ERR_COEFF_1 */
    {0x00006E30, 16, 5},   /* 43 bit 353: GAIN_ERR_COEFF_2 */
    {0x00006E30, 24, 5},   /* 44 bit 358: GAIN_ERR_COEFF_3 */
    {0x00006E34, 0, 5},   /* 45 bit 363: GAIN_ERR_COEFF_4 */
    {0x00006E34, 8, 5},   /* 46 bit 368: GAIN_ERR_COEFF_5 */
    {0x00006E34, 16, 5},   /* 47 bit 373: GAIN_ERR_COEFF_6 */
    {0x00006E34, 24, 5},   /* 48 bit 378: GAIN_ERR_COEFF_7 */
    {0x00006E38, 0, 5},   /* 49 bit 383: GAIN_ERR_COEFF_8 */
    {0x00006E38, 8, 5},   /* 50 bit 388: GAIN_ERR_COEFF_9 */
    {0x00006E38, 16, 5},   /* 51 bit 393: GAIN_ERR_COEFF_10 */
    {0x00006E38, 24, 5},   /* 52 bit 398: GAIN_ERR_COEFF_11 */
    {0x00006E3C, 0, 5},   /* 53 bit 403: GAIN_ERR_COEFF_12 */
    {0x00006E3C, 8, 5},   /* 54 bit 408: GAIN_ERR_COEFF_13 */
    {0x00006E3C, 16, 5},   /* 55 bit 413: GAIN_ERR_COEFF_14 */
    {0x00006E3C, 24, 5},   /* 56 bit 418: GAIN_ERR_COEFF_15 */
    {0x00006E40, 0, 5},   /* 57 bit 423: GAIN_ERR_COEFF_16 */
    {0x00006E40, 8, 5},   /* 58 bit 428: GAIN_ERR_COEFF_17 */
    {0x00006E40, 16, 5},   /* 59 bit 433: GAIN_ERR_COEFF_18 */
    {0x00006E40, 24, 5},   /* 60 bit 438: GAIN_ERR_COEFF_19 */
    {0x00006E44, 0, 5},   /* 61 bit 443: GAIN_ERR_COEFF_20 */
    {0x00006E48, 0, 10},   /* 62 bit 448: VOFF_GAIN_0 */
    {0x00006E48, 10, 10},   /* 63 bit 458: VOFF_GAIN_1 */
    {0x00006E48, 20, 10},   /* 64 bit 468: VOFF_GAIN_2 */
    {0x00006E4C, 0, 10},   /* 65 bit 478: VOFF_GAIN_3 */
    {0x00006E4C, 10, 10},   /* 66 bit 488: VOFF_GAIN_4 */
    {0x00006E4C, 20, 10},   /* 67 bit 498: VOFF_GAIN_5 */
    {0x00006E50, 0, 10},   /* 68 bit 508: VOFF_GAIN_6 */
    {0x00006E50, 10, 10},   /* 69 bit 518: VOFF_GAIN_7 */
    {0x00006E50, 20, 10},   /* 70 bit 528: VOFF_GAIN_8 */
    {0x00006E54, 0, 10},   /* 71 bit 538: VOFF_GAIN_9 */
    {0x00006E54, 10, 10},   /* 72 bit 548: VOFF_GAIN_10 */
    {0x00006E54, 20, 10},   /* 73 bit 558: VOFF_GAIN_11 */
    {0x00006E58, 0, 10},   /* 74 bit 568: VOFF_GAIN_12 */
    {0x00006E58, 10, 10},   /* 75 bit 578: VOFF_GAIN_13 */
    {0x00006E58, 20, 10},   /* 76 bit 588: VOFF_GAIN_14 */
    {0x00006E5C, 0, 10},   /* 77 bit 598: VOFF_GAIN_15 */
    {0x00006E5C, 10, 10},   /* 78 bit 608: VOFF_GAIN_16 */
    {0x00006E5C, 20, 10},   /* 79 bit 618: VOFF_GAIN_17 */
    {0x00006E60, 0, 10},   /* 80 bit 628: VOFF_GAIN_18 */
    {0x00006E60, 10, 10},   /* 81 bit 638: VOFF_GAIN_19 */
    {0x00006E60, 20, 10},   /* 82 bit 648: VOFF_GAIN_20 */
    {0x00006E64, 0, 10},   /* 83 bit 658: VOFF_INT1 */
    {0x00007418, 7, 5},   /* 84 bit 668: DS_SPK_INT1_CAP_TRIM */
    {0x0000741C, 0, 5},   /* 85 bit 673: DS_SPK_INT2_CAP_TRIM */
    {0x0000741C, 11, 4},   /* 86 bit 678: DS_SPK_LPF_CAP_TRIM */
    {0x0000741C, 19, 4},   /* 87 bit 682: DS_SPK_QUAN_CAP_TRIM */
    {0x00007434, 17, 1},   /* 88 bit 686: FORCE_CAL */
    {0x00007434, 18, 7},   /* 89 bit 687: CAL_OVERRIDE */
    {0x00007068, 0, 9},   /* 90 bit 694: MODIX */
    {0x0000410C, 7, 1},   /* 91 bit 703: VIMON_DLY_NOT_COMB */
    {0x0000400C, 0, 7},   /* 92 bit 704: VIMON_DLY */
    {0x00000000, 0, 1},   /* 93 bit 711: map 1: consumed, not written (map 2: VMON_POL 0x4000 bit 11) */
    {0x00017040, 0, 8},   /* 94 bit 712: X_COORDINATE */
    {0x00017040, 8, 8},   /* 95 bit 720: Y_COORDINATE */
    {0x00017040, 16, 8},   /* 96 bit 728: WAFER_ID */
    {0x00017040, 24, 8},   /* 97 bit 736: DVS */
    {0x00017044, 0, 24},   /* 98 bit 744: LOT_NUMBER */
};
#endif
