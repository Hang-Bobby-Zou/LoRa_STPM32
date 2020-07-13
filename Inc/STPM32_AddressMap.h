// This the the address map for all registers
#include <stdlib.h>
#include <stdio.h>



#define dsp_cr1 	0x00
#define dsp_cr2		0x02
#define dsp_cr3		0x04
#define dsp_cr4		0x06
#define dsp_cr5 	0x08
#define dsp_cr6		0x0A
#define dsp_cr7		0x0C
#define dsp_cr8		0x0E
#define dsp_cr9		0x10
#define dsp_cr10	0x12
#define dsp_cr11 	0x14
#define dsp_cr12	0x16

#define dfe_cr1		0x18
#define dfe_cr2		0x1A
#define dsp_irq1	0x1C
#define dsp_irq2	0x1E
#define dsp_sr1		0x20
#define dsp_sr2		0x22
#define us_reg1		0x24
#define us_reg2		0x26
#define us_reg3		0x28

#define dsp_ev1		0x2A
#define dsp_ev2		0x2C
#define dsp_reg1	0x2E
#define dsp_reg2	0x30
#define dsp_reg3	0x32
#define dsp_reg4	0x34
#define dsp_reg5	0x36
#define dsp_reg6	0x38
#define dsp_reg7	0x3A
#define dsp_reg8	0x3C
#define dsp_reg9	0x3E

#define dsp_reg14	0x48
#define dsp_reg15	0x4A
#define dsp_reg16	0x4C
#define dsp_reg17	0x4E
#define dsp_reg18	0x50
#define dsp_reg19	0x52

#define ph1_reg1	0x54
#define ph1_reg2	0x56
#define ph1_reg3	0x58
#define ph1_reg4	0x5A
#define ph1_reg5	0x5C
#define ph1_reg6	0x5E
#define ph1_reg7	0x60
#define ph1_reg8	0x62
#define ph1_reg9	0x64
#define ph1_reg10	0x66
#define ph1_reg11	0x68
#define ph1_reg12	0x6A

#define ph2_reg1	0x6C
#define ph2_reg2	0x6E
#define ph2_reg3	0x70
#define ph2_reg4	0x72
#define ph2_reg5	0x74
#define ph2_reg6	0x76
#define ph2_reg7	0x78
#define ph2_reg8	0x7A
#define ph2_reg9	0x7C
#define ph2_reg10	0x7E
#define ph2_reg11	0x80
#define ph2_reg12	0x82

#define tot_reg1	0x84
#define tot_reg2	0x86
#define tot_reg3	0x88
#define tot_reg4	0x8A


#define FlashAddr_Freq							0x000000
#define FlashAddr_RMS								0x010000
#define FlashAddr_Phase 						0x020000
#define FlashAddr_Active_Energy			0x030000
#define FlashAddr_Funda_Energy			0x040000
#define FlashAddr_React_Energy			0x050000
#define FlashAddr_App_Energy				0x060000
#define FlashAddr_Active_Power			0x070000
#define FlashAddr_Funda_Pwr					0x080000
#define FlashAddr_React_Pwr					0x090000
#define FlashAddr_App_RMS_Pwr				0x0A0000
#define FlashAddr_Tot_Active_Energy	0x0B0000
#define FlashAddr_Tot_Funda_Energy	0x0C0000
#define FlashAddr_Tot_React_Energy	0x0D0000
#define FlashAddr_Tot_App_Energy		0x0E0000










