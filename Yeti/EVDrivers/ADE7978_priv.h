//
// ADE7978_priv.h
//
//  Created on: Apr 19, 2021
//      Author: aw@pionier-manufaktur.de
//

#ifndef SRC_EVDRIVERS_ADE7978_PRIV_H_
#define SRC_EVDRIVERS_ADE7978_PRIV_H_

#include <cstdint>

namespace ade7978 {

// Registers located in DSP Data Memory RAM
const uint16_t AIGAIN = 0x4380;
const uint16_t AVGAIN = 0x4381;
const uint16_t AV2GAIN = 0x4382;
const uint16_t BIGAIN = 0x4383;
const uint16_t BVGAIN = 0x4384;
const uint16_t BV2GAIN = 0x4385;
const uint16_t CIGAIN = 0x4386;
const uint16_t CVGAIN = 0x4387;
const uint16_t CV2GAIN = 0x4388;
const uint16_t NIGAIN = 0x4389;
const uint16_t NVGAIN = 0x438A;
const uint16_t NV2GAIN = 0x438B;
const uint16_t AIRMSOS = 0x438C;
const uint16_t AVRMSOS = 0x438D;
const uint16_t AV2RMSOS = 0x438E;
const uint16_t BIRMSOS = 0x438F;
const uint16_t BVRMSOS = 0x4390;
const uint16_t BV2RMSOS = 0x4391;
const uint16_t CIRMSOS = 0x4392;
const uint16_t CVRMSOS = 0x4393;
const uint16_t CV2RMSOS = 0x4394;
const uint16_t NIRMSOS = 0x4395;
const uint16_t NVRMSOS = 0x4396;
const uint16_t NV2RMSOS = 0x4397;
const uint16_t ISUMLVL = 0x4398;
const uint16_t APGAIN = 0x4399;
const uint16_t BPGAIN = 0x439A;
const uint16_t CPGAIN = 0x439B;
const uint16_t AWATTOS = 0x439C;
const uint16_t BWATTOS = 0x439D;
const uint16_t CWATTOS = 0x439E;
const uint16_t AVAROS = 0x439F;
const uint16_t BVAROS = 0x43A0;
const uint16_t CVAROS = 0x43A1;
const uint16_t VLEVEL = 0x43A2;
const uint16_t AFWATTOS = 0x43A3;
const uint16_t BFWATTOS = 0x43A4;
const uint16_t CFWATTOS = 0x43A5;
const uint16_t AFVAROS = 0x43A6;
const uint16_t BFVAROS = 0x43A7;
const uint16_t CFVAROS = 0x43A8;
const uint16_t AFIRMSOS = 0x43A9;
const uint16_t BFIRMSOS = 0x43AA;
const uint16_t CFIRMSOS = 0x43AB;
const uint16_t AFVRMSOS = 0x43AC;
const uint16_t BFVRMSOS = 0x43AD;
const uint16_t CFVRMSOS = 0x43AE;
const uint16_t TEMPCO = 0x43AF;
const uint16_t ATEMP0 = 0x43B0;
const uint16_t BTEMP0 = 0x43B1;
const uint16_t CTEMP0 = 0x43B2;
const uint16_t NTEMP0 = 0x43B3;
const uint16_t ATGAIN = 0x43B4;
const uint16_t BTGAIN = 0x43B5;
const uint16_t CTGAIN = 0x43B6;
const uint16_t NTGAIN = 0x43B7;

// 0x43B8 to 0x43BF is reserved

// RMS value registers
const uint16_t AIRMS = 0x43C0;
const uint16_t AVRMS = 0x43C1;
const uint16_t AV2RMS = 0x43C2;
const uint16_t BIRMS = 0x43C3;
const uint16_t BVRMS = 0x43C4;
const uint16_t BV2RMS = 0x43C5;
const uint16_t CIRMS = 0x43C6;
const uint16_t CVRMS = 0x43C7;
const uint16_t CV2RMS = 0x43C8;
const uint16_t NIRMS = 0x43C9;
const uint16_t ISUM = 0x43CA;
const uint16_t ATEMP = 0x43CB;
const uint16_t BTEMP = 0x43CD;
const uint16_t CTEMP = 0x43CE;
const uint8_t XRMS_VAL_Pos = 0;
const uint32_t XRMS_VAL_Msk = (0xFFFFFF << XRMS_VAL_Pos);

// DSP run/stop register
const uint16_t RUN = 0xE228;
const uint16_t RUN_VAL_START = 0x1;
const uint16_t RUN_VAL_STOP = 0x0;

// Billable registers
const uint16_t AWATTHR = 0xE400;
const uint16_t BWATTHR = 0xE401;
const uint16_t CWATTHR = 0xE402;
const uint16_t AFWATTHR = 0xE403;
const uint16_t BFWATTHR = 0xE404;
const uint16_t CFWATTHR = 0xE405;
const uint16_t AVARHR = 0xE406;
const uint16_t BVARHR = 0xE407;
const uint16_t CVARHR = 0xE408;
const uint16_t AFVARHR = 0xE409;
const uint16_t BFVARHR = 0xE40A;
const uint16_t CFVARHR = 0xE40B;
const uint16_t AVAHR = 0xE40C;
const uint16_t BVAHR = 0xE40D;
const uint16_t CVAHR = 0xE40E;

// Peak level registers
const uint16_t IPEAK = 0xE500;
const uint16_t VPEAK = 0xE501;
const uint8_t XPEAK_VAL_Pos = 0;
const uint32_t XPEAK_VAL_Msk = (0xFFFFFF << XPEAK_VAL_Pos);
const uint8_t XPEAK_PHASE_Pos = 24;
const uint32_t XPEAK_PHASE_Msk = (0x7 << XPEAK_PHASE_Pos);
const uint32_t XPEAK_PHASE_A = (0x1 << (XPEAK_PHASE_Pos + 0));
const uint32_t XPEAK_PHASE_B = (0x1 << (XPEAK_PHASE_Pos + 1));
const uint32_t XPEAK_PHASE_C = (0x1 << (XPEAK_PHASE_Pos + 2));

// STATUS0 Register
const uint16_t STATUS0 = 0xE502;

// STATUS1 Register
const uint16_t STATUS1 = 0xE503;
const uint8_t STATUS1_RSTDONE_Pos = 15;
const uint32_t STATUS1_RSTDONE_Msk = (0x1 << STATUS1_RSTDONE_Pos);
// Zero crossing timeout
const uint8_t STATUS1_ZXTOVA_Pos = 3;
const uint32_t STATUS1_ZXTOVA_Msk = (1 << STATUS1_ZXTOVA_Pos);
const uint8_t STATUS1_ZXTOVB_Pos = 4;
const uint32_t STATUS1_ZXTOVB_Msk = (1 << STATUS1_ZXTOVB_Pos);
const uint8_t STATUS1_ZXTOVC_Pos = 5;
const uint32_t STATUS1_ZXTOVC_Msk = (1 << STATUS1_ZXTOVC_Pos);
const uint8_t STATUS1_ZXTOIA_Pos = 6;
const uint32_t STATUS1_ZXTOIA_Msk = (1 << STATUS1_ZXTOIA_Pos);
const uint8_t STATUS1_ZXTOIB_Pos = 7;
const uint32_t STATUS1_ZXTOIB_Msk = (1 << STATUS1_ZXTOIB_Pos);
const uint8_t STATUS1_ZXTOIC_Pos = 8;
const uint32_t STATUS1_ZXTOIC_Msk = (1 << STATUS1_ZXTOIC_Pos);
// Zero crossing detection
const uint8_t STATUS1_ZXVA_Pos = 9;
const uint32_t STATUS1_ZXVA_Msk = (1 << STATUS1_ZXVA_Pos);
const uint8_t STATUS1_ZXVB_Pos = 10;
const uint32_t STATUS1_ZXVB_Msk = (1 << STATUS1_ZXVB_Pos);
const uint8_t STATUS1_ZXVC_Pos = 11;
const uint32_t STATUS1_ZXVC_Msk = (1 << STATUS1_ZXVC_Pos);
const uint8_t STATUS1_ZXIA_Pos = 12;
const uint32_t STATUS1_ZXIA_Msk = (1 << STATUS1_ZXIA_Pos);
const uint8_t STATUS1_ZXIB_Pos = 13;
const uint32_t STATUS1_ZXIB_Msk = (1 << STATUS1_ZXIB_Pos);
const uint8_t STATUS1_ZXIC_Pos = 14;
const uint32_t STATUS1_ZXIC_Msk = (1 << STATUS1_ZXIC_Pos);
const uint8_t STATUS1_OI_Pos = 17;
const uint32_t STATUS1_OI_Msk = (1 << STATUS1_OI_Pos);
// Phase sequence detection, error if A -> C -> B
const uint8_t STATUS1_SEQERR_Pos = 19;
const uint32_t STATUS1_SEQERR_Msk = (1 << STATUS1_SEQERR_Pos);

// registers 0xE504 - 0xE506 are reserved

const uint16_t OILVL = 0xE507;
const uint16_t OVLVL = 0xE508;
const uint16_t SAGLVL = 0xE509;
const uint16_t MASK0 = 0xE50A;
const uint16_t MASK1 = 0xE50B;
const uint16_t IAWV = 0xE50C;
const uint16_t IBWV = 0xE50D;
const uint16_t ICWV = 0xE50E;
const uint16_t INWV = 0xE50F;
const uint16_t VAWV = 0xE510;
const uint16_t VBWV = 0xE511;
const uint16_t VCWV = 0xE512;
const uint16_t VA2WV = 0xE513;
const uint16_t VB2WV = 0xE514;
const uint16_t VC2WV = 0xE515;
const uint16_t VNWV = 0xE516;
const uint16_t VN2WV = 0xE517;
const uint16_t AWATT = 0xE518;
const uint16_t BWATT = 0xE519;
const uint16_t CWATT = 0xE51A;
const uint16_t AVAR = 0xE51B;
const uint16_t BVAR = 0xE51C;
const uint16_t CVAR = 0xE51D;
const uint16_t AVA = 0xE51E;
const uint16_t BVA = 0xE51F;
const uint16_t CVA = 0xE520;
const uint16_t AVTHD = 0xE521;
const uint16_t AITHD = 0xE522;
const uint16_t BVTHD = 0xE523;
const uint16_t BITHD = 0xE524;
const uint16_t CVTHD = 0xE525;
const uint16_t CITHD = 0xE526;

// registers 0xE527 - 0xE52F are reserved

const uint16_t NVRMS = 0xE530;
const uint16_t NV2RMS = 0xE531;

// Checksum register
const uint16_t CHECKSUM = 0xE532;
const uint32_t CHECKSUM_RESET_VAL = 0x6BF87803;

const uint16_t PHSTATUS = 0xE600;
// Phase angle measurement registers
const uint16_t ANGLE0 = 0xE601;
const uint16_t ANGLE1 = 0xE602;
const uint16_t ANGLE2 = 0xE603;

const uint16_t COMPMODE = 0xE60E;
const uint8_t COMPMODE_ANGLESEL_Pos = 0;
const uint16_t COMPMODE_ANGLESEL_Msk = (0x3 << COMPMODE_ANGLESEL_Pos);
const uint16_t COMPMODE_ANGLESEL_V2I = (0x0 << COMPMODE_ANGLESEL_Pos);
const uint16_t COMPMODE_ANGLESEL_V2V = (0x1 << COMPMODE_ANGLESEL_Pos);
const uint16_t COMPMODE_ANGLESEL_I2I = (0x2 << COMPMODE_ANGLESEL_Pos);
const uint8_t COMPMODE_SELFREQ_Pos = 14;
const uint16_t COMPMODE_SELFREQ_Msk = (0x1 << COMPMODE_SELFREQ_Pos);
const uint16_t COMPMODE_SELFREQ_50HZ = (0x0 << COMPMODE_SELFREQ_Pos);
const uint16_t COMPMODE_SELFREQ_60HZ = (0x1 << COMPMODE_SELFREQ_Pos);

const uint16_t CONFIG = 0xE618;
const uint8_t CONFIG_HPFEN_Pos = 4;
const uint16_t CONFIG_HPFEN_Mask = (0x1 << CONFIG_HPFEN_Mask);
const uint16_t CONFIG_HPFEN = CONFIG_HPFEN_Mask;
const uint8_t CONFIG_SWRST_Pos = 7;
const uint16_t CONFIG_SWRST_Msk = (0x1 << CONFIG_SWRST_Pos);
const uint16_t CONFIG_SWRST = CONFIG_SWRST_Msk;
const uint8_t CONFIG_INSEL_Pos = 14;
const uint16_t CONFIG_INSEL_Msk = (0x1 << CONFIG_INSEL_Pos);
const uint16_t CONFIG_INSEL = CONFIG_INSEL_Msk;
const uint16_t CONFIG_ZX_DREADY_Pos = 0;
const uint16_t CONFIG_ZX_DREADY_Msk = (0x03 << CONFIG_ZX_DREADY_Pos);
const uint16_t CONFIG_ZX_DREADY_PHASEA = (0x01 << CONFIG_ZX_DREADY_Pos);

const uint16_t MMODE = 0xE700;
const uint8_t MMODE_PEAKSEL_Pos = 2;
const uint8_t MMODE_PEAKSEL_Msk = (0x7 << MMODE_PEAKSEL_Pos);
const uint8_t MMODE_PEAKSEL_A = (0x1 << (MMODE_PEAKSEL_Pos + 0));
const uint8_t MMODE_PEAKSEL_B = (0x1 << (MMODE_PEAKSEL_Pos + 1));
const uint8_t MMODE_PEAKSEL_C = (0x1 << (MMODE_PEAKSEL_Pos + 2));

const uint16_t LCYCMODE = 0xE702;
const uint8_t LCYCMODE_RSTREAD_Pos = 6;
const uint8_t LCYCMODE_RSTREAD_Msk = (0x1 << LCYCMODE_RSTREAD_Pos);

const uint16_t PEAKCYC = 0xE703;

// Voltage period measurement
const uint16_t APERIOD = 0xE905;
const uint16_t BPERIOD = 0xE906;
const uint16_t CPERIOD = 0xE907;

const uint16_t CONFIG2 = 0xEA00;
const uint16_t CONFIG3 = 0xE708;
} // namespace ade7978

#endif // SRC_EVDRIVERS_ADE7978_PRIV_H_
