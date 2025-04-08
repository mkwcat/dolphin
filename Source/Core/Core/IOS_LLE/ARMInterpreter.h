/*
    Copyright 2016-2024 melonDS team

    This file is part of melonDS.

    melonDS is free software: you can redistribute it and/or modify it under
    the terms of the GNU General Public License as published by the Free
    Software Foundation, either version 3 of the License, or (at your option)
    any later version.

    melonDS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with melonDS. If not, see http://www.gnu.org/licenses/.
*/

#ifndef ARMINTERPRETER_H
#define ARMINTERPRETER_H

#include "Core/IOS_LLE/ARM.h"

namespace IOS::LLE
{

class ARMInterpreter
{
public:
  static void (*ARMInstrTable[4096])(ARMv5* cpu);
  static void (*THUMBInstrTable[1024])(ARMv5* cpu);

  static void A_UNK(ARMv5* cpu);
  static void T_UNK(ARMv5* cpu);

  static void A_MSR_IMM(ARMv5* cpu);
  static void A_MSR_REG(ARMv5* cpu);
  static void A_MRS(ARMv5* cpu);
  static void A_MCR(ARMv5* cpu);
  static void A_MRC(ARMv5* cpu);
  static void A_SVC(ARMv5* cpu);

  static void T_SVC(ARMv5* cpu);

  static void A_BLX_IMM(ARMv5* cpu);  // I'm a special one look at me

  // ALU

#define A_PROTO_ALU_OP(x)                                                                          \
                                                                                                   \
  static void A_##x##_IMM(ARMv5* cpu);                                                             \
  static void A_##x##_REG_LSL_IMM(ARMv5* cpu);                                                     \
  static void A_##x##_REG_LSR_IMM(ARMv5* cpu);                                                     \
  static void A_##x##_REG_ASR_IMM(ARMv5* cpu);                                                     \
  static void A_##x##_REG_ROR_IMM(ARMv5* cpu);                                                     \
  static void A_##x##_REG_LSL_REG(ARMv5* cpu);                                                     \
  static void A_##x##_REG_LSR_REG(ARMv5* cpu);                                                     \
  static void A_##x##_REG_ASR_REG(ARMv5* cpu);                                                     \
  static void A_##x##_REG_ROR_REG(ARMv5* cpu);                                                     \
  static void A_##x##_IMM_S(ARMv5* cpu);                                                           \
  static void A_##x##_REG_LSL_IMM_S(ARMv5* cpu);                                                   \
  static void A_##x##_REG_LSR_IMM_S(ARMv5* cpu);                                                   \
  static void A_##x##_REG_ASR_IMM_S(ARMv5* cpu);                                                   \
  static void A_##x##_REG_ROR_IMM_S(ARMv5* cpu);                                                   \
  static void A_##x##_REG_LSL_REG_S(ARMv5* cpu);                                                   \
  static void A_##x##_REG_LSR_REG_S(ARMv5* cpu);                                                   \
  static void A_##x##_REG_ASR_REG_S(ARMv5* cpu);                                                   \
  static void A_##x##_REG_ROR_REG_S(ARMv5* cpu);

#define A_PROTO_ALU_TEST(x)                                                                        \
                                                                                                   \
  static void A_##x##_IMM(ARMv5* cpu);                                                             \
  static void A_##x##_REG_LSL_IMM(ARMv5* cpu);                                                     \
  static void A_##x##_REG_LSR_IMM(ARMv5* cpu);                                                     \
  static void A_##x##_REG_ASR_IMM(ARMv5* cpu);                                                     \
  static void A_##x##_REG_ROR_IMM(ARMv5* cpu);                                                     \
  static void A_##x##_REG_LSL_REG(ARMv5* cpu);                                                     \
  static void A_##x##_REG_LSR_REG(ARMv5* cpu);                                                     \
  static void A_##x##_REG_ASR_REG(ARMv5* cpu);                                                     \
  static void A_##x##_REG_ROR_REG(ARMv5* cpu);

  A_PROTO_ALU_OP(AND)
  A_PROTO_ALU_OP(EOR)
  A_PROTO_ALU_OP(SUB)
  A_PROTO_ALU_OP(RSB)
  A_PROTO_ALU_OP(ADD)
  A_PROTO_ALU_OP(ADC)
  A_PROTO_ALU_OP(SBC)
  A_PROTO_ALU_OP(RSC)
  A_PROTO_ALU_TEST(TST)
  A_PROTO_ALU_TEST(TEQ)
  A_PROTO_ALU_TEST(CMP)
  A_PROTO_ALU_TEST(CMN)
  A_PROTO_ALU_OP(ORR)
  A_PROTO_ALU_OP(MOV)
  A_PROTO_ALU_OP(BIC)
  A_PROTO_ALU_OP(MVN)

  static void A_MOV_REG_LSL_IMM_DBG(ARMv5* cpu);

  static void A_MUL(ARMv5* cpu);
  static void A_MLA(ARMv5* cpu);
  static void A_UMULL(ARMv5* cpu);
  static void A_UMLAL(ARMv5* cpu);
  static void A_SMULL(ARMv5* cpu);
  static void A_SMLAL(ARMv5* cpu);
  static void A_SMLAxy(ARMv5* cpu);
  static void A_SMLAWy(ARMv5* cpu);
  static void A_SMULxy(ARMv5* cpu);
  static void A_SMULWy(ARMv5* cpu);
  static void A_SMLALxy(ARMv5* cpu);

  static void A_CLZ(ARMv5* cpu);
  static void A_QADD(ARMv5* cpu);
  static void A_QSUB(ARMv5* cpu);
  static void A_QDADD(ARMv5* cpu);
  static void A_QDSUB(ARMv5* cpu);

  static void T_LSL_IMM(ARMv5* cpu);
  static void T_LSR_IMM(ARMv5* cpu);
  static void T_ASR_IMM(ARMv5* cpu);

  static void T_ADD_REG_(ARMv5* cpu);
  static void T_SUB_REG_(ARMv5* cpu);
  static void T_ADD_IMM_(ARMv5* cpu);
  static void T_SUB_IMM_(ARMv5* cpu);

  static void T_MOV_IMM(ARMv5* cpu);
  static void T_CMP_IMM(ARMv5* cpu);
  static void T_ADD_IMM(ARMv5* cpu);
  static void T_SUB_IMM(ARMv5* cpu);

  static void T_AND_REG(ARMv5* cpu);
  static void T_EOR_REG(ARMv5* cpu);
  static void T_LSL_REG(ARMv5* cpu);
  static void T_LSR_REG(ARMv5* cpu);
  static void T_ASR_REG(ARMv5* cpu);
  static void T_ADC_REG(ARMv5* cpu);
  static void T_SBC_REG(ARMv5* cpu);
  static void T_ROR_REG(ARMv5* cpu);
  static void T_TST_REG(ARMv5* cpu);
  static void T_NEG_REG(ARMv5* cpu);
  static void T_CMP_REG(ARMv5* cpu);
  static void T_CMN_REG(ARMv5* cpu);
  static void T_ORR_REG(ARMv5* cpu);
  static void T_MUL_REG(ARMv5* cpu);
  static void T_BIC_REG(ARMv5* cpu);
  static void T_MVN_REG(ARMv5* cpu);

  static void T_ADD_HIREG(ARMv5* cpu);
  static void T_CMP_HIREG(ARMv5* cpu);
  static void T_MOV_HIREG(ARMv5* cpu);

  static void T_ADD_PCREL(ARMv5* cpu);
  static void T_ADD_SPREL(ARMv5* cpu);
  static void T_ADD_SP(ARMv5* cpu);

#undef A_PROTO_ALU_OP
#undef A_PROTO_ALU_TEST

  // Branch

  static void A_B(ARMv5* cpu);
  static void A_BL(ARMv5* cpu);
  static void A_BX(ARMv5* cpu);
  static void A_BLX_REG(ARMv5* cpu);

  static void T_BCOND(ARMv5* cpu);
  static void T_BX(ARMv5* cpu);
  static void T_BLX_REG(ARMv5* cpu);
  static void T_B(ARMv5* cpu);
  static void T_BL_LONG_1(ARMv5* cpu);
  static void T_BL_LONG_2(ARMv5* cpu);

  // Load/Store

#define A_PROTO_WB_LDRSTR(x)                                                                       \
                                                                                                   \
  static void A_##x##_IMM(ARMv5* cpu);                                                             \
  static void A_##x##_REG_LSL(ARMv5* cpu);                                                         \
  static void A_##x##_REG_LSR(ARMv5* cpu);                                                         \
  static void A_##x##_REG_ASR(ARMv5* cpu);                                                         \
  static void A_##x##_REG_ROR(ARMv5* cpu);                                                         \
  static void A_##x##_POST_IMM(ARMv5* cpu);                                                        \
  static void A_##x##_POST_REG_LSL(ARMv5* cpu);                                                    \
  static void A_##x##_POST_REG_LSR(ARMv5* cpu);                                                    \
  static void A_##x##_POST_REG_ASR(ARMv5* cpu);                                                    \
  static void A_##x##_POST_REG_ROR(ARMv5* cpu);

  A_PROTO_WB_LDRSTR(STR)
  A_PROTO_WB_LDRSTR(STRB)
  A_PROTO_WB_LDRSTR(LDR)
  A_PROTO_WB_LDRSTR(LDRB)

#define A_PROTO_HD_LDRSTR(x)                                                                       \
                                                                                                   \
  static void A_##x##_IMM(ARMv5* cpu);                                                             \
  static void A_##x##_REG(ARMv5* cpu);                                                             \
  static void A_##x##_POST_IMM(ARMv5* cpu);                                                        \
  static void A_##x##_POST_REG(ARMv5* cpu);

  A_PROTO_HD_LDRSTR(STRH)
  A_PROTO_HD_LDRSTR(LDRD)
  A_PROTO_HD_LDRSTR(STRD)
  A_PROTO_HD_LDRSTR(LDRH)
  A_PROTO_HD_LDRSTR(LDRSB)
  A_PROTO_HD_LDRSTR(LDRSH)

  static void A_LDM(ARMv5* cpu);
  static void A_STM(ARMv5* cpu);

  static void A_SWP(ARMv5* cpu);
  static void A_SWPB(ARMv5* cpu);

  static void T_LDR_PCREL(ARMv5* cpu);

  static void T_STR_REG(ARMv5* cpu);
  static void T_STRB_REG(ARMv5* cpu);
  static void T_LDR_REG(ARMv5* cpu);
  static void T_LDRB_REG(ARMv5* cpu);

  static void T_STRH_REG(ARMv5* cpu);
  static void T_LDRSB_REG(ARMv5* cpu);
  static void T_LDRH_REG(ARMv5* cpu);
  static void T_LDRSH_REG(ARMv5* cpu);

  static void T_STR_IMM(ARMv5* cpu);
  static void T_LDR_IMM(ARMv5* cpu);
  static void T_STRB_IMM(ARMv5* cpu);
  static void T_LDRB_IMM(ARMv5* cpu);

  static void T_STRH_IMM(ARMv5* cpu);
  static void T_LDRH_IMM(ARMv5* cpu);

  static void T_STR_SPREL(ARMv5* cpu);
  static void T_LDR_SPREL(ARMv5* cpu);

  static void T_PUSH(ARMv5* cpu);
  static void T_POP(ARMv5* cpu);
  static void T_STMIA(ARMv5* cpu);
  static void T_LDMIA(ARMv5* cpu);

#undef A_PROTO_WB_LDRSTR
#undef A_PROTO_HD_LDRSTR
};

}  // namespace IOS::LLE
#endif  // ARMINTERPRETER_H
