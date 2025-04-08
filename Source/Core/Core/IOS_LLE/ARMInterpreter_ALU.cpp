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

#include <stdio.h>
#include "Core/IOS_LLE/ARM.h"
#include "Core/IOS_LLE/ARMInterpreter.h"

namespace IOS::LLE
{

inline bool CarryAdd(u32 a, u32 b)
{
  return (0xFFFFFFFF - a) < b;
}

inline bool CarrySub(u32 a, u32 b)
{
  return a >= b;
}

inline bool OverflowAdd(u32 a, u32 b)
{
  u32 res = a + b;
  return (!((a ^ b) & 0x80000000)) && ((a ^ res) & 0x80000000);
}

inline bool OverflowSub(u32 a, u32 b)
{
  u32 res = a - b;
  return ((a ^ b) & 0x80000000) && ((a ^ res) & 0x80000000);
}

inline bool OverflowAdc(u32 a, u32 b, u32 carry)
{
  s64 fullResult = (s64)(s32)a + (s32)b + carry;
  u32 res = a + b + carry;
  return (s32)res != fullResult;
}

inline bool OverflowSbc(u32 a, u32 b, u32 carry)
{
  s64 fullResult = (s64)(s32)a - (s32)b - carry;
  u32 res = a - b - carry;
  return (s32)res != fullResult;
}

#define LSL_IMM(x, s) x <<= s;

#define LSR_IMM(x, s)                                                                              \
  if (s == 0)                                                                                      \
    x = 0;                                                                                         \
  else                                                                                             \
    x >>= s;

#define ASR_IMM(x, s)                                                                              \
  if (s == 0)                                                                                      \
    x = ((s32)x) >> 31;                                                                            \
  else                                                                                             \
    x = ((s32)x) >> s;

#define ROR_IMM(x, s)                                                                              \
  if (s == 0)                                                                                      \
  {                                                                                                \
    x = (x >> 1) | ((cpu->m_reg_cpsr & 0x20000000) << 2);                                          \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    x = ROR(x, s);                                                                                 \
  }

#define LSL_IMM_S(x, s)                                                                            \
  if (s > 0)                                                                                       \
  {                                                                                                \
    cpu->SetC(x & (1 << (32 - s)));                                                                \
    x <<= s;                                                                                       \
  }

#define LSR_IMM_S(x, s)                                                                            \
  if (s == 0)                                                                                      \
  {                                                                                                \
    cpu->SetC(x & (1 << 31));                                                                      \
    x = 0;                                                                                         \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->SetC(x & (1 << (s - 1)));                                                                 \
    x >>= s;                                                                                       \
  }

#define ASR_IMM_S(x, s)                                                                            \
  if (s == 0)                                                                                      \
  {                                                                                                \
    cpu->SetC(x & (1 << 31));                                                                      \
    x = ((s32)x) >> 31;                                                                            \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->SetC(x & (1 << (s - 1)));                                                                 \
    x = ((s32)x) >> s;                                                                             \
  }

#define ROR_IMM_S(x, s)                                                                            \
  if (s == 0)                                                                                      \
  {                                                                                                \
    u32 newc = (x & 1);                                                                            \
    x = (x >> 1) | ((cpu->m_reg_cpsr & 0x20000000) << 2);                                          \
    cpu->SetC(newc);                                                                               \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->SetC(x & (1 << (s - 1)));                                                                 \
    x = ROR(x, s);                                                                                 \
  }

#define LSL_REG(x, s)                                                                              \
  if (s > 31)                                                                                      \
    x = 0;                                                                                         \
  else                                                                                             \
    x <<= s;

#define LSR_REG(x, s)                                                                              \
  if (s > 31)                                                                                      \
    x = 0;                                                                                         \
  else                                                                                             \
    x >>= s;

#define ASR_REG(x, s)                                                                              \
  if (s > 31)                                                                                      \
    x = ((s32)x) >> 31;                                                                            \
  else                                                                                             \
    x = ((s32)x) >> s;

#define ROR_REG(x, s) x = ROR(x, (s & 0x1F));

#define LSL_REG_S(x, s)                                                                            \
  if (s > 31)                                                                                      \
  {                                                                                                \
    cpu->SetC((s > 32) ? 0 : (x & (1 << 0)));                                                      \
    x = 0;                                                                                         \
  }                                                                                                \
  else if (s > 0)                                                                                  \
  {                                                                                                \
    cpu->SetC(x & (1 << (32 - s)));                                                                \
    x <<= s;                                                                                       \
  }

#define LSR_REG_S(x, s)                                                                            \
  if (s > 31)                                                                                      \
  {                                                                                                \
    cpu->SetC((s > 32) ? 0 : (x & (1 << 31)));                                                     \
    x = 0;                                                                                         \
  }                                                                                                \
  else if (s > 0)                                                                                  \
  {                                                                                                \
    cpu->SetC(x & (1 << (s - 1)));                                                                 \
    x >>= s;                                                                                       \
  }

#define ASR_REG_S(x, s)                                                                            \
  if (s > 31)                                                                                      \
  {                                                                                                \
    cpu->SetC(x & (1 << 31));                                                                      \
    x = ((s32)x) >> 31;                                                                            \
  }                                                                                                \
  else if (s > 0)                                                                                  \
  {                                                                                                \
    cpu->SetC(x & (1 << (s - 1)));                                                                 \
    x = ((s32)x) >> s;                                                                             \
  }

#define ROR_REG_S(x, s)                                                                            \
  if (s > 0)                                                                                       \
    cpu->SetC(x & (1 << (s - 1)));                                                                 \
  x = ROR(x, (s & 0x1F));

#define A_CALC_OP2_IMM u32 b = ROR(cpu->m_inst & 0xFF, (cpu->m_inst >> 7) & 0x1E);

#define A_CALC_OP2_IMM_S                                                                           \
  u32 b = ROR(cpu->m_inst & 0xFF, (cpu->m_inst >> 7) & 0x1E);                                  \
  if ((cpu->m_inst >> 7) & 0x1E)                                                                 \
    cpu->SetC(b & 0x80000000);

#define A_CALC_OP2_REG_SHIFT_IMM(shiftop)                                                          \
  u32 b = cpu->m_reg[cpu->m_inst & 0xF];                                                             \
  u32 s = (cpu->m_inst >> 7) & 0x1F;                                                             \
  shiftop(b, s);

#define A_CALC_OP2_REG_SHIFT_REG(shiftop)                                                          \
  u32 b = cpu->m_reg[cpu->m_inst & 0xF];                                                             \
  if ((cpu->m_inst & 0xF) == 15)                                                                 \
    b += 4;                                                                                        \
  shiftop(b, (cpu->m_reg[(cpu->m_inst >> 8) & 0xF] & 0xFF));

#define A_IMPLEMENT_ALU_OP(x, s)                                                                   \
                                                                                                   \
  void ARMInterpreter::A_##x##_IMM(ARMv5* cpu)                                                     \
  {                                                                                                \
    A_CALC_OP2_IMM                                                                                 \
    A_##x(0)                                                                                       \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_LSL_IMM(ARMv5* cpu)                                             \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_IMM(LSL_IMM)                                                              \
    A_##x(0)                                                                                       \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_LSR_IMM(ARMv5* cpu)                                             \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_IMM(LSR_IMM)                                                              \
    A_##x(0)                                                                                       \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_ASR_IMM(ARMv5* cpu)                                             \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_IMM(ASR_IMM)                                                              \
    A_##x(0)                                                                                       \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_ROR_IMM(ARMv5* cpu)                                             \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_IMM(ROR_IMM)                                                              \
    A_##x(0)                                                                                       \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_LSL_REG(ARMv5* cpu)                                             \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_REG(LSL_REG)                                                              \
    A_##x(1)                                                                                       \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_LSR_REG(ARMv5* cpu)                                             \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_REG(LSR_REG)                                                              \
    A_##x(1)                                                                                       \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_ASR_REG(ARMv5* cpu)                                             \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_REG(ASR_REG)                                                              \
    A_##x(1)                                                                                       \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_ROR_REG(ARMv5* cpu)                                             \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_REG(ROR_REG)                                                              \
    A_##x(1)                                                                                       \
  }                                                                                                \
  void ARMInterpreter::A_##x##_IMM_S(ARMv5* cpu)                                                   \
  {                                                                                                \
    A_CALC_OP2_IMM##s A_##x##_S(0)                                                                 \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_LSL_IMM_S(ARMv5* cpu)                                           \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_IMM(LSL_IMM##s)                                                           \
    A_##x##_S(0)                                                                                   \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_LSR_IMM_S(ARMv5* cpu)                                           \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_IMM(LSR_IMM##s)                                                           \
    A_##x##_S(0)                                                                                   \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_ASR_IMM_S(ARMv5* cpu)                                           \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_IMM(ASR_IMM##s)                                                           \
    A_##x##_S(0)                                                                                   \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_ROR_IMM_S(ARMv5* cpu)                                           \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_IMM(ROR_IMM##s)                                                           \
    A_##x##_S(0)                                                                                   \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_LSL_REG_S(ARMv5* cpu)                                           \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_REG(LSL_REG##s)                                                           \
    A_##x##_S(1)                                                                                   \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_LSR_REG_S(ARMv5* cpu)                                           \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_REG(LSR_REG##s)                                                           \
    A_##x##_S(1)                                                                                   \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_ASR_REG_S(ARMv5* cpu)                                           \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_REG(ASR_REG##s)                                                           \
    A_##x##_S(1)                                                                                   \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_ROR_REG_S(ARMv5* cpu)                                           \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_REG(ROR_REG##s)                                                           \
    A_##x##_S(1)                                                                                   \
  }

#define A_IMPLEMENT_ALU_TEST(x, s)                                                                 \
                                                                                                   \
  void ARMInterpreter::A_##x##_IMM(ARMv5* cpu)                                                     \
  {                                                                                                \
    A_CALC_OP2_IMM##s A_##x(0)                                                                     \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_LSL_IMM(ARMv5* cpu)                                             \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_IMM(LSL_IMM##s)                                                           \
    A_##x(0)                                                                                       \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_LSR_IMM(ARMv5* cpu)                                             \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_IMM(LSR_IMM##s)                                                           \
    A_##x(0)                                                                                       \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_ASR_IMM(ARMv5* cpu)                                             \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_IMM(ASR_IMM##s)                                                           \
    A_##x(0)                                                                                       \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_ROR_IMM(ARMv5* cpu)                                             \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_IMM(ROR_IMM##s)                                                           \
    A_##x(0)                                                                                       \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_LSL_REG(ARMv5* cpu)                                             \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_REG(LSL_REG##s)                                                           \
    A_##x(1)                                                                                       \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_LSR_REG(ARMv5* cpu)                                             \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_REG(LSR_REG##s)                                                           \
    A_##x(1)                                                                                       \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_ASR_REG(ARMv5* cpu)                                             \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_REG(ASR_REG##s)                                                           \
    A_##x(1)                                                                                       \
  }                                                                                                \
  void ARMInterpreter::A_##x##_REG_ROR_REG(ARMv5* cpu)                                             \
  {                                                                                                \
    A_CALC_OP2_REG_SHIFT_REG(ROR_REG##s)                                                           \
    A_##x(1)                                                                                       \
  }

#define A_AND(c)                                                                                   \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res = a & b;                                                                                 \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(res & ~1);                                                                         \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;                                                     \
  }

#define A_AND_S(c)                                                                                 \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res = a & b;                                                                                 \
  cpu->SetNZ(res & 0x80000000, !res);                                                              \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(res, true);                                                                        \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;                                                     \
  }

A_IMPLEMENT_ALU_OP(AND, _S)

#define A_EOR(c)                                                                                   \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res = a ^ b;                                                                                 \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(res & ~1);                                                                         \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;                                                     \
  }

#define A_EOR_S(c)                                                                                 \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res = a ^ b;                                                                                 \
  cpu->SetNZ(res & 0x80000000, !res);                                                              \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(res, true);                                                                        \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;                                                     \
  }

A_IMPLEMENT_ALU_OP(EOR, _S)

#define A_SUB(c)                                                                                   \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res = a - b;                                                                                 \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(res & ~1);                                                                         \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;                                                     \
  }

#define A_SUB_S(c)                                                                                 \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res = a - b;                                                                                 \
  cpu->SetNZCV(res & 0x80000000, !res, CarrySub(a, b), OverflowSub(a, b));                         \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(res, true);                                                                        \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;                                                     \
  }

A_IMPLEMENT_ALU_OP(SUB, )

#define A_RSB(c)                                                                                   \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res = b - a;                                                                                 \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(res & ~1);                                                                         \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;                                                     \
  }

#define A_RSB_S(c)                                                                                 \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res = b - a;                                                                                 \
  cpu->SetNZCV(res & 0x80000000, !res, CarrySub(b, a), OverflowSub(b, a));                         \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(res, true);                                                                        \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;                                                     \
  }

A_IMPLEMENT_ALU_OP(RSB, )

#define A_ADD(c)                                                                                   \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res = a + b;                                                                                 \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(res & ~1);                                                                         \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;                                                     \
  }

#define A_ADD_S(c)                                                                                 \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res = a + b;                                                                                 \
  cpu->SetNZCV(res & 0x80000000, !res, CarryAdd(a, b), OverflowAdd(a, b));                         \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(res, true);                                                                        \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;                                                     \
  }

A_IMPLEMENT_ALU_OP(ADD, )

#define A_ADC(c)                                                                                   \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res = a + b + (cpu->m_reg_cpsr & 0x20000000 ? 1 : 0);                                        \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(res & ~1);                                                                         \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;                                                     \
  }

#define A_ADC_S(c)                                                                                 \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res_tmp = a + b;                                                                             \
  u32 carry = (cpu->m_reg_cpsr & 0x20000000 ? 1 : 0);                                              \
  u32 res = res_tmp + carry;                                                                       \
  cpu->SetNZCV(res & 0x80000000, !res, CarryAdd(a, b) || CarryAdd(res_tmp, carry),                 \
               OverflowAdc(a, b, carry));                                                          \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(res, true);                                                                        \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;                                                     \
  }

A_IMPLEMENT_ALU_OP(ADC, )

#define A_SBC(c)                                                                                   \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res = a - b - (cpu->m_reg_cpsr & 0x20000000 ? 0 : 1);                                        \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(res & ~1);                                                                         \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;                                                     \
  }

#define A_SBC_S(c)                                                                                 \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res_tmp = a - b;                                                                             \
  u32 carry = (cpu->m_reg_cpsr & 0x20000000 ? 0 : 1);                                              \
  u32 res = res_tmp - carry;                                                                       \
  cpu->SetNZCV(res & 0x80000000, !res, CarrySub(a, b) && CarrySub(res_tmp, carry),                 \
               OverflowSbc(a, b, carry));                                                          \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(res, true);                                                                        \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;                                                     \
  }

A_IMPLEMENT_ALU_OP(SBC, )

#define A_RSC(c)                                                                                   \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res = b - a - (cpu->m_reg_cpsr & 0x20000000 ? 0 : 1);                                        \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(res & ~1);                                                                         \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;                                                     \
  }

#define A_RSC_S(c)                                                                                 \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res_tmp = b - a;                                                                             \
  u32 carry = (cpu->m_reg_cpsr & 0x20000000 ? 0 : 1);                                              \
  u32 res = res_tmp - carry;                                                                       \
  cpu->SetNZCV(res & 0x80000000, !res, CarrySub(b, a) && CarrySub(res_tmp, carry),                 \
               OverflowSbc(b, a, carry));                                                          \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(res, true);                                                                        \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;                                                     \
  }

A_IMPLEMENT_ALU_OP(RSC, )

#define A_TST(c)                                                                                   \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res = a & b;                                                                                 \
  cpu->SetNZ(res & 0x80000000, !res);                                                              \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();

A_IMPLEMENT_ALU_TEST(TST, _S)

#define A_TEQ(c)                                                                                   \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res = a ^ b;                                                                                 \
  cpu->SetNZ(res & 0x80000000, !res);                                                              \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();

A_IMPLEMENT_ALU_TEST(TEQ, _S)

#define A_CMP(c)                                                                                   \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res = a - b;                                                                                 \
  cpu->SetNZCV(res & 0x80000000, !res, CarrySub(a, b), OverflowSub(a, b));                         \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();

A_IMPLEMENT_ALU_TEST(CMP, )

#define A_CMN(c)                                                                                   \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res = a + b;                                                                                 \
  cpu->SetNZCV(res & 0x80000000, !res, CarryAdd(a, b), OverflowAdd(a, b));                         \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();

A_IMPLEMENT_ALU_TEST(CMN, )

#define A_ORR(c)                                                                                   \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res = a | b;                                                                                 \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(res & ~1);                                                                         \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;                                                     \
  }

#define A_ORR_S(c)                                                                                 \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res = a | b;                                                                                 \
  cpu->SetNZ(res & 0x80000000, !res);                                                              \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(res, true);                                                                        \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;                                                     \
  }

A_IMPLEMENT_ALU_OP(ORR, _S)

#define A_MOV(c)                                                                                   \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(b & ~1);                                                                           \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = b;                                                       \
  }

#define A_MOV_S(c)                                                                                 \
  cpu->SetNZ(b & 0x80000000, !b);                                                                  \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(b, true);                                                                          \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = b;                                                       \
  }

A_IMPLEMENT_ALU_OP(MOV, _S)

// debug hook

void ARMInterpreter::A_MOV_REG_LSL_IMM_DBG(ARMv5* cpu)
{
  A_MOV_REG_LSL_IMM(cpu);

#if 0
    // nocash-style debugging hook
    if ( cpu->m_inst == 0xE1A0C00C &&                   // mov r12, r12
        (cpu->m_next_inst[0] & 0xFF000000) == 0xEA000000 && // branch
        (cpu->m_next_inst[1] & 0xFFFF) == 0x6464)
    {
        // GBATek says the two bytes after the 2nd ID are _reserved_ for flags
        // but since they serve no purpose ATTOW, we can skip them
        u32 addr = cpu->m_reg[15] + 4; // Skip 2nd ID and flags
        // TODO: Pass flags to NocashPrint
        cpu->NDS.NocashPrint(cpu->Num, addr);
    }
#endif
}

#define A_BIC(c)                                                                                   \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res = a & ~b;                                                                                \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(res & ~1);                                                                         \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;                                                     \
  }

#define A_BIC_S(c)                                                                                 \
  u32 a = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];                                                     \
  u32 res = a & ~b;                                                                                \
  cpu->SetNZ(res & 0x80000000, !res);                                                              \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(res, true);                                                                        \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;                                                     \
  }

A_IMPLEMENT_ALU_OP(BIC, _S)

#define A_MVN(c)                                                                                   \
  b = ~b;                                                                                          \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(b & ~1);                                                                           \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = b;                                                       \
  }

#define A_MVN_S(c)                                                                                 \
  b = ~b;                                                                                          \
  cpu->SetNZ(b & 0x80000000, !b);                                                                  \
  if (c)                                                                                           \
    cpu->AddCycles_CI(c);                                                                          \
  else                                                                                             \
    cpu->AddCycles_C();                                                                            \
  if (((cpu->m_inst >> 12) & 0xF) == 15)                                                         \
  {                                                                                                \
    cpu->JumpTo(b, true);                                                                          \
  }                                                                                                \
  else                                                                                             \
  {                                                                                                \
    cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = b;                                                       \
  }

A_IMPLEMENT_ALU_OP(MVN, _S)

void ARMInterpreter::A_MUL(ARMv5* cpu)
{
  u32 rm = cpu->m_reg[cpu->m_inst & 0xF];
  u32 rs = cpu->m_reg[(cpu->m_inst >> 8) & 0xF];

  u32 res = rm * rs;

  cpu->m_reg[(cpu->m_inst >> 16) & 0xF] = res;
  if (cpu->m_inst & (1 << 20))
  {
    cpu->SetNZ(res & 0x80000000, !res);
    if (cpu->Num == 1)
      cpu->SetC(0);
  }

  u32 cycles;
  if (cpu->Num == 0)
    cycles = (cpu->m_inst & (1 << 20)) ? 3 : 1;
  else
  {
    if ((rs & 0xFFFFFF00) == 0x00000000 || (rs & 0xFFFFFF00) == 0xFFFFFF00)
      cycles = 1;
    else if ((rs & 0xFFFF0000) == 0x00000000 || (rs & 0xFFFF0000) == 0xFFFF0000)
      cycles = 2;
    else if ((rs & 0xFF000000) == 0x00000000 || (rs & 0xFF000000) == 0xFF000000)
      cycles = 3;
    else
      cycles = 4;
  }

  cpu->AddCycles_CI(cycles);
}

void ARMInterpreter::A_MLA(ARMv5* cpu)
{
  u32 rm = cpu->m_reg[cpu->m_inst & 0xF];
  u32 rs = cpu->m_reg[(cpu->m_inst >> 8) & 0xF];
  u32 rn = cpu->m_reg[(cpu->m_inst >> 12) & 0xF];

  u32 res = (rm * rs) + rn;

  cpu->m_reg[(cpu->m_inst >> 16) & 0xF] = res;
  if (cpu->m_inst & (1 << 20))
  {
    cpu->SetNZ(res & 0x80000000, !res);
    if (cpu->Num == 1)
      cpu->SetC(0);
  }

  u32 cycles;
  if (cpu->Num == 0)
    cycles = (cpu->m_inst & (1 << 20)) ? 3 : 1;
  else
  {
    if ((rs & 0xFFFFFF00) == 0x00000000 || (rs & 0xFFFFFF00) == 0xFFFFFF00)
      cycles = 2;
    else if ((rs & 0xFFFF0000) == 0x00000000 || (rs & 0xFFFF0000) == 0xFFFF0000)
      cycles = 3;
    else if ((rs & 0xFF000000) == 0x00000000 || (rs & 0xFF000000) == 0xFF000000)
      cycles = 4;
    else
      cycles = 5;
  }

  cpu->AddCycles_CI(cycles);
}

void ARMInterpreter::A_UMULL(ARMv5* cpu)
{
  u32 rm = cpu->m_reg[cpu->m_inst & 0xF];
  u32 rs = cpu->m_reg[(cpu->m_inst >> 8) & 0xF];

  u64 res = (u64)rm * (u64)rs;

  cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = (u32)res;
  cpu->m_reg[(cpu->m_inst >> 16) & 0xF] = (u32)(res >> 32ULL);
  if (cpu->m_inst & (1 << 20))
  {
    cpu->SetNZ((u32)(res >> 63ULL), !res);
  }

  u32 cycles = (cpu->m_inst & (1 << 20)) ? 3 : 1;

  cpu->AddCycles_CI(cycles);
}

void ARMInterpreter::A_UMLAL(ARMv5* cpu)
{
  u32 rm = cpu->m_reg[cpu->m_inst & 0xF];
  u32 rs = cpu->m_reg[(cpu->m_inst >> 8) & 0xF];

  u64 res = (u64)rm * (u64)rs;

  u64 rd = (u64)cpu->m_reg[(cpu->m_inst >> 12) & 0xF] |
           ((u64)cpu->m_reg[(cpu->m_inst >> 16) & 0xF] << 32ULL);
  res += rd;

  cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = (u32)res;
  cpu->m_reg[(cpu->m_inst >> 16) & 0xF] = (u32)(res >> 32ULL);
  if (cpu->m_inst & (1 << 20))
  {
    cpu->SetNZ((u32)(res >> 63ULL), !res);
  }

  u32 cycles = (cpu->m_inst & (1 << 20)) ? 3 : 1;

  cpu->AddCycles_CI(cycles);
}

void ARMInterpreter::A_SMULL(ARMv5* cpu)
{
  u32 rm = cpu->m_reg[cpu->m_inst & 0xF];
  u32 rs = cpu->m_reg[(cpu->m_inst >> 8) & 0xF];

  s64 res = (s64)(s32)rm * (s64)(s32)rs;

  cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = (u32)res;
  cpu->m_reg[(cpu->m_inst >> 16) & 0xF] = (u32)(res >> 32ULL);
  if (cpu->m_inst & (1 << 20))
  {
    cpu->SetNZ((u32)(res >> 63ULL), !res);
    if (cpu->Num == 1)
      cpu->SetC(0);
  }

  u32 cycles;
  if (cpu->Num == 0)
    cycles = (cpu->m_inst & (1 << 20)) ? 3 : 1;
  else
  {
    if ((rs & 0xFFFFFF00) == 0x00000000 || (rs & 0xFFFFFF00) == 0xFFFFFF00)
      cycles = 2;
    else if ((rs & 0xFFFF0000) == 0x00000000 || (rs & 0xFFFF0000) == 0xFFFF0000)
      cycles = 3;
    else if ((rs & 0xFF000000) == 0x00000000 || (rs & 0xFF000000) == 0xFF000000)
      cycles = 4;
    else
      cycles = 5;
  }

  cpu->AddCycles_CI(cycles);
}

void ARMInterpreter::A_SMLAL(ARMv5* cpu)
{
  u32 rm = cpu->m_reg[cpu->m_inst & 0xF];
  u32 rs = cpu->m_reg[(cpu->m_inst >> 8) & 0xF];

  s64 res = (s64)(s32)rm * (s64)(s32)rs;

  s64 rd = (s64)((u64)cpu->m_reg[(cpu->m_inst >> 12) & 0xF] |
                 ((u64)cpu->m_reg[(cpu->m_inst >> 16) & 0xF] << 32ULL));
  res += rd;

  cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = (u32)res;
  cpu->m_reg[(cpu->m_inst >> 16) & 0xF] = (u32)(res >> 32ULL);
  if (cpu->m_inst & (1 << 20))
  {
    cpu->SetNZ((u32)(res >> 63ULL), !res);
    if (cpu->Num == 1)
      cpu->SetC(0);
  }

  u32 cycles;
  if (cpu->Num == 0)
    cycles = (cpu->m_inst & (1 << 20)) ? 3 : 1;
  else
  {
    if ((rs & 0xFFFFFF00) == 0x00000000 || (rs & 0xFFFFFF00) == 0xFFFFFF00)
      cycles = 2;
    else if ((rs & 0xFFFF0000) == 0x00000000 || (rs & 0xFFFF0000) == 0xFFFF0000)
      cycles = 3;
    else if ((rs & 0xFF000000) == 0x00000000 || (rs & 0xFF000000) == 0xFF000000)
      cycles = 4;
    else
      cycles = 5;
  }

  cpu->AddCycles_CI(cycles);
}

void ARMInterpreter::A_SMLAxy(ARMv5* cpu)
{
  if (cpu->Num != 0)
    return;

  u32 rm = cpu->m_reg[cpu->m_inst & 0xF];
  u32 rs = cpu->m_reg[(cpu->m_inst >> 8) & 0xF];
  u32 rn = cpu->m_reg[(cpu->m_inst >> 12) & 0xF];

  if (cpu->m_inst & (1 << 5))
    rm >>= 16;
  else
    rm &= 0xFFFF;
  if (cpu->m_inst & (1 << 6))
    rs >>= 16;
  else
    rs &= 0xFFFF;

  u32 res_mul = ((s16)rm * (s16)rs);
  u32 res = res_mul + rn;

  cpu->m_reg[(cpu->m_inst >> 16) & 0xF] = res;
  if (OverflowAdd(res_mul, rn))
    cpu->m_reg_cpsr |= 0x08000000;

  cpu->AddCycles_C();  // TODO: interlock??
}

void ARMInterpreter::A_SMLAWy(ARMv5* cpu)
{
  if (cpu->Num != 0)
    return;

  u32 rm = cpu->m_reg[cpu->m_inst & 0xF];
  u32 rs = cpu->m_reg[(cpu->m_inst >> 8) & 0xF];
  u32 rn = cpu->m_reg[(cpu->m_inst >> 12) & 0xF];

  if (cpu->m_inst & (1 << 6))
    rs >>= 16;
  else
    rs &= 0xFFFF;

  u32 res_mul = ((s64)(s32)rm * (s16)rs) >> 16;
  u32 res = res_mul + rn;

  cpu->m_reg[(cpu->m_inst >> 16) & 0xF] = res;
  if (OverflowAdd(res_mul, rn))
    cpu->m_reg_cpsr |= 0x08000000;

  cpu->AddCycles_C();  // TODO: interlock??
}

void ARMInterpreter::A_SMULxy(ARMv5* cpu)
{
  if (cpu->Num != 0)
    return;

  u32 rm = cpu->m_reg[cpu->m_inst & 0xF];
  u32 rs = cpu->m_reg[(cpu->m_inst >> 8) & 0xF];

  if (cpu->m_inst & (1 << 5))
    rm >>= 16;
  else
    rm &= 0xFFFF;
  if (cpu->m_inst & (1 << 6))
    rs >>= 16;
  else
    rs &= 0xFFFF;

  u32 res = ((s16)rm * (s16)rs);

  cpu->m_reg[(cpu->m_inst >> 16) & 0xF] = res;
  cpu->AddCycles_C();  // TODO: interlock??
}

void ARMInterpreter::A_SMULWy(ARMv5* cpu)
{
  if (cpu->Num != 0)
    return;

  u32 rm = cpu->m_reg[cpu->m_inst & 0xF];
  u32 rs = cpu->m_reg[(cpu->m_inst >> 8) & 0xF];

  if (cpu->m_inst & (1 << 6))
    rs >>= 16;
  else
    rs &= 0xFFFF;

  u32 res = ((s64)(s32)rm * (s16)rs) >> 16;

  cpu->m_reg[(cpu->m_inst >> 16) & 0xF] = res;
  cpu->AddCycles_C();  // TODO: interlock??
}

void ARMInterpreter::A_SMLALxy(ARMv5* cpu)
{
  if (cpu->Num != 0)
    return;

  u32 rm = cpu->m_reg[cpu->m_inst & 0xF];
  u32 rs = cpu->m_reg[(cpu->m_inst >> 8) & 0xF];

  if (cpu->m_inst & (1 << 5))
    rm >>= 16;
  else
    rm &= 0xFFFF;
  if (cpu->m_inst & (1 << 6))
    rs >>= 16;
  else
    rs &= 0xFFFF;

  s64 res = (s64)(s16)rm * (s64)(s16)rs;

  s64 rd = (s64)((u64)cpu->m_reg[(cpu->m_inst >> 12) & 0xF] |
                 ((u64)cpu->m_reg[(cpu->m_inst >> 16) & 0xF] << 32ULL));
  res += rd;

  cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = (u32)res;
  cpu->m_reg[(cpu->m_inst >> 16) & 0xF] = (u32)(res >> 32ULL);

  cpu->AddCycles_CI(1);  // TODO: interlock??
}

void ARMInterpreter::A_CLZ(ARMv5* cpu)
{
  if (cpu->Num != 0)
    return A_UNK(cpu);

  u32 val = cpu->m_reg[cpu->m_inst & 0xF];

  u32 res = 0;
  while ((val & 0xFF000000) == 0)
  {
    res += 8;
    val <<= 8;
    val |= 0xFF;
  }
  while ((val & 0x80000000) == 0)
  {
    res++;
    val <<= 1;
    val |= 0x1;
  }

  cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;
  cpu->AddCycles_C();
}

void ARMInterpreter::A_QADD(ARMv5* cpu)
{
  if (cpu->Num != 0)
    return A_UNK(cpu);

  u32 rm = cpu->m_reg[cpu->m_inst & 0xF];
  u32 rn = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];

  u32 res = rm + rn;
  if (OverflowAdd(rm, rn))
  {
    res = (res & 0x80000000) ? 0x7FFFFFFF : 0x80000000;
    cpu->m_reg_cpsr |= 0x08000000;
  }

  cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;
  cpu->AddCycles_C();  // TODO: interlock??
}

void ARMInterpreter::A_QSUB(ARMv5* cpu)
{
  if (cpu->Num != 0)
    return A_UNK(cpu);

  u32 rm = cpu->m_reg[cpu->m_inst & 0xF];
  u32 rn = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];

  u32 res = rm - rn;
  if (OverflowSub(rm, rn))
  {
    res = (res & 0x80000000) ? 0x7FFFFFFF : 0x80000000;
    cpu->m_reg_cpsr |= 0x08000000;
  }

  cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;
  cpu->AddCycles_C();  // TODO: interlock??
}

void ARMInterpreter::A_QDADD(ARMv5* cpu)
{
  if (cpu->Num != 0)
    return A_UNK(cpu);

  u32 rm = cpu->m_reg[cpu->m_inst & 0xF];
  u32 rn = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];

  if (OverflowAdd(rn, rn))
  {
    rn = (rn & 0x80000000) ? 0x80000000 : 0x7FFFFFFF;
    cpu->m_reg_cpsr |= 0x08000000;  // CHECKME
  }
  else
    rn <<= 1;

  u32 res = rm + rn;
  if (OverflowAdd(rm, rn))
  {
    res = (res & 0x80000000) ? 0x7FFFFFFF : 0x80000000;
    cpu->m_reg_cpsr |= 0x08000000;
  }

  cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;
  cpu->AddCycles_C();  // TODO: interlock??
}

void ARMInterpreter::A_QDSUB(ARMv5* cpu)
{
  if (cpu->Num != 0)
    return A_UNK(cpu);

  u32 rm = cpu->m_reg[cpu->m_inst & 0xF];
  u32 rn = cpu->m_reg[(cpu->m_inst >> 16) & 0xF];

  if (OverflowAdd(rn, rn))
  {
    rn = (rn & 0x80000000) ? 0x80000000 : 0x7FFFFFFF;
    cpu->m_reg_cpsr |= 0x08000000;  // CHECKME
  }
  else
    rn <<= 1;

  u32 res = rm - rn;
  if (OverflowSub(rm, rn))
  {
    res = (res & 0x80000000) ? 0x7FFFFFFF : 0x80000000;
    cpu->m_reg_cpsr |= 0x08000000;
  }

  cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = res;
  cpu->AddCycles_C();  // TODO: interlock??
}

// ---- THUMB ----------------------------------

void ARMInterpreter::T_LSL_IMM(ARMv5* cpu)
{
  u32 op = cpu->m_reg[(cpu->m_inst >> 3) & 0x7];
  u32 s = (cpu->m_inst >> 6) & 0x1F;
  LSL_IMM_S(op, s);
  cpu->m_reg[cpu->m_inst & 0x7] = op;
  cpu->SetNZ(op & 0x80000000, !op);
  cpu->AddCycles_C();
}

void ARMInterpreter::T_LSR_IMM(ARMv5* cpu)
{
  u32 op = cpu->m_reg[(cpu->m_inst >> 3) & 0x7];
  u32 s = (cpu->m_inst >> 6) & 0x1F;
  LSR_IMM_S(op, s);
  cpu->m_reg[cpu->m_inst & 0x7] = op;
  cpu->SetNZ(op & 0x80000000, !op);
  cpu->AddCycles_C();
}

void ARMInterpreter::T_ASR_IMM(ARMv5* cpu)
{
  u32 op = cpu->m_reg[(cpu->m_inst >> 3) & 0x7];
  u32 s = (cpu->m_inst >> 6) & 0x1F;
  ASR_IMM_S(op, s);
  cpu->m_reg[cpu->m_inst & 0x7] = op;
  cpu->SetNZ(op & 0x80000000, !op);
  cpu->AddCycles_C();
}

void ARMInterpreter::T_ADD_REG_(ARMv5* cpu)
{
  u32 a = cpu->m_reg[(cpu->m_inst >> 3) & 0x7];
  u32 b = cpu->m_reg[(cpu->m_inst >> 6) & 0x7];
  u32 res = a + b;
  cpu->m_reg[cpu->m_inst & 0x7] = res;
  cpu->SetNZCV(res & 0x80000000, !res, CarryAdd(a, b), OverflowAdd(a, b));
  cpu->AddCycles_C();
}

void ARMInterpreter::T_SUB_REG_(ARMv5* cpu)
{
  u32 a = cpu->m_reg[(cpu->m_inst >> 3) & 0x7];
  u32 b = cpu->m_reg[(cpu->m_inst >> 6) & 0x7];
  u32 res = a - b;
  cpu->m_reg[cpu->m_inst & 0x7] = res;
  cpu->SetNZCV(res & 0x80000000, !res, CarrySub(a, b), OverflowSub(a, b));
  cpu->AddCycles_C();
}

void ARMInterpreter::T_ADD_IMM_(ARMv5* cpu)
{
  u32 a = cpu->m_reg[(cpu->m_inst >> 3) & 0x7];
  u32 b = (cpu->m_inst >> 6) & 0x7;
  u32 res = a + b;
  cpu->m_reg[cpu->m_inst & 0x7] = res;
  cpu->SetNZCV(res & 0x80000000, !res, CarryAdd(a, b), OverflowAdd(a, b));
  cpu->AddCycles_C();
}

void ARMInterpreter::T_SUB_IMM_(ARMv5* cpu)
{
  u32 a = cpu->m_reg[(cpu->m_inst >> 3) & 0x7];
  u32 b = (cpu->m_inst >> 6) & 0x7;
  u32 res = a - b;
  cpu->m_reg[cpu->m_inst & 0x7] = res;
  cpu->SetNZCV(res & 0x80000000, !res, CarrySub(a, b), OverflowSub(a, b));
  cpu->AddCycles_C();
}

void ARMInterpreter::T_MOV_IMM(ARMv5* cpu)
{
  u32 b = cpu->m_inst & 0xFF;
  cpu->m_reg[(cpu->m_inst >> 8) & 0x7] = b;
  cpu->SetNZ(0, !b);
  cpu->AddCycles_C();
}

void ARMInterpreter::T_CMP_IMM(ARMv5* cpu)
{
  u32 a = cpu->m_reg[(cpu->m_inst >> 8) & 0x7];
  u32 b = cpu->m_inst & 0xFF;
  u32 res = a - b;
  cpu->SetNZCV(res & 0x80000000, !res, CarrySub(a, b), OverflowSub(a, b));
  cpu->AddCycles_C();
}

void ARMInterpreter::T_ADD_IMM(ARMv5* cpu)
{
  u32 a = cpu->m_reg[(cpu->m_inst >> 8) & 0x7];
  u32 b = cpu->m_inst & 0xFF;
  u32 res = a + b;
  cpu->m_reg[(cpu->m_inst >> 8) & 0x7] = res;
  cpu->SetNZCV(res & 0x80000000, !res, CarryAdd(a, b), OverflowAdd(a, b));
  cpu->AddCycles_C();
}

void ARMInterpreter::T_SUB_IMM(ARMv5* cpu)
{
  u32 a = cpu->m_reg[(cpu->m_inst >> 8) & 0x7];
  u32 b = cpu->m_inst & 0xFF;
  u32 res = a - b;
  cpu->m_reg[(cpu->m_inst >> 8) & 0x7] = res;
  cpu->SetNZCV(res & 0x80000000, !res, CarrySub(a, b), OverflowSub(a, b));
  cpu->AddCycles_C();
}

void ARMInterpreter::T_AND_REG(ARMv5* cpu)
{
  u32 a = cpu->m_reg[cpu->m_inst & 0x7];
  u32 b = cpu->m_reg[(cpu->m_inst >> 3) & 0x7];
  u32 res = a & b;
  cpu->m_reg[cpu->m_inst & 0x7] = res;
  cpu->SetNZ(res & 0x80000000, !res);
  cpu->AddCycles_C();
}

void ARMInterpreter::T_EOR_REG(ARMv5* cpu)
{
  u32 a = cpu->m_reg[cpu->m_inst & 0x7];
  u32 b = cpu->m_reg[(cpu->m_inst >> 3) & 0x7];
  u32 res = a ^ b;
  cpu->m_reg[cpu->m_inst & 0x7] = res;
  cpu->SetNZ(res & 0x80000000, !res);
  cpu->AddCycles_C();
}

void ARMInterpreter::T_LSL_REG(ARMv5* cpu)
{
  u32 a = cpu->m_reg[cpu->m_inst & 0x7];
  u32 b = cpu->m_reg[(cpu->m_inst >> 3) & 0x7] & 0xFF;
  LSL_REG_S(a, b);
  cpu->m_reg[cpu->m_inst & 0x7] = a;
  cpu->SetNZ(a & 0x80000000, !a);
  cpu->AddCycles_CI(1);
}

void ARMInterpreter::T_LSR_REG(ARMv5* cpu)
{
  u32 a = cpu->m_reg[cpu->m_inst & 0x7];
  u32 b = cpu->m_reg[(cpu->m_inst >> 3) & 0x7] & 0xFF;
  LSR_REG_S(a, b);
  cpu->m_reg[cpu->m_inst & 0x7] = a;
  cpu->SetNZ(a & 0x80000000, !a);
  cpu->AddCycles_CI(1);
}

void ARMInterpreter::T_ASR_REG(ARMv5* cpu)
{
  u32 a = cpu->m_reg[cpu->m_inst & 0x7];
  u32 b = cpu->m_reg[(cpu->m_inst >> 3) & 0x7] & 0xFF;
  ASR_REG_S(a, b);
  cpu->m_reg[cpu->m_inst & 0x7] = a;
  cpu->SetNZ(a & 0x80000000, !a);
  cpu->AddCycles_CI(1);
}

void ARMInterpreter::T_ADC_REG(ARMv5* cpu)
{
  u32 a = cpu->m_reg[cpu->m_inst & 0x7];
  u32 b = cpu->m_reg[(cpu->m_inst >> 3) & 0x7];
  u32 res_tmp = a + b;
  u32 carry = (cpu->m_reg_cpsr & 0x20000000 ? 1 : 0);
  u32 res = res_tmp + carry;
  cpu->m_reg[cpu->m_inst & 0x7] = res;
  cpu->SetNZCV(res & 0x80000000, !res, CarryAdd(a, b) || CarryAdd(res_tmp, carry),
               OverflowAdc(a, b, carry));
  cpu->AddCycles_C();
}

void ARMInterpreter::T_SBC_REG(ARMv5* cpu)
{
  u32 a = cpu->m_reg[cpu->m_inst & 0x7];
  u32 b = cpu->m_reg[(cpu->m_inst >> 3) & 0x7];
  u32 res_tmp = a - b;
  u32 carry = (cpu->m_reg_cpsr & 0x20000000 ? 0 : 1);
  u32 res = res_tmp - carry;
  cpu->m_reg[cpu->m_inst & 0x7] = res;
  cpu->SetNZCV(res & 0x80000000, !res, CarrySub(a, b) && CarrySub(res_tmp, carry),
               OverflowSbc(a, b, carry));
  cpu->AddCycles_C();
}

void ARMInterpreter::T_ROR_REG(ARMv5* cpu)
{
  u32 a = cpu->m_reg[cpu->m_inst & 0x7];
  u32 b = cpu->m_reg[(cpu->m_inst >> 3) & 0x7] & 0xFF;
  ROR_REG_S(a, b);
  cpu->m_reg[cpu->m_inst & 0x7] = a;
  cpu->SetNZ(a & 0x80000000, !a);
  cpu->AddCycles_CI(1);
}

void ARMInterpreter::T_TST_REG(ARMv5* cpu)
{
  u32 a = cpu->m_reg[cpu->m_inst & 0x7];
  u32 b = cpu->m_reg[(cpu->m_inst >> 3) & 0x7];
  u32 res = a & b;
  cpu->SetNZ(res & 0x80000000, !res);
  cpu->AddCycles_C();
}

void ARMInterpreter::T_NEG_REG(ARMv5* cpu)
{
  u32 b = cpu->m_reg[(cpu->m_inst >> 3) & 0x7];
  u32 res = -b;
  cpu->m_reg[cpu->m_inst & 0x7] = res;
  cpu->SetNZCV(res & 0x80000000, !res, CarrySub(0, b), OverflowSub(0, b));
  cpu->AddCycles_C();
}

void ARMInterpreter::T_CMP_REG(ARMv5* cpu)
{
  u32 a = cpu->m_reg[cpu->m_inst & 0x7];
  u32 b = cpu->m_reg[(cpu->m_inst >> 3) & 0x7];
  u32 res = a - b;
  cpu->SetNZCV(res & 0x80000000, !res, CarrySub(a, b), OverflowSub(a, b));
  cpu->AddCycles_C();
}

void ARMInterpreter::T_CMN_REG(ARMv5* cpu)
{
  u32 a = cpu->m_reg[cpu->m_inst & 0x7];
  u32 b = cpu->m_reg[(cpu->m_inst >> 3) & 0x7];
  u32 res = a + b;
  cpu->SetNZCV(res & 0x80000000, !res, CarryAdd(a, b), OverflowAdd(a, b));
  cpu->AddCycles_C();
}

void ARMInterpreter::T_ORR_REG(ARMv5* cpu)
{
  u32 a = cpu->m_reg[cpu->m_inst & 0x7];
  u32 b = cpu->m_reg[(cpu->m_inst >> 3) & 0x7];
  u32 res = a | b;
  cpu->m_reg[cpu->m_inst & 0x7] = res;
  cpu->SetNZ(res & 0x80000000, !res);
  cpu->AddCycles_C();
}

void ARMInterpreter::T_MUL_REG(ARMv5* cpu)
{
  u32 a = cpu->m_reg[cpu->m_inst & 0x7];
  u32 b = cpu->m_reg[(cpu->m_inst >> 3) & 0x7];
  u32 res = a * b;
  cpu->m_reg[cpu->m_inst & 0x7] = res;
  cpu->SetNZ(res & 0x80000000, !res);

  s32 cycles = 0;
  if (cpu->Num == 0)
  {
    cycles += 3;
  }
  else
  {
    cpu->SetC(0);  // carry flag destroyed, they say. whatever that means...
    if (a & 0xFF000000)
      cycles += 4;
    else if (a & 0x00FF0000)
      cycles += 3;
    else if (a & 0x0000FF00)
      cycles += 2;
    else
      cycles += 1;
  }
  cpu->AddCycles_CI(cycles);
}

void ARMInterpreter::T_BIC_REG(ARMv5* cpu)
{
  u32 a = cpu->m_reg[cpu->m_inst & 0x7];
  u32 b = cpu->m_reg[(cpu->m_inst >> 3) & 0x7];
  u32 res = a & ~b;
  cpu->m_reg[cpu->m_inst & 0x7] = res;
  cpu->SetNZ(res & 0x80000000, !res);
  cpu->AddCycles_C();
}

void ARMInterpreter::T_MVN_REG(ARMv5* cpu)
{
  u32 b = cpu->m_reg[(cpu->m_inst >> 3) & 0x7];
  u32 res = ~b;
  cpu->m_reg[cpu->m_inst & 0x7] = res;
  cpu->SetNZ(res & 0x80000000, !res);
  cpu->AddCycles_C();
}

// TODO: check those when MSBs and MSBd are cleared
// GBAtek says it's not allowed, but it works atleast on the ARM9

void ARMInterpreter::T_ADD_HIREG(ARMv5* cpu)
{
  u32 rd = (cpu->m_inst & 0x7) | ((cpu->m_inst >> 4) & 0x8);
  u32 rs = (cpu->m_inst >> 3) & 0xF;

  u32 a = cpu->m_reg[rd];
  u32 b = cpu->m_reg[rs];

  cpu->AddCycles_C();

  if (rd == 15)
  {
    cpu->JumpTo((a + b) | 1);
  }
  else
  {
    cpu->m_reg[rd] = a + b;
  }
}

void ARMInterpreter::T_CMP_HIREG(ARMv5* cpu)
{
  u32 rd = (cpu->m_inst & 0x7) | ((cpu->m_inst >> 4) & 0x8);
  u32 rs = (cpu->m_inst >> 3) & 0xF;

  u32 a = cpu->m_reg[rd];
  u32 b = cpu->m_reg[rs];
  u32 res = a - b;

  cpu->SetNZCV(res & 0x80000000, !res, CarrySub(a, b), OverflowSub(a, b));
  cpu->AddCycles_C();
}

void ARMInterpreter::T_MOV_HIREG(ARMv5* cpu)
{
  u32 rd = (cpu->m_inst & 0x7) | ((cpu->m_inst >> 4) & 0x8);
  u32 rs = (cpu->m_inst >> 3) & 0xF;

  cpu->AddCycles_C();

  if (rd == 15)
  {
    cpu->JumpTo(cpu->m_reg[rs] | 1);
  }
  else
  {
    cpu->m_reg[rd] = cpu->m_reg[rs];
  }

#if 0
  // nocash-style debugging hook
  if ((cpu->m_inst & 0xFFFF) == 0x46E4 &&      // mov r12, r12
      (cpu->m_next_inst[0] & 0xF800) == 0xE000 &&  // branch
      (cpu->m_next_inst[1] & 0xFFFF) == 0x6464)
  {
    // GBATek says the two bytes after the 2nd ID are _reserved_ for flags
    // but since they serve no purpose ATTOW, we can skip them
    u32 addr = cpu->m_reg[15] + 4;  // Skip 2nd ID and flags
    // TODO: Pass flags to NocashPrint
    cpu->NDS.NocashPrint(cpu->Num, addr);
  }
#endif
}

void ARMInterpreter::T_ADD_PCREL(ARMv5* cpu)
{
  u32 val = cpu->m_reg[15] & ~2;
  val += ((cpu->m_inst & 0xFF) << 2);
  cpu->m_reg[(cpu->m_inst >> 8) & 0x7] = val;
  cpu->AddCycles_C();
}

void ARMInterpreter::T_ADD_SPREL(ARMv5* cpu)
{
  u32 val = cpu->m_reg[13];
  val += ((cpu->m_inst & 0xFF) << 2);
  cpu->m_reg[(cpu->m_inst >> 8) & 0x7] = val;
  cpu->AddCycles_C();
}

void ARMInterpreter::T_ADD_SP(ARMv5* cpu)
{
  u32 val = cpu->m_reg[13];
  if (cpu->m_inst & (1 << 7))
    val -= ((cpu->m_inst & 0x7F) << 2);
  else
    val += ((cpu->m_inst & 0x7F) << 2);
  cpu->m_reg[13] = val;
  cpu->AddCycles_C();
}

}  // namespace IOS::LLE
