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

#include "Core/IOS_LLE/ARM.h"
#include "Core/IOS_LLE/ARMInterpreter.h"

namespace IOS::LLE
{

void ARMInterpreter::A_B(ARMv5* cpu)
{
  s32 offset = (s32)(cpu->m_inst << 8) >> 6;
  cpu->JumpTo(cpu->m_reg[15] + offset);
}

void ARMInterpreter::A_BL(ARMv5* cpu)
{
  s32 offset = (s32)(cpu->m_inst << 8) >> 6;
  cpu->m_reg[14] = cpu->m_reg[15] - 4;
  cpu->JumpTo(cpu->m_reg[15] + offset);
}

void ARMInterpreter::A_BLX_IMM(ARMv5* cpu)
{
  s32 offset = (s32)(cpu->m_inst << 8) >> 6;
  if (cpu->m_inst & 0x01000000)
    offset += 2;
  cpu->m_reg[14] = cpu->m_reg[15] - 4;
  cpu->JumpTo(cpu->m_reg[15] + offset + 1);
}

void ARMInterpreter::A_BX(ARMv5* cpu)
{
  cpu->JumpTo(cpu->m_reg[cpu->m_inst & 0xF]);
}

void ARMInterpreter::A_BLX_REG(ARMv5* cpu)
{
  u32 lr = cpu->m_reg[15] - 4;
  cpu->JumpTo(cpu->m_reg[cpu->m_inst & 0xF]);
  cpu->m_reg[14] = lr;
}

void ARMInterpreter::T_BCOND(ARMv5* cpu)
{
  if (cpu->CheckCondition((cpu->m_inst >> 8) & 0xF))
  {
    s32 offset = (s32)(cpu->m_inst << 24) >> 23;
    cpu->JumpTo(cpu->m_reg[15] + offset + 1);
  }
  else
    cpu->AddCycles_C();
}

void ARMInterpreter::T_BX(ARMv5* cpu)
{
  cpu->JumpTo(cpu->m_reg[(cpu->m_inst >> 3) & 0xF]);
}

void ARMInterpreter::T_BLX_REG(ARMv5* cpu)
{
  u32 lr = cpu->m_reg[15] - 1;
  cpu->JumpTo(cpu->m_reg[(cpu->m_inst >> 3) & 0xF]);
  cpu->m_reg[14] = lr;
}

void ARMInterpreter::T_B(ARMv5* cpu)
{
  s32 offset = (s32)((cpu->m_inst & 0x7FF) << 21) >> 20;
  cpu->JumpTo(cpu->m_reg[15] + offset + 1);
}

void ARMInterpreter::T_BL_LONG_1(ARMv5* cpu)
{
  s32 offset = (s32)((cpu->m_inst & 0x7FF) << 21) >> 9;
  cpu->m_reg[14] = cpu->m_reg[15] + offset;
  cpu->AddCycles_C();
}

void ARMInterpreter::T_BL_LONG_2(ARMv5* cpu)
{
  s32 offset = (cpu->m_inst & 0x7FF) << 1;
  u32 pc = cpu->m_reg[14] + offset;
  cpu->m_reg[14] = (cpu->m_reg[15] - 2) | 1;

  if (cpu->m_inst & (1 << 12))
    pc |= 1;

  cpu->JumpTo(pc);
}

}  // namespace IOS::LLE
