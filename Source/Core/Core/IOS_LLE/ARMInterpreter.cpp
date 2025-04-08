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

#include "ARMInterpreter.h"
#include <stdio.h>
#include "Common/Logging/Log.h"

#ifdef GDBSTUB_ENABLED
#include "debug/GdbStub.h"
#endif

namespace IOS::LLE
{

void ARMInterpreter::A_UNK(ARMv5* cpu)
{
  if ((cpu->m_inst & 0xE6000010) == 0xE6000010)
  {
    // IOS Syscall
    cpu->LogSyscall(cpu->m_inst);
  }
  else
  {
    WARN_LOG_FMT(IOS_LLE, "undefined ARM9 instruction {:08x} @ {:08x}\n", cpu->m_inst,
                 cpu->m_reg[15] - 8);
  }

#ifdef GDBSTUB_ENABLED
  cpu->GdbStub.Enter(cpu->GdbStub.IsConnected(), Gdb::TgtStatus::FaultInsn, cpu->R[15] - 8);
#endif
  // for (int i = 0; i < 16; i++) printf("R{}: {:08x}\n", i, cpu->R[i]);
  // NDS::Halt();
  u32 oldcpsr = cpu->m_reg_cpsr;
  cpu->m_reg_cpsr &= ~0xBF;
  cpu->m_reg_cpsr |= 0x9B;
  cpu->UpdateMode(oldcpsr, cpu->m_reg_cpsr);

  cpu->m_und_reg[2] = oldcpsr;
  cpu->m_reg[14] = cpu->m_reg[15] - 4;
  cpu->JumpTo(cpu->m_exception_base + 0x04);
}

void ARMInterpreter::T_UNK(ARMv5* cpu)
{
  WARN_LOG_FMT(IOS_LLE, "undefined THUMB9 instruction {:04x} @ {:08x}\n", cpu->m_inst,
               cpu->m_reg[15] - 4);
#ifdef GDBSTUB_ENABLED
  cpu->GdbStub.Enter(cpu->GdbStub.IsConnected(), Gdb::TgtStatus::FaultInsn, cpu->R[15] - 4);
#endif
  // NDS::Halt();
  u32 oldcpsr = cpu->m_reg_cpsr;
  cpu->m_reg_cpsr &= ~0xBF;
  cpu->m_reg_cpsr |= 0x9B;
  cpu->UpdateMode(oldcpsr, cpu->m_reg_cpsr);

  cpu->m_und_reg[2] = oldcpsr;
  cpu->m_reg[14] = cpu->m_reg[15] - 2;
  cpu->JumpTo(cpu->m_exception_base + 0x04);
}

void ARMInterpreter::A_MSR_IMM(ARMv5* cpu)
{
  u32* psr;
  if (cpu->m_inst & (1 << 22))
  {
    switch (cpu->m_reg_cpsr & 0x1F)
    {
    case 0x11:
      psr = &cpu->m_fiq_reg[7];
      break;
    case 0x12:
      psr = &cpu->m_irq_reg[2];
      break;
    case 0x13:
      psr = &cpu->m_svc_reg[2];
      break;
    case 0x14:
    case 0x15:
    case 0x16:
    case 0x17:
      psr = &cpu->m_abt_reg[2];
      break;
    case 0x18:
    case 0x19:
    case 0x1A:
    case 0x1B:
      psr = &cpu->m_und_reg[2];
      break;
    default:
      cpu->AddCycles_C();
      return;
    }
  }
  else
    psr = &cpu->m_reg_cpsr;

  u32 oldpsr = *psr;

  u32 mask = 0;
  if (cpu->m_inst & (1 << 16))
    mask |= 0x000000FF;
  if (cpu->m_inst & (1 << 17))
    mask |= 0x0000FF00;
  if (cpu->m_inst & (1 << 18))
    mask |= 0x00FF0000;
  if (cpu->m_inst & (1 << 19))
    mask |= 0xFF000000;

  if (!(cpu->m_inst & (1 << 22)))
    mask &= 0xFFFFFFDF;

  if ((cpu->m_reg_cpsr & 0x1F) == 0x10)
    mask &= 0xFFFFFF00;

  u32 val = ROR((cpu->m_inst & 0xFF), ((cpu->m_inst >> 7) & 0x1E));

  // bit4 is forced to 1
  val |= 0x00000010;

  *psr &= ~mask;
  *psr |= (val & mask);

  if (!(cpu->m_inst & (1 << 22)))
    cpu->UpdateMode(oldpsr, cpu->m_reg_cpsr);

  cpu->AddCycles_C();
}

void ARMInterpreter::A_MSR_REG(ARMv5* cpu)
{
  u32* psr;
  if (cpu->m_inst & (1 << 22))
  {
    switch (cpu->m_reg_cpsr & 0x1F)
    {
    case 0x11:
      psr = &cpu->m_fiq_reg[7];
      break;
    case 0x12:
      psr = &cpu->m_irq_reg[2];
      break;
    case 0x13:
      psr = &cpu->m_svc_reg[2];
      break;
    case 0x14:
    case 0x15:
    case 0x16:
    case 0x17:
      psr = &cpu->m_abt_reg[2];
      break;
    case 0x18:
    case 0x19:
    case 0x1A:
    case 0x1B:
      psr = &cpu->m_und_reg[2];
      break;
    default:
      cpu->AddCycles_C();
      return;
    }
  }
  else
    psr = &cpu->m_reg_cpsr;

  u32 oldpsr = *psr;

  u32 mask = 0;
  if (cpu->m_inst & (1 << 16))
    mask |= 0x000000FF;
  if (cpu->m_inst & (1 << 17))
    mask |= 0x0000FF00;
  if (cpu->m_inst & (1 << 18))
    mask |= 0x00FF0000;
  if (cpu->m_inst & (1 << 19))
    mask |= 0xFF000000;

  if (!(cpu->m_inst & (1 << 22)))
    mask &= 0xFFFFFFDF;

  if ((cpu->m_reg_cpsr & 0x1F) == 0x10)
    mask &= 0xFFFFFF00;

  u32 val = cpu->m_reg[cpu->m_inst & 0xF];

  // bit4 is forced to 1
  val |= 0x00000010;

  *psr &= ~mask;
  *psr |= (val & mask);

  if (!(cpu->m_inst & (1 << 22)))
    cpu->UpdateMode(oldpsr, cpu->m_reg_cpsr);

  cpu->AddCycles_C();
}

void ARMInterpreter::A_MRS(ARMv5* cpu)
{
  u32 psr;
  if (cpu->m_inst & (1 << 22))
  {
    switch (cpu->m_reg_cpsr & 0x1F)
    {
    case 0x11:
      psr = cpu->m_fiq_reg[7];
      break;
    case 0x12:
      psr = cpu->m_irq_reg[2];
      break;
    case 0x13:
      psr = cpu->m_svc_reg[2];
      break;
    case 0x14:
    case 0x15:
    case 0x16:
    case 0x17:
      psr = cpu->m_abt_reg[2];
      break;
    case 0x18:
    case 0x19:
    case 0x1A:
    case 0x1B:
      psr = cpu->m_und_reg[2];
      break;
    default:
      psr = cpu->m_reg_cpsr;
      break;
    }
  }
  else
    psr = cpu->m_reg_cpsr;

  cpu->m_reg[(cpu->m_inst >> 12) & 0xF] = psr;
  cpu->AddCycles_C();
}

void ARMInterpreter::A_MCR(ARMv5* cpu)
{
  if ((cpu->m_reg_cpsr & 0x1F) == 0x10)
    return A_UNK(cpu);

  u32 cp = (cpu->m_inst >> 8) & 0xF;
  // u32 op = (cpu->m_inst >> 21) & 0x7;
  u32 cn = (cpu->m_inst >> 16) & 0xF;
  u32 cm = cpu->m_inst & 0xF;
  u32 cpinfo = (cpu->m_inst >> 5) & 0x7;

  // mcr xxx, 0, r0, cn, cm, cpinfo

  if (cp == 15)
  {
    ((ARMv5*)cpu)->CP15Write((cn << 8) | (cm << 4) | cpinfo, cpu->m_reg[(cpu->m_inst >> 12) & 0xF]);
  }
  else
  {
    WARN_LOG_FMT(IOS_LLE, "bad MCR opcode p{},{},{},{} on ARM9\n", cp, cn, cm, cpinfo);
    return A_UNK(cpu);  // TODO: check what kind of exception it really is
  }

  cpu->AddCycles_CI(1 + 1);  // TODO: checkme
}

void ARMInterpreter::A_MRC(ARMv5* cpu)
{
  if ((cpu->m_reg_cpsr & 0x1F) == 0x10)
    return A_UNK(cpu);

  u32 cp = (cpu->m_inst >> 8) & 0xF;
  // u32 op = (cpu->m_inst >> 21) & 0x7;
  u32 cn = (cpu->m_inst >> 16) & 0xF;
  u32 cm = cpu->m_inst & 0xF;
  u32 cpinfo = (cpu->m_inst >> 5) & 0x7;

  if (cp == 15)
  {
    u32 id = (cn << 8) | (cm << 4) | cpinfo;
    u32 dest_reg = (cpu->m_inst >> 12) & 0xF;

    if (id == 0x7A3 && dest_reg == 15)
    {
      // Special case for test and clean data cache, it does not update the PC
      // Set beq
      cpu->m_reg_cpsr &= ~0xF8000000;
      cpu->m_reg_cpsr |= 0x40000000;
    }
    else
    {
      cpu->m_reg[dest_reg] = ((ARMv5*)cpu)->CP15Read(id);
    }
  }
  else
  {
    WARN_LOG_FMT(IOS_LLE, "bad MRC opcode p{},{},{},{} on ARM9\n", cp, cn, cm, cpinfo);
    return A_UNK(cpu);  // TODO: check what kind of exception it really is
  }

  cpu->AddCycles_CI(2 + 1);  // TODO: checkme
}

void ARMInterpreter::A_SVC(ARMv5* cpu)
{
  // INFO_LOG_FMT(IOS_LLE, "SVC instruction {:08x} @ {:08x}\n", cpu->m_inst, cpu->R[15] - 8);
  if (cpu->m_reg[0] == 4)
  {
    cpu->SVCWrite0(cpu->m_reg[1]);
  }

  u32 oldcpsr = cpu->m_reg_cpsr;
  cpu->m_reg_cpsr &= ~0xBF;
  cpu->m_reg_cpsr |= 0x93;
  cpu->UpdateMode(oldcpsr, cpu->m_reg_cpsr);

  cpu->m_svc_reg[2] = oldcpsr;
  cpu->m_reg[14] = cpu->m_reg[15] - 4;
  cpu->JumpTo(cpu->m_exception_base + 0x08);
}

void ARMInterpreter::T_SVC(ARMv5* cpu)
{
  // INFO_LOG_FMT(IOS_LLE, "SVC instruction {:04x} @ {:08x}\n", cpu->m_inst, cpu->R[15] - 4 + 1);
  if (cpu->m_reg[0] == 4)
  {
    cpu->SVCWrite0(cpu->m_reg[1]);
  }

  u32 oldcpsr = cpu->m_reg_cpsr;
  cpu->m_reg_cpsr &= ~0xBF;
  cpu->m_reg_cpsr |= 0x93;
  cpu->UpdateMode(oldcpsr, cpu->m_reg_cpsr);

  cpu->m_svc_reg[2] = oldcpsr;
  cpu->m_reg[14] = cpu->m_reg[15] - 2;
  cpu->JumpTo(cpu->m_exception_base + 0x08);
}

#define INSTRFUNC_PROTO(x) void (*ARMInterpreter::x)(ARMv5 * cpu)
#include "ARM_InstrTable.h"
#undef INSTRFUNC_PROTO

}  // namespace IOS::LLE
