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

#include "ARM.h"
#include <algorithm>
#include <assert.h>
#include <stdio.h>
#include "ARMInterpreter.h"
#include "Common/ChunkFile.h"
#include "Common/Logging/Log.h"
#include "Common/MsgHandler.h"
#include "Core/HW/CPU.h"
#include "Core/HW/MMIO.h"
#include "Core/HW/Memmap.h"
#include "Core/HW/WII_IPC.h"
#include "Core/Host.h"
#include "Core/PowerPC/PowerPC.h"
// #include "ARMJIT.h"
// #include "ARMJIT_Memory.h"

namespace IOS::LLE
{

#ifdef GDBSTUB_ENABLED
void ARMv5::GdbCheckA()
{
  if (!IsSingleStep && !BreakReq)
  {  // check if eg. break signal is incoming etc.
    Gdb::StubState st = GdbStub.Enter(false, Gdb::TgtStatus::NoEvent, ~(u32)0u, BreakOnStartup);
    BreakOnStartup = false;
    IsSingleStep = st == Gdb::StubState::Step;
    BreakReq = st == Gdb::StubState::Attach || st == Gdb::StubState::Break;
  }
}
void ARMv5::GdbCheckB()
{
  if (IsSingleStep || BreakReq)
  {  // use else here or we single-step the same insn twice in gdb
    u32 pc_real = R[15] - ((CPSR & 0x20) ? 2 : 4);
    Gdb::StubState st = GdbStub.Enter(true, Gdb::TgtStatus::SingleStep, pc_real);
    IsSingleStep = st == Gdb::StubState::Step;
    BreakReq = st == Gdb::StubState::Attach || st == Gdb::StubState::Break;
  }
}
void ARMv5::GdbCheckC()
{
  u32 pc_real = R[15] - ((CPSR & 0x20) ? 2 : 4);
  Gdb::StubState st = GdbStub.CheckBkpt(pc_real, true, true);
  if (st != Gdb::StubState::CheckNoHit)
  {
    IsSingleStep = st == Gdb::StubState::Step;
    BreakReq = st == Gdb::StubState::Attach || st == Gdb::StubState::Break;
  }
  else
    GdbCheckB();
}
#else
void ARMv5::GdbCheckA()
{
}
void ARMv5::GdbCheckB()
{
}
void ARMv5::GdbCheckC()
{
}
#endif

// instruction timing notes
//
// * simple instruction: 1S (code)
// * LDR: 1N+1N+1I (code/data/internal)
// * STR: 1N+1N (code/data)
// * LDM: 1N+1N+(n-1)S+1I
// * STM: 1N+1N+(n-1)S
// * MUL/etc: 1N+xI (code/internal)
// * branch: 1N+1S (code/code) (pipeline refill)
//
// MUL/MLA seems to take 1I on ARM9

const u32 ARMv5::ConditionTable[16] = {
    0xF0F0,  // EQ
    0x0F0F,  // NE
    0xCCCC,  // CS
    0x3333,  // CC
    0xFF00,  // MI
    0x00FF,  // PL
    0xAAAA,  // VS
    0x5555,  // VC
    0x0C0C,  // HI
    0xF3F3,  // LS
    0xAA55,  // GE
    0x55AA,  // LT
    0x0A05,  // GT
    0xF5FA,  // LE
    0xFFFF,  // AL
    0x0000   // NE
};

ARMv5::ARMv5(Core::System& system, Memory::MemoryManager& memory, std::optional<GDBArgs> gdb,
             bool jit)
    : m_system(system), m_memory(memory)
#ifdef GDBSTUB_ENABLED
      ,
      GdbStub(this), BreakOnStartup(false),
#endif
{
  SetGdbArgs(jit ? std::nullopt : gdb);
}

ARMv5::~ARMv5()
{
}

void ARMv5::SetGdbArgs(std::optional<GDBArgs> gdb)
{
#ifdef GDBSTUB_ENABLED
  GdbStub.Close();
  if (gdb)
  {
    int port = Num ? gdb->PortARM7 : gdb->PortARM9;
    GdbStub.Init(port);
    BreakOnStartup = Num ? gdb->ARM7BreakOnStartup : gdb->ARM9BreakOnStartup;
  }
  IsSingleStep = false;
#endif
}

void ARMv5::Reset()
{
  m_cycles = 0;
  m_is_halted = 0;

  m_is_irq = 0;

  for (int i = 0; i < 16; i++)
    m_reg[i] = 0;

  m_reg_cpsr = 0x000000D3;

  for (int i = 0; i < 7; i++)
    m_fiq_reg[i] = 0;
  for (int i = 0; i < 2; i++)
  {
    m_svc_reg[i] = 0;
    m_abt_reg[i] = 0;
    m_irq_reg[i] = 0;
    m_und_reg[i] = 0;
  }

  m_fiq_reg[7] = 0x00000010;
  m_svc_reg[2] = 0x00000010;
  m_abt_reg[2] = 0x00000010;
  m_irq_reg[2] = 0x00000010;
  m_und_reg[2] = 0x00000010;

  m_exception_base = 0xFFFF0000;

  CP15Reset();

  // CodeMem.Mem = NULL;

#ifdef JIT_ENABLED
  FastBlockLookup = NULL;
  FastBlockLookupStart = 0;
  FastBlockLookupSize = 0;
#endif

#ifdef GDBSTUB_ENABLED
  IsSingleStep = false;
  BreakReq = false;
#endif

  // zorp
  JumpTo(m_exception_base);
}

#if 0

void ARMv5::DoSavestate(Savestate* file)
{
    file->Section((char*)(Num ? "ARM7" : "ARM9"));

    file->Var32((u32*)&Cycles);
    //file->Var32((u32*)&CyclesToRun);

    // hack to make save states compatible
    u32 halted = Halted;
    file->Var32(&halted);
    Halted = halted;

    file->VarArray(R, 16*sizeof(u32));
    file->Var32(&CPSR);
    file->VarArray(R_FIQ, 8*sizeof(u32));
    file->VarArray(R_SVC, 3*sizeof(u32));
    file->VarArray(R_ABT, 3*sizeof(u32));
    file->VarArray(R_IRQ, 3*sizeof(u32));
    file->VarArray(R_UND, 3*sizeof(u32));
    file->Var32(&m_inst);
#ifdef JIT_ENABLED
    if (file->Saving && NDS.IsJITEnabled())
    {
        // hack, the JIT doesn't really pipeline
        // but we still want JIT save states to be
        // loaded while running the interpreter
        FillPipeline();
    }
#endif
    file->VarArray(m_next_inst, 2*sizeof(u32));

    file->Var32(&ExceptionBase);

    if (!file->Saving)
    {
        CPSR |= 0x00000010;
        R_FIQ[7] |= 0x00000010;
        R_SVC[2] |= 0x00000010;
        R_ABT[2] |= 0x00000010;
        R_IRQ[2] |= 0x00000010;
        R_UND[2] |= 0x00000010;

        if (!Num)
        {
            SetupCodeMem(R[15]); // should fix it
            ((ARMv5*)this)->RegionCodeCycles = ((ARMv5*)this)->MemTimings[R[15] >> 12][0];
        }
        else
        {
            CodeRegion = R[15] >> 24;
            CodeCycles = R[15] >> 15; // cheato
        }
    }
}

void ARMv5::DoSavestate(Savestate* file)
{
    ARMv5::DoSavestate(file);
    CP15DoSavestate(file);
}

#endif

void ARMv5::JumpTo(u32 addr, bool restorecpsr)
{
  if (restorecpsr)
  {
    RestoreCPSR();

    if (m_reg_cpsr & 0x20)
      addr |= 0x1;
    else
      addr &= ~0x1;
  }

  // aging cart debug crap
  // if (addr == 0x0201764C) printf("capture test %d: R1={:08x}\n", R[6], R[1]);
  // if (addr == 0x020175D8) printf("capture test %d: res={:08x}\n", R[6], R[0]);

  // u32 oldregion = R[15] >> 24;
  // u32 newregion = addr >> 24;

  // RegionCodeCycles = MemTimings[addr >> 12][0];

  if (addr & 0x1)
  {
    addr &= ~0x1;
    m_reg[15] = addr + 2;

    // if (newregion != oldregion)
    //   SetupCodeMem(addr);

    // two-opcodes-at-once fetch
    // doesn't matter if we put garbage in the MSbs there
    if (addr & 0x2)
    {
      m_next_inst[0] = CodeRead32(addr - 2, true);
      m_cycles += m_code_cycles;
      m_next_inst[1] = CodeRead32(addr + 2, false);
      m_cycles += m_code_cycles;
    }
    else
    {
      u32 instr = CodeRead32(addr, true);
      m_next_inst[0] = instr >> 16;
      m_next_inst[1] = instr << 16;
      m_cycles += m_code_cycles;
    }

    m_reg_cpsr |= 0x20;
  }
  else
  {
    addr &= ~0x3;
    m_reg[15] = addr + 4;

    // if (newregion != oldregion)
    //   SetupCodeMem(addr);

    m_next_inst[0] = CodeRead32(addr, true);
    m_cycles += m_code_cycles;
    m_next_inst[1] = CodeRead32(addr + 4, false);
    m_cycles += m_code_cycles;

    m_reg_cpsr &= ~0x20;
  }

  u32 addrCopy = addr;
  if (!TranslateAddress(addrCopy, false))
  {
    m_reg_far = addr;
    PrefetchAbort();
    return;
  }
}

void ARMv5::RestoreCPSR()
{
  u32 oldcpsr = m_reg_cpsr;

  switch (m_reg_cpsr & 0x1F)
  {
  case 0x11:
    m_reg_cpsr = m_fiq_reg[7];
    break;

  case 0x12:
    m_reg_cpsr = m_irq_reg[2];
    break;

  case 0x13:
    m_reg_cpsr = m_svc_reg[2];
    break;

  case 0x14:
  case 0x15:
  case 0x16:
  case 0x17:
    m_reg_cpsr = m_abt_reg[2];
    break;

  case 0x18:
  case 0x19:
  case 0x1A:
  case 0x1B:
    m_reg_cpsr = m_und_reg[2];
    break;

  default:
    WARN_LOG_FMT(IOS_LLE, "!! attempt to restore CPSR under bad mode {:02x}, {:08x}\n",
                 m_reg_cpsr & 0x1F, m_reg[15]);
    break;
  }

  m_reg_cpsr |= 0x00000010;

  UpdateMode(oldcpsr, m_reg_cpsr);
}

void ARMv5::UpdateMode(u32 oldmode, u32 newmode, bool phony)
{
  if ((oldmode & 0x1F) == (newmode & 0x1F))
    return;

  switch (oldmode & 0x1F)
  {
  case 0x11:
    std::swap(m_reg[8], m_fiq_reg[0]);
    std::swap(m_reg[9], m_fiq_reg[1]);
    std::swap(m_reg[10], m_fiq_reg[2]);
    std::swap(m_reg[11], m_fiq_reg[3]);
    std::swap(m_reg[12], m_fiq_reg[4]);
    std::swap(m_reg[13], m_fiq_reg[5]);
    std::swap(m_reg[14], m_fiq_reg[6]);
    break;

  case 0x12:
    std::swap(m_reg[13], m_irq_reg[0]);
    std::swap(m_reg[14], m_irq_reg[1]);
    break;

  case 0x13:
    std::swap(m_reg[13], m_svc_reg[0]);
    std::swap(m_reg[14], m_svc_reg[1]);
    break;

  case 0x17:
    std::swap(m_reg[13], m_abt_reg[0]);
    std::swap(m_reg[14], m_abt_reg[1]);
    break;

  case 0x1B:
    std::swap(m_reg[13], m_und_reg[0]);
    std::swap(m_reg[14], m_und_reg[1]);
    break;
  }

  switch (newmode & 0x1F)
  {
  case 0x11:
    std::swap(m_reg[8], m_fiq_reg[0]);
    std::swap(m_reg[9], m_fiq_reg[1]);
    std::swap(m_reg[10], m_fiq_reg[2]);
    std::swap(m_reg[11], m_fiq_reg[3]);
    std::swap(m_reg[12], m_fiq_reg[4]);
    std::swap(m_reg[13], m_fiq_reg[5]);
    std::swap(m_reg[14], m_fiq_reg[6]);
    break;

  case 0x12:
    std::swap(m_reg[13], m_irq_reg[0]);
    std::swap(m_reg[14], m_irq_reg[1]);
    break;

  case 0x13:
    std::swap(m_reg[13], m_svc_reg[0]);
    std::swap(m_reg[14], m_svc_reg[1]);
    break;

  case 0x17:
    std::swap(m_reg[13], m_abt_reg[0]);
    std::swap(m_reg[14], m_abt_reg[1]);
    break;

  case 0x1B:
    std::swap(m_reg[13], m_und_reg[0]);
    std::swap(m_reg[14], m_und_reg[1]);
    break;
  }
}

void ARMv5::TriggerIRQ()
{
  if (m_reg_cpsr & 0x80)
    return;

  u32 oldcpsr = m_reg_cpsr;
  m_reg_cpsr &= ~0xFF;
  m_reg_cpsr |= 0xD2;
  UpdateMode(oldcpsr, m_reg_cpsr);

  m_irq_reg[2] = oldcpsr;
  m_reg[14] = m_reg[15] + (oldcpsr & 0x20 ? 2 : 0);
  JumpTo(m_exception_base + 0x18);
}

void ARMv5::PrefetchAbort()
{
  WARN_LOG_FMT(IOS_LLE, "ARM9: Prefetch abort ({:08x})\n", m_reg[15]);

  u32 oldcpsr = m_reg_cpsr;
  m_reg_cpsr &= ~0xBF;
  m_reg_cpsr |= 0x97;
  UpdateMode(oldcpsr, m_reg_cpsr);

  // this shouldn't happen, but if it does, we're stuck in some nasty endless loop
  // so better take care of it
  u32 exception_base_copy = m_exception_base + 0x0C;
  if (!TranslateAddress(exception_base_copy, false))
  {
    PanicAlertFmt("!!!!! ARM9 EXCEPTION REGION NOT EXECUTABLE. THIS IS VERY BAD!!\n");
    // NDS.Stop(Platform::StopReason::BadExceptionRegion);
    return;
  }

  m_abt_reg[2] = oldcpsr;
  m_reg[14] = m_reg[15] + (oldcpsr & 0x20 ? 2 : 0);
  JumpTo(m_exception_base + 0x0C);
}

void ARMv5::DataAbort()
{
  WARN_LOG_FMT(IOS_LLE, "ARM9: Data abort ({:08x})\n", m_reg[15]);

  u32 oldcpsr = m_reg_cpsr;
  m_reg_cpsr &= ~0xBF;
  m_reg_cpsr |= 0x97;
  UpdateMode(oldcpsr, m_reg_cpsr);

  m_abt_reg[2] = oldcpsr;
  m_reg[14] = m_reg[15] + (oldcpsr & 0x20 ? 4 : 0);
  JumpTo(m_exception_base + 0x10);
}

void ARMv5::CheckGdbIncoming()
{
  GdbCheckA();
}

u32 ARMv5::GetPC()
{
  if (m_reg_cpsr & 0x20)
    return (m_reg[15] - 2) + 1;
  else
    return m_reg[15] - 4;
}

void ARMv5::Shutdown()
{
}

void ARMv5::DoState(PointerWrap& p)
{
}

void ARMv5::SingleStepInterpreter(bool skip_bp)
{
#if 0
  if (R[15] > 0xFFFF0000)
  {
    Common::SleepCurrentThread(500);
  }
#endif

  if (m_reg_cpsr & 0x20)  // THUMB
  {
    // if constexpr (mode == CPUExecuteMode::InterpreterGDB)
    //   GdbCheckC();

    u32 pcAddr = m_reg[15] - 2;

    if (!skip_bp)
    {
      auto& breakpoints = m_system.GetPowerPC().GetBreakPoints();
      const TBreakPoint* bp = breakpoints.GetBreakpoint(pcAddr);
      if (bp && bp->is_enabled && EvaluateCondition(m_system, bp->condition))
      {
        if (bp->log_on_hit)
        {
          NOTICE_LOG_FMT(MEMMAP,
                         "IOS_LLE THUMB BP {:08x} ({:08x} {:08x} {:08x} {:08x} {:08x} {:08x} "
                         "{:08x} {:08x} {:08x} "
                         "{:08x}) LR={:08x}",
                         pcAddr, m_reg[0], m_reg[1], m_reg[2], m_reg[3], m_reg[4], m_reg[5],
                         m_reg[6], m_reg[7], m_reg[8], m_reg[9], m_reg[14]);
        }

        if (bp->break_on_hit)
        {
          m_system.GetCPU(m_cpu_number).Break();
          return;
        }
      }
    }

    // prefetch
    m_reg[15] += 2;
    m_inst = m_next_inst[0];
    m_next_inst[0] = m_next_inst[1] >> 16;
    if (m_reg[15] & 0x2)
    {
      m_next_inst[1] <<= 16;
      m_code_cycles = 0;
    }
    else
      m_next_inst[1] = CodeRead32(m_reg[15], false);

    // INFO_LOG_FMT(IOS_LLE, "THUMB9 instruction {:04x} @ {:08x}\n", m_inst & 0xFFFF, pcAddr);

    // actually execute
    u32 icode = (m_inst >> 6) & 0x3FF;
    ARMInterpreter::THUMBInstrTable[icode](this);
  }
  else
  {
    // if constexpr (mode == CPUExecuteMode::InterpreterGDB)
    //   GdbCheckC();

    u32 pcAddr = m_reg[15] - 4;

    if (!skip_bp)
    {
      auto& breakpoints = m_system.GetPowerPC().GetBreakPoints();
      const TBreakPoint* bp = breakpoints.GetBreakpoint(pcAddr);
      if (bp && bp->is_enabled && EvaluateCondition(m_system, bp->condition))
      {
        if (bp->log_on_hit)
        {
          NOTICE_LOG_FMT(MEMMAP,
                         "IOS_LLE ARM BP {:08x} ({:08x} {:08x} {:08x} {:08x} {:08x} {:08x} "
                         "{:08x} {:08x} {:08x} "
                         "{:08x}) LR={:08x}",
                         pcAddr, m_reg[0], m_reg[1], m_reg[2], m_reg[3], m_reg[4], m_reg[5],
                         m_reg[6], m_reg[7], m_reg[8], m_reg[9], m_reg[14]);
        }

        if (bp->break_on_hit)
        {
          m_system.GetCPU(m_cpu_number).Break();
          return;
        }
      }
    }

    // prefetch
    m_reg[15] += 4;
    m_inst = m_next_inst[0];
    m_next_inst[0] = m_next_inst[1];
    m_next_inst[1] = CodeRead32(m_reg[15], false);

    // INFO_LOG_FMT(IOS_LLE, "ARM9 instruction {:08x} @ {:08x}\n", m_inst, pcAddr);

    // actually execute
    if (CheckCondition(m_inst >> 28))
    {
      u32 icode = ((m_inst >> 4) & 0xF) | ((m_inst >> 16) & 0xFF0);
      ARMInterpreter::ARMInstrTable[icode](this);
    }
    else if ((m_inst & 0xFE000000) == 0xFA000000)
    {
      ARMInterpreter::A_BLX_IMM(this);
    }
    else
    {
      AddCycles_C();
    }
  }

  // TODO optimize this shit!!!
  // if (Halted)
  // {
  //     if (Halted == 1 && NDS.ARM9Timestamp < NDS.ARM9Target)
  //     {
  //         NDS.ARM9Timestamp = NDS.ARM9Target;
  //     }
  //     break;
  // }
  /*if (NDS::IF[0] & NDS::IE[0])
  {
      if (NDS::IME[0] & 0x1)
          TriggerIRQ();
  }*/
  if (m_is_irq)
    TriggerIRQ();
}

void ARMv5::SingleStepJIT()
{
// TODO
#ifdef JIT_ENABLED
  u32 instrAddr = R[15] - ((CPSR & 0x20) ? 2 : 4);

  if ((instrAddr < FastBlockLookupStart ||
       instrAddr >= (FastBlockLookupStart + FastBlockLookupSize)) &&
      !NDS.JIT.SetupExecutableRegion(0, instrAddr, FastBlockLookup, FastBlockLookupStart,
                                     FastBlockLookupSize))
  {
    NDS.ARM9Timestamp = NDS.ARM9Target;
    ERROR_LOG_FMT(IOS_LLE, "ARMv5 PC in non executable region {:08x}\n", R[15]);
    return;
  }

  JitBlockEntry block =
      NDS.JIT.LookUpBlock(0, FastBlockLookup, instrAddr - FastBlockLookupStart, instrAddr);
  if (block)
    ARM_Dispatch(this, block);
  else
    NDS.JIT.CompileBlock(this);

  if (StopExecution)
  {
    // this order is crucial otherwise idle loops waiting for an IRQ won't function
    if (IRQ)
      TriggerIRQ();

    if (Halted || IdleLoop)
    {
      if ((Halted == 1 || IdleLoop) && NDS.ARM9Timestamp < NDS.ARM9Target)
      {
        Cycles = 0;
        NDS.ARM9Timestamp = NDS.ARM9Target;
      }
      IdleLoop = 0;
      break;
    }
  }
#else
  SingleStepInterpreter(true);
#endif
}

void ARMv5::SingleStep()
{
  // This would be called from the outside via Step function, which we'd want to disable breakpoints
  // for as we're only stepping one instruction and don't want the breakpoint to stop us again.
  SingleStep(true);
}

void ARMv5::SingleStep(bool skip_bp)
{
  if (m_mode == CPU::CoreMode::Interpreter)
  {
    SingleStepInterpreter(skip_bp);
  }
  else
  {
    SingleStepJIT();
  }

  // NDS.ARM9Timestamp += Cycles;
  m_cycles = 0;
}

void ARMv5::RunLoop()
{
  // if constexpr (mode == CPUExecuteMode::InterpreterGDB)
  //   GdbCheckB();

  auto& cpu = m_system.GetCPU(m_cpu_number);

  if (m_is_halted)
  {
    if (m_is_halted == 2)
    {
      m_is_halted = 0;
    }
    else if (false)
    {
      m_is_halted = 0;
      if (/* NDS.IME[0] */ 0 & 0x1)
        TriggerIRQ();
    }
    else
    {
      return;
    }
  }

  bool start = true;
  while (cpu.GetState() == CPU::State::Running)
  {
    SingleStep(start);
    start = false;
  }

  if (m_is_halted == 2)
    m_is_halted = 0;

  Host_UpdateDisasmDialog();
}

void ARMv5::FillPipeline()
{
  // SetupCodeMem(R[15]);

  if (m_reg_cpsr & 0x20)
  {
    if ((m_reg[15] - 2) & 0x2)
    {
      m_next_inst[0] = CodeRead32(m_reg[15] - 4, false) >> 16;
      m_next_inst[1] = CodeRead32(m_reg[15], false);
    }
    else
    {
      m_next_inst[0] = CodeRead32(m_reg[15] - 2, false);
      m_next_inst[1] = m_next_inst[0] >> 16;
    }
  }
  else
  {
    m_next_inst[0] = CodeRead32(m_reg[15] - 4, false);
    m_next_inst[1] = CodeRead32(m_reg[15], false);
  }
}

#ifdef GDBSTUB_ENABLED
u32 ARMv5::ReadReg(Gdb::Register reg)
{
  using Gdb::Register;
  int r = static_cast<int>(reg);

  if (reg < Register::pc)
    return R[r];
  else if (reg == Register::pc)
  {
    return R[r] - ((CPSR & 0x20) ? 2 : 4);
  }
  else if (reg == Register::cpsr)
    return CPSR;
  else if (reg == Register::sp_usr || reg == Register::lr_usr)
  {
    r -= static_cast<int>(Register::sp_usr);
    if (ModeIs(0x10) || ModeIs(0x1f))
    {
      return R[13 + r];
    }
    else
      switch (CPSR & 0x1f)
      {
      case 0x11:
        return R_FIQ[5 + r];
      case 0x12:
        return R_IRQ[0 + r];
      case 0x13:
        return R_SVC[0 + r];
      case 0x17:
        return R_ABT[0 + r];
      case 0x1b:
        return R_UND[0 + r];
      }
  }
  else if (reg >= Register::r8_fiq && reg <= Register::lr_fiq)
  {
    r -= static_cast<int>(Register::r8_fiq);
    return ModeIs(0x11) ? R[8 + r] : R_FIQ[r];
  }
  else if (reg == Register::sp_irq || reg == Register::lr_irq)
  {
    r -= static_cast<int>(Register::sp_irq);
    return ModeIs(0x12) ? R[13 + r] : R_IRQ[r];
  }
  else if (reg == Register::sp_svc || reg == Register::lr_svc)
  {
    r -= static_cast<int>(Register::sp_svc);
    return ModeIs(0x13) ? R[13 + r] : R_SVC[r];
  }
  else if (reg == Register::sp_abt || reg == Register::lr_abt)
  {
    r -= static_cast<int>(Register::sp_abt);
    return ModeIs(0x17) ? R[13 + r] : R_ABT[r];
  }
  else if (reg == Register::sp_und || reg == Register::lr_und)
  {
    r -= static_cast<int>(Register::sp_und);
    return ModeIs(0x1b) ? R[13 + r] : R_UND[r];
  }
  else if (reg == Register::spsr_fiq)
    return ModeIs(0x11) ? CPSR : R_FIQ[7];
  else if (reg == Register::spsr_irq)
    return ModeIs(0x12) ? CPSR : R_IRQ[2];
  else if (reg == Register::spsr_svc)
    return ModeIs(0x13) ? CPSR : R_SVC[2];
  else if (reg == Register::spsr_abt)
    return ModeIs(0x17) ? CPSR : R_ABT[2];
  else if (reg == Register::spsr_und)
    return ModeIs(0x1b) ? CPSR : R_UND[2];

  WARN_LOG_FMT(IOS_LLE, "GDB reg read: unknown reg no %d\n", r);
  return 0xdeadbeef;
}
void ARMv5::WriteReg(Gdb::Register reg, u32 v)
{
  using Gdb::Register;
  int r = static_cast<int>(reg);

  if (reg < Register::pc)
    R[r] = v;
  else if (reg == Register::pc)
    JumpTo(v);
  else if (reg == Register::cpsr)
    CPSR = v;
  else if (reg == Register::sp_usr || reg == Register::lr_usr)
  {
    r -= static_cast<int>(Register::sp_usr);
    if (ModeIs(0x10) || ModeIs(0x1f))
    {
      R[13 + r] = v;
    }
    else
      switch (CPSR & 0x1f)
      {
      case 0x11:
        R_FIQ[5 + r] = v;
        break;
      case 0x12:
        R_IRQ[0 + r] = v;
        break;
      case 0x13:
        R_SVC[0 + r] = v;
        break;
      case 0x17:
        R_ABT[0 + r] = v;
        break;
      case 0x1b:
        R_UND[0 + r] = v;
        break;
      }
  }
  else if (reg >= Register::r8_fiq && reg <= Register::lr_fiq)
  {
    r -= static_cast<int>(Register::r8_fiq);
    *(ModeIs(0x11) ? &R[8 + r] : &R_FIQ[r]) = v;
  }
  else if (reg == Register::sp_irq || reg == Register::lr_irq)
  {
    r -= static_cast<int>(Register::sp_irq);
    *(ModeIs(0x12) ? &R[13 + r] : &R_IRQ[r]) = v;
  }
  else if (reg == Register::sp_svc || reg == Register::lr_svc)
  {
    r -= static_cast<int>(Register::sp_svc);
    *(ModeIs(0x13) ? &R[13 + r] : &R_SVC[r]) = v;
  }
  else if (reg == Register::sp_abt || reg == Register::lr_abt)
  {
    r -= static_cast<int>(Register::sp_abt);
    *(ModeIs(0x17) ? &R[13 + r] : &R_ABT[r]) = v;
  }
  else if (reg == Register::sp_und || reg == Register::lr_und)
  {
    r -= static_cast<int>(Register::sp_und);
    *(ModeIs(0x1b) ? &R[13 + r] : &R_UND[r]) = v;
  }
  else if (reg == Register::spsr_fiq)
  {
    *(ModeIs(0x11) ? &CPSR : &R_FIQ[7]) = v;
  }
  else if (reg == Register::spsr_irq)
  {
    *(ModeIs(0x12) ? &CPSR : &R_IRQ[2]) = v;
  }
  else if (reg == Register::spsr_svc)
  {
    *(ModeIs(0x13) ? &CPSR : &R_SVC[2]) = v;
  }
  else if (reg == Register::spsr_abt)
  {
    *(ModeIs(0x17) ? &CPSR : &R_ABT[2]) = v;
  }
  else if (reg == Register::spsr_und)
  {
    *(ModeIs(0x1b) ? &CPSR : &R_UND[2]) = v;
  }
  else
    WARN_LOG_FMT(IOS_LLE, "GDB reg write: unknown reg no %d (write 0x{:08x})\n", r, v);
}
u32 ARMv5::ReadMem(u32 addr, int size)
{
  if (size == 8)
    return BusRead8(addr);
  else if (size == 16)
    return BusRead16(addr);
  else if (size == 32)
    return BusRead32(addr);
  else
    return 0xfeedface;
}
void ARMv5::WriteMem(u32 addr, int size, u32 v)
{
  if (size == 8)
    BusWrite8(addr, (u8)v);
  else if (size == 16)
    BusWrite16(addr, (u16)v);
  else if (size == 32)
    BusWrite32(addr, v);
}

void ARMv5::ResetGdb()
{
  NDS.Reset();
  NDS.GPU.StartFrame();  // need this to properly kick off the scheduler & frame output
}
int ARMv5::RemoteCmd(const u8* cmd, size_t len)
{
  (void)len;

  Log(LogLevel::Info, "[ARMGDB] Rcmd: \"%s\"\n", cmd);
  if (!strcmp((const char*)cmd, "reset") || !strcmp((const char*)cmd, "r"))
  {
    Reset();
    return 0;
  }

  return 1;  // not implemented (yet)
}

void ARMv5::WriteMem(u32 addr, int size, u32 v)
{
  if (addr < ITCMSize)
  {
    if (size == 8)
      *(u8*)&ITCM[addr & (ITCMPhysicalSize - 1)] = (u8)v;
    else if (size == 16)
      *(u16*)&ITCM[addr & (ITCMPhysicalSize - 1)] = (u16)v;
    else if (size == 32)
      *(u32*)&ITCM[addr & (ITCMPhysicalSize - 1)] = (u32)v;
    else
    {
    }
    return;
  }
  else if ((addr & DTCMMask) == DTCMBase)
  {
    if (size == 8)
      *(u8*)&DTCM[addr & (DTCMPhysicalSize - 1)] = (u8)v;
    else if (size == 16)
      *(u16*)&DTCM[addr & (DTCMPhysicalSize - 1)] = (u16)v;
    else if (size == 32)
      *(u32*)&DTCM[addr & (DTCMPhysicalSize - 1)] = (u32)v;
    else
    {
    }
    return;
  }

  ARMv5::WriteMem(addr, size, v);
}
u32 ARMv5::ReadMem(u32 addr, int size)
{
  if (addr < ITCMSize)
  {
    if (size == 8)
      return *(u8*)&ITCM[addr & (ITCMPhysicalSize - 1)];
    else if (size == 16)
      return *(u16*)&ITCM[addr & (ITCMPhysicalSize - 1)];
    else if (size == 32)
      return *(u32*)&ITCM[addr & (ITCMPhysicalSize - 1)];
    else
      return 0xfeedface;
  }
  else if ((addr & DTCMMask) == DTCMBase)
  {
    if (size == 8)
      return *(u8*)&DTCM[addr & (DTCMPhysicalSize - 1)];
    else if (size == 16)
      return *(u16*)&DTCM[addr & (DTCMPhysicalSize - 1)];
    else if (size == 32)
      return *(u32*)&DTCM[addr & (DTCMPhysicalSize - 1)];
    else
      return 0xfeedface;
  }

  return ARMv5::ReadMem(addr, size);
}
#endif

// Overloaded byteswap functions, for use within the templated functions below.
[[maybe_unused]] static u8 bswap(u8 val)
{
  return val;
}
[[maybe_unused]] static s8 bswap(s8 val)
{
  return val;
}
[[maybe_unused]] static u16 bswap(u16 val)
{
  return Common::swap16(val);
}
[[maybe_unused]] static s16 bswap(s16 val)
{
  return Common::swap16(val);
}
[[maybe_unused]] static u32 bswap(u32 val)
{
  return Common::swap32(val);
}
[[maybe_unused]] static u64 bswap(u64 val)
{
  return Common::swap64(val);
}

template <typename T>
T ARMv5::ReadFromHardware(u32 addr, bool host)
{
  if ((addr & 0xFF000000) == 0x0D000000)
  {
    // Enforce the 0x00800000 bit to be one to signal to the hardware that the request is from ARM
    // (privileged)
    addr |= 0x00800000;
    return static_cast<T>(m_memory.GetMMIOMapping()->Read<std::make_unsigned_t<T>>(m_system, addr));
  }

  if (m_memory.GetRAM() && (addr & 0xF8000000) == 0x00000000)
  {
    // Handle RAM; the masking intentionally discards bits (essentially creating
    // mirrors of memory).
    addr &= m_memory.GetRamMask();

    if (addr + sizeof(T) > m_memory.GetRamSizeReal())
    {
      if (!host)
        ERROR_LOG_FMT(IOS_LLE, "Attempt to read past RAM bounds: {:x} PC {:x}", addr, m_reg[15]);
      return 0;
    }

    T data;
    std::memcpy(&data, &m_memory.GetRAM()[addr], sizeof(T));

    return bswap(data);
  }

  if (m_memory.GetEXRAM() && (addr >> 28) == 0x1 &&
      (addr & 0x0FFFFFFF) < m_memory.GetExRamSizeReal())
  {
    addr &= 0x0FFFFFFF;

    if (addr + sizeof(T) > m_memory.GetExRamSizeReal())
    {
      if (!host)
        ERROR_LOG_FMT(IOS_LLE, "Attempt to read past EXRAM bounds: {:x} PC {:x}", addr, m_reg[15]);
      return 0;
    }

    T data;
    std::memcpy(&data, &m_memory.GetEXRAM()[addr], sizeof(T));

    return bswap(data);
  }

  // SRAM
  bool sram_mirr = !!m_system.GetWiiIPC().GetSRNPROTFlags()[IOS::SRNPROT::IOUEN];
  if (m_memory.GetIopSRAM() &&
      ((addr & 0xFF400000) == 0x0D400000 || (sram_mirr && (addr & 0xFFF00000) == 0xFFF00000)))
  {
    addr &= m_memory.GetIopSramMask();

    if (addr + sizeof(T) > m_memory.GetIopSramSize())
    {
      if (!host)
        ERROR_LOG_FMT(IOS_LLE, "Attempt to read past IOP SRAM bounds: {:x} PC {:x}", addr,
                      m_reg[15]);
      return 0;
    }

    T data;
    std::memcpy(&data, &m_memory.GetIopSRAM()[addr], sizeof(T));

    return bswap(data);
  }

  // TODO: boot0 ROM

  if (!host)
    ERROR_LOG_FMT(IOS_LLE, "Unable to resolve ARM read address {:x} PC {:x}", addr, m_reg[15]);
  return 0;
}

void ARMv5::WriteToHardware(u32 addr, const u32 data, const u32 size, bool host)
{
  if ((addr & 0xFF000000) == 0x0D000000)
  {
    // Enforce the 0x00800000 bit to be one to signal to the hardware that the request is from ARM
    // (privileged)
    addr |= 0x00800000;

    switch (size)
    {
    case 1:
      m_memory.GetMMIOMapping()->Write<u8>(m_system, addr, static_cast<u8>(data));
      return;
    case 2:
      m_memory.GetMMIOMapping()->Write<u16>(m_system, addr, static_cast<u16>(data));
      return;
    case 4:
      m_memory.GetMMIOMapping()->Write<u32>(m_system, addr, data);
      return;
    default:
      // Some kind of misaligned write. TODO: Does this match how the actual hardware handles it?
      for (size_t i = size * 8; i > 0; addr++)
      {
        i -= 8;
        m_memory.GetMMIOMapping()->Write<u8>(m_system, addr, static_cast<u8>(data >> i));
      }
      return;
    }
  }

  const u32 swapped_data = Common::swap32(std::rotr(data, size * 8));

  if (m_memory.GetRAM() && (addr & 0xF8000000) == 0x00000000)
  {
    // Handle RAM; the masking intentionally discards bits (essentially creating
    // mirrors of memory).
    addr &= m_memory.GetRamMask();

    if (addr + size > m_memory.GetRamSizeReal())
    {
      if (!host)
        ERROR_LOG_FMT(IOS_LLE, "Attempt to write past RAM bounds: {:x} PC {:x}", addr, m_reg[15]);
      return;
    }

    std::memcpy(&m_memory.GetRAM()[addr], &swapped_data, size);

    return;
  }

  if (m_memory.GetEXRAM() && (addr >> 28) == 0x1 &&
      (addr & 0x0FFFFFFF) < m_memory.GetExRamSizeReal())
  {
    addr &= 0x0FFFFFFF;

    if (addr + size > m_memory.GetExRamSizeReal())
    {
      if (!host)
        ERROR_LOG_FMT(IOS_LLE, "Attempt to write past EXRAM bounds: {:x} PC {:x}", addr, m_reg[15]);
      return;
    }

    std::memcpy(&m_memory.GetEXRAM()[addr], &swapped_data, size);

    return;
  }

  // SRAM
  bool sram_mirr = !!m_system.GetWiiIPC().GetSRNPROTFlags()[IOS::SRNPROT::IOUEN];
  if (m_memory.GetIopSRAM() &&
      ((addr & 0xFF400000) == 0x0D400000 || (sram_mirr && (addr & 0xFFF00000) == 0xFFF00000)))
  {
    addr &= m_memory.GetIopSramMask();
    if (addr + size > m_memory.GetIopSramSize())
    {
      if (!host)
        ERROR_LOG_FMT(IOS_LLE, "Attempt to write past IOP SRAM bounds: {:x} PC {:x}", addr,
                      m_reg[15]);
      return;
    }

    std::memcpy(&m_memory.GetIopSRAM()[addr], &swapped_data, size);

    return;
  }

  // TODO: boot0 ROM

  if (!host)
    ERROR_LOG_FMT(IOS_LLE, "Unable to resolve ARM write address {:x} PC {:x}", addr, m_reg[15]);
}

u8 ARMv5::BusRead8(u32 addr)
{
  return ReadFromHardware<u8>(addr, false);
}

u16 ARMv5::BusRead16(u32 addr)
{
  return ReadFromHardware<u16>(addr, false);
}

u32 ARMv5::BusRead32(u32 addr)
{
  return ReadFromHardware<u32>(addr, false);
}

void ARMv5::BusWrite8(u32 addr, u8 val)
{
  WriteToHardware(addr, val, sizeof(val), false);
}

void ARMv5::BusWrite16(u32 addr, u16 val)
{
  WriteToHardware(addr, val, sizeof(val), false);
}

void ARMv5::BusWrite32(u32 addr, u32 val)
{
  WriteToHardware(addr, val, sizeof(val), false);
}

u32 ARMv5::HostRead_U32(u32 addr)
{
  TranslateAddress(addr, false, true);
  return ReadFromHardware<u32>(addr, true);
}

u32 ARMv5::HostRead_Instruction(u32 addr)
{
  TranslateAddress(addr, false, true);
  return ReadFromHardware<u32>(addr, true);
}

bool ARMv5::HostIsRAMAddress(u32 addr)
{
  TranslateAddress(addr, false, true);

  if ((addr & 0xF8000000) == 0x00000000 &&
      ((addr & m_memory.GetRamMask()) + 3) < m_memory.GetRamSizeReal())
    return true;

  if (m_memory.GetEXRAM() && (addr >> 28) == 0x1 &&
      (addr & 0x0FFFFFFF) < m_memory.GetExRamSizeReal() &&
      ((addr & m_memory.GetExRamMask()) + 3) < m_memory.GetExRamSizeReal())
    return true;

  bool sram_mirr = !!m_system.GetWiiIPC().GetSRNPROTFlags()[IOS::SRNPROT::IOUEN];
  if (m_memory.GetIopSRAM() &&
      ((addr & 0xFF400000) == 0x0D400000 || (sram_mirr && (addr & 0xFFF00000) == 0xFFF00000)) &&
      (addr & 0x1FFFF) <= 0x1FFFC)
    return true;

  return false;
}

}  // namespace IOS::LLE
