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
#include "Common/Thread.h"
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
void ARM::GdbCheckA()
{
  if (!IsSingleStep && !BreakReq)
  {  // check if eg. break signal is incoming etc.
    Gdb::StubState st = GdbStub.Enter(false, Gdb::TgtStatus::NoEvent, ~(u32)0u, BreakOnStartup);
    BreakOnStartup = false;
    IsSingleStep = st == Gdb::StubState::Step;
    BreakReq = st == Gdb::StubState::Attach || st == Gdb::StubState::Break;
  }
}
void ARM::GdbCheckB()
{
  if (IsSingleStep || BreakReq)
  {  // use else here or we single-step the same insn twice in gdb
    u32 pc_real = R[15] - ((CPSR & 0x20) ? 2 : 4);
    Gdb::StubState st = GdbStub.Enter(true, Gdb::TgtStatus::SingleStep, pc_real);
    IsSingleStep = st == Gdb::StubState::Step;
    BreakReq = st == Gdb::StubState::Attach || st == Gdb::StubState::Break;
  }
}
void ARM::GdbCheckC()
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
void ARM::GdbCheckA()
{
}
void ARM::GdbCheckB()
{
}
void ARM::GdbCheckC()
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

const u32 ARM::ConditionTable[16] = {
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

ARM::ARM(u32 num, bool jit, std::optional<GDBArgs> gdb)
#ifdef GDBSTUB_ENABLED
    : GdbStub(this), BreakOnStartup(false),
#endif
{
  SetGdbArgs(jit ? std::nullopt : gdb);
}

ARM::~ARM()
{
  // dorp
}

ARMv5::ARMv5(Core::System& system, Memory::MemoryManager& memory, std::optional<GDBArgs> gdb,
             bool jit)
    : ARM(0, jit, gdb), m_system(system), m_memory(memory)
{
}

ARMv5::~ARMv5()
{
}

void ARM::SetGdbArgs(std::optional<GDBArgs> gdb)
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
  Cycles = 0;
  Halted = 0;

  IRQ = 0;

  for (int i = 0; i < 16; i++)
    R[i] = 0;

  CPSR = 0x000000D3;

  for (int i = 0; i < 7; i++)
    R_FIQ[i] = 0;
  for (int i = 0; i < 2; i++)
  {
    R_SVC[i] = 0;
    R_ABT[i] = 0;
    R_IRQ[i] = 0;
    R_UND[i] = 0;
  }

  R_FIQ[7] = 0x00000010;
  R_SVC[2] = 0x00000010;
  R_ABT[2] = 0x00000010;
  R_IRQ[2] = 0x00000010;
  R_UND[2] = 0x00000010;

  ExceptionBase = 0xFFFF0000;

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
  JumpTo(ExceptionBase);
}

#if 0

void ARM::DoSavestate(Savestate* file)
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
    file->Var32(&CurInstr);
#ifdef JIT_ENABLED
    if (file->Saving && NDS.IsJITEnabled())
    {
        // hack, the JIT doesn't really pipeline
        // but we still want JIT save states to be
        // loaded while running the interpreter
        FillPipeline();
    }
#endif
    file->VarArray(NextInstr, 2*sizeof(u32));

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
    ARM::DoSavestate(file);
    CP15DoSavestate(file);
}

#endif

void ARMv5::JumpTo(u32 addr, bool restorecpsr)
{
  if (restorecpsr)
  {
    RestoreCPSR();

    if (CPSR & 0x20)
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
    R[15] = addr + 2;

    // if (newregion != oldregion)
    //   SetupCodeMem(addr);

    // two-opcodes-at-once fetch
    // doesn't matter if we put garbage in the MSbs there
    if (addr & 0x2)
    {
      NextInstr[0] = CodeRead32(addr - 2, true);
      Cycles += CodeCycles;
      NextInstr[1] = CodeRead32(addr + 2, false);
      Cycles += CodeCycles;
    }
    else
    {
      u32 instr = CodeRead32(addr, true);
      NextInstr[0] = instr >> 16;
      NextInstr[1] = instr << 16;
      Cycles += CodeCycles;
    }

    CPSR |= 0x20;
  }
  else
  {
    addr &= ~0x3;
    R[15] = addr + 4;

    // if (newregion != oldregion)
    //   SetupCodeMem(addr);

    NextInstr[0] = CodeRead32(addr, true);
    Cycles += CodeCycles;
    NextInstr[1] = CodeRead32(addr + 4, false);
    Cycles += CodeCycles;

    CPSR &= ~0x20;
  }

  u32 addrCopy = addr;
  if (!TranslateAddress(addrCopy, false))
  {
    m_reg_far = addr;
    PrefetchAbort();
    return;
  }
}

void ARM::RestoreCPSR()
{
  u32 oldcpsr = CPSR;

  switch (CPSR & 0x1F)
  {
  case 0x11:
    CPSR = R_FIQ[7];
    break;

  case 0x12:
    CPSR = R_IRQ[2];
    break;

  case 0x13:
    CPSR = R_SVC[2];
    break;

  case 0x14:
  case 0x15:
  case 0x16:
  case 0x17:
    CPSR = R_ABT[2];
    break;

  case 0x18:
  case 0x19:
  case 0x1A:
  case 0x1B:
    CPSR = R_UND[2];
    break;

  default:
    WARN_LOG_FMT(IOS_LLE, "!! attempt to restore CPSR under bad mode {:02x}, {:08x}\n", CPSR & 0x1F,
                 R[15]);
    break;
  }

  CPSR |= 0x00000010;

  UpdateMode(oldcpsr, CPSR);
}

void ARM::UpdateMode(u32 oldmode, u32 newmode, bool phony)
{
  if ((oldmode & 0x1F) == (newmode & 0x1F))
    return;

  switch (oldmode & 0x1F)
  {
  case 0x11:
    std::swap(R[8], R_FIQ[0]);
    std::swap(R[9], R_FIQ[1]);
    std::swap(R[10], R_FIQ[2]);
    std::swap(R[11], R_FIQ[3]);
    std::swap(R[12], R_FIQ[4]);
    std::swap(R[13], R_FIQ[5]);
    std::swap(R[14], R_FIQ[6]);
    break;

  case 0x12:
    std::swap(R[13], R_IRQ[0]);
    std::swap(R[14], R_IRQ[1]);
    break;

  case 0x13:
    std::swap(R[13], R_SVC[0]);
    std::swap(R[14], R_SVC[1]);
    break;

  case 0x17:
    std::swap(R[13], R_ABT[0]);
    std::swap(R[14], R_ABT[1]);
    break;

  case 0x1B:
    std::swap(R[13], R_UND[0]);
    std::swap(R[14], R_UND[1]);
    break;
  }

  switch (newmode & 0x1F)
  {
  case 0x11:
    std::swap(R[8], R_FIQ[0]);
    std::swap(R[9], R_FIQ[1]);
    std::swap(R[10], R_FIQ[2]);
    std::swap(R[11], R_FIQ[3]);
    std::swap(R[12], R_FIQ[4]);
    std::swap(R[13], R_FIQ[5]);
    std::swap(R[14], R_FIQ[6]);
    break;

  case 0x12:
    std::swap(R[13], R_IRQ[0]);
    std::swap(R[14], R_IRQ[1]);
    break;

  case 0x13:
    std::swap(R[13], R_SVC[0]);
    std::swap(R[14], R_SVC[1]);
    break;

  case 0x17:
    std::swap(R[13], R_ABT[0]);
    std::swap(R[14], R_ABT[1]);
    break;

  case 0x1B:
    std::swap(R[13], R_UND[0]);
    std::swap(R[14], R_UND[1]);
    break;
  }
}

void ARM::TriggerIRQ()
{
  if (CPSR & 0x80)
    return;

  u32 oldcpsr = CPSR;
  CPSR &= ~0xFF;
  CPSR |= 0xD2;
  UpdateMode(oldcpsr, CPSR);

  R_IRQ[2] = oldcpsr;
  R[14] = R[15] + (oldcpsr & 0x20 ? 2 : 0);
  JumpTo(ExceptionBase + 0x18);
}

void ARMv5::PrefetchAbort()
{
  WARN_LOG_FMT(IOS_LLE, "ARM9: prefetch abort ({:08x})\n", R[15]);

  u32 oldcpsr = CPSR;
  CPSR &= ~0xBF;
  CPSR |= 0x97;
  UpdateMode(oldcpsr, CPSR);

  // this shouldn't happen, but if it does, we're stuck in some nasty endless loop
  // so better take care of it
  u32 exceptionBaseCopy = ExceptionBase + 0x0C;
  if (!TranslateAddress(exceptionBaseCopy, false))
  {
    ERROR_LOG_FMT(IOS_LLE, "!!!!! EXCEPTION REGION NOT EXECUTABLE. THIS IS VERY BAD!!\n");
    // NDS.Stop(Platform::StopReason::BadExceptionRegion);
    return;
  }

  R_ABT[2] = oldcpsr;
  R[14] = R[15] + (oldcpsr & 0x20 ? 2 : 0);
  JumpTo(ExceptionBase + 0x0C);
}

void ARMv5::DataAbort()
{
  WARN_LOG_FMT(IOS_LLE, "ARM9: data abort ({:08x})\n", R[15]);

  u32 oldcpsr = CPSR;
  CPSR &= ~0xBF;
  CPSR |= 0x97;
  UpdateMode(oldcpsr, CPSR);

  R_ABT[2] = oldcpsr;
  R[14] = R[15] + (oldcpsr & 0x20 ? 4 : 0);
  JumpTo(ExceptionBase + 0x10);
}

void ARM::CheckGdbIncoming()
{
  GdbCheckA();
}

u32 ARMv5::GetPC()
{
  if (CPSR & 0x20)
    return (R[15] - 2) + 1;
  else
    return R[15] - 4;
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

  if (CPSR & 0x20)  // THUMB
  {
    // if constexpr (mode == CPUExecuteMode::InterpreterGDB)
    //   GdbCheckC();

    u32 pcAddr = R[15] - 2;

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
                         pcAddr, R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8], R[9], R[14]);
        }

        if (bp->break_on_hit)
        {
          m_system.GetCPU(m_cpu_number).Break();
          return;
        }
      }
    }

    // prefetch
    R[15] += 2;
    CurInstr = NextInstr[0];
    NextInstr[0] = NextInstr[1] >> 16;
    if (R[15] & 0x2)
    {
      NextInstr[1] <<= 16;
      CodeCycles = 0;
    }
    else
      NextInstr[1] = CodeRead32(R[15], false);

    // INFO_LOG_FMT(IOS_LLE, "THUMB9 instruction {:04x} @ {:08x}\n", CurInstr & 0xFFFF, pcAddr);

    // actually execute
    u32 icode = (CurInstr >> 6) & 0x3FF;
    ARMInterpreter::THUMBInstrTable[icode](this);
  }
  else
  {
    // if constexpr (mode == CPUExecuteMode::InterpreterGDB)
    //   GdbCheckC();

    u32 pcAddr = R[15] - 4;

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
                         pcAddr, R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8], R[9], R[14]);
        }

        if (bp->break_on_hit)
        {
          m_system.GetCPU(m_cpu_number).Break();
          return;
        }
      }
    }

    // prefetch
    R[15] += 4;
    CurInstr = NextInstr[0];
    NextInstr[0] = NextInstr[1];
    NextInstr[1] = CodeRead32(R[15], false);

    // INFO_LOG_FMT(IOS_LLE, "ARM9 instruction {:08x} @ {:08x}\n", CurInstr, pcAddr);

    // actually execute
    if (CheckCondition(CurInstr >> 28))
    {
      u32 icode = ((CurInstr >> 4) & 0xF) | ((CurInstr >> 16) & 0xFF0);
      ARMInterpreter::ARMInstrTable[icode](this);
    }
    else if ((CurInstr & 0xFE000000) == 0xFA000000)
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
  if (IRQ)
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
  Cycles = 0;
}

void ARMv5::RunLoop()
{
  // if constexpr (mode == CPUExecuteMode::InterpreterGDB)
  //   GdbCheckB();

  auto& cpu = m_system.GetCPU(m_cpu_number);

  if (Halted)
  {
    if (Halted == 2)
    {
      Halted = 0;
    }
    else if (false)
    {
      Halted = 0;
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

  if (Halted == 2)
    Halted = 0;

  Host_UpdateDisasmDialog();
}

void ARMv5::FillPipeline()
{
  // SetupCodeMem(R[15]);

  if (CPSR & 0x20)
  {
    if ((R[15] - 2) & 0x2)
    {
      NextInstr[0] = CodeRead32(R[15] - 4, false) >> 16;
      NextInstr[1] = CodeRead32(R[15], false);
    }
    else
    {
      NextInstr[0] = CodeRead32(R[15] - 2, false);
      NextInstr[1] = NextInstr[0] >> 16;
    }
  }
  else
  {
    NextInstr[0] = CodeRead32(R[15] - 4, false);
    NextInstr[1] = CodeRead32(R[15], false);
  }
}

#ifdef GDBSTUB_ENABLED
u32 ARM::ReadReg(Gdb::Register reg)
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
void ARM::WriteReg(Gdb::Register reg, u32 v)
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
u32 ARM::ReadMem(u32 addr, int size)
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
void ARM::WriteMem(u32 addr, int size, u32 v)
{
  if (size == 8)
    BusWrite8(addr, (u8)v);
  else if (size == 16)
    BusWrite16(addr, (u16)v);
  else if (size == 32)
    BusWrite32(addr, v);
}

void ARM::ResetGdb()
{
  NDS.Reset();
  NDS.GPU.StartFrame();  // need this to properly kick off the scheduler & frame output
}
int ARM::RemoteCmd(const u8* cmd, size_t len)
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

  ARM::WriteMem(addr, size, v);
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

  return ARM::ReadMem(addr, size);
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
  if ((addr & 0xF8000000) == 0x08000000)
  {
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
        ERROR_LOG_FMT(IOS_LLE, "Attempt to read past RAM bounds: {:x} PC {:x}", addr, R[15]);
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
        ERROR_LOG_FMT(IOS_LLE, "Attempt to read past EXRAM bounds: {:x} PC {:x}", addr, R[15]);
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
        ERROR_LOG_FMT(IOS_LLE, "Attempt to read past IOP SRAM bounds: {:x} PC {:x}", addr, R[15]);
      return 0;
    }

    T data;
    std::memcpy(&data, &m_memory.GetIopSRAM()[addr], sizeof(T));

    return bswap(data);
  }

  // TODO: boot0 ROM

  if (!host)
    ERROR_LOG_FMT(IOS_LLE, "Unable to resolve ARM read address {:x} PC {:x}", addr, R[15]);
  return 0;
}

void ARMv5::WriteToHardware(u32 addr, const u32 data, const u32 size, bool host)
{
  if ((addr & 0xF8000000) == 0x08000000)
  {
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
        ERROR_LOG_FMT(IOS_LLE, "Attempt to write past RAM bounds: {:x} PC {:x}", addr, R[15]);
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
        ERROR_LOG_FMT(IOS_LLE, "Attempt to write past EXRAM bounds: {:x} PC {:x}", addr, R[15]);
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
        ERROR_LOG_FMT(IOS_LLE, "Attempt to write past IOP SRAM bounds: {:x} PC {:x}", addr, R[15]);
      return;
    }

    std::memcpy(&m_memory.GetIopSRAM()[addr], &swapped_data, size);

    return;
  }

  // TODO: boot0 ROM

  if (!host)
    ERROR_LOG_FMT(IOS_LLE, "Unable to resolve ARM write address {:x} PC {:x}", addr, R[15]);
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
  TranslateAddress(addr, false);
  return ReadFromHardware<u32>(addr, true);
}
u32 ARMv5::HostRead_Instruction(u32 addr)
{
  TranslateAddress(addr, false);
  return ReadFromHardware<u32>(addr, true);
}

bool ARMv5::HostIsRAMAddress(u32 addr)
{
  TranslateAddress(addr, false);

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

void ARM::SVCWrite0(u32 addr)
{
  ARMv5* v5 = static_cast<ARMv5*>(this);

  while (true)
  {
    if (!v5->HostIsRAMAddress(addr))
      break;

    u32 v;
    v5->DataRead8(addr, &v);

    if (v == 0)
      break;

    if (v == '\n')
    {
      INFO_LOG_FMT(IOS_LLE, "IOS REPORT: {:s}", m_svc_write_buffer.data());
      m_svc_write_buffer.clear();
    }
    else if (v != '\r')
    {
      m_svc_write_buffer.push_back(v);
    }

    addr += 1;
  }
}

}  // namespace IOS::LLE
