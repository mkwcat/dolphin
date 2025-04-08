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

#pragma once

#include <algorithm>
#include <optional>

#include "Common/CommonTypes.h"
#include "Core/HW/CPU.h"
#include "Core/System.h"

#ifdef GDBSTUB_ENABLED
#include "debug/GdbStub.h"
#endif

namespace IOS::LLE
{
inline u32 ROR(u32 x, u32 n)
{
  return (x >> (n & 0x1F)) | (x << ((32 - n) & 0x1F));
}

enum
{
  RWFlags_Nonseq = (1 << 5),
  RWFlags_ForceUser = (1 << 21),
};

enum class CPUExecuteMode : u32
{
  Interpreter,
  InterpreterGDB,
#ifdef JIT_ENABLED
  JIT
#endif
};

struct GDBArgs
{
  // TODO
};
class ARMJIT;
class GPU;
class ARMJIT_Memory;
class NDS;
class Savestate;

class ARMv5 : public CPU::CPUManagerImplBase
#ifdef GDBSTUB_ENABLED
    : public Gdb::StubCallbacks
#endif
{
  friend class ARMInterpreter;

public:
  ARMv5(Core::System& system, Memory::MemoryManager& memory, std::optional<GDBArgs> gdb, bool jit);
  ~ARMv5();

  ARMv5* GetARM9() override { return this; }
  const ARMv5* GetARM9() const override { return this; }

  void Reset() override;
  void Shutdown() override;

  void DoState(PointerWrap& p) override;

  // void UpdateRegionTimings(u32 addrstart, u32 addrend);

  void FillPipeline();

  void JumpTo(u32 addr, bool restorecpsr = false);

  void PrefetchAbort();
  void DataAbort();

  void RunLoop() override;
  void SingleStep() override;
  void SingleStep(bool skip_bp);

  void SingleStepInterpreter(bool skip_bp);
  void SingleStepJIT();

  CPU::CoreMode GetMode() const override { return m_mode; }
  void SetMode(CPU::CoreMode mode) override { m_mode = mode; }

  u32 GetPC() override;

  const char* GetCPUName() const override { return "Interpreter"; }

  static constexpr int Num = 0;

  void SetGdbArgs(std::optional<GDBArgs> gdb);

  void RestoreCPSR();

  void Halt(u32 halt)
  {
    if (halt == 2 && m_is_halted == 1)
      return;
    m_is_halted = halt;
  }

  void NocashPrint(u32 addr) noexcept;

  bool CheckCondition(u32 code) const
  {
    if (code == 0xE)
      return true;
    if (ConditionTable[code] & (1 << (m_reg_cpsr >> 28)))
      return true;
    return false;
  }

  void SetC(bool c)
  {
    if (c)
      m_reg_cpsr |= 0x20000000;
    else
      m_reg_cpsr &= ~0x20000000;
  }

  void SetNZ(bool n, bool z)
  {
    m_reg_cpsr &= ~0xC0000000;
    if (n)
      m_reg_cpsr |= 0x80000000;
    if (z)
      m_reg_cpsr |= 0x40000000;
  }

  void SetNZCV(bool n, bool z, bool c, bool v)
  {
    m_reg_cpsr &= ~0xF0000000;
    if (n)
      m_reg_cpsr |= 0x80000000;
    if (z)
      m_reg_cpsr |= 0x40000000;
    if (c)
      m_reg_cpsr |= 0x20000000;
    if (v)
      m_reg_cpsr |= 0x10000000;
  }

  inline bool ModeIs(u32 mode) const
  {
    u32 cm = m_reg_cpsr & 0x1f;
    mode &= 0x1f;

    if (mode == cm)
      return true;
    if (mode == 0x17)
      return cm >= 0x14 && cm <= 0x17;  // abt
    if (mode == 0x1b)
      return cm >= 0x18 && cm <= 0x1b;  // und

    return false;
  }

  void UpdateMode(u32 oldmode, u32 newmode, bool phony = false);

  void TriggerIRQ();

  // all code accesses are forced nonseq 32bit
  u32 CodeRead32(u32 addr, bool branch);

  void DataRead8(u32 addr, u32* val);
  void DataRead16(u32 addr, u32* val);
  void DataRead32(u32 addr, u32* val);
  void DataRead32S(u32 addr, u32* val);
  void DataWrite8(u32 addr, u8 val);
  void DataWrite16(u32 addr, u16 val);
  void DataWrite32(u32 addr, u32 val);
  void DataWrite32S(u32 addr, u32 val);

  void AddCycles_C()
  {
    // code only. always nonseq 32-bit for ARM9.
    s32 numC = (m_reg[15] & 0x2) ? 0 : m_code_cycles;
    m_cycles += numC;
  }

  void AddCycles_CI(s32 numI)
  {
    // code+internal
    s32 numC = (m_reg[15] & 0x2) ? 0 : m_code_cycles;
    m_cycles += numC + numI;
  }

  void AddCycles_CDI()
  {
    // LDR/LDM cycles. ARM9 seems to skip the internal cycle there.
    // TODO: ITCM data fetches shouldn't be parallelized, they say
    s32 numC = (m_reg[15] & 0x2) ? 0 : m_code_cycles;
    s32 numD = m_data_cycles;

    // if (DataRegion != CodeRegion)
    m_cycles += std::max(numC + numD - 6, std::max(numC, numD));
    // else
    //     Cycles += numC + numD;
  }

  void AddCycles_CD()
  {
    // TODO: ITCM data fetches shouldn't be parallelized, they say
    s32 numC = (m_reg[15] & 0x2) ? 0 : m_code_cycles;
    s32 numD = m_data_cycles;

    // if (DataRegion != CodeRegion)
    m_cycles += std::max(numC + numD - 6, std::max(numC, numD));
    // else
    //     Cycles += numC + numD;
  }

  // void GetCodeMemRegion(u32 addr, MemRegion* region);

  void CP15Reset();
  void CP15DoSavestate(Savestate* file);

  void UpdateDTCMSetting();
  void UpdateITCMSetting();

  void UpdatePURegion(u32 n);
  void UpdatePURegions(bool update_all);

  u32 RandomLineIndex();

  void ICacheLookup(u32 addr);
  void ICacheInvalidateByAddr(u32 addr);
  void ICacheInvalidateAll();

  void CP15Write(u32 id, u32 val);
  u32 CP15Read(u32 id) const;

  void CheckGdbIncoming();

  void SVCWrite0(u32 addr);
  void LogSyscall(u32 instr);

#ifdef GDBSTUB_ENABLED
  int GetCPU() const { return 9; }

  u32 ReadReg(Gdb::Register reg) override;
  void WriteReg(Gdb::Register reg, u32 v) override;
  u32 ReadMem(u32 addr, int size) override;
  void WriteMem(u32 addr, int size, u32 v) override;

  void ResetGdb() override;
  int RemoteCmd(const u8* cmd, size_t len) override;
#endif

protected:
  void GdbCheckA();
  void GdbCheckB();
  void GdbCheckC();

  // Memory functions

protected:
  bool CheckAccessPermission(u32 ap, bool is_write);
  bool TranslateAddress(u32& addr, bool is_write, bool host = false);

  template <typename T>
  T ReadFromHardware(u32 em_address, bool host);
  void WriteToHardware(u32 em_address, const u32 data, const u32 size, bool host);

  u8 BusRead8(u32 addr);
  u16 BusRead16(u32 addr);
  u32 BusRead32(u32 addr);
  void BusWrite8(u32 addr, u8 val);
  void BusWrite16(u32 addr, u16 val);
  void BusWrite32(u32 addr, u32 val);

public:
  u32 HostRead_U32(u32 addr);
  u32 HostRead_Instruction(u32 addr);
  bool HostIsRAMAddress(u32 addr);

private:
  u32 m_reg_cp15_control;

  u32 m_rng_seed;

  u8 m_icache_data[0x2000];
  u32 m_icache_tags[64 * 4];
  u8 m_icache_count[64];
  u8* m_icache_line;

  u32 m_reg_ttbr;
  u32 m_reg_dfsr;
  u32 m_reg_ifsr;
  u32 m_reg_far;
  u32 m_reg_dacr;
  u32 m_reg_fcse_pid;
  u32 m_reg_context_id;

public:
  s32 m_cycles;
  union
  {
    struct
    {
      u8 m_is_halted;
      u8 m_is_irq;  // nonzero to trigger IRQ
      u8 m_is_idle_loop;
    };
    u32 m_stop_execution;
  };

  u32 m_code_region;
  s32 m_code_cycles;

  u32 m_data_region;
  s32 m_data_cycles;

  u32 m_reg[16];  // heh
  u32 m_reg_cpsr;

  u32 m_fiq_reg[8];  // holding SPSR too
  u32 m_svc_reg[3];
  u32 m_abt_reg[3];
  u32 m_irq_reg[3];
  u32 m_und_reg[3];

  // Current instruction
  u32 m_inst;

  // Next instruction in pipeline
  u32 m_next_inst[2];

private:
  u32 m_exception_base;

#ifdef JIT_ENABLED
  u32 FastBlockLookupStart, FastBlockLookupSize;
  u64* FastBlockLookup;
#endif

  static const u32 ConditionTable[16];
#ifdef GDBSTUB_ENABLED
  Gdb::GdbStub GdbStub;
#endif

  std::string m_svc_write_buffer;

  bool IsSingleStep;
  bool BreakReq;
  bool BreakOnStartup;
  u16 Port;

  Core::System& m_system;
  Memory::MemoryManager& m_memory;

  CPU::CoreMode m_mode = CPU::CoreMode::Interpreter;
  // TODO: Set dynamically?
  CPU::CPUNumber m_cpu_number = CPU::CPUNumber::ARM9;
};

}  // namespace IOS::LLE
