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
#include <string.h>
#include "ARM.h"
// #include "ARMJIT_Memory.h"
// #include "ARMJIT.h"
#include "Common/BitField.h"
#include "Common/CommonTypes.h"
#include "Common/Logging/Log.h"
#include "Common/Unreachable.h"

namespace IOS::LLE
{

// const int kDataCacheTiming = 3;  // 2;
// const int kCodeCacheTiming = 3;  // 5;

void ARMv5::CP15Reset()
{
  CP15Control = 0x2078;  // dunno

  RNGSeed = 44203;
  m_reg_context_id = 0;

  memset(ICache, 0, 0x2000);
  ICacheInvalidateAll();
  memset(ICacheCount, 0, 64);

  CurICacheLine = NULL;
}

#if 0
void ARMv5::CP15DoSavestate(Savestate* file)
{
    file->Section("CP15");

    file->Var32(&CP15Control);

    file->Var32(&DTCMSetting);
    file->Var32(&ITCMSetting);

    file->VarArray(ITCM, ITCMPhysicalSize);
    file->VarArray(DTCM, DTCMPhysicalSize);
}
#endif

union TLBFirstLevel
{
  u32 m_hex = 0;

  BitField<20, 12, u32> m_base;
  BitField<12, 20, u32> m_fine_base;
  BitField<10, 22, u32> m_coarse_base;

  BitField<0, 2, u32> m_type;

  // Section descriptor
  BitField<2, 1, u32> m_buffered;
  BitField<3, 1, u32> m_cached;
  BitField<5, 4, u32> m_domain;
  BitField<9, 2, u32> m_ap;

  explicit TLBFirstLevel(u32 hex) : m_hex(hex) {}
  TLBFirstLevel() = default;
};

union TLBSecondLevel
{
  u32 m_hex = 0;

  BitField<16, 16, u32> m_large_base;
  BitField<12, 20, u32> m_small_base;
  BitField<10, 22, u32> m_tiny_base;

  BitField<0, 2, u32> m_type;
  BitField<2, 1, u32> m_buffered;
  BitField<3, 1, u32> m_cached;
  BitField<4, 2, u32> m_ap;

  explicit TLBSecondLevel(u32 hex) : m_hex(hex) {}
  TLBSecondLevel() = default;
};

bool ARMv5::CheckAccessPermission(u32 ap, bool is_write)
{
  if (ap == 0b00)
  {
    // Special case for ROM data: check SR bits (flipped here) for read-only access.
    if (is_write)
    {
      return false;
    }
    switch ((CP15Control >> 8) & 0b11)
    {
    case 0b00:
    case 0b11:
      // No access.
      return false;

    case 0b10:
      // Read-only access.
      return true;

    case 0b01:
      // Read-only if privileged.
      return !!(CPSR & 0b1111);
    }
  }

  // Can always access everything in a privileged mode.
  if (CPSR & 0b1111)
  {
    return true;
  }

  // dac == 0b01, check access permission bits.
  switch (ap)
  {
  default:
  case 0b01:
    // No access.
    return false;

  case 0b10:
    // Read-only access.
    return !is_write;

  case 0b11:
    // Read-write access.
    return true;
  }
}

bool ARMv5::TranslateAddress(u32& addr, bool is_write, bool host)
{
  if (!(CP15Control & 0x5))
    return true;

  const u32 first_level_offset = (addr >> 20) & 0xFFF;

  // Fetch first-level descriptor
  const TLBFirstLevel first(BusRead32(m_reg_ttbr | (first_level_offset << 2)));
  if (first.m_type == 0b00)
  {
    // Fault
    if (!host)
      ERROR_LOG_FMT(IOS_LLE, "TLB fault for address {:08x} (domain {:x})", addr, first.m_domain);
    return false;
  }

  u32 dac = (m_reg_dacr >> (first.m_domain * 2)) & 0b11;
  if (dac == 0b00 || dac == 0b10)
  {
    return false;
  }

  bool is_manager = dac == 0b11;
  u32 second_level_offset = 0;

  switch (first.m_type)
  {
  case 0b00:  // Fault
    return false;

  case 0b10:  // Section
  {
    if (!host && !is_manager && !CheckAccessPermission(first.m_ap, is_write))
    {
      // Permission denied
      ERROR_LOG_FMT(IOS_LLE, "TLB permission denied for address {:08x} (ap {:x}, domain {:x})",
                    addr, first.m_ap, first.m_domain);
      return false;
    }

    // Mask address together
    addr = (first.m_base << 20) | (addr & 0x000FFFFF);
    return true;
  }

  case 0b01:  // Coarse page table (0xXXXFF000)
    second_level_offset = (first.m_coarse_base << 10) | ((addr >> 10) & 0x3FC);
    break;

  case 0b11:  // Fine page table (0xXXXFFC00)
    second_level_offset = (first.m_fine_base << 12) | ((addr >> 8) & 0xFFC);
    break;
  }

  // Fetch second-level descriptor
  const TLBSecondLevel second(BusRead32(second_level_offset));
  if (second.m_type == 0b00)
  {
    // Fault
    if (!host)
      ERROR_LOG_FMT(IOS_LLE, "TLB fault for address {:08x} (domain {:x})", addr, first.m_domain);
    return false;
  }

  if (!host && !is_manager && !CheckAccessPermission(second.m_ap, is_write))
  {
    // Permission denied
    ERROR_LOG_FMT(IOS_LLE, "TLB permission denied for address {:08x} (ap {:x}, domain {:x})", addr,
                  second.m_ap, first.m_domain);
    return false;
  }

  // Mask address together
  switch (second.m_type)
  {
  case 0b01:  // Large page
    addr = (second.m_large_base << 16) | (addr & 0x0000FFFF);
    return true;

  case 0b10:  // Small page
    addr = (second.m_small_base << 12) | (addr & 0x00000FFF);
    return true;

  case 0b11:  // Tiny page
    addr = (second.m_tiny_base << 10) | (addr & 0x000003FF);
    return true;
  }

  Common::Unreachable();
}

u32 ARMv5::RandomLineIndex()
{
  // lame RNG, but good enough for this purpose
  u32 s = RNGSeed;
  RNGSeed ^= (s * 17);
  RNGSeed ^= (s * 7);

  return (RNGSeed >> 17) & 0x3;
}

void ARMv5::ICacheLookup(u32 addr)
{
  u32 tag = addr & 0xFFFFF800;
  u32 id = (addr >> 5) & 0x3F;

  id <<= 2;
  if (ICacheTags[id + 0] == tag)
  {
    CodeCycles = 1;
    CurICacheLine = &ICache[(id + 0) << 5];
    return;
  }
  if (ICacheTags[id + 1] == tag)
  {
    CodeCycles = 1;
    CurICacheLine = &ICache[(id + 1) << 5];
    return;
  }
  if (ICacheTags[id + 2] == tag)
  {
    CodeCycles = 1;
    CurICacheLine = &ICache[(id + 2) << 5];
    return;
  }
  if (ICacheTags[id + 3] == tag)
  {
    CodeCycles = 1;
    CurICacheLine = &ICache[(id + 3) << 5];
    return;
  }

  // cache miss

  u32 line;
  if (CP15Control & (1 << 14))
  {
    line = ICacheCount[id >> 2];
    ICacheCount[id >> 2] = (line + 1) & 0x3;
  }
  else
  {
    line = RandomLineIndex();
  }

  line += id;

  addr &= ~0x1F;
  u8* ptr = &ICache[line << 5];

#if 0
    if (CodeMem.Mem)
    {
        memcpy(ptr, &CodeMem.Mem[addr & CodeMem.Mask], 32);
    }
    else
#endif
  {
    for (int i = 0; i < 32; i += 4)
      *(u32*)&ptr[i] = BusRead32(addr + i);
  }

  ICacheTags[line] = tag;

  // ouch :/
  // printf("cache miss %08X: %d/%d\n", addr, NDS::ARM9MemTimings[addr >> 14][2],
  // NDS::ARM9MemTimings[addr >> 14][3]);
  // CodeCycles = (NDS.ARM9MemTimings[addr >> 14][2] + (NDS.ARM9MemTimings[addr >> 14][3] * 7)) <<
  // NDS.ARM9ClockShift;
  CurICacheLine = ptr;
}

void ARMv5::ICacheInvalidateByAddr(u32 addr)
{
  u32 tag = addr & 0xFFFFF800;
  u32 id = (addr >> 5) & 0x3F;

  id <<= 2;
  if (ICacheTags[id + 0] == tag)
  {
    ICacheTags[id + 0] = 1;
    return;
  }
  if (ICacheTags[id + 1] == tag)
  {
    ICacheTags[id + 1] = 1;
    return;
  }
  if (ICacheTags[id + 2] == tag)
  {
    ICacheTags[id + 2] = 1;
    return;
  }
  if (ICacheTags[id + 3] == tag)
  {
    ICacheTags[id + 3] = 1;
    return;
  }
}

void ARMv5::ICacheInvalidateAll()
{
  for (int i = 0; i < 64 * 4; i++)
    ICacheTags[i] = 1;
}

void ARMv5::CP15Write(u32 id, u32 val)
{
  // if(id!=0x704)printf("CP15 write op %03X %08X %08X\n", id, val, R[15]);

  switch (id)
  {
  case 0x100:
  {
    u32 old = CP15Control;
    val &= 0x000FF085;
    CP15Control &= ~0x000FF085;
    CP15Control |= val;
    // printf("CP15Control = %08X (%08X->%08X)\n", CP15Control, old, val);
    // if (val & (1<<7)) WARN_LOG_FMT(IOS_LLE, "!!!! ARM9 BIG ENDIAN MODE. VERY BAD. SHIT GONNA
    // ASPLODE NOW\n");
    if (val & (1 << 13))
      ExceptionBase = 0xFFFF0000;
    else
      ExceptionBase = 0x00000000;
  }
    return;

  case 0x200:  // TTBR (Translation Table Base Register)
    m_reg_ttbr = val;
    return;

  case 0x300:  // DACR (Domain Access Control Register)
    m_reg_dacr = val;
    return;

  case 0x500:  // DFSR (Data Fault Status Register)
    m_reg_dfsr = val;
    return;

  case 0x501:  // IFSR (Instruction Fault Status Register)
    m_reg_ifsr = val;
    return;

  case 0x600:  // FAR (Fault Address Register)
    m_reg_far = val;
    return;

  case 0x704:
  case 0x782:
    Halt(1);
    return;

  case 0x750:
    ICacheInvalidateAll();
    // Halt(255);
    return;
  case 0x751:
    ICacheInvalidateByAddr(val);
    // Halt(255);
    return;
  case 0x752:
    WARN_LOG_FMT(IOS_LLE, "CP15: ICACHE INVALIDATE WEIRD. {:08x}\n", val);
    // Halt(255);
    return;

  case 0x761:
    // printf("inval data cache %08X\n", val);
    return;
  case 0x762:
    // printf("inval data cache SI\n");
    return;

  case 0x7A1:
    // printf("flush data cache %08X\n", val);
    return;
  case 0x7A2:
    // printf("flush data cache SI\n");
    return;

  case 0x7A4:  // Drain write buffer
    return;

    // case 0x910:
    //     DTCMSetting = val & 0xFFFFF03E;
    //     UpdateDTCMSetting();
    //     return;

    // case 0x911:
    //     ITCMSetting = val & 0x0000003E;
    //     UpdateITCMSetting();
    //     return;

  case 0x870:  // Invalidate TLB
    return;

  case 0xD00:  // FSCE PID
    m_reg_fcse_pid = val;
    return;

  case 0xD01:  // Context ID
    m_reg_context_id = val;
    return;

  case 0xF00:
    // printf("cache debug index register %08X\n", val);
    return;

  case 0xF10:
    // printf("cache debug instruction tag %08X\n", val);
    return;

  case 0xF20:
    // printf("cache debug data tag %08X\n", val);
    return;

  case 0xF30:
    // printf("cache debug instruction cache %08X\n", val);
    return;

  case 0xF40:
    // printf("cache debug data cache %08X\n", val);
    return;
  }

  if ((id & 0xF00) == 0xF00)  // test/debug shit?
    return;

  if ((id & 0xF00) != 0x700)
    DEBUG_LOG_FMT(IOS_LLE, "unknown CP15 write op {:03x} {:08x}\n", id, val);
}

u32 ARMv5::CP15Read(u32 id) const
{
  // printf("CP15 read op %03X %08X\n", id, NDS::ARM9->R[15]);

  switch (id)
  {
  case 0x000:  // CPU ID
  case 0x003:
  case 0x004:
  case 0x005:
  case 0x006:
  case 0x007:
    return 0x41059461;

  case 0x001:  // cache type
    return 0x0F0D2112;

  case 0x100:  // control reg
    return CP15Control;

  case 0x200:  // TTBR (Translation Table Base Register)
    return m_reg_ttbr;

  case 0x300:  // DACR (Domain Access Control Register)
    return m_reg_dacr;

  case 0x500:  // DFSR (Data Fault Status Register)
    return m_reg_dfsr;

  case 0x501:  // IFSR (Instruction Fault Status Register)
    return m_reg_ifsr;

  case 0x600:  // FAR (Fault Address Register)
    return m_reg_far;

    // case 0x910:
    //     return DTCMSetting;
    // case 0x911:
    //     return ITCMSetting;

  case 0xD01:  // Context ID
    return m_reg_context_id;
  }

  if ((id & 0xF00) == 0xF00)  // test/debug shit?
    return 0;

  DEBUG_LOG_FMT(IOS_LLE, "unknown CP15 read op {:03x}\n", id);
  return 0;
}

// TCM are handled here.
// TODO: later on, handle PU, and maybe caches

u32 ARMv5::CodeRead32(u32 addr, bool branch)
{
#if 0
    /*if (branch || (!(addr & 0xFFF)))
    {
        if (!(PU_Map[addr>>12] & 0x04))
        {
            PrefetchAbort();
            return 0;
        }
    }*/

    CodeCycles = RegionCodeCycles;
    if (CodeCycles == 0xFF) // cached memory. hax
    {
        if (branch || !(addr & 0x1F))
            CodeCycles = kCodeCacheTiming;//ICacheLookup(addr);
        else
            CodeCycles = 1;

        //return *(u32*)&CurICacheLine[addr & 0x1C];
    }

    if (CodeMem.Mem) return *(u32*)&CodeMem.Mem[addr & CodeMem.Mask];
#endif

  if (addr & 3 || (CP15Control & 0x5 && !TranslateAddress(addr, false)))
  {
    m_reg_far = addr;
    PrefetchAbort();
    return 0;
  }

  return BusRead32(addr);
}

void ARMv5::DataRead8(u32 addr, u32* val)
{
  if (CP15Control & 0x5 && !TranslateAddress(addr, false))
  {
    m_reg_far = addr;
    DataAbort();
    return;
  }

  DataRegion = addr;

  *val = BusRead8(addr);
  // WARN_LOG_FMT(IOS_LLE, "DataRead8: {:08x} = {:02x}\n", addr, *val);
  // DataCycles = MemTimings[addr >> 12][1];
}

void ARMv5::DataRead16(u32 addr, u32* val)
{
  if (addr & 1 || !TranslateAddress(addr, false))
  {
    m_reg_far = addr;
    DataAbort();
    return;
  }

  DataRegion = addr;

  addr &= ~1;

  *val = BusRead16(addr);
  // WARN_LOG_FMT(IOS_LLE, "DataRead16: {:08x} = {:04x}\n", addr, *val);
  // DataCycles = MemTimings[addr >> 12][1];
}

void ARMv5::DataRead32(u32 addr, u32* val)
{
  if (addr & 3 || !TranslateAddress(addr, false))
  {
    m_reg_far = addr;
    DataAbort();
    return;
  }

  DataRegion = addr;

  addr &= ~3;

  *val = BusRead32(addr);
  // WARN_LOG_FMT(IOS_LLE, "DataRead32: {:08x} = {:08x}\n", addr, *val);
  // DataCycles = MemTimings[addr >> 12][2];
}

void ARMv5::DataRead32S(u32 addr, u32* val)
{
  addr &= ~3;

  *val = BusRead32(addr);
  // DataCycles += MemTimings[addr >> 12][3];
}

void ARMv5::DataWrite8(u32 addr, u8 val)
{
  if (!TranslateAddress(addr, true))
  {
    m_reg_far = addr;
    DataAbort();
    return;
  }

  DataRegion = addr;

  BusWrite8(addr, val);
  // DataCycles = MemTimings[addr >> 12][1];
}

void ARMv5::DataWrite16(u32 addr, u16 val)
{
  if (addr & 1 || !TranslateAddress(addr, true))
  {
    m_reg_far = addr;
    DataAbort();
    return;
  }

  DataRegion = addr;

  addr &= ~1;

  BusWrite16(addr, val);
  // DataCycles = MemTimings[addr >> 12][1];
}

void ARMv5::DataWrite32(u32 addr, u32 val)
{
  if (addr & 3 || !TranslateAddress(addr, true))
  {
    m_reg_far = addr;
    DataAbort();
    return;
  }

  DataRegion = addr;

  addr &= ~3;

  BusWrite32(addr, val);
  // DataCycles = MemTimings[addr >> 12][2];
}

void ARMv5::DataWrite32S(u32 addr, u32 val)
{
  addr &= ~3;

  BusWrite32(addr, val);
  // DataCycles += MemTimings[addr >> 12][3];
}

}  // namespace IOS::LLE
