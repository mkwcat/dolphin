// Copyright 2025 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include "Common/BitField.h"
#include "Common/CommonTypes.h"
#include "Core/CoreTiming.h"

namespace Core
{
class System;
}
namespace MMIO
{
class Mapping;
}

namespace NANDInterface
{

union NANDCtrlReg
{
  u32 Hex = 0;

  BitField<0, 12, u32> DATALEN;
  BitField<12, 1, u32> ECC;
  BitField<13, 1, u32> RD;
  BitField<14, 1, u32> WR;
  BitField<15, 1, u32> WAIT;
  BitField<16, 8, u32> COMMAND;
  BitField<24, 1, u32> A1;
  BitField<25, 1, u32> A2;
  BitField<26, 1, u32> A3;
  BitField<27, 1, u32> A4;
  BitField<28, 1, u32> A5;
  BitField<29, 1, u32> ERR;
  BitField<30, 1, u32> IRQ;
  BitField<31, 1, u32> EXEC;

  NANDCtrlReg() = default;
  explicit NANDCtrlReg(u32 hex) : Hex{hex} {}
};

union NANDConfigReg
{
  u32 Hex = 0;

  BitField<0, 8, u32> ATTR4;
  BitField<8, 8, u32> ATTR3;
  BitField<16, 7, u32> ATTR2;
  BitField<23, 4, u32> ATTR1;
  BitField<27, 1, u32> ENABLE;
  BitField<28, 4, u32> ATTR0;

  NANDConfigReg() = default;
  explicit NANDConfigReg(u32 hex) : Hex{hex} {}
};

union NANDAddr1Reg
{
  u32 Hex = 0;

  BitField<0, 8, u32> ADDR1;
  BitField<8, 8, u32> ADDR2;

  NANDAddr1Reg() = default;
  explicit NANDAddr1Reg(u32 hex) : Hex{hex} {}
};

union NANDAddr2Reg
{
  u32 Hex = 0;

  BitField<0, 8, u32> ADDR3;
  BitField<8, 8, u32> ADDR4;
  BitField<16, 8, u32> ADDR5;

  NANDAddr2Reg() = default;
  explicit NANDAddr2Reg(u32 hex) : Hex{hex} {}
};

class NANDInterfaceManager
{
public:
  explicit NANDInterfaceManager(Core::System& system);
  NANDInterfaceManager(const NANDInterfaceManager&) = delete;
  NANDInterfaceManager(NANDInterfaceManager&&) = delete;
  NANDInterfaceManager& operator=(const NANDInterfaceManager&) = delete;
  NANDInterfaceManager& operator=(NANDInterfaceManager&&) = delete;
  ~NANDInterfaceManager();

  void Init();
  void Reset();
  void RegisterMMIO(MMIO::Mapping* mmio, u32 base);
  void ExecuteCommand(NANDCtrlReg ctrl);

private:
  static void ExecuteCommandCallback(Core::System& system, u64 userdata, s64 cycles_late);

  NANDCtrlReg m_ctrl;
  NANDConfigReg m_config;
  NANDAddr1Reg m_addr1;
  NANDAddr2Reg m_addr2;
  u32 m_databuf_addr;
  u32 m_eccbuf_addr;

  Core::System& m_system;
  CoreTiming::EventType* m_event_handle_nand_command = nullptr;
};

}  // namespace NANDInterface