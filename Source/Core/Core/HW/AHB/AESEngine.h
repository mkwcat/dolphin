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

namespace AESEngine
{

union AESControlReg
{
  u32 Hex = 0;

  BitField<0, 12, u32> BLOCKS;
  BitField<12, 1, u32> IV;
  BitField<27, 1, u32> DEC;
  BitField<28, 1, u32> ENA;
  BitField<29, 1, u32> ERR;
  BitField<30, 1, u32> IRQ;
  BitField<31, 1, u32> EXEC;

  AESControlReg() = default;
  explicit AESControlReg(u32 hex) : Hex{hex} {}
};

class AESEngineInterface
{
public:
  explicit AESEngineInterface();
  AESEngineInterface(const AESEngineInterface&) = delete;
  AESEngineInterface(AESEngineInterface&&) = delete;
  AESEngineInterface& operator=(const AESEngineInterface&) = delete;
  AESEngineInterface& operator=(AESEngineInterface&&) = delete;
  ~AESEngineInterface();

  void Init(Core::System& system);
  void Reset();
  void RegisterMMIO(MMIO::Mapping* mmio, u32 base);

private:
  static void ExecuteCommandCallback(Core::System& system, u64 userdata, s64 cycles_late);

  AESControlReg m_ctrl;
  u32 m_src;
  u32 m_dest;
  u32 m_key[4];
  u32 m_iv[4];
  u8 m_prev_iv[16];

  CoreTiming::EventType* m_event_handle_aes_command = nullptr;
};

}  // namespace AESEngine