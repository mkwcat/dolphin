// Copyright 2025 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include "Common/BitField.h"
#include "Common/CommonTypes.h"

namespace Core
{
class System;
}
namespace MMIO
{
class Mapping;
}

namespace SHAEngine
{

union SHAControlReg
{
  u32 Hex = 0;

  BitField<0, 10, u32> BLOCKS;
  BitField<29, 1, u32> ERR;
  BitField<30, 1, u32> IRQ;
  BitField<31, 1, u32> EXEC;

  SHAControlReg() = default;
  explicit SHAControlReg(u32 hex) : Hex{hex} {}
};

class SHAEngineInterface
{
public:
  explicit SHAEngineInterface();
  SHAEngineInterface(const SHAEngineInterface&) = delete;
  SHAEngineInterface(SHAEngineInterface&&) = delete;
  SHAEngineInterface& operator=(const SHAEngineInterface&) = delete;
  SHAEngineInterface& operator=(SHAEngineInterface&&) = delete;
  ~SHAEngineInterface();

  void Init();
  void Reset();
  void RegisterMMIO(MMIO::Mapping* mmio, u32 base);

private:
  SHAControlReg m_ctrl;
  u32 m_src;
  u32 m_hash[5];
};

}  // namespace SHAEngine