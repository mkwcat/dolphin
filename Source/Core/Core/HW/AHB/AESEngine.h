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

namespace AESEngine
{

union AESControlReg
{
  u32 Hex = 0;

  BitField<0, 12, u32> BLOCKS;
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

  void Init();
  void Reset();
  void RegisterMMIO(MMIO::Mapping* mmio, u32 base);

private:
  AESControlReg m_ctrl;
  u32 m_src;
  u32 m_dest;
  u32 m_key[4];
  u32 m_iv[4];
};

}  // namespace AESEngine