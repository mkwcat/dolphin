// Copyright 2025 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include "Common/CommonTypes.h"

namespace Core
{
class System;
}
namespace MMIO
{
class Mapping;
}

namespace AHB
{

class AHBMemBridgeInterface
{
public:
  explicit AHBMemBridgeInterface(Core::System& system);
  AHBMemBridgeInterface(const AHBMemBridgeInterface&) = delete;
  AHBMemBridgeInterface(AHBMemBridgeInterface&&) = delete;
  AHBMemBridgeInterface& operator=(const AHBMemBridgeInterface&) = delete;
  AHBMemBridgeInterface& operator=(AHBMemBridgeInterface&&) = delete;
  ~AHBMemBridgeInterface();

  void Init();
  void Reset();
  void RegisterMMIO(MMIO::Mapping* mmio, u32 base);

private:
  u32 m_secddr;
  u32 m_secspl;
  u32 m_rdbi;
  u32 m_prefcfg;
  u32 m_intmsk;
  u32 m_intsts;
  u32 m_unk_40;
  u32 m_unk_44;
  u32 m_protddr[128];
  u32 m_protspl[128];

  Core::System& m_system;
};

}  // namespace AHB