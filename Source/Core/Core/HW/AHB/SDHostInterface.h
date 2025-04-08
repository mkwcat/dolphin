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

namespace SDHostInterface
{

class SDHostInterfaceManager
{
public:
  explicit SDHostInterfaceManager();
  SDHostInterfaceManager(const SDHostInterfaceManager&) = delete;
  SDHostInterfaceManager(SDHostInterfaceManager&&) = delete;
  SDHostInterfaceManager& operator=(const SDHostInterfaceManager&) = delete;
  SDHostInterfaceManager& operator=(SDHostInterfaceManager&&) = delete;
  ~SDHostInterfaceManager();

  void Init(Core::System& system);
  void Reset();
  void RegisterMMIO(MMIO::Mapping* mmio, u32 base);
};

}  // namespace SDHostInterface
