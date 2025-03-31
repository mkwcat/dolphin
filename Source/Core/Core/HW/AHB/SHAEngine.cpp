// Copyright 2025 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "Core/HW/AHB/SHAEngine.h"
#include "Common/Logging/Log.h"
#include "Core/HW/MMIO.h"
#include "Core/HW/MMIOHandlers.h"
#include "Core/System.h"

namespace SHAEngine
{
// Internal hardware addresses
enum
{
  SHA_CTRL = 0x00,
  SHA_SRC = 0x04,
  SHA_H0 = 0x08,
  SHA_H1 = 0x0C,
  SHA_H2 = 0x10,
  SHA_H3 = 0x14,
  SHA_H4 = 0x18,
};

SHAEngineInterface::SHAEngineInterface() = default;

SHAEngineInterface::~SHAEngineInterface() = default;

void SHAEngineInterface::Init()
{
  Reset();
}

void SHAEngineInterface::Reset()
{
  m_ctrl.Hex = 0x00000000;
  m_src = 0x00000000;
  m_hash[0] = 0x00000000;
  m_hash[1] = 0x00000000;
  m_hash[2] = 0x00000000;
  m_hash[3] = 0x00000000;
  m_hash[4] = 0x00000000;
}

void SHAEngineInterface::RegisterMMIO(MMIO::Mapping* mmio, u32 base)
{
  mmio->Register(base | SHA_CTRL, MMIO::DirectRead<u32>(&m_ctrl.Hex),
                 MMIO::DirectWrite<u32>(&m_ctrl.Hex));
  mmio->Register(base | SHA_SRC, MMIO::DirectRead<u32>(&m_src), MMIO::DirectWrite<u32>(&m_src));
  mmio->Register(base | SHA_H0, MMIO::DirectRead<u32>(&m_hash[0]),
                 MMIO::DirectWrite<u32>(&m_hash[0]));
  mmio->Register(base | SHA_H1, MMIO::DirectRead<u32>(&m_hash[1]),
                 MMIO::DirectWrite<u32>(&m_hash[1]));
  mmio->Register(base | SHA_H2, MMIO::DirectRead<u32>(&m_hash[2]),
                 MMIO::DirectWrite<u32>(&m_hash[2]));
  mmio->Register(base | SHA_H3, MMIO::DirectRead<u32>(&m_hash[3]),
                 MMIO::DirectWrite<u32>(&m_hash[3]));
  mmio->Register(base | SHA_H4, MMIO::DirectRead<u32>(&m_hash[4]),
                 MMIO::DirectWrite<u32>(&m_hash[4]));
}

}  // namespace SHAEngine