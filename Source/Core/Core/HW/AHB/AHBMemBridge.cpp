// Copyright 2025 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "Core/HW/AHB/AHBMemBridge.h"
#include "Core/HW/MMIO.h"
#include "Core/System.h"

namespace AHB
{
// Internal hardware addresses
enum
{
  AHM_SECDDR = 0xB0000,
  AHM_SECSPL = 0xB0004,
  AHM_RDBI = 0xB0008,
  AHM_PREFCFG = 0xB0010,
  AHM_INTMSK = 0xB0020,
  AHM_INTSTS = 0xB0030,
  AHM_UNK_40 = 0xB0040,
  AHM_UNK_44 = 0xB0044,
  AHM_PROTDDR = 0xB0100,
  AHM_PROTSPL = 0xB0500,
};

AHBMemBridgeInterface::AHBMemBridgeInterface(Core::System& system) : m_system(system)
{
}

AHBMemBridgeInterface::~AHBMemBridgeInterface() = default;

void AHBMemBridgeInterface::Init()
{
  Reset();
}

void AHBMemBridgeInterface::Reset()
{
  // Not sure what the values usually are on boot; these are just from a random snapshot running
  // under IOS.

  m_secddr = 0x08008000;
  m_secspl = 0x01001000;
  m_rdbi = 0x0000ffff;
  m_prefcfg = 0x00000000;
  m_intmsk = 0x00000000;
  m_intsts = 0x00060000;
  m_unk_40 = 0xFFFFFF12;
  m_unk_44 = 0x00000000;

  for (u32 i = 0; i < 128; i++)
  {
    m_protddr[i] = 0x80000000;
    m_protspl[i] = 0x80000000;
  }
}

void AHBMemBridgeInterface::RegisterMMIO(MMIO::Mapping* mmio, u32 base)
{
  mmio->Register(base | AHM_SECDDR, MMIO::DirectRead<u32>(&m_secddr),
                 MMIO::DirectWrite<u32>(&m_secddr));
  mmio->Register(base | AHM_SECSPL, MMIO::DirectRead<u32>(&m_secspl),
                 MMIO::DirectWrite<u32>(&m_secspl));
  mmio->Register(base | AHM_RDBI, MMIO::DirectRead<u32>(&m_rdbi), MMIO::DirectWrite<u32>(&m_rdbi));
  mmio->Register(base | AHM_PREFCFG, MMIO::DirectRead<u32>(&m_prefcfg),
                 MMIO::DirectWrite<u32>(&m_prefcfg));
  mmio->Register(base | AHM_INTMSK, MMIO::DirectRead<u32>(&m_intmsk),
                 MMIO::DirectWrite<u32>(&m_intmsk));
  mmio->Register(base | AHM_INTSTS, MMIO::DirectRead<u32>(&m_intsts),
                 MMIO::DirectWrite<u32>(&m_intsts));
  mmio->Register(base | AHM_UNK_40, MMIO::DirectRead<u32>(&m_unk_40),
                 MMIO::DirectWrite<u32>(&m_unk_40));
  mmio->Register(base | AHM_UNK_44, MMIO::DirectRead<u32>(&m_unk_44),
                 MMIO::DirectWrite<u32>(&m_unk_44));
  for (u32 i = 0; i < 128; i++)
  {
    mmio->Register(base | (AHM_PROTDDR + i * 4), MMIO::DirectRead<u32>(&m_protddr[i]),
                   MMIO::DirectWrite<u32>(&m_protddr[i]));
    mmio->Register(base | (AHM_PROTSPL + i * 4), MMIO::DirectRead<u32>(&m_protspl[i]),
                   MMIO::DirectWrite<u32>(&m_protspl[i]));
  }
}

}  // namespace AHB