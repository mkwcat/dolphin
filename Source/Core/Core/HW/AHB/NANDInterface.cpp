// Copyright 2025 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "Core/HW/AHB/NANDInterface.h"
#include "Common/Logging/Log.h"
#include "Core/HW/MMIO.h"
#include "Core/System.h"

namespace NANDInterface
{
// Internal hardware addresses
enum
{
  NAND_CTRL = 0x10000,
  NAND_CONFIG = 0x10004,
  NAND_ADDR1 = 0x10008,
  NAND_ADDR2 = 0x1000C,
  NAND_DATABUF = 0x10010,
  NAND_ECCBUF = 0x10014,
};

NANDInterfaceManager::NANDInterfaceManager() = default;

NANDInterfaceManager::~NANDInterfaceManager() = default;

void NANDInterfaceManager::Init()
{
  Reset();
}

void NANDInterfaceManager::Reset()
{
  m_ctrl.Hex = 0x00000000;
  m_config.Hex = 0x00000000;
  m_addr1.Hex = 0x00000000;
  m_addr2.Hex = 0x00000000;
  m_databuf_addr = 0x00000000;
  m_eccbuf_addr = 0x00000000;
}

void NANDInterfaceManager::RegisterMMIO(MMIO::Mapping* mmio, u32 base)
{
  mmio->Register(base | NAND_CTRL, MMIO::DirectRead<u32>(&m_ctrl.Hex),
                 MMIO::DirectWrite<u32>(&m_ctrl.Hex));
  mmio->Register(base | NAND_CONFIG, MMIO::DirectRead<u32>(&m_config.Hex),
                 MMIO::DirectWrite<u32>(&m_config.Hex));
  mmio->Register(base | NAND_ADDR1, MMIO::DirectRead<u32>(&m_addr1.Hex),
                 MMIO::DirectWrite<u32>(&m_addr1.Hex));
  mmio->Register(base | NAND_ADDR2, MMIO::DirectRead<u32>(&m_addr2.Hex),
                 MMIO::DirectWrite<u32>(&m_addr2.Hex));
  mmio->Register(base | NAND_DATABUF, MMIO::DirectRead<u32>(&m_databuf_addr),
                 MMIO::DirectWrite<u32>(&m_databuf_addr));
  mmio->Register(base | NAND_ECCBUF, MMIO::DirectRead<u32>(&m_eccbuf_addr),
                 MMIO::DirectWrite<u32>(&m_eccbuf_addr));
}

}  // namespace NANDInterface
