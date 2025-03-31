// Copyright 2025 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "Core/HW/AHB/AESEngine.h"
#include "Common/Logging/Log.h"
#include "Core/HW/MMIO.h"
#include "Core/HW/MMIOHandlers.h"
#include "Core/System.h"

namespace AESEngine
{
// Internal hardware addresses
enum
{
  AES_CTRL = 0x00,
  AES_SRC = 0x04,
  AES_DEST = 0x08,
  AES_KEY = 0x0C,
  AES_IV = 0x10,
};

AESEngineInterface::AESEngineInterface() = default;

AESEngineInterface::~AESEngineInterface() = default;

void AESEngineInterface::Init()
{
  Reset();
}

void AESEngineInterface::Reset()
{
  m_ctrl.Hex = 0x00000000;
  m_src = 0x00000000;
  m_dest = 0x00000000;
  m_key[0] = 0x00000000;
  m_key[1] = 0x00000000;
  m_key[2] = 0x00000000;
  m_key[3] = 0x00000000;
  m_iv[0] = 0x00000000;
  m_iv[1] = 0x00000000;
  m_iv[2] = 0x00000000;
  m_iv[3] = 0x00000000;
}

void AESEngineInterface::RegisterMMIO(MMIO::Mapping* mmio, u32 base)
{
  mmio->Register(base | AES_CTRL, MMIO::DirectRead<u32>(&m_ctrl.Hex),
                 MMIO::DirectWrite<u32>(&m_ctrl.Hex));
  mmio->Register(base | AES_SRC, MMIO::DirectRead<u32>(&m_src), MMIO::DirectWrite<u32>(&m_src));
  mmio->Register(base | AES_DEST, MMIO::DirectRead<u32>(&m_dest), MMIO::DirectWrite<u32>(&m_dest));
  mmio->Register(base | AES_KEY, MMIO::DirectRead<u32>(&m_key[3]),
                 MMIO::ComplexWrite<u32>([](Core::System& system, u32, u32 val) {
                   auto& aes = system.GetAESEngine();
                   aes.m_key[0] = aes.m_key[1];
                   aes.m_key[1] = aes.m_key[2];
                   aes.m_key[2] = aes.m_key[3];
                   aes.m_key[3] = val;
                 }));
  mmio->Register(base | AES_IV, MMIO::DirectRead<u32>(&m_iv[3]),
                 MMIO::ComplexWrite<u32>([](Core::System& system, u32, u32 val) {
                   auto& aes = system.GetAESEngine();
                   aes.m_iv[0] = aes.m_iv[1];
                   aes.m_iv[1] = aes.m_iv[2];
                   aes.m_iv[2] = aes.m_iv[3];
                   aes.m_iv[3] = val;
                 }));
}

}  // namespace AESEngine