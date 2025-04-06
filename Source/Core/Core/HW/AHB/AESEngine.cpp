// Copyright 2025 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "Core/HW/AHB/AESEngine.h"

#include "Common/Crypto/AES.h"
#include "Common/Logging/Log.h"
#include "Core/HW/MMIO.h"
#include "Core/HW/MMIOHandlers.h"
#include "Core/HW/Memmap.h"
#include "Core/HW/SystemTimers.h"
#include "Core/HW/WII_IPC.h"
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

void AESEngineInterface::Init(Core::System& system)
{
  m_event_handle_aes_command =
      system.GetCoreTiming().RegisterEvent("HandleAESCommand", ExecuteCommandCallback);
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
  m_prev_iv[0] = 0x00000000;
  m_prev_iv[1] = 0x00000000;
  m_prev_iv[2] = 0x00000000;
  m_prev_iv[3] = 0x00000000;
}

void AESEngineInterface::RegisterMMIO(MMIO::Mapping* mmio, u32 base)
{
  mmio->Register(base | AES_CTRL, MMIO::DirectRead<u32>(&m_ctrl.Hex),
                 MMIO::ComplexWrite<u32>([this](Core::System& system, u32, u32 val) {
                   AESControlReg ctrl(val);

                   if (ctrl.EXEC == 0)
                   {
                     // Reset interface
                     m_ctrl.EXEC = 0;
                     m_ctrl.Hex &= ctrl.Hex;
                     return;
                   }

                   m_ctrl.Hex = val;
                   m_ctrl.ERR = 0;
                   // Schedule a command to be executed
                   // TODO: Use a different thread
                   system.GetCoreTiming().ScheduleEvent(100_tbticks, m_event_handle_aes_command,
                                                        val, CoreTiming::FromThread::ANY);
                 }),
                 IOS::WiiIPC::CheckBusAccess);

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

void AESEngineInterface::ExecuteCommandCallback(Core::System& system, u64 userdata, s64 cycles_late)
{
  auto& aes = system.GetAESEngine();
  auto& memory = system.GetMemory();
  bool err = false;

  AESControlReg ctrl = AESControlReg(userdata);

  std::array<u8, 0x10> key = {};

  for (u32 i = 0; i < 4; ++i)
  {
    key[i * 4] = aes.m_key[i] >> 24;
    key[i * 4 + 1] = aes.m_key[i] >> 16;
    key[i * 4 + 2] = aes.m_key[i] >> 8;
    key[i * 4 + 3] = aes.m_key[i];

    if (!ctrl.IV)
    {
      aes.m_prev_iv[i * 4] = aes.m_iv[i] >> 24;
      aes.m_prev_iv[i * 4 + 1] = aes.m_iv[i] >> 16;
      aes.m_prev_iv[i * 4 + 2] = aes.m_iv[i] >> 8;
      aes.m_prev_iv[i * 4 + 3] = aes.m_iv[i];
    }
  }

  auto context = ctrl.DEC ? Common::AES::CreateContextDecrypt(key.data()) :
                            Common::AES::CreateContextEncrypt(key.data());

  for (u32 i = 0; !err && i < ctrl.BLOCKS + 1; i++)
  {
    std::array<u8, Common::AES::Context::BLOCK_SIZE> data;
    memory.CopyFromEmu(data.data(), aes.m_src, Common::AES::Context::BLOCK_SIZE, true);

    if (ctrl.ENA)
    {
      err = !context->Crypt(aes.m_prev_iv, aes.m_prev_iv, data.data(), data.data(), data.size());
    }

    if (!err)
      memory.CopyToEmu(aes.m_dest, data.data(), data.size(), true);
    aes.m_src += Common::AES::Context::BLOCK_SIZE;
    aes.m_dest += Common::AES::Context::BLOCK_SIZE;
  }

  aes.m_ctrl.ERR = err ? 1 : 0;
  aes.m_ctrl.EXEC = 0;  // Clear EXEC bit to indicate command completion
  if (aes.m_ctrl.IRQ)
  {
    // Trigger an interrupt if requested
    system.GetWiiIPC().TriggerIRQ(IOS::INT_CAUSE_AES);
  }
}

}  // namespace AESEngine