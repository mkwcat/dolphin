// Copyright 2025 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "Core/HW/AHB/SHAEngine.h"

#include <mbedtls/sha1.h>

#include "Common/Logging/Log.h"
#include "Core/HW/MMIO.h"
#include "Core/HW/MMIOHandlers.h"
#include "Core/HW/Memmap.h"
#include "Core/HW/SystemTimers.h"
#include "Core/HW/WII_IPC.h"
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

void SHAEngineInterface::Init(Core::System& system)
{
  m_event_handle_sha_command =
      system.GetCoreTiming().RegisterEvent("HandleSHACommand", ExecuteCommandCallback);
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
                 MMIO::ComplexWrite<u32>([this](Core::System& system, u32, u32 val) {
                   SHAControlReg ctrl(val);

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
                   system.GetCoreTiming().ScheduleEvent(100_tbticks, m_event_handle_sha_command,
                                                        val, CoreTiming::FromThread::ANY);
                 }),
                 IOS::WiiIPC::CheckBusAccess);

  mmio->Register(base | SHA_SRC, MMIO::DirectRead<u32>(&m_src), MMIO::DirectWrite<u32>(&m_src),
                 IOS::WiiIPC::CheckBusAccess);
  mmio->Register(base | SHA_H0, MMIO::DirectRead<u32>(&m_hash[0]),
                 MMIO::DirectWrite<u32>(&m_hash[0]), IOS::WiiIPC::CheckBusAccess);
  mmio->Register(base | SHA_H1, MMIO::DirectRead<u32>(&m_hash[1]),
                 MMIO::DirectWrite<u32>(&m_hash[1]), IOS::WiiIPC::CheckBusAccess);
  mmio->Register(base | SHA_H2, MMIO::DirectRead<u32>(&m_hash[2]),
                 MMIO::DirectWrite<u32>(&m_hash[2]), IOS::WiiIPC::CheckBusAccess);
  mmio->Register(base | SHA_H3, MMIO::DirectRead<u32>(&m_hash[3]),
                 MMIO::DirectWrite<u32>(&m_hash[3]), IOS::WiiIPC::CheckBusAccess);
  mmio->Register(base | SHA_H4, MMIO::DirectRead<u32>(&m_hash[4]),
                 MMIO::DirectWrite<u32>(&m_hash[4]), IOS::WiiIPC::CheckBusAccess);
}

void SHAEngineInterface::ExecuteCommandCallback(Core::System& system, u64 userdata, s64 cycles_late)
{
  auto& sha = system.GetSHAEngine();
  auto& memory = system.GetMemory();
  bool err = false;

  SHAControlReg ctrl = SHAControlReg(userdata);

  mbedtls_sha1_context context = {};
  for (u32 i = 0; i < 5; i++)
    context.state[i] = sha.m_hash[i];

  for (u32 i = 0; !err && i < ctrl.BLOCKS + 1; i++)
  {
    u8 data[SHA_BLOCK_SIZE];
    memory.CopyFromEmu(data, sha.m_src, SHA_BLOCK_SIZE, true);
    err = !!mbedtls_internal_sha1_process(&context, data);
    sha.m_src += SHA_BLOCK_SIZE;

    if (!err)
      for (u32 j = 0; j < 5; j++)
        sha.m_hash[j] = context.state[j];
  }

  sha.m_ctrl.ERR = err ? 1 : 0;
  sha.m_ctrl.EXEC = 0;  // Clear EXEC bit to indicate command completion
  if (sha.m_ctrl.IRQ)
  {
    // Trigger an interrupt if requested
    system.GetWiiIPC().TriggerIRQ(IOS::INT_CAUSE_SHA1);
  }
}

}  // namespace SHAEngine