// Copyright 2025 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "Core/HW/AHB/NANDInterface.h"
#include "Common/Logging/Log.h"
#include "Core/CoreTiming.h"
#include "Core/HW/MMIO.h"
#include "Core/HW/Memmap.h"
#include "Core/HW/SystemTimers.h"
#include "Core/HW/WII_IPC.h"
#include "Core/System.h"

namespace NANDInterface
{
// Internal hardware addresses
enum
{
  NAND_CTRL = 0x00,
  NAND_CONFIG = 0x04,
  NAND_ADDR1 = 0x08,
  NAND_ADDR2 = 0x0C,
  NAND_DATABUF = 0x10,
  NAND_ECCBUF = 0x14,
  NAND_UNK18 = 0x18,
};

// NAND commands
enum
{
  NAND_CMD_RESET = 0xff,
  NAND_CMD_CHIPID = 0x90,
  NAND_CMD_GETSTATUS = 0x70,
  NAND_CMD_ERASE_PRE = 0x60,
  NAND_CMD_ERASE_POST = 0xd0,
  NAND_CMD_READ_PRE = 0x00,
  NAND_CMD_READ_POST = 0x30,
  NAND_CMD_WRITE_PRE = 0x80,
  NAND_CMD_WRITE_POST = 0x10,
};

NANDInterfaceManager::NANDInterfaceManager(Core::System& system) : m_system(system)
{
}

NANDInterfaceManager::~NANDInterfaceManager() = default;

void NANDInterfaceManager::Init()
{
  m_event_handle_nand_command =
      m_system.GetCoreTiming().RegisterEvent("HandleNANDCommand", ExecuteCommandCallback);
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
                 MMIO::ComplexWrite<u32>([this](Core::System& system, u32, u32 val) {
                   NANDCtrlReg ctrl(val);

                   if (ctrl.EXEC == 0)
                   {
                     // Reset the interface
                     Reset();
                     m_ctrl.Hex = val;
                     return;
                   }

                   m_ctrl.Hex = val;
                   m_ctrl.ERR = 0;
                   // Schedule a command to be executed
                   system.GetCoreTiming().ScheduleEvent(100_tbticks, m_event_handle_nand_command,
                                                        val, CoreTiming::FromThread::ANY);
                 }),
                 IOS::WiiIPC::CheckBusAccess);
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

  // Unknown register
  mmio->Register(base | NAND_UNK18, MMIO::Constant<u32>(0x00000000), MMIO::Nop<u32>(),
                 IOS::WiiIPC::CheckBusAccess);
}

void NANDInterfaceManager::ExecuteCommandCallback(Core::System& system, u64 userdata,
                                                  s64 cycles_late)
{
  system.GetNANDInterface().ExecuteCommand(NANDCtrlReg(userdata));
}

void NANDInterfaceManager::ExecuteCommand(NANDCtrlReg ctrl)
{
  auto& memory = m_system.GetMemory();

  switch (ctrl.COMMAND)
  {
  default:
    ERROR_LOG_FMT(NAND, "Unknown NAND command: {:#x}", ctrl.COMMAND);
    m_ctrl.ERR = 1;
    break;

  case NAND_CMD_RESET:
    INFO_LOG_FMT(NAND, "NAND Reset");
    break;

  case NAND_CMD_CHIPID:
    // Simulating Hynix HY27UF084G2M
    constexpr u8 HY27UF084G2M_ID[68] = {
        0xAD, 0xDC, 0x00, 0x00, 0xFF, 0x00, 0x30, 0xFE, 0xFE, 0x35, 0xFE, 0x80, 0x10, 0x10,
        0xFE, 0xFE, 0x85, 0x60, 0xFE, 0xD0, 0x05, 0xE0, 0x85, 0x70, 0xFE, 0x1F, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x1D, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x00,
        0x00, 0x06, 0x00, 0x00, 0x00, 0x05, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x07, 0x04, 0x3F, 0x3F, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    if (m_databuf_addr != 0xffffffff)
    {
      memory.CopyToEmu(m_databuf_addr, HY27UF084G2M_ID, sizeof(HY27UF084G2M_ID), true);
    }
    break;
  }

  m_ctrl.EXEC = 0;  // Clear EXEC bit to indicate command completion
  if (m_ctrl.IRQ)
  {
    // Trigger an interrupt if requested
    m_system.GetWiiIPC().TriggerIRQ(IOS::INT_CAUSE_NAND);
  }
}

}  // namespace NANDInterface
