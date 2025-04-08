// Copyright 2025 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "Core/HW/AHB/NANDInterface.h"

#include "Common/FileUtil.h"
#include "Common/IOFile.h"
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
  NAND_BANK = 0x18,
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
  NAND_CMD_RAND_IN = 0x85,
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

  m_stored_addr1.Hex = 0x00000000;
  m_stored_addr2.Hex = 0x00000000;
}

void NANDInterfaceManager::RegisterMMIO(MMIO::Mapping* mmio, u32 base)
{
  mmio->Register(base | NAND_CTRL, MMIO::DirectRead<u32>(&m_ctrl.Hex),
                 MMIO::ComplexWrite<u32>([this](Core::System& system, u32, u32 val) {
                   NANDCtrlReg ctrl(val);

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

  // For compatibility with Wii U (vWii) IOS, 1 is vWii NAND bank.
  mmio->Register(base | NAND_BANK, MMIO::Constant<u32>(0x00000001), MMIO::Nop<u32>(),
                 IOS::WiiIPC::CheckBusAccess);
}

bool NANDInterfaceManager::OpenNANDFile()
{
  if (m_nand.IsOpen())
  {
    return true;
  }

  m_nand.Open(File::GetUserPath(D_WIIROOT_IDX) + "nand.bin", "r+b");
  return !!m_nand;
}

// Wii NAND ECC code adapted from segher's unecc.c
static u8 Parity(u8 x)
{
  u8 y = 0;

  while (x)
  {
    y ^= (x & 1);
    x >>= 1;
  }

  return y;
}

static std::array<u8, 16> CalculateECC(const std::array<u8, NAND_FULLPAGE_SIZE>& data)
{
  u8 a[12][2];
  u32 a0, a1;
  u8 x;

  std::array<u8, 16> ecc;

  for (int k = 0; k < 4; ++k)
  {
    std::memset(a, 0, sizeof(a));
    for (int i = 0; i < 512; ++i)
    {
      x = data[k * 512 + i];
      for (int j = 0; j < 9; j++)
        a[3 + j][(i >> j) & 1] ^= x;
    }

    x = a[3][0] ^ a[3][1];
    a[0][0] = x & 0x55;
    a[0][1] = x & 0xaa;
    a[1][0] = x & 0x33;
    a[1][1] = x & 0xcc;
    a[2][0] = x & 0x0f;
    a[2][1] = x & 0xf0;

    for (int j = 0; j < 12; j++)
    {
      a[j][0] = Parity(a[j][0]);
      a[j][1] = Parity(a[j][1]);
    }
    a0 = a1 = 0;

    for (int j = 0; j < 12; j++)
    {
      a0 |= a[j][0] << j;
      a1 |= a[j][1] << j;
    }
    ecc[0 + 4 * k] = a0;
    ecc[1 + 4 * k] = a0 >> 8;
    ecc[2 + 4 * k] = a1;
    ecc[3 + 4 * k] = a1 >> 8;
  }

  return ecc;
}

bool NANDInterfaceManager::ReadPage(std::array<u8, NAND_FULLPAGE_SIZE>& data, u32 row, u32 column)
{
  const u32 offset = row * NAND_FULLPAGE_SIZE;
  if (!m_nand.Seek(offset, File::SeekOrigin::Begin))
  {
    ERROR_LOG_FMT(NAND, "Failed to seek to page at offset 0x{:x}", offset);
    return false;
  }

  if (!m_nand.ReadBytes(data.data(), NAND_FULLPAGE_SIZE))
  {
    ERROR_LOG_FMT(NAND, "Failed to read page at offset 0x{:x}", offset);
    return false;
  }

  INFO_LOG_FMT(NAND, "Read page at offset 0x{:x}, column 0x{:x}", offset, column);

  return true;
}

bool NANDInterfaceManager::WritePage(std::array<u8, NAND_FULLPAGE_SIZE>& data, u32 row, u32 column,
                                     u32 size)
{
  const u32 offset = row * NAND_FULLPAGE_SIZE + column;
  if (!m_nand.Seek(offset, File::SeekOrigin::Begin))
  {
    ERROR_LOG_FMT(NAND, "Failed to seek to page at offset 0x{:x}", offset);
    return false;
  }

  if (!m_nand.WriteBytes(data.data(), size))
  {
    ERROR_LOG_FMT(NAND, "Failed to write page at offset 0x{:x}", offset);
    return false;
  }

  INFO_LOG_FMT(NAND, "Wrote page at offset 0x{:x}, column 0x{:x}", offset, column);

  return true;
}

bool NANDInterfaceManager::ErasePages(u32 page, u32 count)
{
  const u32 offset = page * NAND_FULLPAGE_SIZE;
  if (!m_nand.Seek(offset, File::SeekOrigin::Begin))
  {
    ERROR_LOG_FMT(NAND, "Failed to seek to page at offset 0x{:x}", offset);
    return false;
  }

  std::array<u8, NAND_FULLPAGE_SIZE> data = {};
  std::memset(data.data(), 0xFF, NAND_FULLPAGE_SIZE);

  for (u32 i = 0; i < count; ++i)
  {
    if (!m_nand.WriteBytes(data.data(), NAND_FULLPAGE_SIZE))
    {
      ERROR_LOG_FMT(NAND, "Failed to erase page at offset 0x{:x}", offset + i * NAND_FULLPAGE_SIZE);
      return false;
    }
  }

  INFO_LOG_FMT(NAND, "Erased {} pages starting from offset 0x{:x}", count, offset);

  return true;
}

void NANDInterfaceManager::ExecuteCommandCallback(Core::System& system, u64 userdata,
                                                  s64 cycles_late)
{
  auto& nand = system.GetNANDInterface();
  std::array<u8, NAND_FULLPAGE_SIZE> data = {};

  NANDCtrlReg ctrl = NANDCtrlReg(userdata);
  if (ctrl.WR && ctrl.DATALEN > 0)
  {
    // Update data with the data to be written
    auto& memory = nand.m_system.GetMemory();

    memory.CopyFromEmu(data.data(), nand.m_databuf_addr,
                       std::min<size_t>(ctrl.DATALEN, NAND_PAGE_SIZE), true);

    if (ctrl.DATALEN > NAND_PAGE_SIZE)
    {
      // Copy spare data too
      memory.CopyFromEmu(data.data() + NAND_PAGE_SIZE, nand.m_eccbuf_addr,
                         std::min<size_t>(ctrl.DATALEN - NAND_PAGE_SIZE, NAND_SPARE_SIZE), true);
    }
  }

  bool err = false;
  if (!nand.OpenNANDFile())
  {
    ERROR_LOG_FMT(NAND, "Failed to open NAND file");
    err = true;
  }
  else
  {
    err = !system.GetNANDInterface().ExecuteCommand(data, ctrl);
  }

  if (!err && ctrl.RD && ctrl.DATALEN > 0)
  {
    auto& memory = nand.m_system.GetMemory();

    memory.CopyToEmu(nand.m_databuf_addr, data.data() + nand.m_stored_addr1.Hex,
                     std::min<size_t>(ctrl.DATALEN, NAND_PAGE_SIZE), true);

    if (ctrl.DATALEN > NAND_PAGE_SIZE)
    {
      // Copy spare data too
      memory.CopyToEmu(nand.m_eccbuf_addr, data.data() + nand.m_stored_addr1.Hex + NAND_PAGE_SIZE,
                       std::min<size_t>(ctrl.DATALEN - NAND_PAGE_SIZE, NAND_SPARE_SIZE), true);
    }
  }

  if (ctrl.ECC)
  {
    auto& memory = nand.m_system.GetMemory();

    // Emulate hardware ECC calculation
    // TODO: Does it always write here?
    memory.CopyToEmu(nand.m_eccbuf_addr + NAND_SPARE_SIZE, CalculateECC(data).data(), 16, true);
  }

  nand.m_ctrl.ERR = err ? 1 : 0;
  nand.m_ctrl.EXEC = 0;  // Clear EXEC bit to indicate command completion
  if (nand.m_ctrl.IRQ)
  {
    // Trigger an interrupt if requested
    system.GetWiiIPC().TriggerIRQ(IOS::INT_CAUSE_NAND);
  }
}

bool NANDInterfaceManager::ExecuteCommand(std::array<u8, NAND_FULLPAGE_SIZE>& data,
                                          NANDCtrlReg ctrl)
{
  // Update stored address values
  if (ctrl.COMMAND != NAND_CMD_RAND_IN)
  {
    if (ctrl.A1)
      m_stored_addr1.ADDR1 = m_addr1.ADDR1.Value();
    if (ctrl.A2)
      m_stored_addr1.ADDR2 = m_addr1.ADDR2.Value();
  }
  if (ctrl.A3)
    m_stored_addr2.ADDR3 = m_addr2.ADDR3.Value();
  if (ctrl.A4)
    m_stored_addr2.ADDR4 = m_addr2.ADDR4.Value();
  if (ctrl.A5)
    m_stored_addr2.ADDR5 = m_addr2.ADDR5.Value();

  auto& memory = m_system.GetMemory();

  switch (ctrl.COMMAND)
  {
  default:
    ERROR_LOG_FMT(NAND, "Unknown NAND command: {:#x}", ctrl.COMMAND);
    return false;

  case NAND_CMD_RESET:
    INFO_LOG_FMT(NAND, "NAND Reset");
    m_stored_addr1.Hex = 0x00000000;
    m_stored_addr2.Hex = 0x00000000;
    return true;

  case NAND_CMD_CHIPID:
  {
    // Simulating Hynix HY27UF084G2M
    constexpr u8 HY27UF084G2M_ID[68] = {0xAD, 0xDC, 0x00, 0x00};

    memory.CopyToEmu(m_databuf_addr, HY27UF084G2M_ID, sizeof(HY27UF084G2M_ID), true);
    return true;
  }

  case NAND_CMD_READ_PRE:
    return true;

  case NAND_CMD_READ_POST:
    if (ctrl.RD && m_stored_addr1.Hex + ctrl.DATALEN > NAND_FULLPAGE_SIZE)
    {
      ERROR_LOG_FMT(NAND, "Read out of bounds: 0x{:x} + 0x{:x} > 0x{:x}", m_stored_addr1.Hex,
                    ctrl.DATALEN, NAND_FULLPAGE_SIZE);
      return false;
    }

    return ReadPage(data, m_stored_addr2.Hex, m_stored_addr1.Hex);

  case NAND_CMD_WRITE_PRE:
    return true;

  case NAND_CMD_RAND_IN:
  case NAND_CMD_WRITE_POST:
    if (ctrl.WR && m_stored_addr1.Hex + ctrl.DATALEN > NAND_FULLPAGE_SIZE)
    {
      ERROR_LOG_FMT(NAND, "Write out of bounds: 0x{:x} + 0x{:x} > 0x{:x}", m_stored_addr1.Hex,
                    ctrl.DATALEN, NAND_FULLPAGE_SIZE);
      return false;
    }

    return WritePage(data, m_stored_addr2.Hex, m_stored_addr1.Hex, ctrl.DATALEN);

  case NAND_CMD_ERASE_PRE:
    return true;

  case NAND_CMD_ERASE_POST:
    return ErasePages(m_stored_addr2.Hex, NAND_PAGE_PER_BLOCK);

  case NAND_CMD_GETSTATUS:
  {
    const u8 status = 192;
    memory.CopyToEmu(m_databuf_addr, &status, sizeof(status), true);
    return true;
  }
  }
}

}  // namespace NANDInterface
