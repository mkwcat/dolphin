// Copyright 2025 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "WiiKeys.h"
#include "Common/FileUtil.h"
#include "Common/IOFile.h"
#include "Common/Logging/Log.h"
#include "Common/MsgHandler.h"
#include "Common/Swap.h"

namespace IOS
{

WiiKeys::WiiKeys() : m_file{File::GetUserPath(D_WIIROOT_IDX) + "keys.bin", "r+b"}
{
  m_valid = false;

  if (!m_file)
  {
    WARN_LOG_FMT(IOS, "keys.bin could not be found. Default values will be used.");
    return;
  }

  if (!m_file.ReadBytes(&m_keys, sizeof(m_keys)))
  {
    ERROR_LOG_FMT(IOS, "Failed to read from keys.bin.");
    return;
  }

  m_valid = true;
}

bool WiiKeys::StoreEepromValue(u32 index, u16 value)
{
  if (index >= std::size(m_keys.eeprom))
  {
    PanicAlertFmt("EEPROM index out of bounds: {}. This should not happen.", index);
    return false;
  }

  m_keys.eeprom[index] = Common::swap16(value);

  u32 write_offset = offsetof(BootMiiKeyDump, eeprom) + index * sizeof(u16);
  if (!m_file.Seek(write_offset, File::SeekOrigin::Begin))
  {
    ERROR_LOG_FMT(IOS, "Failed to seek to EEPROM index {} in keys.bin.", index);
    return false;
  }

  if (!m_file.WriteBytes(&m_keys.eeprom[index], sizeof(u16)))
  {
    ERROR_LOG_FMT(IOS, "Failed to write to EEPROM index {} in keys.bin.", index);
    return false;
  }

  NOTICE_LOG_FMT(IOS, "Stored value {:02x} at EEPROM index {} in keys.bin.", value, index);
  return true;
}

}  // namespace IOS