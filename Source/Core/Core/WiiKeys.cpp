// Copyright 2025 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "WiiKeys.h"
#include "Common/FileUtil.h"
#include "Common/IOFile.h"
#include "Common/Logging/Log.h"

namespace IOS
{

WiiKeys::WiiKeys()
{
  m_valid = false;

  File::IOFile file{File::GetUserPath(D_WIIROOT_IDX) + "keys.bin", "rb"};
  if (!file)
  {
    WARN_LOG_FMT(IOS, "keys.bin could not be found. Default values will be used.");
    return;
  }

  if (!file.ReadBytes(&m_keys, sizeof(m_keys)))
  {
    ERROR_LOG_FMT(IOS, "Failed to read from keys.bin.");
    return;
  }

  m_valid = true;
}

}  // namespace IOS