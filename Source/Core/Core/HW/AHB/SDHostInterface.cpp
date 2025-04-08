// Copyright 2025 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "Core/HW/AHB/SDHostInterface.h"

namespace SDHostInterface
{
// Internal hardware addresses
enum
{
  SD_DMA_ADDR = 0x70000,
  SD_BLK_SIZE = 0x70004,
  SD_BLK_CNT = 0x70006,
  SD_ARG = 0x70008,
  SD_XFER_MODE = 0x7000C,
  SD_CMD = 0x7000E,
  SD_RESP = 0x70010,
  SD_BUF_DATA_PORT = 0x70020,
  SD_PRESENT_STATE = 0x70024,
  SD_HOST_CTRL = 0x70028,
  SD_POWER_CTRL = 0x70029,
  SD_BLK_GAP_CTRL = 0x7002A,
  SD_WAKEUP_CTRL = 0x7002B,
  SD_CLK_CTRL = 0x7002C,
  SD_TIMEOUT_CTRL = 0x7002E,
  SD_SW_RESET = 0x7002F,
  SD_NORMAL_INT_STS = 0x70030,
  SD_ERROR_INT_STS = 0x70032,
  SD_NORMAL_INT_EN = 0x70034,
  SD_ERROR_INT_EN = 0x70036,
  SD_NORMAL_INT_SIG_EN = 0x70038,
  SD_ERROR_INT_SIG_EN = 0x7003A,
  SD_ACMD12_ERR_STS = 0x7003C,
  SD_HOST_CTRL_2 = 0x7003E,
  SD_CAPABILITIES_0 = 0x70040,
  SD_CAPABILITIES_1 = 0x70042,
  SD_CAPABILITIES_2 = 0x70044,
  SD_CAPABILITIES_3 = 0x70046,
};

SDHostInterfaceManager::SDHostInterfaceManager() = default;
SDHostInterfaceManager::~SDHostInterfaceManager() = default;

}  // namespace SDHostInterface