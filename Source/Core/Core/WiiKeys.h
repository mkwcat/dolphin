// Copyright 2025 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include <array>
#include "Common/CommonTypes.h"
#include "Common/Crypto/ec.h"

namespace IOS
{

class WiiKeys
{
public:
#pragma pack(push, 1)
  /*
   * Structs for keys.bin taken from:
   *
   * mini - a Free Software replacement for the Nintendo/BroadOn IOS.
   * crypto hardware support
   *
   * Copyright (C) 2008, 2009 Haxx Enterprises <bushing@gmail.com>
   * Copyright (C) 2008, 2009 Sven Peter <svenpeter@gmail.com>
   * Copyright (C) 2008, 2009 Hector Martin "marcan" <marcan@marcansoft.com>
   *
   * # This code is licensed to you under the terms of the GNU GPL, version 2;
   * # see file COPYING or http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
   */
  struct BootMiiKeyDump
  {
    std::array<char, 256> creator;
    union
    {
      u32 otp[32];

      struct
      {
        std::array<u8, 20> boot1_hash;  // 0x100
        std::array<u8, 16> common_key;  // 0x114
        u32 ng_id;                      // 0x124
        union
        {
          struct
          {
            std::array<u8, 0x1e> ng_priv;  // 0x128
            std::array<u8, 0x12> pad1;
          };
          struct
          {
            std::array<u8, 0x1c> pad2;
            std::array<u8, 0x14> nand_hmac;  // 0x144
          };
        };
        std::array<u8, 16> nand_key;    // 0x158
        std::array<u8, 16> backup_key;  // 0x168
        u32 unk1;                       // 0x178
        u32 unk2;                       // 0x17C
      };
    };
    std::array<u8, 0x80> eeprom_pad;  // 0x180

    struct Counter
    {
      u8 boot2version;
      u8 unknown1;
      u8 unknown2;
      u8 pad;
      u32 update_tag;
      u16 checksum;
    };

    union
    {
      u16 eeprom[128];

      struct
      {
        u32 ms_id;                        // 0x200
        u32 ca_id;                        // 0x204
        u32 ng_key_id;                    // 0x208
        Common::ec::Signature ng_sig;     // 0x20c
        std::array<Counter, 2> counters;  // 0x248
        std::array<u8, 0x18> fill;        // 0x25c
        std::array<u8, 16> korean_key;    // 0x274
        std::array<u8, 0x74> pad3;        // 0x284
        std::array<u16, 2> prng_seed;     // 0x2F8
        std::array<u8, 4> pad4;           // 0x2FC
      };
    };

    std::array<u8, 0x100> crack_pad;  // 0x300
  };
  static_assert(sizeof(BootMiiKeyDump) == 0x400, "Wrong size");
#pragma pack(pop)

  WiiKeys();
  ~WiiKeys() = default;

  bool IsValid() const { return m_valid; }
  const BootMiiKeyDump& GetBackupMiiKeys() const { return m_keys; }

private:
  BootMiiKeyDump m_keys;
  bool m_valid;
};

}  // namespace IOS