// Copyright 2008 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include "Common/BitUtils.h"
#include "Common/CommonTypes.h"
#include "Common/Flag.h"

class PointerWrap;
namespace Core
{
class System;
}
namespace CoreTiming
{
struct EventType;
}
namespace MMIO
{
class Mapping;
}

namespace IOS
{
enum StarletInterruptCause
{
  INT_CAUSE_NONE = 0x0,

  INT_CAUSE_TIMER = 0x1,
  INT_CAUSE_NAND = 0x2,
  INT_CAUSE_AES = 0x4,
  INT_CAUSE_SHA1 = 0x8,
  INT_CAUSE_EHCI = 0x10,
  INT_CAUSE_OHCI0 = 0x20,
  INT_CAUSE_OHCI1 = 0x40,
  INT_CAUSE_SD = 0x80,
  INT_CAUSE_WIFI = 0x100,

  INT_CAUSE_GPIO_BROADWAY = 0x400,
  INT_CAUSE_GPIO_STARLET = 0x800,

  INT_CAUSE_RST_BUTTON = 0x40000,

  INT_CAUSE_IPC_BROADWAY = 0x40000000,
  INT_CAUSE_IPC_STARLET = 0x80000000
};

enum class GPIO : u32
{
  POWER = 0x1,
  SHUTDOWN = 0x2,
  FAN = 0x4,
  DC_DC = 0x8,
  DI_SPIN = 0x10,
  SLOT_LED = 0x20,
  EJECT_BTN = 0x40,
  SLOT_IN = 0x80,
  SENSOR_BAR = 0x100,
  DO_EJECT = 0x200,
  EEP_CS = 0x400,
  EEP_CLK = 0x800,
  EEP_MOSI = 0x1000,
  EEP_MISO = 0x2000,
  AVE_SCL = 0x4000,
  AVE_SDA = 0x8000,
  DEBUG0 = 0x10000,
  DEBUG1 = 0x20000,
  DEBUG2 = 0x40000,
  DEBUG3 = 0x80000,
  DEBUG4 = 0x100000,
  DEBUG5 = 0x200000,
  DEBUG6 = 0x400000,
  DEBUG7 = 0x800000,
};

enum class SRNPROT : u32
{
  AESEN = 0x1,
  SHAEN = 0x2,
  FLAEN = 0x4,
  AHPEN = 0x8,
  OH1EN = 0x10,
  IOUEN = 0x20,
  IOPDBGEN = 0x40,
};

enum class BUSPROT : u32
{
  PPCACREN = 0x1,  // ?
  PPCFLAEN = 0x2,
  PPCAESEN = 0x4,
  PPCSHAEN = 0x8,
  PPCEHCEN = 0x10,
  PPCOH0EN = 0x20,
  PPCOH1EN = 0x40,
  PPCSD0EN = 0x80,
  PPCSD1EN = 0x100,
  PPCSREN = 0x400,
  PPCAHMEN = 0x800,

  IOPACREN = 0x10000,  // ?
  IOPFLAEN = 0x20000,
  IOPAESEN = 0x40000,
  IOPSHAEN = 0x80000,
  IOPEHCEN = 0x100000,
  IOPOH0EN = 0x200000,
  IOPOH1EN = 0x400000,
  IOPSD0EN = 0x800000,
  IOPSD1EN = 0x1000000,

  PPCKERN = 0x80000000,
};

struct CtrlRegister
{
  u8 X1 : 1;
  u8 X2 : 1;
  u8 Y1 : 1;
  u8 Y2 : 1;
  u8 IX1 : 1;
  u8 IX2 : 1;
  u8 IY1 : 1;
  u8 IY2 : 1;

  CtrlRegister() { X1 = X2 = Y1 = Y2 = IX1 = IX2 = IY1 = IY2 = 0; }
  inline u8 ppc() { return (IY2 << 5) | (IY1 << 4) | (X2 << 3) | (Y1 << 2) | (Y2 << 1) | X1; }
  inline u8 arm() { return (IX2 << 5) | (IX1 << 4) | (Y2 << 3) | (X1 << 2) | (X2 << 1) | Y1; }
  inline void ppc(u32 v)
  {
    X1 = v & 1;
    X2 = (v >> 3) & 1;
    if ((v >> 2) & 1)
      Y1 = 0;
    if ((v >> 1) & 1)
      Y2 = 0;
    IY1 = (v >> 4) & 1;
    IY2 = (v >> 5) & 1;
  }

  inline void arm(u32 v)
  {
    Y1 = v & 1;
    Y2 = (v >> 3) & 1;
    if ((v >> 2) & 1)
      X1 = 0;
    if ((v >> 1) & 1)
      X2 = 0;
    IX1 = (v >> 4) & 1;
    IX2 = (v >> 5) & 1;
  }
};

class WiiIPC
{
public:
  explicit WiiIPC(Core::System& system);
  WiiIPC(const WiiIPC&) = delete;
  WiiIPC(WiiIPC&&) = delete;
  WiiIPC& operator=(const WiiIPC&) = delete;
  WiiIPC& operator=(WiiIPC&&) = delete;
  ~WiiIPC();

  void Init();
  void Reset();
  void Shutdown();
  void UpdateGPIO(Core::System& system);
  void DoState(PointerWrap& p);
  static bool CheckBusAccess(Core::System& system, u32 addr, bool is_write);

  void RegisterMMIO(MMIO::Mapping* mmio, u32 base);

  void ClearX1();
  void GenerateAck(u32 address);
  void GenerateReply(u32 address);

  bool IsReady() const;

  Common::Flags<GPIO> GetGPIOInFlags() const;
  Common::Flags<GPIO> GetGPIOOutFlags() const { return m_gpio_out; }
  Common::Flags<SRNPROT> GetSRNPROTFlags() const { return m_srnprot; }
  u32 GetSpare1() const { return m_spare1; }

  u32 GetStarletTimer() const;

  void TriggerIRQ(StarletInterruptCause cause);

private:
  void InitState();
  void ScheduleAlarm();

  static void UpdateInterruptsCallback(Core::System& system, u64 userdata, s64 cycles_late);
  void UpdateInterrupts();
  static void TriggerAlarmCallback(Core::System& system, u64 userdata, s64 cycles_late);
  void TriggerAlarm(u64 userdata);

  void TriggerScheduledInterrupts();

  void ClockEeprom();

  u32 m_ppc_msg = 0;
  u32 m_arm_msg = 0;
  CtrlRegister m_ctrl{};

  u32 m_ppc_irq_flags = 0;
  u32 m_ppc_irq_masks = 0;
  u32 m_arm_irq_flags = 0;
  u32 m_arm_irq_masks = 0;

  std::mutex m_gpio_mutex;
  Common::Flags<GPIO> m_gpio_dir{};
  Common::Flags<GPIO> m_gpio_out{};
  Common::Flags<GPIO> m_gpio_ppcowner{};
  Common::Flags<GPIO> m_gpio_intlvl{};
  Common::Flags<GPIO> m_gpio_intsts{};
  Common::Flags<GPIO> m_gpio_inten{};

  Common::Flags<SRNPROT> m_srnprot{};
  Common::Flags<BUSPROT> m_busprot{};
  u32 m_aipprot;

  u32 m_spare0 = 0;
  u32 m_spare1 = 0;
  u32 m_sysctrl = 0;
  u32 m_resets = 0;

  CoreTiming::EventType* m_event_type_update_interrupts = nullptr;
  CoreTiming::EventType* m_event_type_iop_alarm = nullptr;

  // The timer register is incremented every 1/128th of the core clock frequency, or around every
  // 526.7 nanoseconds. It can be written to to reset to an arbitrary value.
  u32 m_timer_base = 0;

  // Real timestamp of the last time the timer value was set
  u64 m_timer_base_time = {};

  u32 m_alarm = 0;

  u32 m_efuse_addr = 0;
  u32 m_efuse_data = 0;

  Core::System& m_system;

  bool m_eeprom_last_clock = false;
  bool m_eeprom_last_chip_select = false;
  u16 m_eeprom_data_in = 0;
  u32 m_eeprom_bit_count = 0;
  bool m_eeprom_miso = false;
  u16 m_eeprom_stored_value = 0;
  bool m_eeprom_enable_programming = false;
};
}  // namespace IOS
