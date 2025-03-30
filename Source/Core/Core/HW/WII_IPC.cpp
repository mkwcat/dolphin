// Copyright 2008 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "Core/HW/WII_IPC.h"

#include "Common/ChunkFile.h"
#include "Common/CommonTypes.h"
#include "Common/Logging/Log.h"
#include "Common/Timer.h"
#include "Core/Core.h"
#include "Core/CoreTiming.h"
#include "Core/HW/DVD/DVDInterface.h"
#include "Core/HW/MMIO.h"
#include "Core/HW/MMIOHandlers.h"
#include "Core/HW/ProcessorInterface.h"
#include "Core/IOS/IOS.h"
#include "Core/System.h"

// This is the intercommunication between ARM and PPC. Currently only PPC actually uses it, because
// of the IOS HLE
// How IOS uses IPC:
// X1 Execute command: a new pointer is available in HW_IPC_PPCCTRL
// X2 Reload (a new IOS is being loaded, old one doesn't need to reply anymore)
// Y1 Command executed and reply available in HW_IPC_ARMMSG
// Y2 Command acknowledge
// m_ppc_msg is a pointer to 0x40byte command structure
// m_arm_msg is, similarly, starlet's response buffer*

namespace IOS
{
enum
{
  IPC_PPCMSG = 0x00,
  IPC_PPCCTRL = 0x04,
  IPC_ARMMSG = 0x08,
  IPC_ARMCTRL = 0x0c,

  TIMER = 0x10,
  ALARM = 0x14,

  VI1CFG = 0x18,
  VISOLID = 0x24,

  PPC_IRQFLAG = 0x30,
  PPC_IRQMASK = 0x34,
  ARM_IRQFLAG = 0x38,
  ARM_IRQMASK = 0x3c,

  SRNPROT = 0x60,
  BUSPROT = 0x64,

  GPIO_PPCOUT = 0xc0,
  GPIO_PPCDIR = 0xc4,
  GPIO_PPCIN = 0xc8,
  GPIO_PPCINTLVL = 0xcc,
  GPIO_PPCINTSTS = 0xd0,
  GPIO_PPCINTEN = 0xd4,
  GPIO_PPCSTRAPS = 0xd8,

  GPIO_IOPEN = 0xdc,
  GPIO_IOPOUT = 0xe0,
  GPIO_IOPDIR = 0xe4,
  GPIO_IOPIN = 0xe8,
  GPIO_IOPINTLVL = 0xec,
  GPIO_IOPINTSTS = 0xf0,
  GPIO_IOPINTEN = 0xf4,
  GPIO_IOPSTRAPS = 0xf8,
  GPIO_IOPPPCOWNER = 0xfc,

  COMPAT = 0x180,

  SYSCTRL = 0x190,
  RSTCTRL = 0x194,

  PLLSYS = 0x1b0,
  PLLSYSEXT = 0x1b4,

  PLLAI = 0x1cc,
  PLLAIEXT = 0x1d0,

  EFUSEADDR = 0x1ec,
  EFUSEDATA = 0x1f0,

  CHIPREVID = 0x214,
};

// Indicates which pins are accessible by broadway.  Writable by starlet only.
static constexpr Common::Flags<GPIO> default_ppcowner = {
    GPIO::SLOT_LED, GPIO::SLOT_IN, GPIO::SENSOR_BAR, GPIO::DO_EJECT, GPIO::AVE_SCL, GPIO::AVE_SDA};

WiiIPC::WiiIPC(Core::System& system) : m_system(system)
{
}

WiiIPC::~WiiIPC() = default;

void WiiIPC::DoState(PointerWrap& p)
{
  p.Do(m_ppc_msg);
  p.Do(m_arm_msg);
  p.Do(m_ctrl);
  p.Do(m_ppc_irq_flags);
  p.Do(m_ppc_irq_masks);
  p.Do(m_arm_irq_flags);
  p.Do(m_arm_irq_masks);
  p.Do(m_gpio_dir);
  p.Do(m_gpio_out);
  p.Do(m_resets);
}

void WiiIPC::InitState()
{
  m_ctrl = CtrlRegister();
  m_ppc_msg = 0;
  m_arm_msg = 0;

  m_ppc_irq_flags = 0;
  m_ppc_irq_masks = 0;
  m_arm_irq_flags = 0;
  m_arm_irq_masks = 0;

  m_gpio_ppcowner = default_ppcowner;

  // The only inputs are POWER, EJECT_BTN, SLOT_IN, and EEP_MISO; Broadway only has access to
  // SLOT_IN
  m_gpio_dir = {
      GPIO::POWER,      GPIO::SHUTDOWN, GPIO::FAN,    GPIO::DC_DC,   GPIO::DI_SPIN,  GPIO::SLOT_LED,
      GPIO::SENSOR_BAR, GPIO::DO_EJECT, GPIO::EEP_CS, GPIO::EEP_CLK, GPIO::EEP_MOSI, GPIO::AVE_SCL,
      GPIO::AVE_SDA,    GPIO::DEBUG0,   GPIO::DEBUG1, GPIO::DEBUG2,  GPIO::DEBUG3,   GPIO::DEBUG4,
      GPIO::DEBUG5,     GPIO::DEBUG6,   GPIO::DEBUG7,
  };
  m_gpio_out = {};

  // A cleared bit indicates the device is reset/off, so set everything to 1 (this may not exactly
  // match hardware)
  m_resets = 0xffffffff;

  m_sysctrl = 0;

  m_busprot.m_hex = 0xFFFFFFFF & ~0x80000DFE;

  m_timer_base = 0;
  m_timer_base_time = Common::Timer::NowUs();

  m_ppc_irq_masks |= INT_CAUSE_IPC_BROADWAY;
}

void WiiIPC::Init()
{
  InitState();
  m_event_type_update_interrupts =
      m_system.GetCoreTiming().RegisterEvent("IPCInterrupt", UpdateInterruptsCallback);
}

void WiiIPC::Reset()
{
  INFO_LOG_FMT(WII_IPC, "Resetting ...");
  InitState();
}

void WiiIPC::Shutdown()
{
}

void WiiIPC::UpdateGPIO(Core::System& system)
{
  if (m_gpio_out[GPIO::DO_EJECT])
  {
    INFO_LOG_FMT(WII_IPC, "Ejecting disc due to GPIO write");
    system.GetDVDInterface().EjectDisc(Core::CPUThreadGuard{system}, DVD::EjectCause::Software);
  }
  // SENSOR_BAR is checked by WiimoteEmu::CameraLogic
  // TODO: AVE, SLOT_LED
}

bool WiiIPC::CheckBusAccess(Core::System& system, u32 addr, bool is_write)
{
  auto& wii_ipc = system.GetWiiIPC();

  if ((addr >> 24) == 0x0D)
  {
    bool is_privileged = (addr & 0x00800000) || wii_ipc.m_busprot[BUSPROT::PPCKERN];

    // SRAM
    if (addr & 0x00400000)
    {
      // TODO
      return true;
    }

    u32 reg = addr & 0x0000FFFF;

    switch (reg)
    {
    case IPC_PPCMSG:
    case IPC_PPCCTRL:
    case IPC_ARMMSG:
    case PPC_IRQFLAG:
    case PPC_IRQMASK:
    case GPIO_PPCOUT:
    case GPIO_PPCDIR:
    case GPIO_PPCIN:
    case GPIO_PPCINTLVL:
    case GPIO_PPCINTSTS:
    case GPIO_PPCINTEN:
    case GPIO_PPCSTRAPS:
    case PLLAI:
    case PLLAIEXT:
      return true;
    }

    return is_privileged;
  }

  return true;
}

void WiiIPC::RegisterMMIO(MMIO::Mapping* mmio, u32 base)
{
  mmio->Register(base | IPC_PPCMSG, MMIO::InvalidRead<u32>(), MMIO::DirectWrite<u32>(&m_ppc_msg),
                 CheckBusAccess);

  mmio->Register(base | IPC_PPCCTRL, MMIO::ComplexRead<u32>([](Core::System& system, u32) {
                   auto& wii_ipc = system.GetWiiIPC();
                   return wii_ipc.m_ctrl.ppc();
                 }),
                 MMIO::ComplexWrite<u32>([](Core::System& system, u32, u32 val) {
                   auto& wii_ipc = system.GetWiiIPC();
                   wii_ipc.m_ctrl.ppc(val);
                   // The IPC interrupt is triggered when IY1/IY2 is set and
                   // Y1/Y2 is written to -- even when this results in clearing the bit.
                   if ((val >> 2 & 1 && wii_ipc.m_ctrl.IY1) || (val >> 1 & 1 && wii_ipc.m_ctrl.IY2))
                     wii_ipc.m_ppc_irq_flags |= INT_CAUSE_IPC_BROADWAY;
                   if (system.GetIOS())
                   {
                     if (wii_ipc.m_ctrl.X1)
                       system.GetIOS()->EnqueueIPCRequest(wii_ipc.m_ppc_msg);
                     system.GetIOS()->UpdateIPC();
                     system.GetCoreTiming().ScheduleEvent(0, wii_ipc.m_event_type_update_interrupts,
                                                          0);
                   }
                 }),
                 CheckBusAccess);

  mmio->Register(base | IPC_ARMMSG, MMIO::DirectRead<u32>(&m_arm_msg), MMIO::InvalidWrite<u32>());

  mmio->Register(base | TIMER, MMIO::ComplexRead<u32>([](Core::System& system, u32) {
                   auto& wii_ipc = system.GetWiiIPC();
                   u64 elapsed_usec = Common::Timer::NowUs() - wii_ipc.m_timer_base_time;
                   return wii_ipc.m_timer_base + static_cast<u32>(elapsed_usec / 526.7);
                 }),
                 MMIO::ComplexWrite<u32>([](Core::System& system, u32, u32 val) {
                   auto& wii_ipc = system.GetWiiIPC();
                   wii_ipc.m_timer_base = val;
                   wii_ipc.m_timer_base_time = Common::Timer::NowUs();
                 }),
                 CheckBusAccess);

  mmio->Register(base | PPC_IRQFLAG, MMIO::InvalidRead<u32>(),
                 MMIO::ComplexWrite<u32>([](Core::System& system, u32, u32 val) {
                   auto& wii_ipc = system.GetWiiIPC();
                   wii_ipc.m_ppc_irq_flags &= ~val;
                   if (system.GetIOS())
                   {
                     system.GetIOS()->UpdateIPC();
                     system.GetCoreTiming().ScheduleEvent(0, wii_ipc.m_event_type_update_interrupts,
                                                          0);
                   }
                 }),
                 CheckBusAccess);

  mmio->Register(base | PPC_IRQMASK, MMIO::InvalidRead<u32>(),
                 MMIO::ComplexWrite<u32>([](Core::System& system, u32, u32 val) {
                   auto& wii_ipc = system.GetWiiIPC();
                   wii_ipc.m_ppc_irq_masks = val;
                   if (wii_ipc.m_ppc_irq_masks & INT_CAUSE_IPC_BROADWAY)  // wtf?
                     wii_ipc.Reset();
                   if (system.GetIOS())
                   {
                     system.GetIOS()->UpdateIPC();
                     system.GetCoreTiming().ScheduleEvent(0, wii_ipc.m_event_type_update_interrupts,
                                                          0);
                   }
                 }),
                 CheckBusAccess);

  mmio->Register(base | SRNPROT, MMIO::DirectRead<u32>(&m_srnprot.m_hex),
                 MMIO::DirectWrite<u32>(&m_srnprot.m_hex), CheckBusAccess);

  mmio->Register(base | BUSPROT, MMIO::DirectRead<u32>(&m_busprot.m_hex),
                 MMIO::DirectWrite<u32>(&m_busprot.m_hex), CheckBusAccess);

  mmio->Register(base | GPIO_PPCOUT, MMIO::ComplexRead<u32>([](Core::System& system, u32) {
                   auto& wii_ipc = system.GetWiiIPC();
                   return wii_ipc.m_gpio_out.m_hex & wii_ipc.m_gpio_ppcowner.m_hex;
                 }),
                 MMIO::ComplexWrite<u32>([](Core::System& system, u32, u32 val) {
                   auto& wii_ipc = system.GetWiiIPC();
                   wii_ipc.m_gpio_out.m_hex =
                       (val & wii_ipc.m_gpio_ppcowner.m_hex) |
                       (wii_ipc.m_gpio_out.m_hex & ~wii_ipc.m_gpio_ppcowner.m_hex);
                   wii_ipc.UpdateGPIO(system);
                 }),
                 CheckBusAccess);
  mmio->Register(base | GPIO_PPCDIR, MMIO::DirectRead<u32>(&m_gpio_dir.m_hex),
                 MMIO::ComplexWrite<u32>([](Core::System& system, u32, u32 val) {
                   auto& wii_ipc = system.GetWiiIPC();
                   wii_ipc.m_gpio_dir.m_hex =
                       (val & wii_ipc.m_gpio_ppcowner.m_hex) |
                       (wii_ipc.m_gpio_dir.m_hex & ~wii_ipc.m_gpio_ppcowner.m_hex);
                 }),
                 CheckBusAccess);
  mmio->Register(base | GPIO_PPCIN, MMIO::ComplexRead<u32>([](Core::System& system, u32) {
                   Common::Flags<GPIO> gpio_in;
                   gpio_in[GPIO::SLOT_IN] = system.GetDVDInterface().IsDiscInside();
                   return gpio_in.m_hex & system.GetWiiIPC().m_gpio_ppcowner.m_hex;
                 }),
                 MMIO::Nop<u32>(), CheckBusAccess);

  // Starlet GPIO registers, not normally accessible by PPC (but they can be depending on how
  // BUSPROT is set up).

  // Note from WiiBrew: When switching owners, copying of the data is not necessary. For example, if
  // pin 0 has certain configuration in the HW_GPIO registers, and that bit is then set in the
  // HW_GPIOIOPPPCOWNER register, those settings will immediately be visible in the HW_GPIOPPC
  // registers. There is only one set of data registers, and the HW_GPIOIOPPPCOWNER register just
  // controls the access that the HW_GPIOPPC registers have to that data. Also: The HW_GPIO
  // registers always have read access to all pins, but any writes (changes) must go through the
  // HW_GPIOPPC registers if the corresponding bit is set in the HW_GPIOIOPPPCOWNER register.
  mmio->Register(base | GPIO_IOPOUT, MMIO::DirectRead<u32>(&m_gpio_out.m_hex),
                 MMIO::ComplexWrite<u32>([](Core::System& system, u32, u32 val) {
                   auto& wii_ipc = system.GetWiiIPC();
                   wii_ipc.m_gpio_out.m_hex =
                       (wii_ipc.m_gpio_out.m_hex & wii_ipc.m_gpio_ppcowner.m_hex) |
                       (val & ~wii_ipc.m_gpio_ppcowner.m_hex);
                   wii_ipc.UpdateGPIO(system);
                 }),
                 CheckBusAccess);
  mmio->Register(base | GPIO_IOPDIR, MMIO::DirectRead<u32>(&m_gpio_dir.m_hex),
                 MMIO::ComplexWrite<u32>([](Core::System& system, u32, u32 val) {
                   auto& wii_ipc = system.GetWiiIPC();
                   wii_ipc.m_gpio_dir.m_hex =
                       (wii_ipc.m_gpio_dir.m_hex & wii_ipc.m_gpio_ppcowner.m_hex) |
                       (val & ~wii_ipc.m_gpio_ppcowner.m_hex);
                 }),
                 CheckBusAccess);
  mmio->Register(base | GPIO_IOPIN, MMIO::ComplexRead<u32>([](Core::System& system, u32) {
                   Common::Flags<GPIO> gpio_in;
                   gpio_in[GPIO::SLOT_IN] = system.GetDVDInterface().IsDiscInside();
                   return gpio_in.m_hex;
                 }),
                 MMIO::Nop<u32>(), CheckBusAccess);

  mmio->Register(base | RSTCTRL, MMIO::DirectRead<u32>(&m_resets),
                 MMIO::ComplexWrite<u32>([](Core::System& system, u32, u32 val) {
                   // A reset occurs when the corresponding bit is cleared
                   auto& wii_ipc = system.GetWiiIPC();
                   const bool di_reset_triggered = (wii_ipc.m_resets & 0x400) && !(val & 0x400);
                   wii_ipc.m_resets = val;
                   if (di_reset_triggered)
                   {
                     // The GPIO *disables* spinning up the drive
                     const bool spinup = !wii_ipc.m_gpio_out[GPIO::DI_SPIN];
                     INFO_LOG_FMT(WII_IPC, "Resetting DI {} spinup", spinup ? "with" : "without");
                     system.GetDVDInterface().ResetDrive(spinup);
                   }
                 }),
                 CheckBusAccess);

  // Register some stubbed/unknown MMIOs required to make Wii games work.
  mmio->Register(base | VI1CFG, MMIO::InvalidRead<u32>(), MMIO::Nop<u32>(), CheckBusAccess);
  mmio->Register(base | VISOLID, MMIO::InvalidRead<u32>(), MMIO::Nop<u32>(), CheckBusAccess);
  mmio->Register(base | COMPAT, MMIO::Constant<u32>(0), MMIO::Nop<u32>(), CheckBusAccess);
  mmio->Register(base | PLLAI, MMIO::Constant<u32>(0), MMIO::Nop<u32>(), CheckBusAccess);
  mmio->Register(base | PLLAIEXT, MMIO::Constant<u32>(0), MMIO::Nop<u32>(), CheckBusAccess);

  mmio->Register(base | SYSCTRL, MMIO::DirectRead<u32>(&m_sysctrl),
                 MMIO::DirectWrite<u32>(&m_sysctrl), CheckBusAccess);
  mmio->Register(base | CHIPREVID, MMIO::Constant<u32>(0x11), MMIO::Nop<u32>(), CheckBusAccess);
}

void WiiIPC::UpdateInterruptsCallback(Core::System& system, u64 userdata, s64 cycles_late)
{
  system.GetWiiIPC().UpdateInterrupts();
}

void WiiIPC::UpdateInterrupts()
{
  if ((m_ctrl.Y1 & m_ctrl.IY1) || (m_ctrl.Y2 & m_ctrl.IY2))
  {
    m_ppc_irq_flags |= INT_CAUSE_IPC_BROADWAY;
  }

  if ((m_ctrl.X1 & m_ctrl.IX1) || (m_ctrl.X2 & m_ctrl.IX2))
  {
    m_ppc_irq_flags |= INT_CAUSE_IPC_STARLET;
  }

  // Generate interrupt on PI if any of the devices behind starlet have an interrupt and mask is set
  m_system.GetProcessorInterface().SetInterrupt(ProcessorInterface::INT_CAUSE_WII_IPC,
                                                !!(m_ppc_irq_flags & m_ppc_irq_masks));
}

void WiiIPC::ClearX1()
{
  m_ctrl.X1 = 0;
}

void WiiIPC::GenerateAck(u32 address)
{
  m_ctrl.Y2 = 1;
  DEBUG_LOG_FMT(WII_IPC, "GenerateAck: {:08x} | {:08x} [R:{} A:{} E:{}]", m_ppc_msg, address,
                m_ctrl.Y1, m_ctrl.Y2, m_ctrl.X1);
  // Based on a hardware test, the IPC interrupt takes approximately 100 TB ticks to fire
  // after Y2 is seen in the control register.
  m_system.GetCoreTiming().ScheduleEvent(100_tbticks, m_event_type_update_interrupts);
}

void WiiIPC::GenerateReply(u32 address)
{
  m_arm_msg = address;
  m_ctrl.Y1 = 1;
  DEBUG_LOG_FMT(WII_IPC, "GenerateReply: {:08x} | {:08x} [R:{} A:{} E:{}]", m_ppc_msg, address,
                m_ctrl.Y1, m_ctrl.Y2, m_ctrl.X1);
  // Based on a hardware test, the IPC interrupt takes approximately 100 TB ticks to fire
  // after Y1 is seen in the control register.
  m_system.GetCoreTiming().ScheduleEvent(100_tbticks, m_event_type_update_interrupts);
}

bool WiiIPC::IsReady() const
{
  return ((m_ctrl.Y1 == 0) && (m_ctrl.Y2 == 0) &&
          ((m_ppc_irq_flags & INT_CAUSE_IPC_BROADWAY) == 0));
}
}  // namespace IOS
