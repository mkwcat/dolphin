// Copyright 2025 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "Core/HW/AHB/AHBMemoryController.h"
#include "Common/Logging/Log.h"
#include "Core/HW/MMIO.h"
#include "Core/HW/WII_IPC.h"
#include "Core/System.h"

namespace AHB
{

// Internal hardware addresses
enum
{
  MEM_COMPAT = 0xB4200,
  MEM_PROT_REG = 0xB4202,
  MEM_PROT_SPL = 0xB4204,
  MEM_PROT_SPL_BASE = 0xB4206,
  MEM_PROT_SPL_END = 0xB4208,
  MEM_PROT_DDR = 0xB420A,
  MEM_PROT_DDR_BASE = 0xB420C,
  MEM_PROT_DDR_END = 0xB420E,
  MEM_COLSEL = 0xB4210,
  MEM_ROWSEL = 0xB4212,
  MEM_BANKSEL = 0xB4214,
  MEM_RANKSEL = 0xB4216,
  MEM_COLMSK = 0xB4218,
  MEM_ROWMSK = 0xB421A,
  MEM_BANKMSK = 0xB421C,
  MEM_PROT_SPL_ERR = 0xB421E,
  MEM_PROT_DDR_ERR = 0xB4220,
  MEM_PROT_SPL_MSK = 0xB4222,
  MEM_PROT_DDR_MSK = 0xB4224,
  MEM_RFSH = 0xB4226,
  MEM_AHMFLUSH = 0xB4228,
  MEM_AHMFLUSH_ACK = 0xB422A,
  MEM_SEQRD_HWM = 0xB4268,
  MEM_SEQWR_HWM = 0xB426A,
  MEM_SEQCMD_HWM = 0xB426C,
  MEM_CPUAHM_WR_T = 0xB426E,
  MEM_DMAAHM_WR_T = 0xB4270,
  MEM_DMAAHM0_WR_T = 0xB4272,
  MEM_DMAAHM1_WR_T = 0xB4274,
  MEM_PI_WR_T = 0xB4276,
  MEM_PE_WR_T = 0xB4278,
  MEM_IO_WR_T = 0xB427A,
  MEM_DSP_WR_T = 0xB427C,
  MEM_ACC_WR_T = 0xB427E,
  MEM_ARB_MAXWR = 0xB4280,
  MEM_ARB_MINRD = 0xB4282,
  MEM_PROF_CPUAHM = 0xB4284,
  MEM_PROF_CPUAHM0 = 0xB4286,
  MEM_PROF_DMAAHM = 0xB4288,
  MEM_PROF_DMAAHM0 = 0xB428A,
  MEM_PROF_DMAAHM1 = 0xB428C,
  MEM_PROF_PI = 0xB428E,
  MEM_PROF_VI = 0xB4290,
  MEM_PROF_IO = 0xB4292,
  MEM_PROF_DSP = 0xB4294,
  MEM_PROF_TC = 0xB4296,
  MEM_PROF_CP = 0xB4298,
  MEM_PROF_ACC = 0xB429A,
  MEM_RDPR_CPUAHM = 0xB429C,
  MEM_RDPR_CPUAHM0 = 0xB429E,
  MEM_RDPR_DMAAHM = 0xB42A0,
  MEM_RDPR_DMAAHM0 = 0xB42A2,
  MEM_RDPR_DMAAHM1 = 0xB42A4,
  MEM_RDPR_PI = 0xB42A6,
  MEM_RDPR_VI = 0xB42A8,
  MEM_RDPR_IO = 0xB42AA,
  MEM_RDPR_DSP = 0xB42AC,
  MEM_RDPR_TC = 0xB42AE,
  MEM_RDPR_CP = 0xB42B0,
  MEM_RDPR_ACC = 0xB42B2,
  MEM_ARB_MAXRD = 0xB42B4,
  MEM_ARB_MISC = 0xB42B6,
  MEM_ARAM_EMUL = 0xB42B8,
  MEM_WRMUX = 0xB42BA,
  MEM_PERF = 0xB42BC,
  MEM_PERF_READ = 0xB42BE,
  MEM_ARB_EXADDR = 0xB42C0,
  MEM_ARB_EXCMD = 0xB42C2,
  MEM_SEQ_DATA = 0xB42C4,
  MEM_SEQ_ADDR = 0xB42C6,
  MEM_BIST_DATA = 0xB42C8,
  MEM_BIST_ADDR = 0xB42CA
};

AHBMemoryController::AHBMemoryController() = default;

AHBMemoryController::~AHBMemoryController() = default;

void AHBMemoryController::Init()
{
  Reset();
}

void AHBMemoryController::Reset()
{
  m_compat = 0x0018;
  m_prot_reg = 0;
  m_prot_spl = 0;
  m_prot_spl_base = 0;
  m_prot_spl_end = 0;
  m_prot_ddr = 0;
  m_prot_ddr_base = 0;
  m_prot_ddr_end = 0;
  //   m_colsel = 6;
  //   m_rowsel = 0;
  //   m_banksel = 2;
  //   m_ranksel = 0;
  //   m_colmsk = 0x01FF;
  //   m_rowmsk = 0;
  //   m_bankmsk = 0x7;
  //   m_prot_spl_err = 0;
  //   m_prot_ddr_err = 0;
  //   m_prot_spl_msk = 0;
  //   m_prot_ddr_msk = 0;
  //   m_rfsh = 0;
  m_ahmflush = 0;
  //   m_ahmflush_ack = 0;
  //   m_seqrd_hwm = 0;
  //   m_seqwr_hwm = 0;
  //   m_seqcmd_hwm = 0;
  //   m_cpuahm_wr_t = 0;
  //   m_dmaahm_wr_t = 0;
  //   m_dmaahm0_wr_t = 0;
  //   m_dmaahm1_wr_t = 0;
  //   m_pi_wr_t = 0;
  //   m_pe_wr_t = 0;
  //   m_io_wr_t = 0;
  //   m_dsp_wr_t = 0;
  //   m_acc_wr_t = 0;
  //   m_arb_maxwr = 0;
  //   m_arb_minrd = 0;
  //   m_prof_cpuahm = 0;
  //   m_prof_cpuahm0 = 0;
  //   m_prof_dmaahm = 0;
  //   m_prof_dmaahm0 = 0;
  //   m_prof_dmaahm1 = 0;
  //   m_prof_pi = 0;
  //   m_prof_vi = 0;
  //   m_prof_io = 0;
  //   m_prof_dsp = 0;
  //   m_prof_tc = 0;
  //   m_prof_cp = 0;
  //   m_prof_acc = 0;
  //   m_rdpr_cpuahm = 0;
  //   m_rdpr_cpuahm0 = 0;
  //   m_rdpr_dmaahm = 0;
  //   m_rdpr_dmaahm0 = 0;
  //   m_rdpr_dmaahm1 = 0;
  //   m_rdpr_pi = 0;
  //   m_rdpr_vi = 0;
  //   m_rdpr_io = 0;
  //   m_rdpr_dsp = 0;
  //   m_rdpr_tc = 0;
  //   m_rdpr_cp = 0;
  //   m_rdpr_acc = 0;
  //   m_arb_maxrd = 0;
  //   m_arb_misc = 0;
  //   m_aram_emul = 0;
  //   m_wrmux = 0;
  //   m_perf = 0;
  //   m_perf_read = 0;
  //   m_arb_exaddr = 0;
  //   m_arb_excmd = 0;
  //   m_seq_data = 0;
  //   m_seq_addr = 0;
  //   m_bist_data = 0;
  //   m_bist_addr = 0;
}

void AHBMemoryController::RegisterMMIO(MMIO::Mapping* mmio, u32 base)
{
  const auto RegisterValue = [base, mmio](u32 offset, u16& value, u32 mask = 0xFFFFFFFF) {
    mmio->Register(base | offset, MMIO::DirectRead<u16>(&value, mask),
                   MMIO::DirectWrite<u16>(&value, mask), IOS::WiiIPC::CheckBusAccess);
  };

  RegisterValue(MEM_COMPAT, m_compat);
  RegisterValue(MEM_PROT_REG, m_prot_reg);
  RegisterValue(MEM_PROT_SPL, m_prot_spl);
  RegisterValue(MEM_PROT_SPL_BASE, m_prot_spl_base);
  RegisterValue(MEM_PROT_SPL_END, m_prot_spl_end);
  RegisterValue(MEM_PROT_DDR, m_prot_ddr);
  RegisterValue(MEM_PROT_DDR_BASE, m_prot_ddr_base);
  RegisterValue(MEM_PROT_DDR_END, m_prot_ddr_end);

  mmio->Register(base | MEM_AHMFLUSH, MMIO::DirectRead<u16>(&m_ahmflush),
                 MMIO::ComplexWrite<u16>([](Core::System& system, u32, u16 val) {
                   auto& ahb = system.GetAHBMemoryController();
                   if (val != 0)
                   {
                     bool iop = val & 0x1;
                     bool crypto = val & 0x2;
                     bool nand = val & 0x8;
                     INFO_LOG_FMT(IOS_LLE, "AHB Flush: {:x}: iop({}) crypto({}) nand/sdhc({})", val,
                                  iop, crypto, nand);
                   }
                   ahb.m_ahmflush = val;
                 }),
                 IOS::WiiIPC::CheckBusAccess);

  // TODO: Check the usual timing
  mmio->Register(base | MEM_AHMFLUSH_ACK, MMIO::DirectRead<u16>(&m_ahmflush),
                 MMIO::InvalidWrite<u16>(), IOS::WiiIPC::CheckBusAccess);
}

}  // namespace AHB