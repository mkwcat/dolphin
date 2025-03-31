// Copyright 2025 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include "Common/CommonTypes.h"

namespace Core
{
class System;
}
namespace MMIO
{
class Mapping;
}

namespace AHB
{

class AHBMemoryController
{
public:
  explicit AHBMemoryController();
  AHBMemoryController(const AHBMemoryController&) = delete;
  AHBMemoryController(AHBMemoryController&&) = delete;
  AHBMemoryController& operator=(const AHBMemoryController&) = delete;
  AHBMemoryController& operator=(AHBMemoryController&&) = delete;
  ~AHBMemoryController();

  void Init();
  void Reset();
  void RegisterMMIO(MMIO::Mapping* mmio, u32 base);

private:
  u16 m_compat;
  u16 m_prot_reg;
  u16 m_prot_spl;
  u16 m_prot_spl_base;
  u16 m_prot_spl_end;
  u16 m_prot_ddr;
  u16 m_prot_ddr_base;
  u16 m_prot_ddr_end;
  //   u16 m_colsel;
  //   u16 m_rowsel;
  //   u16 m_banksel;
  //   u16 m_ranksel;
  //   u16 m_colmsk;
  //   u16 m_rowmsk;
  //   u16 m_bankmsk;
  //   u16 m_prot_spl_err;
  //   u16 m_prot_ddr_err;
  //   u16 m_prot_spl_msk;
  //   u16 m_prot_ddr_msk;
  //   u16 m_rfsh;
  u16 m_ahmflush;
  //   u16 m_ahmflush_ack;
  //   u16 m_seqrd_hwm;
  //   u16 m_seqwr_hwm;
  //   u16 m_seqcmd_hwm;
  //   u16 m_cpuahm_wr_t;
  //   u16 m_dmaahm_wr_t;
  //   u16 m_dmaahm0_wr_t;
  //   u16 m_dmaahm1_wr_t;
  //   u16 m_pi_wr_t;
  //   u16 m_pe_wr_t;
  //   u16 m_io_wr_t;
  //   u16 m_dsp_wr_t;
  //   u16 m_acc_wr_t;
  //   u16 m_arb_maxwr;
  //   u16 m_arb_minrd;
  //   u16 m_prof_cpuahm;
  //   u16 m_prof_cpuahm0;
  //   u16 m_prof_dmaahm;
  //   u16 m_prof_dmaahm0;
  //   u16 m_prof_dmaahm1;
  //   u16 m_prof_pi;
  //   u16 m_prof_vi;
  //   u16 m_prof_io;
  //   u16 m_prof_dsp;
  //   u16 m_prof_tc;
  //   u16 m_prof_cp;
  //   u16 m_prof_acc;
  //   u16 m_rdpr_cpuahm;
  //   u16 m_rdpr_cpuahm0;
  //   u16 m_rdpr_dmaahm;
  //   u16 m_rdpr_dmaahm0;
  //   u16 m_rdpr_dmaahm1;
  //   u16 m_rdpr_pi;
  //   u16 m_rdpr_vi;
  //   u16 m_rdpr_io;
  //   u16 m_rdpr_dsp;
  //   u16 m_rdpr_tc;
  //   u16 m_rdpr_cp;
  //   u16 m_rdpr_acc;
  //   u16 m_arb_maxrd;
  //   u16 m_arb_misc;
  //   u16 m_aram_emul;
  //   u16 m_wrmux;
  //   u16 m_perf;
  //   u16 m_perf_read;
  //   u16 m_arb_exaddr;
  //   u16 m_arb_excmd;
  //   u16 m_seq_data;
  //   u16 m_seq_addr;
  //   u16 m_bist_data;
  //   u16 m_bist_addr;
};

}  // namespace AHB