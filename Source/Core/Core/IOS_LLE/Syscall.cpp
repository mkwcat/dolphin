// Copyright 2025 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ARM.h"

namespace IOS::LLE
{

void ARM::SVCWrite0(u32 addr)
{
  ARMv5* v5 = static_cast<ARMv5*>(this);

  while (true)
  {
    if (!v5->HostIsRAMAddress(addr))
      break;

    u32 v;
    v5->DataRead8(addr, &v);

    if (v == 0)
      break;

    if (v == '\n')
    {
      NOTICE_LOG_FMT(IOS_LLE, "IOS REPORT: {:s}", m_svc_write_buffer.data());
      m_svc_write_buffer.clear();
    }
    else if (v != '\r')
    {
      m_svc_write_buffer.push_back(v);
    }

    addr += 1;
  }
}

void ARM::LogSyscall(u32 instr)
{
  u8 syscall = instr >> 5 & 0xFF;

  struct IOSSyscallDef
  {
    int arg_count;
    const char* name;
  } syscall_def;

  static constexpr IOSSyscallDef Syscalls[] = {
      {6, "IOS_CreateThread"},
      {2, "JoinThread"},
      {2, "CancelThread"},
      {0, "IOS_GetThreadId"},
      {0, "GetProcessId"},
      {1, "IOS_StartThread"},
      {1, "SuspendThread"},
      {0, "YieldThread"},
      {1, "GetThreadPriority"},
      {2, "SetThreadPriority"},
      {2, "IOS_CreateMessageQueue"},
      {2, "IOS_DestroyMessageQueue"},
      {3, "IOS_SendMessage"},
      {3, "IOS_JamMessage"},
      {3, "IOS_ReceiveMessage"},
      {3, "IOS_HandleEvent"},
      {1, "UnregisterEventHandler"},
      {5, "IOS_CreateTimer"},
      {3, "IOS_RestartTimer"},
      {1, "IOS_StopTimer"},
      {1, "IOS_DestroyTimer"},
      {0, "time_now"},
      {2, "IOS_CreateHeap"},
      {1, "IOS_DestroyHeap"},
      {2, "IOS_Alloc"},
      {3, "IOS_AllocAligned"},
      {2, "IOS_Free"},
      {2, "IOS_RegisterResourceManager"},
      {2, "IOS_Open"},
      {1, "IOS_Close"},
      {3, "IOS_Read"},
      {3, "IOS_Write"},
      {3, "IOS_Seek"},
      {6, "IOS_Ioctl"},
      {6, "IOS_Ioctlv"},
      {4, "IOS_OpenAsync"},
      {4, "IOS_CloseAsync"},
      {5, "IOS_ReadAsync"},
      {5, "IOS_WriteAsync"},
      {5, "IOS_SeekAsync"},
      {7, "IOS_IoctlAsync"},
      {7, "IOS_IoctlvAsync"},
      {2, "IOS_ResourceReply"},
      {3, "SetUid"},
      {0, "GetUid"},
      {3, "SetGid"},
      {0, "GetGid"},
      {1, "AHB_MemFlush"},
      {1, "AHB_MemRBInvalidate"},
      {0, "ClearAndEnableIPCIOPIntr"},
      {0, "ClearAndEnableDIIntr"},
      {0, "ClearAndEnableSDIntr"},
      {0, "ClearAndEnableEvent"},
      {1, "AccessIobPool"},
      {2, "alloc_iobuf"},
      {1, "free_iobuf"},
      {0, "iobuf_log_header_info"},
      {0, "iobuf_log_buffer_info"},
      {1, "extend_iobuf"},
      {1, "IOS_PushIob"},
      {1, "IOS_PullIob"},
      {1, "verify_iobuf"},
      {4, "syscall_3e"},  // Unknown; related to IO buffer functionality
      {2, "IOS_InvalidateDCache"},
      {2, "IOS_FlushDCache"},
      {1, "IOS_StartPPC"},
      {3, "ios_boot"},
      {3, "boot_new_ios_kernel"},
      {1, "assert_di_reset"},
      {1, "deassert_di_reset"},
      {0, "check_di_reset"},
      {2, "get_kernel_flavor"},
      {2, "get_unk_flavor"},
      {0, "get_boot_vector"},
      {0, "GetHollywoodId"},
      {0, "kernel_debug_print"},
      {1, "SetLoMemOSVersion"},
      {0, "GetLoMemOSVersion"},
      {1, "SetDiSpinup"},
      {1, "VirtualToPhysical"},
      {1, "SetDvdReadDisable"},
      {0, "GetDvdReadDisable"},
      {1, "SetEnableAHBPI2DI"},
      {0, "GetEnableAHBPI2DI"},
      {1, "SetPPCACRPerms"},
      {0, "GetBusSpeed"},
      {1, "ACRRegWrite"},
      {1, "DDRRegWrite"},
      {1, "OutputDebugPort"},
      {1, "SetIpcAccessRights"},
      {2, "LaunchRM"},
      {3, "IOSC_CreateObject"},
      {2, "IOSC_DeleteObject"},
      {5, "IOSC_ImportSecretKey"},
      {5, "IOSC_ExportSecretKey"},
      {4, "IOSC_ImportPublicKey"},
      {4, "IOSC_ExportPublicKey"},
      {3, "IOSC_ComputeSharedKey"},
      {2, "IOSC_SetData"},
      {2, "IOSC_GetData"},
      {2, "IOSC_GetKeySize"},
      {2, "IOSC_GetSignatureSize"},
      {6, "IOSC_GenerateHashAsync"},
      {2, "IOSC_GenerateHash"},
      {6, "IOSC_EncryptAsync"},
      {2, "IOSC_Encrypt"},
      {6, "IOSC_DecryptAsync"},
      {2, "IOSC_Decrypt"},
      {4, "IOSC_VerifyPublicKeySign"},
      {6, "IOSC_GenerateBlockMAC"},
      {6, "IOSC_GenerateBlockMACAsync"},
      {4, "IOSC_ImportCertificate"},
      {2, "IOSC_GetDeviceCertificate"},
      {2, "IOSC_SetOwnership"},
      {2, "IOSC_GetOwnership"},
      {2, "IOSC_GenerateRand"},
      {1, "IOSC_GenerateKey"},
      {5, "IOSC_GeneratePublicKeySign"},
      {5, "IOSC_GenerateCertificate"},
      {4, "IOSC_CheckDiHashes"},
      {3, "SetProcessPriorities"},
      {0, "GetProcessPriorities"},
      {4, "syscall_7a"},  // Unknown
      {4, "syscall_7b"},  // Unknown
      {4, "syscall_7c"},  // Unknown
  };

  if (syscall < sizeof(Syscalls) / sizeof(Syscalls[0]))
  {
    syscall_def = Syscalls[syscall];
  }
  else
  {
    syscall_def = {4, "Unknown syscall"};
  }

  // TODO: Stack args

  u32 lr = R[14];
  switch (syscall_def.arg_count)
  {
  case 0:
    INFO_LOG_FMT(IOS_LLE, "IOS syscall {:02x} from {:08x}: {}()", syscall, lr, syscall_def.name);
    break;

  case 1:
    INFO_LOG_FMT(IOS_LLE, "IOS syscall {:02x} from {:08x}: {}({:08x})", syscall, lr,
                 syscall_def.name, R[0]);
    break;

  case 2:
    INFO_LOG_FMT(IOS_LLE, "IOS syscall {:02x} from {:08x}: {}({:08x}, {:08x})", syscall, lr,
                 syscall_def.name, R[0], R[1]);
    break;

  case 3:
    INFO_LOG_FMT(IOS_LLE, "IOS syscall {:02x} from {:08x}: {}({:08x}, {:08x}, {:08x})", syscall, lr,
                 syscall_def.name, R[0], R[1], R[2]);
    break;

  default:
  case 4:
    INFO_LOG_FMT(IOS_LLE, "IOS syscall {:02x} from {:08x}: {}({:08x}, {:08x}, {:08x}, {:08x})",
                 syscall, lr, syscall_def.name, R[0], R[1], R[2], R[3]);
    break;
  }
}

}  // namespace IOS::LLE