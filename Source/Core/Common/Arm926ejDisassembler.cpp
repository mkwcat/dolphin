// VisualBoyAdvance - Nintendo Gameboy/GameboyAdvance (TM) emulator.
// Copyright (C) 1999-2003 Forgotten
// Copyright (C) 2005 Forgotten and the VBA development team

// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2, or(at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software Foundation,
// Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

/************************************************************************/
/* Arm/Thumb command set disassembler                                   */
/************************************************************************/

#include "Arm926ejDisassembler.h"

#include <string>

#include <fmt/format.h>

namespace Common
{

struct Opcodes
{
  u32 mask;
  u32 cval;
  const char* mnemonic;
};

static const char* DecVals[16] = {"0", "1", "2",  "3",  "4",  "5",  "6",  "7",
                                  "8", "9", "10", "11", "12", "13", "14", "15"};

static const char* Regs[16] = {"r0", "r1", "r2",  "r3",  "r4",  "r5", "r6", "r7",
                               "r8", "r9", "r10", "r11", "r12", "sp", "lr", "pc"};

static const char* Conditions[16] = {"eq", "ne", "cs", "cc", "mi", "pl", "vs", "vc",
                                     "hi", "ls", "ge", "lt", "gt", "le", "",   "nv"};

static const char* Shifts[5] = {"lsl", "lsr", "asr", "ror", "rrx"};

static const char* MultLoadStore[12] = {
    // non-stack
    "da", "ia", "db", "ib",
    // stack store
    "ed", "ea", "fd", "fa",
    // stack load
    "fa", "fd", "ea", "ed"};

static const Opcodes ThumbOpcodes[] = {
    // Format 1
    {0xf800, 0x0000, "lsl\t%r0, %r3, %o"},
    {0xf800, 0x0800, "lsr\t%r0, %r3, %o"},
    {0xf800, 0x1000, "asr\t%r0, %r3, %o"},
    // Format 2
    {0xfe00, 0x1800, "add\t%r0, %r3, %r6"},
    {0xfe00, 0x1a00, "sub\t%r0, %r3, %r6"},
    {0xfe00, 0x1c00, "add\t%r0, %r3, %i"},
    {0xfe00, 0x1e00, "sub\t%r0, %r3, %i"},
    // Format 3
    {0xf800, 0x2000, "mov\t%r8, %O"},
    {0xf800, 0x2800, "cmp\t%r8, %O"},
    {0xf800, 0x3000, "add\t%r8, %O"},
    {0xf800, 0x3800, "sub\t%r8, %O"},
    // Format 4
    {0xffc0, 0x4000, "and\t%r0, %r3"},
    {0xffc0, 0x4040, "eor\t%r0, %r3"},
    {0xffc0, 0x4080, "lsl\t%r0, %r3"},
    {0xffc0, 0x40c0, "lsr\t%r0, %r3"},
    {0xffc0, 0x4100, "asr\t%r0, %r3"},
    {0xffc0, 0x4140, "adc\t%r0, %r3"},
    {0xffc0, 0x4180, "sbc\t%r0, %r3"},
    {0xffc0, 0x41c0, "ror\t%r0, %r3"},
    {0xffc0, 0x4200, "tst\t%r0, %r3"},
    {0xffc0, 0x4240, "neg\t%r0, %r3"},
    {0xffc0, 0x4280, "cmp\t%r0, %r3"},
    {0xffc0, 0x42c0, "cmn\t%r0, %r3"},
    {0xffc0, 0x4300, "orr\t%r0, %r3"},
    {0xffc0, 0x4340, "mul\t%r0, %r3"},
    {0xffc0, 0x4380, "bic\t%r0, %r3"},
    {0xffc0, 0x43c0, "mvn\t%r0, %r3"},
    // Format 5
    {0xff80, 0x4700, "bx\t%h36"},
    {0xff80, 0x4780, "blx\t%h36"},
    {0xfcc0, 0x4400, "(ill)\t%Q"},
    {0xff00, 0x4400, "add\t%h07, %h36"},
    {0xff00, 0x4500, "cmp\t%h07, %h36"},
    {0xff00, 0x4600, "mov\t%h07, %h36"},
    // Format 6
    {0xf800, 0x4800, "ldr\t%r8, [%I]"},
    // Format 7
    {0xfa00, 0x5000, "str%b\t%r0, [%r3, %r6]"},
    {0xfa00, 0x5800, "ldr%b\t%r0, [%r3, %r6]"},
    // Format 8
    {0xfe00, 0x5200, "strh\t%r0, [%r3, %r6]"},
    {0xfe00, 0x5600, "ldsb\t%r0, [%r3, %r6]"},
    {0xfe00, 0x5a00, "ldrh\t%r0, [%r3, %r6]"},
    {0xfe00, 0x5e00, "ldsh\t%r0, [%r3, %r6]"},
    // Format 9
    {0xe800, 0x6000, "str%B\t%r0, [%r3, %p]"},
    {0xe800, 0x6800, "ldr%B\t%r0, [%r3, %p]"},
    // Format 10
    {0xf800, 0x8000, "strh\t%r0, [%r3, %e]"},
    {0xf800, 0x8800, "ldrh\t%r0, [%r3, %e]"},
    // Format 11
    {0xf800, 0x9000, "str\t%r8, [sp, %w]"},
    {0xf800, 0x9800, "ldr\t%r8, [sp, %w]"},
    // Format 12
    {0xf800, 0xa000, "add\t%r8, pc, %w"},
    {0xf800, 0xa800, "add\t%r8, sp, %w"},
    // Format 13
    {0xff00, 0xb000, "add\tsp, %s"},
    // Format 14
    {0xffff, 0xb500, "push\t{lr}"},
    {0xff00, 0xb400, "push\t{%l}"},
    {0xff00, 0xb500, "push\t{%l,lr}"},
    {0xffff, 0xbd00, "pop\t{pc}"},
    {0xff00, 0xbd00, "pop\t{%l,pc}"},
    {0xff00, 0xbc00, "pop\t{%l}"},
    // Format 15
    {0xf800, 0xc000, "stmia\t%r8!, {%l}"},
    {0xf800, 0xc800, "ldmia\t%r8!, {%l}"},
    // Format 17
    {0xff00, 0xdf00, "swi\t%m"},
    // Format 16
    {0xf000, 0xd000, "b%c\t%W"},
    // Format 18
    {0xf800, 0xe000, "b\t%a"},
    // Format 19
    {0xf800, 0xf000, "bl\t%A"},
    {0xf800, 0xf800, "blh\t%Z"},
    {0xff00, 0xbe00, "bkpt\t%O"},
    // Unknown
    {0x0000, 0x0000, "(ill)\t%Q"}};

const Opcodes ArmOpcodes[] = {
    // Undefined
    {0x0e000010, 0x06000010, "(ill)\t%Q"},
    // Branch instructions
    {0x0ff000f0, 0x01200010, "bx%c\t%r0"},
    {0x0f000000, 0x0a000000, "b%c\t%o"},
    {0x0f000000, 0x0b000000, "bl%c\t%o"},
    {0x0f000000, 0x0f000000, "swi%c\t%q"},
    // PSR transfer
    {0x0fbf0fff, 0x010f0000, "mrs%c\t%r3, %p"},
    {0x0db0f000, 0x0120f000, "msr%c\t%p, %i"},
    // Multiply instructions
    {0x0fe000f0, 0x00000090, "mul%c%s\t%r4, %r0, %r2"},
    {0x0fe000f0, 0x00200090, "mla%c%s\t%r4, %r0, %r2, %r3"},
    {0x0fa000f0, 0x00800090, "%umull%c%s\t%r3, %r4, %r0, %r2"},
    {0x0fa000f0, 0x00a00090, "%umlal%c%s\t%r3, %r4, %r0, %r2"},
    // Load/Store instructions
    {0x0fb00ff0, 0x01000090, "swp%c%b\t%r3, %r0, [%r4]"},
    {0x0fb000f0, 0x01000090, "(ill)\t%Q"},
    {0x0c100000, 0x04000000, "str%c%b%t\t%r3, %a"},
    {0x0c100000, 0x04100000, "ldr%c%b%t\t%r3, %a"},
    {0x0e100090, 0x00000090, "str%c%h\t%r3, %a"},
    {0x0e100090, 0x00100090, "ldr%c%h\t%r3, %a"},
    {0x0e100000, 0x08000000, "stm%c%m\t%r4%l"},
    {0x0e100000, 0x08100000, "ldm%c%m\t%r4%l"},
    // Data processing
    {0x0de00000, 0x00000000, "and%c%s\t%r3, %r4, %i"},
    {0x0de00000, 0x00200000, "eor%c%s\t%r3, %r4, %i"},
    {0x0de00000, 0x00400000, "sub%c%s\t%r3, %r4, %i"},
    {0x0de00000, 0x00600000, "rsb%c%s\t%r3, %r4, %i"},
    {0x0de00000, 0x00800000, "add%c%s\t%r3, %r4, %i"},
    {0x0de00000, 0x00a00000, "adc%c%s\t%r3, %r4, %i"},
    {0x0de00000, 0x00c00000, "sbc%c%s\t%r3, %r4, %i"},
    {0x0de00000, 0x00e00000, "rsc%c%s\t%r3, %r4, %i"},
    {0x0de00000, 0x01000000, "tst%c%s\t%r4, %i"},
    {0x0de00000, 0x01200000, "teq%c%s\t%r4, %i"},
    {0x0de00000, 0x01400000, "cmp%c%s\t%r4, %i"},
    {0x0de00000, 0x01600000, "cmn%c%s\t%r4, %i"},
    {0x0de00000, 0x01800000, "orr%c%s\t%r3, %r4, %i"},
    {0x0de00000, 0x01a00000, "mov%c%s\t%r3, %i"},
    {0x0de00000, 0x01c00000, "bic%c%s\t%r3, %r4, %i"},
    {0x0de00000, 0x01e00000, "mvn%c%s\t%r3, %i"},
    // Coprocessor operations
    {0x0f000010, 0x0e000000, "cdp%c\t%P, %N, %r3, %R4, %R0%V"},
    {0x0e100000, 0x0c000000, "stc%c\t%L %P, %r3, %A"},
    {0x0f100010, 0x0e000010, "mcr%c\t%P, %N, %r3, %R4, %R0%V"},
    {0x0f100010, 0x0e100010, "mrc%c\t%P, %N, %r3, %R4, %R0%V"},
    // Unknown
    {0x00000000, 0x00000000, "(ill)\t%Q"}};

std::string Arm926ejDisassembler::DisassembleArm(u32 opcode, u32 current_instruction_address,
                                                 bool big_endian)
{
  const Opcodes* sp = ArmOpcodes;
  std::string dest;

  while (sp->cval != (opcode & sp->mask))
    sp++;

  const char* src = sp->mnemonic;
  while (*src)
  {
    if (*src != '%')
    {
      dest += *src++;
      continue;
    }

    switch (*++src)
    {
    case 'c':
      dest += Conditions[opcode >> 28];
      break;

    case 'r':
      dest += Regs[(opcode >> ((*(++src) - '0') * 4)) & 15];
      break;

    case 'o':
    {
      u32 off = opcode & 0xffffff;
      if (off & 0x800000)
        off |= 0xff000000;
      off <<= 2;
      dest += fmt::format("->0x{:08x}", current_instruction_address + 8u + off);
    }
    break;

    case 'i':
      if (opcode & (1 << 25))
      {
        int imm = opcode & 0xff;
        int rot = (opcode & 0xf00) >> 7;
        u32 val = (imm << (32 - rot)) | (imm >> rot);
        dest += fmt::format("#0x{:x}", val);
      }
      else
      {
        dest += Regs[opcode & 0x0f];
        int shi = (opcode >> 5) & 3;
        int sdw = (opcode >> 7) & 0x1f;
        if ((sdw == 0) && (shi == 3))
          shi = 4;
        if ((sdw) || (opcode & 0x10) || (shi))
        {
          dest += ", ";
          dest += Shifts[shi];
          if (opcode & 0x10)
          {
            dest += ' ';
            dest += Regs[(opcode >> 8) & 15];
          }
          else
          {
            if (sdw == 0 && ((shi == 1) || (shi == 2)))
              sdw = 32;
            if (shi != 4)
            {
              dest += fmt::format(" #0x{:02x}", sdw);
            }
          }
        }
      }
      break;

    case 'p':
      if (opcode & (1 << 22))
        dest += "spsr";
      else
        dest += "cpsr";
      if (opcode & 0x00F00000)
      {
        dest += '_';
        if (opcode & 0x00080000)
          dest += 'f';
        if (opcode & 0x00040000)
          dest += 's';
        if (opcode & 0x00020000)
          dest += 'x';
        if (opcode & 0x00010000)
          dest += 'c';
      }
      break;

    case 's':
      if (opcode & (1 << 20))
        dest += 's';
      break;

    case 'S':
      if (opcode & (1 << 22))
        dest += 's';
      break;

    case 'u':
      if (opcode & (1 << 22))
        dest += 's';
      else
        dest += 'u';
      break;

    case 'b':
      if (opcode & (1 << 22))
        dest += 'b';
      break;

    case 'a':
      if ((opcode & 0x076f0000) == 0x004f0000)
      {
        u32 adr = current_instruction_address + 8;
        u32 add = (opcode & 15) | ((opcode >> 8) & 0xf0);
        if (opcode & (1 << 23))
          adr += add;
        else
          adr -= add;
        dest += fmt::format("[0x{:08x}]", adr);
        // dest += " (=";
        // dest += '$';
        // dest += fmt::format("{:08x}", debuggerReadMemory(adr));
        // dest += ')';
      }
      if ((opcode & 0x072f0000) == 0x050f0000)
      {
        u32 adr = current_instruction_address + 8;
        if (opcode & (1 << 23))
          adr += opcode & 0xfff;
        else
          adr -= opcode & 0xfff;
        dest += fmt::format("[0x{:08x}]", adr);
        // dest += " (=";
        // dest += '$';
        // dest += fmt::format("{:08x}", debuggerReadMemory(adr));
        // dest += ')';
      }
      else
      {
        int reg = (opcode >> 16) & 15;
        dest += '[';
        dest += Regs[reg];
        if (!(opcode & (1 << 24)))
          dest += ']';
        if (((opcode & (1 << 25)) && (opcode & (1 << 26))) ||
            (!(opcode & (1 << 22)) && !(opcode & (1 << 26))))
        {
          dest += ", ";
          if (!(opcode & (1 << 23)))
            dest += '-';
          dest += Regs[opcode & 0x0f];
          int shi = (opcode >> 5) & 3;
          if (opcode & (1 << 26))
          {
            if (((opcode >> 7) & 0x1f) || (opcode & 0x10) || (shi == 1) || (shi == 2))
            {
              dest += ", ";
              dest += Shifts[shi];
              if (opcode & 0x10)
              {
                dest += ' ';
                dest += Regs[(opcode >> 8) & 15];
              }
              else
              {
                u32 sdw = (opcode >> 7) & 0x1f;
                if (sdw == 0 && ((shi == 1) || (shi == 2)))
                  sdw = 32;
                dest += fmt::format(" #0x{:02x}", sdw);
              }
            }
          }
        }
        else
        {
          u32 off;
          if (opcode & (1 << 26))
            off = opcode & 0xfff;
          else
            off = (opcode & 15) | ((opcode >> 4) & 0xf0);
          if (off)
          {
            dest += ", #";
            if (!(opcode & (1 << 23)))
              dest += '-';
            dest += fmt::format("0x{:x}", off);
          }
        }
        if (opcode & (1 << 24))
        {
          dest += ']';
          if (opcode & (1 << 21))
            dest += '!';
        }
      }
      break;

    case 't':
      if ((opcode & 0x01200000) == 0x01200000)
        dest += 't';
      break;

    case 'h':
      if (opcode & (1 << 6))
        dest += 's';
      if (opcode & (1 << 5))
        dest += 'h';
      else
        dest += 'b';
      break;

    case 'm':
      if (((opcode >> 16) & 15) == 13)
      {
        if (opcode & 0x00100000)
          dest += MultLoadStore[8 + ((opcode >> 23) & 3)];
        else
          dest += MultLoadStore[4 + ((opcode >> 23) & 3)];
      }
      else
        dest += MultLoadStore[(opcode >> 23) & 3];
      break;

    case 'l':
      if (opcode & (1 << 21))
        dest += '!';
      dest += ", {";
      {
        int rlst = opcode & 0xffff;
        int msk = 0;
        int not_first = 0;
        while (msk < 16)
        {
          if (rlst & (1 << msk))
          {
            int fr = msk;
            while (rlst & (1 << msk))
              msk++;
            int to = msk - 1;
            if (not_first)
              // dest += ", ";
              dest += ',';
            dest += Regs[fr];
            if (fr != to)
            {
              if (fr == to - 1)
                // dest = addStr(", ");
                dest += ',';
              else
                dest += '-';
              dest += Regs[to];
            }
            not_first = 1;
          }
          else
            msk++;
        }
        dest += '}';
        if (opcode & (1 << 22))
          dest += '^';
      }
      break;

    case 'q':
      dest += fmt::format("0x{:06x}", opcode & 0xffffff);
      break;

    case 'Q':
      dest += fmt::format("0x{:08x}", opcode);
      break;

    case 'P':
      dest += 'p';
      dest += DecVals[(opcode >> 8) & 15];
      break;

    case 'N':
      if (opcode & 0x10)
        dest += DecVals[(opcode >> 21) & 7];
      else
        dest += DecVals[(opcode >> 20) & 15];
      break;

    case 'R':
    {
      src++;
      int reg = 4 * (*src - '0');
      dest += 'c';
      dest += DecVals[(opcode >> reg) & 15];
    }
    break;

    case 'V':
    {
      int val = (opcode >> 5) & 7;
      if (val)
      {
        dest += ", ";
        dest += DecVals[val];
      }
    }
    break;

    case 'L':
      if (opcode & (1 << 22))
        dest += 'l';
      break;

    case 'A':
      if ((opcode & 0x012f0000) == 0x010f0000)
      {
        u32 adr = current_instruction_address + 8;
        u32 add = (opcode & 0xff) << 2;
        if (opcode & (1 << 23))
          adr += add;
        else
          adr -= add;
        dest += fmt::format("0x{:08x}", adr);
      }
      else
      {
        dest += '[';
        dest += Regs[(opcode >> 16) & 15];
        if (!(opcode & (1 << 24)))
          dest += ']';
        u32 off = (opcode & 0xff) << 2;
        if (off)
        {
          dest += ", ";
          if (!(opcode & (1 << 23)))
            dest += '-';
          dest += fmt::format("#0x{:x}", off);
        }
        if (opcode & (1 << 24))
        {
          dest += ']';
          if (opcode & (1 << 21))
            dest += '!';
        }
      }
      break;
    }
    src++;
  }

  return dest;
}

std::string Arm926ejDisassembler::DisassembleThumb(u32 opcode, u32 current_instruction_address,
                                                   bool big_endian)
{
  const Opcodes* sp = ThumbOpcodes;
  std::string dest;

  while (sp->cval != (opcode & sp->mask))
    sp++;

  const char* src = sp->mnemonic;
  while (*src)
  {
    if (*src != '%')
    {
      dest += *src++;
      continue;
    }

    switch (*++src)
    {
    case 'r':
      src++;
      dest += Regs[(opcode >> (*src - '0')) & 7];
      break;

    case 'o':
    {
      int val = (opcode >> 6) & 0x1f;
      dest += fmt::format("#0x{:x}", val);
    }
    break;

    case 'p':
    {
      int val = (opcode >> 6) & 0x1f;
      if (!(opcode & (1 << 12)))
        val <<= 2;
      dest += fmt::format("#0x{:x}", val);
    }
    break;

    case 'e':
      dest += fmt::format("#0x{:x}", ((opcode >> 6) & 0x1f) << 1);
      break;

    case 'i':
      dest += fmt::format("#0x{:x}", (opcode >> 6) & 7);
      break;

    case 'h':
    {
      src++;
      int reg = (opcode >> (*src - '0')) & 7;
      src++;
      if (opcode & (1 << (*src - '0')))
        reg += 8;
      dest += Regs[reg];
    }
    break;

    case 'O':
      dest += fmt::format("#0x{:x}", (opcode & 0xff));
      break;

    case 'I':
      dest += fmt::format("0x{:08x}",
                          (current_instruction_address & 0xfffffffc) + 4 + ((opcode & 0xff) << 2));
      break;

    case 'b':
      if (opcode & (1 << 10))
        dest += 'b';
      break;

    case 'B':
      if (opcode & (1 << 12))
        dest += 'b';
      break;

    case 'w':
      dest += fmt::format("#0x{:x}", (opcode & 0xff) << 2);
      break;

    case 'W':
    {
      u32 add = opcode & 0xff;
      if (add & 0x80)
        add |= 0xffffff00;
      dest +=
          fmt::format("->0x{:08x}", (current_instruction_address & 0xfffffffeu) + 4u + (add << 1));
    }
    break;

    case 'c':
      dest += Conditions[(opcode >> 8) & 15];
      break;

    case 's':
      dest += "#";
      if (opcode & (1 << 7))
        dest += '-';
      dest += fmt::format("0x{:x}", (opcode & 0x7f) << 2);
      break;

    case 'l':
    {
      int rlst = opcode & 0xff;
      int msk = 0;
      int not_first = 0;
      while (msk < 8)
      {
        if (rlst & (1 << msk))
        {
          int fr = msk;
          while (rlst & (1 << msk))
            msk++;
          int to = msk - 1;
          if (not_first)
            dest += ',';
          dest += Regs[fr];
          if (fr != to)
          {
            if (fr == to - 1)
              dest += ',';
            else
              dest += '-';
            dest += Regs[to];
          }
          not_first = 1;
        }
        else
          msk++;
      }
    }
    break;

    case 'm':
      dest += fmt::format("${:02x}", opcode & 0xff);
      break;

    case 'Z':
      dest += fmt::format("${:04x}", (opcode & 0x7ff) << 1);
      break;

    case 'a':
    {
      int add = opcode & 0x07ff;
      if (add & 0x400)
        add |= 0xfffff800;
      add <<= 1;
      dest += fmt::format("0x{:08x}", current_instruction_address + 4 + add);
    }
    break;

    case 'Q':
      dest += fmt::format("{:04x}", opcode & 0xffff);
      break;
    }

    src++;
  }

  return dest;
}

}  // namespace Common