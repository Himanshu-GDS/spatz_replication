"""
RV32I base integer instruction-set simulator.

This module provides a functional model of the RISC-V 32-bit base integer ISA
(RV32I) as defined in the RISC-V Unprivileged ISA specification.  It is used
as the scalar host core in the Spatz replication project.

Supported instructions
----------------------
  R-type : ADD, SUB, AND, OR, XOR, SLL, SRL, SRA, SLT, SLTU
  I-type : ADDI, ANDI, ORI, XORI, SLLI, SRLI, SRAI, SLTI, SLTIU
           LB, LH, LW, LBU, LHU
           JALR
  S-type : SB, SH, SW
  B-type : BEQ, BNE, BLT, BGE, BLTU, BGEU
  U-type : LUI, AUIPC
  J-type : JAL
  System : ECALL, EBREAK
"""

import struct
from typing import Optional


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _sign_extend(value: int, bits: int) -> int:
    """Return the signed interpretation of *value* using *bits* bits."""
    sign_bit = 1 << (bits - 1)
    return (value & (sign_bit - 1)) - (value & sign_bit)


def _u32(value: int) -> int:
    """Mask to unsigned 32-bit integer."""
    return value & 0xFFFF_FFFF


def _s32(value: int) -> int:
    """Interpret *value* as a signed 32-bit integer."""
    return _sign_extend(_u32(value), 32)


# ---------------------------------------------------------------------------
# Memory
# ---------------------------------------------------------------------------

class Memory:
    """Flat byte-addressable memory."""

    def __init__(self, size: int = 1 << 20) -> None:
        self._data = bytearray(size)
        self.size = size

    # ------------------------------------------------------------------
    # Raw access
    # ------------------------------------------------------------------

    def read_byte(self, addr: int) -> int:
        self._check(addr, 1)
        return self._data[addr]

    def write_byte(self, addr: int, value: int) -> None:
        self._check(addr, 1)
        self._data[addr] = value & 0xFF

    def read_half(self, addr: int) -> int:
        self._check(addr, 2)
        return struct.unpack_from("<H", self._data, addr)[0]

    def write_half(self, addr: int, value: int) -> None:
        self._check(addr, 2)
        struct.pack_into("<H", self._data, addr, value & 0xFFFF)

    def read_word(self, addr: int) -> int:
        self._check(addr, 4)
        return struct.unpack_from("<I", self._data, addr)[0]

    def write_word(self, addr: int, value: int) -> None:
        self._check(addr, 4)
        struct.pack_into("<I", self._data, addr, _u32(value))

    # ------------------------------------------------------------------
    # Bulk load
    # ------------------------------------------------------------------

    def load(self, addr: int, data: bytes) -> None:
        """Load *data* bytes starting at *addr*."""
        end = addr + len(data)
        if end > self.size:
            raise MemoryError(f"load out of range: 0x{addr:08x}+{len(data)}")
        self._data[addr:end] = data

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _check(self, addr: int, width: int) -> None:
        if addr < 0 or addr + width > self.size:
            raise MemoryError(f"address out of range: 0x{addr:08x}")


# ---------------------------------------------------------------------------
# Core
# ---------------------------------------------------------------------------

_REG_NAMES = [
    "zero", "ra", "sp",  "gp",  "tp",  "t0",  "t1",  "t2",
    "s0",   "s1", "a0",  "a1",  "a2",  "a3",  "a4",  "a5",
    "a6",   "a7", "s2",  "s3",  "s4",  "s5",  "s6",  "s7",
    "s8",   "s9", "s10", "s11", "t3",  "t4",  "t5",  "t6",
]


class RV32ICore:
    """
    Functional model of an RV32I processor core.

    Parameters
    ----------
    mem_size:
        Size of the flat memory in bytes (default 1 MiB).
    pc_init:
        Initial program counter value.
    """

    def __init__(self, mem_size: int = 1 << 20, pc_init: int = 0) -> None:
        self.mem = Memory(mem_size)
        self.regs: list[int] = [0] * 32   # x0 … x31 (x0 always 0)
        self.pc: int = pc_init
        self.halted: bool = False
        self._step_count: int = 0
        # Callback for unrecognised/system instructions (used by Simulator)
        self._trap_handler: Optional[object] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def load_program(self, code: bytes, base: int = 0) -> None:
        """Load *code* into memory at *base* and reset PC to *base*."""
        self.mem.load(base, code)
        self.pc = base
        self.halted = False

    def reg(self, index: int) -> int:
        """Return the value of integer register *index* (unsigned 32-bit)."""
        return self.regs[index] if index != 0 else 0

    def set_reg(self, index: int, value: int) -> None:
        """Write *value* to integer register *index*."""
        if index != 0:
            self.regs[index] = _u32(value)

    def step(self) -> None:
        """Execute a single instruction."""
        if self.halted:
            return
        raw = self.mem.read_word(self.pc)
        self._decode_execute(raw)
        self._step_count += 1

    def run(self, max_steps: int = 10_000) -> int:
        """
        Run until ``halted`` is set or *max_steps* is reached.

        Returns the number of steps executed.
        """
        steps = 0
        while not self.halted and steps < max_steps:
            self.step()
            steps += 1
        return steps

    def dump_regs(self) -> str:
        """Return a human-readable register dump."""
        lines = []
        for i in range(0, 32, 4):
            row = "  ".join(
                f"x{i+j:02d}({_REG_NAMES[i+j]:4s})={self.regs[i+j]:08x}"
                for j in range(4)
            )
            lines.append(row)
        lines.append(f"  pc={self.pc:08x}")
        return "\n".join(lines)

    # ------------------------------------------------------------------
    # Instruction decode/execute
    # ------------------------------------------------------------------

    def _decode_execute(self, raw: int) -> None:  # noqa: C901 (complexity OK)
        opcode = raw & 0x7F

        # ---- R-type -------------------------------------------------------
        if opcode == 0x33:
            rd, funct3, rs1, rs2, funct7 = self._r(raw)
            a, b = self.reg(rs1), self.reg(rs2)
            sa = _s32(a)
            sb = _s32(b)
            if funct3 == 0x0:
                res = (a + b) if funct7 == 0x00 else (_u32(a - b))  # ADD/SUB
            elif funct3 == 0x4:
                res = a ^ b                          # XOR
            elif funct3 == 0x6:
                res = a | b                          # OR
            elif funct3 == 0x7:
                res = a & b                          # AND
            elif funct3 == 0x1:
                res = a << (b & 0x1F)               # SLL
            elif funct3 == 0x5:
                res = (a >> (b & 0x1F)) if funct7 == 0x00 else _u32(sa >> (b & 0x1F))  # SRL/SRA
            elif funct3 == 0x2:
                res = 1 if sa < sb else 0            # SLT
            elif funct3 == 0x3:
                res = 1 if a < b else 0             # SLTU
            else:
                self._illegal(raw)
                return
            self.set_reg(rd, res)
            self.pc += 4
            return

        # ---- I-type (ALU) -------------------------------------------------
        if opcode == 0x13:
            rd, funct3, rs1, imm = self._i(raw)
            a = self.reg(rs1)
            sa = _s32(a)
            shamt = imm & 0x1F
            funct7 = (imm >> 5) & 0x7F
            if funct3 == 0x0:
                res = a + imm                        # ADDI
            elif funct3 == 0x4:
                res = a ^ imm                        # XORI
            elif funct3 == 0x6:
                res = a | imm                        # ORI
            elif funct3 == 0x7:
                res = a & imm                        # ANDI
            elif funct3 == 0x1:
                res = a << shamt                     # SLLI
            elif funct3 == 0x5:
                res = (a >> shamt) if funct7 == 0x00 else _u32(sa >> shamt)  # SRLI/SRAI
            elif funct3 == 0x2:
                res = 1 if sa < imm else 0           # SLTI
            elif funct3 == 0x3:
                res = 1 if a < _u32(imm) else 0     # SLTIU
            else:
                self._illegal(raw)
                return
            self.set_reg(rd, res)
            self.pc += 4
            return

        # ---- Load ---------------------------------------------------------
        if opcode == 0x03:
            rd, funct3, rs1, imm = self._i(raw)
            addr = _u32(self.reg(rs1) + imm)
            if funct3 == 0x0:
                res = _sign_extend(self.mem.read_byte(addr), 8)   # LB
            elif funct3 == 0x4:
                res = self.mem.read_byte(addr)                    # LBU
            elif funct3 == 0x1:
                res = _sign_extend(self.mem.read_half(addr), 16)  # LH
            elif funct3 == 0x5:
                res = self.mem.read_half(addr)                    # LHU
            elif funct3 == 0x2:
                res = self.mem.read_word(addr)                    # LW
            else:
                self._illegal(raw)
                return
            self.set_reg(rd, res)
            self.pc += 4
            return

        # ---- Store --------------------------------------------------------
        if opcode == 0x23:
            funct3, rs1, rs2, imm = self._s(raw)
            addr = _u32(self.reg(rs1) + imm)
            val = self.reg(rs2)
            if funct3 == 0x0:
                self.mem.write_byte(addr, val)   # SB
            elif funct3 == 0x1:
                self.mem.write_half(addr, val)   # SH
            elif funct3 == 0x2:
                self.mem.write_word(addr, val)   # SW
            else:
                self._illegal(raw)
                return
            self.pc += 4
            return

        # ---- Branch -------------------------------------------------------
        if opcode == 0x63:
            funct3, rs1, rs2, imm = self._b(raw)
            a, b = self.reg(rs1), self.reg(rs2)
            sa, sb = _s32(a), _s32(b)
            taken = False
            if funct3 == 0x0:
                taken = a == b    # BEQ
            elif funct3 == 0x1:
                taken = a != b    # BNE
            elif funct3 == 0x4:
                taken = sa < sb   # BLT
            elif funct3 == 0x5:
                taken = sa >= sb  # BGE
            elif funct3 == 0x6:
                taken = a < b     # BLTU
            elif funct3 == 0x7:
                taken = a >= b    # BGEU
            else:
                self._illegal(raw)
                return
            self.pc = _u32(self.pc + imm) if taken else self.pc + 4
            return

        # ---- LUI ----------------------------------------------------------
        if opcode == 0x37:
            rd, imm = self._u(raw)
            self.set_reg(rd, imm)
            self.pc += 4
            return

        # ---- AUIPC --------------------------------------------------------
        if opcode == 0x17:
            rd, imm = self._u(raw)
            self.set_reg(rd, _u32(self.pc + imm))
            self.pc += 4
            return

        # ---- JAL ----------------------------------------------------------
        if opcode == 0x6F:
            rd, imm = self._j(raw)
            self.set_reg(rd, self.pc + 4)
            self.pc = _u32(self.pc + imm)
            return

        # ---- JALR ---------------------------------------------------------
        if opcode == 0x67:
            rd, funct3, rs1, imm = self._i(raw)
            target = _u32((self.reg(rs1) + imm) & ~1)
            self.set_reg(rd, self.pc + 4)
            self.pc = target
            return

        # ---- SYSTEM / ECALL / EBREAK --------------------------------------
        if opcode == 0x73:
            funct12 = (raw >> 20) & 0xFFF
            if funct12 == 0x000:
                self._ecall()
            elif funct12 == 0x001:
                self.halted = True
            else:
                self._illegal(raw)
            return

        # ---- Unknown opcode – forward to trap handler --------------------
        if self._trap_handler is not None:
            handled = self._trap_handler(raw, self)  # type: ignore[call-arg]
            if handled:
                return
        self._illegal(raw)

    # ------------------------------------------------------------------
    # Instruction format decoders
    # ------------------------------------------------------------------

    @staticmethod
    def _r(raw: int):
        rd     = (raw >> 7)  & 0x1F
        funct3 = (raw >> 12) & 0x07
        rs1    = (raw >> 15) & 0x1F
        rs2    = (raw >> 20) & 0x1F
        funct7 = (raw >> 25) & 0x7F
        return rd, funct3, rs1, rs2, funct7

    @staticmethod
    def _i(raw: int):
        rd     = (raw >> 7)  & 0x1F
        funct3 = (raw >> 12) & 0x07
        rs1    = (raw >> 15) & 0x1F
        imm    = _sign_extend((raw >> 20) & 0xFFF, 12)
        return rd, funct3, rs1, imm

    @staticmethod
    def _s(raw: int):
        funct3 = (raw >> 12) & 0x07
        rs1    = (raw >> 15) & 0x1F
        rs2    = (raw >> 20) & 0x1F
        imm    = _sign_extend(((raw >> 25) << 5) | ((raw >> 7) & 0x1F), 12)
        return funct3, rs1, rs2, imm

    @staticmethod
    def _b(raw: int):
        funct3 = (raw >> 12) & 0x07
        rs1    = (raw >> 15) & 0x1F
        rs2    = (raw >> 20) & 0x1F
        imm    = _sign_extend(
            ((raw >> 31) << 12) |
            (((raw >> 7) & 1) << 11) |
            (((raw >> 25) & 0x3F) << 5) |
            (((raw >> 8) & 0xF) << 1),
            13,
        )
        return funct3, rs1, rs2, imm

    @staticmethod
    def _u(raw: int):
        rd  = (raw >> 7) & 0x1F
        imm = _sign_extend(raw & 0xFFFFF000, 32)
        return rd, imm

    @staticmethod
    def _j(raw: int):
        rd  = (raw >> 7) & 0x1F
        imm = _sign_extend(
            ((raw >> 31) << 20) |
            (((raw >> 12) & 0xFF) << 12) |
            (((raw >> 20) & 1) << 11) |
            (((raw >> 21) & 0x3FF) << 1),
            21,
        )
        return rd, imm

    # ------------------------------------------------------------------
    # Traps / ECALL
    # ------------------------------------------------------------------

    def _ecall(self) -> None:
        """
        Minimal ECALL handler.

        syscall numbers:
          93  – exit   (code=a0)               – halts simulation
          64  – write  (fd=a0, buf=a1, len=a2) – prints to stdout
        """
        a7 = self.reg(17)
        if a7 == 93:  # exit
            self.halted = True
        elif a7 == 64:  # write (Linux ABI)
            buf  = self.reg(11)
            size = self.reg(12)
            try:
                data = bytes(self.mem._data[buf:buf + size])
                import sys
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()
            except Exception:
                pass
            self.set_reg(10, size)  # return value: bytes written
        self.pc += 4

    def _illegal(self, raw: int) -> None:
        raise RuntimeError(
            f"Illegal instruction 0x{raw:08x} at PC=0x{self.pc:08x}"
        )
