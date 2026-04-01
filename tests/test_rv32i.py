"""
Tests for the RV32I base integer ISA simulator.
"""

import struct
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from riscv.rv32i import RV32ICore, _u32, _s32, _sign_extend


# ---------------------------------------------------------------------------
# Helpers: tiny assembler (hand-encoded instructions)
# ---------------------------------------------------------------------------

def enc_r(funct7, rs2, rs1, funct3, rd, opcode):
    return (funct7 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode

def enc_i(imm12, rs1, funct3, rd, opcode):
    imm = imm12 & 0xFFF
    return (imm << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode

def enc_s(imm12, rs2, rs1, funct3, opcode):
    imm = imm12 & 0xFFF
    hi = (imm >> 5) & 0x7F
    lo = imm & 0x1F
    return (hi << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (lo << 7) | opcode

def enc_b(imm13, rs2, rs1, funct3, opcode=0x63):
    imm = imm13 & 0x1FFF
    b12   = (imm >> 12) & 1
    b11   = (imm >> 11) & 1
    b10_5 = (imm >> 5)  & 0x3F
    b4_1  = (imm >> 1)  & 0xF
    return (b12 << 31) | (b10_5 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (b4_1 << 8) | (b11 << 7) | opcode

def enc_u(imm20, rd, opcode):
    return ((imm20 & 0xFFFFF) << 12) | (rd << 7) | opcode

def enc_j(imm21, rd, opcode=0x6F):
    imm = imm21 & 0x1FFFFF
    b20    = (imm >> 20) & 1
    b19_12 = (imm >> 12) & 0xFF
    b11    = (imm >> 11) & 1
    b10_1  = (imm >> 1)  & 0x3FF
    return (b20 << 31) | (b10_1 << 21) | (b11 << 20) | (b19_12 << 12) | (rd << 7) | opcode

def ebreak():
    return enc_i(1, 0, 0, 0, 0x73)

def pack_words(*words):
    return struct.pack(f"<{len(words)}I", *words)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def core():
    return RV32ICore()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def run_program(words, regs_init=None):
    """Load *words*, optionally pre-set registers, run until EBREAK."""
    c = RV32ICore()
    code = pack_words(*words, ebreak())
    c.load_program(code)
    if regs_init:
        for r, v in regs_init.items():
            c.set_reg(r, v)
    c.run()
    return c


# ===========================================================================
# Helpers / sign_extend
# ===========================================================================

def test_sign_extend_positive():
    assert _sign_extend(0x7F, 8) == 127

def test_sign_extend_negative():
    assert _sign_extend(0x80, 8) == -128

def test_sign_extend_12bit():
    assert _sign_extend(0x800, 12) == -2048

def test_u32():
    assert _u32(-1) == 0xFFFF_FFFF

def test_s32():
    assert _s32(0xFFFF_FFFF) == -1


# ===========================================================================
# R-type
# ===========================================================================

class TestRType:
    def test_add(self):
        c = run_program(
            [enc_r(0, 2, 1, 0x0, 3, 0x33)],
            {1: 10, 2: 20},
        )
        assert c.reg(3) == 30

    def test_sub(self):
        c = run_program(
            [enc_r(0x20, 2, 1, 0x0, 3, 0x33)],
            {1: 30, 2: 10},
        )
        assert c.reg(3) == 20

    def test_and(self):
        c = run_program(
            [enc_r(0, 2, 1, 0x7, 3, 0x33)],
            {1: 0xFF00, 2: 0xF0F0},
        )
        assert c.reg(3) == 0xF000

    def test_or(self):
        c = run_program(
            [enc_r(0, 2, 1, 0x6, 3, 0x33)],
            {1: 0xFF00, 2: 0x00FF},
        )
        assert c.reg(3) == 0xFFFF

    def test_xor(self):
        c = run_program(
            [enc_r(0, 2, 1, 0x4, 3, 0x33)],
            {1: 0xFFFF, 2: 0x0F0F},
        )
        assert c.reg(3) == 0xF0F0

    def test_sll(self):
        c = run_program(
            [enc_r(0, 2, 1, 0x1, 3, 0x33)],
            {1: 1, 2: 4},
        )
        assert c.reg(3) == 16

    def test_srl(self):
        c = run_program(
            [enc_r(0, 2, 1, 0x5, 3, 0x33)],
            {1: 0x80000000, 2: 1},
        )
        assert c.reg(3) == 0x40000000

    def test_sra(self):
        c = run_program(
            [enc_r(0x20, 2, 1, 0x5, 3, 0x33)],
            {1: 0x80000000, 2: 1},
        )
        assert c.reg(3) == 0xC0000000

    def test_slt_true(self):
        c = run_program(
            [enc_r(0, 2, 1, 0x2, 3, 0x33)],
            {1: _u32(-1), 2: 1},
        )
        assert c.reg(3) == 1

    def test_sltu_false(self):
        c = run_program(
            [enc_r(0, 2, 1, 0x3, 3, 0x33)],
            {1: 0xFFFFFFFF, 2: 1},
        )
        assert c.reg(3) == 0

    def test_x0_always_zero(self):
        c = run_program(
            [enc_r(0, 2, 1, 0x0, 0, 0x33)],   # write to x0
            {1: 5, 2: 10},
        )
        assert c.reg(0) == 0


# ===========================================================================
# I-type ALU
# ===========================================================================

class TestITypeALU:
    def test_addi(self):
        c = run_program([enc_i(-1, 1, 0x0, 2, 0x13)], {1: 10})
        assert c.reg(2) == 9

    def test_andi(self):
        c = run_program([enc_i(0xFF, 1, 0x7, 2, 0x13)], {1: 0x1234})
        assert c.reg(2) == 0x34

    def test_ori(self):
        c = run_program([enc_i(0x0F, 1, 0x6, 2, 0x13)], {1: 0xF0})
        assert c.reg(2) == 0xFF

    def test_xori(self):
        c = run_program([enc_i(0xFF, 1, 0x4, 2, 0x13)], {1: 0x0F})
        assert c.reg(2) == 0xF0

    def test_slli(self):
        c = run_program([enc_i(3, 1, 0x1, 2, 0x13)], {1: 1})
        assert c.reg(2) == 8

    def test_srli(self):
        c = run_program([enc_i(1, 1, 0x5, 2, 0x13)], {1: 0x80000000})
        assert c.reg(2) == 0x40000000

    def test_srai(self):
        imm = 0x400 | 1   # funct7=0x20, shamt=1
        c = run_program([enc_i(imm, 1, 0x5, 2, 0x13)], {1: 0x80000000})
        assert c.reg(2) == 0xC0000000

    def test_slti(self):
        c = run_program([enc_i(-1, 1, 0x2, 2, 0x13)], {1: _u32(-2)})
        assert c.reg(2) == 1

    def test_sltiu(self):
        # sltiu x2, x1, 1  → x1 (unsigned) < 1 → false (x1=5)
        c = run_program([enc_i(1, 1, 0x3, 2, 0x13)], {1: 5})
        assert c.reg(2) == 0


# ===========================================================================
# Load / Store
# ===========================================================================

class TestLoadStore:
    def test_sw_lw_roundtrip(self):
        c = RV32ICore()
        # Place data area at 0x100
        code = pack_words(
            enc_s(0x100, 2, 0, 0x2, 0x23),   # SW x2, 0x100(x0)
            enc_i(0x100, 0, 0x2, 3, 0x03),   # LW x3, 0x100(x0)
            ebreak(),
        )
        c.load_program(code)
        c.set_reg(2, 0xDEADBEEF)
        c.run()
        assert c.reg(3) == 0xDEADBEEF

    def test_sb_lb(self):
        c = RV32ICore()
        code = pack_words(
            enc_s(0x200, 2, 0, 0x0, 0x23),   # SB x2, 0x200(x0)
            enc_i(0x200, 0, 0x0, 3, 0x03),   # LB x3, 0x200(x0)
            ebreak(),
        )
        c.load_program(code)
        c.set_reg(2, 0xFF)   # only lowest byte stored
        c.run()
        assert c.reg(3) == _u32(-1)  # sign-extended 0xFF → -1

    def test_lbu(self):
        c = RV32ICore()
        code = pack_words(
            enc_s(0x300, 2, 0, 0x0, 0x23),   # SB x2, 0x300(x0)
            enc_i(0x300, 0, 0x4, 3, 0x03),   # LBU x3, 0x300(x0)
            ebreak(),
        )
        c.load_program(code)
        c.set_reg(2, 0xFF)
        c.run()
        assert c.reg(3) == 0xFF      # zero-extended

    def test_sh_lh(self):
        c = RV32ICore()
        code = pack_words(
            enc_s(0x400, 2, 0, 0x1, 0x23),   # SH x2, 0x400(x0)
            enc_i(0x400, 0, 0x1, 3, 0x03),   # LH x3, 0x400(x0)
            ebreak(),
        )
        c.load_program(code)
        c.set_reg(2, 0x8001)
        c.run()
        assert c.reg(3) == _u32(_sign_extend(0x8001, 16))


# ===========================================================================
# Branch
# ===========================================================================

class TestBranch:
    def test_beq_taken(self):
        # beq x1, x2, +8  (skip addi that would set x3=99)
        c = RV32ICore()
        code = pack_words(
            enc_b(8, 2, 1, 0x0),              # BEQ x1,x2 → skip next
            enc_i(99, 0, 0x0, 3, 0x13),       # ADDI x3, x0, 99  ← skipped
            ebreak(),
        )
        c.load_program(code)
        c.set_reg(1, 5)
        c.set_reg(2, 5)
        c.run()
        assert c.reg(3) == 0   # not executed

    def test_beq_not_taken(self):
        c = RV32ICore()
        code = pack_words(
            enc_b(8, 2, 1, 0x0),              # BEQ x1,x2 → not taken
            enc_i(99, 0, 0x0, 3, 0x13),       # ADDI x3, x0, 99  ← executed
            ebreak(),
        )
        c.load_program(code)
        c.set_reg(1, 1)
        c.set_reg(2, 2)
        c.run()
        assert c.reg(3) == 99

    def test_bne(self):
        c = RV32ICore()
        code = pack_words(
            enc_b(8, 2, 1, 0x1),              # BNE x1,x2 → taken
            enc_i(99, 0, 0x0, 3, 0x13),       # ADDI x3, x0, 99  ← skipped
            ebreak(),
        )
        c.load_program(code)
        c.set_reg(1, 1)
        c.set_reg(2, 2)
        c.run()
        assert c.reg(3) == 0

    def test_blt(self):
        c = run_program(
            [enc_b(8, 2, 1, 0x4), enc_i(99, 0, 0x0, 3, 0x13)],
            {1: _u32(-1), 2: 1},
        )
        assert c.reg(3) == 0  # branch taken (-1 < 1 signed)

    def test_bltu(self):
        # unsigned: x1=1 < x2=2
        c = run_program(
            [enc_b(8, 2, 1, 0x6), enc_i(99, 0, 0x0, 3, 0x13)],
            {1: 1, 2: 2},
        )
        assert c.reg(3) == 0  # branch taken


# ===========================================================================
# LUI / AUIPC
# ===========================================================================

class TestUType:
    def test_lui(self):
        c = run_program([enc_u(0xDEADB, 1, 0x37)])
        assert c.reg(1) == 0xDEADB000

    def test_auipc(self):
        c = run_program([enc_u(1, 1, 0x17)])  # AUIPC x1, 1  (pc=0)
        assert c.reg(1) == 0x1000


# ===========================================================================
# JAL / JALR
# ===========================================================================

class TestJump:
    def test_jal(self):
        c = RV32ICore()
        # JAL x1, +8  → skip addi; land on ebreak
        code = pack_words(
            enc_j(8, 1),                        # JAL x1, +8
            enc_i(99, 0, 0x0, 3, 0x13),         # ADDI x3, x0, 99  ← skipped
            ebreak(),
        )
        c.load_program(code)
        c.run()
        assert c.reg(3) == 0
        assert c.reg(1) == 4   # return address

    def test_jalr(self):
        c = RV32ICore()
        # Set x2 = 12, then JALR x1, x2, 0 → jump to addr 12 (ebreak),
        # skipping the ADDI x3, x0, 99 at addr 8.
        code = pack_words(
            enc_i(12, 0, 0x0, 2, 0x13),         # ADDI x2, x0, 12
            enc_i(0, 2, 0x0, 1, 0x67),          # JALR x1, x2, 0  → PC=12
            enc_i(99, 0, 0x0, 3, 0x13),         # ADDI x3, x0, 99  ← skipped
            ebreak(),
        )
        c.load_program(code)
        c.run()
        assert c.reg(3) == 0
        assert c.reg(1) == 8   # return address = PC+4 of JALR = 4+4


# ===========================================================================
# ECALL (exit)
# ===========================================================================

def test_ecall_exit():
    c = RV32ICore()
    # a7 = 93 (exit syscall)
    code = pack_words(
        enc_i(93, 0, 0x0, 17, 0x13),   # ADDI x17, x0, 93
        0x00000073,                      # ECALL
        enc_i(99, 0, 0x0, 1, 0x13),    # ADDI x1, x0, 99  ← never reached
    )
    c.load_program(code)
    c.run()
    assert c.halted
    assert c.reg(1) == 0


# ===========================================================================
# Memory bounds
# ===========================================================================

def test_memory_out_of_bounds():
    c = RV32ICore(mem_size=128)
    with pytest.raises(MemoryError):
        c.mem.read_word(200)
