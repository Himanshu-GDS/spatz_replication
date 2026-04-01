"""
Tests for the RVV (RISC-V Vector Extension) unit – Spatz replication.

We hand-encode the minimal RVV binary encoding needed to exercise each
major instruction group, then verify register and memory state.
"""

import struct
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from riscv.simulator import Simulator
from riscv.rv32i import _u32


# ---------------------------------------------------------------------------
# Encoding helpers (shared with test_rv32i.py, inlined here for clarity)
# ---------------------------------------------------------------------------

def enc_i(imm12, rs1, funct3, rd, opcode):
    imm = imm12 & 0xFFF
    return (imm << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode

def enc_r(funct7, rs2, rs1, funct3, rd, opcode):
    return (funct7 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode

def ebreak():
    return enc_i(1, 0, 0, 0, 0x73)

def pack_words(*words):
    return struct.pack(f"<{len(words)}I", *words)


# ---------------------------------------------------------------------------
# RVV instruction encoders
# ---------------------------------------------------------------------------

def vsetvli(rd, rs1, vsew_idx, vlmul=0):
    """
    VSETVLI rd, rs1, imm11

    bit 11=0 (VSETVLI), bits 10:8 = 0 (vma/vta), bits 5:3 = vsew_idx, bits 2:0 = vlmul
    """
    zimm = (vsew_idx << 3) | (vlmul & 7)
    return (zimm << 20) | (rs1 << 15) | (0x7 << 12) | (rd << 7) | 0x57

def vle32(vd, rs1, vm=1):
    """VLE32.V vd, (rs1)  – unit-stride load, 32-bit elements."""
    # width=6 (EEW=32), mop=00 (unit-stride), nf=0
    width = 6
    mop   = 0b00
    nf    = 0
    mew   = 0
    lumop = 0   # unit-stride
    return (nf << 29) | (mew << 28) | (mop << 26) | (vm << 25) | (lumop << 20) | (rs1 << 15) | (width << 12) | (vd << 7) | 0x07

def vse32(vs3, rs1, vm=1):
    """VSE32.V vs3, (rs1)  – unit-stride store, 32-bit elements."""
    width = 6
    mop   = 0b00
    nf    = 0
    mew   = 0
    sumop = 0
    return (nf << 29) | (mew << 28) | (mop << 26) | (vm << 25) | (sumop << 20) | (rs1 << 15) | (width << 12) | (vs3 << 7) | 0x27

def vadd_vv(vd, vs2, vs1, vm=1):
    return (0x00 << 26) | (vm << 25) | (vs2 << 20) | (vs1 << 15) | (0x0 << 12) | (vd << 7) | 0x57

def vadd_vx(vd, vs2, rs1, vm=1):
    return (0x00 << 26) | (vm << 25) | (vs2 << 20) | (rs1 << 15) | (0x4 << 12) | (vd << 7) | 0x57

def vadd_vi(vd, vs2, imm5, vm=1):
    return (0x00 << 26) | (vm << 25) | (vs2 << 20) | (imm5 << 15) | (0x3 << 12) | (vd << 7) | 0x57

def vsub_vv(vd, vs2, vs1, vm=1):
    return (0x02 << 26) | (vm << 25) | (vs2 << 20) | (vs1 << 15) | (0x0 << 12) | (vd << 7) | 0x57

def vsub_vx(vd, vs2, rs1, vm=1):
    return (0x02 << 26) | (vm << 25) | (vs2 << 20) | (rs1 << 15) | (0x4 << 12) | (vd << 7) | 0x57

def vand_vv(vd, vs2, vs1, vm=1):
    return (0x09 << 26) | (vm << 25) | (vs2 << 20) | (vs1 << 15) | (0x0 << 12) | (vd << 7) | 0x57

def vor_vv(vd, vs2, vs1, vm=1):
    return (0x0A << 26) | (vm << 25) | (vs2 << 20) | (vs1 << 15) | (0x0 << 12) | (vd << 7) | 0x57

def vxor_vv(vd, vs2, vs1, vm=1):
    return (0x0B << 26) | (vm << 25) | (vs2 << 20) | (vs1 << 15) | (0x0 << 12) | (vd << 7) | 0x57

def vmul_vv(vd, vs2, vs1, vm=1):
    return (0x25 << 26) | (vm << 25) | (vs2 << 20) | (vs1 << 15) | (0x2 << 12) | (vd << 7) | 0x57

def vmv_vx(vd, rs1):
    """VMV.V.X  vd, rs1  (funct6=0x17, funct3=OPIVX=4, vm=1)"""
    return (0x17 << 26) | (1 << 25) | (0 << 20) | (rs1 << 15) | (0x4 << 12) | (vd << 7) | 0x57

def vredsum_vs(vd, vs2, vs1, vm=1):
    return (0x01 << 26) | (vm << 25) | (vs2 << 20) | (vs1 << 15) | (0x2 << 12) | (vd << 7) | 0x57

def vmv_xs(rd, vs2):
    """VMV.X.S  rd, vs2"""
    return (0x10 << 26) | (1 << 25) | (vs2 << 20) | (0 << 15) | (0x2 << 12) | (rd << 7) | 0x57

def vmseq_vv(vd, vs2, vs1, vm=1):
    return (0x18 << 26) | (vm << 25) | (vs2 << 20) | (vs1 << 15) | (0x0 << 12) | (vd << 7) | 0x57


# ---------------------------------------------------------------------------
# Helper: build a simulator pre-loaded with a program
# ---------------------------------------------------------------------------

DATA_BASE = 0x80000   # where test data lives (above code)

def make_sim(*words):
    sim = Simulator(mem_size=1 << 20)
    code = pack_words(*words, ebreak())
    sim.load_program(code)
    return sim


# ===========================================================================
# vsetvli
# ===========================================================================

class TestVsetvli:
    def test_sew32_lmul1(self):
        sim = make_sim(vsetvli(1, 2, vsew_idx=2, vlmul=0))
        sim.core.set_reg(2, 8)
        sim.run()
        assert sim.vcsr.sew == 32
        assert sim.vcsr.vl  == 8
        assert sim.core.reg(1) == 8

    def test_vl_capped_at_vlmax(self):
        sim = make_sim(vsetvli(1, 2, vsew_idx=2, vlmul=0))
        # VLEN=256 bits, SEW=32 → vlmax = 256/32 = 8
        sim.core.set_reg(2, 100)  # request more than vlmax
        sim.run()
        assert sim.vcsr.vl == 8
        assert sim.core.reg(1) == 8

    def test_sew8(self):
        sim = make_sim(vsetvli(1, 2, vsew_idx=0, vlmul=0))
        sim.core.set_reg(2, 32)
        sim.run()
        assert sim.vcsr.sew == 8
        assert sim.vcsr.vl == 32   # vlmax = 256/8 = 32


# ===========================================================================
# Vector load / store
# ===========================================================================

class TestVectorLoadStore:
    def _setup_data(self, sim, values, sew=32):
        """Write *values* to DATA_BASE and return the sim."""
        bw = sew // 8
        for i, v in enumerate(values):
            addr = DATA_BASE + i * bw
            if bw == 4:
                sim.mem.write_word(addr, v)
            elif bw == 2:
                sim.mem.write_half(addr, v)
            else:
                sim.mem.write_byte(addr, v)
        return sim

    def test_vle32_basic(self):
        n = 4
        sim = Simulator(mem_size=1 << 20)
        code = pack_words(
            vsetvli(0, 1, 2, 0),       # vsetvli x0, x1, e32, m1
            vle32(2, 3),               # vle32.v v2, (x3)
            ebreak(),
        )
        sim.load_program(code)
        sim.core.set_reg(1, n)
        sim.core.set_reg(3, DATA_BASE)
        data = [10, 20, 30, 40]
        self._setup_data(sim, data)
        sim.run()
        for i, expected in enumerate(data):
            assert sim.vregs.get_elem(2, i, 32) == expected

    def test_vse32_roundtrip(self):
        n = 4
        src = [1, 2, 3, 4]
        dst_base = DATA_BASE + 0x100

        sim = Simulator(mem_size=1 << 20)
        code = pack_words(
            vsetvli(0, 1, 2, 0),       # set vl=4, sew=32
            vle32(2, 3),               # load v2 ← src
            vse32(2, 4),               # store v2 → dst
            ebreak(),
        )
        sim.load_program(code)
        sim.core.set_reg(1, n)
        sim.core.set_reg(3, DATA_BASE)
        sim.core.set_reg(4, dst_base)
        self._setup_data(sim, src)
        sim.run()
        for i, expected in enumerate(src):
            assert sim.mem.read_word(dst_base + i * 4) == expected


# ===========================================================================
# Vector arithmetic
# ===========================================================================

class TestVectorArithmetic:
    def _prep(self, vs2_vals, vs1_vals, sew=32):
        """Return a Simulator with vs2 (v2) and vs1 (v3) pre-loaded."""
        src2 = DATA_BASE
        src1 = DATA_BASE + 0x100
        n = max(len(vs2_vals), len(vs1_vals))
        sim = Simulator(mem_size=1 << 20)
        instrs = [
            vsetvli(0, 10, 2, 0),
            vle32(2, 11),
            vle32(3, 12),
            ebreak(),
        ]
        sim.load_program(pack_words(*instrs))
        sim.core.set_reg(10, n)
        sim.core.set_reg(11, src2)
        sim.core.set_reg(12, src1)
        for i, v in enumerate(vs2_vals):
            sim.mem.write_word(src2 + i * 4, v)
        for i, v in enumerate(vs1_vals):
            sim.mem.write_word(src1 + i * 4, v)
        sim.run()
        return sim

    def test_vadd_vv(self):
        sim = self._prep([1, 2, 3, 4], [10, 20, 30, 40])
        # Now run vadd.vv v4, v2, v3
        code2 = pack_words(
            vsetvli(0, 10, 2, 0),
            vadd_vv(4, 2, 3),
            ebreak(),
        )
        sim.core.set_reg(10, 4)
        sim.load_program(code2)
        sim.run()
        expected = [11, 22, 33, 44]
        for i, e in enumerate(expected):
            assert sim.vregs.get_elem(4, i, 32) == e

    def test_vadd_vx(self):
        sim = Simulator(mem_size=1 << 20)
        instrs = [
            vsetvli(0, 10, 2, 0),
            vle32(2, 11),
            vadd_vx(4, 2, 5),   # v4 = v2 + x5
            ebreak(),
        ]
        sim.load_program(pack_words(*instrs))
        sim.core.set_reg(10, 4)
        sim.core.set_reg(11, DATA_BASE)
        sim.core.set_reg(5, 100)
        for i, v in enumerate([1, 2, 3, 4]):
            sim.mem.write_word(DATA_BASE + i * 4, v)
        sim.run()
        for i, e in enumerate([101, 102, 103, 104]):
            assert sim.vregs.get_elem(4, i, 32) == e

    def test_vsub_vv(self):
        sim = Simulator(mem_size=1 << 20)
        instrs = [
            vsetvli(0, 10, 2, 0),
            vle32(2, 11),
            vle32(3, 12),
            vsub_vv(4, 2, 3),   # v4 = v2 - v3
            ebreak(),
        ]
        sim.load_program(pack_words(*instrs))
        sim.core.set_reg(10, 4)
        sim.core.set_reg(11, DATA_BASE)
        sim.core.set_reg(12, DATA_BASE + 0x100)
        for i, v in enumerate([10, 20, 30, 40]):
            sim.mem.write_word(DATA_BASE + i * 4, v)
        for i, v in enumerate([1, 2, 3, 4]):
            sim.mem.write_word(DATA_BASE + 0x100 + i * 4, v)
        sim.run()
        for i, e in enumerate([9, 18, 27, 36]):
            assert sim.vregs.get_elem(4, i, 32) == e

    def test_vand_vv(self):
        sim = Simulator(mem_size=1 << 20)
        instrs = [
            vsetvli(0, 10, 2, 0),
            vle32(2, 11),
            vle32(3, 12),
            vand_vv(4, 2, 3),
            ebreak(),
        ]
        sim.load_program(pack_words(*instrs))
        sim.core.set_reg(10, 4)
        sim.core.set_reg(11, DATA_BASE)
        sim.core.set_reg(12, DATA_BASE + 0x100)
        for i, v in enumerate([0xFF, 0x0F, 0xF0, 0xAA]):
            sim.mem.write_word(DATA_BASE + i * 4, v)
        for i, v in enumerate([0x0F, 0xFF, 0x0F, 0x55]):
            sim.mem.write_word(DATA_BASE + 0x100 + i * 4, v)
        sim.run()
        for i, e in enumerate([0x0F, 0x0F, 0x00, 0x00]):
            assert sim.vregs.get_elem(4, i, 32) == e

    def test_vmv_vx(self):
        sim = Simulator(mem_size=1 << 20)
        instrs = [
            vsetvli(0, 10, 2, 0),
            vmv_vx(5, 7),   # v5[i] = x7 for all i
            ebreak(),
        ]
        sim.load_program(pack_words(*instrs))
        sim.core.set_reg(10, 4)
        sim.core.set_reg(7, 42)
        sim.run()
        for i in range(4):
            assert sim.vregs.get_elem(5, i, 32) == 42

    def test_vmul_vv(self):
        sim = Simulator(mem_size=1 << 20)
        instrs = [
            vsetvli(0, 10, 2, 0),
            vle32(2, 11),
            vle32(3, 12),
            vmul_vv(4, 2, 3),
            ebreak(),
        ]
        sim.load_program(pack_words(*instrs))
        sim.core.set_reg(10, 4)
        sim.core.set_reg(11, DATA_BASE)
        sim.core.set_reg(12, DATA_BASE + 0x100)
        for i, v in enumerate([2, 3, 4, 5]):
            sim.mem.write_word(DATA_BASE + i * 4, v)
        for i, v in enumerate([10, 10, 10, 10]):
            sim.mem.write_word(DATA_BASE + 0x100 + i * 4, v)
        sim.run()
        for i, e in enumerate([20, 30, 40, 50]):
            assert sim.vregs.get_elem(4, i, 32) == e


# ===========================================================================
# Reduction
# ===========================================================================

class TestReduction:
    def test_vredsum(self):
        sim = Simulator(mem_size=1 << 20)
        instrs = [
            vsetvli(0, 10, 2, 0),
            vle32(2, 11),
            vredsum_vs(4, 2, 3),   # v4[0] = sum(v2) + v3[0]
            vmv_xs(6, 4),          # x6 = v4[0]
            ebreak(),
        ]
        sim.load_program(pack_words(*instrs))
        sim.core.set_reg(10, 4)
        sim.core.set_reg(11, DATA_BASE)
        for i, v in enumerate([1, 2, 3, 4]):
            sim.mem.write_word(DATA_BASE + i * 4, v)
        # v3[0] = 0 (initial accumulator)
        sim.run()
        assert sim.core.reg(6) == 10   # 0 + 1+2+3+4


# ===========================================================================
# VMV.X.S  (scalar extraction)
# ===========================================================================

class TestVmvXS:
    def test_vmv_xs(self):
        sim = Simulator(mem_size=1 << 20)
        instrs = [
            vsetvli(0, 10, 2, 0),
            vle32(2, 11),
            vmv_xs(5, 2),    # x5 = v2[0]
            ebreak(),
        ]
        sim.load_program(pack_words(*instrs))
        sim.core.set_reg(10, 4)
        sim.core.set_reg(11, DATA_BASE)
        for i, v in enumerate([99, 1, 2, 3]):
            sim.mem.write_word(DATA_BASE + i * 4, v)
        sim.run()
        assert sim.core.reg(5) == 99


# ===========================================================================
# Integration: vector dot product
# ===========================================================================

def test_dot_product():
    """
    Compute dot_product = sum(a[i] * b[i]) for i in 0..3 using vector ops.
    This exercises: vsetvli, vle32, vmul, vredsum, vmv.x.s.
    """
    n = 4
    a = [1, 2, 3, 4]
    b = [5, 6, 7, 8]
    expected = sum(x * y for x, y in zip(a, b))  # 5+12+21+32 = 70

    sim = Simulator(mem_size=1 << 20)
    src_a = DATA_BASE
    src_b = DATA_BASE + 0x100

    instrs = [
        vsetvli(0, 10, 2, 0),   # vl=4, sew=32
        vle32(1, 11),            # v1 = a[]
        vle32(2, 12),            # v2 = b[]
        vmul_vv(3, 1, 2),        # v3 = a * b  (element-wise)
        vredsum_vs(4, 3, 5),     # v4[0] = 0 + sum(v3)
        vmv_xs(6, 4),            # x6 = v4[0]
        ebreak(),
    ]
    sim.load_program(pack_words(*instrs))
    sim.core.set_reg(10, n)
    sim.core.set_reg(11, src_a)
    sim.core.set_reg(12, src_b)
    for i, v in enumerate(a):
        sim.mem.write_word(src_a + i * 4, v)
    for i, v in enumerate(b):
        sim.mem.write_word(src_b + i * 4, v)

    sim.run()
    assert sim.core.reg(6) == expected
