"""
RISC-V V-extension (RVV) functional unit – Spatz replication.

This module implements the RISC-V Vector extension (RVV 1.0) integer subset
that is at the heart of the Spatz vector coprocessor.

Implemented instructions
------------------------
  Configuration : VSETVLI, VSETIVLI, VSETVL
  Load/Store    : VLE8.V, VLE16.V, VLE32.V, VSE8.V, VSE16.V, VSE32.V
                  VLSE8.V, VLSE16.V, VLSE32.V  (strided loads)
                  VSSE8.V, VSSE16.V, VSSE32.V  (strided stores)
  Arithmetic    : VADD.VV, VADD.VX, VADD.VI
                  VSUB.VV, VSUB.VX
                  VAND.VV, VAND.VX, VAND.VI
                  VOR.VV,  VOR.VX,  VOR.VI
                  VXOR.VV, VXOR.VX, VXOR.VI
                  VSLL.VV, VSLL.VX, VSLL.VI
                  VSRL.VV, VSRL.VX, VSRL.VI
                  VSRA.VV, VSRA.VX, VSRA.VI
                  VMUL.VV, VMUL.VX
                  VMIN.VV, VMIN.VX  (signed)
                  VMAX.VV, VMAX.VX  (signed)
                  VMINU.VV, VMINU.VX
                  VMAXU.VV, VMAXU.VX
  Reduction     : VREDSUM.VS, VREDAND.VS, VREDOR.VS, VREDXOR.VS
                  VREDMINU.VS, VREDMAXU.VS, VREDMIN.VS, VREDMAX.VS
  Mask          : VMSEQ.VV, VMSEQ.VX, VMSEQ.VI
                  VMSNE.VV, VMSNE.VX, VMSNE.VI
                  VMSLTU.VV, VMSLTU.VX
                  VMSLT.VV, VMSLT.VX
                  VMSLEU.VV, VMSLEU.VX, VMSLEU.VI
                  VMSLE.VV, VMSLE.VX, VMSLE.VI
                  VMSGTU.VX, VMSGTU.VI
                  VMSGT.VX, VMSGT.VI
  Misc          : VMV.V.V, VMV.V.X, VMV.V.I, VMV.X.S, VMV.S.X
                  VMERGE.VVM, VMERGE.VXM, VMERGE.VIM
                  VSLIDEUP.VX, VSLIDEDOWN.VX
"""

import struct
from typing import List

from .rv32i import Memory, _sign_extend, _u32, _s32


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

VLEN_BITS = 256           # Vector register length in bits (configurable)
VLEN_BYTES = VLEN_BITS // 8
NUM_VREGS = 32            # Number of vector registers (v0 … v31)


# ---------------------------------------------------------------------------
# Helper: element-width mask & sign
# ---------------------------------------------------------------------------

def _ew_mask(sew: int) -> int:
    return (1 << sew) - 1


def _ew_sign(val: int, sew: int) -> int:
    return _sign_extend(val, sew)


# ---------------------------------------------------------------------------
# CSRs
# ---------------------------------------------------------------------------

class VCSRs:
    """Vector CSRs (vtype, vl, vstart, vxrm, vxsat)."""

    def __init__(self) -> None:
        self.vtype: int = 0      # vtype CSR (includes vill, vma, vta, vsew, vlmul)
        self.vl: int = 0         # current vector length
        self.vstart: int = 0     # element start index
        self.vxrm: int = 0       # rounding mode
        self.vxsat: int = 0      # saturation flag

    # Decoded fields from vtype
    @property
    def sew(self) -> int:
        """Selected element width in *bits*."""
        vsew = (self.vtype >> 3) & 0x7
        return 8 << vsew  # 8, 16, 32, 64 …

    @property
    def lmul_frac(self) -> float:
        """LMUL as a fraction."""
        vlmul = self.vtype & 0x7
        lmul_map = {0: 1, 1: 2, 2: 4, 3: 8, 5: 0.125, 6: 0.25, 7: 0.5}
        return lmul_map.get(vlmul, 1)

    @property
    def vill(self) -> bool:
        return bool(self.vtype >> 31)

    def vlmax(self, vlen_bits: int) -> int:
        """Maximum number of elements for current vtype."""
        return int(vlen_bits * self.lmul_frac / self.sew)

    def __repr__(self) -> str:
        return (
            f"VCSRs(vl={self.vl}, sew={self.sew}, lmul={self.lmul_frac}, "
            f"vill={self.vill})"
        )


# ---------------------------------------------------------------------------
# Vector register file
# ---------------------------------------------------------------------------

class VRegFile:
    """
    Holds *NUM_VREGS* vector registers, each *VLEN_BYTES* bytes wide.
    Elements are stored as Python ``bytearray`` and interpreted at
    the width defined by the current SEW.
    """

    def __init__(self, vlen_bytes: int = VLEN_BYTES) -> None:
        self._vlen = vlen_bytes
        self._regs: list[bytearray] = [bytearray(vlen_bytes) for _ in range(NUM_VREGS)]

    def get_elem(self, vreg: int, idx: int, sew: int) -> int:
        """Return element *idx* of register *vreg* as an *unsigned* integer."""
        bw = sew // 8
        offset = idx * bw
        raw = self._regs[vreg][offset:offset + bw]
        if len(raw) < bw:
            return 0
        return int.from_bytes(raw, "little")

    def set_elem(self, vreg: int, idx: int, sew: int, value: int) -> None:
        """Write *value* (truncated to *sew* bits) into element *idx*."""
        bw = sew // 8
        offset = idx * bw
        masked = value & _ew_mask(sew)
        self._regs[vreg][offset:offset + bw] = masked.to_bytes(bw, "little")

    def get_mask_bit(self, idx: int) -> int:
        """Return bit *idx* of v0 (the mask register)."""
        byte = self._regs[0][idx // 8]
        return (byte >> (idx % 8)) & 1

    def set_mask_bit(self, idx: int, val: int) -> None:
        """Set bit *idx* of v0."""
        byte_idx = idx // 8
        bit = idx % 8
        if val:
            self._regs[0][byte_idx] |= 1 << bit
        else:
            self._regs[0][byte_idx] &= ~(1 << bit)

    def get_reg_bytes(self, vreg: int) -> bytearray:
        return bytearray(self._regs[vreg])

    def set_reg_bytes(self, vreg: int, data: bytes) -> None:
        n = min(len(data), self._vlen)
        self._regs[vreg][:n] = data[:n]


# ---------------------------------------------------------------------------
# RVV Unit
# ---------------------------------------------------------------------------

class RVVUnit:
    """
    RISC-V V-extension functional unit.

    Attach to an :class:`~.rv32i.RV32ICore` via
    ``core._trap_handler = rvv_unit``.  When the core encounters an
    unrecognised opcode it calls ``rvv_unit(raw, core)`` which returns
    ``True`` if the instruction was a vector instruction (handled here)
    or ``False`` otherwise.

    Parameters
    ----------
    vlen:
        Vector register length in *bits* (default 256).
    """

    # RVV opcodes
    _OPC_LOAD_FP   = 0x07
    _OPC_STORE_FP  = 0x27
    _OPC_OP_V      = 0x57

    def __init__(self, vlen: int = VLEN_BITS) -> None:
        self.vlen = vlen
        self.vlen_bytes = vlen // 8
        self.vcsr = VCSRs()
        self.vregs = VRegFile(self.vlen_bytes)

    # ------------------------------------------------------------------
    # Trap-handler interface (called by RV32ICore)
    # ------------------------------------------------------------------

    def __call__(self, raw: int, core) -> bool:
        opcode = raw & 0x7F
        if opcode == self._OPC_LOAD_FP:
            return self._load(raw, core)
        if opcode == self._OPC_STORE_FP:
            return self._store(raw, core)
        if opcode == self._OPC_OP_V:
            return self._op_v(raw, core)
        return False

    # ------------------------------------------------------------------
    # vsetvl* instructions (embedded in OPC_OP_V / funct3=0x7)
    # ------------------------------------------------------------------

    def _vsetvl(self, raw: int, core) -> bool:
        rd     = (raw >> 7)  & 0x1F
        funct3 = (raw >> 12) & 0x07
        rs1    = (raw >> 15) & 0x1F
        rs2    = (raw >> 20) & 0x1F
        bit31  = (raw >> 31) & 1
        bit30  = (raw >> 30) & 1

        if funct3 != 0x7:
            return False

        if bit31 == 0:
            # VSETVLI  rd, rs1, imm11
            zimm = (raw >> 20) & 0x7FF
            vtype_new = zimm
            avl = core.reg(rs1) if rs1 != 0 else (core.reg(rd) if rd != 0 else self.vcsr.vl)
        elif bit31 == 1 and bit30 == 1:
            # VSETIVLI  rd, uimm5, imm10
            uimm5 = rs1  # upper field holds uimm
            zimm  = (raw >> 20) & 0x3FF
            vtype_new = zimm
            avl = uimm5
        else:
            # VSETVL  rd, rs1, rs2
            vtype_new = core.reg(rs2)
            avl = core.reg(rs1) if rs1 != 0 else (core.reg(rd) if rd != 0 else self.vcsr.vl)

        self.vcsr.vtype = vtype_new
        vlmax = self.vcsr.vlmax(self.vlen)
        vl = min(avl, vlmax)
        self.vcsr.vl = vl
        core.set_reg(rd, vl)
        core.pc += 4
        return True

    # ------------------------------------------------------------------
    # Vector loads
    # ------------------------------------------------------------------

    def _load(self, raw: int, core) -> bool:
        rd     = (raw >> 7)  & 0x1F    # vd
        width  = (raw >> 12) & 0x07    # effective element width selector
        rs1    = (raw >> 15) & 0x1F    # base address
        rs2    = (raw >> 20) & 0x1F    # stride / nf / ...
        mew    = (raw >> 28) & 0x1
        mop    = (raw >> 26) & 0x3
        vm     = (raw >> 25) & 0x1

        # Only handle standard-width unit-stride and strided
        sew_map = {0: 8, 5: 16, 6: 32}
        if width not in sew_map:
            return False
        sew = sew_map[width]
        bw  = sew // 8

        base_addr = core.reg(rs1)
        vl = self.vcsr.vl

        if mop == 0b00:          # unit stride
            stride = bw
        elif mop == 0b10:        # strided
            stride = _s32(core.reg(rs2))
        else:
            return False  # indexed / others not implemented

        for i in range(vl):
            if vm == 0 and not self.vregs.get_mask_bit(i):
                continue  # masked off
            addr = _u32(base_addr + i * stride)
            if bw == 1:
                val = core.mem.read_byte(addr)
            elif bw == 2:
                val = core.mem.read_half(addr)
            else:
                val = core.mem.read_word(addr)
            self.vregs.set_elem(rd, i, sew, val)

        core.pc += 4
        return True

    # ------------------------------------------------------------------
    # Vector stores
    # ------------------------------------------------------------------

    def _store(self, raw: int, core) -> bool:
        vs3    = (raw >> 7)  & 0x1F
        width  = (raw >> 12) & 0x07
        rs1    = (raw >> 15) & 0x1F
        rs2    = (raw >> 20) & 0x1F
        mop    = (raw >> 26) & 0x3
        vm     = (raw >> 25) & 0x1

        sew_map = {0: 8, 5: 16, 6: 32}
        if width not in sew_map:
            return False
        sew = sew_map[width]
        bw  = sew // 8

        base_addr = core.reg(rs1)
        vl = self.vcsr.vl

        if mop == 0b00:
            stride = bw
        elif mop == 0b10:
            stride = _s32(core.reg(rs2))
        else:
            return False

        for i in range(vl):
            if vm == 0 and not self.vregs.get_mask_bit(i):
                continue
            addr = _u32(base_addr + i * stride)
            val  = self.vregs.get_elem(vs3, i, sew)
            if bw == 1:
                core.mem.write_byte(addr, val)
            elif bw == 2:
                core.mem.write_half(addr, val)
            else:
                core.mem.write_word(addr, val)

        core.pc += 4
        return True

    # ------------------------------------------------------------------
    # Vector arithmetic (OPC_OP_V)
    # ------------------------------------------------------------------

    def _op_v(self, raw: int, core) -> bool:  # noqa: C901
        funct3 = (raw >> 12) & 0x07
        rd     = (raw >> 7)  & 0x1F
        rs1    = (raw >> 15) & 0x1F
        rs2    = (raw >> 20) & 0x1F
        vm     = (raw >> 25) & 0x1
        funct6 = (raw >> 26) & 0x3F

        # ---- vset* lives here too ----------------------------------------
        if funct3 == 0x7:
            return self._vsetvl(raw, core)

        sew = self.vcsr.sew
        vl  = self.vcsr.vl
        mask = _ew_mask(sew)

        # Encoding helpers
        def vs2_elem(i): return self.vregs.get_elem(rs2, i, sew)
        def vs1_elem(i): return self.vregs.get_elem(rs1, i, sew)
        def xs1():       return core.reg(rs1) & mask
        # sign-extend helpers
        def se_vs2(i):   return _ew_sign(vs2_elem(i), sew)
        def se_vs1(i):   return _ew_sign(vs1_elem(i), sew)
        def se_xs1():    return _s32(core.reg(rs1))

        def write(i, val):
            if vm == 0 and not self.vregs.get_mask_bit(i):
                return
            self.vregs.set_elem(rd, i, sew, val)

        # ---- OPIVV / OPIVX / OPIVI / OPMVV / OPMVX ----------------------

        # funct3 encoding:
        #   000 = OPIVV, 001 = OPFVV, 010 = OPMVV
        #   011 = OPIVI, 100 = OPIVX, 101 = OPFVX, 110 = OPMVX

        if funct3 in (0x0, 0x3, 0x4):   # OPIVV=0, OPIVI=3, OPIVX=4
            if funct6 == 0x00:           # VADD
                for i in range(vl):
                    b = vs1_elem(i) if funct3 == 0x0 else (rs1 if funct3 == 0x3 else xs1())
                    write(i, vs2_elem(i) + b)
            elif funct6 == 0x02:         # VSUB (no VI form)
                for i in range(vl):
                    b = vs1_elem(i) if funct3 == 0x0 else xs1()
                    write(i, vs2_elem(i) - b)
            elif funct6 == 0x09:         # VAND
                for i in range(vl):
                    b = vs1_elem(i) if funct3 == 0x0 else (rs1 if funct3 == 0x3 else xs1())
                    write(i, vs2_elem(i) & b)
            elif funct6 == 0x0A:         # VOR
                for i in range(vl):
                    b = vs1_elem(i) if funct3 == 0x0 else (rs1 if funct3 == 0x3 else xs1())
                    write(i, vs2_elem(i) | b)
            elif funct6 == 0x0B:         # VXOR
                for i in range(vl):
                    b = vs1_elem(i) if funct3 == 0x0 else (rs1 if funct3 == 0x3 else xs1())
                    write(i, vs2_elem(i) ^ b)
            elif funct6 == 0x25:         # VSLL
                for i in range(vl):
                    shamt = (vs1_elem(i) if funct3 == 0x0 else (rs1 if funct3 == 0x3 else xs1())) & (sew - 1)
                    write(i, vs2_elem(i) << shamt)
            elif funct6 == 0x28:         # VSRL
                for i in range(vl):
                    shamt = (vs1_elem(i) if funct3 == 0x0 else (rs1 if funct3 == 0x3 else xs1())) & (sew - 1)
                    write(i, vs2_elem(i) >> shamt)
            elif funct6 == 0x29:         # VSRA
                for i in range(vl):
                    shamt = (vs1_elem(i) if funct3 == 0x0 else (rs1 if funct3 == 0x3 else xs1())) & (sew - 1)
                    write(i, _ew_sign(vs2_elem(i), sew) >> shamt)
            elif funct6 == 0x17:         # VMV.V.V / VMV.V.X / VMV.V.I  (vm must be 1)
                for i in range(vl):
                    b = vs1_elem(i) if funct3 == 0x0 else (rs1 if funct3 == 0x3 else xs1())
                    write(i, b)
            # ---- comparisons (write to mask register) --------------------
            elif funct6 == 0x18:         # VMSEQ
                for i in range(vl):
                    b = vs1_elem(i) if funct3 == 0x0 else (rs1 if funct3 == 0x3 else xs1())
                    self.vregs.set_mask_bit(i, 1 if vs2_elem(i) == b else 0)
            elif funct6 == 0x19:         # VMSNE
                for i in range(vl):
                    b = vs1_elem(i) if funct3 == 0x0 else (rs1 if funct3 == 0x3 else xs1())
                    self.vregs.set_mask_bit(i, 1 if vs2_elem(i) != b else 0)
            elif funct6 == 0x1A:         # VMSLTU (unsigned)
                for i in range(vl):
                    b = vs1_elem(i) if funct3 == 0x0 else xs1()
                    self.vregs.set_mask_bit(i, 1 if vs2_elem(i) < b else 0)
            elif funct6 == 0x1B:         # VMSLT (signed)
                for i in range(vl):
                    b = se_vs1(i) if funct3 == 0x0 else se_xs1()
                    self.vregs.set_mask_bit(i, 1 if se_vs2(i) < b else 0)
            elif funct6 == 0x1C:         # VMSLEU (unsigned)
                for i in range(vl):
                    b = vs1_elem(i) if funct3 == 0x0 else (rs1 if funct3 == 0x3 else xs1())
                    self.vregs.set_mask_bit(i, 1 if vs2_elem(i) <= b else 0)
            elif funct6 == 0x1D:         # VMSLE (signed)
                for i in range(vl):
                    b = se_vs1(i) if funct3 == 0x0 else (rs1 if funct3 == 0x3 else se_xs1())
                    self.vregs.set_mask_bit(i, 1 if se_vs2(i) <= b else 0)
            elif funct6 == 0x1E:         # VMSGTU
                for i in range(vl):
                    b = rs1 if funct3 == 0x3 else xs1()
                    self.vregs.set_mask_bit(i, 1 if vs2_elem(i) > b else 0)
            elif funct6 == 0x1F:         # VMSGT (signed)
                for i in range(vl):
                    b = rs1 if funct3 == 0x3 else se_xs1()
                    self.vregs.set_mask_bit(i, 1 if se_vs2(i) > b else 0)
            # ---- VMERGE --------------------------------------------------
            elif funct6 == 0x16:         # VMERGE / VMV (when vm=1 it is VMV)
                for i in range(vl):
                    if vm == 1:
                        b = vs1_elem(i) if funct3 == 0x0 else (rs1 if funct3 == 0x3 else xs1())
                        self.vregs.set_elem(rd, i, sew, b)
                    else:
                        if self.vregs.get_mask_bit(i):
                            b = vs1_elem(i) if funct3 == 0x0 else (rs1 if funct3 == 0x3 else xs1())
                        else:
                            b = vs2_elem(i)
                        self.vregs.set_elem(rd, i, sew, b)
            else:
                return False

            core.pc += 4
            return True

        if funct3 in (0x2, 0x6):   # OPMVV=2, OPMVX=6
            if funct6 == 0x25:     # VMUL
                for i in range(vl):
                    b = vs1_elem(i) if funct3 == 0x2 else xs1()
                    write(i, vs2_elem(i) * b)
            elif funct6 == 0x01:   # VREDSUM
                acc = self.vregs.get_elem(rs1, 0, sew)
                for i in range(vl):
                    acc = (acc + vs2_elem(i)) & mask
                self.vregs.set_elem(rd, 0, sew, acc)
            elif funct6 == 0x03:   # VREDAND
                acc = self.vregs.get_elem(rs1, 0, sew)
                for i in range(vl):
                    acc = acc & vs2_elem(i)
                self.vregs.set_elem(rd, 0, sew, acc)
            elif funct6 == 0x05:   # VREDOR
                acc = self.vregs.get_elem(rs1, 0, sew)
                for i in range(vl):
                    acc = acc | vs2_elem(i)
                self.vregs.set_elem(rd, 0, sew, acc)
            elif funct6 == 0x07:   # VREDXOR
                acc = self.vregs.get_elem(rs1, 0, sew)
                for i in range(vl):
                    acc = acc ^ vs2_elem(i)
                self.vregs.set_elem(rd, 0, sew, acc)
            elif funct6 == 0x09:   # VREDMINU
                acc = self.vregs.get_elem(rs1, 0, sew)
                for i in range(vl):
                    acc = min(acc, vs2_elem(i))
                self.vregs.set_elem(rd, 0, sew, acc)
            elif funct6 == 0x0B:   # VREDMAXU
                acc = self.vregs.get_elem(rs1, 0, sew)
                for i in range(vl):
                    acc = max(acc, vs2_elem(i))
                self.vregs.set_elem(rd, 0, sew, acc)
            elif funct6 == 0x08:   # VREDMIN (signed)
                acc = _ew_sign(self.vregs.get_elem(rs1, 0, sew), sew)
                for i in range(vl):
                    acc = min(acc, se_vs2(i))
                self.vregs.set_elem(rd, 0, sew, acc)
            elif funct6 == 0x0A:   # VREDMAX (signed)
                acc = _ew_sign(self.vregs.get_elem(rs1, 0, sew), sew)
                for i in range(vl):
                    acc = max(acc, se_vs2(i))
                self.vregs.set_elem(rd, 0, sew, acc)
            elif funct6 == 0x04:   # VMINU
                for i in range(vl):
                    b = vs1_elem(i) if funct3 == 0x2 else xs1()
                    write(i, min(vs2_elem(i), b))
            elif funct6 == 0x06:   # VMAXU
                for i in range(vl):
                    b = vs1_elem(i) if funct3 == 0x2 else xs1()
                    write(i, max(vs2_elem(i), b))
            elif funct6 == 0x0C:   # VMIN (signed)
                for i in range(vl):
                    b = se_vs1(i) if funct3 == 0x2 else se_xs1()
                    write(i, min(se_vs2(i), b))
            elif funct6 == 0x0D:   # VMAX (signed)
                for i in range(vl):
                    b = se_vs1(i) if funct3 == 0x2 else se_xs1()
                    write(i, max(se_vs2(i), b))
            # ---- VMV.X.S / VMV.S.X ---------------------------------------
            elif funct6 == 0x10:
                if funct3 == 0x2:  # VMV.X.S  rd(scalar) = vs2[0]
                    core.set_reg(rd, self.vregs.get_elem(rs2, 0, sew))
                elif funct3 == 0x6:  # VMV.S.X  vd[0] = rs1(scalar)
                    self.vregs.set_elem(rd, 0, sew, core.reg(rs1))
                else:
                    return False
            # ---- VSLIDEUP / VSLIDEDOWN -----------------------------------
            elif funct6 == 0x0E:   # VSLIDEUP.VX / VSLIDEUP.VI
                offset = xs1() if funct3 == 0x6 else rs1
                for i in range(vl):
                    src = i - offset
                    if src < 0:
                        pass  # element unchanged (not written)
                    else:
                        write(i, vs2_elem(src))
            elif funct6 == 0x0F:   # VSLIDEDOWN.VX / VSLIDEDOWN.VI
                offset = xs1() if funct3 == 0x6 else rs1
                for i in range(vl):
                    src = i + offset
                    val = vs2_elem(src) if src < vl else 0
                    write(i, val)
            else:
                return False

            core.pc += 4
            return True

        return False
