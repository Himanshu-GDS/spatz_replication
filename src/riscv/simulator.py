"""
Spatz-replication simulator.

Combines an :class:`~.rv32i.RV32ICore` scalar host with an
:class:`~.rvv.RVVUnit` vector coprocessor, mirroring the Spatz/Snitch
architecture.
"""

from .rv32i import RV32ICore
from .rvv import RVVUnit


class Simulator:
    """
    Integrated Spatz-style simulator.

    Connects a scalar RV32I core (Snitch-like) with an RVV vector unit
    (Spatz-like) so that vector instructions are offloaded transparently.

    Parameters
    ----------
    mem_size:
        Size of shared memory in bytes (default 4 MiB).
    pc_init:
        Initial program counter.
    vlen:
        Vector register length in bits (default 256).
    """

    def __init__(
        self,
        mem_size: int = 1 << 22,
        pc_init: int = 0,
        vlen: int = 256,
    ) -> None:
        self.core = RV32ICore(mem_size=mem_size, pc_init=pc_init)
        self.vpu  = RVVUnit(vlen=vlen)
        # Wire the vector unit as the scalar core's trap handler
        self.core._trap_handler = self.vpu

    # ------------------------------------------------------------------
    # Convenience passthrough
    # ------------------------------------------------------------------

    @property
    def mem(self):
        return self.core.mem

    @property
    def regs(self):
        return self.core.regs

    @property
    def pc(self) -> int:
        return self.core.pc

    @property
    def halted(self) -> bool:
        return self.core.halted

    @property
    def vcsr(self):
        return self.vpu.vcsr

    @property
    def vregs(self):
        return self.vpu.vregs

    def load_program(self, code: bytes, base: int = 0) -> None:
        """Load *code* bytes at *base* and reset PC."""
        self.core.load_program(code, base)

    def step(self) -> None:
        """Execute one instruction."""
        self.core.step()

    def run(self, max_steps: int = 100_000) -> int:
        """Run until halted or *max_steps* executed; return steps taken."""
        return self.core.run(max_steps)

    def dump(self) -> str:
        """Return a human-readable state dump."""
        return (
            "=== Scalar registers ===\n"
            + self.core.dump_regs()
            + "\n=== Vector CSRs ===\n"
            + repr(self.vpu.vcsr)
        )
