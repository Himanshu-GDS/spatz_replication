# spatz_replication

A Python functional simulator that replicates the key elements of the
[Spatz](https://github.com/pulp-platform/spatz) RISC-V vector coprocessor
developed at ETH Zurich.

## Overview

The simulator models the Spatz architecture:

```
 ┌──────────────────────────────────────────┐
 │  Simulator (Spatz system)                │
 │                                          │
 │  ┌─────────────────┐  vector offload     │
 │  │  RV32ICore      │ ─────────────────►  │
 │  │  (Snitch-like   │                     │
 │  │   scalar host)  │ ◄─────────────────  │
 │  └─────────────────┘  result/done        │
 │                                          │
 │  ┌─────────────────┐                     │
 │  │  RVVUnit        │                     │
 │  │  (Spatz-like    │                     │
 │  │   vector unit)  │                     │
 │  └─────────────────┘                     │
 └──────────────────────────────────────────┘
```

- **RV32ICore** – functional model of the RISC-V 32-bit base integer ISA (RV32I):
  R, I, S, B, U, J instruction types; ECALL (exit/write).
- **RVVUnit** – RISC-V V extension (RVV 1.0) integer subset:
  configuration (`vsetvli`/`vsetivli`/`vsetvl`), unit-stride and strided
  loads/stores, arithmetic (`vadd`, `vsub`, `vand`, `vor`, `vxor`, `vsll`,
  `vsrl`, `vsra`, `vmul`, `vmin`, `vmax`, …), reductions (`vredsum`, …),
  mask operations, and element moves.
- **Simulator** – integrates both units and exposes a clean API.

## Repository layout

```
spatz_replication/
├── src/
│   └── riscv/
│       ├── __init__.py     – package exports
│       ├── rv32i.py        – RV32I core + memory model
│       ├── rvv.py          – RVV vector unit (Spatz-like)
│       └── simulator.py    – integrated Simulator class
├── tests/
│   ├── test_rv32i.py       – RV32I unit tests
│   └── test_rvv.py         – RVV + integration tests
├── Makefile
└── README.md
```

## Quick start

```bash
# Install test dependencies (pytest only)
pip install pytest

# Run the full test suite
make test
# or directly:
python -m pytest tests/ -v
```

## Example: vector addition

```python
import struct
from src.riscv.simulator import Simulator

sim = Simulator(mem_size=1 << 20, vlen=256)

DATA = 0x80000
a = [1, 2, 3, 4]
b = [10, 20, 30, 40]

for i, v in enumerate(a):
    sim.mem.write_word(DATA + i * 4, v)
for i, v in enumerate(b):
    sim.mem.write_word(DATA + 0x100 + i * 4, v)

# Hand-assembled program:
#   vsetvli x0, x10, e32, m1
#   vle32.v v1, (x11)   ; load a
#   vle32.v v2, (x12)   ; load b
#   vadd.vv v3, v1, v2  ; v3 = a + b
#   vse32.v v3, (x13)   ; store result
#   ebreak
program = struct.pack(
    "<6I",
    0x02057057,   # vsetvli x0, x10, e32, m1
    0x0205e087,   # vle32.v v1, (x11)
    0x0206e107,   # vle32.v v2, (x12)
    0x02318157,   # vadd.vv v3, v1, v2
    0x0206ea27,   # vse32.v v3, (x13)
    0x00100073,   # ebreak
)

sim.core.set_reg(10, 4)             # avl = 4
sim.core.set_reg(11, DATA)          # base of a
sim.core.set_reg(12, DATA + 0x100)  # base of b
sim.core.set_reg(13, DATA + 0x200)  # result base
sim.load_program(program)
sim.run()

result = [sim.mem.read_word(DATA + 0x200 + i * 4) for i in range(4)]
print(result)  # [11, 22, 33, 44]
```

## Supported RVV instructions

| Category      | Instructions                                                        |
|---------------|---------------------------------------------------------------------|
| Configuration | `vsetvli`, `vsetivli`, `vsetvl`                                     |
| Load          | `vle8.v`, `vle16.v`, `vle32.v`, `vlse8.v`, `vlse16.v`, `vlse32.v` |
| Store         | `vse8.v`, `vse16.v`, `vse32.v`, `vsse8.v`, `vsse16.v`, `vsse32.v` |
| Arithmetic    | `vadd`, `vsub`, `vand`, `vor`, `vxor`, `vsll`, `vsrl`, `vsra`, `vmul`, `vmin`, `vmax`, `vminu`, `vmaxu` |
| Reduction     | `vredsum`, `vredand`, `vredor`, `vredxor`, `vredmin`, `vredmax`, `vredminu`, `vredmaxu` |
| Compare/Mask  | `vmseq`, `vmsne`, `vmsltu`, `vmslt`, `vmsleu`, `vmsle`, `vmsgtu`, `vmsgt` |
| Move          | `vmv.v.v`, `vmv.v.x`, `vmv.v.i`, `vmv.x.s`, `vmv.s.x`             |
| Merge         | `vmerge.vvm`, `vmerge.vxm`, `vmerge.vim`                           |
| Permute       | `vslideup.vx`, `vslidedown.vx`                                     |

## Reference

- [RISC-V Unprivileged ISA Specification](https://github.com/riscv/riscv-isa-manual)
- [RISC-V V Extension Specification](https://github.com/riscv/riscv-v-spec)
- [Spatz – ETH Zurich](https://github.com/pulp-platform/spatz)
