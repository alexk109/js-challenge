"""MVP firmware: the protocol programs in `fw/*/program.pio` and their host drivers.

One package per protocol, each with `program.pio` (assembled with `asm/pioasm.py`), a
`driver.py` that writes the registers of `docs/mvp-regmap.md` through
`iss.pio_iss.ChipDevice`, and a pytest that runs them against the pin-level models of
`verif/vip/`.  `make fw PROTO=<proto> [TARGET=iss|rtl]` runs one of them.
"""