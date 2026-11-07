"""Host side of the PIO SPI *target*: loads the program, arms it, and streams MISO in.

Everything here is written against `docs/mvp-regmap.md` and goes through
`iss.pio_iss.ChipDevice`'s access queue, which performs at most one register access per core
cycle and in order.  A read returns a `concurrent.futures.Future` resolved in the cycle the
read was performed, so the driver is a small state machine: at most one access in flight, and
the test loop calls `SpiTargetDriver.tick(bus)` once per core cycle.

## What the host has to do, and why so much of it is the host's job

The MVP PIO cannot decode an opcode or index an array (the ISA argument is in
`fw/spi_target/program.pio`), so the *response bytes* are the host's: the driver pushes one
32-bit TX FIFO word per MISO byte, left-justified, and the engine shifts out the word's MSB
first (`OUT_SHIFTDIR = 0`, `docs/mvp-isa.md` difference 11 -- the host port moves whole
words, so host software left-justifies the data).  AUTOPULL is off and the program refills
with an explicit `pull block` once per byte, so

    N MISO bytes  ->  N + 1 TX FIFO words

the extra one being the refill that follows the last byte of the transaction (the engine
refills before it notices CSn high; see `program.pio`).  `start()` adds it, so a caller
passes the bytes the device should shift out and nothing else.

## The register writes, and their order

`docs/mvp-regmap.md` `### Typical host sequences` gives the start sequence and the order is
load-bearing in three places:

* **levels and directions before the engine runs.**  The pins come out of reset with
  `uio_oe = 0`, so MISO is an input until the forced `set pindirs, 1`; without it the OUT
  window would only write a level register nobody reads.
* **the forced instructions come before the TX FIFO is filled.**  `set pindirs`, `pull block`
  and `jmp idle` are host-forced through `SMe_INSTR`: the host cannot write the PC any other
  way (`SMe_ADDR` is read-only) and `force` bypasses `SM_ENABLE` and the divider
  (`docs/mvp-isa.md` `### EXEC`).  The `pull block` must therefore be queued *after* the
  first stream word, since the two are one FIFO-ordered queue.
* **`CTRL` last, with `SM_RESTART`, `CLKDIV_RESTART` and `SM_ENABLE` together**, so the first
  SM cycle is `h + 2` with the PC already at `idle`.

Between transactions only the tail is needed: the FIFOs survive `SM_RESTART`
(`docs/mvp-isa.md` `### Enable, restart`, MVP: FIFOs are emptied only by reset).
"""
from __future__ import annotations

from fw import regmap

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

from asm.pioasm import Program, assemble

# ---- word addresses, docs/mvp-regmap.md `### Address map` ---------------------------------
A_CTRL = regmap.A_CTRL
A_FSTAT = regmap.A_FSTAT
A_FDEBUG = regmap.A_FDEBUG
A_FLEVEL = regmap.A_FLEVEL
A_TXF0, A_TXF1 = regmap.txf_addrs()[:2]  # engine e: A_TXF0 + e
A_RXF0, A_RXF1 = regmap.rxf_addrs()[:2]  # engine e: A_RXF0 + e

#: `SMe_*` is at word `0x32 + 6e + i` (docs/mvp-regmap.md `### Per-engine registers`).
SM_BASE = regmap.sm_base()
SM_CLKDIV, SM_EXECCTRL, SM_SHIFTCTRL, SM_ADDR, SM_INSTR, SM_PINCTRL = range(6)

#: The per-engine bits of CTRL, shifted into place for engine `e`: SM_ENABLE 1:0,
#: SM_RESTART 5:4, CLKDIV_RESTART 9:8.
CTRL_SM_ENABLE = 0x1
CTRL_SM_RESTART = 0x10
CTRL_CLKDIV_RESTART = 0x100

#: Engine `e`'s IMEM word `k` is at word address 0x12 + 0x40 e + k.
IMEM_BASE = regmap.imem_bases()[0]
IMEM_STRIDE = regmap.imem_bases()[1] - IMEM_BASE if regmap.n_engines() > 1 else 0x40
NUM_ENGINES = regmap.n_engines()

# ---- pin map, must match fw/spi_target/program.pio ----------------------------------------
CS, MOSI, SCK, MISO = 0, 1, 2, 3
#: `IN_BASE = MOSI`, so `in pins, 1` samples MOSI and `wait ... pin 1` is SCK.  The program
#: watches SCK and CSn with the absolute GPIO source instead, which is why IN_BASE only has
#: to make MOSI data bit 0 (`program.pio`, "Why IN_BASE = 1").
IN_BASE = MOSI
#: The OUT and SET windows are one pin each, both on MISO: the device drives exactly one pin.
OUT_BASE = MISO
OUT_COUNT = 1
SET_BASE = MISO
SET_COUNT = 1
#: `EXECCTRL.JMP_PIN` = CSn, for the loop's `jmp pin, idle`.
JMP_PIN = CS

#: FLEVEL nibbles of engine 0: TX0 3:0, RX0 7:4, TX1 11:8, RX1 15:12.  FDEBUG is spaced the
#: same way, so one engine's flags are one byte of it.
FLEVEL_TX0, FLEVEL_RX0 = 0x0, 0x4
FLEVEL_STRIDE = 8
FDEBUG_STRIDE = 8


def __getattr__(name):  # FIFO_DEPTH is read at use time: the active map is fixed at target install
    if name == "FIFO_DEPTH":
        return regmap.fifo_depth()
    raise AttributeError(name)


#: Longest MISO byte stream one transaction may queue.  Nothing in the hardware sets it: the
#: FIFO is four words deep and the driver streams one word per byte the engine consumes, so a
#: read of any length works, and this is only where the caller is asked whether it meant it.
MAX_STREAM_BYTES = 255

#: `SET PINDIRS` is the only SET destination that writes directions (docs/mvp-isa.md
#: `### Field values`, `SET Destination` = 100).
PINDIR = 0b100
#: `set pindirs, 1`: SET opcode `111`, destination PINDIRS `100`, immediate 1.  No side-set
#: bit rides along, because this program has SIDESET_COUNT = 0, so the whole of bits 12:8 is
#: delay and the word is opcode|destination|immediate with nothing else in it.
SET_PINDIRS_MISO = 0xE000 | (PINDIR << 5) | 1
#: `jmp idle`, and `pull block` -- both from docs/mvp-isa.md `### Delay/side-set field`
#: ("Worked encodings"): `pull block` is 0x80A0, and a jump is opcode `000` with address in
#: bits 4:0.
JMP_WORD = 0x0000
PULL_BLOCK = 0x80A0

PIO_FILE = Path(__file__).with_name("program.pio")
PROGRAM_NAME = "spi_target"


def programs() -> dict[str, Program]:
    """The assembled programs of `program.pio`, keyed by program name."""
    out = {}
    for prog in assemble(PIO_FILE.read_text(encoding="utf-8")):
        out[prog.name] = prog
    if PROGRAM_NAME not in out:
        raise RuntimeError(f"{PIO_FILE} has no `{PROGRAM_NAME}` program")
    return out


def program_with_pins(pins: tuple[int, int, int, int]) -> Program:
    """`program.pio` re-assembled with its four pin numbers replaced.

    The two clock lines are watched with `wait <pol> gpio <n>`, and that index is *absolute*
    (docs/mvp-isa.md `### WAIT Source`), so it is baked into the instruction word: moving CSn or
    SCK is a change to the program text, not to a register.  That is the price of the pin-map
    freedom the absolute source buys, and it is why this function exists -- the driver writes
    the matching windows (IN_BASE = MOSI, OUT_BASE = SET_BASE = MISO, JMP_PIN = CSn) from
    `SpiTargetConfig.pins`, and the two have to agree.

    For the shipped map this returns the shipped program, word for word, which
    `test_re_assembling_the_default_pin_map_changes_nothing` checks.
    """
    cs, mosi, sck, miso = pins
    text = PIO_FILE.read_text(encoding="utf-8")
    for name, value in (("CS_PIN", cs), ("MOSI_PIN", mosi), ("SCK_PIN", sck),
                        ("MISO_PIN", miso)):
        text = re.sub(rf"^\.define PUBLIC {name} \d+$", f".define PUBLIC {name} {value}",
                      text, flags=re.MULTILINE)
    program = next(p for p in assemble(text) if p.name == PROGRAM_NAME)
    if len(program.instrs) > 32:
        raise ValueError(f"the pin map {pins} needs {len(program.instrs)} words, IMEM holds 32")
    return program


def tx_word(miso_byte: int) -> int:
    """One MISO byte as the left-justified TX FIFO word the engine shifts out.

    With `OUT_SHIFTDIR = 0` the MSB leaves the OSR first, so the byte has to sit in bits
    31:24 of the word.  The three bytes below it are never clocked: the program's
    `pull block` refills the whole OSR every eight bits (`program.pio`, "The byte boundary"),
    which is what makes one byte per FIFO word.
    """
    if not 0 <= miso_byte <= 0xFF:
        raise ValueError(f"a MISO byte is 0..255, got {miso_byte}")
    return miso_byte << 24


def jedec_stream(jedec_id: Sequence[int] = (0xC0, 0x28, 0x17)) -> list[int]:
    """The MISO byte stream of a 0x9F transaction that returns `jedec_id`.

    A JEDEC ID read clocks four bytes out: the byte clocked while the opcode itself goes out,
    which on real silicon is the flash's idle output, and then the three ID bytes
    (`verif/vip/spi.py` `SpiFlashTarget._command`: `_out_base = 8` for 0x9F, so output bits
    0..7 are before the ID).  That first byte is 0.
    """
    ids = list(jedec_id)
    if not 1 <= len(ids) <= 3:
        raise ValueError(f"a JEDEC ID is 1..3 bytes, got {len(ids)}")
    return [0x00] + ids


def read_stream(data: Sequence[int]) -> list[int]:
    """The MISO byte stream of a 0x03 READ that returns `data`.

    A READ clocks out four bytes of nothing before the data: the opcode and the three
    address bytes are on the wire while the device is still silent (`_out_base = 32` for an
    addressed command), and that is four zero bytes here.
    """
    return [0x00] * 4 + [b & 0xFF for b in data]

def _thresh_field(value: int) -> int:
    """A 5-bit threshold field: 1..31 as written, 32 as 0 (docs/mvp-isa.md `thresh(f)`)."""
    if not 1 <= value <= 32:
        raise ValueError(f"a shift threshold is 1..32 bits, got {value}")
    return 0 if value == 32 else value


@dataclass
class SpiTargetConfig:
    """The engine configuration one transaction runs with.

    `clkdiv` is `(INT, FRAC)` with `FRAC` in 1/256 (`docs/mvp-isa.md` `### Clock divider`),
    so `(1, 0)` is one SM cycle per core cycle, the fastest the chip can go.  The defaults are
    the ones `program.pio` is written for: `pull_thresh = 8` because `jmp !osre` *is* the
    byte counter, `push_thresh = 8` so one received byte is one RX FIFO word, both shift
    directions left (MSB first, in and out), autopush on and autopull off.

    `pins` is `(CSn, MOSI, SCK, MISO)` as protocol pin indices, and it is a parameter rather
    than four constants because the windows the program needs are derived from it: IN_BASE is
    MOSI (so `in pins, 1` samples it), OUT_BASE and SET_BASE are MISO (one pin each), and
    EXECCTRL.JMP_PIN is CSn (so the loop's `jmp pin, idle` parks on the master's release).

    The only constraint is that all four are distinct `uio` pins 0..7, and that is not an
    accident of this program: `uio` is the only kind of pin with a direction register, so MISO
    has to be one for the host's forced `set pindirs` to reach a pad
    (docs/mvp-isa.md `### Open drain`), and a device must not drive the master's three lines.
    The two clock lines are watched with the *absolute* GPIO wait source, so -- unlike the
    relative `pin` source -- they are not tied to IN_BASE and may sit anywhere
    (`program.pio`, "Why IN_BASE = 1").
    """

    pins: tuple[int, int, int, int] = (CS, MOSI, SCK, MISO)
    clkdiv: tuple[int, int] = (1, 0)
    push_thresh: int = 8
    pull_thresh: int = 8
    in_shiftdir: int = 0
    out_shiftdir: int = 0
    autopull: int = 0
    autopush: int = 1

    def __post_init__(self) -> None:
        # INT 0 means 65536 on this chip, not 1 (docs/mvp-isa.md `### Clock divider`), so a
        # zero divisor is a mistake rather than a request for full speed.
        integer, frac = self.clkdiv
        if not 1 <= integer <= 0xFFFF or not 0 <= frac <= 0xFF:
            raise ValueError(
                f"CLKDIV {self.clkdiv} out of range (INT 1..65535, FRAC 0..255)"
            )
        if len(self.pins) != 4 or len(set(self.pins)) != 4 or not all(
            0 <= p < 8 for p in self.pins
        ):
            raise ValueError(
                f"pins must be four distinct uio pins 0..7 (docs/mvp-isa.md "
                f"`### The 18 protocol pins`), got {self.pins}"
            )
        for name, value in (("push_thresh", self.push_thresh), ("pull_thresh", self.pull_thresh)):
            _thresh_field(value)                  # 1..32, checked here and used in shiftctrl()
        if self.autopull:
            raise ValueError(
                "this program refills the OSR with `pull block` at each byte boundary, so "
                "AUTOPULL must be off (with it on, `pull` is a no-op when osr_cnt == 0 and "
                "the host's pre-load would silently not refill)"
            )

    def clkdiv_word(self) -> int:
        integer, frac = self.clkdiv
        return (integer << 16) | (frac << 8)

    def shiftctrl(self) -> int:
        """SHIFTCTRL, field positions from `docs/mvp-regmap.md` `### Per-engine registers`:
        29:25 PULL_THRESH, 24:20 PUSH_THRESH, 19 OUT_SHIFTDIR, 18 IN_SHIFTDIR, 17 AUTOPULL,
        16 AUTOPUSH."""
        return (
            (_thresh_field(self.pull_thresh) << 25)
            | (_thresh_field(self.push_thresh) << 20)
            | (self.out_shiftdir << 19)
            | (self.in_shiftdir << 18)
            | (self.autopull << 17)
            | (self.autopush << 16)
        )

    @property
    def cs(self) -> int:
        """CSn's protocol pin index, which is what EXECCTRL.JMP_PIN has to be."""
        return self.pins[0]

    @property
    def mosi(self) -> int:
        """MOSI's pin index, which is what IN_BASE has to be."""
        return self.pins[1]

    @property
    def miso(self) -> int:
        """MISO's pin index: the one pin the OUT and SET windows cover."""
        return self.pins[3]

    def execctrl(self, program: Program) -> int:
        """JMP_PIN = CSn, no side-set enable, and the wrap window the `.pio` asks for."""
        if program.sideset_opt or program.sideset_pindirs or program.sideset_bits:
            raise ValueError("this driver only programs a program with no side-set")
        return (
            (self.cs << 24)
            | ((program.wrap & 0x1F) << 12)
            | ((program.wrap_target & 0x1F) << 7)
        )

    def pinctrl(self, program: Program) -> int:
        """IN_BASE = MOSI, one OUT pin (MISO), one SET pin (MISO), no side-set."""
        return (
            (SET_COUNT << 26)
            | (OUT_COUNT << 20)
            | (self.mosi << 15)
            | (SET_COUNT and 0)                  # (SIDESET_BASE stays 0: SIDESET_COUNT = 0)
            | (self.miso << 5)
            | self.miso
        )


class SpiTargetDriver:
    """One `ChipDevice` running the SPI target program, plus the host state machine.

    `load()` puts the program and its configuration in the chip; `start(stream)` arms it for
    one transaction; `tick()` advances one core cycle and issues at most one register access;
    `run()` does the ticking.  The driver keeps one access in flight, because that is all the
    register bus offers (`docs/mvp-interfaces.md` `## Register bus`) and because a read's
    result arrives one or two cycles after it was queued.  It reads `FLEVEL` whenever it has
    nothing better to do, which is plenty: the engine moves one FIFO word per byte, so the
    FIFOs never run dry and no `FDEBUG` flag is ever raised -- which is the property the
    tests assert, because a stall there is not a slow transaction but a corrupted one
    (`program.pio`, "The two waits are level waits").
    """

    def __init__(
        self,
        device,
        config: Optional[SpiTargetConfig] = None,
        program: Optional[Program] = None,
        sm: int = 0,
    ) -> None:
        if not 0 <= sm < NUM_ENGINES:
            raise ValueError(f"engine index must be 0..{NUM_ENGINES - 1}, got {sm}")
        self.dev = device
        self.sm = sm
        self.config = config or SpiTargetConfig()
        #: `program` defaults to the one `program.pio` assembles.  It is an argument so a
        #: test can run a variant (a different loop body, say) through the same driver
        #: without touching the shipped `.pio` -- that is how `measure.py` shows the loop's
        #: instruction count is not the binding constraint at any rate.
        self.program = program if program is not None else programs()[PROGRAM_NAME]
        self.imem_base = IMEM_BASE + IMEM_STRIDE * sm
        self.ctrl_enable = CTRL_SM_ENABLE << sm
        self.ctrl_restart = (CTRL_SM_RESTART | CTRL_CLKDIV_RESTART) << sm
        self._loaded = False
        self._tx: list[int] = []
        self._rx: list[int] = []
        self._tx_level = 0
        self._rx_level = 0
        self._inflight: Optional[tuple[str, object]] = None
        self._pc: Optional[int] = None
        self._fdebug = 0
        self._ticks = 0

    # ---- register word addresses of this engine ------------------------------------------
    def _sm(self, index: int) -> int:
        """`SMe_<index>`: word 0x32 + 6 e + index."""
        return SM_BASE + 6 * self.sm + index

    @property
    def txf(self) -> int:
        return A_TXF0 + self.sm

    @property
    def rxf(self) -> int:
        return A_RXF0 + self.sm

    # ---- setup -----------------------------------------------------------------------------

    def load(self) -> None:
        """Queue everything the chip needs before a transaction: the program and its config.

        The sequence is `docs/mvp-regmap.md` `### Typical host sequences` steps 1 to 3, with
        the engine disabled while its IMEM is written.
        """
        prog = self.program
        if len(prog.instrs) > 32:
            raise ValueError(f"{prog.name} is {len(prog.instrs)} words, IMEM holds 32")
        self.dev.write(A_CTRL, 0)                                   # step 1: SM_ENABLE = 0
        for k, word in enumerate(prog.instrs):                      # step 2: the program
            self.dev.write(self.imem_base + k, word)
        self.dev.write(self._sm(SM_CLKDIV), self.config.clkdiv_word())
        self.dev.write(self._sm(SM_EXECCTRL), self.config.execctrl(prog))
        self.dev.write(self._sm(SM_SHIFTCTRL), self.config.shiftctrl())
        self.dev.write(self._sm(SM_PINCTRL), self.config.pinctrl(prog))
        self._loaded = True

    def start(self, miso: Sequence[int], n_rx: Optional[int] = None) -> None:
        """Arm the program for one transaction that shifts `miso` out on MISO.

        `n_rx` is how many bytes the master will clock in, which the driver only uses to
        report progress; the engine receives whatever arrives.  The stream is queued as one
        TX FIFO word per byte (`tx_word`) plus one filler word, because the program's
        boundary `pull block` runs once more after the last byte (see the module docstring).
        """
        if not self._loaded:
            self.load()
        stream = [b & 0xFF for b in miso]
        if not 1 <= len(stream) <= MAX_STREAM_BYTES:
            raise ValueError(
                f"a transaction shifts out 1..{MAX_STREAM_BYTES} MISO bytes, "
                f"{len(stream)} were asked for"
            )
        self._tx = [tx_word(b) for b in stream] + [0x0000_0000]
        self._rx = []
        self._tx_level = 0
        self._rx_level = 0
        self._inflight = None
        self._pc = None
        self._n_rx = len(stream) if n_rx is None else int(n_rx)
        # Step 1 of `### Typical host sequences`, and it is load-bearing between transactions:
        # a finished engine sits in `wait 0 gpio CS` with SM_ENABLE still 1, and every SM
        # cycle of a stalled WAIT is an SM cycle (docs/mvp-isa.md `### When an engine
        # executes`), so background_autopull runs -- harmlessly with AUTOPULL off, but the
        # engine must be stopped before the host refills the FIFO anyway.
        self.dev.write(A_CTRL, 0)
        # Step 4: the pin directions.  MISO is a `uio` pin, so it needs PINDIRS before the
        # OUT window's level register reaches a pad (docs/mvp-isa.md `### Open drain`: "PINDIRS
        # = 1 means drive").  CSn, MOSI and SCK stay inputs: this device never drives them.
        self.dev.write(self._sm(SM_INSTR), SET_PINDIRS_MISO)
        # The first stream word, so the pre-load below has something to pop.  The queue is
        # FIFO-ordered (`iss.pio_iss.ChipDevice`), so this write retires before the forced
        # instruction queued after it.
        self.dev.write(self.txf, self._tx[0])
        self._tx.pop(0)
        self._tx_level = 1
        self.dev.write(A_CTRL, self.ctrl_enable | self.ctrl_restart)      # step 6
        # `pull block`: pre-load the OSR so the program's first `out pins, 1` does not fall
        # through the byte boundary and shift in the wrong word.  AUTOPULL is off, so with
        # IfEmpty clear this always pops (docs/mvp-isa.md `### PULL`) -- and it is a forced
        # instruction, so it runs whatever SM_ENABLE and the divider say.
        #
        # It has to come *after* the CTRL write above, not before: SM_RESTART clears osr_cnt
        # to 32 (docs/mvp-isa.md `### Engine state`) while keeping the OSR itself, so a
        # pre-load before the restart leaves a loaded OSR that the program still believes is
        # empty.  It then falls through `jmp !osre` on the first pass, refills from the FIFO
        # one word too late, and the transaction comes back shifted by eight bits.  That was
        # measured, not guessed.
        self.dev.write(self._sm(SM_INSTR), PULL_BLOCK)
        # `jmp idle`: the only way to set the PC (SMe_ADDR is read-only).
        self.dev.write(
            self._sm(SM_INSTR), JMP_WORD | (self.program.labels["idle"] & 0x1F)
        )
        # Whatever else the FIFO has room for, before the first byte boundary reaches it.
        while self._tx and self._tx_level < regmap.fifo_full_level():
            self.dev.write(self.txf, self._tx.pop(0))
            self._tx_level += 1

    # ---- one core cycle ---------------------------------------------------------------------

    def tick(self, bus) -> bool:
        """Advance one core cycle; True when the driver has nothing left to do.

        Accesses are queued before the step and retire on it, because the driver keeps
        exactly one in flight and `ChipDevice` performs exactly one queued access per tick.
        """
        if self._inflight is None:
            self._issue()
        bus.step()
        self._ticks += 1
        if self._inflight is not None:
            kind, fut = self._inflight
            # A write has no future: it retires on the step it was queued for, because the
            # driver keeps exactly one access in flight and the device does one per tick.
            if fut is None or fut.done():
                self._inflight = None
                self._retire(kind, None if fut is None else fut.result())
        return self.idle_bus

    def _issue(self) -> None:
        """Queue the single access that helps most right now."""
        if self._tx and self._tx_level < regmap.fifo_full_level():
            self.dev.write(self.txf, self._tx[0])
            self._inflight = ("tx", None)
        elif self._rx_level > 0:
            self._inflight = ("rx", self.dev.read(self.rxf))
        else:
            self._inflight = ("flevel", self.dev.read(A_FLEVEL))

    def _retire(self, kind: str, data) -> None:
        if kind == "tx":
            self._tx.pop(0)
            self._tx_level += 1
        elif kind == "rx":
            self._rx.append(data)
            self._rx_level -= 1
        else:
            shift = FLEVEL_STRIDE * self.sm
            self._tx_level = (data >> (FLEVEL_TX0 + shift)) & 0xF
            self._rx_level = (data >> (FLEVEL_RX0 + shift)) & 0xF

    @property
    def idle_bus(self) -> bool:
        """True when the stream is out, the RX FIFO is drained and nothing is in flight."""
        return not self._tx and self._rx_level == 0 and self._inflight is None

    def run(self, bus, ticks: int) -> None:
        """Run `ticks` core cycles, feeding the TX FIFO and draining the RX FIFO.

        A transaction is not bounded by the driver: the master decides how many bytes it
        clocks, so the caller runs the bus for as many cycles as the master's plan needs plus
        a tail (`verif/vip/spi.py` `SpiController.pending_ticks`).  `stop()` is what ends the
        engine afterwards.
        """
        for _ in range(ticks):
            if not self.dev.idle and self._inflight is None:
                # The setup is a queue of register writes and the state machine above assumes
                # the device's queue holds nothing but the access it issued itself.
                bus.step()
                self._ticks += 1
                continue
            self.tick(bus)
        # Drain whatever is still queued so `dev.idle` holds and the RX FIFO is empty.
        for _ in range(64):
            if self.dev.idle and self._inflight is None and self._rx_level == 0:
                break
            self.tick(bus)

    def stop(self, bus) -> None:
        """Disable the engine and release MISO, the way a real target sits between reads.

        Two register writes, both forced instructions or plain ones:
        `set pindirs, 0` releases MISO (high-Z, so the pull-up wins between transactions, as
        `SpiFlashTarget` does with its `miso=None`), and `CTRL = 0` stops the engine so it
        does not sit in `wait 0 gpio CS` consuming SM cycles.
        """
        self.dev.write(A_CTRL, 0)
        self.dev.write(self._sm(SM_INSTR), SET_PINDIRS_MISO & ~1)
        for _ in range(16):
            bus.step()
            self._ticks += 1

    # ---- results ----------------------------------------------------------------------------

    @property
    def rx_words(self) -> list[int]:
        """The RX FIFO words, one received byte each in bits 7:0.

        With `IN_SHIFTDIR = 0` and `PUSH_THRESH = 8`, eight `in pins, 1` shift the byte into
        ISR bits 7:0 with the first bit received as the MSB, and the autopush hands that
        32-bit ISR to the FIFO as it stands -- so the byte is right-aligned, not at bits
        31:24 the way the host port's UART path puts it (docs/mvp-isa.md difference 11).
        """
        return list(self._rx)

    @property
    def rx_bytes(self) -> list[int]:
        """`rx_words` as bytes, in the order the engine pushed them.

        More bytes than the master clocked in are possible: when CSn goes high the engine is
        still inside the capture half of a bit, and its remaining `in pins, 1`s shift the
        master's fill bits in until `jmp pin, idle` parks it.  A caller wants
        `rx_bytes[:n_rx]`, which is what `received()` returns.
        """
        return [w & 0xFF for w in self._rx]

    def received(self) -> list[int]:
        """The `n_rx` bytes the master clocked in, as the chip received them."""
        return self.rx_bytes[: self._n_rx]

    @property
    def pc(self) -> Optional[int]:
        return self._pc

    def read_pc(self, bus) -> int:
        """Read `SMe_ADDR`, so a test can say where the engine is parked."""
        return self.read_reg(bus, self._sm(SM_ADDR)) & 0x1F

    def read_reg(self, bus, addr: int, max_ticks: int = 64) -> int:
        """One register read, run to completion.  For the test's register-level checks."""
        fut = self.dev.read(addr)
        for _ in range(max_ticks):
            bus.step()
            if fut.done():
                return fut.result()
        raise TimeoutError(f"no answer from word address {addr:#04x}")

    def read_fdebug(self, bus) -> int:
        """This engine's `FDEBUG` flags, which are W1C: read them, then clear them.

        Only this engine's byte is cleared, so a test that looks at both engines does not
        take the other's flags away.  Every flag here is a stall or an overrun that the
        transaction survived by luck, so the tests assert this is 0.
        """
        raw = self.read_reg(bus, A_FDEBUG)
        mine = (raw >> (FDEBUG_STRIDE * self.sm)) & 0xFF
        if raw:
            self.dev.write(A_FDEBUG, raw)
            for _ in range(8):
                bus.step()
        self._fdebug |= mine
        return mine
