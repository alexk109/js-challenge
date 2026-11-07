"""Host side of the PIO SPI master: loads the program, arms it, and feeds the FIFOs.

Everything here is written against `docs/mvp-regmap.md` and goes through
`iss.pio_iss.ChipDevice`'s write/read queue, which performs at most one bus access per core
cycle and in order.  A read returns a `concurrent.futures.Future` that is resolved in the
cycle the read was performed, so the driver is a small state machine: it keeps at most one
access in flight, and the test loop calls `SpiMaster.tick(bus)` once per core cycle.

## The transaction the program performs

The program clocks whole bytes: the host arms X with the number of bytes in the
transaction, the opcode and the 24-bit address included, and the program clocks exactly
that many and raises CS.  A byte out of the TX FIFO becomes 4 bytes on MOSI, so

    JEDEC ID   N = 4      TX word 0x9F000000        ->  4 RX words, ID in words 1..3
    read 16 B  N = 20     TX word 0x03AA0000 + 4x0 ->  20 RX words, data in 4..19

(`spi_read` in the docstring of `fw/spi/program.pio` says why the program counts bytes
instead of using the RP2040 `jmp !osre` exit: `!osre` ends the transaction the moment the
TX FIFO runs dry, so a host that is a tick late truncates the CS window.  Counting bytes
costs 2 SM cycles of the 66 a byte takes and cannot end early.)

## Why the register writes are in this order

`docs/mvp-regmap.md` `### Typical host sequences` gives the start sequence, and the order
matters in three places:

* levels before directions.  The pins come out of reset with `uio_oe = 0`, so the forced
  `set pindirs, 3` is what makes the chip drive CS and SCK.  Doing it before `set pins`
  would drive CS at its reset level 0, i.e. pulse CS low before the transaction.
* `set x, N` and `jmp entry` are forced instructions (`SMe_INSTR`).  The host cannot write
  the PC or X any other way (`docs/mvp-isa.md` `### Enable, restart`), and a forced
  instruction runs in the cycle after the write whatever `SM_ENABLE` and the divider say.
* the final `CTRL` write sets `SM_RESTART`, `CLKDIV_RESTART` and `SM_ENABLE` together, so
  the first SM cycle is `h + 2` with the PC already at `entry`.

Between transactions only the tail of that sequence is needed: the FIFOs survive
`SM_RESTART` (`docs/mvp-isa.md` `### Enable, restart`, MVP: FIFOs are emptied only by
reset), so the driver only has to drain the RX FIFO, push the TX words, and re-force the
pins, the counter and the PC.
"""
from __future__ import annotations

from fw import regmap

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Sequence

from asm.pioasm import Program, assemble

# ---- word addresses, docs/mvp-regmap.md `### Address map` -------------------------------
A_CTRL = regmap.A_CTRL
A_FSTAT = regmap.A_FSTAT
A_FDEBUG = regmap.A_FDEBUG
A_FLEVEL = regmap.A_FLEVEL
A_TXF0, A_TXF1 = regmap.txf_addrs()[:2]  # engine e: A_TXF0 + e
A_RXF0, A_RXF1 = regmap.rxf_addrs()[:2]  # engine e: A_RXF0 + e

#: `SMe_*` is at word `0x32 + 6e + i` (docs/mvp-regmap.md `### Per-engine registers`).
SM_BASE = regmap.sm_base()
SM_CLKDIV, SM_EXECCTRL, SM_SHIFTCTRL, SM_ADDR, SM_INSTR, SM_PINCTRL = range(6)

#: The per-engine bits of CTRL, shifted into place for engine `e` (docs/mvp-regmap.md
#: `### Global registers`: SM_ENABLE 1:0, SM_RESTART 5:4, CLKDIV_RESTART 9:8).
CTRL_SM_ENABLE = 0x1
CTRL_SM_RESTART = 0x10
CTRL_CLKDIV_RESTART = 0x100
#: Engine `e`'s IMEM word `k` is at word address 0x12 + 0x40 e + k, and `SMe_*` sits at
#: word 0x32 + 6 e + i (docs/mvp-regmap.md `### Address map`).
IMEM_BASE = regmap.imem_bases()[0]
IMEM_STRIDE = regmap.imem_bases()[1] - IMEM_BASE if regmap.n_engines() > 1 else 0x40
NUM_ENGINES = regmap.n_engines()

# ---- pin map, must match fw/spi/program.pio ----------------------------------------------
CS, SCK, MOSI, MISO = 0, 1, 2, 3
#: `set pindirs, 7` drives CS, SCK and MOSI.  CS idle-high is driven rather than just
#: released, so the line does not depend on the board's pull-up, and MOSI has to be driven
#: or the OUT window only writes a level register nobody reads (the pin stays an input and
#: the board pull-up wins).  MISO, pin 3, is left as an input: the program never writes it.
PINDIRS_IDLE = 0b111
#: `SET PINDIRS` is the only SET destination that writes directions (docs/mvp-isa.md
#: `### Field values`, `SET Destination` = 100), and the 5-bit immediate is the pin mask.
PINDIR = 0b100
#: The CS level bit of the SET window (`SET_BASE = 0`, `SET_COUNT = 2`).
CS_IDLE = 0b01

#: FLEVEL nibbles of engine 0, docs/mvp-regmap.md `### Global registers`: TX0 3:0,
#: RX0 7:4, TX1 11:8, RX1 15:12 -- engine `e`'s pair is 8 bits further up.  FDEBUG is
#: spaced the same way (TXSTALL 25:24, TXOVER 17:16, RXUNDER 9:8, RXSTALL 1:0), so one
#: engine's flags are one byte of FDEBUG.
FLEVEL_TX0, FLEVEL_RX0 = 0x0, 0x4
FLEVEL_STRIDE = 8
FDEBUG_STRIDE = 8


def __getattr__(name):  # FIFO_DEPTH is read at use time: the active map is fixed at target install
    if name == "FIFO_DEPTH":
        return regmap.fifo_depth()
    raise AttributeError(name)


PIO_FILE = Path(__file__).with_name("program.pio")
#: `spi_read_m0` and `spi_read_m3`, keyed by SPI mode: the two the chip shipped with, CPHA 0.
PROGRAMS = {0: "spi_read_m0", 3: "spi_read_m3"}
#: The CPHA 1 programs, keyed by the same modes: 0 is datasheet mode 1 and 3 is mode 2.  They are
#: the CPHA 0 programs with the two side-set values of the bit loop swapped (`program.pio`'s
#: comment works out why), and `SpiConfig` reaches them with `cpha=1` rather than by widening
#: `mode`, which stays the chip's CPOL choice of 0 or 3.
CPHA1_PROGRAMS = {0: "spi_read_m1", 3: "spi_read_m2"}


#: A transaction is 1..31 bytes: the byte counter is armed with a forced `set x, N`, and N is a
#: 5-bit immediate (`docs/mvp-isa.md` `## Encoding`).  A longer transaction is several of those
#: back to back inside one CS window -- see `SpiMaster.start_long`.
MAX_BYTES_PER_CHUNK = 31
#: The default chunk of `start_long`: 7 TX words = 28 bytes, the most that still fits the counter
#: (31) with a whole number of TX words.  Fewer words per chunk means more re-arms and more gaps
#: on the wire; more is not possible.
DEFAULT_CHUNK_WORDS = 7
#: Side-set field of the `done` instruction, with `CS = 1`: what a released CS looks like.  The
#: program raises CS by putting this in the field of the `jmp done` it parks on.
CS_HIGH_SIDE = 1


def _thresh_field(value: int) -> int:
    """A 5-bit threshold field: 1..31 as written, 32 as 0 (docs/mvp-isa.md `thresh(f)`)."""
    if not 1 <= value <= 32:
        raise ValueError(f"a shift threshold is 1..32 bits, got {value}")
    return 0 if value == 32 else value


def programs(cpha: int = 0) -> dict[int, Program]:
    """The assembled programs of `program.pio`, keyed by SPI mode.

    `cpha=0` (the default) is the pair the chip shipped with, `{0: spi_read_m0, 3: spi_read_m3}`;
    `cpha=1` is the CPHA 1 pair, datasheet modes 1 and 2.  A `Program` for a mode that the .pio does
    not hold is a `RuntimeError` naming it, not a silent omission.
    """
    wanted = PROGRAMS if cpha == 0 else CPHA1_PROGRAMS
    if cpha not in (0, 1):
        raise ValueError(f"cpha must be 0 or 1, got {cpha!r}")
    out = {}
    for prog in assemble(PIO_FILE.read_text(encoding="utf-8")):
        for mode, name in wanted.items():
            if prog.name == name:
                out[mode] = prog
    missing = set(wanted.values()) - {p.name for p in out.values()}
    if missing:
        raise RuntimeError(f"{PIO_FILE} is missing {sorted(missing)}")
    return out


@dataclass
class SpiConfig:
    """The engine configuration one transaction runs with.

    `clkdiv` is `(INT, FRAC)` with `FRAC` in 1/256 (docs/mvp-isa.md `### Clock divider`),
    so `(1, 0)` is one SM cycle per core cycle, the fastest the chip can go.
    """

    mode: int = 0
    clkdiv: tuple[int, int] = (1, 0)
    push_thresh: int = 8          # autopush one byte per RX word
    pull_thresh: int = 32         # autopull one TX word per 32 bits
    in_shiftdir: int = 0          # left: the first bit received ends up as the MSB
    out_shiftdir: int = 0         # left: the MSB leaves first
    autopull: int = 1
    autopush: int = 1
    #: CPHA, 0 or 1.  `mode` is the chip's CPOL choice (0 or 3), which is what every other field
    #: and the pin side-set levels depend on; `cpha` picks which of the two programs in
    #: `program.pio` runs, so datasheet modes 1 and 2 are `mode=0, cpha=1` and `mode=3, cpha=1`.
    #: 0 is the shipped behaviour and the default.
    cpha: int = 0
    #: `hold_cs` puts the host in charge of the end of the transaction: the word at `done` is
    #: patched so the program parks with CS still low, and the host raises CS itself when it has
    #: finished.  That is what lets one CS window carry more than the 31 bytes the 5-bit byte
    #: counter can express (`SpiMaster.start_long`).  Off by default, so a plain `start()` behaves
    #: exactly as before.
    hold_cs: int = 0

    def __post_init__(self) -> None:
        if self.hold_cs not in (0, 1):
            raise ValueError(f"hold_cs must be 0 or 1, got {self.hold_cs!r}")
        if self.cpha not in (0, 1):
            raise ValueError(f"cpha must be 0 or 1, got {self.cpha!r}")
        if self.mode not in PROGRAMS:
            raise ValueError(f"SPI mode must be 0 or 3, got {self.mode}")
        # INT 0 means 65536 on this chip, not 1 (docs/mvp-isa.md `### Clock divider`), so a
        # zero divisor is a mistake rather than a request for full speed.
        integer, frac = self.clkdiv
        if not 1 <= integer <= 0xFFFF or not 0 <= frac <= 0xFF:
            raise ValueError(f"CLKDIV {self.clkdiv} out of range (INT 1..65535, FRAC 0..255)")

    def clkdiv_word(self) -> int:
        integer, frac = self.clkdiv
        return (integer << 16) | (frac << 8)

    def shiftctrl(self) -> int:
        # Field positions, docs/mvp-regmap.md `### Per-engine registers`: 29:25
        # PULL_THRESH, 24:20 PUSH_THRESH, 19 OUT_SHIFTDIR, 18 IN_SHIFTDIR, 17 AUTOPULL,
        # 16 AUTOPUSH.  Both threshold fields are 5 bits wide and 0 means 32, so a
        # threshold of 32 is written as 0 (`thresh(f)` in docs/mvp-isa.md `## Conventions`).
        return (
            (_thresh_field(self.pull_thresh) << 25)
            | (_thresh_field(self.push_thresh) << 20)
            | (self.out_shiftdir << 19)
            | (self.in_shiftdir << 18)
            | (self.autopull << 17)
            | (self.autopush << 16)
        )

    def execctrl(self, program: Program) -> int:
        """Side-set off (the program writes levels, not directions), wrap as the .pio says."""
        if program.sideset_opt or program.sideset_pindirs:
            raise ValueError("this driver only programs level side-set, not opt/pindirs")
        return ((program.wrap & 0x1F) << 12) | ((program.wrap_target & 0x1F) << 7)

    def pinctrl(self, program: Program) -> int:
        sideset_count = program.sideset_bits
        if sideset_count != 2:
            raise ValueError(
                f"the SPI program needs SIDESET_COUNT 2 (CS and SCK), the .pio says "
                f"{sideset_count}"
            )
        return (
            (sideset_count << 29)
            | (3 << 26)                     # SET_COUNT: pins 0, 1 and 2
            | (1 << 20)                     # OUT_COUNT: one pin (MOSI)
            | (MISO << 15)                  # IN_BASE
            | (CS << 10)                    # SIDESET_BASE
            | (CS << 5)                     # SET_BASE
            | MOSI                          # OUT_BASE
        )


class SpiMaster:
    """One `ChipDevice` running the SPI program, plus the host state machine around it.

    `load()` puts the program and its configuration in the chip; `start()` arms it and
    launches a transaction; `tick()` advances one core cycle and issues at most one bus
    access; `run()` does the ticking until the program has halted with CS high.

    The state machine keeps one access in flight, because that is all the register bus
    offers (`docs/mvp-interfaces.md` `## Register bus`: one access per core cycle) and
    because a read's result arrives one or two cycles after it was queued.  It reads
    `FLEVEL` whenever it has nothing better to do, which is plenty: the engine moves a word
    every 66 core cycles and the driver moves an access every 1, so the FIFOs never run
    dry and no `FDEBUG` flag is ever raised.
    """

    def __init__(
        self,
        device,
        config: Optional[SpiConfig] = None,
        program: Optional[Program] = None,
        sm: int = 0,
    ) -> None:
        if not 0 <= sm < NUM_ENGINES:
            raise ValueError(f"engine index must be 0..{NUM_ENGINES - 1}, got {sm}")
        self.dev = device
        self.sm = sm
        self.config = config or SpiConfig()
        #: `program` defaults to the one `program.pio` assembles for this mode.  It is an
        #: argument so that a test can run a variant (a different bit period, say) through
        #: the same driver without touching the shipped `.pio`.
        self.program = (program if program is not None
                        else programs(self.config.cpha)[self.config.mode])
        self.done_addr = self.program.labels["done"]
        #: The word actually loaded at `done`: the program's own, or the CS-held patch when
        #: `config.hold_cs` is set (`load()` writes it).
        self.done_word = self.program.instrs[self.done_addr]
        self.imem_base = IMEM_BASE + IMEM_STRIDE * sm
        self.ctrl_enable = CTRL_SM_ENABLE << sm
        self.ctrl_restart = (CTRL_SM_RESTART | CTRL_CLKDIV_RESTART) << sm
        self._loaded = False
        self._tx: list[int] = []
        self._rx: list[int] = []
        self._tx_level = 0
        self._rx_level = 0
        self._n_bytes = 0
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

    def _set_pindirs_sideset(self, release_cs: bool = True) -> int:
        """`set pindirs, 7 side <idle>`: drive CS, SCK and MOSI at their idle levels.

        Encoded by hand, the way pico-sdk's `pio_encode_set_pins_with_side_set` does it:
        SET PINDIRS is opcode `111`, destination `100`; with `SIDESET_COUNT = 2` the two
        side-set data bits sit in bits 11:10 of the word, and side-set bit `k` drives pin
        `SIDESET_BASE + k`, so bit 0 is CS and bit 1 is SCK.

        `release_cs = False` holds CS low instead, which is what a chunk boundary inside a long
        transaction needs (`_arm`).
        """
        return 0xE000 | (PINDIR << 5) | PINDIRS_IDLE | self._side_field(release_cs) << 11

    def _idle_side_field(self) -> int:
        """The side-set field that leaves the pins idle: CS high, SCK at the mode's CPOL.

        Every forced instruction side-sets too, including the ones that only set X or the PC,
        so they all need the idle state in their side-set field.  Without it a forced
        `set x, N` drives CS low for a cycle before the engine has even started, which both
        opens a CS window the flash sees and, in mode 3, moves SCK off its idle level early
        enough to add a launch edge before the first capture.
        """
        return self._side_field(True) << 11

    def _side_field(self, release_cs: bool = True) -> int:
        """The side-set field value (not shifted) for CS/SCK at CS high or CS low.

        Bit 0 is CS and bit 1 is SCK, and SCK's idle level is the mode's CPOL.  Releasing CS and
        holding it are the same two levels with bit 0 clear or set -- the state `enter` leaves
        behind is exactly the held one, which is why holding CS across a chunk boundary moves no
        pin at all.
        """
        sck_idle = 1 if self.config.mode >= 2 else 0
        return (CS_IDLE if release_cs else 0) | (sck_idle << 1)

    # ---- setup -----------------------------------------------------------------------------

    def load(self) -> None:
        """Queue everything the chip needs before a transaction: the program and its config.

        The sequence is `docs/mvp-regmap.md` `### Typical host sequences` steps 1 to 3,
        with the engine disabled while its IMEM is written.
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
        # `hold_cs`: rewrite the one word at `done` so the program parks with CS low instead of
        # raising it.  The side-set field is bits 12:11 of the word (`.side_set 2`, so two data
        # bits and no `opt`), and the shipped `done` carries CS high with SCK at the mode's idle
        # level; holding CS is that same side-set value with bit 0 (CS) clear -- which is exactly
        # the state `enter` leaves behind, so the change is invisible on the wire.
        if self.config.hold_cs:
            word = prog.instrs[self.done_addr]
            released = (word >> 11) & 0x3
            held = released & ~CS_HIGH_SIDE & 0x3
            patched = (word & ~(0x3 << 11)) | (held << 11)
            if held == released:
                raise ValueError(
                    f"{prog.name}: `done` does not raise CS (side-set {released}), so there is "
                    f"nothing to hold")
            self.dev.write(self.imem_base + self.done_addr, patched)
            self.done_word = patched
        else:
            self.done_word = prog.instrs[self.done_addr]
        self._loaded = True

    def _arm(self, tx_words: Sequence[int], n_bytes: int, release_cs: bool = True) -> None:
        """Queue the register writes that start one chunk of `n_bytes`, `tx_words` long.

        This is what `start()` has always done, split out so that `run_long()` can use it for every
        chunk after the first: the same forced instructions, the same prefill, the same CTRL write.

        `release_cs` is the one thing that differs, and it is the whole of the chunking: the forced
        instructions carry the side-set field of the *idle* state (CS high) when this is the first
        chunk, and the held state (CS low, SCK at the mode's idle level) for every later one.  CS
        low is what `enter` leaves behind and what the patched `done` parks on, so a later chunk's
        arming moves no pin at all: the wire sees a pause in SCK, not a gap in CS.
        """
        self._tx = [w & 0xFFFF_FFFF for w in tx_words]
        self._tx_level = 0
        self._inflight = None
        self._pc = None
        # How many bytes the *whole* transaction has clocked once this chunk is done.  For a
        # single-chunk transaction that is the transaction, so `finished` means what it always did;
        # for a chunked one it is the end of *this* chunk, which is what the host waits for before
        # it drains the RX FIFO and arms the next.
        self._sent_bytes = getattr(self, "_sent_bytes", 0)
        self._chunk_end = self._sent_bytes + n_bytes
        self._sent_bytes = self._chunk_end
        # Step 1 of docs/mvp-regmap.md `### Typical host sequences`, and it is load-bearing
        # between chunks: a finished engine sits in `jmp done` with SM_ENABLE still 1, and every
        # SM cycle of that self-loop runs `background_autopull()`
        # (docs/mvp-isa.md `### Reference step`).  With an empty OSR -- osr_cnt = 32, which
        # is what SM_RESTART leaves -- that loop eats one TX FIFO word per core cycle, so a
        # host that fills the FIFO without stopping the engine first loses words to TXOVER
        # and the engine then stalls on an empty FIFO.  The forced instructions below still
        # run while the engine is disabled (`force` bypasses SM_ENABLE), so they come
        # before the FIFO is filled, not after.
        self.dev.write(A_CTRL, 0)
        # Step 4: the pin levels and the pin directions, in one forced instruction.
        #
        # `.side_set 2` without `opt` means *every* instruction side-sets (docs/mvp-isa.md
        # `### Delay/side-set field, bits 12:8`), and within one engine the side-set channel
        # beats the OUT/SET channel (`### Pin writes`), so a plain forced `set pins` is
        # overwritten by its own side-set in the same cycle.  The idle levels therefore have
        # to travel *in* the forced instruction's side-set field, which is what the pico-sdk
        # does with `pio_encode_set_pins_with_side_set`.  The SET window covers pins 0 to 2
        # (SET_COUNT = 3), so `set pindirs, 7` drives CS, SCK and MOSI -- MOSI has to be
        # driven or the OUT window only writes a level register nobody reads -- and the
        # side-set field gives CS and SCK their levels: CS high, SCK at the mode's idle.
        self.dev.write(self._sm(SM_INSTR), self._set_pindirs_sideset(release_cs))
        # `set x, N`: 5-bit immediate, so a chunk is 1..31 bytes.  Side-set at the same level.
        self.dev.write(
            self._sm(SM_INSTR), 0xE020 | n_bytes | self._side_field(release_cs) << 11
        )
        # `jmp enter`: the only way to set the PC (SMe_ADDR is read-only).  Same side-set level.
        self.dev.write(
            self._sm(SM_INSTR),
            self.program.labels["enter"] | self._side_field(release_cs) << 11,
        )
        for word in self._tx[:regmap.fifo_depth()]:
            self.dev.write(self.txf, word)
            self._tx_level += 1
        self._tx = self._tx[regmap.fifo_depth():]
        self.dev.write(A_CTRL, self.ctrl_enable | self.ctrl_restart)      # step 6

    def start(self, tx_words: Sequence[int]) -> None:
        """Arm the program and run one transaction of `tx_words` words.

        Every TX word is four bytes on the wire, so the program is told to clock
        `4 * len(tx_words)` bytes: opcode, address and payload, and as many dummy bytes as
        the read needs.  The first `regmap.fifo_depth()` words go in before the engine is enabled, so
        its first `out pins, 1` never stalls on an empty TX FIFO.

        More than 31 bytes is refused here, and always has been: the byte counter goes in as a
        5-bit immediate through `SMe_INSTR`.  `start_long()` is how a longer transaction is done,
        and it needs `config.hold_cs`.
        """
        if not self._loaded:
            self.load()
        self._rx = []
        self._rx_level = 0
        self._n_bytes = 4 * len(tx_words)
        self._chunks = 1
        self._chunk_index = 0
        self._sent_bytes = 0
        self._released = True
        if not 1 <= self._n_bytes <= MAX_BYTES_PER_CHUNK:
            raise ValueError(
                f"a transaction is 1..{MAX_BYTES_PER_CHUNK} bytes (the byte counter is a 5-bit "
                f"immediate in SMe_INSTR), {self._n_bytes} bytes were asked for; use start_long() "
                f"for a longer one"
            )
        self._arm(tx_words, self._n_bytes)

    def start_long(self, tx_words: Sequence[int],
                   chunk_words: int = DEFAULT_CHUNK_WORDS) -> None:
        """Arm the first chunk of a transaction longer than 31 bytes, inside one CS window.

        The gap this fills: "the SPI firmware cannot do a transaction longer than 31 bytes, because
        the byte counter goes in as a 5-bit immediate through `SMe_INSTR`, and nothing splits the
        transaction."  This is the split, and `run_long()` runs it:

        *   `config.hold_cs` must be set, so the word at `done` parks with CS low (`load()`), and
            the host raises CS itself when the last byte is back.  The flash sees **one** CS
            window, not several: a chunk boundary is a gap in SCK, not a gap in CS.
        *   the transaction is cut into chunks of `chunk_words` TX words (28 bytes by default, the
            most that still fits the 5-bit counter), each armed exactly as `start()` arms one.
        *   every chunk is identical in every respect except its byte count, so the wire cannot
            tell where one ends and the next begins -- which is the point, and what
            `fw/spi/test_spi_long.py` checks against the flash and the monitor.

        32, 64 and 256 bytes are 8, 16 and 64 TX words, so at the default 7-word chunk those are
        2, 3 and 10 chunks (64 words is 7 x 9 + 1), and the last chunk of the 256-byte transaction
        is one word, 4 bytes: a transaction has to be a whole number of TX words.  The 10 is what
        the long-transaction test measures.
        """
        if not self.config.hold_cs:
            raise ValueError(
                "start_long() needs SpiConfig(hold_cs=1): without it the program raises CS at "
                "`done`, so each chunk would be its own transaction on the wire")
        if not 1 <= chunk_words <= 7:
            raise ValueError(
                f"a chunk is 1..7 TX words (28 bytes is the most that fits the 31-byte counter), "
                f"got {chunk_words}")
        if not tx_words:
            raise ValueError("a transaction needs at least one TX word")
        if not self._loaded:
            self.load()
        self._rx = []
        self._rx_level = 0
        self._n_bytes = 4 * len(tx_words)
        self._all_tx = [w & 0xFFFF_FFFF for w in tx_words]
        self._sent_bytes = 0
        self._chunk_words = chunk_words
        self._chunks = (len(tx_words) + chunk_words - 1) // chunk_words
        self._chunk_index = 0
        self._released = False
        first = self._all_tx[:chunk_words]
        self._arm(first, 4 * len(first))

    def next_chunk(self) -> bool:
        """Queue the next chunk's arming, if there is one. True if a chunk was queued.

        Called by `run_long()` between chunks.  The host state machine's own words come first --
        the RX FIFO has to be drained and the next TX words pushed before the engine is started --
        which is `tick()`'s job, so this only queues the arming once the caller has done that.
        """
        if self._chunk_index + 1 >= self._chunks:
            return False
        self._chunk_index += 1
        start = self._chunk_index * self._chunk_words
        words = self._all_tx[start:start + self._chunk_words]
        self._arm(words, 4 * len(words), release_cs=False)
        return True

    def release_cs(self) -> None:
        """Queue the forced instruction that raises CS, ending the transaction.

        One forced `set pindirs, 7` whose side-set field is CS high and SCK at the mode's idle
        level -- the idle state a released CS leaves behind.  Nothing else changes: the engine is
        already parked on `done`, and the forced instruction runs in the next core cycle whatever
        `SM_ENABLE` says (`docs/mvp-isa.md` `### EXEC`).
        """
        if self._released:
            return
        self._released = True
        self.dev.write(self._sm(SM_INSTR), 0xE000 | (PINDIR << 5) | PINDIRS_IDLE |
                       self._idle_side_field())

    # ---- one core cycle ---------------------------------------------------------------------

    def tick(self, bus) -> bool:
        """Advance one core cycle; True when the transaction is over.

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
        return self.finished

    def _issue(self) -> None:
        """Queue the single access that helps most right now."""
        if self._tx and self._tx_level < regmap.fifo_full_level():
            self.dev.write(self.txf, self._tx[0])
            self._inflight = ("tx", None)
        elif self._rx_level > 0:
            self._inflight = ("rx", self.dev.read(self.rxf))
        elif len(self._rx) >= self._chunk_end:
            self._inflight = ("addr", self.dev.read(self._sm(SM_ADDR)))
        else:
            self._inflight = ("flevel", self.dev.read(A_FLEVEL))

    def _retire(self, kind: str, data) -> None:
        if kind == "tx":
            self._tx.pop(0)
            self._tx_level += 1
        elif kind == "rx":
            self._rx.append(data)
            self._rx_level -= 1
        elif kind == "flevel":
            shift = FLEVEL_STRIDE * self.sm
            self._tx_level = (data >> (FLEVEL_TX0 + shift)) & 0xF
            self._rx_level = (data >> (FLEVEL_RX0 + shift)) & 0xF
        else:
            self._pc = data

    # ---- running ----------------------------------------------------------------------------

    @property
    def finished(self) -> bool:
        """True once this chunk's bytes have come back and the program is parked on `done`.

        Both halves are needed: the words can all be in the RX FIFO while the engine is still
        clocking the last one, and `done` is only reached when X has run out, so the PC at the
        halt means the chunk is over.  For a single-chunk transaction that is the whole transaction
        and CS is high; for a chunked one (`config.hold_cs`) CS is still low and the host decides
        when to raise it.
        """
        return (
            len(self._rx) >= self._chunk_end
            and self._pc == self.done_addr
            and self._inflight is None
        )

    @property
    def rx_words(self) -> list[int]:
        """The RX FIFO words, one received byte each in bits 7:0 (see `rx_bytes`)."""
        return list(self._rx)

    @property
    def rx_bytes(self) -> list[int]:
        """The RX FIFO words as bytes, in the order the engine pushed them.

        With `IN_SHIFTDIR = 0` and `PUSH_THRESH = 8`, eight `in pins, 1` shift the byte
        into ISR bits 7:0 with the first bit received as the MSB, and the autopush hands
        that 32-bit ISR to the FIFO as it stands -- so the byte is right-aligned, not at
        bits 31:24 the way the host port's UART path puts it (docs/mvp-isa.md
        difference 11).  Shift direction is what decides it: `IN_SHIFTDIR = 1` would put
        the byte at bits 31:24 and also reverse the bit order inside it.
        """
        return [w & 0xFF for w in self._rx]

    @property
    def pc(self) -> Optional[int]:
        return self._pc

    def run(self, bus, max_ticks: int = 20000, tail: int = 8) -> bool:
        """Drain whatever `load`/`start` queued, then run until the transaction completes.

        The setup is a queue of register writes, and the state machine below assumes the
        device's queue holds nothing but the access it issued itself, so the setup is run
        out first.  `tail` extra core cycles are run after the program halts, so the CS
        rise is on the bus and a monitor has seen the whole window.  False if `max_ticks`
        ran out first.
        """
        for _ in range(max_ticks):
            if not self.dev.idle and self._inflight is None:
                bus.step()
                self._ticks += 1
                continue
            if self.tick(bus):
                bus.run(tail)
                return True
        return False

    def run_long(self, bus, max_ticks: int = 200_000, tail: int = 8,
                 chunk_words: int = DEFAULT_CHUNK_WORDS) -> bool:
        """Run a `start_long()` transaction to the end: every chunk, then CS high.

        The loop is host policy and nothing else, and it is the whole of "how a transaction is
        split", so it is worth stating plainly:

        1.  run the current chunk exactly as `run()` runs a transaction, until this chunk's bytes
            are all in the RX FIFO and the engine is parked on `done`.  CS has not moved: the
            `done` word is the held one (`load()`), so the wire sees a pause in SCK.
        2.  drain the RX FIFO and push the next chunk's TX words.  The host state machine keeps
            one access in flight and the engine is stopped by the arming below, so the FIFO never
            runs dry and no `FDEBUG` flag is raised.
        3.  `next_chunk()` queues the arming.  If there is no next chunk, `release_cs()` queues
            the one forced instruction that raises CS, and the loop runs `tail` more cycles so the
            rise is on the bus for the monitor.

        `max_ticks` is per chunk.  False if any chunk ran out of ticks, which is a failure and not
        a slow run: nothing about a long transaction is allowed to end early.
        """
        if self._released and self._chunks <= 1:
            raise ValueError("run_long() is for start_long(); a plain start() is run()")
        for _ in range(self._chunks):
            if not self.run(bus, max_ticks=max_ticks, tail=0):
                return False
            # The chunk is complete: its bytes are back and the engine is parked on `done`.
            if not self.next_chunk():
                # The last chunk: the host ends the transaction, not the program.
                self._drain_rx(bus)
                self.release_cs()
                bus.run(tail)
                return True
            # Between chunks the host must empty the RX FIFO before the engine restarts, or the
            # next chunk's pushes find it full.
            self._drain_rx(bus)
            for _ in range(max_ticks):
                if not self.dev.idle:
                    bus.step()
                    self._ticks += 1
                    continue
                break
        self._drain_rx(bus)
        self.release_cs()
        bus.run(tail)
        return True

    def _drain_rx(self, bus, max_ticks: int = 4096) -> None:
        """Read the RX FIFO empty, so the next chunk's pushes find room.

        The engine is stopped by the time this is called -- the arming queued `SM_ENABLE = 0` and
        this runs after the device queue -- so nothing pushes into the FIFO while it drains.  It
        keeps the same one-access-in-flight discipline as `tick()` rather than queueing reads of its
        own, because a read's answer arrives a cycle or two after it is queued and a second read
        issued before the first retires would be answered by the wrong one.
        """
        for _ in range(max_ticks):
            if self._rx_level <= 0:
                return
            if self._inflight is None:
                self._inflight = ("rx", self.dev.read(self.rxf))
            self.tick(bus)
        raise TimeoutError(f"RX FIFO still holds {self._rx_level} word(s) after {max_ticks} ticks")

    def read_reg(self, bus, addr: int, max_ticks: int = 64) -> int:
        """One register read, run to completion. For the test's register-level checks."""
        fut = self.dev.read(addr)
        for _ in range(max_ticks):
            bus.step()
            if fut.done():
                return fut.result()
        raise TimeoutError(f"no answer from word address {addr:#04x}")

    def read_fdebug(self, bus) -> int:
        """This engine's `FDEBUG` flags, which are W1C: read them, then clear them.

        Only this engine's byte is cleared, so a test that looks at both engines does not
        take the other's flags away.
        """
        raw = self.read_reg(bus, A_FDEBUG)
        mine = (raw >> (FDEBUG_STRIDE * self.sm)) & 0xFF
        if raw:
            self.dev.write(A_FDEBUG, raw)
            for _ in range(8):
                bus.step()
        self._fdebug |= mine
        return mine

