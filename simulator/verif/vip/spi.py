"""Pin-level SPI reference models on the tick-based bus in `core.py`.

Four nets, all push-pull: ``sck``, ``mosi``, ``miso``, ``cs_n`` (active low).

Timing model. A bit period is ``2 * half_period`` ticks. With ``T`` the tick the bit
period starts (the controller issues its first drive), the master drives

    ticks [T, T+half)            SCK = CPOL      MOSI = the bit to send
    ticks [T+half, T+2*half)     SCK = !CPOL     MOSI = the bit to send

so the leading edge (idle -> active) is issued at tick ``T + half - 1``, and the trailing
edge at ``T + 2*half``, the first tick after the active frames. ``mode`` picks CPOL/CPHA:

    0: CPOL 0, CPHA 0     1: CPOL 0, CPHA 1     2: CPOL 1, CPHA 1     3: CPOL 1, CPHA 0

The first edge of a bit period (idle -> active) is the *capture* edge when CPHA is 0 and
the *launch* edge when CPHA is 1; the second edge is the other one. Because `Bus` resolves
drives at the end of a tick and every device reacts one tick after it sees a change, a
device that sees an SCK edge at tick ``t`` can change its output at tick ``t + 1`` and the
new value is on the bus from ``t + 2``. So a device may only react *after* an edge, and a
master may sample MISO at the earliest tick the slave's value can be there. The controller
therefore samples MISO bit ``k`` at

    CPHA 0:  T_k + half + 1        (the slave launched it at the trailing edge, T_k + 2)
    CPHA 1:  T_k + 2*half + 1      (the slave launched it at the leading edge, T_k + half + 2)

which is the first tick the value can be present, so a slave one tick late still works.
half_period 1 is legal but degenerate for CPHA 0: the master's sample lands on the same
tick the slave's new bit appears, so there is zero margin. Use half_period >= 2 to test
anything that depends on setup time.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Optional, Sequence

from .core import Bus, Device

SCK = "sck"
MOSI = "mosi"
MISO = "miso"
CS_N = "cs_n"
NETS = (SCK, MOSI, MISO, CS_N)

#: CS_N is active low; 0 selects the device, 1 deselects it.
CS_ACTIVE = 0
CS_IDLE = 1


def cpol(mode: int) -> int:
    """Idle level of SCK for `mode`."""
    _check_mode(mode)
    return 1 if mode >= 2 else 0


def cpha(mode: int) -> int:
    """Clock phase for `mode`: 0 samples on the leading edge, 1 on the trailing one."""
    _check_mode(mode)
    return 0 if mode in (0, 3) else 1


def _check_mode(mode: int) -> None:
    if mode not in (0, 1, 2, 3):
        raise ValueError(f"SPI mode must be 0-3, got {mode}")


def bits_of(value: int, width: int = 8) -> list[int]:
    """`width` bits of `value`, MSB first."""
    return [(value >> (width - 1 - i)) & 1 for i in range(width)]


def byte_of(bits: Sequence[int], index: int = 0) -> int:
    """The byte at bit `index` of an MSB-first bit list."""
    window = bits[index:index + 8]
    return sum((b & 1) << (7 - i) for i, b in enumerate(window))


def is_capture_edge(mode: int, prev_sck: int, sck: int) -> bool:
    """True if this SCK transition is the one that samples the data bit.

    False when nothing moved, so a caller can use it without checking for an edge first.
    """
    _check_mode(mode)
    if prev_sck == sck:
        return False
    leading = (sck == 1) if cpol(mode) == 0 else (sck == 0)
    return leading if cpha(mode) == 0 else not leading


def first_output_index(mode: int) -> int:
    """The MISO bit a target puts on the bus when CS_N falls.

    Every launch edge advances one bit. With CPHA 0 the target presents bit 0 straight
    away and its first launch edge puts bit 1 on the bus; with CPHA 1 the first launch
    edge is the one that puts bit 0 out, so it presents a dummy (-1, read as 0) first.
    """
    _check_mode(mode)
    return 0 if cpha(mode) == 0 else -1


def spi_bus(ns_per_tick: float = 20.0, trace: bool = False) -> Bus:
    """A `Bus` with the four SPI nets, all push-pull."""
    bus = Bus(ns_per_tick=ns_per_tick, trace=trace)
    for name in NETS:
        bus.add_net(name, push_pull=True)
    return bus


def bytes_to_ints(data: Sequence[int]) -> list[int]:
    """`data` as a list of bytes, so callers can pass bytes or lists of ints."""
    return [int(b) & 0xFF for b in data]


@dataclass
class Transfer:
    """One queued transaction: `mosi` bytes out, `miso` bytes back in."""

    mosi: list[int] = field(default_factory=list)
    n_miso: int = 0
    #: MISO bytes sampled during this transaction.
    miso: list[int] = field(default_factory=list)
    #: Plan ticks, relative to the start of the run (None until planned).
    t_start: Optional[int] = None
    t_end: Optional[int] = None


#: One tick of driving: (cs_n, sck, mosi_bit, sample). `sample` is None, or the
#: Transfer whose MISO stream this tick banks one bit of.
_PlanTick = tuple[int, int, int, Optional["Transfer"]]


class SpiController(Device):
    """SPI master: drives SCK, MOSI and CS_N for a queue of transactions.

    A transaction is `(mosi bytes, number of bytes to read)`. Bytes are MSB first, and
    read bytes are padded with `fill` on MOSI. One transaction is one CS_N low window,
    which is what firmware does. Transactions may be queued before the first tick or
    between runs; `run` drives until the queue is empty.

    This is the reference chip firmware is compared against, and the driver for
    target-mode tests. Parameters:

        mode         0-3, see the module docstring
        half_period  ticks per SCK half period, integer >= 1
        cs_setup     ticks CS_N is low before the first SCK edge
        cs_hold      ticks CS_N stays low after the last SCK edge
        cs_gap       ticks CS_N is high between transactions; 0 merges them into one
                     CS_N low window, which only works for targets that read a stream
        fill         byte clocked out while reading (0x00)
        name         device name, the key used in bus contention records
    """

    name = "spi_ctl"

    def __init__(
        self,
        mode: int = 0,
        half_period: int = 4,
        cs_setup: int = 2,
        cs_hold: int = 2,
        cs_gap: int = 2,
        fill: int = 0x00,
        name: str = "spi_ctl",
    ):
        _check_mode(mode)
        if half_period < 1:
            raise ValueError(f"half_period must be >= 1 tick, got {half_period}")
        if cs_setup < 0 or cs_hold < 0 or cs_gap < 0:
            raise ValueError("cs_setup, cs_hold and cs_gap must be >= 0 ticks")
        self.mode = mode
        self.half = int(half_period)
        self.cs_setup = int(cs_setup)
        self.cs_hold = int(cs_hold)
        self.cs_gap = int(cs_gap)
        self.fill = int(fill) & 0xFF
        self.name = name

        self.queue: list[Transfer] = []
        #: Every MISO byte sampled, across all transactions.
        self.rx: list[int] = []
        #: Every MOSI byte the queue asked for. Read bytes are clocked out as `fill`
        #: and are not counted here; the monitor sees those too.
        self.tx: list[int] = []
        #: Every transaction, in order, whether or not it has run yet.
        self.transfers: list[Transfer] = []

        self._plan: list[_PlanTick] = []
        self._t0 = 0            # the bus tick that plan entry 0 runs on
        self._t = 0             # the last tick this device saw
        self._bits: list[int] = []

    # -- queueing ---------------------------------------------------------------

    def queue_transfer(self, mosi: Sequence[int], n_miso: int = 0) -> Transfer:
        """Add a transaction: send `mosi` bytes, then clock out `n_miso` bytes."""
        if n_miso < 0:
            raise ValueError("n_miso must be >= 0")
        xfer = Transfer(mosi=bytes_to_ints(mosi), n_miso=int(n_miso))
        self.queue.append(xfer)
        self.transfers.append(xfer)
        return xfer

    #: `add` reads better at a call site; `queue` is the queue attribute.
    add = queue_transfer

    def idle(self) -> bool:
        """True when nothing is left to drive."""
        return not self.queue and self._t + 1 >= self._t0 + len(self._plan)

    def pending_ticks(self, t: Optional[int] = None) -> int:
        """Ticks of driving left, so a caller can size a bus.run()."""
        if self.queue:
            self._plan_queue(self._t if t is None else t)
        return max(0, self._t0 + len(self._plan) - (self._t if t is None else t))

    def run(self, bus: Bus, limit: Optional[int] = None) -> int:
        """Run the bus until the queue is empty. Returns the ticks it took.

        Transactions may also be queued between runs; the plan is indexed by bus tick.
        """
        if limit is None:
            limit = self.pending_ticks(bus.t) + 2
        start = bus.t
        bus.run_until(lambda _b: self.idle(), limit)
        return bus.t - start

    # -- driving ----------------------------------------------------------------

    def tick(self, t: int, pins: Mapping[str, int]) -> dict[str, Optional[int]]:
        self._t = t
        if self.queue:
            self._plan_queue(t)
        i = t - self._t0
        if i >= len(self._plan):
            # Nothing left: park CS_N high and leave the other two lines released.
            return {SCK: cpol(self.mode), CS_N: CS_IDLE}
        cs_n, sck, mosi_bit, sample = self._plan[i]
        if sample is None:
            return {SCK: sck, MOSI: mosi_bit, CS_N: cs_n}
        # Sample from the snapshot, so the bit read is the one that was on the bus.
        self._sample(sample, pins.get(MISO, 1))
        return {SCK: sck, MOSI: mosi_bit, CS_N: cs_n}

    def _sample(self, xfer: Transfer, value: int) -> None:
        """Bank one sampled MISO bit; a full byte goes into the transfer it belongs to.

        Samples arrive in transaction order and every transaction samples a whole number
        of bytes, so one shared bit buffer is enough.
        """
        self._bits.append(value & 1)
        if len(self._bits) == 8:
            xfer.miso.append(byte_of(self._bits))
            self.rx.append(xfer.miso[-1])
            self._bits = []

    def _plan_queue(self, t: int) -> None:
        """Turn the queued transfers into a tick-by-tick drive plan starting at tick `t`.

        The plan is indexed by bus tick, so transactions can be queued at any time; an
        earlier plan keeps its ticks and the new ones start where the bus is now.
        """
        if not self.queue:
            return
        idle = cpol(self.mode)
        active = 1 - idle
        # The master's MISO sample for bit k lands this many ticks into bit period k.
        sample_offset = self.half + 1 if cpha(self.mode) == 0 else 2 * self.half + 1
        park = (CS_IDLE, idle, 0, None)
        # Ticks the bus already ran past the old plan are idle ones, so the plan stays
        # indexed by absolute tick and transactions can be queued at any time.
        if not self._plan:
            self._t0 = t
        while self._t0 + len(self._plan) < t:
            self._plan.append(park)
        base = self._t0 + len(self._plan)
        plan: list[_PlanTick] = []
        wanted: list[tuple[Transfer, int]] = []
        for n_xfer, xfer in enumerate(self.queue):
            if n_xfer:      # CS_N high between transactions, or a target sees one stream
                plan += [park] * self.cs_gap
            xfer.t_start = base + len(plan)
            out_bits: list[int] = []
            for b in xfer.mosi:
                out_bits += bits_of(b)
            out_bits += bits_of(self.fill) * xfer.n_miso
            n_tx = 8 * len(xfer.mosi)      # MISO is only sampled while the master reads
            plan += [(CS_ACTIVE, idle, 0, None)] * self.cs_setup
            for k, _bit in enumerate(out_bits):
                start_k = len(plan)
                # CPHA 1 launches on the leading edge, so MOSI only changes there.
                first = k if cpha(self.mode) == 0 else max(0, k - 1)
                for _ in range(self.half):
                    plan.append((CS_ACTIVE, idle, out_bits[first], None))
                for _ in range(self.half):
                    plan.append((CS_ACTIVE, active, out_bits[k], None))
                if k >= n_tx:
                    wanted.append((xfer, start_k + sample_offset))
            plan += [(CS_ACTIVE, idle, 0, None)] * self.cs_hold
            xfer.t_end = base + len(plan)
            self.tx += xfer.mosi
        # Slack so the last sample, which can be a tick past the last drive, lands.
        plan += [park] * (2 * self.half + 2)
        # Attach the samples after the fact: a transaction's last sample can land on a
        # later transaction's first tick when cs_hold is 0.
        for xfer, idx in wanted:
            cs_n, sck, mosi_bit, _ = plan[idx]
            plan[idx] = (cs_n, sck, mosi_bit, xfer)
        self._plan += plan
        self._bits = []
        self.queue = []


# Serial flash opcodes, as in every SPI flash datasheet.
CMD_JEDEC = 0x9F     # 3-byte JEDEC ID, no address
CMD_READ = 0x03      # 24-bit address, then data out until CS_N goes high
CMD_WREN = 0x06      # set write enable
CMD_WRDI = 0x04      # clear write enable
CMD_RDSR = 0x05      # one status byte out, bit 0 is WEL
CMD_PROGRAM = 0x02   # 24-bit address, then page data in, applied when CS_N goes high
CMD_ERASE = 0x20     # 24-bit address, 4 KB sector erased when CS_N goes high

PAGE_SIZE = 256
SECTOR_SIZE = 4096
WEL_BIT = 0          # bit 0 of the status register
#: Commands whose three address bytes follow the opcode.
ADDRESSED = (CMD_READ, CMD_PROGRAM, CMD_ERASE)


class SpiFlashTarget(Device):
    """A small serial flash: byte array, JEDEC ID, read, page program, sector erase.

    Modes 0 and 3, which is what flash datasheets support and what the chip's host port
    uses. It drives MISO only while CS_N is low (None, released, otherwise) and reacts one
    tick after an SCK edge, like a flop. Its output bit is a function of how many launch
    edges have gone by, so it answers exactly like the hardware does: 0 until the opcode
    and address are in, real data from then on.

    Programming needs WEL, merges with AND (programming only clears bits), wraps inside the
    256-byte page that holds the address, and is applied when CS_N goes high. Data past the
    end of that page is dropped. WEL clears after a successful program or erase unless
    `clear_wel_after_write` is False. An opcode the model does not know is ignored and the
    rest of the CS_N window is discarded.
    """

    name = "spi_flash"

    def __init__(
        self,
        size: int = 4096,
        jedec_id: Sequence[int] = (0xC0, 0x28, 0x17),
        mode: int = 0,
        fill: int = 0xFF,
        clear_wel_after_write: bool = True,
        cs_net: str = CS_N,
        sck_net: str = SCK,
        mosi_net: str = MOSI,
        miso_net: str = MISO,
        name: str = "spi_flash",
    ):
        if mode not in (0, 3):
            raise ValueError(
                f"SpiFlashTarget supports SPI modes 0 and 3 (CPHA 0), got mode {mode}"
            )
        if size <= 0:
            raise ValueError("size must be > 0")
        self.mode = mode
        self.size = int(size)
        self.jedec_id = bytes_to_ints(jedec_id)
        self.mem = bytearray([fill & 0xFF]) * self.size
        self.clear_wel_after_write = bool(clear_wel_after_write)
        self.cs_net = cs_net
        self.sck_net = sck_net
        self.mosi_net = mosi_net
        self.miso_net = miso_net
        self.name = name

        self.wel = 0                #: write-enable latch, bit 0 of the status register
        self.programs = 0           #: page programs that took effect
        self.erases = 0             #: sector erases that took effect
        self.last_command: Optional[int] = None

        # Transaction state, all of it reset whenever CS_N goes high.
        self._cs: Optional[int] = None
        self._sck: Optional[int] = None
        self._reset_session()

    # -- memory helpers ---------------------------------------------------------

    @property
    def status(self) -> int:
        """The status byte RDSR returns: WEL in bit `WEL_BIT`."""
        return (self.wel >> WEL_BIT) & 1

    def peek(self, addr: int, n: int) -> list[int]:
        """`n` bytes from `addr`, wrapping like the READ command does."""
        return [self.mem[(addr + i) % self.size] for i in range(n)]

    def poke(self, addr: int, data: Sequence[int]) -> None:
        """Load `data` at `addr` directly, with no commands involved (test setup)."""
        for i, b in enumerate(bytes_to_ints(data)):
            self.mem[(addr + i) % self.size] = b

    def _reset_session(self) -> None:
        """Forget the transaction in progress, ready for the next CS_N low window."""
        self._in_bits: list[int] = []
        self._out_idx = first_output_index(self.mode)
        self._cmd: Optional[int] = None
        self._cmd_seen = False
        self._nbytes = 0
        self._addr = 0
        self._out_base = 0
        self._status = 0
        self._read_addr = 0
        self._page = []

    # -- bus --------------------------------------------------------------------

    def tick(self, t: int, pins: Mapping[str, int]) -> dict[str, Optional[int]]:
        cs = pins[self.cs_net]
        sck = pins[self.sck_net]
        mosi = pins[self.mosi_net]
        if self._cs is None:
            self._cs = cs
            self._sck = sck
        elif cs != self._cs:
            # CS_N resets the shift registers, so whatever level SCK sits at when it moves
            # is not an edge. Taking it as one would read the pull-up level at t=0 as a
            # clock edge, so the level is taken before any edge is looked for.
            if cs == CS_ACTIVE:
                self._select()
            else:
                self._deselect()
            self._cs = cs
            self._sck = sck
        miso: Optional[int] = None
        if cs == CS_ACTIVE:
            if self._sck is not None and sck != self._sck:
                if is_capture_edge(self.mode, self._sck, sck):
                    self._capture(mosi)
                else:
                    self._launch()
            miso = self._out_bit(self._out_idx)
        self._sck = sck
        return {self.miso_net: miso}

    # -- command engine ---------------------------------------------------------

    def _select(self) -> None:
        """CS_N went low: a new transaction starts, MISO presents output bit 0."""
        self._reset_session()

    def _deselect(self) -> None:
        """CS_N went high: a program or erase lands here, then the session is dropped."""
        if self._cmd == CMD_PROGRAM and self._page:
            if self.wel:
                self._apply_program()
            self._page = []
        elif self._cmd == CMD_ERASE:
            if self.wel:
                self._erase()
        self._reset_session()

    def _capture(self, mosi: int) -> None:
        """One data bit in. Eight of them are a byte, which the command engine sees."""
        self._in_bits.append(mosi & 1)
        if len(self._in_bits) == 8:
            byte = byte_of(self._in_bits)
            self._in_bits = []
            self._byte(byte)

    def _launch(self) -> None:
        """One launch edge: the next output bit goes on the bus next tick."""
        self._out_idx += 1

    def _byte(self, byte: int) -> None:
        self._nbytes += 1
        if not self._cmd_seen:
            self._command(byte)
            return
        if self._cmd in ADDRESSED:
            if self._nbytes <= 4:      # the three address bytes
                self._addr = ((self._addr << 8) | byte) & 0xFFFFFF
                if self._nbytes == 4:
                    self._address_done()
                return
            if self._cmd == CMD_PROGRAM and len(self._page) < PAGE_SIZE:
                self._page.append(byte)   # bytes past the page are dropped, as on silicon
            return
        # No address and no data phase: extra bytes are ignored, as on real silicon.

    def _command(self, byte: int) -> None:
        self.last_command = byte
        self._cmd_seen = True
        if byte in (CMD_WREN, CMD_WRDI, CMD_RDSR, CMD_JEDEC, *ADDRESSED):
            self._cmd = byte
        else:
            self._cmd = None           # unknown opcode: stay quiet until CS_N goes high
            return
        self._addr = 0
        if byte == CMD_WREN:
            self.wel = 1
        elif byte == CMD_WRDI:
            self.wel = 0
        elif byte == CMD_RDSR:
            self._out_base = 8         # the status byte follows the opcode
            self._status = self.status
        elif byte == CMD_JEDEC:
            self._out_base = 8         # the ID follows the opcode
        else:
            self._out_base = 32        # opcode plus 24 address bits come first

    def _address_done(self) -> None:
        if self._cmd == CMD_READ:
            self._read_addr = self._addr
        elif self._cmd == CMD_PROGRAM:
            self._page = []
        elif self._cmd == CMD_ERASE:
            pass                        # the erase lands when CS_N goes high

    def _apply_program(self) -> None:
        """Merge the page in with AND: programming only clears bits, and it wraps."""
        page_base = (self._addr // PAGE_SIZE) * PAGE_SIZE
        offset = self._addr % PAGE_SIZE
        for i, byte in enumerate(self._page):
            addr = (page_base + (offset + i) % PAGE_SIZE) % self.size
            self.mem[addr] &= byte
        self.programs += 1
        if self.clear_wel_after_write:
            self.wel = 0

    def _erase(self) -> None:
        base = (self._addr // SECTOR_SIZE) * SECTOR_SIZE
        for i in range(SECTOR_SIZE):
            self.mem[(base + i) % self.size] = 0xFF
        self.erases += 1
        if self.clear_wel_after_write:
            self.wel = 0

    def _out_bit(self, index: int) -> int:
        """MISO bit `index` of this session, counting launch edges from CS_N low."""
        rel = index - self._out_base
        if rel < 0 or self._cmd is None:
            return 0
        if self._cmd == CMD_JEDEC:
            byte = rel // 8
            if byte >= len(self.jedec_id):
                return 0
            return (self.jedec_id[byte] >> (7 - rel % 8)) & 1
        if self._cmd == CMD_READ:
            return (self.mem[(self._read_addr + rel // 8) % self.size] >> (7 - rel % 8)) & 1
        if self._cmd == CMD_RDSR:
            return (self._status >> (7 - rel % 8)) & 1 if rel < 8 else 0
        return 0        # nothing to send while a program or erase address is in flight


@dataclass
class MonitoredTransfer:
    """What the monitor decoded for one CS_N low window."""

    mosi: list[int] = field(default_factory=list)
    miso: list[int] = field(default_factory=list)
    #: Ticks between CS_N falling and the first SCK edge (None if there were none).
    cs_setup: Optional[int] = None
    #: Ticks between the last SCK edge and CS_N rising (None if there were none).
    cs_hold: Optional[int] = None
    t_start: Optional[int] = None
    t_end: Optional[int] = None


class SpiMonitor(Device):
    """Passive bus watcher: decodes transfers and checks the timing rules.

    It drives nothing, so it can sit on a bus between a controller and a target and
    report what the wire actually carried. For `mode` it decodes each CS_N low window
    into (mosi bytes, miso bytes) and checks, in ticks:

        min_sck_ticks  the least SCK may hold either level
        min_cs_setup   the least CS_N may be low before the first SCK edge
        min_cs_hold    the least CS_N may be low after the last SCK edge

    Every breach is appended to `violations` as a string, so a test can assert on them
    and a log can print them. `stats` holds the smallest values seen, which is what a
    test usually wants to quote.

    Times are in ticks between the edges the monitor observed, and it observes them one
    tick after the drive that made them, so a controller's `cs_setup` shows up as
    `cs_setup + half_period` here, while its `cs_hold` shows up as it stands.
    """

    name = "spi_mon"

    def __init__(
        self,
        mode: int = 0,
        min_sck_ticks: int = 0,
        min_cs_setup: int = 0,
        min_cs_hold: int = 0,
        cs_net: str = CS_N,
        sck_net: str = SCK,
        mosi_net: str = MOSI,
        miso_net: str = MISO,
        name: str = "spi_mon",
    ):
        _check_mode(mode)
        if min_sck_ticks < 0 or min_cs_setup < 0 or min_cs_hold < 0:
            raise ValueError("the minimum times must be >= 0 ticks")
        self.mode = mode
        self.min_sck_ticks = int(min_sck_ticks)
        self.min_cs_setup = int(min_cs_setup)
        self.min_cs_hold = int(min_cs_hold)
        self.cs_net = cs_net
        self.sck_net = sck_net
        self.mosi_net = mosi_net
        self.miso_net = miso_net
        self.name = name

        self.transfers: list[MonitoredTransfer] = []
        self.violations: list[str] = []
        self.stats: dict[str, Optional[int]] = {
            "sck_low_min": None,     # shortest SCK low time seen
            "sck_high_min": None,    # shortest SCK high time seen
            "cs_setup_min": None,    # shortest CS_N setup seen
            "cs_hold_min": None,     # shortest CS_N hold seen
            "transactions": 0,
            "bits": 0,
        }

        self._cs: Optional[int] = None
        self._sck: Optional[int] = None
        self._sck_t: Optional[int] = None
        self._xfer: Optional[MonitoredTransfer] = None
        self._edges = 0
        self._in_bits: list[int] = []
        self._out_bits: list[int] = []

    @property
    def last(self) -> Optional[MonitoredTransfer]:
        return self.transfers[-1] if self.transfers else None

    def tick(self, t: int, pins: Mapping[str, int]) -> dict[str, Optional[int]]:
        cs = pins[self.cs_net]
        sck = pins[self.sck_net]
        if self._cs is None:
            self._cs, self._sck, self._sck_t = cs, sck, t
            return {}
        if cs != self._cs:
            if self._xfer is not None and sck != self._sck:
                # An SCK edge in the same tick CS_N moves still ends the clock run, so
                # the hold time is measured from it rather than from the edge before.
                self._sck, self._sck_t = sck, t
            if cs == CS_ACTIVE:
                self._select(t)
            else:
                self._deselect(t)
            self._cs = cs
            self._sck, self._sck_t = sck, t
            return {}
        if cs == CS_ACTIVE and self._xfer is not None:
            if sck != self._sck:
                self._edge(t, sck, pins[self.mosi_net], pins[self.miso_net])
                # Only an edge restarts the level timer.
                self._sck, self._sck_t = sck, t
        return {}

    def _select(self, t: int) -> None:
        self._xfer = MonitoredTransfer(t_start=t)
        self._in_bits = []
        self._out_bits = []

    def _deselect(self, t: int) -> None:
        """CS_N went high at tick `t`: the hold time is the tail of the last SCK level."""
        xfer = self._xfer
        if xfer is None:
            return
        xfer.t_end = t
        if self._edges and self._sck_t is not None:
            # The tail from the last edge to CS_N high is the hold time, not an SCK
            # level time: the clock has stopped, so it has its own check.
            xfer.cs_hold = t - self._sck_t
            self._note_min("cs_hold_min", xfer.cs_hold)
            if xfer.cs_hold < self.min_cs_hold:
                self._flag(f"cs hold {xfer.cs_hold} ticks < {self.min_cs_hold} at t={t}")
        self.transfers.append(xfer)
        self.stats["transactions"] = len(self.transfers)
        self._xfer = None
        self._edges = 0
        self._in_bits = []
        self._out_bits = []

    def _edge(self, t: int, sck: int, mosi: int, miso: int) -> None:
        """One SCK edge, observed at tick `t`."""
        xfer = self._xfer
        assert xfer is not None and self._sck_t is not None
        if self._edges == 0:
            # Ticks CS_N was low before the first edge: that is the setup time.
            xfer.cs_setup = t - (xfer.t_start if xfer.t_start is not None else t)
            self._note_min("cs_setup_min", xfer.cs_setup)
            if xfer.cs_setup < self.min_cs_setup:
                self._flag(
                    f"cs setup {xfer.cs_setup} ticks < {self.min_cs_setup} at t={t}"
                )
        else:
            self._level_time(t, sck, t - self._sck_t)
        self._edges += 1
        if is_capture_edge(self.mode, self._sck, sck):
            self._in_bits.append(mosi & 1)
            self._out_bits.append(miso & 1)
            self.stats["bits"] += 1
            if len(self._in_bits) == 8:
                xfer.mosi.append(byte_of(self._in_bits))
                xfer.miso.append(byte_of(self._out_bits))
                self._in_bits = []
                self._out_bits = []

    def _level_time(self, t: int, sck: int, ticks: int) -> None:
        """SCK held `sck` for `ticks` ticks ending at the edge observed at `t`."""
        level = "high" if sck == 1 else "low"
        self._note_min(f"sck_{level}_min", ticks)
        if ticks < self.min_sck_ticks:
            self._flag(f"sck {level} time {ticks} ticks < {self.min_sck_ticks} at t={t}")

    def _note_min(self, key: str, value: int) -> None:
        prev = self.stats[key]
        self.stats[key] = value if prev is None else min(prev, value)

    def _flag(self, message: str) -> None:
        self.violations.append(message)
