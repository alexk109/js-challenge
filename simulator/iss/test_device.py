"""ChipDevice: the ISS on a verif.vip.core.Bus (docs/mvp-interfaces.md `## ISS API`)."""
from verif.vip.core import Bus, Device

from iss.pio_iss import ChipDevice
from iss.testkit import CTRL, DBG_CFGINFO, FLEVEL, RXF, TXF, execctrl, imem_addr, jmp, mov, \
    pinctrl, push, set_, sm


class Driver(Device):
    name = "drv"

    def __init__(self):
        self.drive = {}

    def tick(self, t, pins):
        return dict(self.drive)


def make():
    bus = Bus()
    for k in range(18):
        bus.add_net(f"p{k}")
    dev = bus.attach(ChipDevice())
    drv = bus.attach(Driver())
    return bus, dev, drv


def drain(bus, dev, extra=0):
    while not dev.idle:
        bus.step()
    bus.run(extra)


def test_read_future_and_queue_order():
    bus, dev, _ = make()
    dev.write(TXF[0], 5)
    f1 = dev.read(FLEVEL)
    f2 = dev.read(DBG_CFGINFO)
    assert not f1.done()
    bus.step()
    assert not f1.done()
    bus.step()
    assert f1.result() == 1
    bus.step()
    assert f2.result() == 0x0020_0204 and dev.idle


def test_drive_rule_after_reset():
    bus, dev, _ = make()
    bus.step()
    for k in range(13):
        assert bus.values[f"p{k}"] == 1          # released: pull-up
    for k in range(13, 18):
        assert bus.values[f"p{k}"] == 0          # output-only pins always driven


def test_open_drain_on_uio_pin():
    bus, dev, _ = make()
    dev.write(sm(0, "PINCTRL"), pinctrl(set_base=0, set_count=1))
    dev.write(sm(0, "INSTR"), set_("pindirs", 1))  # level stays 0: pull low
    drain(bus, dev, 2)
    assert bus.values["p0"] == 0
    dev.write(sm(0, "INSTR"), set_("pindirs", 0))  # release
    drain(bus, dev, 2)
    assert bus.values["p0"] == 1


def test_inputs_come_from_nets():
    bus, dev, drv = make()
    drv.drive = {"p9": 0, "p2": 0}
    bus.run(5)
    dev.write(sm(0, "INSTR"), mov("x", "pins"))
    drain(bus, dev, 1)
    x = dev.chip.state(0)["x"]
    assert x == 0x01FFF & ~(1 << 9) & ~(1 << 2)


def test_loopback_latency_on_bus():
    """Chip drives uio[0] high: an unloaded net follows, and IN_VEC sees it T_OUT + T_IN later."""
    bus, dev, _ = make()
    dev.write(sm(0, "PINCTRL"), pinctrl(set_base=0, set_count=1))
    dev.write(sm(0, "INSTR"), set_("pins", 0))
    dev.write(sm(0, "INSTR"), set_("pindirs", 1))
    drain(bus, dev, 6)
    assert bus.values["p0"] == 0
    dev.write(sm(0, "INSTR"), set_("pins", 1))   # tick w: write; executes in w+1
    bus.step()
    seen = []
    for _ in range(6):
        bus.step()
        seen.append(dev.chip.last_in_vec & 1)
    # seen[i] is IN_VEC in w+1+i: executes w+1, on the pad in w+2, in IN_VEC from w+5
    assert seen == [0, 0, 0, 0, 1, 1]
    assert bus.values["p0"] == 1


def test_program_runs_and_pushes_through_device():
    bus, dev, drv = make()
    prog = [mov("isr", "pins"), push(), jmp(2)] + [jmp(2)] * 29
    for k, w in enumerate(prog):
        dev.write(imem_addr(1, k), w)
    dev.write(sm(1, "PINCTRL"), pinctrl(in_base=8))
    dev.write(CTRL, 0x202 | 0x20)
    drv.drive = {"p8": 0, "p10": 0}
    drain(bus, dev, 6)
    f = dev.read(RXF[1])
    drain(bus, dev)
    assert f.result() & 0x1F == 0b11010
