import struct
import logging
from nmigen import *
from nmigen.lib.cdc import FFSynchronizer

from ....support.logging import *
from ... import *


class SPISnifferBus(Elaboratable):
    def __init__(self, pads, sck_edge, cs_active):
        self._pads      = pads
        self._sck_edge  = sck_edge
        self._cs_active = cs_active

        self.strobe   = Signal()
        self.selected = Signal()
        self.data     = Signal.like(pads.data_t.i)

    def elaborate(self, platform):
        m = Module()

        sck = Signal()
        cs  = Signal()
        m.d.comb += [
            self._pads.sck_t.oe.eq(0),
            self._pads.cs_t.oe.eq(0),
            self._pads.data_t.oe.eq(0),
        ]
        m.submodules += [
            FFSynchronizer(self._pads.sck_t.i,  sck),
            FFSynchronizer(self._pads.cs_t.i,   cs),
            FFSynchronizer(self._pads.data_t.i, self.data),
        ]

        sck_r = Signal()
        m.d.sync += sck_r.eq(sck)
        if self._sck_edge in ("r", "rising"):
            m.d.comb += [
                self.strobe.eq(~sck_r &  sck),
            ]
        elif self._sck_edge in ("f", "falling"):
            m.d.comb += [
                self.strobe.eq( sck_r & ~sck),
            ]
        else:
            assert False
        m.d.comb += self.selected.eq(cs if self._cs_active else ~cs)

        return m


class MultibyteSource(Elaboratable):
    def __init__(self, bytes_in):
        self._bytes = bytes_in
        self._counter = Signal(range(self._bytes))

        self.w_en   = Signal()
        self.w_data = Signal(bytes_in*8)
        self.r_en   = Signal()
        self.r_rdy  = Signal()
        self.r_data = Signal(8)

    def elaborate(self, platform):
        m = Module()

        data = Signal.like(self.w_data)
        m.d.comb += self.r_data.eq(data[0:8])
        with m.FSM() as fsm:
            with m.State("IDLE"):
                with m.If(self.w_en):
                    m.d.sync += [
                        data.eq(self.w_data),
                        self._counter.eq(self._bytes - 1),
                        self.r_rdy.eq(1),
                    ]
                    m.next = "SHIFT"

            with m.State("SHIFT"):
                with m.If(self.r_en):
                    with m.If(self._counter == 0):
                        m.d.sync += self.r_rdy.eq(0)
                        m.next = "IDLE"
                    with m.Else():
                        m.d.sync += [
                            data.eq(data.shift_right(8)),
                            self._counter.eq(self._counter - 1),
                        ]

        return m


class SPISnifferSubtarget(Elaboratable):
    def __init__(self, pads, in_fifo, max_length, sck_edge, cs_active):
        self.bus = SPISnifferBus(pads, sck_edge, cs_active)
        self._length = max_length
        self._fifo = in_fifo

    def elaborate(self, platform):
        m = Module()
        m.submodules += self.bus

        mask = Signal(self._length * 8)
        data = [Signal(self._length * 8) for _ in self.bus.data]

        m.submodules.mb = mb = MultibyteSource((1 + len(data)) * self._length)
        m.d.comb += [
            mb.w_data        .eq(Cat(mask, *data)),
            mb.r_en          .eq(self._fifo.w_rdy),
            self._fifo.w_en   .eq(mb.r_rdy),
            self._fifo.w_data .eq(mb.r_data),
        ]

        with m.FSM() as fsm:
            with m.State("START"):
                # Wait for a deselect to ensure we don't capture half a transaction.
                with m.If(~self.bus.selected):
                    m.next = "IDLE"

            with m.State("IDLE"):
                m.d.sync += mb.w_en.eq(0),
                with m.If(self.bus.selected):
                    m.d.sync += mask.eq(0)
                    for i in range(len(data)):
                        m.d.sync += data[i].eq(0)
                    m.next = "SELECTED"

            with m.State("SELECTED"):
                with m.If(self.bus.strobe):
                    m.d.sync += mask.eq(Cat(Const(1, 1), mask[0:-1]))
                    for i in range(len(data)):
                        m.d.sync += data[i].eq(Cat(self.bus.data[i], data[0:-1]))

                with m.If(~self.bus.selected):
                    m.d.sync += mb.w_en.eq(1),
                    m.next = "IDLE"


        return m


class SPIControllerInterface:
    def __init__(self, interface, logger, count, length):
        self.lower   = interface
        self._logger = logger
        self._level  = logging.DEBUG if self._logger.name == __name__ else logging.TRACE
        self._count  = count
        self._length = length

    def _log(self, message, *args):
        self._logger.log(self._level, "SPI: " + message, *args)

    async def reset(self):
        self._log("reset")
        await self.lower.reset()

    async def read(self):
        data = (await self.lower.read((self._count + 1) * self._length))
        self._log("read-in=<%s>", dump_hex(data))
        return data


class SPISnifferApplet(GlasgowApplet, name="spi-sniffer"):
    logger = logging.getLogger(__name__)
    help = "initiate SPI transactions"
    description = """
    Initiate transactions on the SPI bus.
    """

    @classmethod
    def add_build_arguments(cls, parser, access, omit_pins=False):
        super().add_build_arguments(parser, access)

        if not omit_pins:
            access.add_pin_argument(parser, "sck", required=True)
            access.add_pin_argument(parser, "cs", required=True)
            access.add_pin_set_argument(parser, "data", width=range(1, 17), required=True)

        parser.add_argument(
            "--sck-edge", metavar="EDGE", type=str, choices=["r", "rising", "f", "falling"],
            default="rising",
            help="latch data at clock edge EDGE (default: %(default)s)")
        parser.add_argument(
            "--cs-active", metavar="LEVEL", type=int, choices=[0, 1], default=0,
            help="set active chip select level to LEVEL (default: %(default)s)")
        parser.add_argument(
            "--length", type=int, default=4,
            help="set max transaction length in bytes")

    def build(self, target, args):
        self.mux_interface = iface = target.multiplexer.claim_interface(self, args)
        return iface.add_subtarget(SPISnifferSubtarget(
            pads=iface.get_pads(args, pins=("sck", "cs"), pin_sets=("data",)),
            in_fifo=iface.get_in_fifo(auto_flush=True),
            max_length=args.length,
            sck_edge=args.sck_edge,
            cs_active=args.cs_active,
        ))

    async def run(self, device, args):
        iface = await device.demultiplexer.claim_interface(self, self.mux_interface, args)
        spi_iface = SPIControllerInterface(iface, self.logger, len(args.pin_set_data), args.length)
        return spi_iface

    @classmethod
    def add_interact_arguments(cls, parser):
        pass

    async def interact(self, device, args, spi_iface):
        data_count = len(args.pin_set_data)
        byte_count = args.length
        while True:
            buffer = await spi_iface.read()
            mask = int.from_bytes(buffer[0:byte_count], "big")
            data = [
                mask & int.from_bytes(buffer[(i+1)*byte_count:(i+2)*byte_count], "big")
                for i in range(data_count)
            ]
            print(bin(i) for i in data)

