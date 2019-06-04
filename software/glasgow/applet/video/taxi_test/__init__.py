import argparse
import logging
import math
from migen import *
from migen.genlib.cdc import *

from ... import *


class TAXITestSubtarget(Module):
    def __init__(self, pads, in_fifo, sys_clk_freq):
        datax    = Signal(8)
        cmdx     = Signal(4)
        cstrb    = Signal()
        dstrb    = Signal()
        vltn     = Signal()
        self.specials += [
            MultiReg(pads.data_t.i,  datax),
            MultiReg(pads.cmd_t.i,   cmdx),
            MultiReg(pads.cstrb_t.i, cstrb),
            MultiReg(pads.dstrb_t.i, dstrb),
            MultiReg(pads.vltn_t.i,  vltn),
        ]

        dck = Signal()
        self.comb += [
            dck.eq(cstrb | dstrb),
        ]

        we    = Signal.like(in_fifo.we)
        din   = Signal.like(in_fifo.din)
        ovf   = Signal()
        ovf_c = Signal()
        self.comb += [
            in_fifo.we.eq(we & ~ovf),
            in_fifo.din.eq(din),
        ]
        self.sync += \
            ovf.eq(~ovf_c & (ovf | (we & ~in_fifo.writable)))

        dck_r = Signal()
        fifo_data_r = Signal(16)

        stb   = Signal()
        self.sync += [
            dck_r.eq(dck),
            fifo_data_r.eq(Cat(datax, cmdx, dstrb, cstrb, vltn, ovf)),
            stb.eq(~dck_r & dck),
        ]

        fifo_data = Signal(16)
        self.sync += \
            If(stb, fifo_data.eq(fifo_data_r))

        self.submodules.fsm = ResetInserter()(FSM(reset_state="CAPTURE-PIXEL"))
        for (state, offset, nextstate) in (
            ("REPORT-1",  0, "REPORT-2"),
            ("REPORT-2",  8, "CAPTURE-PIXEL"),
        ):
            self.fsm.act(state,
                din.eq((fifo_data >> offset) & 0xff),
                we.eq(1),
                NextState(nextstate)
            )
        self.fsm.act("CAPTURE-PIXEL",
            If(stb,
                NextState("REPORT-1")
            )
        )


class TAXITestApplet(GlasgowApplet, name="taxi-test"):
    preview = True
    logger = logging.getLogger(__name__)
    help = "capture data from AM7969 TAXIchip receiver"
    description = """
    no longer streams screen contents from a color parallel RGB555 LCD, such as Sharp LQ035Q7DH06.
    """

    @classmethod
    def add_build_arguments(cls, parser, access):
        access.add_build_arguments(parser)
        access.add_pin_set_argument(parser, "data", width=8)
        access.add_pin_set_argument(parser, "cmd", width=4)
        access.add_pin_argument(parser, "cstrb")
        access.add_pin_argument(parser, "dstrb")
        access.add_pin_argument(parser, "vltn")

    def build(self, target, args):
        self.mux_interface = iface = target.multiplexer.claim_interface(self, args)
        iface.add_subtarget(TAXITestSubtarget(
            pads=iface.get_pads(args, pins=("cstrb","dstrb","vltn",), pin_sets=("data", "cmd")),
            in_fifo=iface.get_in_fifo(depth=512 * 30, auto_flush=False),
            sys_clk_freq=target.sys_clk_freq,
        ))

    @classmethod
    def add_run_arguments(cls, parser, access):
        super().add_run_arguments(parser, access)

        parser.add_argument(
            "-f", "--file", metavar="FILE", type=argparse.FileType("wb"),
            help="file to write")


    async def run(self, device, args):
        iface = await device.demultiplexer.claim_interface(self, self.mux_interface, args)

        while True:
            data = (await iface.read(512))
            args.file.write(data)

# -------------------------------------------------------------------------------------------------

class TAXITestAppletTestCase(GlasgowAppletTestCase, applet=TAXITestApplet):
    @synthesis_test
    def test_build(self):
        self.assertBuilds(args=["--pins-r", "0:4", "--pins-g", "5:9", "--pins-b", "10:14",
                                "--pin-dck", "15", "--columns", "160", "--rows", "144",
                                "--vblank", "960"])
