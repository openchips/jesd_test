#!/usr/bin/env python3

#
# This file is part of LitePCIe.
#
# Copyright (c) 2015-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import argparse

from functools import reduce
from operator import and_

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex_boards.platforms import xilinx_kc705

from litex.soc.cores.clock import S7MMCM
from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from litex.build.generic_platform import *
from litex.build.xilinx import Xilinx7SeriesPlatform

from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.core import LitePCIeEndpoint, LitePCIeMSI
from litepcie.frontend.dma import LitePCIeDMA
from litepcie.frontend.wishbone import LitePCIeWishboneBridge
from litepcie.software import generate_litepcie_software

from litex.soc.cores.code_8b10b import K

from litex.build.io import DifferentialInput, DifferentialOutput

from litex.soc.cores.clock import S7MMCM

from liteiclink.serdes.gtp_7series import GTPQuadPLL, GTP
from liteiclink.serdes.gtx_7series import GTXQuadPLL, GTX

from litejesd204b.common import *
from litejesd204b.core import LiteJESD204BCoreTX
from litejesd204b.core import LiteJESD204BCoreRX
from litejesd204b.core import LiteJESD204BCoreControl
from litex.soc.cores import dna, uart, spi, freqmeter

from litescope import LiteScopeAnalyzer

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.cd_sys = ClockDomain()

        # # #

        # PLL
        self.pll = pll = S7MMCM(speedgrade=-2)
        pll.register_clkin(platform.request("clk200"), 200e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)

# LitePCIeSoC --------------------------------------------------------------------------------------

class LitePCIeSoC(SoCMini):
    configs = {
        # Gen2  data_width, sys_clk_freq
        "gen2:x1": (64,   int(125e6)),
        "gen2:x4": (128,  int(200e6)),
        "gen2:x8": (128,  int(200e6)),
    }
    def __init__(self, platform, speed="gen2", nlanes=4):
        data_width, sys_clk_freq = self.configs[speed + f":x{nlanes}"]

        platform.add_extension([
            ("ADC08DJ5200RF_JESD204", 0,
                # jesd clk A from ADC
                Subsignal("jesd_clk_a_p", Pins("HPC:GBTCLK0_M2C_P")),
                Subsignal("jesd_clk_a_n", Pins("HPC:GBTCLK0_M2C_N")),

                # GTX data lanes
                Subsignal("tx_p",  Pins(" ".join(
                    ["HPC:DP{}_C2M_P".format(i) for i in [3, 2, 1, 0][:4]]
                ))),
                Subsignal("tx_n",  Pins(" ".join(
                    ["HPC:DP{}_C2M_N".format(i) for i in [3, 2, 1, 0][:4]]
                ))),

                # JSYNC comes from AD9174 SYNC_OUT_0B, SYNC_OUT_1B
                #Subsignal("jsync0_p", Pins("HPC:LA01_CC_P"), IOStandard("LVDS")),
                #Subsignal("jsync0_n", Pins("HPC:LA01_CC_N"), IOStandard("LVDS")),

                # SYSREF_FPGA from ADC
                Subsignal("sysref_p", Pins("HPC:LA03_P"), IOStandard("LVDS_25"), Misc("DIFF_TERM=TRUE")),
                Subsignal("sysref_n", Pins("HPC:LA03_N"), IOStandard("LVDS_25"), Misc("DIFF_TERM=TRUE")),

                # SYSREF comes from HMC7044 CLKOUT13 (16 MHz)
                # Subsignal("sysref_p", Pins("HPC:LA00_CC_P"), IOStandard("LVDS")),
                # Subsignal("sysref_n", Pins("HPC:LA00_CC_N"), IOStandard("LVDS"))
            ),
            # ("AD9174_SPI", 0,
            #     # FMC_CS1 (AD9174), FMC_CS2 (HMC7044)
            #     Subsignal("cs_n", Pins("HPC:LA04_N HPC:LA05_P")),
            #     Subsignal("miso", Pins("HPC:LA04_P"), Misc("PULLUP TRUE")),
            #     Subsignal("mosi", Pins("HPC:LA03_N")),
            #     Subsignal("clk",  Pins("HPC:LA03_P")),
            #     Subsignal("spi_en", Pins("HPC:LA05_N")),
            #     IOStandard("LVCMOS18")
            # ),
        ])

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq, ident=f"LitePCIe example design on KC705 ({speed}:x{nlanes})")

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # UARTBone ---------------------------------------------------------------------------------
        self.add_uartbone()

        # LiteICLink
        # self.serdes = serdes = liteiclink.LiteICLink(platform.request("HPC.GBTCLK0_M2C_P"))
        
        # Counter
        counter = Signal(32)
        
        # Logic to update counter based on SerDes input
        # self.sync += If(serdes.sink.valid,
        #     counter.eq(counter + 1)
        # )

        # # JESD Clocking (Device) -------------------------------------------------------------------
        # userclk_freq = ad937x_jesd_linerate/40
        # self.clock_domains.cd_jesd_122_88 = ClockDomain()
        # self.clock_domains.cd_jesd_61_44  = ClockDomain()
        # self.clock_domains.cd_jesd        = ClockDomain()
        # self.clock_domains.cd_clk122_88   = ClockDomain()
        #refclk_pads_p = platform.request("HPC.GBTCLK0_M2C_P")


        serd_pads = platform.request("ADC08DJ5200RF_JESD204")

        self.jref = j_ref = Signal()
        self.specials += DifferentialInput(
            serd_pads.sysref_p, serd_pads.sysref_n, j_ref
        )
        #self.core.register_jref(j_ref)
        #self.f_ref = freqmeter.FreqMeter(
        #     self.sys_clk_freq,
        #     clk=j_ref
        #)


        self.usrclk = usrclk = Signal()
        usrclk_pads = platform.request("user_sma_clock")
        self.specials += DifferentialOutput(usrclk, usrclk_pads.p, usrclk_pads.n)

        self.comb += [
            usrclk.eq(j_ref)
        ]

        # Analyzer
        analyzer_groups = {
            0: [
                self.jref,
            ]
        }
        self.submodules.analyzer = LiteScopeAnalyzer(
            analyzer_groups,
            4096,
            csr_csv="build/analyzer.csv",
            clock_domain='sys'
        )
        self.add_csr("analyzer")

        # LED Control
        self.comb += platform.request("user_led").eq(counter[25])



        # ad937x_phy               = "gtx"
        # ad937x_phy_tx_order      = [0, 1, 2, 3]
        # ad937x_phy_rx_order      = [0, 1, 2, 3]
        # ad937x_refclk_freq       = 122.88e6
        # ad937x_jesd_linerate     = 2.4576e9

        # # JESD Configuration -----------------------------------------------------------------------
        # jesd_lanes = len(ad937x_phy_tx_order)

        # # 2 lanes / 4 converters / (4.9152Gbps linerate : IQ rate 61.44MSPS)
        # if jesd_lanes == 2:
        #     ps_tx = JESD204BPhysicalSettings(l=2, m=4, n=16, np=16)
        #     ts_tx = JESD204BTransportSettings(f=4, s=1, k=32, cs=0)
        #     settings_tx = JESD204BSettings(ps_tx, ts_tx, did=0x5a, bid=0x5, framing=framing, scrambling=scrambling)

        #     ps_rx = JESD204BPhysicalSettings(l=2, m=4, n=16, np=16)
        #     ts_rx = JESD204BTransportSettings(f=4, s=1, k=32, cs=0)
        #     settings_rx = JESD204BSettings(ps_rx, ts_rx, did=0x5a, bid=0x5, framing=framing, scrambling=scrambling)
        # # 4 lanes / 4 converters / (2.4576Gbps linerate : IQ rate 122.88MSPS)
        # elif jesd_lanes == 4:
        #     ps_tx = JESD204BPhysicalSettings(l=4, m=4, n=16, np=16)
        #     ts_tx = JESD204BTransportSettings(f=2, s=1, k=32, cs=0)
        #     settings_tx = JESD204BSettings(ps_tx, ts_tx, did=0x5a, bid=0x5, framing=framing, scrambling=scrambling)

        #     ps_rx = JESD204BPhysicalSettings(l=4, m=4, n=16, np=16)
        #     ts_rx = JESD204BTransportSettings(f=2, s=1, k=32, cs=0)
        #     settings_rx = JESD204BSettings(ps_rx, ts_rx, did=0x5a, bid=0x5, framing=framing, scrambling=scrambling)
        # else:
        #     raise NotImplementedError

        # # JESD Clocking (Device) -------------------------------------------------------------------
        # userclk_freq = ad937x_jesd_linerate/40
        # self.clock_domains.cd_jesd_122_88 = ClockDomain()
        # self.clock_domains.cd_jesd_61_44  = ClockDomain()
        # self.clock_domains.cd_jesd        = ClockDomain()
        # self.clock_domains.cd_clk122_88   = ClockDomain()
        # refclk_pads = platform.request("ad937x_refclk")
        # refclk      = Signal()
        # refclk_div2 = Signal()
        # self.specials += Instance("IBUFDS_GTE2",
        #     i_CEB   = 0,
        #     i_I     = refclk_pads.p,
        #     i_IB    = refclk_pads.n,
        #     o_O     = refclk,
        #     o_ODIV2 = refclk_div2)
        # self.submodules.pll = pll = S7MMCM(speedgrade=-2)
        # pll.register_clkin(refclk_div2, ad937x_refclk_freq/2)
        # pll.create_clkout(self.cd_jesd_122_88, userclk_freq,   buf=None, with_reset=False)
        # pll.create_clkout(self.cd_jesd_61_44,  userclk_freq/2, buf=None, with_reset=False)
        # pll.create_clkout(self.cd_clk122_88,   122.88e6, with_reset=False)
        # self.specials += Instance("BUFGMUX",
        #     i_S  = self._speed.storage,
        #     i_I0 = ClockSignal("jesd_122_88"),
        #     i_I1 = ClockSignal("jesd_61_44"),
        #     o_O  = ClockSignal("jesd")
        # )
        # platform.add_period_constraint(refclk_div2, 1e9/(ad937x_refclk_freq/2))

        # # JESD Clocking (SYSREF) -------------------------------------------------------------------
        # self.sysref = sysref = Signal()
        # sysref_pads = platform.request("ad937x_sysref")
        # self.specials += DifferentialInput(sysref_pads.p, sysref_pads.n, sysref)

        # # JESD PHYs --------------------------------------------------------------------------------
        # jesd_pll_cls = {
        #     "gtx": GTXQuadPLL,
        #     "gtp": GTPQuadPLL,
        # }[ad937x_phy]
        # jesd_phy_cls = {
        #     "gtx": GTX,
        #     "gtp": GTP,
        # }[ad937x_phy]
        # jesd_phy_data_width = {
        #     "gtx": 20,
        #     "gtp": 20,
        # }[ad937x_phy]

        # jesd_pll = jesd_pll_cls(refclk, ad937x_refclk_freq, ad937x_jesd_linerate)
        # self.submodules += jesd_pll
        # #print(jesd_pll)

        # self.jesd_phys = jesd_phys = []
        # for i in range(jesd_lanes):
        #     jesd_tx_pads = platform.request("ad937x_jesd_tx", i)
        #     jesd_rx_pads = platform.request("ad937x_jesd_rx", i)
        #     jesd_phy = jesd_phy_cls(jesd_pll, jesd_tx_pads, jesd_rx_pads, sys_clk_freq,
        #         data_width       = jesd_phy_data_width,
        #         clock_aligner    = False,
        #         tx_buffer_enable = True,
        #         rx_buffer_enable = True)
        #     jesd_phy.add_stream_endpoints()
        #     jesd_phy.add_controls(auto_enable=False)
        #     jesd_phy.n = i
        #     setattr(self.submodules, "jesd_phy" + str(i), jesd_phy)
        #     platform.add_period_constraint(jesd_phy.cd_tx.clk, 1e9/jesd_phy.tx_clk_freq)
        #     platform.add_period_constraint(jesd_phy.cd_rx.clk, 1e9/jesd_phy.rx_clk_freq)
        #     platform.add_false_path_constraints(
        #         soc.crg.cd_sys.clk,
        #         self.cd_jesd.clk,
        #         jesd_phy.cd_tx.clk,
        #         jesd_phy.cd_rx.clk)
        #     jesd_phys.append(jesd_phy)

        # jesd_phys_tx_init_done = reduce(and_, [phy.tx_init.done for phy in jesd_phys])
        # jesd_phys_rx_init_done = reduce(and_, [phy.rx_init.done for phy in jesd_phys])
        # self.specials += AsyncResetSynchronizer(self.cd_jesd, ~(jesd_phys_tx_init_done & jesd_phys_rx_init_done))

        # jesd_phys_tx = [jesd_phys[n] for n in ad937x_phy_tx_order]
        # jesd_phys_rx = [jesd_phys[n] for n in ad937x_phy_rx_order]

        # # JESD TX ----------------------------------------------------------------------------------
        # self.submodules.jesd_tx_core    = LiteJESD204BCoreTX(jesd_phys_tx, settings_tx,
        #     converter_data_width = jesd_lanes*8,
        #     scrambling           = scrambling,
        #     stpl_random          = stpl_random)
        # self.submodules.jesd_tx_control = LiteJESD204BCoreControl(self.jesd_tx_core, sys_clk_freq)
        # self.jesd_tx_core.register_jsync(platform.request("ad937x_sync_tx"))
        # self.jesd_tx_core.register_jref(sysref)

        # # JESD RX ----------------------------------------------------------------------------------
        # self.submodules.jesd_rx_core    = LiteJESD204BCoreRX(jesd_phys_rx, settings_rx,
        #     converter_data_width = jesd_lanes*8,
        #     scrambling           = scrambling,
        #     stpl_random          = stpl_random)
        # self.submodules.jesd_rx_control = LiteJESD204BCoreControl(self.jesd_rx_core, sys_clk_freq)
        # self.jesd_rx_core.register_jsync(platform.request("ad937x_sync_rx"))
        # self.jesd_rx_core.register_jref(sysref)

        # # JESD Link Status ------------------------------------------------------------------------------
        # self.jesd_link_status = Signal()
        # self.comb += self.jesd_link_status.eq(
        #     (self.jesd_tx_core.enable & self.jesd_tx_core.jsync) &
        #     (self.jesd_rx_core.enable & self.jesd_rx_core.jsync))

        # talise_refclk_freq   = 245.76e6
        # talise_jesd_linerate = 4.9152e9

        # # PHY
        # self.pcie_phy = S7PCIEPHY(platform, platform.request(f"pcie_x{nlanes}"),
        #     data_width = data_width,
        #     bar0_size  = 0x20000,
        # )

        # ps_rx = JESD204BPhysicalSettings(l=4, m=4, n=16, np=16)
        # ts_rx = JESD204BTransportSettings(f=2, s=1, k=32, cs=0)
        # settings_rx = JESD204BSettings(ps_rx, ts_rx, did=0x5a, bid=0x5, framing=False, scrambling=False)

        # self.jesd204b_transport = LiteJESD204BTransportRX()

        # # PCIe -------------------------------------------------------------------------------------
        # # PHY
        # self.pcie_phy = S7PCIEPHY(platform, platform.request(f"pcie_x{nlanes}"),
        #     data_width = data_width,
        #     bar0_size  = 0x20000,
        # )
        # self.pcie_phy.add_ltssm_tracer()

        # # Endpoint
        # self.pcie_endpoint = LitePCIeEndpoint(self.pcie_phy,
        #     endianness           = "big",
        #     max_pending_requests = 8
        # )

        # # Wishbone bridge
        # self.pcie_bridge = LitePCIeWishboneBridge(self.pcie_endpoint,
        #     base_address = self.mem_map["csr"])
        # self.bus.add_master(master=self.pcie_bridge.wishbone)

        # # DMA0
        # self.pcie_dma0 = LitePCIeDMA(self.pcie_phy, self.pcie_endpoint,
        #     with_buffering = True, buffering_depth=1024,
        #     with_loopback  = True)

        # # DMA1
        # self.pcie_dma1 = LitePCIeDMA(self.pcie_phy, self.pcie_endpoint,
        #     with_buffering = True, buffering_depth=1024,
        #     with_loopback  = True)

        # self.add_constant("DMA_CHANNELS", 2)
        # self.add_constant("DMA_ADDR_WIDTH", 32)

        # # MSI
        # self.pcie_msi = LitePCIeMSI()
        # self.comb += self.pcie_msi.source.connect(self.pcie_phy.msi)
        # self.interrupts = {
        #     "PCIE_DMA0_WRITER":    self.pcie_dma0.writer.irq,
        #     "PCIE_DMA0_READER":    self.pcie_dma0.reader.irq,
        #     "PCIE_DMA1_WRITER":    self.pcie_dma1.writer.irq,
        #     "PCIE_DMA1_READER":    self.pcie_dma1.reader.irq,
        # }
        # for i, (k, v) in enumerate(sorted(self.interrupts.items())):
        #     self.comb += self.pcie_msi.irqs[i].eq(v)
        #     self.add_constant(k + "_INTERRUPT", i)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LitePCIe SoC on KC705")
    parser.add_argument("--build",  action="store_true", help="Build bitstream")
    parser.add_argument("--driver", action="store_true", help="Generate LitePCIe driver")
    parser.add_argument("--load",   action="store_true", help="Load bitstream (to SRAM)")
    parser.add_argument("--nlanes", default=4,           help="PCIe lanes: 1, 4 (default) or 8")
    args = parser.parse_args()

    platform = xilinx_kc705.Platform()
    soc      = LitePCIeSoC(platform, nlanes=int(args.nlanes))
    builder  = Builder(soc, output_dir="build/kc705", csr_csv="csr.csv")
    builder.build(build_name="kc705", run=args.build)

    if args.driver:
        generate_litepcie_software(soc, os.path.join(builder.output_dir, "driver"))

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

if __name__ == "__main__":
    main()



# from migen import *
# from litex.build.generic_platform import *
# from litex.build.xilinx import XilinxPlatform
# from litex.soc.cores.clock import *
# from litex.soc.interconnect import stream
# from litex.soc.cores import liteiclink

# # KC705 Platform Definition
# _io = [
#     ("clk200", 0, Pins("AB8"), IOStandard("LVDS_25")),
#     ("rst_n", 0, Pins("H6"), IOStandard("LVCMOS25")),
#     ("user_led", 0, Pins("AA21"), IOStandard("LVCMOS25")),
#     # LiteICLink
#     ("dp_m2c", 0,
#         Subsignal("p", Pins("Y2")),
#         Subsignal("n", Pins("Y1")),
#         IOStandard("LVDS_25")
#     ),
# ]

# class KC705Platform(XilinxPlatform):
#     def __init__(self):
#         XilinxPlatform.__init__(self, "xc7k325t-ffg900-2", _io)

# # LiteX SoC
# class LiteXSoC(Module):
#     def __init__(self, platform):
#         self.clock_domains.cd_sys = ClockDomain()
#         self.submodules.pll = pll = S7PLL(speedgrade=-2)
#         self.comb += pll.reset.eq(~platform.request("rst_n"))
#         pll.register_clkin(platform.request("clk200"), 200e6)
#         pll.create_clkout(self.cd_sys, 100e6)

        

# # Generate the SoC
# platform = KC705Platform()
# soc = LiteXSoC(platform)
# builder = Builder(soc, output_dir="build", csr_csv="test/csr.csv")
# builder.build()
