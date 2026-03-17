import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge, ReadOnly, NextTimeStep, Event, with_timeout
from cocotbext.ahb import AHBBus, AHBLiteMaster
from cocotbext.axi import AxiBus, AxiRam

import itertools
_TEST_IDS = itertools.count(1)

def set_test_id(dut):
    dut.tb_test_id.value = next(_TEST_IDS)

CLK_NS = 10
RAM_SIZE = 64 * 1024

AHB_IDLE = 0
AHB_BUSY = 1
AHB_NONSEQ = 2
AHB_SEQ = 3

AHB_BURST_SINGLE = 0
AHB_BURST_INCR4 = 3

AHB_SIZE_8 = 3  # 2^3 = 8 bytes / 64-bit beat


def _extract_read_data(read_resp, dut):
    if isinstance(read_resp, list) and read_resp:
        item = read_resp[0]
        if isinstance(item, dict):
            for key in ("data", "hrdata", "value"):
                if key in item:
                    v = item[key]
                    if isinstance(v, int):
                        return v
                    if isinstance(v, str):
                        return int(v, 0)
                    return int(v)

    return int(dut.hrdata.value)


# def _init_direct_ahb_signals(dut):
#     dut.haddr.value = 0
#     dut.hburst.value = AHB_BURST_SINGLE
#     dut.hmastlock.value = 0
#     dut.hprot.value = 0
#     dut.hsize.value = AHB_SIZE_8
#     dut.htrans.value = AHB_IDLE
#     dut.hwdata.value = 0
#     dut.hwrite.value = 0

def _init_direct_ahb_signals(dut):
    dut.haddr.value = 0
    dut.hburst.value = 0
    dut.hmastlock.value = 0
    dut.hprot.value = 0
    dut.hsize.value = 0
    dut.htrans.value = 0
    dut.hwdata.value = 0
    dut.hwrite.value = 0
    dut.hsel.value = 1
    dut.hreadyin.value = 1

async def setup_dut(dut, with_ahb_master=True):
    cocotb.start_soon(Clock(dut.clk, CLK_NS, units="ns").start())

    dut.resetn.setimmediatevalue(0)
    _init_direct_ahb_signals(dut)
    await ClockCycles(dut.clk, 5)
    dut.resetn.value = 1
    await ClockCycles(dut.clk, 2)

    ahb = None
    if with_ahb_master:
        ahb = AHBLiteMaster(
            AHBBus.from_entity(dut),
            dut.clk,
            dut.resetn,
            timeout=200,
            def_val=0,
        )

    axi_ram = AxiRam(
        AxiBus.from_prefix(dut, "m_axi"),
        dut.clk,
        dut.resetn,
        reset_active_level=False,
        size=RAM_SIZE,
    )

    await ClockCycles(dut.clk, 1)

    return ahb, axi_ram


def _size_bytes_to_axsize(size_bytes):
    return {1: 0, 2: 1, 4: 2, 8: 3}[size_bytes]


def _lane_shift(addr):
    return (addr & 0x7) * 8


def _mask_nbytes(size_bytes):
    return (1 << (8 * size_bytes)) - 1


def _strb_mask(addr, size_bytes):
    return ((1 << size_bytes) - 1) << (addr & 0x7)


def _apply_subword_to_qword(old_qword, addr, size_bytes, value):
    shift = _lane_shift(addr)
    field_mask = _mask_nbytes(size_bytes) << shift
    new_bits = (value & _mask_nbytes(size_bytes)) << shift
    return (old_qword & ~field_mask) | new_bits


async def wait_for_n_w_handshakes(dut, nbeats):
    beats = []
    while len(beats) < nbeats:
        await RisingEdge(dut.clk)
        if dut.m_axi_wvalid.value and dut.m_axi_wready.value:
            beats.append({
                "data": int(dut.m_axi_wdata.value),
                "strb": int(dut.m_axi_wstrb.value),
                "last": int(dut.m_axi_wlast.value),
            })
    return beats


async def wait_for_n_r_handshakes(dut, nbeats):
    beats = []
    while len(beats) < nbeats:
        await RisingEdge(dut.clk)
        if dut.m_axi_rvalid.value and dut.m_axi_rready.value:
            beats.append({
                "data": int(dut.m_axi_rdata.value),
                "resp": int(dut.m_axi_rresp.value),
                "last": int(dut.m_axi_rlast.value),
            })
    return beats


async def _wait_hready_high(dut):
    while True:
        await RisingEdge(dut.clk)
        if int(dut.hready.value):
            return

async def ahb_write_single_manual(dut, addr, value, size_bytes):
    assert size_bytes in (1, 2, 4, 8)

    hsize = _size_bytes_to_axsize(size_bytes)
    shifted_data = (value & _mask_nbytes(size_bytes)) << _lane_shift(addr)

    # address/control phase
    dut.haddr.value = addr
    dut.hburst.value = 0          # SINGLE
    dut.hmastlock.value = 0
    dut.hprot.value = 0
    dut.hsize.value = hsize
    dut.htrans.value = 0b10       # NONSEQ
    dut.hwrite.value = 1
    dut.hwdata.value = 0

    await _wait_hready_high(dut)

    # data phase for previous write; next transfer is IDLE
    dut.haddr.value = 0
    dut.hburst.value = 0
    dut.hmastlock.value = 0
    dut.hprot.value = 0
    dut.hsize.value = 0
    dut.htrans.value = 0          # IDLE
    dut.hwrite.value = 0
    dut.hwdata.value = shifted_data

    await _wait_hready_high(dut)
    _init_direct_ahb_signals(dut)

async def ahb_write_incr4_manual(dut, start_addr, data_beats, size_bytes=8):
    assert len(data_beats) == 4
    assert size_bytes in (1, 2, 4, 8)

    beat_stride = size_bytes
    hsize = _size_bytes_to_axsize(size_bytes)

    dut.haddr.value = start_addr
    dut.hburst.value = 0b011   # INCR4
    dut.hmastlock.value = 0
    dut.hprot.value = 0
    dut.hsize.value = hsize
    dut.htrans.value = 0b10    # NONSEQ
    dut.hwrite.value = 1
    dut.hwdata.value = 0

    for i in range(4):
        await _wait_hready_high(dut)

        if i == 3:
            dut.haddr.value = 0
            dut.hburst.value = 0
            dut.hsize.value = 0
            dut.htrans.value = 0   # IDLE
            dut.hwrite.value = 0
        else:
            dut.haddr.value = start_addr + (i + 1) * beat_stride
            dut.hburst.value = 0b011
            dut.hsize.value = hsize
            dut.htrans.value = 0b11   # SEQ
            dut.hwrite.value = 1

        dut.hwdata.value = data_beats[i]

    await _wait_hready_high(dut)
    _init_direct_ahb_signals(dut)


async def ahb_read_incr4_manual(dut, start_addr, size_bytes=8):
    assert size_bytes in (1, 2, 4, 8)

    beat_stride = size_bytes
    hsize = _size_bytes_to_axsize(size_bytes)
    results = []

    dut.haddr.value = start_addr
    dut.hburst.value = 0b011   # INCR4
    dut.hmastlock.value = 0
    dut.hprot.value = 0
    dut.hsize.value = hsize
    dut.htrans.value = 0b10    # NONSEQ
    dut.hwrite.value = 0
    dut.hwdata.value = 0

    for i in range(4):
        await _wait_hready_high(dut)

        if i > 0:
            results.append(int(dut.hrdata.value))

        if i == 3:
            dut.haddr.value = 0
            dut.hburst.value = 0
            dut.hsize.value = 0
            dut.htrans.value = 0   # IDLE
            dut.hwrite.value = 0
        else:
            dut.haddr.value = start_addr + (i + 1) * beat_stride
            dut.hburst.value = 0b011
            dut.hsize.value = hsize
            dut.htrans.value = 0b11   # SEQ
            dut.hwrite.value = 0

    await _wait_hready_high(dut)
    results.append(int(dut.hrdata.value))

    _init_direct_ahb_signals(dut)
    return results


async def wait_for_aw_handshake(dut):
    while True:
        await RisingEdge(dut.clk)
        if dut.m_axi_awvalid.value and dut.m_axi_awready.value:
            return {
                "addr":  int(dut.m_axi_awaddr.value),
                "len":   int(dut.m_axi_awlen.value),
                "size":  int(dut.m_axi_awsize.value),
                "burst": int(dut.m_axi_awburst.value),
                "lock":  int(dut.m_axi_awlock.value),
            }

async def wait_for_w_handshake(dut):
    while True:
        await RisingEdge(dut.clk)
        if dut.m_axi_wvalid.value and dut.m_axi_wready.value:
            return {
                "data": int(dut.m_axi_wdata.value),
                "strb": int(dut.m_axi_wstrb.value),
                "last": int(dut.m_axi_wlast.value),
            }


async def wait_for_n_w_handshakes(dut, count):
    beats = []
    while len(beats) < count:
        beats.append(await wait_for_w_handshake(dut))
    return beats


async def wait_for_ar_handshake(dut):
    while True:
        await RisingEdge(dut.clk)
        if dut.m_axi_arvalid.value and dut.m_axi_arready.value:
            return {
                "addr":  int(dut.m_axi_araddr.value),
                "len":   int(dut.m_axi_arlen.value),
                "size":  int(dut.m_axi_arsize.value),
                "burst": int(dut.m_axi_arburst.value),
                "lock":  int(dut.m_axi_arlock.value),
            }

async def drive_ahb_incr4_write_burst(dut, addr, data_words):
    assert len(data_words) == 4, "this helper currently drives exactly one INCR4 burst"
    assert (addr & 0x7) == 0, "burst base address must be 8-byte aligned"

    cycle_idx = 0

    while True:
        if cycle_idx == 0:
            dut.haddr.value = addr
            dut.hburst.value = AHB_BURST_INCR4
            dut.hmastlock.value = 0
            dut.hprot.value = 0
            dut.hsize.value = AHB_SIZE_8
            dut.htrans.value = AHB_NONSEQ
            dut.hwdata.value = 0
            dut.hwrite.value = 1
        elif cycle_idx == 1:
            dut.haddr.value = addr + 0x08
            dut.hburst.value = AHB_BURST_INCR4
            dut.hmastlock.value = 0
            dut.hprot.value = 0
            dut.hsize.value = AHB_SIZE_8
            dut.htrans.value = AHB_SEQ
            dut.hwdata.value = data_words[0]
            dut.hwrite.value = 1
        elif cycle_idx == 2:
            dut.haddr.value = addr + 0x10
            dut.hburst.value = AHB_BURST_INCR4
            dut.hmastlock.value = 0
            dut.hprot.value = 0
            dut.hsize.value = AHB_SIZE_8
            dut.htrans.value = AHB_SEQ
            dut.hwdata.value = data_words[1]
            dut.hwrite.value = 1
        elif cycle_idx == 3:
            dut.haddr.value = addr + 0x18
            dut.hburst.value = AHB_BURST_INCR4
            dut.hmastlock.value = 0
            dut.hprot.value = 0
            dut.hsize.value = AHB_SIZE_8
            dut.htrans.value = AHB_SEQ
            dut.hwdata.value = data_words[2]
            dut.hwrite.value = 1
        elif cycle_idx == 4:
            dut.haddr.value = 0
            dut.hburst.value = AHB_BURST_SINGLE
            dut.hmastlock.value = 0
            dut.hprot.value = 0
            dut.hsize.value = AHB_SIZE_8
            dut.htrans.value = AHB_IDLE
            dut.hwdata.value = data_words[3]
            dut.hwrite.value = 0
        else:
            break

        await RisingEdge(dut.clk)
        if int(dut.hready.value):
            cycle_idx += 1

    _init_direct_ahb_signals(dut)
    await ClockCycles(dut.clk, 2)


def pause_2_on_2_off():
    while True:
        yield 1
        yield 1
        yield 0
        yield 0


def set_axi_ram_backpressure(
    axi_ram,
    *,
    stall_aw=False,
    stall_w=False,
    stall_b=False,
    stall_ar=False,
    stall_r=False,
    generator=pause_2_on_2_off,
):
    if stall_aw:
        axi_ram.write_if.aw_channel.set_pause_generator(generator())
    if stall_w:
        axi_ram.write_if.w_channel.set_pause_generator(generator())
    if stall_b:
        axi_ram.write_if.b_channel.set_pause_generator(generator())
    if stall_ar:
        axi_ram.read_if.ar_channel.set_pause_generator(generator())
    if stall_r:
        axi_ram.read_if.r_channel.set_pause_generator(generator())


async def run_and_watch_hready(dut, coro):
    task = cocotb.start_soon(coro)
    hready_low_cycles = 0
    total_cycles = 0

    while not task.done():
        await RisingEdge(dut.clk)
        total_cycles += 1

        if int(dut.hready.value) == 0:
            hready_low_cycles += 1
            assert int(dut.hresp.value) == 0, (
                f"Expected HRESP=OKAY while HREADY=0, got {int(dut.hresp.value)}"
            )

    result = await task
    return result, hready_low_cycles, total_cycles


async def assert_no_axi_valids_for(dut, cycles):
    for _ in range(cycles):
        await RisingEdge(dut.clk)
        assert int(dut.m_axi_awvalid.value) == 0, "Unexpected AWVALID"
        assert int(dut.m_axi_wvalid.value) == 0, "Unexpected WVALID"
        assert int(dut.m_axi_arvalid.value) == 0, "Unexpected ARVALID"


async def ahb_control_only_cycle(
    dut,
    *,
    addr=0,
    hburst=0,
    hmastlock=0,
    hprot=0,
    size_bytes=8,
    htrans=0,
    hwrite=0,
):
    dut.haddr.value = addr
    dut.hburst.value = hburst
    dut.hmastlock.value = hmastlock
    dut.hprot.value = hprot
    dut.hsize.value = _size_bytes_to_axsize(size_bytes)
    dut.htrans.value = htrans
    dut.hwrite.value = hwrite
    dut.hwdata.value = 0

    await RisingEdge(dut.clk)

    _init_direct_ahb_signals(dut)
    await RisingEdge(dut.clk)



def _init_manual_axi_slave_inputs(dut):
    dut.m_axi_awready.value = 0
    dut.m_axi_wready.value = 0
    dut.m_axi_bid.value = 0
    dut.m_axi_bresp.value = 0
    dut.m_axi_bvalid.value = 0

    dut.m_axi_arready.value = 0
    dut.m_axi_rid.value = 0
    dut.m_axi_rdata.value = 0
    dut.m_axi_rresp.value = 0
    dut.m_axi_rlast.value = 0
    dut.m_axi_rvalid.value = 0


async def setup_dut_no_axi_slave(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_NS, units="ns").start())

    dut.resetn.setimmediatevalue(0)
    _init_direct_ahb_signals(dut)
    _init_manual_axi_slave_inputs(dut)

    dut.hsel.value = 1
    dut.hreadyin.value = 1

    await ClockCycles(dut.clk, 5)
    dut.resetn.value = 1
    await ClockCycles(dut.clk, 2)

    await ClockCycles(dut.clk, 1)


async def axi_slave_respond_single_write(dut, *, bresp=0, aw_stall=0, w_stall=0, b_delay=0):
    aw = {}
    w = {}

    dut.m_axi_awready.value = 0
    dut.m_axi_wready.value = 0
    dut.m_axi_bvalid.value = 0
    dut.m_axi_bresp.value = 0
    dut.m_axi_bid.value = 0

    while not int(dut.m_axi_awvalid.value):
        await RisingEdge(dut.clk)

    for _ in range(aw_stall):
        await RisingEdge(dut.clk)

    dut.m_axi_awready.value = 1
    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_awvalid.value) and int(dut.m_axi_awready.value):
            aw = {
                "addr": int(dut.m_axi_awaddr.value),
                "len": int(dut.m_axi_awlen.value),
                "size": int(dut.m_axi_awsize.value),
                "burst": int(dut.m_axi_awburst.value),
                "lock": int(dut.m_axi_awlock.value),
                "prot": int(dut.m_axi_awprot.value),
            }
            break
    dut.m_axi_awready.value = 0

    while not int(dut.m_axi_wvalid.value):
        await RisingEdge(dut.clk)

    for _ in range(w_stall):
        await RisingEdge(dut.clk)

    dut.m_axi_wready.value = 1
    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_wvalid.value) and int(dut.m_axi_wready.value):
            w = {
                "data": int(dut.m_axi_wdata.value),
                "strb": int(dut.m_axi_wstrb.value),
                "last": int(dut.m_axi_wlast.value),
            }
            break
    dut.m_axi_wready.value = 0

    for _ in range(b_delay):
        await RisingEdge(dut.clk)

    dut.m_axi_bid.value = 0
    dut.m_axi_bresp.value = bresp
    dut.m_axi_bvalid.value = 1

    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_bvalid.value) and int(dut.m_axi_bready.value):
            break

    dut.m_axi_bvalid.value = 0
    dut.m_axi_bresp.value = 0

    return {"aw": aw, "w": w}


async def axi_slave_respond_single_read(dut, *, rdata=0, rresp=0, ar_stall=0, r_delay=0):
    ar = {}

    dut.m_axi_arready.value = 0
    dut.m_axi_rvalid.value = 0
    dut.m_axi_rdata.value = 0
    dut.m_axi_rresp.value = 0
    dut.m_axi_rlast.value = 0
    dut.m_axi_rid.value = 0

    while not int(dut.m_axi_arvalid.value):
        await RisingEdge(dut.clk)

    for _ in range(ar_stall):
        await RisingEdge(dut.clk)

    dut.m_axi_arready.value = 1
    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_arvalid.value) and int(dut.m_axi_arready.value):
            ar = {
                "addr": int(dut.m_axi_araddr.value),
                "len": int(dut.m_axi_arlen.value),
                "size": int(dut.m_axi_arsize.value),
                "burst": int(dut.m_axi_arburst.value),
                "lock": int(dut.m_axi_arlock.value),
                "prot": int(dut.m_axi_arprot.value),
            }
            break
    dut.m_axi_arready.value = 0

    for _ in range(r_delay):
        await RisingEdge(dut.clk)

    dut.m_axi_rid.value = 0
    dut.m_axi_rdata.value = rdata
    dut.m_axi_rresp.value = rresp
    dut.m_axi_rlast.value = 1
    dut.m_axi_rvalid.value = 1

    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_rvalid.value) and int(dut.m_axi_rready.value):
            break

    dut.m_axi_rvalid.value = 0
    dut.m_axi_rresp.value = 0
    dut.m_axi_rlast.value = 0

    return {"ar": ar}


async def ahb_write_single_manual_status(dut, addr, value, size_bytes, *, hprot=0, hmastlock=0):
    assert size_bytes in (1, 2, 4, 8)

    hsize = _size_bytes_to_axsize(size_bytes)
    shifted_data = (value & _mask_nbytes(size_bytes)) << _lane_shift(addr)

    # AHB address/control phase
    dut.haddr.value = addr
    dut.hburst.value = 0
    dut.hmastlock.value = hmastlock
    dut.hprot.value = hprot
    dut.hsize.value = hsize
    dut.htrans.value = 0b10   # NONSEQ
    dut.hwrite.value = 1
    dut.hwdata.value = 0

    await _wait_hready_high(dut)

    # AHB write data phase; next transfer is IDLE
    dut.haddr.value = 0
    dut.hburst.value = 0
    dut.hmastlock.value = 0
    dut.hprot.value = 0
    dut.hsize.value = 0
    dut.htrans.value = 0      # IDLE
    dut.hwrite.value = 0
    dut.hwdata.value = shifted_data

    # Wait until the AXI write response handshake actually happens
    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_bvalid.value) and int(dut.m_axi_bready.value):
            break

    # After B handshake, allow the bridge to drive the final AHB response.
    # For ERROR, expect the 2-cycle AHB pattern:
    #   cycle 1: HRESP=1, HREADY=0
    #   cycle 2: HRESP=1, HREADY=1
    #
    # For OKAY, completion may simply remain HREADY=1 / HRESP=0.
    saw_hready_low = False
    for _ in range(16):
        await RisingEdge(dut.clk)

        if int(dut.hready.value) == 0:
            saw_hready_low = True

        if saw_hready_low:
            if int(dut.hready.value) == 1:
                resp = int(dut.hresp.value)
                _init_direct_ahb_signals(dut)
                await RisingEdge(dut.clk)
                return resp
        else:
            # No extra wait-state/error phase appeared; treat as normal completion
            if int(dut.hready.value) == 1:
                resp = int(dut.hresp.value)
                _init_direct_ahb_signals(dut)
                await RisingEdge(dut.clk)
                return resp

    raise AssertionError("Timed out waiting for final AHB write response after AXI B handshake")

async def ahb_read_single_manual_status(dut, addr, size_bytes, *, hprot=0, hmastlock=0):
    assert size_bytes in (1, 2, 4, 8)

    dut.haddr.value = addr
    dut.hburst.value = 0
    dut.hmastlock.value = hmastlock
    dut.hprot.value = hprot
    dut.hsize.value = _size_bytes_to_axsize(size_bytes)
    dut.htrans.value = 0b10
    dut.hwrite.value = 0
    dut.hwdata.value = 0

    await _wait_hready_high(dut)

    dut.haddr.value = 0
    dut.hburst.value = 0
    dut.hmastlock.value = 0
    dut.hprot.value = 0
    dut.hsize.value = 0
    dut.htrans.value = 0
    dut.hwrite.value = 0
    dut.hwdata.value = 0

    while True:
        await RisingEdge(dut.clk)
        if int(dut.hready.value):
            data = int(dut.hrdata.value)
            resp = int(dut.hresp.value)
            break

    _init_direct_ahb_signals(dut)
    await RisingEdge(dut.clk)
    return data, resp


async def axi_slave_respond_incr4_write_burst(dut, *, bresp=0, aw_stall=0, w_stall_each=0, b_delay=0):
    aw = {}
    w_beats = []

    dut.m_axi_awready.value = 0
    dut.m_axi_wready.value = 0
    dut.m_axi_bvalid.value = 0
    dut.m_axi_bresp.value = 0
    dut.m_axi_bid.value = 0

    while not int(dut.m_axi_awvalid.value):
        await RisingEdge(dut.clk)

    for _ in range(aw_stall):
        await RisingEdge(dut.clk)

    dut.m_axi_awready.value = 1
    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_awvalid.value) and int(dut.m_axi_awready.value):
            aw = {
                "addr": int(dut.m_axi_awaddr.value),
                "len": int(dut.m_axi_awlen.value),
                "size": int(dut.m_axi_awsize.value),
                "burst": int(dut.m_axi_awburst.value),
            }
            break
    dut.m_axi_awready.value = 0

    for i in range(4):
        while not int(dut.m_axi_wvalid.value):
            await RisingEdge(dut.clk)

        for _ in range(w_stall_each):
            await RisingEdge(dut.clk)

        dut.m_axi_wready.value = 1
        while True:
            await RisingEdge(dut.clk)
            if int(dut.m_axi_wvalid.value) and int(dut.m_axi_wready.value):
                w_beats.append({
                    "data": int(dut.m_axi_wdata.value),
                    "strb": int(dut.m_axi_wstrb.value),
                    "last": int(dut.m_axi_wlast.value),
                })
                break
        dut.m_axi_wready.value = 0

    for _ in range(b_delay):
        await RisingEdge(dut.clk)

    dut.m_axi_bid.value = 0
    dut.m_axi_bresp.value = bresp
    dut.m_axi_bvalid.value = 1

    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_bvalid.value) and int(dut.m_axi_bready.value):
            break

    dut.m_axi_bvalid.value = 0
    dut.m_axi_bresp.value = 0

    return {"aw": aw, "w_beats": w_beats}


async def axi_slave_respond_incr4_read_burst(dut, beats, *, error_index=None, error_resp=0b10, ar_stall=0, r_gap=0):
    assert len(beats) == 4
    ar = {}

    dut.m_axi_arready.value = 0
    dut.m_axi_rvalid.value = 0
    dut.m_axi_rdata.value = 0
    dut.m_axi_rresp.value = 0
    dut.m_axi_rlast.value = 0
    dut.m_axi_rid.value = 0

    while not int(dut.m_axi_arvalid.value):
        await RisingEdge(dut.clk)

    for _ in range(ar_stall):
        await RisingEdge(dut.clk)

    dut.m_axi_arready.value = 1
    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_arvalid.value) and int(dut.m_axi_arready.value):
            ar = {
                "addr": int(dut.m_axi_araddr.value),
                "len": int(dut.m_axi_arlen.value),
                "size": int(dut.m_axi_arsize.value),
                "burst": int(dut.m_axi_arburst.value),
            }
            break
    dut.m_axi_arready.value = 0

    for i, value in enumerate(beats):
        for _ in range(r_gap):
            await RisingEdge(dut.clk)

        dut.m_axi_rid.value = 0
        dut.m_axi_rdata.value = value
        dut.m_axi_rresp.value = error_resp if i == error_index else 0
        dut.m_axi_rlast.value = 1 if i == 3 else 0
        dut.m_axi_rvalid.value = 1

        while True:
            await RisingEdge(dut.clk)
            if int(dut.m_axi_rvalid.value) and int(dut.m_axi_rready.value):
                break

        dut.m_axi_rvalid.value = 0
        dut.m_axi_rresp.value = 0
        dut.m_axi_rlast.value = 0

    return {"ar": ar}


async def ahb_write_incr4_manual_status(dut, start_addr, data_beats, size_bytes=8):
    assert len(data_beats) == 4
    assert size_bytes in (1, 2, 4, 8)

    beat_stride = size_bytes
    hsize = _size_bytes_to_axsize(size_bytes)

    dut.haddr.value = start_addr
    dut.hburst.value = 0b011   # INCR4
    dut.hmastlock.value = 0
    dut.hprot.value = 0
    dut.hsize.value = hsize
    dut.htrans.value = 0b10    # NONSEQ
    dut.hwrite.value = 1
    dut.hwdata.value = 0

    hresp_seen = []

    for i in range(4):
        await _wait_hready_high(dut)
        hresp_seen.append(int(dut.hresp.value))

        if i == 3:
            dut.haddr.value = 0
            dut.hburst.value = 0
            dut.hsize.value = 0
            dut.htrans.value = 0
            dut.hwrite.value = 0
        else:
            dut.haddr.value = start_addr + (i + 1) * beat_stride
            dut.hburst.value = 0b011
            dut.hsize.value = hsize
            dut.htrans.value = 0b11
            dut.hwrite.value = 1

        dut.hwdata.value = data_beats[i]

    # Wait for AXI B handshake
    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_bvalid.value) and int(dut.m_axi_bready.value):
            break

    # Capture final AHB completion after B
    final_resp = None
    for _ in range(16):
        await RisingEdge(dut.clk)
        if int(dut.hresp.value):
            final_resp = int(dut.hresp.value)
        if int(dut.hready.value):
            if final_resp is None:
                final_resp = int(dut.hresp.value)
            break

    _init_direct_ahb_signals(dut)
    await RisingEdge(dut.clk)

    return {"beat_resps": hresp_seen, "final_resp": final_resp}


async def ahb_read_incr4_manual_status(dut, start_addr, size_bytes=8):
    assert size_bytes in (1, 2, 4, 8)

    beat_stride = size_bytes
    hsize = _size_bytes_to_axsize(size_bytes)
    samples = []

    dut.haddr.value = start_addr
    dut.hburst.value = 0b011
    dut.hmastlock.value = 0
    dut.hprot.value = 0
    dut.hsize.value = hsize
    dut.htrans.value = 0b10
    dut.hwrite.value = 0
    dut.hwdata.value = 0

    for i in range(4):
        await _wait_hready_high(dut)

        if i > 0:
            samples.append({
                "data": int(dut.hrdata.value),
                "resp": int(dut.hresp.value),
            })

        if i == 3:
            dut.haddr.value = 0
            dut.hburst.value = 0
            dut.hsize.value = 0
            dut.htrans.value = 0
            dut.hwrite.value = 0
        else:
            dut.haddr.value = start_addr + (i + 1) * beat_stride
            dut.hburst.value = 0b011
            dut.hsize.value = hsize
            dut.htrans.value = 0b11
            dut.hwrite.value = 0

    await _wait_hready_high(dut)
    samples.append({
        "data": int(dut.hrdata.value),
        "resp": int(dut.hresp.value),
    })

    _init_direct_ahb_signals(dut)
    await RisingEdge(dut.clk)
    return samples


async def wait_until_all_axi_outputs_idle(dut, max_cycles=20):
    for _ in range(max_cycles):
        await RisingEdge(dut.clk)
        if (
            int(dut.m_axi_awvalid.value) == 0 and
            int(dut.m_axi_wvalid.value) == 0 and
            int(dut.m_axi_arvalid.value) == 0
        ):
            return
    raise AssertionError("AXI outputs did not return to idle")


def _hburst_incrementing_code(nbeats, fixed=True):
    if not fixed:
        return 0b001   # INCR (undefined length)

    return {
        1: 0b000,      # SINGLE
        4: 0b011,      # INCR4
        8: 0b101,      # INCR8
        16: 0b111,     # INCR16
    }[nbeats]


async def ahb_write_inc_burst_manual(dut, start_addr, data_beats, size_bytes=8, fixed=True):
    nbeats = len(data_beats)
    assert size_bytes in (1, 2, 4, 8)

    beat_stride = size_bytes
    hsize = _size_bytes_to_axsize(size_bytes)
    hburst = _hburst_incrementing_code(nbeats, fixed=fixed)

    dut.haddr.value = start_addr
    dut.hburst.value = hburst
    dut.hmastlock.value = 0
    dut.hprot.value = 0
    dut.hsize.value = hsize
    dut.htrans.value = 0b10    # NONSEQ
    dut.hwrite.value = 1
    dut.hwdata.value = 0

    for i in range(nbeats):
        await _wait_hready_high(dut)

        if i == nbeats - 1:
            dut.haddr.value = 0
            dut.hburst.value = 0
            dut.hsize.value = 0
            dut.htrans.value = 0   # IDLE
            dut.hwrite.value = 0
        else:
            dut.haddr.value = start_addr + (i + 1) * beat_stride
            dut.hburst.value = hburst
            dut.hsize.value = hsize
            dut.htrans.value = 0b11   # SEQ
            dut.hwrite.value = 1

        #dut.hwdata.value = data_beats[i]
        addr_i = start_addr + i * beat_stride
        dut.hwdata.value = (data_beats[i] & _mask_nbytes(size_bytes)) << _lane_shift(addr_i)        

    await _wait_hready_high(dut)
    _init_direct_ahb_signals(dut)


async def ahb_read_inc_burst_manual(dut, start_addr, nbeats, size_bytes=8, fixed=True):
    assert size_bytes in (1, 2, 4, 8)

    beat_stride = size_bytes
    hsize = _size_bytes_to_axsize(size_bytes)
    hburst = _hburst_incrementing_code(nbeats, fixed=fixed)
    results = []

    dut.haddr.value = start_addr
    dut.hburst.value = hburst
    dut.hmastlock.value = 0
    dut.hprot.value = 0
    dut.hsize.value = hsize
    dut.htrans.value = 0b10    # NONSEQ
    dut.hwrite.value = 0
    dut.hwdata.value = 0

    for i in range(nbeats):
        await _wait_hready_high(dut)

        if i > 0:
            results.append(int(dut.hrdata.value))

        if i == nbeats - 1:
            dut.haddr.value = 0
            dut.hburst.value = 0
            dut.hsize.value = 0
            dut.htrans.value = 0   # IDLE
            dut.hwrite.value = 0
        else:
            dut.haddr.value = start_addr + (i + 1) * beat_stride
            dut.hburst.value = hburst
            dut.hsize.value = hsize
            dut.htrans.value = 0b11   # SEQ
            dut.hwrite.value = 0

    await _wait_hready_high(dut)
    results.append(int(dut.hrdata.value))

    _init_direct_ahb_signals(dut)
    return results


async def ahb_write_incr_burst_with_busy_manual(
    dut,
    start_addr,
    data_beats,
    *,
    size_bytes=8,
    busy_after_beats=(1,),
):
    nbeats = len(data_beats)
    assert size_bytes in (1, 2, 4, 8)

    beat_stride = size_bytes
    hsize = _size_bytes_to_axsize(size_bytes)
    hburst = 0b001   # INCR
    busy_after_beats = set(busy_after_beats)

    def drive_beat(idx):
        dut.haddr.value = start_addr + idx * beat_stride
        dut.hburst.value = hburst
        dut.hmastlock.value = 0
        dut.hprot.value = 0
        dut.hsize.value = hsize
        dut.htrans.value = 0b10 if idx == 0 else 0b11   # NONSEQ then SEQ
        dut.hwrite.value = 1

    def drive_busy(next_idx):
        dut.haddr.value = start_addr + next_idx * beat_stride
        dut.hburst.value = hburst
        dut.hmastlock.value = 0
        dut.hprot.value = 0
        dut.hsize.value = hsize
        dut.htrans.value = 0b01   # BUSY
        dut.hwrite.value = 1

    drive_beat(0)
    phase_kind = "beat"
    phase_idx = 0

    while True:
        await _wait_hready_high(dut)

        if phase_kind == "beat":
            #dut.hwdata.value = data_beats[phase_idx]
            addr_i = start_addr + phase_idx * beat_stride
            dut.hwdata.value = (data_beats[phase_idx] & _mask_nbytes(size_bytes)) << _lane_shift(addr_i)

            if phase_idx == nbeats - 1:
                dut.haddr.value = 0
                dut.hburst.value = 0
                dut.hsize.value = 0
                dut.htrans.value = 0
                dut.hwrite.value = 0
                break

            if phase_idx in busy_after_beats:
                drive_busy(phase_idx + 1)
                phase_kind = "busy"
                phase_idx = phase_idx + 1
            else:
                drive_beat(phase_idx + 1)
                phase_kind = "beat"
                phase_idx = phase_idx + 1

        else:
            # BUSY cycle has no write data beat of its own
            dut.hwdata.value = 0
            drive_beat(phase_idx)
            phase_kind = "beat"

    await _wait_hready_high(dut)
    _init_direct_ahb_signals(dut)


async def ahb_read_incr_burst_with_busy_manual(
    dut,
    start_addr,
    nbeats,
    *,
    size_bytes=8,
    busy_after_beats=(1,),
):
    assert size_bytes in (1, 2, 4, 8)

    beat_stride = size_bytes
    hsize = _size_bytes_to_axsize(size_bytes)
    hburst = 0b001   # INCR
    busy_after_beats = set(busy_after_beats)
    results = []

    def drive_beat(idx):
        dut.haddr.value = start_addr + idx * beat_stride
        dut.hburst.value = hburst
        dut.hmastlock.value = 0
        dut.hprot.value = 0
        dut.hsize.value = hsize
        dut.htrans.value = 0b10 if idx == 0 else 0b11   # NONSEQ then SEQ
        dut.hwrite.value = 0

    def drive_busy(next_idx):
        dut.haddr.value = start_addr + next_idx * beat_stride
        dut.hburst.value = hburst
        dut.hmastlock.value = 0
        dut.hprot.value = 0
        dut.hsize.value = hsize
        dut.htrans.value = 0b01   # BUSY
        dut.hwrite.value = 0

    drive_beat(0)
    phase_kind = "beat"
    phase_idx = 0
    pending_data = None

    while True:
        await _wait_hready_high(dut)

        if pending_data is not None:
            results.append(int(dut.hrdata.value))
            pending_data = None

        if phase_kind == "beat":
            pending_data = phase_idx

            if phase_idx == nbeats - 1:
                dut.haddr.value = 0
                dut.hburst.value = 0
                dut.hsize.value = 0
                dut.htrans.value = 0
                dut.hwrite.value = 0
                break

            if phase_idx in busy_after_beats:
                drive_busy(phase_idx + 1)
                phase_kind = "busy"
                phase_idx = phase_idx + 1
            else:
                drive_beat(phase_idx + 1)
                phase_kind = "beat"
                phase_idx = phase_idx + 1

        else:
            drive_beat(phase_idx)
            phase_kind = "beat"

    await _wait_hready_high(dut)
    if pending_data is not None:
        results.append(int(dut.hrdata.value))

    _init_direct_ahb_signals(dut)
    return results

def _qword_addr(addr):
    return addr & ~0x7


def _extract_subword(raw, addr, size_bytes):
    return (raw >> _lane_shift(addr)) & _mask_nbytes(size_bytes)


def _expected_burst_qwords(initial_qwords, start_addr, beats, size_bytes):
    expected = dict(initial_qwords)

    for i, value in enumerate(beats):
        addr = start_addr + i * size_bytes
        qaddr = _qword_addr(addr)
        old_qword = expected.get(qaddr, 0)
        expected[qaddr] = _apply_subword_to_qword(old_qword, addr, size_bytes, value)

    return expected


def _expected_wrap_qwords(initial_qwords, start_addr, beats, size_bytes, nbeats):
    expected = dict(initial_qwords)

    for i, value in enumerate(beats):
        addr = _wrap_addr(start_addr, i, size_bytes, nbeats)
        qaddr = _qword_addr(addr)
        old_qword = expected.get(qaddr, 0)
        expected[qaddr] = _apply_subword_to_qword(old_qword, addr, size_bytes, value)

    return expected


def _expected_wrap_subwords(initial_qwords, start_addr, size_bytes, nbeats):
    expected = []
    for i in range(nbeats):
        addr = _wrap_addr(start_addr, i, size_bytes, nbeats)
        qaddr = _qword_addr(addr)
        expected.append(_extract_subword(initial_qwords[qaddr], addr, size_bytes))
    return expected


async def ahb_start_partial_incr_write(dut, start_addr, data_beats, *, beats_to_issue=2, size_bytes=8):
    assert size_bytes in (1, 2, 4, 8)
    assert 1 <= beats_to_issue <= len(data_beats)

    beat_stride = size_bytes
    hsize = _size_bytes_to_axsize(size_bytes)
    hburst = 0b001   # INCR

    dut.haddr.value = start_addr
    dut.hburst.value = hburst
    dut.hmastlock.value = 0
    dut.hprot.value = 0
    dut.hsize.value = hsize
    dut.htrans.value = 0b10   # NONSEQ
    dut.hwrite.value = 1
    dut.hwdata.value = 0

    for i in range(beats_to_issue):
        await _wait_hready_high(dut)

        addr_i = start_addr + i * beat_stride
        dut.hwdata.value = (data_beats[i] & _mask_nbytes(size_bytes)) << _lane_shift(addr_i)

        next_addr = start_addr + (i + 1) * beat_stride
        dut.haddr.value = next_addr
        dut.hburst.value = hburst
        dut.hsize.value = hsize
        dut.htrans.value = 0b11   # SEQ
        dut.hwrite.value = 1


async def ahb_start_partial_incr_read(dut, start_addr, *, beats_to_issue=2, size_bytes=8):
    assert size_bytes in (1, 2, 4, 8)
    assert beats_to_issue >= 1

    beat_stride = size_bytes
    hsize = _size_bytes_to_axsize(size_bytes)
    hburst = 0b001   # INCR

    dut.haddr.value = start_addr
    dut.hburst.value = hburst
    dut.hmastlock.value = 0
    dut.hprot.value = 0
    dut.hsize.value = hsize
    dut.htrans.value = 0b10   # NONSEQ
    dut.hwrite.value = 0
    dut.hwdata.value = 0

    for i in range(beats_to_issue):
        await _wait_hready_high(dut)

        next_addr = start_addr + (i + 1) * beat_stride
        dut.haddr.value = next_addr
        dut.hburst.value = hburst
        dut.hsize.value = hsize
        dut.htrans.value = 0b11   # SEQ
        dut.hwrite.value = 0

async def ahb_write_wrap_burst_manual_with_attrs(
    dut, start_addr, data_beats, *, size_bytes=8, hmastlock=0, hprot=0
):
    nbeats = len(data_beats)
    assert nbeats in (4, 8, 16)
    assert size_bytes in (1, 2, 4, 8)

    hsize = _size_bytes_to_axsize(size_bytes)
    hburst = _hburst_wrap_code(nbeats)

    first_addr = _wrap_addr(start_addr, 0, size_bytes, nbeats)
    dut.haddr.value = first_addr
    dut.hburst.value = hburst
    dut.hmastlock.value = hmastlock
    dut.hprot.value = hprot
    dut.hsize.value = hsize
    dut.htrans.value = 0b10
    dut.hwrite.value = 1
    dut.hwdata.value = 0

    for i in range(nbeats):
        await _wait_hready_high(dut)

        addr_i = _wrap_addr(start_addr, i, size_bytes, nbeats)
        dut.hwdata.value = (data_beats[i] & _mask_nbytes(size_bytes)) << _lane_shift(addr_i)

        if i == nbeats - 1:
            dut.haddr.value = 0
            dut.hburst.value = 0
            dut.hmastlock.value = 0
            dut.hprot.value = 0
            dut.hsize.value = 0
            dut.htrans.value = 0
            dut.hwrite.value = 0
        else:
            next_addr = _wrap_addr(start_addr, i + 1, size_bytes, nbeats)
            dut.haddr.value = next_addr
            dut.hburst.value = hburst
            dut.hmastlock.value = hmastlock
            dut.hprot.value = hprot
            dut.hsize.value = hsize
            dut.htrans.value = 0b11
            dut.hwrite.value = 1

    await _wait_hready_high(dut)
    _init_direct_ahb_signals(dut)


async def ahb_read_wrap_burst_manual_with_attrs(
    dut, start_addr, nbeats, *, size_bytes=8, hmastlock=0, hprot=0
):
    assert nbeats in (4, 8, 16)
    assert size_bytes in (1, 2, 4, 8)

    hsize = _size_bytes_to_axsize(size_bytes)
    hburst = _hburst_wrap_code(nbeats)
    results = []

    first_addr = _wrap_addr(start_addr, 0, size_bytes, nbeats)
    dut.haddr.value = first_addr
    dut.hburst.value = hburst
    dut.hmastlock.value = hmastlock
    dut.hprot.value = hprot
    dut.hsize.value = hsize
    dut.htrans.value = 0b10
    dut.hwrite.value = 0
    dut.hwdata.value = 0

    for i in range(nbeats):
        await _wait_hready_high(dut)

        if i > 0:
            results.append(int(dut.hrdata.value))

        if i == nbeats - 1:
            dut.haddr.value = 0
            dut.hburst.value = 0
            dut.hmastlock.value = 0
            dut.hprot.value = 0
            dut.hsize.value = 0
            dut.htrans.value = 0
            dut.hwrite.value = 0
        else:
            next_addr = _wrap_addr(start_addr, i + 1, size_bytes, nbeats)
            dut.haddr.value = next_addr
            dut.hburst.value = hburst
            dut.hmastlock.value = hmastlock
            dut.hprot.value = hprot
            dut.hsize.value = hsize
            dut.htrans.value = 0b11
            dut.hwrite.value = 0

    await _wait_hready_high(dut)
    results.append(int(dut.hrdata.value))

    _init_direct_ahb_signals(dut)
    return results


async def wait_until_any_axi_valid(dut, max_cycles=20):
    for _ in range(max_cycles):
        await RisingEdge(dut.clk)
        if (
            int(dut.m_axi_awvalid.value)
            or int(dut.m_axi_wvalid.value)
            or int(dut.m_axi_arvalid.value)
        ):
            return
    raise AssertionError("No AXI VALID observed")


async def ahb_write_wrap_burst_manual_status(dut, start_addr, data_beats, size_bytes=8):
    nbeats = len(data_beats)
    assert nbeats in (4, 8, 16)
    assert size_bytes in (1, 2, 4, 8)

    hsize = _size_bytes_to_axsize(size_bytes)
    hburst = _hburst_wrap_code(nbeats)

    first_addr = _wrap_addr(start_addr, 0, size_bytes, nbeats)
    dut.haddr.value = first_addr
    dut.hburst.value = hburst
    dut.hmastlock.value = 0
    dut.hprot.value = 0
    dut.hsize.value = hsize
    dut.htrans.value = 0b10
    dut.hwrite.value = 1
    dut.hwdata.value = 0

    for i in range(nbeats):
        await _wait_hready_high(dut)

        addr_i = _wrap_addr(start_addr, i, size_bytes, nbeats)
        dut.hwdata.value = (data_beats[i] & _mask_nbytes(size_bytes)) << _lane_shift(addr_i)

        if i == nbeats - 1:
            dut.haddr.value = 0
            dut.hburst.value = 0
            dut.hsize.value = 0
            dut.htrans.value = 0
            dut.hwrite.value = 0
        else:
            next_addr = _wrap_addr(start_addr, i + 1, size_bytes, nbeats)
            dut.haddr.value = next_addr
            dut.hburst.value = hburst
            dut.hsize.value = hsize
            dut.htrans.value = 0b11
            dut.hwrite.value = 1

    # wait for AXI B handshake
    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_bvalid.value) and int(dut.m_axi_bready.value):
            break

    final_resp = None
    for _ in range(16):
        await RisingEdge(dut.clk)
        if int(dut.hresp.value):
            final_resp = int(dut.hresp.value)
        if int(dut.hready.value):
            if final_resp is None:
                final_resp = int(dut.hresp.value)
            break

    _init_direct_ahb_signals(dut)
    await RisingEdge(dut.clk)

    return final_resp

async def axi_slave_respond_write_burst(dut, nbeats, *, bresp=0, aw_stall=0, w_stall_each=0, b_delay=0):
    aw = {}
    w_beats = []

    dut.m_axi_awready.value = 0
    dut.m_axi_wready.value = 0
    dut.m_axi_bvalid.value = 0
    dut.m_axi_bresp.value = 0
    dut.m_axi_bid.value = 0

    while not int(dut.m_axi_awvalid.value):
        await RisingEdge(dut.clk)

    for _ in range(aw_stall):
        await RisingEdge(dut.clk)

    dut.m_axi_awready.value = 1
    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_awvalid.value) and int(dut.m_axi_awready.value):
            aw = {
                "addr": int(dut.m_axi_awaddr.value),
                "len": int(dut.m_axi_awlen.value),
                "size": int(dut.m_axi_awsize.value),
                "burst": int(dut.m_axi_awburst.value),
                "lock": int(dut.m_axi_awlock.value),
                "prot": int(dut.m_axi_awprot.value),
            }
            break
    dut.m_axi_awready.value = 0

    for _ in range(nbeats):
        while not int(dut.m_axi_wvalid.value):
            await RisingEdge(dut.clk)

        for _ in range(w_stall_each):
            await RisingEdge(dut.clk)

        dut.m_axi_wready.value = 1
        while True:
            await RisingEdge(dut.clk)
            if int(dut.m_axi_wvalid.value) and int(dut.m_axi_wready.value):
                w_beats.append({
                    "data": int(dut.m_axi_wdata.value),
                    "strb": int(dut.m_axi_wstrb.value),
                    "last": int(dut.m_axi_wlast.value),
                })
                break
        dut.m_axi_wready.value = 0

    for _ in range(b_delay):
        await RisingEdge(dut.clk)

    dut.m_axi_bid.value = 0
    dut.m_axi_bresp.value = bresp
    dut.m_axi_bvalid.value = 1

    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_bvalid.value) and int(dut.m_axi_bready.value):
            break

    dut.m_axi_bvalid.value = 0
    dut.m_axi_bresp.value = 0

    return {"aw": aw, "w_beats": w_beats}


async def axi_slave_respond_read_burst(dut, beats, *, error_index=None, error_resp=0b10, ar_stall=0, r_gap=0):
    ar = {}

    dut.m_axi_arready.value = 0
    dut.m_axi_rvalid.value = 0
    dut.m_axi_rdata.value = 0
    dut.m_axi_rresp.value = 0
    dut.m_axi_rlast.value = 0
    dut.m_axi_rid.value = 0

    while not int(dut.m_axi_arvalid.value):
        await RisingEdge(dut.clk)

    for _ in range(ar_stall):
        await RisingEdge(dut.clk)

    dut.m_axi_arready.value = 1
    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_arvalid.value) and int(dut.m_axi_arready.value):
            ar = {
                "addr": int(dut.m_axi_araddr.value),
                "len": int(dut.m_axi_arlen.value),
                "size": int(dut.m_axi_arsize.value),
                "burst": int(dut.m_axi_arburst.value),
                "lock": int(dut.m_axi_arlock.value),
                "prot": int(dut.m_axi_arprot.value),
            }
            break
    dut.m_axi_arready.value = 0

    for i, value in enumerate(beats):
        for _ in range(r_gap):
            await RisingEdge(dut.clk)

        dut.m_axi_rid.value = 0
        dut.m_axi_rdata.value = value
        dut.m_axi_rresp.value = error_resp if i == error_index else 0
        dut.m_axi_rlast.value = 1 if i == len(beats) - 1 else 0
        dut.m_axi_rvalid.value = 1

        while True:
            await RisingEdge(dut.clk)
            if int(dut.m_axi_rvalid.value) and int(dut.m_axi_rready.value):
                break

        dut.m_axi_rvalid.value = 0
        dut.m_axi_rresp.value = 0
        dut.m_axi_rlast.value = 0

    return {"ar": ar}


def _sparse_apply_wbeat(mem, addr, wdata, wstrb):
    base = addr & ~0x7
    old = mem.get(base, 0)
    new = old

    for b in range(8):
        if (wstrb >> b) & 1:
            byte_val = (wdata >> (8 * b)) & 0xFF
            mask = 0xFF << (8 * b)
            new = (new & ~mask) | (byte_val << (8 * b))

    mem[base] = new


async def axi_sparse_mem_write_slave(dut, mem):
    dut.m_axi_awready.value = 0
    dut.m_axi_wready.value = 0
    dut.m_axi_bvalid.value = 0
    dut.m_axi_bresp.value = 0
    dut.m_axi_bid.value = 0

    while True:
        while not int(dut.m_axi_awvalid.value):
            await RisingEdge(dut.clk)

        dut.m_axi_awready.value = 1
        while True:
            await RisingEdge(dut.clk)
            if int(dut.m_axi_awvalid.value) and int(dut.m_axi_awready.value):
                awaddr = int(dut.m_axi_awaddr.value)
                awlen = int(dut.m_axi_awlen.value)
                awsize = int(dut.m_axi_awsize.value)
                awburst = int(dut.m_axi_awburst.value)
                break
        dut.m_axi_awready.value = 0

        for beat in range(awlen + 1):
            while not int(dut.m_axi_wvalid.value):
                await RisingEdge(dut.clk)

            dut.m_axi_wready.value = 1
            while True:
                await RisingEdge(dut.clk)
                if int(dut.m_axi_wvalid.value) and int(dut.m_axi_wready.value):
                    wdata = int(dut.m_axi_wdata.value)
                    wstrb = int(dut.m_axi_wstrb.value)
                    wlast = int(dut.m_axi_wlast.value)
                    break
            dut.m_axi_wready.value = 0

            if awburst == 1:      # INCR
                addr = awaddr + beat * (1 << awsize)
            else:                 # SINGLE/FIXED fallback
                addr = awaddr

            _sparse_apply_wbeat(mem, addr, wdata, wstrb)

            if beat == awlen:
                assert wlast == 1, f"Expected WLAST on final beat, got {wlast}"
            else:
                assert wlast == 0, f"Unexpected early WLAST on beat {beat}"

        dut.m_axi_bvalid.value = 1
        dut.m_axi_bresp.value = 0
        while True:
            await RisingEdge(dut.clk)
            if int(dut.m_axi_bvalid.value) and int(dut.m_axi_bready.value):
                break
        dut.m_axi_bvalid.value = 0
        dut.m_axi_bresp.value = 0


async def axi_sparse_mem_read_slave(dut, mem):
    dut.m_axi_arready.value = 0
    dut.m_axi_rvalid.value = 0
    dut.m_axi_rdata.value = 0
    dut.m_axi_rresp.value = 0
    dut.m_axi_rlast.value = 0
    dut.m_axi_rid.value = 0

    while True:
        while not int(dut.m_axi_arvalid.value):
            await RisingEdge(dut.clk)

        dut.m_axi_arready.value = 1
        while True:
            await RisingEdge(dut.clk)
            if int(dut.m_axi_arvalid.value) and int(dut.m_axi_arready.value):
                araddr = int(dut.m_axi_araddr.value)
                arlen = int(dut.m_axi_arlen.value)
                arsize = int(dut.m_axi_arsize.value)
                arburst = int(dut.m_axi_arburst.value)
                break
        dut.m_axi_arready.value = 0

        for beat in range(arlen + 1):
            if arburst == 1:      # INCR
                addr = araddr + beat * (1 << arsize)
            else:                 # SINGLE/FIXED fallback
                addr = araddr

            base = addr & ~0x7
            rdata = mem.get(base, 0)

            dut.m_axi_rdata.value = rdata
            dut.m_axi_rresp.value = 0
            dut.m_axi_rlast.value = 1 if beat == arlen else 0
            dut.m_axi_rvalid.value = 1

            while True:
                await RisingEdge(dut.clk)
                if int(dut.m_axi_rvalid.value) and int(dut.m_axi_rready.value):
                    break

            dut.m_axi_rvalid.value = 0
            dut.m_axi_rlast.value = 0

# async def _ahb_write_incr8_then_read_incr8_no_idle_gap_bug3(dut, write_addr, write_beats, read_addr):
#     """
#     Legal single-driver AHB sequence for BUG3:
#       - drive INCR8 write
#       - on the cycle after the last accepted write beat, present read NONSEQ
#         with no IDLE gap
#       - then complete the INCR8 read and return the 8 read beats

#     Dedicated to BUG3 only. Does not modify shared helpers.
#     """
#     assert len(write_beats) == 8

#     hburst = 0b101   # INCR8
#     hsize  = 3       # 64-bit
#     results = []

#     # Start write burst
#     dut.haddr.value      = write_addr
#     dut.hburst.value     = hburst
#     dut.hmastlock.value  = 0
#     dut.hprot.value      = 0
#     dut.hsize.value      = hsize
#     dut.htrans.value     = 0b10   # NONSEQ
#     dut.hwrite.value     = 1
#     dut.hwdata.value     = 0

#     # Write phase: same style as existing manual helper, except that instead of
#     # going IDLE after the last beat we launch the read NONSEQ immediately.
#     for i in range(8):
#         await _wait_hready(dut)

#         if i == 7:
#             # Last write beat's data phase is completing now.
#             # Present next transaction immediately: read NONSEQ, no idle gap.
#             dut.haddr.value      = read_addr
#             dut.hburst.value     = hburst
#             dut.hmastlock.value  = 0
#             dut.hprot.value      = 0
#             dut.hsize.value      = hsize
#             dut.htrans.value     = 0b10   # NONSEQ
#             dut.hwrite.value     = 0
#         else:
#             dut.haddr.value  = write_addr + (i + 1) * 8
#             dut.htrans.value = 0b11       # SEQ
#             dut.hwrite.value = 1

#         dut.hwdata.value = write_beats[i]

#     # Read phase
#     for i in range(8):
#         await _wait_hready(dut)
#         await ReadOnly()

#         if i > 0:
#             results.append(int(dut.hrdata.value))

#         await NextTimeStep()

#         if i == 7:
#             dut.haddr.value  = 0
#             dut.hburst.value = 0
#             dut.hsize.value  = 0
#             dut.htrans.value = 0   # IDLE
#             dut.hwrite.value = 0
#             dut.hwdata.value = 0
#         else:
#             dut.haddr.value  = read_addr + (i + 1) * 8
#             dut.htrans.value = 0b11   # SEQ
#             dut.hwrite.value = 0

#     await _wait_hready(dut)
#     await ReadOnly()
#     results.append(int(dut.hrdata.value))
#     await NextTimeStep()

#     _init_direct_ahb_signals(dut)
#     return results

async def _ahb_write_incr8_then_read_incr8_no_idle_gap_bug3(dut, write_addr, write_beats, read_addr):
    """
    Dedicated BUG3 helper.

    Legal single-driver AHB sequence:
      - INCR8 write
      - immediate read NONSEQ with no IDLE gap
      - return the 8 read beats

    Important: after launching the read NONSEQ, do NOT count generic HREADY
    cycles as data beats. Wait for actual read-data-valid cycles.
    """
    assert len(write_beats) == 8

    hburst = 0b101   # INCR8
    hsize  = 3       # 64-bit
    results = []

    async def _wait_actual_read_beat(timeout_cycles=2000):
        for _ in range(timeout_cycles):
            await RisingEdge(dut.clk)
            await ReadOnly()
            # After the no-gap read launch, returned AHB read beats are identified
            # by HREADY. Do not key this off raw AXI RVALID, because the bridge may
            # present a buffered beat one cycle later.
            if int(dut.hready.value):
                return int(dut.hrdata.value)
        raise TimeoutError("BUG3 helper: timed out waiting for actual read beat")


    # Start write burst
    dut.haddr.value      = write_addr
    dut.hburst.value     = hburst
    dut.hmastlock.value  = 0
    dut.hprot.value      = 0
    dut.hsize.value      = hsize
    dut.htrans.value     = 0b10   # NONSEQ
    dut.hwrite.value     = 1
    dut.hwdata.value     = write_beats[0]

    # Complete write burst; on last accepted beat, launch read NONSEQ immediately
    for i in range(8):
        await _wait_hready(dut)

        if i == 7:
            dut.haddr.value      = read_addr
            dut.hburst.value     = hburst
            dut.hmastlock.value  = 0
            dut.hprot.value      = 0
            dut.hsize.value      = hsize
            dut.htrans.value     = 0b10   # NONSEQ
            dut.hwrite.value     = 0
            dut.hwdata.value     = 0
        else:
            dut.haddr.value      = write_addr + (i + 1) * 8
            dut.hburst.value     = hburst
            dut.hmastlock.value  = 0
            dut.hprot.value      = 0
            dut.hsize.value      = hsize
            dut.htrans.value     = 0b11   # SEQ
            dut.hwrite.value     = 1
            dut.hwdata.value     = write_beats[i + 1]

    # Read burst
    # First wait until the bridge actually enters the read stall phase.
    # This skips the write-response / pending-read handoff HREADY pulses.
    for _ in range(2000):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.hready.value) == 0:
            break
    else:
        raise TimeoutError("BUG3 helper: never entered read stall after no-gap read launch")

    # Now count the 8 returned AHB read beats.
    for i in range(8):
        results.append(await _wait_actual_read_beat())
        await NextTimeStep()

        if i == 7:
            dut.haddr.value      = 0
            dut.hburst.value     = 0
            dut.hmastlock.value  = 0
            dut.hprot.value      = 0
            dut.hsize.value      = 0
            dut.htrans.value     = 0   # IDLE
            dut.hwrite.value     = 0
            dut.hwdata.value     = 0
        else:
            dut.haddr.value      = read_addr + (i + 1) * 8
            dut.hburst.value     = hburst
            dut.hmastlock.value  = 0
            dut.hprot.value      = 0
            dut.hsize.value      = hsize
            dut.htrans.value     = 0b11   # SEQ
            dut.hwrite.value     = 0
            dut.hwdata.value     = 0

    _init_direct_ahb_signals(dut)
    return results


# async def _bug3_read_slave_monitor(dut, mem, ar_seen_evt, rd_done_evt, rd_info):
#     """
#     Dedicated AXI read responder for BUG3 only.
#     Records AR info, serves exactly one burst, then sets rd_done_evt.
#     Never rely on awaiting the task object itself.
#     """
#     while not int(dut.m_axi_arvalid.value):
#         await RisingEdge(dut.clk)

#     dut.m_axi_arready.value = 1
#     while True:
#         await RisingEdge(dut.clk)
#         if int(dut.m_axi_arvalid.value) and int(dut.m_axi_arready.value):
#             rd_info["addr"]  = int(dut.m_axi_araddr.value)
#             rd_info["len"]   = int(dut.m_axi_arlen.value)
#             rd_info["size"]  = int(dut.m_axi_arsize.value)
#             rd_info["burst"] = int(dut.m_axi_arburst.value)
#             ar_seen_evt.set()
#             break
#     dut.m_axi_arready.value = 0

#     await ClockCycles(dut.clk, 2)

#     for beat in range(rd_info["len"] + 1):
#         if rd_info["burst"] == 1:
#             addr = rd_info["addr"] + beat * (1 << rd_info["size"])
#         else:
#             addr = rd_info["addr"]

#         rdata = mem.get(addr & ~0x7, 0)

#         dut.m_axi_rdata.value  = rdata
#         dut.m_axi_rresp.value  = 0
#         dut.m_axi_rlast.value  = 1 if beat == rd_info["len"] else 0
#         dut.m_axi_rvalid.value = 1

#         while True:
#             await RisingEdge(dut.clk)
#             if int(dut.m_axi_rvalid.value) and int(dut.m_axi_rready.value):
#                 break

#         dut.m_axi_rvalid.value = 0
#         dut.m_axi_rlast.value  = 0

#     rd_done_evt.set()

async def _bug3_read_slave_monitor(dut, mem, ar_seen_evt, rd_done_evt, rd_info):
    """
    Dedicated AXI read responder for BUG3 only.

    Important difference vs previous broken version:
    keep ARREADY asserted up front, so we cannot miss a short ARVALID pulse
    or the handshake cycle.
    """
    dut.m_axi_arready.value = 1

    # Wait for AR handshake and capture it in the same cycle it happens
    while True:
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.m_axi_arvalid.value) and int(dut.m_axi_arready.value):
            rd_info["addr"]  = int(dut.m_axi_araddr.value)
            rd_info["len"]   = int(dut.m_axi_arlen.value)
            rd_info["size"]  = int(dut.m_axi_arsize.value)
            rd_info["burst"] = int(dut.m_axi_arburst.value)
            ar_seen_evt.set()
            break

    await NextTimeStep()
    dut.m_axi_arready.value = 0

    await ClockCycles(dut.clk, 2)

    for beat in range(rd_info["len"] + 1):
        if rd_info["burst"] == 1:
            addr = rd_info["addr"] + beat * (1 << rd_info["size"])
        else:
            addr = rd_info["addr"]

        rdata = mem.get(addr & ~0x7, 0)

        dut.m_axi_rdata.value  = rdata
        dut.m_axi_rresp.value  = 0
        dut.m_axi_rlast.value  = 1 if beat == rd_info["len"] else 0
        dut.m_axi_rvalid.value = 1

        while True:
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.m_axi_rvalid.value) and int(dut.m_axi_rready.value):
                break

        await NextTimeStep()
        dut.m_axi_rvalid.value = 0
        dut.m_axi_rlast.value  = 0

    rd_done_evt.set()

async def _ahb_read_incr8_manual_bug3_sampled(dut, start_addr):
    """Dedicated helper for BUG3 immediate read-after-write case."""
    hburst = 0b101   # INCR8
    hsize  = 3
    results = []

    async def _wait_hready_sampled(timeout_cycles=2000):
        for _ in range(timeout_cycles):
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.hready.value):
                return int(dut.hrdata.value)
        raise TimeoutError("hready never went high")

    dut.haddr.value  = start_addr
    dut.hburst.value = hburst
    dut.hmastlock.value = 0
    dut.hprot.value  = 0
    dut.hsize.value  = hsize
    dut.htrans.value = 0b10   # NONSEQ
    dut.hwrite.value = 0
    dut.hwdata.value = 0

    for i in range(8):
        sampled = await _wait_hready_sampled()
        results.append(sampled)

        await NextTimeStep()

        if i == 7:
            dut.haddr.value  = 0
            dut.hburst.value = 0
            dut.hsize.value  = 0
            dut.htrans.value = 0
        else:
            dut.haddr.value  = start_addr + (i + 1) * 8
            dut.htrans.value = 0b11   # SEQ

    await NextTimeStep()

    _init_direct_ahb_signals(dut)
    return results

async def _ahb_read_incr8_manual_stalled_first_beat(dut, start_addr):
    """Dedicated helper for reads where HREADY stays low until beat[0] is valid."""
    hburst = 0b101   # INCR8
    hsize  = 3
    results = []

    async def _wait_hready_sampled(timeout_cycles=2000):
        for _ in range(timeout_cycles):
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.hready.value):
                return int(dut.hrdata.value)
        raise TimeoutError("hready never went high")

    dut.haddr.value  = start_addr
    dut.hburst.value = hburst
    dut.hmastlock.value = 0
    dut.hprot.value  = 0
    dut.hsize.value  = hsize
    dut.htrans.value = 0b10   # NONSEQ
    dut.hwrite.value = 0
    dut.hwdata.value = 0

    for i in range(8):
        results.append(await _wait_hready_sampled())
        await NextTimeStep()

        if i == 7:
            dut.haddr.value  = 0
            dut.hburst.value = 0
            dut.hsize.value  = 0
            dut.htrans.value = 0
        else:
            dut.haddr.value  = start_addr + (i + 1) * 8
            dut.htrans.value = 0b11   # SEQ

    _init_direct_ahb_signals(dut)
    return results

async def _ahb_read_incr8_manual_actual_rbeats(dut, start_addr, timeout_cycles=5000):
    """
    Dedicated helper for delayed-read / post-write-read cases.
    Samples only on ACTUAL returned read beats:
      HREADY == 1 and m_axi_rvalid == 1
    """
    hburst = 0b101   # INCR8
    hsize  = 3
    results = []

    async def _wait_actual_read_beat():
        for _ in range(timeout_cycles):
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.hready.value) and int(dut.m_axi_rvalid.value):
                return int(dut.hrdata.value)
        raise TimeoutError("actual read beat never arrived")

    dut.haddr.value  = start_addr
    dut.hburst.value = hburst
    dut.hmastlock.value = 0
    dut.hprot.value  = 0
    dut.hsize.value  = hsize
    dut.htrans.value = 0b10   # NONSEQ
    dut.hwrite.value = 0
    dut.hwdata.value = 0

    for i in range(8):
        results.append(await _wait_actual_read_beat())
        await NextTimeStep()

        if i == 7:
            dut.haddr.value  = 0
            dut.hburst.value = 0
            dut.hsize.value  = 0
            dut.htrans.value = 0
        else:
            dut.haddr.value  = start_addr + (i + 1) * 8
            dut.htrans.value = 0b11   # SEQ

    _init_direct_ahb_signals(dut)
    return results

@cocotb.test()
async def test_single_qword_write_updates_axi_ram(dut):
    set_test_id(dut)
    ahb, axi_ram = await setup_dut(dut)

    addr = 0x100
    data = 0x0123456789ABCDEF

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_w_handshake(dut))

    await ahb.write(addr, data, size=8, sync=True)
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w = await w_task

    assert aw["addr"] == addr, f"unexpected AWADDR: 0x{aw['addr']:08x}"
    assert aw["len"] == 0, f"expected single-beat AXI write, got AWLEN={aw['len']}"
    assert aw["size"] == 3, f"expected 8-byte beat, got AWSIZE={aw['size']}"
    assert aw["burst"] == 1, f"expected INCR burst type, got AWBURST={aw['burst']}"

    assert w["data"] == data, f"unexpected WDATA: 0x{w['data']:016x}"
    assert w["strb"] == 0xFF, f"unexpected WSTRB: 0x{w['strb']:02x}"
    assert w["last"] == 1, f"expected WLAST=1, got {w['last']}"

    got = axi_ram.read_qword(addr)
    assert got == data, f"AXI RAM mismatch: expected 0x{data:016x}, got 0x{got:016x}"


@cocotb.test()
async def test_single_qword_read_returns_axi_ram_contents(dut):

    set_test_id(dut)

    ahb, axi_ram = await setup_dut(dut)

    addr = 0x180
    expected = 0xDEADBEEFCAFEBABE
    axi_ram.write_qword(addr, expected)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))

    read_resp = await ahb.read(addr, size=8, sync=True)
    await ClockCycles(dut.clk, 2)

    ar = await ar_task
    assert ar["addr"] == addr, f"unexpected ARADDR: 0x{ar['addr']:08x}"
    assert ar["len"] == 0, f"expected single-beat AXI read, got ARLEN={ar['len']}"
    assert ar["size"] == 3, f"expected 8-byte beat, got ARSIZE={ar['size']}"
    assert ar["burst"] == 1, f"expected INCR burst type, got ARBURST={ar['burst']}"

    got = _extract_read_data(read_resp, dut)
    assert got == expected, f"readback mismatch: expected 0x{expected:016x}, got 0x{got:016x}"


@cocotb.test()
async def test_incr4_qword_write_burst_updates_axi_ram(dut):

    set_test_id(dut)

    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    addr = 0x200
    data_words = [
        0x1111111122222222,
        0x3333333344444444,
        0x5555555566666666,
        0x7777777788888888,
    ]

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 4))

    await drive_ahb_incr4_write_burst(dut, addr, data_words)

    aw = await aw_task
    w_beats = await w_task

    assert aw["addr"] == addr, f"unexpected AWADDR: 0x{aw['addr']:08x}"
    assert aw["len"] == 3, f"expected 4-beat AXI burst, got AWLEN={aw['len']}"
    assert aw["size"] == 3, f"expected 8-byte beat, got AWSIZE={aw['size']}"
    assert aw["burst"] == 1, f"expected INCR burst type, got AWBURST={aw['burst']}"

    assert len(w_beats) == 4, f"expected 4 W beats, got {len(w_beats)}"
    for i, beat in enumerate(w_beats):
        assert beat["data"] == data_words[i], (
            f"unexpected WDATA beat {i}: expected 0x{data_words[i]:016x}, got 0x{beat['data']:016x}"
        )
        assert beat["strb"] == 0xFF, f"unexpected WSTRB beat {i}: 0x{beat['strb']:02x}"
        assert beat["last"] == (1 if i == 3 else 0), (
            f"unexpected WLAST beat {i}: expected {(1 if i == 3 else 0)}, got {beat['last']}"
        )

    for i, expected in enumerate(data_words):
        got = axi_ram.read_qword(addr + i * 8)
        assert got == expected, (
            f"AXI RAM mismatch at beat {i}: expected 0x{expected:016x}, got 0x{got:016x}"
        )


@cocotb.test()
async def test_single_write_sizes_and_wstrb(dut):

    set_test_id(dut)

    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    cases = [
        (1, 0, 0xA1),
        (1, 3, 0xB2),
        (1, 7, 0xC3),
        (2, 0, 0xD4E5),
        (2, 2, 0x1627),
        (2, 6, 0x3849),
        (4, 0, 0x55667788),
        (4, 4, 0x99AABBCC),
        (8, 0, 0x0123456789ABCDEF),
    ]

    for idx, (size_bytes, offset, value) in enumerate(cases):
        base_addr = 0x400 + idx * 0x20
        addr = base_addr + offset
        old_qword = 0xF0E1D2C3B4A59687

        axi_ram.write_qword(base_addr, old_qword)

        aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
        w_task = cocotb.start_soon(wait_for_w_handshake(dut))

        await ahb_write_single_manual(dut, addr, value, size_bytes)
        await ClockCycles(dut.clk, 2)

        aw = await aw_task
        w = await w_task

        expected_wdata = (value & _mask_nbytes(size_bytes)) << _lane_shift(addr)
        expected_wstrb = _strb_mask(addr, size_bytes)
        expected_qword = _apply_subword_to_qword(old_qword, addr, size_bytes, value)

        assert aw["addr"] == addr, f"case {idx}: bad AWADDR 0x{aw['addr']:08x}"
        assert aw["len"] == 0, f"case {idx}: expected AWLEN=0, got {aw['len']}"
        assert aw["size"] == _size_bytes_to_axsize(size_bytes), f"case {idx}: bad AWSIZE {aw['size']}"
        assert aw["burst"] == 1, f"case {idx}: expected AWBURST=INCR, got {aw['burst']}"

        assert w["data"] == expected_wdata, (
            f"case {idx}: bad WDATA expected 0x{expected_wdata:016x}, got 0x{w['data']:016x}"
        )
        assert w["strb"] == expected_wstrb, (
            f"case {idx}: bad WSTRB expected 0x{expected_wstrb:02x}, got 0x{w['strb']:02x}"
        )
        assert w["last"] == 1, f"case {idx}: expected WLAST=1, got {w['last']}"

        got_qword = axi_ram.read_qword(base_addr)
        assert got_qword == expected_qword, (
            f"case {idx}: RAM mismatch expected 0x{expected_qword:016x}, got 0x{got_qword:016x}"
        )


@cocotb.test()
async def test_single_read_sizes_and_data_lanes(dut):
    set_test_id(dut)
    ahb, axi_ram = await setup_dut(dut)

    cases = [
        # size_bytes, offset
        (1, 0),
        (1, 3),
        (1, 7),
        (2, 0),
        (2, 2),
        (2, 6),
        (4, 0),
        (4, 4),
        (8, 0),
    ]

    for idx, (size_bytes, offset) in enumerate(cases):
        base_addr = 0x800 + idx * 0x20
        addr = base_addr + offset
        pattern = 0x8877665544332211

        axi_ram.write_qword(base_addr, pattern)

        ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))

        read_resp = await ahb.read(addr, size=size_bytes, sync=True)
        await ClockCycles(dut.clk, 2)

        ar = await ar_task
        raw = _extract_read_data(read_resp, dut)

        expected_subword = (pattern >> _lane_shift(addr)) & _mask_nbytes(size_bytes)
        got_subword = (raw >> _lane_shift(addr)) & _mask_nbytes(size_bytes)

        assert ar["addr"] == addr, f"case {idx}: bad ARADDR 0x{ar['addr']:08x}"
        assert ar["len"] == 0, f"case {idx}: expected ARLEN=0, got {ar['len']}"
        assert ar["size"] == _size_bytes_to_axsize(size_bytes), (
            f"case {idx}: bad ARSIZE {ar['size']}"
        )
        assert ar["burst"] == 1, f"case {idx}: expected ARBURST=INCR, got {ar['burst']}"

        assert got_subword == expected_subword, (
            f"case {idx}: read mismatch expected 0x{expected_subword:x}, got 0x{got_subword:x} "
            f"(raw HRDATA 0x{raw:016x})"
        )


@cocotb.test()
async def test_incr4_qword_read_burst_returns_expected(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x1400
    expected_beats = [
        0x0123456789ABCDEF,
        0x1111222233334444,
        0xAAAABBBBCCCCDDDD,
        0xDEADBEEFCAFEBABE,
    ]

    for i, value in enumerate(expected_beats):
        axi_ram.write_qword(start_addr + i * 8, value)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))
    r_task = cocotb.start_soon(wait_for_n_r_handshakes(dut, 4))

    got_beats = await ahb_read_incr4_manual(dut, start_addr, size_bytes=8)
    await ClockCycles(dut.clk, 2)

    ar = await ar_task
    r_beats = await r_task

    assert ar["addr"] == start_addr, f"bad ARADDR 0x{ar['addr']:08x}"
    assert ar["len"] == 3, f"expected ARLEN=3, got {ar['len']}"
    assert ar["size"] == 3, f"expected ARSIZE=3, got {ar['size']}"
    assert ar["burst"] == 1, f"expected ARBURST=INCR, got {ar['burst']}"

    assert got_beats == expected_beats, (
        f"AHB read burst mismatch expected {expected_beats}, got {got_beats}"
    )

    for i, rb in enumerate(r_beats):
        assert rb["data"] == expected_beats[i], (
            f"R beat {i}: bad RDATA expected 0x{expected_beats[i]:016x}, got 0x{rb['data']:016x}"
        )
        assert rb["resp"] == 0, f"R beat {i}: expected RRESP=OKAY, got {rb['resp']}"
        assert rb["last"] == (1 if i == 3 else 0), (
            f"R beat {i}: bad RLAST expected {1 if i == 3 else 0}, got {rb['last']}"
        )


@cocotb.test()
async def test_idle_and_busy_do_not_emit_axi(dut):
    set_test_id(dut)
    _, _ = await setup_dut(dut, with_ahb_master=False)

    mon = cocotb.start_soon(assert_no_axi_valids_for(dut, 3))
    await ahb_control_only_cycle(
        dut,
        addr=0x2000,
        hburst=0,
        size_bytes=8,
        htrans=0b00,   # IDLE
        hwrite=0,
    )
    await mon

    mon = cocotb.start_soon(assert_no_axi_valids_for(dut, 3))
    await ahb_control_only_cycle(
        dut,
        addr=0x2000,
        hburst=0b011,  # INCR4 context, but BUSY should still not start a transfer
        size_bytes=8,
        htrans=0b01,   # BUSY
        hwrite=1,
    )
    await mon


@cocotb.test()
async def test_single_qword_write_stalls_under_axi_backpressure(dut):
    set_test_id(dut)
    ahb, axi_ram = await setup_dut(dut)

    set_axi_ram_backpressure(
        axi_ram,
        stall_aw=True,
        stall_w=True,
        stall_b=True,
    )

    addr = 0x1800
    data = 0x1122334455667788

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_w_handshake(dut))

    _, hready_low_cycles, _ = await run_and_watch_hready(
        dut,
        ahb.write(addr, data, size=8, sync=True)
    )
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w = await w_task

    assert hready_low_cycles > 0, "Expected at least one AHB wait-state"
    assert aw["addr"] == addr, f"Bad AWADDR 0x{aw['addr']:08x}"
    assert aw["len"] == 0, f"Expected AWLEN=0, got {aw['len']}"
    assert aw["size"] == 3, f"Expected AWSIZE=3, got {aw['size']}"
    assert w["data"] == data, f"Bad WDATA 0x{w['data']:016x}"
    assert w["strb"] == 0xFF, f"Bad WSTRB 0x{w['strb']:02x}"
    assert w["last"] == 1, f"Expected WLAST=1, got {w['last']}"

    got = axi_ram.read_qword(addr)
    assert got == data, f"RAM mismatch expected 0x{data:016x}, got 0x{got:016x}"


@cocotb.test()
async def test_single_qword_read_stalls_under_axi_backpressure(dut):
    set_test_id(dut)
    ahb, axi_ram = await setup_dut(dut)

    set_axi_ram_backpressure(
        axi_ram,
        stall_ar=True,
        stall_r=True,
    )

    addr = 0x1A00
    data = 0xDEADBEEFCAFEBABE
    axi_ram.write_qword(addr, data)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))

    read_resp, hready_low_cycles, _ = await run_and_watch_hready(
        dut,
        ahb.read(addr, size=8, sync=True)
    )
    await ClockCycles(dut.clk, 2)

    ar = await ar_task
    got = _extract_read_data(read_resp, dut)

    assert hready_low_cycles > 0, "Expected at least one AHB wait-state"
    assert ar["addr"] == addr, f"Bad ARADDR 0x{ar['addr']:08x}"
    assert ar["len"] == 0, f"Expected ARLEN=0, got {ar['len']}"
    assert ar["size"] == 3, f"Expected ARSIZE=3, got {ar['size']}"
    assert got == data, f"Read mismatch expected 0x{data:016x}, got 0x{got:016x}"


@cocotb.test()
async def test_incr4_qword_write_burst_stalls_under_axi_backpressure(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    set_axi_ram_backpressure(
        axi_ram,
        stall_aw=True,
        stall_w=True,
        stall_b=True,
    )

    start_addr = 0x1C00
    beats = [
        0x0101010102020202,
        0x0303030304040404,
        0x0505050506060606,
        0x0707070708080808,
    ]

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 4))

    _, hready_low_cycles, _ = await run_and_watch_hready(
        dut,
        ahb_write_incr4_manual(dut, start_addr, beats, size_bytes=8)
    )
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w_beats = await w_task

    assert hready_low_cycles > 0, "Expected at least one AHB wait-state in burst"
    assert aw["addr"] == start_addr, f"Bad AWADDR 0x{aw['addr']:08x}"
    assert aw["len"] == 3, f"Expected AWLEN=3, got {aw['len']}"
    assert aw["size"] == 3, f"Expected AWSIZE=3, got {aw['size']}"

    for i, wb in enumerate(w_beats):
        assert wb["data"] == beats[i], (
            f"beat {i}: bad WDATA expected 0x{beats[i]:016x}, got 0x{wb['data']:016x}"
        )
        assert wb["strb"] == 0xFF, (
            f"beat {i}: bad WSTRB expected 0xFF, got 0x{wb['strb']:02x}"
        )
        assert wb["last"] == (1 if i == 3 else 0), (
            f"beat {i}: bad WLAST expected {1 if i == 3 else 0}, got {wb['last']}"
        )

    for i, expected in enumerate(beats):
        got = axi_ram.read_qword(start_addr + i * 8)
        assert got == expected, (
            f"RAM beat {i}: expected 0x{expected:016x}, got 0x{got:016x}"
        )

@cocotb.test()
async def test_single_write_axi_error_propagates_to_hresp_and_recovers(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    cases = [
        ("SLVERR", 0b10),
        ("DECERR", 0b11),
    ]

    for idx, (name, bresp) in enumerate(cases):
        addr = 0x2400 + idx * 0x40
        data = 0x1111222233334444 + idx

        slave_task = cocotb.start_soon(
            axi_slave_respond_single_write(dut, bresp=bresp, b_delay=2)
        )

        hresp = await ahb_write_single_manual_status(dut, addr, data, 8)
        info = await slave_task

        assert info["aw"]["addr"] == addr, f"{name}: bad AWADDR 0x{info['aw']['addr']:08x}"
        assert info["aw"]["len"] == 0, f"{name}: expected AWLEN=0, got {info['aw']['len']}"
        assert info["aw"]["size"] == 3, f"{name}: expected AWSIZE=3, got {info['aw']['size']}"
        assert info["w"]["data"] == data, (
            f"{name}: bad WDATA expected 0x{data:016x}, got 0x{info['w']['data']:016x}"
        )
        assert info["w"]["strb"] == 0xFF, (
            f"{name}: bad WSTRB expected 0xFF, got 0x{info['w']['strb']:02x}"
        )
        assert info["w"]["last"] == 1, f"{name}: expected WLAST=1, got {info['w']['last']}"
        assert hresp == 1, f"{name}: expected HRESP=ERROR(1), got {hresp}"

        ok_addr = addr + 0x100
        ok_data = 0xAAAABBBBCCCCDDDD + idx
        ok_task = cocotb.start_soon(
            axi_slave_respond_single_write(dut, bresp=0, b_delay=1)
        )
        ok_hresp = await ahb_write_single_manual_status(dut, ok_addr, ok_data, 8)
        ok_info = await ok_task

        assert ok_info["aw"]["addr"] == ok_addr
        assert ok_info["w"]["data"] == ok_data
        assert ok_hresp == 0, f"{name}: recovery transfer should return HRESP=OKAY(0), got {ok_hresp}"


@cocotb.test()
async def test_single_read_axi_error_propagates_to_hresp_and_recovers(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    cases = [
        ("SLVERR", 0b10),
        ("DECERR", 0b11),
    ]

    for idx, (name, rresp) in enumerate(cases):
        addr = 0x2800 + idx * 0x40

        slave_task = cocotb.start_soon(
            axi_slave_respond_single_read(
                dut,
                rdata=0xDEADBEEFCAFEBABE,
                rresp=rresp,
                r_delay=2,
            )
        )

        _, hresp = await ahb_read_single_manual_status(dut, addr, 8)
        info = await slave_task

        assert info["ar"]["addr"] == addr, f"{name}: bad ARADDR 0x{info['ar']['addr']:08x}"
        assert info["ar"]["len"] == 0, f"{name}: expected ARLEN=0, got {info['ar']['len']}"
        assert info["ar"]["size"] == 3, f"{name}: expected ARSIZE=3, got {info['ar']['size']}"
        assert hresp == 1, f"{name}: expected HRESP=ERROR(1), got {hresp}"

        ok_addr = addr + 0x100
        ok_data = 0x0123456789ABCDEF + idx
        ok_task = cocotb.start_soon(
            axi_slave_respond_single_read(
                dut,
                rdata=ok_data,
                rresp=0,
                r_delay=1,
            )
        )
        got, ok_hresp = await ahb_read_single_manual_status(dut, ok_addr, 8)
        ok_info = await ok_task

        assert ok_info["ar"]["addr"] == ok_addr
        assert got == ok_data, (
            f"{name}: recovery read expected 0x{ok_data:016x}, got 0x{got:016x}"
        )
        assert ok_hresp == 0, f"{name}: recovery transfer should return HRESP=OKAY(0), got {ok_hresp}"


@cocotb.test()
async def test_hmastlock_maps_to_awlock_on_single_writes(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    cases = [
        (0, 0x2C00, 0x1111111122222222),
        (1, 0x2C40, 0x3333333344444444),
    ]

    for hmastlock, addr, data in cases:
        slave_task = cocotb.start_soon(
            axi_slave_respond_single_write(dut, bresp=0, b_delay=1)
        )

        hresp = await ahb_write_single_manual_status(
            dut, addr, data, 8, hmastlock=hmastlock
        )
        info = await slave_task

        assert hresp == 0, f"hmastlock={hmastlock}: expected HRESP=OKAY(0), got {hresp}"
        assert info["aw"]["addr"] == addr
        assert info["aw"]["lock"] == hmastlock, (
            f"hmastlock={hmastlock}: expected AWLOCK={hmastlock}, got {info['aw']['lock']}"
        )
        assert info["w"]["data"] == data


@cocotb.test()
async def test_hmastlock_maps_to_arlock_on_single_reads(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    cases = [
        (0, 0x3000, 0x5566778899AABBCC),
        (1, 0x3040, 0xDDEEFF0011223344),
    ]

    for hmastlock, addr, rdata in cases:
        slave_task = cocotb.start_soon(
            axi_slave_respond_single_read(
                dut,
                rdata=rdata,
                rresp=0,
                r_delay=1,
            )
        )

        got, hresp = await ahb_read_single_manual_status(
            dut, addr, 8, hmastlock=hmastlock
        )
        info = await slave_task

        assert hresp == 0, f"hmastlock={hmastlock}: expected HRESP=OKAY(0), got {hresp}"
        assert info["ar"]["addr"] == addr
        assert info["ar"]["lock"] == hmastlock, (
            f"hmastlock={hmastlock}: expected ARLOCK={hmastlock}, got {info['ar']['lock']}"
        )
        assert got == rdata, (
            f"hmastlock={hmastlock}: expected HRDATA=0x{rdata:016x}, got 0x{got:016x}"
        )

@cocotb.test()
async def test_incr4_write_burst_bresp_slverr_propagates(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    start_addr = 0x3400
    beats = [
        0x1111111122222222,
        0x3333333344444444,
        0x5555555566666666,
        0x7777777788888888,
    ]

    slave_task = cocotb.start_soon(
        axi_slave_respond_incr4_write_burst(dut, bresp=0b10, b_delay=2)
    )
    status = await ahb_write_incr4_manual_status(dut, start_addr, beats, size_bytes=8)
    info = await slave_task

    assert info["aw"]["addr"] == start_addr
    assert info["aw"]["len"] == 3
    assert info["aw"]["size"] == 3
    assert info["aw"]["burst"] == 1

    for i, wb in enumerate(info["w_beats"]):
        assert wb["data"] == beats[i]
        assert wb["strb"] == 0xFF
        assert wb["last"] == (1 if i == 3 else 0)

    assert status["final_resp"] == 1, f"Expected final HRESP=ERROR(1), got {status['final_resp']}"


@cocotb.test()
async def test_incr4_write_burst_bresp_decerr_propagates(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    start_addr = 0x3480
    beats = [
        0x0101010102020202,
        0x0303030304040404,
        0x0505050506060606,
        0x0707070708080808,
    ]

    slave_task = cocotb.start_soon(
        axi_slave_respond_incr4_write_burst(dut, bresp=0b11, b_delay=2)
    )
    status = await ahb_write_incr4_manual_status(dut, start_addr, beats, size_bytes=8)
    info = await slave_task

    assert info["aw"]["addr"] == start_addr
    assert info["aw"]["len"] == 3
    assert status["final_resp"] == 1, f"Expected final HRESP=ERROR(1), got {status['final_resp']}"


@cocotb.test()
async def test_incr4_read_burst_rresp_error_on_last_beat(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    start_addr = 0x3500
    beats = [
        0x0123456789ABCDEF,
        0x1111222233334444,
        0xAAAABBBBCCCCDDDD,
        0xDEADBEEFCAFEBABE,
    ]

    slave_task = cocotb.start_soon(
        axi_slave_respond_incr4_read_burst(
            dut, beats, error_index=3, error_resp=0b10, r_gap=1
        )
    )
    samples = await ahb_read_incr4_manual_status(dut, start_addr, size_bytes=8)
    info = await slave_task

    assert info["ar"]["addr"] == start_addr
    assert info["ar"]["len"] == 3
    assert info["ar"]["size"] == 3

    assert len(samples) == 4
    assert samples[0]["resp"] == 0
    assert samples[1]["resp"] == 0
    assert samples[2]["resp"] == 0
    assert samples[3]["resp"] == 1, f"Expected last beat HRESP=ERROR(1), got {samples[3]['resp']}"


@cocotb.test()
async def test_incr4_read_burst_rresp_error_on_middle_beat(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    start_addr = 0x3580
    beats = [
        0x1111111122222222,
        0x3333333344444444,
        0x5555555566666666,
        0x7777777788888888,
    ]

    slave_task = cocotb.start_soon(
        axi_slave_respond_incr4_read_burst(
            dut, beats, error_index=1, error_resp=0b10, r_gap=1
        )
    )
    samples = await ahb_read_incr4_manual_status(dut, start_addr, size_bytes=8)
    info = await slave_task

    assert info["ar"]["addr"] == start_addr
    assert info["ar"]["len"] == 3

    assert len(samples) == 4
    assert samples[0]["resp"] == 0
    assert samples[1]["resp"] == 1, f"Expected beat1 HRESP=ERROR(1), got {samples[1]['resp']}"

    # Do not over-constrain beats 2/3 yet; just log what the DUT does after the first read error.
    dut._log.info(f"Post-error burst behavior: beat2 resp={samples[2]['resp']} beat3 resp={samples[3]['resp']}")


@cocotb.test()
async def test_reset_during_pending_single_write_clears_bus(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    dut.haddr.value = 0x3600
    dut.hburst.value = 0
    dut.hmastlock.value = 0
    dut.hprot.value = 0
    dut.hsize.value = 3
    dut.htrans.value = 0b10
    dut.hwrite.value = 1
    dut.hwdata.value = 0

    await RisingEdge(dut.clk)
    dut.resetn.value = 0
    await ClockCycles(dut.clk, 2)
    dut.resetn.value = 1

    _init_direct_ahb_signals(dut)
    await wait_until_all_axi_outputs_idle(dut, max_cycles=20)

    assert int(dut.hready.value) in (0, 1)
    assert int(dut.hresp.value) == 0


@cocotb.test()
async def test_reset_during_pending_read_burst_clears_bus(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    dut.haddr.value = 0x3700
    dut.hburst.value = 0b011
    dut.hmastlock.value = 0
    dut.hprot.value = 0
    dut.hsize.value = 3
    dut.htrans.value = 0b10
    dut.hwrite.value = 0
    dut.hwdata.value = 0

    await RisingEdge(dut.clk)
    dut.resetn.value = 0
    await ClockCycles(dut.clk, 2)
    dut.resetn.value = 1

    _init_direct_ahb_signals(dut)
    await wait_until_all_axi_outputs_idle(dut, max_cycles=20)

    assert int(dut.hresp.value) == 0


@cocotb.test()
async def test_incr8_qword_write_burst_updates_axi_ram(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x4000
    beats = [
        0x0101010101010101,
        0x0202020202020202,
        0x0303030303030303,
        0x0404040404040404,
        0x0505050505050505,
        0x0606060606060606,
        0x0707070707070707,
        0x0808080808080808,
    ]

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 8))

    await ahb_write_inc_burst_manual(dut, start_addr, beats, size_bytes=8, fixed=True)
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w_beats = await w_task

    assert aw["addr"] == start_addr, f"bad AWADDR 0x{aw['addr']:08x}"
    assert aw["len"] == 7, f"expected AWLEN=7, got {aw['len']}"
    assert aw["size"] == 3, f"expected AWSIZE=3, got {aw['size']}"
    assert aw["burst"] == 1, f"expected AWBURST=INCR, got {aw['burst']}"

    for i, wb in enumerate(w_beats):
        assert wb["data"] == beats[i], (
            f"beat {i}: bad WDATA expected 0x{beats[i]:016x}, got 0x{wb['data']:016x}"
        )
        assert wb["strb"] == 0xFF, (
            f"beat {i}: bad WSTRB expected 0xFF, got 0x{wb['strb']:02x}"
        )
        assert wb["last"] == (1 if i == 7 else 0), (
            f"beat {i}: bad WLAST expected {1 if i == 7 else 0}, got {wb['last']}"
        )

    for i, expected in enumerate(beats):
        got = axi_ram.read_qword(start_addr + i * 8)
        assert got == expected, (
            f"RAM beat {i}: expected 0x{expected:016x}, got 0x{got:016x}"
        )


@cocotb.test()
async def test_incr8_qword_read_burst_returns_expected(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x4200
    expected_beats = [
        0x1111111122222222,
        0x3333333344444444,
        0x5555555566666666,
        0x7777777788888888,
        0x99999999AAAAAAAA,
        0xBBBBBBBBCCCCCCCC,
        0xDDDDDDDDEEEEEEEE,
        0x123456789ABCDEF0,
    ]

    for i, value in enumerate(expected_beats):
        axi_ram.write_qword(start_addr + i * 8, value)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))
    r_task = cocotb.start_soon(wait_for_n_r_handshakes(dut, 8))

    got_beats = await ahb_read_inc_burst_manual(dut, start_addr, 8, size_bytes=8, fixed=True)
    await ClockCycles(dut.clk, 2)

    ar = await ar_task
    r_beats = await r_task

    assert ar["addr"] == start_addr, f"bad ARADDR 0x{ar['addr']:08x}"
    assert ar["len"] == 7, f"expected ARLEN=7, got {ar['len']}"
    assert ar["size"] == 3, f"expected ARSIZE=3, got {ar['size']}"
    assert ar["burst"] == 1, f"expected ARBURST=INCR, got {ar['burst']}"

    assert got_beats == expected_beats, (
        f"AHB read burst mismatch expected {expected_beats}, got {got_beats}"
    )

    for i, rb in enumerate(r_beats):
        assert rb["data"] == expected_beats[i], (
            f"R beat {i}: bad RDATA expected 0x{expected_beats[i]:016x}, got 0x{rb['data']:016x}"
        )
        assert rb["resp"] == 0, f"R beat {i}: expected RRESP=OKAY, got {rb['resp']}"
        assert rb["last"] == (1 if i == 7 else 0), (
            f"R beat {i}: bad RLAST expected {1 if i == 7 else 0}, got {rb['last']}"
        )


@cocotb.test()
async def test_incr16_qword_write_burst_updates_axi_ram(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x4400
    beats = [0x1000000000000000 + i for i in range(16)]

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 16))

    await ahb_write_inc_burst_manual(dut, start_addr, beats, size_bytes=8, fixed=True)
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w_beats = await w_task

    assert aw["addr"] == start_addr, f"bad AWADDR 0x{aw['addr']:08x}"
    assert aw["len"] == 15, f"expected AWLEN=15, got {aw['len']}"
    assert aw["size"] == 3, f"expected AWSIZE=3, got {aw['size']}"
    assert aw["burst"] == 1, f"expected AWBURST=INCR, got {aw['burst']}"

    for i, wb in enumerate(w_beats):
        assert wb["data"] == beats[i], (
            f"beat {i}: bad WDATA expected 0x{beats[i]:016x}, got 0x{wb['data']:016x}"
        )
        assert wb["strb"] == 0xFF, (
            f"beat {i}: bad WSTRB expected 0xFF, got 0x{wb['strb']:02x}"
        )
        assert wb["last"] == (1 if i == 15 else 0), (
            f"beat {i}: bad WLAST expected {1 if i == 15 else 0}, got {wb['last']}"
        )

    for i, expected in enumerate(beats):
        got = axi_ram.read_qword(start_addr + i * 8)
        assert got == expected, (
            f"RAM beat {i}: expected 0x{expected:016x}, got 0x{got:016x}"
        )


@cocotb.test()
async def test_incr16_qword_read_burst_returns_expected(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x4800
    expected_beats = [0xABCDEF0000000000 + i for i in range(16)]

    for i, value in enumerate(expected_beats):
        axi_ram.write_qword(start_addr + i * 8, value)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))
    r_task = cocotb.start_soon(wait_for_n_r_handshakes(dut, 16))

    got_beats = await ahb_read_inc_burst_manual(dut, start_addr, 16, size_bytes=8, fixed=True)
    await ClockCycles(dut.clk, 2)

    ar = await ar_task
    r_beats = await r_task

    assert ar["addr"] == start_addr, f"bad ARADDR 0x{ar['addr']:08x}"
    assert ar["len"] == 15, f"expected ARLEN=15, got {ar['len']}"
    assert ar["size"] == 3, f"expected ARSIZE=3, got {ar['size']}"
    assert ar["burst"] == 1, f"expected ARBURST=INCR, got {ar['burst']}"

    assert got_beats == expected_beats, (
        f"AHB read burst mismatch expected {expected_beats}, got {got_beats}"
    )

    for i, rb in enumerate(r_beats):
        assert rb["data"] == expected_beats[i], (
            f"R beat {i}: bad RDATA expected 0x{expected_beats[i]:016x}, got 0x{rb['data']:016x}"
        )
        assert rb["resp"] == 0, f"R beat {i}: expected RRESP=OKAY, got {rb['resp']}"
        assert rb["last"] == (1 if i == 15 else 0), (
            f"R beat {i}: bad RLAST expected {1 if i == 15 else 0}, got {rb['last']}"
        )


@cocotb.test()
async def test_incr_len5_qword_write_burst_updates_axi_ram(dut):
    set_test_id(dut)    
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x5000
    beats = [
        0x1010101010101010,
        0x2020202020202020,
        0x3030303030303030,
        0x4040404040404040,
        0x5050505050505050,
    ]

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 5))
    b_task = cocotb.start_soon(wait_for_b_handshake(dut))    

    await ahb_write_inc_burst_manual(dut, start_addr, beats, size_bytes=8, fixed=False)
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w_beats = await w_task
    b = await b_task
    await ClockCycles(dut.clk, 1)

    assert aw["addr"] == start_addr, f"bad AWADDR 0x{aw['addr']:08x}"
    assert aw["len"] == 4, f"expected AWLEN=4, got {aw['len']}"
    assert aw["size"] == 3, f"expected AWSIZE=3, got {aw['size']}"
    assert aw["burst"] == 1, f"expected AWBURST=INCR, got {aw['burst']}"

    for i, wb in enumerate(w_beats):
        assert wb["data"] == beats[i], (
            f"beat {i}: bad WDATA expected 0x{beats[i]:016x}, got 0x{wb['data']:016x}"
        )
        assert wb["strb"] == 0xFF, (
            f"beat {i}: bad WSTRB expected 0xFF, got 0x{wb['strb']:02x}"
        )
        assert wb["last"] == (1 if i == 4 else 0), (
            f"beat {i}: bad WLAST expected {1 if i == 4 else 0}, got {wb['last']}"
        )

    for i, expected in enumerate(beats):
        got = axi_ram.read_qword(start_addr + i * 8)
        assert got == expected, (
            f"RAM beat {i}: expected 0x{expected:016x}, got 0x{got:016x}"
        )


async def wait_for_b_handshake(dut):
    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_bvalid.value) and int(dut.m_axi_bready.value):
            return {
                "resp": int(dut.m_axi_bresp.value),
            }

@cocotb.test()
async def test_incr_len5_qword_read_burst_returns_expected(dut):
    set_test_id(dut)    
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x5200
    expected_beats = [
        0x1111111122222222,
        0x3333333344444444,
        0x5555555566666666,
        0x7777777788888888,
        0x99999999AAAAAAAA,
    ]

    for i, value in enumerate(expected_beats):
        axi_ram.write_qword(start_addr + i * 8, value)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))

    got_beats = await ahb_read_inc_burst_manual(dut, start_addr, 5, size_bytes=8, fixed=False)
    await ClockCycles(dut.clk, 20)

    ar = await ar_task

    assert ar["addr"] == start_addr, f"bad ARADDR 0x{ar['addr']:08x}"
    ### For bare INCR reads the bridge issues a chunked AR; ARLEN depends on
    #### the chunk size (MAX_INCR_BEATS) and distance to a 4 KB boundary, NOT
    ##### on the number of AHB beats.  Only check >= nbeats-1.
    assert ar["len"] >= 4, f"expected ARLEN>=4, got {ar['len']}"
    assert ar["size"] == 3, f"expected ARSIZE=3, got {ar['size']}"
    assert ar["burst"] == 1, f"expected ARBURST=INCR, got {ar['burst']}"

    assert got_beats == expected_beats, (
        f"AHB read burst mismatch expected {expected_beats}, got {got_beats}"
    )


@cocotb.test()
async def test_incr_len5_qword_write_burst_with_busy_after_beat1(dut):
    set_test_id(dut)    
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x5400
    beats = [
        0x0101010101010101,
        0x0202020202020202,
        0x0303030303030303,
        0x0404040404040404,
        0x0505050505050505,
    ]

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 5))
    b_task = cocotb.start_soon(wait_for_b_handshake(dut))

    await ahb_write_incr_burst_with_busy_manual(
        dut,
        start_addr,
        beats,
        size_bytes=8,
        busy_after_beats=(1,),
    )
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w_beats = await w_task
    await b_task
    await ClockCycles(dut.clk, 1)    

    assert aw["addr"] == start_addr, f"bad AWADDR 0x{aw['addr']:08x}"
    assert aw["len"] == 4, f"expected AWLEN=4, got {aw['len']}"
    assert aw["size"] == 3, f"expected AWSIZE=3, got {aw['size']}"
    assert aw["burst"] == 1, f"expected AWBURST=INCR, got {aw['burst']}"

    for i, wb in enumerate(w_beats):
        assert wb["data"] == beats[i], (
            f"beat {i}: bad WDATA expected 0x{beats[i]:016x}, got 0x{wb['data']:016x}"
        )
        assert wb["strb"] == 0xFF, (
            f"beat {i}: bad WSTRB expected 0xFF, got 0x{wb['strb']:02x}"
        )
        assert wb["last"] == (1 if i == 4 else 0), (
            f"beat {i}: bad WLAST expected {1 if i == 4 else 0}, got {wb['last']}"
        )

    for i, expected in enumerate(beats):
        got = axi_ram.read_qword(start_addr + i * 8)
        assert got == expected, (
            f"RAM beat {i}: expected 0x{expected:016x}, got 0x{got:016x}"
        )


@cocotb.test()
async def test_incr_len5_qword_read_burst_with_busy_after_beat1(dut):
    set_test_id(dut)    
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x5600
    expected_beats = [
        0xAAAABBBBCCCCDDDD,
        0x1111222233334444,
        0x5555666677778888,
        0x9999AAAABBBBCCCC,
        0xDDDDEEEEFFFF0000,
    ]

    for i, value in enumerate(expected_beats):
        axi_ram.write_qword(start_addr + i * 8, value)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))

    got_beats = await ahb_read_incr_burst_with_busy_manual(
        dut,
        start_addr,
        5,
        size_bytes=8,
        busy_after_beats=(1,),
    )
    ###### Wait for drain of unused AXI R beats
    # BUSY inside the AHB INCR burst must not add an AXI read beat.
    # The bridge is expected to emit one AXI read burst matching only the
    # real data beats of the AHB burst.    
    await ClockCycles(dut.clk, 20)

    ar = await ar_task

    assert ar["addr"] == start_addr, f"bad ARADDR 0x{ar['addr']:08x}"
    assert ar["len"] >= 4, f"expected ARLEN>=4, got {ar['len']}"
    assert ar["size"] == 3, f"expected ARSIZE=3, got {ar['size']}"
    assert ar["burst"] == 1, f"expected ARBURST=INCR, got {ar['burst']}"

    assert got_beats == expected_beats, (
        f"AHB read burst mismatch expected {expected_beats}, got {got_beats}"
    )


def _hburst_wrap_code(nbeats):
    return {
        4: 0b010,   # WRAP4
        8: 0b100,   # WRAP8
        16: 0b110,  # WRAP16
    }[nbeats]


def _wrap_base(addr, size_bytes, nbeats):
    total_bytes = size_bytes * nbeats
    return addr & ~(total_bytes - 1)


def _wrap_addr(start_addr, beat_idx, size_bytes, nbeats):
    total_bytes = size_bytes * nbeats
    base = _wrap_base(start_addr, size_bytes, nbeats)
    start_off = start_addr - base
    off = (start_off + beat_idx * size_bytes) % total_bytes
    return base + off


async def ahb_write_wrap_burst_manual(dut, start_addr, data_beats, size_bytes=8):
    nbeats = len(data_beats)
    assert nbeats in (4, 8, 16)
    assert size_bytes in (1, 2, 4, 8)

    hsize = _size_bytes_to_axsize(size_bytes)
    hburst = _hburst_wrap_code(nbeats)

    first_addr = _wrap_addr(start_addr, 0, size_bytes, nbeats)
    dut.haddr.value = first_addr
    dut.hburst.value = hburst
    dut.hmastlock.value = 0
    dut.hprot.value = 0
    dut.hsize.value = hsize
    dut.htrans.value = 0b10   # NONSEQ
    dut.hwrite.value = 1
    dut.hwdata.value = 0

    for i in range(nbeats):
        await _wait_hready_high(dut)

        addr_i = _wrap_addr(start_addr, i, size_bytes, nbeats)
        dut.hwdata.value = (data_beats[i] & _mask_nbytes(size_bytes)) << _lane_shift(addr_i)

        if i == nbeats - 1:
            dut.haddr.value = 0
            dut.hburst.value = 0
            dut.hsize.value = 0
            dut.htrans.value = 0   # IDLE
            dut.hwrite.value = 0
        else:
            next_addr = _wrap_addr(start_addr, i + 1, size_bytes, nbeats)
            dut.haddr.value = next_addr
            dut.hburst.value = hburst
            dut.hsize.value = hsize
            dut.htrans.value = 0b11   # SEQ
            dut.hwrite.value = 1

    await _wait_hready_high(dut)
    _init_direct_ahb_signals(dut)


async def ahb_read_wrap_burst_manual(dut, start_addr, nbeats, size_bytes=8):
    assert nbeats in (4, 8, 16)
    assert size_bytes in (1, 2, 4, 8)

    hsize = _size_bytes_to_axsize(size_bytes)
    hburst = _hburst_wrap_code(nbeats)
    results = []

    first_addr = _wrap_addr(start_addr, 0, size_bytes, nbeats)
    dut.haddr.value = first_addr
    dut.hburst.value = hburst
    dut.hmastlock.value = 0
    dut.hprot.value = 0
    dut.hsize.value = hsize
    dut.htrans.value = 0b10   # NONSEQ
    dut.hwrite.value = 0
    dut.hwdata.value = 0

    for i in range(nbeats):
        await _wait_hready_high(dut)

        if i > 0:
            results.append(int(dut.hrdata.value))

        if i == nbeats - 1:
            dut.haddr.value = 0
            dut.hburst.value = 0
            dut.hsize.value = 0
            dut.htrans.value = 0   # IDLE
            dut.hwrite.value = 0
        else:
            next_addr = _wrap_addr(start_addr, i + 1, size_bytes, nbeats)
            dut.haddr.value = next_addr
            dut.hburst.value = hburst
            dut.hsize.value = hsize
            dut.htrans.value = 0b11   # SEQ
            dut.hwrite.value = 0

    await _wait_hready_high(dut)
    results.append(int(dut.hrdata.value))

    _init_direct_ahb_signals(dut)
    return results

@cocotb.test()
async def test_incr4_byte_write_burst_updates_byte_lanes(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x6003
    beats = [0x11, 0x22, 0x33, 0x44]

    initial_qwords = {
        0x6000: 0xFFEEDDCCBBAA9988,
    }

    for addr, value in initial_qwords.items():
        axi_ram.write_qword(addr, value)

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 4))

    await ahb_write_inc_burst_manual(dut, start_addr, beats, size_bytes=1, fixed=True)
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w_beats = await w_task

    assert aw["addr"] == start_addr, f"bad AWADDR 0x{aw['addr']:08x}"
    assert aw["len"] == 3, f"expected AWLEN=3, got {aw['len']}"
    assert aw["size"] == 0, f"expected AWSIZE=0, got {aw['size']}"
    assert aw["burst"] == 1, f"expected AWBURST=INCR, got {aw['burst']}"

    for i, wb in enumerate(w_beats):
        addr = start_addr + i
        expected_wdata = (beats[i] & 0xFF) << _lane_shift(addr)
        expected_wstrb = _strb_mask(addr, 1)

        assert wb["data"] == expected_wdata, (
            f"beat {i}: bad WDATA expected 0x{expected_wdata:016x}, got 0x{wb['data']:016x}"
        )
        assert wb["strb"] == expected_wstrb, (
            f"beat {i}: bad WSTRB expected 0x{expected_wstrb:02x}, got 0x{wb['strb']:02x}"
        )
        assert wb["last"] == (1 if i == 3 else 0), (
            f"beat {i}: bad WLAST expected {1 if i == 3 else 0}, got {wb['last']}"
        )

    expected_qwords = _expected_burst_qwords(initial_qwords, start_addr, beats, 1)
    for qaddr, expected in expected_qwords.items():
        got = axi_ram.read_qword(qaddr)
        assert got == expected, (
            f"qword @0x{qaddr:08x}: expected 0x{expected:016x}, got 0x{got:016x}"
        )


@cocotb.test()
async def test_incr4_halfword_write_burst_updates_memory(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x6202
    beats = [0x1111, 0x2222, 0x3333, 0x4444]

    initial_qwords = {
        0x6200: 0x0123456789ABCDEF,
        0x6208: 0x0F1E2D3C4B5A6978,
    }

    for addr, value in initial_qwords.items():
        axi_ram.write_qword(addr, value)

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 4))

    await ahb_write_inc_burst_manual(dut, start_addr, beats, size_bytes=2, fixed=True)
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w_beats = await w_task

    assert aw["addr"] == start_addr, f"bad AWADDR 0x{aw['addr']:08x}"
    assert aw["len"] == 3, f"expected AWLEN=3, got {aw['len']}"
    assert aw["size"] == 1, f"expected AWSIZE=1, got {aw['size']}"
    assert aw["burst"] == 1, f"expected AWBURST=INCR, got {aw['burst']}"

    for i, wb in enumerate(w_beats):
        addr = start_addr + i * 2
        expected_wdata = (beats[i] & 0xFFFF) << _lane_shift(addr)
        expected_wstrb = _strb_mask(addr, 2)

        assert wb["data"] == expected_wdata, (
            f"beat {i}: bad WDATA expected 0x{expected_wdata:016x}, got 0x{wb['data']:016x}"
        )
        assert wb["strb"] == expected_wstrb, (
            f"beat {i}: bad WSTRB expected 0x{expected_wstrb:02x}, got 0x{wb['strb']:02x}"
        )
        assert wb["last"] == (1 if i == 3 else 0), (
            f"beat {i}: bad WLAST expected {1 if i == 3 else 0}, got {wb['last']}"
        )

    expected_qwords = _expected_burst_qwords(initial_qwords, start_addr, beats, 2)
    for qaddr, expected in expected_qwords.items():
        got = axi_ram.read_qword(qaddr)
        assert got == expected, (
            f"qword @0x{qaddr:08x}: expected 0x{expected:016x}, got 0x{got:016x}"
        )


@cocotb.test()
async def test_incr4_byte_read_burst_returns_expected_lanes(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x6403
    base_qword_addr = 0x6400
    pattern = 0x8877665544332211
    axi_ram.write_qword(base_qword_addr, pattern)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))
    r_task = cocotb.start_soon(wait_for_n_r_handshakes(dut, 4))

    got_beats = await ahb_read_inc_burst_manual(dut, start_addr, 4, size_bytes=1, fixed=True)
    await ClockCycles(dut.clk, 2)

    ar = await ar_task
    r_beats = await r_task

    assert ar["addr"] == start_addr, f"bad ARADDR 0x{ar['addr']:08x}"
    assert ar["len"] == 3, f"expected ARLEN=3, got {ar['len']}"
    assert ar["size"] == 0, f"expected ARSIZE=0, got {ar['size']}"
    assert ar["burst"] == 1, f"expected ARBURST=INCR, got {ar['burst']}"

    for i, raw in enumerate(got_beats):
        addr = start_addr + i
        expected = _extract_subword(pattern, addr, 1)
        got = _extract_subword(raw, addr, 1)

        assert got == expected, (
            f"beat {i}: expected byte 0x{expected:02x}, got 0x{got:02x} "
            f"(raw HRDATA 0x{raw:016x})"
        )

    for i, rb in enumerate(r_beats):
        assert rb["resp"] == 0, f"beat {i}: expected RRESP=OKAY, got {rb['resp']}"
        assert rb["last"] == (1 if i == 3 else 0), (
            f"beat {i}: bad RLAST expected {1 if i == 3 else 0}, got {rb['last']}"
        )


@cocotb.test()
async def test_incr4_word_read_burst_returns_expected_lanes(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x6604
    initial_qwords = {
        0x6600: 0x1122334455667788,
        0x6608: 0x99AABBCCDDEEFF00,
        0x6610: 0x0123456789ABCDEF,
    }

    for addr, value in initial_qwords.items():
        axi_ram.write_qword(addr, value)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))
    r_task = cocotb.start_soon(wait_for_n_r_handshakes(dut, 4))

    got_beats = await ahb_read_inc_burst_manual(dut, start_addr, 4, size_bytes=4, fixed=True)
    await ClockCycles(dut.clk, 2)

    ar = await ar_task
    r_beats = await r_task

    assert ar["addr"] == start_addr, f"bad ARADDR 0x{ar['addr']:08x}"
    assert ar["len"] == 3, f"expected ARLEN=3, got {ar['len']}"
    assert ar["size"] == 2, f"expected ARSIZE=2, got {ar['size']}"
    assert ar["burst"] == 1, f"expected ARBURST=INCR, got {ar['burst']}"

    expected_subwords = []
    for i in range(4):
        addr = start_addr + i * 4
        qaddr = _qword_addr(addr)
        expected_subwords.append(_extract_subword(initial_qwords[qaddr], addr, 4))

    for i, raw in enumerate(got_beats):
        addr = start_addr + i * 4
        got = _extract_subword(raw, addr, 4)
        expected = expected_subwords[i]

        assert got == expected, (
            f"beat {i}: expected word 0x{expected:08x}, got 0x{got:08x} "
            f"(raw HRDATA 0x{raw:016x})"
        )

    for i, rb in enumerate(r_beats):
        assert rb["resp"] == 0, f"beat {i}: expected RRESP=OKAY, got {rb['resp']}"
        assert rb["last"] == (1 if i == 3 else 0), (
            f"beat {i}: bad RLAST expected {1 if i == 3 else 0}, got {rb['last']}"
        )

@cocotb.test()
async def test_wrap4_qword_write_burst_updates_axi_ram(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x7018
    beats = [
        0x1111111122222222,
        0x3333333344444444,
        0x5555555566666666,
        0x7777777788888888,
    ]

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 4))
    b_task = cocotb.start_soon(wait_for_b_handshake(dut))

    await ahb_write_wrap_burst_manual(dut, start_addr, beats, size_bytes=8)
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w_beats = await w_task
    await b_task
    await ClockCycles(dut.clk, 1)

    assert aw["addr"] == start_addr, f"bad AWADDR 0x{aw['addr']:08x}"
    assert aw["len"] == 3, f"expected AWLEN=3, got {aw['len']}"
    assert aw["size"] == 3, f"expected AWSIZE=3, got {aw['size']}"
    assert aw["burst"] == 2, f"expected AWBURST=WRAP(2), got {aw['burst']}"

    for i, wb in enumerate(w_beats):
        assert wb["data"] == beats[i], (
            f"beat {i}: bad WDATA expected 0x{beats[i]:016x}, got 0x{wb['data']:016x}"
        )
        assert wb["strb"] == 0xFF, (
            f"beat {i}: bad WSTRB expected 0xFF, got 0x{wb['strb']:02x}"
        )
        assert wb["last"] == (1 if i == 3 else 0), (
            f"beat {i}: bad WLAST expected {1 if i == 3 else 0}, got {wb['last']}"
        )

    expected_addrs = [_wrap_addr(start_addr, i, 8, 4) for i in range(4)]
    for i, expected in enumerate(beats):
        got = axi_ram.read_qword(expected_addrs[i])
        assert got == expected, (
            f"RAM beat {i} @0x{expected_addrs[i]:08x}: expected 0x{expected:016x}, got 0x{got:016x}"
        )


@cocotb.test()
async def test_wrap4_qword_read_burst_returns_expected(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x7118
    expected_beats = [
        0x0123456789ABCDEF,
        0x1111222233334444,
        0xAAAABBBBCCCCDDDD,
        0xDEADBEEFCAFEBABE,
    ]

    expected_addrs = [_wrap_addr(start_addr, i, 8, 4) for i in range(4)]
    for i, value in enumerate(expected_beats):
        axi_ram.write_qword(expected_addrs[i], value)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))
    r_task = cocotb.start_soon(wait_for_n_r_handshakes(dut, 4))

    got_beats = await ahb_read_wrap_burst_manual(dut, start_addr, 4, size_bytes=8)
    await ClockCycles(dut.clk, 2)

    ar = await ar_task
    r_beats = await r_task

    assert ar["addr"] == start_addr, f"bad ARADDR 0x{ar['addr']:08x}"
    assert ar["len"] == 3, f"expected ARLEN=3, got {ar['len']}"
    assert ar["size"] == 3, f"expected ARSIZE=3, got {ar['size']}"
    assert ar["burst"] == 2, f"expected ARBURST=WRAP(2), got {ar['burst']}"

    assert got_beats == expected_beats, (
        f"AHB read burst mismatch expected {expected_beats}, got {got_beats}"
    )

    for i, rb in enumerate(r_beats):
        assert rb["data"] == expected_beats[i], (
            f"R beat {i}: bad RDATA expected 0x{expected_beats[i]:016x}, got 0x{rb['data']:016x}"
        )
        assert rb["resp"] == 0, f"R beat {i}: expected RRESP=OKAY, got {rb['resp']}"
        assert rb["last"] == (1 if i == 3 else 0), (
            f"R beat {i}: bad RLAST expected {1 if i == 3 else 0}, got {rb['last']}"
        )


@cocotb.test()
async def test_wrap8_qword_write_burst_updates_axi_ram(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x7238
    beats = [0x1000000000000000 + i for i in range(8)]

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 8))
    b_task = cocotb.start_soon(wait_for_b_handshake(dut))

    await ahb_write_wrap_burst_manual(dut, start_addr, beats, size_bytes=8)
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w_beats = await w_task
    await b_task
    await ClockCycles(dut.clk, 1)

    assert aw["addr"] == start_addr, f"bad AWADDR 0x{aw['addr']:08x}"
    assert aw["len"] == 7, f"expected AWLEN=7, got {aw['len']}"
    assert aw["size"] == 3, f"expected AWSIZE=3, got {aw['size']}"
    assert aw["burst"] == 2, f"expected AWBURST=WRAP(2), got {aw['burst']}"

    for i, wb in enumerate(w_beats):
        assert wb["data"] == beats[i], (
            f"beat {i}: bad WDATA expected 0x{beats[i]:016x}, got 0x{wb['data']:016x}"
        )
        assert wb["strb"] == 0xFF, (
            f"beat {i}: bad WSTRB expected 0xFF, got 0x{wb['strb']:02x}"
        )
        assert wb["last"] == (1 if i == 7 else 0), (
            f"beat {i}: bad WLAST expected {1 if i == 7 else 0}, got {wb['last']}"
        )

    expected_addrs = [_wrap_addr(start_addr, i, 8, 8) for i in range(8)]
    for i, expected in enumerate(beats):
        got = axi_ram.read_qword(expected_addrs[i])
        assert got == expected, (
            f"RAM beat {i} @0x{expected_addrs[i]:08x}: expected 0x{expected:016x}, got 0x{got:016x}"
        )


@cocotb.test()
async def test_wrap8_qword_read_burst_returns_expected(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x7338
    expected_beats = [0xABCDEF0000000000 + i for i in range(8)]

    expected_addrs = [_wrap_addr(start_addr, i, 8, 8) for i in range(8)]
    for i, value in enumerate(expected_beats):
        axi_ram.write_qword(expected_addrs[i], value)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))
    r_task = cocotb.start_soon(wait_for_n_r_handshakes(dut, 8))

    got_beats = await ahb_read_wrap_burst_manual(dut, start_addr, 8, size_bytes=8)
    await ClockCycles(dut.clk, 2)

    ar = await ar_task
    r_beats = await r_task

    assert ar["addr"] == start_addr, f"bad ARADDR 0x{ar['addr']:08x}"
    assert ar["len"] == 7, f"expected ARLEN=7, got {ar['len']}"
    assert ar["size"] == 3, f"expected ARSIZE=3, got {ar['size']}"
    assert ar["burst"] == 2, f"expected ARBURST=WRAP(2), got {ar['burst']}"

    assert got_beats == expected_beats, (
        f"AHB read burst mismatch expected {expected_beats}, got {got_beats}"
    )

    for i, rb in enumerate(r_beats):
        assert rb["data"] == expected_beats[i], (
            f"R beat {i}: bad RDATA expected 0x{expected_beats[i]:016x}, got 0x{rb['data']:016x}"
        )
        assert rb["resp"] == 0, f"R beat {i}: expected RRESP=OKAY, got {rb['resp']}"
        assert rb["last"] == (1 if i == 7 else 0), (
            f"R beat {i}: bad RLAST expected {1 if i == 7 else 0}, got {rb['last']}"
        )


@cocotb.test()
async def test_wrap16_qword_write_burst_updates_axi_ram(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x7808
    beats = [0x9000000000000000 + i for i in range(16)]

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 16))
    b_task = cocotb.start_soon(wait_for_b_handshake(dut))

    await ahb_write_wrap_burst_manual(dut, start_addr, beats, size_bytes=8)
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w_beats = await w_task
    await b_task
    await ClockCycles(dut.clk, 1)

    assert aw["addr"] == start_addr, f"bad AWADDR 0x{aw['addr']:08x}"
    assert aw["len"] == 15, f"expected AWLEN=15, got {aw['len']}"
    assert aw["size"] == 3, f"expected AWSIZE=3, got {aw['size']}"
    assert aw["burst"] == 2, f"expected AWBURST=WRAP(2), got {aw['burst']}"

    for i, wb in enumerate(w_beats):
        assert wb["data"] == beats[i], (
            f"beat {i}: bad WDATA expected 0x{beats[i]:016x}, got 0x{wb['data']:016x}"
        )
        assert wb["strb"] == 0xFF, (
            f"beat {i}: bad WSTRB expected 0xFF, got 0x{wb['strb']:02x}"
        )
        assert wb["last"] == (1 if i == 15 else 0), (
            f"beat {i}: bad WLAST expected {1 if i == 15 else 0}, got {wb['last']}"
        )

    expected_addrs = [_wrap_addr(start_addr, i, 8, 16) for i in range(16)]
    for i, expected in enumerate(beats):
        got = axi_ram.read_qword(expected_addrs[i])
        assert got == expected, (
            f"RAM beat {i} @0x{expected_addrs[i]:08x}: expected 0x{expected:016x}, got 0x{got:016x}"
        )


@cocotb.test()
async def test_wrap16_qword_read_burst_returns_expected(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x7908
    expected_beats = [0xA000000000000000 + i for i in range(16)]

    expected_addrs = [_wrap_addr(start_addr, i, 8, 16) for i in range(16)]
    for i, value in enumerate(expected_beats):
        axi_ram.write_qword(expected_addrs[i], value)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))
    r_task = cocotb.start_soon(wait_for_n_r_handshakes(dut, 16))

    got_beats = await ahb_read_wrap_burst_manual(dut, start_addr, 16, size_bytes=8)
    await ClockCycles(dut.clk, 2)

    ar = await ar_task
    r_beats = await r_task

    assert ar["addr"] == start_addr, f"bad ARADDR 0x{ar['addr']:08x}"
    assert ar["len"] == 15, f"expected ARLEN=15, got {ar['len']}"
    assert ar["size"] == 3, f"expected ARSIZE=3, got {ar['size']}"
    assert ar["burst"] == 2, f"expected ARBURST=WRAP(2), got {ar['burst']}"

    assert got_beats == expected_beats, (
        f"AHB read burst mismatch expected {expected_beats}, got {got_beats}"
    )

    for i, rb in enumerate(r_beats):
        assert rb["data"] == expected_beats[i], (
            f"R beat {i}: bad RDATA expected 0x{expected_beats[i]:016x}, got 0x{rb['data']:016x}"
        )
        assert rb["resp"] == 0, f"R beat {i}: expected RRESP=OKAY, got {rb['resp']}"
        assert rb["last"] == (1 if i == 15 else 0), (
            f"R beat {i}: bad RLAST expected {1 if i == 15 else 0}, got {rb['last']}"
        )


@cocotb.test()
async def test_wrap4_byte_write_burst_updates_byte_lanes(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x7A07
    beats = [0x11, 0x22, 0x33, 0x44]

    initial_qwords = {
        0x7A00: 0xFFEEDDCCBBAA9988,
    }

    for addr, value in initial_qwords.items():
        axi_ram.write_qword(addr, value)

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 4))
    b_task = cocotb.start_soon(wait_for_b_handshake(dut))

    await ahb_write_wrap_burst_manual(dut, start_addr, beats, size_bytes=1)
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w_beats = await w_task
    await b_task
    await ClockCycles(dut.clk, 1)

    assert aw["addr"] == start_addr, f"bad AWADDR 0x{aw['addr']:08x}"
    assert aw["len"] == 3, f"expected AWLEN=3, got {aw['len']}"
    assert aw["size"] == 0, f"expected AWSIZE=0, got {aw['size']}"
    assert aw["burst"] == 2, f"expected AWBURST=WRAP(2), got {aw['burst']}"

    for i, wb in enumerate(w_beats):
        addr = _wrap_addr(start_addr, i, 1, 4)
        expected_wdata = (beats[i] & 0xFF) << _lane_shift(addr)
        expected_wstrb = _strb_mask(addr, 1)

        assert wb["data"] == expected_wdata, (
            f"beat {i}: bad WDATA expected 0x{expected_wdata:016x}, got 0x{wb['data']:016x}"
        )
        assert wb["strb"] == expected_wstrb, (
            f"beat {i}: bad WSTRB expected 0x{expected_wstrb:02x}, got 0x{wb['strb']:02x}"
        )
        assert wb["last"] == (1 if i == 3 else 0), (
            f"beat {i}: bad WLAST expected {1 if i == 3 else 0}, got {wb['last']}"
        )

    expected_qwords = _expected_wrap_qwords(initial_qwords, start_addr, beats, 1, 4)
    for qaddr, expected in expected_qwords.items():
        got = axi_ram.read_qword(qaddr)
        assert got == expected, (
            f"qword @0x{qaddr:08x}: expected 0x{expected:016x}, got 0x{got:016x}"
        )


@cocotb.test()
async def test_wrap4_byte_read_burst_returns_expected_lanes(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x7B07
    initial_qwords = {
        0x7B00: 0x8877665544332211,
    }

    for addr, value in initial_qwords.items():
        axi_ram.write_qword(addr, value)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))
    r_task = cocotb.start_soon(wait_for_n_r_handshakes(dut, 4))

    got_beats = await ahb_read_wrap_burst_manual(dut, start_addr, 4, size_bytes=1)
    await ClockCycles(dut.clk, 2)

    ar = await ar_task
    r_beats = await r_task

    assert ar["addr"] == start_addr, f"bad ARADDR 0x{ar['addr']:08x}"
    assert ar["len"] == 3, f"expected ARLEN=3, got {ar['len']}"
    assert ar["size"] == 0, f"expected ARSIZE=0, got {ar['size']}"
    assert ar["burst"] == 2, f"expected ARBURST=WRAP(2), got {ar['burst']}"

    expected_bytes = _expected_wrap_subwords(initial_qwords, start_addr, 1, 4)

    for i, raw in enumerate(got_beats):
        addr = _wrap_addr(start_addr, i, 1, 4)
        got = _extract_subword(raw, addr, 1)
        expected = expected_bytes[i]

        assert got == expected, (
            f"beat {i}: expected byte 0x{expected:02x}, got 0x{got:02x} "
            f"(raw HRDATA 0x{raw:016x})"
        )

    for i, rb in enumerate(r_beats):
        assert rb["resp"] == 0, f"beat {i}: expected RRESP=OKAY, got {rb['resp']}"
        assert rb["last"] == (1 if i == 3 else 0), (
            f"beat {i}: bad RLAST expected {1 if i == 3 else 0}, got {rb['last']}"
        )


@cocotb.test()
async def test_wrap4_halfword_write_burst_updates_memory(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x7C06
    beats = [0x1111, 0x2222, 0x3333, 0x4444]

    initial_qwords = {
        0x7C00: 0x0123456789ABCDEF,
    }

    for addr, value in initial_qwords.items():
        axi_ram.write_qword(addr, value)

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 4))
    b_task = cocotb.start_soon(wait_for_b_handshake(dut))

    await ahb_write_wrap_burst_manual(dut, start_addr, beats, size_bytes=2)
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w_beats = await w_task
    await b_task
    await ClockCycles(dut.clk, 1)

    assert aw["addr"] == start_addr, f"bad AWADDR 0x{aw['addr']:08x}"
    assert aw["len"] == 3, f"expected AWLEN=3, got {aw['len']}"
    assert aw["size"] == 1, f"expected AWSIZE=1, got {aw['size']}"
    assert aw["burst"] == 2, f"expected AWBURST=WRAP(2), got {aw['burst']}"

    for i, wb in enumerate(w_beats):
        addr = _wrap_addr(start_addr, i, 2, 4)
        expected_wdata = (beats[i] & 0xFFFF) << _lane_shift(addr)
        expected_wstrb = _strb_mask(addr, 2)

        assert wb["data"] == expected_wdata, (
            f"beat {i}: bad WDATA expected 0x{expected_wdata:016x}, got 0x{wb['data']:016x}"
        )
        assert wb["strb"] == expected_wstrb, (
            f"beat {i}: bad WSTRB expected 0x{expected_wstrb:02x}, got 0x{wb['strb']:02x}"
        )
        assert wb["last"] == (1 if i == 3 else 0), (
            f"beat {i}: bad WLAST expected {1 if i == 3 else 0}, got {wb['last']}"
        )

    expected_qwords = _expected_wrap_qwords(initial_qwords, start_addr, beats, 2, 4)
    for qaddr, expected in expected_qwords.items():
        got = axi_ram.read_qword(qaddr)
        assert got == expected, (
            f"qword @0x{qaddr:08x}: expected 0x{expected:016x}, got 0x{got:016x}"
        )


@cocotb.test()
async def test_wrap4_word_read_burst_returns_expected_lanes(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x7D0C
    initial_qwords = {
        0x7D00: 0x1122334455667788,
        0x7D08: 0x99AABBCCDDEEFF00,
    }

    for addr, value in initial_qwords.items():
        axi_ram.write_qword(addr, value)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))
    r_task = cocotb.start_soon(wait_for_n_r_handshakes(dut, 4))

    got_beats = await ahb_read_wrap_burst_manual(dut, start_addr, 4, size_bytes=4)
    await ClockCycles(dut.clk, 2)

    ar = await ar_task
    r_beats = await r_task

    assert ar["addr"] == start_addr, f"bad ARADDR 0x{ar['addr']:08x}"
    assert ar["len"] == 3, f"expected ARLEN=3, got {ar['len']}"
    assert ar["size"] == 2, f"expected ARSIZE=2, got {ar['size']}"
    assert ar["burst"] == 2, f"expected ARBURST=WRAP(2), got {ar['burst']}"

    expected_words = _expected_wrap_subwords(initial_qwords, start_addr, 4, 4)

    for i, raw in enumerate(got_beats):
        addr = _wrap_addr(start_addr, i, 4, 4)
        got = _extract_subword(raw, addr, 4)
        expected = expected_words[i]

        assert got == expected, (
            f"beat {i}: expected word 0x{expected:08x}, got 0x{got:08x} "
            f"(raw HRDATA 0x{raw:016x})"
        )

    for i, rb in enumerate(r_beats):
        assert rb["resp"] == 0, f"beat {i}: expected RRESP=OKAY, got {rb['resp']}"
        assert rb["last"] == (1 if i == 3 else 0), (
            f"beat {i}: bad RLAST expected {1 if i == 3 else 0}, got {rb['last']}"
        )

@cocotb.test()
async def test_incr4_qword_write_ends_exactly_at_1kb_boundary(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x83E0   # 4 * 8 bytes ends exactly at 0x8400 boundary
    beats = [
        0x1111111122222222,
        0x3333333344444444,
        0x5555555566666666,
        0x7777777788888888,
    ]

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 4))
    b_task = cocotb.start_soon(wait_for_b_handshake(dut))

    await ahb_write_inc_burst_manual(dut, start_addr, beats, size_bytes=8, fixed=True)
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w_beats = await w_task
    await b_task
    await ClockCycles(dut.clk, 1)

    assert aw["addr"] == start_addr
    assert aw["len"] == 3
    assert aw["size"] == 3
    assert aw["burst"] == 1

    for i, wb in enumerate(w_beats):
        assert wb["data"] == beats[i]
        assert wb["strb"] == 0xFF
        assert wb["last"] == (1 if i == 3 else 0)

    for i, expected in enumerate(beats):
        got = axi_ram.read_qword(start_addr + i * 8)
        assert got == expected, (
            f"RAM beat {i}: expected 0x{expected:016x}, got 0x{got:016x}"
        )


@cocotb.test()
async def test_incr4_qword_read_ends_exactly_at_1kb_boundary(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x85E0
    expected_beats = [
        0x0123456789ABCDEF,
        0x1111222233334444,
        0xAAAABBBBCCCCDDDD,
        0xDEADBEEFCAFEBABE,
    ]

    for i, value in enumerate(expected_beats):
        axi_ram.write_qword(start_addr + i * 8, value)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))
    r_task = cocotb.start_soon(wait_for_n_r_handshakes(dut, 4))

    got_beats = await ahb_read_inc_burst_manual(dut, start_addr, 4, size_bytes=8, fixed=True)
    await ClockCycles(dut.clk, 2)

    ar = await ar_task
    r_beats = await r_task

    assert ar["addr"] == start_addr
    assert ar["len"] == 3
    assert ar["size"] == 3
    assert ar["burst"] == 1

    assert got_beats == expected_beats

    for i, rb in enumerate(r_beats):
        assert rb["data"] == expected_beats[i]
        assert rb["resp"] == 0
        assert rb["last"] == (1 if i == 3 else 0)


@cocotb.test()
async def test_incr_len4_qword_write_ends_exactly_at_1kb_boundary(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x87E0
    beats = [
        0x1010101010101010,
        0x2020202020202020,
        0x3030303030303030,
        0x4040404040404040,
    ]

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 4))
    b_task = cocotb.start_soon(wait_for_b_handshake(dut))

    await ahb_write_inc_burst_manual(dut, start_addr, beats, size_bytes=8, fixed=False)
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w_beats = await w_task
    await b_task
    await ClockCycles(dut.clk, 1)

    assert aw["addr"] == start_addr
    assert aw["len"] == 3
    assert aw["size"] == 3
    assert aw["burst"] == 1

    for i, wb in enumerate(w_beats):
        assert wb["data"] == beats[i]
        assert wb["strb"] == 0xFF
        assert wb["last"] == (1 if i == 3 else 0)

    for i, expected in enumerate(beats):
        got = axi_ram.read_qword(start_addr + i * 8)
        assert got == expected, (
            f"RAM beat {i}: expected 0x{expected:016x}, got 0x{got:016x}"
        )


@cocotb.test()
async def test_incr_len4_qword_read_ends_exactly_at_1kb_boundary(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0x89E0
    expected_beats = [
        0x1111111122222222,
        0x3333333344444444,
        0x5555555566666666,
        0x7777777788888888,
    ]

    for i, value in enumerate(expected_beats):
        axi_ram.write_qword(start_addr + i * 8, value)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))

    got_beats = await ahb_read_inc_burst_manual(dut, start_addr, 4, size_bytes=8, fixed=False)
    await ClockCycles(dut.clk, 20)

    ar = await ar_task

    assert ar["addr"] == start_addr
    # For bare INCR reads, ARLEN depends on chunk size and 4 KB boundary,
    # not on the number of AHB beats.
    assert ar["len"] >= 3, f"expected ARLEN>=3, got {ar['len']}"
    assert ar["size"] == 3
    assert ar["burst"] == 1

    assert got_beats == expected_beats


@cocotb.test()
async def test_reset_during_buffered_incr_write_clears_bus(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    beats = [
        0x1111111122222222,
        0x3333333344444444,
        0x5555555566666666,
        0x7777777788888888,
    ]

    await ahb_start_partial_incr_write(
        dut,
        start_addr=0x8C00,
        data_beats=beats,
        beats_to_issue=2,
        size_bytes=8,
    )

    # Buffered INCR should not have launched AXI yet
    assert int(dut.m_axi_awvalid.value) == 0
    assert int(dut.m_axi_wvalid.value) == 0
    assert int(dut.m_axi_arvalid.value) == 0

    dut.resetn.value = 0
    await ClockCycles(dut.clk, 2)
    dut.resetn.value = 1

    _init_direct_ahb_signals(dut)
    await wait_until_all_axi_outputs_idle(dut, max_cycles=20)

    assert int(dut.hresp.value) == 0


@cocotb.test()
async def test_reset_during_pending_incr_read_clears_bus(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    task = cocotb.start_soon(
        ahb_start_partial_incr_read(
            dut,
            start_addr=0x8E00,
            beats_to_issue=2,
            size_bytes=8,
        )
    )

    # Read path should launch AXI AR, then stall because there is no AXI slave
    await wait_until_any_axi_valid(dut, max_cycles=20)

    dut.resetn.value = 0
    await ClockCycles(dut.clk, 2)
    dut.resetn.value = 1

    task.kill()
    _init_direct_ahb_signals(dut)
    _init_manual_axi_slave_inputs(dut)

    await wait_until_all_axi_outputs_idle(dut, max_cycles=20)
    assert int(dut.hresp.value) == 0


@cocotb.test()
async def test_wrap4_qword_write_burst_stalls_under_axi_backpressure(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    set_axi_ram_backpressure(
        axi_ram,
        stall_aw=True,
        stall_w=True,
        stall_b=True,
    )

    start_addr = 0x9008
    beats = [
        0x1111111122222222,
        0x3333333344444444,
        0x5555555566666666,
        0x7777777788888888,
    ]

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 4))

    _, hready_low_cycles, _ = await run_and_watch_hready(
        dut,
        ahb_write_wrap_burst_manual(dut, start_addr, beats, size_bytes=8)
    )
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w_beats = await w_task

    assert hready_low_cycles > 0, "Expected at least one AHB wait-state"
    assert aw["addr"] == start_addr
    assert aw["len"] == 3
    assert aw["size"] == 3
    assert aw["burst"] == 2

    for i, wb in enumerate(w_beats):
        assert wb["data"] == beats[i]
        assert wb["strb"] == 0xFF
        assert wb["last"] == (1 if i == 3 else 0)


@cocotb.test()
async def test_wrap4_qword_read_burst_stalls_under_axi_backpressure(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    set_axi_ram_backpressure(
        axi_ram,
        stall_ar=True,
        stall_r=True,
    )

    start_addr = 0x9118
    expected_beats = [
        0x0123456789ABCDEF,
        0x1111222233334444,
        0xAAAABBBBCCCCDDDD,
        0xDEADBEEFCAFEBABE,
    ]

    expected_addrs = [_wrap_addr(start_addr, i, 8, 4) for i in range(4)]
    for i, value in enumerate(expected_beats):
        axi_ram.write_qword(expected_addrs[i], value)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))

    got_beats, hready_low_cycles, _ = await run_and_watch_hready(
        dut,
        ahb_read_wrap_burst_manual(dut, start_addr, 4, size_bytes=8)
    )
    await ClockCycles(dut.clk, 2)

    ar = await ar_task

    assert hready_low_cycles > 0, "Expected at least one AHB wait-state"
    assert ar["addr"] == start_addr
    assert ar["len"] == 3
    assert ar["size"] == 3
    assert ar["burst"] == 2
    assert got_beats == expected_beats


@cocotb.test()
async def test_reset_during_wrap4_write_clears_bus(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    beats = [
        0x1010101010101010,
        0x2020202020202020,
        0x3030303030303030,
        0x4040404040404040,
    ]

    task = cocotb.start_soon(
        ahb_write_wrap_burst_manual(dut, 0x9208, beats, size_bytes=8)
    )

    await wait_until_any_axi_valid(dut, max_cycles=20)
    dut.resetn.value = 0
    await ClockCycles(dut.clk, 2)
    dut.resetn.value = 1

    task.kill()
    _init_direct_ahb_signals(dut)
    _init_manual_axi_slave_inputs(dut)

    await wait_until_all_axi_outputs_idle(dut, max_cycles=20)
    assert int(dut.hresp.value) == 0


@cocotb.test()
async def test_reset_during_wrap4_read_clears_bus(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    task = cocotb.start_soon(
        ahb_read_wrap_burst_manual(dut, 0x9318, 4, size_bytes=8)
    )

    await wait_until_any_axi_valid(dut, max_cycles=20)
    dut.resetn.value = 0
    await ClockCycles(dut.clk, 2)
    dut.resetn.value = 1

    task.kill()
    _init_direct_ahb_signals(dut)
    _init_manual_axi_slave_inputs(dut)

    await wait_until_all_axi_outputs_idle(dut, max_cycles=20)
    assert int(dut.hresp.value) == 0


@cocotb.test()
async def test_hmastlock_maps_to_awlock_on_wrap4_write(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    start_addr = 0x9408
    beats = [
        0x1111111122222222,
        0x3333333344444444,
        0x5555555566666666,
        0x7777777788888888,
    ]

    dut.m_axi_awready.value = 1
    dut.m_axi_wready.value = 1
    dut.m_axi_bid.value = 0
    dut.m_axi_bresp.value = 0
    dut.m_axi_bvalid.value = 0

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 4))

    await ahb_write_wrap_burst_manual_with_attrs(
        dut, start_addr, beats, size_bytes=8, hmastlock=1
    )

    aw_info = await aw_task
    w_beats = await w_task

    dut.m_axi_bvalid.value = 1
    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_bvalid.value) and int(dut.m_axi_bready.value):
            break
    dut.m_axi_bvalid.value = 0

    assert aw_info["addr"] == start_addr
    assert aw_info["len"] == 3
    assert aw_info["lock"] == 1

    for i, wb in enumerate(w_beats):
        assert wb["data"] == beats[i]
        assert wb["strb"] == 0xFF
        assert wb["last"] == (1 if i == 3 else 0)


@cocotb.test()
async def test_hmastlock_maps_to_arlock_on_wrap4_read(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    start_addr = 0x9518
    beats = [
        0x0123456789ABCDEF,
        0x1111222233334444,
        0xAAAABBBBCCCCDDDD,
        0xDEADBEEFCAFEBABE,
    ]

    dut.m_axi_arready.value = 1
    dut.m_axi_rvalid.value = 0
    dut.m_axi_rresp.value = 0
    dut.m_axi_rlast.value = 0
    dut.m_axi_rid.value = 0

    ar_info = None

    async def slave():
        nonlocal ar_info

        while ar_info is None:
            await RisingEdge(dut.clk)
            if int(dut.m_axi_arvalid.value) and int(dut.m_axi_arready.value):
                ar_info = {
                    "addr": int(dut.m_axi_araddr.value),
                    "len": int(dut.m_axi_arlen.value),
                    "lock": int(dut.m_axi_arlock.value),
                }

        for i, value in enumerate(beats):
            dut.m_axi_rdata.value = value
            dut.m_axi_rresp.value = 0
            dut.m_axi_rlast.value = 1 if i == 3 else 0
            dut.m_axi_rvalid.value = 1

            while True:
                await RisingEdge(dut.clk)
                if int(dut.m_axi_rvalid.value) and int(dut.m_axi_rready.value):
                    break

            dut.m_axi_rvalid.value = 0
            dut.m_axi_rlast.value = 0
            await RisingEdge(dut.clk)

    slave_task = cocotb.start_soon(slave())
    _ = await ahb_read_wrap_burst_manual_with_attrs(
        dut, start_addr, 4, size_bytes=8, hmastlock=1
    )
    await slave_task

    assert ar_info["addr"] == start_addr
    assert ar_info["len"] == 3
    assert ar_info["lock"] == 1



@cocotb.test()
async def test_wrap4_write_burst_bresp_slverr_propagates(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    start_addr = 0xA008
    beats = [
        0x1111111122222222,
        0x3333333344444444,
        0x5555555566666666,
        0x7777777788888888,
    ]

    # Reuse the existing INCR4 write responder: WRAP/INCR difference is only AWBURST/AWADDR shape,
    # W/B channel behavior is the same for this error propagation check.
    slave_task = cocotb.start_soon(
        axi_slave_respond_incr4_write_burst(dut, bresp=0b10, b_delay=2)
    )

    final_resp = await ahb_write_wrap_burst_manual_status(dut, start_addr, beats, size_bytes=8)
    info = await slave_task

    assert info["aw"]["addr"] == start_addr
    assert info["aw"]["len"] == 3
    assert info["aw"]["size"] == 3
    assert info["aw"]["burst"] == 2, f"expected AWBURST=WRAP(2), got {info['aw']['burst']}"

    for i, wb in enumerate(info["w_beats"]):
        assert wb["data"] == beats[i], (
            f"beat {i}: bad WDATA expected 0x{beats[i]:016x}, got 0x{wb['data']:016x}"
        )
        assert wb["strb"] == 0xFF
        assert wb["last"] == (1 if i == 3 else 0)

    assert final_resp == 1, f"expected final HRESP=ERROR(1), got {final_resp}"


@cocotb.test()
async def test_wrap4_write_burst_bresp_decerr_propagates(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    start_addr = 0xA108
    beats = [
        0x0101010102020202,
        0x0303030304040404,
        0x0505050506060606,
        0x0707070708080808,
    ]

    slave_task = cocotb.start_soon(
        axi_slave_respond_incr4_write_burst(dut, bresp=0b11, b_delay=2)
    )

    final_resp = await ahb_write_wrap_burst_manual_status(dut, start_addr, beats, size_bytes=8)
    info = await slave_task

    assert info["aw"]["addr"] == start_addr
    assert info["aw"]["len"] == 3
    assert info["aw"]["size"] == 3
    assert info["aw"]["burst"] == 2, f"expected AWBURST=WRAP(2), got {info['aw']['burst']}"
    assert final_resp == 1, f"expected final HRESP=ERROR(1), got {final_resp}"

# @cocotb.test()
# async def test_wrap4_read_burst_rresp_error_on_last_beat(dut):
#     set_test_id(dut)
#     await setup_dut_no_axi_slave(dut)

#     start_addr = 0xA218
#     beats = [
#         0x0123456789ABCDEF,
#         0x1111222233334444,
#         0xAAAABBBBCCCCDDDD,
#         0xDEADBEEFCAFEBABE,
#     ]

#     slave_task = cocotb.start_soon(
#         axi_slave_respond_incr4_read_burst(
#             dut,
#             beats,
#             error_index=3,
#             error_resp=0b10,
#             r_gap=1,
#         )
#     )

#     samples = await ahb_read_wrap_burst_manual(dut, start_addr, 4, size_bytes=8)
#     info = await slave_task

#     assert info["ar"]["addr"] == start_addr
#     assert info["ar"]["len"] == 3
#     assert info["ar"]["size"] == 3
#     assert info["ar"]["burst"] == 2, f"expected ARBURST=WRAP(2), got {info['ar']['burst']}"

#     # We only require the final returned beat to show the error.
#     # samples are raw HRDATA values from the AHB side helper.
#     # Use direct HRESP observation in a stricter checker later if you want protocol-shape validation.
#     assert len(samples) == 4


async def ahb_read_wrap_burst_manual_status(dut, start_addr, nbeats, size_bytes=8):
    set_test_id(dut)
    assert nbeats in (4, 8, 16)
    assert size_bytes in (1, 2, 4, 8)

    hsize = _size_bytes_to_axsize(size_bytes)
    hburst = _hburst_wrap_code(nbeats)
    samples = []

    first_addr = _wrap_addr(start_addr, 0, size_bytes, nbeats)
    dut.haddr.value = first_addr
    dut.hburst.value = hburst
    dut.hmastlock.value = 0
    dut.hprot.value = 0
    dut.hsize.value = hsize
    dut.htrans.value = 0b10
    dut.hwrite.value = 0
    dut.hwdata.value = 0

    for i in range(nbeats):
        await _wait_hready_high(dut)

        if i > 0:
            samples.append({
                "data": int(dut.hrdata.value),
                "resp": int(dut.hresp.value),
            })

        if i == nbeats - 1:
            dut.haddr.value = 0
            dut.hburst.value = 0
            dut.hsize.value = 0
            dut.htrans.value = 0
            dut.hwrite.value = 0
        else:
            next_addr = _wrap_addr(start_addr, i + 1, size_bytes, nbeats)
            dut.haddr.value = next_addr
            dut.hburst.value = hburst
            dut.hsize.value = hsize
            dut.htrans.value = 0b11
            dut.hwrite.value = 0

    await _wait_hready_high(dut)
    samples.append({
        "data": int(dut.hrdata.value),
        "resp": int(dut.hresp.value),
    })

    _init_direct_ahb_signals(dut)
    await RisingEdge(dut.clk)
    return samples


@cocotb.test()
async def test_wrap4_read_burst_rresp_error_on_last_beat(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    start_addr = 0xA218
    beats = [
        0x0123456789ABCDEF,
        0x1111222233334444,
        0xAAAABBBBCCCCDDDD,
        0xDEADBEEFCAFEBABE,
    ]

    slave_task = cocotb.start_soon(
        axi_slave_respond_incr4_read_burst(
            dut,
            beats,
            error_index=3,
            error_resp=0b10,
            r_gap=1,
        )
    )

    samples = await ahb_read_wrap_burst_manual_status(dut, start_addr, 4, size_bytes=8)
    info = await slave_task

    assert info["ar"]["addr"] == start_addr
    assert info["ar"]["len"] == 3
    assert info["ar"]["size"] == 3
    assert info["ar"]["burst"] == 2, f"expected ARBURST=WRAP(2), got {info['ar']['burst']}"

    assert len(samples) == 4
    assert samples[0]["resp"] == 0
    assert samples[1]["resp"] == 0
    assert samples[2]["resp"] == 0
    assert samples[3]["resp"] == 1, f"expected last beat HRESP=ERROR(1), got {samples[3]['resp']}"


@cocotb.test()
async def test_wrap4_read_burst_rresp_error_on_middle_beat(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    start_addr = 0xA318
    beats = [
        0x1111111122222222,
        0x3333333344444444,
        0x5555555566666666,
        0x7777777788888888,
    ]

    slave_task = cocotb.start_soon(
        axi_slave_respond_incr4_read_burst(
            dut,
            beats,
            error_index=1,
            error_resp=0b10,
            r_gap=1,
        )
    )

    samples = await ahb_read_wrap_burst_manual_status(dut, start_addr, 4, size_bytes=8)
    info = await slave_task

    assert info["ar"]["addr"] == start_addr
    assert info["ar"]["len"] == 3
    assert info["ar"]["size"] == 3
    assert info["ar"]["burst"] == 2, f"expected ARBURST=WRAP(2), got {info['ar']['burst']}"

    assert len(samples) == 4
    assert samples[0]["resp"] == 0
    assert samples[1]["resp"] == 1, f"expected beat1 HRESP=ERROR(1), got {samples[1]['resp']}"

    dut._log.info(
        f"Post-error WRAP4 read behavior: beat2 resp={samples[2]['resp']} beat3 resp={samples[3]['resp']}"
    )

@cocotb.test()
async def test_wrap8_qword_write_burst_stalls_under_axi_backpressure(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    set_axi_ram_backpressure(
        axi_ram,
        stall_aw=True,
        stall_w=True,
        stall_b=True,
    )

    start_addr = 0xA408
    beats = [0x1000000000000000 + i for i in range(8)]

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 8))

    _, hready_low_cycles, _ = await run_and_watch_hready(
        dut,
        ahb_write_wrap_burst_manual(dut, start_addr, beats, size_bytes=8)
    )
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w_beats = await w_task

    assert hready_low_cycles > 0, "expected at least one AHB wait-state"
    assert aw["addr"] == start_addr
    assert aw["len"] == 7
    assert aw["size"] == 3
    assert aw["burst"] == 2

    for i, wb in enumerate(w_beats):
        assert wb["data"] == beats[i]
        assert wb["strb"] == 0xFF
        assert wb["last"] == (1 if i == 7 else 0)


@cocotb.test()
async def test_wrap8_qword_read_burst_stalls_under_axi_backpressure(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    set_axi_ram_backpressure(
        axi_ram,
        stall_ar=True,
        stall_r=True,
    )

    start_addr = 0xA518
    expected_beats = [0x2000000000000000 + i for i in range(8)]

    expected_addrs = [_wrap_addr(start_addr, i, 8, 8) for i in range(8)]
    for i, value in enumerate(expected_beats):
        axi_ram.write_qword(expected_addrs[i], value)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))

    got_beats, hready_low_cycles, _ = await run_and_watch_hready(
        dut,
        ahb_read_wrap_burst_manual(dut, start_addr, 8, size_bytes=8)
    )
    await ClockCycles(dut.clk, 2)

    ar = await ar_task

    assert hready_low_cycles > 0, "expected at least one AHB wait-state"
    assert ar["addr"] == start_addr
    assert ar["len"] == 7
    assert ar["size"] == 3
    assert ar["burst"] == 2
    assert got_beats == expected_beats


@cocotb.test()
async def test_wrap16_qword_write_burst_stalls_under_axi_backpressure(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    set_axi_ram_backpressure(
        axi_ram,
        stall_aw=True,
        stall_w=True,
        stall_b=True,
    )

    start_addr = 0xA608
    beats = [0x3000000000000000 + i for i in range(16)]

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 16))

    _, hready_low_cycles, _ = await run_and_watch_hready(
        dut,
        ahb_write_wrap_burst_manual(dut, start_addr, beats, size_bytes=8)
    )
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w_beats = await w_task

    assert hready_low_cycles > 0, "expected at least one AHB wait-state"
    assert aw["addr"] == start_addr
    assert aw["len"] == 15
    assert aw["size"] == 3
    assert aw["burst"] == 2

    for i, wb in enumerate(w_beats):
        assert wb["data"] == beats[i]
        assert wb["strb"] == 0xFF
        assert wb["last"] == (1 if i == 15 else 0)


@cocotb.test()
async def test_wrap16_qword_read_burst_stalls_under_axi_backpressure(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    set_axi_ram_backpressure(
        axi_ram,
        stall_ar=True,
        stall_r=True,
    )

    start_addr = 0xA718
    expected_beats = [0x4000000000000000 + i for i in range(16)]

    expected_addrs = [_wrap_addr(start_addr, i, 8, 16) for i in range(16)]
    for i, value in enumerate(expected_beats):
        axi_ram.write_qword(expected_addrs[i], value)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))

    got_beats, hready_low_cycles, _ = await run_and_watch_hready(
        dut,
        ahb_read_wrap_burst_manual(dut, start_addr, 16, size_bytes=8)
    )
    await ClockCycles(dut.clk, 2)

    ar = await ar_task

    assert hready_low_cycles > 0, "expected at least one AHB wait-state"
    assert ar["addr"] == start_addr
    assert ar["len"] == 15
    assert ar["size"] == 3
    assert ar["burst"] == 2
    assert got_beats == expected_beats


@cocotb.test()
async def test_wrap8_write_burst_bresp_slverr_propagates(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    start_addr = 0xB008
    beats = [0x1000000000000000 + i for i in range(8)]

    slave_task = cocotb.start_soon(
        axi_slave_respond_write_burst(dut, 8, bresp=0b10, b_delay=2)
    )

    final_resp = await ahb_write_wrap_burst_manual_status(dut, start_addr, beats, size_bytes=8)
    info = await slave_task

    assert info["aw"]["addr"] == start_addr
    assert info["aw"]["len"] == 7
    assert info["aw"]["size"] == 3
    assert info["aw"]["burst"] == 2
    assert final_resp == 1


@cocotb.test()
async def test_wrap8_write_burst_bresp_decerr_propagates(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    start_addr = 0xB108
    beats = [0x2000000000000000 + i for i in range(8)]

    slave_task = cocotb.start_soon(
        axi_slave_respond_write_burst(dut, 8, bresp=0b11, b_delay=2)
    )

    final_resp = await ahb_write_wrap_burst_manual_status(dut, start_addr, beats, size_bytes=8)
    info = await slave_task

    assert info["aw"]["addr"] == start_addr
    assert info["aw"]["len"] == 7
    assert info["aw"]["size"] == 3
    assert info["aw"]["burst"] == 2
    assert final_resp == 1


@cocotb.test()
async def test_wrap8_read_burst_rresp_error_on_last_beat(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    start_addr = 0xB218
    beats = [0x3000000000000000 + i for i in range(8)]

    slave_task = cocotb.start_soon(
        axi_slave_respond_read_burst(dut, beats, error_index=7, error_resp=0b10, r_gap=1)
    )

    samples = await ahb_read_wrap_burst_manual_status(dut, start_addr, 8, size_bytes=8)
    info = await slave_task

    assert info["ar"]["addr"] == start_addr
    assert info["ar"]["len"] == 7
    assert info["ar"]["size"] == 3
    assert info["ar"]["burst"] == 2

    assert len(samples) == 8
    assert all(s["resp"] == 0 for s in samples[:-1])
    assert samples[-1]["resp"] == 1


# @cocotb.test()
# async def test_wrap8_read_burst_rresp_error_on_middle_beat(dut):
#     set_test_id(dut)
#     await setup_dut_no_axi_slave(dut)

#     start_addr = 0xB318
#     beats = [0x4000000000000000 + i for i in range(8)]

#     slave_task = cocotb.start_soon(
#         axi_slave_respond_read_burst(dut, beats, error_index=3, error_resp=0b10, r_gap=1)
#     )

#     samples = await ahb_read_wrap_burst_manual_status(dut, start_addr, 8, size_bytes=8)
#     info = await slave_task

#     assert info["ar"]["addr"] == start_addr
#     assert info["ar"]["len"] == 7
#     assert info["ar"]["size"] == 3
#     assert info["ar"]["burst"] == 2

#     assert len(samples) == 8
#     assert samples[0]["resp"] == 0
#     assert samples[1]["resp"] == 0
#     assert samples[2]["resp"] == 0
#     assert samples[3]["resp"] == 1


# @cocotb.test()
# async def test_wrap16_write_burst_bresp_slverr_propagates(dut):
#     set_test_id(dut)
#     await setup_dut_no_axi_slave(dut)

#     start_addr = 0xB408
#     beats = [0x5000000000000000 + i for i in range(16)]

#     slave_task = cocotb.start_soon(
#         axi_slave_respond_write_burst(dut, 16, bresp=0b10, b_delay=2)
#     )

#     final_resp = await ahb_write_wrap_burst_manual_status(dut, start_addr, beats, size_bytes=8)
#     info = await slave_task

#     assert info["aw"]["addr"] == start_addr
#     assert info["aw"]["len"] == 15
#     assert info["aw"]["size"] == 3
#     assert info["aw"]["burst"] == 2
#     assert final_resp == 1


@cocotb.test()
async def test_wrap16_read_burst_rresp_error_on_last_beat(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    start_addr = 0xB518
    beats = [0x6000000000000000 + i for i in range(16)]

    slave_task = cocotb.start_soon(
        axi_slave_respond_read_burst(dut, beats, error_index=15, error_resp=0b10, r_gap=1)
    )

    samples = await ahb_read_wrap_burst_manual_status(dut, start_addr, 16, size_bytes=8)
    info = await slave_task

    assert info["ar"]["addr"] == start_addr
    assert info["ar"]["len"] == 15
    assert info["ar"]["size"] == 3
    assert info["ar"]["burst"] == 2

    assert len(samples) == 16
    assert all(s["resp"] == 0 for s in samples[:-1])
    assert samples[-1]["resp"] == 1


# @cocotb.test()
# async def test_wrap16_byte_write_burst_updates_byte_lanes(dut):
#     set_test_id(dut)
#     _, axi_ram = await setup_dut(dut, with_ahb_master=False)

#     start_addr = 0xB607
#     beats = [i + 0x10 for i in range(16)]

#     initial_qwords = {
#         0xB600: 0xFFEEDDCCBBAA9988,
#         0xB608: 0x7766554433221100,
#     }

#     for addr, value in initial_qwords.items():
#         axi_ram.write_qword(addr, value)

#     aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
#     w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 16))

#     await ahb_write_wrap_burst_manual(dut, start_addr, beats, size_bytes=1)
#     await ClockCycles(dut.clk, 2)

#     aw = await aw_task
#     w_beats = await w_task

#     assert aw["addr"] == start_addr
#     assert aw["len"] == 15
#     assert aw["size"] == 0
#     assert aw["burst"] == 2

#     for i, wb in enumerate(w_beats):
#         addr = _wrap_addr(start_addr, i, 1, 16)
#         expected_wdata = (beats[i] & 0xFF) << _lane_shift(addr)
#         expected_wstrb = _strb_mask(addr, 1)

#         assert wb["data"] == expected_wdata
#         assert wb["strb"] == expected_wstrb
#         assert wb["last"] == (1 if i == 15 else 0)

#     expected_qwords = _expected_wrap_qwords(initial_qwords, start_addr, beats, 1, 16)
#     for qaddr, expected in expected_qwords.items():
#         got = axi_ram.read_qword(qaddr)
#         assert got == expected


# @cocotb.test()
# async def test_wrap16_halfword_read_burst_returns_expected_lanes(dut):
#     set_test_id(dut)
#     _, axi_ram = await setup_dut(dut, with_ahb_master=False)

#     start_addr = 0xB706
#     initial_qwords = {
#         0xB700: 0x0123456789ABCDEF,
#         0xB708: 0x1122334455667788,
#         0xB710: 0x99AABBCCDDEEFF00,
#         0xB718: 0x0F1E2D3C4B5A6978,
#     }

#     for addr, value in initial_qwords.items():
#         axi_ram.write_qword(addr, value)

#     ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))
#     r_task = cocotb.start_soon(wait_for_n_r_handshakes(dut, 16))

#     got_beats = await ahb_read_wrap_burst_manual(dut, start_addr, 16, size_bytes=2)
#     await ClockCycles(dut.clk, 2)

#     ar = await ar_task
#     r_beats = await r_task

#     assert ar["addr"] == start_addr
#     assert ar["len"] == 15
#     assert ar["size"] == 1
#     assert ar["burst"] == 2

#     expected_halfwords = _expected_wrap_subwords(initial_qwords, start_addr, 2, 16)

#     for i, raw in enumerate(got_beats):
#         addr = _wrap_addr(start_addr, i, 2, 16)
#         got = _extract_subword(raw, addr, 2)
#         expected = expected_halfwords[i]
#         assert got == expected

#     for i, rb in enumerate(r_beats):
#         assert rb["resp"] == 0
#         assert rb["last"] == (1 if i == 15 else 0)


# ----------------------------------------------------------------------
# Generic AXI manual responders for larger-burst error tests
# ----------------------------------------------------------------------

async def axi_slave_respond_write_burst(
    dut,
    nbeats,
    *,
    bresp=0,
    aw_stall=0,
    w_stall_each=0,
    b_delay=0,
):
    aw = {}
    w_beats = []

    dut.m_axi_awready.value = 0
    dut.m_axi_wready.value = 0
    dut.m_axi_bvalid.value = 0
    dut.m_axi_bresp.value = 0
    dut.m_axi_bid.value = 0

    while not int(dut.m_axi_awvalid.value):
        await RisingEdge(dut.clk)

    for _ in range(aw_stall):
        await RisingEdge(dut.clk)

    dut.m_axi_awready.value = 1
    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_awvalid.value) and int(dut.m_axi_awready.value):
            aw = {
                "addr": int(dut.m_axi_awaddr.value),
                "len": int(dut.m_axi_awlen.value),
                "size": int(dut.m_axi_awsize.value),
                "burst": int(dut.m_axi_awburst.value),
            }
            break
    dut.m_axi_awready.value = 0

    for _ in range(nbeats):
        while not int(dut.m_axi_wvalid.value):
            await RisingEdge(dut.clk)

        for _ in range(w_stall_each):
            await RisingEdge(dut.clk)

        dut.m_axi_wready.value = 1
        while True:
            await RisingEdge(dut.clk)
            if int(dut.m_axi_wvalid.value) and int(dut.m_axi_wready.value):
                w_beats.append({
                    "data": int(dut.m_axi_wdata.value),
                    "strb": int(dut.m_axi_wstrb.value),
                    "last": int(dut.m_axi_wlast.value),
                })
                break
        dut.m_axi_wready.value = 0

    for _ in range(b_delay):
        await RisingEdge(dut.clk)

    dut.m_axi_bid.value = 0
    dut.m_axi_bresp.value = bresp
    dut.m_axi_bvalid.value = 1
    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_bvalid.value) and int(dut.m_axi_bready.value):
            break

    dut.m_axi_bvalid.value = 0
    dut.m_axi_bresp.value = 0
    return {"aw": aw, "w_beats": w_beats}


async def axi_slave_respond_read_burst(
    dut,
    beats,
    *,
    error_index=None,
    error_resp=0b10,
    ar_stall=0,
    r_gap=0,
):
    ar = {}

    dut.m_axi_arready.value = 0
    dut.m_axi_rvalid.value = 0
    dut.m_axi_rdata.value = 0
    dut.m_axi_rresp.value = 0
    dut.m_axi_rlast.value = 0
    dut.m_axi_rid.value = 0

    while not int(dut.m_axi_arvalid.value):
        await RisingEdge(dut.clk)

    for _ in range(ar_stall):
        await RisingEdge(dut.clk)

    dut.m_axi_arready.value = 1
    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_arvalid.value) and int(dut.m_axi_arready.value):
            ar = {
                "addr": int(dut.m_axi_araddr.value),
                "len": int(dut.m_axi_arlen.value),
                "size": int(dut.m_axi_arsize.value),
                "burst": int(dut.m_axi_arburst.value),
            }
            break
    dut.m_axi_arready.value = 0

    for i, value in enumerate(beats):
        for _ in range(r_gap):
            await RisingEdge(dut.clk)

        dut.m_axi_rid.value = 0
        dut.m_axi_rdata.value = value
        dut.m_axi_rresp.value = error_resp if i == error_index else 0
        dut.m_axi_rlast.value = 1 if i == len(beats) - 1 else 0
        dut.m_axi_rvalid.value = 1

        while True:
            await RisingEdge(dut.clk)
            if int(dut.m_axi_rvalid.value) and int(dut.m_axi_rready.value):
                break

        dut.m_axi_rvalid.value = 0
        dut.m_axi_rresp.value = 0
        dut.m_axi_rlast.value = 0

    return {"ar": ar}


# ----------------------------------------------------------------------
# End-to-end AHB write -> AHB readback
# ----------------------------------------------------------------------

@cocotb.test()
async def test_single_qword_write_then_readback_via_ahb(dut):
    set_test_id(dut)
    _, _ = await setup_dut(dut, with_ahb_master=False)

    addr = 0xB800
    data = 0x0123456789ABCDEF

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 1))
    b_task = cocotb.start_soon(wait_for_b_handshake(dut))

    await ahb_write_inc_burst_manual(dut, addr, [data], size_bytes=8, fixed=True)
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w_beats = await w_task
    await b_task
    await ClockCycles(dut.clk, 1)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))
    r_task = cocotb.start_soon(wait_for_n_r_handshakes(dut, 1))

    got = await ahb_read_inc_burst_manual(dut, addr, 1, size_bytes=8, fixed=True)
    await ClockCycles(dut.clk, 2)

    ar = await ar_task
    r_beats = await r_task

    assert aw["addr"] == addr
    assert aw["len"] == 0
    assert aw["size"] == 3
    assert aw["burst"] == 1
    assert w_beats[0]["data"] == data
    assert w_beats[0]["strb"] == 0xFF
    assert w_beats[0]["last"] == 1

    assert ar["addr"] == addr
    assert ar["len"] == 0
    assert ar["size"] == 3
    assert ar["burst"] == 1

    assert got == [data]
    assert r_beats[0]["data"] == data
    assert r_beats[0]["resp"] == 0
    assert r_beats[0]["last"] == 1


@cocotb.test()
async def test_incr8_qword_write_then_readback_via_ahb(dut):
    set_test_id(dut)
    _, _ = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0xB900
    beats = [0x1111000000000000 + i for i in range(8)]

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 8))
    b_task = cocotb.start_soon(wait_for_b_handshake(dut))

    await ahb_write_inc_burst_manual(dut, start_addr, beats, size_bytes=8, fixed=True)
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w_beats = await w_task
    await b_task
    await ClockCycles(dut.clk, 1)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))
    r_task = cocotb.start_soon(wait_for_n_r_handshakes(dut, 8))

    got = await ahb_read_inc_burst_manual(dut, start_addr, 8, size_bytes=8, fixed=True)
    await ClockCycles(dut.clk, 2)

    ar = await ar_task
    r_beats = await r_task

    assert aw["addr"] == start_addr
    assert aw["len"] == 7
    assert aw["size"] == 3
    assert aw["burst"] == 1

    for i, wb in enumerate(w_beats):
        assert wb["data"] == beats[i]
        assert wb["strb"] == 0xFF
        assert wb["last"] == (1 if i == 7 else 0)

    assert ar["addr"] == start_addr
    assert ar["len"] == 7
    assert ar["size"] == 3
    assert ar["burst"] == 1

    assert got == beats
    for i, rb in enumerate(r_beats):
        assert rb["data"] == beats[i]
        assert rb["resp"] == 0
        assert rb["last"] == (1 if i == 7 else 0)


@cocotb.test()
async def test_incr_len5_qword_write_then_readback_via_ahb(dut):
    set_test_id(dut)
    _, _ = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0xBA00
    beats = [
        0x1010101010101010,
        0x2020202020202020,
        0x3030303030303030,
        0x4040404040404040,
        0x5050505050505050,
    ]

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    b_task = cocotb.start_soon(wait_for_b_handshake(dut))

    await ahb_write_inc_burst_manual(dut, start_addr, beats, size_bytes=8, fixed=False)
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    await b_task
    await ClockCycles(dut.clk, 1)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))
    got = await ahb_read_inc_burst_manual(dut, start_addr, 5, size_bytes=8, fixed=False)
    await ClockCycles(dut.clk, 6)
    ar = await ar_task

    # Keep this generic: do not lock AWLEN/ARLEN to one implementation.
    assert aw["addr"] == start_addr
    assert aw["size"] == 3
    assert aw["burst"] == 1

    assert ar["addr"] == start_addr
    assert ar["size"] == 3
    assert ar["burst"] == 1

    assert got == beats


# ----------------------------------------------------------------------
# Reset + backpressure
# ----------------------------------------------------------------------

@cocotb.test()
async def test_reset_during_wrap8_read_with_axi_backpressure_clears_bus(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    set_axi_ram_backpressure(
        axi_ram,
        stall_ar=True,
        stall_r=True,
    )

    start_addr = 0xBB18
    expected_beats = [0x2200000000000000 + i for i in range(8)]
    expected_addrs = [_wrap_addr(start_addr, i, 8, 8) for i in range(8)]
    for i, value in enumerate(expected_beats):
        axi_ram.write_qword(expected_addrs[i], value)

    task = cocotb.start_soon(
        ahb_read_wrap_burst_manual(dut, start_addr, 8, size_bytes=8)
    )

    await wait_until_any_axi_valid(dut, max_cycles=20)

    saw_wait = False
    for _ in range(30):
        await RisingEdge(dut.clk)
        if int(dut.hready.value) == 0:
            saw_wait = True
            break
    assert saw_wait, "expected AHB wait-state before reset"

    dut.resetn.value = 0
    await ClockCycles(dut.clk, 2)
    dut.resetn.value = 1

    task.kill()
    _init_direct_ahb_signals(dut)
    await wait_until_all_axi_outputs_idle(dut, max_cycles=20)

    assert int(dut.hresp.value) == 0


@cocotb.test()
async def test_reset_during_wrap16_write_with_axi_backpressure_clears_bus(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    set_axi_ram_backpressure(
        axi_ram,
        stall_aw=True,
        stall_w=True,
        stall_b=True,
    )

    start_addr = 0xBC08
    beats = [0x3300000000000000 + i for i in range(16)]

    task = cocotb.start_soon(
        ahb_write_wrap_burst_manual(dut, start_addr, beats, size_bytes=8)
    )

    await wait_until_any_axi_valid(dut, max_cycles=20)

    saw_wait = False
    for _ in range(40):
        await RisingEdge(dut.clk)
        if int(dut.hready.value) == 0:
            saw_wait = True
            break
    assert saw_wait, "expected AHB wait-state before reset"

    dut.resetn.value = 0
    await ClockCycles(dut.clk, 2)
    dut.resetn.value = 1

    task.kill()
    _init_direct_ahb_signals(dut)
    await wait_until_all_axi_outputs_idle(dut, max_cycles=20)

    assert int(dut.hresp.value) == 0


# ----------------------------------------------------------------------
# Larger-burst error propagation
# ----------------------------------------------------------------------

@cocotb.test()
async def test_wrap8_read_burst_rresp_error_on_middle_beat(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    start_addr = 0xBD18
    beats = [0x4400000000000000 + i for i in range(8)]

    slave_task = cocotb.start_soon(
        axi_slave_respond_read_burst(
            dut,
            beats,
            error_index=3,
            error_resp=0b10,
            r_gap=1,
        )
    )

    samples = await ahb_read_wrap_burst_manual_status(dut, start_addr, 8, size_bytes=8)
    info = await slave_task

    assert info["ar"]["addr"] == start_addr
    assert info["ar"]["len"] == 7
    assert info["ar"]["size"] == 3
    assert info["ar"]["burst"] == 2

    assert len(samples) == 8
    assert samples[0]["resp"] == 0
    assert samples[1]["resp"] == 0
    assert samples[2]["resp"] == 0
    assert samples[3]["resp"] == 1


@cocotb.test()
async def test_wrap16_write_burst_bresp_slverr_propagates(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    start_addr = 0xBE08
    beats = [0x5500000000000000 + i for i in range(16)]

    slave_task = cocotb.start_soon(
        axi_slave_respond_write_burst(
            dut,
            16,
            bresp=0b10,
            b_delay=2,
        )
    )

    final_resp = await ahb_write_wrap_burst_manual_status(dut, start_addr, beats, size_bytes=8)
    info = await slave_task

    assert info["aw"]["addr"] == start_addr
    assert info["aw"]["len"] == 15
    assert info["aw"]["size"] == 3
    assert info["aw"]["burst"] == 2

    for i, wb in enumerate(info["w_beats"]):
        assert wb["data"] == beats[i]
        assert wb["strb"] == 0xFF
        assert wb["last"] == (1 if i == 15 else 0)

    assert final_resp == 1


@cocotb.test()
async def test_wrap16_read_burst_rresp_error_on_middle_beat(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    start_addr = 0xBF08
    beats = [0x6600000000000000 + i for i in range(16)]

    slave_task = cocotb.start_soon(
        axi_slave_respond_read_burst(
            dut,
            beats,
            error_index=7,
            error_resp=0b10,
            r_gap=1,
        )
    )

    samples = await ahb_read_wrap_burst_manual_status(dut, start_addr, 16, size_bytes=8)
    info = await slave_task

    assert info["ar"]["addr"] == start_addr
    assert info["ar"]["len"] == 15
    assert info["ar"]["size"] == 3
    assert info["ar"]["burst"] == 2

    assert len(samples) == 16
    for i in range(7):
        assert samples[i]["resp"] == 0
    assert samples[7]["resp"] == 1


# ----------------------------------------------------------------------
# Narrow WRAP16 coverage
# ----------------------------------------------------------------------

@cocotb.test()
async def test_wrap16_byte_write_burst_updates_byte_lanes(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0xC007
    beats = [0x10 + i for i in range(16)]

    initial_qwords = {
        0xC000: 0xFFEEDDCCBBAA9988,
        0xC008: 0x7766554433221100,
        0xC010: 0x0123456789ABCDEF,
        0xC018: 0x0F1E2D3C4B5A6978,
        0xC020: 0x8899AABBCCDDEEFF,
        0xC028: 0x1021324354657687,
        0xC030: 0xDEADBEEFCAFEBABE,
        0xC038: 0x55AA55AA55AA55AA,
    }

    for addr, value in initial_qwords.items():
        axi_ram.write_qword(addr, value)

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 16))
    b_task = cocotb.start_soon(wait_for_b_handshake(dut))

    await ahb_write_wrap_burst_manual(dut, start_addr, beats, size_bytes=1)
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w_beats = await w_task
    await b_task
    await ClockCycles(dut.clk, 1)

    assert aw["addr"] == start_addr
    assert aw["len"] == 15
    assert aw["size"] == 0
    assert aw["burst"] == 2

    for i, wb in enumerate(w_beats):
        addr = _wrap_addr(start_addr, i, 1, 16)
        expected_wdata = (beats[i] & 0xFF) << _lane_shift(addr)
        expected_wstrb = _strb_mask(addr, 1)

        assert wb["data"] == expected_wdata
        assert wb["strb"] == expected_wstrb
        assert wb["last"] == (1 if i == 15 else 0)

    expected_qwords = _expected_wrap_qwords(initial_qwords, start_addr, beats, 1, 16)
    for qaddr, expected in expected_qwords.items():
        got = axi_ram.read_qword(qaddr)
        assert got == expected


@cocotb.test()
async def test_wrap16_halfword_read_burst_returns_expected_lanes(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0xC106
    initial_qwords = {
        0xC100: 0x0123456789ABCDEF,
        0xC108: 0x1122334455667788,
        0xC110: 0x99AABBCCDDEEFF00,
        0xC118: 0x0F1E2D3C4B5A6978,
        0xC120: 0x8899AABBCCDDEEFF,
        0xC128: 0x1021324354657687,
        0xC130: 0xDEADBEEFCAFEBABE,
        0xC138: 0x55AA55AA55AA55AA,
    }

    for addr, value in initial_qwords.items():
        axi_ram.write_qword(addr, value)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))
    r_task = cocotb.start_soon(wait_for_n_r_handshakes(dut, 16))

    got_beats = await ahb_read_wrap_burst_manual(dut, start_addr, 16, size_bytes=2)
    await ClockCycles(dut.clk, 2)

    ar = await ar_task
    r_beats = await r_task

    assert ar["addr"] == start_addr
    assert ar["len"] == 15
    assert ar["size"] == 1
    assert ar["burst"] == 2

    expected_halfwords = _expected_wrap_subwords(initial_qwords, start_addr, 2, 16)

    for i, raw in enumerate(got_beats):
        addr = _wrap_addr(start_addr, i, 2, 16)
        got = _extract_subword(raw, addr, 2)
        assert got == expected_halfwords[i]

    for i, rb in enumerate(r_beats):
        assert rb["resp"] == 0
        assert rb["last"] == (1 if i == 15 else 0)


@cocotb.test()
async def test_wrap16_word_write_burst_updates_memory(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0xC20C
    beats = [0x10000000 + i for i in range(16)]

    initial_qwords = {
        0xC200: 0x0011223344556677,
        0xC208: 0x8899AABBCCDDEEFF,
        0xC210: 0x0123456789ABCDEF,
        0xC218: 0x0F1E2D3C4B5A6978,
        0xC220: 0xFFEEDDCCBBAA9988,
        0xC228: 0x7766554433221100,
        0xC230: 0xDEADBEEFCAFEBABE,
        0xC238: 0x55AA55AA55AA55AA,
    }

    for addr, value in initial_qwords.items():
        axi_ram.write_qword(addr, value)

    aw_task = cocotb.start_soon(wait_for_aw_handshake(dut))
    w_task = cocotb.start_soon(wait_for_n_w_handshakes(dut, 16))
    b_task = cocotb.start_soon(wait_for_b_handshake(dut))

    await ahb_write_wrap_burst_manual(dut, start_addr, beats, size_bytes=4)
    await ClockCycles(dut.clk, 2)

    aw = await aw_task
    w_beats = await w_task
    await b_task
    await ClockCycles(dut.clk, 1)

    assert aw["addr"] == start_addr
    assert aw["len"] == 15
    assert aw["size"] == 2
    assert aw["burst"] == 2

    for i, wb in enumerate(w_beats):
        addr = _wrap_addr(start_addr, i, 4, 16)
        expected_wdata = (beats[i] & 0xFFFFFFFF) << _lane_shift(addr)
        expected_wstrb = _strb_mask(addr, 4)

        assert wb["data"] == expected_wdata
        assert wb["strb"] == expected_wstrb
        assert wb["last"] == (1 if i == 15 else 0)

    expected_qwords = _expected_wrap_qwords(initial_qwords, start_addr, beats, 4, 16)
    for qaddr, expected in expected_qwords.items():
        got = axi_ram.read_qword(qaddr)
        assert got == expected


@cocotb.test()
async def test_wrap16_word_read_burst_returns_expected_lanes(dut):
    set_test_id(dut)
    _, axi_ram = await setup_dut(dut, with_ahb_master=False)

    start_addr = 0xC30C
    initial_qwords = {
        0xC300: 0x0011223344556677,
        0xC308: 0x8899AABBCCDDEEFF,
        0xC310: 0x0123456789ABCDEF,
        0xC318: 0x0F1E2D3C4B5A6978,
        0xC320: 0xFFEEDDCCBBAA9988,
        0xC328: 0x7766554433221100,
        0xC330: 0xDEADBEEFCAFEBABE,
        0xC338: 0x55AA55AA55AA55AA,
    }

    for addr, value in initial_qwords.items():
        axi_ram.write_qword(addr, value)

    ar_task = cocotb.start_soon(wait_for_ar_handshake(dut))
    r_task = cocotb.start_soon(wait_for_n_r_handshakes(dut, 16))

    got_beats = await ahb_read_wrap_burst_manual(dut, start_addr, 16, size_bytes=4)
    await ClockCycles(dut.clk, 2)

    ar = await ar_task
    r_beats = await r_task

    assert ar["addr"] == start_addr
    assert ar["len"] == 15
    assert ar["size"] == 2
    assert ar["burst"] == 2

    expected_words = _expected_wrap_subwords(initial_qwords, start_addr, 4, 16)

    for i, raw in enumerate(got_beats):
        addr = _wrap_addr(start_addr, i, 4, 16)
        got = _extract_subword(raw, addr, 4)
        assert got == expected_words[i]

    for i, rb in enumerate(r_beats):
        assert rb["resp"] == 0
        assert rb["last"] == (1 if i == 15 else 0)



#This test was wrong
# @cocotb.test()
# async def test_long_sequential_write_does_not_corrupt_far_stack_address(dut):
#     set_test_id(dut)
#     _, axi_ram = await setup_dut(dut, with_ahb_master=False)

#     stack_addr = 0xBFFFFBC0
#     load_base  = 0x80000000
#     load_size  = 0x43000          # 268 KB, same order as your OpenSBI load
#     chunk_beats = 16              # use INCR16 chunks
#     sentinel = 0xDEADBEEFCAFEBABE

#     def pattern(addr):
#         # deterministic nonzero 64-bit pattern per qword address
#         word_index = (addr - load_base) >> 3
#         return (0x1234000000000000 | word_index) ^ 0x55AA55AA55AA55AA

#     # Step 1: plant sentinel at the "stack" address
#     await ahb_write_single_manual(dut, stack_addr, sentinel, size_bytes=8)
#     await ClockCycles(dut.clk, 2)

#     # Optional immediate sanity check
#     got0 = await ahb_read_inc_burst_manual(dut, stack_addr, 1, size_bytes=8, fixed=True)
#     assert got0 == [sentinel], (
#         f"initial stack sentinel mismatch: expected 0x{sentinel:016x}, got 0x{got0[0]:016x}"
#     )

#     # Step 2: long sequential write into the load region, chunked as INCR16 bursts
#     beats_total = load_size // 8
#     assert beats_total * 8 == load_size

#     addr = load_base
#     beats_left = beats_total
#     while beats_left > 0:
#         n = min(chunk_beats, beats_left)
#         beats = [pattern(addr + i * 8) for i in range(n)]

#         await ahb_write_inc_burst_manual(
#             dut,
#             start_addr=addr,
#             data_beats=beats,
#             size_bytes=8,
#             fixed=(n in (4, 8, 16)),
#         )

#         addr += n * 8
#         beats_left -= n

#     await ClockCycles(dut.clk, 8)

#     # Step 3: read back the sentinel via AHB
#     got = await ahb_read_inc_burst_manual(dut, stack_addr, 1, size_bytes=8, fixed=True)
#     assert got == [sentinel], (
#         f"stack address corrupted after long write: expected 0x{sentinel:016x}, got 0x{got[0]:016x}"
#     )

#     # Useful extra checks: start/end of the loaded region contain what was written
#     start_expected = pattern(load_base)
#     end_addr = load_base + load_size - 8
#     end_expected = pattern(end_addr)

#     assert axi_ram.read_qword(load_base) == start_expected, (
#         f"load_base mismatch: expected 0x{start_expected:016x}, got 0x{axi_ram.read_qword(load_base):016x}"
#     )
#     assert axi_ram.read_qword(end_addr) == end_expected, (
#         f"load_end mismatch: expected 0x{end_expected:016x}, got 0x{axi_ram.read_qword(end_addr):016x}"
#     )



# This test was also wrong
# @cocotb.test()
# async def test_find_first_write_that_corrupts_far_stack_address(dut):
#     set_test_id(dut)
#     _, axi_ram = await setup_dut(dut, with_ahb_master=False)

#     stack_addr = 0xBFFFFBC0
#     load_base  = 0x80000000
#     load_size  = 0x43000
#     sentinel   = 0xDEADBEEFCAFEBABE

#     def pattern_from_offset(offset):
#         # Encodes qword offset directly in the low bits so corruption source
#         # can be inferred from the observed bad value.
#         return 0xA5A5000000000000 | (offset >> 3)

#     await ahb_write_single_manual(dut, stack_addr, sentinel, size_bytes=8)
#     await ClockCycles(dut.clk, 2)

#     got0 = await ahb_read_inc_burst_manual(dut, stack_addr, 1, size_bytes=8, fixed=True)
#     assert got0 == [sentinel], f"initial sentinel mismatch: got 0x{got0[0]:016x}"

#     first_bad = None
#     first_bad_val = None
#     first_bad_offset = None

#     for offset in range(0, load_size, 8):
#         addr = load_base + offset
#         data = pattern_from_offset(offset)

#         await ahb_write_single_manual(dut, addr, data, size_bytes=8)

#         got = await ahb_read_inc_burst_manual(dut, stack_addr, 1, size_bytes=8, fixed=True)
#         if got[0] != sentinel:
#             first_bad = addr
#             first_bad_val = got[0]
#             first_bad_offset = offset
#             break

#     assert first_bad is None, (
#         f"stack address corrupted after write to 0x{first_bad:08x} "
#         f"(offset 0x{first_bad_offset:x}); "
#         f"stack readback became 0x{first_bad_val:016x}"
#     )



async def collect_n_aw_handshakes(dut, n):
    out = []
    while len(out) < n:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_awvalid.value) and int(dut.m_axi_awready.value):
            out.append({
                "addr": int(dut.m_axi_awaddr.value),
                "len": int(dut.m_axi_awlen.value),
                "size": int(dut.m_axi_awsize.value),
                "burst": int(dut.m_axi_awburst.value),
            })
    return out


@cocotb.test()
async def test_high_address_single_write_stream_emits_expected_awaddrs(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    load_base = 0x80000000
    n = 64

    for i in range(n):
        addr = load_base + i * 8
        data = 0xC0DE000000000000 | i

        slave = cocotb.start_soon(
            axi_slave_respond_single_write(dut, b_delay=2)
        )

        await with_timeout(
            ahb_write_single_manual(dut, addr, data, size_bytes=8),
            200, "us"
        )
        info = await with_timeout(slave, 50, "us")

        aw = info["aw"]
        w = info["w"]

        assert aw["addr"] == addr, (
            f"AW[{i}] addr mismatch: expected 0x{addr:08x}, got 0x{aw['addr']:08x}"
        )
        assert aw["len"] == 0, f"AW[{i}] expected LEN=0, got {aw['len']}"
        assert aw["size"] == 3, f"AW[{i}] expected SIZE=3, got {aw['size']}"
        assert aw["burst"] == 1, f"AW[{i}] expected BURST=INCR, got {aw['burst']}"

        assert w["data"] == data, (
            f"W[{i}] data mismatch: expected 0x{data:016x}, got 0x{w['data']:016x}"
        )
        assert w["strb"] == 0xFF, f"W[{i}] expected WSTRB=0xFF, got 0x{w['strb']:02x}"
        assert w["last"] == 1, f"W[{i}] expected WLAST=1, got {w['last']}"

@cocotb.test()
async def test_high_address_long_write_does_not_corrupt_far_stack_address_sparse_axi(dut):
    set_test_id(dut)
    await setup_dut_no_axi_slave(dut)

    mem = {}
    cocotb.start_soon(axi_sparse_mem_write_slave(dut, mem))
    cocotb.start_soon(axi_sparse_mem_read_slave(dut, mem))

    stack_addr = 0xBFFFFBC0
    load_base  = 0x80000000
    load_size  = 0x43000          # 534 * 512 = 0x42C00; keep slightly above that
    sentinel   = 0xDEADBEEFCAFEBABE
    chunk_beats = 16

    def pattern(addr):
        word_index = (addr - load_base) >> 3
        return (0x1234000000000000 | word_index) ^ 0x55AA55AA55AA55AA

    # Step 1: write sentinel to far "stack" address
    await with_timeout(
        ahb_write_single_manual(dut, stack_addr, sentinel, size_bytes=8),
        200, "us"
    )

    await ClockCycles(dut.clk, 3)

    # Sanity-check sentinel before bulk load
    got0 = await with_timeout(
        ahb_read_inc_burst_manual(dut, stack_addr, 1, size_bytes=8, fixed=True),
        200, "us"
    )
    assert got0 == [sentinel], (
        f"initial stack sentinel mismatch: expected 0x{sentinel:016x}, got 0x{got0[0]:016x}"
    )

    # Step 2: long sequential write using INCR16 chunks
    addr = load_base
    beats_left = load_size // 8
    chunk_idx = 0

    while beats_left > 0:
        n = min(chunk_beats, beats_left)
        beats = [pattern(addr + i * 8) for i in range(n)]

        await with_timeout(
            ahb_write_inc_burst_manual(
                dut,
                addr,
                beats,
                size_bytes=8,
                fixed=(n in (4, 8, 16)),
            ),
            500, "us"
        )

        addr += n * 8
        beats_left -= n
        chunk_idx += 1

        if (chunk_idx % 256) == 0:
            dut._log.info(f"completed {chunk_idx} chunks, last written addr=0x{addr - 8:08x}")

    #await ClockCycles(dut.clk, 4)
    await ClockCycles(dut.clk, 3)

    # Step 3: read back sentinel via AHB
    got = await with_timeout(
        ahb_read_inc_burst_manual(dut, stack_addr, 1, size_bytes=8, fixed=True),
        200, "us"
    )
    
    assert got == [sentinel], (
        f"stack address corrupted after long write: expected 0x{sentinel:016x}, got 0x{got[0]:016x}"
    )

    # Optional extra end-to-end checks on the bulk-loaded region
    start_expected = pattern(load_base)
    end_addr = load_base + load_size - 8
    end_expected = pattern(end_addr)

    await ClockCycles(dut.clk, 3)
    
    got_start = await with_timeout(
        ahb_read_inc_burst_manual(dut, load_base, 1, size_bytes=8, fixed=True),
        200, "us"
    )
    got_end = await with_timeout(
        ahb_read_inc_burst_manual(dut, end_addr, 1, size_bytes=8, fixed=True),
        200, "us"
    )

    assert got_start == [start_expected], (
        f"load_base mismatch: expected 0x{start_expected:016x}, got 0x{got_start[0]:016x}"
    )
    assert got_end == [end_expected], (
        f"load_end mismatch: expected 0x{end_expected:016x}, got 0x{got_end[0]:016x}"
    )


# =============================================================================
# Regression tests for two bridge bugs found during Wally boot debugging
# (2026-03-10).
#
# BUG 1 — stale-RVALID / ar_done (fixed in RTL rev 1):
#   The bridge issues an AR for a read-allocate fill, then an AHB write arrives
#   before the R beats return through the CDC.  While in ST_WR_D/ST_WR_RESP,
#   RREADY=0, so the R beats pile up in the downstream FIFO.  When the bridge
#   later enters ST_RD_D for the *next* AHB read it immediately sees RVALID=1
#   from the stale beats and drives HREADY=1 with the wrong RDATA, causing
#   Wally's epilogue `ld ra, 104(sp)` to load 0 instead of 0x132A.
#   Fix: ar_done flag — HREADY/RREADY in ST_RD_D gated on ar_done which is
#   cleared on every ST_RD_A entry and set only when ARREADY=1.
#
# BUG 2 — ST_WR_RESP fast-path samples AHB too early (found in RTL rev 1):
#   The ST_WR_RESP block tried to fast-path directly to ST_WR_D or ST_RD_A
#   in the same cycle BVALID=1 fires by sampling HADDR/HBURST/HWRITE.
#   But HREADY goes high *combinatorially* from BVALID, so the AHB master's
#   response (stable new address phase) only appears one cycle later.
#   Result A: AWLEN correct but beat_cnt loaded from stale c_axlen → wrong
#             number of W beats sent (WLAST too early), corrupting the AXI
#             write burst.
#   Result B: HTRANS=NONSEQ check fails on the stale bus → bridge falls to
#             ST_IDLE swallowing the pending read, ARVALID never asserted,
#             system deadlock.
#   Fix: unconditionally go to ST_IDLE on BVALID; ST_IDLE picks up the next
#        NONSEQ with stable signals one cycle later at zero throughput cost.
# =============================================================================


# ---------------------------------------------------------------------------
# Helpers shared by both regression tests
# ---------------------------------------------------------------------------

async def _wait_hready(dut, timeout_cycles=2000):
    """Wait for hready=1, using the signal name the wrapper exposes."""
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        dut.hreadyin.value = int(dut.hready.value)  # <-- ADD THIS
        if int(dut.hready.value):
            return
    raise TimeoutError("hready never went high")

async def _ahb_write_incr8_manual(dut, start_addr, beats_8):
    """Drive an AHB INCR8 write burst (8 x 64-bit beats)."""
    assert len(beats_8) == 8
    hburst = 0b101   # INCR8
    hsize  = 3       # 64-bit

    dut.haddr.value  = start_addr
    dut.hburst.value = hburst
    dut.hmastlock.value = 0
    dut.hprot.value  = 0
    dut.hsize.value  = hsize
    dut.htrans.value = 0b10   # NONSEQ
    dut.hwrite.value = 1
    dut.hwdata.value = 0

    for i in range(8):
        await _wait_hready(dut)
        if i == 7:
            dut.haddr.value  = 0
            dut.hburst.value = 0
            dut.hsize.value  = 0
            dut.htrans.value = 0   # IDLE
            dut.hwrite.value = 0
        else:
            dut.haddr.value  = start_addr + (i + 1) * 8
            dut.htrans.value = 0b11   # SEQ
        dut.hwdata.value = beats_8[i]

    await _wait_hready(dut)
    _init_direct_ahb_signals(dut)


async def _ahb_read_incr8_manual(dut, start_addr):
    """Drive an AHB INCR8 read burst (8 x 64-bit beats). Returns list[8]."""
    hburst = 0b101   # INCR8
    hsize  = 3
    results = []

    dut.haddr.value  = start_addr
    dut.hburst.value = hburst
    dut.hmastlock.value = 0
    dut.hprot.value  = 0
    dut.hsize.value  = hsize
    dut.htrans.value = 0b10   # NONSEQ
    dut.hwrite.value = 0
    dut.hwdata.value = 0

    for i in range(8):
        await _wait_hready(dut)
        await ReadOnly()

        if i > 0:
            results.append(int(dut.hrdata.value))

        await NextTimeStep()

        if i == 7:
            dut.haddr.value  = 0
            dut.hburst.value = 0
            dut.hsize.value  = 0
            dut.htrans.value = 0
        else:
            dut.haddr.value  = start_addr + (i + 1) * 8
            dut.htrans.value = 0b11   # SEQ

    await _wait_hready(dut)
    await ReadOnly()
    results.append(int(dut.hrdata.value))
    await NextTimeStep()

    _init_direct_ahb_signals(dut)
    return results

async def _ahb_read_incr8_manual_bug2b_sampled(dut, start_addr):
    """INCR8 read helper used only by BUG2b test. Samples HRDATA in ReadOnly phase."""
    hburst = 0b101   # INCR8
    hsize  = 3
    results = []

    async def wait_hready_and_sample():
        while True:
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.hready.value):
                return int(dut.hrdata.value)

    dut.haddr.value  = start_addr
    dut.hburst.value = hburst
    dut.hmastlock.value = 0
    dut.hprot.value  = 0
    dut.hsize.value  = hsize
    dut.htrans.value = 0b10   # NONSEQ
    dut.hwrite.value = 0
    dut.hwdata.value = 0

    for i in range(8):
        sampled = await wait_hready_and_sample()
        results.append(sampled)

        await NextTimeStep()

        if i == 7:
            dut.haddr.value  = 0
            dut.hburst.value = 0
            dut.hsize.value  = 0
            dut.htrans.value = 0   # IDLE
        else:
            dut.haddr.value  = start_addr + (i + 1) * 8
            dut.htrans.value = 0b11   # SEQ

    await NextTimeStep()
    _init_direct_ahb_signals(dut)
    return results

async def _sparse_write_slave_one_burst(dut, mem):
    """
    Service exactly one AXI write burst into mem.
    Returns (awaddr, awlen, beats_received).
    beats_received is a list of (wdata, wlast) tuples.
    Does NOT assert on WLAST position — lets the test do that.
    """
    # wait for AW
    while not int(dut.m_axi_awvalid.value):
        await RisingEdge(dut.clk)
    dut.m_axi_awready.value = 1
    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_awvalid.value) and int(dut.m_axi_awready.value):
            awaddr = int(dut.m_axi_awaddr.value)
            awlen  = int(dut.m_axi_awlen.value)
            awsize = int(dut.m_axi_awsize.value)
            awburst= int(dut.m_axi_awburst.value)
            break
    dut.m_axi_awready.value = 0

    # collect W beats until WLAST or until the slave has seen awlen+1
    beats = []
    while True:
        while not int(dut.m_axi_wvalid.value):
            await RisingEdge(dut.clk)
        dut.m_axi_wready.value = 1
        await RisingEdge(dut.clk)
        wdata = int(dut.m_axi_wdata.value)
        wstrb = int(dut.m_axi_wstrb.value)
        wlast = int(dut.m_axi_wlast.value)
        dut.m_axi_wready.value = 0

        addr = awaddr + len(beats) * (1 << awsize) if awburst == 1 else awaddr
        _sparse_apply_wbeat(mem, addr, wdata, wstrb)
        beats.append((wdata, wlast))

        if wlast:
            break
        # Safety: stop if we've received more beats than AWLEN declares
        if len(beats) > awlen + 1:
            break

    # send B
    dut.m_axi_bvalid.value = 1
    dut.m_axi_bresp.value  = 0
    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_bvalid.value) and int(dut.m_axi_bready.value):
            break
    dut.m_axi_bvalid.value = 0

    return awaddr, awlen, beats


async def _sparse_read_slave_one_burst(dut, mem, *, r_delay=0):
    """
    Service exactly one AXI read burst from mem.
    r_delay: extra cycles to hold off RVALID after ARREADY (simulates CDC).
    Returns araddr.
    """
    while not int(dut.m_axi_arvalid.value):
        await RisingEdge(dut.clk)
    dut.m_axi_arready.value = 1
    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_arvalid.value) and int(dut.m_axi_arready.value):
            araddr  = int(dut.m_axi_araddr.value)
            arlen   = int(dut.m_axi_arlen.value)
            arsize  = int(dut.m_axi_arsize.value)
            arburst = int(dut.m_axi_arburst.value)
            break
    dut.m_axi_arready.value = 0

    await ClockCycles(dut.clk, r_delay)

    for beat in range(arlen + 1):
        addr = araddr + beat * (1 << arsize) if arburst == 1 else araddr
        base = addr & ~0x7
        rdata = mem.get(base, 0)
        dut.m_axi_rdata.value  = rdata
        dut.m_axi_rresp.value  = 0
        dut.m_axi_rlast.value  = 1 if beat == arlen else 0
        dut.m_axi_rvalid.value = 1
        while True:
            await RisingEdge(dut.clk)
            if int(dut.m_axi_rvalid.value) and int(dut.m_axi_rready.value):
                break
        dut.m_axi_rvalid.value = 0
        dut.m_axi_rlast.value  = 0

    return araddr


# ---------------------------------------------------------------------------
# BUG 1 REGRESSION: stale RVALID consumed before ar_done
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_regression_stale_rvalid_write_interleaved_with_read(dut):
    set_test_id(dut)
    """
    INVALID as a protocol regression.

    This test uses two concurrent coroutines to drive the same AHB-Lite master
    pins (`haddr/hwrite/htrans/...`) at the same time. AXI read/write channel
    overlap is legal, but AHB-Lite exposes one shared address/control interface
    per master. Keep this skipped so it does not block RTL debug with undefined
    stimulus.
    """
    raise cocotb.result.TestSuccess(
        "Skipping invalid regression: it double-drives one AHB-Lite master interface."
    )


@cocotb.test()
async def test_regression_delayed_read_then_write_then_read_legal_ahb(dut):
    set_test_id(dut)
    """
    Legal AHB-Lite replacement for BUG1.

    Sequence:
      1. INCR8 read @ READ_ADDR; AXI AR is accepted immediately, AXI R is delayed.
         The bridge must stall AHB (HREADY low) until read data is available.
      2. After the read completes, issue an INCR8 write @ WRITE_ADDR.
      3. Immediately issue a second INCR8 read @ READ_ADDR.
    """
    READ_ADDR    = 0x00001000
    WRITE_ADDR   = 0x00002000
    CORRECT_DATA = [0x000000000000132A + i for i in range(8)]
    WRITE_DATA   = [0xABCD000000000000 + i for i in range(8)]
    R_DELAY      = 20

    async def _wait_hready_ro(timeout_cycles=5000):
        for _ in range(timeout_cycles):
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.hready.value):
                return int(dut.hrdata.value)
        raise TimeoutError("BUG1 dedicated read helper: no HREADY")

    async def _read_incr8_delayed_or_postwrite(start_addr):
        hburst = 0b101
        hsize  = 3
        results = []

        # Start driving the address in a known half-cycle BEFORE the accepting edge.
        await FallingEdge(dut.clk)

        dut.haddr.value      = start_addr
        dut.hburst.value     = hburst
        dut.hmastlock.value  = 0
        dut.hprot.value      = 0
        dut.hsize.value      = hsize
        dut.htrans.value     = 0b10   # NONSEQ
        dut.hwrite.value     = 0
        dut.hwdata.value     = 0

        # This edge is the address-phase edge. Do NOT count HRDATA here.
        await RisingEdge(dut.clk)
        await ReadOnly()

        # Now collect exactly 8 returned beats.
        for beat_index in range(8):
            while True:
                await RisingEdge(dut.clk)
                await ReadOnly()
                if int(dut.hready.value):
                    results.append(int(dut.hrdata.value))
                    break

            await NextTimeStep()

            if beat_index == 7:
                dut.haddr.value  = 0
                dut.hburst.value = 0
                dut.hsize.value  = 0
                dut.htrans.value = 0
            else:
                dut.haddr.value  = start_addr + (beat_index + 1) * 8
                dut.htrans.value = 0b11   # SEQ

        _init_direct_ahb_signals(dut)
        return results

    await setup_dut_no_axi_slave(dut)

    mem = {}
    for i in range(8):
        mem[(READ_ADDR & ~0x7) + i * 8] = CORRECT_DATA[i]

    dut._log.info("Phase 1: delayed INCR8 read")

    slave1 = cocotb.start_soon(_sparse_read_slave_one_burst(dut, mem, r_delay=R_DELAY))
    read1  = cocotb.start_soon(_read_incr8_delayed_or_postwrite(READ_ADDR))

    await ClockCycles(dut.clk, 8)
    assert int(dut.hready.value) == 0, (
        "AHB should be stalled while delayed read data is outstanding"
    )

    read1_result = await with_timeout(read1, 5, "us")
    await with_timeout(slave1, 5, "us")
    assert read1_result[0] == CORRECT_DATA[0], (
        f"read1 beat[0] = 0x{read1_result[0]:016x}, expected 0x{CORRECT_DATA[0]:016x}"
    )

    dut._log.info("Phase 2: INCR8 write after read completion")

    slave_wr = cocotb.start_soon(_sparse_write_slave_one_burst(dut, mem))
    await with_timeout(_ahb_write_incr8_manual(dut, WRITE_ADDR, WRITE_DATA), 5, "us")
    wr_awaddr, wr_awlen, wr_beats = await with_timeout(slave_wr, 5, "us")
    assert wr_awaddr == WRITE_ADDR
    assert wr_awlen == 7
    assert len(wr_beats) == 8

    dut._log.info("Phase 3: second INCR8 read with no idle gap after write")

    slave3 = cocotb.start_soon(_sparse_read_slave_one_burst(dut, mem, r_delay=2))
    read2_result = await with_timeout(_read_incr8_delayed_or_postwrite(READ_ADDR), 5, "us")
    await with_timeout(slave3, 5, "us")
    assert read2_result[0] == CORRECT_DATA[0], (
        f"read2 beat[0] = 0x{read2_result[0]:016x}, expected 0x{CORRECT_DATA[0]:016x}"
    )

# ---------------------------------------------------------------------------
# BUG 2 REGRESSION: ST_WR_RESP fast-path samples AHB one cycle too early
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_regression_wr_resp_fastpath_wrong_awlen(dut):
    set_test_id(dut)
    """
    BUG 2a regression: AWLEN wrong when write follows write via ST_WR_RESP.

    The ST_WR_RESP fast-path sampled HBURST in the same cycle BVALID fires.
    HREADY goes high combinatorially from BVALID, so the AHB master's new
    address phase only arrives one cycle later.  The stale HBURST produced
    the wrong c_axlen, loading beat_cnt with an incorrect value, causing
    WLAST to fire too early — e.g. AWLEN=7 but only 4 W beats sent.

    Sequence:
      1. AHB INCR8 write @ ADDR_A (8 beats) — completes normally.
      2. AHB INCR8 write @ ADDR_B (8 beats) — bridge should produce
         AWLEN=7 and exactly 8 W beats before WLAST.

    On the BUGGY bridge: second write has WLAST at wrong beat → FAIL.
    On the FIXED bridge: both writes produce exactly 8 beats     → PASS.
    """
    ADDR_A = 0x00001000
    ADDR_B = 0x00002000
    DATA_A = [0xAAAA000000000000 + i for i in range(8)]
    DATA_B = [0xBBBB000000000000 + i for i in range(8)]

    await setup_dut_no_axi_slave(dut)
    mem = {}

    # ── Write 1 ──────────────────────────────────────────────────────────────
    dut._log.info("Write 1: INCR8 @ ADDR_A")
    slave1 = cocotb.start_soon(_sparse_write_slave_one_burst(dut, mem))
    await with_timeout(_ahb_write_incr8_manual(dut, ADDR_A, DATA_A), 500, "us")
    aw1_addr, aw1_len, beats1 = await with_timeout(slave1, 500, "us")
    dut._log.info(f"  write1: awlen={aw1_len} beats_received={len(beats1)} "
                  f"wlast_at={[i for i,(d,l) in enumerate(beats1) if l]}")

    # ── Write 2 (immediately after, same pipeline) ────────────────────────────
    dut._log.info("Write 2: INCR8 @ ADDR_B")
    slave2 = cocotb.start_soon(_sparse_write_slave_one_burst(dut, mem))
    await with_timeout(_ahb_write_incr8_manual(dut, ADDR_B, DATA_B), 500, "us")
    aw2_addr, aw2_len, beats2 = await with_timeout(slave2, 500, "us")
    dut._log.info(f"  write2: awlen={aw2_len} beats_received={len(beats2)} "
                  f"wlast_at={[i for i,(d,l) in enumerate(beats2) if l]}")

    # ── Assertions ───────────────────────────────────────────────────────────
    assert aw1_len == 7, f"Write1 AWLEN={aw1_len}, expected 7"
    assert len(beats1) == 8, f"Write1: {len(beats1)} beats received, expected 8"
    assert beats1[7][1] == 1, f"Write1: WLAST not on beat 7"

    assert aw2_len == 7, f"Write2 AWLEN={aw2_len}, expected 7"
    assert len(beats2) == 8, (
        f"BUG2a: Write2 sent {len(beats2)} beats (expected 8). "
        f"AWLEN={aw2_len} but WLAST fired early — ST_WR_RESP fast-path "
        f"loaded beat_cnt from stale HBURST."
    )
    assert beats2[7][1] == 1, (
        f"BUG2a: Write2 WLAST at wrong beat. beats={[(hex(d),l) for d,l in beats2]}"
    )

    # Verify data integrity in mem
    for i in range(8):
        addr = ADDR_B + i * 8
        got = mem.get(addr & ~0x7, 0)
        assert got == DATA_B[i], (
            f"BUG2a: mem[0x{addr:08x}] = 0x{got:016x}, expected 0x{DATA_B[i]:016x}"
        )
    dut._log.info("PASS: ST_WR_RESP AWLEN regression (Bug 2a)")


@cocotb.test()
async def test_regression_wr_resp_fastpath_missing_arvalid(dut):
    set_test_id(dut)
    """
    BUG 2b regression: ARVALID never asserted when read follows write.

    Same timing problem as Bug 2a but for write→read transition.
    The fast-path sampled HTRANS in the same cycle BVALID fires; since
    HREADY goes high combinatorially, the master's new NONSEQ only appears
    the next cycle.  The fast-path saw stale HTRANS != NONSEQ and fell
    through to ST_IDLE — but then ST_IDLE also saw nothing (the NONSEQ had
    already been presented and the master had moved on), so ARVALID was
    never asserted and the bridge deadlocked.

    Sequence:
      1. AHB INCR8 write @ WRITE_ADDR — completes normally.
      2. AHB INCR8 read  @ READ_ADDR  — must produce ARVALID within a
                                         reasonable number of cycles and
                                         return the correct data.

    On the BUGGY bridge: ARVALID never rises → timeout → FAIL.
    On the FIXED bridge: read completes with correct data → PASS.
    """
    WRITE_ADDR   = 0x00002000
    READ_ADDR    = 0x00001000
    WRITE_DATA   = [0xCCCC000000000000 + i for i in range(8)]
    CORRECT_DATA = [0x000000000000132A + i for i in range(8)]

    await setup_dut_no_axi_slave(dut)
    mem = {}
    for i in range(8):
        mem[(READ_ADDR & ~0x7) + i * 8] = CORRECT_DATA[i]

    dut._log.info("Write: INCR8 @ WRITE_ADDR")
    slave_wr = cocotb.start_soon(_sparse_write_slave_one_burst(dut, mem))
    write_task = cocotb.start_soon(
        _ahb_write_incr8_manual(dut, WRITE_ADDR, WRITE_DATA)
    )

    await with_timeout(write_task, 500, "us")

    dut._log.info("Read: INCR8 @ READ_ADDR (immediately after write)")
    slave_rd = cocotb.start_soon(_sparse_read_slave_one_burst(dut, mem, r_delay=2))
    read_result = await with_timeout(
        _ahb_read_incr8_manual_bug2b_sampled(dut, READ_ADDR),
        500, "us",
    )
    await slave_rd
    await slave_wr

    dut._log.info(f"  read beat[0] = 0x{read_result[0]:016x}")

    assert read_result[0] == CORRECT_DATA[0], (
        f"BUG2b: read beat[0] = 0x{read_result[0]:016x}, "
        f"expected 0x{CORRECT_DATA[0]:016x}. "
        f"Bridge may have deadlocked (ARVALID never asserted after write) "
        f"or returned wrong data."
    )
    for i, got in enumerate(read_result):
        assert got == CORRECT_DATA[i], (
            f"BUG2b: read beat[{i}] = 0x{got:016x}, "
            f"expected 0x{CORRECT_DATA[i]:016x}"
        )

    dut._log.info("PASS: ST_WR_RESP missing ARVALID regression (Bug 2b)")


# ---------------------------------------------------------------------------
# BUG 3 REGRESSION: INCR write immediately followed by read — pnd_valid fix
#
# Root cause (confirmed by hardware ILA capture 20260310_2029):
#   ST_WR_INCR_ACC asserts HREADY=1.  On the last SEQ beat the master
#   simultaneously presents the next NONSEQ (address phase of the following
#   transaction).  The bridge captures the last HWDATA and transitions to
#   ST_WR_INCR_FLUSH — without latching the next HADDR/HBURST/HWRITE.
#   By the time the bridge reaches ST_IDLE after the flush+B-response, the
#   master is presenting HTRANS=IDLE, so the read is silently dropped.
#   The CPU's ld instruction returns 0x0, ret jumps to 0x0, trap fires.
#
#   The fix adds a pnd_valid/pnd_addr/... register set.  ST_WR_INCR_ACC and
#   ST_WR_INCR_RESUME latch the NONSEQ when they exit with HREADY=1.
#   ST_IDLE checks pnd_valid first and dispatches from the latch.
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_regression_incr8_write_then_read_no_idle_gap(dut):
    set_test_id(dut)
    """
    BUG3: legal single-driver no-idle-gap write->read regression.

    We intentionally do NOT await the read-responder task as an observer.
    The AHB read result itself is the proof:
      - if the pending read is lost, this helper times out or returns wrong data
      - if it passes, the read launch/return path worked
    """
    WRITE_ADDR = 0x8003_FBC0
    READ_ADDR  = 0xBFFF_FBC0
    WRITE_DATA = [0xDEAD_BEEF_0000_0000 + i for i in range(8)]
    READ_DATA  = [0x0000_0000_0000_132A + i for i in range(8)]

    await setup_dut_no_axi_slave(dut)

    mem = {}
    for i in range(8):
        mem[READ_ADDR + i * 8] = READ_DATA[i]

    dut._log.info("BUG3: start write responder")
    slave_wr = cocotb.start_soon(_sparse_write_slave_one_burst(dut, mem))

    dut._log.info("BUG3: start read responder")
    slave_rd = cocotb.start_soon(_sparse_read_slave_one_burst(dut, mem, r_delay=2))

    dut._log.info("BUG3: drive legal no-gap AHB write->read sequence")
    read_result = await with_timeout(
        _ahb_write_incr8_then_read_incr8_no_idle_gap_bug3(
            dut, WRITE_ADDR, WRITE_DATA, READ_ADDR
        ),
        2, "us",
    )

    dut._log.info("BUG3: collect write responder result")
    wr_awaddr, wr_awlen, wr_beats = await with_timeout(slave_wr, 2, "us")

    # Do NOT await slave_rd here. In this no-gap sequence its task completion is
    # fragile as a bookkeeping mechanism, but the AHB read_result already proves
    # the AXI read path happened.
    slave_rd.kill()

    assert wr_awaddr == WRITE_ADDR, (
        f"BUG3 write: AWADDR=0x{wr_awaddr:08x}, expected 0x{WRITE_ADDR:08x}"
    )
    assert wr_awlen == 7, (
        f"BUG3 write: AWLEN={wr_awlen}, expected 7"
    )
    assert len(wr_beats) == 8, (
        f"BUG3 write: got {len(wr_beats)} beats, expected 8"
    )

    for i, got in enumerate(read_result):
        exp = READ_DATA[i]
        assert got == exp, (
            f"BUG3: read beat[{i}] = 0x{got:016x}, expected 0x{exp:016x}. "
            f"If beat[0] is 0x0 or the helper times out, the pending read was lost."
        )

    dut._log.info("PASS: BUG3 no-idle-gap write->read")

@cocotb.test()
async def test_regression_incr8_write_then_read_data_not_zero(dut):
    set_test_id(dut)
    """
    BUG 3 regression (exact ILA repro): verifies that a read immediately
    after an INCR8 write does NOT return all-zeros.
    """
    WRITE_ADDR  = 0x8003_FBC0
    STACK_ADDR  = 0xBFFF_FBC0
    RA_VALUE    = 0x0000_0000_0000_132A
    WRITE_DATA  = [0xCC_00_00_00_00_00_00_00 + i for i in range(8)]
    READ_DATA   = [RA_VALUE + i for i in range(8)]

    await setup_dut_no_axi_slave(dut)
    mem = {}
    for i in range(8):
        mem[STACK_ADDR + i * 8] = READ_DATA[i]

    slave_wr = cocotb.start_soon(_sparse_write_slave_one_burst(dut, mem))
    await with_timeout(_ahb_write_incr8_manual(dut, WRITE_ADDR, WRITE_DATA), 5, "us")

    slave_rd = cocotb.start_soon(_sparse_read_slave_one_burst(dut, mem, r_delay=10))
    result = await with_timeout(_ahb_read_incr8_manual_bug3_sampled(dut, STACK_ADDR), 5, "us")
    await with_timeout(slave_rd, 5, "us")
    await with_timeout(slave_wr, 5, "us")

    assert result[0] != 0, (
        f"BUG3: Read returned 0x0 at RA slot — bridge dropped the read."
    )
    assert result[0] == RA_VALUE, (
        f"BUG3: RA slot = 0x{result[0]:016x}, expected 0x{RA_VALUE:016x}"
    )
    for i, got in enumerate(result):
        assert got == READ_DATA[i], (
            f"BUG3: beat[{i}] = 0x{got:016x}, expected 0x{READ_DATA[i]:016x}"
        )




# ---------------------------------------------------------------------------
# BUG: stale rd_buf_valid captured as beat 0 when RVALID is held across
# the ST_RD_A → ST_RD_D transition.
#
# Root cause: in ST_RD_A, RREADY=0 on the cycle ARREADY=1. If the AXI slave
# (or CDC FIFO) presents RVALID=1 on that same cycle, the beat is not
# accepted — but on the very next cycle the bridge is in ST_RD_D with
# ar_done=1 and RREADY=1, so the still-asserted RVALID is captured
# immediately into rd_buf as beat 0 — with whatever data the slave had on
# the bus at the ARREADY cycle (stale / wrong).
#
# Fix: clear rd_buf_valid in ST_RD_A every cycle so no stale content can
# survive into ST_RD_D.
# ---------------------------------------------------------------------------

async def _sparse_read_slave_rvalid_on_arready_cycle(dut, mem, *, stale_value=0xDEAD_0000_DEAD_0000):
    """
    Serve one AXI read burst, but assert RVALID=stale_value simultaneously
    with ARREADY=1. This is the hazard: RREADY=0 on that cycle in ST_RD_A,
    so the beat is held. Next cycle (ST_RD_D, RREADY=1) it gets captured
    as beat 0 on a buggy bridge.
    After the hazard cycle, deassert RVALID for one cycle, then send the
    correct beats from mem.
    """
    while not int(dut.m_axi_arvalid.value):
        await RisingEdge(dut.clk)

    araddr  = int(dut.m_axi_araddr.value)
    arlen   = int(dut.m_axi_arlen.value)
    arsize  = int(dut.m_axi_arsize.value)
    arburst = int(dut.m_axi_arburst.value)

    # Assert ARREADY and RVALID=stale simultaneously — the hazard cycle.
    dut.m_axi_arready.value = 1
    dut.m_axi_rvalid.value  = 1
    dut.m_axi_rdata.value   = stale_value
    dut.m_axi_rlast.value   = 0
    dut.m_axi_rresp.value   = 0

    await RisingEdge(dut.clk)   # AR handshake fires; RREADY=0 so R not accepted
    dut.m_axi_arready.value = 0
    # Hold RVALID one more cycle to ensure it is seen in ST_RD_D
    await RisingEdge(dut.clk)
    dut.m_axi_rvalid.value = 0
    dut.m_axi_rdata.value  = 0

    # One idle cycle before correct data
    await RisingEdge(dut.clk)

    # Now send correct beats from mem
    for beat in range(arlen + 1):
        addr = araddr + beat * (1 << arsize) if arburst == 1 else araddr
        rdata = mem.get(addr & ~0x7, 0)
        dut.m_axi_rdata.value  = rdata
        dut.m_axi_rresp.value  = 0
        dut.m_axi_rlast.value  = 1 if beat == arlen else 0
        dut.m_axi_rvalid.value = 1
        while True:
            await RisingEdge(dut.clk)
            if int(dut.m_axi_rvalid.value) and int(dut.m_axi_rready.value):
                break
        dut.m_axi_rvalid.value = 0
        dut.m_axi_rlast.value  = 0

    return araddr


@cocotb.test()
async def test_regression_stale_rvalid_on_arready_cycle_poisons_beat0(dut):
    set_test_id(dut)
    """
    Regression: RVALID asserted on the ARREADY cycle (where RREADY=0 in
    ST_RD_A) causes the stale beat to be captured as beat 0 in ST_RD_D.

    The slave asserts RVALID=STALE_VALUE simultaneously with ARREADY=1.
    On a buggy bridge beat[0] = STALE_VALUE instead of the correct data.
    On the fixed bridge (rd_buf_valid cleared every cycle in ST_RD_A)
    beat[0] = correct data from mem.
    """
    READ_ADDR   = 0xBFFF_FBC0
    STALE_VALUE = 0x0000_0000_0000_0216
    CORRECT_DATA = [0x0000_0000_0000_132A + i for i in range(8)]

    await setup_dut_no_axi_slave(dut)
    mem = {}
    for i in range(8):
        mem[READ_ADDR + i * 8] = CORRECT_DATA[i]

    slave = cocotb.start_soon(
        _sparse_read_slave_rvalid_on_arready_cycle(
            dut, mem, stale_value=STALE_VALUE
        )
    )

    results = await with_timeout(
        _ahb_read_incr8_manual(dut, READ_ADDR),
        timeout_time=500, timeout_unit="us"
    )
    await slave

    assert results[0] != STALE_VALUE, (
        f"REGRESSION: beat[0] = 0x{results[0]:016x} = STALE_VALUE. "
        f"rd_buf_valid was not cleared in ST_RD_A — stale RVALID on ARREADY "
        f"cycle was captured as beat 0."
    )
    for i, (got, exp) in enumerate(zip(results, CORRECT_DATA)):
        assert got == exp, (
            f"beat[{i}] = 0x{got:016x}, expected 0x{exp:016x}"
        )

    dut._log.info("PASS: stale RVALID on ARREADY cycle correctly rejected")


#!/usr/bin/env python3
"""
==========================================================================
BUG #8 REGRESSION TEST
==========================================================================

HOW TO INTEGRATE:
  1. Your cocotb wrapper must expose `hsel` and `hreadyin` as independently
     drivable top-level ports (not tied internally). See tb_ahb_to_axi4_burst.sv.

  2. Patch TWO existing functions so all prior tests still work:

     a) _init_direct_ahb_signals -- add two lines at the end:
            dut.hsel.value = 1
            dut.hreadyin.value = 1

     b) _wait_hready -- add one line after each `await RisingEdge(dut.clk)`:
            dut.hreadyin.value = int(dut.hready.value)

        Specifically, replace:
            async def _wait_hready(dut, timeout_cycles=2000):
                for _ in range(timeout_cycles):
                    await RisingEdge(dut.clk)
                    if int(dut.hready.value):
                        return
                raise TimeoutError("hready never went high")

        With:
            async def _wait_hready(dut, timeout_cycles=2000):
                for _ in range(timeout_cycles):
                    await RisingEdge(dut.clk)
                    dut.hreadyin.value = int(dut.hready.value)
                    if int(dut.hready.value):
                        return
                raise TimeoutError("hready never went high")

        This makes HREADYIN track HREADY for all existing tests (identical
        to the old tied behavior) while allowing the bug8 test to override it.

     c) setup_dut_no_axi_slave -- add after _init_manual_axi_slave_inputs(dut):
            dut.hsel.value = 1
            dut.hreadyin.value = 1

  3. Append the test code below to the end of test_ahb_to_axi4_burst_v53.py.

==========================================================================
"""

# ---------- Paste everything below this line at the END of the test file ----------


# ---------------------------------------------------------------------------
# Bug #8 helpers
# ---------------------------------------------------------------------------

async def _bug8_axi_write_slave_multi(dut, mem, burst_count, b_delay=2):
    """
    Service `burst_count` AXI write bursts into `mem`.
    Returns list of (awaddr, awlen, beat_list) per burst.
    beat_list entries are (wdata, wstrb, wlast) tuples.
    b_delay: cycles between last W beat accepted and BVALID assertion,
             simulating CDC + crossbar latency seen on real hardware.
    """
    results = []

    for burst_idx in range(burst_count):
        # Wait for AW
        while not int(dut.m_axi_awvalid.value):
            await RisingEdge(dut.clk)
        dut.m_axi_awready.value = 1
        while True:
            await RisingEdge(dut.clk)
            if int(dut.m_axi_awvalid.value) and int(dut.m_axi_awready.value):
                awaddr  = int(dut.m_axi_awaddr.value)
                awlen   = int(dut.m_axi_awlen.value)
                awsize  = int(dut.m_axi_awsize.value)
                awburst = int(dut.m_axi_awburst.value)
                break
        dut.m_axi_awready.value = 0

        # Collect W beats
        beats = []
        while True:
            dut.m_axi_wready.value = 1
            while True:
                await RisingEdge(dut.clk)
                if int(dut.m_axi_wvalid.value) and int(dut.m_axi_wready.value):
                    wdata = int(dut.m_axi_wdata.value)
                    wstrb = int(dut.m_axi_wstrb.value)
                    wlast = int(dut.m_axi_wlast.value)
                    break
            dut.m_axi_wready.value = 0

            addr = awaddr + len(beats) * (1 << awsize) if awburst == 1 else awaddr
            _sparse_apply_wbeat(mem, addr, wdata, wstrb)
            beats.append((wdata, wstrb, wlast))

            if wlast or len(beats) > awlen + 2:
                break

        # B response with delay
        for _ in range(b_delay):
            await RisingEdge(dut.clk)

        dut.m_axi_bvalid.value = 1
        dut.m_axi_bresp.value  = 0
        while True:
            await RisingEdge(dut.clk)
            if int(dut.m_axi_bvalid.value) and int(dut.m_axi_bready.value):
                break
        dut.m_axi_bvalid.value = 0

        results.append((awaddr, awlen, beats))

    return results


# ---------------------------------------------------------------------------
# Bug #8 regression test
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_regression_bug8_back_to_back_incr8_writes_dropped(dut):
    set_test_id(dut)
    """
    Bug #8: second of two back-to-back INCR8 writes dropped.

    Root cause: in ST_WR_RESP, the bridge only checked for a new NONSEQ
    inside the `if (BVALID)` branch.  When BVALID took multiple cycles
    (CDC + crossbar latency), the AHB master could present NONSEQ while
    HREADYIN=1 (from the SoC mux).  The master advanced past NONSEQ to
    SEQ before BVALID arrived, so the bridge never latched pnd_valid.
    The entire second burst was silently discarded.

    This reproduces the exact ILA-observed sequence:
      - Burst 1 completes, bridge enters WR_RESP (HREADY=0)
      - NONSEQ for burst 2 appears with HREADYIN=1 (SoC mux artifact)
      - Master advances to SEQ (it saw HREADYIN=1)
      - BVALID arrives later; bridge sees SEQ, not NONSEQ -> drops burst 2

    PASS: both bursts appear on AXI with correct AWADDR, AWLEN, and WDATA.
    FAIL (buggy bridge): AXI slave times out waiting for second burst.
    """
    ADDR_A = 0x00001000   # first cache-line write-back
    ADDR_B = 0x00001040   # second (adjacent), like D-cache eviction
    DATA_A = [0xAA00_0000_0000_0000 | i for i in range(8)]
    DATA_B = [0xBB00_0000_0000_0000 | i for i in range(8)]

    HBURST_INCR8 = 0b101
    HSIZE_64     = 3
    B_DELAY      = 4   # cycles before BVALID — must be > 2 to ensure
                        # the NONSEQ injection happens while BVALID=0

    await setup_dut_no_axi_slave(dut)
    dut.hsel.value     = 1
    dut.hreadyin.value = 1
    mem = {}

    # Start AXI slave expecting TWO bursts
    slave_task = cocotb.start_soon(
        _bug8_axi_write_slave_multi(dut, mem, burst_count=2, b_delay=B_DELAY)
    )

    # ==================================================================
    # Burst 1: INCR8 write at ADDR_A
    # Drive NONSEQ + 7 SEQ address phases. On beat 7, set IDLE.
    # ==================================================================
    dut.haddr.value  = ADDR_A
    dut.hburst.value = HBURST_INCR8
    dut.hsize.value  = HSIZE_64
    dut.htrans.value = 0b10   # NONSEQ
    dut.hwrite.value = 1
    dut.hwdata.value = 0

    for beat in range(8):
        # Wait for bridge HREADY=1 (tracking HREADYIN = HREADY)
        while True:
            await RisingEdge(dut.clk)
            await ReadOnly()
            hr = int(dut.hready.value)
            await NextTimeStep()
            dut.hreadyin.value = hr
            if hr:
                break

        # Set up next address phase + data for current beat
        if beat == 7:
            # Last address phase accepted. Go IDLE, provide last data beat.
            dut.htrans.value = 0b00   # IDLE
            dut.haddr.value  = 0
            dut.hburst.value = 0
            dut.hsize.value  = 0
            dut.hwrite.value = 0
        else:
            dut.htrans.value = 0b11   # SEQ
            dut.haddr.value  = ADDR_A + (beat + 1) * 8

        dut.hwdata.value = DATA_A[beat]

    # ==================================================================
    # Critical window: DO NOT wait for HREADY=1 here.
    #
    # The bridge is now processing the last W beat and will enter
    # WR_RESP with HREADY=0 (BVALID pending). The AXI slave will
    # assert BVALID after B_DELAY cycles.
    #
    # We need to inject NONSEQ for burst 2 during this HREADY=0
    # window, with HREADYIN=1, to reproduce the SoC mux behavior.
    #
    # Wait exactly 2 cycles: the bridge needs 1 cycle for the last
    # WR_D beat and 1 cycle to enter WR_RESP.
    # ==================================================================

    # Cycle 1: bridge processes last WR_D beat (beat_cnt=0) -> WR_RESP
    await RisingEdge(dut.clk)
    await NextTimeStep()
    dut.hreadyin.value = 0  # bridge HREADY is 0 now

    # Cycle 2: bridge is in WR_RESP, HREADY=0, BVALID=0 (B_DELAY > 2)
    await RisingEdge(dut.clk)
    await NextTimeStep()

    # ==================================================================
    # Inject NONSEQ for burst 2 with HREADYIN=1
    # This is the exact scenario from the ILA: the SoC mux still has
    # HREADY=1 from the previous cycle's routing, so the master
    # presents NONSEQ even though the bridge's own HREADY=0.
    # ==================================================================
    dut.haddr.value    = ADDR_B
    dut.hburst.value   = HBURST_INCR8
    dut.hsize.value    = HSIZE_64
    dut.htrans.value   = 0b10   # NONSEQ  <-- the critical signal
    dut.hwrite.value   = 1
    dut.hsel.value     = 1
    dut.hreadyin.value = 1      # <-- KEY: master saw system HREADY=1

    # One cycle: NONSEQ visible to bridge with HREADYIN=1, HREADY=0
    await RisingEdge(dut.clk)
    await NextTimeStep()

    # Master advances to SEQ (it thinks address was accepted via HREADYIN)
    dut.htrans.value   = 0b11   # SEQ
    dut.haddr.value    = ADDR_B + 8
    dut.hwdata.value   = DATA_B[0]   # data phase for beat 0 (NONSEQ's data)
    dut.hreadyin.value = 0            # stop forcing; track bridge HREADY now

    # ==================================================================
    # Burst 2: remaining SEQ beats, paced by bridge HREADY
    # ==================================================================
    for beat in range(1, 8):
        while True:
            await RisingEdge(dut.clk)
            await ReadOnly()
            hr = int(dut.hready.value)
            await NextTimeStep()
            dut.hreadyin.value = hr
            if hr:
                break

        if beat == 7:
            dut.htrans.value = 0b00   # IDLE
            dut.haddr.value  = 0
            dut.hburst.value = 0
            dut.hsize.value  = 0
            dut.hwrite.value = 0
        else:
            dut.htrans.value = 0b11   # SEQ
            dut.haddr.value  = ADDR_B + (beat + 1) * 8

        dut.hwdata.value = DATA_B[beat]

    # Wait for last beat acceptance
    while True:
        await RisingEdge(dut.clk)
        await ReadOnly()
        hr = int(dut.hready.value)
        await NextTimeStep()
        dut.hreadyin.value = hr
        if hr:
            break

    _init_direct_ahb_signals(dut)
    dut.hsel.value     = 1
    dut.hreadyin.value = 1

    # Let everything settle
    await ClockCycles(dut.clk, 30)

    # ==================================================================
    # Verify: AXI slave must have received TWO complete write bursts
    # ==================================================================
    try:
        results = await with_timeout(slave_task, 2000, "us")
    except Exception as e:
        raise AssertionError(
            f"BUG8: AXI slave timed out — the bridge dropped the second "
            f"INCR8 write. The NONSEQ arrived during ST_WR_RESP while "
            f"HREADYIN=1, but the bridge only checked for NONSEQ inside "
            f"the BVALID branch. Error: {e}"
        )

    assert len(results) == 2, (
        f"BUG8: Expected 2 AXI write bursts, got {len(results)}. "
        f"The bridge dropped the second INCR8 write."
    )

    # ---- Burst 1 ----
    aw1_addr, aw1_len, beats1 = results[0]
    assert aw1_addr == ADDR_A, (
        f"Burst 1 AWADDR=0x{aw1_addr:08x}, expected 0x{ADDR_A:08x}"
    )
    assert aw1_len == 7, f"Burst 1 AWLEN={aw1_len}, expected 7"
    assert len(beats1) == 8, f"Burst 1: {len(beats1)} W beats, expected 8"
    for i, (wd, ws, wl) in enumerate(beats1):
        assert wd == DATA_A[i], (
            f"Burst 1 beat {i}: WDATA=0x{wd:016x}, expected 0x{DATA_A[i]:016x}"
        )

    # ---- Burst 2 ----
    aw2_addr, aw2_len, beats2 = results[1]
    assert aw2_addr == ADDR_B, (
        f"BUG8: Burst 2 AWADDR=0x{aw2_addr:08x}, expected 0x{ADDR_B:08x}"
    )
    assert aw2_len == 7, f"Burst 2 AWLEN={aw2_len}, expected 7"
    assert len(beats2) == 8, (
        f"BUG8: Burst 2 sent {len(beats2)} W beats, expected 8"
    )
    for i, (wd, ws, wl) in enumerate(beats2):
        assert wd == DATA_B[i], (
            f"BUG8: Burst 2 beat {i}: WDATA=0x{wd:016x}, "
            f"expected 0x{DATA_B[i]:016x}"
        )

    # ---- Verify memory ----
    for i in range(8):
        addr = ADDR_A + i * 8
        got = mem.get(addr, 0)
        assert got == DATA_A[i], (
            f"mem[0x{addr:08x}]=0x{got:016x}, expected 0x{DATA_A[i]:016x}"
        )
    for i in range(8):
        addr = ADDR_B + i * 8
        got = mem.get(addr, 0)
        assert got == DATA_B[i], (
            f"BUG8: mem[0x{addr:08x}]=0x{got:016x}, "
            f"expected 0x{DATA_B[i]:016x}. Second burst data was lost."
        )

    dut._log.info(
        "PASS: Bug #8 regression — back-to-back INCR8 writes with "
        "HREADYIN=1 during ST_WR_RESP correctly captured via pnd_valid"
    )