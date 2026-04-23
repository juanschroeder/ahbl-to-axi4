// =============================================================================
// ahb_to_axi4_burst_v11c_linux_hang_fv.sv
//
// KEY PROPERTY: ARVALID must not assert while a write B response is pending.
// =============================================================================

`default_nettype none

module ahb_to_axi4_burst_v11c_linux_hang_fv;

  localparam int unsigned AW             = 32;
  localparam int unsigned DW             = 64;
  localparam int unsigned IW             = 4;
  localparam int unsigned MAX_INCR_BEATS = 16;

  logic                clk;
  logic                resetn;

  logic                HSEL;
  logic                HREADYIN;
  logic [AW-1:0]       HADDR;
  logic [2:0]          HBURST;
  logic                HMASTLOCK;
  logic [3:0]          HPROT;
  logic [2:0]          HSIZE;
  logic [1:0]          HTRANS;
  logic [DW-1:0]       HWDATA;
  logic                HWRITE;

  logic                AWREADY;
  logic                WREADY;
  logic [IW-1:0]       BID;
  logic [1:0]          BRESP;
  logic                BVALID;
  logic                ARREADY;
  logic [IW-1:0]       RID;
  logic [DW-1:0]       RDATA;
  logic [1:0]          RRESP;
  logic                RLAST;
  logic                RVALID;

  logic [DW-1:0]       HRDATA;
  logic                HREADY;
  logic                HRESP;
  logic [IW-1:0]       AWID;
  logic [AW-1:0]       AWADDR;
  logic [7:0]          AWLEN;
  logic [2:0]          AWSIZE;
  logic [1:0]          AWBURST;
  logic                AWLOCK;
  logic [3:0]          AWCACHE;
  logic [2:0]          AWPROT;
  logic [3:0]          AWQOS;
  logic                AWVALID;
  logic [DW-1:0]       WDATA;
  logic [DW/8-1:0]     WSTRB;
  logic                WLAST;
  logic                WVALID;
  logic                BREADY;
  logic [IW-1:0]       ARID;
  logic [AW-1:0]       ARADDR;
  logic [7:0]          ARLEN;
  logic [2:0]          ARSIZE;
  logic [1:0]          ARBURST;
  logic                ARLOCK;
  logic [3:0]          ARCACHE;
  logic [2:0]          ARPROT;
  logic [3:0]          ARQOS;
  logic                ARVALID;
  logic                RREADY;

  ahb_to_axi4_burst #(
    .AW(AW), .DW(DW), .IW(IW), .MAX_INCR_BEATS(MAX_INCR_BEATS)
  ) dut (
    .clk(clk), .resetn(resetn),
    .HSEL(HSEL), .HREADYIN(HREADYIN), .HADDR(HADDR),
    .HBURST(HBURST), .HMASTLOCK(HMASTLOCK), .HPROT(HPROT),
    .HSIZE(HSIZE), .HTRANS(HTRANS), .HWDATA(HWDATA),
    .HWRITE(HWRITE), .HRDATA(HRDATA), .HREADY(HREADY), .HRESP(HRESP),
    .AWID(AWID), .AWADDR(AWADDR), .AWLEN(AWLEN),
    .AWSIZE(AWSIZE), .AWBURST(AWBURST), .AWLOCK(AWLOCK),
    .AWCACHE(AWCACHE), .AWPROT(AWPROT), .AWQOS(AWQOS),
    .AWVALID(AWVALID), .AWREADY(AWREADY),
    .WDATA(WDATA), .WSTRB(WSTRB), .WLAST(WLAST),
    .WVALID(WVALID), .WREADY(WREADY),
    .BID(BID), .BRESP(BRESP), .BVALID(BVALID), .BREADY(BREADY),
    .ARID(ARID), .ARADDR(ARADDR), .ARLEN(ARLEN),
    .ARSIZE(ARSIZE), .ARBURST(ARBURST), .ARLOCK(ARLOCK),
    .ARCACHE(ARCACHE), .ARPROT(ARPROT), .ARQOS(ARQOS),
    .ARVALID(ARVALID), .ARREADY(ARREADY),
    .RID(RID), .RDATA(RDATA), .RRESP(RRESP),
    .RLAST(RLAST), .RVALID(RVALID), .RREADY(RREADY)
  );

  // Ghost: write in flight from AW accepted until B accepted
  logic write_in_flight;
  initial write_in_flight = 1'b0;
  always_ff @(posedge clk) begin
    if (!resetn)
      write_in_flight <= 1'b0;
    else if (AWVALID && AWREADY)
      write_in_flight <= 1'b1;
    else if (BVALID && BREADY)
      write_in_flight <= 1'b0;
  end

  // Ghost: WLAST fired last cycle -> next cycle BVALID must be high
  logic wlast_fired;
  initial wlast_fired = 1'b0;
  always_ff @(posedge clk) begin
    if (!resetn) wlast_fired <= 1'b0;
    else         wlast_fired <= WVALID && WREADY && WLAST;
  end

  // ── Environment assumptions ─────────────────────────────────────────────────
  // Force reset at step 0
  assume property (!$initstate || !resetn);

  // Slave accepts W immediately
  assume property (!WVALID || WREADY);
  // Slave accepts AR immediately
  assume property (!ARVALID || ARREADY);
  // Slave always accepts B
  assume property (!BVALID || BREADY);
  // Slave always accepts R
  assume property (!RVALID || RREADY);
  // BVALID must follow one cycle after WLAST (via ghost register)
  assume property (!wlast_fired || BVALID);
  // Reasonable size
  assume property (HSIZE <= 3'd3);

  // ── KEY PROPERTY ────────────────────────────────────────────────────────────
  // No read address issued while write B response is pending
  assert property (!write_in_flight || !ARVALID);

  // ── Coverage ────────────────────────────────────────────────────────────────
  cover property (WVALID && WREADY && WLAST);
  cover property (BVALID && BREADY);
  cover property (ARVALID && ARREADY);

endmodule
