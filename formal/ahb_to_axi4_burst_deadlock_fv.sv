// =============================================================================
// ahb_to_axi4_burst_v11c_deadlock_fv.sv — Formal verification for deadlock prevention
//
// FOCUS: Write completion ordering to prevent deadlocks when B response is delayed.
// KEY ISSUE: Read must not launch until write B response is fully accepted.
// This was found to be violated in the BMC check - a real bug!
// =============================================================================

`default_nettype none

module ahb_to_axi4_burst_deadlock_fv;

  // ── Parameters ─────────────────────────────────────────────────────────────
  localparam int unsigned AW             = 32;
  localparam int unsigned DW             = 64;
  localparam int unsigned IW             = 4;
  localparam int unsigned MAX_INCR_BEATS = 16;
  localparam int unsigned IBUF_IDX_W     = 4;
  localparam int unsigned IBUF_CNT_W     = 5;

  // ── State encoding ──────────────────────────────────────────────────────────
  localparam logic [4:0] ST_IDLE              = 5'd0;
  localparam logic [4:0] ST_WR_PND_ALIGN      = 5'd1;
  localparam logic [4:0] ST_WR_D              = 5'd2;
  localparam logic [4:0] ST_WR_FIX_FLUSH      = 5'd3;
  localparam logic [4:0] ST_WR_INCR_ACC       = 5'd4;
  localparam logic [4:0] ST_WR_INCR_POST_BUSY = 5'd5;
  localparam logic [4:0] ST_WR_INCR_RESUME    = 5'd6;
  localparam logic [4:0] ST_WR_INCR_FLUSH     = 5'd7;
  localparam logic [4:0] ST_WR_RESP           = 5'd8;
  localparam logic [4:0] ST_WR_LAST_RESP      = 5'd9;
  localparam logic [4:0] ST_WR_ERR            = 5'd10;
  localparam logic [4:0] ST_RD_A              = 5'd11;
  localparam logic [4:0] ST_RD_D              = 5'd12;
  localparam logic [4:0] ST_RD_INCR_WAIT      = 5'd13;
  localparam logic [4:0] ST_RD_ERR            = 5'd14;
  localparam logic [4:0] ST_RD_DRAIN          = 5'd15;
  localparam logic [4:0] ST_RD_FENCE          = 5'd16;

  // ── All signals ─────────────────────────────────────────────────────────────
  logic                clk;
  logic                resetn;

  // AHB slave-side inputs
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

  // AXI slave-side inputs
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

  // DUT outputs
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

  // ── DUT instance ───────────────────────────────────────────────────────────
  ahb_to_axi4_burst #(
    .AW(AW), .DW(DW), .IW(IW), .MAX_INCR_BEATS(MAX_INCR_BEATS)
  ) dut (
    .clk      (clk),      .resetn   (resetn),
    .HSEL     (HSEL),     .HREADYIN (HREADYIN), .HADDR    (HADDR),
    .HBURST   (HBURST),   .HMASTLOCK(HMASTLOCK),.HPROT    (HPROT),
    .HSIZE    (HSIZE),    .HTRANS   (HTRANS),   .HWDATA   (HWDATA),
    .HWRITE   (HWRITE),   .HRDATA   (HRDATA),   .HREADY   (HREADY),
    .HRESP    (HRESP),
    .AWID     (AWID),     .AWADDR   (AWADDR),   .AWLEN    (AWLEN),
    .AWSIZE   (AWSIZE),   .AWBURST  (AWBURST),  .AWLOCK   (AWLOCK),
    .AWCACHE  (AWCACHE),  .AWPROT   (AWPROT),   .AWQOS    (AWQOS),
    .AWVALID  (AWVALID),  .AWREADY  (AWREADY),
    .WDATA    (WDATA),    .WSTRB    (WSTRB),    .WLAST    (WLAST),
    .WVALID   (WVALID),   .WREADY   (WREADY),
    .BID      (BID),      .BRESP    (BRESP),    .BVALID   (BVALID),
    .BREADY   (BREADY),
    .ARID     (ARID),     .ARADDR   (ARADDR),   .ARLEN    (ARLEN),
    .ARSIZE   (ARSIZE),   .ARBURST  (ARBURST),  .ARLOCK   (ARLOCK),
    .ARCACHE  (ARCACHE),  .ARPROT   (ARPROT),   .ARQOS    (ARQOS),
    .ARVALID  (ARVALID),  .ARREADY  (ARREADY),
    .RID      (RID),      .RDATA    (RDATA),    .RRESP    (RRESP),
    .RLAST    (RLAST),    .RVALID   (RVALID),   .RREADY   (RREADY)
  );

  // ── Ghost signals for tracking write-in-flight state ─────────────────────────
  logic write_in_flight;

  // ═══════════════════════════════════════════════════════════════════════════
  //  GHOST SIGNAL TRACKING: Write-in-flight flag
  // ═══════════════════════════════════════════════════════════════════════════

  always_ff @(posedge clk) begin
    if (!resetn)
      write_in_flight <= 1'b0;
    else if (AWVALID && AWREADY)
      write_in_flight <= 1'b1;
    else if (BVALID && BREADY)
      write_in_flight <= 1'b0;
  end

  // ═══════════════════════════════════════════════════════════════════════════
  //  ASSUMPTIONS: Constrain environment for deadlock verification
  // ═══════════════════════════════════════════════════════════════════════════

  // Assume AXI responses are eventually provided (no infinite stalls)
  assume property (AWVALID && AWREADY |-> ##[1:100] WVALID && WREADY);
  assume property (WVALID && WREADY |-> ##[1:100] BVALID);
  assume property (ARVALID && ARREADY |-> ##[1:100] RVALID);

  // Assume BREADY is always ready (focus on ordering, not backpressure)
  assume property (BVALID |-> BREADY);

  // Assume RREADY is always ready
  assume property (RVALID |-> RREADY);

  // ═══════════════════════════════════════════════════════════════════════════
  //  DEADLOCK PREVENTION PROPERTIES
  // ═══════════════════════════════════════════════════════════════════════════

  // **CRITICAL ASSERTION: No read launch while write B pending**
  // This FAILED in BMC - found a real bug!
  assert property (!write_in_flight || !ARVALID);

  // **LIVENESS: Write responses must eventually complete**
  assert property (write_in_flight |-> ##[1:200] !write_in_flight);

  // **PROGRESS: Bridge must eventually accept new transactions**
  assert property (HSEL && HREADYIN |-> ##[1:100] HREADY);

  // ═══════════════════════════════════════════════════════════════════════════
  //  COVERAGE: Ensure deadlock scenarios are tested
  // ═══════════════════════════════════════════════════════════════════════════

  // Cover write followed by read with delayed B response
  cover property (
    (AWVALID && AWREADY) ##[1:20] (!BVALID) ##[1:20] (ARVALID && ARREADY)
  );

  // Cover back-to-back write+read sequences
  cover property (
    (AWVALID && AWREADY) ##[1:10] (ARVALID && ARREADY)
  );

  // Cover delayed B response scenario
  cover property (
    (WVALID && WREADY && WLAST) ##[10:50] BVALID
  );

endmodule
