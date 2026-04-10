// ============================================================================
// ahb_to_axi4_burst.sv
//
// Full-featured AHB-Lite ??? AXI4 bridge designed for Core-V Wally SoC.
//
// SUPPORTED FEATURES
//   ??? All HBURST types: SINGLE, INCR, INCR4/8/16, WRAP4/8/16
//   ??? Fixed-length bursts (SINGLE/INCR4/8/16/WRAP4/8/16):
//       AW issued immediately on NONSEQ; W channel lock-step with AHB.
//   ??? INCR (undefined length) WRITE bursts:
//       AHB data beats accumulated into a FIFO while HTRANS=SEQ/BUSY.
//       HREADY=0 during BUSY so the master holds write data stable until
//       a non-BUSY cycle (avoiding data-phase completion during BUSY).
//       After accumulation AW+W are issued together. Depth: MAX_INCR_BEATS.
//       When buffer fills mid-burst (more SEQ beats coming), flush current
//       buffer as an AXI transaction and resume accumulation.
//   ??? INCR (undefined length) READ bursts:
//       Handled as chunked multi-beat AXI transactions. ARLEN set to up to
//       MAX_INCR_BEATS-1 (capped at the 4 KB AXI boundary). When a chunk
//       is fully consumed and the AHB master still presents SEQ, a new AR
//       is issued for the next chunk. If the master terminates the burst
//       mid-chunk (IDLE/NONSEQ), remaining AXI R beats are drained.
//   ??? AHB two-cycle error response (spec ??3.5):
//       Cycle 1: HRESP=1, HREADY=0
//       Cycle 2: HRESP=1, HREADY=1
//   ??? Mid-burst read error: remaining AXI R beats drained (ST_RD_DRAIN).
//   ??? Zero-bubble back-to-back pipelining on OK completions.
//   ??? HWSTRB forwarded directly to WSTRB.
//   ??? HPROT[1:0] ??? AXPROT; HMASTLOCK ??? AxLOCK.
//
// LIMITATIONS
//   ??? Combinational path WREADY ??? HREADY on fixed-length write path (fixed in v5+).
//   ??? Single outstanding transaction per direction.
//
// v7 additions (vs v6d):
//   ??? AWCACHE/ARCACHE derived from HPROT[3:2]: HPROT[2]=bufferable???AXCACHE[0],
//       HPROT[3]=cacheable???AXCACHE[1].  Matches GRLIB mapping.
//   ??? pnd_wfirst_data capture gated on HREADYIN: only sample HWDATA when the
//       upstream bus is presenting valid write data.  Prevents data corruption
//       when a stall (HREADYIN=0) changes the live bus before the data phase.
//
// License: Apache 2.0
// ============================================================================

`default_nettype none

module ahb_to_axi4_burst #(
  parameter int unsigned AW             = 32,
  parameter int unsigned DW             = 64,
  parameter int unsigned IW             = 4,
  parameter int unsigned MAX_INCR_BEATS = 16
) (
  input  wire               clk,
  input  wire               resetn,

  // ?????? AHB-Lite Slave ??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  input  wire               HSEL,
  input  wire               HREADYIN,
  input  wire  [AW-1:0]     HADDR,
  input  wire  [2:0]        HBURST,
  input  wire               HMASTLOCK,
  input  wire  [3:0]        HPROT,
  input  wire  [2:0]        HSIZE,
  input  wire  [1:0]        HTRANS,
  input  wire  [DW-1:0]     HWDATA,
  //input  wire  [DW/8-1:0]   HWSTRB,
  input  wire               HWRITE,
  output logic [DW-1:0]     HRDATA,
  output logic              HREADY,
  output logic              HRESP,

  // ?????? AXI4 Master: Write Address ??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  output logic [IW-1:0]     AWID,
  output logic [AW-1:0]     AWADDR,
  output logic [7:0]        AWLEN,
  output logic [2:0]        AWSIZE,
  output logic [1:0]        AWBURST,
  output logic              AWLOCK,
  output logic [3:0]        AWCACHE,
  output logic [2:0]        AWPROT,
  output logic [3:0]        AWQOS,
  output logic              AWVALID,
  input  wire               AWREADY,

  // ?????? AXI4 Master: Write Data ???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  output logic [DW-1:0]     WDATA,
  output logic [DW/8-1:0]   WSTRB,
  output logic              WLAST,
  output logic              WVALID,
  input  wire               WREADY,

  // ?????? AXI4 Master: Write Response ???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  input  wire  [IW-1:0]     BID,
  input  wire  [1:0]        BRESP,
  input  wire               BVALID,
  output logic              BREADY,

  // ?????? AXI4 Master: Read Address ?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  output logic [IW-1:0]     ARID,
  output logic [AW-1:0]     ARADDR,
  output logic [7:0]        ARLEN,
  output logic [2:0]        ARSIZE,
  output logic [1:0]        ARBURST,
  output logic              ARLOCK,
  output logic [3:0]        ARCACHE,
  output logic [2:0]        ARPROT,
  output logic [3:0]        ARQOS,
  output logic              ARVALID,
  input  wire               ARREADY,

  // ?????? AXI4 Master: Read Data ??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  input  wire  [IW-1:0]     RID,
  input  wire  [DW-1:0]     RDATA,
  input  wire  [1:0]        RRESP,
  input  wire               RLAST,
  input  wire               RVALID,
  output logic              RREADY
);

  // ?????? AHB HTRANS encoding ??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  localparam logic [1:0] TRN_IDLE   = 2'b00;
  localparam logic [1:0] TRN_BUSY   = 2'b01;
  localparam logic [1:0] TRN_NONSEQ = 2'b10;
  localparam logic [1:0] TRN_SEQ    = 2'b11;

  // ?????? AHB HBURST encoding ??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  localparam logic [2:0] HB_SINGLE = 3'b000;
  localparam logic [2:0] HB_INCR   = 3'b001;
  localparam logic [2:0] HB_WRAP4  = 3'b010;
  localparam logic [2:0] HB_INCR4  = 3'b011;
  localparam logic [2:0] HB_WRAP8  = 3'b100;
  localparam logic [2:0] HB_INCR8  = 3'b101;
  localparam logic [2:0] HB_WRAP16 = 3'b110;
  localparam logic [2:0] HB_INCR16 = 3'b111;

  // ?????? AXI4 AxBURST encoding ????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  localparam logic [1:0] AXI_INCR = 2'b01;
  localparam logic [1:0] AXI_WRAP = 2'b10;

  // ?????? INCR write buffer sizing ???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  localparam int unsigned IBUF_IDX_W = $clog2(MAX_INCR_BEATS);
  localparam int unsigned IBUF_CNT_W = IBUF_IDX_W + 1;
  localparam logic [12:0]       MAX_INCR_BEATS_13 = MAX_INCR_BEATS;
  localparam logic [IBUF_CNT_W-1:0] MAX_INCR_BEATS_M1 = MAX_INCR_BEATS - 1;

  // ?????? State machine ???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  typedef enum logic [4:0] {
    ST_IDLE,                // Ready; watching for NONSEQ
    ST_WR_PND_ALIGN,        // One-cycle stall to align pending fixed-write beat 0
    ST_WR_D,                // Fixed-length write: accumulate AHB beats into buffer
    ST_WR_FIX_FLUSH,        // Fixed-length write: drain buffered beats to AXI W
    ST_WR_INCR_ACC,         // INCR write: accumulate AHB beats into buffer
    ST_WR_INCR_POST_BUSY,   // INCR write: one-cycle stall after BUSY so HWDATA settles
    ST_WR_INCR_RESUME,      // INCR write: one free-running cycle after POST_BUSY; do not capture
    ST_WR_INCR_FLUSH,       // INCR write: drain buffer to AXI W channel
    ST_WR_RESP,             // INCR flush path: waiting for AXI B-channel
    ST_WR_LAST_RESP,        // Fixed write: last W accepted, AHB waits for AXI B
    ST_WR_ERR,              // AHB 2-cycle error cycle 2, write
    ST_RD_A,                // Read: present AR, wait for ARREADY
    ST_RD_D,                // Read: forward R beats to AHB
    ST_RD_INCR_WAIT,        // INCR read: last beat done, HTRANS=BUSY; stall until resolved
    ST_RD_ERR,              // AHB 2-cycle error cycle 2, read
    ST_RD_DRAIN,             // Read: drain remaining AXI R beats after error or early INCR end
    ST_RD_FENCE             // Drain/flush any late stale R beats before next read transaction   
  } state_t;

  (* mark_debug = "true" *) state_t state;

  // ?????? Address-phase capture registers ??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  logic [AW-1:0]   ap_addr;
  logic [2:0]      ap_size;
  logic [1:0]      ap_axburst;
  logic [7:0]      ap_axlen;
  logic [3:0]      ap_prot;
  logic            ap_lock;

  // ?????? Beat counter (fixed-length write and all read paths) ??????????????????????????????????????????????????????????????????
  (* mark_debug = "true" *) logic [7:0] beat_cnt;

  // ?????? AW-sent flag (shared by both write paths) ???????????????????????????????????????????????????????????????????????????????????????????????????
  (* mark_debug = "true" *) logic aw_sent;

  // ?????? AR-done flag: prevents consuming stale R beats from a prior transaction ?????????
  // Cleared whenever ST_RD_A is entered, set when ARREADY=1 in ST_RD_A.
  // RREADY and HREADY in ST_RD_D are gated on this flag so that leftover R beats
  // sitting in the CDC FIFO from a previously-abandoned read are not mistaken for
  // the response to the current AR.
  (* mark_debug = "true" *) logic ar_done;
  (* mark_debug = "true" *) logic rd_d_entry;  // high for exactly one cycle on entry to ST_RD_D; suppresses RREADY

  // wr_resp_entry removed: the ST_WR_D last-beat fast-path already handles
  // the case where BVALID is asserted on the same cycle as WLAST (zero-
  // latency slave).  Once ST_WR_RESP is entered, BVALID was NOT present on
  // the WLAST cycle, so we must sample it unconditionally on every cycle
  // inside ST_WR_RESP — including the very first one.

  // ?????? INCR write accumulation buffer ?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  logic [DW-1:0]         ibuf_data [0:MAX_INCR_BEATS-1];
  logic [DW/8-1:0]       ibuf_strb [0:MAX_INCR_BEATS-1];
  (* mark_debug = "true" *) logic [IBUF_CNT_W-1:0] acc_cnt;
  (* mark_debug = "true" *) logic [IBUF_IDX_W-1:0] flush_ptr;
  logic [AW-1:0]               acc_cnt_aw;
  logic [7:0]                  acc_cnt_u8;

  // ?????? INCR-specific tracking registers ???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  (* mark_debug = "true" *) logic incr_rd;          // Active INCR read burst (chunked)
  (* mark_debug = "true" *) logic incr_wr_cont;     // INCR write continues accumulation after flush+resp
  logic incr_drain;       // Drain is for early INCR termination (not error)
  logic incr_rd_busy;     // Stall R channel during mid-chunk BUSY on AHB side

  // ?????? Next-transaction latch ???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  // INCR write states (ACC, RESUME) assert HREADY=1.  When the AHB master
  // presents a new NONSEQ in that cycle (or the following IDLE data phase),
  // the address is captured here so it survives until the bridge reaches
  // ST_IDLE after the write completes.
  (* mark_debug = "true" *) logic        pnd_valid;   // a NONSEQ was latched while INCR write was busy
  logic [AW-1:0] pnd_addr;
  logic [2:0]  pnd_size;
  logic [2:0]  pnd_burst;
  logic        pnd_write;
  logic [3:0]  pnd_prot;
  logic        pnd_lock;
  logic              pnd_wfirst_valid;
  logic [DW-1:0]     pnd_wfirst_data;
  logic [DW/8-1:0]   pnd_wfirst_strb;
  logic              pnd_wpipe_mode;
  logic              pnd_align_pulse_q;
  logic              pnd_wfirst_sysphase_q;

  // wstrb derivation
  logic [DW/8-1:0] ahb_wstrb_d;
  logic [AW-1:0]   ahb_waddr_d;
  logic [2:0]      ahb_wsize_d;
  (* mark_debug = "true" *) logic            ahb_wphase_valid;  

  // race condition fix
  logic [DW-1:0]   wdata_first;
  logic [DW/8-1:0] wstrb_first;
  logic            wr_d_wvalid;
  logic            wr_d_last_issue;
  logic            wr_d_fire;
  (* mark_debug = "true" *) logic            fix_ahb_fire;
  logic            fix_flush_wvalid;
  logic            fix_flush_last_issue;

  // loops fix
  (* mark_debug = "true" *) logic             hsel_q;
  (* mark_debug = "true" *) logic             hreadyin_q;
  (* mark_debug = "true" *) logic [AW-1:0]    haddr_q;
  logic [2:0]       hburst_q;
  logic             hmastlock_q;
  logic [3:0]       hprot_q;
  (* mark_debug = "true" *) logic [2:0]       hsize_q;
  (* mark_debug = "true" *) logic [1:0]       htrans_q;
  (* mark_debug = "true" *) logic             hwrite_q;

  // Functional helpers for read-side handoff decisions.
  // These are intentionally separate from dbg_* so the fix does not depend on
  // debug instrumentation being kept.
  logic             rd_live_nonseq;
  logic             rd_q_nonseq;
  logic             rd_live_interleave;
  logic             rd_q_interleave;
  logic             pnd_same_live_write_addrphase;

  // AXI->AHB read-return hold register.
  // Do not expose raw RVALID/RDATA directly to HREADY/HRDATA.
  (* mark_debug = "true" *) logic [DW-1:0] rd_buf_data;
  logic [1:0]    rd_buf_resp;
  (* mark_debug = "true" *) logic          rd_buf_valid;
  (* mark_debug = "true" *) logic          raw_r_accept;
  (* mark_debug = "true" *) logic          raw_r_capture;

  (* mark_debug = "true" *) logic          rd_fence_seen_idle;
  

  logic [DW/8-1:0] busy_wstrb;   // strobe saved on entry to POST_BUSY

  // ?????? HBURST ??? {AxLEN, AxBURST} ????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  // For fixed-length bursts and SINGLE.  INCR reads/writes override axlen.
  function automatic void hburst_decode (
    input  logic [2:0] hburst,
    output logic [7:0] axlen,
    output logic [1:0] axburst
  );
    unique case (hburst)
      HB_SINGLE: begin axlen = 8'd0;  axburst = AXI_INCR; end
      HB_INCR:   begin axlen = 8'd0;  axburst = AXI_INCR; end  // placeholder; overridden
      HB_WRAP4:  begin axlen = 8'd3;  axburst = AXI_WRAP; end
      HB_INCR4:  begin axlen = 8'd3;  axburst = AXI_INCR; end
      HB_WRAP8:  begin axlen = 8'd7;  axburst = AXI_WRAP; end
      HB_INCR8:  begin axlen = 8'd7;  axburst = AXI_INCR; end
      HB_WRAP16: begin axlen = 8'd15; axburst = AXI_WRAP; end
      HB_INCR16: begin axlen = 8'd15; axburst = AXI_INCR; end
      default:   begin axlen = 8'd0;  axburst = AXI_INCR; end
    endcase
  endfunction

  logic [7:0] c_axlen;
  logic [1:0] c_axburst;
  always_comb hburst_decode(HBURST, c_axlen, c_axburst);

  // Combinational decode for the pending-latch burst type
  logic [7:0] pnd_c_axlen;
  logic [1:0] pnd_c_axburst;
  always_comb hburst_decode(pnd_burst, pnd_c_axlen, pnd_c_axburst);

  // ?????? INCR chunk-length calculation (respects 4 KB AXI boundary) ??????????????????????????????????????????
  // Returns the AxLEN value (number of beats minus one).
  // Caps at MAX_INCR_BEATS-1 and ensures the burst does not cross a 4 KB page.
  function automatic [7:0] calc_incr_len (
    input [AW-1:0] addr,
    input [2:0]    size
  );
    reg [12:0] bytes_to_boundary;
    reg [12:0] beats_to_boundary;
    begin
      bytes_to_boundary = 13'h1000 - {1'b0, addr[11:0]};
      beats_to_boundary = bytes_to_boundary >> size;
      if (beats_to_boundary >= MAX_INCR_BEATS_13)
        calc_incr_len = MAX_INCR_BEATS - 1;
      else if (beats_to_boundary <= 13'd1)
        calc_incr_len = 8'd0;
      else
        calc_incr_len = beats_to_boundary - 13'd1;
    end
  endfunction

  // ?????? HPROT ??? AXPROT ??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  function automatic [2:0] hprot_to_axprot (input [3:0] hp);
    begin
      // AHB HPROT[0] = data(1)/opcode(0)
      // AHB HPROT[1] = privileged(1)/user(0)
      // AXI AXPROT[2] = instruction(1)/data(0)
      // AXI AXPROT[1] = non-secure
      // AXI AXPROT[0] = privileged
      hprot_to_axprot = {~hp[0], 1'b0, hp[1]};
    end
  endfunction


  function automatic [DW/8-1:0] ahb_addrsize_to_wstrb (
    input [AW-1:0] addr,
    input [2:0]    size
  );
    reg [DW/8-1:0] mask;
    integer nbytes;
    integer lane0;
    integer b;
    begin
      mask   = {DW/8{1'b0}};
      nbytes = 1 << size;
      lane0  = addr[$clog2(DW/8)-1:0];

      for (b = 0; b < DW/8; b = b + 1) begin
        if ((b >= lane0) && (b < lane0 + nbytes))
          mask[b] = 1'b1;
      end

      ahb_addrsize_to_wstrb = mask;
    end
  endfunction  

  // ?????? Helper: capture new transaction from address phase ??????????????????????????????????????????????????????????????????
  // (Used by multiple states for lookahead / back-to-back capture.)
  // Returns next state, updates registers via procedural assignments.

  assign acc_cnt_aw = {{(AW-IBUF_CNT_W){1'b0}}, acc_cnt};
  assign acc_cnt_u8 = {{(8-IBUF_CNT_W){1'b0}}, acc_cnt};

  // ?????? Sequential state machine ??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  always_ff @(posedge clk) begin
    if (!resetn) begin
      state        <= ST_IDLE;
      aw_sent      <= 1'b0;
      ar_done       <= 1'b0;
      beat_cnt      <= 8'd0;
      acc_cnt      <= '0;
      flush_ptr    <= '0;
      ap_addr      <= '0;
      ap_size      <= 3'd0;
      ap_axburst   <= AXI_INCR;
      ap_axlen     <= 8'd0;
      ap_prot      <= 4'd0;
      ap_lock      <= 1'b0;
      incr_rd      <= 1'b0;
      incr_wr_cont <= 1'b0;
      incr_drain   <= 1'b0;
      incr_rd_busy <= 1'b0;
      pnd_valid    <= 1'b0;
      pnd_wfirst_valid <= 1'b0;
      pnd_wfirst_data  <= '0;
      pnd_wfirst_strb  <= '0;
      pnd_wpipe_mode   <= 1'b0;
      pnd_align_pulse_q <= 1'b0;
      pnd_wfirst_sysphase_q <= 1'b0;
      // loop fix
      hsel_q      <= 1'b0;
      hreadyin_q  <= 1'b0;
      haddr_q     <= '0;
      hburst_q    <= 3'd0;
      hmastlock_q <= 1'b0;
      hprot_q     <= 4'd0;
      hsize_q     <= 3'd0;
      htrans_q    <= TRN_IDLE;
      hwrite_q    <= 1'b0;
      rd_buf_data   <= '0;
      rd_buf_resp   <= 2'b00;
      rd_buf_valid  <= 1'b0;
      rd_fence_seen_idle <= 1'b0;
      busy_wstrb <= '0;
      rd_d_entry <= 1'b0;
    end else begin
// loop fix: only advance when master considers address phase accepted
      if (pnd_wfirst_sysphase_q)
        pnd_wfirst_sysphase_q <= 1'b0;

      hreadyin_q  <= HREADYIN;
      if (HREADY && HREADYIN) begin
        hsel_q      <= HSEL;
        haddr_q     <= HADDR;
        hburst_q    <= HBURST;
        hmastlock_q <= HMASTLOCK;
        hprot_q     <= HPROT;
        hsize_q     <= HSIZE;
        htrans_q    <= HTRANS;
        hwrite_q    <= HWRITE;
      end      
      // Capture one AXI R beat into the local holding register ONLY when it
      // belongs to the currently active read transaction in ST_RD_D.
      // Any AXI R beats accepted in other states are intentionally scrubbed.
      if (!rd_buf_valid && raw_r_capture) begin
        rd_buf_data  <= RDATA;
        rd_buf_resp  <= RRESP;
        rd_buf_valid <= 1'b1;
      end

      case (state)

        // ?????? ST_IDLE ??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        // HREADY=1 (comb). A NONSEQ is accepted and captured this cycle.
        // If pnd_valid is set, consume the transaction latched by an INCR write
        // state (which asserted HREADY=1 while the master presented a new NONSEQ
        // that the bridge couldn't service immediately).
        ST_IDLE: begin
          incr_rd      <= 1'b0;
          incr_wr_cont <= 1'b0;
          incr_drain   <= 1'b0;
          incr_rd_busy <= 1'b0;
          pnd_valid    <= 1'b0;  // consume any pending latch

          // Priority: latched pending transaction > live AHB bus
          if (pnd_valid || (HSEL && HREADYIN && (HTRANS == TRN_NONSEQ))) begin
            ap_addr    <= pnd_valid ? pnd_addr  : HADDR;
            ap_size    <= pnd_valid ? pnd_size  : HSIZE;
            ap_prot    <= pnd_valid ? pnd_prot  : HPROT;
            ap_lock    <= pnd_valid ? pnd_lock  : HMASTLOCK;

            if (pnd_valid) begin
              ap_axlen   <= pnd_c_axlen;
              ap_axburst <= pnd_c_axburst;

                if (pnd_write) begin
                if (pnd_burst == HB_INCR) begin
                  acc_cnt      <= '0;
                  flush_ptr    <= '0;
                  incr_wr_cont <= 1'b0;
                  state        <= ST_WR_INCR_ACC;
                end else begin
                  acc_cnt      <= '0;
                  flush_ptr    <= '0;
                  beat_cnt     <= pnd_c_axlen;
                  aw_sent      <= 1'b0;

                  // v6:
                  // If the pending fixed write already captured beat 0 while
                  // the previous AXI write was still draining, start ST_WR_D
                  // directly from that fully latched context and ignore the
                  // live AHB bus.  Otherwise, fall back to the old align path.
                  if (pnd_wfirst_valid) begin
                    pnd_wpipe_mode   <= 1'b0;
                    pnd_align_pulse_q <= 1'b0;
                    wstrb_first      <= pnd_wfirst_strb;
                    state            <= ST_WR_D;
                  end else if (pnd_wfirst_capture_ok) begin
                    // A pending fixed write can reach ST_IDLE with beat 0
                    // already proven on the delayed AHB write-phase tracker.
                    // Capture it here instead of entering ST_WR_PND_ALIGN and
                    // waiting for a live bus phase that may no longer be
                    // visible to this slave.
                    pnd_wfirst_valid <= 1'b1;
                    pnd_wfirst_data  <= HWDATA;
                    pnd_wfirst_strb  <= pnd_wfirst_capture_strb;
                    pnd_wpipe_mode   <= 1'b0;
                    pnd_align_pulse_q <= 1'b0;
                    wstrb_first      <= pnd_wfirst_capture_strb;
                    state            <= ST_WR_D;
                  end else begin
                    pnd_wfirst_valid <= 1'b0;
                    pnd_wpipe_mode   <= 1'b0;
                    pnd_align_pulse_q <= HSEL && HREADYIN &&
                                         (HADDR == pnd_addr) &&
                                         (HSIZE == pnd_size) &&
                                         (HTRANS == TRN_NONSEQ) &&
                                         HWRITE;
                    wstrb_first      <= ahb_addrsize_to_wstrb(pnd_addr, pnd_size);
                    state            <= ST_WR_PND_ALIGN;
                  end
                end
              end else begin
                if (pnd_burst == HB_INCR) begin
                  incr_rd      <= 1'b1;
                  ap_axlen     <= calc_incr_len(pnd_addr, pnd_size);
                  beat_cnt     <= calc_incr_len(pnd_addr, pnd_size);
                  incr_rd_busy <= 1'b0;
                end else begin
                  incr_rd  <= 1'b0;
                  beat_cnt <= pnd_c_axlen;
                end
                ar_done <= 1'b0;
                state   <= ST_RD_A;
              end
            end else begin
              ap_axlen   <= c_axlen;
              ap_axburst <= c_axburst;

              if (HWRITE) begin
                if (HBURST == HB_INCR) begin
                  acc_cnt      <= '0;
                  flush_ptr    <= '0;
                  incr_wr_cont <= 1'b0;
                  state        <= ST_WR_INCR_ACC;
                end else begin
                  // Live fixed write: preload beat-0 strobe from the current
                  // accepted AHB address phase.
                  acc_cnt            <= '0;
                  flush_ptr          <= '0;
                  beat_cnt          <= c_axlen;
                  aw_sent           <= 1'b0;
                  wstrb_first       <= ahb_addrsize_to_wstrb(HADDR, HSIZE);
                  state             <= ST_WR_D;
                end
              end else begin
                if (HBURST == HB_INCR) begin
                  incr_rd      <= 1'b1;
                  ap_axlen     <= calc_incr_len(HADDR, HSIZE);
                  beat_cnt     <= calc_incr_len(HADDR, HSIZE);
                  incr_rd_busy <= 1'b0;
                end else begin
                  incr_rd  <= 1'b0;
                  beat_cnt <= c_axlen;
                end
                ar_done <= 1'b0;
                state   <= ST_RD_A;
              end
            end
          end
        end

        // One-cycle alignment stall before starting a pending fixed write.
        // HREADY=0 in this state keeps the first real data beat stable on HWDATA.
        // Capture that beat here, then start ST_WR_D on the next cycle.
        ST_WR_PND_ALIGN: begin
          // Two startup cases exist for a pending fixed write:
          // 1) The master is still holding the pending NONSEQ address phase.
          //    Give it one HREADY pulse so beat 0 reaches HWDATA next cycle.
        // 2) The master has already advanced beyond that NONSEQ (bug8-style).
        //    Beat 0 is already on HWDATA, so capture it immediately.
          if (pnd_align_pulse_q) begin
            pnd_align_pulse_q <= 1'b0;
            pnd_wpipe_mode <= 1'b1;
            state          <= ST_WR_PND_ALIGN;
          end else if (pnd_wfirst_capture_ok) begin
            pnd_wfirst_valid <= 1'b1;
            pnd_wfirst_data  <= HWDATA;
            pnd_wfirst_strb  <= pnd_wfirst_capture_strb;
            pnd_wpipe_mode   <= 1'b0;
            state            <= ST_WR_D;
          end else if (HSEL && HREADYIN &&
                       (HADDR == pnd_addr) &&
                       (HSIZE == pnd_size) &&
                       (HTRANS == TRN_NONSEQ) &&
                       HWRITE) begin
            // If the master is still holding the pending write NONSEQ while we
            // are stalled here, re-issue the one-cycle HREADY pulse and wait
            // for the proven beat-0 data phase on the next cycle.
            pnd_align_pulse_q <= 1'b1;
            pnd_wpipe_mode    <= 1'b0;
            state             <= ST_WR_PND_ALIGN;
          end else begin
            // Do not sample HWDATA until either the held write address phase
            // is visible or the delayed write-phase tracker proves beat 0.
            state <= ST_WR_PND_ALIGN;
          end
        end

        // ?????? ST_WR_D ??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        // Fixed-length write. Accept all AHB beats into the local buffer at
        // full AHB speed, then drain them later on the AXI W channel.
        ST_WR_D: begin
          if (!aw_sent && AWREADY) begin
            aw_sent <= 1'b1;
          end

          if (fix_ahb_fire) begin
            ibuf_data[acc_cnt[IBUF_IDX_W-1:0]] <= pnd_wfirst_valid ? pnd_wfirst_data : HWDATA;
            ibuf_strb[acc_cnt[IBUF_IDX_W-1:0]] <= pnd_wfirst_valid ? pnd_wfirst_strb : ahb_wstrb_d;
            acc_cnt <= acc_cnt + 1'b1;

            if (pnd_wfirst_valid) begin
              if (pnd_wpipe_mode) begin
                if (beat_cnt != 8'd0) begin
                  pnd_wfirst_data  <= HWDATA;
                  pnd_wfirst_strb  <= ahb_wstrb_d;
                  pnd_wfirst_valid <= 1'b1;
                end else begin
                  pnd_wfirst_valid <= 1'b0;
                  pnd_wpipe_mode   <= 1'b0;
                end
              end else begin
                pnd_wfirst_valid <= 1'b0;
              end
            end

            // Capture the next transaction only on an actual accepted near-final beat.
            if ((beat_cnt <= 8'd1) &&
                !pnd_valid &&
                HSEL && HREADYIN &&
                (HTRANS == TRN_NONSEQ)) begin
              pnd_valid      <= 1'b1;
              pnd_addr       <= HADDR;
              pnd_size       <= HSIZE;
              pnd_burst      <= HBURST;
              pnd_write      <= HWRITE;
              pnd_prot       <= HPROT;
              pnd_lock       <= HMASTLOCK;
              pnd_wfirst_valid <= 1'b0;
              pnd_wpipe_mode <= 1'b0;
              pnd_wfirst_sysphase_q <= HWRITE && (HBURST != HB_INCR) && !HREADY;
            end

            if (beat_cnt != 8'd0) begin
              beat_cnt <= beat_cnt - 8'd1;
            end else begin
              flush_ptr <= '0;
              state     <= ST_WR_FIX_FLUSH;
            end
          end
        end

        // HREADY stays high until a pending next transaction is captured.
        // This gives the AHB master one accept window for the follow-on NONSEQ
        // while the buffered first burst is still draining to AXI.
        ST_WR_FIX_FLUSH: begin
          if (!aw_sent && AWREADY)
            aw_sent <= 1'b1;

          if (!pnd_valid &&
              HSEL && HREADYIN &&
              (HTRANS == TRN_NONSEQ)) begin
            pnd_valid      <= 1'b1;
            pnd_addr       <= HADDR;
            pnd_size       <= HSIZE;
            pnd_burst      <= HBURST;
            pnd_write      <= HWRITE;
            pnd_prot       <= HPROT;
            pnd_lock       <= HMASTLOCK;
            pnd_wfirst_valid <= 1'b0;
            pnd_wpipe_mode <= 1'b0;
            pnd_wfirst_sysphase_q <= HWRITE && (HBURST != HB_INCR) && !HREADY;
          end

          // v12b:
          // Capture beat 0 for a pending fixed write only when the delayed
          // AHB write-phase view proves that the pending write's data phase is
          // currently on HWDATA. HREADYIN alone is too weak here, but removing
          // this capture entirely breaks the valid latched-pending-SINGLE path.
          if (pnd_valid &&
              pnd_write &&
              (pnd_burst != HB_INCR) &&
              !pnd_wfirst_valid &&
              pnd_wfirst_capture_ok) begin
            pnd_wfirst_valid <= 1'b1;
            pnd_wfirst_data  <= HWDATA;
            pnd_wfirst_strb  <= pnd_wfirst_capture_strb;
            pnd_wpipe_mode   <= 1'b0;
          end

          if (aw_sent && WREADY) begin
            if (!fix_flush_last_issue) begin
              flush_ptr <= flush_ptr + 1'b1;
            end else begin
              if (BVALID && !BRESP[1])
                state <= ST_IDLE;
              else if (BVALID && BRESP[1])
                state <= ST_WR_ERR;
              else
                state <= ST_WR_LAST_RESP;
            end
          end
        end

        // ?????? ST_WR_INCR_ACC ?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        // INCR write accumulation. HREADY=1 always (comb).
        //
        // AHB data-phase timing:
        //   T0 (IDLE):   NONSEQ, A0         ??? addr captured, no HWDATA yet
        //   T1 (ACC):    SEQ,    HWDATA=D0  ??? capture ibuf[0]
        //   T2 (ACC):    BUSY,   HWDATA=D1  ??? D1 is valid (data phase for A1)
        //                                     capture ibuf[1], ??? POST_BUSY
        //   ...
        //   Tn (ACC):    IDLE/NONSEQ ??? capture D_last + ??? FLUSH
        //
        // When acc_cnt reaches MAX_INCR_BEATS while HTRANS=SEQ, flush the
        // current buffer and resume accumulating (chunked write).
        ST_WR_INCR_ACC: begin
          if (HTRANS == TRN_BUSY) begin
            // Data phase for previous SEQ address completes; capture it.
            ibuf_data[acc_cnt[IBUF_IDX_W-1:0]] <= HWDATA;
            ibuf_strb[acc_cnt[IBUF_IDX_W-1:0]] <= ahb_wstrb_d;
            acc_cnt <= acc_cnt + 1'b1;
            // Save the next strobe now while HREADY=1 so POST_BUSY can use it.
            busy_wstrb <= ahb_addrsize_to_wstrb(HADDR, HSIZE);
            // Buffer-full check after this capture
            if (acc_cnt == MAX_INCR_BEATS_M1) begin
              flush_ptr    <= '0;
              aw_sent      <= 1'b0;
              incr_wr_cont <= 1'b1;
              state        <= ST_WR_INCR_FLUSH;
            end else begin
              state <= ST_WR_INCR_POST_BUSY;
            end
          end else if (HTRANS == TRN_SEQ) begin
            ibuf_data[acc_cnt[IBUF_IDX_W-1:0]] <= HWDATA;
            ibuf_strb[acc_cnt[IBUF_IDX_W-1:0]] <= ahb_wstrb_d;
            acc_cnt <= acc_cnt + 1'b1;
            // Buffer-full check
            if (acc_cnt == MAX_INCR_BEATS_M1) begin
              flush_ptr    <= '0;
              aw_sent      <= 1'b0;
              incr_wr_cont <= 1'b1;
              state        <= ST_WR_INCR_FLUSH;
            end
            // else: stay in ACC; more beats coming
          end else begin
            // IDLE or NONSEQ: last data phase; exit to flush.
            // If NONSEQ, latch the next transaction address now - HREADY=1
            // this cycle so the master's address phase is valid here but will
            // be gone by the time ST_IDLE is reached after the flush.
            ibuf_data[acc_cnt[IBUF_IDX_W-1:0]] <= HWDATA;
            ibuf_strb[acc_cnt[IBUF_IDX_W-1:0]] <= ahb_wstrb_d;
            acc_cnt      <= acc_cnt + 1'b1;
            flush_ptr    <= '0;
            aw_sent      <= 1'b0;
            incr_wr_cont <= 1'b0;
            if (HSEL && HREADYIN && (HTRANS == TRN_NONSEQ)) begin
              pnd_valid <= 1'b1;
              pnd_addr  <= HADDR;
              pnd_size  <= HSIZE;
              pnd_burst <= HBURST;
              pnd_write <= HWRITE;
              pnd_prot  <= HPROT;
              pnd_lock  <= HMASTLOCK;
              pnd_wfirst_valid <= 1'b0;
              pnd_wfirst_sysphase_q <= HWRITE && (HBURST != HB_INCR) && !HREADY;
            end else begin
              pnd_valid <= 1'b0;
            end
            state        <= ST_WR_INCR_FLUSH;
          end
        end

        // ?????? ST_WR_INCR_POST_BUSY ???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        // One-cycle stall (HREADY=0, comb) after a BUSY cycle.
        // busy_wstrb holds the strobe for the *next* beat (saved when HREADY was 1).
        ST_WR_INCR_POST_BUSY: begin
          if (HTRANS == TRN_BUSY) begin
            // Back-to-back BUSY: capture D_n+1 with the saved strobe.
            ibuf_data[acc_cnt[IBUF_IDX_W-1:0]] <= HWDATA;
            ibuf_strb[acc_cnt[IBUF_IDX_W-1:0]] <= busy_wstrb;
            acc_cnt <= acc_cnt + 1'b1;
            // Refresh busy_wstrb for a potential further BUSY beat.
            // HADDR is stable during BUSY (master holds it per AHB spec).
            busy_wstrb <= ahb_addrsize_to_wstrb(HADDR, HSIZE);
            if (acc_cnt == MAX_INCR_BEATS_M1) begin
              flush_ptr    <= '0;
              aw_sent      <= 1'b0;
              incr_wr_cont <= 1'b1;
              state        <= ST_WR_INCR_FLUSH;
            end
            // else remain in POST_BUSY
          end else if (HTRANS == TRN_SEQ) begin
            // Release the stall, skip capture for one cycle.
            state <= ST_WR_INCR_RESUME;
          end else begin
            // IDLE or NONSEQ: burst ended; D_last already captured during BUSY.
            flush_ptr    <= '0;
            aw_sent      <= 1'b0;
            incr_wr_cont <= 1'b0;
            state        <= ST_WR_INCR_FLUSH;
          end
        end

        // ?????? ST_WR_INCR_RESUME ?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        // First free-running cycle after POST_BUSY. HREADY=1, but HWDATA
        // settling, so never capture here.
        ST_WR_INCR_RESUME: begin
          if (HTRANS == TRN_BUSY) begin
            state <= ST_WR_INCR_POST_BUSY;
          end else if (HTRANS == TRN_SEQ) begin
            state <= ST_WR_INCR_ACC;
          end else begin
            // IDLE or NONSEQ: burst ended while HREADY=1. Latch if NONSEQ.
            flush_ptr    <= '0;
            aw_sent      <= 1'b0;
            incr_wr_cont <= 1'b0;
            if (HSEL && HREADYIN && (HTRANS == TRN_NONSEQ)) begin
              pnd_valid <= 1'b1;
              pnd_addr  <= HADDR;
              pnd_size  <= HSIZE;
              pnd_burst <= HBURST;
              pnd_write <= HWRITE;
              pnd_prot  <= HPROT;
              pnd_lock  <= HMASTLOCK;
              pnd_wfirst_valid <= 1'b0;
              pnd_wfirst_sysphase_q <= HWRITE && (HBURST != HB_INCR) && !HREADY;
            end else begin
              pnd_valid <= 1'b0;
            end
            state        <= ST_WR_INCR_FLUSH;
          end
        end
        // HREADY=0 (comb). Issue AW (AWLEN=acc_cnt-1) and drain ibuf to W.
        ST_WR_INCR_FLUSH: begin
          if (!aw_sent && AWREADY)
            aw_sent <= 1'b1;
          if (aw_sent && WREADY) begin
            if (flush_ptr != acc_cnt[IBUF_IDX_W-1:0] - 1'b1)
              flush_ptr <= flush_ptr + 1'b1;
            else begin
              // Same BVALID fast-path as ST_WR_D: avoid multi-delta collapse.
              if (BVALID && !BRESP[1] && !incr_wr_cont)
                state <= ST_IDLE;
              else if (BVALID && BRESP[1])
                state <= ST_WR_ERR;
              else
                state <= ST_WR_RESP;
            end
          end
        end

        // ?????? ST_WR_RESP ?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        // HREADY depends on context (comb).
        //   incr_wr_cont=1 ??? HREADY=0 (stall master; will resume ACC).
        //   Otherwise: HREADY=BVALID&~BRESP[1] (normal completion handshake).
        //
        // IMPORTANT: do NOT fast-path directly to ST_RD_A / ST_WR_D here.
        // When BVALID=1, HREADY goes high combinatorially this same cycle.
        // The AHB master only presents a stable new address phase on the
        // *following* cycle (it responds to HREADY with one cycle of latency).
        // Sampling HADDR/HBURST/HWRITE here would capture stale or
        // transitioning values, producing wrong AWLEN / missing ARVALID.
        // Routing through ST_IDLE costs zero extra cycles: ST_IDLE asserts
        // HREADY=1 and captures NONSEQ in a single cycle.
        ST_WR_RESP: begin
          // Capture pending NONSEQ on EVERY cycle, not just the BVALID cycle.
          if (!pnd_valid &&
              HSEL && HREADYIN &&
              (HTRANS == TRN_NONSEQ)) begin
            pnd_valid <= 1'b1;
            pnd_addr  <= HADDR;
            pnd_size  <= HSIZE;
            pnd_burst <= HBURST;
            pnd_write <= HWRITE;
            pnd_prot  <= HPROT;
            pnd_lock  <= HMASTLOCK;
            pnd_wfirst_valid <= 1'b0;
            pnd_wfirst_sysphase_q <= HWRITE && (HBURST != HB_INCR) && !HREADY;
          end

          // v12b:
          // Capture beat 0 for a pending fixed write only when the delayed
          // AHB write-phase view proves that the pending write's data phase is
          // currently on HWDATA. Do not use HREADYIN alone here.
          if (pnd_valid &&
              pnd_write &&
              (pnd_burst != HB_INCR) &&
              !pnd_wfirst_valid &&
              pnd_wfirst_capture_ok) begin
            pnd_wfirst_valid <= 1'b1;
            pnd_wfirst_data  <= HWDATA;
            pnd_wfirst_strb  <= pnd_wfirst_capture_strb;
            pnd_wpipe_mode   <= 1'b0;
          end

          if (BVALID) begin
            if (BRESP[1]) begin
              incr_wr_cont <= 1'b0;
              state <= ST_WR_ERR;
            end else if (incr_wr_cont) begin
              ap_addr      <= ap_addr + (acc_cnt_aw << ap_size);
              acc_cnt      <= '0;
              flush_ptr    <= '0;
              incr_wr_cont <= 1'b0;
              state        <= ST_WR_INCR_ACC;
            end else begin
              state <= ST_IDLE;
            end
          end
        end

        // Fixed-write wait state: last AXI W beat was accepted but B has not
        // yet arrived. Hold AHB stalled until B handshakes. Capture any
        // NONSEQ that appears during this window (bug8-style bus-mux case).
        ST_WR_LAST_RESP: begin
          if (!pnd_valid &&
              HSEL && HREADYIN &&
              (HTRANS == TRN_NONSEQ)) begin
            pnd_valid      <= 1'b1;
            pnd_addr       <= HADDR;
            pnd_size       <= HSIZE;
            pnd_burst      <= HBURST;
            pnd_write      <= HWRITE;
            pnd_prot       <= HPROT;
            pnd_lock       <= HMASTLOCK;
            pnd_wfirst_valid <= 1'b0;
            pnd_wpipe_mode <= 1'b0;
            pnd_wfirst_sysphase_q <= HWRITE && (HBURST != HB_INCR) && !HREADY;
          end

          // v12b:
          // Capture beat 0 for a pending fixed write only when the delayed
          // AHB write-phase view proves that the pending write's data phase is
          // currently on HWDATA. Do not use HREADYIN alone here.
          if (pnd_valid &&
              pnd_write &&
              (pnd_burst != HB_INCR) &&
              !pnd_wfirst_valid &&
              pnd_wfirst_capture_ok) begin
            pnd_wfirst_valid <= 1'b1;
            pnd_wfirst_data  <= HWDATA;
            pnd_wfirst_strb  <= pnd_wfirst_capture_strb;
            pnd_wpipe_mode   <= 1'b0;
          end

          if (BVALID) begin
            if (BRESP[1])
              state <= ST_WR_ERR;
            else
              state <= ST_IDLE;
          end
        end

        // ?????? ST_WR_ERR ????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        // Second AHB error cycle: HRESP=1, HREADY=1 (comb). ??? IDLE.
        ST_WR_ERR: begin
          state <= ST_IDLE;
        end

        // ?????? ST_RD_A ??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        // ARVALID=1 (comb), HREADY=0 (comb) until AR handshake.
        ST_RD_A: begin
          incr_rd_busy <= 1'b0;
          // Invalidate any stale rd_buf content from a prior transaction.
          // rd_buf_valid=1 here means the fence did not drain it; clearing it
          // now prevents it from being delivered as beat 0 of this new read.
          rd_buf_valid <= 1'b0;
          if (ARREADY) begin
            ar_done    <= 1'b1;
            rd_d_entry <= 1'b1;   // suppress RREADY on first cycle of ST_RD_D
            state      <= ST_RD_D;
          end
        end

        // ?????? ST_RD_D ??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        // RREADY=1 (comb, unless INCR BUSY stall). HREADY=RVALID unless error.
        //
        // For INCR reads, additional mid-chunk handling:
        //   ??? HTRANS=BUSY ??? stall R channel, HREADY=1 (AHB spec for BUSY).
        //   ??? HTRANS=IDLE/NONSEQ mid-chunk ??? drain remaining R beats.
        //   ??? beat_cnt=0 + HTRANS=SEQ ??? issue new chunked AR (continuation).
        //   ??? beat_cnt=0 + HTRANS=BUSY ??? ST_RD_INCR_WAIT.
        ST_RD_D: begin
          rd_d_entry <= 1'b0;   // clear after one cycle
          if (rd_d_entry) rd_buf_valid <= 1'b0;  // discard any stale content

          // If a distinct pending fixed WRITE was already latched during a
          // read-data overlap, preserve its first data beat as soon as the
          // delayed AHB write-phase view proves beat-0 is now on HWDATA.
          //
          // This is earlier than ST_RD_FENCE and is required for BUG9:
          // the write data phase can occur while we are still in ST_RD_D
          // delivering the final read beat / scrubbing the final AXI R beat.
          if (pnd_valid && pnd_write && (pnd_burst != HB_INCR) &&
              !pnd_wfirst_valid && pnd_wfirst_capture_ok &&
              !pnd_same_live_write_addrphase) begin
            pnd_wfirst_valid <= 1'b1;
            pnd_wfirst_data  <= HWDATA;
            pnd_wfirst_strb  <= pnd_wfirst_capture_strb;
          end

          if (incr_rd && incr_rd_busy) begin
            if (HTRANS != TRN_BUSY) begin
              incr_rd_busy <= 1'b0;
              if (HTRANS == TRN_SEQ) begin
                // stay in ST_RD_D
              end else begin
                if (beat_cnt != 8'd0) begin
                  incr_drain <= 1'b1;
                  state      <= ST_RD_DRAIN;
                end else begin
                  incr_rd            <= 1'b0;
                  rd_fence_seen_idle <= 1'b0;
                  state              <= ST_RD_FENCE;
                end
              end
            end

          end else begin
            if (!pnd_valid &&
                HSEL && HREADYIN &&
                (HTRANS == TRN_NONSEQ) &&
                (HWRITE || (HADDR != ap_addr) || (HSIZE != ap_size))) begin
              // Latch a *different* pending NONSEQ as soon as it becomes visible,
              // even if we are still holding or capturing read data.
              //
              // IMPORTANT: this must NOT preempt rd_buf_valid processing.
              // On the final-beat boundary, the bridge can be simultaneously
              // delivering the current read beat and observing the next NONSEQ.
              // If we handle the pending latch in an outer else-if chain, we can
              // skip the current-beat retirement and return beat[N-1] twice while
              // losing beat[N].
              pnd_valid <= 1'b1;
              pnd_addr  <= HADDR;
              pnd_size  <= HSIZE;
              pnd_burst <= HBURST;
              pnd_write <= HWRITE;
              pnd_prot  <= HPROT;
              pnd_lock  <= HMASTLOCK;
              pnd_wfirst_valid <= 1'b0;
              pnd_wfirst_sysphase_q <= HWRITE && (HBURST != HB_INCR) && !HREADY;
            end

            // ?????? Normal R-beat delivery ????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
            if (rd_buf_valid) begin
            if (rd_buf_resp[1]) begin
              rd_buf_valid <= 1'b0;
              state <= ST_RD_ERR;

            end else if (incr_rd && (beat_cnt != 8'd0) && rd_q_interleave) begin
              // Keep the buffered beat until AHB can actually accept it.
              // Clearing rd_buf_valid here would drop a read beat while
              // HREADY=0 during a no-idle interleave stall.

            end else begin
              rd_buf_valid <= 1'b0;

            if (beat_cnt != 8'd0) begin
              // ?????? Mid-burst beat ????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
              beat_cnt <= beat_cnt - 8'd1;

              if (incr_rd) begin
                // Check AHB side for early termination or BUSY.
                if (htrans_q == TRN_BUSY) begin
                  // Use the registered AHB view for current-beat disposition so
                  // the sequential state update matches the combinational HREADY
                  // decision for this beat. A live NONSEQ for the *following*
                  // transfer will appear in *_q on the next cycle.
                  incr_rd_busy <= 1'b1;

                end else if (htrans_q == TRN_IDLE) begin
                  // Legal early termination of undefined-INCR read.
                  // Current beat IS delivered this cycle. Drain the rest.
                  if ((beat_cnt - 8'd1) != 8'd0) begin
                    incr_drain <= 1'b1;
                    state      <= ST_RD_DRAIN;
                  end else begin
                    incr_rd    <= 1'b0;
                    incr_drain <= 1'b0;
                    state      <= ST_RD_FENCE;
                  end

                end else if (rd_q_interleave) begin
                  // Write interleave or new NONSEQ for the NEXT transfer.
                  // Use the registered AHB view here as well; this keeps the
                  // read-data state transition consistent with HREADY for the
                  // current beat, and latches the pending NONSEQ one cycle later
                  // when it becomes the registered view.
                  if (rd_q_nonseq) begin
                    pnd_valid <= 1'b1;
                    pnd_addr  <= haddr_q;
                    pnd_size  <= hsize_q;
                    pnd_burst <= hburst_q;
                    pnd_write <= hwrite_q;
                    pnd_prot  <= hprot_q;
                    pnd_lock  <= hmastlock_q;
                    pnd_wfirst_valid <= 1'b0;
                  end

                  if ((beat_cnt - 8'd1) != 8'd0) begin
                    incr_drain <= 1'b1;
                    state      <= ST_RD_DRAIN;
                  end else begin
                    incr_rd    <= 1'b0;
                    incr_drain <= 1'b0;
                    state      <= ST_RD_FENCE;
                  end
                end
                // else q-view says SEQ: normal, continue delivering.
              end
              // For non-INCR reads: unconditional decrement (no mid-burst check).

            end else begin
              // ?????? Last beat (beat_cnt == 0) ??? lookahead ???????????????????????????????????????????????????????????????????????????
              //
              // IMPORTANT:
              //   The final read-data beat and the next AHB address phase can
              //   legally overlap in the SAME cycle.  For this boundary we must
              //   look at the live AHB bus, not the registered *_q view,
              //   otherwise a completed single-read error/recovery sequence can
              //   re-consume a stale prior NONSEQ from *_q.
              //
              // Mid-burst handling still uses the registered view so the
              // current-beat HREADY decision and the sequential transition stay
              // aligned.  Only this last-beat handoff uses the live lookahead.
              if (HSEL && HREADYIN && (HTRANS == TRN_NONSEQ)) begin
                // Do NOT launch the next transaction directly from the last-beat cycle.
                // First fence off any late stale AXI R beats from the just-completed read.
                pnd_valid <= 1'b1;
                pnd_addr  <= HADDR;
                pnd_size  <= HSIZE;
                pnd_burst <= HBURST;
                pnd_write <= HWRITE;
                pnd_prot  <= HPROT;
                pnd_lock  <= HMASTLOCK;
                incr_rd   <= 1'b0;
                ar_done   <= 1'b0;
                pnd_wfirst_sysphase_q <= HWRITE && (HBURST != HB_INCR) && !HREADY;
                state     <= ST_RD_FENCE;
              end else if (incr_rd && (HTRANS == TRN_SEQ)) begin
                // INCR read continuation: chunk consumed, master wants more.
                ap_addr      <= HADDR;
                ap_axlen     <= calc_incr_len(HADDR, ap_size);
                beat_cnt     <= calc_incr_len(HADDR, ap_size);
                incr_rd_busy <= 1'b0;
                ar_done      <= 1'b0;
                state        <= ST_RD_A;
              end else if (incr_rd && (HTRANS == TRN_BUSY)) begin
                // Master needs more time ??? wait for resolution.
                state <= ST_RD_INCR_WAIT;
              end else begin
                // IDLE (or non-INCR SEQ that shouldn't happen).
                incr_rd <= 1'b0;
                state   <= ST_RD_FENCE;
              end
            end
            end
            end
          end
        end

        // ?????? ST_RD_INCR_WAIT ??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        // Reached when the last R beat of a chunk was delivered but the master
        // presented HTRANS=BUSY.  HREADY=0 keeps the master stalled.
        ST_RD_INCR_WAIT: begin
          if (HTRANS != TRN_BUSY) begin
            if (HTRANS == TRN_SEQ) begin
              ap_addr      <= HADDR;
              ap_axlen     <= calc_incr_len(HADDR, ap_size);
              beat_cnt     <= calc_incr_len(HADDR, ap_size);
              incr_rd_busy <= 1'b0;
              ar_done      <= 1'b0;
              state        <= ST_RD_A;
            end else begin
              incr_rd <= 1'b0;
              state   <= ST_RD_FENCE;
            end
          end
        end

        // ?????? ST_RD_ERR ????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        ST_RD_ERR: begin
          incr_drain <= 1'b0;
          if (beat_cnt == 8'd0) begin
            incr_rd <= 1'b0;
            state   <= ST_RD_FENCE;
          end else begin
            state <= ST_RD_DRAIN;
          end
        end

        // ?????? ST_RD_DRAIN ??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        // Silently consume remaining AXI R beats.
        // Error drain: HREADY=1 (original behaviour, master already got error).
        // INCR early-termination drain: HREADY=0 (stall master until done).
        ST_RD_DRAIN: begin
          if (rd_buf_valid) begin
            rd_buf_valid <= 1'b0;
            if (beat_cnt != 8'd0)
              beat_cnt <= beat_cnt - 8'd1;
            else begin
              incr_rd    <= 1'b0;
              incr_drain <= 1'b0;
              state      <= ST_RD_FENCE;
            end
          end
        end

        ST_RD_FENCE: begin
          // Drain any late stale AXI R beats that appear after the logical end
          // of the previous read transaction, before allowing the next read.
          // One empty cycle is not enough; require two consecutive clean cycles
          // after the AXI read is fully retired.
          //
          // If a fixed-length pending WRITE was already latched during the last
          // read-beat overlap, preserve its first data beat here as soon as the
          // delayed AHB write-phase view proves that beat-0 is on HWDATA.
          //
          // This is exactly the read->write no-idle-gap handoff from BUG9:
          //   cycle N   : final read beat delivered, next write NONSEQ visible
          //   cycle N+1 : bridge fences stale R beats, write beat-0 is on HWDATA
          //
          // HREADY is low in ST_RD_FENCE, so the master holds HWDATA stable.
          ar_done <= 1'b0;

          if (pnd_valid && pnd_write && (pnd_burst != HB_INCR) &&
              !pnd_wfirst_valid && pnd_wfirst_capture_ok &&
              !pnd_same_live_write_addrphase) begin
            pnd_wfirst_valid <= 1'b1;
            pnd_wfirst_data  <= HWDATA;
            pnd_wfirst_strb  <= pnd_wfirst_capture_strb;
          end

          if (rd_buf_valid) begin
            rd_buf_valid        <= 1'b0;
            rd_fence_seen_idle  <= 1'b0;

          end else if (RVALID || dbg_rd_outstanding) begin
            rd_fence_seen_idle  <= 1'b0;

          end else if (!rd_fence_seen_idle) begin
            rd_fence_seen_idle  <= 1'b1;

          end else begin
            rd_fence_seen_idle  <= 1'b0;
            state               <= ST_IDLE;
          end
        end

        default: state <= ST_IDLE;

      endcase
    end
  end // always_ff

// derivation of wstrb
  // Gate on HREADY && HREADYIN: this is the AHB "address phase accepted" condition.
  // HREADY here is this bridge's own output; HREADYIN comes from the prior slave.
  // Both must be 1 for the master to consider the address phase complete.
  // In ST_WR_D, HREADY = aw_sent && WREADY. If HREADYIN=0 on that cycle the
  // master is NOT advancing its address pipeline, so we must not advance ours.
  always_ff @(posedge clk) begin
    if (!resetn) begin
      ahb_wstrb_d      <= '0;
      ahb_waddr_d      <= '0;
      ahb_wsize_d      <= 3'd0;
      ahb_wphase_valid <= 1'b0;
    end else if (HREADY && HREADYIN) begin
      ahb_wphase_valid <= HSEL && HTRANS[1] && HWRITE;
      if (HSEL && HTRANS[1] && HWRITE) begin
        ahb_waddr_d <= HADDR;
        ahb_wsize_d <= HSIZE;
        ahb_wstrb_d <= ahb_addrsize_to_wstrb(HADDR, HSIZE);
      end else begin
        // Address phase accepted but not a write: clear validity so a stale
        // strobe is never used if direction changes next beat.
        ahb_wphase_valid <= 1'b0;
      end
    end
  end

  // ?????? AHB combinational outputs ???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  always_comb begin
    HREADY = 1'b1;
    HRDATA = rd_buf_data;
    HRESP  = 1'b0;

    unique case (state)
      ST_IDLE: begin
        // If any pending transaction is already latched, keep AHB stalled until
        // the bridge has actually moved into the corresponding active state.
        //
        // This is especially required for pending READs captured after a write:
        // the latest ILA proves that asserting HREADY=1 here exposes stale
        // HRDATA before any AXI R beat has been accepted.
        if (pnd_valid)
          HREADY = 1'b0;
        else
          HREADY = 1'b1;
      end

      ST_WR_PND_ALIGN: begin
        HREADY = pnd_align_pulse_q;
      end

      ST_WR_D: begin
        HREADY = 1'b1;
      end

      ST_WR_FIX_FLUSH:     HREADY = !pnd_valid;

      ST_WR_INCR_ACC:       HREADY = 1'b1;
      ST_WR_INCR_POST_BUSY: HREADY = 1'b0;
      ST_WR_INCR_RESUME:    HREADY = 1'b1;
      ST_WR_INCR_FLUSH:     HREADY = 1'b0;

      ST_WR_RESP: begin
        if (incr_wr_cont) begin
          HREADY = 1'b0;
        end else if (pnd_valid) begin
          // Bug #8 fix part 2: stall for ANY pending transaction,
          // not just reads. When a pending write's NONSEQ was captured,
          // we must keep HREADY=0 so the master does not advance further.
          // The data phase for the NONSEQ has not yet completed; releasing
          // HREADY here would let the master slip one beat and misalign
          // the burst data.
          HREADY = 1'b0;
        end else begin
          HREADY = BVALID & ~BRESP[1];
          HRESP  = BVALID &  BRESP[1];
        end
      end

      ST_WR_LAST_RESP: begin
        if (pnd_valid) begin
          HREADY = 1'b0;
        end else begin
          HREADY = BVALID & ~BRESP[1];
          HRESP  = BVALID &  BRESP[1];
        end
      end

      ST_WR_ERR: begin
        HREADY = 1'b1;
        HRESP  = 1'b1;
      end

      ST_RD_A: HREADY = 1'b0;

    //   ST_RD_D: begin
    //     if (incr_rd && incr_rd_busy) begin
    //       // BUSY stall: AHB spec requires HREADY=1 for BUSY response.
    //       // RREADY=0 (comb below) holds off AXI R channel.
    //       HREADY = 1'b1;
    //     end else if (RVALID && incr_rd && (beat_cnt != 8'd0) &&
    //                  (HWRITE ||
    //                   (HSEL && HREADYIN && (HTRANS == TRN_NONSEQ)))) begin
    //       // Suppress current beat only for write interleave or a new NONSEQ.
    //       // Do NOT suppress it for IDLE termination of a legal undefined-INCR read.
    //       HREADY = 1'b0;
    //     end else if (RVALID & RRESP[1]) begin
    //       HREADY = 1'b0;
    //       HRESP  = 1'b1;
    //     end else begin
    //       HREADY = ar_done & RVALID;
    //       HRDATA = RDATA;
    //     end
      ST_RD_D: begin
        if (incr_rd && incr_rd_busy) begin
          HREADY = 1'b1;
        end else if (rd_buf_valid && incr_rd && (beat_cnt != 8'd0) &&
                     rd_q_interleave) begin
          HREADY = 1'b0;
        end else if (rd_buf_valid && rd_buf_resp[1]) begin
          HREADY = 1'b0;
          HRESP  = 1'b1;
        end else begin
          HREADY = rd_buf_valid;
          HRDATA = rd_buf_data;
        end
      end

      ST_RD_ERR: begin
        HREADY = 1'b1;
        HRESP  = 1'b1;
      end

      ST_RD_INCR_WAIT: HREADY = 1'b1;
      ST_RD_FENCE:     HREADY = 1'b0;

      ST_RD_DRAIN: begin
        HREADY = incr_drain ? 1'b0 : 1'b1;
      end

      default: HREADY = 1'b1;
    endcase
  end

  // ?????? AXI4: Write Address Channel ??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  assign AWID    = '0;
  assign AWADDR  = ap_addr;
  assign AWLEN   = (state == ST_WR_INCR_FLUSH) ? (acc_cnt_u8 - 8'd1) : ap_axlen;
  assign AWSIZE  = ap_size;
  assign AWBURST = (state == ST_WR_INCR_FLUSH) ? AXI_INCR : ap_axburst;
  assign AWLOCK  = ap_lock;
  assign AWCACHE = {2'b00, ap_prot[3], ap_prot[2]};  // HPROT[3]=cacheable→[1], HPROT[2]=bufferable→[0]
  assign AWPROT  = hprot_to_axprot(ap_prot);
  assign AWQOS   = 4'b0000;
  assign AWVALID = ((state == ST_WR_D) || (state == ST_WR_FIX_FLUSH) || (state == ST_WR_INCR_FLUSH)) && !aw_sent;

  // In ST_WR_D, consume a fixed-write beat only when it is already latched
  // (pnd_wfirst_valid) or when the previous accepted AHB address phase proves
  // that the current HWDATA is a real write data phase (ahb_wphase_valid).
  // Do NOT key this off raw HREADYIN alone: a legal BUSY / non-write cycle can
  // leave HREADYIN=1 while no fixed write beat is actually due.
  assign fix_ahb_fire          = (state == ST_WR_D) && (pnd_wfirst_valid || ahb_wphase_valid);
  assign fix_flush_wvalid      = (state == ST_WR_FIX_FLUSH) && aw_sent;
  assign fix_flush_last_issue  = fix_flush_wvalid && (flush_ptr == acc_cnt[IBUF_IDX_W-1:0] - 1'b1);
  assign wr_d_wvalid           = 1'b0;
  assign wr_d_last_issue       = 1'b0;
  assign wr_d_fire             = 1'b0;

  // ?????? AXI4: Write Data Channel ???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  assign WVALID  = fix_flush_wvalid || ((state == ST_WR_INCR_FLUSH) && aw_sent);

  assign WDATA = ((state == ST_WR_FIX_FLUSH) || (state == ST_WR_INCR_FLUSH))
               ? ibuf_data[flush_ptr]
               : HWDATA;

  assign WSTRB = ((state == ST_WR_FIX_FLUSH) || (state == ST_WR_INCR_FLUSH))
               ? ibuf_strb[flush_ptr]
               : ahb_wstrb_d;
  assign WLAST  = fix_flush_last_issue
               || ((state == ST_WR_INCR_FLUSH) && (flush_ptr == acc_cnt[IBUF_IDX_W-1:0] - 1'b1));

  // ?????? AXI4: Write Response Channel ???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  assign BREADY  = (state == ST_WR_RESP)
                |  (state == ST_WR_LAST_RESP)
                |  fix_flush_last_issue
                |  (state == ST_WR_INCR_FLUSH && aw_sent
                    && flush_ptr == acc_cnt[IBUF_IDX_W-1:0] - 1'b1);
  // ?????? AXI4: Read Address Channel ?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  assign ARID    = '0;
  assign ARADDR  = ap_addr;
  assign ARLEN   = ap_axlen;
  assign ARSIZE  = ap_size;
  assign ARBURST = ap_axburst;
  assign ARLOCK  = ap_lock;
  assign ARCACHE = {2'b00, ap_prot[3], ap_prot[2]};  // HPROT[3]=cacheable→[1], HPROT[2]=bufferable→[0]
  assign ARPROT  = hprot_to_axprot(ap_prot);
  assign ARQOS   = 4'b0000;
  assign ARVALID = (state == ST_RD_A);

  // ?????? AXI4: Read Data Channel ??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  //
  // Accept AXI R beats in four cases:
  //   1) ST_RD_D: current read transaction -> capture into rd_buf_*
  //   2) ST_RD_DRAIN / ST_RD_FENCE: drain/scrub old beats
  //   3) ST_RD_A *before* AR handshake: scrub stray late beats while waiting
  //      for the new read to become active
  //   4) non-read states (IDLE / write states / RD_ERR): scrub stray late beats
  //
  // The remaining hole seen on ILA was ST_RD_A: stale R beats could survive
  // through write/idle, arrive while waiting for ARREADY, and then become the
  // first beats accepted in ST_RD_D.
  //
  // In this system, stale late R beats can still appear during the ST_RD_A
  // handshake cycle itself. Scrub throughout all of ST_RD_A.
  //
  // The AXI-side ILA shows the legitimate first beat for this read does NOT
  // arrive on the AR handshake cycle, so this does not drop the real beat here.
  assign RREADY =
      (
        // Current active read: accept and CAPTURE.
        ((state == ST_RD_D) && ar_done && !rd_d_entry && !(incr_rd && incr_rd_busy))

        // Drain / fence states: accept and DROP.
        || (state == ST_RD_DRAIN)
        || (state == ST_RD_FENCE)

        // Read-address state: accept and DROP any stray late beats.
        || (state == ST_RD_A)

        // Non-read states: accept and DROP any stray late R beats.
        || (state == ST_IDLE)
        || (state == ST_WR_PND_ALIGN)
        || (state == ST_WR_D)
        || (state == ST_WR_FIX_FLUSH)
        || (state == ST_WR_INCR_ACC)
        || (state == ST_WR_INCR_POST_BUSY)
        || (state == ST_WR_INCR_RESUME)
        || (state == ST_WR_INCR_FLUSH)
        || (state == ST_WR_RESP)
        || (state == ST_WR_LAST_RESP)
        || (state == ST_WR_ERR)
        || (state == ST_RD_ERR)
      )
      && !rd_buf_valid;

  assign raw_r_accept  = RVALID && RREADY;

  // Only beats accepted during the active read-data state are allowed to enter
  // the AHB-visible read buffer. All other accepted beats are scrubbed.
  assign raw_r_capture = RVALID
                      && !rd_buf_valid
                      && (state == ST_RD_D)
                      && ar_done
                      && !rd_d_entry
                      && !(incr_rd && incr_rd_busy);



  // --------------------------------------------------------------------------
  // Debug-only instrumentation
  //
  // Goal:
  //   Catch the FIRST internal bridge bookkeeping violation, and also preserve
  //   the most recent AR/R/AW/W/B/AHB events in sticky registers so a post-hang
  //   quiet-state ILA still contains useful information.
  //
  // Notes:
  //   - No functional behavior is changed.
  //   - Trigger on dbg_trip == 1 if you want the first hard invariant break.
  //   - If dbg_trip never fires, inspect dbg_last_* and dbg_hist* after hang.
  // --------------------------------------------------------------------------

  localparam logic [7:0] DBG_EVT_NONE     = 8'd0;
  localparam logic [7:0] DBG_EVT_AHB_AP   = 8'd1;
  localparam logic [7:0] DBG_EVT_AR_FIRE  = 8'd2;
  localparam logic [7:0] DBG_EVT_R_ACCEPT = 8'd3;
  localparam logic [7:0] DBG_EVT_R_CAPTURE= 8'd4;
  localparam logic [7:0] DBG_EVT_AW_FIRE  = 8'd5;
  localparam logic [7:0] DBG_EVT_W_FIRE   = 8'd6;
  localparam logic [7:0] DBG_EVT_B_FIRE   = 8'd7;
  localparam logic [7:0] DBG_EVT_STATE    = 8'd8;

  localparam logic [7:0] DBG_TRIP_AR_DOUBLE       = 8'd1;
  localparam logic [7:0] DBG_TRIP_R_NO_RD         = 8'd2;
  localparam logic [7:0] DBG_TRIP_R_IN_RD_A       = 8'd3;
  localparam logic [7:0] DBG_TRIP_R_OUTSIDE_RD    = 8'd4;
  localparam logic [7:0] DBG_TRIP_RLAST_MISMATCH  = 8'd5;
  localparam logic [7:0] DBG_TRIP_AW_DOUBLE       = 8'd6;
  localparam logic [7:0] DBG_TRIP_B_NO_WR         = 8'd7;
  localparam logic [7:0] DBG_TRIP_FIX_WRITE_UNPROVEN = 8'd8;
  localparam logic [7:0] DBG_TRIP_RD_POP_NO_HREADY   = 8'd9;
  localparam logic [7:0] DBG_TRIP_RD_LIVE_Q_DISAGREE = 8'd10;
  localparam logic [7:0] DBG_TRIP_PND_ALIGN_UNPROVEN = 8'd11;
  localparam logic [7:0] DBG_TRIP_PND_FLUSH_UNPROVEN = 8'd12;
  localparam logic [7:0] DBG_TRIP_RD_REPEAT_DELIVER = 8'd13;
  localparam logic [7:0] DBG_TRIP_RD_LASTBEAT_NO_DELIVER = 8'd14;

  (* mark_debug = "true" *) logic dbg_ahb_ap_fire;
  (* mark_debug = "true" *) logic dbg_ar_fire;
  (* mark_debug = "true" *) logic dbg_aw_fire;
  (* mark_debug = "true" *) logic dbg_w_fire;
  (* mark_debug = "true" *) logic dbg_b_fire;
  (* mark_debug = "true" *) logic dbg_live_interleave;
  (* mark_debug = "true" *) logic dbg_q_interleave;

  assign dbg_ahb_ap_fire = HREADY && HREADYIN && HSEL && HTRANS[1];
  assign dbg_ar_fire     = ARVALID && ARREADY;
  assign dbg_aw_fire     = AWVALID && AWREADY;
  assign dbg_w_fire      = WVALID && WREADY;
  assign dbg_b_fire      = BVALID && BREADY;

  assign rd_live_nonseq      = HSEL && HREADYIN && (HTRANS == TRN_NONSEQ);
  assign rd_q_nonseq         = hsel_q && hreadyin_q && (htrans_q == TRN_NONSEQ);
  assign rd_live_interleave  = HWRITE || (rd_live_nonseq &&
                                          ((HADDR != ap_addr) || (HSIZE != ap_size)));
  assign rd_q_interleave     = hwrite_q || (rd_q_nonseq &&
                                            ((haddr_q != ap_addr) || (hsize_q != ap_size)));

  assign pnd_same_live_write_addrphase =
      HSEL && HREADYIN && HWRITE && HTRANS[1] &&
      (HADDR == pnd_addr) && (HSIZE == pnd_size);

  assign dbg_live_interleave = rd_live_interleave;
  assign dbg_q_interleave    = rd_q_interleave;

  logic pnd_wfirst_capture_ok;
  logic [DW/8-1:0] pnd_wfirst_capture_strb;
  logic dbg_rd_deliver_fire;
  logic [15:0] dbg_rd_capture_seq;
  logic [15:0] dbg_rd_buf_capture_seq;
  logic [15:0] dbg_rd_last_deliver_seq;
  logic        dbg_prev_rd_deliver_fire;
  logic        dbg_prev_rd_buf_valid;
  logic [1:0]  dbg_prev_rd_buf_resp;
  logic [7:0]  dbg_prev_beat_cnt;
  state_t      dbg_prev_cycle_state;

  assign pnd_wfirst_capture_ok = pnd_wfirst_sysphase_q
                              || (ahb_wphase_valid
                                  && (ahb_waddr_d == pnd_addr)
                                  && (ahb_wsize_d == pnd_size));

  assign pnd_wfirst_capture_strb = pnd_wfirst_sysphase_q
                                ? ahb_addrsize_to_wstrb(pnd_addr, pnd_size)
                                : ahb_wstrb_d;

  assign dbg_rd_deliver_fire = (state == ST_RD_D)
                            && rd_buf_valid
                            && HREADY
                            && !rd_buf_resp[1];

  (* mark_debug = "true" *) logic        dbg_trip;
  (* mark_debug = "true" *) logic        dbg_trip_sticky;
  (* mark_debug = "true" *) logic [7:0]  dbg_trip_code_live;
  (* mark_debug = "true" *) logic [7:0]  dbg_trip_cause;
  (* mark_debug = "true" *) logic        dbg_rd_outstanding;
  (* mark_debug = "true" *) logic        dbg_wr_outstanding;
  (* mark_debug = "true" *) logic [31:0] dbg_cycle_counter;
  (* mark_debug = "true" *) logic [31:0] dbg_trip_cycle;
  (* mark_debug = "true" *) state_t      dbg_trip_state;
  (* mark_debug = "true" *) logic [AW-1:0] dbg_trip_ap_addr;
  (* mark_debug = "true" *) logic [AW-1:0] dbg_trip_haddr;
  (* mark_debug = "true" *) logic [7:0]  dbg_trip_beat_cnt;
  (* mark_debug = "true" *) logic [IBUF_CNT_W-1:0] dbg_trip_acc_cnt;
  (* mark_debug = "true" *) logic [IBUF_IDX_W-1:0] dbg_trip_flush_ptr;
  (* mark_debug = "true" *) logic [1:0]  dbg_trip_htrans;
  (* mark_debug = "true" *) logic        dbg_trip_hwrite;
  (* mark_debug = "true" *) logic [1:0]  dbg_trip_rresp;
  (* mark_debug = "true" *) logic        dbg_trip_rlast;

  (* mark_debug = "true" *) logic        dbg_fix_write_unproven;
  (* mark_debug = "true" *) logic [31:0] dbg_fix_write_cycle;
  (* mark_debug = "true" *) state_t      dbg_fix_write_state;
  (* mark_debug = "true" *) logic [AW-1:0] dbg_fix_write_haddr;
  (* mark_debug = "true" *) logic [1:0]  dbg_fix_write_htrans;
  (* mark_debug = "true" *) logic        dbg_fix_write_hwrite;
  (* mark_debug = "true" *) logic        dbg_fix_write_hreadyin;
  (* mark_debug = "true" *) logic        dbg_fix_write_ahb_wphase_valid;
  (* mark_debug = "true" *) logic        dbg_fix_write_pnd_wfirst_valid;
  (* mark_debug = "true" *) logic [DW-1:0] dbg_fix_write_hwd_data;
  (* mark_debug = "true" *) logic [DW/8-1:0] dbg_fix_write_hwd_strb;
  (* mark_debug = "true" *) logic [IBUF_CNT_W-1:0] dbg_fix_write_acc_cnt;
  (* mark_debug = "true" *) logic [IBUF_IDX_W-1:0] dbg_fix_write_flush_ptr;

  (* mark_debug = "true" *) logic        dbg_rd_pop_while_not_ready;
  (* mark_debug = "true" *) logic [31:0] dbg_rd_pop_cycle;
  (* mark_debug = "true" *) state_t      dbg_rd_pop_state;
  (* mark_debug = "true" *) logic [AW-1:0] dbg_rd_pop_haddr;
  (* mark_debug = "true" *) logic [7:0]  dbg_rd_pop_beat_cnt;
  (* mark_debug = "true" *) logic        dbg_rd_pop_hready;
  (* mark_debug = "true" *) logic [1:0]  dbg_rd_pop_resp;

  (* mark_debug = "true" *) logic        dbg_pnd_align_unproven;
  (* mark_debug = "true" *) logic        dbg_pnd_flush_unproven;
  (* mark_debug = "true" *) logic        dbg_rd_repeat_deliver;
  (* mark_debug = "true" *) logic        dbg_rd_lastbeat_no_deliver;
  (* mark_debug = "true" *) logic [31:0] dbg_pnd_align_cycle;
  (* mark_debug = "true" *) logic [31:0] dbg_pnd_flush_cycle;
  (* mark_debug = "true" *) logic [31:0] dbg_rd_repeat_cycle;
  (* mark_debug = "true" *) logic [31:0] dbg_rd_lastbeat_cycle;
  (* mark_debug = "true" *) logic [15:0] dbg_rd_buf_capture_seq_dbg;
  (* mark_debug = "true" *) logic [15:0] dbg_rd_last_deliver_seq_dbg;

  (* mark_debug = "true" *) logic        dbg_rd_live_q_disagree;
  (* mark_debug = "true" *) logic [31:0] dbg_rd_live_q_cycle;
  (* mark_debug = "true" *) state_t      dbg_rd_live_q_state;
  (* mark_debug = "true" *) logic [AW-1:0] dbg_rd_live_haddr;
  (* mark_debug = "true" *) logic [AW-1:0] dbg_rd_q_haddr;
  (* mark_debug = "true" *) logic [1:0]  dbg_rd_live_htrans;
  (* mark_debug = "true" *) logic [1:0]  dbg_rd_q_htrans;
  (* mark_debug = "true" *) logic        dbg_rd_live_hwrite;
  (* mark_debug = "true" *) logic        dbg_rd_q_hwrite;
  (* mark_debug = "true" *) logic [2:0]  dbg_rd_live_hsize;
  (* mark_debug = "true" *) logic [2:0]  dbg_rd_q_hsize;
  (* mark_debug = "true" *) logic [AW-1:0] dbg_rd_live_ap_addr;
  (* mark_debug = "true" *) logic [2:0]  dbg_rd_live_ap_size;

  (* mark_debug = "true" *) state_t      dbg_prev_state;

  (* mark_debug = "true" *) logic [AW-1:0] dbg_last_ahb_addr;
  (* mark_debug = "true" *) logic [2:0]  dbg_last_ahb_size;
  (* mark_debug = "true" *) logic [2:0]  dbg_last_ahb_burst;
  (* mark_debug = "true" *) logic [1:0]  dbg_last_ahb_trans;
  (* mark_debug = "true" *) logic        dbg_last_ahb_write;
  (* mark_debug = "true" *) logic [3:0]  dbg_last_ahb_prot;
  (* mark_debug = "true" *) logic [DW-1:0] dbg_last_hwd_data;
  (* mark_debug = "true" *) logic [DW/8-1:0] dbg_last_hwd_strb;

  (* mark_debug = "true" *) logic [AW-1:0] dbg_last_ar_addr;
  (* mark_debug = "true" *) logic [7:0]  dbg_last_ar_len;
  (* mark_debug = "true" *) logic [2:0]  dbg_last_ar_size;
  (* mark_debug = "true" *) logic [1:0]  dbg_last_ar_burst;

  (* mark_debug = "true" *) logic [DW-1:0] dbg_last_r_data;
  (* mark_debug = "true" *) logic [1:0]  dbg_last_r_resp;
  (* mark_debug = "true" *) logic        dbg_last_r_last;
  (* mark_debug = "true" *) logic        dbg_last_r_was_capture;
  (* mark_debug = "true" *) state_t      dbg_last_r_state;
  (* mark_debug = "true" *) logic [7:0]  dbg_last_r_beat_cnt;

  (* mark_debug = "true" *) logic [AW-1:0] dbg_last_aw_addr;
  (* mark_debug = "true" *) logic [7:0]  dbg_last_aw_len;
  (* mark_debug = "true" *) logic [2:0]  dbg_last_aw_size;
  (* mark_debug = "true" *) logic [1:0]  dbg_last_aw_burst;

  (* mark_debug = "true" *) logic [DW-1:0] dbg_last_w_data;
  (* mark_debug = "true" *) logic [DW/8-1:0] dbg_last_w_strb;
  (* mark_debug = "true" *) logic        dbg_last_w_last;
  (* mark_debug = "true" *) logic [IBUF_IDX_W-1:0] dbg_last_w_flush_ptr;

  (* mark_debug = "true" *) logic [1:0]  dbg_last_b_resp;

  (* mark_debug = "true" *) logic [7:0]  dbg_hist0_code, dbg_hist1_code, dbg_hist2_code, dbg_hist3_code;
  (* mark_debug = "true" *) state_t      dbg_hist0_state, dbg_hist1_state, dbg_hist2_state, dbg_hist3_state;
  (* mark_debug = "true" *) logic [AW-1:0] dbg_hist0_ap_addr, dbg_hist1_ap_addr, dbg_hist2_ap_addr, dbg_hist3_ap_addr;
  (* mark_debug = "true" *) logic [AW-1:0] dbg_hist0_haddr,   dbg_hist1_haddr,   dbg_hist2_haddr,   dbg_hist3_haddr;
  (* mark_debug = "true" *) logic [7:0]  dbg_hist0_beat_cnt, dbg_hist1_beat_cnt, dbg_hist2_beat_cnt, dbg_hist3_beat_cnt;

  logic dbg_trip_ar_double_p;
  logic dbg_trip_r_no_rd_p;
  logic dbg_trip_r_in_rd_a_p;
  logic dbg_trip_r_outside_rd_p;
  logic dbg_trip_rlast_mismatch_p;
  logic dbg_trip_fix_write_unproven_p;
  logic dbg_trip_rd_pop_no_hready_p;
  logic dbg_trip_rd_live_q_disagree_p;
  logic dbg_trip_rd_repeat_deliver_p;
  logic dbg_trip_rd_lastbeat_no_deliver_p;
  logic dbg_trip_aw_double_p;
  logic dbg_trip_b_no_wr_p;

  assign dbg_trip_ar_double_p =
      dbg_ar_fire && dbg_rd_outstanding;

  assign dbg_trip_r_no_rd_p =
      raw_r_accept && !dbg_rd_outstanding;

  assign dbg_trip_r_in_rd_a_p =
      raw_r_accept && (state == ST_RD_A);

  assign dbg_trip_r_outside_rd_p =
      raw_r_accept &&
      (state != ST_RD_D) &&
      (state != ST_RD_DRAIN) &&
      (state != ST_RD_FENCE) &&
      (state != ST_RD_A);

  assign dbg_trip_rlast_mismatch_p =
      raw_r_capture && (RLAST != (beat_cnt == 8'd0));

  assign dbg_trip_fix_write_unproven_p =
      (state == ST_WR_D) &&
      fix_ahb_fire &&
      !pnd_wfirst_valid &&
      !ahb_wphase_valid;

  assign dbg_trip_rd_pop_no_hready_p =
      (state == ST_RD_D) &&
      rd_buf_valid &&
      !HREADY &&
      !rd_buf_resp[1];

  assign dbg_trip_rd_live_q_disagree_p =
      (state == ST_RD_D) &&
      rd_buf_valid &&
      (beat_cnt != 8'd0) &&
      (dbg_live_interleave != dbg_q_interleave) &&
      !(HSEL && HREADYIN && (HTRANS == TRN_NONSEQ) &&
        (HWRITE || (HADDR != ap_addr) || (HSIZE != ap_size)));

  assign dbg_trip_rd_repeat_deliver_p =
      dbg_rd_deliver_fire &&
      (dbg_rd_buf_capture_seq != 16'd0) &&
      (dbg_rd_buf_capture_seq == dbg_rd_last_deliver_seq);

  assign dbg_trip_rd_lastbeat_no_deliver_p =
      (dbg_prev_cycle_state == ST_RD_D) &&
      dbg_prev_rd_buf_valid &&
      (dbg_prev_beat_cnt == 8'd0) &&
      !dbg_prev_rd_buf_resp[1] &&
      (state != ST_RD_D) &&
      !dbg_prev_rd_deliver_fire;

  assign dbg_trip_aw_double_p =
      dbg_aw_fire && dbg_wr_outstanding;

  assign dbg_trip_b_no_wr_p =
      dbg_b_fire && !dbg_wr_outstanding;

  assign dbg_trip =
      dbg_trip_ar_double_p ||
      dbg_trip_r_no_rd_p ||
      dbg_trip_r_in_rd_a_p ||
      dbg_trip_r_outside_rd_p ||
      dbg_trip_rlast_mismatch_p ||
      dbg_trip_fix_write_unproven_p ||
      dbg_trip_rd_pop_no_hready_p ||
      dbg_trip_rd_live_q_disagree_p ||
      dbg_trip_rd_repeat_deliver_p ||
      dbg_trip_rd_lastbeat_no_deliver_p ||
      dbg_trip_aw_double_p ||
      dbg_trip_b_no_wr_p;

  assign dbg_trip_code_live =
      dbg_trip_ar_double_p              ? DBG_TRIP_AR_DOUBLE :
      dbg_trip_r_no_rd_p                ? DBG_TRIP_R_NO_RD :
      dbg_trip_r_in_rd_a_p              ? DBG_TRIP_R_IN_RD_A :
      dbg_trip_r_outside_rd_p           ? DBG_TRIP_R_OUTSIDE_RD :
      dbg_trip_rlast_mismatch_p         ? DBG_TRIP_RLAST_MISMATCH :
      dbg_trip_fix_write_unproven_p     ? DBG_TRIP_FIX_WRITE_UNPROVEN :
      dbg_trip_rd_pop_no_hready_p       ? DBG_TRIP_RD_POP_NO_HREADY :
      dbg_trip_rd_live_q_disagree_p     ? DBG_TRIP_RD_LIVE_Q_DISAGREE :
      dbg_trip_rd_repeat_deliver_p      ? DBG_TRIP_RD_REPEAT_DELIVER :
      dbg_trip_rd_lastbeat_no_deliver_p ? DBG_TRIP_RD_LASTBEAT_NO_DELIVER :
      dbg_trip_aw_double_p              ? DBG_TRIP_AW_DOUBLE :
      dbg_trip_b_no_wr_p                ? DBG_TRIP_B_NO_WR :
                                          8'd0;

  logic [7:0] dbg_evt_code;

  always_comb begin
    dbg_evt_code = DBG_EVT_NONE;
    if (dbg_ahb_ap_fire)         dbg_evt_code = DBG_EVT_AHB_AP;
    if (dbg_ar_fire)             dbg_evt_code = DBG_EVT_AR_FIRE;
    if (raw_r_accept)            dbg_evt_code = DBG_EVT_R_ACCEPT;
    if (raw_r_capture)           dbg_evt_code = DBG_EVT_R_CAPTURE;
    if (dbg_aw_fire)             dbg_evt_code = DBG_EVT_AW_FIRE;
    if (dbg_w_fire)              dbg_evt_code = DBG_EVT_W_FIRE;
    if (dbg_b_fire)              dbg_evt_code = DBG_EVT_B_FIRE;
    if (state != dbg_prev_state) dbg_evt_code = DBG_EVT_STATE;
  end

  always_ff @(posedge clk) begin
    if (!resetn) begin
      dbg_trip_sticky     <= 1'b0;
      dbg_trip_cause      <= 8'd0;
      dbg_rd_outstanding  <= 1'b0;
      dbg_wr_outstanding  <= 1'b0;
      dbg_cycle_counter   <= 32'd0;
      dbg_trip_cycle      <= 32'd0;
      dbg_trip_state      <= ST_IDLE;
      dbg_trip_ap_addr    <= '0;
      dbg_trip_haddr      <= '0;
      dbg_trip_beat_cnt   <= 8'd0;
      dbg_trip_acc_cnt    <= '0;
      dbg_trip_flush_ptr  <= '0;
      dbg_trip_htrans     <= 2'b00;
      dbg_trip_hwrite     <= 1'b0;
      dbg_trip_rresp      <= 2'b00;
      dbg_trip_rlast      <= 1'b0;

      dbg_fix_write_unproven     <= 1'b0;
      dbg_fix_write_cycle        <= 32'd0;
      dbg_fix_write_state        <= ST_IDLE;
      dbg_fix_write_haddr        <= '0;
      dbg_fix_write_htrans       <= 2'b00;
      dbg_fix_write_hwrite       <= 1'b0;
      dbg_fix_write_hreadyin     <= 1'b0;
      dbg_fix_write_ahb_wphase_valid <= 1'b0;
      dbg_fix_write_pnd_wfirst_valid <= 1'b0;
      dbg_fix_write_hwd_data     <= '0;
      dbg_fix_write_hwd_strb     <= '0;
      dbg_fix_write_acc_cnt      <= '0;
      dbg_fix_write_flush_ptr    <= '0;

      dbg_pnd_align_unproven     <= 1'b0;
      dbg_pnd_flush_unproven     <= 1'b0;
      dbg_rd_repeat_deliver      <= 1'b0;
      dbg_rd_lastbeat_no_deliver <= 1'b0;
      dbg_pnd_align_cycle        <= 32'd0;
      dbg_pnd_flush_cycle        <= 32'd0;
      dbg_rd_repeat_cycle        <= 32'd0;
      dbg_rd_lastbeat_cycle      <= 32'd0;
      dbg_rd_capture_seq         <= 16'd0;
      dbg_rd_buf_capture_seq     <= 16'd0;
      dbg_rd_last_deliver_seq    <= 16'd0;
      dbg_rd_buf_capture_seq_dbg <= 16'd0;
      dbg_rd_last_deliver_seq_dbg<= 16'd0;
      dbg_prev_rd_deliver_fire   <= 1'b0;
      dbg_prev_rd_buf_valid      <= 1'b0;
      dbg_prev_rd_buf_resp       <= 2'b00;
      dbg_prev_beat_cnt          <= 8'd0;
      dbg_prev_cycle_state       <= ST_IDLE;

      dbg_rd_pop_while_not_ready <= 1'b0;
      dbg_rd_pop_cycle           <= 32'd0;
      dbg_rd_pop_state           <= ST_IDLE;
      dbg_rd_pop_haddr           <= '0;
      dbg_rd_pop_beat_cnt        <= 8'd0;
      dbg_rd_pop_hready          <= 1'b0;
      dbg_rd_pop_resp            <= 2'b00;

      dbg_rd_live_q_disagree     <= 1'b0;
      dbg_rd_live_q_cycle        <= 32'd0;
      dbg_rd_live_q_state        <= ST_IDLE;
      dbg_rd_live_haddr          <= '0;
      dbg_rd_q_haddr             <= '0;
      dbg_rd_live_htrans         <= 2'b00;
      dbg_rd_q_htrans            <= 2'b00;
      dbg_rd_live_hwrite         <= 1'b0;
      dbg_rd_q_hwrite            <= 1'b0;
      dbg_rd_live_hsize          <= 3'd0;
      dbg_rd_q_hsize             <= 3'd0;
      dbg_rd_live_ap_addr        <= '0;
      dbg_rd_live_ap_size        <= 3'd0;

      dbg_prev_state      <= ST_IDLE;

      dbg_last_ahb_addr   <= '0;
      dbg_last_ahb_size   <= 3'd0;
      dbg_last_ahb_burst  <= 3'd0;
      dbg_last_ahb_trans  <= 2'b00;
      dbg_last_ahb_write  <= 1'b0;
      dbg_last_ahb_prot   <= 4'd0;
      dbg_last_hwd_data   <= '0;
      dbg_last_hwd_strb   <= '0;

      dbg_last_ar_addr    <= '0;
      dbg_last_ar_len     <= 8'd0;
      dbg_last_ar_size    <= 3'd0;
      dbg_last_ar_burst   <= 2'b00;

      dbg_last_r_data     <= '0;
      dbg_last_r_resp     <= 2'b00;
      dbg_last_r_last     <= 1'b0;
      dbg_last_r_was_capture <= 1'b0;
      dbg_last_r_state    <= ST_IDLE;
      dbg_last_r_beat_cnt <= 8'd0;

      dbg_last_aw_addr    <= '0;
      dbg_last_aw_len     <= 8'd0;
      dbg_last_aw_size    <= 3'd0;
      dbg_last_aw_burst   <= 2'b00;

      dbg_last_w_data     <= '0;
      dbg_last_w_strb     <= '0;
      dbg_last_w_last     <= 1'b0;
      dbg_last_w_flush_ptr<= '0;

      dbg_last_b_resp     <= 2'b00;

      dbg_hist0_code      <= DBG_EVT_NONE;
      dbg_hist1_code      <= DBG_EVT_NONE;
      dbg_hist2_code      <= DBG_EVT_NONE;
      dbg_hist3_code      <= DBG_EVT_NONE;
      dbg_hist0_state     <= ST_IDLE;
      dbg_hist1_state     <= ST_IDLE;
      dbg_hist2_state     <= ST_IDLE;
      dbg_hist3_state     <= ST_IDLE;
      dbg_hist0_ap_addr   <= '0;
      dbg_hist1_ap_addr   <= '0;
      dbg_hist2_ap_addr   <= '0;
      dbg_hist3_ap_addr   <= '0;
      dbg_hist0_haddr     <= '0;
      dbg_hist1_haddr     <= '0;
      dbg_hist2_haddr     <= '0;
      dbg_hist3_haddr     <= '0;
      dbg_hist0_beat_cnt  <= 8'd0;
      dbg_hist1_beat_cnt  <= 8'd0;
      dbg_hist2_beat_cnt  <= 8'd0;
      dbg_hist3_beat_cnt  <= 8'd0;
    end else begin
      dbg_cycle_counter <= dbg_cycle_counter + 32'd1;

      if (dbg_ahb_ap_fire) begin
        dbg_last_ahb_addr  <= HADDR;
        dbg_last_ahb_size  <= HSIZE;
        dbg_last_ahb_burst <= HBURST;
        dbg_last_ahb_trans <= HTRANS;
        dbg_last_ahb_write <= HWRITE;
        dbg_last_ahb_prot  <= HPROT;
        dbg_last_hwd_data  <= HWDATA;
        dbg_last_hwd_strb  <= ahb_addrsize_to_wstrb(HADDR, HSIZE);
      end

      if (dbg_ar_fire) begin
        dbg_last_ar_addr   <= ARADDR;
        dbg_last_ar_len    <= ARLEN;
        dbg_last_ar_size   <= ARSIZE;
        dbg_last_ar_burst  <= ARBURST;

        if (dbg_rd_outstanding && !dbg_trip_sticky) begin
          dbg_trip_sticky    <= 1'b1;
          dbg_trip_cause     <= DBG_TRIP_AR_DOUBLE;
          dbg_trip_cycle     <= dbg_cycle_counter;
          dbg_trip_state     <= state;
          dbg_trip_ap_addr   <= ap_addr;
          dbg_trip_haddr     <= HADDR;
          dbg_trip_beat_cnt  <= beat_cnt;
          dbg_trip_acc_cnt   <= acc_cnt;
          dbg_trip_flush_ptr <= flush_ptr;
          dbg_trip_htrans    <= HTRANS;
          dbg_trip_hwrite    <= HWRITE;
          dbg_trip_rresp     <= 2'b00;
          dbg_trip_rlast     <= 1'b0;
        end
        dbg_rd_outstanding <= 1'b1;
      end

      if (raw_r_accept) begin
        dbg_last_r_data        <= RDATA;
        dbg_last_r_resp        <= RRESP;
        dbg_last_r_last        <= RLAST;
        dbg_last_r_was_capture <= raw_r_capture;
        dbg_last_r_state       <= state;
        dbg_last_r_beat_cnt    <= beat_cnt;

        if (!dbg_rd_outstanding && !dbg_trip_sticky) begin
          dbg_trip_sticky    <= 1'b1;
          dbg_trip_cause     <= DBG_TRIP_R_NO_RD;
          dbg_trip_cycle     <= dbg_cycle_counter;
          dbg_trip_state     <= state;
          dbg_trip_ap_addr   <= ap_addr;
          dbg_trip_haddr     <= HADDR;
          dbg_trip_beat_cnt  <= beat_cnt;
          dbg_trip_acc_cnt   <= acc_cnt;
          dbg_trip_flush_ptr <= flush_ptr;
          dbg_trip_htrans    <= HTRANS;
          dbg_trip_hwrite    <= HWRITE;
          dbg_trip_rresp     <= RRESP;
          dbg_trip_rlast     <= RLAST;
        end else if ((state == ST_RD_A) && !dbg_trip_sticky) begin
          dbg_trip_sticky    <= 1'b1;
          dbg_trip_cause     <= DBG_TRIP_R_IN_RD_A;
          dbg_trip_cycle     <= dbg_cycle_counter;
          dbg_trip_state     <= state;
          dbg_trip_ap_addr   <= ap_addr;
          dbg_trip_haddr     <= HADDR;
          dbg_trip_beat_cnt  <= beat_cnt;
          dbg_trip_acc_cnt   <= acc_cnt;
          dbg_trip_flush_ptr <= flush_ptr;
          dbg_trip_htrans    <= HTRANS;
          dbg_trip_hwrite    <= HWRITE;
          dbg_trip_rresp     <= RRESP;
          dbg_trip_rlast     <= RLAST;
        end else if ((state != ST_RD_D) &&
                     (state != ST_RD_DRAIN) &&
                     (state != ST_RD_FENCE) &&
                     (state != ST_RD_A) &&
                     !dbg_trip_sticky) begin
          dbg_trip_sticky    <= 1'b1;
          dbg_trip_cause     <= DBG_TRIP_R_OUTSIDE_RD;
          dbg_trip_cycle     <= dbg_cycle_counter;
          dbg_trip_state     <= state;
          dbg_trip_ap_addr   <= ap_addr;
          dbg_trip_haddr     <= HADDR;
          dbg_trip_beat_cnt  <= beat_cnt;
          dbg_trip_acc_cnt   <= acc_cnt;
          dbg_trip_flush_ptr <= flush_ptr;
          dbg_trip_htrans    <= HTRANS;
          dbg_trip_hwrite    <= HWRITE;
          dbg_trip_rresp     <= RRESP;
          dbg_trip_rlast     <= RLAST;
        end else if (raw_r_capture && (RLAST != (beat_cnt == 8'd0)) && !dbg_trip_sticky) begin
          dbg_trip_sticky    <= 1'b1;
          dbg_trip_cause     <= DBG_TRIP_RLAST_MISMATCH;
          dbg_trip_cycle     <= dbg_cycle_counter;
          dbg_trip_state     <= state;
          dbg_trip_ap_addr   <= ap_addr;
          dbg_trip_haddr     <= HADDR;
          dbg_trip_beat_cnt  <= beat_cnt;
          dbg_trip_acc_cnt   <= acc_cnt;
          dbg_trip_flush_ptr <= flush_ptr;
          dbg_trip_htrans    <= HTRANS;
          dbg_trip_hwrite    <= HWRITE;
          dbg_trip_rresp     <= RRESP;
          dbg_trip_rlast     <= RLAST;
        end

        if (RLAST)
          dbg_rd_outstanding <= 1'b0;
      end

      if ((state == ST_WR_D) && fix_ahb_fire && !pnd_wfirst_valid && !ahb_wphase_valid) begin
        if (!dbg_fix_write_unproven) begin
          dbg_fix_write_unproven        <= 1'b1;
          dbg_fix_write_cycle           <= dbg_cycle_counter;
          dbg_fix_write_state           <= state;
          dbg_fix_write_haddr           <= HADDR;
          dbg_fix_write_htrans          <= HTRANS;
          dbg_fix_write_hwrite          <= HWRITE;
          dbg_fix_write_hreadyin        <= HREADYIN;
          dbg_fix_write_ahb_wphase_valid<= ahb_wphase_valid;
          dbg_fix_write_pnd_wfirst_valid<= pnd_wfirst_valid;
          dbg_fix_write_hwd_data        <= HWDATA;
          dbg_fix_write_hwd_strb        <= ahb_wstrb_d;
          dbg_fix_write_acc_cnt         <= acc_cnt;
          dbg_fix_write_flush_ptr       <= flush_ptr;
        end
        if (!dbg_trip_sticky) begin
          dbg_trip_sticky    <= 1'b1;
          dbg_trip_cause     <= DBG_TRIP_FIX_WRITE_UNPROVEN;
          dbg_trip_cycle     <= dbg_cycle_counter;
          dbg_trip_state     <= state;
          dbg_trip_ap_addr   <= ap_addr;
          dbg_trip_haddr     <= HADDR;
          dbg_trip_beat_cnt  <= beat_cnt;
          dbg_trip_acc_cnt   <= acc_cnt;
          dbg_trip_flush_ptr <= flush_ptr;
          dbg_trip_htrans    <= HTRANS;
          dbg_trip_hwrite    <= HWRITE;
          dbg_trip_rresp     <= 2'b00;
          dbg_trip_rlast     <= 1'b0;
        end
      end

      if ((state == ST_WR_PND_ALIGN) && !pnd_align_pulse_q && !pnd_wfirst_capture_ok) begin
        if (!dbg_pnd_align_unproven) begin
          dbg_pnd_align_unproven <= 1'b1;
          dbg_pnd_align_cycle    <= dbg_cycle_counter;
        end
      end

      if ((state == ST_WR_FIX_FLUSH) &&
          pnd_valid && pnd_write && (pnd_burst != HB_INCR) &&
          !pnd_wfirst_valid && HREADYIN && !pnd_wfirst_capture_ok) begin
        if (!dbg_pnd_flush_unproven) begin
          dbg_pnd_flush_unproven <= 1'b1;
          dbg_pnd_flush_cycle    <= dbg_cycle_counter;
        end
      end

      if (dbg_rd_deliver_fire && (dbg_rd_buf_capture_seq != 16'd0) &&
          (dbg_rd_buf_capture_seq == dbg_rd_last_deliver_seq)) begin
        if (!dbg_rd_repeat_deliver) begin
          dbg_rd_repeat_deliver <= 1'b1;
          dbg_rd_repeat_cycle   <= dbg_cycle_counter;
        end
        if (!dbg_trip_sticky) begin
          dbg_trip_sticky    <= 1'b1;
          dbg_trip_cause     <= DBG_TRIP_RD_REPEAT_DELIVER;
          dbg_trip_cycle     <= dbg_cycle_counter;
          dbg_trip_state     <= state;
          dbg_trip_ap_addr   <= ap_addr;
          dbg_trip_haddr     <= HADDR;
          dbg_trip_beat_cnt  <= beat_cnt;
          dbg_trip_acc_cnt   <= acc_cnt;
          dbg_trip_flush_ptr <= flush_ptr;
          dbg_trip_htrans    <= HTRANS;
          dbg_trip_hwrite    <= HWRITE;
          dbg_trip_rresp     <= rd_buf_resp;
          dbg_trip_rlast     <= 1'b0;
        end
      end

      if ((dbg_prev_cycle_state == ST_RD_D) && dbg_prev_rd_buf_valid &&
          (dbg_prev_beat_cnt == 8'd0) && !dbg_prev_rd_buf_resp[1] &&
          (state != ST_RD_D) && !dbg_prev_rd_deliver_fire) begin
        if (!dbg_rd_lastbeat_no_deliver) begin
          dbg_rd_lastbeat_no_deliver <= 1'b1;
          dbg_rd_lastbeat_cycle      <= dbg_cycle_counter;
        end
        if (!dbg_trip_sticky) begin
          dbg_trip_sticky    <= 1'b1;
          dbg_trip_cause     <= DBG_TRIP_RD_LASTBEAT_NO_DELIVER;
          dbg_trip_cycle     <= dbg_cycle_counter;
          dbg_trip_state     <= dbg_prev_cycle_state;
          dbg_trip_ap_addr   <= ap_addr;
          dbg_trip_haddr     <= HADDR;
          dbg_trip_beat_cnt  <= dbg_prev_beat_cnt;
          dbg_trip_acc_cnt   <= acc_cnt;
          dbg_trip_flush_ptr <= flush_ptr;
          dbg_trip_htrans    <= HTRANS;
          dbg_trip_hwrite    <= HWRITE;
          dbg_trip_rresp     <= dbg_prev_rd_buf_resp;
          dbg_trip_rlast     <= 1'b1;
        end
      end

      if ((state == ST_RD_D) && rd_buf_valid && !HREADY && !rd_buf_resp[1]) begin
        if (!dbg_rd_pop_while_not_ready) begin
          dbg_rd_pop_while_not_ready <= 1'b1;
          dbg_rd_pop_cycle           <= dbg_cycle_counter;
          dbg_rd_pop_state           <= state;
          dbg_rd_pop_haddr           <= HADDR;
          dbg_rd_pop_beat_cnt        <= beat_cnt;
          dbg_rd_pop_hready          <= HREADY;
          dbg_rd_pop_resp            <= rd_buf_resp;
        end
        if (!dbg_trip_sticky) begin
          dbg_trip_sticky    <= 1'b1;
          dbg_trip_cause     <= DBG_TRIP_RD_POP_NO_HREADY;
          dbg_trip_cycle     <= dbg_cycle_counter;
          dbg_trip_state     <= state;
          dbg_trip_ap_addr   <= ap_addr;
          dbg_trip_haddr     <= HADDR;
          dbg_trip_beat_cnt  <= beat_cnt;
          dbg_trip_acc_cnt   <= acc_cnt;
          dbg_trip_flush_ptr <= flush_ptr;
          dbg_trip_htrans    <= HTRANS;
          dbg_trip_hwrite    <= HWRITE;
          dbg_trip_rresp     <= rd_buf_resp;
          dbg_trip_rlast     <= 1'b0;
        end
      end

      if ((state == ST_RD_D) && rd_buf_valid && (beat_cnt != 8'd0) &&
          (dbg_live_interleave != dbg_q_interleave) &&
          !(HSEL && HREADYIN && (HTRANS == TRN_NONSEQ) &&
            (HWRITE || (HADDR != ap_addr) || (HSIZE != ap_size)))) begin
        if (!dbg_rd_live_q_disagree) begin
          dbg_rd_live_q_disagree <= 1'b1;
          dbg_rd_live_q_cycle    <= dbg_cycle_counter;
          dbg_rd_live_q_state    <= state;
          dbg_rd_live_haddr      <= HADDR;
          dbg_rd_q_haddr         <= haddr_q;
          dbg_rd_live_htrans     <= HTRANS;
          dbg_rd_q_htrans        <= htrans_q;
          dbg_rd_live_hwrite     <= HWRITE;
          dbg_rd_q_hwrite        <= hwrite_q;
          dbg_rd_live_hsize      <= HSIZE;
          dbg_rd_q_hsize         <= hsize_q;
          dbg_rd_live_ap_addr    <= ap_addr;
          dbg_rd_live_ap_size    <= ap_size;
        end
        if (!dbg_trip_sticky) begin
          dbg_trip_sticky    <= 1'b1;
          dbg_trip_cause     <= DBG_TRIP_RD_LIVE_Q_DISAGREE;
          dbg_trip_cycle     <= dbg_cycle_counter;
          dbg_trip_state     <= state;
          dbg_trip_ap_addr   <= ap_addr;
          dbg_trip_haddr     <= HADDR;
          dbg_trip_beat_cnt  <= beat_cnt;
          dbg_trip_acc_cnt   <= acc_cnt;
          dbg_trip_flush_ptr <= flush_ptr;
          dbg_trip_htrans    <= HTRANS;
          dbg_trip_hwrite    <= HWRITE;
          dbg_trip_rresp     <= rd_buf_resp;
          dbg_trip_rlast     <= 1'b0;
        end
      end

      if (dbg_aw_fire) begin
        dbg_last_aw_addr   <= AWADDR;
        dbg_last_aw_len    <= AWLEN;
        dbg_last_aw_size   <= AWSIZE;
        dbg_last_aw_burst  <= AWBURST;

        if (dbg_wr_outstanding && !dbg_trip_sticky) begin
          dbg_trip_sticky    <= 1'b1;
          dbg_trip_cause     <= DBG_TRIP_AW_DOUBLE;
          dbg_trip_cycle     <= dbg_cycle_counter;
          dbg_trip_state     <= state;
          dbg_trip_ap_addr   <= ap_addr;
          dbg_trip_haddr     <= HADDR;
          dbg_trip_beat_cnt  <= beat_cnt;
          dbg_trip_acc_cnt   <= acc_cnt;
          dbg_trip_flush_ptr <= flush_ptr;
          dbg_trip_htrans    <= HTRANS;
          dbg_trip_hwrite    <= HWRITE;
          dbg_trip_rresp     <= 2'b00;
          dbg_trip_rlast     <= 1'b0;
        end
        dbg_wr_outstanding <= 1'b1;
      end

      if (dbg_w_fire) begin
        dbg_last_w_data      <= WDATA;
        dbg_last_w_strb      <= WSTRB;
        dbg_last_w_last      <= WLAST;
        dbg_last_w_flush_ptr <= flush_ptr;
      end

      if (dbg_b_fire) begin
        dbg_last_b_resp <= BRESP;

        if (!dbg_wr_outstanding && !dbg_trip_sticky) begin
          dbg_trip_sticky    <= 1'b1;
          dbg_trip_cause     <= DBG_TRIP_B_NO_WR;
          dbg_trip_cycle     <= dbg_cycle_counter;
          dbg_trip_state     <= state;
          dbg_trip_ap_addr   <= ap_addr;
          dbg_trip_haddr     <= HADDR;
          dbg_trip_beat_cnt  <= beat_cnt;
          dbg_trip_acc_cnt   <= acc_cnt;
          dbg_trip_flush_ptr <= flush_ptr;
          dbg_trip_htrans    <= HTRANS;
          dbg_trip_hwrite    <= HWRITE;
          dbg_trip_rresp     <= 2'b00;
          dbg_trip_rlast     <= 1'b0;
        end
        dbg_wr_outstanding <= 1'b0;
      end

      if (dbg_evt_code != DBG_EVT_NONE) begin
        dbg_hist3_code     <= dbg_hist2_code;
        dbg_hist2_code     <= dbg_hist1_code;
        dbg_hist1_code     <= dbg_hist0_code;
        dbg_hist0_code     <= dbg_evt_code;

        dbg_hist3_state    <= dbg_hist2_state;
        dbg_hist2_state    <= dbg_hist1_state;
        dbg_hist1_state    <= dbg_hist0_state;
        dbg_hist0_state    <= state;

        dbg_hist3_ap_addr  <= dbg_hist2_ap_addr;
        dbg_hist2_ap_addr  <= dbg_hist1_ap_addr;
        dbg_hist1_ap_addr  <= dbg_hist0_ap_addr;
        dbg_hist0_ap_addr  <= ap_addr;

        dbg_hist3_haddr    <= dbg_hist2_haddr;
        dbg_hist2_haddr    <= dbg_hist1_haddr;
        dbg_hist1_haddr    <= dbg_hist0_haddr;
        dbg_hist0_haddr    <= HADDR;

        dbg_hist3_beat_cnt <= dbg_hist2_beat_cnt;
        dbg_hist2_beat_cnt <= dbg_hist1_beat_cnt;
        dbg_hist1_beat_cnt <= dbg_hist0_beat_cnt;
        dbg_hist0_beat_cnt <= beat_cnt;
      end

      if (raw_r_capture) begin
        dbg_rd_capture_seq     <= dbg_rd_capture_seq + 16'd1;
        dbg_rd_buf_capture_seq <= dbg_rd_capture_seq + 16'd1;
      end
      if (dbg_rd_deliver_fire) begin
        dbg_rd_last_deliver_seq <= dbg_rd_buf_capture_seq;
      end
      dbg_rd_buf_capture_seq_dbg  <= dbg_rd_buf_capture_seq;
      dbg_rd_last_deliver_seq_dbg <= dbg_rd_last_deliver_seq;
      dbg_prev_rd_deliver_fire    <= dbg_rd_deliver_fire;
      dbg_prev_rd_buf_valid       <= rd_buf_valid;
      dbg_prev_rd_buf_resp        <= rd_buf_resp;
      dbg_prev_beat_cnt           <= beat_cnt;
      dbg_prev_cycle_state        <= state;
      dbg_prev_state              <= state;
    end
  end

`default_nettype wire
endmodule
