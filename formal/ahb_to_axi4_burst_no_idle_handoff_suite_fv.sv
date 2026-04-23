`default_nettype none

module ahb_to_axi4_burst_no_idle_handoff_suite_fv;
    (* gclk *) reg         clk;
    reg                    resetn;

    (* anyseq *) reg         hsel;
    (* anyseq *) reg [31:0]  haddr;
    (* anyseq *) reg [1:0]   htrans;
    (* anyseq *) reg [2:0]   hsize;
    (* anyseq *) reg [2:0]   hburst;
    (* anyseq *) reg [3:0]   hprot;
    (* anyseq *) reg         hmastlock;
    (* anyseq *) reg         hwrite;
    (* anyseq *) reg [63:0]  hwdata;
    (* anyseq *) reg         hreadyin;

    wire [63:0]  hrdata;
    wire         hready;
    wire         hresp;

    wire [31:0]  m_axi_awaddr;
    wire [7:0]   m_axi_awlen;
    wire [2:0]   m_axi_awsize;
    wire [1:0]   m_axi_awburst;
    wire         m_axi_awlock;
    wire [3:0]   m_axi_awcache;
    wire [2:0]   m_axi_awprot;
    wire         m_axi_awvalid;
    (* anyseq *) reg         m_axi_awready;

    wire [63:0]  m_axi_wdata;
    wire [7:0]   m_axi_wstrb;
    wire         m_axi_wlast;
    wire         m_axi_wvalid;
    (* anyseq *) reg         m_axi_wready;

    (* anyseq *) reg [1:0]   m_axi_bresp;
    (* anyseq *) reg         m_axi_bvalid;
    wire         m_axi_bready;

    wire [31:0]  m_axi_araddr;
    wire [7:0]   m_axi_arlen;
    wire [2:0]   m_axi_arsize;
    wire [1:0]   m_axi_arburst;
    wire         m_axi_arlock;
    wire [3:0]   m_axi_arcache;
    wire [2:0]   m_axi_arprot;
    wire         m_axi_arvalid;
    (* anyseq *) reg         m_axi_arready;

    (* anyseq *) reg [63:0]  m_axi_rdata;
    (* anyseq *) reg [1:0]   m_axi_rresp;
    (* anyseq *) reg         m_axi_rlast;
    (* anyseq *) reg         m_axi_rvalid;
    wire         m_axi_rready;

    initial resetn = 1'b0;
    always @(posedge clk) resetn <= 1'b1;
    wire [3:0] unused_awid;
    wire [3:0] m_axi_awqos;
    wire [3:0] unused_arid;
    wire [3:0] m_axi_arqos;

    wire [4:0]  tap_state;
    wire [31:0] tap_ap_addr;
    wire [2:0]  tap_ap_size;
    wire [3:0]  tap_ap_prot;
    wire        tap_ap_lock;
    wire [7:0]  tap_beat_cnt;
    wire        tap_ar_done;
    wire        tap_pnd_valid;
    wire [31:0] tap_pnd_addr;
    wire [2:0]  tap_pnd_size;
    wire [2:0]  tap_pnd_burst;
    wire        tap_pnd_write;
    wire [3:0]  tap_pnd_prot;
    wire        tap_pnd_lock;
    wire        tap_pnd_wfirst_valid;
    wire [63:0] tap_pnd_wfirst_data;
    wire [7:0]  tap_pnd_wfirst_strb;
    wire        tap_rd_buf_valid;
    wire        tap_ahb_wphase_valid;
    wire [31:0] tap_ahb_waddr_d;
    wire [2:0]  tap_ahb_wsize_d;
    wire [7:0]  tap_ahb_wstrb_d;
    wire        tap_pnd_same_live_write_addrphase;

    ahb_to_axi4_burst_formal_tap #(
        .AW             (32),
        .DW             (64),
        .IW             (4),
        .MAX_INCR_BEATS (16)
    ) dut (
        .clk            (clk),
        .resetn         (resetn),
        .HSEL           (hsel),
        .HREADYIN       (hreadyin),
        .HADDR          (haddr),
        .HBURST         (hburst),
        .HMASTLOCK      (hmastlock),
        .HPROT          (hprot),
        .HSIZE          (hsize),
        .HTRANS         (htrans),
        .HWDATA         (hwdata),
        .HWRITE         (hwrite),
        .HRDATA         (hrdata),
        .HREADY         (hready),
        .HRESP          (hresp),
        .AWID           (unused_awid),
        .AWADDR         (m_axi_awaddr),
        .AWLEN          (m_axi_awlen),
        .AWSIZE         (m_axi_awsize),
        .AWBURST        (m_axi_awburst),
        .AWLOCK         (m_axi_awlock),
        .AWCACHE        (m_axi_awcache),
        .AWPROT         (m_axi_awprot),
        .AWQOS          (m_axi_awqos),
        .AWVALID        (m_axi_awvalid),
        .AWREADY        (m_axi_awready),
        .WDATA          (m_axi_wdata),
        .WSTRB          (m_axi_wstrb),
        .WLAST          (m_axi_wlast),
        .WVALID         (m_axi_wvalid),
        .WREADY         (m_axi_wready),
        .BID            (4'b0),
        .BRESP          (m_axi_bresp),
        .BVALID         (m_axi_bvalid),
        .BREADY         (m_axi_bready),
        .ARID           (unused_arid),
        .ARADDR         (m_axi_araddr),
        .ARLEN          (m_axi_arlen),
        .ARSIZE         (m_axi_arsize),
        .ARBURST        (m_axi_arburst),
        .ARLOCK         (m_axi_arlock),
        .ARCACHE        (m_axi_arcache),
        .ARPROT         (m_axi_arprot),
        .ARQOS          (m_axi_arqos),
        .ARVALID        (m_axi_arvalid),
        .ARREADY        (m_axi_arready),
        .RID            (4'b0),
        .RDATA          (m_axi_rdata),
        .RRESP          (m_axi_rresp),
        .RLAST          (m_axi_rlast),
        .RVALID         (m_axi_rvalid),
        .RREADY         (m_axi_rready),

        .tap_state            (tap_state),
        .tap_ap_addr          (tap_ap_addr),
        .tap_ap_size          (tap_ap_size),
        .tap_ap_prot          (tap_ap_prot),
        .tap_ap_lock          (tap_ap_lock),
        .tap_beat_cnt         (tap_beat_cnt),
        .tap_ar_done          (tap_ar_done),
        .tap_pnd_valid        (tap_pnd_valid),
        .tap_pnd_addr         (tap_pnd_addr),
        .tap_pnd_size         (tap_pnd_size),
        .tap_pnd_burst        (tap_pnd_burst),
        .tap_pnd_write        (tap_pnd_write),
        .tap_pnd_prot         (tap_pnd_prot),
        .tap_pnd_lock         (tap_pnd_lock),
        .tap_pnd_wfirst_valid (tap_pnd_wfirst_valid),
        .tap_pnd_wfirst_data  (tap_pnd_wfirst_data),
        .tap_pnd_wfirst_strb  (tap_pnd_wfirst_strb),
        .tap_rd_buf_valid     (tap_rd_buf_valid),
        .tap_ahb_wphase_valid (tap_ahb_wphase_valid),
        .tap_ahb_waddr_d      (tap_ahb_waddr_d),
        .tap_ahb_wsize_d      (tap_ahb_wsize_d),
        .tap_ahb_wstrb_d      (tap_ahb_wstrb_d),
        .tap_pnd_same_live_write_addrphase (tap_pnd_same_live_write_addrphase)
    );

`ifdef FORMAL
    reg f_past_valid;
    initial f_past_valid = 1'b0;

    localparam [4:0] FV_ST_IDLE           = 5'd0;
    localparam [4:0] FV_ST_WR_PND_ALIGN   = 5'd1;
    localparam [4:0] FV_ST_WR_D           = 5'd2;
    localparam [4:0] FV_ST_WR_FIX_FLUSH   = 5'd3;
    localparam [4:0] FV_ST_WR_INCR_ACC    = 5'd4;
    localparam [4:0] FV_ST_WR_INCR_RESUME = 5'd6;
    localparam [4:0] FV_ST_WR_RESP        = 5'd8;
    localparam [4:0] FV_ST_WR_LAST_RESP   = 5'd9;
    localparam [4:0] FV_ST_RD_D           = 5'd12;
    localparam [4:0] FV_ST_RD_FENCE       = 5'd16;

    localparam [1:0] FV_TRN_NONSEQ = 2'b10;
    localparam [2:0] FV_HB_INCR    = 3'b001;

    reg [1:0]  post_reset_ctr;
    wire       post_reset_ok = &post_reset_ctr;

    reg prev_wr_to_rd;
    reg prev_wr_to_wr;
    reg prev_rd_to_rd;
    reg prev_rd_to_wr;

    reg [31:0] prev_addr;
    reg [2:0]  prev_size;
    reg [2:0]  prev_burst;
    reg [3:0]  prev_prot;
    reg        prev_lock;
    reg        prev_is_write;

    reg        track_fixed_pnd_write;
    reg        track_fixed_wait_wfirst;
    reg [31:0] track_pw_addr;
    reg [2:0]  track_pw_size;
    reg [63:0] prev_wfirst_data;
    reg [7:0]  prev_wfirst_strb;
    reg        prev_capture_wfirst;

    // Write-side pending-capture windows must match the actual DUT paths.
    //
    // Fixed write (ST_WR_D): the next NONSEQ is only latched on an actual
    // accepted near-final write beat.
    // Fixed flush / response wait: pending NONSEQ can be latched directly.
    // INCR ACC/RESUME: a live NONSEQ means the INCR burst is ending and the
    // DUT takes the pending transaction on that exit path.
    wire wr_fix_d_capture_window = (tap_state == FV_ST_WR_D)
                                && tap_ahb_wphase_valid
                                && (tap_beat_cnt <= 8'd1);

    wire wr_fix_post_window = (tap_state == FV_ST_WR_FIX_FLUSH)
                           || (tap_state == FV_ST_WR_RESP)
                           || (tap_state == FV_ST_WR_LAST_RESP);

    wire wr_incr_exit_window = (tap_state == FV_ST_WR_INCR_ACC)
                            || (tap_state == FV_ST_WR_INCR_RESUME);

    wire wr_capture_window = wr_fix_d_capture_window
                          || wr_fix_post_window
                          || wr_incr_exit_window;

    wire wr_noidle_rd = wr_capture_window
                     && !tap_pnd_valid
                     && hsel && hreadyin && hready
                     && (htrans == FV_TRN_NONSEQ)
                     && !hwrite;

    wire wr_noidle_wr = wr_capture_window
                     && !tap_pnd_valid
                     && hsel && hreadyin && hready
                     && (htrans == FV_TRN_NONSEQ)
                     && hwrite;

    wire rd_noidle_wr = (tap_state == FV_ST_RD_D)
                     && !tap_pnd_valid
                     && tap_rd_buf_valid
                     && (tap_beat_cnt == 8'd0)
                     && !hresp
                     && hsel && hreadyin && hready
                     && (htrans == FV_TRN_NONSEQ)
                     && hwrite;

    wire rd_noidle_rd = (tap_state == FV_ST_RD_D)
                     && !tap_pnd_valid
                     && tap_rd_buf_valid
                     && (tap_beat_cnt == 8'd0)
                     && !hresp
                     && hsel && hreadyin && hready
                     && (htrans == FV_TRN_NONSEQ)
                     && !hwrite
                     && ((haddr != tap_ap_addr) || (hsize != tap_ap_size));

    wire tracked_fixed_write_can_capture_wfirst = track_fixed_pnd_write
                                               && tap_pnd_valid
                                               && tap_pnd_write
                                               && (tap_pnd_burst != FV_HB_INCR)
                                               && !tap_pnd_wfirst_valid
                                               && tap_ahb_wphase_valid
                                               && (tap_ahb_waddr_d == track_pw_addr)
                                               && (tap_ahb_wsize_d == track_pw_size)
                                               && !tap_pnd_same_live_write_addrphase
                                               && ((tap_state == FV_ST_RD_D)
                                                   || (tap_state == FV_ST_RD_FENCE));
    wire live_same_track_fixed = hsel
                              && hreadyin
                              && (htrans == FV_TRN_NONSEQ)
                              && hwrite
                              && (haddr == track_pw_addr)
                              && (hsize == track_pw_size);

    always @(posedge clk) begin
        f_past_valid <= 1'b1;

        if (!f_past_valid) begin
            assume(!resetn);
        end else if (!$past(resetn)) begin
            assume(resetn);
        end else begin
            assume(resetn);
        end

        // basic legality so the suite does not fail on meaningless anyseq traffic
        assume(hsize <= 3'd3);
        assume(tap_ap_size <= 3'd3);
        assume(tap_pnd_size <= 3'd3);

        // AHB masters must hold address/control steady while the slave stalls
        // an active transfer. Without this, the solver can invent impossible
        // no-idle handoffs that overwrite the pending slot while HREADY=0.
        if (resetn && hsel && htrans[1] && !hready) begin
            assume($stable(haddr));
            assume($stable(hsize));
            assume($stable(hburst));
            assume($stable(hprot));
            assume($stable(hmastlock));
            assume($stable(htrans));
            assume($stable(hwrite));
            assume($stable(hwdata));
        end

        if (!resetn) begin
            post_reset_ctr        <= 2'd0;
            prev_wr_to_rd         <= 1'b0;
            prev_wr_to_wr         <= 1'b0;
            prev_rd_to_rd         <= 1'b0;
            prev_rd_to_wr         <= 1'b0;
            prev_addr             <= 32'd0;
            prev_size             <= 3'd0;
            prev_burst            <= 3'd0;
            prev_prot             <= 4'd0;
            prev_lock             <= 1'b0;
            prev_is_write         <= 1'b0;
            track_fixed_pnd_write <= 1'b0;
            track_fixed_wait_wfirst <= 1'b0;
            track_pw_addr         <= 32'd0;
            track_pw_size         <= 3'd0;
            prev_capture_wfirst   <= 1'b0;
            prev_wfirst_data      <= 64'd0;
            prev_wfirst_strb      <= 8'd0;
        end else begin
            if (!post_reset_ok)
                post_reset_ctr <= post_reset_ctr + 2'd1;

            prev_wr_to_rd       <= 1'b0;
            prev_wr_to_wr       <= 1'b0;
            prev_rd_to_rd       <= 1'b0;
            prev_rd_to_wr       <= 1'b0;
            prev_capture_wfirst <= 1'b0;

            if (post_reset_ok && track_fixed_pnd_write) begin
                assume(!hsel || !hreadyin || (htrans != FV_TRN_NONSEQ) || live_same_track_fixed);
            end

            if (!post_reset_ok) begin
                track_fixed_pnd_write <= 1'b0;
            end else begin
                if (prev_wr_to_rd || prev_rd_to_rd) begin
                    assert(tap_pnd_valid);
                    assert(!tap_pnd_write);
                    assert(tap_pnd_addr  == prev_addr);
                    assert(tap_pnd_size  == prev_size);
                    assert(tap_pnd_burst == prev_burst);
                    assert(tap_pnd_prot  == prev_prot);
                    assert(tap_pnd_lock  == prev_lock);
                    assert(!tap_pnd_wfirst_valid);
                end

                if (prev_wr_to_wr || prev_rd_to_wr) begin
                    assert(tap_pnd_valid);
                    assert(tap_pnd_write);
                    assert(tap_pnd_addr  == prev_addr);
                    assert(tap_pnd_size  == prev_size);
                    assert(tap_pnd_burst == prev_burst);
                    assert(tap_pnd_prot  == prev_prot);
                    assert(tap_pnd_lock  == prev_lock);
                    assert(!tap_pnd_wfirst_valid);

                    if (prev_burst != FV_HB_INCR) begin
                        track_fixed_pnd_write <= 1'b1;
                        track_fixed_wait_wfirst <= 1'b0;
                        track_pw_addr         <= prev_addr;
                        track_pw_size         <= prev_size;
                    end
                end
                if (track_fixed_pnd_write) begin
                    if (!track_fixed_wait_wfirst) begin
                        if (tap_pnd_valid) begin
                            assert(tap_pnd_write);
                            assert(tap_pnd_addr == track_pw_addr);
                            assert(tap_pnd_size == track_pw_size);
                        end else begin
                            assert((tap_state == FV_ST_WR_PND_ALIGN) || (tap_state == FV_ST_WR_D));
                            assert(tap_ap_addr == track_pw_addr);
                            assert(tap_ap_size == track_pw_size);
                            if (tap_pnd_wfirst_valid) begin
                                track_fixed_pnd_write   <= 1'b0;
                                track_fixed_wait_wfirst <= 1'b0;
                            end else begin
                                track_fixed_wait_wfirst <= 1'b1;
                            end
                        end
                    end else begin
                        if (tap_pnd_valid) begin
                            assert(tap_pnd_write);
                            assert(tap_pnd_addr == track_pw_addr);
                            assert(tap_pnd_size == track_pw_size);
                        end else begin
                            assert((tap_state == FV_ST_WR_PND_ALIGN)
                                || (tap_state == FV_ST_WR_D)
                                || (tap_state == FV_ST_WR_FIX_FLUSH)
                                || (tap_state == FV_ST_WR_RESP)
                                || (tap_state == FV_ST_WR_LAST_RESP));
                            assert(tap_ap_addr == track_pw_addr);
                            assert(tap_ap_size == track_pw_size);
                        end
                        if (tap_pnd_wfirst_valid) begin
                            track_fixed_pnd_write <= 1'b0;
                            track_fixed_wait_wfirst <= 1'b0;
                        end
                    end
                end

                if (prev_capture_wfirst) begin
                    assert(tap_pnd_wfirst_valid);
                    assert(tap_pnd_wfirst_data == prev_wfirst_data);
                    assert(tap_pnd_wfirst_strb == prev_wfirst_strb);
                end
            end

            if (post_reset_ok) begin
                if (wr_noidle_rd) begin
                    prev_wr_to_rd <= 1'b1;
                    prev_addr     <= haddr;
                    prev_size     <= hsize;
                    prev_burst    <= hburst;
                    prev_prot     <= hprot;
                    prev_lock     <= hmastlock;
                    prev_is_write <= 1'b0;
                end

                if (wr_noidle_wr) begin
                    prev_wr_to_wr <= 1'b1;
                    prev_addr     <= haddr;
                    prev_size     <= hsize;
                    prev_burst    <= hburst;
                    prev_prot     <= hprot;
                    prev_lock     <= hmastlock;
                    prev_is_write <= 1'b1;
                end

                if (rd_noidle_rd) begin
                    prev_rd_to_rd <= 1'b1;
                    prev_addr     <= haddr;
                    prev_size     <= hsize;
                    prev_burst    <= hburst;
                    prev_prot     <= hprot;
                    prev_lock     <= hmastlock;
                    prev_is_write <= 1'b0;
                end

                if (rd_noidle_wr) begin
                    prev_rd_to_wr <= 1'b1;
                    prev_addr     <= haddr;
                    prev_size     <= hsize;
                    prev_burst    <= hburst;
                    prev_prot     <= hprot;
                    prev_lock     <= hmastlock;
                    prev_is_write <= 1'b1;
                end

                if (tracked_fixed_write_can_capture_wfirst) begin
                    prev_capture_wfirst <= 1'b1;
                    prev_wfirst_data    <= hwdata;
                    prev_wfirst_strb    <= tap_ahb_wstrb_d;
                end

                cover(wr_noidle_rd);
                cover(wr_noidle_wr);
                cover(rd_noidle_rd);
                cover(rd_noidle_wr);
                cover(tracked_fixed_write_can_capture_wfirst);
            end
        end
    end
`endif

endmodule

`default_nettype wire
