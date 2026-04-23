`default_nettype none

module ahb_to_axi4_burst_linux_crash_rootcause_fv;
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
    wire        tap_pnd_wfirst_capture_ok;
    wire        tap_rd_buf_valid;
    wire [1:0]  tap_rd_buf_resp;
    wire        tap_rd_d_entry;
    wire        tap_ahb_wphase_valid;
    wire [31:0] tap_ahb_waddr_d;
    wire [2:0]  tap_ahb_wsize_d;
    wire [7:0]  tap_ahb_wstrb_d;
    wire        tap_pnd_same_live_write_addrphase;
    wire        tap_fix_ahb_fire;

    wire        tap_dbg_trip;
    wire        tap_dbg_trip_sticky;
    wire [7:0]  tap_dbg_trip_cause;
    wire [7:0]  tap_dbg_trip_code_live;
    wire        tap_dbg_rd_outstanding;
    wire        tap_dbg_wr_outstanding;
    wire        tap_dbg_fix_write_unproven;
    wire        tap_dbg_pnd_align_unproven;
    wire        tap_dbg_pnd_flush_unproven;
    wire        tap_dbg_rd_pop_while_not_ready;
    wire        tap_dbg_rd_live_q_disagree;
    wire        tap_dbg_rd_repeat_deliver;
    wire        tap_dbg_rd_lastbeat_no_deliver;

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
        .tap_pnd_wfirst_capture_ok (tap_pnd_wfirst_capture_ok),
        .tap_rd_buf_valid     (tap_rd_buf_valid),
        .tap_rd_buf_resp      (tap_rd_buf_resp),
        .tap_rd_d_entry       (tap_rd_d_entry),
        .tap_ahb_wphase_valid (tap_ahb_wphase_valid),
        .tap_ahb_waddr_d      (tap_ahb_waddr_d),
        .tap_ahb_wsize_d      (tap_ahb_wsize_d),
        .tap_ahb_wstrb_d      (tap_ahb_wstrb_d),
        .tap_pnd_same_live_write_addrphase (tap_pnd_same_live_write_addrphase),
        .tap_fix_ahb_fire     (tap_fix_ahb_fire),
        .tap_dbg_trip         (tap_dbg_trip),
        .tap_dbg_trip_sticky  (tap_dbg_trip_sticky),
        .tap_dbg_trip_cause   (tap_dbg_trip_cause),
        .tap_dbg_trip_code_live (tap_dbg_trip_code_live),
        .tap_dbg_rd_outstanding (tap_dbg_rd_outstanding),
        .tap_dbg_wr_outstanding (tap_dbg_wr_outstanding),
        .tap_dbg_fix_write_unproven (tap_dbg_fix_write_unproven),
        .tap_dbg_pnd_align_unproven (tap_dbg_pnd_align_unproven),
        .tap_dbg_pnd_flush_unproven (tap_dbg_pnd_flush_unproven),
        .tap_dbg_rd_pop_while_not_ready (tap_dbg_rd_pop_while_not_ready),
        .tap_dbg_rd_live_q_disagree (tap_dbg_rd_live_q_disagree),
        .tap_dbg_rd_repeat_deliver (tap_dbg_rd_repeat_deliver),
        .tap_dbg_rd_lastbeat_no_deliver (tap_dbg_rd_lastbeat_no_deliver)
    );

`ifdef FORMAL
    reg f_past_valid;
    initial f_past_valid = 1'b0;

    localparam [4:0] FV_ST_WR_D         = 5'd2;
    localparam [4:0] FV_ST_RD_A         = 5'd11;
    localparam [4:0] FV_ST_RD_D         = 5'd12;
    localparam [4:0] FV_ST_RD_FENCE     = 5'd16;
    localparam [1:0] FV_TRN_NONSEQ      = 2'b10;
    localparam [2:0] FV_HB_INCR         = 3'b001;
    localparam [7:0] FV_DBG_TRIP_AR_DOUBLE      = 8'd1;
    localparam [7:0] FV_DBG_TRIP_R_NO_RD        = 8'd2;
    localparam [7:0] FV_DBG_TRIP_R_IN_RD_A      = 8'd3;
    localparam [7:0] FV_DBG_TRIP_R_OUTSIDE_RD   = 8'd4;
    localparam [7:0] FV_DBG_TRIP_RLAST_MISMATCH = 8'd5;
    localparam [7:0] FV_DBG_TRIP_AW_DOUBLE      = 8'd6;
    localparam [7:0] FV_DBG_TRIP_B_NO_WR        = 8'd7;
    localparam [7:0] FV_DBG_TRIP_FIX_WRITE_UNPROVEN   = 8'd8;
    localparam [7:0] FV_DBG_TRIP_RD_POP_NO_HREADY     = 8'd9;
    localparam [7:0] FV_DBG_TRIP_RD_LIVE_Q_DISAGREE   = 8'd10;
    localparam [7:0] FV_DBG_TRIP_PND_ALIGN_UNPROVEN   = 8'd11;
    localparam [7:0] FV_DBG_TRIP_PND_FLUSH_UNPROVEN   = 8'd12;
    localparam [7:0] FV_DBG_TRIP_RD_REPEAT_DELIVER    = 8'd13;
    localparam [7:0] FV_DBG_TRIP_RD_LASTBEAT_NO_DELIVER = 8'd14;

    reg [1:0] post_reset_ctr;
    wire      post_reset_ok = &post_reset_ctr;

    wire aw_fire     = m_axi_awvalid && m_axi_awready;
    wire w_fire      = m_axi_wvalid && m_axi_wready;
    wire w_last_fire = w_fire && m_axi_wlast;
    wire b_fire      = m_axi_bvalid && m_axi_bready;
    wire ar_fire     = m_axi_arvalid && m_axi_arready;
    wire r_fire      = m_axi_rvalid && m_axi_rready;
    wire r_last_fire = r_fire && m_axi_rlast;

    reg        write_in_flight;
    reg        wr_resp_pending;
    reg        read_in_flight;
    reg [7:0]  read_beats_left;
    reg [6:0]  aw_wait_age;
    reg [6:0]  w_wait_age;
    reg [6:0]  ar_wait_age;
    reg [6:0]  b_wait_age;
    reg [6:0]  r_gap_age;
    reg [6:0]  write_age;
    reg [6:0]  read_age;
    reg [6:0]  ahb_stall_age;
    reg        prev_delayed_b_waiting_read;
    reg        prev_fixed_capture_window;

    wire rd_noidle_rd = (tap_state == FV_ST_RD_D)
                     && !tap_pnd_valid
                     && tap_rd_buf_valid
                     && (tap_beat_cnt == 8'd0)
                     && !hresp
                     && hsel && hreadyin && hready
                     && (htrans == FV_TRN_NONSEQ)
                     && !hwrite
                     && ((haddr != tap_ap_addr) || (hsize != tap_ap_size));

    wire rd_noidle_wr = (tap_state == FV_ST_RD_D)
                     && !tap_pnd_valid
                     && tap_rd_buf_valid
                     && (tap_beat_cnt == 8'd0)
                     && !hresp
                     && hsel && hreadyin && hready
                     && (htrans == FV_TRN_NONSEQ)
                     && hwrite;

    wire delayed_b_waiting_read = write_in_flight
                               && !m_axi_bvalid
                               && tap_pnd_valid
                               && !tap_pnd_write;

    wire live_same_fixed_pending_write = hsel
                                      && hreadyin
                                      && (htrans == FV_TRN_NONSEQ)
                                      && hwrite
                                      && (haddr     == tap_pnd_addr)
                                      && (hsize     == tap_pnd_size)
                                      && (hburst    == tap_pnd_burst)
                                      && (hprot     == tap_pnd_prot)
                                      && (hmastlock == tap_pnd_lock);

    wire rd_fence_fixed_capture_window = tap_pnd_valid
                                      && tap_pnd_write
                                      && (tap_pnd_burst != FV_HB_INCR)
                                      && !tap_pnd_wfirst_valid
                                      && tap_ahb_wphase_valid
                                      && (tap_ahb_waddr_d == tap_pnd_addr)
                                      && (tap_ahb_wsize_d == tap_pnd_size)
                                      && !tap_pnd_same_live_write_addrphase
                                      && ((tap_state == FV_ST_RD_D)
                                          || (tap_state == FV_ST_RD_FENCE));

    always @(posedge clk) begin
        f_past_valid <= 1'b1;

        if (!f_past_valid) begin
            assume(!resetn);
        end else if (!$past(resetn)) begin
            assume(resetn);
        end else begin
            assume(resetn);
        end

        assume(hsize <= 3'd3);
        assume(tap_ap_size <= 3'd3);
        assume(tap_pnd_size <= 3'd3);

        if (post_reset_ok
            && tap_pnd_valid
            && tap_pnd_write
            && (tap_pnd_burst != FV_HB_INCR)
            && !tap_pnd_wfirst_valid) begin
            assume(!hsel || !hreadyin || (htrans != FV_TRN_NONSEQ) || live_same_fixed_pending_write);
        end

        // If the previous cycle presented an active AHB transfer while the
        // bridge stalled it, the master must keep the address/control/data
        // stable on this cycle.
        if (f_past_valid && $past(resetn && hsel && htrans[1] && !hready)) begin
            assume($stable(haddr));
            assume($stable(hsize));
            assume($stable(hburst));
            assume($stable(hprot));
            assume($stable(hmastlock));
            assume($stable(htrans));
            assume($stable(hwrite));
            assume($stable(hwdata));
        end

        if (resetn && m_axi_bvalid && !m_axi_bready) begin
            assume(m_axi_bvalid);
            assume($stable(m_axi_bresp));
        end

        if (resetn && m_axi_rvalid && !m_axi_rready) begin
            assume(m_axi_rvalid);
            assume($stable(m_axi_rdata));
            assume($stable(m_axi_rresp));
            assume($stable(m_axi_rlast));
        end

        if (resetn && m_axi_bvalid) begin
            assume(m_axi_bresp == 2'b00);
            assume(wr_resp_pending || w_last_fire);
        end

        if (resetn && m_axi_rvalid) begin
            assume(m_axi_rresp == 2'b00);
            assume(read_in_flight);
            assume(read_beats_left != 8'd0);
            assume(m_axi_rlast == (read_beats_left == 8'd1));
        end

        if (!resetn) begin
            post_reset_ctr               <= 2'd0;
            write_in_flight              <= 1'b0;
            wr_resp_pending              <= 1'b0;
            read_in_flight               <= 1'b0;
            read_beats_left              <= 8'd0;
            aw_wait_age                  <= 7'd0;
            w_wait_age                   <= 7'd0;
            ar_wait_age                  <= 7'd0;
            b_wait_age                   <= 7'd0;
            r_gap_age                    <= 7'd0;
            write_age                    <= 7'd0;
            read_age                     <= 7'd0;
            ahb_stall_age                <= 7'd0;
            prev_delayed_b_waiting_read  <= 1'b0;
            prev_fixed_capture_window    <= 1'b0;
        end else begin
            if (!post_reset_ok)
                post_reset_ctr <= post_reset_ctr + 2'd1;

            prev_delayed_b_waiting_read <= delayed_b_waiting_read;
            prev_fixed_capture_window   <= rd_fence_fixed_capture_window;

            if (!m_axi_awvalid || m_axi_awready)
                aw_wait_age <= 7'd0;
            else
                aw_wait_age <= aw_wait_age + 7'd1;

            if (!m_axi_wvalid || m_axi_wready)
                w_wait_age <= 7'd0;
            else
                w_wait_age <= w_wait_age + 7'd1;

            if (!m_axi_arvalid || m_axi_arready)
                ar_wait_age <= 7'd0;
            else
                ar_wait_age <= ar_wait_age + 7'd1;

            if (!wr_resp_pending || m_axi_bvalid)
                b_wait_age <= 7'd0;
            else
                b_wait_age <= b_wait_age + 7'd1;

            if (!read_in_flight || m_axi_rvalid)
                r_gap_age <= 7'd0;
            else
                r_gap_age <= r_gap_age + 7'd1;

            if (!write_in_flight)
                write_age <= 7'd0;
            else if (!b_fire)
                write_age <= write_age + 7'd1;

            if (!read_in_flight)
                read_age <= 7'd0;
            else if (!r_last_fire)
                read_age <= read_age + 7'd1;

            if (!(hsel && hreadyin && htrans[1] && !hready))
                ahb_stall_age <= 7'd0;
            else
                ahb_stall_age <= ahb_stall_age + 7'd1;

            if (aw_fire)
                write_in_flight <= 1'b1;
            if (b_fire)
                write_in_flight <= 1'b0;

            if (w_last_fire)
                wr_resp_pending <= 1'b1;
            if (b_fire)
                wr_resp_pending <= 1'b0;

            if (ar_fire) begin
                read_in_flight  <= 1'b1;
                read_beats_left <= m_axi_arlen + 8'd1;
            end

            if (r_fire && (read_beats_left != 8'd0))
                read_beats_left <= read_beats_left - 8'd1;
            if (r_last_fire) begin
                read_in_flight  <= 1'b0;
                read_beats_left <= 8'd0;
            end

            if (post_reset_ok) begin
                assume(aw_wait_age < 7'd4);
                assume(w_wait_age < 7'd4);
                assume(ar_wait_age < 7'd4);
                assume(b_wait_age < 7'd16);
                assume(r_gap_age < 7'd16);

                assert(!tap_dbg_fix_write_unproven);
                assert(!tap_dbg_pnd_flush_unproven);
                assert(!tap_dbg_rd_pop_while_not_ready);
                assert(!tap_dbg_rd_live_q_disagree);
                assert(!tap_dbg_rd_repeat_deliver);
                assert(!tap_dbg_rd_lastbeat_no_deliver);

                assert(tap_dbg_trip_code_live != FV_DBG_TRIP_AR_DOUBLE);
                assert(tap_dbg_trip_code_live != FV_DBG_TRIP_R_NO_RD);
                assert(tap_dbg_trip_code_live != FV_DBG_TRIP_R_IN_RD_A);
                assert(tap_dbg_trip_code_live != FV_DBG_TRIP_R_OUTSIDE_RD);
                assert(tap_dbg_trip_code_live != FV_DBG_TRIP_RLAST_MISMATCH);
                assert(tap_dbg_trip_code_live != FV_DBG_TRIP_AW_DOUBLE);
                assert(tap_dbg_trip_code_live != FV_DBG_TRIP_B_NO_WR);
                assert(tap_dbg_trip_code_live != FV_DBG_TRIP_FIX_WRITE_UNPROVEN);
                assert(tap_dbg_trip_code_live != FV_DBG_TRIP_RD_POP_NO_HREADY);
                assert(tap_dbg_trip_code_live != FV_DBG_TRIP_RD_LIVE_Q_DISAGREE);
                assert(tap_dbg_trip_code_live != FV_DBG_TRIP_PND_FLUSH_UNPROVEN);
                assert(tap_dbg_trip_code_live != FV_DBG_TRIP_RD_REPEAT_DELIVER);
                assert(tap_dbg_trip_code_live != FV_DBG_TRIP_RD_LASTBEAT_NO_DELIVER);

                // ST_WR_PND_ALIGN can legally stall until either the delayed
                // write-phase tracker proves beat 0 or the held NONSEQ is seen
                // again, so do not treat PND_ALIGN_UNPROVEN as a hard failure
                // in this crash-focused pass.
                assert(!tap_dbg_trip || (tap_dbg_trip_code_live == FV_DBG_TRIP_PND_ALIGN_UNPROVEN));
                assert(!tap_dbg_trip_sticky || (tap_dbg_trip_cause == FV_DBG_TRIP_PND_ALIGN_UNPROVEN));
                assert((tap_dbg_trip_code_live == 8'd0)
                    || (tap_dbg_trip_code_live == FV_DBG_TRIP_PND_ALIGN_UNPROVEN));
                assert((tap_dbg_trip_cause == 8'd0)
                    || (tap_dbg_trip_cause == FV_DBG_TRIP_PND_ALIGN_UNPROVEN));

                assert(tap_dbg_wr_outstanding == write_in_flight);
                assert(tap_dbg_rd_outstanding == read_in_flight);

                assert(!write_in_flight || !m_axi_arvalid);
                assert(write_age < 7'd100);
                assert(read_age < 7'd100);
                assert(ahb_stall_age < 7'd100);

                if (read_in_flight)
                    assert(read_beats_left != 8'd0);

                cover(rd_noidle_rd);
                cover(rd_noidle_wr);
                cover(prev_delayed_b_waiting_read && m_axi_bvalid);
                cover(prev_fixed_capture_window && tap_pnd_wfirst_valid);
            end
        end
    end
`endif

endmodule

`default_nettype wire
