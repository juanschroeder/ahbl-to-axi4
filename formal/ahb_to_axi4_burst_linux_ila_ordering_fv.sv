`default_nettype none

module ahb_to_axi4_burst_linux_ila_ordering_fv;
    (* gclk *) reg clk;
    reg            resetn;

    reg         hsel;
    reg [31:0]  haddr;
    reg [1:0]   htrans;
    reg [2:0]   hsize;
    reg [2:0]   hburst;
    reg [3:0]   hprot;
    reg         hmastlock;
    reg         hwrite;
    reg [63:0]  hwdata;
    reg         hreadyin;

    wire [63:0] hrdata;
    wire        hready;
    wire        hresp;

    wire [31:0] m_axi_awaddr;
    wire [7:0]  m_axi_awlen;
    wire [2:0]  m_axi_awsize;
    wire [1:0]  m_axi_awburst;
    wire        m_axi_awlock;
    wire [3:0]  m_axi_awcache;
    wire [2:0]  m_axi_awprot;
    wire        m_axi_awvalid;
    reg         m_axi_awready;

    wire [63:0] m_axi_wdata;
    wire [7:0]  m_axi_wstrb;
    wire        m_axi_wlast;
    wire        m_axi_wvalid;
    reg         m_axi_wready;

    reg [1:0]   m_axi_bresp;
    reg         m_axi_bvalid;
    wire        m_axi_bready;

    wire [31:0] m_axi_araddr;
    wire [7:0]  m_axi_arlen;
    wire [2:0]  m_axi_arsize;
    wire [1:0]  m_axi_arburst;
    wire        m_axi_arlock;
    wire [3:0]  m_axi_arcache;
    wire [2:0]  m_axi_arprot;
    wire        m_axi_arvalid;
    reg         m_axi_arready;

    reg [63:0]  m_axi_rdata;
    reg [1:0]   m_axi_rresp;
    reg         m_axi_rlast;
    reg         m_axi_rvalid;
    wire        m_axi_rready;

    localparam [31:0] WRITE_ADDR      = 32'hBE7A_CEC8;
    localparam [31:0] READ_ADDR       = 32'h81F8_14C0;
    localparam [2:0]  HSIZE_QWORD     = 3'd3;
    localparam [2:0]  HBURST_INCR8    = 3'b101;
    localparam [1:0]  HTRANS_IDLE     = 2'b00;
    localparam [1:0]  HTRANS_NONSEQ   = 2'b10;
    localparam [1:0]  HTRANS_SEQ      = 2'b11;
    localparam [2:0]  AXI_BURST_INCR  = 2'b01;
    localparam integer NUM_BEATS      = 8;
    localparam integer B_DELAY_CYCLES = 6;

    localparam [2:0] M_WR_ACTIVE      = 3'd0;
    localparam [2:0] M_WR_POST_WAIT   = 3'd1;
    localparam [2:0] M_RD_ACTIVE      = 3'd2;
    localparam [2:0] M_RD_DONE        = 3'd3;

    wire [3:0] unused_awid;
    wire [3:0] m_axi_awqos;
    wire [3:0] unused_arid;
    wire [3:0] m_axi_arqos;

    wire [4:0] tap_state;
    wire [7:0] tap_beat_cnt;
    wire       tap_pnd_valid;
    wire       tap_pnd_write;
    wire       tap_dbg_trip;
    wire [7:0] tap_dbg_trip_cause;
    wire [7:0] tap_dbg_trip_code_live;

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

        .tap_state          (tap_state),
        .tap_ap_addr        (),
        .tap_ap_size        (),
        .tap_ap_prot        (),
        .tap_ap_lock        (),
        .tap_beat_cnt       (tap_beat_cnt),
        .tap_ar_done        (),
        .tap_pnd_valid      (tap_pnd_valid),
        .tap_pnd_addr       (),
        .tap_pnd_size       (),
        .tap_pnd_burst      (),
        .tap_pnd_write      (tap_pnd_write),
        .tap_pnd_prot       (),
        .tap_pnd_lock       (),
        .tap_pnd_wfirst_valid (),
        .tap_pnd_wfirst_data (),
        .tap_pnd_wfirst_strb (),
        .tap_pnd_wfirst_capture_ok (),
        .tap_rd_buf_valid   (),
        .tap_rd_buf_resp    (),
        .tap_rd_d_entry     (),
        .tap_ahb_wphase_valid (),
        .tap_ahb_waddr_d    (),
        .tap_ahb_wsize_d    (),
        .tap_ahb_wstrb_d    (),
        .tap_pnd_same_live_write_addrphase (),
        .tap_fix_ahb_fire   (),
        .tap_incr_rd        (),
        .tap_incr_wr_cont   (),
        .tap_dbg_trip       (tap_dbg_trip),
        .tap_dbg_trip_sticky (),
        .tap_dbg_trip_cause (tap_dbg_trip_cause),
        .tap_dbg_trip_code_live (tap_dbg_trip_code_live),
        .tap_dbg_rd_outstanding (),
        .tap_dbg_wr_outstanding (),
        .tap_dbg_fix_write_unproven (),
        .tap_dbg_pnd_align_unproven (),
        .tap_dbg_pnd_flush_unproven (),
        .tap_dbg_rd_pop_while_not_ready (),
        .tap_dbg_rd_live_q_disagree (),
        .tap_dbg_rd_repeat_deliver (),
        .tap_dbg_rd_lastbeat_no_deliver ()
    );

`ifdef FORMAL
    reg f_past_valid;
    initial f_past_valid = 1'b0;

    reg [1:0] post_reset_ctr;
    wire      post_reset_ok = &post_reset_ctr;

    reg [2:0]  master_phase;
    reg [3:0]  wr_beat_idx;
    reg [3:0]  rd_beat_idx;
    reg        write_helper_done;
    reg        read_helper_done;

    reg        aw_seen;
    reg [3:0]  w_fire_count;
    reg        last_w_seen;
    reg        b_hs_done;
    reg [3:0]  b_wait_ctr;
    reg        ar_hs_done;
    reg [3:0]  r_fire_count;
    reg        saw_delayed_b_window;

    wire aw_fire     = m_axi_awvalid && m_axi_awready;
    wire w_fire      = m_axi_wvalid && m_axi_wready;
    wire w_last_fire = w_fire && m_axi_wlast;
    wire b_fire      = m_axi_bvalid && m_axi_bready;
    wire ar_fire     = m_axi_arvalid && m_axi_arready;
    wire r_fire      = m_axi_rvalid && m_axi_rready;
    wire r_last_fire = r_fire && m_axi_rlast;

    function automatic [63:0] wr_data(input [3:0] idx);
        wr_data = 64'h1000_0000_0000_0000 | idx;
    endfunction

    function automatic [63:0] rd_data(input [3:0] idx);
        rd_data = 64'h2000_0000_0000_0000 | idx;
    endfunction

    always @(posedge clk) begin
        f_past_valid <= 1'b1;

        if (!f_past_valid) begin
            assume(!resetn);
        end else if (!$past(resetn)) begin
            assume(resetn);
        end else begin
            assume(resetn);
        end

        if (!resetn) begin
            resetn            <= 1'b1;

            hsel              <= 1'b1;
            haddr             <= WRITE_ADDR;
            htrans            <= HTRANS_NONSEQ;
            hsize             <= HSIZE_QWORD;
            hburst            <= HBURST_INCR8;
            hprot             <= 4'd0;
            hmastlock         <= 1'b0;
            hwrite            <= 1'b1;
            hwdata            <= wr_data(4'd0);
            hreadyin          <= 1'b1;

            m_axi_awready     <= 1'b1;
            m_axi_wready      <= 1'b1;
            m_axi_bresp       <= 2'b00;
            m_axi_bvalid      <= 1'b0;
            m_axi_arready     <= 1'b1;
            m_axi_rdata       <= rd_data(4'd0);
            m_axi_rresp       <= 2'b00;
            m_axi_rlast       <= 1'b0;
            m_axi_rvalid      <= 1'b0;

            post_reset_ctr    <= 2'd0;
            master_phase      <= M_WR_ACTIVE;
            wr_beat_idx       <= 4'd0;
            rd_beat_idx       <= 4'd0;
            write_helper_done <= 1'b0;
            read_helper_done  <= 1'b0;

            aw_seen           <= 1'b0;
            w_fire_count      <= 4'd0;
            last_w_seen       <= 1'b0;
            b_hs_done         <= 1'b0;
            b_wait_ctr        <= 4'd0;
            ar_hs_done        <= 1'b0;
            r_fire_count      <= 4'd0;
            saw_delayed_b_window <= 1'b0;
        end else begin
            resetn        <= 1'b1;
            m_axi_awready <= 1'b1;
            m_axi_wready  <= 1'b1;
            m_axi_arready <= 1'b1;
            hreadyin      <= 1'b1;
            m_axi_bresp   <= 2'b00;
            m_axi_rresp   <= 2'b00;

            if (!post_reset_ok)
                post_reset_ctr <= post_reset_ctr + 2'd1;

            if (aw_fire)
                aw_seen <= 1'b1;

            if (w_fire) begin
                w_fire_count <= w_fire_count + 4'd1;
                if (m_axi_wlast) begin
                    last_w_seen <= 1'b1;
                    if (!b_hs_done && (b_wait_ctr == 4'd0))
                        b_wait_ctr <= B_DELAY_CYCLES[3:0];
                end
            end

            if (last_w_seen && !b_hs_done)
                saw_delayed_b_window <= 1'b1;

            if (!b_hs_done) begin
                if (m_axi_bvalid) begin
                    if (b_fire) begin
                        m_axi_bvalid <= 1'b0;
                        b_hs_done    <= 1'b1;
                    end else begin
                        m_axi_bvalid <= 1'b1;
                    end
                end else if (last_w_seen) begin
                    if (b_wait_ctr != 4'd0) begin
                        b_wait_ctr   <= b_wait_ctr - 4'd1;
                        m_axi_bvalid <= 1'b0;
                    end else begin
                        m_axi_bvalid <= 1'b1;
                    end
                end else begin
                    m_axi_bvalid <= 1'b0;
                end
            end else begin
                m_axi_bvalid <= 1'b0;
            end

            if (!ar_hs_done) begin
                if (ar_fire) begin
                    ar_hs_done  <= 1'b1;
                    m_axi_rvalid <= 1'b1;
                    m_axi_rdata  <= rd_data(4'd0);
                    m_axi_rlast  <= (NUM_BEATS == 1);
                end else begin
                    m_axi_rvalid <= 1'b0;
                    m_axi_rlast  <= 1'b0;
                end
            end else if (!read_helper_done) begin
                if (r_fire) begin
                    r_fire_count <= r_fire_count + 4'd1;
                    if (r_last_fire) begin
                        m_axi_rvalid <= 1'b0;
                        m_axi_rlast  <= 1'b0;
                    end else begin
                        m_axi_rvalid <= 1'b1;
                        m_axi_rdata  <= rd_data(r_fire_count + 4'd1);
                        m_axi_rlast  <= ((r_fire_count + 4'd1) == (NUM_BEATS - 1));
                    end
                end else begin
                    m_axi_rvalid <= m_axi_rvalid;
                    m_axi_rlast  <= m_axi_rlast;
                    m_axi_rdata  <= m_axi_rdata;
                end
            end else begin
                m_axi_rvalid <= 1'b0;
                m_axi_rlast  <= 1'b0;
            end

            case (master_phase)
                M_WR_ACTIVE: begin
                    if (hready) begin
                        if (wr_beat_idx == (NUM_BEATS - 1)) begin
                            haddr             <= 32'd0;
                            hburst            <= 3'd0;
                            hsize             <= 3'd0;
                            htrans            <= HTRANS_IDLE;
                            hwrite            <= 1'b0;
                            hwdata            <= 64'd0;
                            master_phase      <= M_WR_POST_WAIT;
                        end else begin
                            wr_beat_idx       <= wr_beat_idx + 4'd1;
                            haddr             <= WRITE_ADDR + ((wr_beat_idx + 4'd1) << 3);
                            hburst            <= HBURST_INCR8;
                            hsize             <= HSIZE_QWORD;
                            htrans            <= HTRANS_SEQ;
                            hwrite            <= 1'b1;
                            hwdata            <= wr_data(wr_beat_idx + 4'd1);
                        end
                    end
                end

                M_WR_POST_WAIT: begin
                    if (hready) begin
                        write_helper_done <= 1'b1;
                        haddr             <= READ_ADDR;
                        hburst            <= HBURST_INCR8;
                        hsize             <= HSIZE_QWORD;
                        htrans            <= HTRANS_NONSEQ;
                        hwrite            <= 1'b0;
                        hwdata            <= 64'd0;
                        rd_beat_idx       <= 4'd0;
                        master_phase      <= M_RD_ACTIVE;
                    end
                end

                M_RD_ACTIVE: begin
                    if (hready) begin
                        if (rd_beat_idx == (NUM_BEATS - 1)) begin
                            haddr             <= 32'd0;
                            hburst            <= 3'd0;
                            hsize             <= 3'd0;
                            htrans            <= HTRANS_IDLE;
                            hwrite            <= 1'b0;
                            hwdata            <= 64'd0;
                            read_helper_done  <= 1'b1;
                            master_phase      <= M_RD_DONE;
                        end else begin
                            rd_beat_idx       <= rd_beat_idx + 4'd1;
                            haddr             <= READ_ADDR + ((rd_beat_idx + 4'd1) << 3);
                            hburst            <= HBURST_INCR8;
                            hsize             <= HSIZE_QWORD;
                            htrans            <= HTRANS_SEQ;
                            hwrite            <= 1'b0;
                            hwdata            <= 64'd0;
                        end
                    end
                end

                default: begin
                    haddr        <= 32'd0;
                    hburst       <= 3'd0;
                    hsize        <= 3'd0;
                    htrans       <= HTRANS_IDLE;
                    hwrite       <= 1'b0;
                    hwdata       <= 64'd0;
                    master_phase <= M_RD_DONE;
                end
            endcase

            if (post_reset_ok) begin
                // Posted fixed-write contract:
                //  - write helper may finish before AXI B
                //  - read helper may already be active but stalled before B
                //  - what must NOT happen before B is:
                //      * AXI AR handshake
                //      * whole read sequence completion
                assert(!(last_w_seen
                      && !b_hs_done
                      && (ar_fire || read_helper_done)));

                // Sanity / anti-vacuity checkpoints for this scripted harness.
                // These are intentionally loose, but they ensure the proof is
                // actually exercising the intended write->delayed-B->read flow.
                assert((post_reset_ctr < 2'd3) || aw_seen || (master_phase == M_WR_ACTIVE));
                assert((post_reset_ctr < 2'd3) || (w_fire_count != 4'd0) || (master_phase == M_WR_ACTIVE));
                assert((post_reset_ctr < 2'd3) || !write_helper_done || last_w_seen);
                assert((post_reset_ctr < 2'd3) || !ar_hs_done || b_hs_done);
                assert((post_reset_ctr < 2'd3) || !read_helper_done || ar_hs_done);

                cover(saw_delayed_b_window && b_hs_done && ar_hs_done && read_helper_done);
                cover(aw_seen && (w_fire_count == NUM_BEATS[3:0]) && b_hs_done);
                cover(ar_hs_done && (r_fire_count == NUM_BEATS[3:0]) && read_helper_done);
            end
        end
    end
`endif

endmodule

`default_nettype wire
