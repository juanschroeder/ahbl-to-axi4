`default_nettype none

module ahb_to_axi4_burst_wr_to_rd_handoff_fv (
    input  wire         clk,
    input  wire         resetn,

    input  wire         hsel,
    input  wire [31:0]  haddr,
    input  wire [1:0]   htrans,
    input  wire [2:0]   hsize,
    input  wire [2:0]   hburst,
    input  wire [3:0]   hprot,
    input  wire         hmastlock,
    input  wire         hwrite,
    input  wire [63:0]  hwdata,
    input  wire         hreadyin,

    output wire [63:0]  hrdata,
    output wire         hready,
    output wire         hresp,

    output wire [31:0]  m_axi_awaddr,
    output wire [7:0]   m_axi_awlen,
    output wire [2:0]   m_axi_awsize,
    output wire [1:0]   m_axi_awburst,
    output wire         m_axi_awlock,
    output wire [3:0]   m_axi_awcache,
    output wire [2:0]   m_axi_awprot,
    output wire         m_axi_awvalid,
    input  wire         m_axi_awready,

    output wire [63:0]  m_axi_wdata,
    output wire [7:0]   m_axi_wstrb,
    output wire         m_axi_wlast,
    output wire         m_axi_wvalid,
    input  wire         m_axi_wready,

    input  wire [1:0]   m_axi_bresp,
    input  wire         m_axi_bvalid,
    output wire         m_axi_bready,

    output wire [31:0]  m_axi_araddr,
    output wire [7:0]   m_axi_arlen,
    output wire [2:0]   m_axi_arsize,
    output wire [1:0]   m_axi_arburst,
    output wire         m_axi_arlock,
    output wire [3:0]   m_axi_arcache,
    output wire [2:0]   m_axi_arprot,
    output wire         m_axi_arvalid,
    input  wire         m_axi_arready,

    input  wire [63:0]  m_axi_rdata,
    input  wire [1:0]   m_axi_rresp,
    input  wire         m_axi_rlast,
    input  wire         m_axi_rvalid,
    output wire         m_axi_rready
);

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
    wire        tap_fix_ahb_fire;
    wire        tap_incr_rd;
    wire        tap_incr_wr_cont;

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
        .tap_fix_ahb_fire     (tap_fix_ahb_fire),
        .tap_incr_rd          (tap_incr_rd),
        .tap_incr_wr_cont     (tap_incr_wr_cont)
    );

`ifdef FORMAL
    reg f_past_valid;
    initial f_past_valid = 1'b0;

    localparam [4:0] FV_ST_IDLE           = 5'd0;
    localparam [4:0] FV_ST_WR_D           = 5'd2;
    localparam [4:0] FV_ST_WR_FIX_FLUSH   = 5'd3;
    localparam [4:0] FV_ST_WR_INCR_ACC    = 5'd4;
    localparam [4:0] FV_ST_WR_INCR_RESUME = 5'd6;
    localparam [4:0] FV_ST_WR_RESP        = 5'd8;
    localparam [4:0] FV_ST_WR_LAST_RESP   = 5'd9;
    localparam [4:0] FV_ST_RD_A           = 5'd11;

    localparam [1:0] FV_TRN_NONSEQ = 2'b10;
    localparam [2:0] FV_HB_INCR    = 3'b001;

    reg [1:0]  post_reset_ctr;
    wire       post_reset_ok = &post_reset_ctr;

    reg        prev_capture_any_read;
    reg [31:0] prev_cap_addr;
    reg [2:0]  prev_cap_size;
    reg [2:0]  prev_cap_burst;
    reg [3:0]  prev_cap_prot;
    reg        prev_cap_lock;

    reg        exp_pnd_read;
    reg [31:0] exp_addr;
    reg [2:0]  exp_size;
    reg [2:0]  exp_burst;
    reg [3:0]  exp_prot;
    reg        exp_lock;

    reg        prev_dispatch_pnd_read;
    reg [31:0] prev_dispatch_addr;
    reg [2:0]  prev_dispatch_size;
    reg [2:0]  prev_dispatch_burst;
    reg [3:0]  prev_dispatch_prot;
    reg        prev_dispatch_lock;

    wire capture_wr_d_read = (tap_state == FV_ST_WR_D)
                          && tap_fix_ahb_fire
                          && (tap_beat_cnt <= 8'd1)
                          && !tap_pnd_valid
                          && hsel
                          && hreadyin
                          && (htrans == FV_TRN_NONSEQ)
                          && !hwrite;

    wire capture_wr_fix_flush_read = (tap_state == FV_ST_WR_FIX_FLUSH)
                                  && !tap_pnd_valid
                                  && hsel
                                  && hreadyin
                                  && (htrans == FV_TRN_NONSEQ)
                                  && !hwrite;

    wire capture_wr_resp_read = (tap_state == FV_ST_WR_RESP)
                             && !tap_incr_wr_cont
                             && !tap_pnd_valid
                             && hsel
                             && hreadyin
                             && (htrans == FV_TRN_NONSEQ)
                             && !hwrite;

    wire capture_wr_last_resp_read = (tap_state == FV_ST_WR_LAST_RESP)
                                  && !tap_pnd_valid
                                  && hsel
                                  && hreadyin
                                  && (htrans == FV_TRN_NONSEQ)
                                  && !hwrite;

    wire capture_wr_incr_acc_read = (tap_state == FV_ST_WR_INCR_ACC)
                                 && hsel
                                 && hreadyin
                                 && (htrans == FV_TRN_NONSEQ)
                                 && !hwrite;

    wire capture_wr_incr_resume_read = (tap_state == FV_ST_WR_INCR_RESUME)
                                    && hsel
                                    && hreadyin
                                    && (htrans == FV_TRN_NONSEQ)
                                    && !hwrite;

    wire capture_any_read = capture_wr_d_read
                         || capture_wr_fix_flush_read
                         || capture_wr_resp_read
                         || capture_wr_last_resp_read
                         || capture_wr_incr_acc_read
                         || capture_wr_incr_resume_read;

    wire tracked_pnd_read = prev_capture_any_read || exp_pnd_read;
    wire [31:0] tracked_addr  = prev_capture_any_read ? prev_cap_addr  : exp_addr;
    wire [2:0]  tracked_size  = prev_capture_any_read ? prev_cap_size  : exp_size;
    wire [2:0]  tracked_burst = prev_capture_any_read ? prev_cap_burst : exp_burst;
    wire [3:0]  tracked_prot  = prev_capture_any_read ? prev_cap_prot  : exp_prot;
    wire        tracked_lock  = prev_capture_any_read ? prev_cap_lock  : exp_lock;

    wire live_same_tracked_read = hsel
                                && hreadyin
                                && (htrans == FV_TRN_NONSEQ)
                                && !hwrite
                                && (haddr     == tracked_addr)
                                && (hsize     == tracked_size)
                                && (hburst    == tracked_burst)
                                && (hprot     == tracked_prot)
                                && (hmastlock == tracked_lock);

    always @(posedge clk) begin
        f_past_valid <= 1'b1;

        if (!f_past_valid) begin
            assume(!resetn);
        end else if ($past(resetn)) begin
            assume(resetn);
        end

        if (!resetn) begin
            post_reset_ctr         <= 2'd0;
            prev_capture_any_read  <= 1'b0;
            prev_cap_addr          <= 32'd0;
            prev_cap_size          <= 3'd0;
            prev_cap_burst         <= 3'd0;
            prev_cap_prot          <= 4'd0;
            prev_cap_lock          <= 1'b0;
            exp_pnd_read           <= 1'b0;
            exp_addr               <= 32'd0;
            exp_size               <= 3'd0;
            exp_burst              <= 3'd0;
            exp_prot               <= 4'd0;
            exp_lock               <= 1'b0;
            prev_dispatch_pnd_read <= 1'b0;
            prev_dispatch_addr     <= 32'd0;
            prev_dispatch_size     <= 3'd0;
            prev_dispatch_burst    <= 3'd0;
            prev_dispatch_prot     <= 4'd0;
            prev_dispatch_lock     <= 1'b0;
        end else begin
            if (!post_reset_ok)
                post_reset_ctr <= post_reset_ctr + 2'd1;

            prev_capture_any_read  <= 1'b0;
            prev_dispatch_pnd_read <= 1'b0;

            if (post_reset_ok && tracked_pnd_read) begin
                assume(!hwrite || !hsel || !hreadyin || (htrans != FV_TRN_NONSEQ));
                assume(!hsel || !hreadyin || (htrans != FV_TRN_NONSEQ) || live_same_tracked_read);
            end

            if (!post_reset_ok) begin
                exp_pnd_read <= 1'b0;
            end else begin
                if (prev_capture_any_read) begin
                    assert(tap_pnd_valid);
                    assert(!tap_pnd_write);
                    assert(tap_pnd_addr  == prev_cap_addr);
                    assert(tap_pnd_size  == prev_cap_size);
                    assert(tap_pnd_burst == prev_cap_burst);
                    assert(tap_pnd_prot  == prev_cap_prot);
                    assert(tap_pnd_lock  == prev_cap_lock);
                    assert(!tap_pnd_wfirst_valid);
                end

                if (exp_pnd_read) begin
                    if ((tap_state == FV_ST_IDLE) && tap_pnd_valid && !tap_pnd_write) begin
                        assert(tap_pnd_addr  == exp_addr);
                        assert(tap_pnd_size  == exp_size);
                        assert(tap_pnd_burst == exp_burst);
                        assert(tap_pnd_prot  == exp_prot);
                        assert(tap_pnd_lock  == exp_lock);

                        prev_dispatch_pnd_read <= 1'b1;
                        prev_dispatch_addr     <= exp_addr;
                        prev_dispatch_size     <= exp_size;
                        prev_dispatch_burst    <= exp_burst;
                        prev_dispatch_prot     <= exp_prot;
                        prev_dispatch_lock     <= exp_lock;

                        exp_pnd_read <= 1'b0;
                    end else begin
                        assert(tap_pnd_valid);
                        assert(!tap_pnd_write);
                        assert(tap_pnd_addr  == exp_addr);
                        assert(tap_pnd_size  == exp_size);
                        assert(tap_pnd_burst == exp_burst);
                        assert(tap_pnd_prot  == exp_prot);
                        assert(tap_pnd_lock  == exp_lock);
                        assert(!tap_pnd_wfirst_valid);
                    end
                end

                if (capture_any_read) begin
                    prev_capture_any_read <= 1'b1;
                    prev_cap_addr         <= haddr;
                    prev_cap_size         <= hsize;
                    prev_cap_burst        <= hburst;
                    prev_cap_prot         <= hprot;
                    prev_cap_lock         <= hmastlock;

                    exp_pnd_read          <= 1'b1;
                    exp_addr              <= haddr;
                    exp_size              <= hsize;
                    exp_burst             <= hburst;
                    exp_prot              <= hprot;
                    exp_lock              <= hmastlock;
                end

                if (prev_dispatch_pnd_read) begin
                    assert(tap_state     == FV_ST_RD_A);
                    assert(tap_ap_addr   == prev_dispatch_addr);
                    assert(tap_ap_size   == prev_dispatch_size);
                    assert(tap_ap_prot   == prev_dispatch_prot);
                    assert(tap_ap_lock   == prev_dispatch_lock);
                    assert(!tap_ar_done);
                    assert(m_axi_arvalid);
                    assert(m_axi_araddr  == prev_dispatch_addr);
                    assert(m_axi_arsize  == prev_dispatch_size);
                    assert(m_axi_arlock  == prev_dispatch_lock);
                    if (prev_dispatch_burst == FV_HB_INCR)
                        assert(tap_incr_rd);
                    else
                        assert(!tap_incr_rd);
                end
            end
        end
    end
`endif

endmodule

`default_nettype wire
