`default_nettype none

module ahb_to_axi4_burst_rd_buf_capture_delivery_fv (
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
    wire [7:0]  tap_beat_cnt;
    wire        tap_ar_done;
    wire        tap_rd_buf_valid;
    wire [63:0] tap_rd_buf_data;
    wire [1:0]  tap_rd_buf_resp;
    wire        tap_rd_d_entry;
    wire        tap_incr_rd;
    wire        tap_incr_rd_busy;
    wire        tap_rd_q_interleave;

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
        .tap_beat_cnt         (tap_beat_cnt),
        .tap_ar_done          (tap_ar_done),
        .tap_rd_buf_valid     (tap_rd_buf_valid),
        .tap_rd_buf_data      (tap_rd_buf_data),
        .tap_rd_buf_resp      (tap_rd_buf_resp),
        .tap_rd_d_entry       (tap_rd_d_entry),
        .tap_incr_rd          (tap_incr_rd),
        .tap_incr_rd_busy     (tap_incr_rd_busy),
        .tap_rd_q_interleave  (tap_rd_q_interleave)
    );

`ifdef FORMAL
    reg f_past_valid;
    initial f_past_valid = 1'b0;

    localparam [4:0] FV_ST_RD_A     = 5'd11;
    localparam [4:0] FV_ST_RD_D     = 5'd12;
    localparam [4:0] FV_ST_RD_ERR   = 5'd14;
    localparam [4:0] FV_ST_RD_DRAIN = 5'd15;

    reg        prev_raw_r_capture;
    reg [63:0] prev_rdata;
    reg [1:0]  prev_rresp;
    reg        prev_rd_drain_with_buf;
    reg [1:0]  post_reset_ctr;
    wire       post_reset_ok = &post_reset_ctr;

    wire capture_fire = (tap_state == FV_ST_RD_D)
                      && tap_ar_done
                      && !tap_rd_d_entry
                      && !tap_rd_buf_valid
                      && m_axi_rvalid
                      && m_axi_rready;

    wire rd_normal_deliver_now = (tap_state == FV_ST_RD_D)
                              && tap_rd_buf_valid
                              && !tap_rd_buf_resp[1]
                              && !(tap_incr_rd && tap_incr_rd_busy)
                              && !(tap_incr_rd && (tap_beat_cnt != 8'd0) && tap_rd_q_interleave);

    always @(posedge clk) begin
        f_past_valid <= 1'b1;

        if (!f_past_valid) begin
            assume(!resetn);
        end else if ($past(resetn)) begin
            assume(resetn);
        end

        if (!resetn) begin
            post_reset_ctr         <= 2'd0;
            prev_raw_r_capture     <= 1'b0;
            prev_rdata             <= 64'd0;
            prev_rresp             <= 2'b00;
            prev_rd_drain_with_buf <= 1'b0;
        end else begin
            if (!post_reset_ok)
                post_reset_ctr <= post_reset_ctr + 2'd1;

            if (post_reset_ok) begin
                if (prev_raw_r_capture) begin
                    assert(tap_rd_buf_valid);
                    assert(tap_rd_buf_data == prev_rdata);
                    assert(tap_rd_buf_resp == prev_rresp);
                end

                if (tap_state == FV_ST_RD_A)
                    assert(!tap_rd_buf_valid);

                if (rd_normal_deliver_now) begin
                    assert(hready);
                    assert(!hresp);
                    assert(hrdata == tap_rd_buf_data);
                end

                if ((tap_state == FV_ST_RD_D) && tap_rd_buf_valid && tap_rd_buf_resp[1]) begin
                    assert(!hready);
                end

                if (tap_state == FV_ST_RD_ERR) begin
                    assert(hready);
                    assert(hresp);
                end

                if (prev_rd_drain_with_buf)
                    assert(!tap_rd_buf_valid);

                cover(capture_fire);
                cover(rd_normal_deliver_now);
                cover((tap_state == FV_ST_RD_D) && tap_rd_buf_valid && tap_rd_buf_resp[1]);
                cover(tap_state == FV_ST_RD_A);

                prev_raw_r_capture     <= capture_fire;
                prev_rdata             <= m_axi_rdata;
                prev_rresp             <= m_axi_rresp;
                prev_rd_drain_with_buf <= (tap_state == FV_ST_RD_DRAIN) && tap_rd_buf_valid;
            end else begin
                prev_raw_r_capture     <= 1'b0;
                prev_rdata             <= 64'd0;
                prev_rresp             <= 2'b00;
                prev_rd_drain_with_buf <= 1'b0;
            end
        end
    end
`endif

endmodule

`default_nettype wire
