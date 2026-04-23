`default_nettype none

module ahb_to_axi4_burst_wr_buffer_capture_fv (
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

    wire [4:0]   tap_state;
    wire [4:0]   tap_acc_cnt;
    wire [3:0]   tap_flush_ptr;
    wire         tap_aw_sent;
    wire         tap_fix_ahb_fire;
    wire         tap_incr_wr_cont;
    wire         tap_pnd_wfirst_valid;
    wire [63:0]  tap_pnd_wfirst_data;
    wire [7:0]   tap_pnd_wfirst_strb;
    wire [7:0]   tap_ahb_wstrb_d;
    wire [7:0]   tap_busy_wstrb;
    wire [1023:0] tap_ibuf_data_flat;
    wire [127:0]  tap_ibuf_strb_flat;

    function automatic [63:0] tap_ibuf_data_at(input [3:0] idx);
        tap_ibuf_data_at = tap_ibuf_data_flat[idx*64 +: 64];
    endfunction

    function automatic [7:0] tap_ibuf_strb_at(input [3:0] idx);
        tap_ibuf_strb_at = tap_ibuf_strb_flat[idx*8 +: 8];
    endfunction

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
        .tap_pnd_wfirst_valid (tap_pnd_wfirst_valid),
        .tap_pnd_wfirst_data  (tap_pnd_wfirst_data),
        .tap_pnd_wfirst_strb  (tap_pnd_wfirst_strb),
        .tap_ahb_wstrb_d      (tap_ahb_wstrb_d),
        .tap_acc_cnt          (tap_acc_cnt),
        .tap_flush_ptr        (tap_flush_ptr),
        .tap_aw_sent          (tap_aw_sent),
        .tap_fix_ahb_fire     (tap_fix_ahb_fire),
        .tap_incr_wr_cont     (tap_incr_wr_cont),
        .tap_busy_wstrb       (tap_busy_wstrb),
        .tap_ibuf_data_flat   (tap_ibuf_data_flat),
        .tap_ibuf_strb_flat   (tap_ibuf_strb_flat)
    );

`ifdef FORMAL
    reg f_past_valid;
    initial f_past_valid = 1'b0;

    localparam [4:0] FV_ST_WR_D              = 5'd2;
    localparam [4:0] FV_ST_WR_INCR_ACC       = 5'd4;
    localparam [4:0] FV_ST_WR_INCR_POST_BUSY = 5'd5;
    localparam [4:0] FV_ST_WR_INCR_FLUSH     = 5'd7;

    localparam [1:0] FV_TRN_BUSY = 2'b01;
    localparam [1:0] FV_TRN_SEQ  = 2'b11;

    reg [1:0]  post_reset_ctr;
    wire       post_reset_ok = &post_reset_ctr;

    reg        prev_fix_live_capture;
    reg        prev_fix_pending_capture;
    reg        prev_incr_acc_busy_capture;
    reg        prev_incr_acc_seq_capture;
    reg        prev_incr_acc_last_capture;
    reg        prev_post_busy_capture;

    reg [4:0]  prev_acc_cnt;
    reg [3:0]  prev_idx;
    reg [63:0] prev_hwdata;
    reg [7:0]  prev_ahb_wstrb_d;
    reg [7:0]  prev_busy_wstrb;
    reg [63:0] prev_pnd_wfirst_data;
    reg [7:0]  prev_pnd_wfirst_strb;

    always @(posedge clk) begin
        f_past_valid <= 1'b1;

        if (!f_past_valid) begin
            assume(!resetn);
        end else if ($past(resetn)) begin
            assume(resetn);
        end

        if (!resetn) begin
            post_reset_ctr            <= 2'd0;
            prev_fix_live_capture     <= 1'b0;
            prev_fix_pending_capture  <= 1'b0;
            prev_incr_acc_busy_capture<= 1'b0;
            prev_incr_acc_seq_capture <= 1'b0;
            prev_incr_acc_last_capture<= 1'b0;
            prev_post_busy_capture    <= 1'b0;
            prev_acc_cnt              <= 5'd0;
            prev_idx                  <= 4'd0;
            prev_hwdata               <= 64'd0;
            prev_ahb_wstrb_d          <= 8'd0;
            prev_busy_wstrb           <= 8'd0;
            prev_pnd_wfirst_data      <= 64'd0;
            prev_pnd_wfirst_strb      <= 8'd0;
        end else begin
            if (!post_reset_ok)
                post_reset_ctr <= post_reset_ctr + 2'd1;

            if (post_reset_ok) begin
                if (prev_fix_live_capture) begin
                    assert(tap_ibuf_data_at(prev_idx) == prev_hwdata);
                    assert(tap_ibuf_strb_at(prev_idx) == prev_ahb_wstrb_d);
                    assert(tap_acc_cnt == prev_acc_cnt + 5'd1);
                end

                if (prev_fix_pending_capture) begin
                    assert(tap_ibuf_data_at(prev_idx) == prev_pnd_wfirst_data);
                    assert(tap_ibuf_strb_at(prev_idx) == prev_pnd_wfirst_strb);
                    assert(tap_acc_cnt == prev_acc_cnt + 5'd1);
                end

                if (prev_incr_acc_busy_capture) begin
                    assert(tap_ibuf_data_at(prev_idx) == prev_hwdata);
                    assert(tap_ibuf_strb_at(prev_idx) == prev_ahb_wstrb_d);
                    assert(tap_acc_cnt == prev_acc_cnt + 5'd1);
                end

                if (prev_incr_acc_seq_capture) begin
                    assert(tap_ibuf_data_at(prev_idx) == prev_hwdata);
                    assert(tap_ibuf_strb_at(prev_idx) == prev_ahb_wstrb_d);
                    assert(tap_acc_cnt == prev_acc_cnt + 5'd1);
                end

                if (prev_incr_acc_last_capture) begin
                    assert(tap_ibuf_data_at(prev_idx) == prev_hwdata);
                    assert(tap_ibuf_strb_at(prev_idx) == prev_ahb_wstrb_d);
                    assert(tap_acc_cnt == prev_acc_cnt + 5'd1);
                end

                if (prev_post_busy_capture) begin
                    assert(tap_ibuf_data_at(prev_idx) == prev_hwdata);
                    assert(tap_ibuf_strb_at(prev_idx) == prev_busy_wstrb);
                    assert(tap_acc_cnt == prev_acc_cnt + 5'd1);
                end

                // Full-buffer transitions on incrementing writes.
                if (prev_incr_acc_busy_capture && (prev_acc_cnt == 5'd15)) begin
                    assert(tap_state == FV_ST_WR_INCR_FLUSH);
                    assert(tap_flush_ptr == 4'd0);
                    assert(!tap_aw_sent);
                    assert(tap_incr_wr_cont);
                end

                if (prev_incr_acc_seq_capture && (prev_acc_cnt == 5'd15)) begin
                    assert(tap_state == FV_ST_WR_INCR_FLUSH);
                    assert(tap_flush_ptr == 4'd0);
                    assert(!tap_aw_sent);
                    assert(tap_incr_wr_cont);
                end

                if (prev_post_busy_capture && (prev_acc_cnt == 5'd15)) begin
                    assert(tap_state == FV_ST_WR_INCR_FLUSH);
                    assert(tap_flush_ptr == 4'd0);
                    assert(!tap_aw_sent);
                    assert(tap_incr_wr_cont);
                end

                prev_fix_live_capture      <= (tap_state == FV_ST_WR_D)
                                           && tap_fix_ahb_fire
                                           && !tap_pnd_wfirst_valid;
                prev_fix_pending_capture   <= (tap_state == FV_ST_WR_D)
                                           && tap_fix_ahb_fire
                                           && tap_pnd_wfirst_valid;
                prev_incr_acc_busy_capture <= (tap_state == FV_ST_WR_INCR_ACC)
                                           && (htrans == FV_TRN_BUSY);
                prev_incr_acc_seq_capture  <= (tap_state == FV_ST_WR_INCR_ACC)
                                           && (htrans == FV_TRN_SEQ);
                prev_incr_acc_last_capture <= (tap_state == FV_ST_WR_INCR_ACC)
                                           && (htrans != FV_TRN_BUSY)
                                           && (htrans != FV_TRN_SEQ);
                prev_post_busy_capture     <= (tap_state == FV_ST_WR_INCR_POST_BUSY)
                                           && (htrans == FV_TRN_BUSY);

                prev_acc_cnt          <= tap_acc_cnt;
                prev_idx              <= tap_acc_cnt[3:0];
                prev_hwdata           <= hwdata;
                prev_ahb_wstrb_d      <= tap_ahb_wstrb_d;
                prev_busy_wstrb       <= tap_busy_wstrb;
                prev_pnd_wfirst_data  <= tap_pnd_wfirst_data;
                prev_pnd_wfirst_strb  <= tap_pnd_wfirst_strb;
            end else begin
                prev_fix_live_capture      <= 1'b0;
                prev_fix_pending_capture   <= 1'b0;
                prev_incr_acc_busy_capture <= 1'b0;
                prev_incr_acc_seq_capture  <= 1'b0;
                prev_incr_acc_last_capture <= 1'b0;
                prev_post_busy_capture     <= 1'b0;
                prev_acc_cnt               <= 5'd0;
                prev_idx                   <= 4'd0;
                prev_hwdata                <= 64'd0;
                prev_ahb_wstrb_d           <= 8'd0;
                prev_busy_wstrb            <= 8'd0;
                prev_pnd_wfirst_data       <= 64'd0;
                prev_pnd_wfirst_strb       <= 8'd0;
            end
        end
    end
`endif

endmodule

`default_nettype wire
