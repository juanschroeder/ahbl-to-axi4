`default_nettype none

module ahb_to_axi4_burst_wr_pending_firstbeat_fv (
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
    wire         tap_pnd_valid;
    wire         tap_pnd_write;
    wire [2:0]   tap_pnd_burst;
    wire [31:0]  tap_pnd_addr;
    wire [2:0]   tap_pnd_size;
    wire [3:0]   tap_pnd_prot;
    wire         tap_pnd_lock;
    wire         tap_pnd_wfirst_valid;
    wire [63:0]  tap_pnd_wfirst_data;
    wire [7:0]   tap_pnd_wfirst_strb;
    wire         tap_pnd_wpipe_mode;
    wire         tap_pnd_wfirst_capture_ok;
    wire         tap_ahb_wphase_valid;
    wire [31:0]  tap_ahb_waddr_d;
    wire [2:0]   tap_ahb_wsize_d;
    wire [7:0]   tap_ahb_wstrb_d;
    wire         tap_pnd_same_live_write_addrphase;
    wire         tap_fix_ahb_fire;
    wire [4:0]   tap_acc_cnt;
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
        .tap_pnd_wpipe_mode   (tap_pnd_wpipe_mode),
        .tap_pnd_wfirst_capture_ok (tap_pnd_wfirst_capture_ok),
        .tap_ahb_wphase_valid (tap_ahb_wphase_valid),
        .tap_ahb_waddr_d      (tap_ahb_waddr_d),
        .tap_ahb_wsize_d      (tap_ahb_wsize_d),
        .tap_ahb_wstrb_d      (tap_ahb_wstrb_d),
        .tap_pnd_same_live_write_addrphase (tap_pnd_same_live_write_addrphase),
        .tap_fix_ahb_fire     (tap_fix_ahb_fire),
        .tap_acc_cnt          (tap_acc_cnt),
        .tap_ibuf_data_flat   (tap_ibuf_data_flat),
        .tap_ibuf_strb_flat   (tap_ibuf_strb_flat)
    );

`ifdef FORMAL
    reg f_past_valid;
    initial f_past_valid = 1'b0;

    localparam [4:0] FV_ST_WR_PND_ALIGN = 5'd1;
    localparam [4:0] FV_ST_WR_D         = 5'd2;
    localparam [4:0] FV_ST_WR_FIX_FLUSH = 5'd3;
    localparam [4:0] FV_ST_RD_A         = 5'd11;
    localparam [4:0] FV_ST_RD_D         = 5'd12;
    localparam [4:0] FV_ST_RD_INCR_WAIT = 5'd13;
    localparam [4:0] FV_ST_RD_ERR       = 5'd14;
    localparam [4:0] FV_ST_RD_DRAIN     = 5'd15;
    localparam [4:0] FV_ST_RD_FENCE     = 5'd16;
    localparam [1:0] FV_TRN_NONSEQ      = 2'b10;
    localparam [2:0] FV_HB_INCR         = 3'b001;

    reg        prev_flush_capture_fire;
    reg        prev_rd_d_capture_fire;
    reg        prev_rd_fence_capture_fire;
    reg        prev_consume_pending_first;

    reg [63:0] prev_hwdata;
    reg [7:0]  prev_ahb_wstrb_d;
    reg [63:0] prev_pnd_wfirst_data;
    reg [7:0]  prev_pnd_wfirst_strb;
    reg [3:0]  prev_acc_cnt_idx;
    reg [1:0]  post_reset_ctr;
    wire       post_reset_ok = &post_reset_ctr;

    wire live_same_fixed_pending_write = hsel
                                      && hreadyin
                                      && (htrans == FV_TRN_NONSEQ)
                                      && hwrite
                                      && (haddr     == tap_pnd_addr)
                                      && (hsize     == tap_pnd_size)
                                      && (hburst    == tap_pnd_burst)
                                      && (hprot     == tap_pnd_prot)
                                      && (hmastlock == tap_pnd_lock);

    always @(posedge clk) begin
        f_past_valid <= 1'b1;

        if (!f_past_valid) begin
            assume(!resetn);
        end else if ($past(resetn)) begin
            assume(resetn);
        end

        if (!resetn) begin
            post_reset_ctr             <= 2'd0;
            prev_flush_capture_fire    <= 1'b0;
            prev_rd_d_capture_fire     <= 1'b0;
            prev_rd_fence_capture_fire <= 1'b0;
            prev_consume_pending_first <= 1'b0;
            prev_hwdata                <= 64'd0;
            prev_ahb_wstrb_d           <= 8'd0;
            prev_pnd_wfirst_data       <= 64'd0;
            prev_pnd_wfirst_strb       <= 8'd0;
            prev_acc_cnt_idx           <= 4'd0;
        end else begin
            if (!post_reset_ok)
                post_reset_ctr <= post_reset_ctr + 2'd1;

            if (post_reset_ok
                && tap_pnd_valid
                && tap_pnd_write
                && (tap_pnd_burst != FV_HB_INCR)
                && !tap_pnd_wfirst_valid) begin
                assume(!hsel || !hreadyin || (htrans != FV_TRN_NONSEQ) || live_same_fixed_pending_write);
            end

            if (post_reset_ok) begin
                if (prev_flush_capture_fire) begin
                    assert(tap_pnd_wfirst_valid);
                    assert(tap_pnd_wfirst_data == prev_hwdata);
                    assert(tap_pnd_wfirst_strb == prev_ahb_wstrb_d);
                    assert(!tap_pnd_wpipe_mode);
                end

                if (prev_rd_d_capture_fire) begin
                    assert((tap_state == FV_ST_RD_A)
                        || (tap_state == FV_ST_RD_D)
                        || (tap_state == FV_ST_RD_INCR_WAIT)
                        || (tap_state == FV_ST_RD_ERR)
                        || (tap_state == FV_ST_RD_DRAIN)
                        || (tap_state == FV_ST_RD_FENCE));
                    if (tap_pnd_wfirst_valid) begin
                        assert(tap_pnd_wfirst_data == prev_hwdata);
                        assert(tap_pnd_wfirst_strb == prev_ahb_wstrb_d);
                    end else begin
                        assert((tap_state == FV_ST_RD_DRAIN) || (tap_state == FV_ST_RD_FENCE));
                    end
                end

                if (prev_rd_fence_capture_fire) begin
                    assert(tap_pnd_wfirst_valid);
                    assert(tap_pnd_wfirst_data == prev_hwdata);
                    assert(tap_pnd_wfirst_strb == prev_ahb_wstrb_d);
                end

                if (prev_consume_pending_first) begin
                    assert(tap_ibuf_data_at(prev_acc_cnt_idx) == prev_pnd_wfirst_data);
                    assert(tap_ibuf_strb_at(prev_acc_cnt_idx) == prev_pnd_wfirst_strb);
                end

                cover((tap_state == FV_ST_WR_FIX_FLUSH)
                   && tap_pnd_valid && tap_pnd_write && (tap_pnd_burst != 3'b001)
                   && !tap_pnd_wfirst_valid && tap_pnd_wfirst_capture_ok);
                cover((tap_state == FV_ST_RD_D)
                   && tap_pnd_valid && tap_pnd_write && (tap_pnd_burst != 3'b001)
                   && !tap_pnd_wfirst_valid && tap_ahb_wphase_valid
                   && (tap_ahb_waddr_d == tap_pnd_addr)
                   && (tap_ahb_wsize_d == tap_pnd_size)
                   && !tap_pnd_same_live_write_addrphase);
                cover((tap_state == FV_ST_RD_FENCE)
                   && tap_pnd_valid && tap_pnd_write && (tap_pnd_burst != 3'b001)
                   && !tap_pnd_wfirst_valid && tap_ahb_wphase_valid
                   && (tap_ahb_waddr_d == tap_pnd_addr)
                   && (tap_ahb_wsize_d == tap_pnd_size)
                   && !tap_pnd_same_live_write_addrphase);
                cover((tap_state == FV_ST_WR_D) && tap_fix_ahb_fire && tap_pnd_wfirst_valid);

                prev_flush_capture_fire <= (tap_state == FV_ST_WR_FIX_FLUSH)
                                        && tap_pnd_valid && tap_pnd_write && (tap_pnd_burst != 3'b001)
                                        && !tap_pnd_wfirst_valid && tap_pnd_wfirst_capture_ok;
                prev_rd_d_capture_fire <= (tap_state == FV_ST_RD_D)
                                       && tap_pnd_valid && tap_pnd_write && (tap_pnd_burst != 3'b001)
                                       && !tap_pnd_wfirst_valid && tap_ahb_wphase_valid
                                       && (tap_ahb_waddr_d == tap_pnd_addr)
                                       && (tap_ahb_wsize_d == tap_pnd_size)
                                       && !tap_pnd_same_live_write_addrphase;
                prev_rd_fence_capture_fire <= (tap_state == FV_ST_RD_FENCE)
                                           && tap_pnd_valid && tap_pnd_write && (tap_pnd_burst != 3'b001)
                                           && !tap_pnd_wfirst_valid && tap_ahb_wphase_valid
                                           && (tap_ahb_waddr_d == tap_pnd_addr)
                                           && (tap_ahb_wsize_d == tap_pnd_size)
                                           && !tap_pnd_same_live_write_addrphase;
                prev_consume_pending_first <= (tap_state == FV_ST_WR_D) && tap_fix_ahb_fire && tap_pnd_wfirst_valid;

                prev_hwdata          <= hwdata;
                prev_ahb_wstrb_d     <= tap_ahb_wstrb_d;
                prev_pnd_wfirst_data <= tap_pnd_wfirst_data;
                prev_pnd_wfirst_strb <= tap_pnd_wfirst_strb;
                prev_acc_cnt_idx     <= tap_acc_cnt[3:0];
            end else begin
                prev_flush_capture_fire    <= 1'b0;
                prev_rd_d_capture_fire     <= 1'b0;
                prev_rd_fence_capture_fire <= 1'b0;
                prev_consume_pending_first <= 1'b0;
                prev_hwdata                <= 64'd0;
                prev_ahb_wstrb_d           <= 8'd0;
                prev_pnd_wfirst_data       <= 64'd0;
                prev_pnd_wfirst_strb       <= 8'd0;
                prev_acc_cnt_idx           <= 4'd0;
            end
        end
    end
`endif

endmodule

`default_nettype wire
