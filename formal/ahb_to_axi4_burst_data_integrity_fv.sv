`default_nettype none

module ahb_to_axi4_burst_data_integrity_fv (
    input  wire         clk,
    input  wire         resetn,

    // AHB slave side into DUT
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

    // AXI master side out of DUT
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

    wire [3:0]  unused_awid;
    wire [3:0]  m_axi_awqos;
    wire [3:0]  unused_arid;
    wire [3:0]  m_axi_arqos;

    ahb_to_axi4_burst #(
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
        .RREADY         (m_axi_rready)
    );

`ifdef FORMAL
    reg f_past_valid;
    initial f_past_valid = 1'b0;
    always @(posedge clk)
        f_past_valid <= 1'b1;

    always @(posedge clk) begin
        if (!f_past_valid)
            assume(!resetn);
    end

    // Narrow environment: exactly one aligned 64-bit SINGLE write, no reads, no other transfers.
    reg started;
    reg addr_phase_seen;
    reg data_phase_seen;
    reg done;
    reg [31:0] cap_addr;
    reg [63:0] cap_data;

    reg saw_aw;
    reg saw_w;
    reg [31:0] awaddr_seen;
    reg [7:0]  awlen_seen;
    reg [2:0]  awsize_seen;
    reg [1:0]  awburst_seen;
    reg [63:0] wdata_seen;
    reg [7:0]  wstrb_seen;
    reg        wlast_seen;

    initial begin
        started = 1'b0;
        addr_phase_seen = 1'b0;
        data_phase_seen = 1'b0;
        done = 1'b0;
        cap_addr = 32'd0;
        cap_data = 64'd0;
        saw_aw = 1'b0;
        saw_w = 1'b0;
        awaddr_seen = 32'd0;
        awlen_seen = 8'd0;
        awsize_seen = 3'd0;
        awburst_seen = 2'd0;
        wdata_seen = 64'd0;
        wstrb_seen = 8'd0;
        wlast_seen = 1'b0;
    end

    wire ahb_addr_fire = resetn && hsel && hready && hreadyin && htrans[1] && hwrite;
    wire aw_fire = m_axi_awvalid && m_axi_awready;
    wire w_fire  = m_axi_wvalid  && m_axi_wready;

    always @(posedge clk) begin
        if (!resetn) begin
            started <= 1'b0;
            addr_phase_seen <= 1'b0;
            data_phase_seen <= 1'b0;
            done <= 1'b0;
            saw_aw <= 1'b0;
            saw_w <= 1'b0;
        end else begin
            // forbid read traffic in this narrow proof
            assume(!(hsel && hready && hreadyin && htrans[1] && !hwrite));
            assume(!m_axi_arvalid);
            assume(!m_axi_rvalid);
            assume(m_axi_bresp == 2'b00);

            if (!started) begin
                assume(ahb_addr_fire);
                assume(hburst == 3'b000);
                assume(hsize  == 3'b011);
                assume(haddr[2:0] == 3'b000);
                cap_addr <= haddr;
                started <= 1'b1;
                addr_phase_seen <= 1'b1;
            end else if (started && !data_phase_seen) begin
                assume(hready);
                assume(hreadyin);
                assume(!hsel || !htrans[1]);
                assume(!hwrite);
                cap_data <= hwdata;
                data_phase_seen <= 1'b1;
            end else if (!done) begin
                assume(!hsel || !htrans[1]);
                assume(!hwrite);

                if (aw_fire && !saw_aw) begin
                    saw_aw <= 1'b1;
                    awaddr_seen <= m_axi_awaddr;
                    awlen_seen <= m_axi_awlen;
                    awsize_seen <= m_axi_awsize;
                    awburst_seen <= m_axi_awburst;
                end

                if (w_fire && !saw_w) begin
                    saw_w <= 1'b1;
                    wdata_seen <= m_axi_wdata;
                    wstrb_seen <= m_axi_wstrb;
                    wlast_seen <= m_axi_wlast;
                end

                if (saw_aw && saw_w)
                    done <= 1'b1;
            end

            if (done) begin
                assert(awaddr_seen == cap_addr);
                assert(awlen_seen  == 8'd0);
                assert(awsize_seen == 3'd3);
                assert(awburst_seen == 2'b01);
                assert(wdata_seen == cap_data);
                assert(wstrb_seen == 8'hFF);
                assert(wlast_seen == 1'b1);
            end

            // Non-vacuity covers for this smoke property.
            cover(started && data_phase_seen);
            cover(aw_fire);
            cover(w_fire);
            cover(done);
        end
    end
`endif

endmodule

`default_nettype wire
