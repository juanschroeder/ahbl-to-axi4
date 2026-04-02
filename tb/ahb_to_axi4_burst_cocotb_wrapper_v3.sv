module ahb_to_axi4_burst_cocotb_wrapper_v3 #(
  parameter int unsigned AW = 32,
  parameter int unsigned DW = 64,
  parameter int unsigned IW = 4,
  parameter int unsigned MAX_INCR_BEATS = 16
) (
  input  logic               clk,
  input  logic               resetn,

  // Standardized cocotb-facing AHB-Lite-ish interface
  input logic                hsel,
  input logic                hreadyin,
  input  logic [AW-1:0]      haddr,
  input  logic [2:0]         hburst,
  input  logic               hmastlock,
  input  logic [3:0]         hprot,
  input  logic [2:0]         hsize,
  input  logic [1:0]         htrans,
  input  logic [DW-1:0]      hwdata,
  input  logic               hwrite,
  output logic [DW-1:0]      hrdata,
  output logic               hready,
  output logic               hresp,
  output logic               hselect,

  // cocotbext-axi-facing AXI master interface
  output logic [IW-1:0]      m_axi_awid,
  output logic [AW-1:0]      m_axi_awaddr,
  output logic [7:0]         m_axi_awlen,
  output logic [2:0]         m_axi_awsize,
  output logic [1:0]         m_axi_awburst,
  output logic               m_axi_awlock,
  output logic [3:0]         m_axi_awcache,
  output logic [2:0]         m_axi_awprot,
  output logic [3:0]         m_axi_awqos,
  output logic               m_axi_awvalid,
  input  logic               m_axi_awready,

  output logic [DW-1:0]      m_axi_wdata,
  output logic [DW/8-1:0]    m_axi_wstrb,
  output logic               m_axi_wlast,
  output logic               m_axi_wvalid,
  input  logic               m_axi_wready,

  input  logic [IW-1:0]      m_axi_bid,
  input  logic [1:0]         m_axi_bresp,
  input  logic               m_axi_bvalid,
  output logic               m_axi_bready,

  output logic [IW-1:0]      m_axi_arid,
  output logic [AW-1:0]      m_axi_araddr,
  output logic [7:0]         m_axi_arlen,
  output logic [2:0]         m_axi_arsize,
  output logic [1:0]         m_axi_arburst,
  output logic               m_axi_arlock,
  output logic [3:0]         m_axi_arcache,
  output logic [2:0]         m_axi_arprot,
  output logic [3:0]         m_axi_arqos,
  output logic               m_axi_arvalid,
  input  logic               m_axi_arready,

  input  logic [IW-1:0]      m_axi_rid,
  input  logic [DW-1:0]      m_axi_rdata,
  input  logic [1:0]         m_axi_rresp,
  input  logic               m_axi_rlast,
  input  logic               m_axi_rvalid,
  output logic               m_axi_rready
);

  localparam int unsigned STRB_W = DW/8;
  localparam int unsigned ADDR_LSB_W = (STRB_W > 1) ? $clog2(STRB_W) : 1;

  // Test ID, to add see in the waveform
  logic [31:0] tb_test_id;

  //logic [STRB_W-1:0] hwstrb;

  // AHB write address/control phase must be carried into the following data phase.
  logic              wr_phase_d;
  logic [AW-1:0]     haddr_d;
  logic [2:0]        hsize_d;

//   function automatic [STRB_W-1:0] gen_hwstrb(
//     input logic [AW-1:0] addr,
//     input logic [2:0]    size
//   );
//     int unsigned nbytes;
//     logic [STRB_W-1:0] mask;
//     begin
//       nbytes = 1 << size;

//       if (nbytes >= STRB_W) begin
//         gen_hwstrb = {STRB_W{1'b1}};
//       end else begin
//         mask = ({STRB_W{1'b1}} >> (STRB_W - nbytes));
//         gen_hwstrb = mask << addr[ADDR_LSB_W-1:0];
//       end
//     end
//   endfunction

  always_ff @(posedge clk) begin
    if (!resetn) begin
      wr_phase_d <= 1'b0;
      haddr_d    <= '0;
      hsize_d    <= '0;
    end else if (hready) begin
      wr_phase_d <= htrans[1] && hwrite;
      if (htrans[1] && hwrite) begin
        haddr_d <= haddr;
        hsize_d <= hsize;
      end
    end
  end

  always_comb begin
    // hwstrb = '0;
    // if (wr_phase_d) begin
    //   hwstrb = gen_hwstrb(haddr_d, hsize_d);
    // end
  end

  // easier internal state view
  wire [3:0]        state;
  assign state = dut.state;


  ahb_to_axi4_burst #(
    .AW (AW),
    .DW (DW),
    .IW (IW),
    .MAX_INCR_BEATS(MAX_INCR_BEATS)
  ) dut (
    .clk        (clk),
    .resetn     (resetn),

    .HSEL       (hsel),
    .HREADYIN   (hreadyin),    
    .HADDR      (haddr),
    .HBURST     (hburst),
    .HMASTLOCK  (hmastlock),
    .HPROT      (hprot),
    .HSIZE      (hsize),
    .HTRANS     (htrans),
    .HWDATA     (hwdata),
    //.HWSTRB     (hwstrb),
    .HWRITE     (hwrite),
    .HRDATA     (hrdata),
    .HREADY     (hready),
    .HRESP      (hresp),
    // added later

    .AWID       (m_axi_awid),
    .AWADDR     (m_axi_awaddr),
    .AWLEN      (m_axi_awlen),
    .AWSIZE     (m_axi_awsize),
    .AWBURST    (m_axi_awburst),
    .AWLOCK     (m_axi_awlock),
    .AWCACHE    (m_axi_awcache),
    .AWPROT     (m_axi_awprot),
    .AWQOS      (m_axi_awqos),
    .AWVALID    (m_axi_awvalid),
    .AWREADY    (m_axi_awready),

    .WDATA      (m_axi_wdata),
    .WSTRB      (m_axi_wstrb),
    .WLAST      (m_axi_wlast),
    .WVALID     (m_axi_wvalid),
    .WREADY     (m_axi_wready),

    .BID        (m_axi_bid),
    .BRESP      (m_axi_bresp),
    .BVALID     (m_axi_bvalid),
    .BREADY     (m_axi_bready),

    .ARID       (m_axi_arid),
    .ARADDR     (m_axi_araddr),
    .ARLEN      (m_axi_arlen),
    .ARSIZE     (m_axi_arsize),
    .ARBURST    (m_axi_arburst),
    .ARLOCK     (m_axi_arlock),
    .ARCACHE    (m_axi_arcache),
    .ARPROT     (m_axi_arprot),
    .ARQOS      (m_axi_arqos),
    .ARVALID    (m_axi_arvalid),
    .ARREADY    (m_axi_arready),

    .RID        (m_axi_rid),
    .RDATA      (m_axi_rdata),
    .RRESP      (m_axi_rresp),
    .RLAST      (m_axi_rlast),
    .RVALID     (m_axi_rvalid),
    .RREADY     (m_axi_rready)
  );

endmodule
