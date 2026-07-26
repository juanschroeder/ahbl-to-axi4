// Passive AXI4 performance monitor.
//
// All counters wrap naturally.  The module deliberately has no control
// outputs: it is safe to insert on any AXI interface without changing its
// timing or protocol behaviour.
`default_nettype none

module axi_perf_monitor #(
  parameter int unsigned COUNTER_W = 32
) (
  input  wire                   clk,
  input  wire                   resetn,

  input  wire                   awvalid,
  input  wire                   awready,
  input  wire [7:0]             awlen,
  input  wire                   wvalid,
  input  wire                   wready,
  input  wire                   wlast,
  input  wire                   bvalid,
  input  wire                   bready,

  input  wire                   arvalid,
  input  wire                   arready,
  input  wire [7:0]             arlen,
  input  wire                   rvalid,
  input  wire                   rready,
  input  wire                   rlast,

  output logic [COUNTER_W-1:0]  cycles,
  output logic [COUNTER_W-1:0]  aw_transactions,
  output logic [COUNTER_W-1:0]  ar_transactions,
  output logic [COUNTER_W-1:0]  w_beats,
  output logic [COUNTER_W-1:0]  r_beats,
  output logic [COUNTER_W-1:0]  b_responses,

  output logic [COUNTER_W-1:0]  aw_stall_cycles,
  output logic [COUNTER_W-1:0]  w_stall_cycles,
  output logic [COUNTER_W-1:0]  b_stall_cycles,
  output logic [COUNTER_W-1:0]  ar_stall_cycles,
  output logic [COUNTER_W-1:0]  r_stall_cycles,

  output logic [COUNTER_W-1:0]  write_data_bubble_cycles,
  output logic [COUNTER_W-1:0]  write_b_wait_cycles,
  output logic [COUNTER_W-1:0]  read_first_r_wait_cycles,
  output logic [COUNTER_W-1:0]  read_r_bubble_cycles,

  output logic [COUNTER_W-1:0]  aw_single_bursts,
  output logic [COUNTER_W-1:0]  aw_incr4_bursts,
  output logic [COUNTER_W-1:0]  aw_incr8_bursts,
  output logic [COUNTER_W-1:0]  aw_incr16_bursts,
  output logic [COUNTER_W-1:0]  aw_other_bursts,
  output logic [COUNTER_W-1:0]  ar_single_bursts,
  output logic [COUNTER_W-1:0]  ar_incr4_bursts,
  output logic [COUNTER_W-1:0]  ar_incr8_bursts,
  output logic [COUNTER_W-1:0]  ar_incr16_bursts,
  output logic [COUNTER_W-1:0]  ar_other_bursts
);

  logic write_active;
  logic write_last_accepted;
  logic read_active;
  logic read_beat_seen;

  always_ff @(posedge clk) begin
    if (!resetn) begin
      cycles                   <= '0;
      aw_transactions          <= '0;
      ar_transactions          <= '0;
      w_beats                  <= '0;
      r_beats                  <= '0;
      b_responses              <= '0;
      aw_stall_cycles          <= '0;
      w_stall_cycles           <= '0;
      b_stall_cycles           <= '0;
      ar_stall_cycles          <= '0;
      r_stall_cycles           <= '0;
      write_data_bubble_cycles <= '0;
      write_b_wait_cycles      <= '0;
      read_first_r_wait_cycles <= '0;
      read_r_bubble_cycles     <= '0;
      aw_single_bursts         <= '0;
      aw_incr4_bursts          <= '0;
      aw_incr8_bursts          <= '0;
      aw_incr16_bursts         <= '0;
      aw_other_bursts          <= '0;
      ar_single_bursts         <= '0;
      ar_incr4_bursts          <= '0;
      ar_incr8_bursts          <= '0;
      ar_incr16_bursts         <= '0;
      ar_other_bursts          <= '0;
      write_active             <= 1'b0;
      write_last_accepted      <= 1'b0;
      read_active              <= 1'b0;
      read_beat_seen           <= 1'b0;
    end else begin
      cycles <= cycles + 1'b1;

      if (awvalid && awready) begin
        aw_transactions     <= aw_transactions + 1'b1;
        write_active        <= 1'b1;
        write_last_accepted <= 1'b0;
        case (awlen)
          8'd0:  aw_single_bursts <= aw_single_bursts + 1'b1;
          8'd3:  aw_incr4_bursts  <= aw_incr4_bursts + 1'b1;
          8'd7:  aw_incr8_bursts  <= aw_incr8_bursts + 1'b1;
          8'd15: aw_incr16_bursts <= aw_incr16_bursts + 1'b1;
          default: aw_other_bursts <= aw_other_bursts + 1'b1;
        endcase
      end
      if (wvalid && wready) begin
        w_beats <= w_beats + 1'b1;
        if (wlast)
          write_last_accepted <= 1'b1;
      end
      if (bvalid && bready) begin
        b_responses         <= b_responses + 1'b1;
        write_active        <= 1'b0;
        write_last_accepted <= 1'b0;
      end

      if (arvalid && arready) begin
        ar_transactions <= ar_transactions + 1'b1;
        read_active     <= 1'b1;
        read_beat_seen  <= 1'b0;
        case (arlen)
          8'd0:  ar_single_bursts <= ar_single_bursts + 1'b1;
          8'd3:  ar_incr4_bursts  <= ar_incr4_bursts + 1'b1;
          8'd7:  ar_incr8_bursts  <= ar_incr8_bursts + 1'b1;
          8'd15: ar_incr16_bursts <= ar_incr16_bursts + 1'b1;
          default: ar_other_bursts <= ar_other_bursts + 1'b1;
        endcase
      end
      if (rvalid && rready) begin
        r_beats        <= r_beats + 1'b1;
        read_beat_seen <= 1'b1;
        if (rlast) begin
          read_active    <= 1'b0;
          read_beat_seen <= 1'b0;
        end
      end

      if (awvalid && !awready) aw_stall_cycles <= aw_stall_cycles + 1'b1;
      if (wvalid  && !wready)  w_stall_cycles  <= w_stall_cycles + 1'b1;
      if (bvalid  && !bready)  b_stall_cycles  <= b_stall_cycles + 1'b1;
      if (arvalid && !arready) ar_stall_cycles <= ar_stall_cycles + 1'b1;
      if (rvalid  && !rready)  r_stall_cycles  <= r_stall_cycles + 1'b1;

      // Bubbles exclude a VALID/READY stall.  They show an absent producer,
      // rather than backpressure from the component hosting this monitor.
      if (write_active && !write_last_accepted && !wvalid)
        write_data_bubble_cycles <= write_data_bubble_cycles + 1'b1;
      if (write_last_accepted && !bvalid)
        write_b_wait_cycles <= write_b_wait_cycles + 1'b1;
      if (read_active && !read_beat_seen && !rvalid)
        read_first_r_wait_cycles <= read_first_r_wait_cycles + 1'b1;
      if (read_active && read_beat_seen && !rvalid)
        read_r_bubble_cycles <= read_r_bubble_cycles + 1'b1;
    end
  end
endmodule

`default_nettype wire
