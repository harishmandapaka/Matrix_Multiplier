//==============================================================================
//  design.sv  --  DUT models for the systolic-array DV project
//------------------------------------------------------------------------------
//  The assignment does not require the RTL. These SystemVerilog models are
//  provided so that the UVM testbench (testbench.sv) compiles and runs on
//  EDA Playground. They are also the optional "cycle-accurate DUT model".
//
//    sa_pe          : processing element (signed MAC, forwards a right / b down)
//    systolic_array : N x N PE mesh + input skew + output row alignment
//                     (port list exactly as spec page 11)
//    mm_rst_sync    : reset synchronizer (async assert / sync de-assert)
//    mm_async_fifo  : dual-clock FIFO, gray-code pointers, 2-flop synchronizers
//    sub_sys        : input FIFO -> controller/alignment -> systolic_array ->
//                     output alignment -> output FIFO   (ports as spec page 12)
//
//  Array-level protocol implemented here (= testbench assumptions, see README)
//    P1  One sample per clock: a_din[i] = A[i][k] (column k of A),
//                              b_din[j] = B[k][j] (row k of B).
//    P2  in_valid = 1 on the LAST sample (k = M-1) of each A/B pair. There is no
//        per-sample valid, so idle cycles must carry a_din = b_din = 0.
//    P3  Two in_valid pulses must be >= N cycles apart (N result rows per matrix
//        share one N-wide output port). M >= N therefore streams back-to-back.
//    P4  C is returned row by row on N consecutive cycles: c_dout[j] = C[i][j],
//        out_valid = 1 with row N-1. c_dout = 0 when no row is being returned.
//    P5  Latency: out_valid is seen 2*N clocks after in_valid.
//    P6  c_din is held at 0 (C = A x B, D ignored as the spec allows).
//    P7  Elements are signed; C wraps modulo 2^(2*DIN_WIDTH).
//==============================================================================
`timescale 1ns/1ps

//------------------------------------------------------------------------------
// Processing element (output stationary): acc += a*b every clock; on the last
// sample the finished dot product moves to 'res' and acc restarts at 0, so the
// next matrix can follow without a bubble.
//------------------------------------------------------------------------------
module sa_pe #(parameter int DW = 8) (
  input  logic                   clk,
  input  logic                   rst_n,
  input  logic signed [DW-1:0]   a_in,      // input activation (from the left)
  input  logic signed [DW-1:0]   b_in,      // weight (from above)
  input  logic                   last_in,   // last-sample flag, travels with a
  output logic signed [DW-1:0]   a_out,     // forwarded activation (to the right)
  output logic signed [DW-1:0]   b_out,     // forwarded weight (to below)
  output logic                   last_out,
  output logic signed [2*DW-1:0] res,       // finished C[i][j]
  output logic                   res_vld    // 1-clock pulse when res is updated
);
  logic signed [2*DW-1:0] acc;
  logic signed [2*DW-1:0] mac;

  assign mac = acc + a_in * b_in;            // signed MAC, wraps modulo 2^(2*DW)

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      a_out    <= '0;
      b_out    <= '0;
      last_out <= 1'b0;
      acc      <= '0;
      res      <= '0;
      res_vld  <= 1'b0;
    end else begin
      a_out    <= a_in;
      b_out    <= b_in;
      last_out <= last_in;
      res_vld  <= last_in;
      if (last_in) begin
        res <= mac;
        acc <= '0;
      end else begin
        acc <= mac;
      end
    end
  end
endmodule

//------------------------------------------------------------------------------
// N x N systolic array (interface exactly as in the specification)
//------------------------------------------------------------------------------
module systolic_array #(
  parameter int DIN_WIDTH = 8,
  parameter int N         = 4
) (
  input  logic                   rst_n,               // active-low reset
  input  logic                   clk,
  input  logic [2*DIN_WIDTH-1:0] c_din  [0:N-1],      // partial-sum input of row 0 (held at 0)
  input  logic [DIN_WIDTH-1:0]   a_din  [0:N-1],      // column k of A (signed)
  input  logic [DIN_WIDTH-1:0]   b_din  [0:N-1],      // row k of B (signed)
  input  logic                   in_valid,            // last sample of the current A/B pair
  output logic [2*DIN_WIDTH-1:0] c_dout [N-1:0],      // one row of C per clock (signed)
  output logic                   out_valid            // last row of the current C
);
  localparam int DW = DIN_WIDTH;

  // mesh interconnect
  wire signed [DW-1:0]   a_h   [0:N-1][0:N];          // a_h[i][j]: activation entering PE(i,j)
  wire                   l_h   [0:N-1][0:N];          // last flag entering PE(i,j)
  wire signed [DW-1:0]   b_v   [0:N][0:N-1];          // b_v[i][j]: weight entering PE(i,j)
  wire signed [2*DW-1:0] res   [0:N-1][0:N-1];
  wire                   res_v [0:N-1][0:N-1];

  //----------------------------------------------------------------------------
  // Input alignment: row i of A (with its last flag) is delayed by i clocks and
  // column j of B by j clocks, so A[i][k] and B[k][j] meet in PE(i,j).
  //----------------------------------------------------------------------------
  for (genvar i = 0; i < N; i++) begin : g_a_skew
    if (i == 0) begin : g_d0
      assign a_h[0][0] = a_din[0];
      assign l_h[0][0] = in_valid;
    end else begin : g_dn
      logic [DW-1:0] a_sr [0:i-1];
      logic          l_sr [0:i-1];
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          for (int d = 0; d < i; d++) begin
            a_sr[d] <= '0;
            l_sr[d] <= 1'b0;
          end
        end else begin
          a_sr[0] <= a_din[i];
          l_sr[0] <= in_valid;
          for (int d = 1; d < i; d++) begin
            a_sr[d] <= a_sr[d-1];
            l_sr[d] <= l_sr[d-1];
          end
        end
      end
      assign a_h[i][0] = a_sr[i-1];
      assign l_h[i][0] = l_sr[i-1];
    end
  end

  for (genvar j = 0; j < N; j++) begin : g_b_skew
    if (j == 0) begin : g_d0
      assign b_v[0][0] = b_din[0];
    end else begin : g_dn
      logic [DW-1:0] b_sr [0:j-1];
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          for (int d = 0; d < j; d++) b_sr[d] <= '0;
        end else begin
          b_sr[0] <= b_din[j];
          for (int d = 1; d < j; d++) b_sr[d] <= b_sr[d-1];
        end
      end
      assign b_v[0][j] = b_sr[j-1];
    end
  end

  //----------------------------------------------------------------------------
  // PE mesh
  //----------------------------------------------------------------------------
  for (genvar i = 0; i < N; i++) begin : g_row
    for (genvar j = 0; j < N; j++) begin : g_col
      sa_pe #(.DW(DW)) u_pe (
        .clk      (clk),
        .rst_n    (rst_n),
        .a_in     (a_h[i][j]),
        .b_in     (b_v[i][j]),
        .last_in  (l_h[i][j]),
        .a_out    (a_h[i][j+1]),
        .b_out    (b_v[i+1][j]),
        .last_out (l_h[i][j+1]),
        .res      (res[i][j]),
        .res_vld  (res_v[i][j])
      );
    end
  end

  //----------------------------------------------------------------------------
  // Output alignment: row i is complete when PE(i,N-1) finishes. Because two
  // in_valid pulses are >= N clocks apart (P3), at most one row completes per
  // clock; rows leave in order 0..N-1 on consecutive clocks.
  //----------------------------------------------------------------------------
  logic [2*DW-1:0] c_q [0:N-1];
  logic            ov_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int j = 0; j < N; j++) c_q[j] <= '0;
      ov_q <= 1'b0;
    end else begin
      for (int j = 0; j < N; j++) c_q[j] <= '0;     // default output is zero
      ov_q <= 1'b0;
      for (int i = 0; i < N; i++) begin
        if (res_v[i][N-1]) begin
          for (int j = 0; j < N; j++) c_q[j] <= res[i][j];
          ov_q <= (i == N-1);
        end
      end
    end
  end

  always_comb begin
    for (int j = 0; j < N; j++) c_dout[j] = c_q[j];  // index by index ([N-1:0] vs [0:N-1])
  end
  assign out_valid = ov_q;

  // synthesis translate_off
  // This model computes C = A x B only; make a non-zero c_din visible in the log.
  always @(posedge clk) begin
    if (rst_n === 1'b1) begin
      for (int j = 0; j < N; j++)
        if (c_din[j] !== '0)
          $display("%0t [SA_MODEL] WARNING: c_din[%0d]=%0h is ignored by this model", $time, j, c_din[j]);
    end
  end
  // synthesis translate_on
endmodule

//------------------------------------------------------------------------------
// Reset synchronizer: asynchronous assert, synchronous de-assert
//------------------------------------------------------------------------------
module mm_rst_sync (
  input  logic clk,
  input  logic arst_n,
  output logic srst_n
);
  logic meta;
  always_ff @(posedge clk or negedge arst_n) begin
    if (!arst_n) begin
      meta   <= 1'b0;
      srst_n <= 1'b0;
    end else begin
      meta   <= 1'b1;
      srst_n <= meta;
    end
  end
endmodule

//------------------------------------------------------------------------------
// Dual-clock FIFO (gray-code pointers + 2-flop synchronizers).
//   * write ignored when full, read ignored when empty
//   * rdata is registered: valid in the clock after an accepted read
//   * wlevel is the fill level seen from the write side (pessimistic)
//------------------------------------------------------------------------------
module mm_async_fifo #(
  parameter int WIDTH  = 32,
  parameter int ADDR_W = 4                      // depth = 2**ADDR_W
) (
  input  logic              wclk,
  input  logic              wrst_n,
  input  logic              winc,
  input  logic [WIDTH-1:0]  wdata,
  output logic              wfull,
  output logic [ADDR_W:0]   wlevel,
  input  logic              rclk,
  input  logic              rrst_n,
  input  logic              rinc,
  output logic [WIDTH-1:0]  rdata,
  output logic              rempty
);
  localparam int DEPTH = 1 << ADDR_W;

  logic [WIDTH-1:0] mem [0:DEPTH-1];
  logic [ADDR_W:0]  wbin, wgray, rbin, rgray;
  logic [ADDR_W:0]  rgray_w1, rgray_w2;         // read pointer synchronized to wclk
  logic [ADDR_W:0]  wgray_r1, wgray_r2;         // write pointer synchronized to rclk
  logic [ADDR_W:0]  wbin_nxt, rbin_nxt, wcnt_nxt;
  logic             wr_en, rd_en;

  function automatic logic [ADDR_W:0] bin2gray(input logic [ADDR_W:0] b);
    return b ^ (b >> 1);
  endfunction

  function automatic logic [ADDR_W:0] gray2bin(input logic [ADDR_W:0] g);
    logic [ADDR_W:0] b;
    b[ADDR_W] = g[ADDR_W];
    for (int k = ADDR_W - 1; k >= 0; k--) b[k] = b[k+1] ^ g[k];
    return b;
  endfunction

  // ---------------- write clock domain ----------------
  assign wr_en    = winc & ~wfull;
  assign wbin_nxt = wbin + wr_en;
  assign wcnt_nxt = wbin_nxt - gray2bin(rgray_w2);
  assign wlevel   = wbin - gray2bin(rgray_w2);

  always_ff @(posedge wclk or negedge wrst_n) begin
    if (!wrst_n) begin
      wbin  <= '0;
      wgray <= '0;
      wfull <= 1'b0;
    end else begin
      wbin  <= wbin_nxt;
      wgray <= bin2gray(wbin_nxt);
      wfull <= (wcnt_nxt == DEPTH);
    end
  end

  always_ff @(posedge wclk) begin
    if (wr_en) mem[wbin[ADDR_W-1:0]] <= wdata;
  end

  always_ff @(posedge wclk or negedge wrst_n) begin
    if (!wrst_n) begin
      rgray_w1 <= '0;
      rgray_w2 <= '0;
    end else begin
      rgray_w1 <= rgray;
      rgray_w2 <= rgray_w1;
    end
  end

  // ---------------- read clock domain ----------------
  assign rd_en    = rinc & ~rempty;
  assign rbin_nxt = rbin + rd_en;

  always_ff @(posedge rclk or negedge rrst_n) begin
    if (!rrst_n) begin
      rbin   <= '0;
      rgray  <= '0;
      rempty <= 1'b1;
      rdata  <= '0;
    end else begin
      rbin   <= rbin_nxt;
      rgray  <= bin2gray(rbin_nxt);
      rempty <= (bin2gray(rbin_nxt) == wgray_r2);
      if (rd_en) rdata <= mem[rbin[ADDR_W-1:0]];
    end
  end

  always_ff @(posedge rclk or negedge rrst_n) begin
    if (!rrst_n) begin
      wgray_r1 <= '0;
      wgray_r2 <= '0;
    end else begin
      wgray_r1 <= wgray;
      wgray_r2 <= wgray_r1;
    end
  end
endmodule

//------------------------------------------------------------------------------
// Sub-system (interface as in the specification; FIFO depths are extra
// parameters with defaults, so the spec instantiation is unchanged)
//   din  word: din[i*DW +: DW]        = A[i][k]  (column k of A)
//              din[N*DW + j*DW +: DW] = B[k][j]  (row k of B)
//   dout word: dout[j*2DW +: 2DW]     = C[i][j]  (row i of C), rows in order
//------------------------------------------------------------------------------
module sub_sys #(
  parameter int DIN_WIDTH   = 8,
  parameter int N           = 4,
  parameter int BUS_WIDTH   = 2*DIN_WIDTH*N,
  parameter int IN_FIFO_AW  = 4,                // input  FIFO depth = 2**IN_FIFO_AW
  parameter int OUT_FIFO_AW = $clog2(4*N)       // output FIFO depth = 2**OUT_FIFO_AW (>= 4N)
) (
  input  logic                 rst_n,
  input  logic                 sys_clk,         // bus clock: FIFO write and read side
  input  logic                 sr_clk,          // systolic array + controller clock
  input  logic [7:0]           M_minus_one,     // M - 1 (quasi-static, change only when idle)
  input  logic [BUS_WIDTH-1:0] din,
  input  logic                 wr_fifo,
  input  logic                 rd_fifo,
  output logic                 in_fifo_full,
  output logic [BUS_WIDTH-1:0] dout,            // valid the sys_clk after an accepted read
  output logic                 out_fifo_empty
);
  localparam int DW        = DIN_WIDTH;
  localparam int CW        = 2*DIN_WIDTH;
  localparam int OUT_DEPTH = 1 << OUT_FIFO_AW;

  // synthesis translate_off
  initial begin
    if (BUS_WIDTH != 2*DIN_WIDTH*N)
      $fatal(1, "sub_sys: BUS_WIDTH (%0d) must equal 2*DIN_WIDTH*N (%0d)", BUS_WIDTH, 2*DIN_WIDTH*N);
    if (OUT_DEPTH < N)
      $fatal(1, "sub_sys: output FIFO depth (%0d) must be >= N (%0d)", OUT_DEPTH, N);
  end
  // synthesis translate_on

  //----------------------------------------------------------------------------
  // Resets (one synchronizer per clock domain)
  //----------------------------------------------------------------------------
  logic rst_sys_n, rst_sr_n;
  mm_rst_sync u_rst_sys (.clk(sys_clk), .arst_n(rst_n), .srst_n(rst_sys_n));
  mm_rst_sync u_rst_sr  (.clk(sr_clk),  .arst_n(rst_n), .srst_n(rst_sr_n));

  //----------------------------------------------------------------------------
  // Input FIFO: sys_clk -> sr_clk
  //----------------------------------------------------------------------------
  logic [BUS_WIDTH-1:0] in_rdata;
  logic                 in_empty;
  logic                 in_pop;

  mm_async_fifo #(.WIDTH(BUS_WIDTH), .ADDR_W(IN_FIFO_AW)) u_in_fifo (
    .wclk   (sys_clk),  .wrst_n (rst_sys_n), .winc (wr_fifo), .wdata (din),
    .wfull  (in_fifo_full), .wlevel (),
    .rclk   (sr_clk),   .rrst_n (rst_sr_n),  .rinc (in_pop),  .rdata (in_rdata),
    .rempty (in_empty)
  );

  //----------------------------------------------------------------------------
  // Controller (sr_clk)
  //  * pops one sample per clock; the sample reaches the array one clock later
  //  * counts samples against M_minus_one and flags the last one (in_valid)
  //  * the last sample of a matrix is only issued when
  //      (a) >= N clocks have passed since the previous last sample (rule P3)
  //      (b) the output FIFO has room for this matrix's N rows (credits)
  //    so the array never has to stall and the output FIFO never overflows
  //----------------------------------------------------------------------------
  logic [7:0]           m1_s1, m1_s2;           // M_minus_one, 2-flop synchronized
  logic [7:0]           smp_cnt;                // sample index inside the matrix
  logic [15:0]          gap_cnt;                // clocks since the last 'last' pop (saturating)
  logic [15:0]          pend_rows;              // rows promised to the output FIFO, not yet written
  logic [OUT_FIFO_AW:0] out_level;
  logic                 out_full;
  logic                 is_last, credit_ok, close_ok;
  logic                 iss_q, iss_last_q;      // a sample is presented to the array this clock
  logic                 out_winc;
  logic [BUS_WIDTH-1:0] out_wdata;

  always_ff @(posedge sr_clk) begin
    m1_s1 <= M_minus_one;
    m1_s2 <= m1_s1;
  end

  assign is_last   = (smp_cnt == m1_s2);
  assign credit_ok = (OUT_DEPTH - int'(out_level) - int'(pend_rows)) >= N;
  assign close_ok  = (gap_cnt >= N) && credit_ok;
  assign in_pop    = !in_empty && (!is_last || close_ok);

  always_ff @(posedge sr_clk or negedge rst_sr_n) begin
    if (!rst_sr_n) begin
      smp_cnt    <= '0;
      gap_cnt    <= 16'(N);
      pend_rows  <= '0;
      iss_q      <= 1'b0;
      iss_last_q <= 1'b0;
    end else begin
      iss_q      <= in_pop;
      iss_last_q <= in_pop && is_last;
      if (in_pop) smp_cnt <= is_last ? 8'd0 : smp_cnt + 8'd1;
      if (in_pop && is_last)  gap_cnt <= 16'd1;
      else if (gap_cnt < N)   gap_cnt <= gap_cnt + 16'd1;
      pend_rows <= pend_rows + ((in_pop && is_last) ? 16'(N) : 16'd0)
                             - ((out_winc && !out_full) ? 16'd1 : 16'd0);
    end
  end

  //----------------------------------------------------------------------------
  // Input data alignment: unpack one FIFO word into a column of A / row of B
  //----------------------------------------------------------------------------
  logic [DW-1:0] sa_a     [0:N-1];
  logic [DW-1:0] sa_b     [0:N-1];
  logic [CW-1:0] sa_c_in  [0:N-1];
  logic [CW-1:0] sa_c_out [N-1:0];
  logic          sa_out_valid;

  always_comb begin
    for (int g = 0; g < N; g++) begin
      sa_a[g]    = iss_q ? in_rdata[g*DW +: DW]        : '0;   // idle clocks carry zeros (P2)
      sa_b[g]    = iss_q ? in_rdata[N*DW + g*DW +: DW] : '0;
      sa_c_in[g] = '0;                                         // C = A x B (P6)
    end
  end

  systolic_array #(.DIN_WIDTH(DW), .N(N)) u_sa (
    .rst_n     (rst_sr_n),
    .clk       (sr_clk),
    .c_din     (sa_c_in),
    .a_din     (sa_a),
    .b_din     (sa_b),
    .in_valid  (iss_last_q),
    .c_dout    (sa_c_out),
    .out_valid (sa_out_valid)
  );

  //----------------------------------------------------------------------------
  // Output data alignment: keep the last N rows; on out_valid copy them to a
  // hold buffer and write them to the output FIFO, one row per clock.
  //----------------------------------------------------------------------------
  logic [BUS_WIDTH-1:0] row_now;
  logic [BUS_WIDTH-1:0] hist [0:N-1];           // hist[N-1] = row seen at the previous edge
  logic [BUS_WIDTH-1:0] hold [0:N-1];
  logic [15:0]          drain_left, drain_idx;

  always_comb begin
    for (int g = 0; g < N; g++) row_now[g*CW +: CW] = sa_c_out[g];
  end

  always_ff @(posedge sr_clk or negedge rst_sr_n) begin
    if (!rst_sr_n) begin
      for (int r = 0; r < N; r++) begin
        hist[r] <= '0;
        hold[r] <= '0;
      end
      drain_left <= '0;
      drain_idx  <= '0;
    end else begin
      for (int r = 0; r < N-1; r++) hist[r] <= hist[r+1];
      hist[N-1] <= row_now;
      if (drain_left != 0) begin
        drain_left <= drain_left - 16'd1;
        drain_idx  <= drain_idx  + 16'd1;
      end
      if (sa_out_valid) begin                    // rows of C = previous N-1 rows + this one
        for (int r = 0; r < N-1; r++) hold[r] <= hist[r+1];
        hold[N-1]  <= row_now;
        drain_left <= 16'(N);
        drain_idx  <= '0;
      end
    end
  end

  assign out_winc  = (drain_left != 0);
  assign out_wdata = (drain_idx < N) ? hold[drain_idx] : '0;

  //----------------------------------------------------------------------------
  // Output FIFO: sr_clk -> sys_clk
  //----------------------------------------------------------------------------
  mm_async_fifo #(.WIDTH(BUS_WIDTH), .ADDR_W(OUT_FIFO_AW)) u_out_fifo (
    .wclk   (sr_clk),   .wrst_n (rst_sr_n),  .winc (out_winc), .wdata (out_wdata),
    .wfull  (out_full), .wlevel (out_level),
    .rclk   (sys_clk),  .rrst_n (rst_sys_n), .rinc (rd_fifo),  .rdata (dout),
    .rempty (out_fifo_empty)
  );

  // synthesis translate_off
  always @(posedge sr_clk) begin
    if (rst_sr_n === 1'b1 && out_winc && out_full)
      $display("%0t [SUB_SYS_MODEL] ERROR: output FIFO overflow (credit logic broken)", $time);
  end
  // synthesis translate_on
endmodule
