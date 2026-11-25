//==============================================================================
//  testbench.sv  --  UVM testbench: N x N systolic array + sub-system
//------------------------------------------------------------------------------
//  Design-verification project
//
//  Module level     : systolic_array -> sa_env (active agent, golden model,
//                     scoreboard, latency/throughput and idle-zero checks, coverage)
//  Sub-system level : sub_sys        -> ss_env (FIFO write agent, FIFO read agent,
//                     the same golden model / scoreboard / coverage / sequences,
//                     plus sa_env re-used in PASSIVE mode on the embedded array)
//
//  EDA Playground
//    Testbench pane : this file            Design pane : design.sv
//    UVM / OVM      : UVM 1.2              Simulator   : Synopsys VCS (or another
//                                                        commercial SV simulator)
//    DUT parameters : Compile options  +define+TB_N=<N> +define+TB_DW=<DIN_WIDTH>
//    Test           : Run options      +UVM_TESTNAME=<test>   (default mm_full_test)
//    Waveforms      : tick "Open EPWave after run"  (+WAVE_FULL, +WAVE_NS=<ns>, +NO_WAVES)
//
//  Tests   : mm_full_test (both levels), sa_smoke_test, sa_rand_test, sa_corner_test,
//            sa_b2b_test, ss_smoke_test, ss_rand_test, ss_clk_sweep_test,
//            ss_m_sweep_test, ss_backpressure_test
//  Options : +SYS_CLK_NS=<ns> +SR_CLK_NS=<ns> +SA_CLK_NS=<ns>  clock periods
//            +SS_M=<1..256>     sub-system M (M_minus_one = M-1)
//            +SA_LATENCY=<clk>  expected in_valid -> out_valid latency (-1: not checked)
//            +RD_LATENCY=<0|1>  dout valid in the same / next sys_clk after a read
//            +NO_ARR_MON        no white-box monitor on the array inside sub_sys
//            +SB_PRINT=<n>      print full matrices for the first n matches
//==============================================================================
`timescale 1ns/1ps
`include "uvm_macros.svh"

`ifndef TB_N
  `define TB_N 4
`endif
`ifndef TB_DW
  `define TB_DW 8
`endif

//==============================================================================
// Clock / reset generator. Periods are programmable at run time, so tests can
// choose (and change) the sys_clk / sr_clk relationship.
//==============================================================================
interface clk_rst_if;
  bit      clk;                         // systolic_array clock (module level)
  bit      sys_clk;                     // sub_sys bus clock
  bit      sr_clk;                      // sub_sys systolic-array clock
  realtime clk_hp = 5.0;                // half periods in ns
  realtime sys_hp = 5.0;
  realtime sr_hp  = 5.0;
  realtime sr_ofs = 0.0;                // sr_clk start offset (phase) in ns
  bit      clk_en = 1'b0;
  bit      ss_en  = 1'b0;

  initial begin : gen_clk
    wait (clk_en);
    forever #(clk_hp) clk = ~clk;
  end

  initial begin : gen_sys_clk
    wait (ss_en);
    forever #(sys_hp) sys_clk = ~sys_clk;
  end

  initial begin : gen_sr_clk
    wait (ss_en);
    #(sr_ofs);
    forever #(sr_hp) sr_clk = ~sr_clk;
  end

  function automatic void start_sa_clk(real period_ns);
    clk_hp = period_ns / 2.0;
    clk_en = 1'b1;
  endfunction

  // first call starts both clocks, later calls change the frequencies on the fly
  function automatic void set_ss_clks(real sys_ns, real sr_ns, real sr_phase_ns);
    sys_hp = sys_ns / 2.0;
    sr_hp  = sr_ns  / 2.0;
    if (!ss_en) sr_ofs = sr_phase_ns;
    ss_en = 1'b1;
  endfunction
endinterface

//==============================================================================
// systolic_array interface (names and ranges identical to the DUT ports)
//==============================================================================
interface sa_if #(parameter int DW = 8, parameter int N = 4) (input logic clk);
  import uvm_pkg::*;

  logic            rst_n;
  logic [2*DW-1:0] c_din  [0:N-1];
  logic [DW-1:0]   a_din  [0:N-1];
  logic [DW-1:0]   b_din  [0:N-1];
  logic            in_valid;
  logic [2*DW-1:0] c_dout [N-1:0];
  logic            out_valid;

  // P3: two last-sample markers must be at least N clocks apart
  if (N > 1) begin : g_p3
    a_in_valid_spacing : assert property (
      @(posedge clk) disable iff (rst_n !== 1'b1) in_valid |=> (!in_valid) [*(N-1)])
      else `uvm_error("SA_IF", $sformatf("in_valid asserted again within %0d clocks (rule P3)", N))
  end

  a_ctrl_known : assert property (
    @(posedge clk) disable iff (rst_n !== 1'b1) !$isunknown({in_valid, out_valid}))
    else `uvm_error("SA_IF", "X/Z on in_valid / out_valid")
endinterface

//==============================================================================
// sub_sys interface (bus side, sys_clk domain)
//==============================================================================
interface ss_if #(parameter int DW = 8, parameter int N = 4, parameter int BW = 2*DW*N)
                 (input logic sys_clk);
  import uvm_pkg::*;

  logic          rst_n;
  logic [7:0]    M_minus_one;
  logic [BW-1:0] din;
  logic          wr_fifo;
  logic          rd_fifo;
  logic          in_fifo_full;
  logic [BW-1:0] dout;
  logic          out_fifo_empty;

  a_hs_known : assert property (
    @(posedge sys_clk) disable iff (rst_n !== 1'b1)
    !$isunknown({wr_fifo, rd_fifo, in_fifo_full, out_fifo_empty}))
    else `uvm_error("SS_IF", "X/Z on FIFO handshake signals")

  a_din_known : assert property (
    @(posedge sys_clk) disable iff (rst_n !== 1'b1) wr_fifo |-> !$isunknown(din))
    else `uvm_error("SS_IF", "X/Z on din while wr_fifo = 1")

  // dout is valid in the sys_clk after an accepted read (dout_fwft = 1: during the read)
  bit dout_fwft = 1'b0;

  a_dout_known : assert property (
    @(posedge sys_clk) disable iff (rst_n !== 1'b1)
    (rd_fifo && !out_fifo_empty && !dout_fwft) |=> !$isunknown(dout))
    else `uvm_error("SS_IF", "X/Z on dout after an accepted read")

  a_dout_known_fwft : assert property (
    @(posedge sys_clk) disable iff (rst_n !== 1'b1)
    (rd_fifo && !out_fifo_empty && dout_fwft) |-> !$isunknown(dout))
    else `uvm_error("SS_IF", "X/Z on dout during an accepted read")
endinterface

//==============================================================================
// Testbench package
//==============================================================================
package mm_tb_pkg;
  import uvm_pkg::*;

  //----------------------------------------------------------------------------
  // Compile-time DUT parameters (+define+TB_N=.. +define+TB_DW=..)
  //----------------------------------------------------------------------------
  localparam int N     = `TB_N;         // PE mesh is N x N
  localparam int DW    = `TB_DW;        // DIN_WIDTH
  localparam int CW    = 2*DW;          // width of one C element
  localparam int BW    = 2*DW*N;        // sub_sys BUS_WIDTH
  localparam int MAX_M = 256;           // M_minus_one is 8 bits wide

  typedef virtual sa_if #(.DW(DW), .N(N))          sa_vif_t;
  typedef virtual ss_if #(.DW(DW), .N(N), .BW(BW)) ss_vif_t;
  typedef virtual clk_rst_if                       clk_vif_t;

  // data patterns used by the constrained-random generator
  typedef enum {PAT_RANDOM, PAT_SMALL, PAT_CORNER, PAT_MAX_POS, PAT_MAX_NEG,
                PAT_ALT_EXT, PAT_ZERO, PAT_SPARSE, PAT_IDENTITY} pat_e;

  // two's-complement wrap of v to 'bits' bits (bits <= 62)
  function automatic longint sext(longint v, int bits);
    longint modulus;
    longint r;
    modulus = longint'(1) << bits;
    r = v & (modulus - 1);
    if (r >= (modulus >> 1)) r = r - modulus;
    return r;
  endfunction

  //============================================================================
  // Transaction: one matrix pair (A: N x M, B: M x N) and its result C (N x N)
  //============================================================================
  class mm_txn extends uvm_sequence_item;
    `uvm_object_utils(mm_txn)

    // -------- stimulus --------
    rand int unsigned m;                // inner dimension M
    rand pat_e        pat_a;
    rand pat_e        pat_b;
    rand int          a[];              // A[i][k] = a[i*m + k]   (sample k = column k of A)
    rand int          b[];              // B[k][j] = b[k*N + j]   (sample k = row k of B)
    rand int unsigned gap;              // idle clocks inserted before this matrix pair

    // -------- result / observation --------
    longint           c[];              // C[i][j] = c[i*N + j]
    int unsigned      txn_id;
    longint           in_cyc  = -1;     // monitor clock of the last input sample
    longint           out_cyc = -1;     // monitor clock of out_valid
    realtime          t_in;
    realtime          t_out;
    int unsigned      bp_cycles;        // in_fifo_full back-pressure seen while writing it

    // -------- knobs, set by sequences before randomize() --------
    int min_m  = 1;
    int max_m  = 3*N;
    int gap_lo = 0;
    int gap_hi = 8;
    int b2b_w  = 60;                    // weight (1..99) of gap == 0 (back-to-back)

    int min_v;                          // -2^(DW-1)
    int max_v;                          //  2^(DW-1) - 1

    constraint c_m    { m >= 1; m <= MAX_M; m >= min_m; m <= max_m; }
    constraint c_size { a.size() == N*m; b.size() == m*N; }
    // b2b_w % back-to-back, the rest spread evenly over 1..gap_hi
    constraint c_gap  {
      gap >= gap_lo;
      gap <= gap_hi;
      if (gap_lo == 0 && gap_hi > 0) {
        gap dist {0 := b2b_w, [1:gap_hi] :/ (100 - b2b_w)};
      }
    }
    constraint c_pat  {
      pat_a dist {PAT_RANDOM := 40, PAT_SMALL := 10, PAT_CORNER := 15, PAT_MAX_POS := 5,
                  PAT_MAX_NEG := 5, PAT_ALT_EXT := 5, PAT_ZERO := 3, PAT_SPARSE := 10,
                  PAT_IDENTITY := 7};
      pat_b dist {PAT_RANDOM := 40, PAT_SMALL := 10, PAT_CORNER := 15, PAT_MAX_POS := 5,
                  PAT_MAX_NEG := 5, PAT_ALT_EXT := 5, PAT_ZERO := 3, PAT_SPARSE := 10,
                  PAT_IDENTITY := 7};
      if (pat_a == PAT_IDENTITY) { m == N; }
      if (pat_b == PAT_IDENTITY) { m == N; }
    }
    constraint c_range {
      foreach (a[x]) a[x] inside {[min_v:max_v]};
      foreach (b[x]) b[x] inside {[min_v:max_v]};
    }
    // PAT_RANDOM: full signed range (c_range only)
    constraint c_data_a {
      foreach (a[x]) {
        if (pat_a == PAT_SMALL)         { a[x] inside {[-4:4]}; }
        else if (pat_a == PAT_CORNER)   { a[x] inside {min_v, min_v + 1, -1, 0, 1, max_v - 1, max_v}; }
        else if (pat_a == PAT_MAX_POS)  { a[x] == max_v; }
        else if (pat_a == PAT_MAX_NEG)  { a[x] == min_v; }
        else if (pat_a == PAT_ALT_EXT)  { a[x] == ((x % 2 == 0) ? max_v : min_v); }
        else if (pat_a == PAT_ZERO)     { a[x] == 0; }
        else if (pat_a == PAT_SPARSE)   { a[x] dist {0 := 75, [min_v:-1] :/ 12, [1:max_v] :/ 13}; }
        else if (pat_a == PAT_IDENTITY) { a[x] == ((x / N == x % N) ? 1 : 0); }
      }
    }
    constraint c_data_b {
      foreach (b[x]) {
        if (pat_b == PAT_SMALL)         { b[x] inside {[-4:4]}; }
        else if (pat_b == PAT_CORNER)   { b[x] inside {min_v, min_v + 1, -1, 0, 1, max_v - 1, max_v}; }
        else if (pat_b == PAT_MAX_POS)  { b[x] == max_v; }
        else if (pat_b == PAT_MAX_NEG)  { b[x] == min_v; }
        else if (pat_b == PAT_ALT_EXT)  { b[x] == ((x % 2 == 0) ? min_v : max_v); }
        else if (pat_b == PAT_ZERO)     { b[x] == 0; }
        else if (pat_b == PAT_SPARSE)   { b[x] dist {0 := 75, [min_v:-1] :/ 12, [1:max_v] :/ 13}; }
        else if (pat_b == PAT_IDENTITY) { b[x] == ((x / N == x % N) ? 1 : 0); }
      }
    }

    function new(string name = "mm_txn");
      super.new(name);
      min_v = -(1 << (DW - 1));
      max_v =  (1 << (DW - 1)) - 1;
    endfunction

    function int get_a(int i, int k);
      return a[i*int'(m) + k];
    endfunction

    function int get_b(int k, int j);
      return b[k*N + j];
    endfunction

    function longint get_c(int i, int j);
      return c[i*N + j];
    endfunction

    function void do_copy(uvm_object rhs);
      mm_txn r;
      super.do_copy(rhs);
      if (!$cast(r, rhs)) `uvm_fatal("MM_TXN", "do_copy: wrong type")
      m         = r.m;
      pat_a     = r.pat_a;
      pat_b     = r.pat_b;
      a         = r.a;
      b         = r.b;
      gap       = r.gap;
      c         = r.c;
      txn_id    = r.txn_id;
      in_cyc    = r.in_cyc;
      out_cyc   = r.out_cyc;
      t_in      = r.t_in;
      t_out     = r.t_out;
      bp_cycles = r.bp_cycles;
    endfunction

    function bit do_compare(uvm_object rhs, uvm_comparer comparer);
      mm_txn r;
      if (!$cast(r, rhs)) return 0;
      if (c.size() != r.c.size()) return 0;
      foreach (c[x]) if (c[x] != r.c[x]) return 0;
      return 1;
    endfunction

    function string convert2string();
      return $sformatf("id=%0d M=%0d pat_a=%s pat_b=%s gap=%0d",
                       txn_id, m, pat_a.name(), pat_b.name(), gap);
    endfunction

    // matrices as text (A and B are summarized when M > 16)
    function string mat2str(bit show_ab = 1, bit show_c = 1);
      string s;
      s = "";
      if (show_ab && m > 0 && a.size() == N*m && b.size() == m*N) begin
        if (m <= 16) begin
          s = {s, $sformatf("\n      A (%0d x %0d):", N, m)};
          for (int i = 0; i < N; i++) begin
            s = {s, "\n        "};
            for (int k = 0; k < int'(m); k++) s = {s, $sformatf("%8d", get_a(i, k))};
          end
          s = {s, $sformatf("\n      B (%0d x %0d):", m, N)};
          for (int k = 0; k < int'(m); k++) begin
            s = {s, "\n        "};
            for (int j = 0; j < N; j++) s = {s, $sformatf("%8d", get_b(k, j))};
          end
        end else begin
          s = {s, $sformatf("\n      A (%0d x %0d) / B (%0d x %0d): not printed (M > 16)", N, m, m, N)};
        end
      end
      if (show_c && c.size() == N*N) begin
        s = {s, $sformatf("\n      C (%0d x %0d):", N, N)};
        for (int i = 0; i < N; i++) begin
          s = {s, "\n        "};
          for (int j = 0; j < N; j++) s = {s, $sformatf("%12d", get_c(i, j))};
        end
      end
      return s;
    endfunction
  endclass

  //----------------------------------------------------------------------------
  // sub_sys bus word for sample k:  {B row k, A column k}
  //----------------------------------------------------------------------------
  function automatic logic [BW-1:0] pack_sample(mm_txn t, int k);
    logic [BW-1:0] w;
    logic [31:0]   v;
    w = '0;
    for (int i = 0; i < N; i++) begin
      v = t.get_a(i, k);
      w[i*DW +: DW] = v[DW-1:0];
    end
    for (int j = 0; j < N; j++) begin
      v = t.get_b(k, j);
      w[N*DW + j*DW +: DW] = v[DW-1:0];
    end
    return w;
  endfunction

  //============================================================================
  // Golden reference model: C = A x B, each element wrapped to 2*DIN_WIDTH bits
  //============================================================================
  class mm_ref_model;
    static function void predict(mm_txn t);
      t.c = new[N*N];
      for (int i = 0; i < N; i++) begin
        for (int j = 0; j < N; j++) begin
          longint acc;
          acc = 0;
          for (int k = 0; k < int'(t.m); k++)
            acc += longint'(t.get_a(i, k)) * longint'(t.get_b(k, j));
          t.c[i*N + j] = sext(acc, CW);
        end
      end
    endfunction

    // 1 if any exact (un-wrapped) dot product does not fit in 2*DIN_WIDTH bits
    static function bit has_wrap(mm_txn t);
      for (int i = 0; i < N; i++) begin
        for (int j = 0; j < N; j++) begin
          longint acc;
          acc = 0;
          for (int k = 0; k < int'(t.m); k++)
            acc += longint'(t.get_a(i, k)) * longint'(t.get_b(k, j));
          if (acc != sext(acc, CW)) return 1;
        end
      end
      return 0;
    endfunction
  endclass

  //============================================================================
  // Configuration objects
  //============================================================================
  class sa_cfg extends uvm_object;
    `uvm_object_utils(sa_cfg)
    sa_vif_t                vif;
    uvm_active_passive_enum is_active       = UVM_ACTIVE;
    string                  rpt_id          = "SA";
    int                     exp_latency     = 2*N;  // in_valid -> out_valid clocks (-1: not checked)
    bit                     check_idle_zero = 1;    // c_dout must be 0 outside result rows
    int                     print_first     = 2;    // print full matrices of the first matches
    real                    clk_ns          = 10.0;
    function new(string name = "sa_cfg");
      super.new(name);
    endfunction
  endclass

  class ss_cfg extends uvm_object;
    `uvm_object_utils(ss_cfg)
    ss_vif_t     vif;
    clk_vif_t    cvif;
    sa_vif_t     arr_vif;                // white-box probe of the embedded systolic_array
    bit          has_arr_mon = 1;        // re-use sa_env (passive) on the embedded array
    int unsigned m           = N;        // programmed M (M_minus_one = M - 1)
    real         sys_ns      = 10.0;     // sys_clk period
    real         sr_ns       = 10.0;     // sr_clk period
    real         sr_phase_ns = 0.0;      // sr_clk start offset
    int          rd_latency  = 1;        // dout valid 1 sys_clk after an accepted read
    int          arr_latency = 2*N;      // embedded array in_valid -> out_valid clocks (-1: not checked)
    bit          idle_junk   = 1;        // random din while wr_fifo = 0 (must be ignored)
    int          print_first = 2;
    function new(string name = "ss_cfg");
      super.new(name);
    endfunction
  endclass

  //============================================================================
  // Sequences (shared by the module-level and the sub-system-level agents)
  //============================================================================
  typedef uvm_sequencer #(mm_txn) mm_sqr_t;

  class mm_base_seq extends uvm_sequence #(mm_txn);
    `uvm_object_utils(mm_base_seq)
    int unsigned num_txn   = 10;
    int          min_m     = 1;
    int          max_m     = 3*N;
    int          gap_lo    = 0;
    int          gap_hi    = 8;
    int          b2b_w     = 60;
    bit          fix_pat   = 0;
    pat_e        pat_a_fix = PAT_RANDOM;
    pat_e        pat_b_fix = PAT_RANDOM;
    static int unsigned id_cnt = 0;

    function new(string name = "mm_base_seq");
      super.new(name);
    endfunction

    task body();
      for (int unsigned n = 0; n < num_txn; n++) begin
        req = mm_txn::type_id::create("req");
        req.min_m  = min_m;
        req.max_m  = max_m;
        req.gap_lo = gap_lo;
        req.gap_hi = gap_hi;
        req.b2b_w  = b2b_w;
        start_item(req);
        if (fix_pat) begin
          if (!req.randomize() with { pat_a == pat_a_fix; pat_b == pat_b_fix; })
            `uvm_fatal("SEQ", "mm_txn randomization failed")
        end else begin
          if (!req.randomize())
            `uvm_fatal("SEQ", "mm_txn randomization failed")
        end
        req.txn_id = id_cnt;
        id_cnt++;
        finish_item(req);
      end
    endtask
  endclass

  // directed sanity: identities, small values back-to-back, extremes
  class mm_directed_seq extends mm_base_seq;
    `uvm_object_utils(mm_directed_seq)
    int unsigned dm = N;                 // M to use (sub-system: the programmed M)

    function new(string name = "mm_directed_seq");
      super.new(name);
    endfunction

    task body();
      if (dm == N) begin
        send_one(PAT_IDENTITY, PAT_SMALL,    0);   // A = I  ->  C = B
        send_one(PAT_SMALL,    PAT_IDENTITY, 0);   // B = I  ->  C = A
      end
      send_one(PAT_SMALL,   PAT_SMALL,   0);       // back-to-back small values
      send_one(PAT_SMALL,   PAT_SMALL,   0);
      send_one(PAT_SMALL,   PAT_SMALL,   0);
      send_one(PAT_MAX_NEG, PAT_MAX_NEG, 2);       // (-2^(W-1))^2 * M : largest positive sum
      send_one(PAT_MAX_POS, PAT_MAX_NEG, 0);       // most negative sum
      send_one(PAT_CORNER,  PAT_CORNER,  0);
    endtask

    task send_one(pat_e pa, pat_e pb, int unsigned g);
      req = mm_txn::type_id::create("req");
      req.min_m  = dm;
      req.max_m  = dm;
      req.gap_lo = g;
      req.gap_hi = g;
      start_item(req);
      if (!req.randomize() with { pat_a == pa; pat_b == pb; })
        `uvm_fatal("SEQ", "directed mm_txn randomization failed")
      req.txn_id = id_cnt;
      id_cnt++;
      finish_item(req);
    endtask
  endclass

  // output-FIFO read throttling (sub-system read agent)
  class ss_rd_item extends uvm_sequence_item;
    `uvm_object_utils(ss_rd_item)
    rand int unsigned cycles;            // length of this phase in sys_clk cycles
    rand int unsigned rate_pct;          // probability of rd_fifo = 1 in each cycle
    constraint c_legal { cycles inside {[1:4096]}; rate_pct <= 100; }
    function new(string name = "ss_rd_item");
      super.new(name);
    endfunction
    function string convert2string();
      return $sformatf("cycles=%0d rate=%0d%%", cycles, rate_pct);
    endfunction
  endclass

  typedef uvm_sequencer #(ss_rd_item) rd_sqr_t;

  class ss_rd_seq extends uvm_sequence #(ss_rd_item);
    `uvm_object_utils(ss_rd_seq)
    int unsigned rate_lo   = 40;         // read probability range of a normal phase (%)
    int unsigned rate_hi   = 100;
    int unsigned stall_pct = 10;         // chance (%) that a phase is a complete stall
    int unsigned stall_max = 40;         // longest stall (cycles)

    function new(string name = "ss_rd_seq");
      super.new(name);
    endfunction

    task body();
      forever begin
        req = ss_rd_item::type_id::create("req");
        start_item(req);
        if ($urandom_range(99, 0) < stall_pct) begin
          if (!req.randomize() with { rate_pct == 0; cycles inside {[1:stall_max]}; })
            `uvm_fatal("RD_SEQ", "ss_rd_item randomization failed")
        end else begin
          if (!req.randomize() with { rate_pct inside {[rate_lo:rate_hi]}; cycles inside {[4:64]}; })
            `uvm_fatal("RD_SEQ", "ss_rd_item randomization failed")
        end
        finish_item(req);
      end
    endtask
  endclass

  //============================================================================
  // Module-level agent (systolic_array ports)
  //============================================================================
  class sa_driver extends uvm_driver #(mm_txn);
    `uvm_component_utils(sa_driver)
    sa_cfg   cfg;
    sa_vif_t vif;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(sa_cfg)::get(this, "", "cfg", cfg))
        `uvm_fatal("SA_DRV", "sa_cfg not found")
      vif = cfg.vif;
    endfunction

    task drive_idle();
      for (int i = 0; i < N; i++) begin
        vif.a_din[i] <= '0;
        vif.b_din[i] <= '0;
        vif.c_din[i] <= '0;                  // C = A x B: partial-sum input held at 0
      end
      vif.in_valid <= 1'b0;
    endtask

    task drive_sample(mm_txn t, int k);
      logic [31:0] v;
      for (int i = 0; i < N; i++) begin
        v = t.get_a(i, k);
        vif.a_din[i] <= v[DW-1:0];           // column k of A
      end
      for (int j = 0; j < N; j++) begin
        v = t.get_b(k, j);
        vif.b_din[j] <= v[DW-1:0];           // row k of B
        vif.c_din[j] <= '0;
      end
      vif.in_valid <= (k == int'(t.m) - 1);  // last sample of the pair
    endtask

    // Cycle-based: at every clock decide what is presented in the next cycle.
    // try_next_item() keeps back-to-back matrices without a bubble.
    task run_phase(uvm_phase phase);
      mm_txn  cur;
      int     k;
      int     gap_left;
      int     min_gap;
      int     mm;
      longint cyc;
      longint last_cyc;
      cur      = null;
      k        = 0;
      gap_left = 0;
      cyc      = 0;
      last_cyc = -N;
      drive_idle();
      forever begin
        @(posedge vif.clk);
        cyc++;
        if (vif.rst_n !== 1'b1) begin
          drive_idle();
          continue;
        end
        if (cur == null) begin
          seq_item_port.try_next_item(cur);
          if (cur != null) begin
            k        = 0;
            mm       = int'(cur.m);
            // P3: the next last sample must come >= N clocks after the previous one
            min_gap  = int'(last_cyc + N - (cyc + mm - 1));
            gap_left = (int'(cur.gap) > min_gap) ? int'(cur.gap) : min_gap;
            if (gap_left < 0) gap_left = 0;
            `uvm_info("SA_DRV", {"driving ", cur.convert2string()}, UVM_HIGH)
          end
        end
        if (cur == null || gap_left > 0) begin
          drive_idle();
          if (gap_left > 0) gap_left--;
        end else begin
          drive_sample(cur, k);
          if (k == int'(cur.m) - 1) begin
            last_cyc = cyc;
            seq_item_port.item_done();
            cur = null;
          end else begin
            k++;
          end
        end
      end
    endtask
  endclass

  class sa_monitor extends uvm_monitor;
    `uvm_component_utils(sa_monitor)

    typedef struct {
      int a [N];                             // column of A seen in this clock
      int b [N];                             // row of B seen in this clock
    } smp_t;

    typedef struct {
      longint v [N];                         // c_dout row seen in this clock
      bit     nz;                            // any element non-zero
      bit     claimed;                       // belongs to a result window
    } row_t;

    sa_cfg   cfg;
    sa_vif_t vif;
    uvm_analysis_port #(mm_txn) ap_in;       // A/B observed at the inputs (closed by in_valid)
    uvm_analysis_port #(mm_txn) ap_out;      // C observed at the outputs (closed by out_valid)

    smp_t   win[$];                          // samples of the matrix being received
    row_t   rows[$];                         // the last N output rows
    longint cyc;
    int     n_in;
    int     n_out;

    function new(string name, uvm_component parent);
      super.new(name, parent);
      ap_in  = new("ap_in", this);
      ap_out = new("ap_out", this);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(sa_cfg)::get(this, "", "cfg", cfg))
        `uvm_fatal("SA_MON", "sa_cfg not found")
      vif = cfg.vif;
    endfunction

    task run_phase(uvm_phase phase);
      cyc = 0;
      forever begin
        @(posedge vif.clk);
        cyc++;
        if (vif.rst_n !== 1'b1) begin
          win.delete();
          rows.delete();
          continue;
        end
        sample_inputs();
        sample_outputs();
      end
    endtask

    // every clock is a sample (idle clocks carry zeros); in_valid closes the pair
    function void sample_inputs();
      smp_t  s;
      bit    nz;
      int    mm;
      mm_txn t;
      nz = 0;
      for (int i = 0; i < N; i++) begin
        if ($isunknown(vif.a_din[i]) || $isunknown(vif.b_din[i]))
          `uvm_error(cfg.rpt_id, $sformatf("X/Z on a_din/b_din[%0d]", i))
        s.a[i] = int'($signed(vif.a_din[i]));
        s.b[i] = int'($signed(vif.b_din[i]));
        if (s.a[i] != 0 || s.b[i] != 0) nz = 1;
      end
      // leading all-zero samples cannot change C: skip them so idle time is not counted in M
      if (win.size() != 0 || nz || vif.in_valid === 1'b1) win.push_back(s);
      if (vif.in_valid === 1'b1) begin
        mm  = win.size();
        t   = mm_txn::type_id::create("sa_in");
        t.m = mm;
        t.a = new[N*mm];
        t.b = new[mm*N];
        for (int k = 0; k < mm; k++) begin
          for (int i = 0; i < N; i++) t.a[i*mm + k] = win[k].a[i];
          for (int j = 0; j < N; j++) t.b[k*N + j]  = win[k].b[j];
        end
        t.txn_id = n_in;
        n_in++;
        t.in_cyc = cyc;
        t.t_in   = $realtime;
        win.delete();
        ap_in.write(t);
      end
    endfunction

    // c_dout carries one row of C per clock; out_valid flags row N-1
    function void sample_outputs();
      row_t  r;
      int    base;
      mm_txn t;
      r.nz      = 0;
      r.claimed = 0;
      for (int j = 0; j < N; j++) begin
        if ($isunknown(vif.c_dout[j]))
          `uvm_error(cfg.rpt_id, $sformatf("X/Z on c_dout[%0d]", j))
        r.v[j] = longint'($signed(vif.c_dout[j]));
        if (r.v[j] != 0) r.nz = 1;
      end
      rows.push_back(r);
      if (vif.out_valid === 1'b1) begin
        if (rows.size() < N) begin
          `uvm_error(cfg.rpt_id, "out_valid seen less than N clocks after reset")
        end else begin
          base = rows.size() - N;
          t    = mm_txn::type_id::create("sa_out");
          t.c  = new[N*N];
          for (int i = 0; i < N; i++) begin
            if (rows[base + i].claimed)
              `uvm_error(cfg.rpt_id, "result rows of two matrices overlap (out_valid < N clocks apart)")
            rows[base + i].claimed = 1;
            for (int j = 0; j < N; j++) t.c[i*N + j] = rows[base + i].v[j];
          end
          t.txn_id  = n_out;
          n_out++;
          t.out_cyc = cyc;
          t.t_out   = $realtime;
          ap_out.write(t);
        end
      end
      // a row that leaves the N-deep window without being claimed must be all zero
      while (rows.size() > N) begin
        row_t old;
        old = rows.pop_front();
        if (cfg.check_idle_zero && !old.claimed && old.nz)
          `uvm_error(cfg.rpt_id, "c_dout is non-zero outside a result window (spec: default output 0)")
      end
    endfunction
  endclass

  class sa_agent extends uvm_agent;
    `uvm_component_utils(sa_agent)
    sa_cfg     cfg;
    sa_driver  drv;
    mm_sqr_t   sqr;
    sa_monitor mon;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(sa_cfg)::get(this, "", "cfg", cfg))
        `uvm_fatal("SA_AGT", "sa_cfg not found")
      mon = sa_monitor::type_id::create("mon", this);
      if (cfg.is_active == UVM_ACTIVE) begin
        drv = sa_driver::type_id::create("drv", this);
        sqr = mm_sqr_t::type_id::create("sqr", this);
      end
    endfunction

    function void connect_phase(uvm_phase phase);
      super.connect_phase(phase);
      if (cfg.is_active == UVM_ACTIVE) drv.seq_item_port.connect(sqr.seq_item_export);
    endfunction
  endclass

  //============================================================================
  // Sub-system agents: input-FIFO writer and output-FIFO reader (sys_clk)
  //============================================================================
  class ss_wr_driver extends uvm_driver #(mm_txn);
    `uvm_component_utils(ss_wr_driver)
    ss_cfg   cfg;
    ss_vif_t vif;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(ss_cfg)::get(this, "", "cfg", cfg))
        `uvm_fatal("SS_WR_DRV", "ss_cfg not found")
      vif = cfg.vif;
    endfunction

    function logic [BW-1:0] junk();
      logic [BW-1:0] w;
      for (int x = 0; x < BW; x++) w[x] = $urandom_range(1, 0);
      return w;
    endfunction

    // One FIFO word per sample. A write is accepted on a sys_clk edge with
    // wr_fifo = 1 and in_fifo_full = 0; otherwise the word is held.
    task run_phase(uvm_phase phase);
      mm_txn cur;
      int    k;
      int    gap_left;
      cur      = null;
      k        = 0;
      gap_left = 0;
      vif.wr_fifo <= 1'b0;
      vif.din     <= '0;
      forever begin
        @(posedge vif.sys_clk);
        if (vif.rst_n !== 1'b1) begin
          vif.wr_fifo <= 1'b0;
          continue;
        end
        // 1) the word presented in the last cycle was accepted at this edge?
        if (cur != null && vif.wr_fifo === 1'b1 && vif.in_fifo_full === 1'b0) begin
          k++;
          if (k == int'(cur.m)) begin
            seq_item_port.item_done();
            cur = null;
          end
        end
        // 2) fetch the next matrix pair without blocking (keeps the bus streaming)
        if (cur == null) begin
          seq_item_port.try_next_item(cur);
          if (cur != null) begin
            k        = 0;
            gap_left = int'(cur.gap);
            if (cur.m != cfg.m)
              `uvm_error("SS_WR_DRV", $sformatf("item M=%0d but the DUT is programmed for M=%0d", cur.m, cfg.m))
            `uvm_info("SS_WR_DRV", {"writing ", cur.convert2string()}, UVM_HIGH)
          end
        end
        // 3) present the next word (held while in_fifo_full = 1)
        if (cur != null && gap_left == 0) begin
          vif.din     <= pack_sample(cur, k);
          vif.wr_fifo <= 1'b1;
        end else begin
          vif.wr_fifo <= 1'b0;
          vif.din     <= cfg.idle_junk ? junk() : '0;
          if (gap_left > 0) gap_left--;
        end
      end
    endtask
  endclass

  class ss_wr_monitor extends uvm_monitor;
    `uvm_component_utils(ss_wr_monitor)
    ss_cfg       cfg;
    ss_vif_t     vif;
    uvm_analysis_port #(mm_txn) ap;          // one matrix pair per M accepted writes
    int unsigned n_wr;                       // accepted writes
    int unsigned n_bp;                       // cycles with wr_fifo = 1 and in_fifo_full = 1
    int unsigned n_txn;

    function new(string name, uvm_component parent);
      super.new(name, parent);
      ap = new("ap", this);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(ss_cfg)::get(this, "", "cfg", cfg))
        `uvm_fatal("SS_WR_MON", "ss_cfg not found")
      vif = cfg.vif;
    endfunction

    task run_phase(uvm_phase phase);
      mm_txn  cur;
      int     k;
      int     mm;
      longint cyc;
      cur = null;
      k   = 0;
      mm  = 1;
      cyc = 0;
      forever begin
        @(posedge vif.sys_clk);
        cyc++;
        if (vif.rst_n !== 1'b1) begin
          cur = null;
          k   = 0;
          continue;
        end
        if (vif.wr_fifo === 1'b1) begin
          if (vif.in_fifo_full === 1'b1) begin
            n_bp++;
            if (cur != null) cur.bp_cycles++;
          end else begin
            if (cur == null) begin
              mm  = int'(vif.M_minus_one) + 1;   // M is taken from the DUT pin
              cur = mm_txn::type_id::create("ss_in");
              cur.m = mm;
              cur.a = new[N*mm];
              cur.b = new[mm*N];
              cur.bp_cycles = 0;
              k = 0;
            end
            for (int i = 0; i < N; i++) cur.a[i*mm + k] = int'($signed(vif.din[i*DW +: DW]));
            for (int j = 0; j < N; j++) cur.b[k*N + j]  = int'($signed(vif.din[N*DW + j*DW +: DW]));
            n_wr++;
            k++;
            if (k == mm) begin
              cur.txn_id = n_txn;
              n_txn++;
              cur.in_cyc = cyc;
              cur.t_in   = $realtime;
              ap.write(cur);
              cur = null;
            end
          end
        end
      end
    endtask
  endclass

  class ss_wr_agent extends uvm_agent;
    `uvm_component_utils(ss_wr_agent)
    ss_cfg        cfg;
    ss_wr_driver  drv;
    mm_sqr_t      sqr;
    ss_wr_monitor mon;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      mon = ss_wr_monitor::type_id::create("mon", this);
      drv = ss_wr_driver::type_id::create("drv", this);
      sqr = mm_sqr_t::type_id::create("sqr", this);
    endfunction

    function void connect_phase(uvm_phase phase);
      super.connect_phase(phase);
      drv.seq_item_port.connect(sqr.seq_item_export);
    endfunction
  endclass

  class ss_rd_driver extends uvm_driver #(ss_rd_item);
    `uvm_component_utils(ss_rd_driver)
    ss_cfg   cfg;
    ss_vif_t vif;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(ss_cfg)::get(this, "", "cfg", cfg))
        `uvm_fatal("SS_RD_DRV", "ss_cfg not found")
      vif = cfg.vif;
    endfunction

    // rd_fifo is also raised while the FIFO is empty: such reads must be ignored
    task run_phase(uvm_phase phase);
      vif.rd_fifo <= 1'b0;
      forever begin
        seq_item_port.get_next_item(req);
        repeat (req.cycles) begin
          @(posedge vif.sys_clk);
          vif.rd_fifo <= (vif.rst_n === 1'b1) && ($urandom_range(99, 0) < req.rate_pct);
        end
        seq_item_port.item_done();
      end
    endtask
  endclass

  class ss_rd_monitor extends uvm_monitor;
    `uvm_component_utils(ss_rd_monitor)
    ss_cfg       cfg;
    ss_vif_t     vif;
    uvm_analysis_port #(mm_txn) ap;          // one C matrix per N accepted reads
    mm_txn       cur;
    int          row;
    int unsigned n_rd;
    int unsigned n_txn;

    function new(string name, uvm_component parent);
      super.new(name, parent);
      ap = new("ap", this);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(ss_cfg)::get(this, "", "cfg", cfg))
        `uvm_fatal("SS_RD_MON", "ss_cfg not found")
      vif = cfg.vif;
    endfunction

    task run_phase(uvm_phase phase);
      bit rd_acc;
      bit rd_pend;
      cur     = null;
      row     = 0;
      rd_pend = 0;
      forever begin
        @(posedge vif.sys_clk);
        if (vif.rst_n !== 1'b1) begin
          cur     = null;
          row     = 0;
          rd_pend = 0;
          continue;
        end
        rd_acc = (vif.rd_fifo === 1'b1) && (vif.out_fifo_empty === 1'b0);
        if (cfg.rd_latency == 0) begin
          if (rd_acc) capture_row();
        end else begin
          if (rd_pend) capture_row();
        end
        rd_pend = rd_acc;
      end
    endtask

    function void capture_row();
      if (cur == null) begin
        cur   = mm_txn::type_id::create("ss_out");
        cur.c = new[N*N];
        row   = 0;
      end
      if ($isunknown(vif.dout)) `uvm_error("SS_RD_MON", "X/Z on dout")
      for (int j = 0; j < N; j++) cur.c[row*N + j] = longint'($signed(vif.dout[j*CW +: CW]));
      n_rd++;
      row++;
      if (row == N) begin
        cur.txn_id = n_txn;
        n_txn++;
        cur.t_out  = $realtime;
        ap.write(cur);
        cur = null;
      end
    endfunction
  endclass

  class ss_rd_agent extends uvm_agent;
    `uvm_component_utils(ss_rd_agent)
    ss_rd_driver  drv;
    rd_sqr_t      sqr;
    ss_rd_monitor mon;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      mon = ss_rd_monitor::type_id::create("mon", this);
      drv = ss_rd_driver::type_id::create("drv", this);
      sqr = rd_sqr_t::type_id::create("sqr", this);
    endfunction

    function void connect_phase(uvm_phase phase);
      super.connect_phase(phase);
      drv.seq_item_port.connect(sqr.seq_item_export);
    endfunction
  endclass

  //============================================================================
  // Scoreboard (re-used at both levels): golden model on every input matrix,
  // in-order compare with every output matrix, latency / throughput statistics
  //============================================================================
  `uvm_analysis_imp_decl(_in)
  `uvm_analysis_imp_decl(_out)

  class mm_scoreboard extends uvm_scoreboard;
    `uvm_component_utils(mm_scoreboard)
    uvm_analysis_imp_in  #(mm_txn, mm_scoreboard) in_imp;
    uvm_analysis_imp_out #(mm_txn, mm_scoreboard) out_imp;

    string   rpt_id      = "SB";
    int      exp_latency = -1;               // clocks; -1 = not checked
    int      print_first = 2;
    mm_txn   exp_q[$];
    int      n_in;
    int      n_out;
    int      n_match;
    int      n_mismatch;
    longint  lat_min     = -1;               // clocks (same-clock monitors only)
    longint  lat_max     = -1;
    longint  last_out_cyc = -1;
    longint  ival_min    = -1;               // clocks between consecutive out_valid
    longint  ival_max    = -1;
    realtime tl_min, tl_max, tl_sum;         // input -> output latency in ns
    realtime t_first_out, t_last_out;

    function new(string name, uvm_component parent);
      super.new(name, parent);
      in_imp  = new("in_imp", this);
      out_imp = new("out_imp", this);
    endfunction

    function int pending();
      return exp_q.size();
    endfunction

    function void write_in(mm_txn t);
      mm_txn e;
      $cast(e, t.clone());
      mm_ref_model::predict(e);
      exp_q.push_back(e);
      n_in++;
    endfunction

    function void write_out(mm_txn t);
      mm_txn   e;
      int      errs;
      longint  lat;
      realtime tl;
      n_out++;
      if (exp_q.size() == 0) begin
        `uvm_error(rpt_id, $sformatf("unexpected C matrix #%0d (no input matrix pending)%s",
                                  n_out - 1, t.mat2str(0, 1)))
        return;
      end
      e    = exp_q.pop_front();
      errs = 0;
      if (t.c.size() != N*N) begin
        `uvm_error(rpt_id, $sformatf("observed C has %0d elements, expected %0d", t.c.size(), N*N))
        errs = 1;
      end else begin
        for (int x = 0; x < N*N; x++) begin
          if (t.c[x] != e.c[x]) begin
            errs++;
            if (errs <= 8)
              `uvm_error(rpt_id, $sformatf("matrix #%0d C[%0d][%0d]: expected %0d, observed %0d (M=%0d)",
                                        n_out - 1, x / N, x % N, e.c[x], t.c[x], e.m))
          end
        end
      end
      if (errs != 0) begin
        n_mismatch++;
        `uvm_info(rpt_id, {"failing matrix pair and expected C:", e.mat2str(1, 1),
                        "\n    observed:", t.mat2str(0, 1)}, UVM_NONE)
      end else begin
        n_match++;
        if (n_match <= print_first)
          `uvm_info(rpt_id, $sformatf("MATCH #%0d  M=%0d  observed C == golden C%s", n_out - 1, e.m,
                                   e.mat2str(1, 1)), UVM_LOW)
        else
          `uvm_info(rpt_id, $sformatf("MATCH #%0d  M=%0d", n_out - 1, e.m), UVM_HIGH)
      end
      // latency in clocks (module level: both sides seen by one monitor)
      if (e.in_cyc >= 0 && t.out_cyc >= 0) begin
        lat = t.out_cyc - e.in_cyc;
        if (lat_min < 0 || lat < lat_min) lat_min = lat;
        if (lat > lat_max) lat_max = lat;
        if (exp_latency >= 0 && lat != exp_latency)
          `uvm_error(rpt_id, $sformatf("matrix #%0d latency %0d clocks, expected %0d", n_out - 1, lat, exp_latency))
      end
      // output spacing (throughput) in clocks
      if (t.out_cyc >= 0) begin
        if (last_out_cyc >= 0) begin
          if (ival_min < 0 || t.out_cyc - last_out_cyc < ival_min) ival_min = t.out_cyc - last_out_cyc;
          if (t.out_cyc - last_out_cyc > ival_max) ival_max = t.out_cyc - last_out_cyc;
        end
        last_out_cyc = t.out_cyc;
      end
      // latency in ns (any level)
      tl = t.t_out - e.t_in;
      if (n_out == 1 || tl < tl_min) tl_min = tl;
      if (tl > tl_max) tl_max = tl;
      tl_sum += tl;
      if (n_out == 1) t_first_out = t.t_out;
      t_last_out = t.t_out;
    endfunction

    function void check_phase(uvm_phase phase);
      super.check_phase(phase);
      if (exp_q.size() != 0)
        `uvm_error(rpt_id, $sformatf("%0d input matrices never produced a C matrix", exp_q.size()))
    endfunction

    function string summary();
      string s;
      s = $sformatf("[%s] matrices in/out %0d/%0d  match %0d  mismatch %0d  pending %0d",
                    rpt_id, n_in, n_out, n_match, n_mismatch, exp_q.size());
      if (lat_min >= 0) s = {s, $sformatf("  latency %0d..%0d clk", lat_min, lat_max)};
      else if (n_out > 0) s = {s, $sformatf("  latency %0.1f..%0.1f ns", tl_min, tl_max)};
      return s;
    endfunction

    function void report_phase(uvm_phase phase);
      super.report_phase(phase);
      `uvm_info(rpt_id, summary(), UVM_NONE)
      if (n_out > 1 && ival_min >= 0)
        `uvm_info(rpt_id, $sformatf("out_valid spacing %0d..%0d clocks (one C matrix per M clocks when streaming)",
                                 ival_min, ival_max), UVM_LOW)
      if (n_out > 1 && lat_min < 0)
        `uvm_info(rpt_id, $sformatf("throughput %0.2f matrices/us, mean latency %0.1f ns",
                                 (n_out - 1) * 1000.0 / (t_last_out - t_first_out + 1.0e-3),
                                 tl_sum / n_out), UVM_LOW)
    endfunction
  endclass

  //============================================================================
  // Functional coverage (re-used at both levels)
  //============================================================================
  class mm_coverage extends uvm_subscriber #(mm_txn);
    `uvm_component_utils(mm_coverage)
    string    rpt_id  = "COV";
    bit       ss_mode = 0;                   // sub-system: clock ratio / back-pressure
    clk_vif_t cvif;
    int       n_samples;

    // sampled features
    int       cv_m;
    int       cv_mrel;                         // 0: M < N   1: M == N   2: M > N
    bit       a_min_s, a_max_s, b_min_s, b_max_s;
    bit       a_zero_s, b_zero_s;
    bit       wrap_s, neg_s, pos_s;
    bit       b2b_s;
    bit       ival_ok;
    longint   prev_in_cyc = -1;
    int       ratio_s;
    bit       bp_s;

    covergroup cg_mm;
      option.per_instance = 1;
      cp_m : coverpoint cv_m {
        bins m_1      = {1};
        bins m_2_4    = {[2:4]};
        bins m_5_8    = {[5:8]};
        bins m_9_16   = {[9:16]};
        bins m_17_64  = {[17:64]};
        bins m_65_255 = {[65:255]};
        bins m_256    = {256};
      }
      cp_mrel  : coverpoint cv_mrel { bins lt_n = {0}; bins eq_n = {1}; bins gt_n = {2}; }
      cp_a_min  : coverpoint a_min_s;
      cp_a_max  : coverpoint a_max_s;
      cp_b_min  : coverpoint b_min_s;
      cp_b_max  : coverpoint b_max_s;
      cp_a_zero : coverpoint a_zero_s;
      cp_b_zero : coverpoint b_zero_s;
      cp_wrap   : coverpoint wrap_s;
      cp_neg    : coverpoint neg_s;
      cp_pos    : coverpoint pos_s;
      cp_b2b    : coverpoint b2b_s iff (ival_ok);
      x_rel_wrap : cross cp_mrel, cp_wrap;
      x_rel_b2b  : cross cp_mrel, cp_b2b {
        ignore_bins lt_n_b2b = binsof(cp_mrel.lt_n) && binsof(cp_b2b) intersect {1};   // illegal by P3
      }
    endgroup

    covergroup cg_ss;
      option.per_instance = 1;
      cp_ratio : coverpoint ratio_s {
        bins sr_much_faster  = {0};          // sys/sr period ratio >= 2
        bins sr_faster       = {1};
        bins similar         = {2};
        bins sys_faster      = {3};
        bins sys_much_faster = {4};          // ratio < 0.5
      }
      cp_mrel : coverpoint cv_mrel { bins lt_n = {0}; bins eq_n = {1}; bins gt_n = {2}; }
      cp_bp    : coverpoint bp_s;
      cp_b2b   : coverpoint b2b_s iff (ival_ok);
      x_ratio_m : cross cp_ratio, cp_mrel;
      x_rel_b2b : cross cp_mrel, cp_b2b;     // M < N back-to-back is legal on the bus
    endgroup

    function new(string name, uvm_component parent);
      super.new(name, parent);
      cg_mm = new();
      cg_ss = new();
    endfunction

    function int ratio_bin();
      real r;
      r = cvif.sys_hp / cvif.sr_hp;           // sys_clk period / sr_clk period
      if (r >= 2.0)  return 0;
      if (r >= 1.2)  return 1;
      if (r > 0.83)  return 2;
      if (r > 0.5)   return 3;
      return 4;
    endfunction

    function void write(mm_txn t);
      mm_txn e;
      $cast(e, t.clone());
      mm_ref_model::predict(e);
      cv_m   = int'(e.m);
      cv_mrel = (cv_m < N) ? 0 : ((cv_m == N) ? 1 : 2);
      a_min_s = 0; a_max_s = 0; b_min_s = 0; b_max_s = 0;
      a_zero_s = 1; b_zero_s = 1;
      foreach (e.a[x]) begin
        if (e.a[x] == e.min_v) a_min_s = 1;
        if (e.a[x] == e.max_v) a_max_s = 1;
        if (e.a[x] != 0)       a_zero_s = 0;
      end
      foreach (e.b[x]) begin
        if (e.b[x] == e.min_v) b_min_s = 1;
        if (e.b[x] == e.max_v) b_max_s = 1;
        if (e.b[x] != 0)       b_zero_s = 0;
      end
      wrap_s = mm_ref_model::has_wrap(e);
      neg_s  = 0;
      pos_s  = 0;
      foreach (e.c[x]) begin
        if (e.c[x] < 0) neg_s = 1;
        if (e.c[x] > 0) pos_s = 1;
      end
      ival_ok = (prev_in_cyc >= 0);
      b2b_s   = ival_ok && ((e.in_cyc - prev_in_cyc) == e.m);
      prev_in_cyc = e.in_cyc;
      cg_mm.sample();
      if (ss_mode && cvif != null) begin
        ratio_s = ratio_bin();
        bp_s    = (e.bp_cycles != 0);
        cg_ss.sample();
      end
      n_samples++;
    endfunction

    function void report_phase(uvm_phase phase);
      super.report_phase(phase);
      if (n_samples == 0) return;
      if (ss_mode)
        `uvm_info(rpt_id, $sformatf("functional coverage: matrix/data %0.1f%%, clock-ratio x M %0.1f%%  (%0d samples)",
                                 cg_mm.get_inst_coverage(), cg_ss.get_inst_coverage(), n_samples), UVM_NONE)
      else
        `uvm_info(rpt_id, $sformatf("functional coverage: matrix/data %0.1f%%  (%0d samples)",
                                 cg_mm.get_inst_coverage(), n_samples), UVM_NONE)
    endfunction
  endclass

  //============================================================================
  // Module-level environment (active on the stand-alone array, passive when
  // re-used inside the sub-system environment)
  //============================================================================
  class sa_env extends uvm_env;
    `uvm_component_utils(sa_env)
    sa_cfg        cfg;
    sa_agent      agent;
    mm_scoreboard sb;
    mm_coverage   cov;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(sa_cfg)::get(this, "", "cfg", cfg))
        `uvm_fatal("SA_ENV", "sa_cfg not found")
      uvm_config_db#(sa_cfg)::set(this, "agent*", "cfg", cfg);
      agent = sa_agent::type_id::create("agent", this);
      sb    = mm_scoreboard::type_id::create("sb", this);
      cov   = mm_coverage::type_id::create("cov", this);
      sb.rpt_id      = cfg.rpt_id;
      sb.exp_latency = cfg.exp_latency;
      sb.print_first = cfg.print_first;
      cov.rpt_id     = {cfg.rpt_id, "_COV"};
    endfunction

    function void connect_phase(uvm_phase phase);
      super.connect_phase(phase);
      agent.mon.ap_in.connect(sb.in_imp);
      agent.mon.ap_out.connect(sb.out_imp);
      // a passive monitor inside the sub-system also sees idle (zero) bubbles,
      // which inflate the observed M: matrix coverage is collected on active agents only
      if (cfg.is_active == UVM_ACTIVE) agent.mon.ap_in.connect(cov.analysis_export);
    endfunction

    task reset_dut(int cycles = 5);
      sa_vif_t v;
      v = cfg.vif;
      if (cfg.is_active != UVM_ACTIVE) return;
      v.rst_n <= 1'b0;
      repeat (cycles) @(posedge v.clk);
      v.rst_n <= 1'b1;
      repeat (3) @(posedge v.clk);
      `uvm_info(cfg.rpt_id, "reset released", UVM_MEDIUM)
    endtask

    // wait until every input matrix has produced its C matrix
    task wait_drain(int unsigned max_cycles = 50000);
      sa_vif_t     v;
      int unsigned n;
      v = cfg.vif;
      n = 0;
      repeat (4*N + 4) @(posedge v.clk);
      while (sb.pending() > 0 && n < max_cycles) begin
        @(posedge v.clk);
        n++;
      end
      if (sb.pending() > 0)
        `uvm_error(cfg.rpt_id, $sformatf("drain timeout: %0d matrices still pending", sb.pending()))
    endtask
  endclass

  //============================================================================
  // Sub-system environment
  //============================================================================
  class ss_env extends uvm_env;
    `uvm_component_utils(ss_env)
    ss_cfg        cfg;
    ss_wr_agent   wr_agent;
    ss_rd_agent   rd_agent;
    mm_scoreboard sb;                        // end-to-end: FIFO writes -> FIFO reads
    mm_coverage   cov;
    sa_env        arr_env;                   // module-level env, PASSIVE, on the embedded array
    sa_cfg        arr_cfg;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(ss_cfg)::get(this, "", "cfg", cfg))
        `uvm_fatal("SS_ENV", "ss_cfg not found")
      uvm_config_db#(ss_cfg)::set(this, "wr_agent*", "cfg", cfg);
      uvm_config_db#(ss_cfg)::set(this, "rd_agent*", "cfg", cfg);
      wr_agent = ss_wr_agent::type_id::create("wr_agent", this);
      rd_agent = ss_rd_agent::type_id::create("rd_agent", this);
      sb       = mm_scoreboard::type_id::create("sb", this);
      cov      = mm_coverage::type_id::create("cov", this);
      sb.rpt_id      = "SS_E2E";
      sb.exp_latency = -1;                   // two clock domains: latency is reported, not fixed
      sb.print_first = cfg.print_first;
      cov.rpt_id     = "SS_COV";
      cov.ss_mode    = 1;
      cov.cvif       = cfg.cvif;
      if (cfg.has_arr_mon) begin
        arr_cfg             = sa_cfg::type_id::create("arr_cfg");
        arr_cfg.vif         = cfg.arr_vif;
        arr_cfg.is_active   = UVM_PASSIVE;
        arr_cfg.rpt_id      = "SS_ARR";
        arr_cfg.exp_latency = cfg.arr_latency;
        arr_cfg.print_first = 0;
        uvm_config_db#(sa_cfg)::set(this, "arr_env", "cfg", arr_cfg);
        arr_env = sa_env::type_id::create("arr_env", this);
      end
    endfunction

    function void connect_phase(uvm_phase phase);
      super.connect_phase(phase);
      wr_agent.mon.ap.connect(sb.in_imp);
      wr_agent.mon.ap.connect(cov.analysis_export);
      rd_agent.mon.ap.connect(sb.out_imp);
    endfunction

    task reset_dut();
      ss_vif_t  v;
      clk_vif_t c;
      v = cfg.vif;
      c = cfg.cvif;
      v.rst_n       <= 1'b0;
      v.M_minus_one <= 8'(cfg.m - 1);
      repeat (4) @(posedge c.sys_clk);
      repeat (4) @(posedge c.sr_clk);
      v.rst_n <= 1'b1;
      repeat (4) @(posedge c.sys_clk);
      repeat (4) @(posedge c.sr_clk);
      `uvm_info("SS_ENV", $sformatf("reset released (M=%0d, sys_clk %0.2f ns, sr_clk %0.2f ns)",
                                    cfg.m, cfg.sys_ns, cfg.sr_ns), UVM_LOW)
    endtask

    // program a new M (call only while the sub-system is idle)
    task set_m(int unsigned m);
      ss_vif_t  v;
      clk_vif_t c;
      v = cfg.vif;
      c = cfg.cvif;
      cfg.m = m;
      v.M_minus_one <= 8'(m - 1);
      repeat (4) @(posedge c.sr_clk);        // 2-flop synchronizer inside the DUT
      repeat (2) @(posedge c.sys_clk);
    endtask

    // change the sys_clk / sr_clk relationship (call only while idle)
    task set_clocks(real sys_ns, real sr_ns);
      clk_vif_t c;
      c = cfg.cvif;
      cfg.sys_ns = sys_ns;
      cfg.sr_ns  = sr_ns;
      c.set_ss_clks(sys_ns, sr_ns, 0.0);
      repeat (4) @(posedge c.sr_clk);
      repeat (4) @(posedge c.sys_clk);
    endtask

    // wait until every written matrix pair has been read back and checked
    task wait_drain(int unsigned max_cycles = 100000);
      clk_vif_t    c;
      int unsigned n;
      c = cfg.cvif;
      n = 0;
      repeat (8) @(posedge c.sys_clk);
      while ((sb.pending() > 0 || (arr_env != null && arr_env.sb.pending() > 0)) && n < max_cycles) begin
        @(posedge c.sys_clk);
        n++;
      end
      if (sb.pending() > 0)
        `uvm_error("SS_ENV", $sformatf("drain timeout: %0d matrices still pending", sb.pending()))
      repeat (4) @(posedge c.sr_clk);
    endtask
  endclass

  //============================================================================
  // Tests
  //============================================================================
  class mm_base_test extends uvm_test;
    `uvm_component_utils(mm_base_test)
    bit       use_sa = 0;
    bit       use_ss = 0;
    sa_cfg    sa_c;
    ss_cfg    ss_c;
    sa_env    sa_e;
    ss_env    ss_e;
    clk_vif_t cvif;
    ss_rd_seq rd_seq;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    // hook for derived tests: select the levels and tune the configuration
    virtual function void setup_test();
    endfunction

    function void build_phase(uvm_phase phase);
      uvm_root top;
      super.build_phase(phase);
      if (!uvm_config_db#(clk_vif_t)::get(this, "", "clk_vif", cvif))
        `uvm_fatal("TEST", "clk_vif not found")
      sa_c = sa_cfg::type_id::create("sa_c");
      ss_c = ss_cfg::type_id::create("ss_c");
      if (!uvm_config_db#(sa_vif_t)::get(this, "", "sa_vif", sa_c.vif))
        `uvm_fatal("TEST", "sa_vif not found")
      if (!uvm_config_db#(ss_vif_t)::get(this, "", "ss_vif", ss_c.vif))
        `uvm_fatal("TEST", "ss_vif not found")
      if (!uvm_config_db#(sa_vif_t)::get(this, "", "ss_arr_vif", ss_c.arr_vif))
        `uvm_fatal("TEST", "ss_arr_vif not found")
      ss_c.cvif = cvif;
      setup_test();
      apply_plusargs();
      if (use_sa) begin
        uvm_config_db#(sa_cfg)::set(this, "sa_e", "cfg", sa_c);
        sa_e = sa_env::type_id::create("sa_e", this);
      end
      if (use_ss) begin
        uvm_config_db#(ss_cfg)::set(this, "ss_e", "cfg", ss_c);
        ss_e = ss_env::type_id::create("ss_e", this);
      end
      top = uvm_root::get();
      top.set_timeout(50ms, 1);
    endfunction

    // optional run-time overrides
    function void apply_plusargs();
      real r;
      int  v;
      if ($value$plusargs("SYS_CLK_NS=%f", r)) ss_c.sys_ns = r;
      if ($value$plusargs("SR_CLK_NS=%f",  r)) ss_c.sr_ns  = r;
      if ($value$plusargs("SA_CLK_NS=%f",  r)) sa_c.clk_ns = r;
      if ($value$plusargs("SS_M=%d", v)) begin
        if (v >= 1 && v <= MAX_M) ss_c.m = v;
        else `uvm_error("TEST", $sformatf("+SS_M=%0d is outside 1..%0d", v, MAX_M))
      end
      if ($value$plusargs("SA_LATENCY=%d", v)) begin
        sa_c.exp_latency = v;
        ss_c.arr_latency = v;
      end
      if ($value$plusargs("SB_PRINT=%d", v)) begin
        sa_c.print_first = v;
        ss_c.print_first = v;
      end
      if ($value$plusargs("RD_LATENCY=%d", v)) begin
        ss_c.rd_latency    = (v == 0) ? 0 : 1;
        ss_c.vif.dout_fwft = (v == 0);
      end
      if ($test$plusargs("NO_ARR_MON")) ss_c.has_arr_mon = 0;
    endfunction

    function void end_of_elaboration_phase(uvm_phase phase);
      uvm_root top;
      super.end_of_elaboration_phase(phase);
      `uvm_info("TEST", $sformatf("DUT parameters: N=%0d  DIN_WIDTH=%0d  BUS_WIDTH=%0d  | module level %s, sub-system level %s",
                                  N, DW, BW, use_sa ? "ON" : "off", use_ss ? "ON" : "off"), UVM_NONE)
      if (uvm_report_enabled(UVM_LOW, UVM_INFO, "TOPO")) begin
        top = uvm_root::get();
        top.print_topology();
      end
    endfunction

    task run_phase(uvm_phase phase);
      phase.raise_objection(this);
      fork
        if (use_sa) begin
          cvif.start_sa_clk(sa_c.clk_ns);
          sa_e.reset_dut();
          run_sa();
          sa_e.wait_drain();
        end
        if (use_ss) begin
          cvif.set_ss_clks(ss_c.sys_ns, ss_c.sr_ns, ss_c.sr_phase_ns);
          ss_e.reset_dut();
          start_reader();
          run_ss();
          ss_e.wait_drain();
        end
      join
      phase.drop_objection(this);
    endtask

    virtual task run_sa();
    endtask

    virtual task run_ss();
    endtask

    // background reader on the output FIFO (random throttling)
    virtual task start_reader();
      rd_seq = ss_rd_seq::type_id::create("rd_seq");
      fork
        rd_seq.start(ss_e.rd_agent.sqr);
      join_none
    endtask

    task run_seq(mm_sqr_t sqr, int unsigned n, int min_m, int max_m,
                 int gap_hi = 8, int b2b_w = 60,
                 bit fix = 0, pat_e pa = PAT_RANDOM, pat_e pb = PAT_RANDOM);
      mm_base_seq s;
      s = mm_base_seq::type_id::create("s");
      s.num_txn   = n;
      s.min_m     = min_m;
      s.max_m     = max_m;
      s.gap_hi    = gap_hi;
      s.b2b_w     = b2b_w;
      s.fix_pat   = fix;
      s.pat_a_fix = pa;
      s.pat_b_fix = pb;
      s.start(sqr);
    endtask

    task run_directed(mm_sqr_t sqr, int unsigned m);
      mm_directed_seq d;
      d = mm_directed_seq::type_id::create("d");
      d.dm = m;
      d.start(sqr);
    endtask

    // one sub-system batch: program M and clocks while idle, stream, drain
    task ss_batch(int unsigned m, real sys_ns, real sr_ns, int unsigned n);
      ss_e.wait_drain();                                       // M / clocks change only when idle
      ss_e.set_m(m);
      ss_e.set_clocks(sys_ns, sr_ns);
      `uvm_info("TEST", $sformatf("sub-system batch: M=%0d  sys_clk %0.2f ns  sr_clk %0.2f ns  (%0d matrices)",
                                  m, sys_ns, sr_ns, n), UVM_LOW)
      run_seq(ss_e.wr_agent.sqr, n, m, m, 6, 60);
      ss_e.wait_drain();
    endtask

    function void check_phase(uvm_phase phase);
      super.check_phase(phase);
      if (use_sa && sa_e.sb.n_match == 0)
        `uvm_error("TEST", "module level: no matrix was checked")
      if (use_ss && ss_e.sb.n_match == 0)
        `uvm_error("TEST", "sub-system level: no matrix was checked")
    endfunction

    function void report_phase(uvm_phase phase);
      uvm_report_server rs;
      int    n_err;
      int    n_fat;
      string s;
      super.report_phase(phase);
      rs    = uvm_report_server::get_server();
      n_err = rs.get_severity_count(UVM_ERROR);
      n_fat = rs.get_severity_count(UVM_FATAL);
      s = "\n==========================================================================";
      s = {s, $sformatf("\n  TEST %s : %s   (UVM_ERROR=%0d  UVM_FATAL=%0d)", get_type_name(),
                        (n_err == 0 && n_fat == 0) ? "PASSED" : "FAILED", n_err, n_fat)};
      s = {s, $sformatf("\n  N=%0d  DIN_WIDTH=%0d  BUS_WIDTH=%0d", N, DW, BW)};
      if (use_sa) s = {s, "\n  ", sa_e.sb.summary()};
      if (use_ss) begin
        s = {s, "\n  ", ss_e.sb.summary()};
        if (ss_e.arr_env != null) s = {s, "\n  ", ss_e.arr_env.sb.summary()};
        s = {s, $sformatf("\n  SS bus  writes %0d (back-pressure cycles %0d), reads %0d",
                          ss_e.wr_agent.mon.n_wr, ss_e.wr_agent.mon.n_bp, ss_e.rd_agent.mon.n_rd)};
      end
      s = {s, "\n=========================================================================="};
      `uvm_info("RESULT", s, UVM_NONE)
    endfunction
  endclass

  //----------------------------------------------------------------------------
  // Module-level tests
  //----------------------------------------------------------------------------
  class sa_smoke_test extends mm_base_test;
    `uvm_component_utils(sa_smoke_test)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void setup_test();
      use_sa = 1;
    endfunction
    task run_sa();
      run_directed(sa_e.agent.sqr, N);
    endtask
  endclass

  class sa_rand_test extends mm_base_test;
    `uvm_component_utils(sa_rand_test)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void setup_test();
      use_sa = 1;
    endfunction
    task run_sa();
      run_seq(sa_e.agent.sqr, 300, 1, 3*N, 8, 60);        // random M per matrix, random gaps
    endtask
  endclass

  class sa_corner_test extends mm_base_test;
    `uvm_component_utils(sa_corner_test)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void setup_test();
      use_sa = 1;
    endfunction
    task run_sa();
      run_seq(sa_e.agent.sqr,  3, MAX_M - 60, MAX_M, 0, 50, 1, PAT_MAX_NEG, PAT_MAX_NEG); // wrap, positive
      run_seq(sa_e.agent.sqr,  3, MAX_M - 60, MAX_M, 0, 50, 1, PAT_MAX_POS, PAT_MAX_NEG); // wrap, negative
      run_seq(sa_e.agent.sqr,  3, MAX_M - 60, MAX_M, 0, 50, 1, PAT_CORNER,  PAT_CORNER);
      run_seq(sa_e.agent.sqr,  2, MAX_M, MAX_M,      0, 50, 1, PAT_RANDOM,  PAT_RANDOM);  // M = 256
      run_seq(sa_e.agent.sqr, 12, 1, 1,              0, 50, 1, PAT_CORNER,  PAT_CORNER);  // M = 1 back-to-back (P3)
      run_seq(sa_e.agent.sqr, 20, 1, 3*N,            3, 50, 1, PAT_ALT_EXT, PAT_ALT_EXT);
      run_seq(sa_e.agent.sqr, 20, 1, 3*N,            3, 50, 1, PAT_CORNER,  PAT_CORNER);
      run_seq(sa_e.agent.sqr, 10, 1, 3*N,            3, 50, 1, PAT_ZERO,    PAT_RANDOM);
      run_seq(sa_e.agent.sqr, 10, 1, 3*N,            3, 50, 1, PAT_SPARSE,  PAT_SPARSE);
    endtask
  endclass

  class sa_b2b_test extends mm_base_test;
    `uvm_component_utils(sa_b2b_test)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void setup_test();
      use_sa = 1;
    endfunction
    task run_sa();
      run_seq(sa_e.agent.sqr, 64, N, N, 0, 50);           // M = N, no gaps
    endtask
    function void check_phase(uvm_phase phase);
      super.check_phase(phase);
      if (sa_e.sb.ival_min != N || sa_e.sb.ival_max != N)
        `uvm_error("PERF", $sformatf("expected one C matrix every %0d clocks, observed %0d..%0d",
                                     N, sa_e.sb.ival_min, sa_e.sb.ival_max))
      else
        `uvm_info("PERF", $sformatf("throughput check passed: one C matrix every M=%0d clocks", N), UVM_NONE)
    endfunction
  endclass

  //----------------------------------------------------------------------------
  // Sub-system tests
  //----------------------------------------------------------------------------
  class ss_smoke_test extends mm_base_test;
    `uvm_component_utils(ss_smoke_test)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void setup_test();
      use_ss      = 1;
      ss_c.m      = N;
      ss_c.sys_ns = 10.0;
      ss_c.sr_ns  = 10.0;
    endfunction
    task run_ss();
      run_directed(ss_e.wr_agent.sqr, ss_c.m);
      run_seq(ss_e.wr_agent.sqr, 20, ss_c.m, ss_c.m, 4, 60);
    endtask
  endclass

  class ss_rand_test extends mm_base_test;
    `uvm_component_utils(ss_rand_test)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void setup_test();
      use_ss           = 1;
      ss_c.m           = $urandom_range(2*N + 4, 1);
      ss_c.sys_ns      = $urandom_range(200, 30) / 10.0;     //  3.0 .. 20.0 ns
      ss_c.sr_ns       = $urandom_range(200, 20) / 10.0;     //  2.0 .. 20.0 ns
      ss_c.sr_phase_ns = $urandom_range(100, 0)  / 10.0;
    endfunction
    task run_ss();
      run_seq(ss_e.wr_agent.sqr, 150, ss_c.m, ss_c.m, 6, 60);
    endtask
  endclass

  class ss_clk_sweep_test extends mm_base_test;
    `uvm_component_utils(ss_clk_sweep_test)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void setup_test();
      use_ss = 1;
      ss_c.m = N;
    endfunction
    task run_ss();
      real sys_l [7] = '{10.0, 10.0, 10.0,  4.0, 17.0, 7.3, 6.1};
      real sr_l  [7] = '{10.0,  3.3, 25.0, 13.1,  4.0, 7.3, 9.7};
      foreach (sys_l[i]) ss_batch(ss_c.m, sys_l[i], sr_l[i], 20);
    endtask
  endclass

  class ss_m_sweep_test extends mm_base_test;
    `uvm_component_utils(ss_m_sweep_test)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void setup_test();
      use_ss = 1;
    endfunction
    task run_ss();
      int cand [8];
      int ms [$];
      bit dup;
      cand = '{1, N - 1, N, N + 1, 2*N, 3*N + 1, 64, MAX_M};
      foreach (cand[i]) begin
        dup = (cand[i] < 1 || cand[i] > MAX_M);
        foreach (ms[j]) if (ms[j] == cand[i]) dup = 1;
        if (!dup) ms.push_back(cand[i]);
      end
      foreach (ms[i]) ss_batch(ms[i], 10.0, 7.0, (ms[i] > 32) ? 4 : 10);
    endtask
  endclass

  class ss_backpressure_test extends mm_base_test;
    `uvm_component_utils(ss_backpressure_test)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void setup_test();
      use_ss      = 1;
      ss_c.m      = N;
      ss_c.sys_ns = 5.0;                                       // fast writer
      ss_c.sr_ns  = 10.0;
    endfunction
    task start_reader();                                       // slow, bursty reader
      rd_seq = ss_rd_seq::type_id::create("rd_seq");
      rd_seq.rate_lo   = 5;
      rd_seq.rate_hi   = 30;
      rd_seq.stall_pct = 40;
      rd_seq.stall_max = 300;
      fork
        rd_seq.start(ss_e.rd_agent.sqr);
      join_none
    endtask
    task run_ss();
      run_seq(ss_e.wr_agent.sqr, 60, ss_c.m, ss_c.m, 0, 50);  // writer streams back-to-back
    endtask
    function void check_phase(uvm_phase phase);
      super.check_phase(phase);
      if (ss_e.wr_agent.mon.n_bp == 0)
        `uvm_error("BP", "in_fifo_full back-pressure never happened")
    endfunction
  endclass

  //----------------------------------------------------------------------------
  // Default test: both levels in one run (convenient on EDA Playground)
  //----------------------------------------------------------------------------
  class mm_full_test extends mm_base_test;
    `uvm_component_utils(mm_full_test)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void setup_test();
      use_sa      = 1;
      use_ss      = 1;
      ss_c.m      = N;
      ss_c.sys_ns = 10.0;
      ss_c.sr_ns  = 10.0;
    endfunction

    task run_sa();
      `uvm_info("TEST", "module level: directed matrices", UVM_LOW)
      run_directed(sa_e.agent.sqr, N);
      `uvm_info("TEST", "module level: corner cases (wrap-around, M=1, M=256, zero/sparse data)", UVM_LOW)
      run_seq(sa_e.agent.sqr,  2, MAX_M - 60, MAX_M, 0, 50, 1, PAT_MAX_NEG, PAT_MAX_NEG);
      run_seq(sa_e.agent.sqr,  2, MAX_M - 60, MAX_M, 0, 50, 1, PAT_CORNER,  PAT_CORNER);
      run_seq(sa_e.agent.sqr,  8, 1, 1,              0, 50, 1, PAT_CORNER,  PAT_CORNER);
      run_seq(sa_e.agent.sqr,  4, 2, (N > 2) ? N - 1 : 2, 0, 50, 1, PAT_MAX_NEG, PAT_MAX_NEG); // wrap with M < N
      run_seq(sa_e.agent.sqr, 10, 1, 3*N,            3, 50, 1, PAT_SPARSE,  PAT_ZERO);
      `uvm_info("TEST", "module level: constrained-random stream", UVM_LOW)
      run_seq(sa_e.agent.sqr, 150, 1, 3*N, 8, 60);
    endtask

    task run_ss();
      `uvm_info("TEST", "sub-system: directed matrices, equal clocks", UVM_LOW)
      run_directed(ss_e.wr_agent.sqr, ss_c.m);
      // M sweep combined with a clock-relationship sweep (no reset in between)
      ss_batch(1,      10.0, 10.0, 8);
      ss_batch(N - 1 > 0 ? N - 1 : 2, 10.0, 3.3, 8);           // sr_clk 3x faster
      ss_batch(N,      10.0, 25.0, 8);                         // sys_clk 2.5x faster
      ss_batch(N + 1,   6.4,  9.1, 8);                         // sys_clk faster, ratio 0.70
      ss_batch(2*N,    15.0,  6.0, 8);                         // sr_clk 2.5x faster
      ss_batch(37,     10.0,  6.5, 6);                         // sr_clk faster, ratio 1.54
      ss_batch(MAX_M,   4.0,  9.7, 3);
      // back-pressure: fast writer, slow and bursty reader
      `uvm_info("TEST", "sub-system: back-pressure (fast writer, slow reader)", UVM_LOW)
      ss_e.set_m(N);
      ss_e.set_clocks(5.0, 10.0);
      rd_seq.rate_lo   = 5;
      rd_seq.rate_hi   = 30;
      rd_seq.stall_pct = 40;
      rd_seq.stall_max = 200;
      run_seq(ss_e.wr_agent.sqr, 40, N, N, 0, 50);
      rd_seq.rate_lo   = 60;                                   // let the drain finish quickly
      rd_seq.rate_hi   = 100;
      rd_seq.stall_pct = 0;
    endtask
  endclass

endpackage : mm_tb_pkg

//==============================================================================
// Top level: both DUTs, their interfaces, white-box probe, waveform helpers
//==============================================================================
module tb_top;
  import uvm_pkg::*;
  import mm_tb_pkg::*;

  clk_rst_if                       crif ();
  sa_if #(.DW(DW), .N(N))          sa_vif     (.clk(crif.clk));
  ss_if #(.DW(DW), .N(N), .BW(BW)) ss_vif     (.sys_clk(crif.sys_clk));
  sa_if #(.DW(DW), .N(N))          ss_arr_vif (.clk(crif.sr_clk));   // probe of the embedded array

  //----------------------------------------------------------------------------
  // Module-level DUT
  //----------------------------------------------------------------------------
  systolic_array #(.DIN_WIDTH(DW), .N(N)) u_sa (
    .rst_n     (sa_vif.rst_n),
    .clk       (crif.clk),
    .c_din     (sa_vif.c_din),
    .a_din     (sa_vif.a_din),
    .b_din     (sa_vif.b_din),
    .in_valid  (sa_vif.in_valid),
    .c_dout    (sa_vif.c_dout),
    .out_valid (sa_vif.out_valid)
  );

  //----------------------------------------------------------------------------
  // Sub-system DUT
  //----------------------------------------------------------------------------
  sub_sys #(.DIN_WIDTH(DW), .N(N), .BUS_WIDTH(BW)) u_ss (
    .rst_n          (ss_vif.rst_n),
    .sys_clk        (crif.sys_clk),
    .sr_clk         (crif.sr_clk),
    .M_minus_one    (ss_vif.M_minus_one),
    .din            (ss_vif.din),
    .wr_fifo        (ss_vif.wr_fifo),
    .rd_fifo        (ss_vif.rd_fifo),
    .in_fifo_full   (ss_vif.in_fifo_full),
    .dout           (ss_vif.dout),
    .out_fifo_empty (ss_vif.out_fifo_empty)
  );

  //----------------------------------------------------------------------------
  // White-box probe of the systolic_array inside sub_sys (adapt the path when
  // the real RTL is plugged in). Registered copy: every signal is one sr_clk
  // late, inputs and outputs alike, so data and latency are unchanged. It uses
  // procedural drivers only, because the same interface type is also written
  // through a virtual interface by the active module-level agent.
  //----------------------------------------------------------------------------
  always @(posedge crif.sr_clk) begin
    ss_arr_vif.rst_n     <= u_ss.u_sa.rst_n;
    ss_arr_vif.c_din     <= u_ss.u_sa.c_din;
    ss_arr_vif.a_din     <= u_ss.u_sa.a_din;
    ss_arr_vif.b_din     <= u_ss.u_sa.b_din;
    ss_arr_vif.in_valid  <= u_ss.u_sa.in_valid;
    ss_arr_vif.c_dout    <= u_ss.u_sa.c_dout;
    ss_arr_vif.out_valid <= u_ss.u_sa.out_valid;
  end

  //----------------------------------------------------------------------------
  // Idle values until the UVM environments take over
  //----------------------------------------------------------------------------
  initial begin
    sa_vif.rst_n    = 1'b0;
    sa_vif.in_valid = 1'b0;
    for (int i = 0; i < N; i++) begin
      sa_vif.a_din[i] = '0;
      sa_vif.b_din[i] = '0;
      sa_vif.c_din[i] = '0;
    end
    ss_vif.rst_n       = 1'b0;
    ss_vif.M_minus_one = 8'(N - 1);
    ss_vif.din         = '0;
    ss_vif.wr_fifo     = 1'b0;
    ss_vif.rd_fifo     = 1'b0;
  end

  //----------------------------------------------------------------------------
  // UVM start
  //----------------------------------------------------------------------------
  initial begin
    if (DW < 2 || DW > 31) $fatal(1, "TB_DW (DIN_WIDTH) must be in 2..31, got %0d", DW);
    if (N < 1)             $fatal(1, "TB_N must be >= 1, got %0d", N);
    uvm_config_db#(sa_vif_t)::set(null, "uvm_test_top", "sa_vif",     sa_vif);
    uvm_config_db#(sa_vif_t)::set(null, "uvm_test_top", "ss_arr_vif", ss_arr_vif);
    uvm_config_db#(ss_vif_t)::set(null, "uvm_test_top", "ss_vif",     ss_vif);
    uvm_config_db#(clk_vif_t)::set(null, "uvm_test_top", "clk_vif",   crif);
    run_test("mm_full_test");
  end

  //----------------------------------------------------------------------------
  // Waveforms (EPWave cannot display unpacked arrays: per-element copies below)
  //   default : interface-level signals for the first WAVE_NS ns (20 us)
  //   +WAVE_FULL : whole hierarchy incl. every PE,  +WAVE_NS=0 : no time limit,
  //   +NO_WAVES  : no dump file
  //----------------------------------------------------------------------------
  wire sa_clk       = crif.clk;
  wire sa_rst_n     = sa_vif.rst_n;
  wire sa_in_valid  = sa_vif.in_valid;
  wire sa_out_valid = sa_vif.out_valid;
  wire ss_sys_clk   = crif.sys_clk;
  wire ss_sr_clk    = crif.sr_clk;
  wire ss_rst_n     = ss_vif.rst_n;
  wire ss_wr_fifo   = ss_vif.wr_fifo;
  wire ss_in_full   = ss_vif.in_fifo_full;
  wire ss_rd_fifo   = ss_vif.rd_fifo;
  wire ss_out_empty = ss_vif.out_fifo_empty;
  wire [7:0] ss_m_minus_one = ss_vif.M_minus_one;
  wire ss_arr_in_valid  = u_ss.u_sa.in_valid;                // embedded array, live
  wire ss_arr_out_valid = u_ss.u_sa.out_valid;

  for (genvar g = 0; g < N; g++) begin : W_SA
    wire signed [DW-1:0]   a_din  = sa_vif.a_din[g];         // A[g][k]
    wire signed [DW-1:0]   b_din  = sa_vif.b_din[g];         // B[k][g]
    wire signed [2*DW-1:0] c_dout = sa_vif.c_dout[g];        // C[i][g]
  end

  for (genvar g = 0; g < N; g++) begin : W_SS
    wire signed [DW-1:0]   din_a  = ss_vif.din[g*DW +: DW];
    wire signed [DW-1:0]   din_b  = ss_vif.din[N*DW + g*DW +: DW];
    wire signed [2*DW-1:0] dout_c = ss_vif.dout[g*2*DW +: 2*DW];
    wire signed [DW-1:0]   arr_a  = u_ss.u_sa.a_din[g];      // embedded array, live
    wire signed [DW-1:0]   arr_b  = u_ss.u_sa.b_din[g];
    wire signed [2*DW-1:0] arr_c  = u_ss.u_sa.c_dout[g];
  end

  initial begin
    int unsigned wave_ns;
    wave_ns = 20000;
    void'($value$plusargs("WAVE_NS=%d", wave_ns));
    if (!$test$plusargs("NO_WAVES")) begin
      $dumpfile("dump.vcd");
      if ($test$plusargs("WAVE_FULL")) $dumpvars(0, tb_top);
      else                             $dumpvars(2, tb_top);
      if (wave_ns != 0) begin
        #(wave_ns * 1ns);
        $dumpoff;
      end
    end
  end
endmodule
