`timescale 1ns/10ps
module GSIM ( clk, reset, in_en, b_in, out_valid, x_out);
input   clk ;
input   reset ;
input   in_en;
output  reg out_valid;
input   [15:0]  b_in;
output  reg [31:0]  x_out;
/*------Parameter------*/
localparam integer M_ITER = 80; // 與下方 iter_ctrl 的 parameter 預設一致；改迭代次數時請一併改 iter_ctrl 例化
localparam integer TOTAL = M_ITER * 16;
localparam IDLE = 3'd0;
localparam READ_B = 3'd1;    // in_en=1, 讀進 b_in, 存b[0]~b[15]
localparam INIT_X = 3'd2;    // X的初始值
localparam ITERATE = 3'd3;   // 做 M 次 Gauss-Seidel
localparam WRITE_OUT = 3'd4; // out_valid=1, 依序輸出x[0]~x[15]

// Iterations are handled inside iter_ctrl

/*------Wires and Registers------*/
reg [2:0] state, next_state;
reg [15:0] x_reg;
reg [3:0] out_idx;
reg [3:0] b_idx;
reg signed [15:0] b_mem [0:15];
//reg signed [31:0] x_mem [0:15];
reg [10:0] total_issued;
reg [10:0] total_commit;


// ---- debug taps: expose b_mem as normal wires (Verilog-2001 friendly) ----
wire signed [15:0] b_mem0_dbg;
wire signed [15:0] b_mem1_dbg;
wire signed [15:0] b_mem2_dbg;
wire signed [15:0] b_mem3_dbg;
wire signed [15:0] b_mem4_dbg;
wire signed [15:0] b_mem5_dbg;
wire signed [15:0] b_mem6_dbg;
wire signed [15:0] b_mem7_dbg;
wire signed [15:0] b_mem8_dbg;
wire signed [15:0] b_mem9_dbg;
wire signed [15:0] b_mem10_dbg;
wire signed [15:0] b_mem11_dbg;
wire signed [15:0] b_mem12_dbg;
wire signed [15:0] b_mem13_dbg;
wire signed [15:0] b_mem14_dbg;
wire signed [15:0] b_mem15_dbg;
assign b_mem0_dbg  = b_mem[0];
assign b_mem1_dbg  = b_mem[1];
assign b_mem2_dbg  = b_mem[2];
assign b_mem3_dbg  = b_mem[3];
assign b_mem4_dbg  = b_mem[4];
assign b_mem5_dbg  = b_mem[5];
assign b_mem6_dbg  = b_mem[6];
assign b_mem7_dbg  = b_mem[7];
assign b_mem8_dbg  = b_mem[8];
assign b_mem9_dbg  = b_mem[9];
assign b_mem10_dbg = b_mem[10];
assign b_mem11_dbg = b_mem[11];
assign b_mem12_dbg = b_mem[12];
assign b_mem13_dbg = b_mem[13];
assign b_mem14_dbg = b_mem[14];
assign b_mem15_dbg = b_mem[15];

// ---- iter_ctrl integration ----
// IMPORTANT: `iter_ctrl` samples `start` on posedge. If we generate `iter_start`
// with a reg assigned in this same always block, it can be missed (NBA update).
// So generate a combinational pulse that is HIGH *before* the posedge where the
// last b is latched.
wire iter_start;
wire iter_busy, iter_done;
wire signed [31:0] x0, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14, x15;

// iter_start：在 ITERATE 狀態維持為 1，讓 iter_ctrl 能看見 start（依 iter_ctrl 內邏輯可再改成脈波）
assign iter_start = (state == READ_B) && in_en && (b_idx == 4'd15);

iter_ctrl #(
    .M_ITER(M_ITER)
) u_iter_ctrl (
    .clk(clk),
    .reset(reset),
    .start(iter_start),
    .busy(iter_busy),
    .done(iter_done),
    .b0(b_mem[0]),   .b1(b_mem[1]),   .b2(b_mem[2]),   .b3(b_mem[3]),
    .b4(b_mem[4]),   .b5(b_mem[5]),   .b6(b_mem[6]),   .b7(b_mem[7]),
    .b8(b_mem[8]),   .b9(b_mem[9]),   .b10(b_mem[10]), .b11(b_mem[11]),
    .b12(b_mem[12]), .b13(b_mem[13]), .b14(b_mem[14]), .b15(b_mem[15]),
    .x0(x0),   .x1(x1),   .x2(x2),   .x3(x3),
    .x4(x4),   .x5(x5),   .x6(x6),   .x7(x7),
    .x8(x8),   .x9(x9),   .x10(x10), .x11(x11),
    .x12(x12), .x13(x13), .x14(x14), .x15(x15)
);
/*------Combanational Block------*/
//FSM
always @(*) begin
    next_state = IDLE;
    case (state)
        IDLE: next_state = in_en ? READ_B : IDLE;
        READ_B: begin
            // Only advance once we actually capture the last sample (b_idx==15 with in_en=1).
            // This avoids accidentally latching X into b_mem[15] when the host has already
            // deasserted in_en (and may no longer be driving b_in).
            if (in_en && (b_idx == 4'd15)) next_state = INIT_X;
            else if (!in_en) next_state = IDLE;
            else next_state = READ_B;
        end
        INIT_X: next_state = ITERATE; // iter_start is generated in READ_B when b[15] is latched
        ITERATE: next_state = iter_done ? WRITE_OUT : ITERATE;
        WRITE_OUT: next_state = (out_idx == 4'd15) ? IDLE : WRITE_OUT;
        default: next_state = IDLE;
    endcase
end

/*------Sequential Block------*/
//FSM state reg
always @(posedge clk or posedge reset) begin
    if (reset) begin
        state <= IDLE;
    end
    else begin
        state <= next_state;
    end
end

// b_in 前面16個cycle把b_in存進x_reg, in_en=1開始讀資料, 讀完後in_en=0
integer i;
always @(posedge clk or posedge reset) begin
    //Initialize
    if (reset) begin
        x_reg <= 16'd0;
        out_idx <= 4'd0;
        b_idx <= 4'd0;
        out_valid <= 1'b0;
        x_out <= 32'd0;
        for (i = 0; i < 16; i = i + 1) begin
            b_mem[i] <= 16'd0;
        end
    end 
    else begin
        case(state)
        IDLE: begin
            out_valid <= 1'b0;
            x_out <= 32'd0;
            out_idx <= 4'd0;
            b_idx <= 4'd0;
            // ← 加這段：IDLE→READ_B 轉移的同一個 posedge 就捕 b[0]
            if(in_en)begin
              b_mem[0] <= b_in;
              b_idx <= 4'd1; //下一個 READ_B cycle從 idx=1開始
            end
        end
        READ_B: begin
            out_valid <= 1'b0;
            
            // Only latch b_in when host asserts in_en.
            // If in_en drops right after the 16th transfer, b_in can become X and would
            // otherwise poison b_mem[15] and propagate X into the solver.
            if (in_en) begin
                b_mem[b_idx] <= b_in;
                x_reg <= b_in;
                if (b_idx != 4'd15) b_idx <= b_idx + 4'd1;
            end
        end
        INIT_X: begin
            out_valid <= 1'b0;
        end
        ITERATE: begin
            out_valid <= 1'b0;
        end
        WRITE_OUT: begin
            out_valid <= 1'b1;
            case (out_idx)
                4'd0:  x_out <= x0;
                4'd1:  x_out <= x1;
                4'd2:  x_out <= x2;
                4'd3:  x_out <= x3;
                4'd4:  x_out <= x4;
                4'd5:  x_out <= x5;
                4'd6:  x_out <= x6;
                4'd7:  x_out <= x7;
                4'd8:  x_out <= x8;
                4'd9:  x_out <= x9;
                4'd10: x_out <= x10;
                4'd11: x_out <= x11;
                4'd12: x_out <= x12;
                4'd13: x_out <= x13;
                4'd14: x_out <= x14;
                default: x_out <= x15;
            endcase
            out_idx <= out_idx + 4'd1;
        end
        default: begin
            out_valid <= 1'b0;
            x_out <= 32'd0;
        end
        endcase
    end
end

endmodule

// -----------------------------------------------------------------------------
// Integrated modules (so TB can compile GSIM.v only)
// -----------------------------------------------------------------------------

// Generic compute for any row i (boundary handled by feeding 0s):
// x_i = ( b_i
//         + 13*(x_{i-1}+x_{i+1})
//         -  6*(x_{i-2}+x_{i+2})
//         +  1*(x_{i-3}+x_{i+3}) ) / 20
//
// Formats:
// - bi: 16-bit signed integer (2's complement)
// - x_*: 32-bit signed Q16.16
// - x_out: 32-bit signed Q16.16
module core_xi (
  input wire clk,
  input wire reset,
  input wire in_valid,
  input  wire signed [15:0] bi,
  input  wire signed [31:0] x_im3,
  input  wire signed [31:0] x_im2,
  input  wire signed [31:0] x_im1,
  input  wire signed [31:0] x_ip1,
  input  wire signed [31:0] x_ip2,
  input  wire signed [31:0] x_ip3,
  output reg out_valid,
  output reg signed [31:0] x_out
);

  // True 3-stage pipeline (accepts a new input every cycle).
  reg v1, v2, v3;
  reg signed [47:0] term_p13_r;
  reg signed [47:0] term_m6_r;
  reg signed [47:0] term_p1_r;
  reg signed [47:0] numer_r;
  reg signed [47:0] bi_q16_r;
  // Sign-extend to wider bitwidth for safe shift-add and accumulation.
  wire signed [47:0] bi_q16  = {{32{bi[15]}},  bi}  <<< 16; // int -> Q16.16 in 48b
  wire signed [47:0] xim3_48 = {{16{x_im3[31]}}, x_im3};
  wire signed [47:0] xim2_48 = {{16{x_im2[31]}}, x_im2};
  wire signed [47:0] xim1_48 = {{16{x_im1[31]}}, x_im1};
  wire signed [47:0] xip1_48 = {{16{x_ip1[31]}}, x_ip1};
  wire signed [47:0] xip2_48 = {{16{x_ip2[31]}}, x_ip2};
  wire signed [47:0] xip3_48 = {{16{x_ip3[31]}}, x_ip3};

  // Constant multipliers (shift-add).
  wire signed [47:0] wire_sum_p13 = (xim1_48 + xip1_48);
  wire signed [47:0] wire_sum_m6  = (xim2_48 + xip2_48);
  wire signed [47:0] wire_sum_p1  = (xim3_48 + xip3_48);

  wire signed [47:0] wire_term_p13 = wire_sum_p13 + (wire_sum_p13 <<< 2) + (wire_sum_p13 <<< 3); // *13
  wire signed [47:0] wire_term_m6  = -((wire_sum_m6  <<< 1) + (wire_sum_m6  <<< 2));        // *(-6)
  wire signed [47:0] wire_term_p1  = wire_sum_p1;

  wire signed [47:0] wire_numer = bi_q16_r + term_p13_r + term_m6_r + term_p1_r; // Q16.16

  // Exact divide by 20, rewritten as (/4) then (/5).
  // This is bit-exact for signed truncation-toward-zero division and often synthesizes smaller:
  // - /4 can become a cheap shift with sign handling
  // - /5 constant division is simpler than /20 in some libraries/tools
  wire signed [47:0] numer_div4 = numer_r / 48'sd4;
  wire signed [47:0] wire_div20 = numer_div4 / 48'sd5;

//pipeline
  always @(posedge clk or posedge reset) begin
    if (reset) begin
      v1 <= 1'b0;
      v2 <= 1'b0;
      v3 <= 1'b0;
      term_p13_r <= 48'd0;
      term_m6_r  <= 48'd0;
      term_p1_r  <= 48'd0;
      numer_r    <= 48'd0;
      bi_q16_r   <= 48'd0;
      x_out <= 32'd0;
      out_valid <= 1'b0;
    end else begin
      // valid pipeline
      v3 <= v2;
      v2 <= v1;
      v1 <= in_valid;

      // stage1: capture constant-mult terms
      if (in_valid) begin
        bi_q16_r   <= bi_q16;
        term_p13_r <= wire_term_p13;
        term_m6_r  <= wire_term_m6;
        term_p1_r  <= wire_term_p1;
      end

      // stage2: capture numerator
      if (v1) begin
        numer_r <= wire_numer;
      end

      // stage3: output
      out_valid <= v2;
      if (v2) begin
        x_out <= wire_div20[31:0];
      end
    end
  end

endmodule

// Iteration controller (single-core baseline).
module iter_ctrl #(
  parameter integer M_ITER = 80
) (
  input  wire              clk,
  input  wire              reset,     // async high

  input  wire              start,     // pulse high to begin iterations (after b_mem loaded)
  output reg               busy,
  output reg               done,      // pulse high for 1 cycle when finished

  // b vector (loaded by GSIM)
  input  wire signed [15:0] b0,
  input  wire signed [15:0] b1,
  input  wire signed [15:0] b2,
  input  wire signed [15:0] b3,
  input  wire signed [15:0] b4,
  input  wire signed [15:0] b5,
  input  wire signed [15:0] b6,
  input  wire signed [15:0] b7,
  input  wire signed [15:0] b8,
  input  wire signed [15:0] b9,
  input  wire signed [15:0] b10,
  input  wire signed [15:0] b11,
  input  wire signed [15:0] b12,
  input  wire signed [15:0] b13,
  input  wire signed [15:0] b14,
  input  wire signed [15:0] b15,

  // Final x vector out (Q16.16)
  output reg signed [31:0] x0,
  output reg signed [31:0] x1,
  output reg signed [31:0] x2,
  output reg signed [31:0] x3,
  output reg signed [31:0] x4,
  output reg signed [31:0] x5,
  output reg signed [31:0] x6,
  output reg signed [31:0] x7,
  output reg signed [31:0] x8,
  output reg signed [31:0] x9,
  output reg signed [31:0] x10,
  output reg signed [31:0] x11,
  output reg signed [31:0] x12,
  output reg signed [31:0] x13,
  output reg signed [31:0] x14,
  output reg signed [31:0] x15
);

  //Internal storage
  reg signed [31:0] x_mem [0:15];
  // ---- Sweep counters ----
  reg [15:0] iter_cnt; // 0..M_ITER-1（需 ≥ ceil(log2(M_ITER))，M_ITER>64 時 6bit 會溢位永遠等不到 done）
  reg [1:0] group;    // 0..3
  reg [1:0] slot;     // 0..3
  reg [3:0] cur_idx;  // 0..15

  // FSM
  localparam [1:0]
    IDLE = 2'b00,
    INIT = 2'b01,
    RUN = 2'b10,
    DONE = 2'b11;
  localparam integer TOTAL = M_ITER * 16;
  reg [1:0] state;
  reg [10:0] total_issued;     
  reg [10:0] total_commit; 
  // single-core: done pulses directly in DONE state

  // ---- Helpers ----
  function automatic signed [15:0] pick_b(input [3:0] idx);
    begin
      case (idx)
        4'd0:  pick_b = b0;
        4'd1:  pick_b = b1;
        4'd2:  pick_b = b2;
        4'd3:  pick_b = b3;
        4'd4:  pick_b = b4;
        4'd5:  pick_b = b5;
        4'd6:  pick_b = b6;
        4'd7:  pick_b = b7;
        4'd8:  pick_b = b8;
        4'd9:  pick_b = b9;
        4'd10: pick_b = b10;
        4'd11: pick_b = b11;
        4'd12: pick_b = b12;
        4'd13: pick_b = b13;
        4'd14: pick_b = b14;
        default: pick_b = b15;
      endcase
    end
  endfunction

  // ---------------------------------------------------------------------------
  // Single-core issue (A-ranked reference): launch one index per cycle using
  // the proven order 0,4,8,12,1,5,9,13,2,6,10,14,3,7,11,15 (next_idx={slot,group}).
  // ---------------------------------------------------------------------------

  // Pipeline/issue bookkeeping regs.
  reg [3:0] idx_pipe0, idx_pipe1, idx_pipe2;
  reg        v_pipe0, v_pipe1, v_pipe2;

  // Launch decision for this cycle (must be stable BEFORE posedge).
  wire [3:0] next_idx = {slot, group};
  wire       launch   = (state == RUN) && (total_issued < TOTAL);
  // Core inputs (idx = next_idx)
  wire signed [15:0] bi_cur = pick_b(next_idx);
  reg signed [31:0] xim3, xim2, xim1, xip1, xip2, xip3;
  wire              core_in_valid  = launch;
  wire              core_out_valid;
  wire signed [31:0] x_calc;
  core_xi u_core (
    .clk    (clk),
    .reset  (reset),
    .in_valid(core_in_valid),
    .out_valid(core_out_valid),
    .bi   (bi_cur),
    .x_im3(xim3),
    .x_im2(xim2),
    .x_im1(xim1),
    .x_ip1(xip1),
    .x_ip2(xip2),
    .x_ip3(xip3),
    .x_out(x_calc)
  );

  // Neighbor pick using updated/x_new vs x_old (avoid integer index).
  always @(*) begin
    xim3 = (next_idx < 4'd3)  ? 32'sd0 : x_mem[next_idx-4'd3];
    xim2 = (next_idx < 4'd2)  ? 32'sd0 : x_mem[next_idx-4'd2];
    xim1 = (next_idx < 4'd1)  ? 32'sd0 : x_mem[next_idx-4'd1];
    xip1 = (next_idx > 4'd14) ? 32'sd0 : x_mem[next_idx+4'd1];
    xip2 = (next_idx > 4'd13) ? 32'sd0 : x_mem[next_idx+4'd2];
    xip3 = (next_idx > 4'd12) ? 32'sd0 : x_mem[next_idx+4'd3];
end
  integer k;
  always @(posedge clk or posedge reset) begin
    if (reset) begin
      state <= IDLE;
      v_pipe0 <= 1'b0; v_pipe1 <= 1'b0; v_pipe2 <= 1'b0;
      idx_pipe0 <= 4'd0; idx_pipe1 <= 4'd0; idx_pipe2 <= 4'd0;
      busy <= 1'b0;
      done <= 1'b0;
      total_issued <= 11'd0;
      total_commit <= 11'd0;
      group <= 2'd0;
      slot <= 2'd0;
      for (k = 0; k < 16; k = k + 1) begin
        x_mem[k] <= 32'd0;
      end
      x0 <= 32'sd0;  x1 <= 32'sd0;  x2 <= 32'sd0;  x3 <= 32'sd0;
      x4 <= 32'sd0;  x5 <= 32'sd0;  x6 <= 32'sd0;  x7 <= 32'sd0;
      x8 <= 32'sd0;  x9 <= 32'sd0;  x10 <= 32'sd0; x11 <= 32'sd0;
      x12 <= 32'sd0; x13 <= 32'sd0; x14 <= 32'sd0; x15 <= 32'sd0;
    end else begin
      // defaults
      done <= 1'b0;

      // ---- pipeline bookkeeping: shift + conditional load ----
      v_pipe2   <= v_pipe1;
      idx_pipe2 <= idx_pipe1;
      v_pipe1   <= v_pipe0;
      idx_pipe1 <= idx_pipe0;
      if (launch) begin
        v_pipe0   <= 1'b1;
        idx_pipe0 <= next_idx;
      end else begin
        v_pipe0   <= 1'b0;
      end
      if ((state == RUN) && core_out_valid && v_pipe2) begin
        x_mem[idx_pipe2] <= x_calc;
        total_commit <= total_commit + 11'd1;
      end

      case (state)
        IDLE: begin
          busy <= 1'b0;
          if (start) begin
            state <= RUN;
            total_issued <= 11'd0;
            total_commit <= 11'd0;
          end
        end
        RUN: begin 
          busy <= 1'b1;
          if (launch) begin
            total_issued <= total_issued + 11'd1;
            if(slot == 2'd3) begin
              slot <= 2'd0;
              group <= (group == 2'd3) ? 2'd0 : group + 2'd1;
            end 
            else begin
              slot <= slot +2'd1;
            end
          end
          if(total_commit == TOTAL) begin
            state <= DONE;
          end
        end
        DONE: begin
            state <= IDLE;
            x0  <= x_mem[0];   x1  <= x_mem[1];   x2  <= x_mem[2];   x3  <= x_mem[3];
            x4  <= x_mem[4];   x5  <= x_mem[5];   x6  <= x_mem[6];   x7  <= x_mem[7];
            x8  <= x_mem[8];   x9  <= x_mem[9];   x10 <= x_mem[10];  x11 <= x_mem[11];
            x12 <= x_mem[12];  x13 <= x_mem[13];  x14 <= x_mem[14];  x15 <= x_mem[15];
            busy <= 1'b0;
            done <= 1'b1;
        end
        default: state <= IDLE;
      endcase
    end
  end

endmodule
