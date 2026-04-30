`timescale 1ns/10ps

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

  // True 3-stage pipeline (accepts new input every cycle)
  reg v1, v2, v3;
  reg signed [47:0] term_p13_r;
  reg signed [47:0] term_m6_r;
  reg signed [47:0] term_p1_r;
  reg signed [47:0] numer_r;

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

  wire signed [47:0] wire_numer = bi_q16 + term_p13_r + term_m6_r + term_p1_r; // Q16.16

  // First version: exact constant divide by 20 (synthesizable in most flows).
  // Truncates toward zero (Verilog signed division behavior).
  wire signed [47:0] wire_div20 = numer_r / 48'sd20;

  always @(posedge clk or posedge reset) begin
    if (reset) begin
      v1 <= 1'b0;
      v2 <= 1'b0;
      v3 <= 1'b0;
      term_p13_r <= 48'sd0;
      term_m6_r  <= 48'sd0;
      term_p1_r  <= 48'sd0;
      numer_r    <= 48'sd0;
      x_out <= 32'sd0;
      out_valid <= 1'b0;
    end else begin
      // valid pipeline
      v3 <= v2;
      v2 <= v1;
      v1 <= in_valid;

      // stage1: capture constant-mult terms
      if (in_valid) begin
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

