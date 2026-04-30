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

  reg [1:0] state;
  reg signed [47:0] term_p13;
  reg signed [47:0] term_m6;
  reg signed [47:0] term_p1;
  reg signed [47:0] numer;
  // div20 register not needed; use wire_div20 directly in stage3

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

  wire signed [47:0] wire_numer = bi_q16 + term_p13 + term_m6 + term_p1; // Q16.16

  // First version: exact constant divide by 20 (synthesizable in most flows).
  // Truncates toward zero (Verilog signed division behavior).
  wire signed [47:0] wire_div20 = numer / 48'sd20;

  always @(posedge clk or posedge reset) begin
    if (reset) begin
      state <= 2'b00;
      term_p13 <= 48'sd0;
      term_m6  <= 48'sd0;
      term_p1  <= 48'sd0;
      numer    <= 48'sd0;
      x_out <= 32'b0;
      out_valid <= 1'b0;
    end else begin
      case (state)
        2'b00: begin
          out_valid <= 1'b0;
          if (in_valid) begin
            state <= 2'b01;
            term_p13 <= wire_term_p13;
            term_m6  <= wire_term_m6;
            term_p1  <= wire_term_p1;
          end else begin
            state <= 2'b00;
            term_p13 <= 48'sd0;
            term_m6  <= 48'sd0;
            term_p1  <= 48'sd0;
          end
        end
        2'b01: begin
          state <= 2'b10;
          numer <= wire_numer;
        end
        2'b10: begin
          state <= 2'b00;
          x_out <= wire_div20[31:0];
          out_valid <= 1'b1;
        end
        default: begin
          state <= 2'b00;
          out_valid <= 1'b0;
        end
      endcase
    end
  end

endmodule

