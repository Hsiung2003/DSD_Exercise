`timescale 1ns/10ps

// Iteration controller (single-core baseline).
// - Owns x_old/x_new/updated bookkeeping for one full GS iteration sweep.
// - Provides a clean interface for GSIM.v to "start" iterations and read final x.
// - Uses group/slot ordering: group0(1,5,9,13), group1(2,6,10,14), group2(3,7,11,15), group3(4,8,12,16)
//
// Notes:
// - This is a skeleton: fill in init policy (x^0), M_ITER, and exact timing of start/done as you integrate.
module iter_ctrl #(
  parameter integer M_ITER = 30
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

  // ---- Initial guess x^(0) (Q16.16) ----
  // Edit the assigns below to set each x[i] initial value.
  wire signed [31:0] x_init [0:15];
  assign x_init[0]  = 32'sd0;
  assign x_init[1]  = 32'sd0;
  assign x_init[2]  = 32'sd0;
  assign x_init[3]  = 32'sd0;
  assign x_init[4]  = 32'sd0;
  assign x_init[5]  = 32'sd0;
  assign x_init[6]  = 32'sd0;
  assign x_init[7]  = 32'sd0;
  assign x_init[8]  = 32'sd0;
  assign x_init[9]  = 32'sd0;
  assign x_init[10] = 32'sd0;
  assign x_init[11] = 32'sd0;
  assign x_init[12] = 32'sd0;
  assign x_init[13] = 32'sd0;
  assign x_init[14] = 32'sd0;
  assign x_init[15] = 32'sd0;

  // ---- Internal storage ----
  reg signed [31:0] x_old [0:15];
  reg signed [31:0] x_new [0:15];
  reg        [15:0] updated; // 1=updated in current sweep (for new/old mux)

  // ---- Sweep counters ----
  reg [5:0] iter_cnt; // 0..M_ITER-1
  reg [1:0] group;    // 0..3
  reg [1:0] slot;     // 0..3
  reg [3:0] cur_idx;  // 0..15

  // FSM
  localparam [1:0]
    IDLE = 2'b00,
    INIT = 2'b01,
    RUN = 2'b10,
    DONE = 2'b11;
  reg [1:0] state;

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

  function automatic signed [31:0] pick_x(input integer j);
    begin
      if (j < 0 || j > 15) pick_x = 32'sd0;
      else if (updated[j]) pick_x = x_new[j];
      else                 pick_x = x_old[j];
    end
  endfunction

  // Neighbors for issued idx (combinational).
  // The pipelined core samples inputs on the clock edge when `core_in_valid` is high,
  // so drive core inputs from a registered issue index.
  // Launch decision for this cycle (must be stable BEFORE posedge).
  wire [3:0] next_idx = group + (slot << 2);
  wire       launch   = (state == RUN) && (bubble_cnt == 2'd0) && (issued_cnt < 5'd16);

  // Drive core inputs from the idx being launched this cycle.
  wire signed [15:0] bi_cur = pick_b(next_idx);
  wire signed [31:0] xim3 = pick_x($signed({1'b0,next_idx}) - 3);
  wire signed [31:0] xim2 = pick_x($signed({1'b0,next_idx}) - 2);
  wire signed [31:0] xim1 = pick_x($signed({1'b0,next_idx}) - 1);
  wire signed [31:0] xip1 = pick_x($signed({1'b0,next_idx}) + 1);
  wire signed [31:0] xip2 = pick_x($signed({1'b0,next_idx}) + 2);
  wire signed [31:0] xip3 = pick_x($signed({1'b0,next_idx}) + 3);

  wire core_in_valid = launch;
  wire core_out_valid;
  reg [3:0] idx_pipe0, idx_pipe1, idx_pipe2;
  reg        v_pipe0, v_pipe1, v_pipe2;
  reg [4:0] issued_cnt, commit_cnt;
  reg [1:0] bubble_cnt;

  // Core compute (combinational)
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

  integer k;
  always @(posedge clk or posedge reset) begin
    if (reset) begin
      state <= IDLE;
      issued_cnt <= 5'd0;
      commit_cnt <= 5'd0;
      bubble_cnt <= 2'd0;

      busy <= 1'b0;
      done <= 1'b0;
      iter_cnt <= 6'd0;
      group <= 2'd0;
      slot <= 2'd0;
      cur_idx <= 4'd0;
      updated <= 16'd0;
      for (k = 0; k < 16; k = k + 1) begin
        x_old[k] <= 32'sd0;
        x_new[k] <= 32'sd0;
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
      if (core_out_valid && v_pipe2) begin
        x_new[idx_pipe2] <= x_calc;
        updated[idx_pipe2] <= 1'b1;
        commit_cnt <= commit_cnt + 5'd1;
      end

      case (state)
        IDLE: begin
            busy <= 1'b0;
          if (start) begin
            state <= INIT;
            issued_cnt <= 5'd0;
            commit_cnt <= 5'd0;
            bubble_cnt <= 2'd0;
            iter_cnt <= 6'd0;
          end
        end
        INIT: begin
            busy <= 1'b1;
          for (k = 0; k < 16; k = k + 1) begin
            if (iter_cnt == 6'd0) begin
              x_old[k] <= x_init[k];
              x_new[k] <= x_init[k];
            end else begin
              x_old[k] <= x_new[k];
            end
          end
          updated <= 16'd0;
          issued_cnt <= 5'd0;
          commit_cnt <= 5'd0;
          bubble_cnt <= 2'd0;
          group <= 2'd0;
          slot <= 2'd0;
          v_pipe0 <= 1'b0;
          v_pipe1 <= 1'b0;
          v_pipe2 <= 1'b0;
          state <= RUN;
        end

        RUN: begin 
          busy <= 1'b1;
          if (bubble_cnt != 2'd0) begin
            bubble_cnt <= bubble_cnt - 2'd1;
          end else if (issued_cnt < 5'd16) begin // issue next (launch==1)
            cur_idx <= next_idx;
            issued_cnt <= issued_cnt + 5'd1;

            if (slot == 2'd3) begin // done with current slot, core proceed next
                slot <= 2'd0;
                if (group == 2'd3) begin // done with last group
                    group <= 2'd0;
                end else begin // next group
                    group <= group + 2'd1;
                    bubble_cnt <= 2'd2;
                end 
            end else begin
                slot <= slot + 2'd1;
            end 
          end

          // done condition
          if (commit_cnt == 5'd16 ) begin // done with current iteration
            if (iter_cnt == M_ITER-1) begin // last iteration
              state <= DONE;
            end else begin // next iteration
                iter_cnt <= iter_cnt + 6'd1;
                state <= INIT;
            end
          end
        end

        DONE: begin
          state <= IDLE;
          x0  <= x_new[0];  x1  <= x_new[1];  x2  <= x_new[2];  x3  <= x_new[3];
          x4  <= x_new[4];  x5  <= x_new[5];  x6  <= x_new[6];  x7  <= x_new[7];
          x8  <= x_new[8];  x9  <= x_new[9];  x10 <= x_new[10]; x11 <= x_new[11];
          x12 <= x_new[12]; x13 <= x_new[13]; x14 <= x_new[14]; x15 <= x_new[15];
          busy <= 1'b0;
          done <= 1'b1;
        end
        default: state <= IDLE;
      endcase
    end
  end

endmodule

