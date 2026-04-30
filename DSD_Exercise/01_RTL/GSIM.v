`timescale 1ns/10ps
module GSIM ( clk, reset, in_en, b_in, out_valid, x_out);
input   clk ;
input   reset ;
input   in_en;
output  reg out_valid;
input   [15:0]  b_in;
output  reg [31:0]  x_out;
/*------Parameter------*/
localparam IDLE = 3'd0;
localparam READ_B = 3'd1;    // in_en=1, 讀進 b_in, 存b[0]~b[15]
localparam INIT_X = 3'd2;    // X的初始值
localparam ITERATE = 3'd3;   // 做 M 次 Gauss-Seidel
localparam WRITE_OUT = 3'd4; // out_valid=1, 依序輸出x[0]~x[15]
localparam integer ITERATIONS = 16; // M times iteration

/*------Wires and Registers------*/
reg [2:0] state, next_state;
reg [15:0] x_reg;
reg [3:0] out_idx;
reg [3:0] b_idx;
reg signed [15:0] b_mem [0:15];
reg [31:0] x_mem [0:15];

// Gauss-Seidel control
reg [3:0] xi_idx;            // current i in [0..15]
reg [7:0] iter_cnt;          // iteration counter
wire core_out_valid;
wire signed [31:0] core_x_out;

// Latch core inputs for the 3-stage pipeline
reg signed [15:0]  core_bi;
reg signed [31:0]  core_x_im3, core_x_im2, core_x_im1, core_x_ip1, core_x_ip2, core_x_ip3;

// Window for x (取前三、後三). Helper: boundary-safe reads (outside range => 0)
wire signed [31:0] x_m3 = (xi_idx >= 4'd3) ? $signed(x_mem[xi_idx - 4'd3]) : 32'd0;
wire signed [31:0] x_m2 = (xi_idx >= 4'd2) ? $signed(x_mem[xi_idx - 4'd2]) : 32'd0;
wire signed [31:0] x_m1 = (xi_idx >= 4'd1) ? $signed(x_mem[xi_idx - 4'd1]) : 32'd0;
wire signed [31:0] x_p1 = (xi_idx <= 4'd14) ? $signed(x_mem[xi_idx + 4'd1]) : 32'd0;
wire signed [31:0] x_p2 = (xi_idx <= 4'd13) ? $signed(x_mem[xi_idx + 4'd2]) : 32'd0;
wire signed [31:0] x_p3 = (xi_idx <= 4'd12) ? $signed(x_mem[xi_idx + 4'd3]) : 32'd0;
wire core_in_valid = (state == ITERATE) && !core_out_valid;

core_xi u_core_xi (
    .clk(clk),
    .reset(reset),
    .in_valid(core_in_valid),
    .bi(core_bi),
    .x_im3(core_x_im3),
    .x_im2(core_x_im2),
    .x_im1(core_x_im1),
    .x_ip1(core_x_ip1),
    .x_ip2(core_x_ip2),
    .x_ip3(core_x_ip3),
    .out_valid(core_out_valid),
    .x_out(core_x_out)
);
/*------Combanational Block------*/
//FSM
always @(*) begin
    next_state = IDLE;
    case (state)
        IDLE: next_state = in_en ? READ_B : IDLE;
        READ_B: begin
            if (!in_en) next_state = IDLE;
            else next_state = (b_idx == 4'd15) ? INIT_X : READ_B;
        end
        INIT_X: next_state = ITERATE;
        ITERATE: next_state = (iter_cnt == ITERATIONS[7:0]) ? WRITE_OUT : ITERATE;
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
        xi_idx <= 4'd0;
        iter_cnt <= 8'd0;
        core_bi <= 16'd0;
        core_x_im3 <= 32'd0;
        core_x_im2 <= 32'd0;
        core_x_im1 <= 32'd0;
        core_x_ip1 <= 32'd0;
        core_x_ip2 <= 32'd0;
        core_x_ip3 <= 32'd0;

        for (i = 0; i < 16; i = i + 1) begin
            b_mem[i] <= 16'd0;
            x_mem[i] <= 32'd0;
        end
    end 
    else begin
        case(state)
        IDLE: begin
            out_valid <= 1'b0;
            x_out <= 32'd0;
            out_idx <= 4'd0;
            b_idx <= 4'd0;
            xi_idx <= 4'd0;
            iter_cnt <= 8'd0;
        end
        READ_B: begin
            out_valid <= 1'b0;
            b_mem[b_idx] <= b_in;
            x_reg <= b_in;
            if (b_idx != 4'd15) b_idx <= b_idx + 4'd1;
        end
        INIT_X: begin
            out_valid <= 1'b0;
            for (i = 0; i < 16; i = i + 1) begin
                x_mem[i] <= {b_mem[i], 16'b0};
            end
            xi_idx <= 4'd0;
            iter_cnt <= 8'd0;
        end
        ITERATE: begin
            out_valid <= 1'b0;
            // If core finishes an i, write back x[i] and advance
            if (core_out_valid) begin
                x_mem[xi_idx] <= core_x_out;
                if (xi_idx == 4'd15) begin
                    xi_idx <= 4'd0;
                    if (iter_cnt != ITERATIONS[7:0]) iter_cnt <= iter_cnt + 8'd1;
                end 
                else begin
                    xi_idx <= xi_idx + 4'd1;
                end
            end 
            else begin
                // Feed current i into the core pipeline
                core_bi  <= b_mem[xi_idx];
                core_x_im3 <= x_m3;
                core_x_im2 <= x_m2;
                core_x_im1 <= x_m1;
                core_x_ip1 <= x_p1;
                core_x_ip2 <= x_p2;
                core_x_ip3 <= x_p3;
            end
        end
        WRITE_OUT: begin
            out_valid <= 1'b1;
            x_out <= x_mem[out_idx];
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
