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
localparam ITERATE = 3'd3;   // 做M次 Gauss-Seidel
localparam WRITE_OUT = 3'd4; // out_valid=1, 依序輸出x[0]~x[15]

/*------Wires and Registers------*/
reg [2:0] state, next_state;
reg [15:0] x_reg;
reg [3:0] out_idx;
reg [3:0] b_idx;
reg signed [15:0] b_mem [0:15];
reg [31:0] x_mem [0:15];
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
        ITERATE: next_state = WRITE_OUT;
        WRITE_OUT: next_state = (out_idx == 4'd15) ? IDLE : WRITE_OUT;
        default: next_state = IDLE;
    endcase
end

//Gauss-Seidel Iteration
always @(*) begin
    //先寫單一個core的運算式
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
            x_mem[i] <= 32'd0;
        end
    end 
    else begin
        case(state)
        IDLE: begin
            out_valid <= 1'b0;
            x_out <= 32'd0;
        end
        READ_B: begin
            b_mem[b_idx] <= b_in;
            x_reg <= b_in;
            if (b_idx != 4'd15) b_idx <= b_idx + 4'd1;
        end
        INIT_X: begin
            for (i = 0; i < 16; i = i + 1) begin
                x_mem[i] <= {b_mem[i], 16'b0};
            end
        end
        ITERATE: begin
            for (i = 0; i < 16; i = i + 1) begin
                x_mem[i] <= x_mem[i] - (x_mem[i] * x_mem[i] * x_mem[i] * x_mem[i]) / 65536;
            end
        end
        WRITE_OUT: begin
            out_idx <= out_idx + 4'd1;
            x_out <= x_mem[out_idx];
        end
        default: begin
            out_valid <= 1'b0;
            x_out <= 32'd0;
        end
        endcase
    end
end

endmodule
