`timescale 1ns/10ps

module tb_GSIM_moba;
parameter N_PAT = 16;
parameter CYCLE = 10;
parameter END_CYCLE = 1000000;

reg clk;
reg reset;
reg in_en;
reg [15:0] b_in;
wire out_valid;
wire [31:0] x_out;

reg  [15:0] pat_mem [0:N_PAT-1];
reg  [15:0] b       [0:N_PAT-1];
reg  [31:0] x       [0:N_PAT-1];
real x_f [0:N_PAT-1];
real Mb  [0:N_PAT-1];

integer i, j;
integer loop;
reg stop;
reg [15:0] b_tmp;
real SquareError, error, temp;

GSIM dut (
    .clk(clk),
    .reset(reset),
    .in_en(in_en),
    .b_in(b_in),
    .out_valid(out_valid),
    .x_out(x_out)
);

initial begin
    $readmemh("../00_TB/pattern1.dat", pat_mem);
end

initial begin
    clk   = 1'b0;
    reset = 1'b0;
    in_en = 1'b0;
    b_in  = 16'hzzzz;
    loop  = 0;
    stop  = 1'b0;
end

always #(CYCLE/2.0) clk = ~clk;

initial begin
    #END_CYCLE;
    $display("============================================");
    $display("ERROR: simulation timeout.");
    $display("============================================");
    $finish;
end

initial begin
    // Reset for one cycle
    @(negedge clk);
    reset = 1'b1;
    #(CYCLE);
    reset = 1'b0;

    // Feed b1..b16 while in_en=1
    @(negedge clk);
    for (i = 0; i < N_PAT; i = i + 1) begin
        b_in  = pat_mem[i];
        b[i]  = pat_mem[i];
        in_en = 1'b1;
        @(negedge clk);
    end
    in_en = 1'b0;
    b_in  = 16'hzzzz;
end

always @(negedge clk) begin
    if (loop < 16) begin
        if (out_valid) begin
            x[loop] = x_out;
            loop    = loop + 1;
        end
    end else begin
        stop = 1'b1;
    end
end

initial begin
    @(posedge stop);

    for (j = 0; j < 16; j = j + 1) begin
        if (x[j][31] == 1'b1) begin
            x_f[j] = (~x[j] + 1'b1);
            x_f[j] = -x_f[j] / 65536.0;
        end else begin
            x_f[j] = x[j];
            x_f[j] = x_f[j] / 65536.0;
        end
    end

    Mb[0 ] =  20*x_f[0 ] -13*x_f[1 ] + 6*x_f[2 ] -   x_f[3 ];
    Mb[1 ] = -13*x_f[0 ] +20*x_f[1 ] -13*x_f[2 ] + 6*x_f[3 ] -   x_f[4 ];
    Mb[2 ] =   6*x_f[0 ] -13*x_f[1 ] +20*x_f[2 ] -13*x_f[3 ] + 6*x_f[4 ] -   x_f[5 ];
    Mb[3 ] =    -x_f[0 ] + 6*x_f[1 ] -13*x_f[2 ] +20*x_f[3 ] -13*x_f[4 ] + 6*x_f[5 ] - x_f[6 ];
    Mb[4 ] =    -x_f[1 ] + 6*x_f[2 ] -13*x_f[3 ] +20*x_f[4 ] -13*x_f[5 ] + 6*x_f[6 ] - x_f[7 ];
    Mb[5 ] =    -x_f[2 ] + 6*x_f[3 ] -13*x_f[4 ] +20*x_f[5 ] -13*x_f[6 ] + 6*x_f[7 ] - x_f[8 ];
    Mb[6 ] =    -x_f[3 ] + 6*x_f[4 ] -13*x_f[5 ] +20*x_f[6 ] -13*x_f[7 ] + 6*x_f[8 ] - x_f[9 ];
    Mb[7 ] =    -x_f[4 ] + 6*x_f[5 ] -13*x_f[6 ] +20*x_f[7 ] -13*x_f[8 ] + 6*x_f[9 ] - x_f[10];
    Mb[8 ] =    -x_f[5 ] + 6*x_f[6 ] -13*x_f[7 ] +20*x_f[8 ] -13*x_f[9 ] + 6*x_f[10] - x_f[11];
    Mb[9 ] =    -x_f[6 ] + 6*x_f[7 ] -13*x_f[8 ] +20*x_f[9 ] -13*x_f[10] + 6*x_f[11] - x_f[12];
    Mb[10] =    -x_f[7 ] + 6*x_f[8 ] -13*x_f[9 ] +20*x_f[10]-13*x_f[11] + 6*x_f[12] - x_f[13];
    Mb[11] =    -x_f[8 ] + 6*x_f[9 ] -13*x_f[10]+20*x_f[11]-13*x_f[12] + 6*x_f[13] - x_f[14];
    Mb[12] =    -x_f[9 ] + 6*x_f[10]-13*x_f[11]+20*x_f[12]-13*x_f[13] + 6*x_f[14] - x_f[15];
    Mb[13] =    -x_f[10] + 6*x_f[11]-13*x_f[12]+20*x_f[13]-13*x_f[14] + 6*x_f[15];
    Mb[14] =    -x_f[11] + 6*x_f[12]-13*x_f[13]+20*x_f[14]-13*x_f[15];
    Mb[15] =    -x_f[12] + 6*x_f[13]-13*x_f[14]+20*x_f[15];

    SquareError = 0.0;
    for (j = 0; j < 16; j = j + 1) begin
        if (b[j][15] == 1'b1) begin
            b_tmp = ~b[j] + 1'b1;
            temp  = b_tmp;
            error = temp + Mb[j];
        end else begin
            error = Mb[j] - b[j];
        end
        SquareError = SquareError + error*error;
    end

    $display("============================================");
    for (j = 0; j < 16; j = j + 1) begin
        $display("x[%0d] = %.10f", j, x_f[j]);
    end
    $display("--------------------------------------------");
    $display("SquareError = %.15f", SquareError);
    if (SquareError < 0.000001 && SquareError !== 'hx && SquareError !== 'hz) begin
        $display("LEVEL A / PASS");
    end else begin
        $display("NOT LEVEL A (still useful for debug)");
    end
    $display("============================================");
    #(CYCLE/2.0);
    $finish;
end

endmodule
