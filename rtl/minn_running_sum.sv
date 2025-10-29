`default_nettype none

// Running sum accumulator with sliding window.
// Implements: acc[n] = acc[n-1] + in[n] - in[n-DEPTH]
// Uses a delay line (FIFO) to store the window of input values.
module minn_running_sum #(
    parameter int IN_WIDTH = 25,
    parameter int DEPTH = 512,
    parameter int OUT_WIDTH = 34  // Should be IN_WIDTH + $clog2(DEPTH)
) (
    input  logic                    clk,
    input  logic                    rst,
    
    input  logic                    in_valid,
    input  logic signed [IN_WIDTH-1:0]  in_data,
    
    output logic                    out_valid,
    output logic signed [OUT_WIDTH-1:0] out_sum
);

    // Delayed version of input for sliding window
    logic signed [IN_WIDTH-1:0] delayed_data;
    logic delayed_valid;
    
    // Delay line to store the window
    minn_delay_line #(
        .WIDTH(IN_WIDTH),
        .DEPTH(DEPTH)
    ) fifo_inst (
        .clk(clk),
        .rst(rst),
        .in_valid(in_valid),
        .in_data(in_data),
        .out_valid(delayed_valid),
        .out_data(delayed_data)
    );
    
    // Running sum accumulator
    logic signed [OUT_WIDTH-1:0] accumulator;
    
    // Extended versions for safe arithmetic
    logic signed [OUT_WIDTH-1:0] in_extended;
    logic signed [OUT_WIDTH-1:0] delayed_extended;
    
    initial begin
        accumulator = '0;
        out_sum = '0;
        out_valid = 1'b0;
    end
    
    // Sign-extend inputs to accumulator width
    always_comb begin
        in_extended = {{(OUT_WIDTH-IN_WIDTH){in_data[IN_WIDTH-1]}}, in_data};
        delayed_extended = {{(OUT_WIDTH-IN_WIDTH){delayed_data[IN_WIDTH-1]}}, delayed_data};
    end
    
    // Accumulator update: add new, subtract old
    always_ff @(posedge clk) begin
        if (rst == 1'b1) begin
            accumulator <= '0;
            out_sum <= '0;
            out_valid <= 1'b0;
        end else begin
            if (in_valid == 1'b1) begin
                // During warm-up (delayed_valid==0), only accumulate new values
                accumulator <= (delayed_valid == 1'b1) ? 
                               (accumulator + in_extended - delayed_extended) :
                               (accumulator + in_extended);
                out_sum <= accumulator;
                out_valid <= delayed_valid;  // Valid only after window fills
            end else begin
                out_sum <= accumulator;
                out_valid <= 1'b0;
            end
        end
    end

endmodule

`default_nettype wire