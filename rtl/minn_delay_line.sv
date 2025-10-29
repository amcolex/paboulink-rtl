`default_nettype none

// Parameterized delay line (shift register) for FPGA implementation.
// Can be used for simple delays or as FIFO storage.
// Synthesis tools will infer SRL or BRAM depending on depth and width.
module minn_delay_line #(
    parameter int WIDTH = 16,
    parameter int DEPTH = 8
) (
    input  logic                clk,
    input  logic                rst,
    
    input  logic                in_valid,
    input  logic [WIDTH-1:0]    in_data,
    
    output logic                out_valid,
    output logic [WIDTH-1:0]    out_data
);

    // Delay line storage - will infer SRL32 for small depths, BRAM for large
    logic [WIDTH-1:0] delay_mem [0:DEPTH-1];
    
    // Write pointer for circular buffer operation
    logic [$clog2(DEPTH)-1:0] wr_ptr;
    
    // Read pointer trails write pointer by DEPTH
    logic [$clog2(DEPTH)-1:0] rd_ptr;
    
    // Valid bit propagation
    logic valid_sr [0:DEPTH-1];
    
    // Initialize all storage to zero
    initial begin
        wr_ptr = '0;
        rd_ptr = '0;
        for (int i = 0; i < DEPTH; i = i + 1) begin
            delay_mem[i] = '0;
            valid_sr[i] = 1'b0;
        end
    end
    
    // Write process: shift in new data
    always_ff @(posedge clk) begin
        if (rst == 1'b1) begin
            wr_ptr <= '0;
            rd_ptr <= '0;
            for (int i = 0; i < DEPTH; i = i + 1) begin
                valid_sr[i] <= 1'b0;
            end
        end else begin
            if (in_valid == 1'b1) begin
                delay_mem[wr_ptr] <= in_data;
                valid_sr[wr_ptr] <= 1'b1;
                wr_ptr <= (wr_ptr == DEPTH[$clog2(DEPTH)-1:0] - 1) ? '0 : wr_ptr + 1;
                rd_ptr <= (rd_ptr == DEPTH[$clog2(DEPTH)-1:0] - 1) ? '0 : rd_ptr + 1;
            end
        end
    end
    
    // Read process: output delayed data
    always_ff @(posedge clk) begin
        if (rst == 1'b1) begin
            out_data <= '0;
            out_valid <= 1'b0;
        end else begin
            out_data <= delay_mem[rd_ptr];
            out_valid <= valid_sr[rd_ptr];
        end
    end

endmodule

`default_nettype wire