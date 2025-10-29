`default_nettype none

// Dual-port circular buffer for Minn preamble detector output.
// Port A: Sequential writes of incoming samples
// Port B: Random access flag writes at detection points
// Continuous delayed read stream with frame_start flag
module minn_output_buffer #(
    parameter int W_IN = 12,
    parameter int DEPTH = 2112,
    parameter int OUTPUT_DELAY = 2048,
    
    // Derived parameters
    parameter int ADDR_WIDTH = $clog2(DEPTH),
    parameter int ENTRY_WIDTH = 4*W_IN + 1  // ch0_i, ch0_q, ch1_i, ch1_q, flag
) (
    input  logic                    clk,
    input  logic                    rst,
    
    // Input samples (write port A)
    input  logic                    in_valid,
    input  logic signed [W_IN-1:0]  in_ch0_i,
    input  logic signed [W_IN-1:0]  in_ch0_q,
    input  logic signed [W_IN-1:0]  in_ch1_i,
    input  logic signed [W_IN-1:0]  in_ch1_q,
    
    // Flag write (write port B)
    input  logic                    flag_write,
    input  logic [ADDR_WIDTH-1:0]   flag_addr,
    
    // Current write pointer (for FSM to compute flag address)
    output logic [ADDR_WIDTH-1:0]   current_wptr,
    
    // Output samples (delayed read)
    output logic                    out_valid,
    output logic signed [W_IN-1:0]  out_ch0_i,
    output logic signed [W_IN-1:0]  out_ch0_q,
    output logic signed [W_IN-1:0]  out_ch1_i,
    output logic signed [W_IN-1:0]  out_ch1_q,
    output logic                    frame_start
);

    // Dual-port memory for circular buffer
    // Entry format: {flag, ch1_q, ch1_i, ch0_q, ch0_i}
    logic [ENTRY_WIDTH-1:0] buffer_mem [0:DEPTH-1];
    
    // Write and read pointers
    logic [ADDR_WIDTH-1:0] wr_ptr;
    logic [ADDR_WIDTH-1:0] rd_ptr;
    
    // Sample counter for output delay
    logic [$clog2(OUTPUT_DELAY+1)-1:0] sample_count;
    
    // Entry packing/unpacking
    logic [ENTRY_WIDTH-1:0] write_entry;
    logic [ENTRY_WIDTH-1:0] read_entry;
    logic read_flag;
    
    initial begin
        wr_ptr = '0;
        rd_ptr = '0;
        sample_count = '0;
        out_valid = 1'b0;
        current_wptr = '0;
        for (int i = 0; i < DEPTH; i = i + 1) begin
            buffer_mem[i] = '0;
        end
    end
    
    // Pack input samples into entry (flag=0 by default)
    always_comb begin
        write_entry = {1'b0, in_ch1_q, in_ch1_i, in_ch0_q, in_ch0_i};
    end
    
    // ========== Write Process (Port A: Sequential Sample Writes) ==========
    
    always_ff @(posedge clk) begin
        if (rst == 1'b1) begin
            wr_ptr <= '0;
            sample_count <= '0;
            current_wptr <= '0;
        end else begin
            if (in_valid == 1'b1) begin
                // Write new sample to buffer
                buffer_mem[wr_ptr] <= write_entry;
                
                // Increment write pointer with wraparound
                wr_ptr <= (wr_ptr == DEPTH[ADDR_WIDTH-1:0] - 1) ? '0 : wr_ptr + 1;
                
                // Track samples written for warm-up period
                if (sample_count < OUTPUT_DELAY[$ clog2(OUTPUT_DELAY+1)-1:0]) begin
                    sample_count <= sample_count + 1;
                end
                
                // Expose current write pointer for FSM
                current_wptr <= wr_ptr;
            end
        end
    end
    
    // ========== Flag Write Process (Port B: Random Access Flag Writes) ==========
    
    always_ff @(posedge clk) begin
        if (flag_write == 1'b1) begin
            // Set flag bit at specified address without changing sample data
            buffer_mem[flag_addr][ENTRY_WIDTH-1] <= 1'b1;
        end
    end
    
    // ========== Read Process (Delayed by OUTPUT_DELAY samples) ==========
    
    always_ff @(posedge clk) begin
        if (rst == 1'b1) begin
            rd_ptr <= '0;
            out_valid <= 1'b0;
            out_ch0_i <= '0;
            out_ch0_q <= '0;
            out_ch1_i <= '0;
            out_ch1_q <= '0;
            frame_start <= 1'b0;
        end else begin
            // Start reading after OUTPUT_DELAY samples have been written
            if (sample_count >= OUTPUT_DELAY[$clog2(OUTPUT_DELAY+1)-1:0]) begin
                // Read from buffer at rd_ptr
                read_entry = buffer_mem[rd_ptr];
                
                // Unpack entry
                {read_flag, out_ch1_q, out_ch1_i, out_ch0_q, out_ch0_i} = read_entry;
                frame_start <= read_flag;
                out_valid <= 1'b1;
                
                // Clear flag after reading (for next frame detection)
                if (read_flag == 1'b1) begin
                    buffer_mem[rd_ptr][ENTRY_WIDTH-1] <= 1'b0;
                end
                
                // Increment read pointer with wraparound
                rd_ptr <= (rd_ptr == DEPTH[ADDR_WIDTH-1:0] - 1) ? '0 : rd_ptr + 1;
            end else begin
                out_valid <= 1'b0;
                frame_start <= 1'b0;
            end
        end
    end

endmodule

`default_nettype wire