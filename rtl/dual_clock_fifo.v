`default_nettype none

// Dual-clock FIFO with Gray-coded pointers.
// Parameters:
//   DATA_WIDTH  : width of the stored payload.
//   ADDR_WIDTH  : FIFO depth is 2**ADDR_WIDTH entries.
// Resets are active-low and synchronous to their respective domains.

module dual_clock_fifo #(
    parameter integer DATA_WIDTH = 8,
    parameter integer ADDR_WIDTH = 4
) (
    // Write port (clk_axis domain)
    input  wire                     wr_clk,
    input  wire                     wr_rst_n,
    input  wire                     wr_en,
    input  wire [DATA_WIDTH-1:0]    wr_data,
    output wire                     wr_full,
    output reg                      wr_error,

    // Read port (clk_fft domain)
    input  wire                     rd_clk,
    input  wire                     rd_rst_n,
    input  wire                     rd_en,
    output reg  [DATA_WIDTH-1:0]    rd_data,
    output wire                     rd_empty,
    output reg                      rd_error
);

    localparam integer DEPTH = (1 << ADDR_WIDTH);

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    // Binary and Gray pointers in write domain.
    reg [ADDR_WIDTH:0] wr_ptr_bin;
    reg [ADDR_WIDTH:0] wr_ptr_gray;
    reg [ADDR_WIDTH:0] rd_ptr_gray_wrclk;
    reg [ADDR_WIDTH:0] rd_ptr_gray_wrclk_q;

    // Binary and Gray pointers in read domain.
    reg [ADDR_WIDTH:0] rd_ptr_bin;
    reg [ADDR_WIDTH:0] rd_ptr_gray;
    reg [ADDR_WIDTH:0] wr_ptr_gray_rdclk;
    reg [ADDR_WIDTH:0] wr_ptr_gray_rdclk_q;

    // Binary versions of synchronized pointers.
    // Gray to binary conversion function.
    function automatic [ADDR_WIDTH:0] gray_to_bin;
        input [ADDR_WIDTH:0] gray;
        integer i;
        begin
            gray_to_bin[ADDR_WIDTH] = gray[ADDR_WIDTH];
            for (i = ADDR_WIDTH - 1; i >= 0; i = i - 1) begin
                gray_to_bin[i] = gray_to_bin[i + 1] ^ gray[i];
            end
        end
    endfunction

    // Write pointer logic.
    always @(posedge wr_clk) begin
        if (!wr_rst_n) begin
            wr_ptr_bin  <= {ADDR_WIDTH+1{1'b0}};
            wr_ptr_gray <= {ADDR_WIDTH+1{1'b0}};
            wr_error    <= 1'b0;
        end else begin
            wr_error <= 1'b0;
            if (wr_en && !wr_full) begin
                mem[wr_ptr_bin[ADDR_WIDTH-1:0]] <= wr_data;
                wr_ptr_bin  <= wr_ptr_bin + 1'b1;
                wr_ptr_gray <= (wr_ptr_bin + 1'b1) ^ ((wr_ptr_bin + 1'b1) >> 1);
            end else if (wr_en && wr_full) begin
                wr_error <= 1'b1;
            end
        end
    end

    // Synchronize read pointer into write clock domain.
    always @(posedge wr_clk) begin
        if (!wr_rst_n) begin
            rd_ptr_gray_wrclk   <= {ADDR_WIDTH+1{1'b0}};
            rd_ptr_gray_wrclk_q <= {ADDR_WIDTH+1{1'b0}};
        end else begin
            rd_ptr_gray_wrclk   <= rd_ptr_gray;
            rd_ptr_gray_wrclk_q <= rd_ptr_gray_wrclk;
        end
    end

    assign wr_full = (wr_ptr_gray[ADDR_WIDTH]      != rd_ptr_gray_wrclk_q[ADDR_WIDTH]) &&
                     (wr_ptr_gray[ADDR_WIDTH-1]    != rd_ptr_gray_wrclk_q[ADDR_WIDTH-1]) &&
                     (wr_ptr_gray[ADDR_WIDTH-2:0]  == rd_ptr_gray_wrclk_q[ADDR_WIDTH-2:0]);

    // Read pointer logic.
    always @(posedge rd_clk) begin
        if (!rd_rst_n) begin
            rd_ptr_bin  <= {ADDR_WIDTH+1{1'b0}};
            rd_ptr_gray <= {ADDR_WIDTH+1{1'b0}};
            rd_data     <= {DATA_WIDTH{1'b0}};
            rd_error    <= 1'b0;
        end else begin
            rd_error <= 1'b0;
            if (rd_en && !rd_empty) begin
                rd_data     <= mem[rd_ptr_bin[ADDR_WIDTH-1:0]];
                rd_ptr_bin  <= rd_ptr_bin + 1'b1;
                rd_ptr_gray <= (rd_ptr_bin + 1'b1) ^ ((rd_ptr_bin + 1'b1) >> 1);
            end else if (rd_en && rd_empty) begin
                rd_error <= 1'b1;
            end
        end
    end

    // Synchronize write pointer into read clock domain.
    always @(posedge rd_clk) begin
        if (!rd_rst_n) begin
            wr_ptr_gray_rdclk   <= {ADDR_WIDTH+1{1'b0}};
            wr_ptr_gray_rdclk_q <= {ADDR_WIDTH+1{1'b0}};
        end else begin
            wr_ptr_gray_rdclk   <= wr_ptr_gray;
            wr_ptr_gray_rdclk_q <= wr_ptr_gray_rdclk;
        end
    end

    assign rd_empty = (wr_ptr_gray_rdclk_q == rd_ptr_gray);

endmodule

`default_nettype wire
