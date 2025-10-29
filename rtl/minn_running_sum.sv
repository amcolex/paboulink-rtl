`default_nettype none

module minn_running_sum #(
    parameter int WIDTH = 16,
    parameter int DEPTH = 16
) (
    input  logic                      clk,
    input  logic                      rst,
    input  logic                      in_valid,
    input  logic signed [WIDTH-1:0]   sample_in,
    output logic signed [WIDTH + $clog2(DEPTH + 1)-1:0] sum_out,
    output logic                      sum_valid
);
    if (DEPTH <= 0) begin : g_invalid_depth
        initial begin
            $error("minn_running_sum DEPTH must be positive.");
        end
    end else begin : g_sum
        localparam int ADDR_WIDTH  = (DEPTH <= 1) ? 1 : $clog2(DEPTH);
        localparam int COUNT_WIDTH = $clog2(DEPTH + 1);
        localparam int SUM_WIDTH   = WIDTH + COUNT_WIDTH;

        typedef logic [ADDR_WIDTH-1:0]  addr_t;
        typedef logic [COUNT_WIDTH-1:0] count_t;

        localparam addr_t  LAST_ADDR   = addr_t'(DEPTH - 1);
        localparam count_t DEPTH_COUNT = count_t'(DEPTH);

        logic signed [WIDTH-1:0] window [0:DEPTH-1];
        addr_t                    wr_ptr;
        count_t                   fill_count;
        logic signed [SUM_WIDTH-1:0] sum_reg;

        integer i;
        always_ff @(posedge clk) begin
            if (rst) begin
                for (i = 0; i < DEPTH; i++) begin
                    window[i] = '0;
                end
                wr_ptr     <= '0;
                fill_count <= '0;
                sum_reg    <= '0;
                sum_out    <= '0;
                sum_valid  <= 1'b0;
            end else if (in_valid) begin
                logic signed [WIDTH-1:0] oldest;
                logic signed [SUM_WIDTH-1:0] next_sum;

                oldest  = window[wr_ptr];
                window[wr_ptr] <= sample_in;

                if (wr_ptr == LAST_ADDR) begin
                    wr_ptr <= '0;
                end else begin
                    wr_ptr <= wr_ptr + 1'b1;
                end

                next_sum = sum_reg
                    + $signed({{(SUM_WIDTH-WIDTH){sample_in[WIDTH-1]}}, sample_in})
                    - $signed({{(SUM_WIDTH-WIDTH){oldest[WIDTH-1]}}, oldest});
                sum_reg  <= next_sum;
                sum_out  <= next_sum;

                if (fill_count < DEPTH_COUNT) begin
                    count_t next_count;
                    next_count = fill_count + 1'b1;
                    fill_count <= next_count;
                    if (next_count >= DEPTH_COUNT) begin
                        sum_valid <= 1'b1;
                    end else begin
                        sum_valid <= 1'b0;
                    end
                end else begin
                    sum_valid <= 1'b1;
                end
            end else begin
                sum_out <= sum_reg;
            end
        end
    end
endmodule

`default_nettype wire
