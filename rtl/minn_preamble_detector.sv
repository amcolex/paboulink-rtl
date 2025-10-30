
module minn_preamble_detector #(
    parameter int INPUT_WIDTH       = 12,
    parameter int NFFT              = 2048,
    parameter int CP_LEN            = 512,
    parameter int OUTPUT_MARGIN     = CP_LEN,
    parameter int THRESH_VALUE      = 16'd4096,
    parameter int THRESH_FRAC_BITS  = 15,
    parameter int SMOOTH_SHIFT      = 3,
    parameter int HYSTERESIS        = 2,
    parameter int TIMING_OFFSET     = 0,
    parameter int METRIC_DBG_WIDTH  = 24
) (
    input  logic                          clk,
    input  logic                          rst,
    input  logic                          in_valid,
    input  logic signed [INPUT_WIDTH-1:0] in_ch0_i,
    input  logic signed [INPUT_WIDTH-1:0] in_ch0_q,
    input  logic signed [INPUT_WIDTH-1:0] in_ch1_i,
    input  logic signed [INPUT_WIDTH-1:0] in_ch1_q,
    output logic                          out_valid,
    output logic signed [INPUT_WIDTH-1:0] out_ch0_i,
    output logic signed [INPUT_WIDTH-1:0] out_ch0_q,
    output logic signed [INPUT_WIDTH-1:0] out_ch1_i,
    output logic signed [INPUT_WIDTH-1:0] out_ch1_q,
    output logic                          frame_start
`ifdef MINN_METRIC_DEBUG
   ,output logic [METRIC_DBG_WIDTH-1:0]   metric_dbg
`endif
);
    localparam int QUARTER_LEN = NFFT / 4;
    initial begin
        if ((NFFT % 4) != 0) begin
            $error("NFFT must be divisible by 4 for Minn detector.");
        end
        if (OUTPUT_MARGIN < 0) begin
            $error("OUTPUT_MARGIN must be non-negative.");
        end
    end

    localparam int OUTPUT_DELAY = NFFT;
    localparam int OUTPUT_DEPTH = OUTPUT_DELAY + OUTPUT_MARGIN;
    localparam int OUT_ADDR_WIDTH = (OUTPUT_DEPTH <= 1) ? 1 : $clog2(OUTPUT_DEPTH);

    localparam int SUM_GROWTH    = (QUARTER_LEN <= 1) ? 1 : $clog2(QUARTER_LEN + 1);
    localparam int PRODUCT_WIDTH = (2 * INPUT_WIDTH) + 1;
    localparam int POWER_WIDTH   = (2 * INPUT_WIDTH) + 1;
    localparam int CORR_WIDTH    = PRODUCT_WIDTH + SUM_GROWTH;
    localparam int ENERGY_WIDTH  = POWER_WIDTH + SUM_GROWTH;
    localparam int CORR_TOTAL_WIDTH = CORR_WIDTH + 2; // Sum of four quarter correlations.
    localparam int ENERGY_TOTAL_WIDTH = ENERGY_WIDTH + 3; // Sum of six quarter energies.

    localparam int HYST_WIDTH = (HYSTERESIS <= 0) ? 1 : $clog2(HYSTERESIS + 1);
    localparam logic [HYST_WIDTH-1:0] HYST_LIMIT = (HYSTERESIS <= 1)
        ? HYST_WIDTH'(0)
        : HYST_WIDTH'(HYSTERESIS - 1);
    localparam int THRESH_WIDTH = (THRESH_VALUE <= 0) ? 1 : $clog2(THRESH_VALUE + 1);
    localparam logic [THRESH_WIDTH-1:0] THRESH_CONST = THRESH_WIDTH'(THRESH_VALUE);
    localparam int CORR_SCALED_WIDTH = CORR_TOTAL_WIDTH + THRESH_FRAC_BITS;
    localparam int ENERGY_SCALED_WIDTH = ENERGY_TOTAL_WIDTH + THRESH_WIDTH;
    localparam int COMP_WIDTH = (CORR_SCALED_WIDTH > ENERGY_SCALED_WIDTH)
        ? CORR_SCALED_WIDTH
        : ENERGY_SCALED_WIDTH;
    localparam int TIMING_OFFSET_MOD  = (OUTPUT_DEPTH == 0) ? 0 : (TIMING_OFFSET % OUTPUT_DEPTH);
    localparam int TIMING_OFFSET_NORM = (TIMING_OFFSET_MOD < 0)
        ? (TIMING_OFFSET_MOD + OUTPUT_DEPTH)
        : TIMING_OFFSET_MOD;
    localparam logic [OUT_ADDR_WIDTH-1:0] TIMING_OFFSET_VEC = OUT_ADDR_WIDTH'(TIMING_OFFSET_NORM);
    localparam logic [OUT_ADDR_WIDTH-1:0] WRITE_WRAP = OUT_ADDR_WIDTH'(OUTPUT_DEPTH - 1);
    localparam logic [OUT_ADDR_WIDTH-1:0] READ_WRAP  = WRITE_WRAP;
    localparam logic [OUT_ADDR_WIDTH:0]   OUTPUT_DEPTH_COUNT = (OUT_ADDR_WIDTH+1)'(OUTPUT_DEPTH);
    localparam logic [OUT_ADDR_WIDTH:0]   OUTPUT_DELAY_COUNT = (OUT_ADDR_WIDTH+1)'(OUTPUT_DELAY);

    localparam int ENTRY_WIDTH = INPUT_WIDTH * 4;
    localparam int CH0_I_LSB    = 0;
    localparam int CH0_Q_LSB    = CH0_I_LSB + INPUT_WIDTH;
    localparam int CH1_I_LSB    = CH0_Q_LSB + INPUT_WIDTH;
    localparam int CH1_Q_LSB    = CH1_I_LSB + INPUT_WIDTH;

    (* ram_style = "block" *)
    logic [ENTRY_WIDTH-1:0] sample_mem [0:OUTPUT_DEPTH-1];
    logic [OUTPUT_DEPTH-1:0] flag_mem;

    logic [OUT_ADDR_WIDTH-1:0] write_ptr;
    logic [OUT_ADDR_WIDTH-1:0] read_ptr;
    logic [OUT_ADDR_WIDTH:0]   sample_count;

    // Antenna processing paths.
    logic signed [CORR_WIDTH-1:0]  ch0_corr_recent;
    logic signed [CORR_WIDTH-1:0]  ch0_corr_prev;
    logic signed [ENERGY_WIDTH-1:0] ch0_energy_recent;
    logic signed [ENERGY_WIDTH-1:0] ch0_energy_prev;
    logic signed [ENERGY_WIDTH-1:0] ch0_energy_prev2;
    logic                           ch0_valid;

    logic signed [CORR_WIDTH-1:0]  ch1_corr_recent;
    logic signed [CORR_WIDTH-1:0]  ch1_corr_prev;
    logic signed [ENERGY_WIDTH-1:0] ch1_energy_recent;
    logic signed [ENERGY_WIDTH-1:0] ch1_energy_prev;
    logic signed [ENERGY_WIDTH-1:0] ch1_energy_prev2;
    logic                           ch1_valid;

    minn_antenna_path #(
        .INPUT_WIDTH(INPUT_WIDTH),
        .QUARTER_LEN(QUARTER_LEN)
    ) antenna0 (
        .clk(clk),
        .rst(rst),
        .in_valid(in_valid),
        .in_i(in_ch0_i),
        .in_q(in_ch0_q),
        .corr_recent(ch0_corr_recent),
        .corr_previous(ch0_corr_prev),
        .energy_recent(ch0_energy_recent),
        .energy_previous(ch0_energy_prev),
        .energy_previous2(ch0_energy_prev2),
        .taps_valid(ch0_valid)
    );

    minn_antenna_path #(
        .INPUT_WIDTH(INPUT_WIDTH),
        .QUARTER_LEN(QUARTER_LEN)
    ) antenna1 (
        .clk(clk),
        .rst(rst),
        .in_valid(in_valid),
        .in_i(in_ch1_i),
        .in_q(in_ch1_q),
        .corr_recent(ch1_corr_recent),
        .corr_previous(ch1_corr_prev),
        .energy_recent(ch1_energy_recent),
        .energy_previous(ch1_energy_prev),
        .energy_previous2(ch1_energy_prev2),
        .taps_valid(ch1_valid)
    );

    logic metric_valid;
    assign metric_valid = in_valid && ch0_valid && ch1_valid;

    // Combine correlations and energies across antennas.
    logic signed [CORR_WIDTH:0] corr_sum_ch0;
    logic signed [CORR_WIDTH:0] corr_sum_ch1;
    logic signed [CORR_TOTAL_WIDTH-1:0] corr_total;
    logic [ENERGY_WIDTH+1:0] energy_sum_ch0;
    logic [ENERGY_WIDTH+1:0] energy_sum_ch1;
    logic [ENERGY_TOTAL_WIDTH-1:0] energy_total;

    always_comb begin
        corr_sum_ch0 = $signed(ch0_corr_recent) + $signed(ch0_corr_prev);
        corr_sum_ch1 = $signed(ch1_corr_recent) + $signed(ch1_corr_prev);
        corr_total   = $signed({corr_sum_ch0[CORR_WIDTH], corr_sum_ch0})
                     + $signed({corr_sum_ch1[CORR_WIDTH], corr_sum_ch1});

        energy_sum_ch0 = {2'b00, $unsigned(ch0_energy_recent)}
                        + {2'b00, $unsigned(ch0_energy_prev)}
                        + {2'b00, $unsigned(ch0_energy_prev2)};
        energy_sum_ch1 = {2'b00, $unsigned(ch1_energy_recent)}
                        + {2'b00, $unsigned(ch1_energy_prev)}
                        + {2'b00, $unsigned(ch1_energy_prev2)};
        energy_total   = {1'b0, energy_sum_ch0} + {1'b0, energy_sum_ch1};
    end

    // Positive-only correlation metric.
    logic [CORR_TOTAL_WIDTH-1:0] corr_positive;
    always_comb begin
        if (corr_total[CORR_TOTAL_WIDTH-1] == 1'b1) begin
            corr_positive = '0;
        end else begin
            corr_positive = corr_total[CORR_TOTAL_WIDTH-1:0];
        end
    end

    // Optional smoothing.
    logic [CORR_TOTAL_WIDTH-1:0] smooth_metric;
    generate
        if (SMOOTH_SHIFT == 0) begin : g_no_smooth
            always_ff @(posedge clk) begin
                if (rst) begin
                    smooth_metric <= '0;
                end else if (metric_valid) begin
                    smooth_metric <= corr_positive;
                end
            end
        end else begin : g_smooth
            logic signed [CORR_TOTAL_WIDTH:0] diff;
            logic signed [CORR_TOTAL_WIDTH:0] smooth_next;
            always_ff @(posedge clk) begin
                if (rst) begin
                    smooth_metric <= '0;
                end else if (metric_valid) begin
                    diff = $signed({1'b0, corr_positive}) - $signed({1'b0, smooth_metric});
                    smooth_next = $signed({1'b0, smooth_metric}) + (diff >>> SMOOTH_SHIFT);
                    smooth_metric <= smooth_next[CORR_TOTAL_WIDTH-1:0];
                end
            end
        end
    endgenerate

    // Threshold comparison via cross multiplication.
    logic [CORR_SCALED_WIDTH-1:0] corr_scaled_native;
    logic [ENERGY_SCALED_WIDTH-1:0] energy_scaled_native;
    logic [COMP_WIDTH-1:0] corr_scaled;
    logic [COMP_WIDTH-1:0] energy_scaled;
    logic above_threshold;

    assign corr_scaled_native = {smooth_metric, {THRESH_FRAC_BITS{1'b0}}};
    assign energy_scaled_native = energy_total * THRESH_CONST;
    assign corr_scaled = {{(COMP_WIDTH-CORR_SCALED_WIDTH){1'b0}}, corr_scaled_native};
    assign energy_scaled = {{(COMP_WIDTH-ENERGY_SCALED_WIDTH){1'b0}}, energy_scaled_native};
    assign above_threshold = metric_valid && (corr_scaled >= energy_scaled);

    // Gate and peak tracking.
    logic gate_open;
    logic [CORR_TOTAL_WIDTH-1:0] peak_value;
    logic [OUT_ADDR_WIDTH-1:0]   peak_ptr;
    logic [HYST_WIDTH-1:0]       low_counter;
    logic                        detection_pulse;
    logic [OUT_ADDR_WIDTH-1:0]   detection_addr;

    always_ff @(posedge clk) begin
        if (rst) begin
            gate_open       <= 1'b0;
            peak_value      <= '0;
            peak_ptr        <= '0;
            low_counter     <= '0;
            detection_pulse <= 1'b0;
            detection_addr  <= '0;
        end else begin
            detection_pulse <= 1'b0;
            if (metric_valid) begin
                if (!gate_open) begin
                    low_counter <= '0;
                    if (above_threshold) begin
                        gate_open  <= 1'b1;
                        peak_value <= corr_positive;
                        peak_ptr   <= write_ptr;
                    end
                end else begin
                    // Track the highest raw (unsmoothed) metric to minimize latency bias.
                    if (corr_positive >= peak_value) begin
                        peak_value <= corr_positive;
                        peak_ptr   <= write_ptr;
                    end

                    if (above_threshold) begin
                        low_counter <= '0;
                    end else if (HYSTERESIS == 0) begin
                        gate_open       <= 1'b0;
                        peak_value      <= '0;
                        low_counter     <= '0;
                        detection_pulse <= 1'b1;
                        detection_addr  <= ptr_with_offset(peak_ptr);
                    end else begin
                        if (low_counter == HYST_LIMIT) begin
                            gate_open       <= 1'b0;
                            peak_value      <= '0;
                            low_counter     <= '0;
                            detection_pulse <= 1'b1;
                            detection_addr  <= ptr_with_offset(peak_ptr);
                        end else begin
                            low_counter <= low_counter + 1'b1;
                        end
                    end
                end
            end
        end
    end

    // Compute pointer with timing offset.
    function automatic logic [OUT_ADDR_WIDTH-1:0] ptr_with_offset(
        input logic [OUT_ADDR_WIDTH-1:0] base
    );
        logic [OUT_ADDR_WIDTH:0] sum_vec;
        begin
            sum_vec = {1'b0, base} + {1'b0, TIMING_OFFSET_VEC};
            if (sum_vec >= OUTPUT_DEPTH_COUNT) begin
                sum_vec = sum_vec - OUTPUT_DEPTH_COUNT;
            end
            ptr_with_offset = sum_vec[OUT_ADDR_WIDTH-1:0];
        end
    endfunction

`ifdef MINN_METRIC_DEBUG
    function automatic logic [METRIC_DBG_WIDTH-1:0] metric_trunc(
        input logic [CORR_TOTAL_WIDTH-1:0] value
    );
        logic [METRIC_DBG_WIDTH-1:0] result;
        int start_index;
        start_index = (CORR_TOTAL_WIDTH > METRIC_DBG_WIDTH)
            ? (CORR_TOTAL_WIDTH - METRIC_DBG_WIDTH)
            : 0;
        result = '0;
        for (int idx = 0; idx < METRIC_DBG_WIDTH; idx++) begin
            int source = start_index + idx;
            if (source < CORR_TOTAL_WIDTH) begin
                result[idx] = value[source];
            end
        end
        return result;
    endfunction
`endif

    // Output buffer management.
    always_ff @(posedge clk) begin
        if (rst) begin
            write_ptr    <= '0;
            read_ptr     <= '0;
            sample_count <= '0;
            out_valid    <= 1'b0;
            out_ch0_i    <= '0;
            out_ch0_q    <= '0;
            out_ch1_i    <= '0;
            out_ch1_q    <= '0;
            frame_start  <= 1'b0;
            flag_mem     <= '0;
`ifdef MINN_METRIC_DEBUG
            metric_dbg   <= '0;
`endif
        end else begin
            out_valid   <= 1'b0;
            frame_start <= 1'b0;

            if (in_valid) begin
                sample_mem[write_ptr] <= {
                    in_ch1_q,
                    in_ch1_i,
                    in_ch0_q,
                    in_ch0_i
                };
                flag_mem[write_ptr] <= 1'b0;

                if (write_ptr == WRITE_WRAP) begin
                    write_ptr <= '0;
                end else begin
                    write_ptr <= write_ptr + 1'b1;
                end

                if (sample_count < OUTPUT_DEPTH_COUNT) begin
                    sample_count <= sample_count + 1'b1;
                end

                if (sample_count >= OUTPUT_DELAY_COUNT) begin
                    out_valid <= 1'b1;
                    out_ch0_i <= sample_mem[read_ptr][CH0_I_LSB +: INPUT_WIDTH];
                    out_ch0_q <= sample_mem[read_ptr][CH0_Q_LSB +: INPUT_WIDTH];
                    out_ch1_i <= sample_mem[read_ptr][CH1_I_LSB +: INPUT_WIDTH];
                    out_ch1_q <= sample_mem[read_ptr][CH1_Q_LSB +: INPUT_WIDTH];
                    frame_start <= flag_mem[read_ptr];

                    if (read_ptr == READ_WRAP) begin
                        read_ptr <= '0;
                    end else begin
                        read_ptr <= read_ptr + 1'b1;
                    end
                end else begin
                    out_valid   <= 1'b0;
                    frame_start <= 1'b0;
                end
            end else begin
                out_valid   <= 1'b0;
                frame_start <= 1'b0;
            end

            if (detection_pulse) begin
                flag_mem[detection_addr] <= 1'b1;
            end

`ifdef MINN_METRIC_DEBUG
            if (metric_valid) begin
                if (SMOOTH_SHIFT == 0) begin
                    metric_dbg <= metric_trunc(corr_positive);
                end else begin
                    metric_dbg <= metric_trunc(smooth_metric);
                end
            end
`endif
        end
    end
endmodule

`default_nettype wire
