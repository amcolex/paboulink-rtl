
// Minn OFDM frame start detector implementing the product-stream architecture described in the
// project design document. The block ingests two complex antennas of WIDTH-bit signed IQ samples
// with an input strobe `in_valid`. Internally it maintains the quarter-length sliding correlations
// and three-quarter energy window required by the Minn metric while keeping the multiplier count low.
//
// Outputs are delayed IQ samples aligned with the Minn metric window start (timing_offset = 0) and a
// single-cycle `detect_flag` pulse asserted on the peak metric sample once the metric falls back
// below the configured threshold. The output stream pauses while a detection gate is open to ensure
// the flagged sample remains buffered until finalization, mirroring the reference C implementation.
module minn_frame_detector #(
    parameter int WIDTH = 12,
    parameter int N_FFT = 2048,
    // Square-root of the Minn gate threshold expressed in Q1.15 format.
    parameter int K_Q15 = 16'd14666,
    parameter int SMOOTH_LEN = 1  // Placeholder for future smoothing support (1 disables smoothing).
) (
    input  logic                         clk,
    input  logic                         rst_n,
    input  logic                         in_valid,
    input  logic signed [WIDTH-1:0]      ch0_i,
    input  logic signed [WIDTH-1:0]      ch0_q,
    input  logic signed [WIDTH-1:0]      ch1_i,
    input  logic signed [WIDTH-1:0]      ch1_q,
    output logic                         out_valid,
    output logic signed [WIDTH-1:0]      out_ch0_i,
    output logic signed [WIDTH-1:0]      out_ch0_q,
    output logic signed [WIDTH-1:0]      out_ch1_i,
    output logic signed [WIDTH-1:0]      out_ch1_q,
    output logic                         detect_flag,
    output logic                         dbg_metric_valid,
    output logic [(2*WIDTH + $clog2(N_FFT/4 + 1) + 5)-1:0] dbg_metric_r,
    output logic [(2*WIDTH + $clog2(((3*(N_FFT/4))) + 1) + 4)-1:0] dbg_metric_energy
);
    // -------------------------------------------------------------------------
    // Parameter sanity checks and derived constants
    // -------------------------------------------------------------------------
    localparam int Q = N_FFT / 4;
    initial begin
        if (N_FFT % 4 != 0) begin
            $error("minn_frame_detector: N_FFT must be divisible by 4");
        end
        if (Q <= 0) begin
            $error("minn_frame_detector: Q must be positive");
        end
    end

    localparam int WINDOW_DELAY = 4 * Q - 1;
    localparam int Q_PTR_W = (Q > 1) ? $clog2(Q) : 1;
    localparam int P_DELAY_LEN = 2 * Q;
    localparam int P_DELAY_PTR_W = (P_DELAY_LEN > 1) ? $clog2(P_DELAY_LEN) : 1;
    localparam int ENERGY_LEN = 3 * Q;
    localparam int ENERGY_PTR_W = (ENERGY_LEN > 1) ? $clog2(ENERGY_LEN) : 1;
    localparam int BUFFER_LEN = 8 * Q;
    localparam int BUFFER_PTR_W = (BUFFER_LEN > 1) ? $clog2(BUFFER_LEN) : 1;
    localparam logic [BUFFER_PTR_W:0] BUFFER_LEN_EXT = (BUFFER_PTR_W+1)'(BUFFER_LEN);
    localparam logic [BUFFER_PTR_W-1:0] BUFFER_LEN_MINUS_ONE = BUFFER_PTR_W'(BUFFER_LEN-1);

    // Sample energy and correlation bit widths.
    localparam int ENERGY_SAMPLE_W = 2 * WIDTH + 1;       // |I|^2 + |Q|^2 (per antenna)
    localparam int ENERGY_STREAM_W = ENERGY_SAMPLE_W + 1; // Sum across antennas
    localparam int ENERGY_ACC_W = ENERGY_STREAM_W + $clog2(ENERGY_LEN + 1) + 2;

    localparam int PROD_W = 2 * WIDTH + 1;                // Individual antenna complex product components
    localparam int P_STREAM_W = PROD_W + 1;               // Sum across antennas
    localparam int CORR_SUM_W = P_STREAM_W + $clog2(Q + 1) + 2;
    localparam int R_WIDTH = CORR_SUM_W + 1;
    localparam int R_SHIFT_WIDTH = R_WIDTH + 15;
    localparam int RATIO_CMP_W = R_SHIFT_WIDTH + ENERGY_ACC_W;
    localparam int ENERGY_TIMES_K_W = ENERGY_ACC_W + 16;
    localparam int ENERGY_ASSIGN_W = (R_SHIFT_WIDTH < ENERGY_TIMES_K_W) ? R_SHIFT_WIDTH : ENERGY_TIMES_K_W;

    // -------------------------------------------------------------------------
    // Sample delay for computing x[n-Q] and buffer for output alignment
    // -------------------------------------------------------------------------
    typedef struct packed {
        logic signed [WIDTH-1:0] ch0_i;
        logic signed [WIDTH-1:0] ch0_q;
        logic signed [WIDTH-1:0] ch1_i;
        logic signed [WIDTH-1:0] ch1_q;
    } sample_t;

    sample_t delay_q_mem [0:Q-1];
    logic [Q_PTR_W-1:0] delay_q_ptr;

    sample_t sample_buffer [0:BUFFER_LEN-1];
    logic [BUFFER_LEN-1:0] flag_buffer;
    logic [BUFFER_PTR_W-1:0] sample_wr_ptr;
    logic [BUFFER_PTR_W-1:0] sample_rd_ptr;
    logic [BUFFER_PTR_W:0]   buffer_occupancy;
    logic [31:0]             base_sample_idx;

    // -------------------------------------------------------------------------
    // Product stream buffers (SumB window and SumA delayed window)
    // -------------------------------------------------------------------------
    logic signed [P_STREAM_W-1:0] sumB_fifo_real [0:Q-1];
    logic signed [P_STREAM_W-1:0] sumB_fifo_imag [0:Q-1];
    logic [Q_PTR_W-1:0]           sumB_ptr;
    logic [$clog2(Q+1)-1:0]       sumB_count;
    logic signed [CORR_SUM_W-1:0] sumB_real;
    logic signed [CORR_SUM_W-1:0] sumB_imag;

    logic signed [P_STREAM_W-1:0] sumA_fifo_real [0:Q-1];
    logic signed [P_STREAM_W-1:0] sumA_fifo_imag [0:Q-1];
    logic [Q_PTR_W-1:0]           sumA_ptr;
    logic [$clog2(Q+1)-1:0]       sumA_count;
    logic signed [CORR_SUM_W-1:0] sumA_real;
    logic signed [CORR_SUM_W-1:0] sumA_imag;

    logic signed [P_STREAM_W-1:0] p_delay_mem_real [0:P_DELAY_LEN-1];
    logic signed [P_STREAM_W-1:0] p_delay_mem_imag [0:P_DELAY_LEN-1];
    logic [P_DELAY_PTR_W-1:0]     p_delay_ptr;
    logic [31:0]                  p_count;

    // -------------------------------------------------------------------------
    // Energy accumulator (3Q samples)
    // -------------------------------------------------------------------------
    logic [ENERGY_STREAM_W-1:0] energy_fifo [0:ENERGY_LEN-1];
    logic [ENERGY_PTR_W-1:0]    energy_ptr;
    logic [$clog2(ENERGY_LEN+1)-1:0] energy_count;
    logic [ENERGY_ACC_W-1:0]    energy_sum;

    // -------------------------------------------------------------------------
    // Gate/peak tracking state
    // -------------------------------------------------------------------------
    logic gate_active;
    logic [R_SHIFT_WIDTH-1:0] peak_ratio_num;
    logic [ENERGY_ACC_W-1:0]  peak_energy_den;
    logic [31:0]              peak_index;
    logic                     detection_armed;
    logic [31:0]              holdoff_counter;

    // Sample index counter
    logic [31:0] sample_idx;

    // Temporary variables reused within sequential logic
    logic gate_active_next;
    logic [R_SHIFT_WIDTH-1:0] peak_ratio_num_next;
    logic [ENERGY_ACC_W-1:0]  peak_energy_den_next;
    logic [31:0]              peak_index_next;
    logic                     detection_armed_next;
    logic [31:0]              holdoff_counter_next;
    sample_t current_sample;
    sample_t delayed_sample;
    logic [2*WIDTH-1:0] ch0_i_sq;
    logic [2*WIDTH-1:0] ch0_q_sq;
    logic [2*WIDTH-1:0] ch1_i_sq;
    logic [2*WIDTH-1:0] ch1_q_sq;
    logic [ENERGY_SAMPLE_W-1:0] ant0_energy;
    logic [ENERGY_SAMPLE_W-1:0] ant1_energy;
    logic [ENERGY_STREAM_W-1:0] energy_in;
    logic [ENERGY_ACC_W-1:0] energy_sum_next;
    logic [$clog2(ENERGY_LEN+1)-1:0] energy_count_next;
    logic [ENERGY_STREAM_W-1:0] energy_old;
    logic [ENERGY_ACC_W-1:0] energy_in_ext;
    logic [ENERGY_ACC_W-1:0] energy_old_ext;
    bit has_q_delay;
    logic signed [P_STREAM_W-1:0] p_real_sum;
    logic signed [P_STREAM_W-1:0] p_imag_sum;
    logic signed [CORR_SUM_W-1:0] sumB_real_next;
    logic signed [CORR_SUM_W-1:0] sumB_imag_next;
    logic [$clog2(Q+1)-1:0]       sumB_count_next;
    logic [Q_PTR_W-1:0]           sumB_ptr_next;
    logic signed [CORR_SUM_W-1:0] sumA_real_next;
    logic signed [CORR_SUM_W-1:0] sumA_imag_next;
    logic [$clog2(Q+1)-1:0]       sumA_count_next;
    logic [Q_PTR_W-1:0]           sumA_ptr_next;
    logic [P_DELAY_PTR_W-1:0]     p_delay_ptr_next;
    logic [31:0]                  p_count_next;
    bit p_delay_ready;
    logic signed [P_STREAM_W-1:0] p_delay_real_val;
    logic signed [P_STREAM_W-1:0] p_delay_imag_val;
    logic signed [PROD_W-1:0]     p0_real;
    logic signed [PROD_W-1:0]     p0_imag;
    logic signed [PROD_W-1:0]     p1_real;
    logic signed [PROD_W-1:0]     p1_imag;
    logic signed [CORR_SUM_W-1:0] p_real_ext;
    logic signed [CORR_SUM_W-1:0] p_imag_ext;
    logic signed [P_STREAM_W-1:0] sumB_old_real;
    logic signed [P_STREAM_W-1:0] sumB_old_imag;
    logic signed [CORR_SUM_W-1:0] sumB_old_real_ext;
    logic signed [CORR_SUM_W-1:0] sumB_old_imag_ext;
    logic signed [CORR_SUM_W-1:0] p_delay_real_ext;
    logic signed [CORR_SUM_W-1:0] p_delay_imag_ext;
    logic signed [P_STREAM_W-1:0] sumA_old_real;
    logic signed [P_STREAM_W-1:0] sumA_old_imag;
    logic signed [CORR_SUM_W-1:0] sumA_old_real_ext;
    logic signed [CORR_SUM_W-1:0] sumA_old_imag_ext;
    bit sums_ready;
    bit energy_ready;
    bit metric_valid;
    logic signed [R_WIDTH-1:0]    corr_real_total;
    logic [R_WIDTH-1:0]           R_value;
    logic [R_SHIFT_WIDTH-1:0]     ratio_num;
    logic [ENERGY_ACC_W-1:0]      energy_value;
    logic [ENERGY_ACC_W+16-1:0]   energy_times_k;
    logic [R_SHIFT_WIDTH-1:0]     threshold_scaled;
    bit above_threshold;
    logic [31:0]                  window_start_idx;
    logic [RATIO_CMP_W-1:0]       lhs;
    logic [RATIO_CMP_W-1:0]       rhs;
    logic [31:0]                  offset;
    logic [BUFFER_PTR_W:0]        flag_ptr_ext;
    logic [BUFFER_PTR_W:0]        buffer_after_write;
    logic [BUFFER_PTR_W:0]        offset_ext;
    bit pop_output;
    sample_t out_sample;
    logic [31:0]                  holdoff_temp;
    bit gate_can_pop;
    bit gate_closed_this_cycle;

    // -------------------------------------------------------------------------
    // Reset logic: clear memories and registers
    // -------------------------------------------------------------------------
    integer idx;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            delay_q_ptr       <= '0;
            sample_wr_ptr     <= '0;
            sample_rd_ptr     <= '0;
            buffer_occupancy  <= '0;
            base_sample_idx   <= 32'd0;
            sumB_ptr          <= '0;
            sumB_count        <= '0;
            sumB_real         <= '0;
            sumB_imag         <= '0;
            sumA_ptr          <= '0;
            sumA_count        <= '0;
            sumA_real         <= '0;
            sumA_imag         <= '0;
            p_delay_ptr       <= '0;
            p_count           <= 32'd0;
            energy_ptr        <= '0;
            energy_count      <= '0;
            energy_sum        <= '0;
            gate_active       <= 1'b0;
            peak_ratio_num    <= '0;
            peak_energy_den   <= '0;
            peak_index        <= 32'd0;
            detection_armed   <= 1'b1;
            holdoff_counter   <= 32'd0;
            sample_idx        <= 32'd0;
            out_valid         <= 1'b0;
            detect_flag       <= 1'b0;
            out_ch0_i         <= '0;
            out_ch0_q         <= '0;
            out_ch1_i         <= '0;
            out_ch1_q         <= '0;
            flag_buffer       <= '0;
            dbg_metric_valid  <= 1'b0;
            dbg_metric_r      <= '0;
            dbg_metric_energy <= '0;
            for (idx = 0; idx < Q; idx++) begin
                delay_q_mem[idx]     = '0;
                sumB_fifo_real[idx]  = '0;
                sumB_fifo_imag[idx]  = '0;
                sumA_fifo_real[idx]  = '0;
                sumA_fifo_imag[idx]  = '0;
            end
            for (idx = 0; idx < P_DELAY_LEN; idx++) begin
                p_delay_mem_real[idx] = '0;
                p_delay_mem_imag[idx] = '0;
            end
            for (idx = 0; idx < ENERGY_LEN; idx++) begin
                energy_fifo[idx] = '0;
            end
            for (idx = 0; idx < BUFFER_LEN; idx++) begin
                sample_buffer[idx] = '0;
            end
        end else begin
            // Default outputs de-asserted each cycle unless overwritten later.
            out_valid   <= 1'b0;
            detect_flag <= 1'b0;
            dbg_metric_valid <= 1'b0;

            // Retain current gate state unless updated in the processing block.
            gate_active_next = gate_active;
            peak_ratio_num_next = peak_ratio_num;
            peak_energy_den_next = peak_energy_den;
            peak_index_next = peak_index;
            detection_armed_next = detection_armed;
            holdoff_counter_next = holdoff_counter;
            buffer_after_write = buffer_occupancy;
            pop_output = 1'b0;
            gate_closed_this_cycle = 1'b0;

            if (in_valid) begin
                // -----------------------------------------------------------------
                // Capture current sample and retrieve Q-delayed sample
                // -----------------------------------------------------------------
                current_sample.ch0_i = ch0_i;
                current_sample.ch0_q = ch0_q;
                current_sample.ch1_i = ch1_i;
                current_sample.ch1_q = ch1_q;

                delayed_sample = delay_q_mem[delay_q_ptr];
                delay_q_mem[delay_q_ptr] <= current_sample;
                delay_q_ptr <= (delay_q_ptr == Q_PTR_W'(Q-1)) ? '0 : delay_q_ptr + 1'b1;

                // -----------------------------------------------------------------
                // Compute per-sample energy across both antennas
                // -----------------------------------------------------------------
                ch0_i_sq = $signed(ch0_i) * $signed(ch0_i);
                ch0_q_sq = $signed(ch0_q) * $signed(ch0_q);
                ch1_i_sq = $signed(ch1_i) * $signed(ch1_i);
                ch1_q_sq = $signed(ch1_q) * $signed(ch1_q);

                ant0_energy = ch0_i_sq + ch0_q_sq;
                ant1_energy = ch1_i_sq + ch1_q_sq;
                energy_in = ant0_energy + ant1_energy;

                energy_sum_next = energy_sum;
                energy_count_next = energy_count;
                energy_old = energy_fifo[energy_ptr];
                energy_in_ext = {{(ENERGY_ACC_W-ENERGY_STREAM_W){1'b0}}, energy_in};
                energy_old_ext = {{(ENERGY_ACC_W-ENERGY_STREAM_W){1'b0}}, energy_old};
                if (energy_count == ENERGY_LEN[$clog2(ENERGY_LEN+1)-1:0]) begin
                    energy_sum_next = energy_sum_next + energy_in_ext - energy_old_ext;
                end else begin
                    energy_sum_next = energy_sum_next + energy_in_ext;
                    energy_count_next = energy_count + 1'b1;
                end
                energy_fifo[energy_ptr] <= energy_in;
                energy_ptr <= (energy_ptr == ENERGY_PTR_W'(ENERGY_LEN-1)) ? '0 : energy_ptr + 1'b1;
                energy_sum <= energy_sum_next;
                energy_count <= energy_count_next;

                // -----------------------------------------------------------------
                // Determine if enough history exists to form the product stream
                // -----------------------------------------------------------------
                has_q_delay = (sample_idx >= Q);
                p_real_sum = '0;
                p_imag_sum = '0;
                if (has_q_delay) begin
                    // Antenna 0 complex product x[n-Q] * conj(x[n])
                    p0_real = ($signed(delayed_sample.ch0_i) * $signed(ch0_i)) +
                              ($signed(delayed_sample.ch0_q) * $signed(ch0_q));
                    p0_imag = ($signed(delayed_sample.ch0_q) * $signed(ch0_i)) -
                              ($signed(delayed_sample.ch0_i) * $signed(ch0_q));
                    // Antenna 1
                    p1_real = ($signed(delayed_sample.ch1_i) * $signed(ch1_i)) +
                              ($signed(delayed_sample.ch1_q) * $signed(ch1_q));
                    p1_imag = ($signed(delayed_sample.ch1_q) * $signed(ch1_i)) -
                              ($signed(delayed_sample.ch1_i) * $signed(ch1_q));
                    p_real_sum = $signed(p0_real) + $signed(p1_real);
                    p_imag_sum = $signed(p0_imag) + $signed(p1_imag);
                end

                // -----------------------------------------------------------------
                // Update SumB (recent Q products) and prepare delayed stream for SumA
                // -----------------------------------------------------------------
                sumB_real_next = sumB_real;
                sumB_imag_next = sumB_imag;
                sumB_count_next = sumB_count;
                sumB_ptr_next = sumB_ptr;

                sumA_real_next = sumA_real;
                sumA_imag_next = sumA_imag;
                sumA_count_next = sumA_count;
                sumA_ptr_next = sumA_ptr;

                p_delay_ptr_next = p_delay_ptr;
                p_count_next = p_count;

                p_delay_ready = 1'b0;
                p_delay_real_val = '0;
                p_delay_imag_val = '0;

                if (has_q_delay) begin
                    p_delay_real_val = p_delay_mem_real[p_delay_ptr];
                    p_delay_imag_val = p_delay_mem_imag[p_delay_ptr];
                    p_delay_mem_real[p_delay_ptr] <= p_real_sum;
                    p_delay_mem_imag[p_delay_ptr] <= p_imag_sum;
                    p_delay_ptr_next = (p_delay_ptr == P_DELAY_PTR_W'(P_DELAY_LEN-1)) ? '0 : p_delay_ptr + 1'b1;
                    p_delay_ready = (p_count >= 2 * Q);
                    p_count_next = p_count + 1'b1;

                    p_real_ext = {{(CORR_SUM_W-P_STREAM_W){p_real_sum[P_STREAM_W-1]}}, p_real_sum};
                    p_imag_ext = {{(CORR_SUM_W-P_STREAM_W){p_imag_sum[P_STREAM_W-1]}}, p_imag_sum};
                    sumB_old_real = sumB_fifo_real[sumB_ptr];
                    sumB_old_imag = sumB_fifo_imag[sumB_ptr];
                    sumB_old_real_ext = {{(CORR_SUM_W-P_STREAM_W){sumB_old_real[P_STREAM_W-1]}}, sumB_old_real};
                    sumB_old_imag_ext = {{(CORR_SUM_W-P_STREAM_W){sumB_old_imag[P_STREAM_W-1]}}, sumB_old_imag};
                    if (sumB_count == Q[$clog2(Q+1)-1:0]) begin
                        sumB_real_next = sumB_real + p_real_ext - sumB_old_real_ext;
                        sumB_imag_next = sumB_imag + p_imag_ext - sumB_old_imag_ext;
                    end else begin
                        sumB_real_next = sumB_real + p_real_ext;
                        sumB_imag_next = sumB_imag + p_imag_ext;
                        sumB_count_next = sumB_count + 1'b1;
                    end
                    sumB_fifo_real[sumB_ptr] <= p_real_sum;
                    sumB_fifo_imag[sumB_ptr] <= p_imag_sum;
                    sumB_ptr_next = (sumB_ptr == Q_PTR_W'(Q-1)) ? '0 : sumB_ptr + 1'b1;
                end

                if (p_delay_ready) begin
                    p_delay_real_ext = {{(CORR_SUM_W-P_STREAM_W){p_delay_real_val[P_STREAM_W-1]}}, p_delay_real_val};
                    p_delay_imag_ext = {{(CORR_SUM_W-P_STREAM_W){p_delay_imag_val[P_STREAM_W-1]}}, p_delay_imag_val};
                    sumA_old_real = sumA_fifo_real[sumA_ptr];
                    sumA_old_imag = sumA_fifo_imag[sumA_ptr];
                    sumA_old_real_ext = {{(CORR_SUM_W-P_STREAM_W){sumA_old_real[P_STREAM_W-1]}}, sumA_old_real};
                    sumA_old_imag_ext = {{(CORR_SUM_W-P_STREAM_W){sumA_old_imag[P_STREAM_W-1]}}, sumA_old_imag};
                    if (sumA_count == Q[$clog2(Q+1)-1:0]) begin
                        sumA_real_next = sumA_real + p_delay_real_ext - sumA_old_real_ext;
                        sumA_imag_next = sumA_imag + p_delay_imag_ext - sumA_old_imag_ext;
                    end else begin
                        sumA_real_next = sumA_real + p_delay_real_ext;
                        sumA_imag_next = sumA_imag + p_delay_imag_ext;
                        sumA_count_next = sumA_count + 1'b1;
                    end
                    sumA_fifo_real[sumA_ptr] <= p_delay_real_val;
                    sumA_fifo_imag[sumA_ptr] <= p_delay_imag_val;
                    sumA_ptr_next = (sumA_ptr == Q_PTR_W'(Q-1)) ? '0 : sumA_ptr + 1'b1;
                end

                sumB_real <= sumB_real_next;
                sumB_imag <= sumB_imag_next;
                sumB_count <= sumB_count_next;
                sumB_ptr <= sumB_ptr_next;
                sumA_real <= sumA_real_next;
                sumA_imag <= sumA_imag_next;
                sumA_count <= sumA_count_next;
                sumA_ptr <= sumA_ptr_next;
                p_delay_ptr <= p_delay_ptr_next;
                p_count <= p_count_next;

                // -----------------------------------------------------------------
                // Metric computation (only valid once both sums and energy are full)
                // -----------------------------------------------------------------
                sums_ready = (sumA_count_next == Q[$clog2(Q+1)-1:0]) &&
                             (sumB_count_next == Q[$clog2(Q+1)-1:0]);
                energy_ready = (energy_count_next == ENERGY_LEN[$clog2(ENERGY_LEN+1)-1:0]);
                metric_valid = sums_ready && energy_ready;

                corr_real_total =
                    $signed({sumA_real_next[CORR_SUM_W-1], sumA_real_next}) +
                    $signed({sumB_real_next[CORR_SUM_W-1], sumB_real_next});
                R_value = (corr_real_total[R_WIDTH-1]) ? '0 : corr_real_total;
                ratio_num = {{15{R_value[R_WIDTH-1]}}, R_value} << 15;
                energy_value = energy_sum_next;
                energy_times_k = energy_value * K_Q15;
                threshold_scaled = '0;
                if (ENERGY_ASSIGN_W > 0) begin
                    threshold_scaled[ENERGY_ASSIGN_W-1:0] = energy_times_k[ENERGY_ASSIGN_W-1:0];
                end
                above_threshold = metric_valid && (energy_value != '0) && (ratio_num >= threshold_scaled);

                // Determine the window start index associated with this metric.
                window_start_idx = 32'd0;
                if (metric_valid && sample_idx >= WINDOW_DELAY) begin
                    window_start_idx = sample_idx - WINDOW_DELAY;
                end

                if (metric_valid) begin
                    dbg_metric_valid <= 1'b1;
                    dbg_metric_r <= R_value;
                    dbg_metric_energy <= energy_value;
                end

                // Gate management and peak tracking
                if (metric_valid && above_threshold && !gate_active && detection_armed_next) begin
                    gate_active_next = 1'b1;
                    detection_armed_next = 1'b0;
                    peak_ratio_num_next = ratio_num;
                    peak_energy_den_next = energy_value;
                    peak_index_next = window_start_idx;
                end else if (gate_active) begin
                    if (metric_valid && above_threshold) begin
                        // Compare current ratio (ratio_num/energy_value) to peak ratio
                        lhs = ratio_num * peak_energy_den_next;
                        rhs = peak_ratio_num_next * energy_value;
                        if (lhs >= rhs) begin
                            peak_ratio_num_next = ratio_num;
                            peak_energy_den_next = energy_value;
                            peak_index_next = window_start_idx;
                        end
                    end else if (!above_threshold) begin
                        // Finalize detection by writing a flag into the pending buffer position
                        gate_active_next = 1'b0;
                        if (peak_index_next >= base_sample_idx) begin
                            offset = peak_index_next - base_sample_idx;
                            offset_ext = (BUFFER_PTR_W+1)'(offset);
                            if (offset_ext < buffer_occupancy) begin
                                flag_ptr_ext = {1'b0, sample_rd_ptr} + offset_ext;
                                if (flag_ptr_ext >= BUFFER_LEN_EXT) begin
                                    flag_ptr_ext = flag_ptr_ext - BUFFER_LEN_EXT;
                                end
                                flag_buffer[flag_ptr_ext[BUFFER_PTR_W-1:0]] <= 1'b1;
                            end
                        end
                        holdoff_counter_next = N_FFT;
                        gate_closed_this_cycle = 1'b1;
                    end
                end

                // -----------------------------------------------------------------
                // Push current sample into the output buffer
                // -----------------------------------------------------------------
                sample_buffer[sample_wr_ptr] <= current_sample;
                flag_buffer[sample_wr_ptr] <= 1'b0;
                sample_wr_ptr <= (sample_wr_ptr == BUFFER_LEN_MINUS_ONE) ? '0 : sample_wr_ptr + 1'b1;

                buffer_after_write = buffer_after_write + 1'b1;
                if (buffer_after_write > BUFFER_LEN_EXT) begin
                    buffer_after_write = BUFFER_LEN_EXT;
                    $error("minn_frame_detector: output buffer overflow");
                end

                holdoff_temp = holdoff_counter_next;
                if (holdoff_temp != 0) begin
                    holdoff_temp = holdoff_temp - 1'b1;
                    holdoff_counter_next = holdoff_temp;
                    if (holdoff_temp == 0) begin
                        detection_armed_next = 1'b1;
                    end
                end

                sample_idx <= sample_idx + 1'b1;
            end

            // -----------------------------------------------------------------
            // Decide whether an output sample can be emitted this cycle
            // -----------------------------------------------------------------
            gate_can_pop = !gate_active_next || (gate_active_next && (peak_index_next > base_sample_idx));
            if (gate_closed_this_cycle) begin
                gate_can_pop = 1'b0;
            end
            pop_output = (sample_idx >= WINDOW_DELAY) && gate_can_pop && (buffer_after_write != 0);
            if (pop_output) begin
                out_valid <= 1'b1;
                out_sample = sample_buffer[sample_rd_ptr];
                out_ch0_i <= out_sample.ch0_i;
                out_ch0_q <= out_sample.ch0_q;
                out_ch1_i <= out_sample.ch1_i;
                out_ch1_q <= out_sample.ch1_q;
                detect_flag <= flag_buffer[sample_rd_ptr];
                flag_buffer[sample_rd_ptr] <= 1'b0;
                sample_rd_ptr <= (sample_rd_ptr == BUFFER_LEN_MINUS_ONE) ? '0 : sample_rd_ptr + 1'b1;
                buffer_occupancy <= buffer_after_write - 1'b1;
                base_sample_idx <= base_sample_idx + 1'b1;
            end else begin
                buffer_occupancy <= buffer_after_write;
            end

            gate_active <= gate_active_next;
            peak_ratio_num <= peak_ratio_num_next;
            peak_energy_den <= peak_energy_den_next;
            peak_index <= peak_index_next;
            detection_armed <= detection_armed_next;
            holdoff_counter <= holdoff_counter_next;
        end
    end
endmodule
