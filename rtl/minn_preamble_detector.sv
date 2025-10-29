`default_nettype none

// Top-level Minn preamble detector for OFDM systems.
// Implements streaming correlation-based detection with dual antennas.
// Based on Minn timing metric with fixed threshold and IIR smoothing.
module minn_preamble_detector #(
    parameter int NFFT          = 2048,        // FFT size
    parameter int W_IN          = 12,          // Input I/Q bit width
    parameter int THRESHOLD     = 16'h1800,    // Q1.15 threshold (~0.09375)
    parameter int IIR_SHIFT     = 3,           // Smoother shift amount (S)
    parameter int HYSTERESIS    = 2,           // Gate close hysteresis samples
    parameter int TIMING_OFFSET = 0,           // Signed offset for flag alignment
    parameter int OUTPUT_DELAY  = 2048,        // Output FIFO delay (typically = NFFT)
    
    // Derived parameters (do not set at instantiation)
    parameter int Q             = NFFT / 4,
    parameter int DEPTH         = OUTPUT_DELAY + 64,
    parameter int W_R           = 34,          // Correlation sum width
    parameter int W_E           = 34,          // Energy sum width
    parameter int W_C           = 36,          // Combined correlation width
    parameter int W_E_TOT       = 37,          // Combined energy width
    parameter int ADDR_WIDTH    = $clog2(DEPTH)
) (
    input  logic                    clk,
    input  logic                    rst,
    
    // Input samples (continuous stream, two antennas)
    input  logic                    in_valid,
    input  logic signed [W_IN-1:0]  ch0_i,
    input  logic signed [W_IN-1:0]  ch0_q,
    input  logic signed [W_IN-1:0]  ch1_i,
    input  logic signed [W_IN-1:0]  ch1_q,
    
    // Output samples (delayed stream with frame flag)
    output logic                    out_valid,
    output logic signed [W_IN-1:0]  out_ch0_i,
    output logic signed [W_IN-1:0]  out_ch0_q,
    output logic signed [W_IN-1:0]  out_ch1_i,
    output logic signed [W_IN-1:0]  out_ch1_q,
    output logic                    frame_start,
    
    // Optional debug outputs
    output logic [W_C-1:0]          dbg_metric,
    output logic [W_E_TOT-1:0]      dbg_energy,
    output logic                    dbg_above_thresh
);

    // ========== Antenna 0 Datapath ==========
    
    logic ant0_valid;
    logic signed [W_R-1:0] ant0_r_current;
    logic signed [W_R-1:0] ant0_r_delayed;
    logic signed [W_E-1:0] ant0_e_current;
    logic signed [W_E-1:0] ant0_e_delayed_1q;
    logic signed [W_E-1:0] ant0_e_delayed_2q;
    
    minn_antenna_datapath #(
        .W_IN(W_IN),
        .Q(Q)
    ) antenna0_inst (
        .clk(clk),
        .rst(rst),
        .in_valid(in_valid),
        .in_i(ch0_i),
        .in_q(ch0_q),
        .out_valid(ant0_valid),
        .r_current(ant0_r_current),
        .r_delayed(ant0_r_delayed),
        .e_current(ant0_e_current),
        .e_delayed_1q(ant0_e_delayed_1q),
        .e_delayed_2q(ant0_e_delayed_2q)
    );
    
    // ========== Antenna 1 Datapath ==========
    
    logic ant1_valid;
    logic signed [W_R-1:0] ant1_r_current;
    logic signed [W_R-1:0] ant1_r_delayed;
    logic signed [W_E-1:0] ant1_e_current;
    logic signed [W_E-1:0] ant1_e_delayed_1q;
    logic signed [W_E-1:0] ant1_e_delayed_2q;
    
    minn_antenna_datapath #(
        .W_IN(W_IN),
        .Q(Q)
    ) antenna1_inst (
        .clk(clk),
        .rst(rst),
        .in_valid(in_valid),
        .in_i(ch1_i),
        .in_q(ch1_q),
        .out_valid(ant1_valid),
        .r_current(ant1_r_current),
        .r_delayed(ant1_r_delayed),
        .e_current(ant1_e_current),
        .e_delayed_1q(ant1_e_delayed_1q),
        .e_delayed_2q(ant1_e_delayed_2q)
    );
    
    // ========== Cross-Antenna Combiner ==========
    
    // Valid signal (both antennas must be valid)
    logic combined_valid;
    
    // Extended widths for safe arithmetic
    logic signed [W_R:0] ant0_r_sum_ext;
    logic signed [W_R:0] ant1_r_sum_ext;
    logic signed [W_C-1:0] c_raw;
    logic signed [W_C-1:0] c_pos;
    
    logic signed [W_E:0] ant0_e3_ext;
    logic signed [W_E:0] ant1_e3_ext;
    logic signed [W_E_TOT-1:0] e_total;
    
    always_comb begin
        combined_valid = ant0_valid & ant1_valid;
        
        // Sum correlations: C = (R0[n] + R0[n-Q]) + (R1[n] + R1[n-Q])
        ant0_r_sum_ext = ant0_r_current + ant0_r_delayed;
        ant1_r_sum_ext = ant1_r_current + ant1_r_delayed;
        c_raw = ant0_r_sum_ext + ant1_r_sum_ext;
        
        // Clamp negative to zero: C_pos = max(C, 0)
        c_pos = (c_raw[W_C-1] == 1'b1) ? '0 : c_raw;
        
        // Sum energies: E_tot = (E0[n] + E0[n-Q] + E0[n-2Q]) + (E1[n] + E1[n-Q] + E1[n-2Q])
        ant0_e3_ext = ant0_e_current + ant0_e_delayed_1q + ant0_e_delayed_2q;
        ant1_e3_ext = ant1_e_current + ant1_e_delayed_1q + ant1_e_delayed_2q;
        e_total = ant0_e3_ext + ant1_e3_ext;
    end
    
    // ========== IIR Smoother ==========
    
    // C_smooth[n] = C_smooth[n-1] + (C_pos[n] - C_smooth[n-1]) >> IIR_SHIFT
    logic signed [W_C-1:0] c_smooth;
    logic signed [W_C-1:0] c_diff;
    logic signed [W_C-1:0] c_increment;
    
    initial begin
        c_smooth = '0;
    end
    
    always_comb begin
        c_diff = c_pos - c_smooth;
        c_increment = c_diff >>> IIR_SHIFT;  // Arithmetic right shift
    end
    
    always_ff @(posedge clk) begin
        if (rst == 1'b1) begin
            c_smooth <= '0;
        end else begin
            if (combined_valid == 1'b1) begin
                c_smooth <= c_smooth + c_increment;
            end
        end
    end
    
    // ========== Threshold Comparison ==========
    
    // Compare: C_smooth >= T × E_total
    // Using cross-multiplication to avoid division
    logic signed [W_E_TOT+15:0] threshold_product;  // T is 16-bit Q1.15
    logic above_threshold;
    logic threshold_valid;
    
    // Extend c_smooth to match product width for comparison
    logic signed [W_E_TOT+15:0] c_smooth_ext;
    
    always_comb begin
        // Multiply E_total by threshold (signed × unsigned treated as signed)
        threshold_product = e_total * THRESHOLD;
        
        // Extend c_smooth for comparison
        c_smooth_ext = {{(W_E_TOT+16-W_C){c_smooth[W_C-1]}}, c_smooth};
        
        // Compare (both are signed)
        above_threshold = (c_smooth_ext >= threshold_product) ? 1'b1 : 1'b0;
        threshold_valid = combined_valid;
    end
    
    // ========== Gate & Peak Detection FSM ==========
    
    logic detection_flag;
    logic [ADDR_WIDTH-1:0] flag_address;
    logic [ADDR_WIDTH-1:0] current_wptr;
    
    minn_gate_peak_fsm #(
        .METRIC_WIDTH(W_C),
        .ADDR_WIDTH(ADDR_WIDTH),
        .HYSTERESIS(HYSTERESIS),
        .TIMING_OFFSET(TIMING_OFFSET)
    ) fsm_inst (
        .clk(clk),
        .rst(rst),
        .metric_valid(threshold_valid),
        .above_threshold(above_threshold),
        .metric_value(c_smooth),
        .current_wptr(current_wptr),
        .detection_flag(detection_flag),
        .flag_address(flag_address)
    );
    
    // ========== Output Circular Buffer ==========
    
    minn_output_buffer #(
        .W_IN(W_IN),
        .DEPTH(DEPTH),
        .OUTPUT_DELAY(OUTPUT_DELAY)
    ) output_buffer_inst (
        .clk(clk),
        .rst(rst),
        .in_valid(in_valid),
        .in_ch0_i(ch0_i),
        .in_ch0_q(ch0_q),
        .in_ch1_i(ch1_i),
        .in_ch1_q(ch1_q),
        .flag_write(detection_flag),
        .flag_addr(flag_address),
        .current_wptr(current_wptr),
        .out_valid(out_valid),
        .out_ch0_i(out_ch0_i),
        .out_ch0_q(out_ch0_q),
        .out_ch1_i(out_ch1_i),
        .out_ch1_q(out_ch1_q),
        .frame_start(frame_start)
    );
    
    // ========== Debug Outputs ==========
    
    always_ff @(posedge clk) begin
        if (rst == 1'b1) begin
            dbg_metric <= '0;
            dbg_energy <= '0;
            dbg_above_thresh <= 1'b0;
        end else begin
            dbg_metric <= c_smooth;
            dbg_energy <= e_total;
            dbg_above_thresh <= above_threshold;
        end
    end

endmodule

`default_nettype wire