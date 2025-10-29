`default_nettype none

// Per-antenna datapath for Minn preamble detector.
// Computes quarter-lag correlation and running power sum with delayed taps.
module minn_antenna_datapath #(
    parameter int W_IN = 12,      // Input I/Q bit width
    parameter int Q = 512,        // Quarter length
    
    // Derived parameters (do not set at instantiation)
    parameter int W_PRODUCT = 2*W_IN + 1,       // 25 bits for W_IN=12
    parameter int W_POWER = 2*W_IN + 1,         // 25 bits for W_IN=12
    parameter int W_R = W_PRODUCT + $clog2(Q),  // 34 bits for Q=512
    parameter int W_E = W_POWER + $clog2(Q)     // 34 bits for Q=512
) (
    input  logic                    clk,
    input  logic                    rst,
    
    // Input complex sample stream
    input  logic                    in_valid,
    input  logic signed [W_IN-1:0]  in_i,
    input  logic signed [W_IN-1:0]  in_q,
    
    // Outputs: current and delayed correlation sums
    output logic                    out_valid,
    output logic signed [W_R-1:0]   r_current,    // R[n]
    output logic signed [W_R-1:0]   r_delayed,    // R[n-Q]
    
    // Outputs: current and delayed energy sums  
    output logic signed [W_E-1:0]   e_current,    // E[n]
    output logic signed [W_E-1:0]   e_delayed_1q, // E[n-Q]
    output logic signed [W_E-1:0]   e_delayed_2q  // E[n-2Q]
);

    // ========== Stage 1: Q-delay for complex input ==========
    
    // Delayed I/Q (complex Q-delay)
    logic signed [W_IN-1:0] i_delayed;
    logic signed [W_IN-1:0] q_delayed;
    logic delayed_valid;
    
    // Pack I/Q into single word for delay line
    localparam int COMPLEX_WIDTH = 2 * W_IN;
    logic [COMPLEX_WIDTH-1:0] iq_packed;
    logic [COMPLEX_WIDTH-1:0] iq_delayed_packed;
    
    always_comb begin
        iq_packed = {in_i, in_q};
        {i_delayed, q_delayed} = iq_delayed_packed;
    end
    
    minn_delay_line #(
        .WIDTH(COMPLEX_WIDTH),
        .DEPTH(Q)
    ) complex_delay_inst (
        .clk(clk),
        .rst(rst),
        .in_valid(in_valid),
        .in_data(iq_packed),
        .out_valid(delayed_valid),
        .out_data(iq_delayed_packed)
    );
    
    // ========== Stage 2: Quarter-lag product (real part only) ==========
    
    // p[n] = Re{x[n-Q] * conj(x[n])} = I[n-Q]*I[n] + Q[n-Q]*Q[n]
    logic signed [W_PRODUCT-1:0] product_ii;
    logic signed [W_PRODUCT-1:0] product_qq;
    logic signed [W_PRODUCT-1:0] quarter_lag_product;
    
    always_ff @(posedge clk) begin
        if (rst == 1'b1) begin
            product_ii <= '0;
            product_qq <= '0;
            quarter_lag_product <= '0;
        end else begin
            if (delayed_valid == 1'b1) begin
                product_ii <= i_delayed * in_i;
                product_qq <= q_delayed * in_q;
                quarter_lag_product <= product_ii + product_qq;
            end
        end
    end
    
    // ========== Stage 3: Running sum R (correlation) ==========
    
    logic r_valid;
    logic signed [W_R-1:0] r_sum;
    
    minn_running_sum #(
        .IN_WIDTH(W_PRODUCT),
        .DEPTH(Q),
        .OUT_WIDTH(W_R)
    ) r_running_sum_inst (
        .clk(clk),
        .rst(rst),
        .in_valid(delayed_valid),
        .in_data(quarter_lag_product),
        .out_valid(r_valid),
        .out_sum(r_sum)
    );
    
    // R delay line to get R[n-Q]
    logic signed [W_R-1:0] r_delayed_internal;
    logic r_delayed_valid;
    
    minn_delay_line #(
        .WIDTH(W_R),
        .DEPTH(Q)
    ) r_delay_inst (
        .clk(clk),
        .rst(rst),
        .in_valid(r_valid),
        .in_data(r_sum),
        .out_valid(r_delayed_valid),
        .out_data(r_delayed_internal)
    );
    
    // ========== Stage 4: Power computation ==========
    
    // w[n] = |x[n]|^2 = I[n]^2 + Q[n]^2
    logic signed [W_POWER-1:0] power_i;
    logic signed [W_POWER-1:0] power_q;
    logic signed [W_POWER-1:0] instantaneous_power;
    
    always_ff @(posedge clk) begin
        if (rst == 1'b1) begin
            power_i <= '0;
            power_q <= '0;
            instantaneous_power <= '0;
        end else begin
            if (in_valid == 1'b1) begin
                power_i <= in_i * in_i;
                power_q <= in_q * in_q;
                instantaneous_power <= power_i + power_q;
            end
        end
    end
    
    // ========== Stage 5: Running sum E (energy) ==========
    
    logic e_valid;
    logic signed [W_E-1:0] e_sum;
    
    minn_running_sum #(
        .IN_WIDTH(W_POWER),
        .DEPTH(Q),
        .OUT_WIDTH(W_E)
    ) e_running_sum_inst (
        .clk(clk),
        .rst(rst),
        .in_valid(in_valid),
        .in_data(instantaneous_power),
        .out_valid(e_valid),
        .out_sum(e_sum)
    );
    
    // E delay line to get E[n-Q]
    logic signed [W_E-1:0] e_delayed_1q_internal;
    logic e_delayed_1q_valid;
    
    minn_delay_line #(
        .WIDTH(W_E),
        .DEPTH(Q)
    ) e_delay_1q_inst (
        .clk(clk),
        .rst(rst),
        .in_valid(e_valid),
        .in_data(e_sum),
        .out_valid(e_delayed_1q_valid),
        .out_data(e_delayed_1q_internal)
    );
    
    // E delay line to get E[n-2Q]
    logic signed [W_E-1:0] e_delayed_2q_internal;
    logic e_delayed_2q_valid;
    
    minn_delay_line #(
        .WIDTH(W_E),
        .DEPTH(Q)
    ) e_delay_2q_inst (
        .clk(clk),
        .rst(rst),
        .in_valid(e_delayed_1q_valid),
        .in_data(e_delayed_1q_internal),
        .out_valid(e_delayed_2q_valid),
        .out_data(e_delayed_2q_internal)
    );
    
    // ========== Output Assignment ==========
    
    // Register outputs for timing
    always_ff @(posedge clk) begin
        if (rst == 1'b1) begin
            out_valid <= 1'b0;
            r_current <= '0;
            r_delayed <= '0;
            e_current <= '0;
            e_delayed_1q <= '0;
            e_delayed_2q <= '0;
        end else begin
            // All outputs valid when we have the full delay chain populated
            out_valid <= e_delayed_2q_valid;
            r_current <= r_sum;
            r_delayed <= r_delayed_internal;
            e_current <= e_sum;
            e_delayed_1q <= e_delayed_1q_internal;
            e_delayed_2q <= e_delayed_2q_internal;
        end
    end

endmodule

`default_nettype wire