`default_nettype none

// Gate-and-peak detection FSM for Minn preamble detector.
// Tracks the peak metric value when above threshold, applies hysteresis,
// and generates detection flag with timing offset.
module minn_gate_peak_fsm #(
    parameter int METRIC_WIDTH = 36,
    parameter int ADDR_WIDTH = 12,
    parameter int HYSTERESIS = 2,
    parameter int TIMING_OFFSET = 0
) (
    input  logic                        clk,
    input  logic                        rst,
    
    // Metric input
    input  logic                        metric_valid,
    input  logic                        above_threshold,
    input  logic [METRIC_WIDTH-1:0]    metric_value,
    
    // Current write pointer from output buffer
    input  logic [ADDR_WIDTH-1:0]      current_wptr,
    
    // Detection outputs
    output logic                        detection_flag,
    output logic [ADDR_WIDTH-1:0]      flag_address
);

    // FSM states
    typedef enum logic [1:0] {
        IDLE = 2'b00,
        GATE_OPEN = 2'b01,
        GATE_CLOSING = 2'b10
    } state_t;
    
    state_t state;
    
    // Peak tracking registers
    logic [METRIC_WIDTH-1:0] peak_value;
    logic [ADDR_WIDTH-1:0] peak_wptr;
    
    // Hysteresis counter
    logic [$clog2(HYSTERESIS+1)-1:0] hyst_counter;
    
    // Timing offset computation (signed)
    logic signed [ADDR_WIDTH:0] offset_extended;
    logic [ADDR_WIDTH-1:0] flag_addr_computed;
    
    initial begin
        state = IDLE;
        peak_value = '0;
        peak_wptr = '0;
        hyst_counter = '0;
        detection_flag = 1'b0;
        flag_address = '0;
    end
    
    // Extend timing offset to address width + 1 for signed arithmetic
    always_comb begin
        offset_extended = TIMING_OFFSET;
    end
    
    // FSM and peak tracking logic
    always_ff @(posedge clk) begin
        if (rst == 1'b1) begin
            state <= IDLE;
            peak_value <= '0;
            peak_wptr <= '0;
            hyst_counter <= '0;
            detection_flag <= 1'b0;
            flag_address <= '0;
        end else begin
            // Default: no detection
            detection_flag <= 1'b0;
            
            if (metric_valid == 1'b1) begin
                case (state)
                    
                    IDLE: begin
                        // Wait for metric to exceed threshold
                        if (above_threshold == 1'b1) begin
                            state <= GATE_OPEN;
                            peak_value <= metric_value;
                            peak_wptr <= current_wptr;
                        end
                    end
                    
                    GATE_OPEN: begin
                        if (above_threshold == 1'b1) begin
                            // Still above threshold: update peak if larger
                            if (metric_value > peak_value) begin
                                peak_value <= metric_value;
                                peak_wptr <= current_wptr;
                            end
                            state <= GATE_OPEN;
                        end else begin
                            // Dropped below threshold: start hysteresis countdown
                            state <= GATE_CLOSING;
                            hyst_counter <= HYSTERESIS[$clog2(HYSTERESIS+1)-1:0];
                        end
                    end
                    
                    GATE_CLOSING: begin
                        if (above_threshold == 1'b1) begin
                            // Went back above threshold: reopen gate
                            state <= GATE_OPEN;
                            // Update peak if this new value is larger
                            if (metric_value > peak_value) begin
                                peak_value <= metric_value;
                                peak_wptr <= current_wptr;
                            end
                        end else begin
                            // Still below threshold: decrement hysteresis
                            if (hyst_counter > 0) begin
                                hyst_counter <= hyst_counter - 1;
                                state <= GATE_CLOSING;
                            end else begin
                                // Hysteresis expired: declare detection
                                state <= IDLE;
                                detection_flag <= 1'b1;
                                
                                // Compute flag address with timing offset
                                // Note: modulo arithmetic happens naturally with address width
                                flag_addr_computed = peak_wptr + offset_extended[ADDR_WIDTH-1:0];
                                flag_address <= flag_addr_computed;
                            end
                        end
                    end
                    
                    default: begin
                        state <= IDLE;
                    end
                    
                endcase
            end
        end
    end

endmodule

`default_nettype wire