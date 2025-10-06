// Dual-channel NCO-based CFO compensator with AXI4-Stream I/Q pairs and AXI4-Lite control.
// The datapath time-multiplexes two DSP multipliers across both channels, leveraging the
// 8x fabric clock to reduce multiplier usage while meeting 30.72 Msps throughput.
module nco_cfo_compensator #(
    parameter int WIDTH = 12,
    parameter int NUM_CHANNELS = 2,
    parameter int AXIS_DATA_WIDTH = NUM_CHANNELS * 2 * WIDTH,
    parameter int ACC_WIDTH = 32,
    parameter int LUT_ADDR_WIDTH = 8,
    parameter int LUT_DATA_WIDTH = 16
) (
    input  logic                           clk,
    input  logic                           rst_n,

    // AXI4-Stream input
    input  logic [AXIS_DATA_WIDTH-1:0]     s_axis_tdata,
    input  logic                           s_axis_tvalid,
    output logic                           s_axis_tready,
    input  logic                           s_axis_tlast,

    // AXI4-Stream output
    output logic [AXIS_DATA_WIDTH-1:0]     m_axis_tdata,
    output logic                           m_axis_tvalid,
    input  logic                           m_axis_tready,
    output logic                           m_axis_tlast,

    // AXI4-Lite control (CFO phase increment at address 0x0)
    input  logic                           s_axi_awvalid,
    output logic                           s_axi_awready,
    input  logic  [3:0]                    s_axi_awaddr,
    input  logic                           s_axi_wvalid,
    output logic                           s_axi_wready,
    input  logic  [31:0]                   s_axi_wdata,
    input  logic  [3:0]                    s_axi_wstrb,
    output logic                           s_axi_bvalid,
    input  logic                           s_axi_bready,
    output logic  [1:0]                    s_axi_bresp,
    input  logic                           s_axi_arvalid,
    output logic                           s_axi_arready,
    input  logic  [3:0]                    s_axi_araddr,
    output logic                           s_axi_rvalid,
    input  logic                           s_axi_rready,
    output logic [31:0]                    s_axi_rdata,
    output logic  [1:0]                    s_axi_rresp
);
    localparam int FRACTION_BITS = LUT_DATA_WIDTH - 1;
    localparam int LUT_SIZE = 1 << LUT_ADDR_WIDTH;

    // NCO look-up tables for sine and cosine (signed 1.15 format when LUT_DATA_WIDTH=16).
    (* rom_style = "distributed", ram_style = "distributed" *)
    logic signed [LUT_DATA_WIDTH-1:0] cos_lut [0:LUT_SIZE-1];
    (* rom_style = "distributed", ram_style = "distributed" *)
    logic signed [LUT_DATA_WIDTH-1:0] sin_lut [0:LUT_SIZE-1];

    initial begin
        real angle;
        int amplitude;
        int cos_tmp;
        int sin_tmp;
        logic signed [31:0] cos_bits;
        logic signed [31:0] sin_bits;
        amplitude = (1 << (LUT_DATA_WIDTH - 1)) - 1;
        for (int idx = 0; idx < LUT_SIZE; idx++) begin
            angle = 6.28318530717958647692 * idx / LUT_SIZE;
            cos_tmp = $rtoi($cos(angle) * amplitude);
            sin_tmp = $rtoi($sin(angle) * amplitude);
            cos_bits = cos_tmp;
            sin_bits = sin_tmp;
            cos_lut[idx] = cos_bits[LUT_DATA_WIDTH-1:0];
            sin_lut[idx] = sin_bits[LUT_DATA_WIDTH-1:0];
        end
    end

    function automatic logic signed [WIDTH+LUT_DATA_WIDTH:0] extend_product (
        input logic signed [WIDTH+LUT_DATA_WIDTH-1:0] value
    );
        extend_product = {{1{value[WIDTH+LUT_DATA_WIDTH-1]}}, value};
    endfunction

    logic [ACC_WIDTH-1:0] phase_inc;
    logic [ACC_WIDTH-1:0] phase_acc;
    logic [ACC_WIDTH-1:0] phase_acc_next;
    wire  [LUT_ADDR_WIDTH-1:0] phase_index = phase_acc[ACC_WIDTH-1 -: LUT_ADDR_WIDTH];

    logic signed [LUT_DATA_WIDTH-1:0] cos_val_latched;
    logic signed [LUT_DATA_WIDTH-1:0] sin_val_latched;
    logic                             tlast_latched;

    logic signed [WIDTH-1:0] sample_i [NUM_CHANNELS];
    logic signed [WIDTH-1:0] sample_q [NUM_CHANNELS];
    logic signed [WIDTH-1:0] result_i [NUM_CHANNELS];
    logic signed [WIDTH-1:0] result_q [NUM_CHANNELS];

    logic [AXIS_DATA_WIDTH-1:0] packed_comb;

    typedef enum logic [1:0] {
        STATE_IDLE,
        STATE_COMPUTE,
        STATE_PACK,
        STATE_OUTPUT
    } state_t;

    typedef enum logic [1:0] {
        STEP_PRIME  = 2'd0,
        STEP_REAL   = 2'd1,
        STEP_IMAG   = 2'd2
    } step_t;

    localparam int CHANNEL_COUNTER_WIDTH = (NUM_CHANNELS <= 1) ? 1 : $clog2(NUM_CHANNELS);
    localparam logic [CHANNEL_COUNTER_WIDTH-1:0] LAST_CHANNEL = CHANNEL_COUNTER_WIDTH'(NUM_CHANNELS-1);

    state_t state;
    step_t  compute_step;
    logic [CHANNEL_COUNTER_WIDTH-1:0] channel_idx;

    logic signed [WIDTH-1:0]           mult_op_a;
    logic signed [LUT_DATA_WIDTH-1:0]  mult_op_b;
    logic signed [WIDTH-1:0]           mult_op_c;
    logic signed [LUT_DATA_WIDTH-1:0]  mult_op_d;
    (* use_dsp = "yes" *) logic signed [WIDTH+LUT_DATA_WIDTH-1:0] mult_res_a;
    (* use_dsp = "yes" *) logic signed [WIDTH+LUT_DATA_WIDTH-1:0] mult_res_b;

    (* use_dsp = "no" *) logic signed [WIDTH+LUT_DATA_WIDTH:0] real_temp;
    (* use_dsp = "no" *) logic signed [WIDTH+LUT_DATA_WIDTH:0] imag_temp;

    always_comb begin
        packed_comb = '0;
        for (int ch = 0; ch < NUM_CHANNELS; ch++) begin
            int base = ch * 2 * WIDTH;
            packed_comb[base +: WIDTH]          = result_i[ch];
            packed_comb[base + WIDTH +: WIDTH] = result_q[ch];
        end
    end

    // Multipliers mapped to two DSP blocks (combinational to ease time-multiplexing).
    assign mult_res_a = $signed(mult_op_a) * $signed(mult_op_b);
    assign mult_res_b = $signed(mult_op_c) * $signed(mult_op_d);

    // Helper for saturation after fixed-point shift.
    function automatic logic signed [WIDTH-1:0] sat_shift (
        input logic signed [WIDTH+LUT_DATA_WIDTH:0] value
    );
        logic signed [WIDTH+LUT_DATA_WIDTH:0] shifted;
        logic signed [WIDTH-1:0] max_base;
        logic signed [WIDTH-1:0] min_base;
        logic signed [WIDTH+LUT_DATA_WIDTH:0] max_val;
        logic signed [WIDTH+LUT_DATA_WIDTH:0] min_val;
        begin
            shifted = value >>> FRACTION_BITS;
            max_base = (1 << (WIDTH - 1)) - 1;
            min_base = -(1 << (WIDTH - 1));
            max_val = {{(LUT_DATA_WIDTH+1){max_base[WIDTH-1]}}, max_base};
            min_val = {{(LUT_DATA_WIDTH+1){min_base[WIDTH-1]}}, min_base};
            if (shifted > max_val) begin
                shifted = max_val;
            end else if (shifted < min_val) begin
                shifted = min_val;
            end
            sat_shift = shifted[WIDTH-1:0];
        end
    endfunction

    // AXI4-Stream ready/valid management.
    assign s_axis_tready = (state == STATE_IDLE);

    // AXI4-Lite write channel
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            phase_inc     <= '0;
        end else begin
            if (!s_axi_awready && s_axi_awvalid) begin
                s_axi_awready <= 1'b1;
            end else begin
                s_axi_awready <= 1'b0;
            end

            if (!s_axi_wready && s_axi_wvalid) begin
                s_axi_wready <= 1'b1;
            end else begin
                s_axi_wready <= 1'b0;
            end

            if (s_axi_awready && s_axi_awvalid && s_axi_wready && s_axi_wvalid) begin
                if (s_axi_awaddr[3:0] == 4'h0 && s_axi_wstrb[0]) begin
                    phase_inc <= s_axi_wdata[ACC_WIDTH-1:0];
                end
                s_axi_bvalid <= 1'b1;
            end else if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end

    assign s_axi_bresp = 2'b00;

    // AXI4-Lite read channel
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= '0;
        end else begin
            if (!s_axi_arready && s_axi_arvalid) begin
                s_axi_arready <= 1'b1;
            end else begin
                s_axi_arready <= 1'b0;
            end

            if (s_axi_arready && s_axi_arvalid && !s_axi_rvalid) begin
                case (s_axi_araddr[3:0])
                    default: s_axi_rdata <= {{(32-ACC_WIDTH){1'b0}}, phase_inc};
                endcase
                s_axi_rvalid <= 1'b1;
            end else if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

    assign s_axi_rresp = 2'b00;

    // Stream datapath with time-multiplexed multipliers.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= STATE_IDLE;
            compute_step <= STEP_PRIME;
            channel_idx  <= '0;
            phase_acc    <= '0;
            phase_acc_next <= '0;
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;
            m_axis_tdata  <= '0;
            cos_val_latched <= '0;
            sin_val_latched <= '0;
            real_temp <= '0;
            mult_op_a <= '0;
            mult_op_b <= '0;
            mult_op_c <= '0;
            mult_op_d <= '0;
            tlast_latched <= 1'b0;
            for (int ch = 0; ch < NUM_CHANNELS; ch++) begin
                sample_i[ch] <= '0;
                sample_q[ch] <= '0;
                result_i[ch] <= '0;
                result_q[ch] <= '0;
            end
        end else begin
            case (state)
                STATE_IDLE: begin
                    if (s_axis_tvalid && s_axis_tready) begin
                        for (int ch = 0; ch < NUM_CHANNELS; ch++) begin
                            int base = ch * 2 * WIDTH;
                            sample_i[ch] <= s_axis_tdata[base +: WIDTH];
                            sample_q[ch] <= s_axis_tdata[base + WIDTH +: WIDTH];
                        end
                        cos_val_latched <= cos_lut[phase_index];
                        sin_val_latched <= sin_lut[phase_index];
                        tlast_latched   <= s_axis_tlast;
                        phase_acc_next  <= phase_acc + phase_inc;
                        channel_idx     <= '0;
                        compute_step    <= STEP_PRIME;
                        state           <= STATE_COMPUTE;
                    end
                end

                STATE_COMPUTE: begin
                    case (compute_step)
                        STEP_PRIME: begin
                            mult_op_a <= sample_i[channel_idx];
                            mult_op_b <= cos_val_latched;
                            mult_op_c <= sample_q[channel_idx];
                            mult_op_d <= sin_val_latched;
                            compute_step <= STEP_REAL;
                        end

                        STEP_REAL: begin
                            real_temp <= extend_product(mult_res_a) + extend_product(mult_res_b);
                            mult_op_a <= sample_q[channel_idx];
                            mult_op_b <= cos_val_latched;
                            mult_op_c <= sample_i[channel_idx];
                            mult_op_d <= sin_val_latched;
                            compute_step <= STEP_IMAG;
                        end

                        STEP_IMAG: begin
                            logic signed [WIDTH-1:0]             sat_real;
                            logic signed [WIDTH-1:0]             sat_imag;
                            imag_temp = extend_product(mult_res_a) - extend_product(mult_res_b);
                            sat_real = sat_shift(real_temp);
                            sat_imag = sat_shift(imag_temp);
                            result_i[channel_idx] <= sat_real;
                            result_q[channel_idx] <= sat_imag;

                            if (channel_idx == LAST_CHANNEL) begin
                                state        <= STATE_PACK;
                                compute_step <= STEP_PRIME;
                            end else begin
                                channel_idx   <= channel_idx + 1'b1;
                                compute_step  <= STEP_PRIME;
                            end
                        end
                        default: compute_step <= STEP_PRIME;
                    endcase
                end

                STATE_PACK: begin
                    m_axis_tdata  <= packed_comb;
                    m_axis_tlast  <= tlast_latched;
                    m_axis_tvalid <= 1'b1;
                    state         <= STATE_OUTPUT;
                end

                STATE_OUTPUT: begin
                    if (m_axis_tvalid && m_axis_tready) begin
                        m_axis_tvalid <= 1'b0;
                        phase_acc     <= phase_acc_next;
                        state         <= STATE_IDLE;
                    end
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end
endmodule
