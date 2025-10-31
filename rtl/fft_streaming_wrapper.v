`default_nettype none

module fft_streaming_wrapper #(
    parameter integer NFFT                = 2048,
    parameter integer SAMPLES_PER_CYCLE   = 2,
    parameter integer GAP_CYCLES          = 11287,
    parameter integer INPUT_SHIFT         = 0,
    parameter integer OUTPUT_SHIFT        = 0,
    parameter integer FIFO_DEPTH          = 4096
) (
    input  wire         clk_axis,
    input  wire         rst_axis_n,
    input  wire         clk_fft,
    input  wire         rst_fft_n,

    input  wire [47:0]  s_axis_tdata,
    input  wire [6:0]   s_axis_tuser,
    input  wire         s_axis_tvalid,
    input  wire         s_axis_tlast,
    output reg          s_axis_tready,

    output reg  [47:0]  m_axis_tdata,
    output reg  [6:0]   m_axis_tuser,
    output reg          m_axis_tvalid,
    output reg          m_axis_tlast,
    input  wire         m_axis_tready,

    output reg  [5:0]   status_flags,
    output reg  [15:0]  latency_debug
);

    localparam integer SAMPLE_WIDTH        = 12;
    localparam integer CORE_WIDTH          = 12;
    localparam integer OUTPUT_GUARD_WIDTH  = 18;
    localparam integer COMPLEX_WIDTH       = 2 * CORE_WIDTH;
    localparam integer PAIR_COUNT          = NFFT / SAMPLES_PER_CYCLE;
    localparam integer PAIR_WORD_WIDTH     = 2 * COMPLEX_WIDTH;
    localparam integer OUTPUT_PAIR_WIDTH   = 8 * OUTPUT_GUARD_WIDTH;
    localparam integer TOKEN_WIDTH         = 8;
    localparam integer INPUT_TEMP_WIDTH    = SAMPLE_WIDTH + 4;
    localparam integer OUTPUT_TEMP_WIDTH   = OUTPUT_GUARD_WIDTH + CORE_WIDTH;
    localparam signed [CORE_WIDTH-1:0] CORE_MAX = {1'b0, {(CORE_WIDTH-1){1'b1}}};
    localparam signed [CORE_WIDTH-1:0] CORE_MIN = {1'b1, {(CORE_WIDTH-1){1'b0}}};
    localparam signed [OUTPUT_GUARD_WIDTH-1:0] OUTPUT_MAX = {1'b0, {(OUTPUT_GUARD_WIDTH-1){1'b1}}};
    localparam signed [OUTPUT_GUARD_WIDTH-1:0] OUTPUT_MIN = {1'b1, {(OUTPUT_GUARD_WIDTH-1){1'b0}}};
    localparam signed [SAMPLE_WIDTH-1:0] SAMPLE_MAX = {1'b0, {(SAMPLE_WIDTH-1){1'b1}}};
    localparam signed [SAMPLE_WIDTH-1:0] SAMPLE_MIN = {1'b1, {(SAMPLE_WIDTH-1){1'b0}}};

    // ---------------------------------------------------------------------
    // Utility functions
    // ---------------------------------------------------------------------

    function integer clog2_int;
        input integer value;
        integer i;
        begin
            clog2_int = 0;
            for (i = value - 1; i > 0; i = i >> 1) begin
                clog2_int = clog2_int + 1;
            end
        end
    endfunction

    localparam integer PAIR_ADDR_WIDTH = clog2_int(PAIR_COUNT);
    localparam integer FIFO_ADDR_WIDTH = clog2_int(FIFO_DEPTH);
    localparam integer GAP_COUNTER_WIDTH = clog2_int(GAP_CYCLES + 1);
    localparam integer BIN_COUNTER_WIDTH = clog2_int(NFFT + 1);
    // PAIR_COUNT is configured as a power of two so the final index is all ones.
    localparam [PAIR_ADDR_WIDTH-1:0] PAIR_LAST = {PAIR_ADDR_WIDTH{1'b1}};
    localparam [PAIR_ADDR_WIDTH:0]   PAIR_LAST_EXT = {1'b0, {PAIR_ADDR_WIDTH{1'b1}}};

    // Saturating shift helper for ingress data (axis domain).
    function [CORE_WIDTH-1:0] shift_input_sample;
        input signed [SAMPLE_WIDTH-1:0] value_in;
        reg   signed [INPUT_TEMP_WIDTH-1:0] temp;
        reg   signed [INPUT_TEMP_WIDTH-1:0] upper_bound;
        reg   signed [INPUT_TEMP_WIDTH-1:0] lower_bound;
        begin
            temp = {{(INPUT_TEMP_WIDTH-SAMPLE_WIDTH){value_in[SAMPLE_WIDTH-1]}}, value_in};
            if (INPUT_SHIFT > 0) begin
                temp = temp >>> INPUT_SHIFT;
            end
            upper_bound = {{(INPUT_TEMP_WIDTH-CORE_WIDTH){CORE_MAX[CORE_WIDTH-1]}}, CORE_MAX};
            lower_bound = {{(INPUT_TEMP_WIDTH-CORE_WIDTH){CORE_MIN[CORE_WIDTH-1]}}, CORE_MIN};
            if (temp > upper_bound) begin
                temp = upper_bound;
            end else if (temp < lower_bound) begin
                temp = lower_bound;
            end
            shift_input_sample = temp[CORE_WIDTH-1:0];
        end
    endfunction

    // Saturating shift helper for FFT outputs prior to buffering.
    function [OUTPUT_GUARD_WIDTH-1:0] shift_output_sample;
        input signed [CORE_WIDTH-1:0] value_in;
        reg   signed [OUTPUT_TEMP_WIDTH-1:0] temp;
        reg   signed [OUTPUT_TEMP_WIDTH-1:0] upper_bound;
        reg   signed [OUTPUT_TEMP_WIDTH-1:0] lower_bound;
        begin
            temp = {{(OUTPUT_TEMP_WIDTH-CORE_WIDTH){value_in[CORE_WIDTH-1]}}, value_in};
            if (OUTPUT_SHIFT > 0) begin
                temp = temp >>> OUTPUT_SHIFT;
            end
            upper_bound = {{(OUTPUT_TEMP_WIDTH-OUTPUT_GUARD_WIDTH){OUTPUT_MAX[OUTPUT_GUARD_WIDTH-1]}}, OUTPUT_MAX};
            lower_bound = {{(OUTPUT_TEMP_WIDTH-OUTPUT_GUARD_WIDTH){OUTPUT_MIN[OUTPUT_GUARD_WIDTH-1]}}, OUTPUT_MIN};
            if (temp > upper_bound) begin
                temp = upper_bound;
            end else if (temp < lower_bound) begin
                temp = lower_bound;
            end
            shift_output_sample = temp[OUTPUT_GUARD_WIDTH-1:0];
        end
    endfunction

    function [SAMPLE_WIDTH-1:0] truncate_output_to_axis;
        input signed [OUTPUT_GUARD_WIDTH-1:0] value_in;
        reg signed [OUTPUT_GUARD_WIDTH-1:0] temp;
        reg signed [OUTPUT_GUARD_WIDTH-1:0] upper_bound;
        reg signed [OUTPUT_GUARD_WIDTH-1:0] lower_bound;
        begin
            temp = value_in;
            upper_bound = {{(OUTPUT_GUARD_WIDTH-SAMPLE_WIDTH){SAMPLE_MAX[SAMPLE_WIDTH-1]}}, SAMPLE_MAX};
            lower_bound = {{(OUTPUT_GUARD_WIDTH-SAMPLE_WIDTH){SAMPLE_MIN[SAMPLE_WIDTH-1]}}, SAMPLE_MIN};
            if (temp > upper_bound) begin
                temp = upper_bound;
            end else if (temp < lower_bound) begin
                temp = lower_bound;
            end
            truncate_output_to_axis = temp[SAMPLE_WIDTH-1:0];
        end
    endfunction

    // ---------------------------------------------------------------------
    // Ingress ping-pong memories (clk_axis domain writes)
    // ---------------------------------------------------------------------

    reg                       bank0_wr_en;
    reg                       bank1_wr_en;
    reg [PAIR_ADDR_WIDTH-1:0] bank0_wr_addr;
    reg [PAIR_ADDR_WIDTH-1:0] bank1_wr_addr;
    reg [PAIR_WORD_WIDTH-1:0] ant0_bank0_wr_data;
    reg [PAIR_WORD_WIDTH-1:0] ant0_bank1_wr_data;
    reg [PAIR_WORD_WIDTH-1:0] ant1_bank0_wr_data;
    reg [PAIR_WORD_WIDTH-1:0] ant1_bank1_wr_data;

    wire [PAIR_WORD_WIDTH-1:0] ant0_bank0_q;
    wire [PAIR_WORD_WIDTH-1:0] ant0_bank1_q;
    wire [PAIR_WORD_WIDTH-1:0] ant1_bank0_q;
    wire [PAIR_WORD_WIDTH-1:0] ant1_bank1_q;

    reg [PAIR_ADDR_WIDTH-1:0] ingress_pair_addr;
    reg                       ingress_bank_sel;
    reg                       ingress_wait_bank;
    reg                       pending_sample_valid;
    reg [CORE_WIDTH-1:0]      pending_ant0_real;
    reg [CORE_WIDTH-1:0]      pending_ant0_imag;
    reg [CORE_WIDTH-1:0]      pending_ant1_real;
    reg [CORE_WIDTH-1:0]      pending_ant1_imag;
    reg [6:0]                 ingress_symbol_reg;

    reg [1:0]                 bank_in_use;
    reg [6:0]                 bank_symbol [0:1];

    reg                       pending_token_valid;
    reg [TOKEN_WIDTH-1:0]     pending_token_data;

    reg [15:0]                latency_counter [0:1];
    reg [6:0]                 latency_symbol [0:1];
    reg                       latency_valid  [0:1];
    reg [6:0]                 latency_start_symbol [0:1];
    reg [1:0]                 latency_start_pulse;
    reg [1:0]                 latency_clear_pulse;

    reg [1:0]                 bank_release_sync0;
    reg [1:0]                 bank_release_sync1;
    reg [1:0]                 bank_release_toggle_axis;
    reg                       fft_gap_flag_axis;
    reg                       egress_underflow_axis;
    reg                       egress_overflow_axis;
    reg                       fft_gap_toggle_sync0;
    reg                       fft_gap_toggle_sync1;
    reg                       fft_gap_toggle_axis;
    reg                       egress_overflow_toggle_sync0;
    reg                       egress_overflow_toggle_sync1;
    reg                       egress_overflow_toggle_axis;
    reg [1:0]                 bank_release_toggle_fft;
    reg                       fft_gap_toggle_fft;
    reg                       egress_overflow_toggle_fft;

    // Status helpers
    reg axis_backpressure_flag;
    reg ingress_overrun_flag;
    reg token_overflow_flag;

    wire token_fifo_full;
    wire token_fifo_empty;
    reg  token_fifo_wr_en;
    reg  token_fifo_rd_en_reg;
    wire token_fifo_wr_error;
    wire token_fifo_rd_error;
    wire [TOKEN_WIDTH-1:0] token_fifo_rd_data;
    reg  token_read_pending;
    reg  token_payload_valid;
    reg  [TOKEN_WIDTH-1:0] token_payload_fft;
    reg  token_fifo_rd_en_d;

    // Input parsing
    wire signed [SAMPLE_WIDTH-1:0] ant0_i_raw = s_axis_tdata[11:0];
    wire signed [SAMPLE_WIDTH-1:0] ant0_q_raw = s_axis_tdata[23:12];
    wire signed [SAMPLE_WIDTH-1:0] ant1_i_raw = s_axis_tdata[35:24];
    wire signed [SAMPLE_WIDTH-1:0] ant1_q_raw = s_axis_tdata[47:36];

    wire [CORE_WIDTH-1:0] ant0_real_shift = shift_input_sample(ant0_i_raw);
    wire [CORE_WIDTH-1:0] ant0_imag_shift = shift_input_sample(ant0_q_raw);
    wire [CORE_WIDTH-1:0] ant1_real_shift = shift_input_sample(ant1_i_raw);
    wire [CORE_WIDTH-1:0] ant1_imag_shift = shift_input_sample(ant1_q_raw);
    wire                   unused_s_axis_tlast = s_axis_tlast;

    // ---------------------------------------------------------------------
    // Token FIFO instantiation (axis -> fft domain)
    // ---------------------------------------------------------------------

    dual_clock_fifo #(
        .DATA_WIDTH (TOKEN_WIDTH),
        .ADDR_WIDTH (3)
    ) u_token_fifo (
        .wr_clk   (clk_axis),
        .wr_rst_n (rst_axis_n),
        .wr_en    (token_fifo_wr_en),
        .wr_data  (pending_token_data),
        .wr_full  (token_fifo_full),
        .wr_error (token_fifo_wr_error),
        .rd_clk   (clk_fft),
        .rd_rst_n (rst_fft_n),
        .rd_en    (token_fifo_rd_en_reg),
        .rd_data  (token_fifo_rd_data),
        .rd_empty (token_fifo_empty),
        .rd_error (token_fifo_rd_error)
    );

    // ---------------------------------------------------------------------
    // Ping-pong bank memories
    // ---------------------------------------------------------------------

    dual_clock_ram #(
        .ADDR_WIDTH (PAIR_ADDR_WIDTH),
        .DATA_WIDTH (PAIR_WORD_WIDTH),
        .DEPTH      (PAIR_COUNT)
    ) ant0_bank0_mem (
        .wr_clk  (clk_axis),
        .wr_en   (bank0_wr_en),
        .wr_addr (bank0_wr_addr),
        .wr_data (ant0_bank0_wr_data),
        .rd_clk  (clk_fft),
        .rd_en   (1'b1),
        .rd_addr (fft_read_addr),
        .rd_data (ant0_bank0_q)
    );

    dual_clock_ram #(
        .ADDR_WIDTH (PAIR_ADDR_WIDTH),
        .DATA_WIDTH (PAIR_WORD_WIDTH),
        .DEPTH      (PAIR_COUNT)
    ) ant0_bank1_mem (
        .wr_clk  (clk_axis),
        .wr_en   (bank1_wr_en),
        .wr_addr (bank1_wr_addr),
        .wr_data (ant0_bank1_wr_data),
        .rd_clk  (clk_fft),
        .rd_en   (1'b1),
        .rd_addr (fft_read_addr),
        .rd_data (ant0_bank1_q)
    );

    dual_clock_ram #(
        .ADDR_WIDTH (PAIR_ADDR_WIDTH),
        .DATA_WIDTH (PAIR_WORD_WIDTH),
        .DEPTH      (PAIR_COUNT)
    ) ant1_bank0_mem (
        .wr_clk  (clk_axis),
        .wr_en   (bank0_wr_en),
        .wr_addr (bank0_wr_addr),
        .wr_data (ant1_bank0_wr_data),
        .rd_clk  (clk_fft),
        .rd_en   (1'b1),
        .rd_addr (fft_read_addr),
        .rd_data (ant1_bank0_q)
    );

    dual_clock_ram #(
        .ADDR_WIDTH (PAIR_ADDR_WIDTH),
        .DATA_WIDTH (PAIR_WORD_WIDTH),
        .DEPTH      (PAIR_COUNT)
    ) ant1_bank1_mem (
        .wr_clk  (clk_axis),
        .wr_en   (bank1_wr_en),
        .wr_addr (bank1_wr_addr),
        .wr_data (ant1_bank1_wr_data),
        .rd_clk  (clk_fft),
        .rd_en   (1'b1),
        .rd_addr (fft_read_addr),
        .rd_data (ant1_bank1_q)
    );

    // ---------------------------------------------------------------------
    // Ingress domain control
    // ---------------------------------------------------------------------

    integer idx;

    always @(posedge clk_axis) begin
        if (!rst_axis_n) begin
            ingress_pair_addr      <= {PAIR_ADDR_WIDTH{1'b0}};
            ingress_bank_sel       <= 1'b0;
            ingress_wait_bank      <= 1'b0;
            pending_sample_valid   <= 1'b0;
            ingress_symbol_reg     <= 7'd0;
            bank_in_use            <= 2'b00;
            pending_token_valid    <= 1'b0;
            token_fifo_wr_en       <= 1'b0;
            axis_backpressure_flag <= 1'b0;
            ingress_overrun_flag   <= 1'b0;
            token_overflow_flag    <= 1'b0;
            s_axis_tready          <= 1'b0;
            bank_release_sync0     <= 2'b00;
            bank_release_sync1     <= 2'b00;
            bank_release_toggle_axis <= 2'b00;
            fft_gap_flag_axis      <= 1'b0;
            egress_overflow_axis   <= 1'b0;
            bank0_wr_en            <= 1'b0;
            bank1_wr_en            <= 1'b0;
            bank0_wr_addr          <= {PAIR_ADDR_WIDTH{1'b0}};
            bank1_wr_addr          <= {PAIR_ADDR_WIDTH{1'b0}};
            ant0_bank0_wr_data     <= {PAIR_WORD_WIDTH{1'b0}};
            ant0_bank1_wr_data     <= {PAIR_WORD_WIDTH{1'b0}};
            ant1_bank0_wr_data     <= {PAIR_WORD_WIDTH{1'b0}};
            ant1_bank1_wr_data     <= {PAIR_WORD_WIDTH{1'b0}};
            fft_gap_toggle_sync0   <= 1'b0;
            fft_gap_toggle_sync1   <= 1'b0;
            fft_gap_toggle_axis    <= 1'b0;
            egress_overflow_toggle_sync0 <= 1'b0;
            egress_overflow_toggle_sync1 <= 1'b0;
            egress_overflow_toggle_axis  <= 1'b0;
            latency_start_pulse          <= 2'b00;
            for (idx = 0; idx < 2; idx = idx + 1) begin
                latency_start_symbol[idx] <= 7'd0;
                bank_symbol[idx]          <= 7'd0;
            end
        end else begin
            // Synchronize bank release toggles from FFT domain.
            bank0_wr_en <= 1'b0;
            bank1_wr_en <= 1'b0;
            bank_release_sync0 <= bank_release_toggle_fft;
            bank_release_sync1 <= bank_release_sync0;

            for (idx = 0; idx < 2; idx = idx + 1) begin
                if (bank_release_sync1[idx] != bank_release_toggle_axis[idx]) begin
                    bank_release_toggle_axis[idx] <= bank_release_sync1[idx];
                    bank_in_use[idx] <= 1'b0;
                end
            end

            fft_gap_toggle_sync0 <= fft_gap_toggle_fft;
            fft_gap_toggle_sync1 <= fft_gap_toggle_sync0;
            if (fft_gap_toggle_sync1 != fft_gap_toggle_axis) begin
                fft_gap_toggle_axis <= fft_gap_toggle_sync1;
                fft_gap_flag_axis   <= 1'b1;
            end

            egress_overflow_toggle_sync0 <= egress_overflow_toggle_fft;
            egress_overflow_toggle_sync1 <= egress_overflow_toggle_sync0;
            if (egress_overflow_toggle_sync1 != egress_overflow_toggle_axis) begin
                egress_overflow_toggle_axis <= egress_overflow_toggle_sync1;
                egress_overflow_axis        <= 1'b1;
            end


            latency_start_pulse <= 2'b00;

            // Token FIFO push handling.
            token_fifo_wr_en <= 1'b0;
            if (pending_token_valid && !token_fifo_full) begin
                token_fifo_wr_en    <= 1'b1;
                pending_token_valid <= 1'b0;
            end else if (pending_token_valid && token_fifo_full) begin
                token_overflow_flag <= 1'b1;
            end
            if (token_fifo_wr_error) begin
                token_overflow_flag <= 1'b1;
            end

            // Backpressure detection.
            if (s_axis_tvalid && !s_axis_tready) begin
                axis_backpressure_flag <= 1'b1;
            end

            // Default ready high unless waiting for bank.
            s_axis_tready <= !ingress_wait_bank;

            if (s_axis_tvalid && s_axis_tready) begin
                ingress_symbol_reg <= s_axis_tuser;
                if (!pending_sample_valid) begin
                    pending_sample_valid <= 1'b1;
                    pending_ant0_real   <= ant0_real_shift;
                    pending_ant0_imag   <= ant0_imag_shift;
                    pending_ant1_real   <= ant1_real_shift;
                    pending_ant1_imag   <= ant1_imag_shift;
                end else begin
                    // Form pair and commit to selected bank.
                    pending_sample_valid <= 1'b0;
                    case (ingress_bank_sel)
                        1'b0: begin
                            bank0_wr_en        <= 1'b1;
                            bank0_wr_addr      <= ingress_pair_addr;
                            ant0_bank0_wr_data <= {
                                ant0_imag_shift,
                                ant0_real_shift,
                                pending_ant0_imag,
                                pending_ant0_real
                            };
                            ant1_bank0_wr_data <= {
                                ant1_imag_shift,
                                ant1_real_shift,
                                pending_ant1_imag,
                                pending_ant1_real
                            };
                        end
                        1'b1: begin
                            bank1_wr_en        <= 1'b1;
                            bank1_wr_addr      <= ingress_pair_addr;
                            ant0_bank1_wr_data <= {
                                ant0_imag_shift,
                                ant0_real_shift,
                                pending_ant0_imag,
                                pending_ant0_real
                            };
                            ant1_bank1_wr_data <= {
                                ant1_imag_shift,
                                ant1_real_shift,
                                pending_ant1_imag,
                                pending_ant1_real
                            };
                        end
                    endcase

                    if (ingress_pair_addr == PAIR_LAST) begin
                        bank_in_use[ingress_bank_sel] <= 1'b1;
                        bank_symbol[ingress_bank_sel] <= ingress_symbol_reg;
                        pending_token_data  <= {ingress_bank_sel, ingress_symbol_reg};
                        pending_token_valid <= 1'b1;
                        latency_start_symbol[ingress_bank_sel] <= ingress_symbol_reg;
                        latency_start_pulse[ingress_bank_sel]  <= 1'b1;

                        ingress_pair_addr <= {PAIR_ADDR_WIDTH{1'b0}};
                        if (bank_in_use[~ingress_bank_sel]) begin
                            ingress_wait_bank <= 1'b1;
                        end else begin
                            ingress_bank_sel <= ~ingress_bank_sel;
                        end
                    end else begin
                        ingress_pair_addr <= ingress_pair_addr + 1'b1;
                    end
                end
            end

            if (ingress_wait_bank && !bank_in_use[~ingress_bank_sel]) begin
                ingress_wait_bank <= 1'b0;
                ingress_bank_sel  <= ~ingress_bank_sel;
            end

            if (!s_axis_tvalid) begin
                pending_sample_valid <= pending_sample_valid;
            end

            if (pending_sample_valid && s_axis_tvalid && !s_axis_tready) begin
                ingress_overrun_flag <= 1'b1;
            end
        end
    end

    // ---------------------------------------------------------------------
    // FFT domain control
    // ---------------------------------------------------------------------

    reg [6:0] active_symbol_fft;
    reg       active_bank_fft;
    reg [PAIR_ADDR_WIDTH-1:0] fft_read_addr;
    reg [PAIR_ADDR_WIDTH:0]   fft_pairs_requested;
    reg [PAIR_ADDR_WIDTH:0]   fft_pairs_loaded;
    reg [PAIR_ADDR_WIDTH:0]   fft_pairs_captured;
    reg                       fft_load_active;
    reg                       fft_capture_active;
    reg                       fft_wait_output;
    reg                       fft_capture_start_pending;
    reg                       fft_read_issue;
    reg                       fft_read_issue_d;
    reg                       fft_read_bank_d;
    reg [GAP_COUNTER_WIDTH-1:0] cooldown_counter;

    reg                       fft_next_pulse;
    wire                      fft_next_out0;
    wire                      fft_next_out1;

    reg [PAIR_WORD_WIDTH-1:0] ant0_fft_pair;
    reg [PAIR_WORD_WIDTH-1:0] ant1_fft_pair;

    wire [CORE_WIDTH-1:0] fft0_Y0;
    wire [CORE_WIDTH-1:0] fft0_Y1;
    wire [CORE_WIDTH-1:0] fft0_Y2;
    wire [CORE_WIDTH-1:0] fft0_Y3;
    wire [CORE_WIDTH-1:0] fft1_Y0;
    wire [CORE_WIDTH-1:0] fft1_Y1;
    wire [CORE_WIDTH-1:0] fft1_Y2;
    wire [CORE_WIDTH-1:0] fft1_Y3;

    reg                     metadata_fifo_wr_en;
    reg  [6:0]              metadata_fifo_wr_data;
    wire                    metadata_fifo_full;
    wire                    metadata_fifo_empty;
    reg                     metadata_fifo_rd_en;
    wire [6:0]              metadata_fifo_rd_data;
    wire                    metadata_fifo_rd_error;
    wire                    metadata_fifo_wr_error;

    reg                     data_fifo_wr_en;
    reg [OUTPUT_PAIR_WIDTH-1:0] data_fifo_wr_data;
    wire                    data_fifo_full;
    wire                    data_fifo_empty;
    reg                     data_fifo_rd_en;
    wire [OUTPUT_PAIR_WIDTH-1:0] data_fifo_rd_data;
    wire                    data_fifo_rd_error;
    wire                    data_fifo_wr_error;

    always @(posedge clk_fft) begin
        if (!rst_fft_n) begin
            bank_release_toggle_fft <= 2'b00;
            active_symbol_fft       <= 7'd0;
            active_bank_fft         <= 1'b0;
            fft_read_addr           <= {PAIR_ADDR_WIDTH{1'b0}};
            fft_pairs_requested     <= {(PAIR_ADDR_WIDTH+1){1'b0}};
            fft_pairs_loaded        <= {(PAIR_ADDR_WIDTH+1){1'b0}};
            fft_pairs_captured      <= {(PAIR_ADDR_WIDTH+1){1'b0}};
            fft_load_active         <= 1'b0;
            fft_capture_active      <= 1'b0;
            fft_wait_output         <= 1'b0;
            fft_capture_start_pending <= 1'b0;
            cooldown_counter        <= {GAP_COUNTER_WIDTH{1'b0}};
            fft_next_pulse          <= 1'b0;
            fft_read_issue          <= 1'b0;
            fft_read_issue_d        <= 1'b0;
            fft_read_bank_d         <= 1'b0;
            ant0_fft_pair           <= {PAIR_WORD_WIDTH{1'b0}};
            ant1_fft_pair           <= {PAIR_WORD_WIDTH{1'b0}};
            metadata_fifo_wr_en     <= 1'b0;
            metadata_fifo_wr_data   <= 7'd0;
            data_fifo_wr_en         <= 1'b0;
            token_fifo_rd_en_reg    <= 1'b0;
            fft_gap_toggle_fft      <= 1'b0;
            egress_overflow_toggle_fft <= 1'b0;
            token_read_pending      <= 1'b0;
            token_payload_valid     <= 1'b0;
            token_payload_fft       <= {TOKEN_WIDTH{1'b0}};
            token_fifo_rd_en_d      <= 1'b0;
        end else begin
            if (cooldown_counter != {GAP_COUNTER_WIDTH{1'b0}}) begin
                cooldown_counter <= cooldown_counter - {{GAP_COUNTER_WIDTH-1{1'b0}}, 1'b1};
            end

            fft_next_pulse      <= 1'b0;
            metadata_fifo_wr_en <= 1'b0;
            data_fifo_wr_en     <= 1'b0;
            token_fifo_rd_en_reg <= 1'b0;
            token_fifo_rd_en_d  <= token_fifo_rd_en_reg;
            fft_read_issue_d    <= fft_read_issue;
            fft_read_issue      <= 1'b0;

            if (token_read_pending && token_fifo_rd_en_d) begin
                token_payload_fft   <= token_fifo_rd_data;
                token_payload_valid <= 1'b1;
                token_read_pending  <= 1'b0;
            end

            if (token_payload_valid && !fft_load_active && !fft_capture_active && !fft_wait_output) begin
                active_bank_fft      <= token_payload_fft[7];
                active_symbol_fft    <= token_payload_fft[6:0];
                fft_read_addr        <= {PAIR_ADDR_WIDTH{1'b0}};
                fft_pairs_requested  <= {(PAIR_ADDR_WIDTH+1){1'b0}};
                fft_pairs_loaded     <= {(PAIR_ADDR_WIDTH+1){1'b0}};
                fft_load_active      <= 1'b1;
                cooldown_counter     <= GAP_CYCLES[GAP_COUNTER_WIDTH-1:0];
                metadata_fifo_wr_en  <= 1'b1;
                metadata_fifo_wr_data <= token_payload_fft[6:0];
                token_payload_valid  <= 1'b0;
                fft_read_issue       <= 1'b0;
                fft_read_issue_d     <= 1'b0;
            end else if (!token_payload_valid && !token_read_pending && !fft_load_active && !fft_capture_active && !fft_wait_output) begin
                if (!token_fifo_empty) begin
                    if (cooldown_counter == {GAP_COUNTER_WIDTH{1'b0}}) begin
                        if (!metadata_fifo_full) begin
                            token_fifo_rd_en_reg <= 1'b1;
                            token_read_pending   <= 1'b1;
                        end else begin
                            egress_overflow_toggle_fft <= ~egress_overflow_toggle_fft;
                        end
                    end else begin
                        fft_gap_toggle_fft <= ~fft_gap_toggle_fft;
                    end
                end
            end

            if (fft_load_active) begin
                if (fft_pairs_requested <= PAIR_LAST_EXT) begin
                    fft_read_issue      <= 1'b1;
                    fft_read_bank_d     <= active_bank_fft;
                    fft_pairs_requested <= fft_pairs_requested + 1'b1;
                    fft_read_addr       <= fft_read_addr + 1'b1;
                end

                if (fft_read_issue_d) begin
                    if (fft_pairs_loaded == {(PAIR_ADDR_WIDTH+1){1'b0}}) begin
                        fft_next_pulse <= 1'b1;
                    end

                    case (fft_read_bank_d)
                        1'b0: begin
                            ant0_fft_pair <= ant0_bank0_q;
                            ant1_fft_pair <= ant1_bank0_q;
                        end
                        1'b1: begin
                            ant0_fft_pair <= ant0_bank1_q;
                            ant1_fft_pair <= ant1_bank1_q;
                        end
                    endcase

                    if (fft_pairs_loaded == PAIR_LAST_EXT) begin
                        fft_load_active <= 1'b0;
                        fft_wait_output <= 1'b1;
                    end
                    fft_pairs_loaded <= fft_pairs_loaded + 1'b1;
                end
            end

            if (fft_wait_output) begin
                if (fft_next_out0 && fft_next_out1) begin
                    fft_wait_output            <= 1'b0;
                    fft_capture_start_pending <= 1'b1;
                end
            end

            if (fft_capture_start_pending) begin
                fft_capture_start_pending <= 1'b0;
                fft_capture_active        <= 1'b1;
                fft_pairs_captured        <= {(PAIR_ADDR_WIDTH+1){1'b0}};
            end

            if (fft_capture_active) begin
                // Capture two bins per cycle into FIFO.
                data_fifo_wr_en <= 1'b1;
                data_fifo_wr_data <= {
                    shift_output_sample(fft1_Y3),
                    shift_output_sample(fft1_Y2),
                    shift_output_sample(fft1_Y1),
                    shift_output_sample(fft1_Y0),
                    shift_output_sample(fft0_Y3),
                    shift_output_sample(fft0_Y2),
                    shift_output_sample(fft0_Y1),
                    shift_output_sample(fft0_Y0)
                };

                fft_pairs_captured <= fft_pairs_captured + 1'b1;
                if (fft_pairs_captured == PAIR_LAST_EXT) begin
                    fft_capture_active <= 1'b0;
                    bank_release_toggle_fft[active_bank_fft] <= ~bank_release_toggle_fft[active_bank_fft];
                end
            end

            if (data_fifo_full && data_fifo_wr_en) begin
                egress_overflow_toggle_fft <= ~egress_overflow_toggle_fft;
            end
        end
    end

    // FFT core input assignments
    wire [CORE_WIDTH-1:0] fft0_X0 = ant0_fft_pair[CORE_WIDTH-1:0];
    wire [CORE_WIDTH-1:0] fft0_X1 = ant0_fft_pair[2*CORE_WIDTH-1:CORE_WIDTH];
    wire [CORE_WIDTH-1:0] fft0_X2 = ant0_fft_pair[3*CORE_WIDTH-1:2*CORE_WIDTH];
    wire [CORE_WIDTH-1:0] fft0_X3 = ant0_fft_pair[4*CORE_WIDTH-1:3*CORE_WIDTH];

    wire [CORE_WIDTH-1:0] fft1_X0 = ant1_fft_pair[CORE_WIDTH-1:0];
    wire [CORE_WIDTH-1:0] fft1_X1 = ant1_fft_pair[2*CORE_WIDTH-1:CORE_WIDTH];
    wire [CORE_WIDTH-1:0] fft1_X2 = ant1_fft_pair[3*CORE_WIDTH-1:2*CORE_WIDTH];
    wire [CORE_WIDTH-1:0] fft1_X3 = ant1_fft_pair[4*CORE_WIDTH-1:3*CORE_WIDTH];

    // Instantiate SPIRAL FFT cores
    dft_top fft_ant0 (
        .clk      (clk_fft),
        .reset    (!rst_fft_n),
        .next     (fft_next_pulse),
        .next_out (fft_next_out0),
        .X0       (fft0_X0),
        .Y0       (fft0_Y0),
        .X1       (fft0_X1),
        .Y1       (fft0_Y1),
        .X2       (fft0_X2),
        .Y2       (fft0_Y2),
        .X3       (fft0_X3),
        .Y3       (fft0_Y3)
    );

    dft_top fft_ant1 (
        .clk      (clk_fft),
        .reset    (!rst_fft_n),
        .next     (fft_next_pulse),
        .next_out (fft_next_out1),
        .X0       (fft1_X0),
        .Y0       (fft1_Y0),
        .X1       (fft1_X1),
        .Y1       (fft1_Y1),
        .X2       (fft1_X2),
        .Y2       (fft1_Y2),
        .X3       (fft1_X3),
        .Y3       (fft1_Y3)
    );

    // Metadata FIFO (clk_fft -> clk_axis)
    dual_clock_fifo #(
        .DATA_WIDTH (7),
        .ADDR_WIDTH (4)
    ) u_metadata_fifo (
        .wr_clk   (clk_fft),
        .wr_rst_n (rst_fft_n),
        .wr_en    (metadata_fifo_wr_en),
        .wr_data  (metadata_fifo_wr_data),
        .wr_full  (metadata_fifo_full),
        .wr_error (metadata_fifo_wr_error),
        .rd_clk   (clk_axis),
        .rd_rst_n (rst_axis_n),
        .rd_en    (metadata_fifo_rd_en),
        .rd_data  (metadata_fifo_rd_data),
        .rd_empty (metadata_fifo_empty),
        .rd_error (metadata_fifo_rd_error)
    );

    // Data FIFO (clk_fft -> clk_axis)
    dual_clock_fifo #(
        .DATA_WIDTH (OUTPUT_PAIR_WIDTH),
        .ADDR_WIDTH (FIFO_ADDR_WIDTH)
    ) u_data_fifo (
        .wr_clk   (clk_fft),
        .wr_rst_n (rst_fft_n),
        .wr_en    (data_fifo_wr_en && !data_fifo_full),
        .wr_data  (data_fifo_wr_data),
        .wr_full  (data_fifo_full),
        .wr_error (data_fifo_wr_error),
        .rd_clk   (clk_axis),
        .rd_rst_n (rst_axis_n),
        .rd_en    (data_fifo_rd_en),
        .rd_data  (data_fifo_rd_data),
        .rd_empty (data_fifo_empty),
        .rd_error (data_fifo_rd_error)
    );

    // ---------------------------------------------------------------------
    // AXIS egress domain
    // ---------------------------------------------------------------------

    reg [OUTPUT_PAIR_WIDTH-1:0] pair_buffer;
    reg                         pair_valid;
    reg                         pair_second;
    reg                         data_rd_pending;
    reg [BIN_COUNTER_WIDTH-1:0] bins_remaining;
    reg [6:0]                   current_symbol_axis;
    reg                         symbol_pending_load;
    reg                         first_bin_flag;
    reg                         metadata_fifo_rd_en_d;
    reg                         symbol_data_started;

    wire [OUTPUT_GUARD_WIDTH-1:0] ant0_real_even  = pair_buffer[OUTPUT_GUARD_WIDTH-1:0];
    wire [OUTPUT_GUARD_WIDTH-1:0] ant0_imag_even  = pair_buffer[2*OUTPUT_GUARD_WIDTH-1:OUTPUT_GUARD_WIDTH];
    wire [OUTPUT_GUARD_WIDTH-1:0] ant0_real_odd   = pair_buffer[3*OUTPUT_GUARD_WIDTH-1:2*OUTPUT_GUARD_WIDTH];
    wire [OUTPUT_GUARD_WIDTH-1:0] ant0_imag_odd   = pair_buffer[4*OUTPUT_GUARD_WIDTH-1:3*OUTPUT_GUARD_WIDTH];
    wire [OUTPUT_GUARD_WIDTH-1:0] ant1_real_even  = pair_buffer[5*OUTPUT_GUARD_WIDTH-1:4*OUTPUT_GUARD_WIDTH];
    wire [OUTPUT_GUARD_WIDTH-1:0] ant1_imag_even  = pair_buffer[6*OUTPUT_GUARD_WIDTH-1:5*OUTPUT_GUARD_WIDTH];
    wire [OUTPUT_GUARD_WIDTH-1:0] ant1_real_odd   = pair_buffer[7*OUTPUT_GUARD_WIDTH-1:6*OUTPUT_GUARD_WIDTH];
    wire [OUTPUT_GUARD_WIDTH-1:0] ant1_imag_odd   = pair_buffer[8*OUTPUT_GUARD_WIDTH-1:7*OUTPUT_GUARD_WIDTH];

    always @(posedge clk_axis) begin
        if (!rst_axis_n) begin
            pair_buffer         <= {OUTPUT_PAIR_WIDTH{1'b0}};
            pair_valid          <= 1'b0;
            pair_second         <= 1'b0;
            data_rd_pending     <= 1'b0;
            data_fifo_rd_en     <= 1'b0;
            metadata_fifo_rd_en <= 1'b0;
            metadata_fifo_rd_en_d <= 1'b0;
            bins_remaining      <= 12'd0;
            current_symbol_axis <= 7'd0;
            symbol_pending_load <= 1'b0;
            first_bin_flag      <= 1'b0;
            symbol_data_started <= 1'b0;
            m_axis_tvalid       <= 1'b0;
            m_axis_tdata        <= 48'd0;
            m_axis_tuser        <= 7'd0;
            m_axis_tlast        <= 1'b0;
            latency_debug       <= 16'd0;
            latency_clear_pulse <= 2'b00;
            egress_underflow_axis <= 1'b0;
        end else begin
            m_axis_tvalid       <= 1'b0;
            m_axis_tlast        <= 1'b0;
            data_fifo_rd_en     <= 1'b0;
            metadata_fifo_rd_en <= 1'b0;
            metadata_fifo_rd_en_d <= metadata_fifo_rd_en;
            latency_clear_pulse <= 2'b00;

            if (bins_remaining == 0 && !metadata_fifo_empty && !symbol_pending_load) begin
                metadata_fifo_rd_en <= 1'b1;
                symbol_pending_load <= 1'b1;
            end

            if (symbol_pending_load && metadata_fifo_rd_en_d) begin
                current_symbol_axis <= metadata_fifo_rd_data;
                bins_remaining      <= NFFT[BIN_COUNTER_WIDTH-1:0];
                pair_valid          <= 1'b0;
                pair_second         <= 1'b0;
                symbol_pending_load <= 0;
                first_bin_flag      <= 1'b1;
                symbol_data_started <= 1'b0;
            end

            if (!pair_valid && !data_fifo_empty && !data_rd_pending) begin
                data_fifo_rd_en <= 1'b1;
                data_rd_pending <= 1'b1;
            end else if (data_rd_pending) begin
                pair_buffer     <= data_fifo_rd_data;
                pair_valid      <= 1'b1;
                pair_second     <= 1'b0;
                data_rd_pending <= 1'b0;
                symbol_data_started <= 1'b1;
            end

            if (pair_valid && bins_remaining != {BIN_COUNTER_WIDTH{1'b0}}) begin
                if (m_axis_tready) begin
                    m_axis_tvalid <= 1'b1;
                    bins_remaining <= bins_remaining - {{BIN_COUNTER_WIDTH-1{1'b0}}, 1'b1};
                    m_axis_tuser <= current_symbol_axis;

                    // Extract samples based on pair_second flag.
                    if (!pair_second) begin
                        m_axis_tdata <= {
                            truncate_output_to_axis(ant1_imag_even),
                            truncate_output_to_axis(ant1_real_even),
                            truncate_output_to_axis(ant0_imag_even),
                            truncate_output_to_axis(ant0_real_even)
                        };
                        pair_second <= 1'b1;
                    end else begin
                        m_axis_tdata <= {
                            truncate_output_to_axis(ant1_imag_odd),
                            truncate_output_to_axis(ant1_real_odd),
                            truncate_output_to_axis(ant0_imag_odd),
                            truncate_output_to_axis(ant0_real_odd)
                        };
                        pair_second <= 1'b0;
                        pair_valid  <= 1'b0;
                    end

                    if (bins_remaining == {{BIN_COUNTER_WIDTH-1{1'b0}}, 1'b1}) begin
                        m_axis_tlast <= 1'b1;
                    end

                    if (first_bin_flag) begin
                        first_bin_flag <= 1'b0;
                        for (idx = 0; idx < 2; idx = idx + 1) begin
                            if (latency_valid[idx] && latency_symbol[idx] == current_symbol_axis) begin
                                latency_debug     <= latency_counter[idx];
                                latency_clear_pulse[idx] <= 1'b1;
                            end
                        end
                    end
                end
            end else if (symbol_data_started && (bins_remaining != {BIN_COUNTER_WIDTH{1'b0}}) && data_fifo_empty && !pair_valid && !data_rd_pending) begin
                egress_underflow_axis <= 1'b1;
            end
        end
    end

    // ---------------------------------------------------------------------
    // Latency tracking (clk_axis domain)
    // ---------------------------------------------------------------------

    always @(posedge clk_axis) begin
        if (!rst_axis_n) begin
            for (idx = 0; idx < 2; idx = idx + 1) begin
                latency_counter[idx] <= 16'd0;
                latency_symbol[idx]  <= 7'd0;
                latency_valid[idx]   <= 1'b0;
            end
        end else begin
            for (idx = 0; idx < 2; idx = idx + 1) begin
                if (latency_start_pulse[idx]) begin
                    latency_counter[idx] <= 16'd0;
                    latency_symbol[idx]  <= latency_start_symbol[idx];
                    latency_valid[idx]   <= 1'b1;
                end else begin
                    if (latency_valid[idx]) begin
                        latency_counter[idx] <= latency_counter[idx] + 16'd1;
                    end
                    if (latency_clear_pulse[idx]) begin
                        latency_valid[idx] <= 1'b0;
                    end
                end
            end
        end
    end

    // ---------------------------------------------------------------------
    // Status aggregation
    // ---------------------------------------------------------------------

    always @(posedge clk_axis) begin
        if (!rst_axis_n) begin
            status_flags <= 6'b000000;
        end else begin
            status_flags[0] <= (bank_in_use != 2'b00) || pending_token_valid || pair_valid || (bins_remaining != {BIN_COUNTER_WIDTH{1'b0}});
            status_flags[1] <= axis_backpressure_flag;
            status_flags[2] <= ingress_overrun_flag;
            status_flags[3] <= fft_gap_flag_axis;
            status_flags[4] <= egress_underflow_axis | egress_overflow_axis;
            status_flags[5] <= token_overflow_flag;
        end
    end

endmodule

`default_nettype wire
