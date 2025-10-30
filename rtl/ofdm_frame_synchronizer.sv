// -----------------------------------------------------------------------------
// Module: ofdm_frame_synchronizer
// -----------------------------------------------------------------------------
// AXI4-Stream scheduler that consumes the Minn detector output, strips cyclic
// prefixes, and emits payload samples annotated with a 7-bit symbol index. The
// synchronizer locks to the first start-of-frame (SOF) pulse observed on the
// input stream and subsequently free-runs using the programmed frame geometry.
// Between frames the block enters a tracking window where signed sample-time
// offset (STO) corrections may extend or shorten the inter-frame gap.
// -----------------------------------------------------------------------------
module ofdm_frame_synchronizer #(
    parameter int INPUT_WIDTH         = 12,
    parameter int AXIS_TUSER_WIDTH_IN = 1,
    parameter int NFFT                = 2048,
    parameter int CP_LEN              = 512,
    parameter int N_FRAME_SYMBOLS     = 28,
    parameter int FRAME_GAP_SAMPLES   = N_FRAME_SYMBOLS * (NFFT + CP_LEN),
    parameter int SYMBOL_COUNTER_WIDTH = 7,
    parameter int STO_ACC_WIDTH       = 12
) (
    input  logic                           clk,
    input  logic                           rst,

    // AXI4-Stream input from Minn detector
    input  logic [4*INPUT_WIDTH-1:0]        s_axis_tdata,
    input  logic [AXIS_TUSER_WIDTH_IN-1:0]  s_axis_tuser,
    input  logic                           s_axis_tvalid,
    output logic                           s_axis_tready,
    input  logic                           s_axis_tlast,

    // AXI4-Stream output towards downstream FFT
    output logic [4*INPUT_WIDTH-1:0]        m_axis_tdata,
    output logic [SYMBOL_COUNTER_WIDTH-1:0] m_axis_tuser,
    output logic                           m_axis_tvalid,
    input  logic                           m_axis_tready,
    output logic                           m_axis_tlast,

    // Sample-time offset control
    input  logic signed [7:0]               sto_correction,
    input  logic                           sto_valid,
    output logic                           sto_ready
);

    // -------------------------------------------------------------------------
    // Parameter validation
    // -------------------------------------------------------------------------
    if (INPUT_WIDTH < 4) begin : g_invalid_input_width
        initial $error("ofdm_frame_synchronizer requires INPUT_WIDTH >= 4");
    end

    if (AXIS_TUSER_WIDTH_IN < 1) begin : g_invalid_tuser_width
        initial $error("AXIS_TUSER_WIDTH_IN must be at least 1");
    end

    if (NFFT <= 0) begin : g_invalid_nfft
        initial $error("NFFT must be positive");
    end

    if (CP_LEN < 0) begin : g_invalid_cp
        initial $error("CP_LEN must be non-negative");
    end

    if ((N_FRAME_SYMBOLS <= 0) || (N_FRAME_SYMBOLS > (1 << SYMBOL_COUNTER_WIDTH))) begin
        initial $error("Invalid N_FRAME_SYMBOLS/SYMBOL_COUNTER_WIDTH combination");
    end

    if (FRAME_GAP_SAMPLES < 0) begin : g_invalid_gap
        initial $error("FRAME_GAP_SAMPLES must be non-negative");
    end

    if (STO_ACC_WIDTH < 4) begin : g_invalid_sto_width
        initial $error("STO_ACC_WIDTH must be at least 4 bits");
    end

    // -------------------------------------------------------------------------
    // Derived constants and types
    // -------------------------------------------------------------------------
    localparam int SYMBOL_LEN             = NFFT + CP_LEN;
    localparam int SYMBOL_COUNTER_MAX_VAL = N_FRAME_SYMBOLS - 1;

    localparam int SAMPLE_COUNTER_WIDTH = (SYMBOL_LEN <= 1) ? 1 : $clog2(SYMBOL_LEN);

    // Remaining time between frames while tracking. Sized large enough to cover
    // the nominal gap plus signed STO adjustments.
    localparam int STO_COUNTER_WIDTH = STO_ACC_WIDTH;
    localparam int STO_MAX_INT = (1 << (STO_COUNTER_WIDTH - 1)) - 1;

    typedef logic signed [STO_COUNTER_WIDTH-1:0] sto_count_t;

    localparam sto_count_t STO_MAX = sto_count_t'(STO_MAX_INT);
    localparam sto_count_t GAP_INIT = (FRAME_GAP_SAMPLES > STO_MAX_INT)
        ? STO_MAX
        : sto_count_t'(FRAME_GAP_SAMPLES);

    // FIFO depth for payload buffering. A small skid buffer absorbs short bursts
    // of downstream back-pressure without stalling the Minn detector.
    localparam int PAYLOAD_FIFO_DEPTH = 8;
    localparam int PAYLOAD_FIFO_AW    = (PAYLOAD_FIFO_DEPTH <= 1) ? 1 : $clog2(PAYLOAD_FIFO_DEPTH);

    typedef enum logic [1:0] {
        SEARCHING,
        COLLECTING,
        TRACKING
    } state_t;

    typedef logic [PAYLOAD_FIFO_AW-1:0] fifo_ptr_t;
    typedef logic [PAYLOAD_FIFO_AW:0]   fifo_count_t;

    typedef logic [SAMPLE_COUNTER_WIDTH-1:0] sample_count_t;
    typedef logic [SYMBOL_COUNTER_WIDTH-1:0] symbol_count_t;

    // -------------------------------------------------------------------------
    // State and counters
    // -------------------------------------------------------------------------
    state_t         state;
    sample_count_t  sample_in_symbol;
    symbol_count_t  symbol_index;
    sto_count_t     gap_counter;

    // FIFO bookkeeping
    fifo_ptr_t      fifo_wr_ptr;
    fifo_ptr_t      fifo_rd_ptr;
    fifo_count_t    fifo_count;

    logic [4*INPUT_WIDTH-1:0] fifo_data   [0:PAYLOAD_FIFO_DEPTH-1];
    logic [SYMBOL_COUNTER_WIDTH-1:0] fifo_user [0:PAYLOAD_FIFO_DEPTH-1];
    logic                          fifo_last  [0:PAYLOAD_FIFO_DEPTH-1];

    logic                           push_payload;
    logic [4*INPUT_WIDTH-1:0]        payload_data;
    logic [SYMBOL_COUNTER_WIDTH-1:0] payload_user;
    logic                           payload_last;
    logic                           fifo_pop;
    logic                           fifo_push;
    logic                           fifo_full;

    // -------------------------------------------------------------------------
    // Helper functions
    // -------------------------------------------------------------------------
    function automatic sto_count_t clamp_positive(input logic signed [STO_COUNTER_WIDTH:0] value);
        sto_count_t result;
        logic signed [STO_COUNTER_WIDTH:0] sto_max_ext;

        sto_max_ext = {{1{STO_MAX[STO_COUNTER_WIDTH-1]}}, STO_MAX};

        if (value < 0) begin
            result = '0;
        end else if (value > sto_max_ext) begin
            result = STO_MAX;
        end else begin
            result = sto_count_t'(value[STO_COUNTER_WIDTH-1:0]);
        end
        return result;
    endfunction

    // -------------------------------------------------------------------------
    // Input handshake and SOF detection
    // -------------------------------------------------------------------------
    logic sof_pulse;

    assign sof_pulse = s_axis_tvalid && (s_axis_tuser[0] == 1'b1);

    // -------------------------------------------------------------------------
    // Output AXIS signals driven from payload FIFO
    // -------------------------------------------------------------------------
    assign m_axis_tvalid = (fifo_count != '0);
    assign m_axis_tdata  = fifo_data[fifo_rd_ptr];
    assign m_axis_tuser  = fifo_user[fifo_rd_ptr];
    assign m_axis_tlast  = fifo_last[fifo_rd_ptr];

    // -------------------------------------------------------------------------
    // Control logic
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            state            <= SEARCHING;
            sample_in_symbol <= '0;
            symbol_index     <= '0;
            gap_counter      <= GAP_INIT;

            fifo_wr_ptr      <= '0;
            fifo_rd_ptr      <= '0;
            fifo_count       <= '0;

            s_axis_tready    <= 1'b0;
        end else begin
            s_axis_tready <= 1'b1;

            // -----------------------------------------------------------------
            // Default: no writes each cycle unless explicitly requested
            // -----------------------------------------------------------------
            push_payload  = 1'b0;
            payload_data  = '0;
            payload_user  = '0;
            payload_last  = 1'b0;

            case (state)
                SEARCHING: begin
                    sample_in_symbol <= '0;
                    symbol_index     <= '0;
                    gap_counter      <= sto_count_t'(FRAME_GAP_SAMPLES);

                    if (sof_pulse) begin
                        state            <= COLLECTING;
                        sample_in_symbol <= sample_count_t'(1);
                        symbol_index     <= '0;

                        if (s_axis_tvalid) begin
                            push_payload = 1'b1;
                            payload_data = s_axis_tdata;
                            payload_user = '0;
                            payload_last = (N_FRAME_SYMBOLS == 1) && (NFFT == 1);
                        end
                    end
                end

                COLLECTING: begin
                    if (s_axis_tvalid) begin
                        if (sample_in_symbol < sample_count_t'(NFFT)) begin
                            push_payload = 1'b1;
                            payload_data = s_axis_tdata;
                            payload_user = symbol_index;
                            payload_last =
                                (symbol_index == symbol_count_t'(SYMBOL_COUNTER_MAX_VAL)) &&
                                (sample_in_symbol == sample_count_t'(NFFT - 1));
                        end

                        if (sample_in_symbol == sample_count_t'(SYMBOL_LEN - 1)) begin
                            sample_in_symbol <= '0;

                            if (symbol_index == symbol_count_t'(SYMBOL_COUNTER_MAX_VAL)) begin
                                symbol_index <= '0;
                                state        <= TRACKING;
                                gap_counter  <= GAP_INIT;
                            end else begin
                                symbol_index <= symbol_index + symbol_count_t'(1);
                            end
                        end else begin
                            sample_in_symbol <= sample_in_symbol + sample_count_t'(1);
                        end
                    end
                end

                TRACKING: begin
                    sto_count_t gap_next;

                    gap_next = gap_counter;

                    if (sto_valid) begin
                        logic signed [STO_COUNTER_WIDTH:0] gap_plus_sto;
                        gap_plus_sto = gap_next + {{(STO_COUNTER_WIDTH-8){sto_correction[7]}}, sto_correction};
                        gap_next     = clamp_positive(gap_plus_sto);
                    end

                    if (gap_next == '0) begin
                        state            <= COLLECTING;
                        sample_in_symbol <= '0;
                        symbol_index     <= '0;

                        if (s_axis_tvalid) begin
                            push_payload = 1'b1;
                            payload_data = s_axis_tdata;
                            payload_user = '0;
                            payload_last = (N_FRAME_SYMBOLS == 1) && (NFFT == 1);

                            sample_in_symbol <= sample_count_t'(1);
                        end
                    end else begin
                        if (s_axis_tvalid) begin
                            gap_next = gap_next - sto_count_t'(1);
                        end
                    end

                    gap_counter <= gap_next;
                end
                default: begin
                    state            <= SEARCHING;
                    sample_in_symbol <= '0;
                    symbol_index     <= '0;
                    gap_counter      <= GAP_INIT;
                end
            endcase

            // -----------------------------------------------------------------
            // FIFO write path
            // -----------------------------------------------------------------
            fifo_pop  = m_axis_tvalid && m_axis_tready;
            fifo_push = push_payload;
            fifo_full = (fifo_count == fifo_count_t'(PAYLOAD_FIFO_DEPTH));

            if (fifo_pop) begin
                fifo_rd_ptr <= fifo_rd_ptr + fifo_ptr_t'(1);
            end

            if (fifo_push) begin
                if (fifo_full && !fifo_pop) begin
                    $error("ofdm_frame_synchronizer payload FIFO overflow");
                end else begin
                    fifo_data[fifo_wr_ptr] <= payload_data;
                    fifo_user[fifo_wr_ptr] <= payload_user;
                    fifo_last[fifo_wr_ptr] <= payload_last;
                    fifo_wr_ptr            <= fifo_wr_ptr + fifo_ptr_t'(1);
                end
            end

            fifo_count <= fifo_count
                + (fifo_push ? fifo_count_t'(1) : fifo_count_t'(0))
                - (fifo_pop ? fifo_count_t'(1) : fifo_count_t'(0));
        end
    end

    // STO ready is asserted only while tracking between frames.
    assign sto_ready = (state == TRACKING);

endmodule

`default_nettype wire
