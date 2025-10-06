// Dual-channel NCO-based CFO compensator with AXI4-Stream I/Q pairs and AXI4-Lite control.
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
    logic signed [LUT_DATA_WIDTH-1:0] cos_lut [0:LUT_SIZE-1];
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

    logic [ACC_WIDTH-1:0] phase_inc;
    logic [ACC_WIDTH-1:0] phase_acc;
    wire  [LUT_ADDR_WIDTH-1:0] phase_index = phase_acc[ACC_WIDTH-1 -: LUT_ADDR_WIDTH];

    // Helper for saturation after fixed-point shift
    function automatic logic signed [WIDTH-1:0] sat_shift (
        input logic signed [WIDTH+LUT_DATA_WIDTH:0] value
    );
        logic signed [WIDTH+LUT_DATA_WIDTH:0] shifted;
        logic signed [WIDTH-1:0] max_base;
        logic signed [WIDTH-1:0] min_base;
        logic signed [WIDTH+LUT_DATA_WIDTH:0] max_val;
        logic signed [WIDTH+LUT_DATA_WIDTH:0] min_val;
        logic signed [WIDTH+LUT_DATA_WIDTH:0] clipped;
        begin
            shifted = value >>> FRACTION_BITS;
            max_base = (1 << (WIDTH - 1)) - 1;
            min_base = -(1 << (WIDTH - 1));
            max_val = {{(LUT_DATA_WIDTH+1){max_base[WIDTH-1]}}, max_base};
            min_val = {{(LUT_DATA_WIDTH+1){min_base[WIDTH-1]}}, min_base};
            clipped = shifted;
            if (shifted > max_val) begin
                clipped = max_val;
            end else if (shifted < min_val) begin
                clipped = min_val;
            end
            sat_shift = clipped[WIDTH-1:0];
        end
    endfunction

    // AXI4-Stream flow control
    assign s_axis_tready = m_axis_tready || !m_axis_tvalid;

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

    // Stream processing
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;
            m_axis_tdata  <= '0;
            phase_acc     <= '0;
        end else begin
            if (s_axis_tvalid && s_axis_tready) begin
                logic signed [LUT_DATA_WIDTH-1:0] cos_val;
                logic signed [LUT_DATA_WIDTH-1:0] sin_val;
                cos_val = cos_lut[phase_index];
                sin_val = sin_lut[phase_index];

                for (int ch = 0; ch < NUM_CHANNELS; ch++) begin
                    int base = ch * 2 * WIDTH;
                    logic signed [WIDTH-1:0] in_i;
                    logic signed [WIDTH-1:0] in_q;
                    logic signed [WIDTH+LUT_DATA_WIDTH-1:0] prod_i_cos;
                    logic signed [WIDTH+LUT_DATA_WIDTH-1:0] prod_q_sin;
                    logic signed [WIDTH+LUT_DATA_WIDTH-1:0] prod_q_cos;
                    logic signed [WIDTH+LUT_DATA_WIDTH-1:0] prod_i_sin;
                    logic signed [WIDTH+LUT_DATA_WIDTH:0] real_accum;
                    logic signed [WIDTH+LUT_DATA_WIDTH:0] imag_accum;

                    in_i = s_axis_tdata[base +: WIDTH];
                    in_q = s_axis_tdata[base + WIDTH +: WIDTH];

                    prod_i_cos = $signed(in_i) * $signed(cos_val);
                    prod_q_sin = $signed(in_q) * $signed(sin_val);
                    real_accum = $signed(prod_i_cos) + $signed(prod_q_sin);

                    prod_q_cos = $signed(in_q) * $signed(cos_val);
                    prod_i_sin = $signed(in_i) * $signed(sin_val);
                    imag_accum = $signed(prod_q_cos) - $signed(prod_i_sin);

                    m_axis_tdata[base +: WIDTH] <= sat_shift(real_accum);
                    m_axis_tdata[base + WIDTH +: WIDTH] <= sat_shift(imag_accum);
                end

                m_axis_tvalid <= 1'b1;
                m_axis_tlast  <= s_axis_tlast;
                phase_acc     <= phase_acc + phase_inc;
            end else if (m_axis_tvalid && m_axis_tready) begin
                m_axis_tvalid <= 1'b0;
            end
        end
    end
endmodule
