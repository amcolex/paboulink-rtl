
module ofdm_frontend_top #(
    parameter int INPUT_WIDTH            = 12,
    parameter int NUM_CHANNELS           = 2,
    parameter int AXIS_DATA_WIDTH        = NUM_CHANNELS * 2 * INPUT_WIDTH,
    parameter int AXIS_TUSER_WIDTH_IN    = 1,
    parameter int NFFT                   = 2048,
    parameter int CP_LEN                 = 512,
    parameter int N_FRAME_SYMBOLS        = 28,
    parameter int FRAME_GAP_SAMPLES      = N_FRAME_SYMBOLS * (NFFT + CP_LEN),
    parameter int SYMBOL_COUNTER_WIDTH   = 7,
    parameter int STO_ACC_WIDTH          = 12,
    parameter int OUTPUT_MARGIN          = CP_LEN,
    parameter int THRESH_VALUE           = 32'd4096,
    parameter int THRESH_FRAC_BITS       = 15,
    parameter int SMOOTH_SHIFT           = 3,
    parameter int HYSTERESIS             = 2,
    parameter int TIMING_OFFSET          = -CP_LEN,
    parameter int NCO_ACC_WIDTH          = 32,
    parameter int NCO_LUT_ADDR_WIDTH     = 8,
    parameter int NCO_LUT_DATA_WIDTH     = 16
) (
    input  logic                               clk,
    input  logic                               rst,

    // Raw dual-channel IQ AXI4-Stream input
    input  logic [AXIS_DATA_WIDTH-1:0]         s_axis_tdata,
    input  logic [AXIS_TUSER_WIDTH_IN-1:0]     s_axis_tuser,
    input  logic                               s_axis_tvalid,
    output logic                               s_axis_tready,
    input  logic                               s_axis_tlast,

    // Payload AXI4-Stream output after synchronization
    output logic [AXIS_DATA_WIDTH-1:0]         m_axis_tdata,
    output logic [SYMBOL_COUNTER_WIDTH-1:0]    m_axis_tuser,
    output logic                               m_axis_tvalid,
    input  logic                               m_axis_tready,
    output logic                               m_axis_tlast,

    // Sample-time offset control (forwarded to synchronizer)
    input  logic signed [7:0]                  sto_correction,
    input  logic                               sto_valid,
    output logic                               sto_ready,

    // AXI4-Lite control interface exposed by the NCO
    input  logic                               s_axi_awvalid,
    output logic                               s_axi_awready,
    input  logic  [3:0]                        s_axi_awaddr,
    input  logic                               s_axi_wvalid,
    output logic                               s_axi_wready,
    input  logic  [31:0]                       s_axi_wdata,
    input  logic  [3:0]                        s_axi_wstrb,
    output logic                               s_axi_bvalid,
    input  logic                               s_axi_bready,
    output logic  [1:0]                        s_axi_bresp,
    input  logic                               s_axi_arvalid,
    output logic                               s_axi_arready,
    input  logic  [3:0]                        s_axi_araddr,
    output logic                               s_axi_rvalid,
    input  logic                               s_axi_rready,
    output logic [31:0]                        s_axi_rdata,
    output logic  [1:0]                        s_axi_rresp
);
    localparam int DET_TUSER_WIDTH = (AXIS_TUSER_WIDTH_IN < 1) ? 1 : AXIS_TUSER_WIDTH_IN;

    // -------------------------------------------------------------------------
    // Interconnect wires between detector, NCO, and synchronizer
    // -------------------------------------------------------------------------
    logic [AXIS_DATA_WIDTH-1:0] det_tdata;
    logic [DET_TUSER_WIDTH-1:0] det_tuser;
    logic                       det_tvalid;
    logic                       det_s_axis_tready;
    logic                       det_m_axis_tready;
    logic                       det_tlast;

    logic [AXIS_DATA_WIDTH-1:0] nco_tdata;
    logic [DET_TUSER_WIDTH-1:0] nco_tuser;
    logic                       nco_tvalid;
    logic                       nco_tlast;

    logic                       sync_s_axis_tready;

    // Upstream ready mirrors the Minn detector input ready.
    assign s_axis_tready = det_s_axis_tready;

    // -------------------------------------------------------------------------
    // Minn preamble detector
    // -------------------------------------------------------------------------
    minn_preamble_detector #(
        .INPUT_WIDTH(INPUT_WIDTH),
        .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
        .AXIS_TUSER_WIDTH(DET_TUSER_WIDTH),
        .NFFT(NFFT),
        .CP_LEN(CP_LEN),
        .OUTPUT_MARGIN(OUTPUT_MARGIN),
        .THRESH_VALUE(THRESH_VALUE),
        .THRESH_FRAC_BITS(THRESH_FRAC_BITS),
        .SMOOTH_SHIFT(SMOOTH_SHIFT),
        .HYSTERESIS(HYSTERESIS),
        .TIMING_OFFSET(TIMING_OFFSET)
    ) u_minn_detector (
        .clk(clk),
        .rst(rst),
        .s_axis_tdata(s_axis_tdata),
        .s_axis_tuser(s_axis_tuser),
        .s_axis_tvalid(s_axis_tvalid),
        .s_axis_tready(det_s_axis_tready),
        .s_axis_tlast(s_axis_tlast),
        .m_axis_tdata(det_tdata),
        .m_axis_tuser(det_tuser),
        .m_axis_tvalid(det_tvalid),
        .m_axis_tready(det_m_axis_tready),
        .m_axis_tlast(det_tlast)
    );

    // -------------------------------------------------------------------------
    // NCO-based CFO compensator
    // -------------------------------------------------------------------------
    nco_cfo_compensator #(
        .WIDTH(INPUT_WIDTH),
        .NUM_CHANNELS(NUM_CHANNELS),
        .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
        .AXIS_TUSER_WIDTH(DET_TUSER_WIDTH),
        .ACC_WIDTH(NCO_ACC_WIDTH),
        .LUT_ADDR_WIDTH(NCO_LUT_ADDR_WIDTH),
        .LUT_DATA_WIDTH(NCO_LUT_DATA_WIDTH)
    ) u_cfo (
        .clk(clk),
        .rst_n(~rst),
        .s_axis_tdata(det_tdata),
        .s_axis_tuser(det_tuser),
        .s_axis_tvalid(det_tvalid),
        .s_axis_tready(det_m_axis_tready),
        .s_axis_tlast(det_tlast),
        .m_axis_tdata(nco_tdata),
        .m_axis_tuser(nco_tuser),
        .m_axis_tvalid(nco_tvalid),
        .m_axis_tready(sync_s_axis_tready),
        .m_axis_tlast(nco_tlast),
        .s_axi_awvalid(s_axi_awvalid),
        .s_axi_awready(s_axi_awready),
        .s_axi_awaddr(s_axi_awaddr),
        .s_axi_wvalid(s_axi_wvalid),
        .s_axi_wready(s_axi_wready),
        .s_axi_wdata(s_axi_wdata),
        .s_axi_wstrb(s_axi_wstrb),
        .s_axi_bvalid(s_axi_bvalid),
        .s_axi_bready(s_axi_bready),
        .s_axi_bresp(s_axi_bresp),
        .s_axi_arvalid(s_axi_arvalid),
        .s_axi_arready(s_axi_arready),
        .s_axi_araddr(s_axi_araddr),
        .s_axi_rvalid(s_axi_rvalid),
        .s_axi_rready(s_axi_rready),
        .s_axi_rdata(s_axi_rdata),
        .s_axi_rresp(s_axi_rresp)
    );

    // -------------------------------------------------------------------------
    // OFDM frame synchronizer
    // -------------------------------------------------------------------------
    ofdm_frame_synchronizer #(
        .INPUT_WIDTH(INPUT_WIDTH),
        .AXIS_TUSER_WIDTH_IN(DET_TUSER_WIDTH),
        .NFFT(NFFT),
        .CP_LEN(CP_LEN),
        .N_FRAME_SYMBOLS(N_FRAME_SYMBOLS),
        .FRAME_GAP_SAMPLES(FRAME_GAP_SAMPLES),
        .SYMBOL_COUNTER_WIDTH(SYMBOL_COUNTER_WIDTH),
        .STO_ACC_WIDTH(STO_ACC_WIDTH)
    ) u_sync (
        .clk(clk),
        .rst(rst),
        .s_axis_tdata(nco_tdata),
        .s_axis_tuser(nco_tuser),
        .s_axis_tvalid(nco_tvalid),
        .s_axis_tready(sync_s_axis_tready),
        .s_axis_tlast(nco_tlast),
        .m_axis_tdata(m_axis_tdata),
        .m_axis_tuser(m_axis_tuser),
        .m_axis_tvalid(m_axis_tvalid),
        .m_axis_tready(m_axis_tready),
        .m_axis_tlast(m_axis_tlast),
        .sto_correction(sto_correction),
        .sto_valid(sto_valid),
        .sto_ready(sto_ready)
    );
endmodule

`default_nettype wire
