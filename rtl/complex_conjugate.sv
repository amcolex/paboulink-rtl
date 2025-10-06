// Simple complex conjugate block for OFDM-style IQ data paths.
module complex_conjugate #(
    parameter int WIDTH = 16
) (
    input  logic signed [WIDTH-1:0] i_real,
    input  logic signed [WIDTH-1:0] i_imag,
    output logic signed [WIDTH-1:0] o_real,
    output logic signed [WIDTH-1:0] o_imag
);
    // Pass through the real part and negate the imaginary part to form the conjugate.
    assign o_real = i_real;
    assign o_imag = -i_imag;
endmodule
