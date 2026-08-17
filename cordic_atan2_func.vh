// ============================================================================
// @file      cordic_atan2_func.vh
// @brief     Shared vectoring-mode CORDIC atan2 reference function
// @details
//   Computes theta = atan2(y,x), returned as a 16-bit signed "fraction" in the
//   Xilinx CORDIC convention:  out = round( theta / PI * 2^15 ).
//
//   This function is intentionally a SIMPLE reference CORDIC (not the Xilinx
//   IP). It is included by BOTH the behavioral CORDIC model and the testbench
//   golden so that the two agree EXACTLY. The co-sim only validates TIMING
//   ALIGNMENT (F2) — the absolute numeric accuracy is the IP vendor's job and
//   is out of scope here.
//
//   NOTE: must be `included inside a module (it defines a function, which can
//   only live in a module scope). It reads no external state.
// ============================================================================
`ifndef CORDIC_ATAN2_FUNC_VH
`define CORDIC_ATAN2_FUNC_VH

function [15:0] cordic_atan2_16;
	input signed [15:0] xin;
	input signed [15:0] yin;
	reg signed [31:0] x, y, z;
	reg signed [31:0] xi, yi;
	reg signed [31:0] atan [0:15];
	integer n;
	begin
		// atan(2^-i) in units of (1/PI * 2^15):  round( atan(2^-i)/PI * 32768 )
		atan[0]  = 32'sd8192;  atan[1]  = 32'sd4836;  atan[2]  = 32'sd2555;
		atan[3]  = 32'sd1297;  atan[4]  = 32'sd651;   atan[5]  = 32'sd326;
		atan[6]  = 32'sd163;   atan[7]  = 32'sd81;    atan[8]  = 32'sd41;
		atan[9]  = 32'sd20;    atan[10] = 32'sd10;    atan[11] = 32'sd5;
		atan[12] = 32'sd3;     atan[13] = 32'sd1;     atan[14] = 32'sd1;
		atan[15] = 32'sd0;
		// scale operands up by 2^16 to keep rotation precision
		x = {xin, 16'sd0};
		y = {yin, 16'sd0};
		z = 32'sd0;
		for (n = 0; n < 16; n = n + 1) begin
			if (y >= 0) begin
				xi = x + (y >>> n);
				yi = y - (x >>> n);
				z  = z - atan[n];
			end else begin
				xi = x - (y >>> n);
				yi = y + (x >>> n);
				z  = z + atan[n];
			end
			x = xi; y = yi;
		end
		cordic_atan2_16 = z[15:0];   // 16-bit signed angle fraction
	end
endfunction

`endif // CORDIC_ATAN2_FUNC_VH
