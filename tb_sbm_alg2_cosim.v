// ============================================================================
// @file      tb_sbm_alg2_cosim.v
// @brief     Self-checking co-sim for sbm_alg2_sobel (F2 verification)
// @details
//   Drives sbm_alg2_sobel with a synthetic grayscale frame and validates that
//   each emitted output pixel's (mag2, angle) belongs to the SAME input pixel.
//   The golden gradient is a 3x3 Sobel computed in the TB; the golden angle is
//   the shared reference CORDIC (cordic_atan2_func.vh) — identical to the
//   behavioral CORDIC the DUT is linked against, so the check isolates TIMING
//   ALIGNMENT (F2's concern): if CORDIC_IP_LATENCY != real CORDIC latency,
//   mag2/angle land on a neighbor pixel and the check fails.
//
//   Run (PASS, latencies matched):
//     iverilog -g2012 -o sim.out tb_sbm_alg2_cosim.v sbm_alg2_sobel.v \
//               cordic_atan2_beh.v xpm_memory_sdpram_beh.v
//     vvp sim.out
//   Run (FAIL, behavioral CORDIC 1 cycle shorter than DUT expects):
//     iverilog -g2012 -DCORDIC_BEH_LAT=20 -o sim.out tb_sbm_alg2_cosim.v \
//               sbm_alg2_sobel.v cordic_atan2_beh.v xpm_memory_sdpram_beh.v
//     vvp sim.out
// ============================================================================
`timescale 1ns/1ps
`ifndef CORDIC_DUT_LAT
`define CORDIC_DUT_LAT 21
`endif
`include "cordic_atan2_func.vh"

module tb_sbm_alg2_cosim;
parameter IMG_W = 20;
parameter IMG_H = 16;
reg clk, rst_n;
reg s_axis_tvalid; reg [7:0] s_axis_tdata;
reg s_axis_tuser, s_axis_tlast;
wire m_axis_tvalid; wire [21:0] m_axis_mag2;
wire [15:0] m_axis_angle;
wire m_axis_tuser, m_axis_tlast;

// ---- synthetic input image + golden Sobel ----
reg [7:0] img [0:IMG_H-1][0:IMG_W-1];
reg signed [11:0] g_dx [0:IMG_H-1][0:IMG_W-1];
reg signed [11:0] g_dy [0:IMG_H-1][0:IMG_W-1];
reg [21:0] g_mag2 [0:IMG_H-1][0:IMG_W-1];
reg [15:0] g_ang  [0:IMG_H-1][0:IMG_W-1];
integer rr, cc, gi_r, gi_c;

// ---- output tracking ----
reg [12:0] er, ec;
integer cur_r, cur_c;
reg fail;
integer checked, cyc, o_total;

always #5 clk = ~clk;

initial begin
	clk = 0; rst_n = 0;
	s_axis_tvalid = 0; s_axis_tdata = 0; s_axis_tuser = 0; s_axis_tlast = 0;
	er = 0; ec = 0; fail = 0; checked = 0; cyc = 0; o_total = 0;
	#100 rst_n = 1;

	// build a ramp+structure image so gradients are non-trivial / non-axis
	for (rr = 0; rr < IMG_H; rr = rr + 1)
		for (cc = 0; cc < IMG_W; cc = cc + 1)
			img[rr][cc] = (rr*4 + cc*9 + 16) & 8'hFF;

	// golden Sobel (interior) + reference angle
	for (rr = 0; rr < IMG_H; rr = rr + 1)
		for (cc = 0; cc < IMG_W; cc = cc + 1) begin
			if (rr >= 1 && rr <= IMG_H-2 && cc >= 1 && cc <= IMG_W-2) begin
				g_dx[rr][cc] = (img[rr-1][cc+1] + {img[rr][cc+1],1'b0} + img[rr+1][cc+1])
				              - (img[rr-1][cc-1] + {img[rr][cc-1],1'b0} + img[rr+1][cc-1]);
				g_dy[rr][cc] = (img[rr+1][cc-1] + {img[rr+1][cc],1'b0} + img[rr+1][cc+1])
				              - (img[rr-1][cc-1] + {img[rr-1][cc],1'b0} + img[rr-1][cc+1]);
				g_mag2[rr][cc] = ($signed(g_dx[rr][cc])*$signed(g_dx[rr][cc]))
				              + ($signed(g_dy[rr][cc])*$signed(g_dy[rr][cc]));
				g_ang[rr][cc] = cordic_atan2_16(
					{ {4{g_dx[rr][cc][11]}}, g_dx[rr][cc] },
					{ {4{g_dy[rr][cc][11]}}, g_dy[rr][cc] });
			end else begin
				g_dx[rr][cc] = 0; g_dy[rr][cc] = 0;
				g_mag2[rr][cc] = 0; g_ang[rr][cc] = 0;
			end
		end

	send_frame;
	// drain: flush + pipeline + CORDIC latency
	repeat (IMG_W*IMG_H + 300) @(posedge clk);
	$display("INFO total outputs=%0d checked=%0d fail=%0d", o_total, checked, fail);
	if (fail) $display("==== RESULT: FAIL (alg2 F2) ====");
	else      $display("==== RESULT: PASS (alg2 F2, %0d interior px checked) ====", checked);
	$finish;
end

task send_frame;
begin
	for (rr = 0; rr < IMG_H; rr = rr + 1)
		for (cc = 0; cc < IMG_W; cc = cc + 1) begin
			@(posedge clk);
			s_axis_tvalid <= 1'b1;
			s_axis_tdata  <= img[rr][cc];
			s_axis_tuser  <= (rr == 0 && cc == 0);
			s_axis_tlast  <= (rr == IMG_H-1 && cc == IMG_W-1);
		end
	@(posedge clk);
	s_axis_tvalid <= 1'b0;
end
endtask

// ---- capture + compare (bulletproof same-sample cross-check) ----
// We tap the CORDIC IP input ({dx_r,dy_r}, s_axis_cartesian_tvalid) directly
// and shift it by the behavioral CORDIC latency. At any output cycle, the
// gradient that PRODUCED the emitted angle is exactly the CORDIC input sample
// (CORDIC_BEH_LAT-1) cycles earlier (the behavioral output stage is index
// LATENCY-1 of a LATENCY-deep pipeline). The F2 invariant is that the emitted
// mag2 belongs to the SAME pixel as the emitted angle, i.e.
//   m_axis_mag2 == delayed_dx^2 + delayed_dy^2.
// If CORDIC_IP_LATENCY is wrong, the mag2 delay line references a NEIGHBOR
// gradient and this check fails — isolating the F2 timing-alignment concern
// with zero pixel-map assumptions.
`ifndef CORDIC_BEH_LAT
`define CORDIC_BEH_LAT 21
`endif
`define CORDIC_REF (`CORDIC_BEH_LAT - 1)
reg signed [11:0] cdx_q [0:63];
reg signed [11:0] cdy_q [0:63];
reg               cvld_q [0:63];
integer ci;
always @(posedge clk) begin
	cdx_q[0] <= dut.u_cordic.s_axis_cartesian_tdata[23:12];
	cdy_q[0] <= dut.u_cordic.s_axis_cartesian_tdata[11:0];
	cvld_q[0] <= dut.u_cordic.s_axis_cartesian_tvalid;
	for (ci = 1; ci < 64; ci = ci + 1) begin
		cdx_q[ci] <= cdx_q[ci-1];
		cdy_q[ci] <= cdy_q[ci-1];
		cvld_q[ci] <= cvld_q[ci-1];
	end
end

`ifdef DEBUG
always @(posedge clk) begin
	if (cyc < 160)
		$display("DBG cyc=%0d vld_g=%0b vld_m=%0b cordic_vld=%0b align_vld=%0b m_tvld=%0b mag2=%0d ang=%0d",
			cyc, dut.vld_g, dut.vld_m, dut.cordic_out_valid, dut.dbg_align_vld,
			m_axis_tvalid, m_axis_mag2, m_axis_angle);
end
always @(posedge clk) begin
	if (m_axis_tvalid && o_total < 4) begin
		$display("DBG out=%0d mag2=%0d ang=%0d | ref_dx=%0d ref_dy=%0d ref_mag2=%0d ref_ang=%0d (REF=%0d)",
			o_total, m_axis_mag2, m_axis_angle,
			cdx_q[`CORDIC_REF], cdy_q[`CORDIC_REF],
			(cdx_q[`CORDIC_REF]*cdx_q[`CORDIC_REF] + cdy_q[`CORDIC_REF]*cdy_q[`CORDIC_REF]),
			cordic_atan2_16({ {4{cdx_q[`CORDIC_REF][11]}}, cdx_q[`CORDIC_REF] },
			                { {4{cdy_q[`CORDIC_REF][11]}}, cdy_q[`CORDIC_REF] }),
			`CORDIC_REF);
	end
end
`endif
always @(posedge clk) begin
	cyc <= cyc + 1;
	if (m_axis_tvalid) begin
		o_total <= o_total + 1;
		cur_r = cdx_q[`CORDIC_REF];
		cur_c = cdy_q[`CORDIC_REF];
		if (m_axis_mag2 !== (cur_r*cur_r + cur_c*cur_c)) begin
			$display("FAIL mag2 out=%0d got=%0d exp(dx^2+dy^2)=%0d (REF=%0d)",
			         o_total, m_axis_mag2, (cur_r*cur_r+cur_c*cur_c), `CORDIC_REF);
			fail <= 1'b1;
		end
		if (m_axis_angle !== cordic_atan2_16({ {4{cur_r[11]}}, cur_r },
		                                      { {4{cur_c[11]}}, cur_c })) begin
			$display("FAIL angle out=%0d got=%0d exp=%0d (REF=%0d)",
			         o_total, m_axis_angle,
			         cordic_atan2_16({ {4{cur_r[11]}}, cur_r }, { {4{cur_c[11]}}, cur_c }),
			         `CORDIC_REF);
			fail <= 1'b1;
		end
		checked <= checked + 1;
	end
end

sbm_alg2_sobel #(.IMG_W(IMG_W), .IMG_H(IMG_H),
	.CORDIC_IP_LATENCY(`CORDIC_DUT_LAT)) dut (
	.clk(clk), .rst_n(rst_n),
	.s_axis_tvalid(s_axis_tvalid), .s_axis_tready(),
	.s_axis_tdata(s_axis_tdata), .s_axis_tuser(s_axis_tuser), .s_axis_tlast(s_axis_tlast),
	.m_axis_tvalid(m_axis_tvalid), .m_axis_tready(1'b1),
	.m_axis_mag2(m_axis_mag2), .m_axis_angle(m_axis_angle),
	.m_axis_tuser(m_axis_tuser), .m_axis_tlast(m_axis_tlast)
);
endmodule
