// ============================================================================
// @file      cordic_atan2_beh.v
// @brief     Behavioral CORDIC (vectoring atan2) for OSS co-simulation
// @details
//   Drop-in replacement for the Xilinx CORDIC IP `cordic_atan2` used by
//   sbm_alg2_sobel. Implements a PARAMETERIZED output latency (LATENCY) so the
//   testbench can (a) run with the SAME latency the DUT expects (PASS) and
//   (b) run with a WRONG latency (FAIL) to prove F2's alignment guard works.
//
//   The numeric core (`cordic_atan2_16`) is a reference CORDIC shared via
//   cordic_atan2_func.vh; absolute accuracy is not the point — alignment is.
//
//   Latency contract (must match the real Xilinx CORDIC IP):
//     m_axis_dout_tvalid  = s_axis_cartesian_tvalid  delayed by exactly LATENCY
//     m_axis_dout_tdata   = s_axis_cartesian_tdata   delayed by exactly LATENCY
//   i.e. a fully-pipelined CORDIC configured with "Latency = LATENCY" asserts
//   tvalid LATENCY cycles after tvalid. The output is taken directly from the
//   LAST pipeline register stage (no extra output register) so the delay is
//   exactly LATENCY, not LATENCY+1/+2.
//
//   Port list matches the Xilinx AXI4-Stream CORDIC wrapper used in the RTL.
// @author    WorkBuddy
// ============================================================================
`timescale 1ns/1ps
`ifndef CORDIC_BEH_LAT
`define CORDIC_BEH_LAT 21
`endif
`include "cordic_atan2_func.vh"

module cordic_atan2 #(
	parameter LATENCY = `CORDIC_BEH_LAT
)(
	input  wire        aclk,
	input  wire        s_axis_cartesian_tvalid,
	input  wire [23:0] s_axis_cartesian_tdata,   // {dx[11:0], dy[11:0]}
	output wire        m_axis_dout_tvalid,
	output wire [15:0] m_axis_dout_tdata
);
	// LATENCY pipeline stages (valid + data travel together, so they stay
	// locked: a real CORDIC never skews its own tvalid vs tdata).
	reg signed [11:0] dx_q [0:LATENCY-1];
	reg signed [11:0] dy_q [0:LATENCY-1];
	reg               vld_q [0:LATENCY-1];
	integer k;

	always @(posedge aclk) begin
		if (LATENCY >= 1) begin
			dx_q[0] <= s_axis_cartesian_tdata[23:12];
			dy_q[0] <= s_axis_cartesian_tdata[11:0];
			vld_q[0] <= s_axis_cartesian_tvalid;
			for (k = 1; k < LATENCY; k = k + 1) begin
				dx_q[k] <= dx_q[k-1];
				dy_q[k] <= dy_q[k-1];
				vld_q[k] <= vld_q[k-1];
			end
		end
	end

	// Output = last pipeline stage (index LATENCY-1). Total delay = LATENCY.
	assign m_axis_dout_tvalid = (LATENCY >= 1) ? vld_q[LATENCY-1]
	                                            : s_axis_cartesian_tvalid;
	assign m_axis_dout_tdata  = (LATENCY >= 1)
		? cordic_atan2_16(
			{ {4{dx_q[LATENCY-1][11]}}, dx_q[LATENCY-1] },
			{ {4{dy_q[LATENCY-1][11]}}, dy_q[LATENCY-1] })
		: 16'd0;
endmodule
