// ============================================================================
// @file      xpm_memory_spram_beh.v
// @brief     Behavioral model of XPM_MEMORY_SPRAM for OSS simulation (iverilog)
// @details
//   Minimal behavioral replacement of the Vivado XPM single-port RAM primitive.
//   Implements the ONLY semantics used by sbm_accum_lane.v:
//     - WRITE_MODE_A = "read_first"
//     - READ_LATENCY_A = 1
//   Address presented at cycle T  ->  douta valid at cycle T+1 (old data if a
//   write to the same address occurs at T). This matches the XPM read_first
//   behavior the RTL relies on.
//
//   NOTE: For synthesis the real XPM primitive is used (the instantiation in
//   sbm_accum_lane.v is unchanged). This file is ONLY for co-simulation in
//   iverilog / Verilator where the Xilinx XPM simulation model is unavailable.
// @author    WorkBuddy
// ============================================================================
`timescale 1ns/1ps
module xpm_memory_spram #(
	parameter MEMORY_SIZE         = 2048,   // bits
	parameter MEMORY_PRIMITIVE    = "block",
	parameter WRITE_DATA_WIDTH_A  = 8,
	parameter READ_DATA_WIDTH_A   = 8,
	parameter READ_LATENCY_A      = 1,
	parameter WRITE_MODE_A        = "read_first",
	parameter READ_RESET_VALUE_A  = "0",
	parameter SIM_ASSERT_CHK      = 0
)(
	input  wire        clka,
	input  wire        rsta,
	input  wire        ena,
	input  wire        regcea,
	input  wire        wea,
	input  wire [10:0] addra,
	input  wire [WRITE_DATA_WIDTH_A-1:0] dina,
	output wire [READ_DATA_WIDTH_A-1:0]  douta,
	input  wire        injectsbiterra,
	input  wire        injectdbiterra,
	input  wire        sleep
);
	localparam DEPTH = MEMORY_SIZE / WRITE_DATA_WIDTH_A;
	reg [READ_DATA_WIDTH_A-1:0] mem [0:DEPTH-1];
	reg [READ_DATA_WIDTH_A-1:0] dout_q;
	integer i;

	initial begin
		for (i = 0; i < DEPTH; i = i + 1) mem[i] = {READ_DATA_WIDTH_A{1'b0}};
	end

	// read_first, latency 1:
	//  - dout_q captures OLD mem[addra] (read happens before write)
	//  - if wea, mem[addra] is updated in the same cycle (after the read)
	always @(posedge clka) begin
		if (rsta) begin
			dout_q <= {READ_DATA_WIDTH_A{1'b0}};
		end else if (ena) begin
			dout_q <= mem[addra];
			if (wea) mem[addra] <= dina;
		end
	end

	assign douta = regcea ? dout_q : dout_q;

endmodule
