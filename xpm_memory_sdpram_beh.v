// ============================================================================
// @file      xpm_memory_sdpram_beh.v
// @brief     Behavioral model of XPM_MEMORY_SDPRAM for OSS simulation (iverilog)
// @details
//   Minimal behavioral replacement of the Vivado XPM simple-dual-port RAM
//   primitive. Implements the semantics used by sbm_alg2/alg3/alg9:
//     - WRITE_MODE_A = "read_first"  (port-A read shows OLD data on a same-
//       cycle write; since clka==clkb here, the non-blocking read captures the
//       pre-write value)
//     - READ_LATENCY_B = 1  (one-cycle read latency on port B; up to 2 support)
//   For synthesis the real XPM primitive is used. This file is ONLY for
//   co-simulation under iverilog / Verilator.
// @author    WorkBuddy
// ============================================================================
`timescale 1ns/1ps
module xpm_memory_sdpram #(
	parameter MEMORY_SIZE         = 2048,
	parameter MEMORY_PRIMITIVE    = "block",
	parameter WRITE_DATA_WIDTH_A  = 8,
	parameter READ_DATA_WIDTH_B   = 8,
	parameter READ_LATENCY_B      = 1,
	parameter WRITE_MODE_A        = "read_first",
	parameter SIM_ASSERT_CHK      = 0
)(
	input  wire        clka,
	input  wire        ena,
	input  wire        wea,
	input  wire [31:0] addra,
	input  wire [WRITE_DATA_WIDTH_A-1:0] dina,
	input  wire        clkb,
	input  wire        enb,
	input  wire [31:0] addrb,
	output wire [READ_DATA_WIDTH_B-1:0]  doutb
);
	localparam DEPTH = MEMORY_SIZE / WRITE_DATA_WIDTH_A;
	reg [WRITE_DATA_WIDTH_A-1:0] mem [0:DEPTH-1];
	reg [READ_DATA_WIDTH_B-1:0] dout_q;
	reg [READ_DATA_WIDTH_B-1:0] pipe [0:3];
	integer i, k;

	initial begin
		for (i = 0; i < DEPTH; i = i + 1) mem[i] = {WRITE_DATA_WIDTH_A{1'b0}};
	end

	// ---- write port A (read_first: write takes effect AFTER the read) ----
	always @(posedge clka) begin
		if (ena && wea) mem[addra] <= dina;
	end

	// ---- read port B, READ_LATENCY_B cycles ----
	always @(posedge clkb) begin
		if (enb) begin
			pipe[0] <= mem[addrb];            // old data (write on same edge is captured next)
			for (k = 1; k < READ_LATENCY_B && k < 4; k = k + 1)
				pipe[k] <= pipe[k-1];
		end
	end
	assign doutb = pipe[READ_LATENCY_B-1];

endmodule
