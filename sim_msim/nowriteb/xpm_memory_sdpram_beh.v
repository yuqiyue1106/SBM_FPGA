// ============================================================================
// @file      xpm_memory_sdpram_beh.v
// @brief     Behavioral model of XPM_MEMORY_SDPRAM for OSS simulation (iverilog)
// @details
//   Behavioral replacement of the Vivado XPM simple-dual-port RAM primitive,
//   used ONLY for OSS co-simulation (iverilog / Verilator). It now FAITHFULLY
//   honors WRITE_MODE_A on a same-cycle read/write collision (clka==clkb,
//   addra==addrb, wea active), so iverilog can catch the same class of bugs
//   that ModelSim / Vivado (the real XPM primitive) would expose:
//     - "read_first" : read returns OLD (pre-write) data        [default]
//     - "no_change"  : read output HOLDS its previous value (does not update)
//     - "write_first": read returns NEW (post-write) data
//   READ_LATENCY_B = 1 (one-cycle read latency on port B).
//   For synthesis the real XPM primitive is used.
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
	parameter SIM_ASSERT_CHK      = 0,
	parameter SIM_GENCHECK        = 0
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
	localparam ADDR_BITS = $clog2(DEPTH);
	reg [WRITE_DATA_WIDTH_A-1:0] mem [0:DEPTH-1];
	reg [READ_DATA_WIDTH_B-1:0] pipe [0:3];
	integer i, k;

	// The real XPM address ports are $clog2(DEPTH) bits wide. ModelSim pads a
	// narrow connection with Z (iverilog pads with 0), and mem[addr_with_z]
	// reads all-X -- so mask to the usable bits here.
	wire [ADDR_BITS-1:0] wr_addr = addra[ADDR_BITS-1:0];
	wire [ADDR_BITS-1:0] rd_addr = addrb[ADDR_BITS-1:0];

	initial begin
		for (i = 0; i < DEPTH; i = i + 1) mem[i] = {WRITE_DATA_WIDTH_A{1'b0}};
	end

	// ---- write port A (takes effect AFTER the read on the same edge) ----
	always @(posedge clka) begin
		if (ena && wea) mem[wr_addr] <= dina;
	end

	// ---- read port B, READ_LATENCY_B cycles, honoring WRITE_MODE_A ----
	// On a same-cycle read/write collision (addra==addrb, wea active):
	//   "read_first" -> OLD data (pre-write)   [matches default non-blocking]
	//   "no_change"  -> read output HOLDS its previous value (no update)
	//   "write_first"-> NEW data (post-write)
	wire w_collision = (ena && wea && (wr_addr == rd_addr));
	always @(posedge clkb) begin
		if (enb) begin
			if (w_collision && WRITE_MODE_A == "no_change")
				pipe[0] <= pipe[0];              // hold previous value
			else if (w_collision && WRITE_MODE_A == "write_first")
				pipe[0] <= dina;                // new (post-write) data
			else
				pipe[0] <= mem[rd_addr];         // old (pre-write) data
			for (k = 1; k < READ_LATENCY_B && k < 4; k = k + 1)
				pipe[k] <= pipe[k-1];
		end
	end
	assign doutb = pipe[READ_LATENCY_B-1];

endmodule
