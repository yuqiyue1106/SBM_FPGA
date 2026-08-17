// ============================================================================
// @file      xpm_fifo_async_beh.v
// @brief     Behavioral model of XPM_FIFO_ASYNC for OSS simulation (iverilog)
// @details
//   Minimal behavioral replacement of the Vivado XPM asynchronous FIFO,
//   covering the feature set used by sbm_alg11_accum:
//     - FIFO_MEMORY_TYPE = "distributed" (reg array)
//     - READ_MODE = "fwft"  (first-word-fall-through: dout 组合输出队首,
//       rd_en 弹出; 下一项 1 拍后出现在 dout)
//     - FIFO_READ_LATENCY = 1
//     - full/empty/wr_rst_busy/rd_rst_busy/rd_data_count/wr_data_count
//   Semantics (mirroring XPM):
//     - Write while full  -> ignored
//     - Read  while empty -> ignored
//     - 占用计数用行为整数近似(跨域建模不展开格雷码同步)
//   For synthesis the real XPM primitive is used. This file is ONLY for
//   co-simulation under iverilog / Verilator.
// @author    WorkBuddy
// ============================================================================
`timescale 1ns/1ps
module xpm_fifo_async #(
	parameter FIFO_MEMORY_TYPE    = "distributed",
	parameter FIFO_WRITE_DEPTH    = 64,
	parameter WRITE_DATA_WIDTH    = 32,
	parameter READ_DATA_WIDTH     = 32,
	parameter WR_DATA_COUNT_WIDTH = 7,
	parameter RD_DATA_COUNT_WIDTH = 7,
	parameter RELATED_CLOCKS      = 0,
	parameter FIFO_READ_LATENCY   = 1,
	parameter READ_MODE           = "fwft",
	parameter ECC_MODE            = "no_ecc",
	parameter SIM_ASSERT_CHK      = 0,
	parameter WAKEUP_TIME         = 0
)(
	input  wire                          wr_clk,
	input  wire                          rst,
	input  wire                          wr_en,
	input  wire [WRITE_DATA_WIDTH-1:0]   din,
	output wire                          full,
	output wire                          wr_rst_busy,
	output wire [WR_DATA_COUNT_WIDTH-1:0] wr_data_count,
	output wire                          wr_ack,
	output wire                          overflow,
	output wire                          almost_full,
	output wire                          prog_full,
	input  wire                          rd_clk,
	input  wire                          rd_en,
	output wire [READ_DATA_WIDTH-1:0]    dout,
	output wire                          empty,
	output wire                          rd_rst_busy,
	output wire [RD_DATA_COUNT_WIDTH-1:0] rd_data_count,
	output wire                          underflow,
	output wire                          almost_empty,
	output wire                          prog_empty
);
	reg [WRITE_DATA_WIDTH-1:0] mem [0:FIFO_WRITE_DEPTH-1];
	integer wptr, rptr;
	integer occupy;               // 占用数(行为近似)
	reg wack_r, ovf_r;
	reg wr_rst_busy_r, rd_rst_busy_r;

	// ---- 复位脉冲 ----
	always @(posedge wr_clk) begin
		if (rst) begin wptr <= 0; occupy <= 0; wack_r <= 1'b0; ovf_r <= 1'b0; wr_rst_busy_r <= 1'b1; end
		else     begin wr_rst_busy_r <= 1'b0; wack_r <= 1'b0; ovf_r <= 1'b0; end
	end
	always @(posedge rd_clk) begin
		if (rst) begin rptr <= 0; rd_rst_busy_r <= 1'b1; end
		else     begin rd_rst_busy_r <= 1'b0; end
	end

	// ---- 写端口 ----
	always @(posedge wr_clk) begin
		if (!rst && wr_en) begin
			if (occupy < FIFO_WRITE_DEPTH) begin
				mem[wptr] <= din;
				wptr <= (wptr + 1) % FIFO_WRITE_DEPTH;
				occupy <= occupy + 1;
				wack_r <= 1'b1;
			end else begin
				ovf_r <= 1'b1;
				if (SIM_ASSERT_CHK) $display("XPM_FIFO_ASYNC: write while full (ignored)");
			end
		end
	end

	// ---- 读端口 (fwft: dout 组合输出队首; rd_en 弹) ----
	always @(posedge rd_clk) begin
		if (!rst && rd_en && occupy > 0) begin
			rptr <= (rptr + 1) % FIFO_WRITE_DEPTH;
			occupy <= occupy - 1;
		end
	end

	// ---- fwft 组合 dout ----
	assign dout = (occupy > 0) ? mem[rptr] : {READ_DATA_WIDTH{1'b0}};

	// ---- 标志 ----
	assign full      = (occupy >= FIFO_WRITE_DEPTH);
	assign empty     = (occupy <= 0);
	assign wr_rst_busy = wr_rst_busy_r;
	assign rd_rst_busy = rd_rst_busy_r;
	assign wr_ack    = wack_r;
	assign overflow  = ovf_r;
	assign underflow = 1'b0;
	assign almost_full  = (occupy >= FIFO_WRITE_DEPTH - 1);
	assign almost_empty = (occupy <= 1);
	assign prog_full = (occupy >= FIFO_WRITE_DEPTH);
	assign prog_empty = (occupy <= 0);
	assign rd_data_count = (occupy > 0) ? occupy[RD_DATA_COUNT_WIDTH-1:0] : {RD_DATA_COUNT_WIDTH{1'b0}};
	assign wr_data_count = (occupy > 0) ? occupy[WR_DATA_COUNT_WIDTH-1:0] : {WR_DATA_COUNT_WIDTH{1'b0}};

endmodule
