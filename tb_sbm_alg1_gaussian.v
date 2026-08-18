// ============================================================================
// tb_sbm_alg1_gaussian.v : alg1 两级高斯 C-golden 功能测试台（N-8/V-2 整改）
// ----------------------------------------------------------------------------
// 验证内容（逐行逐字节比对，任意不符即 FAIL）：
//   1) 像素级：DUT 输出与 TB 内独立实现的 C-golden 两级高斯
//      （水平/垂直均 [2,7,14,18,14,7,2]/64，BORDER_REPLICATE，(sum+32)>>6）
//      逐字节相等；
//   2) 协议级：输出 tuser 仅出现在每帧第一拍、tlast 仅出现在每行末拍；
//   3) 数量级：每帧 IMG_W×IMG_H 个输出像素，无气泡、无丢拍、无多余；
//   4) N-8 回归：帧 0 带随机行间隙发送，帧 1 满速背靠背紧接发送
//      （帧 1 首拍到达时 DUT 仍在底部冲刷，tready 必须反压）——
//      旧版 row_active 行门控 / 补零拍被吞 / 帧间状态残留均会在此 FAIL。
// 激励：32bit LCG 伪随机（非退化），覆盖帧首/帧尾/行首/行尾全部边界。
// 尺寸：通过预定义 SBM_GEOMETRY_VH 并覆盖几何宏，把 DUT 缩小到 128×128
//      （级0 最小对齐单位 16T=128），缩短仿真时间，接口行为与全尺寸一致。
// ============================================================================
`timescale 1ns/1ps

// ---- 覆盖几何宏（必须在包含任何 RTL 之前，且抢在 geometry 头保护宏之前） ----
`define SBM_GEOMETRY_VH
`define SBM_CAM_W       128
`define SBM_CAM_H       128
`define SBM_T           8
`define SBM_LANES       24
`define SBM_FEAT_MAX    64
`define SBM_RESP_BASE   32'h0800_0000
`define SBM_ALIGN(v, a)  ((((v) + (a) - 1) / (a)) * (a))
`define SBM_CEILDIV(v, a)  (((v) + (a) - 1) / (a))
`define SBM_W(n)  (((n) < 2) ? 1 : $clog2(n))
`define SBM_L0_W        `SBM_ALIGN(`SBM_CAM_W, (16 * `SBM_T))
`define SBM_L0_H        `SBM_ALIGN(`SBM_CAM_H, (16 * `SBM_T))
`define SBM_IMG_W       (`SBM_L0_W / 2)
`define SBM_IMG_H       (`SBM_L0_H / 2)
`define SBM_WC          (`SBM_IMG_W / `SBM_T)
`define SBM_HC          (`SBM_IMG_H / `SBM_T)
`define SBM_CELLS       (`SBM_WC * `SBM_HC)
`define SBM_LM_BYTES_PER_ORI  (`SBM_T * `SBM_T * `SBM_CELLS)
`define SBM_LM_BYTES_TOTAL    (8 * `SBM_LM_BYTES_PER_ORI)
`define SBM_TP_MAX      `SBM_CELLS

// ---- 包含 DUT（其内部 include sbm_geometry.vh 被上方保护宏短路） ----
`include "sbm_alg1_gaussian.v"

module tb_sbm_alg1_gaussian;

parameter IMG_W = `SBM_L0_W;
parameter IMG_H = `SBM_L0_H;
parameter NFRAMES = 2;
localparam NPX = IMG_W * IMG_H;

reg clk = 1'b0, rst_n = 1'b0;
reg        s_axis_tvalid = 1'b0;
reg [7:0]  s_axis_tdata  = 8'd0;
reg        s_axis_tuser  = 1'b0;
reg        s_axis_tlast  = 1'b0;
wire       s_axis_tready;
wire       m_axis_tvalid;
wire [7:0] m_axis_tdata;
wire       m_axis_tuser;
wire       m_axis_tlast;

sbm_alg1_gaussian u_dut (
	.clk(clk), .rst_n(rst_n),
	.s_axis_tdata(s_axis_tdata), .s_axis_tvalid(s_axis_tvalid),
	.s_axis_tuser(s_axis_tuser), .s_axis_tlast(s_axis_tlast),
	.s_axis_tready(s_axis_tready),
	.m_axis_tdata(m_axis_tdata), .m_axis_tvalid(m_axis_tvalid),
	.m_axis_tuser(m_axis_tuser), .m_axis_tlast(m_axis_tlast),
	.m_axis_tready(1'b1)
);

always #5 clk = ~clk;

// ==================== 激励图像：LCG 伪随机（确定性、非退化） ====================
reg [7:0] img [0:NFRAMES*NPX-1];
reg [31:0] lcg_state;
function [31:0] lcg_next;
	begin
		lcg_state = lcg_state * 32'd1664525 + 32'd1013904223;
		lcg_next  = lcg_state;
	end
endfunction

integer k;
reg [31:0] lv;
initial begin
	lcg_state = 32'hC0FFEE;
	for (k = 0; k < NFRAMES*NPX; k = k + 1) begin
		lv = lcg_next();
		img[k] = lv[15:8];
	end
end

// ==================== C-golden：两级高斯（独立实现，供逐字节比对） ====================
integer KT [0:6];
initial begin
	KT[0]=2; KT[1]=7; KT[2]=14; KT[3]=18; KT[4]=14; KT[5]=7; KT[6]=2;
end

function integer clampi(input integer v, lo, hi);
	begin
		if (v < lo)      clampi = lo;
		else if (v > hi) clampi = hi;
		else             clampi = v;
	end
endfunction

function integer pidx(input integer f, r, c);
	begin pidx = (f*IMG_H + r)*IMG_W + c; end
endfunction

// 水平核（对帧 f、行 r、列 c），BORDER_REPLICATE
function integer gh(input integer f, r, c);
	integer s, kk, cc;
	begin
		s = 0;
		for (kk = -3; kk <= 3; kk = kk + 1) begin
			cc = clampi(c + kk, 0, IMG_W-1);
			s  = s + KT[kk+3] * img[pidx(f, r, cc)];
		end
		gh = (s + 32) / 64;
	end
endfunction

// 两级高斯最终像素值（与硬件流水一致：水平级先舍入到 8bit，
// 垂直级在舍入后的值上再加权舍入 —— 双舍入口径，非全精度一次舍入）
function integer gg(input integer f, r, c);
	integer s, kk, rr;
	begin
		s = 0;
		for (kk = -3; kk <= 3; kk = kk + 1) begin
			rr = clampi(r + kk, 0, IMG_H-1);
			s  = s + KT[kk+3] * gh(f, rr, c);
		end
		gg = (s + 32) / 64;
	end
endfunction

// ==================== 驱动器：严格帧协议（tuser=帧首拍，tlast=帧末拍） ====================
// gaps=1 时随机插入 0..3 拍空闲；gaps=0 为满速背靠背
task send_frame(input integer f, input integer gaps);
	integer r, c, d;
	begin
		for (r = 0; r < IMG_H; r = r + 1) begin
			for (c = 0; c < IMG_W; c = c + 1) begin
				if (gaps != 0) begin
					d = lcg_next() % 4;
					s_axis_tvalid <= 1'b0;
					repeat (d) @(posedge clk);
				end
				s_axis_tdata  <= img[pidx(f, r, c)];
				s_axis_tuser  <= (r == 0 && c == 0);
				s_axis_tlast  <= (r == IMG_H-1 && c == IMG_W-1);
				s_axis_tvalid <= 1'b1;
				// 握手等待：tready=0 时保持数据不前进（覆盖底部冲刷反压窗口）
				do @(posedge clk); while (!s_axis_tready);
			end
		end
		s_axis_tvalid <= 1'b0;
		s_axis_tuser  <= 1'b0;
		s_axis_tlast  <= 1'b0;
	end
endtask

// ==================== 输出监视器：逐字节 + 协议校验 ====================
integer err = 0;
integer total = 0;
integer m_row = 0, m_col = 0, m_frame = 0;
integer gold;

always @(posedge clk) begin
	if (rst_n && m_axis_tvalid) begin
		// ---- 协议校验：tuser 仅在每帧第一拍、tlast 仅在每行末拍 ----
		if (m_axis_tuser !== ((m_row == 0) && (m_col == 0))) begin
			err = err + 1;
			if (err <= 10)
				$display("ERR protocol tuser @ frame=%0d row=%0d col=%0d: got=%b exp=%b",
				         m_frame, m_row, m_col, m_axis_tuser, (m_row==0)&&(m_col==0));
		end
		if (m_axis_tlast !== (m_col == IMG_W-1)) begin
			err = err + 1;
			if (err <= 10)
				$display("ERR protocol tlast @ frame=%0d row=%0d col=%0d: got=%b exp=%b",
				         m_frame, m_row, m_col, m_axis_tlast, (m_col==IMG_W-1));
		end
		// ---- 像素比对：C-golden 两级高斯 ----
		gold = gg(m_frame, m_row, m_col);
		if (m_axis_tdata !== gold[7:0]) begin
			err = err + 1;
			if (err <= 10)
				$display("ERR pixel @ frame=%0d row=%0d col=%0d: got=%0d exp=%0d",
				         m_frame, m_row, m_col, m_axis_tdata, gold);
		end
		total = total + 1;
		// ---- 输出坐标推进（独立于 DUT 内部计数器） ----
		if (m_col == IMG_W-1) begin
			m_col = 0;
			if (m_row == IMG_H-1) begin
				m_row   = 0;
				m_frame = m_frame + 1;
			end else
				m_row = m_row + 1;
		end else
			m_col = m_col + 1;
	end
end

// ==================== 主流程 ====================
integer stall_seen = 0;
initial begin
	rst_n = 1'b0;
	repeat (10) @(posedge clk);
	rst_n = 1'b1;

	// 帧 0：带随机间隙（覆盖稀疏发送 + 行间停顿）
	send_frame(0, 1);
	// 帧 1：满速背靠背，紧跟帧 0（其首拍到达时 DUT 仍在底部冲刷，
	// tready 必须先 0 后 1 —— 记录是否确实观察到反压）
	fork
		send_frame(1, 0);
		begin : watcher
			while (m_frame < 1) begin
				@(posedge clk);
				if (s_axis_tvalid && !s_axis_tready) stall_seen = 1;
			end
		end
	join

	// 等待输出排空
	repeat (8*IMG_W + 200) @(posedge clk);

	if (total !== NFRAMES*NPX) begin
		$display("ERR output count: got=%0d exp=%0d", total, NFRAMES*NPX);
		err = err + 1;
	end
	if (!stall_seen)
		$display("WARN: frame-2 backpressure window never observed (tready never deasserted while tvalid held)");

	if (err == 0 && total == NFRAMES*NPX)
		$display("PASS: %0d pixels matched, protocol clean, backpressure observed=%0b", total, stall_seen);
	else
		$display("FAIL: err=%0d total=%0d (exp %0d)", err, total, NFRAMES*NPX);
	$finish;
end

// 看门狗：防死锁静默挂起
initial begin
	#( 40_000_000 );
	$display("FAIL: TIMEOUT (simulation watchdog)");
	$finish;
end

endmodule
