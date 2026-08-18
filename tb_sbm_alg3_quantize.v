// ============================================================================
// tb_sbm_alg3_quantize.v : alg3 滞后梯度量化 C-golden 功能测试台（V-5 整改）
// ----------------------------------------------------------------------------
// 验证内容（逐拍比对，任意不符即 FAIL）：
//   1) 量化方向：golden 与 line2Dup.cpp hysteresisGradient 逐像素对齐——
//      q16=(angle+2048)>>12(16桶)，label=q16&7(8方向合并)；输入边界像素
//      label 强制 0；3×3 中心窗口投票；strong=mag2>900；票数>=5；
//      平票取小索引；输出边界像素(首末行/首末列)强制 0；
//      输出 = strong && !border && votes>=5 ? (1<<best) : 0；
//   2) 协议：tuser 仅帧首拍、tlast 仅行末拍；每帧 IMG_W×IMG_H 个输出，
//      无气泡、无丢拍；
//   3) 帧间回归：双帧发送，帧间状态残留/输出计数器不复位立即 FAIL。
// 激励（非退化，覆盖全判定路径）：
//   · 均匀标签块(label 3, strong)：内部 votes=9 → 单热输出；
//   · 均匀标签块(label 5, weak mag2=800)：votes=9 但弱梯度抑制 → 0；
//   · 棋盘格 label 1/6：votes 5/4 逐像素交替 → 投票边界(>=5)逐拍覆盖；
//   · mag2=900/901 交替：强弱阈值边界(mag2>900)逐拍覆盖；
//   · 桶号 +8 回绕(bucket 8..15 与 0..7 合并为同 label)；
//   · 背景伪随机标签 + 混合强弱梯度（全 8 方向覆盖）。
// 尺寸：几何宏覆盖缩至 128×128（级0 最小对齐单位 16T=128）。
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
`include "sbm_alg3_quantize.v"

module tb_sbm_alg3_quantize;

parameter IMG_W = `SBM_L0_W;
parameter IMG_H = `SBM_L0_H;
parameter NFRAMES = 2;
localparam NPX = IMG_W * IMG_H;

reg clk = 1'b0, rst_n = 1'b0;
reg        s_axis_tvalid = 1'b0;
reg [21:0] s_axis_mag2   = 22'd0;
reg [15:0] s_axis_angle  = 16'd0;
reg        s_axis_tuser  = 1'b0;
reg        s_axis_tlast  = 1'b0;
wire       s_axis_tready;
wire       m_axis_tvalid;
wire [7:0] m_axis_tdata;
wire       m_axis_tuser;
wire       m_axis_tlast;

sbm_alg3_quantize #(.IMG_W(IMG_W), .IMG_H(IMG_H)) u_dut (
	.clk(clk), .rst_n(rst_n),
	.s_axis_mag2(s_axis_mag2), .s_axis_angle(s_axis_angle),
	.s_axis_tvalid(s_axis_tvalid),
	.s_axis_tuser(s_axis_tuser), .s_axis_tlast(s_axis_tlast),
	.s_axis_tready(s_axis_tready),
	.m_axis_tdata(m_axis_tdata),
	.m_axis_tvalid(m_axis_tvalid),
	.m_axis_tuser(m_axis_tuser), .m_axis_tlast(m_axis_tlast),
	.m_axis_tready(1'b1)
);

always #5 clk = ~clk;

// ==================== 激励图样：分区结构化 + 伪随机背景 ====================
// angle：16bit 全圆归一化相位；桶号 b = round(angle/4096) mod 16，
//        label = b & 7（b 与 b+8 合并为同方向）。
function automatic [15:0] stim_angle(input integer f, r, c);
	begin
		if (r >= 8 && r < 24 && c >= 8 && c < 24)
			stim_angle = 16'd12288;                    // 3*4096：均匀 label 3（strong 块）
		else if (r >= 8 && r < 24 && c >= 40 && c < 56)
			stim_angle = (c[0] ? 16'd53248 : 16'd20480); // label 5：桶 13/5 回绕合并验证
		else if (r >= 40 && r < 64 && c >= 8 && c < 64)
			stim_angle = ((r + c) & 1) ? 16'd24576 : 16'd4096; // 棋盘格 label 6/1
		else if (r >= 70 && r < 80 && c >= 8 && c < 32)
			stim_angle = 16'd16384;                    // 4*4096：均匀 label 4（阈值边界块）
		else if (r >= 70 && r < 80 && c >= 40 && c < 64)
			stim_angle = 16'd61440;                    // 桶15 → label 7（最高桶）
		else
			// 背景：全 8 方向覆盖 + 隔像素 +8 桶回绕
			stim_angle = (((r*3 + c*5 + f*7) % 8) + (((r ^ c ^ f) & 1) << 3)) << 12;
	end
endfunction

// mag2：强弱混合，含阈值边界 900/901（strong = mag2 > 900）
function automatic [21:0] stim_mag2(input integer f, r, c);
	begin
		if (r >= 8 && r < 24 && c >= 8 && c < 24)
			stim_mag2 = 22'd2000;                      // 强：votes=9 → 单热输出
		else if (r >= 8 && r < 24 && c >= 40 && c < 56)
			stim_mag2 = 22'd800;                       // 弱：votes=9 也被抑制
		else if (r >= 40 && r < 64 && c >= 8 && c < 64)
			stim_mag2 = 22'd1500;                      // 强：棋盘格 votes 5/4 边界
		else if (r >= 70 && r < 80 && c >= 8 && c < 32)
			stim_mag2 = 22'd900 + c[0];                // 900/901 交替：强弱阈值边界
		else if (r >= 70 && r < 80 && c >= 40 && c < 64)
			stim_mag2 = 22'd901;                       // 恰好强（>900 最小值）
		else
			stim_mag2 = ((r*13 + c*17 + f*29) % 3) * 700; // 0/700/1400 混合
	end
endfunction

// ==================== C-golden：hysteresisGradient 逐像素参考 ====================
function automatic integer clampi(input integer v, lo, hi);
	begin
		if (v < lo)      clampi = lo;
		else if (v > hi) clampi = hi;
		else             clampi = v;
	end
endfunction

// 输入像素标签：边界(首末行/首末列)强制 0；否则 16桶→8方向合并
function automatic [2:0] g_label(input integer f, r, c);
	reg [15:0] ang;
	reg [3:0]  q16;
	begin
		if (r == 0 || r == IMG_H-1 || c == 0 || c == IMG_W-1)
			g_label = 3'd0;
		else begin
			ang = stim_angle(f, r, c);
			q16 = (ang + 16'd2048) >> 12;      // 与 RTL 同宽回绕
			g_label = q16[2:0];
		end
	end
endfunction

// 输出(R,C) 的 3×3 中心窗口内方向 d 的票数。窗口越界仅发生于边界
// 输出（其输出恒为 0，clamp 取值不影响结果）；窗口内的输入边界像素
// 标签已被 g_label 强制为 0（与 RTL w_border 一致，零标签参与投票）
function automatic integer g_votes(input integer f, R, C, d);
	integer cnt, rr, cc;
	begin
		cnt = 0;
		for (rr = R-1; rr <= R+1; rr = rr + 1)
			for (cc = C-1; cc <= C+1; cc = cc + 1)
				if (g_label(f, clampi(rr,0,IMG_H-1), clampi(cc,0,IMG_W-1)) == d[2:0])
					cnt = cnt + 1;
		g_votes = cnt;
	end
endfunction

// 最大票方向：链式严格大于，平票取小索引（与 RTL 级4 一致）
function automatic integer g_best(input integer f, R, C);
	integer d, v, bv, bd;
	begin
		bd = 0;
		bv = g_votes(f, R, C, 0);
		for (d = 1; d < 8; d = d + 1) begin
			v = g_votes(f, R, C, d);
			if (v > bv) begin bd = d; bv = v; end
		end
		g_best = bd;
	end
endfunction

// 输出单热：strong(中心像素 mag2>900) && 非边界 && 票数>=5
function automatic [7:0] g_out(input integer f, R, C);
	integer bd, bv;
	begin
		if (R == 0 || R == IMG_H-1 || C == 0 || C == IMG_W-1)
			g_out = 8'd0;
		else begin
			bd = g_best(f, R, C);
			bv = g_votes(f, R, C, bd);
			if (stim_mag2(f, R, C) > 22'd900 && bv >= 5)
				g_out = 8'h1 << bd;
			else
				g_out = 8'd0;
		end
	end
endfunction

// ==================== 驱动器：严格帧协议 + 消费契约合规 ====================
// 行间 3 拍空闲、帧间 4*IMG_W 空闲（帧尾补 1 行软契约窗口）。
task send_frame(input integer f);
	integer r, c;
	begin
		for (r = 0; r < IMG_H; r = r + 1) begin
			for (c = 0; c < IMG_W; c = c + 1) begin
				// negedge 驱动：避免与 DUT 采样同沿 NBA 竞态
				@(negedge clk);
				s_axis_mag2   <= stim_mag2(f, r, c);
				s_axis_angle  <= stim_angle(f, r, c);
				s_axis_tuser  <= (r == 0 && c == 0);
				s_axis_tlast  <= (r == IMG_H-1 && c == IMG_W-1);
				s_axis_tvalid <= 1'b1;
				do @(posedge clk); while (!s_axis_tready);
			end
			@(negedge clk);
			s_axis_tvalid <= 1'b0;
			s_axis_tuser  <= 1'b0;
			s_axis_tlast  <= 1'b0;
			repeat (3) @(posedge clk);
		end
	end
endtask

// ==================== 输出监视器：逐拍比对 + 协议校验 ====================
integer err = 0;
integer total = 0;
integer m_row = 0, m_col = 0, m_frame = 0;
reg [7:0] exp_data;

always @(posedge clk) begin
	if (rst_n && m_axis_tvalid) begin
		// ---- 协议校验 ----
		if (m_axis_tuser !== ((m_row == 0) && (m_col == 0))) begin
			err = err + 1;
			if (err <= 40)
				$display("ERR protocol tuser @ frame=%0d row=%0d col=%0d", m_frame, m_row, m_col);
		end
		if (m_axis_tlast !== (m_col == IMG_W-1)) begin
			err = err + 1;
			if (err <= 40)
				$display("ERR protocol tlast @ frame=%0d row=%0d col=%0d", m_frame, m_row, m_col);
		end
		// ---- 量化方向单热比对 ----
		exp_data = g_out(m_frame, m_row, m_col);
		if (m_axis_tdata !== exp_data) begin
			err = err + 1;
			if (err <= 40)
				$display("ERR data @ frame=%0d row=%0d col=%0d: got=%b exp=%b",
				         m_frame, m_row, m_col, m_axis_tdata, exp_data);
		end
		total = total + 1;
		// ---- 输出坐标推进 ----
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
initial begin
	rst_n = 1'b0;
	repeat (10) @(posedge clk);
	rst_n = 1'b1;

	send_frame(0);
	// 帧间间隙 = 帧尾补 1 行(IMG_W+1 拍) + 余量
	repeat (4*IMG_W) @(posedge clk);
	// 帧 1：验证帧间状态无残留
	send_frame(1);

	// 等待输出排空（流水线 5 级 + 帧尾补 1 行 + 余量）
	repeat (3*IMG_W + 100) @(posedge clk);

	if (total !== NFRAMES*NPX) begin
		$display("ERR output count: got=%0d exp=%0d", total, NFRAMES*NPX);
		err = err + 1;
	end
	if (err == 0 && total == NFRAMES*NPX)
		$display("PASS: %0d pixels (quantized dir one-hot + protocol) matched", total);
	else
		$display("FAIL: err=%0d total=%0d (exp %0d)", err, total, NFRAMES*NPX);
	$finish;
end

// 看门狗
initial begin
	#( 20_000_000 );
	$display("FAIL: TIMEOUT (simulation watchdog)");
	$finish;
end

endmodule
