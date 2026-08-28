// ============================================================================
// tb_sbm_alg2_sobel.v : alg2 Sobel+CORDERIC 方向 C-golden 功能测试台（V-3 整改）
// ----------------------------------------------------------------------------
// 验证内容（逐拍比对，任意不符即 FAIL）：
//   1) 梯度：输出(r,c) 对应输入中心(r,c) 的 3×3 Sobel（BORDER_REPLICATE
//      边界复制；因果窗口标签(r+1,c+1)=中心(r,c)，丢弃第0行/第0列后
//      输出坐标与中心坐标重合），dx/dy 经层次探针逐拍比对；
//   2) 幅值：mag2 22bit 位精确比对；
//   3) 方向：angle = 共享参考 CORDIC cordic_atan2_16(dx,dy)（与 DUT 链接的
//      行为级 CORDIC 同一函数），位精确比对；
//   4) 协议：tuser 仅帧首拍、tlast 每行末拍；每帧 IMG_W×IMG_H 个输出，
//      无气泡、无丢拍；
//   5) 帧间回归：帧 1 以真实系统间隙（≥3 行空闲）发送，行尾补 1 拍/帧尾
//      补 1 行的软契约窗口被显式满足，任意行门控/补拍缺失立即 FAIL。
// 激励：ramp + 结构块 + LCG 噪声混合图样（非退化），梯度方向覆盖四象限。
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

// ---- golden 方向函数（与 DUT 行为级 CORDIC 共享同一参考实现） ----
`include "cordic_atan2_func.vh"
// ---- 包含 DUT（其内部 include sbm_geometry.vh 被上方保护宏短路） ----
`include "sbm_alg2_sobel.v"

module tb_sbm_alg2_sobel;

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
wire [21:0] m_axis_mag2;
wire [15:0] m_axis_angle;
wire       m_axis_tuser;
wire       m_axis_tlast;

sbm_alg2_sobel u_dut (
	.clk(clk), .rst_n(rst_n),
	.img_w(IMG_W), .img_h(IMG_H),
	.s_axis_tdata(s_axis_tdata), .s_axis_tvalid(s_axis_tvalid),
	.s_axis_tuser(s_axis_tuser), .s_axis_tlast(s_axis_tlast),
	.s_axis_tready(s_axis_tready),
	.m_axis_mag2(m_axis_mag2), .m_axis_angle(m_axis_angle),
	.m_axis_tvalid(m_axis_tvalid),
	.m_axis_tuser(m_axis_tuser), .m_axis_tlast(m_axis_tlast),
	.m_axis_tready(1'b1)
);

always #5 clk = ~clk;

// ==================== 激励图像：ramp + 结构块 + LCG 噪声 ====================
reg [7:0] img [0:NFRAMES*NPX-1];
reg [31:0] lcg_state;
function [31:0] lcg_next;
	begin
		lcg_state = lcg_state * 32'd1664525 + 32'd1013904223;
		lcg_next  = lcg_state;
	end
endfunction

integer k, kr, kc;
reg [31:0] lv;
reg [15:0] base;
initial begin
	lcg_state = 32'h5EED2;
	for (k = 0; k < NFRAMES*NPX; k = k + 1) begin
		kr = (k / IMG_W) % IMG_H;
		kc = k % IMG_W;
		// 非退化混合图样：ramp 基底 + 中心亮方块（强梯度，四象限方向
		// 均覆盖）+ LCG 噪声。golden 逐像素从 img[] 重新计算，任意
		// 窗口错位/加法树溢出立即 FAIL。（两帧 LCG 相位不同）
		base = (kr + kc) & 16'hFF;
		lv = lcg_next();
		img[k] = base[7:0];
		// 中心 1/4 区域亮方块：四条边分别产生 +dx/-dx/+dy/-dy 强梯度
		if (kr >= IMG_H/4 && kr < 3*IMG_H/4 && kc >= IMG_W/4 && kc < 3*IMG_W/4)
			img[k] = img[k] + 8'd96;
		// LCG 低 3bit 噪声（打破恒定梯度退化，检验任意 dx/dy 组合）
		img[k] = img[k] + lv[2:0];
	end
end

// ==================== C-golden：3×3 Sobel（BORDER_REPLICATE） ====================
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

// 输出(r,c) = 输入中心(r,c) 的 Sobel（因果窗口标签(r+1,c+1)，发射时
// 计数器已 +2、丢弃条件 ≥1，输出坐标与中心坐标恰好重合）。返回 dx/dy 分量
function integer g_dx(input integer f, r, c);
	integer cr, cc, xp, xn;
	begin
		cr = r; cc = c;
		xp = img[pidx(f, clampi(cr-1,0,IMG_H-1), clampi(cc+1,0,IMG_W-1))]
		   + 2*img[pidx(f, cr, clampi(cc+1,0,IMG_W-1))]
		   + img[pidx(f, clampi(cr+1,0,IMG_H-1), clampi(cc+1,0,IMG_W-1))];
		xn = img[pidx(f, clampi(cr-1,0,IMG_H-1), clampi(cc-1,0,IMG_W-1))]
		   + 2*img[pidx(f, cr, clampi(cc-1,0,IMG_W-1))]
		   + img[pidx(f, clampi(cr+1,0,IMG_H-1), clampi(cc-1,0,IMG_W-1))];
		g_dx = xp - xn;
	end
endfunction

function integer g_dy(input integer f, r, c);
	integer cr, cc, yp, yn;
	begin
		cr = r; cc = c;
		yp = img[pidx(f, clampi(cr+1,0,IMG_H-1), clampi(cc-1,0,IMG_W-1))]
		   + 2*img[pidx(f, clampi(cr+1,0,IMG_H-1), cc)]
		   + img[pidx(f, clampi(cr+1,0,IMG_H-1), clampi(cc+1,0,IMG_W-1))];
		yn = img[pidx(f, clampi(cr-1,0,IMG_H-1), clampi(cc-1,0,IMG_W-1))]
		   + 2*img[pidx(f, clampi(cr-1,0,IMG_H-1), cc)]
		   + img[pidx(f, clampi(cr-1,0,IMG_H-1), clampi(cc+1,0,IMG_W-1))];
		g_dy = yp - yn;
	end
endfunction

// ==================== 驱动器：严格帧协议 + 消费契约合规 ====================
// 行间插 3 拍空闲（模拟 alg1 gauss_h 硬件强制的行间隙）、帧间 4*IMG_W
// 空闲（模拟 gauss_v 底部冲刷）：alg2 的行尾补 1 拍/帧尾补 1 行为软契约，
// 背靠背发送会吞掉补拍（TB 驱动必须合规，否则失败属驱动问题而非 RTL）。
task send_frame(input integer f);
	integer r, c;
	begin
		for (r = 0; r < IMG_H; r = r + 1) begin
			for (c = 0; c < IMG_W; c = c + 1) begin
				// 在 posedge 使用 NBA 更新：DUT 于当前沿采样旧值，下一沿
				// 采样本次设置的稳定信号，避免测试平台与 DUT 直接竞态
				@(posedge clk);
				s_axis_tdata  <= img[pidx(f, r, c)];
				s_axis_tuser  <= (r == 0 && c == 0);
				s_axis_tlast  <= (c + 1 == IMG_W);
				s_axis_tvalid <= 1'b1;
			//	do @(posedge clk); while (!s_axis_tready);
			end
			// 行尾契约间隙（≥1 拍，此处 3 拍与真实 alg1 一致）
			@(posedge clk);
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
integer exp_dx, exp_dy, exp_mag2;
reg [15:0] exp_ang;

// 当前输出坐标对应的组合参考结果。
always_comb begin
	exp_dx   = g_dx(m_frame, m_row, m_col);
	exp_dy   = g_dy(m_frame, m_row, m_col);
	exp_mag2 = exp_dx*exp_dx + exp_dy*exp_dy;
	exp_ang  = cordic_atan2_16(exp_dx[15:0], exp_dy[15:0]);
end

// dx/dy 层次探针：输出拍的数据对应 CORDIC 输入时刻的 dx_r/dy_r。
//   时序链（像素 p 于拍 m 出窗）：w_dx(m)组合 → 沿 m+1 dx_r<=w_dx、
//   vld_g<=vld_w → 拍 m+1 起 CORDIC 输入有效，经 21 拍于沿 m+22 输出。
//   TB 移位链 dly[0] 与 dx_r 同沿采样；输出为组合 w_emit（无末级寄存器），
//   监视器在输出沿读到的是 dly[DLAT-1] 的沿前值，故实际深度取 19
//   （实测标定：dx/dy 与 mag2/angle 同拍对齐）。
localparam DLAT = 21;
reg signed [11:0] dx_dly [0:DLAT-1];
reg signed [11:0] dy_dly [0:DLAT-1];
integer dli;
always @(posedge clk) begin
	dx_dly[0] <= u_dut.dx_r;
	dy_dly[0] <= u_dut.dy_r;
	for (dli = 1; dli < DLAT; dli = dli + 1) begin
		dx_dly[dli] <= dx_dly[dli-1];
		dy_dly[dli] <= dy_dly[dli-1];
	end
end



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
		// ---- 梯度/幅值/方向比对 ----
		if (dx_dly[DLAT-1] !== exp_dx[11:0]) begin
			err = err + 1;
			if (err <= 40)
				$display("ERR dx @ frame=%0d row=%0d col=%0d: got=%0d exp=%0d",
				         m_frame, m_row, m_col, dx_dly[DLAT-1], exp_dx);
		end
		if (dy_dly[DLAT-1] !== exp_dy[11:0]) begin
			err = err + 1;
			if (err <= 40)
				$display("ERR dy @ frame=%0d row=%0d col=%0d: got=%0d exp=%0d",
				         m_frame, m_row, m_col, dy_dly[DLAT-1], exp_dy);
		end
		if (m_axis_mag2 !== exp_mag2[21:0]) begin
			err = err + 1;
			if (err <= 40)
				$display("ERR mag2 @ frame=%0d row=%0d col=%0d: got=%0d exp=%0d",
				         m_frame, m_row, m_col, m_axis_mag2, exp_mag2);
		end
		if (m_axis_angle !== exp_ang) begin
			err = err + 1;
			if (err <= 40)
				$display("ERR angle @ frame=%0d row=%0d col=%0d: got=%0d exp=%0d",
				         m_frame, m_row, m_col, m_axis_angle, exp_ang);
		end
		
		total = total + 1;
	end
end

// 输出监视器坐标推进。使用非阻塞赋值，保证上方检查器在当前时钟沿读取
// 推进前的 m_frame/m_row/m_col，避免两个 always 块之间产生仿真竞争。
always @(posedge clk) begin
	if (rst_n && m_axis_tvalid) begin
		if (m_col == IMG_W-1) begin
			m_col <= 0;
			if (m_row == IMG_H-1) begin
				m_row   <= 0;
				m_frame <= m_frame + 1;
			end else begin
				m_row <= m_row + 1;
			end
		end else begin
			m_col <= m_col + 1;
		end
	end
end

// ==================== 主流程 ====================
initial begin
	rst_n = 1'b0;
	repeat (10) @(posedge clk);
	rst_n = 1'b1;

	// 帧 0：满速背靠背
	send_frame(0);
	// 帧间间隙 = 底部冲刷 3 行 + 余量（与 alg1 顶层真实帧间隙语义一致：
	// alg2/alg3 的补 1 拍/补 1 行软契约依赖该间隙，系统级由 alg1 保证）
	repeat (4*IMG_W) @(posedge clk);
	// 帧 1：验证帧间状态无残留
	send_frame(1);

	// 等待输出排空（CORDIC 21 拍 + 帧尾补 1 行 IMG_W+1 拍 + 余量）
	repeat (3*IMG_W + 100) @(posedge clk);

	if (total !== NFRAMES*NPX) begin
		$display("ERR output count: got=%0d exp=%0d", total, NFRAMES*NPX);
		err = err + 1;
	end
	if (err == 0 && total == NFRAMES*NPX)
		$display("PASS: %0d pixels (dx/dy/mag2/angle + protocol) matched", total);
	else
		$display("FAIL: err=%0d total=%0d (exp %0d)", err, total, NFRAMES*NPX);
	$finish;
end

// 看门狗
initial begin
	#( 40_000_000 );
	$display("FAIL: TIMEOUT (simulation watchdog)");
	$finish;
end

endmodule
