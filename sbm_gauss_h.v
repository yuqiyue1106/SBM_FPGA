// ============================================================================
// @file      sbm_gauss_h.v
// @brief     7 抽头水平高斯滤波
// @details
//   核系数 [2, 7, 14, 18, 14, 7, 2] / 64（σ ≈ 1.37），与 OpenCV
//   GaussianBlur(7×7, σ=0) 自动推导核完全一致（修订：原二项核
//   [1,6,15,20,15,6,1]/64 尾系数偏差 1/32，已对齐 OpenCV）。
//
//   边界：BORDER_REPLICATE（与 line2Dup.cpp quantizedOrientations 一致）
//     - 左边界：行首指示 i_row_start=1 时，7 个抽头整体装载行首像素。
//     - 右边界：行末后本模块内部自动补 3 拍、以末像素持续右移复制。
//   中心对齐：因果窗口输出 = 中心窗口滞后 3 列，故每行丢弃前 3 个因果输出，
//     每行恰好输出 IMG_W 个有效像素（列 0..IMG_W-1），无气泡。
//   延迟：2 拍（移位寄存器 1 拍 + 加法树 1 拍），吞吐 1 像素/时钟。
//   舍入：四舍五入，(sum + 32) >> 6。全部移位实现，0 DSP。
//   N-8b 修复（硬件强制行间隙，消除软契约）：行尾补 3 拍期间 o_ready=0，
//     顶层据此拉低 s_axis_tready，上游背靠背满速送数时补零拍必然排空，
//     不再吞掉下一行真实像素（修法照搬 alg8 P2：tready 门控 flush_c==0）。
// @param[in]  clk        时钟
// @param[in]  rst_n      低有效异步复位
// @param[in]  i_valid    输入有效（每行 IMG_W 个像素）
// @param[in]  i_row_start 行首指示（本拍为行首像素，与 i_valid 对齐）
// @param[in]  i_data     输入像素（8bit）
// @param[out] o_ready    输入就绪（行尾补 3 拍期间为 0，硬件强制行间隙）
// @param[out] o_valid    输出有效（每行 IMG_W 个，与 o_data 对齐）
// @param[out] o_row_start 输出行首标记（与 o_valid 对齐）
// @param[out] o_data     输出像素（8bit）
// @author    WorkBuddy
// @date      2026-08-14
// ============================================================================
`include "sbm_geometry.vh"
module sbm_gauss_h (
	input  wire       clk,
	input  wire       rst_n,

	// 图像尺寸（顶层传入）
	input  wire [12:0] i_img_w,      ///< 当前图像宽度
	input  wire [12:0] i_img_h,      ///< 当前图像高度

	// 行首指示（本拍为行首像素，与 i_valid 对齐）
	input  wire       i_row_start,   ///< 行首指示（本拍为行首像素，与 i_valid 对齐）

	// 输入像素（8bit）
	input  wire       i_valid,       ///< 输入有效（每行 IMG_W 个像素）
	input  wire [7:0] i_data,
	output wire       o_ready,       ///< 输入就绪（flush_c!=0 期间为 0，硬件强制行间隙）

	// 输出像素（8bit）
	output wire       o_valid,       ///< 输出有效（每行 IMG_W 个，与 o_data 对齐）
	output wire [7:0] o_data,

	// 末行标记（与 o_valid 对齐）
	output wire       o_row_start,   ///< 输出行首标记（与 o_valid 对齐）
	output wire       o_frame_last_row///< 末行标记（与 o_valid 对齐）
	
);

	// ==================== 内部信号声明 ====================
	reg  [12:0] pix_cnt;             ///< 输入行内列计数，范围 0..IMG_W-1
	reg  [1:0]  flush_c;             ///< 右边界复制剩余拍数，范围 0..3
	reg  [7:0]  last_pix;            ///< 当前行末像素，用作右边界复制数据源
	reg  [12:0] row_cnt;             ///< 当前输入行号，用于识别帧末行
	reg         last_row_active;     ///< 当前行及其右边界复制阶段属于帧末行
	reg  [7:0]  tap [0:6];           ///< 水平 7 抽头移位窗口，行首时整体装载首像素
	reg  [13:0] sum;                 ///< 七抽头加权和寄存器，用于输出舍入与缩放
	reg         vld_d1;              ///< 内部输入有效信号延迟 1 拍
	reg         vld_d2;              ///< 内部输入有效信号延迟 2 拍，对齐卷积结果
	reg         frame_last_d1;       ///< 帧末行标记延迟 1 拍
	reg         frame_last_d2;       ///< 帧末行标记延迟 2 拍，对齐输出数据
	reg  [12:0] w_cycle;             ///< 含右边界复制拍的行内周期计数，范围 0..IMG_W+2
	reg  [12:0] w_cycle_d1;          ///< 行内周期计数延迟 1 拍，对齐输出发射窗口
	reg  [12:0] w_cycle_d2;          ///< 行内周期计数延迟 2 拍（流水线保留）

	wire        w_row_first;         ///< 内部行首标记，排除右边界复制阶段
	wire        w_in_valid;          ///< 卷积数据路径有效，包含真实像素和复制像素
	wire [7:0]  w_in_data;           ///< 卷积数据路径输入，复制阶段选择当前行末像素
	wire        w_frame_last_row;    ///< 当前卷积输入属于帧末行的组合标记
	wire [9:0]  w_t06;               ///< 对称抽头 tap[0] 与 tap[6] 的和
	wire [9:0]  w_t15;               ///< 对称抽头 tap[1] 与 tap[5] 的和
	wire [9:0]  w_t24;               ///< 对称抽头 tap[2] 与 tap[4] 的和
	wire [10:0] w_2t;                ///< w_t06 乘以系数 2
	wire [12:0] w_7t;                ///< w_t15 乘以系数 7
	wire [13:0] w_14t;               ///< w_t24 乘以系数 14
	wire [13:0] w_18t;               ///< 中心抽头 tap[3] 乘以系数 18
	wire [13:0] w_sum;               ///< 水平七抽头加权和（除以 64 前）

	integer i;                        ///< 抽头移位寄存器复位与更新循环索引

	// ==================== 输入行内计数与右边界复制（行尾补 3 拍末像素） ====================
	// N-9 修复（TB 逐拍探针定位）：行首整体装载必须由"本模块自维护的
	//   列计数器"判定，不能直接信任外部 i_row_start 脉冲——上游若把
	//   组合列计数器（如 alg1 顶层 pix_cnt）当判据，i_row_start 会在
	//   第 0、1 两拍连续拉高（pix_cnt 在 posedge 更新，第 2 拍仍读到 0），
	//   首两拍被整体装载两次 → 移位寄存器被覆盖、水平输出整体滞后 2 拍。
	//   现以 i_valid 拍自计 in_col，in_col==0 拍即行首，脉宽恒 1 拍。
	//   注意：行末 pix_cnt 回绕到 0 后紧跟 3 个 flush 拍，必须用
	//   flush_c==0 排除，否则 flush 拍被当行首反复复位计数器/重装抽头。
	assign w_row_first = (pix_cnt == 13'd0) && (flush_c == 2'd0);
	// N-8b: 行尾补零期(flush_c!=0)拉低 o_ready —— 硬件强制行间 ≥3 拍间隙。
	// 旧版软契约下若上游满速连发，i_valid 恒 1 使 flush_c 永不递减，
	// 补零拍吞掉下一行真实像素 → 全图静默错位（与 alg8 P2 同类缺陷）。
	assign o_ready = (flush_c == 2'd0);
	assign w_in_valid = (i_valid && o_ready) || (flush_c != 2'd0);
	assign w_in_data  = (flush_c != 2'd0) ? last_pix : i_data;
	assign w_frame_last_row = (i_valid && o_ready && (row_cnt == i_img_h-1))
	                          || ((flush_c != 2'd0) && last_row_active);

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			pix_cnt <= 13'd0; flush_c <= 2'd0; last_pix <= 8'd0;
		end else begin
			if (i_valid && o_ready) begin
				last_pix <= i_data;
				if (pix_cnt == i_img_w-1) begin
					pix_cnt <= 13'd0;
					flush_c <= 2'd3;             // 行尾补 3 拍复制
				end else
					pix_cnt <= pix_cnt + 13'd1;
			end else if (flush_c != 2'd0)
				flush_c <= flush_c - 2'd1;       // 复制期逐拍递减
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			row_cnt        <= 13'd0;
			last_row_active <= 1'b0;
		end else if (i_valid && o_ready) begin
			if (i_row_start) begin
				last_row_active <= (row_cnt == i_img_h-1);
				if (pix_cnt == i_img_w-1)
					row_cnt <= (row_cnt == i_img_h-1) ? 13'd0 : row_cnt + 13'd1;
			end else begin
				last_row_active <= (row_cnt == i_img_h-1);
				if (pix_cnt == i_img_w-1)
					row_cnt <= (row_cnt == i_img_h-1) ? 13'd0 : row_cnt + 13'd1;
			end
		end
	end

	// ==================== 7 级抽头（行首整体装载 = 左边界复制） ====================
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			for (i = 0; i < 7; i = i + 1) tap[i] <= 8'd0;
		end else if (w_in_valid) begin
			if (w_row_first) begin      // 左边界复制：整体装载（自计列 0，脉宽恒 1 拍）
				for (i = 0; i < 7; i = i + 1) tap[i] <= w_in_data;
			end else begin              // 正常右移
				for (i = 0; i < 6; i = i + 1) tap[i] <= tap[i+1];
				tap[6] <= w_in_data;
			end
		end
	end

	// ==================== 加法树：[2,7,14,18,14,7,2]/64（OpenCV 口径） ====================
	// 2=2, 7=8-1, 14=16-2, 18=16+2，全部移位实现，0 DSP
	assign w_t06 = tap[0] + tap[6];                        // 0..510
	assign w_t15 = tap[1] + tap[5];
	assign w_t24 = tap[2] + tap[4];
	assign w_2t  = w_t06 << 1;                             // *2
	assign w_7t  = (w_t15 << 3) - w_t15;                   // *7
	assign w_14t = (w_t24 << 4) - (w_t24 << 1);            // *14
	assign w_18t = (tap[3] << 4) + (tap[3] << 1);          // *18
	assign w_sum = w_2t + w_7t + w_14t + w_18t;            // 14bit, 最大 16320

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			sum <= 14'd0; vld_d1 <= 1'b0; vld_d2 <= 1'b0;
			frame_last_d1 <= 1'b0;
			frame_last_d2 <= 1'b0;
		end else begin
			sum    <= w_sum;
			vld_d1 <= w_in_valid;
			vld_d2 <= vld_d1;
			frame_last_d1 <= w_frame_last_row;
			frame_last_d2 <= frame_last_d1;
		end
	end

	// ==================== 行内周期计数（含复制拍）：0..i_img_w+2，与 sum/vld_d2 对齐 ====================
	// N-9 一并整改：旧版用外部 i_row_start 复位 w_cycle，同样受其 2 拍
	//   宽脉冲影响（第 2 拍又被清回）。改用本模块自计 w_in_valid 拍数：
	//   w_cycle = 0..IMG_W+2（行内第 1..IMG_W+3 拍）。
	//   对齐推导：抽头寄存 1 拍 → sum 再寄存 1 拍，数据路径共 2 拍；
	//   w_cycle 同拍寄存后经 w_cycle_d1 恰为 2 拍（w_cycle_d2 多滞后
	//   1 拍，旧版误用导致发射窗口选中列 1..W，全行右移 1 列）。
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) w_cycle <= 13'd0;
		else if (w_in_valid) begin
			if (w_row_first) w_cycle <= 13'd0;
			else             w_cycle <= w_cycle + 13'd1;
		end
	end
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin w_cycle_d1 <= 13'd0; w_cycle_d2 <= 13'd0; end
		else begin w_cycle_d1 <= w_cycle; w_cycle_d2 <= w_cycle_d1; end
	end

	// ==================== 输出：丢弃每行前 3 个因果窗口结果（列 -3..-1），发射列 0..IMG_W-1） ====================
	// 有效发射拍：w_cycle_d1 = 3..IMG_W+2（与 sum/vld_d2 严格同拍，含补 3 拍
	//   对应的列 W-3..W-1），共 IMG_W 个；o_row_start 在首拍（列 0）同拍拉高
	assign o_valid     = vld_d2 && (w_cycle_d1 >= 13'd3) && (w_cycle_d1 <= i_img_w + 13'd2);
	assign o_row_start = o_valid && (w_cycle_d1 == 13'd3);
	assign o_frame_last_row = o_valid && frame_last_d2;
	assign o_data      = (sum + 14'd32) >> 6;   // 四舍五入到 8bit，无溢出

endmodule
