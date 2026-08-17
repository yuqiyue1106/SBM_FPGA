// ============================================================================
// @file      sbm_gauss_v.v
// @brief     7 抽头垂直高斯滤波（6 行缓冲轮转 + 加法树）
// @details
//   核系数 [2, 7, 14, 18, 14, 7, 2] / 64（σ ≈ 1.37），与 OpenCV
//   GaussianBlur(7×7, σ=0) 自动推导核一致（修订：原二项核已对齐 OpenCV）。
//   依赖 XPM_MEMORY_SDPRAM（Vivado 自带原语，行缓冲）。
//   边界：BORDER_REPLICATE（与 line2Dup.cpp quantizedOrientations 一致）
//     - 上边界：帧首行（第 0 行）期间 6 个行缓冲全部写入第 0 行（复制第 0 行）。
//     - 下边界：末行结束后本模块自生成 3 行底部复制。
//   中心对齐：因果窗口（行 r-6..r）输出 = 中心窗口（行 r-3），输出行 0..IMG_H-1。
//   帧间约束：两帧之间需预留 ≥ 3 行空闲时间供底部复制排空（顶层以 AXI4-Stream 反压保证）。
// @param[in]  clk         时钟
// @param[in]  rst_n       低有效异步复位
// @param[in]  i_valid     输入有效（每行 IMG_W 个像素，无气泡）
// @param[in]  i_row_start 行首（与 i_valid 对齐）
// @param[in]  i_frame_end 帧末行结束指示（电平，触发底部复制）
// @param[in]  i_data      输入像素（8bit）
// @param[out] o_valid     输出有效（IMG_W × IMG_H）
// @param[out] o_data      输出像素（8bit）
// @author    WorkBuddy
// @date      2026-08-14
// ============================================================================
`include "sbm_geometry.vh"
module sbm_gauss_v #(
	parameter IMG_W = `SBM_L0_W,    ///< 图像宽度（级 0, 16T 对齐, 由 sbm_geometry.vh 派生）
	parameter IMG_H = `SBM_L0_H     ///< 图像高度（级 0, 16T 对齐, 由 sbm_geometry.vh 派生）
)(
	input  wire       clk,
	input  wire       rst_n,
	input  wire       i_valid,       ///< 输入有效（每行 IMG_W 个像素，无气泡）
	input  wire       i_row_start,   ///< 行首（与 i_valid 对齐）
	input  wire       i_frame_end,   ///< 帧末行结束指示（电平，触发底部复制）
	input  wire [7:0] i_data,
	output wire       o_valid,       ///< 输出有效（IMG_W × IMG_H）
	output wire [7:0] o_data
);

	// ==================== 行列计数（对齐流，含底部复制行） ====================
	reg  [12:0] pix_cnt;             ///< 0..IMG_W-1
	reg  [12:0] row_cnt;             ///< 处理行计数 0..IMG_H+2（含底部复制 3 行）
	reg  [2:0]  r6;                  ///< row_cnt mod 6（缓冲轮转选择）
	reg         flush_started;       ///< 底部复制期标记
	reg  [2:0]  flush_k;             ///< 底部复制行序号 1..3
	reg         in_win;              ///< 帧内有效窗（含底部复制）

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			pix_cnt <= 13'd0; row_cnt <= 13'd0; r6 <= 3'd0;
			flush_started <= 1'b0; flush_k <= 3'd0; in_win <= 1'b0;
		end else begin
			// 新帧首个行首：复位行计数（帧间以 i_frame_end 为界）
			if (i_row_start && i_valid && !in_win) begin
				row_cnt <= 13'd0;
				r6      <= 3'd0;
				flush_started <= 1'b0;
				flush_k <= 3'd0;
			end
			if (i_row_start && i_valid)
				in_win <= 1'b1;
			// 底部复制触发：末行最后一像素完成
			if (i_valid && (row_cnt == IMG_H-1) && (pix_cnt == IMG_W-1) && i_frame_end) begin
				flush_started <= 1'b1;
				flush_k <= 3'd1;
			end
			// 行/像素推进：真实输入行与底部复制行统一推进
			if (i_valid || flush_started) begin
				if (pix_cnt == IMG_W-1) begin
					pix_cnt <= 13'd0;
					row_cnt <= row_cnt + 13'd1;
					r6      <= (r6 == 3'd5) ? 3'd0 : r6 + 3'd1;
					if (flush_started) begin
						if (flush_k == 3'd3) begin
							flush_started <= 1'b0;
							in_win  <= 1'b0;
						end else
							flush_k <= flush_k + 3'd1;
					end
				end else
					pix_cnt <= pix_cnt + 13'd1;
			end
		end
	end

	// ==================== 6 个行缓冲：按行号 mod 6 轮转写入，端口 A 写 / 端口 B 读 ====================
	reg  [12:0] w_addr_d;             ///< pix_cnt 延迟 1 拍（底部复制写地址）
	reg  [7:0]  cur_d;                ///< 当前行数据延迟 1 拍（对齐读延迟）
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin w_addr_d <= 13'd0; cur_d <= 8'd0; end
		else begin
			w_addr_d <= pix_cnt;
			cur_d    <= i_data;
		end
	end
	// 底部复制时末行缓冲号 = (IMG_H-1) mod 6 = (r6-flush_k) mod 6
	wire [2:0] w_last_buf = (r6 >= flush_k) ? (r6 - flush_k) : (r6 - flush_k + 3'd6);
	wire [7:0] lb_read [0:5];
	wire       lb_we   [0:5];
	// 写地址：正常 = 当前列；底部复制 = 延迟 1 拍的列
	// 写数据：正常 = 当前行数据；底部复制 = lb_read 末行缓冲（该读出口 = 延迟列地址旧值）
	wire [12:0] w_addra = flush_started ? w_addr_d : pix_cnt;
	wire [7:0]  w_dina  = flush_started ? lb_read[w_last_buf] : i_data;

	genvar g;
	generate
		for (g = 0; g < 6; g = g + 1) begin : gen_lb
			// 写使能：第 0 行期间全部缓冲写第 0 行（上边界复制）；其余仅 r6 缓冲写
			assign lb_we[g] = (i_valid || flush_started) && ((row_cnt == 13'd0) || (r6 == g[2:0]));
			xpm_memory_sdpram #(
				.MEMORY_SIZE        (IMG_W*8),
				.MEMORY_PRIMITIVE   ("auto"),
				.WRITE_DATA_WIDTH_A (8),
				.READ_DATA_WIDTH_B  (8),
				.READ_LATENCY_B     (1),
				.WRITE_MODE_A       ("no_change")
			) u_lb (
				.clka  (clk),
				.ena   (1'b1),
				.wea   (lb_we[g]),
				.addra (w_addra),
				.dina  (w_dina),
				.clkb  (clk),
				.enb   (1'b1),
				.addrb (pix_cnt),
				.doutb (lb_read[g])
			);
		end
	endgenerate

	// ==================== 垂直抽头：v0 = 当前行，v1..v6 = 轮转读出的前 6 行 ====================
	wire [2:0] w_sel1 = (r6 >= 3'd1) ? (r6 - 3'd1) : (r6 + 3'd5);
	wire [2:0] w_sel2 = (r6 >= 3'd2) ? (r6 - 3'd2) : (r6 + 3'd4);
	wire [2:0] w_sel3 = (r6 >= 3'd3) ? (r6 - 3'd3) : (r6 + 3'd3);
	wire [2:0] w_sel4 = (r6 >= 3'd4) ? (r6 - 3'd4) : (r6 + 3'd2);
	wire [2:0] w_sel5 = (r6 >= 3'd5) ? (r6 - 3'd5) : (r6 + 3'd1);
	wire [7:0] v0 = (flush_started) ? lb_read[w_last_buf] : cur_d;
	wire [7:0] v1 = lb_read[w_sel1];
	wire [7:0] v2 = lb_read[w_sel2];
	wire [7:0] v3 = lb_read[w_sel3];
	wire [7:0] v4 = lb_read[w_sel4];
	wire [7:0] v5 = lb_read[w_sel5];
	wire [7:0] v6 = lb_read[r6];

	// ==================== 加法树：[2,7,14,18,14,7,2]/64（OpenCV 口径） ====================
	// 2=2, 7=8-1, 14=16-2, 18=16+2，全部移位实现，0 DSP
	wire [9:0]  w_t06 = v0 + v6;
	wire [9:0]  w_t15 = v1 + v5;
	wire [9:0]  w_t24 = v2 + v4;
	wire [10:0] w_2t  = w_t06 << 1;                            // *2
	wire [12:0] w_7t  = (w_t15 << 3) - w_t15;                  // *7
	wire [13:0] w_14t = (w_t24 << 4) - (w_t24 << 1);           // *14
	wire [13:0] w_18t = (v3 << 4) + (v3 << 1);                // *18
	wire [13:0] w_sum = w_2t + w_7t + w_14t + w_18t;          // 14bit, 最大 16320

	reg  [13:0] sum;
	reg  [12:0] row_cnt_r;             ///< 输出行（处理行 -3）
	reg         vld_r;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			sum <= 14'd0; row_cnt_r <= 13'd0; vld_r <= 1'b0;
		end else begin
			sum       <= w_sum;
			row_cnt_r <= row_cnt;
			vld_r     <= (i_valid || flush_started);
		end
	end

	// 输出有效：处理行 3..IMG_H+2 对应输出行 0..IMG_H-1（因果 → 中心滞后 3 行）
	assign o_valid = vld_r && (row_cnt_r >= 13'd3) && (row_cnt_r < IMG_H + 13'd3);
	assign o_data  = (sum + 14'd32) >> 6;

endmodule
