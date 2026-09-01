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
// 先读ram, 计算卷积，最后写ram

`include "sbm_geometry.vh"
module sbm_gauss_v (
	input  wire       clk,
	input  wire       rst_n,
	
	//
	input  wire [12:0] i_img_w,      ///< 当前图像宽度
	input  wire [12:0] i_img_h,      ///< 当前图像高度
	
	//
	input  wire       i_valid,       ///< 输入有效（每行 IMG_W 个像素，无气泡）
	input  wire       i_row_start,   ///< 行首（与 i_valid 对齐）
	input  wire       i_frame_end,   ///< 帧末行结束指示（电平，触发底部复制）
	input  wire [7:0] i_data,
	
	output wire       o_valid,       ///< 输出有效（IMG_W × IMG_H）
	output wire [7:0] o_data
);

	// ==================== 内部信号声明 ====================
	reg         valid_d;              ///< 输入有效信号延迟 1 拍
	reg  [7:0]  data_d;               ///< 输入像素数据延迟 1 拍
	reg         row_start_d;          ///< 输入行首标记延迟 1 拍（当前保留）
	reg         frame_end_d;          ///< 输入末行标记延迟 1 拍，用于触发底部复制

	reg  [12:0] pix_cnt;             ///< 0..IMG_W-1
	reg  [12:0] row_cnt;             ///< 处理行计数 0..IMG_H+2（含底部复制 3 行）
	reg  [2:0]  r6;                  ///< row_cnt mod 6（缓冲轮转选择）
	reg         flush_started;       ///< 底部复制期标记
	reg         flush_started_q;     ///< 底部复制标记延迟 1 拍，用于切换卷积数据源
	reg  [2:0]  flush_k;             ///< 底部复制行序号 1..3
	reg  [2:0]  flush_k_rd;          ///< 读数据路径使用的底部复制行序号
	reg         in_win;              ///< 帧内有效窗（含底部复制）

	reg  [12:0] pix_cnt_d1;          ///< 列计数延迟 1 拍，对齐 RAM 读数据
	reg  [12:0] pix_cnt_d2;          ///< 列计数延迟 2 拍，对齐输出有效窗口
	reg  [12:0] pix_cnt_d3;          ///< 列计数延迟 3 拍（流水线保留）
	reg  [12:0] pix_cnt_d4;          ///< 列计数延迟 4 拍（流水线保留）
	reg  [12:0] row_cnt_d1;          ///< 行计数延迟 1 拍
	reg  [12:0] row_cnt_d2;          ///< 行计数延迟 2 拍，对齐卷积结果
	reg  [12:0] row_cnt_d3;          ///< 行计数延迟 3 拍（流水线保留）
	reg  [12:0] row_cnt_d4;          ///< 行计数延迟 4 拍（流水线保留）
	reg  [2:0]  r6_d1;               ///< 行缓冲轮转号延迟 1 拍，对齐 RAM 读数据
	reg  [2:0]  r6_d2;               ///< 行缓冲轮转号延迟 2 拍（选择流水线）
	reg  [2:0]  r6_d3;               ///< 行缓冲轮转号延迟 3 拍（选择流水线）
	reg  [2:0]  r6_d4;               ///< 行缓冲轮转号延迟 4 拍（选择流水线）

	reg  [7:0]  cur_d1;              ///< 当前输入像素延迟 1 拍
	reg  [7:0]  cur_d2;              ///< 当前输入像素延迟 2 拍，作为正常卷积 v0
	reg  [7:0]  cur_d3;              ///< 当前输入像素延迟 3 拍
	reg  [7:0]  cur_d4;              ///< 当前输入像素延迟 4 拍，用于帧首上边界旁路
	reg         w_frame_first_beat;  ///< 新帧首像素标记，用于强制写入全部行缓冲
	reg         w_frame_first_d1;    ///< 新帧首像素标记延迟 1 拍
	reg         w_frame_first_d2;    ///< 新帧首像素标记延迟 2 拍，用于上边界复制选择
	reg  [12:0] w_addr_d;            ///< 列地址延迟 1 拍，用作底部复制写地址
	reg  [12:0] w_addra;             ///< 六个行缓冲共用的写地址
	reg  [7:0]  w_dina;              ///< 六个行缓冲共用的写数据
	reg         lb_we [0:5];         ///< 六个行缓冲各自的寄存写使能

	reg  [2:0]  w_sel1;              ///< 前第 1 行数据对应的行缓冲编号
	reg  [2:0]  w_sel2;              ///< 前第 2 行数据对应的行缓冲编号
	reg  [2:0]  w_sel3;              ///< 前第 3 行数据对应的行缓冲编号
	reg  [2:0]  w_sel4;              ///< 前第 4 行数据对应的行缓冲编号
	reg  [2:0]  w_sel5;              ///< 前第 5 行数据对应的行缓冲编号

	wire [2:0]  w_last_buf;          ///< 底部复制期间保存末行数据的缓冲编号
	wire [2:0]  w_r6m1;              ///< 当前轮转号减 1 的模 6 结果
	wire        w_flush_beat0;       ///< 底部复制行的首拍标记
	wire [7:0]  lb_read [0:5];       ///< 六个行缓冲的读数据输出
	wire        w_we_norm [0:5];     ///< 各行缓冲的正常写使能
	wire        w_we_late [0:5];     ///< 冲刷行末列的延迟补写使能
	wire [7:0]  w_t1;                ///< 前第 1 行的抽头候选数据
	wire [7:0]  w_t2;                ///< 前第 2 行的抽头候选数据
	wire [7:0]  w_t3;                ///< 前第 3 行的抽头候选数据
	wire [7:0]  w_t4;                ///< 前第 4 行的抽头候选数据
	wire [7:0]  w_t5;                ///< 前第 5 行的抽头候选数据
	wire [7:0]  w_t6;                ///< 前第 6 行的抽头候选数据
	wire [7:0]  v0;                  ///< 垂直卷积抽头 0：当前行或末行复制数据
	wire [7:0]  v1;                  ///< 垂直卷积抽头 1：前第 1 行或首行复制数据
	wire [7:0]  v2;                  ///< 垂直卷积抽头 2：前第 2 行或首行复制数据
	wire [7:0]  v3;                  ///< 垂直卷积抽头 3：前第 3 行或首行复制数据
	wire [7:0]  v4;                  ///< 垂直卷积抽头 4：前第 4 行或首行复制数据
	wire [7:0]  v5;                  ///< 垂直卷积抽头 5：前第 5 行或首行复制数据
	wire [7:0]  v6;                  ///< 垂直卷积抽头 6：前第 6 行或首行复制数据
	wire [9:0]  w_t06;               ///< 对称抽头 v0 与 v6 的和
	wire [9:0]  w_t15;               ///< 对称抽头 v1 与 v5 的和
	wire [9:0]  w_t24;               ///< 对称抽头 v2 与 v4 的和
	wire [10:0] w_2t;                ///< w_t06 乘以系数 2
	wire [12:0] w_7t;                ///< w_t15 乘以系数 7
	wire [13:0] w_14t;               ///< w_t24 乘以系数 14
	wire [13:0] w_18t;               ///< 中心抽头 v3 乘以系数 18
	wire [13:0] w_sum;               ///< 七抽头加权和（除以 64 前）

	reg  [13:0] sum;                 ///< 加权和寄存器，用于输出舍入与缩放
	reg  [12:0] row_cnt_r;           ///< 旧版行号对齐寄存器（当前未使用）
	reg         vld_r;               ///< 旧版有效对齐寄存器（当前未使用）
	reg         vld_e;               ///< 旧版二级有效寄存器（当前未使用）
	reg  [12:0] row_cnt_e;           ///< 旧版二级行号寄存器（当前未使用）
	reg         vld_d1;              ///< 卷积数据路径有效信号延迟 1 拍
	reg         vld_d2;              ///< 卷积数据路径有效信号延迟 2 拍，用于输出有效
	reg         vld_d3;              ///< 卷积数据路径有效信号延迟 3 拍（流水线保留）
	reg         vld_d4;              ///< 卷积数据路径有效信号延迟 4 拍（流水线保留）

	genvar g;                         ///< 六个行缓冲生成循环索引

	//输入信号延时 1 latency
	// ==================== 行列计数（对齐流，含底部复制行） ====================
	// ====================================================
	//latency 1: 延迟 1 拍，便于后续逻辑简化
	// ====================================================

	//输入信号延时 1 latency
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			valid_d <= 1'b0;
			data_d <= 8'd0;
			row_start_d <= 1'b0;
			frame_end_d <= 1'b0;
		end else begin
			valid_d <= i_valid;
			data_d <= i_data;
			row_start_d <= i_row_start;
			frame_end_d <= i_frame_end;
		end
	end

	// ==================== 帧内有效窗状态 ====================
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			in_win <= 1'b0;
		end else begin
			if (i_row_start && i_valid)
				in_win <= 1'b1;
			else if (i_valid || flush_started) begin
				if (pix_cnt == i_img_w-1 && flush_started && flush_k == 3'd3)
					in_win <= 1'b0;
			end
		end
	end

	// ==================== 像素计数 ====================
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			pix_cnt <= 13'd0;
		end else begin
			if (i_row_start && i_valid && !in_win) begin
				pix_cnt <= 13'd0;
			end else if (valid_d || flush_started) begin
				if (pix_cnt == i_img_w-1)
					pix_cnt <= 13'd0;
				else
					pix_cnt <= pix_cnt + 13'd1;
			end
		end
	end

	// ==================== 行计数 ====================
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			row_cnt <= 13'd0;
		end else begin
			if (i_row_start && i_valid && !in_win) begin
				row_cnt <= 13'd0;
			end else if (valid_d || flush_started) begin
				if (pix_cnt == i_img_w-1)
					row_cnt <= row_cnt + 13'd1;
			end
		end
	end

	// ==================== 底部复制状态 ====================
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			flush_started <= 1'b0;
		end else begin
			// 新帧首个行首：复位行计数（N-12：上一帧底部冲刷完成后 in_win 已清 0，
			// 本分支才会生效；旧版底部冲刷结束后 in_win 恒 1，新帧永不复位，
			// row_cnt/r6/flush 状态跨帧残留 → 帧 1 首几行整体错位）
			if (i_row_start && i_valid && !in_win) begin
				flush_started <= 1'b0;
			end else if (valid_d && (row_cnt == i_img_h-1) && (pix_cnt == i_img_w-1) && frame_end_d) begin
				// 底部复制触发：末行最后一像素完成
				flush_started <= 1'b1;
			end else if ((valid_d || flush_started)
			             && (pix_cnt == i_img_w-1)
			             && flush_started
			             && (flush_k == 3'd3)) begin
				// 第 3 行底部复制完成
				flush_started <= 1'b0;
			end
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			flush_k <= 3'd0;
		end else begin
			if (i_row_start && i_valid && !in_win) begin
				flush_k <= 3'd0;
			end else if (valid_d && (row_cnt == i_img_h-1) && (pix_cnt == i_img_w-1) && frame_end_d) begin
				flush_k <= 3'd1;
			end else if ((valid_d || flush_started)
			             && (pix_cnt == i_img_w-1)
			             && flush_started
			             && (flush_k != 3'd3)) begin
				flush_k <= flush_k + 3'd1;
			end
		end
	end

	// ==================== 底部复制读路延迟 ====================
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			flush_started_q <= 1'b0;
			flush_k_rd      <= 3'd0;
		end else begin
			flush_started_q <= flush_started;
			flush_k_rd      <= flush_k;
		end
	end

	// N-11e（底部复制行末列修复）：底部冲刷行末拍沿，v6 读数据用的是
	// ==================== 行缓冲轮转计数 ====================
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			r6 <= 3'd0;
		end else begin
			if (i_row_start && i_valid && !in_win)
				r6 <= 3'd0;

			if (valid_d || flush_started) begin
				if (pix_cnt == i_img_w-1)
					r6 <= (r6 == 3'd5) ? 3'd0 : r6 + 3'd1;
			end
		end
	end

	// ==================== 6 个行缓冲：按行号 mod 6 轮转写入，端口 A 写 / 端口 B 读 ====================
	///< pix_cnt 延迟 1 拍（冲刷写地址，与复制读数据同滞后）
	


	// N-11 修复（TB 定位：每行末列 col=IMG_W-1 错误）：读出口 lb_read 比写
	//   晚 1 拍（RAM 读延迟），其数据由"上一拍的 r6/flush_k"时刻的轮转决定；
	//   而 r6/flush_k 在行末列拍沿即完成轮转，导致末列卷积拍（行末后一拍）
	//   的抽头缓冲选择用到了新轮转值（v1..v6 全部选错缓冲，末列整列错）。
	//   读路选择统一改用延迟 1 拍的 r6_d/flush_k_d，与 RAM 读出口严格同步；
	//   写路仍用即时 r6（写与轮转同拍，不受影响）。
	// N-11b：flush_k 读路不能用无条件延迟链 —— 行末拍沿 flush_k 与 r6 同时
	//   更新，无条件延迟会在底部复制行首拍取到已递增的新值；改为与 r6_d
	//   同条件同拍采样（见上方计数块 flush_k_rd）。
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			r6_d1 <= 3'd0;
			r6_d2 <= 3'd0;
			r6_d3 <= 3'd0;
			r6_d4 <= 3'd0;
		end
		else begin
			r6_d1 <= r6;
			r6_d2 <= r6_d1;
			r6_d3 <= r6_d2;
			r6_d4 <= r6_d3;
		end
	end

	// latch 延迟 1 拍，对齐读延迟
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin 
			cur_d1 <= 8'd0; 
			cur_d2 <= 'd0;
			cur_d3 <= 'd0;
			cur_d4 <= 'd0;
		end
		else begin
			cur_d1    <= i_data;
			cur_d2 <= cur_d1;
			cur_d3 <= cur_d2;
			cur_d4 <= cur_d3;
		end
	end


	////////////////////////////////////////////////////////////////////////////////////
	// latency 1: 延迟 1 拍，对齐读延迟
	////////////////////////////////////////////////////////////////////////////////////

	// 底部复制时末行缓冲号 = (i_img_h-1) mod 6 = (r6_d-flush_k_rd) mod 6（读路同拍）
	assign w_last_buf = (r6_d1 >= flush_k_rd) ? (r6_d1 - flush_k_rd) : (r6_d1 - flush_k_rd + 3'd6);
	// N-11f（冲刷行末列写序修复）：冲刷写通道每行从 col W-1 开始（首拍
	//   w_addr_d 采到前行末列）——会把 v6 尚需的真实旧行 col W-1 提前
	//   覆盖（v6 的 col W-1 读在本冲刷行末拍沿，晚覆盖 1 拍）。修法：
	//   ① 抑制所有冲刷首拍（pix_cnt==0）的写（消除早覆盖）；
	//   ② col W-1 复制推迟到下一冲刷行首拍写入：该拍 w_addr_d = W-1、
	//     w_dina = lb_read[w_last_buf] = 列 W-1 均天然就位，目标缓冲 =
	//     r6-1（上一冲刷行缓冲）；flush_k>=2 排除首个首拍（r6-1 = 末行
	//     缓冲自身，不可覆写复制源）。buf[R0+2] 永不被当复制读，无需补写。
	assign w_r6m1 = (r6 == 3'd0) ? 3'd5 : (r6 - 3'd1);
	assign w_flush_beat0 = flush_started && (pix_cnt == 13'd0);   // 冲刷行首拍
	
	

	// N-11g（帧首拍上边界复制写修复）：帧首拍（w_frame_first_beat）时沿之前
	//   row_cnt/r6 仍是上一帧冲刷后的残留值（如 IMG_H+3 / 5），复位要下一拍
	//   才生效 —— 旧版 (row_cnt==0)||(r6==g) 条件使该拍仅单个缓冲写入，
	//   buffers 0..4 的 col 0 本帧永不被写（后续首行写 addra≥1），残留上一帧
	//   垃圾 → 帧 1 起 rows 0..3 的 col 0 全错（TB 实证 4 错）。修法：帧首拍
	//   强制全部 6 个缓冲写 col 0（该拍 pix_cnt=0、dina=i_data 天然就位）。
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			w_frame_first_beat <= 1'b0;
		end else begin
			w_frame_first_beat <= (i_row_start && i_valid && !in_win);
		end
	end
	// 写地址：正常 = 当前列；底部复制 = 延迟 1 拍的列（w_addr_d，与复制写
	//   数据 lb_read[w_last_buf] 同源同滞后，列对齐）。
	// 写数据：正常 = 当前行数据；底部复制 = lb_read[w_last_buf]（末行缓冲，
	//   冲刷期间不被覆写）；冲刷行 k 把复制写进缓冲 (H mod 6)+k-1，
	//   自填充后续冲刷行钳位抽头（j<k）所需的缓冲。
	// N-11d：冲刷首拍（= 末行末列卷积拍）RAM 读延迟返回末行缓冲旧值，
	//   写数据必须仍取 cur_d（末行末列真实值），延迟 1 拍再切冲刷路
	//   （该拍写已被 N-11f ① 抑制，此选择仅保证 v0/数据路语义完备）。
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin 
			w_addr_d <= 13'd0; 
		end
		else begin
			w_addr_d <= pix_cnt;
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			w_addra <= 13'd0;
		end
		else if(flush_started) begin
			w_addra <= w_addr_d;
		end
		else begin
			w_addra <= pix_cnt;
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			w_dina <= 8'd0;
		end
		else if(flush_started) begin
			w_dina <= (flush_started_q ? lb_read[w_last_buf] : cur_d1);
		end
		else begin
			w_dina <= data_d;
		end
	end

	//产生写使能信号，用以区分正常行与冲刷行（底部复制）的缓冲写入。

	generate
		for (g = 0; g < 6; g = g + 1) begin : gen_lb
			// 写使能：第 0 行期间全部缓冲写第 0 行（上边界复制）；其余仅 r6 缓冲写。
			// N-11g：帧首拍复位尚未生效（row_cnt 为上一帧残留），须显式并入
			// w_frame_first_beat 强制 6 缓冲全写 col 0，否则首帧以外的帧
			// rows 0..3 的 col 0 读到上一帧垃圾。
			// N-11f ①：冲刷首拍写抑制（该拍自然写 = col W-1 早覆盖，已改由
			//   下一冲刷行首拍补写）；②：冲刷行首拍（flush_k>=2）补写上一
			//   冲刷行缓冲（r6-1）的 col W-1 复制。
			assign w_we_norm[g] = (valid_d || flush_started)
			                 && (w_frame_first_beat || (row_cnt == 13'd0) || (r6 == g[2:0]))
			                 && !(w_flush_beat0 && (r6 == g[2:0]));
			
			assign w_we_late[g] = w_flush_beat0 && (flush_k >= 3'd2) && (w_r6m1 == g[2:0]);
			
			//assign lb_we[g] = w_we_norm[g] || w_we_late[g];
			always @(posedge clk or negedge rst_n) begin
				if (!rst_n) begin
					lb_we[g] <= 1'b0;
				end
				else begin
					lb_we[g] <= w_we_norm[g] || w_we_late[g];
				end
			end

			xpm_memory_sdpram #(
				.MEMORY_SIZE        (8192*8),
				.MEMORY_PRIMITIVE   ("auto"),
				.WRITE_DATA_WIDTH_A (8),
				.READ_DATA_WIDTH_B  (8),
				.READ_LATENCY_B     (1),
//				.WRITE_MODE_A       ("no_change")
                .WRITE_MODE_B       ("no_change")
			) u_lb (
				.clka  (clk),
				.ena   (1'b1),
				.wea   (lb_we[g]),
				.addra (w_addra),
				.dina  (w_dina),
				
				.clkb  (clk),
				.enb   (1'b1),
				.addrb (pix_cnt),//pix_cnt
				.doutb (lb_read[g])
			);
		end
	endgenerate


	///////////////////////////////////////////////////////////////////////////////////////////

	//延迟 1 拍，用以冲刷行首拍（N-11f ①）补写前一冲刷行的缓冲（r6-1）的
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			pix_cnt_d1 <= 13'd0;
			pix_cnt_d2 <= 13'd0;
		end
		else begin
			pix_cnt_d1 <= pix_cnt;
			pix_cnt_d2 <= pix_cnt_d1;
		end
	end


	// ==================== 垂直抽头：v0 = 当前行，v1..v6 = 轮转读出的前 6 行 ====================
	// 读路选择（N-11/N-11c/N-11d/N-12 精确时序推导）：
	//   · v1..v5：恒用轮转选择器 w_sel_j = r6_d - j。冲刷行 k（处理行 H-1+k）
	//     的 v_j = 行 H-1+k-j：j≥k 为真实旧行（缓冲 (H-1+k-j) mod 6 原位）；
	//     j<k 需末行复制，对应缓冲 (H-1+k-j) mod 6 恰已被冲刷行 k-j 自填充
	//     为复制 —— 轮转选择器两种情形均正确；固定偏移选择器（旧 fsel）
	//     在冲刷行 2/3 的 j<k 抽头误选真实行 → 整行错误（TB 实证 741 错）。
	//   · v6：恒用 lb_read[r6_d] —— 正常行 = 行 P-6；冲刷行 k = 行 H-7+k
	//     （真实旧行，仍在缓冲中；该缓冲的本冲刷行写入滞后 1 列，读始终
	//     拿到旧值）——不可用末行复制（权重 2 对应的是最远真实行）。
	//   · v0：正常 = cur_d；冲刷（延迟 1 拍切换，N-11d）= 末行缓冲复制。
	//   · 帧首行 col 0：行首后一拍（col 0 卷积拍）各缓冲 mem[0] 尚无本帧
	//     数据（预写滞后），按上边界复制语义 v1..v6 旁路为 cur_d（= 行首像素）。
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			w_frame_first_d1 <= 1'b0;
			w_frame_first_d2 <= 1'b0;
		end
		else   begin  
			w_frame_first_d1 <= w_frame_first_beat;
			w_frame_first_d2 <= w_frame_first_d1;
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			w_sel1 <= 3'd0;
			w_sel2 <= 3'd0;
			w_sel3 <= 3'd0;
			w_sel4 <= 3'd0;
			w_sel5 <= 3'd0;
		end
		else begin
			w_sel1 <= (r6 >= 3'd1) ? (r6 - 3'd1) : (r6 + 3'd5);
			w_sel2 <= (r6 >= 3'd2) ? (r6 - 3'd2) : (r6 + 3'd4);
			w_sel3 <= (r6 >= 3'd3) ? (r6 - 3'd3) : (r6 + 3'd3);
			w_sel4 <= (r6 >= 3'd4) ? (r6 - 3'd4) : (r6 + 3'd2);
			w_sel5 <= (r6 >= 3'd5) ? (r6 - 3'd5) : (r6 + 3'd1);
		end
	end



	
	assign w_t1 = lb_read[w_sel1];
	assign w_t2 = lb_read[w_sel2];
	assign w_t3 = lb_read[w_sel3];
	assign w_t4 = lb_read[w_sel4];
	assign w_t5 = lb_read[w_sel5];
	assign w_t6 = lb_read[r6_d1];
	


	assign v0 = flush_started_q ? lb_read[w_last_buf] : cur_d2;
	assign v1 = w_frame_first_d2 ? cur_d4 : w_t1;
	assign v2 = w_frame_first_d2 ? cur_d4 : w_t2;
	assign v3 = w_frame_first_d2 ? cur_d4 : w_t3;
	assign v4 = w_frame_first_d2 ? cur_d4 : w_t4;
	assign v5 = w_frame_first_d2 ? cur_d4 : w_t5;
	assign v6 = w_frame_first_d2 ? cur_d4 : w_t6;

	// ==================== 加法树：[2,7,14,18,14,7,2]/64（OpenCV 口径） ====================
	// 2=2, 7=8-1, 14=16-2, 18=16+2，全部移位实现，0 DSP
	assign w_t06 = v0 + v6;
	assign w_t15 = v1 + v5;
	assign w_t24 = v2 + v4;

	assign w_2t  = w_t06 << 1;                            // *2
	assign w_7t  = (w_t15 << 3) - w_t15;                  // *7
	assign w_14t = (w_t24 << 4) - (w_t24 << 1);           // *14
	assign w_18t = (v3 << 4) + (v3 << 1);                 // *18

	assign w_sum = w_2t + w_7t + w_14t + w_18t;           // 14bit, 最大 16320

	// N-10 修复（TB 逐拍探针定位，两个叠加缺陷）：
	//   ① 对齐：数据路径 cur_d/RAM读(1拍)→w_sum→sum(1拍) 共 2 拍，旧版
	//     vld_r/row_cnt_r 仅 1 拍，valid 比 sum 早 1 拍，每个输出拿到错误
	//     时刻的 sum（行首拍叠加行号窗错位：输出行 0 首列丢失、全行移位）。
	//     补 vld_e/row_cnt_e 至 2 拍与 sum 严格同拍。
	//   ② 行首垃圾拍：N-8b 硬件强制行间间隙后，行首拍前存在空闲拍，
	//     cur_d 保持上一行末列旧值并被行首拍采样，产生 1 个垃圾卷积；
	//     把 pix_cnt 随同链延迟，仅发射 pix_cnt_d2 < IMG_W 的拍（垃圾拍
	//     相位 pix_cnt_d2 = IMG_W-1 …… 行首拍 pix_cnt 已回绕为 0，d1 拍到
	//     W-1，d2 拍恰好 = IMG_W-1 被丢弃）。
	// 生成 行计数延时
	// 生成 vld_d1, vld_d2, vld_d3, vld_d4
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			vld_d1 <= 1'b0; vld_d2 <= 1'b0; vld_d3 <= 1'b0; vld_d4 <= 1'b0;
		end else begin
			vld_d1 <= (valid_d || flush_started);
			vld_d2 <= vld_d1;
			vld_d3 <= vld_d2;
			vld_d4 <= vld_d3;
		end
	end

	// 生成 pix_cnt_d1, pix_cnt_d2, pix_cnt_d3, pix_cnt_d4
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			pix_cnt_d3 <= 13'd0; pix_cnt_d4 <= 13'd0;
		end else begin
			pix_cnt_d3 <= pix_cnt_d2;
			pix_cnt_d4 <= pix_cnt_d3;
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			row_cnt_d1 <= 13'd0; row_cnt_d2 <= 13'd0;
			row_cnt_d3 <= 13'd0; row_cnt_d4 <= 13'd0;
		end else begin
			row_cnt_d1 <= row_cnt;
			row_cnt_d2 <= row_cnt_d1;
			row_cnt_d3 <= row_cnt_d2;
			row_cnt_d4 <= row_cnt_d3;
		end
	end

	//
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			sum <= 14'd0; 

		end else begin
			sum        <= w_sum;
		end
	end


	// 输出有效：处理行 3..IMG_H+2 对应输出行 0..IMG_H-1（因果 → 中心滞后 3 行）；
	// vld_e/row_cnt_e/pix_cnt_d2 与 sum 严格同拍（2 拍链），列窗丢弃行首垃圾拍
	assign o_valid = vld_d2 && (row_cnt_d2 >= 13'd3) && (row_cnt_d2 < i_img_h + 13'd3)
	                          && (pix_cnt_d2 < i_img_w);
	assign o_data  = (sum + 14'd32) >> 6;


endmodule
