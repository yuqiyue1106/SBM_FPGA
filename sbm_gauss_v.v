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
	reg         flush_started_q;     ///< 底部复制标记延迟 1 拍（N-11d：数据路切换用。
	                                 ///<  末行末列的卷积拍与冲刷首拍重合：该拍
	                                 ///<  flush_started 刚置位，但 v0/复制写数据仍须
	                                 ///<  取末行真实数据 cur_d，延迟 1 拍再切冲刷路）
	reg  [2:0]  flush_k;             ///< 底部复制行序号 1..3
	reg  [2:0]  flush_k_rd;          ///< 读路 flush_k（N-11b：随 r6_d 同拍采样，
	                                 ///<  不能用无条件延迟链 —— 行末拍沿 flush_k
	                                 ///<  已递增，无条件延迟会取到新值）
	reg         in_win;              ///< 帧内有效窗（含底部复制）

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			pix_cnt <= 13'd0; row_cnt <= 13'd0; r6 <= 3'd0;
			flush_started <= 1'b0; flush_started_q <= 1'b0; flush_k <= 3'd0; flush_k_rd <= 3'd0; in_win <= 1'b0;
		end else begin
			flush_started_q <= flush_started;
			flush_k_rd <= flush_k;     // 与 r6_d 同拍（同条件更新）
			// 新帧首个行首：复位行计数（N-12：上一帧底部冲刷完成后 in_win 已清 0，
			// 本分支才会生效；旧版底部冲刷结束后 in_win 恒 1，新帧永不复位，
			// row_cnt/r6/flush 状态跨帧残留 → 帧 1 首几行整体错位）
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
							in_win  <= 1'b0;   // 底部冲刷排空，允许下一帧复位（N-12）
						end else
							flush_k <= flush_k + 3'd1;
					end
				end else
					pix_cnt <= pix_cnt + 13'd1;
			end
		end
	end

	// ==================== 6 个行缓冲：按行号 mod 6 轮转写入，端口 A 写 / 端口 B 读 ====================
	reg  [12:0] w_addr_d;             ///< pix_cnt 延迟 1 拍（冲刷写地址，与复制读数据同滞后）
	reg  [7:0]  cur_d;                ///< 当前行数据延迟 1 拍（对齐读延迟）
	// N-11 修复（TB 定位：每行末列 col=IMG_W-1 错误）：读出口 lb_read 比写
	//   晚 1 拍（RAM 读延迟），其数据由"上一拍的 r6/flush_k"时刻的轮转决定；
	//   而 r6/flush_k 在行末列拍沿即完成轮转，导致末列卷积拍（行末后一拍）
	//   的抽头缓冲选择用到了新轮转值（v1..v6 全部选错缓冲，末列整列错）。
	//   读路选择统一改用延迟 1 拍的 r6_d/flush_k_d，与 RAM 读出口严格同步；
	//   写路仍用即时 r6（写与轮转同拍，不受影响）。
	reg  [2:0]  r6_d;                 ///< r6 延迟 1 拍（读路轮转选择）
	// N-11b：flush_k 读路不能用无条件延迟链 —— 行末拍沿 flush_k 与 r6 同时
	//   更新，无条件延迟会在底部复制行首拍取到已递增的新值；改为与 r6_d
	//   同条件同拍采样（见上方计数块 flush_k_rd）。
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) r6_d <= 3'd0;
		else        r6_d <= r6;
	end
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin w_addr_d <= 13'd0; cur_d <= 8'd0; end
		else begin
			w_addr_d <= pix_cnt;
			cur_d    <= i_data;
		end
	end
	// 底部复制时末行缓冲号 = (IMG_H-1) mod 6 = (r6_d-flush_k_rd) mod 6（读路同拍）
	wire [2:0] w_last_buf = (r6_d >= flush_k_rd) ? (r6_d - flush_k_rd) : (r6_d - flush_k_rd + 3'd6);
	// N-11f（冲刷行末列写序修复）：冲刷写通道每行从 col W-1 开始（首拍
	//   w_addr_d 采到前行末列）——会把 v6 尚需的真实旧行 col W-1 提前
	//   覆盖（v6 的 col W-1 读在本冲刷行末拍沿，晚覆盖 1 拍）。修法：
	//   ① 抑制所有冲刷首拍（pix_cnt==0）的写（消除早覆盖）；
	//   ② col W-1 复制推迟到下一冲刷行首拍写入：该拍 w_addr_d = W-1、
	//     w_dina = lb_read[w_last_buf] = 列 W-1 均天然就位，目标缓冲 =
	//     r6-1（上一冲刷行缓冲）；flush_k>=2 排除首个首拍（r6-1 = 末行
	//     缓冲自身，不可覆写复制源）。buf[R0+2] 永不被当复制读，无需补写。
	wire [2:0] w_r6m1     = (r6 == 3'd0) ? 3'd5 : (r6 - 3'd1);
	wire       w_flush_beat0 = flush_started && (pix_cnt == 13'd0);   // 冲刷行首拍
	wire [7:0] lb_read [0:5];
	wire       lb_we   [0:5];
	// N-11g（帧首拍上边界复制写修复）：帧首拍（w_frame_first_beat）时沿之前
	//   row_cnt/r6 仍是上一帧冲刷后的残留值（如 IMG_H+3 / 5），复位要下一拍
	//   才生效 —— 旧版 (row_cnt==0)||(r6==g) 条件使该拍仅单个缓冲写入，
	//   buffers 0..4 的 col 0 本帧永不被写（后续首行写 addra≥1），残留上一帧
	//   垃圾 → 帧 1 起 rows 0..3 的 col 0 全错（TB 实证 4 错）。修法：帧首拍
	//   强制全部 6 个缓冲写 col 0（该拍 pix_cnt=0、dina=i_data 天然就位）。
	wire        w_frame_first_beat = (i_row_start && i_valid && !in_win);
	// 写地址：正常 = 当前列；底部复制 = 延迟 1 拍的列（w_addr_d，与复制写
	//   数据 lb_read[w_last_buf] 同源同滞后，列对齐）。
	// 写数据：正常 = 当前行数据；底部复制 = lb_read[w_last_buf]（末行缓冲，
	//   冲刷期间不被覆写）；冲刷行 k 把复制写进缓冲 (H mod 6)+k-1，
	//   自填充后续冲刷行钳位抽头（j<k）所需的缓冲。
	// N-11d：冲刷首拍（= 末行末列卷积拍）RAM 读延迟返回末行缓冲旧值，
	//   写数据必须仍取 cur_d（末行末列真实值），延迟 1 拍再切冲刷路
	//   （该拍写已被 N-11f ① 抑制，此选择仅保证 v0/数据路语义完备）。
	wire [12:0] w_addra = flush_started ? w_addr_d : pix_cnt;
	wire [7:0]  w_dina  = flush_started ? (flush_started_q ? lb_read[w_last_buf] : cur_d) : i_data;

	genvar g;
	generate
		for (g = 0; g < 6; g = g + 1) begin : gen_lb
			// 写使能：第 0 行期间全部缓冲写第 0 行（上边界复制）；其余仅 r6 缓冲写。
			// N-11g：帧首拍复位尚未生效（row_cnt 为上一帧残留），须显式并入
			// w_frame_first_beat 强制 6 缓冲全写 col 0，否则首帧以外的帧
			// rows 0..3 的 col 0 读到上一帧垃圾。
			// N-11f ①：冲刷首拍写抑制（该拍自然写 = col W-1 早覆盖，已改由
			//   下一冲刷行首拍补写）；②：冲刷行首拍（flush_k>=2）补写上一
			//   冲刷行缓冲（r6-1）的 col W-1 复制。
			wire w_we_norm = (i_valid || flush_started)
			                 && (w_frame_first_beat || (row_cnt == 13'd0) || (r6 == g[2:0]))
			                 && !(w_flush_beat0 && (r6 == g[2:0]));
			wire w_we_late = w_flush_beat0 && (flush_k >= 3'd2) && (w_r6m1 == g[2:0]);
			assign lb_we[g] = w_we_norm || w_we_late;
			xpm_memory_sdpram #(
				.MEMORY_SIZE        (IMG_W*8),
				.MEMORY_PRIMITIVE   ("auto"),
				.WRITE_DATA_WIDTH_A (8),
				.READ_DATA_WIDTH_B  (8),
				.READ_LATENCY_B     (1),
				// N-28 修复（ModelSim/Vivado 真实 XPM 暴露）：6 个行缓冲每拍对
				//   同一列地址"又写又读"（写当前行 c，读 6 行前 c）。此冲突下
				//   必须返回"写前的旧值"（即上一行像素），故须 read_first。
				//   原"no_change"在真实原语中会冻结读数，导致垂直抽头错位、
				//   DUT 与 Golden 对不上、tdata 时序错乱；OSS 简化行为模型曾
				//   忽略该参数而误判通过。
				.WRITE_MODE_A       ("read_first")
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
	reg         w_frame_first_d1;    ///< 帧首拍延迟 1 拍（= col 0 卷积拍）
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) w_frame_first_d1 <= 1'b0;
		else        w_frame_first_d1 <= w_frame_first_beat;
	end
	wire [2:0]  w_sel1 = (r6_d >= 3'd1) ? (r6_d - 3'd1) : (r6_d + 3'd5);
	wire [2:0]  w_sel2 = (r6_d >= 3'd2) ? (r6_d - 3'd2) : (r6_d + 3'd4);
	wire [2:0]  w_sel3 = (r6_d >= 3'd3) ? (r6_d - 3'd3) : (r6_d + 3'd3);
	wire [2:0]  w_sel4 = (r6_d >= 3'd4) ? (r6_d - 3'd4) : (r6_d + 3'd2);
	wire [2:0]  w_sel5 = (r6_d >= 3'd5) ? (r6_d - 3'd5) : (r6_d + 3'd1);
	wire [7:0]  w_t1 = lb_read[w_sel1];
	wire [7:0]  w_t2 = lb_read[w_sel2];
	wire [7:0]  w_t3 = lb_read[w_sel3];
	wire [7:0]  w_t4 = lb_read[w_sel4];
	wire [7:0]  w_t5 = lb_read[w_sel5];
	wire [7:0]  w_t6 = lb_read[r6_d];
	wire [7:0] v0 = flush_started_q ? lb_read[w_last_buf] : cur_d;
	wire [7:0] v1 = w_frame_first_d1 ? cur_d : w_t1;
	wire [7:0] v2 = w_frame_first_d1 ? cur_d : w_t2;
	wire [7:0] v3 = w_frame_first_d1 ? cur_d : w_t3;
	wire [7:0] v4 = w_frame_first_d1 ? cur_d : w_t4;
	wire [7:0] v5 = w_frame_first_d1 ? cur_d : w_t5;
	wire [7:0] v6 = w_frame_first_d1 ? cur_d : w_t6;

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
	reg  [12:0] row_cnt_r;             ///< 行号延迟 1 拍（行内与数据拍同行）
	reg         vld_r;
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
	reg         vld_e;
	reg  [12:0] row_cnt_e;
	reg  [12:0] pix_cnt_d1, pix_cnt_d2;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			sum <= 14'd0; row_cnt_r <= 13'd0; vld_r <= 1'b0;
			vld_e <= 1'b0; row_cnt_e <= 13'd0;
			pix_cnt_d1 <= 13'd0; pix_cnt_d2 <= 13'd0;
		end else begin
			sum        <= w_sum;
			row_cnt_r  <= row_cnt;
			vld_r      <= (i_valid || flush_started);
			vld_e      <= vld_r;
			row_cnt_e  <= row_cnt_r;
			pix_cnt_d1 <= pix_cnt;
			pix_cnt_d2 <= pix_cnt_d1;
		end
	end

	// 输出有效：处理行 3..IMG_H+2 对应输出行 0..IMG_H-1（因果 → 中心滞后 3 行）；
	// vld_e/row_cnt_e/pix_cnt_d2 与 sum 严格同拍（2 拍链），列窗丢弃行首垃圾拍
	assign o_valid = vld_e && (row_cnt_e >= 13'd3) && (row_cnt_e < IMG_H + 13'd3)
	                          && (pix_cnt_d2 < IMG_W);
	assign o_data  = (sum + 14'd32) >> 6;

endmodule
