// ============================================================================
// @file      sbm_accum_lane.v
// @brief     单通道分块累加引擎（双 BRAM 乒乓读改写 RMW）
// @details
//   通道 l 负责位置区间 [l*CHUNK, l*CHUNK+CHUNK) ∩ [0, template_positions)。
//
//   双 BRAM（偶/奇位置分 bank）读改写（RMW）语义：
//     - 偶数位置 p = 2m 存于 bank_e 地址 m；
//       奇数位置 p = 2m+1 存于 bank_o 地址 m。
//     - XPM SPRAm 读延迟 1 拍（read_first）：本拍地址读出的旧值于下拍作 dout，
//       同拍写回 dout + 本拍消费的字节。
//
//   关键修正（原版"奇数位置差一"致命 bug）：
//     - 读地址 w_addr_r = j_local>>1（预取下一位置）；
//     - 写地址 w_addr_w = (j_local-1)>>1（当前位置 cell）。
//     二者在相邻两拍数值恒相等，但读与写恒落于不同 bank。
//
//   F5a 参数可维护性修正(2026-08-14):
//     - 全部位宽由 CHUNK/BANK_DEPTH/WC/HC/HIT_FIFO_DEPTH 派生, 不再写死
//       12bit(原版 cnt_l/j_local/s_q 12bit 在 CHUNK>4096 时溢出);
//     - tp 端口位宽 TPW 由顶层派生值决定(原版 12bit 装不下 TP≈9.7万,
//       除 lane0 外所有通道 cnt_l=0 永不工作);
//     - 命中FIFO指针位宽(HFW=log2(DEPTH))与计数位宽(HFCW=log2(DEPTH+1))
//       分离: 原版指针用计数位宽 7bit 索引深度 64 数组, 越过 63 后返回 x;
//     - scan_done 改为"末位置 ev_pos==cnt_l-1 完成判定拍"置位: 原版在末位置
//       地址呈递拍就置位, 与末位置命中压 FIFO 同拍竞争, 顶层可能在命中尚未
//       入 FIFO 时提前退出而丢末位置候选。
// @author    WorkBuddy
// @date      2026-08-14
// ============================================================================
`include "sbm_geometry.vh"
module sbm_accum_lane #(
	parameter CHUNK      = 4096,       ///< 本通道位置数(由顶层按几何派生)
	parameter BANK_DEPTH = 2048,       ///< ceil(CHUNK/2) 向上取 2 的幂(顶层派生)
	parameter WC         = 320,        ///< 线性内存每行单元数(=IMG_W/T, 与 alg9/alg11 一致)
	parameter HC         = 320,        ///< 线性内存行数(=IMG_H/T, 顶层派生, 供坐标宽度)
	parameter TPW        = 17,         ///< tp 端口位宽(顶层按 TP_MAX 派生)
	parameter HIT_FIFO_DEPTH = 64,     ///< 命中FIFO深度(F6: 与顶层并发drain配合防溢出)
	parameter LB         = 0,          ///< l*CHUNK，本通道起始位置
	parameter X0         = 0,          ///< LB % WC（编译期常量）
	parameter Y0         = 0,          ///< LB / WC（编译期常量）
	// ---- 位宽派生参数(默认由几何参数自动计算, 一般无需覆盖) ----
	parameter AW  = (BANK_DEPTH < 2) ? 1 : $clog2(BANK_DEPTH),           ///< bank地址位宽(0..BANK_DEPTH-1)
	parameter PW  = ((CHUNK + 2) < 2) ? 1 : $clog2(CHUNK + 2),             ///< 位置指针位宽
	parameter CW  = ((WC + 1) < 2) ? 1 : $clog2(WC + 1),                   ///< 列坐标位宽
	parameter YW  = ((HC + 1) < 2) ? 1 : $clog2(HC + 1),                   ///< 行坐标位宽
	parameter HFW = (HIT_FIFO_DEPTH < 2) ? 1 : $clog2(HIT_FIFO_DEPTH),     ///< 命中FIFO指针位宽
	parameter HFCW= ((HIT_FIFO_DEPTH + 1) < 2) ? 1 : $clog2(HIT_FIFO_DEPTH + 1) ///< 命中FIFO计数位宽
)(
	input  wire         clk,
	input  wire         rst_n,
	// ---- 特征装载 ----
	input  wire         init,          ///< 特征开始脉冲：锁存 base 与 tp
	input  wire [31:0]  base_addr,     ///< 当前特征基址（含方向偏移，见顶层）
	input  wire [TPW-1:0] tp,          ///< template_positions
	input  wire [15:0]  thresh,        ///< 原始分阈值（0~252，顶层预计算）
	// ---- 清零 ----
	input  wire         zero_en,
	input  wire [AW-1:0] zero_addr,
	// ---- 累加运行 ----
	input  wire         run_en,
	output wire         lane_done,
	// ---- 读请求（接顶层仲裁） ----
	output wire         need_req,
	output wire [31:0]  req_addr,
	output wire         next_slot,     ///< 本通道下一填充槽号（仲裁捕获）
	input  wire         grant,         ///< 仲裁授予一拍
	input  wire         fill_slot,
	input  wire         fill_vld,      ///< R 通道回填
	input  wire [255:0] fill_data,
	// ---- 候选扫描（通道自扫） ----
	input  wire         scan_en,
	output wire         hit_vld,
	output wire [31:0]  hit_dout,      ///< {score[7:0], y[11:0], x[11:0]}
	input  wire         hit_pop,
	output wire         hit_empty,
	output wire [7:0]   lane_max,
	output wire         scan_stall,    ///< F6: 命中FIFO近满时暂停本lane扫描(背压)
	output wire         scan_done      ///< F6: 本lane扫描完所有位置(供顶层判ST_SCAN结束)
);

	// ==================== 位宽与几何防御性断言 ====================
	generate
		if (YW > 12 || CW > 12)
			$error("lane: 坐标位宽须<=12(hit_dout 打包限制), YW=%0d CW=%0d", YW, CW);
		if (CHUNK > BANK_DEPTH*2)
			$error("lane: BANK_DEPTH(%0d) < ceil(CHUNK/2)(%0d), 位置将溢出bank", BANK_DEPTH, CHUNK);
		if (HIT_FIFO_DEPTH < 8)
			$error("lane: HIT_FIFO_DEPTH(%0d) 须 >= 8", HIT_FIFO_DEPTH);
	endgenerate

	// ==================== 通道控制量 ====================
	reg  [PW-1:0] cnt_l;                 ///< 本通道有效位置数
	reg  [PW-1:0] j_local;               ///< 累加位置指针
	reg  [7:0]  byte_d;                ///< 上一拍消费的字节
	reg  [31:0] rd_addr;               ///< 下一请求地址（32B 对齐）
	reg  [31:0] start_addr;            ///< 本特征请求起始地址
	reg         next_slot_r;

	wire [PW-1:0] w_cnt = (tp > LB) ? ((tp - LB > CHUNK) ? CHUNK : (tp - LB)) : {PW{1'b0}};

	// ==================== 双缓冲 / 消费控制 ====================
	reg  [7:0]  rbuf [0:1][0:31];
	reg         valid [0:1];
	reg         cur_sel;
	reg  [4:0]  byte_ptr;
	reg         first_fill;

	/// base_addr + LB 的低 5 位，作为首拍跳过的无效字节数
	wire [31:0] w_base_lb = base_addr + LB;
	wire [4:0]  w_skip    = w_base_lb[4:0];

	wire        slot_ok  = valid[cur_sel];
	wire        consume  = run_en && (j_local < cnt_l) && slot_ok;   // 正常消费拍
	wire        drain    = run_en && (j_local == cnt_l);             // 收尾写拍
	wire [7:0]  cur_byte = rbuf[cur_sel][byte_ptr];

	integer     bi;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			valid[0] <= 1'b0; valid[1] <= 1'b0;
			cur_sel  <= 1'b0; byte_ptr <= 5'd0; first_fill <= 1'b1;
		end else if (init) begin
			valid[0] <= 1'b0; valid[1] <= 1'b0;
			cur_sel  <= 1'b0; byte_ptr <= w_skip; first_fill <= 1'b1;
		end else begin
			if (fill_vld) begin
				for (bi = 0; bi < 32; bi = bi + 1)
					rbuf[fill_slot][bi] <= fill_data[bi*8 +: 8];
				valid[fill_slot] <= 1'b1;
				if (first_fill) byte_ptr <= w_skip;
				else            byte_ptr <= 5'd0;
				first_fill <= 1'b0;
			end
			if (consume) begin
				if (byte_ptr == 5'd31) begin
					byte_ptr <= 5'd0;
					cur_sel  <= ~cur_sel;
					valid[cur_sel] <= 1'b0;
				end else
					byte_ptr <= byte_ptr + 5'd1;
			end
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			cnt_l    <= {PW{1'b0}};
			j_local  <= {PW{1'b0}};
			byte_d   <= 8'd0;
			rd_addr  <= 32'd0;
			start_addr <= 32'd0;
			next_slot_r <= 1'b0;
		end else if (init) begin
			cnt_l    <= w_cnt;
			j_local  <= {PW{1'b0}};
			byte_d   <= 8'd0;
			rd_addr  <= (base_addr + LB) & ~32'h1F;
			start_addr <= (base_addr + LB) & ~32'h1F;
			next_slot_r <= 1'b0;
		end else if (grant) begin
			rd_addr    <= rd_addr + 32'd32;
			next_slot_r <= ~next_slot_r;
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)       j_local <= {PW{1'b0}};
		else if (init)    j_local <= {PW{1'b0}};
		else if (consume || drain) j_local <= j_local + {{(PW-1){1'b0}},1'b1};
	end

	assign lane_done = (cnt_l == {PW{1'b0}}) || (j_local == cnt_l + {{(PW-1){1'b0}},1'b1});

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)       byte_d <= 8'd0;
		else if (init)    byte_d <= 8'd0;
		else if (consume) byte_d <= cur_byte;
	end

	// ==================== 候选扫描：1 拍读延迟的流水线对齐（F1 修正） ====================
	// 本拍向 RAM 呈递位置 s_q 的地址，下一拍 dout 即 score[s_q]；本拍以
	// ev_pos = 上一拍呈递的 s_q 作判定，坐标 (ev_x,ev_y) 与 s_q 同步推进。
	reg  [PW-1:0] s_q;        ///< 本拍取分位置（地址已呈递）
	reg  [CW-1:0] qx;         ///< s_q 的全局列坐标
	reg  [YW-1:0] qy;         ///< s_q 的全局行坐标
	reg  [PW-1:0] ev_pos;     ///< 本拍判定位置 = 上一拍呈递的 s_q
	reg  [CW-1:0] ev_x;
	reg  [YW-1:0] ev_y;
	reg  [7:0]  ev_score;
	reg         ev_vld;
	reg  [7:0]  max_r;
	reg         scan_done_r;   // 声明前置: 扫描推进条件提前引用

	// ==================== 双 bank 读改写地址 ====================
	wire [AW-1:0] w_addr_r = j_local >> 1;
	wire [AW-1:0] w_addr_w = (j_local - {{(PW-1){1'b0}},1'b1}) >> 1;

	wire [AW-1:0] addra_e =
		zero_en  ? zero_addr :
		(scan_en ? s_q[PW-1:1] : (j_local[0] ? w_addr_w : w_addr_r));
	wire [AW-1:0] addra_o =
		zero_en  ? zero_addr :
		(scan_en ? s_q[PW-1:1] : (j_local[0] ? w_addr_r : w_addr_w));

	wire [7:0]  dout_e, dout_o;
	wire w_we_e = zero_en || (((consume || drain) && j_local[0] == 1'b1));
	wire w_we_o = zero_en || (((consume || drain) && j_local[0] == 1'b0 && j_local > {PW{1'b0}}));
	wire [7:0]  w_din_e = (zero_en) ? 8'd0 : (dout_e + byte_d);
	wire [7:0]  w_din_o = (zero_en) ? 8'd0 : (dout_o + byte_d);

	xpm_memory_spram #(
		.MEMORY_SIZE(BANK_DEPTH*8), .MEMORY_PRIMITIVE("block"),
		.WRITE_DATA_WIDTH_A(8), .READ_DATA_WIDTH_A(8),
		.READ_LATENCY_A(1), .WRITE_MODE_A("read_first"),
		.READ_RESET_VALUE_A("0"), .SIM_ASSERT_CHK(0))
	u_bank_e (
		.clka(clk), .rsta(~rst_n), .ena(1'b1), .regcea(1'b1),
		.wea(w_we_e), .addra(addra_e), .dina(w_din_e), .douta(dout_e),
		.injectsbiterra(1'b0), .injectdbiterra(1'b0), .sleep(1'b0));

	xpm_memory_spram #(
		.MEMORY_SIZE(BANK_DEPTH*8), .MEMORY_PRIMITIVE("block"),
		.WRITE_DATA_WIDTH_A(8), .READ_DATA_WIDTH_A(8),
		.READ_LATENCY_A(1), .WRITE_MODE_A("read_first"),
		.READ_RESET_VALUE_A("0"), .SIM_ASSERT_CHK(0))
	u_bank_o (
		.clka(clk), .rsta(~rst_n), .ena(1'b1), .regcea(1'b1),
		.wea(w_we_o), .addra(addra_o), .dina(w_din_o), .douta(dout_o),
		.injectsbiterra(1'b0), .injectdbiterra(1'b0), .sleep(1'b0));

	// F5a: need_req 加 run_en 门控 —— 特征边界(ST_FEAT/ST_FEAT2 拍)时残留的
	// 旧请求不得发出, 否则其 fill 会污染下一特征已 init 清零的双缓冲(valid 槽
	// 与 cur_sel 错位 -> 死锁)。累加运行(ST_RUN)期间才允许发起读请求。
	assign need_req  = run_en && (cnt_l != {PW{1'b0}})
		&& ((valid[0] == 1'b0) || (valid[1] == 1'b0))
		&& ((rd_addr - start_addr) < cnt_l + 32'd32);
	assign req_addr  = rd_addr;
	assign next_slot = next_slot_r;

	// score[ev_pos]：bank 选择用 ev_pos[0]，与累加存储口径（偶→bank_e / 奇→bank_o）一致
	wire [7:0]  w_score = ev_pos[0] ? dout_o : dout_e;
	wire        w_hit   = ev_vld && (ev_pos < cnt_l) && (w_score > thresh);

	// ==================== 候选扫描：通道自扫，1 位置/拍 ====================
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			s_q <= {PW{1'b0}}; qx <= X0[CW-1:0]; qy <= Y0[YW-1:0];
			ev_pos <= {PW{1'b0}}; ev_x <= X0[CW-1:0]; ev_y <= Y0[YW-1:0]; ev_vld <= 1'b0; max_r <= 8'd0;
		end else if (init) begin
			s_q <= {PW{1'b0}}; qx <= X0[CW-1:0]; qy <= Y0[YW-1:0];
			ev_pos <= {PW{1'b0}}; ev_x <= X0[CW-1:0]; ev_y <= Y0[YW-1:0]; ev_vld <= 1'b0; max_r <= 8'd0;
		end else if (scan_en && !scan_stall && !scan_done_r) begin
			ev_pos <= s_q;
			ev_x   <= qx;
			ev_y   <= qy;
			ev_score <= w_score;
			ev_vld <= 1'b1;
			if (qx == WC-1) begin qx <= {CW{1'b0}}; qy <= qy + {{(YW-1){1'b0}},1'b1}; end
			else            qx <= qx + {{(CW-1){1'b0}},1'b1};
			s_q <= s_q + {{(PW-1){1'b0}},1'b1};
		end else begin
			ev_vld <= 1'b0;
		end
	end

	// 通道最大值
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)    max_r <= 8'd0;
		else if (init) max_r <= 8'd0;
		else if (ev_vld && ev_pos < cnt_l) begin
			if (w_score > max_r)
				max_r <= w_score;
		end
	end

	// ==================== 命中 FIFO（F6: 参数化深度 + 背压 + 并发读写不丢） ====================
	// F5a: 指针位宽 HFW 与计数位宽 HFCW 分离, 指针按 2^HFW 自然回绕 -> 不再越界。
	reg  [31:0] hit_fifo [0:HIT_FIFO_DEPTH-1];
	reg  [HFW-1:0] hit_wr, hit_rd;
	reg  [HFCW-1:0] hcnt;
	reg         hit_ovf;
	wire        w_scan_stall = (hcnt >= (HIT_FIFO_DEPTH - 2));
	assign      scan_done = (cnt_l == {PW{1'b0}}) || scan_done_r;
	wire w_pop_e = hit_pop && (hcnt != {HFCW{1'b0}});
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			hit_wr <= {HFW{1'b0}}; hit_rd <= {HFW{1'b0}}; hcnt <= {HFCW{1'b0}}; hit_ovf <= 1'b0;
			scan_done_r <= 1'b0;
		end else if (init) begin
			hit_wr <= {HFW{1'b0}}; hit_rd <= {HFW{1'b0}}; hcnt <= {HFCW{1'b0}}; hit_ovf <= 1'b0;
			scan_done_r <= 1'b0;
		end else begin
			if (w_hit && !scan_stall) begin
				hit_fifo[hit_wr] <= {w_score, {{(12-YW){1'b0}}, ev_y}, {{(12-CW){1'b0}}, ev_x}};
				hit_wr <= hit_wr + {{(HFW-1){1'b0}}, 1'b1};
			end
			if (w_pop_e) hit_rd <= hit_rd + {{(HFW-1){1'b0}}, 1'b1};
			hcnt <= hcnt + (w_hit && !scan_stall ? 1'b1 : 1'b0)
			              - (w_pop_e ? 1'b1 : 1'b0);
			if (w_hit && scan_stall && (hcnt >= (HIT_FIFO_DEPTH-1))) begin
				hit_ovf <= 1'b1;
				$error("F6: hit FIFO overflow (HIT_FIFO_DEPTH too small or drain stalled)");
			end
			// F5a: 末位置判定完成拍置位(与末位置命中压FIFO同拍, hcnt 同步更新,
			// 顶层以 (&hitempty) 兜底, 不会在命中入FIFO前提前退出)
			if (ev_vld && (ev_pos == cnt_l - {{(PW-1){1'b0}},1'b1}) && (cnt_l != {PW{1'b0}}))
				scan_done_r <= 1'b1;
		end
	end

	assign hit_vld   = (hcnt != {HFCW{1'b0}});
	assign hit_dout  = hit_fifo[hit_rd];
	assign hit_empty = (hcnt == {HFCW{1'b0}});
	assign lane_max  = max_r;
	assign scan_stall = w_scan_stall;

endmodule
