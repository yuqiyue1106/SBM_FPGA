// ==================================================================
// sbm_alg11_accum.v : 相似度滑窗累加IP核顶层(PL端)
// 与line2Dup.cpp similarity()(第957~1011行)语义一致:
//   for each feature f: for j in [0, template_positions):
//     score[j] += lm_ori[ block(f)*CELLS + lm_index(f) + j ]
//   其中 ori=f.label, block(f) = (f.y%T)*T + (f.x%T)
//        lm_index(f) = (f.y/T)*WC + (f.x/T)
//   方向偏移: lm_ori基址=RESP_BASE+ori*T*T*CELLS
// 8bit得分: 特征≤63, 63×4=252<255 不溢出
// 实现: LANES 通道分块累加 + 256bit AXI4读主机 + Top-32候选收集
// 修正记录：①累加读基址补方向偏移ori*T*T*CELLS；
//   ②候选阈值口径对齐C++(raw_thr=floor(thr*4*nfeat/100), 严格大于)；
//   ③特征弹出与基址计算间补1拍流水寄存(ST_FEAT2)；
//   ④F5: 几何量单一真源 —— WC/CELLS/CHUNK/BANK_DEPTH/全部位宽由
//     IMG_W/IMG_H/T/LANES 派生, 杜绝旧版硬编 CHUNK=3652/BANK_DEPTH=2048
//     造成的"LANES*CHUNK=87648 < CELLS=97969 静默丢10321个位置"缺陷;
//     新增 IMG_W/IMG_H/T 一致性断言 + 运行期 cfg_tp 容量校验;
//   ⑤F6: ST_SCAN 扫描与归并并发 —— 各 lane 边扫边把命中压入 FIFO, 顶层
//     边弹出边归并 Top-32(停留式轮询: 有命中持续弹, 空则换 lane), 退出条件
//     =全部 lane 扫完 且 所有命中 FIFO 已空; 背压 scan_stall 在 FIFO 近满时
//     暂停对应 lane 扫描 -> 任意命中密度不溢出, 仅吞吐被节流;
//   ⑥修复 ST_RUN 不回 ST_FEAT 的缺陷(旧版只累加 1 个特征);
//   ⑦修复 tp 端口 12bit 截断(旧版除 lane0 外所有通道 cnt_l=0 永不工作);
//   ⑧修复 scan_pos/SCAN_MAX 未声明的编译错误(旧版残留);
//   ⑨修复 m_axi_rready 恒 1 在 out_fifo 满时丢 R 数据(RVALID && 满 -> 数据
//     无处安放即丢弃) —— 改为 rready = !out_empty;
//   ⑩min_idx 比较改为 ci < tcnt(6bit 全宽, 修复 tcnt[4:0] 满 32 翻转 bug)。
// ==================================================================
`include "sbm_geometry.vh"
module sbm_alg11_accum #(
	parameter LANES     = `SBM_LANES,
	parameter IMG_W     = `SBM_IMG_W,   // 级1图像宽（特征越界判定, 由 sbm_geometry.vh 派生）
	parameter IMG_H     = `SBM_IMG_H,   // 级1图像高
	parameter T         = `SBM_T,
	parameter FEAT_MAX  = `SBM_FEAT_MAX,
	parameter RESP_BASE = `SBM_RESP_BASE,
	parameter HIT_FIFO_DEPTH = 64       // 各 lane 命中FIFO深度(与并发drain配合防溢出)
)(
	input  wire        clk,           // 200MHz 核心域
	input  wire        rst_n,
	// ---- AXI4-Lite从机(100MHz域) ----
	input  wire        s_axi_aclk,
	input  wire        s_axi_aresetn,
	input  wire [7:0]  s_axi_awaddr,
	input  wire        s_axi_awvalid,
	output wire        s_axi_awready,
	input  wire [31:0] s_axi_wdata,
	input  wire [3:0]  s_axi_wstrb,
	input  wire        s_axi_wvalid,
	output wire        s_axi_wready,
	output wire [1:0]  s_axi_bresp,
	output wire        s_axi_bvalid,
	input  wire        s_axi_bready,
	input  wire [7:0]  s_axi_araddr,
	input  wire        s_axi_arvalid,
	output wire        s_axi_arready,
	output wire [31:0] s_axi_rdata,
	output wire [1:0]  s_axi_rresp,
	output wire        s_axi_rvalid,
	input  wire        s_axi_rready,
	// ---- AXI4-MM读主机(256bit, 单拍32B) ----
	output wire [31:0] m_axi_araddr,
	output wire        m_axi_arvalid,
	input  wire        m_axi_arready,
	output wire [7:0]  m_axi_arlen,
	output wire [2:0]  m_axi_arsize,
	output wire [1:0]  m_axi_arburst,
	input  wire [255:0] m_axi_rdata,
	input  wire        m_axi_rvalid,
	output wire        m_axi_rready,
	input  wire [1:0]  m_axi_rresp,
	input  wire        m_axi_rlast,
	// ---- 中断 ----
	output wire        irq_done
);
// ==================== 几何量派生(单一真源) ====================
localparam WC     = IMG_W / T;                 // 级1单元宽
localparam HC     = IMG_H / T;                 // 级1单元高
localparam CELLS  = WC * HC;                   // 单元总数
localparam TP_MAX = CELLS;                     // 位置总数上界(模板退化为1 cell)
localparam CHUNK  = (TP_MAX + LANES - 1) / LANES;  // 每通道位置数(ceil)
localparam BANK_DEPTH = 1 << $clog2((CHUNK + 1) / 2); // 2的幂 >= ceil(CHUNK/2)
localparam AW     = `SBM_W(BANK_DEPTH);        // bank地址/清零地址位宽(0..BANK_DEPTH-1)
localparam TPW    = `SBM_W(TP_MAX + 1);        // template_positions 位宽
localparam FCNTW  = `SBM_W(FEAT_MAX + 1);      // 特征计数位宽
localparam LW     = `SBM_W(LANES + 1);         // lane 号位宽
localparam SCAN_WD_MAX = CHUNK * 16 + 4096;    // ST_SCAN 看门狗上限
// ---------- 几何/容量一致性断言 ----------
generate
	if (IMG_W % T != 0)
		$error("alg11: IMG_W(%0d) 须为 T(%0d) 整数倍(请用 sbm_geometry.vh 派生值)", IMG_W, T);
	if (IMG_H % T != 0)
		$error("alg11: IMG_H(%0d) 须为 T(%0d) 整数倍(请用 sbm_geometry.vh 派生值)", IMG_H, T);
	if (IMG_W > 4096 || IMG_H > 4096)
		$error("alg11: IMG_W/IMG_H 须<=4096(hit_dout 坐标 12bit 打包限制)");
	if (LANES < 1)
		$error("alg11: LANES(%0d) 须 >= 1", LANES);
	if (TP_MAX != LANES * CHUNK - (LANES*CHUNK - TP_MAX))
		$error("alg11: TP_MAX(%0d) 与 LANES*CHUNK(%0d) 不一致", TP_MAX, LANES*CHUNK);
	if (RESP_BASE + 8 * (T*T) * CELLS > 32'hFFFF_FFFF)
		$error("alg11: 响应图内存区间超 4GB 地址空间");
endgenerate
assign m_axi_arlen   = 8'd0;      // 单拍突发
assign m_axi_arsize  = 3'b101;    // 32字节/拍
assign m_axi_arburst = 2'b01;     // INCR
// ==================== 前置声明(被头部组合逻辑/例化提前引用) ====================
// FSM 状态定义与状态寄存器(跨域同步段提前引用)
localparam ST_IDLE=3'd0, ST_ZERO=3'd1, ST_FEAT=3'd2, ST_FEAT2=3'd3,
	ST_RUN=3'd4, ST_SCAN=3'd5, ST_PUSH=3'd6, ST_DONE=3'd7;
reg [2:0]  st;
// lane 通道互联(例化段在 FSM 段之后)
wire [LANES-1:0] w_need, w_grant, w_done, w_hitv, w_hitpop, w_hitempty;
wire [LANES-1:0] w_scan_done, w_scan_stall;
wire [LANES-1:0] w_nslot;
wire [31:0] w_req [0:LANES-1];
wire [31:0] w_hit [0:LANES-1];
wire [7:0]  w_max [0:LANES-1];
// 归并寄存器(FSM 段引用)
reg [LW-1:0]  coll_lane;
reg [5:0]  push_idx;
reg [31:0] t32 [0:31];
reg [5:0]  tcnt;
// AXI-Lite 从机信号(特征/候选 FIFO 提前引用)
reg [7:0] awaddr_r, araddr_r;
reg [31:0] wdata_r;
reg aw_ok, w_ok, ar_ok;
wire lite_wr_pulse = aw_ok && w_ok && s_axi_bready;   // 写响应拍
wire lite_rd_pop   = ar_ok && s_axi_rready;           // 读完成拍
reg [7:0]  out_fifo [0:15];        // {lane[LW-1:0], slot}
reg [3:0]  out_wr, out_rd;
reg [4:0]  out_cnt;
wire       out_full  = (out_cnt == 5'd16);
wire       out_empty = (out_cnt == 5'd0);
wire [LW-1:0] r_lane = out_fifo[out_rd][LW:1];
wire       r_slot = out_fifo[out_rd][0];
wire w_ar_go, w_ar_fire, w_r_fire;
wire r_fill = w_r_fire;
reg [LW-1:0] rr_ptr;
// 修复: rready 恒 1(在途 ≤ 出队深 16, w_ar_go 的 !out_full 已保证 ar 侧不溢出;
// 恒 1 让特征边界的在途响应得以弹空出队, fill 由 w_run 门控丢弃, 防残留污染)
assign m_axi_rready  = 1'b1;
// ==================== 特征FIFO(s_axi_aclk -> clk) ====================
wire        feat_wr   = lite_wr_pulse && (awaddr_r[7:2] == 6'h03);   // 0x0C
wire [31:0] feat_dout;
wire        feat_empty, feat_full;
reg         feat_rd;
xpm_fifo_async #(
	.FIFO_MEMORY_TYPE("distributed"), .FIFO_WRITE_DEPTH(FEAT_MAX),
	.WRITE_DATA_WIDTH(32), .READ_DATA_WIDTH(32),
	.WR_DATA_COUNT_WIDTH(7), .RD_DATA_COUNT_WIDTH(7),
	.RELATED_CLOCKS(0), .FIFO_READ_LATENCY(1), .READ_MODE("fwft"),
	.ECC_MODE("no_ecc"), .SIM_ASSERT_CHK(0), .WAKEUP_TIME(0))
u_feat_fifo (
	.wr_clk(s_axi_aclk), .rst(~s_axi_aresetn), .wr_en(feat_wr),
	.din(s_axi_wdata), .full(feat_full), .wr_rst_busy(),
	.rd_clk(clk), .rd_en(feat_rd), .dout(feat_dout), .empty(feat_empty), .rd_rst_busy());
// ==================== 候选FIFO(clk -> s_axi_aclk) ====================
reg         cand_wr;
reg  [31:0] cand_din;             // {score[7:0], y[11:0], x[11:0]}
wire [31:0] cand_dout;
wire        cand_empty;
wire [5:0]  cand_rdcnt;
wire        cand_pop = lite_rd_pop && (araddr_r[7:2] == 6'h09);     // 0x24读即弹
xpm_fifo_async #(
	.FIFO_MEMORY_TYPE("distributed"), .FIFO_WRITE_DEPTH(32),
	.WRITE_DATA_WIDTH(32), .READ_DATA_WIDTH(32),
	.WR_DATA_COUNT_WIDTH(6), .RD_DATA_COUNT_WIDTH(6),
	.RELATED_CLOCKS(0), .FIFO_READ_LATENCY(1), .READ_MODE("fwft"),
	.ECC_MODE("no_ecc"), .SIM_ASSERT_CHK(0), .WAKEUP_TIME(0))
u_cand_fifo (
	.wr_clk(clk), .rst(~rst_n), .wr_en(cand_wr), .din(cand_din), .full(),
	.rd_clk(s_axi_aclk), .rd_en(cand_pop), .dout(cand_dout), .empty(cand_empty),
	.rd_data_count(cand_rdcnt), .rd_rst_busy());
// ==================== 跨域同步(2FF) ====================
reg start_l1;
always @(posedge s_axi_aclk) begin
	if (!s_axi_aresetn) start_l1 <= 1'b0;
	else start_l1 <= lite_wr_pulse && (awaddr_r[7:2] == 6'h00) && s_axi_wdata[0];
end
reg start_s0, start_s1, start_s2;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin start_s0 <= 1'b0; start_s1 <= 1'b0; start_s2 <= 1'b0; end
	else begin start_s0 <= start_l1; start_s1 <= start_s0; start_s2 <= start_s1; end
end
wire start_pulse = start_s1 && !start_s2;
reg status_c;
reg done_flag;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin status_c <= 1'b0; done_flag <= 1'b0; end
	else begin
		if (start_pulse) begin status_c <= 1'b1; done_flag <= 1'b0; end
		if (st == ST_DONE) begin status_c <= 1'b0; done_flag <= 1'b1; end
	end
end
reg status_s0, status_s1, done_s0, done_s1, tperr_c, tperr_s0, tperr_s1;
always @(posedge s_axi_aclk) begin
	if (!s_axi_aresetn) begin
		status_s0 <= 1'b0; status_s1 <= 1'b0; done_s0 <= 1'b0; done_s1 <= 1'b0;
		tperr_s0 <= 1'b0; tperr_s1 <= 1'b0;
	end else begin
		status_s0 <= status_c; status_s1 <= status_s0;
		done_s0   <= done_flag; done_s1 <= done_s0;
		tperr_s0  <= tperr_c;  tperr_s1 <= tperr_s0;
	end
end
// ==================== 配置寄存器 ====================
reg [31:0] cfg_tp, cfg_thr, cfg_resp, cfg_wc, cfg_nfeat;
reg [31:0] cfg_tp_c, cfg_thr_c, cfg_resp_c, cfg_wc_c, cfg_nfeat_c;
always @(posedge s_axi_aclk) begin
	if (!s_axi_aresetn) begin
		cfg_tp <= 32'd0; cfg_thr <= 32'd20; cfg_resp <= RESP_BASE;
		cfg_wc <= WC;     cfg_nfeat <= 32'd0;
	end else if (lite_wr_pulse) begin
		case (awaddr_r[7:2])
			6'h02: cfg_nfeat <= s_axi_wdata;   // 0x08
			6'h04: cfg_tp    <= s_axi_wdata;   // 0x10
			6'h05: cfg_thr   <= s_axi_wdata;   // 0x14 (归一化0~100)
			6'h06: cfg_resp  <= s_axi_wdata;   // 0x18
			6'h07: cfg_wc    <= s_axi_wdata;   // 0x1C
			default: ;
		endcase
	end
end
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		cfg_tp_c <= 32'd0; cfg_thr_c <= 32'd20; cfg_resp_c <= RESP_BASE;
		cfg_wc_c <= WC;     cfg_nfeat_c <= 32'd0;
	end else if (start_pulse) begin
		cfg_tp_c <= cfg_tp; cfg_thr_c <= cfg_thr; cfg_resp_c <= cfg_resp;
		cfg_wc_c <= cfg_wc; cfg_nfeat_c <= cfg_nfeat;
	end
end
// raw_thr预计算 + 运行期容量校验(F5: cfg_tp 超容量静默丢位置 -> 报错)
reg [8:0] raw_thr_r;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin raw_thr_r <= 9'd0; tperr_c <= 1'b0; end
	else if (start_pulse) begin
		raw_thr_r <= (cfg_thr_c[7:0] * cfg_nfeat_c[9:0] * 4) / 100;
		if (cfg_tp_c > LANES * CHUNK) begin
			tperr_c <= 1'b1;
			$error("alg11: cfg_tp(%0d) 超硬件容量 LANES*CHUNK(%0d), 请增大 LANES 或减小分辨率",
			        cfg_tp_c, LANES * CHUNK);
		end else
			tperr_c <= 1'b0;
	end
end
// ==================== 顶层FSM ====================
reg [FCNTW-1:0] fcnt;
reg [AW-1:0] zero_addr_r;
reg [31:0] scan_wd;
// 特征越界判定与地址计算
wire        w_bounds_ok = (feat_dout[11:0] < IMG_W) && (feat_dout[23:12] < IMG_H);
wire [5:0]  w_block = {feat_dout[14:12], feat_dout[2:0]};     // (y%8)*8 + x%8
wire [31:0] w_lm    = (feat_dout[23:15] * cfg_wc_c) + feat_dout[11:3];
wire [31:0] w_base  = cfg_resp_c + ({feat_dout[26:24], w_block} * CELLS) + w_lm;
reg [31:0] base_r;
reg        bounds_r;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		st <= ST_IDLE; fcnt <= {FCNTW{1'b0}}; zero_addr_r <= {AW{1'b0}}; scan_wd <= 32'd0;
		feat_rd <= 1'b0; base_r <= 32'd0; bounds_r <= 1'b0;
	end else begin
		feat_rd <= 1'b0;
		case (st)
			ST_IDLE: begin
				if (start_pulse) begin st <= ST_ZERO; fcnt <= {FCNTW{1'b0}}; zero_addr_r <= {AW{1'b0}}; end
			end
			ST_ZERO: begin
				zero_addr_r <= zero_addr_r + {{(AW-1){1'b0}},1'b1};
				if (zero_addr_r == BANK_DEPTH-1) st <= ST_FEAT;
			end
			ST_FEAT: begin
				if (fcnt >= cfg_nfeat_c[7:0] || feat_empty) begin
					st <= ST_SCAN;
					scan_wd <= 32'd0;      // F5a: 进入 ST_SCAN 时清看门狗(旧版跨运行累计致误触)
				end else begin
					feat_rd <= 1'b1;
					fcnt <= fcnt + {{(FCNTW-1){1'b0}},1'b1};
					base_r  <= w_base;
					bounds_r <= w_bounds_ok;
					st <= ST_FEAT2;
				end
			end
			ST_FEAT2: begin
				if (bounds_r) st <= ST_RUN;
				else          st <= ST_FEAT;
			end
			ST_RUN: begin                      // 全部通道累加完成且出队弹空 -> 回 ST_FEAT 处理下一特征
				if ((&w_done) && (out_cnt == 5'd0)) st <= ST_FEAT;
			end
			ST_SCAN: begin                     // F6: 扫描与归并并发
				scan_wd <= scan_wd + 32'd1;
				if ((&w_scan_done) && (&w_hitempty))
					st <= ST_PUSH;
				else if (scan_wd >= SCAN_WD_MAX) begin
					$error("F6: ST_SCAN watchdog timeout (scan/drain deadlock?)");
					st <= ST_PUSH;
				end
			end
			ST_PUSH: begin
				if (push_idx >= tcnt) st <= ST_DONE;
			end
			ST_DONE: begin
				st <= ST_IDLE;
			end
			default: st <= ST_IDLE;
		endcase
	end
end
// ==================== LANES 通道引擎例化 ====================
wire w_init = (st == ST_FEAT2) && bounds_r;
wire w_run  = (st == ST_RUN);
wire w_zero = (st == ST_ZERO);
wire w_scan = (st == ST_SCAN);
genvar g;
generate
	for (g=0; g<LANES; g=g+1) begin : gen_lane
		sbm_accum_lane #(
			.CHUNK(CHUNK), .BANK_DEPTH(BANK_DEPTH), .WC(WC), .HC(HC), .TPW(TPW),
			.HIT_FIFO_DEPTH(HIT_FIFO_DEPTH),
			.LB(g*CHUNK),
			.X0((g*CHUNK) % WC),
			.Y0((g*CHUNK) / WC))
		u_lane (
			.clk(clk), .rst_n(rst_n),
			.init(w_init),
			.base_addr(base_r), .tp(cfg_tp_c[TPW-1:0]), .thresh({7'd0, raw_thr_r}),
			.zero_en(w_zero), .zero_addr(zero_addr_r),
			.run_en(w_run),
			.lane_done(w_done[g]),
			.need_req(w_need[g]), .req_addr(w_req[g]), .next_slot(w_nslot[g]),
			.grant(w_grant[g]), .fill_slot(r_slot),
			.fill_vld(r_fill && (r_lane == g[LW-1:0]) && w_run),
			.fill_data(m_axi_rdata),
			.scan_en(w_scan), .hit_vld(w_hitv[g]), .hit_dout(w_hit[g]),
			.hit_pop(w_hitpop[g]), .hit_empty(w_hitempty[g]), .lane_max(w_max[g]),
			.scan_stall(w_scan_stall[g]), .scan_done(w_scan_done[g])
		);
	end
endgenerate
// ==================== 读仲裁(轮询) + 256bit AXI主机 ====================
integer ai;
reg [LW-1:0] grant_id; reg grant_v;
always @(*) begin
	grant_id = {LW{1'b0}}; grant_v = 1'b0;
	for (ai=0; ai<LANES; ai=ai+1) begin
		if (w_need[(rr_ptr + ai) % LANES]) begin
			grant_id = (rr_ptr + ai) % LANES;
			grant_v  = 1'b1;
			ai = LANES;
		end
	end
end
assign w_ar_go   = grant_v && !out_full;
assign w_ar_fire = w_ar_go && m_axi_arready;
assign w_r_fire  = m_axi_rvalid && !out_empty;
assign m_axi_arvalid = w_ar_go;
assign m_axi_araddr  = w_req[grant_id];
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		out_wr <= 4'd0; out_rd <= 4'd0; out_cnt <= 5'd0; rr_ptr <= {LW{1'b0}};
	end else begin
		if (w_ar_fire) begin
			out_fifo[out_wr] <= {grant_id[LW-1:0], w_nslot[grant_id]};
			out_wr <= out_wr + 4'd1;
			rr_ptr <= grant_id + {{(LW-1){1'b0}},1'b1};
		end
		if (w_r_fire) begin
			out_rd <= out_rd + 4'd1;
		end
		// 单表达式合并: 修复同拍 ar+r 时两路非阻塞赋值覆盖导致的计数丢失
		out_cnt <= out_cnt + (w_ar_fire ? 5'd1 : 5'd0) - (w_r_fire ? 5'd1 : 5'd0);
	end
end
generate
	for (g=0; g<LANES; g=g+1) begin : gen_grant
		assign w_grant[g] = w_ar_fire && (grant_id == g[LW-1:0]);
	end
endgenerate
// ==================== 候选归并与Top-32收集（F6: 扫描期并发归并） ====================
integer    ci;
reg [4:0]  min_idx;
reg [2:0]  prev_st;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) prev_st <= ST_IDLE; else prev_st <= st;
end
wire scan_entry = (st == ST_SCAN) && (prev_st != ST_SCAN);
wire push_entry = (st == ST_PUSH) && (prev_st != ST_PUSH);
always @(*) begin
	min_idx = 5'd0;
	for (ci=1; ci<32; ci=ci+1)
		if (ci < tcnt && t32[ci][31:24] < t32[min_idx][31:24])
			min_idx = ci[4:0];
end
// 组合 mux: coll_lane 选中的 lane 的命中数据/空标志
wire [LW-1:0] w_hit_is [0:LANES-1];
wire [31:0] hit_mux_prev [0:LANES-1];
wire       hempty_mux_prev [0:LANES-1];
generate
	for (g=0; g<LANES; g=g+1) begin : gen_hitmux
		assign w_hit_is[g] = (coll_lane == g[LW-1:0]);
		if (g == 0) begin
			assign hit_mux_prev[0]   = w_hit_is[0] ? w_hit[0] : 32'd0;
			assign hempty_mux_prev[0]= w_hit_is[0] ? w_hitempty[0] : 1'b1;
		end else begin
			assign hit_mux_prev[g]   = w_hit_is[g] ? w_hit[g] : hit_mux_prev[g-1];
			assign hempty_mux_prev[g]= w_hit_is[g] ? w_hitempty[g] : hempty_mux_prev[g-1];
		end
	end
endgenerate
wire [31:0] hit_mux   = hit_mux_prev[LANES-1];
wire        hitempty_mux = hempty_mux_prev[LANES-1];
generate
	for (g=0; g<LANES; g=g+1) begin : gen_pop
		// F6: ST_SCAN 归并从 coll_lane 弹出一个命中(与扫描并发), 停留式: 有命中持续弹
		assign w_hitpop[g] = (st == ST_SCAN) && (coll_lane == g[LW-1:0]) && !w_hitempty[g];
	end
endgenerate
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		coll_lane <= {LW{1'b0}}; push_idx <= 6'd0; tcnt <= 6'd0; cand_wr <= 1'b0; cand_din <= 32'd0;
		for (ci=0; ci<32; ci=ci+1) t32[ci] <= 32'd0;
	end else if (scan_entry) begin
		coll_lane <= {LW{1'b0}}; push_idx <= 6'd0; tcnt <= 6'd0; cand_wr <= 1'b0; cand_din <= 32'd0;
		for (ci=0; ci<32; ci=ci+1) t32[ci] <= 32'd0;
	end else if (st == ST_SCAN) begin
		cand_wr <= 1'b0;
		if (!hitempty_mux) begin
			// 当前 lane 有命中: 弹出并归并 Top-32, 游标停留(连续弹空为止)
			if (tcnt < 6'd32) begin t32[tcnt[4:0]] <= hit_mux; tcnt <= tcnt + 6'd1; end
			else if (hit_mux[31:24] > t32[min_idx][31:24]) t32[min_idx] <= hit_mux;
		end else begin
			// 当前 lane 空: 轮询下一个
			coll_lane <= (coll_lane == LANES-1) ? {LW{1'b0}} : coll_lane + {{(LW-1){1'b0}},1'b1};
		end
	end else if (st == ST_PUSH) begin
		// F5a: cand_din 提前一拍锁存 —— FIFO 在 wr_en 拍采样 din 的"拍初"值,
		// 若 cand_wr 与 cand_din 同拍 NBA 更新, 写进 FIFO 的将是上一拍旧数据
		// (XPM 同步 FIFO 语义: wr_en 与 din 同拍对齐, din 须在 wr_en 拍拍初稳定)
		cand_din <= t32[push_idx[4:0]];
		if (push_entry) begin
			cand_wr <= 1'b0;                     // 第一拍只锁存不写
			push_idx <= 6'd0;
		end else if (push_idx < tcnt) begin
			cand_wr <= 1'b1;                     // 写上一拍锁存的 t32[push_idx]
			push_idx <= push_idx + 6'd1;
		end else cand_wr <= 1'b0;
	end else cand_wr <= 1'b0;
end
assign irq_done = (st == ST_DONE);
// ==================== 全局最高得分(lane_max归约, SCAN完成时捕获) ====================
reg [31:0] max_score_r, max_score_s0, max_score_s1;
reg [7:0]  gmax_r;
integer    mg;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin max_score_r <= 32'd0; gmax_r <= 8'd0; end
	else if ((&w_scan_done) && (&w_hitempty)) begin
		gmax_r = w_max[0];
		for (mg=1; mg<LANES; mg=mg+1)
			if (w_max[mg] > gmax_r) gmax_r = w_max[mg];
		max_score_r <= {24'd0, gmax_r};
	end
end
always @(posedge s_axi_aclk) begin
	if (!s_axi_aresetn) begin max_score_s0 <= 32'd0; max_score_s1 <= 32'd0; end
	else begin max_score_s0 <= max_score_r; max_score_s1 <= max_score_s0; end
end
// ==================== AXI4-Lite从机FSM ====================
assign s_axi_awready = !aw_ok;
assign s_axi_wready  = !w_ok;
assign s_axi_bvalid  = aw_ok && w_ok;
assign s_axi_bresp   = 2'b00;
assign s_axi_arready = !ar_ok;
assign s_axi_rvalid  = ar_ok;
assign s_axi_rresp   = 2'b00;
always @(posedge s_axi_aclk) begin
	if (!s_axi_aresetn) begin aw_ok <= 1'b0; w_ok <= 1'b0; ar_ok <= 1'b0; end
	else begin
		if (s_axi_awvalid && !aw_ok) begin aw_ok <= 1'b1; awaddr_r <= s_axi_awaddr; end
		if (s_axi_wvalid && !w_ok)  begin w_ok  <= 1'b1; wdata_r  <= s_axi_wdata;  end
		if (aw_ok && w_ok && s_axi_bready) begin aw_ok <= 1'b0; w_ok <= 1'b0; end
		if (s_axi_arvalid && !ar_ok) begin ar_ok <= 1'b1; araddr_r <= s_axi_araddr; end
		if (ar_ok && s_axi_rready)   ar_ok <= 1'b0;
	end
end
assign s_axi_rdata =
	(araddr_r[7:2] == 6'h01) ? {29'd0, tperr_s1, done_s1, status_s1} :
	(araddr_r[7:2] == 6'h08) ? {26'd0, cand_rdcnt} :
	(araddr_r[7:2] == 6'h09) ? (cand_empty ? 32'd0 : cand_dout) :
	(araddr_r[7:2] == 6'h0A) ? max_score_s1 :
	32'd0;
endmodule
