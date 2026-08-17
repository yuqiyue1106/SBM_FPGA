// ==================================================================
// tb_sbm_alg11_accum.v : alg11 端到端联仿(F9 重写, 覆盖 F5/F6)
// 黄金模型: 直接按 line2Dup.cpp similarity() 语义计算
//   for each feature f: for j in [0, tp): score[j] += resp[(ori*T*T+blk)*CELLS + lm + j]
//   ori=label, blk=(y%T)*T+x%T, lm=(y/T)*WC+x/T; score 8bit 回绕
//   命中 = score[j] > raw_thr, raw_thr = floor(thr*4*nfeat/100)
// 校验:
//   1) 候选 FIFO 输出多集 == 黄金命中多集(命中数<=32 时 top-32 即全集);
//   2) max_score 寄存器 == 黄金最高分;
//   3) 密集命中(thr=0, 全命中)压力场景验证 F6 并发 drain 不溢出、无死锁;
//   4) 越界特征被丢弃(坐标越界特征不产生累加)。
// 几何: IMG 64x64, T=8, WC=HC=8, CELLS=64, TP_MAX=64, LANES=4, CHUNK=16
// ==================================================================
`timescale 1ns/1ps
module tb_sbm_alg11_accum;
parameter IMG_W = 64;
parameter IMG_H = 64;
parameter T = 8;
parameter LANES = 4;
parameter FEAT_MAX = 64;
parameter RESP_BASE = 32'h0800_0000;
localparam WC = IMG_W/T;
localparam HC = IMG_H/T;
localparam CELLS = WC*HC;
localparam TP_MAX = CELLS;
localparam LM_BYTES = 8*T*T*CELLS;      // 32768

// ---- 时钟 ----
reg clk, rst_n, s_axi_aclk, s_axi_aresetn;
always #2.5  clk = ~clk;
always #5    s_axi_aclk = ~s_axi_aclk;

// ---- AXI-Lite 主机侧 ----
reg [7:0]  s_axi_awaddr; reg s_axi_awvalid; wire s_axi_awready;
reg [31:0] s_axi_wdata;  reg [3:0] s_axi_wstrb;  reg s_axi_wvalid; wire s_axi_wready;
wire [1:0] s_axi_bresp;  wire s_axi_bvalid; reg s_axi_bready;
reg [7:0]  s_axi_araddr; reg s_axi_arvalid; wire s_axi_arready;
wire [31:0] s_axi_rdata; wire [1:0] s_axi_rresp; wire s_axi_rvalid; reg s_axi_rready;

// ---- AXI4-MM 读从机行为 ----
wire [31:0] m_axi_araddr; wire m_axi_arvalid; wire m_axi_arready;
wire [7:0] m_axi_arlen; wire [2:0] m_axi_arsize; wire [1:0] m_axi_arburst;
wire [255:0] m_axi_rdata; wire m_axi_rvalid; wire m_axi_rready; wire [1:0] m_axi_rresp; wire m_axi_rlast;
reg [31:0] r_addr; reg r_pend;
assign m_axi_arready = 1'b1;
assign m_axi_rvalid  = r_pend;
assign m_axi_rlast   = r_pend;
assign m_axi_rresp   = 2'b00;
reg [7:0] resp_mem [0:LM_BYTES-1];
genvar bb;
generate
	for (bb=0; bb<32; bb=bb+1) begin : gen_rdata
		assign m_axi_rdata[bb*8 +: 8] = resp_mem[(r_addr - RESP_BASE) + bb];
	end
endgenerate
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin r_addr <= 32'd0; r_pend <= 1'b0; end
	else begin
		if (m_axi_arvalid && m_axi_arready) begin r_addr <= m_axi_araddr; r_pend <= 1'b1; end
		else if (r_pend && m_axi_rready) r_pend <= 1'b0;
	end
end

wire irq_done;

// ---- 黄金模型 ----
reg [7:0] g_score [0:TP_MAX-1];
reg [31:0] g_hits [0:63];        // {score,y,x} 黄金命中(收集全部, 再取 top-32)
integer g_nhit;
integer g_feat_cnt, g_tp, g_nfeat;
reg [8:0] g_raw_thr;
integer g_max;
reg [31:0] feats [0:63];         // 特征集(全局, task 不使用数组参数)

task golden_run;
	input integer nfeat, tp, thr_norm;
	integer fi, j, ori, blk, lm, base, sc;
	reg [7:0] acc;
	reg [11:0] gy, gx;
begin
	g_nfeat = nfeat; g_tp = tp;
	g_raw_thr = (thr_norm * nfeat * 4) / 100;
	for (j=0; j<TP_MAX; j=j+1) g_score[j] = 8'd0;
	for (fi=0; fi<nfeat; fi=fi+1) begin
		// 与 DUT 一致: 越界特征被丢弃, 不参与累加(raw_thr 仍按 cfg_nfeat 计算)
		if (feats[fi][11:0] >= IMG_W || feats[fi][23:12] >= IMG_H) begin
			// skip
		end else begin
			ori = feats[fi][31:24];
			blk = (feats[fi][14:12] * 8) + feats[fi][2:0];       // (y%8)*8 + x%8
			lm  = (feats[fi][23:15] * WC) + feats[fi][11:3];     // (y/8)*WC + x/8
			base = (ori * 64 + blk) * CELLS + lm;
			for (j=0; j<tp; j=j+1) begin
				acc = g_score[j] + resp_mem[base + j];
				g_score[j] = acc;                                // 8bit 回绕
			end
		end
	end
	g_nhit = 0; g_max = 0;
	for (j=0; j<tp; j=j+1) begin
		gy = j / WC;
		gx = j % WC;
		if (g_score[j] > g_raw_thr) begin
			if (g_nhit < 64) g_hits[g_nhit] = {g_score[j], gy, gx};
			g_nhit = g_nhit + 1;
		end
		if (g_score[j] > g_max) g_max = g_score[j];
	end
	// top-32: 稳定选择排序(分数降序; 并列时先到者在前, 与 DUT 满表淘汰语义多集等价)
	begin: top32_sel
		integer best, kk;
		reg [31:0] tmp;
		for (j=0; j<32 && j<g_nhit; j=j+1) begin
			best = j;
			for (kk=j+1; kk<g_nhit; kk=kk+1)
				if (g_hits[kk][31:24] > g_hits[best][31:24]) best = kk;
			if (best != j) begin
				tmp = g_hits[j]; g_hits[j] = g_hits[best]; g_hits[best] = tmp;
			end
		end
	end
end
endtask

// ---- AXI-Lite 读写任务 ----
task lite_w;
	input [7:0] a; input [31:0] d;
begin
	@(posedge s_axi_aclk);
	s_axi_awaddr <= a; s_axi_awvalid <= 1'b1;
	s_axi_wdata  <= d; s_axi_wvalid  <= 1'b1;
	s_axi_bready <= 1'b1;
	while (!s_axi_bvalid) @(posedge s_axi_aclk);
	s_axi_awvalid <= 1'b0; s_axi_wvalid <= 1'b0;
	s_axi_bready  <= 1'b0;
	@(posedge s_axi_aclk);
end
endtask

task lite_r;
	input [7:0] a; output [31:0] d;
begin
	@(posedge s_axi_aclk);
	s_axi_araddr <= a; s_axi_arvalid <= 1'b1; s_axi_rready <= 1'b1;
	while (!s_axi_rvalid) @(posedge s_axi_aclk);
	d = s_axi_rdata;
	s_axi_arvalid <= 1'b0; s_axi_rready <= 1'b0;
	@(posedge s_axi_aclk);
end
endtask

// ---- 状态查询 ----
task wait_done;
	output integer to;
	integer wcnt; reg [31:0] st_r;
begin
	wcnt = 0; to = 0;
	while (wcnt < 4000000) begin
		lite_r(8'h04, st_r);
		// bit1=done_s1, bit0=status_s1(busy): done=1 && busy=0 才算完成
		if (st_r[1] && !st_r[0]) begin to = 0; return; end
		if (st_r[2]) begin to = 1; return; end               // tp 容量错误
		wcnt = wcnt + 1;
	end
	to = 2;                                         // timeout
end
endtask

// ---- LFSR 图样 ----
reg [15:0] lfsr;
task fill_mem;
	input integer zero_avoid;
	integer i;
begin
	lfsr = 16'hBEEF;
	for (i=0; i<LM_BYTES; i=i+1) begin
		lfsr = {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
		resp_mem[i] = zero_avoid ? (lfsr[7:0] | 8'h01) : lfsr[7:0];
	end
end
endtask

// ---- 主流程 ----
integer err, ncase;
reg [31:0] rd;
integer i, j, to, rdcnt;

task run_case;
	input integer nfeat, tp, thr_norm, zero_avoid;
	integer d_cnt, cand_n;
	reg [31:0] c;
	integer found;
begin
	fill_mem(zero_avoid);        // 先换响应图, 黄金与 DUT 用同一图样
	golden_run(nfeat, tp, thr_norm);
	// 写特征
	for (i=0; i<nfeat; i=i+1) lite_w(8'h0C, feats[i]);
	// 配置
	lite_w(8'h08, nfeat);
	lite_w(8'h10, tp);
	lite_w(8'h14, thr_norm);
	lite_w(8'h18, RESP_BASE);
	lite_w(8'h1C, WC);
	// 启动: 先清 start 寄存器(第二次及以后运行必须), 再置 1 产生跨域边沿
	lite_w(8'h00, 32'd0);
	lite_w(8'h00, 32'd1);
	wait_done(to);
	if (to != 0) begin
		$display("CASE%0d: ERROR to=%0d", ncase, to);
		err = err + 1;
	end else begin
		// 校验候选数
		lite_r(8'h20, rd);
		rdcnt = rd[5:0];
		cand_n = (g_nhit > 32) ? 32 : g_nhit;
		if (rdcnt != cand_n) begin
			$display("CASE%0d: cnt mismatch dut=%0d golden=%0d", ncase, rdcnt, cand_n);
			// dump DUT lane0 bank_e 前 8 项 vs golden score 偶位置
			$display("  dut bank0e: %02x %02x %02x %02x %02x %02x %02x %02x",
				dut.gen_lane[0].u_lane.u_bank_e.mem[0], dut.gen_lane[0].u_lane.u_bank_e.mem[1],
				dut.gen_lane[0].u_lane.u_bank_e.mem[2], dut.gen_lane[0].u_lane.u_bank_e.mem[3],
				dut.gen_lane[0].u_lane.u_bank_e.mem[4], dut.gen_lane[0].u_lane.u_bank_e.mem[5],
				dut.gen_lane[0].u_lane.u_bank_e.mem[6], dut.gen_lane[0].u_lane.u_bank_e.mem[7]);
			$display("  golden 0,2..14: %02x %02x %02x %02x %02x %02x %02x %02x",
				g_score[0], g_score[2], g_score[4], g_score[6], g_score[8], g_score[10], g_score[12], g_score[14]);
			err = err + 1;
		end
		// 逐条比对多集
		for (i=0; i<rdcnt; i=i+1) begin
			lite_r(8'h24, c);
			if (^c === 1'bx) begin
				$display("CASE%0d: candidate %0d is X", ncase, i);
				err = err + 1;
			end else begin
				found = 0;
				for (j=0; j<cand_n; j=j+1) begin
					if (g_hits[j] === c) begin found = 1; g_hits[j] = 32'hDEAD_BEEF; j = cand_n; end
				end
				if (!found) begin
					$display("CASE%0d: cand %0d = %08x (score=%0d y=%0d x=%0d) 不在黄金命中集",
					         ncase, i, c, c[31:24], c[23:12], c[11:0]);
					err = err + 1;
				end
			end
		end
		// 校验 max_score
		lite_r(8'h28, rd);
		if (rd[7:0] != g_max) begin
			$display("CASE%0d: max_score dut=%0d golden=%0d", ncase, rd[7:0], g_max);
			err = err + 1;
		end
		// 校验未匹配残留
		for (j=0; j<cand_n; j=j+1) if (g_hits[j] !== 32'hDEAD_BEEF && g_hits[j] !== 32'hDEAD_BEEF + 1) begin
			$display("CASE%0d: golden hit %0d = %08x 未被 DUT 输出", ncase, j, g_hits[j]);
			err = err + 1;
		end
	end
end
endtask

initial begin
	clk = 0; s_axi_aclk = 0; rst_n = 0; s_axi_aresetn = 0;
	s_axi_awaddr = 0; s_axi_awvalid = 0; s_axi_wdata = 0; s_axi_wstrb = 0;
	s_axi_wvalid = 0; s_axi_bready = 0; s_axi_araddr = 0; s_axi_arvalid = 0; s_axi_rready = 0;
	err = 0; ncase = 0;
	// 特征集: 3 条 + 1 条越界(应被丢弃)
	feats[0] = {8'd0, 12'd10, 12'd12};   // ori0, y=10, x=12
	feats[1] = {8'd1, 12'd50, 12'd40};   // ori1, y=50, x=40
	feats[2] = {8'd2, 12'd30, 12'd60};   // ori2, y=30, x=60
	feats[3] = {8'd0, 12'd100, 12'd12};  // y 越界(100 >= 64) -> 应丢弃
	feats[4] = {8'd3, 12'd62, 12'd62};   // ori3, 边界内
	feats[5] = {8'd0, 12'd63, 12'd63};
	feats[6] = {8'd1, 12'd1, 12'd1};
	feats[7] = {8'd2, 12'd2, 12'd2};
	repeat (5) @(posedge clk);
	rst_n = 1; s_axi_aresetn = 1;
	repeat (10) @(posedge clk);

	// CASE0: 4 特征(含1越界), tp=20, thr=40, 随机图样
	ncase = 0;
	run_case(4, 20, 40, 0);
	// CASE1: 全命中压力(F6 背压/FIFO 压力), thr=0, resp 全非0
	ncase = 1;
	run_case(4, 20, 0, 1);
	// CASE2: 8 特征, tp=64(全 CELLS), thr=20
	ncase = 2;
	run_case(8, 64, 20, 0);

	$display("==== RESULT: %s ====", (err == 0) ? "PASS" : "FAIL");
	$finish;
end

// 越界特征被丢弃的附加监视: RTL 不应为其发起 AXI 读(score 不变)
// (由 CASE0 黄金不含越界特征即可验证)

sbm_alg11_accum #(
	.LANES(LANES), .IMG_W(IMG_W), .IMG_H(IMG_H), .T(T),
	.FEAT_MAX(FEAT_MAX), .RESP_BASE(RESP_BASE)) dut (
	.clk(clk), .rst_n(rst_n),
	.s_axi_aclk(s_axi_aclk), .s_axi_aresetn(s_axi_aresetn),
	.s_axi_awaddr(s_axi_awaddr), .s_axi_awvalid(s_axi_awvalid), .s_axi_awready(s_axi_awready),
	.s_axi_wdata(s_axi_wdata), .s_axi_wstrb(s_axi_wstrb), .s_axi_wvalid(s_axi_wvalid), .s_axi_wready(s_axi_wready),
	.s_axi_bresp(s_axi_bresp), .s_axi_bvalid(s_axi_bvalid), .s_axi_bready(s_axi_bready),
	.s_axi_araddr(s_axi_araddr), .s_axi_arvalid(s_axi_arvalid), .s_axi_arready(s_axi_arready),
	.s_axi_rdata(s_axi_rdata), .s_axi_rresp(s_axi_rresp), .s_axi_rvalid(s_axi_rvalid), .s_axi_rready(s_axi_rready),
	.m_axi_araddr(m_axi_araddr), .m_axi_arvalid(m_axi_arvalid), .m_axi_arready(m_axi_arready),
	.m_axi_arlen(m_axi_arlen), .m_axi_arsize(m_axi_arsize), .m_axi_arburst(m_axi_arburst),
	.m_axi_rdata(m_axi_rdata), .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(m_axi_rready),
	.m_axi_rresp(m_axi_rresp), .m_axi_rlast(m_axi_rlast),
	.irq_done(irq_done)
);
endmodule
