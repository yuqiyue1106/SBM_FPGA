// ============================================================================
// tb_sbm_alg8_spread.v : alg8 T×T 扩散的背压(stall)与逐帧 tuser 回归 (P0-3)
// 验证点:
//   1) 黄金模型: 直接按 spread 语义 dst(x,y) = OR_{r,c∈[0,T)} src(x+c,y+r),
//      越界按 0(右缘/下缘零填充), 逐拍比对输出数据;
//   2) 下游反压: m_axis_tready 周期拉低(占空比 2/3), 验证 stall 冻结全流水,
//      输出序列与黄金逐拍一致 —— 不丢像素 / 不重复像素 / 不错位;
//   3) 连续两帧: 验证 m_axis_tuser 逐帧重发(旧版 out_row 单调递增不回绕,
//      tuser 仅复位后首帧有效, 第二帧起 alg9 不再锁帧首);
//   4) 输出总数 == 2*IMG_W*IMG_H, 行末 tlast 时序逐拍校验。
// 运行: iverilog -g2012 -o sim_alg8.out tb_sbm_alg8_spread.v sbm_alg8_spread.v \
//       xpm_memory_sdpram_beh.v && vvp sim_alg8.out
// ============================================================================
`timescale 1ns/1ps
module tb_sbm_alg8_spread;

parameter IMG_W = 64;
parameter IMG_H = 48;
parameter T = 8;

reg clk, rst_n;
reg s_axis_tvalid; reg [7:0] s_axis_tdata;
reg s_axis_tuser, s_axis_tlast;
wire s_axis_tready;
wire m_axis_tvalid; reg m_axis_tready;
wire [7:0] m_axis_tdata;
wire m_axis_tuser, m_axis_tlast;

// ---- 输入图样(非退化 LFSR 散列) ----
reg [7:0] img [0:IMG_H-1][0:IMG_W-1];
reg [15:0] lfsr;
integer i;
always #5 clk = ~clk;
initial begin
	lfsr = 16'hACE1;
	for (i = 0; i < IMG_W*IMG_H; i = i + 1) begin
		lfsr = {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
		img[i/IMG_W][i%IMG_W] = lfsr[7:0];
	end
end

// ---- 黄金模型: 前向 T×T OR(越界补 0) ----
reg [7:0] golden [0:IMG_H-1][0:IMG_W-1];
integer y, x, r2, c2;
task golden_build;
begin
	for (y = 0; y < IMG_H; y = y + 1)
		for (x = 0; x < IMG_W; x = x + 1) begin
			golden[y][x] = 8'd0;
			for (r2 = 0; r2 < T; r2 = r2 + 1)
				for (c2 = 0; c2 < T; c2 = c2 + 1)
					if ((y + r2 < IMG_H) && (x + c2 < IMG_W))
						golden[y][x] = golden[y][x] | img[y+r2][x+c2];
		end
end
endtask

// ---- 下游反压: 每 4 拍只接受 3 拍(bp==2 时拉低 tready) ----
reg [1:0] bp;
always @(posedge clk or negedge rst_n)
	if (!rst_n) bp <= 2'd0;
	else         bp <= bp + 2'd1;
// P0-3 背压回归: 使能周期反压, 验证 stall 冻结全流水且不丢/重/错位
// (此前为定位数据错误临时关闭过, 现已恢复)
always @(*) m_axis_tready = (bp != 2'd2);

// ---- 输入帧(严格 AXI-S 握手, tlast 每行末; 行尾留 T-1 拍间隙供 DUT 行尾
//      补零排空 —— flush_c 仅在 s_axis_tvalid=0 时递减, 连续满速发送会使
//      w_in_data 被补零状态恒置 0) ----
// P0-3 TB 修复: s_axis_* 全部用 NBA 驱动。旧版阻塞赋值 + @(posedge) 与 DUT
// 同拍采样存在竞争: 行末像素消费拍 TB 在 posedge 恢复后立即改 tvalid, 若 DUT
// 采样早于 TB 恢复, 行末像素被重复消费一拍, 行尾补零 7 拍被吃掉 1 拍
// (每行 64+6=70 拍, 行缓冲每行欠 1 个补零值, 垂直级错位并最终挂死)。
// NBA 驱动保证 DUT 每个 posedge 采到的都是上一拍稳定值, 竞争消除。
task send_frame;
	integer j;
	integer guard;
begin
	for (j = 0; j < IMG_W*IMG_H; j = j + 1) begin
		s_axis_tvalid <= 1'b1;
		s_axis_tdata  <= img[j/IMG_W][j%IMG_W];
		s_axis_tuser  <= (j == 0);
		s_axis_tlast  <= (j % IMG_W == IMG_W-1);
		@(posedge clk);
		guard = 0;
		while (s_axis_tready !== 1'b1) begin
			@(posedge clk); guard = guard + 1;
			if (guard > 5000) begin
				$display("STUCK in send_frame j=%0d tready=%0d", j, s_axis_tready);
				$finish;
			end
		end
		if (j % IMG_W == IMG_W-1) begin
			s_axis_tvalid <= 1'b0;
			repeat (T-1) begin @(posedge clk); while (s_axis_tready !== 1'b1) @(posedge clk); end
		end
	end
	s_axis_tvalid <= 1'b0;
	@(posedge clk);
end
endtask

// ---- 输出收集与逐拍比对 ----
integer err, tot, yc, xc;
initial begin
	clk = 0; rst_n = 0;
	s_axis_tvalid = 0; s_axis_tdata = 0; s_axis_tuser = 0; s_axis_tlast = 0;
	err = 0; tot = 0; yc = 0; xc = 0;
	golden_build;
	repeat (10) @(posedge clk);
	rst_n = 1;
	send_frame;
	send_frame;                       // 连续两帧: 验证 tuser 逐帧重发
	begin: wait_out
		integer wcnt;
		wcnt = 0;
		while ((tot < 2*IMG_W*IMG_H) && (wcnt < 200000)) begin
			@(posedge clk); wcnt = wcnt + 1;
		end
		$display("DBG wcnt=%0d tot=%0d err=%0d tready=%0d tvalid=%0d", wcnt, tot, err, m_axis_tready, m_axis_tvalid);
		repeat (50) @(posedge clk);   // 排空尾部
	end
	if ((err == 0) && (tot == 2*IMG_W*IMG_H))
		$display("RESULT: PASS (alg8 stall backpressure + per-frame tuser, tot=%0d)", tot);
	else
		$display("RESULT: FAIL err=%0d tot=%0d exp=%0d", err, tot, 2*IMG_W*IMG_H);
	$finish;
end

always @(posedge clk) begin
	if (m_axis_tvalid && m_axis_tready) begin
		tot = tot + 1;
		if (yc >= IMG_H-6)
			$display("  DBG (y=%0d,x=%0d) act=%02X exp=%02X", yc, xc, m_axis_tdata, golden[yc][xc]);
		if (m_axis_tdata !== golden[yc][xc]) begin
			err = err + 1;
			if (err <= 10) $display("  DATA ERR (y=%0d,x=%0d) exp=%02X act=%02X",
			                         yc, xc, golden[yc][xc], m_axis_tdata);
		end
		if (m_axis_tuser !== ((yc == 0) && (xc == 0))) begin
			err = err + 1;
			if (err <= 10) $display("  TUSER ERR (y=%0d,x=%0d) act=%0d", yc, xc, m_axis_tuser);
		end
		if (m_axis_tlast !== (xc == IMG_W-1)) begin
			err = err + 1;
			if (err <= 10) $display("  TLAST ERR (y=%0d,x=%0d) act=%0d", yc, xc, m_axis_tlast);
		end
		if (xc == IMG_W-1) begin
			xc = 0;
			yc = (yc == IMG_H-1) ? 0 : yc + 1;
		end else
			xc = xc + 1;
	end
end

sbm_alg8_spread #(.IMG_W(IMG_W), .IMG_H(IMG_H), .T(T)) u_dut (
	.clk(clk), .rst_n(rst_n),
	.s_axis_tvalid(s_axis_tvalid), .s_axis_tready(s_axis_tready),
	.s_axis_tdata(s_axis_tdata), .s_axis_tuser(s_axis_tuser), .s_axis_tlast(s_axis_tlast),
	.m_axis_tvalid(m_axis_tvalid), .m_axis_tready(m_axis_tready),
	.m_axis_tdata(m_axis_tdata), .m_axis_tuser(m_axis_tuser), .m_axis_tlast(m_axis_tlast)
);

endmodule
