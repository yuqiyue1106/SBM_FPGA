// ==================================================================
// tb_sbm_alg9_cosim.v : alg9(F4) 落盘地址布局/乒乓/AXI 主机 联仿(加固版)
// 与旧版差异(F5a 加固):
//   1) 输入图样改为 16bit LFSR 散列值 & 0xFF, 消除旧图样 (y*W+x)&0xFF 在
//      "band 首像素被覆盖"等地址错位场景下的退化巧合(旧版曾因此假通过);
//   2) 显式 x 检测: m_axi_awaddr / m_axi_wdata 出现 x 一律计入 err
//      (旧版用 integer 承接, x 参与比较得 unknown -> if 判假 -> 假通过);
//   3) wstrb 感知: 末拍部分选通的无效字节不参与数据比对与 tot 计数
//      (支持 SEG_BEATS=ceil(WC/8) 的任意 WC);
//   4) IMG_W/IMG_H 参数化, 可用 iverilog -P 覆盖测试多分辨率。
//   5) P0-2 AXI 对齐断言: 每突发首拍校验 awaddr 8B 对齐(awsize=8B 时
//      awaddr[2:0] 必须为 0), 违反即计入 err —— 旧 TB 从未检查协议对齐。
//   6) P0-1 连续帧回归(-DALG9_CONT_FRAMES): 第二帧 tlast 后紧接 tuser,
//      不等 irq_done/drain 完成 —— 复现"旧帧末 band(bank0) 尚在 drain 时
//      新帧首像素写 bank0"的竞争; 修复前 tready 只查 bank1 放行新帧首拍
//      → 新旧帧数据混入(比对 err), 修复后 tready 帧首查 bank0 阻塞到 drain 完。
// 每拍每有效字节按读取公式反解 (ori,block,band,cx)->(y,x):
//   cell = (awaddr-RESP_BASE) + beat*8 + j
//   y = band*T + block/T;  x = cx*T + block%T, cx = beat*8+j
//   期望 resp[ori] = max(SIM[ori*32+lsb], SIM[ori*32+16+msb])
// 校验写拍有效字节总数 == 8*T*T*HC*WC 且地址不越界。
// ==================================================================
`timescale 1ns/1ps
module tb_sbm_alg9_cosim;
parameter IMG_W = 64;
parameter IMG_H = 64;
parameter T = 8;
parameter RESP_BASE = 32'h0800_0000;

localparam WC = IMG_W/T;
localparam HC = IMG_H/T;
localparam CELLS = WC*HC;
localparam SEG_BEATS = (WC+7)/8;
localparam TOT_BYTES = 8*T*T*HC*WC;
`ifdef ALG9_CONT_FRAMES
localparam FRAMES = 2;    // 连续两帧, 第二帧不等 irq_done
`else
localparam FRAMES = 1;
`endif

reg clk, rst_n;
reg s_axis_tvalid;
reg [7:0] s_axis_tdata;
reg s_axis_tuser, s_axis_tlast;
wire s_axis_tready;
// AXI4-MM 主机接口(连行为从机)
wire m_axi_awready = 1'b1;
wire m_axi_awvalid; wire [31:0] m_axi_awaddr;
wire [7:0] m_axi_awlen; wire [2:0] m_axi_awsize; wire [1:0] m_axi_awburst;
wire m_axi_wready  = 1'b1;
wire m_axi_wvalid; wire [63:0] m_axi_wdata; wire [7:0] m_axi_wstrb; wire m_axi_wlast;
reg  m_axi_bvalid;
wire m_axi_bready;
wire irq_done;

reg [7:0] in_mag [0:4095][0:4095];
reg [7:0] SIM [0:255];
integer y, x, j;
integer cll, ori, rm, blk, rem2, bnd, cx, yy, xx, lsb, msb, exv, actv;
integer err, tot, oob;
reg [5:0] beat_cnt;
reg b_pend;

initial $readmemh("similarity_lut.mem", SIM);

always #5 clk = ~clk;

// 16bit LFSR 生成非退化图样
reg [15:0] lfsr;
task fill_img;
	integer i;
begin
	lfsr = 16'hACE1;
	for (i=0;i<IMG_W*IMG_H;i=i+1) begin
		lfsr = {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
		in_mag[i/IMG_W][i%IMG_W] = lfsr[7:0];
	end
end
endtask

// ---- 行为 AXI 从机 ----
assign m_axi_awready = 1'b1;
assign m_axi_wready  = 1'b1;
assign m_axi_bready  = 1'b1;
always @(posedge clk or negedge rst_n)
	if (!rst_n) b_pend <= 1'b0;
	else if (m_axi_wvalid && m_axi_wready && m_axi_wlast) b_pend <= 1'b1;
	else if (m_axi_bvalid) b_pend <= 1'b0;
assign m_axi_bvalid = b_pend;

// ---- 输入帧(严格 AXI-S 握手) ----
task send_frame;
	integer i;
begin
	for (i=0;i<IMG_W*IMG_H;i=i+1) begin
		s_axis_tvalid = 1'b1;
		s_axis_tdata  = in_mag[i/IMG_W][i%IMG_W];
		s_axis_tuser  = (i==0);
		s_axis_tlast  = (i==IMG_W*IMG_H-1);
		@(posedge clk);
		while (s_axis_tready !== 1'b1) @(posedge clk);
	end
	s_axis_tvalid = 1'b0;
	@(posedge clk);
end
endtask

// ---- P0-1 回归: 连续两帧, 第二帧 tlast 后紧接 tuser(不等 drain 完成) ----
task send_two_frames;
	integer i;
begin
	repeat (2) begin
		for (i=0;i<IMG_W*IMG_H;i=i+1) begin
			s_axis_tvalid = 1'b1;
			s_axis_tdata  = in_mag[i/IMG_W][i%IMG_W];
			s_axis_tuser  = (i==0);
			s_axis_tlast  = (i==IMG_W*IMG_H-1);
			@(posedge clk);
			while (s_axis_tready !== 1'b1) @(posedge clk);
		end
	end
	s_axis_tvalid = 1'b0;
	@(posedge clk);
end
endtask

initial begin
	clk=0; rst_n=0; s_axis_tvalid=0; s_axis_tdata=0; s_axis_tuser=0; s_axis_tlast=0;
	beat_cnt=0; b_pend=0;
	err=0; tot=0; oob=0;
	fill_img;
	#100 rst_n=1;
`ifdef ALG9_CONT_FRAMES
	send_two_frames;
`else
	send_frame;
`endif
	begin: wait_done
		integer wcnt;
		wcnt = 0;
		while (!dut.irq_done && wcnt < 2000000) begin @(posedge clk); wcnt = wcnt + 1; end
		if (!dut.irq_done) $display("WARNING: irq_done 未置位, wcnt=%0d", wcnt);
		repeat (200) @(posedge clk);
	end
	$display("AXI writes: tot=%0d err=%0d oob=%0d  expected=%0d", tot, err, oob, FRAMES*TOT_BYTES);
	if ((err==0) && (oob==0) && (tot == FRAMES*TOT_BYTES))
		$display("RESULT: PASS (alg9 F4 response-map layout matches alg11 readback)");
	else
		$display("RESULT: FAIL (alg9 F4, err=%0d oob=%0d tot=%0d exp=%0d)", err, oob, tot, FRAMES*TOT_BYTES);
	$finish;
end

// 每写拍逐字节黄金校验(wstrb 感知: 掩掉的字节跳过)
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) beat_cnt <= 6'd0;
	else if (m_axi_wvalid && m_axi_wready) begin
		if (m_axi_wlast) beat_cnt <= 6'd0;
		else             beat_cnt <= beat_cnt + 6'd1;
	end
end

always @(posedge clk) begin
	if (m_axi_wvalid && m_axi_wready) begin
		// P0-2 TB 断言: 64bit 写主机每突发首拍 awaddr 必须 8B 对齐 + awsize=3(8B)
		if ((beat_cnt == 6'd0) && ((m_axi_awaddr[2:0] !== 3'b000) || (m_axi_awsize !== 3'd3))) begin
			$display("  AXI ALIGN ERR: awaddr=%08X awsize=%0d", m_axi_awaddr, m_axi_awsize);
			err = err + 1;
		end
		// x 检测: 地址或数据含 x 一律计入错误(旧版假通过的根因)
		if ((^m_axi_awaddr === 1'bx) || (^m_axi_wdata === 1'bx)) begin
			err = err + 8;
		end else begin
			for (j=0;j<8;j=j+1) begin
				if (!m_axi_wstrb[j]) begin
					// 末拍部分选通的无效字节: 不比对、不计 tot
				end else begin
					cll  = (m_axi_awaddr - RESP_BASE) + beat_cnt*8 + j;
					ori  = cll / (T*T*CELLS);
					rm   = cll % (T*T*CELLS);
					blk  = rm / CELLS;
					rem2 = rm % CELLS;
					bnd  = rem2 / WC;
					cx   = beat_cnt*8 + j;   // 段内字节索引 k=beat*8+j 即 x/T
					yy   = bnd*T + blk/T;
					xx   = cx*T + (blk % T);
					lsb  = in_mag[yy][xx][3:0];
					msb  = in_mag[yy][xx][7:4];
					exv  = (SIM[ori*32+lsb] > SIM[ori*32+16+msb]) ? SIM[ori*32+lsb] : SIM[ori*32+16+msb];
					actv = m_axi_wdata[j*8 +: 8];
					tot = tot + 1;
					if (ori>7 || blk>T*T-1 || bnd>HC-1 || cx>WC-1 || (m_axi_awaddr<RESP_BASE)) oob = oob + 1;
					else if (exv != actv) begin
						if (err < 12) $display("  ERR cell=%0d ori=%0d blk=%0d bnd=%0d cx=%0d (y=%0d,x=%0d) exp=%02X act=%02X",
						                        cll, ori, blk, bnd, cx, yy, xx, exv, actv);
						err = err + 1;
					end
				end
			end
		end
	end
end

sbm_alg9_lut #(.IMG_W(IMG_W), .IMG_H(IMG_H), .T(T), .RESP_BASE(RESP_BASE)) dut (
	.clk(clk), .rst_n(rst_n),
	.s_axis_tvalid(s_axis_tvalid), .s_axis_tready(s_axis_tready),
	.s_axis_tdata(s_axis_tdata), .s_axis_tuser(s_axis_tuser), .s_axis_tlast(s_axis_tlast),
	.m_axi_awvalid(m_axi_awvalid), .m_axi_awready(m_axi_awready),
	.m_axi_awaddr(m_axi_awaddr), .m_axi_awlen(m_axi_awlen),
	.m_axi_awsize(m_axi_awsize), .m_axi_awburst(m_axi_awburst),
	.m_axi_wvalid(m_axi_wvalid), .m_axi_wready(m_axi_wready),
	.m_axi_wdata(m_axi_wdata), .m_axi_wstrb(m_axi_wstrb), .m_axi_wlast(m_axi_wlast),
	.m_axi_bvalid(m_axi_bvalid), .m_axi_bready(m_axi_bready),
	.irq_done(irq_done)
);
endmodule
