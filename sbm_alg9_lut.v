// ==================================================================
// sbm_alg9_lut.v : 响应图LUT查表 + 线性化落盘
// 与line2Dup.cpp computeResponseMaps()+linearize()语义一致:
//   resp_ori(x,y) = max(SIM[ori*32 + byte&15], SIM[ori*32 + 16 + byte>>4])
//   落盘地址 = RESP_BASE + (ori*T*T + block)*CELLS + cell
//   其中 block = (y%T)*T + x%T, cell = (y/T)*Wc + x/T = band*WC + cx
// 实现: LUTRAM并行查表 -> T行角转(ping-pong双bank) -> 8方向×分段缓冲
//   -> AXI4-MM(64bit,每段SEG_BEATS拍)顺序写DDR
// 修正记录：①SIMILARITY_LUT改由$readmemh("similarity_lut.mem")装载
//   (该mem由line2Dup.cpp第737行SIMILARITY_LUT自动转储, 原版内联手填表
//   方向2/3值与C++不符、方向4~7全0, 且无readmemh调用)；
//   ②角转级由示意伪代码补全为可综合状态机(原版含TODO与占位地址公式)；
//   ③AXI4-MM写主机补全AW/W/B完整握手与突发控制；
//   ④F4: req_fifo 写入位域 {pong,oi,block,band} 与读侧解包位域严格对齐；
//   ⑤F4: req_cnt 同拍 push+pop 双非阻塞赋值丢 +8 计数下溢 -> 合并为单计数；
//   ⑥F4: 写请求FIFO深度参数化(REQ_FIFO_DEPTH, 默认16, 加安全裕度断言)。
//   ⑦F4: 落盘调度改为 per-band "band_filled" 标志 + 独立 drain FSM；
//   ⑧F5: 默认 IMG_W/IMG_H 由 2500 改为 2504(级1对齐 T=8), 后随 F5a 改为
//   sbm_geometry.vh 派生(2560, WC=320 满足 64bit AXI 写 8B 对齐);
//   WC/HC/CELLS 已由 IMG_W/IMG_H/T 派生。
//   ⑨F5a(参数可维护性, 2026-08-14): 全部几何量与位宽由 sbm_geometry.vh
//   派生, 修复 5 个"换相机即崩坏"的硬编码缺陷:
//     (1) px_cnt[14:0] 装不下 2504^2=6.27M(需23bit) -> 真实分辨率下帧逻辑
//         崩坏(此前只在 64x64 联仿下成立);
//     (2) SEG_BEATS=WC/8 整除截断: WC=313 时 39 拍只写 312 字节, 每行丢 1
//         个 cell -> 改为 ceil 拍数 + 末拍 wstrb 部分选通;
//     (3) req_fifo 指针 6bit 索引深度 16 数组, 越界返回 x(实测 req_rd 达
//         16..20, w_req_r 全 x) -> 指针/计数位宽按深度派生, 索引取模;
//     (4) band_cnt 初始化少 1, 使每个 band 的第一像素被写进上一个 bank 的
//         地址 0(数据被下一个 band 覆盖) -> band_cnt 帧首初始化为 1, 与
//         "像素k的band内序号=k mod BAND_DEPTH" 严格对齐;
//     (5) 段缓冲仅 2 槽(pong), 与"req_full 允许 2 段组驻留"的上界恰冲突,
//         慢速 AXI 从机下存在段覆盖风险 -> 段缓冲槽数参数化 SEG_BUFS(4),
//         请求FIFO高水位按 8*(SEG_BUFS-1) 节流, 数学上保证无覆盖;
//     (6) 写侧行列坐标改由独立 cur_row/cur_col 计数器同步推进(原 px_cnt/
//         IMG_W 除法对真实位宽既慢又易错), 帧首写地址强制(0,0), 修掉多帧
//         连续输入时 tuser 拍残留旧坐标的错位隐患。
// 前置约束: IMG_W、IMG_H 必须为 T 的整数倍 —— 该对齐由 sbm_geometry.vh
//   自动计算(级0对齐到2T再降采样), 本模块只做防御性断言。
// ==================================================================
`include "sbm_geometry.vh"
module sbm_alg9_lut #(
	parameter IMG_W    = `SBM_IMG_W,   // 级1图像宽(由 sbm_geometry.vh 派生)
	parameter IMG_H    = `SBM_IMG_H,   // 级1图像高
	parameter T        = `SBM_T,
	parameter RESP_BASE = `SBM_RESP_BASE,
	parameter REQ_FIFO_DEPTH = 32,     // 写请求FIFO深度(须 >= 8*SEG_BUFS, 见断言)
	parameter SEG_BUFS  = 4            // 每方向段缓冲槽数(>=2, 覆盖保护随槽数增强)
)(
	input  wire        clk,
	input  wire        rst_n,
	input  wire        s_axis_tvalid,
	output wire        s_axis_tready,
	input  wire [7:0]  s_axis_tdata,
	input  wire        s_axis_tuser,
	input  wire        s_axis_tlast,
	// AXI4-MM 写主机(64bit数据, INCR突发)
	output wire        m_axi_awvalid, input  wire m_axi_awready,
	output wire [31:0] m_axi_awaddr,
	output wire [7:0]  m_axi_awlen,
	output wire [2:0]  m_axi_awsize,
	output wire [1:0]  m_axi_awburst,
	output wire        m_axi_wvalid,  input  wire m_axi_wready,
	output wire [63:0] m_axi_wdata,
	output wire [7:0]  m_axi_wstrb,
	output wire        m_axi_wlast,
	input  wire        m_axi_bvalid, output wire m_axi_bready,
	output wire        irq_done
);
// ==================== 几何量全部派生(单一真源) ====================
localparam WC    = IMG_W / T;            // 单元宽
localparam HC    = IMG_H / T;            // 单元高
localparam CELLS = WC * HC;              // 单元总数
localparam BAND_DEPTH = T * IMG_W;       // 每bank深度(像素数)
localparam DRAIN_CYCLES = T * T * WC;    // 每band读侧周期数
localparam SEG_LEN = WC;                 // 每段字节数
localparam SEG_BEATS = (SEG_LEN + 7) / 8;// 每段节拍数(ceil, 末拍wstrb部分选通)
// 位宽派生(SBM_W(n) 表示 0..n-1 所需位宽, n<2 钳到 1)
localparam PW  = `SBM_W(IMG_W * IMG_H);      // 帧像素计数
localparam CW  = `SBM_W(IMG_W + 1);          // 列号(0..IMG_W-1)
localparam RW  = `SBM_W(IMG_H + 1);          // 行号(0..IMG_H-1)
localparam BAW = `SBM_W(BAND_DEPTH + 1);     // band内地址
localparam BCW = `SBM_W(BAND_DEPTH + 1);     // band_cnt(0..BAND_DEPTH-1)
localparam HCW = `SBM_W(HC + 1);             // band号(0..HC-1)
localparam DCW = `SBM_W(DRAIN_CYCLES + 1);   // drain_cnt
localparam CXW = `SBM_W(WC + 1);             // rd_cx(0..WC-1)
localparam QW  = `SBM_W(REQ_FIFO_DEPTH + 1); // FIFO指针/计数
localparam RIDXW = $clog2(SEG_BUFS);         // 段缓冲槽号
localparam PFW = RIDXW + 3 + 6 + HCW;        // 请求项宽 {slot,ori,block,band}
// ---------- 几何/容量防御性断言 ----------
generate
	if (IMG_W % T != 0)
		$error("alg9: IMG_W(%0d) 须为 T(%0d) 整数倍(请用 sbm_geometry.vh 派生值)", IMG_W, T);
	if (IMG_H % T != 0)
		$error("alg9: IMG_H(%0d) 须为 T(%0d) 整数倍(请用 sbm_geometry.vh 派生值)", IMG_H, T);
	if (WC % 8 != 0)
		$error("alg9: WC(%0d) 须为 8 的整数倍(64bit AXI 写 awaddr 8B 对齐, 级0 需 16T 对齐, 请检查 sbm_geometry.vh)", WC);
	if (SEG_BEATS > 256)
		$error("alg9: SEG_BEATS=%0d 超 AXI awlen(8bit)上限 256, 请减小 IMG_W", SEG_BEATS);
	if (REQ_FIFO_DEPTH < 8 * SEG_BUFS)
		$error("alg9: REQ_FIFO_DEPTH(%0d) 须 >= 8*SEG_BUFS(%0d), 否则段缓冲覆盖", REQ_FIFO_DEPTH, 8*SEG_BUFS);
	if (SEG_BUFS < 4)
		$error("alg9: SEG_BUFS(%0d) 须 >= 4(槽覆盖保护不变量要求)", SEG_BUFS);
	if (RESP_BASE + 8 * (T*T) * CELLS > 32'hFFFF_FFFF)
		$error("alg9: 响应图内存区间超 4GB 地址空间, 请减小分辨率或调整 RESP_BASE");
endgenerate
// ==================== 写侧寄存器 ====================
reg [PW-1:0]  px_cnt;             // 本帧已接受像素数(0..IMG_W*IMG_H-1)
reg [CW-1:0]  cur_col;            // 当前帧已接受像素的下一像素列号(0..IMG_W-1)
reg [RW-1:0]  cur_row;            // 下一像素行号
reg [BCW-1:0] band_cnt;           // 当前band已写像素数(0..BAND_DEPTH-1)
reg           band_par;           // 当前写入bank(0/1)
reg [HCW-1:0] band_cur;           // 当前写入band号(0..HC-1)
reg           fill_active;
reg [1:0]     band_filled;        // 每bank: 已写满待 drain
reg [HCW-1:0] bank_band [0:1];    // 各bank当前存放的band号
reg           frame_seen;
// 组合: 帧首拍强制坐标(0,0)/bank0/band0, 修掉多帧连续输入时残留旧坐标错位
wire px_accept   = s_axis_tvalid && s_axis_tready;
wire frame_start = px_accept && s_axis_tuser;
wire [CW-1:0] eff_col = frame_start ? {CW{1'b0}}  : cur_col;
wire [RW-1:0] eff_row = frame_start ? {RW{1'b0}}  : cur_row;
wire         eff_bpar = frame_start ? 1'b0        : band_par;
wire         eff_bcur_ = frame_start ? 1'b0       : 1'b1;
wire [HCW-1:0] eff_bcur = frame_start ? {HCW{1'b0}} : band_cur;
wire band_last  = (band_cnt == BAND_DEPTH-1);
wire frame_last = (eff_row == IMG_H-1) && (eff_col == IMG_W-1);
wire px_tail    = band_last || frame_last;
wire wen = (fill_active || frame_start) && px_accept;
// P1: drain 状态声明前置(写侧块通过 w_drain_req* 引用 drain_on)。
// band_filled 单驱动 —— drain FSM 取走 bank 时不再直接清零 band_filled,
// 改为输出组合请求信号, 由写侧块统一清零(消除"写侧块 + drain 块双驱动"的
// multi-driven 综合错误)。优先级与 drain FSM 内 if/else 一致: bank0 优先。
reg        drain_on;
reg [1:0]  draining;             // 每bank: 正被 drain 读出(保护写侧背压)
wire w_drain_req0 = !drain_on && band_filled[0];
wire w_drain_req1 = !drain_on && !band_filled[0] && band_filled[1];
// 写侧时序: px_cnt/cur_row/cur_col/band_cnt 同块同步推进(无单拍错位);
// band_cnt 帧首初始化为 1(像素0的band内序号为0, 下一像素为1), 修掉旧版
// "每band第一像素写进上一bank地址0"的差一破坏。
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		px_cnt <= {PW{1'b0}}; cur_col <= {CW{1'b0}}; cur_row <= {RW{1'b0}};
		band_cnt <= {BCW{1'b0}}; band_par <= 1'b0; band_cur <= {HCW{1'b0}};
		fill_active <= 1'b0; band_filled <= 2'b00;
		bank_band[0] <= {HCW{1'b0}}; bank_band[1] <= {HCW{1'b0}};
	end else if (frame_start) begin
		px_cnt <= 1'b1;
		cur_col <= (IMG_W == 1) ? {CW{1'b0}} : {{(CW-1){1'b0}},1'b1};
		cur_row <= {RW{1'b0}};
		band_cnt <= {{(BCW-1){1'b0}},1'b1};
		band_par <= 1'b0; band_cur <= {HCW{1'b0}};
		fill_active <= 1'b1; band_filled <= 2'b00;
	end else if (fill_active && px_accept) begin
		if (px_tail) begin
			band_filled[band_par] <= 1'b1;
			bank_band[band_par]   <= band_cur;
			band_cnt <= {BCW{1'b0}};
			band_par <= ~band_par;
			band_cur <= (band_cur == HC-1) ? {HCW{1'b0}} : band_cur + 1;
		end else
			band_cnt <= band_cnt + 1;
		if (frame_last)
			fill_active <= 1'b0;
		else begin
			px_cnt <= px_cnt + 1;
			if (cur_col == IMG_W-1) begin cur_col <= {CW{1'b0}}; cur_row <= cur_row + 1; end
			else                    cur_col <= cur_col + 1;
		end
	end
	// P1: drain FSM 取走 bank 的清零统一在本块执行(单驱动)。独立 if 与 fill
	// 分支同拍共存: drain 请求的 bank 与写侧正填的 bank 必不相同(drain 请求
	// bank0 时 band_filled[0]=1 已置位, 写侧 band_par 已翻转为 1), 无同拍同 bit
	// 竞争; frame_start 分支的 band_filled<=2'b00 与之同值, 亦无冲突。
	if (w_drain_req0) band_filled[0] <= 1'b0;
	if (w_drain_req1) band_filled[1] <= 1'b0;
end
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) frame_seen <= 1'b0;
	else if (frame_start) frame_seen <= 1'b1;
end
// ==================== SIMILARITY_LUT (line2Dup.cpp 第737行, 8×32字节) ====================
reg [7:0] SIM [0:255];
initial begin
	$readmemh("similarity_lut.mem", SIM);
end
// ==================== 查表级: 16路并行LUTRAM读出 ====================
wire [3:0] w_lsb = s_axis_tdata[3:0];
wire [3:0] w_msb = s_axis_tdata[7:4];
wire [7:0] resp [0:7];
genvar gi;
generate
	for (gi=0; gi<8; gi=gi+1) begin: GEN_RESP
		assign resp[gi] = (SIM[gi*32 + w_lsb] > SIM[gi*32 + 16 + w_msb])
		                 ? SIM[gi*32 + w_lsb] : SIM[gi*32 + 16 + w_msb];
	end
endgenerate
// ==================== 写数据/写地址(由 eff_row/eff_col 组合派生) ====================
wire [63:0] w_wrdata = {resp[7],resp[6],resp[5],resp[4],resp[3],resp[2],resp[1],resp[0]};
wire [BAW-1:0] w_wraddr = ((eff_row & (T-1)) * IMG_W) + eff_col;   // T为2的幂, %T==&(T-1)
// ==================== 写请求FIFO 寄存器(声明前置: req_full 组合逻辑在此段引用) ====================
// 每项 {slot(RIDXW), ori[2:0](3), block[5:0](6), band[HCW-1:0](HCW)} = PFW 位
reg [PFW-1:0] req_fifo [0:REQ_FIFO_DEPTH-1];
reg [QW-1:0]  req_wr, req_rd;
reg [QW-1:0]  req_cnt;
reg [PFW-1:0] w_req_r;
reg           wr_pop;
// ==================== 读侧控制: 块序扫描(gy,gx,cx), 由 band_filled 触发 ====================
reg        drain_bank;
reg [HCW-1:0] drain_band;
reg [`SBM_W(T)-1:0] rd_gy, rd_gx;
reg [CXW-1:0] rd_cx;
reg [DCW-1:0] drain_cnt;
// 高水位节流(F5a 修正): 停摆阈值 = 一组(8 项)。停摆后最多 2 拍在途 push(+16 项),
// 峰值 req_cnt = 8+16 = 24 <= DEPTH-8, push 侧门控(req_cnt<=DEPTH-8)数学上不会触发。
// 段槽不变量: push 前 req_cnt<=DEPTH-8, 故某槽请求 pop 前最多再有 2 组新段 push,
// SEG_BUFS=4 槽轮转下该槽不会被提前重写(第 4 组 push 被门控挡住)。停摆越久槽越安全。
wire       req_full = (req_cnt >= 8);
wire       drain_go = drain_on && !req_full;
wire [BAW-1:0] w_rdaddr = rd_gy * IMG_W + rd_cx * T + rd_gx;
// 写侧背压: 待写 bank 若正被 drain 读取或已写满未 drain, 则暂停接收新像素
// P0-1: 帧首拍强制写 bank0(eff_bpar=0), tready 必须按帧首实际写入 bank 检查。
// 帧尾 band_par=1, 若仍按 band_par 查 bank1, 上一帧末 band(偶数, bank0) 尚在
// drain(draining[0]=1) 时新帧首像素会被放行并写 bank0 地址 0 → 与 drain 读
// 并发, 新旧帧数据混入同一响应图。
wire frame_head = s_axis_tvalid && s_axis_tuser;   // 帧首拍(与 tready 无关, 避 免组合环)
wire tready_bpar = frame_head ? 1'b0 : band_par;   // 帧首将写 bank0, 须检查 bank0
assign s_axis_tready = !(draining[tready_bpar] || band_filled[tready_bpar]);
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		drain_on <= 1'b0; drain_bank <= 1'b0; drain_band <= {HCW{1'b0}};
		rd_gy <= {`SBM_W(T){1'b0}}; rd_gx <= {`SBM_W(T){1'b0}}; rd_cx <= {CXW{1'b0}};
		drain_cnt <= {DCW{1'b0}}; draining <= 2'b00;
	end else begin
		if (!drain_on) begin
			if (band_filled[0]) begin
				drain_on <= 1'b1; drain_bank <= 1'b0; drain_band <= bank_band[0]; draining[0] <= 1'b1;
				rd_gy <= {`SBM_W(T){1'b0}}; rd_gx <= {`SBM_W(T){1'b0}}; rd_cx <= {CXW{1'b0}}; drain_cnt <= {DCW{1'b0}};
			end else if (band_filled[1]) begin
				drain_on <= 1'b1; drain_bank <= 1'b1; drain_band <= bank_band[1]; draining[1] <= 1'b1;
				rd_gy <= {`SBM_W(T){1'b0}}; rd_gx <= {`SBM_W(T){1'b0}}; rd_cx <= {CXW{1'b0}}; drain_cnt <= {DCW{1'b0}};
			end
		end else if (drain_go) begin
			if (drain_cnt == DRAIN_CYCLES-1) begin
				drain_on <= 1'b0; draining[drain_bank] <= 1'b0; drain_cnt <= {DCW{1'b0}};
			end else begin
				drain_cnt <= drain_cnt + 1;
				if (rd_cx == WC-1) begin
					rd_cx <= {CXW{1'b0}};
					if (rd_gx == T-1) begin
						rd_gx <= {`SBM_W(T){1'b0}};
						rd_gy <= (rd_gy == T-1) ? {`SBM_W(T){1'b0}} : rd_gy + 1;
					end else
						rd_gx <= rd_gx + 1;
				end else
					rd_cx <= rd_cx + 1;
			end
		end
	end
end
// ==================== 双bank转置存储(64bit SDPRAM: A写/B读) ====================
wire [63:0] rd_data0, rd_data1;
xpm_memory_sdpram #(
	.MEMORY_SIZE(BAND_DEPTH*64), .MEMORY_PRIMITIVE("block"),
	.WRITE_DATA_WIDTH_A(64), .READ_DATA_WIDTH_B(64),
	.READ_LATENCY_B(1), .WRITE_MODE_A("read_first")
) u_bank0 (
	.clka(clk), .ena(1'b1), .wea(wen && (eff_bpar==1'b0)),
	.addra(w_wraddr), .dina(w_wrdata),
	.clkb(clk), .enb(drain_go && (drain_bank==1'b0)), .addrb(w_rdaddr), .doutb(rd_data0)
);
xpm_memory_sdpram #(
	.MEMORY_SIZE(BAND_DEPTH*64), .MEMORY_PRIMITIVE("block"),
	.WRITE_DATA_WIDTH_A(64), .READ_DATA_WIDTH_B(64),
	.READ_LATENCY_B(1), .WRITE_MODE_A("read_first")
) u_bank1 (
	.clka(clk), .ena(1'b1), .wea(wen && (eff_bpar==1'b1)),
	.addra(w_wraddr), .dina(w_wrdata),
	.clkb(clk), .enb(drain_go && (drain_bank==1'b1)), .addrb(w_rdaddr), .doutb(rd_data1)
);
// ==================== 读数据与标签流水对齐 ====================
// rd_data_d 相对读地址有 2 拍延迟(SDPRAM 1拍 + 寄存器1拍), 标签需同延迟
reg [63:0] rd_data_d;
reg [5:0]  rd_block_d;           // gy*T+gx
reg [HCW-1:0] rd_band_d;
reg [CXW-1:0] rd_cx_d;
reg        rd_vld_d;
reg [5:0]  rd_block_r;
reg [HCW-1:0] rd_band_r;
reg [CXW-1:0] rd_cx_r;
reg        rd_vld_r;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin rd_block_r <= 6'd0; rd_band_r <= {HCW{1'b0}}; rd_cx_r <= {CXW{1'b0}}; rd_vld_r <= 1'b0; end
	else begin
		rd_block_r <= rd_gy * T + rd_gx;
		rd_band_r  <= drain_band;
		rd_cx_r    <= rd_cx;
		rd_vld_r   <= drain_go;
	end
end
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin rd_data_d <= 64'd0; rd_block_d <= 6'd0; rd_band_d <= {HCW{1'b0}};
		rd_cx_d <= {CXW{1'b0}}; rd_vld_d <= 1'b0; end
	else begin
		rd_data_d  <= drain_bank ? rd_data1 : rd_data0;
		rd_block_d <= rd_block_r;
		rd_band_d  <= rd_band_r;
		rd_cx_d    <= rd_cx_r;
		rd_vld_d   <= rd_vld_r;
	end
end
// ==================== 8方向×SEG_BUFS段缓冲(每段SEG_LEN字节, 旋转槽号) ====================
integer oi;
reg [7:0] seg_buf [0:7][0:SEG_BUFS-1][0:SEG_LEN-1];
reg [RIDXW-1:0] seg_slot;        // 各方向当前填充槽号(共享旋转)
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		seg_slot <= {RIDXW{1'b0}};
	end else if (rd_vld_d) begin
		for (oi=0; oi<8; oi=oi+1)
			seg_buf[oi][seg_slot][rd_cx_d] <= rd_data_d[oi*8 +: 8];
		if (rd_cx_d == WC-1)
			seg_slot <= (seg_slot == SEG_BUFS-1) ? {RIDXW{1'b0}} : seg_slot + 1;
	end
end
// ==================== 写请求FIFO 时序: push 8项/pop 1项 合并计数 ====================
// 指针/计数位宽 QW 按深度派生; 索引取模, 修掉旧版 6bit 指针越界返回 x 的缺陷
wire          req_empty = (req_cnt == {QW{1'b0}});
reg           push_en, pop_en;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin req_wr <= {QW{1'b0}}; req_rd <= {QW{1'b0}}; req_cnt <= {QW{1'b0}}; w_req_r <= {PFW{1'b0}}; end
	else begin
		push_en = rd_vld_d && (rd_cx_d == WC-1) && (req_cnt <= REQ_FIFO_DEPTH - 8);
		if (rd_vld_d && (rd_cx_d == WC-1) && (req_cnt > REQ_FIFO_DEPTH - 8))
			$error("alg9: push 容量门控触发(drain 节流失效?), req_cnt=%0d", req_cnt);
		pop_en  = wr_pop && !req_empty;
		if (push_en) begin
			for (oi=0; oi<8; oi=oi+1)
				req_fifo[(req_wr + oi) % REQ_FIFO_DEPTH] <= {seg_slot, oi[2:0], rd_block_d, rd_band_d};
			req_wr <= req_wr + 8;
		end
		if (pop_en) begin
			req_rd   <= req_rd + 1;
			w_req_r  <= req_fifo[req_rd % REQ_FIFO_DEPTH];   // 取模: 指针按深度回绕(修越界x)
		end
		req_cnt <= req_cnt + (push_en ? 8 : 0) - (pop_en ? 1 : 0);
	end
end
// ==================== AXI4-MM写主机(单outstanding, SEG_BEATS拍突发/请求) ====================
// 请求地址 = RESP_BASE + (ori*T*T + block)*CELLS + band*WC
wire [PFW-1:0] w_req = w_req_r;
wire [RIDXW-1:0] w_req_slot = w_req[PFW-1 -: RIDXW];
wire [2:0]  w_req_ori  = w_req[PFW-1-RIDXW -: 3];
wire [5:0]  w_req_blk  = w_req[PFW-1-RIDXW-3 -: 6];
wire [HCW-1:0] w_req_band = w_req[HCW-1:0];
wire [31:0] w_req_addr = RESP_BASE
	+ ({23'd0, w_req_ori} * (T*T) + w_req_blk) * CELLS
	+ w_req_band * WC;
localparam WR_IDLE=3'd0, WR_AW=3'd1, WR_W=3'd2, WR_B=3'd3;
reg [2:0]  wr_state;
reg [7:0]  wr_beat;              // 0..SEG_BEATS-1 (SEG_BEATS<=256 已断言)
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		wr_state <= WR_IDLE; wr_beat <= 8'd0; wr_pop <= 1'b0;
	end else begin
		case (wr_state)
			WR_IDLE: begin
				wr_pop <= 1'b0;
				if (!req_empty) begin
					wr_pop <= 1'b1;
					wr_beat <= 8'd0;
					wr_state <= WR_AW;
				end
			end
			WR_AW: begin
				wr_pop <= 1'b0;
				if (m_axi_awready) wr_state <= WR_W;
			end
			WR_W: begin
				if (m_axi_wready) begin
					if (wr_beat == SEG_BEATS-1) wr_state <= WR_B;
					else                       wr_beat <= wr_beat + 8'd1;
				end
			end
			WR_B: begin
				if (m_axi_bvalid) wr_state <= WR_IDLE;
			end
			default: wr_state <= WR_IDLE;
		endcase
	end
end
assign m_axi_awvalid = (wr_state == WR_AW);
assign m_axi_awaddr  = w_req_addr;
assign m_axi_awlen   = SEG_BEATS - 1;
assign m_axi_awsize  = 3'd3;                  // 8B/拍
assign m_axi_awburst = 2'b01;                 // INCR
assign m_axi_wvalid  = (wr_state == WR_W);
// wdata: 末拍越界字节补 0(配合 wstrb 部分选通), 越界索引不读数组
integer wb;
reg [63:0] wdata_r;
always @(*) begin
	for (wb=0; wb<8; wb=wb+1) begin
		if (wr_beat*8 + wb < SEG_LEN)
			wdata_r[wb*8 +: 8] = seg_buf[w_req_ori][w_req_slot][wr_beat*8 + wb];
		else
			wdata_r[wb*8 +: 8] = 8'd0;
	end
end
reg [7:0] wstrb_r;
always @(*) begin
	wstrb_r = 8'hFF;
	if (wr_beat == SEG_BEATS-1)
		for (wb=0; wb<8; wb=wb+1)
			if (wr_beat*8 + wb >= SEG_LEN) wstrb_r[wb] = 1'b0;
end
assign m_axi_wdata   = wdata_r;
assign m_axi_wstrb   = wstrb_r;
assign m_axi_wlast   = (wr_state == WR_W) && (wr_beat == SEG_BEATS-1);
assign m_axi_bready  = 1'b1;
// ==================== 完成中断: 末band读完 + 全部请求写完 ====================
reg done_r;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) done_r <= 1'b0;
	else begin
		if (frame_start) done_r <= 1'b0;
		else if (!fill_active && !drain_on && (band_filled==2'b00) && req_empty && (wr_state == WR_IDLE) && frame_seen)
			done_r <= 1'b1;
	end
end
assign irq_done = done_r;
endmodule
