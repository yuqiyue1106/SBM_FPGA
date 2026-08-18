// ==================================================================
// sbm_alg2_sobel.v : 3×3 Sobel梯度计算 + CORDIC方向
// Gx = [-1 0 1; -2 0 2; -1 0 1], Gy = [-1 -2 -1; 0 0 0; 1 2 1]
// 输出: mag2 = dx²+dy² (22bit, F7 由 21bit 加宽, 消除余量0的溢出风险), angle = 归一化相位(16bit, ±π 全圆)
// 边界: BORDER_REPLICATE(行首3列整体装载/行尾补1拍/帧首双写/帧尾补1行)
// 中心对齐: 因果窗口(r,c)=中心窗口(r-1,c-1)，输出丢弃第0行与每行第0列，
//   每帧输出IMG_W×IMG_H个有效像素
// 修正记录：①CORDIC输入位序由{Y,X}修正为{X_IN,Y_IN}={dx,dy}(PG105为
//   X_IN占高位)；②上边界复制preload_flag改为覆盖整个第0行(原版仅在帧首
//   预拍触发,lb2写入陈旧数据)；③左边界行首3列整体装载(原版缺失)；
//   ④帧尾补1行(原版缺失)；⑤行缓冲改用2缓冲按行号mod2轮转+端口B读取
//   (原版链式lb2.dina=lb1_out在读延迟1拍下错位,且窗口行序未随r2轮转)；
//   ⑥【N-14】w_in_valid 补入 s_axis_tuser 项：原版帧首拍被 row_active=0
//   门控丢弃，整帧左移1列，行尾补拍触发条件(s_col==IMG_W-1)永不命中，
//   右边界复制/帧尾补1行静默丢失、输出缺行且窗口行间对角错位
//   (C-golden TB2 逐拍比对捕获)。
//   ⑦【N-19】行尾补拍硬件强制：原版补拍依赖"上游行间≥1拍空闲"软契约，
//   背靠背满速输入时补拍与下一行列0同拍到达并被吞掉：补拍窗口被行首
//   装载覆盖（右边界梯度为下一行列0垃圾值）、行首标签错列+1、每行多
//   1个发射窗口致输出相位逐行对角漂移（C-golden TB2 ramp 图样捕获：
//   got=1016²+1016²，错误区左端=IMG_W-1-row）。修法照搬 alg8 P2：
//   补拍期间拉低 s_axis_tready 硬件强制行间隙；并将 tready 纳入外部
//   数据消费条件（AXI-S 握手语义：tvalid && tready 才消费）。
//   ⑧【N-20】梯度加法树符号扩展缺陷：w_x_pos/w_x_neg 为 10bit 无符号
//   （和最大 4×255=1020），原代码直接 $signed() 相减使 ≥512 的和别名成
//   负数（512→-512），亮窗口区域 dx/dy 错乱（C-golden TB2 ramp 图样捕获：
//   右端对角漂移区 got=1016²，边界恰为加法树和≥512 的像素）。修法：
//   先零扩展为 12bit 有符号再相减。
//   ⑨【N-22】末行 pad 窗口的当前行源错误：w_row_cur 以 flush_row 为条件
//   切换为末行复制 lb 读，但末行消费沿即置 flush_row=1，致使末行 pad 拍
//   与垫拍窗口采样拍的当前行误用 lb 读（应为 cur_d=last_pix）：倒数第二行
//   右端错（C-golden TB2 捕获）。修法：切换条件收紧为 flush_row && !pad_d1。
//   ⑩【N-23】帧尾补行行首装载三行全取末行复制：win[0]=行 H-2 应为 p2
//   （仅 win[1]/win[2] 为末行复制），否则末行前 2 列窗口顶行错
//   （C-golden TB2 捕获：末行 col0/1 错）。
//   ⑪【N-24】输出帧计数器帧间不复位：out_row 在每帧末像素处递增为
//   IMG_H 后不回卷，第二帧起 m_axis_tuser(=out_row==0) 永不命中且 tuser
//   帧首标记丢失（C-golden TB2 双帧协议校验捕获）。修法：末帧像素
//   (out_row==IMG_H-1 && out_pix==IMG_W-1)发射后计数器回卷归零。
// ==================================================================
`include "sbm_geometry.vh"
module sbm_alg2_sobel #(
	parameter IMG_W = `SBM_L0_W,
	parameter IMG_H = `SBM_L0_H,
	parameter CORDIC_IP_LATENCY = 21    // 全流水 ArcTan/12bit CORDIC 的真实 Latency（须等于 IP 生成报告值）
)(
	input  wire        clk,
	input  wire        rst_n,
	input  wire        s_axis_tvalid,
	output wire        s_axis_tready,
	input  wire [7:0]  s_axis_tdata,
	input  wire        s_axis_tuser,
	input  wire        s_axis_tlast,
	output wire        m_axis_tvalid,
	input  wire        m_axis_tready,
	output wire [21:0] m_axis_mag2,
	output wire [15:0] m_axis_angle,
	output wire        m_axis_tuser,
	output wire        m_axis_tlast
);
// ---------- 对齐流行列跟踪（统一坐标系） ----------
// s_row: 0..IMG_H（IMG_H=帧尾补1行）；s_col: 0..IMG_W（IMG_W=行尾补1拍）
reg [12:0] s_row, s_col;
reg        row_active;            // 帧内真实行激活
reg        frame_end;             // 帧末行指示
reg [1:0]  flush_c;               // 行尾补1拍剩余计数
reg        flush_row;             // 帧尾补1行标记
reg [7:0]  last_pix;              // 行末像素（右边界复制源）
// 消费契约：帧尾补1行依赖帧间≥ IMG_W+2 拍空闲（真实系统由 alg1 gauss_v
// 底部冲刷提供 ≥3 行帧间间隙）。行尾补1拍不再依赖上游软契约：补拍期间
// 【N-19】拉低 tready 硬件强制行间隙，任意上游节奏（含满速背靠背）下
// 补拍窗口必然完整生成。
assign s_axis_tready = (flush_c == 2'd0);   // 【N-19】补拍期间暂停接收
wire w_in_valid = (s_axis_tvalid && s_axis_tready && (row_active || s_axis_tuser))
               || (flush_c != 2'd0) || flush_row;
wire [7:0] w_in_data = (flush_c != 2'd0) ? last_pix : s_axis_tdata;
wire w_preload = (s_row == 13'd0);          // 第0行：上边界复制
wire w_flush_col = (flush_c != 2'd0) || (flush_row && (s_col == IMG_W));
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		s_row <= 13'd0; s_col <= 13'd0;
		row_active <= 1'b0; frame_end <= 1'b0; flush_c <= 2'd0;
		flush_row <= 1'b0; last_pix <= 8'd0;
	end else begin
		// 新帧起始
		if (s_axis_tvalid && s_axis_tuser) begin
			row_active <= 1'b1; frame_end <= 1'b0;
			flush_c <= 2'd0; flush_row <= 1'b0;
			s_row <= 13'd0; s_col <= 13'd0;
		end
		// 行末：帧末行标记
		if (s_axis_tvalid && s_axis_tlast) begin
			row_active <= 1'b0; frame_end <= 1'b1;
		end
		// 行尾补1拍触发（真实行末像素）：消费拍上升沿求值，s_col 为消费前
		// 值，末像素消费时恰为 IMG_W-1。（前提：帧首拍已被消费，见
		// w_in_valid 的 s_axis_tuser 项——若帧首拍丢失，流整体左移 1 列，
		// 本条件将永不命中，行尾补拍静默丢失。）
		if (s_axis_tvalid && s_axis_tready && row_active && (s_col == IMG_W-1)) begin
			flush_c <= 2'd1;
			if (s_row == IMG_H-1) flush_row <= 1'b1;   // 末行：触发帧尾补1行
		end else if (flush_c != 2'd0)
			flush_c <= flush_c - 2'd1;
		// 帧尾补1行结束
		if (flush_row && w_in_valid && (s_col == IMG_W) && (s_row == IMG_H)) begin
			flush_row <= 1'b0;
			frame_end <= 1'b0;
		end
		// 行末像素锁存（右边界复制源）
		if (s_axis_tvalid && s_axis_tready && row_active)
			last_pix <= s_axis_tdata;
		// 行列推进（含补1拍/补1行）
		if (w_in_valid) begin
			if (s_col == IMG_W) begin
				s_col <= 13'd0;
				s_row <= (s_row == IMG_H) ? 13'd0 : s_row + 13'd1;
			end else
				s_col <= s_col + 13'd1;
		end
	end
end
// ---------- 2个行缓冲（按行号mod2轮转，端口A写/端口B读） ----------
// 对齐流时序：cur_d(t+1)=w_in_data(t)=像素(r,c)、doutb(t+1)=addrb(t)=s_col(t)列的
// 旧行数据，两者在t+1拍对齐；故窗口在t+2沿采样对齐流(vld_d1)并用s_row_d/s_col_d
// 记录该像素行列(延迟1拍)。读地址必须与写同拍(s_col)：缓冲内容在消费拍
// 即被改写，read_first 仅保护同拍同址冲突，滞后读会读到新行数据。
// 【N-15】行首整体装载错列修复：lb 读流水有 1 拍延迟（t 拍呈现地址、t+1 拍
//   出口），装载沿(t+2)的 win 赋值采样的是行首消费拍(t)呈现的地址=列1 的
//   读出，超前 cur_d(列0) 一列 → 三行整体错列（C-golden TB2 ramp 图样捕获：
//   每行右端错位区逐行左移）。修法：行首消费拍(t)同沿锁存 lb 出口——该拍
//   lb 出口 = 上一拍(行间间隙/补拍收尾, s_col 已回绕为0)呈现的列0地址 +
//   read_first 写前值 = 旧行列0；装载沿(t+2)改用锁存值 lb_a_rs/lb_b_rs，
//   与 cur_d(列0) 对齐。lb_rs 每行行首刷新一次，帧首第0行的锁存值为上电
//   垃圾，但第0行窗口标签永不被发射（丢弃条件 row≥1），不影响输出。
reg  [7:0]  cur_d;               // 当前行数据延迟1拍（对齐读延迟）
reg         row_start_d;         // 行首标记延迟1拍（对齐，用于左边界复制）
reg         vld_d1;              // 输入有效延迟1拍（对齐流有效）
reg [12:0]  s_row_d, s_col_d;    // 对齐流像素行列（延迟1拍）
wire [12:0] w_lb_addr = w_flush_col ? (IMG_W-1) : s_col;   // 复制列读末列
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin cur_d <= 8'd0; row_start_d <= 1'b0; vld_d1 <= 1'b0;
		s_row_d <= 13'd0; s_col_d <= 13'd0; end
	else begin
		cur_d <= w_in_data;
		row_start_d <= (s_col == 13'd0) && w_in_valid;
		vld_d1 <= w_in_valid;
		s_row_d <= s_row;
		s_col_d <= s_col;
	end
end
wire [7:0] lb_a, lb_b;           // buf0/buf1读出口（1拍延迟）
// 写使能：补1拍/补1行不写；第0行双写（上边界复制）；其余仅当前行缓冲写
wire w_we0 = w_in_valid && !w_flush_col && !flush_row && (w_preload || (s_row[0] == 1'b0));
wire w_we1 = w_in_valid && !w_flush_col && !flush_row && (w_preload || (s_row[0] == 1'b1));
xpm_memory_sdpram #(
	.MEMORY_SIZE(IMG_W*8), .WRITE_DATA_WIDTH_A(8),
	.READ_DATA_WIDTH_B(8), .READ_LATENCY_B(1),
	.WRITE_MODE_A("read_first"), .MEMORY_PRIMITIVE("auto")
) u_buf0 (
	.clka(clk), .ena(1'b1), .wea(w_we0), .addra(w_lb_addr),
	.dina(w_in_data),
	.clkb(clk), .enb(1'b1), .addrb(w_lb_addr), .doutb(lb_a)
);
xpm_memory_sdpram #(
	.MEMORY_SIZE(IMG_W*8), .WRITE_DATA_WIDTH_A(8),
	.READ_DATA_WIDTH_B(8), .READ_LATENCY_B(1),
	.WRITE_MODE_A("read_first"), .MEMORY_PRIMITIVE("auto")
) u_buf1 (
	.clka(clk), .ena(1'b1), .wea(w_we1), .addra(w_lb_addr),
	.dina(w_in_data),
	.clkb(clk), .enb(1'b1), .addrb(w_lb_addr), .doutb(lb_b)
);
// 【N-15】行首对齐锁存：行首消费拍(s_col==0)同沿捕获 lb 出口（=上一拍呈现
// 的列0地址的写前读值），供 t+2 装载沿使用，消除读流水 1 拍超前。
reg [7:0] lb_a_rs, lb_b_rs;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin lb_a_rs <= 8'd0; lb_b_rs <= 8'd0; end
	else if (w_in_valid && (s_col == 13'd0)) begin
		lb_a_rs <= lb_a; lb_b_rs <= lb_b;
	end
end
// 窗口行数据：消费行r写入缓冲r%2，read_first 使缓冲r%2读出=行r-1、
// 缓冲(r+1)%2=行r-2。帧尾补1行(s_row==IMG_H)时"当前行"=末行复制=存末行
// 的缓冲读出口（补行不写、缓冲稳定）。
// 【N-16】补拍窗口的行奇偶修正：补拍(行尾fc=1拍)产生的右边界窗口在
//   补拍的下一拍被窗口块采样，此时 s_col/s_row 已回绕(s_row=r+1)，奇偶
//   翻转将选错缓冲（右端梯度错乱、逐行对角漂移，C-golden TB2 捕获）。
//   修法：pad_d1 指示"本采样拍对应补拍数据"，行奇偶改用补拍触发沿锁存
//   的 s_row_r(=r)；常规拍 pad_d1=0 用实时 s_row。flush_row 拍 pad_d1=0、
//   s_row=IMG_H，p1 恰选中存末行的缓冲 ✓。
reg        pad_d1;
reg [12:0] s_row_r;
reg [7:0]  pad_lb_a, pad_lb_b;   // 补拍当拍 lb 出口锁存（read_first 写前值）
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin pad_d1 <= 1'b0; s_row_r <= 13'd0;
		pad_lb_a <= 8'd0; pad_lb_b <= 8'd0; end
	else begin
		pad_d1 <= (flush_c != 2'd0);
		if (s_axis_tvalid && s_axis_tready && row_active && (s_col == IMG_W-1))
			s_row_r <= s_row;        // 行末像素消费沿：锁存补拍所属行 r
		if (flush_c != 2'd0) begin   // 补拍当拍：捕获 lb 写前读值
			pad_lb_a <= lb_a; pad_lb_b <= lb_b;
		end
	end
end
wire [12:0] w_row_sel = pad_d1 ? s_row_r : s_row;
// 【N-22】末行复制切换仅限真实补行拍：末行 pad 拍(flush_c≠0，pad_d1 尚为0)
//   与垫拍窗口采样拍(pad_d1=1)的当前行仍为 cur_d(=last_pix)；补行拍
//   (F1..FW，flush_c=0 且 pad_d1=0)才取末行缓冲读出口。
wire [7:0] w_row_cur = (flush_row && !pad_d1 && (flush_c == 2'd0)) ? ((w_row_sel[0] == 1'b0) ? lb_b : lb_a) : cur_d;
// 常规移位拍：实时 lb（read_first 写保护在消费沿成立）；补拍窗口采样拍
// 缓冲 r%2 已被行 r 整行覆盖（行 r-2 丢失），改用补拍锁存 pad_lb_*。
wire [7:0] w_live_p1 = (w_row_sel[0] == 1'b0) ? lb_b : lb_a;
wire [7:0] w_live_p2 = (w_row_sel[0] == 1'b0) ? lb_a : lb_b;
wire [7:0] w_pad_p1  = (w_row_sel[0] == 1'b0) ? pad_lb_b : pad_lb_a;
wire [7:0] w_pad_p2  = (w_row_sel[0] == 1'b0) ? pad_lb_a : pad_lb_b;
wire [7:0] w_row_p1  = pad_d1 ? w_pad_p1 : w_live_p1;   // 行r-1
wire [7:0] w_row_p2  = pad_d1 ? w_pad_p2 : w_live_p2;   // 行r-2
// 整体装载专用（列0对齐锁存值）：帧尾补行时装载行的 win[2] 也取末行复制
wire [7:0] w_rs_p1 = (w_row_sel[0] == 1'b0) ? lb_b_rs : lb_a_rs;   // 行r-1 列0
wire [7:0] w_rs_p2 = (w_row_sel[0] == 1'b0) ? lb_a_rs : lb_b_rs;   // 行r-2 列0
wire [7:0] w_rs_cur = flush_row ? w_rs_p1 : cur_d;
// ---------- 3×3窗口寄存器（行首3列整体装载=左边界复制） ----------
reg [7:0] win [0:2][0:2];
reg       vld_w;
reg [12:0] s_row_w, s_col_w;     // 窗口像素行列（与窗口数据对齐）
integer   r, c;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		for (r=0;r<3;r=r+1) for (c=0;c<3;c=c+1) win[r][c] <= 8'd0;
		vld_w <= 1'b0; s_row_w <= 13'd0; s_col_w <= 13'd0;
	end else if (vld_d1) begin      // 对齐流有效（含补拍/补行边界复制拍）
		if (row_start_d && flush_row) begin // 帧尾补行行首：win[0]=行H-2，win[1]/[2]=末行复制
			// 【N-23】原版三行全取 w_row_p1，win[0](行H-2)错误
			win[0][0] <= w_row_p2; win[0][1] <= w_row_p2; win[0][2] <= w_row_p2;
			win[1][0] <= w_row_p1; win[1][1] <= w_row_p1; win[1][2] <= w_row_p1;
			win[2][0] <= w_row_p1; win[2][1] <= w_row_p1; win[2][2] <= w_row_p1;
		end else if (row_start_d) begin   // 行首：3列整体装载（左边界复制，列0对齐）
			win[0][0] <= w_rs_p2; win[0][1] <= w_rs_p2; win[0][2] <= w_rs_p2;
			win[1][0] <= w_rs_p1; win[1][1] <= w_rs_p1; win[1][2] <= w_rs_p1;
			win[2][0] <= w_rs_cur; win[2][1] <= w_rs_cur; win[2][2] <= w_rs_cur;
		end else begin                  // 正常左移
			for (r=0;r<3;r=r+1) begin
				win[r][0] <= win[r][1];
				win[r][1] <= win[r][2];
			end
			win[0][2] <= w_row_p2;          // 行y-2
			win[1][2] <= w_row_p1;          // 行y-1
			win[2][2] <= w_row_cur;         // 行y（帧尾补1行时为末行复制）
		end
		vld_w <= 1'b1;
		s_row_w <= s_row_d;             // 对齐流像素的行（补1拍计入s_col=IMG_W）
		s_col_w <= s_col_d;
	end else
		vld_w <= 1'b0;
end
// ---------- 梯度加法树(移位实现, 0 DSP) ----------
wire [9:0]  w_x_pos = win[0][2] + {win[1][2],1'b0} + win[2][2];
wire [9:0]  w_x_neg = win[0][0] + {win[1][0],1'b0} + win[2][0];
wire [9:0]  w_y_pos = win[2][0] + {win[2][1],1'b0} + win[2][2];
wire [9:0]  w_y_neg = win[0][0] + {win[0][1],1'b0} + win[0][2];
// 【N-20】加法树和为 10bit 无符号（最大 4×255=1020），直接 $signed() 会使
//   ≥512 的和别名成负数（512→-512）：亮窗口区域 dx/dy 错乱（C-golden TB2
//   ramp 图样捕获：右端对角漂移区 got=1016²，边界恰为和≥512 的像素）。
//   修法：先零扩展为 12bit 有符号（±1020 不溢出）再相减。
wire signed [11:0] w_sxp = {2'b00, w_x_pos};
wire signed [11:0] w_sxn = {2'b00, w_x_neg};
wire signed [11:0] w_syp = {2'b00, w_y_pos};
wire signed [11:0] w_syn = {2'b00, w_y_neg};
wire signed [11:0] w_dx = w_sxp - w_sxn;  // ±1020
wire signed [11:0] w_dy = w_syp - w_syn;
reg signed [11:0] dx_r, dy_r;
reg        vld_g;
reg [12:0] row_cnt_g, pix_cnt_g;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin dx_r <= 12'd0; dy_r <= 12'd0; vld_g <= 1'b0;
		row_cnt_g <= 13'd0; pix_cnt_g <= 13'd0; end
	else begin
		dx_r <= w_dx; dy_r <= w_dy; vld_g <= vld_w;
		row_cnt_g <= s_row_w; pix_cnt_g <= s_col_w;
	end
end
// ---------- 幅值平方: 2个DSP48E2 ----------
(* use_dsp = "yes" *) wire signed [23:0] w_dx2 = dx_r * dx_r;
(* use_dsp = "yes" *) wire signed [23:0] w_dy2 = dy_r * dy_r;
reg [21:0] mag2_r;
reg        vld_m;
reg [12:0] row_cnt_m, pix_cnt_m;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin mag2_r <= 21'd0; vld_m <= 1'b0;
		row_cnt_m <= 13'd0; pix_cnt_m <= 13'd0; end
	else begin
		mag2_r <= w_dx2[21:0] + w_dy2[21:0]; vld_m <= vld_g;
		row_cnt_m <= row_cnt_g; pix_cnt_m <= pix_cnt_g;
	end
end
// ---------- CORDIC向量模式(官方IP) ----------
// 配置: ArcTan, 输入宽12bit有符号, 输出宽16bit, Signed Fraction, 全流水
// TDATA位序按PG105: {X_IN, Y_IN}（X_IN=dx占高位, Y_IN=dy占低位）
// 修正: 原版{dy,dx}打包将dx/dy交换, 角度差约90°, 已按PG105修正
wire       cordic_out_valid;
wire [15:0] cordic_angle;
cordic_atan2 u_cordic (
	.aclk(clk),
	.s_axis_cartesian_tvalid(vld_g),
	.s_axis_cartesian_tdata({dx_r, dy_r}),
	.m_axis_dout_tvalid(cordic_out_valid),
	.m_axis_dout_tdata(cordic_angle)
);
// ---------- 幅值/行列与CORDIC输出对齐（同深延迟线） ----------
// 对齐关系（以 vld_g 为时间原点）：
//   · CORDIC 输入有效 = vld_g（端口 s_axis_cartesian_tvalid）；
//     其输出有效 cordic_out_valid 在 vld_g 之后 L_ip 拍（L_ip = 真实 IP Latency）。
//   · mag2 数据路径：dx_r/dy_r 在 vld_g 周期被锁存（同块 vld_g<=vld_w），
//     mag2_r <= dx_r²+dy_r² 在 vld_m(=vld_g+1) 周期有效，再经深度 CORDIC_LAT
//     的延迟线 → mag2_dly[CORDIC_LAT-1] 在 vld_g 之后 (1 + CORDIC_LAT-1) = CORDIC_LAT 拍。
//   · vld_dly 由 vld_m 驱动，故 vld_dly[CORDIC_LAT-1] 同样在 vld_g 之后 CORDIC_LAT 拍。
//   因此必须 CORDIC_LAT == L_ip == CORDIC_IP_LATENCY，mag2 与 angle 才能逐拍对齐。
// 【F2 修复】原代码把延迟写死成 20，且错误地取 L_ip-1（漏算 vld_m 这 1 拍）。
//   一旦实际例化的 CORDIC IP 延迟不同（偏差 1 拍），幅值与相位即错位 →
//   整条匹配链静默失效。现改为由顶层参数 CORDIC_IP_LATENCY 显式传入（必须
//   等于 CORDIC IP 生成报告给出的真实 Latency），并加编译期断言 + 运行期对齐自检。
// 延迟线深度 = CORDIC IP 真实 Latency 减 1。
// 推导（以 vld_g 为原点，实测验证）：
//   · CORDIC 输出有效 cordic_out_valid 在 vld_g 之后 CORDIC_IP_LATENCY 拍；
//   · mag2 路径：vld_m 比 vld_g 慢 1 拍，mag2_r 相对 vld_m 再慢 1 拍，再经深度
//     CORDIC_LAT 的延迟线 → mag2_dly[CORDIC_LAT-1] 在 vld_g 之后 (1+1+(CORDIC_LAT-1)) = CORDIC_LAT+1 拍。
//   令 CORDIC_LAT+1 == CORDIC_IP_LATENCY ⇒ CORDIC_LAT = CORDIC_IP_LATENCY - 1。
//   （注：原审查报告 F2 建议取 L_ip，但实测该值会令幅值比相位慢 1 拍而错位；
//     正确做法是保留 L_ip-1 的推导，仅把 20 改为由 CORDIC_IP_LATENCY 派生。）
localparam CORDIC_LAT = CORDIC_IP_LATENCY - 1;    // 延迟线深度 = CORDIC IP Latency - 1
generate
	if (CORDIC_IP_LATENCY < 2)
		$error("F2: CORDIC_IP_LATENCY must be >= 2 (got %0d); it MUST equal the real CORDIC IP latency",
		       CORDIC_IP_LATENCY);
	if (CORDIC_LAT < 1)
		$error("F2: delay line depth CORDIC_LAT must be >= 1");
endgenerate
reg [21:0] mag2_dly [0:CORDIC_LAT-1];
reg [12:0] row_dly  [0:CORDIC_LAT-1];
reg [12:0] pix_dly  [0:CORDIC_LAT-1];
reg [CORDIC_LAT-1:0] vld_dly;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		for (r=0;r<CORDIC_LAT;r=r+1) begin
			mag2_dly[r] <= 22'd0; row_dly[r] <= 13'd0; pix_dly[r] <= 13'd0;
		end
		vld_dly <= 0;
	end else begin
		mag2_dly[0] <= mag2_r;
		row_dly[0]  <= row_cnt_m;
		pix_dly[0]  <= pix_cnt_m;
		for (r=1;r<CORDIC_LAT;r=r+1) begin
			mag2_dly[r] <= mag2_dly[r-1];
			row_dly[r]  <= row_dly[r-1];
			pix_dly[r]  <= pix_dly[r-1];
		end
		vld_dly <= {vld_dly[CORDIC_LAT-2:0], vld_m};
	end
end
// debug: expose delay-line tail valid for co-sim instrumentation
wire dbg_align_vld = vld_dly[CORDIC_LAT-1];
// ---------- F2 运行期对齐自检 ----------
// CORDIC 输出有效必须与延迟线末级有效 vld_dly[CORDIC_LAT-1] 严格同拍，
// 否则幅值与相位错位（F2 根因）。一旦 CORDIC_IP_LATENCY 与 IP 实际不符，
// 这里会在仿真中报错（仅首次打印，且避开 X 启动瞬态）。
reg f2_align_err_d;
reg f2_align_armed;
always @(posedge clk) begin
	if (!rst_n) begin f2_align_err_d <= 1'b0; f2_align_armed <= 1'b0; end
	else begin
		// 仅在 CORDIC 输出已脱离 X/启动瞬态（首次拉高）后才开始比对，
		// 否则流水线填充期的 X 会误触发"X !== 0"。
		if (cordic_out_valid && !f2_align_armed) f2_align_armed <= 1'b1;
		if (f2_align_armed && cordic_out_valid !== vld_dly[CORDIC_LAT-1]) begin
			if (!f2_align_err_d)
				$error("F2 ALIGN MISMATCH: cordic_out_valid != mag2 delay-line valid (CORDIC_IP_LATENCY=%0d wrong)",
				       CORDIC_IP_LATENCY);
			f2_align_err_d <= 1'b1;
		end
	end
end
// ---------- 输出中心对齐：丢弃第0行与每行第0列 ----------
// 输出像素=窗口(r,c)对应的(r-1,c-1)：r>=1且c>=1才发射（补1拍/补1行计入）
wire [12:0] out_row_cnt = row_dly[CORDIC_LAT-1];
wire [12:0] out_pix_cnt = pix_dly[CORDIC_LAT-1];
wire        w_emit = cordic_out_valid && (out_row_cnt >= 13'd1) && (out_pix_cnt >= 13'd1);
reg [12:0] out_row, out_pix;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin out_row <= 13'd0; out_pix <= 13'd0; end
	else if (w_emit) begin
		if (out_pix == IMG_W-1) begin
			out_pix <= 13'd0;
			// 【N-24】帧末像素后行计数回卷归零，保证下一帧 tuser 命中
			if (out_row == IMG_H-1) out_row <= 13'd0;
			else                    out_row <= out_row + 13'd1;
		end
		else                    out_pix <= out_pix + 13'd1;
	end
end
assign m_axis_tvalid = w_emit;
assign m_axis_mag2   = mag2_dly[CORDIC_LAT-1];
assign m_axis_angle  = cordic_angle;
assign m_axis_tuser  = w_emit && (out_row == 13'd0) && (out_pix == 13'd0);
assign m_axis_tlast  = w_emit && (out_pix == IMG_W-1);
endmodule
