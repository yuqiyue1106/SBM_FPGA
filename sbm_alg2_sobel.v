// ============================================================================
// 3x3 Sobel 梯度与方向计算
//
//   Gx = [-1  0  1]    Gy = [-1 -2 -1]
//        [-2  0  2]         [ 0  0  0]
//        [-1  0  1]         [ 1  2  1]
//
// 输出：
//   mag2 = dx^2 + dy^2，22 bit
//   angle = CORDIC 归一化相位，16 bit，覆盖完整的 +/-pi 区间
//
// 边界采用 BORDER_REPLICATE：首行双写两个行缓冲，行首复制第 0 列，
// 行尾插入 1 个复制像素，帧尾插入 1 行复制像素。窗口标签 (r,c) 对应
// 输出中心 (r-1,c-1)，因此丢弃 r=0 或 c=0 的窗口后，每帧输出
// img_w * img_h 个像素。
// ============================================================================
`include "sbm_geometry.vh"

module sbm_alg2_sobel #(
	// 必须与 CORDIC IP 生成报告中的真实 latency 一致。
	parameter CORDIC_IP_LATENCY = 21
)(
	input  wire        clk,            // 模块工作时钟
	input  wire        rst_n,          // 异步低有效复位

	// 图像尺寸
	input  wire [12:0] img_w,          // 图像有效宽度（像素）
	input  wire [12:0] img_h,          // 图像有效高度（行）

	// 输入像素流
	input  wire        s_axis_tvalid,  // 输入像素有效
	output wire        s_axis_tready,  // 模块可接收输入像素
	input  wire [7:0]  s_axis_tdata,   // 输入灰度像素
	input  wire        s_axis_tuser,   // 输入帧首像素标志
	input  wire        s_axis_tlast,   // 输入行末像素标志

	// Sobel 结果流。内部流水线不支持反压，m_axis_tready 仅保留接口兼容性。
	output wire        m_axis_tvalid,  // 输出结果有效
	input  wire        m_axis_tready,  // 保留的下游就绪信号，当前未使用
	output wire [21:0] m_axis_mag2,    // 梯度幅值平方 dx^2+dy^2
	output wire [15:0] m_axis_angle,   // CORDIC 梯度方向角
	output wire        m_axis_tuser,   // 输出帧首像素标志
	output wire        m_axis_tlast    // 输出行末像素标志
);

localparam LINEBUF_MAX_W = 8192;                   // 行缓冲支持的最大图像宽度
localparam CORDIC_LAT    = CORDIC_IP_LATENCY - 1; // 幅值与坐标延迟线深度

// 输入、边界扩展与坐标跟踪
reg  [7:0]  s_axis_tdata_d;  // 握手后的输入像素，延迟 1 拍
reg         s_axis_tvalid_d; // 握手后的输入有效，延迟 1 拍
reg         s_axis_tuser_d;  // 输入帧首标志，延迟 1 拍
reg         s_axis_tlast_d;  // 输入行末标志，延迟 1 拍
reg  [12:0] s_row;           // 内部当前行坐标，包含帧尾复制行
reg  [12:0] s_col;           // 内部当前列坐标，包含行尾复制列
reg         row_active;      // 正在接收真实图像行
reg         frame_end;       // 帧尾冲刷状态记录，仅用于调试观察
reg  [1:0]  flush_c;         // 行尾复制像素剩余拍数
reg         flush_row;       // 帧尾复制行进行中
reg  [7:0]  last_pix;        // 当前行最后一个真实像素
wire        w_in_valid;      // 含真实像素和边界补拍的内部有效
wire [7:0]  w_in_data;       // 边界扩展后的内部像素
wire        w_preload;       // 首行双写行缓冲标志
wire        w_flush_col;     // 当前周期为行尾复制列

// 行缓冲写事务和读数据
wire [12:0] w_lb_addr;       // 行缓冲当前读地址
reg  [12:0] w_lb_rd_addr;    // 预留的读地址寄存器，当前未使用
reg  [12:0] w_lb_wr_addr;    // 延迟 1 拍的行缓冲写地址
reg  [7:0]  w_wr_data;       // 与写地址对齐的行缓冲写数据
reg         w_wr_en0;        // 与写地址对齐的缓冲 0 写使能
reg         w_wr_en1;        // 与写地址对齐的缓冲 1 写使能
wire        w_we0;           // 缓冲 0 原始写使能
wire        w_we1;           // 缓冲 1 原始写使能
wire [7:0]  lb_a;            // 行缓冲 0 的读数据
wire [7:0]  lb_b;            // 行缓冲 1 的读数据

// 行缓冲读流水与 3x3 窗口
reg  [7:0]  cur_d;          // 当前内部像素，延迟 1 拍
reg         row_start_d;    // 当前流水数据位于行首
reg         vld_d1;         // 行缓冲读流水有效
reg  [12:0] s_row_d;        // 与 cur_d 对齐的行坐标
reg  [12:0] s_col_d;        // 与 cur_d 对齐的列坐标
reg  [7:0]  lb_a_rs;        // 行首锁存的缓冲 0 列 0 数据
reg  [7:0]  lb_b_rs;        // 行首锁存的缓冲 1 列 0 数据
reg         pad_d1;         // 当前窗口采样对应上一拍行尾补点
reg  [12:0] s_row_r;        // 行尾补点所属的原始行号
reg  [7:0]  pad_lb_a;       // 行尾补点时锁存的缓冲 0 数据
reg  [7:0]  pad_lb_b;       // 行尾补点时锁存的缓冲 1 数据
reg  [7:0]  win [0:2][0:2]; // 3x3 像素窗口，[行][列]
reg         vld_w;          // 3x3 窗口数据有效
reg  [12:0] s_row_w;        // 与窗口对齐的行坐标
reg  [12:0] s_col_w;        // 与窗口对齐的列坐标
wire [12:0] w_row_sel;      // 行缓冲奇偶选择使用的行号
wire [7:0]  w_row_cur;      // 窗口当前行的新像素
wire [7:0]  w_live_p1;      // 实时读取的前 1 行像素
wire [7:0]  w_live_p2;      // 实时读取的前 2 行像素
wire [7:0]  w_pad_p1;       // 行尾锁存的前 1 行像素
wire [7:0]  w_pad_p2;       // 行尾锁存的前 2 行像素
wire [7:0]  w_row_p1;       // 送入窗口的前 1 行像素
wire [7:0]  w_row_p2;       // 送入窗口的前 2 行像素
wire [7:0]  w_rs_p1;        // 行首装载用的前 1 行像素
wire [7:0]  w_rs_p2;        // 行首装载用的前 2 行像素
wire [7:0]  w_rs_cur;       // 行首装载用的当前行像素

// Sobel 梯度与幅值
wire [9:0]         w_x_pos;  // Gx 正系数加权和
wire [9:0]         w_x_neg;  // Gx 负系数绝对值加权和
wire [9:0]         w_y_pos;  // Gy 正系数加权和
wire [9:0]         w_y_neg;  // Gy 负系数绝对值加权和
wire signed [11:0] w_sxp;    // 零扩展后的 Gx 正项
wire signed [11:0] w_sxn;    // 零扩展后的 Gx 负项
wire signed [11:0] w_syp;    // 零扩展后的 Gy 正项
wire signed [11:0] w_syn;    // 零扩展后的 Gy 负项
wire signed [11:0] w_dx;     // 水平 Sobel 梯度
wire signed [11:0] w_dy;     // 垂直 Sobel 梯度
reg  signed [11:0] dx_r;     // 寄存后的水平梯度
reg  signed [11:0] dy_r;     // 寄存后的垂直梯度
reg                vld_g;    // dx_r/dy_r 有效
reg  [12:0]        row_cnt_g; // 与梯度对齐的行坐标
reg  [12:0]        pix_cnt_g; // 与梯度对齐的列坐标
reg  [21:0]        mag2_r;    // 寄存后的梯度幅值平方
reg                vld_m;     // mag2_r 有效
reg  [12:0]        row_cnt_m; // 与 mag2_r 对齐的行坐标
reg  [12:0]        pix_cnt_m; // 与 mag2_r 对齐的列坐标

// CORDIC 对齐流水与输出坐标
wire                 cordic_out_valid; // CORDIC 输出方向角有效
wire [15:0]          cordic_angle;     // CORDIC 输出方向角
reg  [21:0]          mag2_dly [0:CORDIC_LAT-1]; // 幅值对齐延迟线
reg  [12:0]          row_dly  [0:CORDIC_LAT-1]; // 行坐标对齐延迟线
reg  [12:0]          pix_dly  [0:CORDIC_LAT-1]; // 列坐标对齐延迟线
reg  [CORDIC_LAT-1:0] vld_dly;         // 有效信号对齐延迟线
reg                  f2_align_err_d;   // CORDIC 对齐错误锁存
reg                  f2_align_armed;   // CORDIC 对齐检查已启动
wire                 dbg_align_vld;    // 延迟线尾端调试有效信号
wire [12:0]          out_row_cnt;      // 延迟线输出的内部行坐标
wire [12:0]          out_pix_cnt;      // 延迟线输出的内部列坐标
wire                 w_emit;           // 当前结果满足输出条件
reg  [12:0]          out_row;          // 输出流当前行计数
reg  [12:0]          out_pix;          // 输出流当前列计数

integer r; // 行循环变量
integer c; // 列循环变量
integer mag_dly_i;   // 幅值延迟线循环变量
integer coord_dly_i; // 坐标延迟线循环变量


// ----------------------------------------------------------------------------
// 1. 输入握手与一级寄存
// ----------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		s_axis_tdata_d  <= 8'd0;
		s_axis_tvalid_d <= 1'b0;
		s_axis_tuser_d  <= 1'b0;
		s_axis_tlast_d  <= 1'b0;
	end else begin
		s_axis_tdata_d  <= s_axis_tdata;
		s_axis_tvalid_d <= s_axis_tvalid && s_axis_tready;
		s_axis_tuser_d  <= s_axis_tuser;
		s_axis_tlast_d  <= s_axis_tlast;
	end
end

// 行尾边界补拍占用内部数据通路，因此该周期暂停接收上游数据。
assign s_axis_tready = (flush_c == 2'd0);

// ----------------------------------------------------------------------------
// 2. 边界扩展与内部行列坐标
// ----------------------------------------------------------------------------
// s_row 的范围为 0..img_h，末值代表帧尾复制行。
// s_col 的范围为 0..img_w，末值代表行尾复制像素。
// 帧尾复制行要求相邻帧之间至少留出 img_w+2 拍；系统上游的冲刷阶段满足
// 此条件。行尾复制像素通过 s_axis_tready 强制产生，不依赖上游主动留空。
assign w_in_valid = (s_axis_tvalid_d && (row_active || s_axis_tuser_d))
				  || (flush_c != 2'd0)
				  || flush_row;
assign w_in_data = (flush_c != 2'd0) ? last_pix : s_axis_tdata_d;
assign w_preload = (s_row == 13'd0);
assign w_flush_col = (flush_c != 2'd0) || (flush_row && (s_col == img_w));

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		row_active <= 1'b0;
	end else begin
		// 新帧起始
		if (s_axis_tvalid_d && s_axis_tuser_d) begin
			row_active <= 1'b1;
		end
		// AXI-Stream 行末：tlast 每行末拍有效；仅最后一行的行末结束帧
		else if (s_axis_tvalid_d && s_axis_tlast_d && (s_row == img_h-1)) begin
			row_active <= 1'b0;
		end
	end
end

// no use
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		frame_end <= 1'b0;
	end else begin
		if (s_axis_tvalid_d && s_axis_tuser_d)
			frame_end <= 1'b0;
		else if (s_axis_tvalid_d && s_axis_tlast_d && (s_row == img_h-1))
			frame_end <= 1'b1;
		else if (flush_row && w_in_valid && (s_col == img_w) && (s_row == img_h))
			frame_end <= 1'b0;
	end
end

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		flush_c <= 2'd0;
	end else begin
		// 消费真实行末像素后，安排一个右边界复制像素。
		if (s_axis_tvalid_d && row_active && (s_col == img_w-1)) begin
			flush_c <= 2'd1;
		end else if (flush_c != 2'd0) begin
			flush_c <= flush_c - 2'd1;
		end
	end
end


always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		flush_row <= 1'b0;
	end else begin
		if (s_axis_tvalid_d && s_axis_tuser_d) begin
			flush_row <= 1'b0;
		end
		// 最后一行的最后一个真实像素消费后，才启动帧尾复制行。
		if (s_axis_tvalid_d && s_axis_tlast_d && row_active
		    && (s_row == img_h-1)) begin
			flush_row <= 1'b1;
		end
		// 帧尾补1行结束
		if (flush_row && w_in_valid && (s_col == img_w) && (s_row == img_h)) begin
			flush_row <= 1'b0;
		end
	end
end

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		last_pix <= 8'd0;
	end else begin
		// 行末像素锁存（右边界复制源）
		if (s_axis_tvalid_d && row_active) begin
			last_pix <= s_axis_tdata_d;
		end
	end
end

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		s_row <= 13'd0;
	end else if (s_axis_tvalid && s_axis_tuser) begin
		s_row <= 13'd0;
	end else if (w_in_valid && (s_col == img_w)) begin
		s_row <= (s_row == img_h) ? 13'd0 : s_row + 13'd1;
	end
end

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		s_col <= 13'd0;
	end else if (s_axis_tvalid && s_axis_tuser) begin
		s_col <= 13'd0;
	end else if (w_in_valid) begin
		s_col <= (s_col == img_w) ? 13'd0 : s_col + 13'd1;
	end
end

// ----------------------------------------------------------------------------
// 3. 双行缓冲：端口 A 写，端口 B 读，按行号奇偶轮转
// ----------------------------------------------------------------------------
// RAM 读延迟为 1 拍。写地址、写数据和写使能整体延迟 1 拍，使三者始终属于
// 同一个写事务；读地址保持当前列。窗口级在随后一拍采样 RAM 输出。

// 写使能：补1拍/补1行不写；第0行双写（上边界复制）；其余仅当前行缓冲写
assign w_we0 = w_in_valid && !w_flush_col && !flush_row
			 && (w_preload || (s_row[0] == 1'b0));
assign w_we1 = w_in_valid && !w_flush_col && !flush_row
			 && (w_preload || (s_row[0] == 1'b1));

assign w_lb_addr = w_flush_col ? (img_w-1) : s_col;

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		w_lb_wr_addr <= 13'd0;
		w_wr_data    <= 8'd0;

		w_wr_en0     <= 1'b0;
		w_wr_en1     <= 1'b0;
	end else begin
		// 完整写事务延迟一拍：地址、数据和使能必须同级对齐。
		w_lb_wr_addr <= w_lb_addr;
		w_wr_data    <= w_in_data;

		w_wr_en0     <= w_we0;
		w_wr_en1     <= w_we1;
	end
end


xpm_memory_sdpram #(
	.MEMORY_SIZE(LINEBUF_MAX_W*8),
	.WRITE_DATA_WIDTH_A(8),
	.READ_DATA_WIDTH_B(8),
	.READ_LATENCY_B(1),
	.MEMORY_PRIMITIVE("auto")
) u_buf0 (
	.clka(clk),
	.ena(1'b1),
	.wea(w_wr_en0),
	.addra(w_lb_wr_addr),
	.dina(w_wr_data),
	.clkb(clk),
	.enb(1'b1),
	.addrb(w_lb_addr),
	.doutb(lb_a)
);

xpm_memory_sdpram #(
	.MEMORY_SIZE(LINEBUF_MAX_W*8),
	.WRITE_DATA_WIDTH_A(8),
	.READ_DATA_WIDTH_B(8),
	.READ_LATENCY_B(1),
	.MEMORY_PRIMITIVE("auto")
) u_buf1 (
	.clka(clk),
	.ena(1'b1),
	.wea(w_wr_en1),
	.addra(w_lb_wr_addr),
	.dina(w_wr_data),
	.clkb(clk),
	.enb(1'b1),
	.addrb(w_lb_addr),
	.doutb(lb_b)
);


// 消费行 r 写入缓冲 r%2。READ_FIRST 模式下，同一缓冲读出行 r-1，另一个
// 缓冲读出行 r-2。行尾补拍采样时坐标已经进入下一行，因此用 s_row_r 保留
// 补拍所属行，并锁存补拍前的 RAM 数据。

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		pad_d1 <= 1'b0;
	end else begin
		pad_d1 <= (flush_c != 2'd0);
	end
end

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		s_row_r <= 13'd0;
	end else begin
		if (s_axis_tvalid_d && row_active && (s_col == img_w-1)) begin
			s_row_r <= s_row;
		end
	end
end

// 行首锁存 RAM 的列 0 读值，供窗口整体装载使用，补偿 RAM 的 1 拍读延迟。
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		lb_a_rs <= 8'd0;
		lb_b_rs <= 8'd0;
	end else if (w_in_valid && (s_col == 13'd0)) begin
		lb_a_rs <= lb_a;
		lb_b_rs <= lb_b;
	end
end

// 行尾锁存
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		pad_lb_a <= 8'd0;
		pad_lb_b <= 8'd0;
	end else begin
		if (flush_c != 2'd0) begin
			pad_lb_a <= lb_a;
			pad_lb_b <= lb_b;
		end
	end
end

assign w_row_sel = pad_d1 ? s_row_r : s_row;



// 常规移位拍：实时 lb（read_first 写保护在消费沿成立）；补拍窗口采样拍
// 缓冲 r%2 已被行 r 整行覆盖（行 r-2 丢失），改用补拍锁存 pad_lb_*。
assign w_live_p1 = (w_row_sel[0] == 1'b0) ? lb_b : lb_a;
assign w_live_p2 = (w_row_sel[0] == 1'b0) ? lb_a : lb_b;

assign w_pad_p1 = (w_row_sel[0] == 1'b0) ? pad_lb_b : pad_lb_a;
assign w_pad_p2 = (w_row_sel[0] == 1'b0) ? pad_lb_a : pad_lb_b;

//
assign w_row_p1 = pad_d1 ? w_pad_p1 : w_live_p1;
assign w_row_p2 = pad_d1 ? w_pad_p2 : w_live_p2;

// 只有真正的帧尾补行周期才从行缓冲复制当前行。末行的行尾补拍仍使用
// cur_d（即 last_pix），不能提前切换到行缓冲。
assign w_row_cur = (flush_row && !pad_d1 && (flush_c == 2'd0))
			 ? ((w_row_sel[0] == 1'b0) ? lb_b : lb_a)
			 : cur_d;


// 整体装载专用（列0对齐锁存值）：帧尾补行时装载行的 win[2] 也取末行复制
assign w_rs_p1 = (w_row_sel[0] == 1'b0) ? lb_b_rs : lb_a_rs;
assign w_rs_p2 = (w_row_sel[0] == 1'b0) ? lb_a_rs : lb_b_rs;
assign w_rs_cur = flush_row ? w_rs_p1 : cur_d;




// ----------------------------------------------------------------------------
// 4. 3x3 窗口：行首整体装载，其他周期向左移位
// ----------------------------------------------------------------------------

//
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		cur_d      <= 8'd0;
		row_start_d <= 1'b0;
		vld_d1     <= 1'b0;
		s_row_d    <= 13'd0;
		s_col_d    <= 13'd0;
	end else begin
		cur_d      <= w_in_data;
		row_start_d <= (s_col == 13'd0) && w_in_valid;
		vld_d1     <= w_in_valid;
		s_row_d    <= s_row;
		s_col_d    <= s_col;
	end
end


always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		for (r=0; r<3; r=r+1) begin
			for (c=0; c<3; c=c+1) begin
				win[r][c] <= 8'd0;
			end
		end
	end else if (vld_d1) begin
		// 帧尾复制行：顶行来自 H-2，中间行和底行都复制 H-1。
		if (row_start_d && flush_row) begin
			win[0][0] <= w_row_p2;
			win[0][1] <= w_row_p2;
			win[0][2] <= w_row_p2;
			win[1][0] <= w_row_p1;
			win[1][1] <= w_row_p1;
			win[1][2] <= w_row_p1;
			win[2][0] <= w_row_p1;
			win[2][1] <= w_row_p1;
			win[2][2] <= w_row_p1;
		end else if (row_start_d) begin
			// 左边界复制：三个窗口列都装入图像第 0 列。
			win[0][0] <= w_rs_p2;
			win[0][1] <= w_rs_p2;
			win[0][2] <= w_rs_p2;
			win[1][0] <= w_rs_p1;
			win[1][1] <= w_rs_p1;
			win[1][2] <= w_rs_p1;
			win[2][0] <= w_rs_cur;
			win[2][1] <= w_rs_cur;
			win[2][2] <= w_rs_cur;
		end else begin
			for (r=0;r<3;r=r+1) begin
				win[r][0] <= win[r][1];
				win[r][1] <= win[r][2];
			end
			win[0][2] <= w_row_p2;          // 行y-2
			win[1][2] <= w_row_p1;          // 行y-1
			win[2][2] <= w_row_cur;         // 行y（帧尾补1行时为末行复制）
		end
	end
end

// 窗口输出有效信号
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		vld_w <= 1'b0;
	end else begin
		vld_w <= vld_d1;
	end
end

// 与窗口像素同拍的行列坐标标签
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		s_row_w <= 13'd0;
		s_col_w <= 13'd0;
	end else if (vld_d1) begin
		s_row_w <= s_row_d;
		s_col_w <= s_col_d;
	end
end
// ----------------------------------------------------------------------------
// 5. Sobel 梯度与幅值平方
// ----------------------------------------------------------------------------
// 系数 2 使用左移实现，四项和的最大值为 4*255=1020。
assign w_x_pos = win[0][2] + {win[1][2], 1'b0} + win[2][2];
assign w_x_neg = win[0][0] + {win[1][0], 1'b0} + win[2][0];
assign w_y_pos = win[2][0] + {win[2][1], 1'b0} + win[2][2];
assign w_y_neg = win[0][0] + {win[0][1], 1'b0} + win[0][2];

// 先零扩展再执行有符号减法，避免 10 bit 无符号值的最高位被当作符号位。
assign w_sxp = {2'b00, w_x_pos};
assign w_sxn = {2'b00, w_x_neg};
assign w_syp = {2'b00, w_y_pos};
assign w_syn = {2'b00, w_y_neg};

assign w_dx = w_sxp - w_sxn;
assign w_dy = w_syp - w_syn;

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		dx_r      <= 12'd0;
		dy_r      <= 12'd0;

		vld_g     <= 1'b0;
		row_cnt_g <= 13'd0;
		pix_cnt_g <= 13'd0;
	end else begin
		dx_r      <= w_dx;
		dy_r      <= w_dy;

		vld_g     <= vld_w;
		row_cnt_g <= s_row_w;
		pix_cnt_g <= s_col_w;
	end
end

// 两个平方运算分别映射到 DSP。
(* use_dsp = "yes" *) wire signed [23:0] w_dx2 = dx_r * dx_r; // 水平梯度平方
(* use_dsp = "yes" *) wire signed [23:0] w_dy2 = dy_r * dy_r; // 垂直梯度平方

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		mag2_r   <= 21'd0;

		vld_m    <= 1'b0;
		row_cnt_m <= 13'd0;
		pix_cnt_m <= 13'd0;
	end else begin
		mag2_r   <= w_dx2[21:0] + w_dy2[21:0];
		
		vld_m    <= vld_g;
		row_cnt_m <= row_cnt_g;
		pix_cnt_m <= pix_cnt_g;
	end
end

// ----------------------------------------------------------------------------
// 6. CORDIC 方向计算
// ----------------------------------------------------------------------------
// IP 配置：ArcTan、12 bit 有符号输入、16 bit Signed Fraction 输出、全流水。
// PG105 规定 TDATA 位序为 {X_IN, Y_IN}，这里对应 {dx, dy}。
cordic_atan2 u_cordic (
	.aclk(clk),
	.s_axis_cartesian_tvalid(vld_g),
	.s_axis_cartesian_tdata({dx_r, dy_r}),
	.m_axis_dout_tvalid(cordic_out_valid),
	.m_axis_dout_tdata(cordic_angle)
);

// ----------------------------------------------------------------------------
// 7. 幅值、坐标与 CORDIC 输出对齐
// ----------------------------------------------------------------------------
// mag2_r 相对 CORDIC 输入 vld_g 已晚 1 拍，因此后续延迟线深度取
// CORDIC_IP_LATENCY-1。延迟线尾端应与 cordic_out_valid 严格同拍。
generate
	if (CORDIC_IP_LATENCY < 2) begin : gen_invalid_cordic_latency
		initial begin
			$error("F2: CORDIC_IP_LATENCY must be >= 2 (got %0d); it MUST equal the real CORDIC IP latency",
			       CORDIC_IP_LATENCY);
		end
	end
	if (CORDIC_LAT < 1) begin : gen_invalid_delay_line
		initial begin
			$error("F2: delay line depth CORDIC_LAT must be >= 1");
		end
	end
endgenerate

// 幅值数据延迟线
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		for (mag_dly_i=0; mag_dly_i<CORDIC_LAT; mag_dly_i=mag_dly_i+1) begin
			mag2_dly[mag_dly_i] <= 22'd0;
		end
	end else begin
		mag2_dly[0] <= mag2_r;
		for (mag_dly_i=1; mag_dly_i<CORDIC_LAT; mag_dly_i=mag_dly_i+1) begin
			mag2_dly[mag_dly_i] <= mag2_dly[mag_dly_i-1];
		end
	end
end

// 行列坐标延迟线
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		for (coord_dly_i=0; coord_dly_i<CORDIC_LAT; coord_dly_i=coord_dly_i+1) begin
			row_dly[coord_dly_i] <= 13'd0;
			pix_dly[coord_dly_i] <= 13'd0;
		end
	end else begin
		row_dly[0]  <= row_cnt_m;
		pix_dly[0]  <= pix_cnt_m;
		for (coord_dly_i=1; coord_dly_i<CORDIC_LAT; coord_dly_i=coord_dly_i+1) begin
			row_dly[coord_dly_i] <= row_dly[coord_dly_i-1];
			pix_dly[coord_dly_i] <= pix_dly[coord_dly_i-1];
		end
	end
end

// 有效信号延迟线
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		vld_dly <= 0;
	end else begin
		vld_dly <= {vld_dly[CORDIC_LAT-2:0], vld_m};
	end
end

// 保留该信号，便于联合仿真直接观察延迟线尾端有效信号。
assign dbg_align_vld = vld_dly[CORDIC_LAT-1];

// 运行期对齐自检：首次有效输出后启动检查，避开启动阶段的 X。
always @(posedge clk) begin
	if (!rst_n) begin
		f2_align_armed <= 1'b0;
	end else if (cordic_out_valid && !f2_align_armed) begin
		f2_align_armed <= 1'b1;
	end
end

// 对齐错误独立锁存，仅首次不匹配时打印错误。
always @(posedge clk) begin
	if (!rst_n) begin
		f2_align_err_d <= 1'b0;
	end else begin
		if (f2_align_armed && cordic_out_valid !== vld_dly[CORDIC_LAT-1]) begin
			if (!f2_align_err_d) begin
				$error("F2 ALIGN MISMATCH: cordic_out_valid != mag2 delay-line valid (CORDIC_IP_LATENCY=%0d wrong)",
				       CORDIC_IP_LATENCY);
			end
			f2_align_err_d <= 1'b1;
		end
	end
end

// ----------------------------------------------------------------------------
// 8. 输出中心对齐与帧坐标
// ----------------------------------------------------------------------------
// 窗口 (r,c) 对应输出像素 (r-1,c-1)，仅发射 r>=1 且 c>=1 的结果。
assign out_row_cnt = row_dly[CORDIC_LAT-1];
assign out_pix_cnt = pix_dly[CORDIC_LAT-1];
assign w_emit = cordic_out_valid
		     && (out_row_cnt >= 13'd1)
		     && (out_pix_cnt >= 13'd1);

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		out_row <= 13'd0;
		out_pix <= 13'd0;
	end else if (w_emit) begin
		if (out_pix == img_w-1) begin
			out_pix <= 13'd0;
			if (out_row == img_h-1) begin
				out_row <= 13'd0;
			end else begin
				out_row <= out_row + 13'd1;
			end
		end else begin
			out_pix <= out_pix + 13'd1;
		end
	end
end

assign m_axis_tvalid = w_emit;
assign m_axis_mag2   = mag2_dly[CORDIC_LAT-1];
assign m_axis_angle  = cordic_angle;
assign m_axis_tuser  = w_emit && (out_row == 13'd0) && (out_pix == 13'd0);
assign m_axis_tlast  = w_emit && (out_pix == img_w-1);

endmodule
