// ============================================================================
// @file      sbm_alg8_spread.v
// @brief     T×T 前向窗口按位或扩散
// @details
//   语义与 line2Dup.cpp spread() 逐像素一致：
//     dst(x,y) = OR_{r,c∈[0,T)} src(x+c, y+r)，越界按 0（右缘/下缘零填充）。
//   实现：因果（后向）T×T OR + 输出丢弃每行前 T-1 列 / 每帧前 T-1 行。
//     因果 OR 在 (p,q) = 前向窗口在 (p-T+1, q-T+1)（按位或交换律 + 补零），
//     故丢弃前缀后输出 = 前向窗口，尺寸与输入一致（IMG_W × IMG_H）。
//   帧间约束：两帧之间需预留 ≥ T-1 行空闲供底部补零排空（由高斯模块帧间
//     间隔与 s_axis_tready 反压共同保证）。行间 ≥ T-1 拍间隙由 s_axis_tready
//     硬件强制（flush_c!=0 时反压，见 P2 修订），上游无需自行留隙。
//   修订：IMG_W/IMG_H 默认由 2500 改为 2504，后随 F5a 改为由
//     sbm_geometry.vh 派生(2560, 8T 对齐, WC 8 对齐)。
//   修订(P0-3, 2026-08-14)：输出背压 —— m_axis_tready 接入全流水 stall，
//     下游(alg9)拉低 tready 时输入反压 + 全部计数器冻结, 不再静默丢像素；
//     同时 out_row 每帧回绕, 修复 tuser 仅复位后首帧有效的缺陷。
// @param[in]  clk          时钟
// @param[in]  rst_n        低有效异步复位
// @param[in]  s_axis_*     AXI4-Stream 从机（响应图输入）
// @param[out] m_axis_*     AXI4-Stream 主机（扩散后输出）
// @author    WorkBuddy
// @date      2026-08-14
// ============================================================================
`include "sbm_geometry.vh"
module sbm_alg8_spread #(
	parameter IMG_W = `SBM_IMG_W,    ///< 图像宽（级1, 由 sbm_geometry.vh 派生, 8T 对齐）
	parameter IMG_H = `SBM_IMG_H,    ///< 图像高
	parameter T     = 8,
	parameter LB_W  = IMG_W + T - 1  ///< 行缓冲深度（含行尾补零 T-1 拍）
)(
	input  wire       clk,
	input  wire       rst_n,
	input  wire       s_axis_tvalid,
	output wire       s_axis_tready,
	input  wire [7:0] s_axis_tdata,
	input  wire       s_axis_tuser,
	input  wire       s_axis_tlast,
	output wire       m_axis_tvalid,
	input  wire       m_axis_tready,
	output wire [7:0] m_axis_tdata,
	output wire       m_axis_tuser,
	output wire       m_axis_tlast
);

	/// T-1（行尾/底部补零拍数），用 localparam 承载避免对表达式做位选
	localparam [3:0] Tm1 = T - 1;

	// ---- 输出级寄存器与发射判定(声明提前: stall 组合逻辑被前级门控引用) ----
	reg  [7:0]  out_r;
	reg         out_vld;
	reg  [12:0] out_row_cnt;          ///< 因果流行号（延迟对齐）
	reg  [12:0] out_col_cnt;
	wire w_emit = out_vld && (out_row_cnt >= (T-1)) && (out_col_cnt >= (T-1));
	// P0-3: 输出背压 —— 下游(alg9)拉低 tready 时全流水冻结(输入反压 + 各阶段
	// 计数器保持), 恢复后从断点续传, 不再静默丢像素
	wire stall = w_emit && !m_axis_tready;

	// ==================== 输入行管理：行尾补零 T-1 拍 ====================
	reg  [12:0] pix_cnt;             ///< 0..IMG_W-1
	reg  [3:0]  flush_c;             ///< 行尾补零剩余拍（0..T-1）
	reg         frame_end;           ///< 帧末行已结束
	reg         flush_active;        ///< 底部补零期（反压保护）
	wire        w_in_valid = (s_axis_tvalid && s_axis_tready) || (flush_c != 4'd0);   // 握手消费 + 行尾补零
	wire [7:0]  w_in_data  = (flush_c != 4'd0) ? 8'd0 : s_axis_tdata;

	// P2: 行尾补零期(flush_c!=0)同样反压 —— 硬件强制行间 ≥T-1 拍间隙。
	// 旧版仅靠上游契约留隙: 若上游满速连发, 握手分支(tvalid&&tready)优先于
	// flush_c 递减, 行尾补零被下一行真实像素吞掉且 w_in_data 被恒置 0,
	// 全图静默错乱。拉低 tready 后上游被迫停, 补零拍必然排空。
	assign s_axis_tready = ~flush_active && ~stall && (flush_c == 4'd0);

	// P0-3 修复: 输入按 AXI-S 握手(tvalid && tready)消费。旧版仅凭 tvalid
	// 消费, 底部补零期(flush_active 反压 tready=0)与输出 stall 期会重复消费
	// 上游保持的同一拍像素, 新旧帧数据混入行缓冲(帧末行错位 + 后续挂死)。
	// flush_active 改由垂直级单一驱动(触发置位/回绕清零), 消除多驱动竞争。
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			pix_cnt <= 13'd0; flush_c <= 4'd0; frame_end <= 1'b0;
		end else if (!stall) begin
			if (s_axis_tvalid && s_axis_tready) begin
				if (s_axis_tuser) frame_end <= 1'b0;
				if (s_axis_tlast) frame_end <= 1'b1;
				if (pix_cnt == IMG_W-1) begin
					pix_cnt <= 13'd0;
					flush_c <= Tm1;
				end else
					pix_cnt <= pix_cnt + 13'd1;
			end else if (flush_c != 4'd0)
				flush_c <= flush_c - 4'd1;
		end
	end

	// ==================== 水平因果 T 级移位 OR ====================
	reg  [7:0] h_sh [0:T-1];
	reg        h_vld;
	integer    i;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			for (i = 0; i < T; i = i + 1) h_sh[i] <= 8'd0;
			h_vld <= 1'b0;
		end else if (!stall) begin
			if (w_in_valid) begin
				h_sh[0] <= w_in_data;
				for (i = 1; i < T; i = i + 1) h_sh[i] <= h_sh[i-1];
				h_vld <= 1'b1;
			end else
				h_vld <= 1'b0;
		end
	end

	wire [7:0] w_h_or [0:T-1];
	genvar g;
	generate
		for (g = 0; g < T; g = g + 1) begin : gen_hor
			if (g == 0) assign w_h_or[0] = h_sh[0];
			else        assign w_h_or[g] = w_h_or[g-1] | h_sh[g];
		end
	endgenerate

	reg  [7:0]  h_out;                 ///< 水平因果 OR（每行 LB_W=IMG_W+T-1 个值）
	reg         h_in_vld;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin h_out <= 8'd0; h_in_vld <= 1'b0; end
		else if (!stall) begin
			h_out    <= w_h_or[T-1];
			h_in_vld <= h_vld;
		end
	end

	// ==================== 垂直：T-1 行缓冲轮转 + 因果 OR + 底部补零 T-1 行 ====================
	// 行结构：每行 LB_W 个值；每帧 IMG_H 真实行 + T-1 补零行
	// OR 顺序无关（按位或交换律）：OR 全部 T-1 个缓冲读出口 = OR 行 v_row-1..v_row-(T-1)
	reg  [12:0] v_col;                ///< 0..LB_W-1
	reg  [12:0] v_row;                ///< 0..IMG_H+T-2（含补零行）
	reg  [3:0]  r_mod;                ///< v_row mod (T-1)（写入缓冲轮转号）
	reg  [3:0]  flush_r;              ///< 底部补零剩余行（0..T-1）
	reg  [3:0]  flush_r_d;            ///< flush_r 延迟 1 拍（驱动回绕/推进）
	reg  [3:0]  flush_r_d2;           ///< flush_r 延迟 2 拍（对齐行尾位置 70 的溢出写）
	reg  [7:0]  h_out_d;              ///< h_out 延迟 1 拍（对齐读延迟）
	reg  [12:0] v_col_d;              ///< 写地址延迟 1 拍（对齐写数据）
	reg         v_in_vld_d;           ///< v_in_vld 延迟 1 拍（对齐写地址/写数据）
	wire        v_in_vld;             ///< 垂直级有效（声明前置：被时序块读取）
	
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			v_col <= 13'd0; v_row <= 13'd0; r_mod <= 4'd0; flush_r <= 4'd0;
			flush_r_d <= 4'd0; flush_r_d2 <= 4'd0;
			h_out_d <= 8'd0; v_col_d <= 13'd0; v_in_vld_d <= 1'b0; flush_active <= 1'b0;
		end else if (!stall) begin
			h_out_d <= h_out;
			v_col_d <= v_col;
			v_in_vld_d <= v_in_vld;
			flush_r_d2 <= flush_r_d;
			// 底部补零触发：末真实行结束（frame_end 已置位）。
			// 注意不能等 v_col==LB_W-1 的回绕拍再触发：该拍 h_in_vld 依赖下一帧
			// 首像素产生（行尾补零 7 拍恰好使末行最后一拍落在 vc=70, 下一拍需输入
			// 驱动才回绕）, 最后一帧无下一帧 → 回绕缺失 → 底部补零不触发 → 冻结。
			// 提前 1 拍到推进至 LB_W-1 的拍(vc==LB_W-2, 该拍 h_in_vld=1 且 frame_end
			// 由行末像素消费拍置位, 恒成立): 触发后下一拍回绕由 flush_r 驱动。
			if (h_in_vld && (v_row == IMG_H-1) && (v_col == LB_W-2) && frame_end) begin
				flush_r <= Tm1;
				flush_active <= 1'b1;           // 补零期反压新帧
			end
			// 垂直流推进（真实行 + 补零行）：回绕拍后由 flush_r_d 驱动，
			// 不依赖下一帧首像素输入
			if (h_in_vld || (flush_r_d != 4'd0)) begin
				if (v_col == LB_W-1) begin
					v_col <= 13'd0;
					r_mod <= (r_mod == (T-1)-1) ? 4'd0 : r_mod + 4'd1;
					if (v_row == IMG_H + (T-1) - 1) begin
						v_row <= 13'd0;        // 帧处理完成，解除反压
						r_mod  <= 4'd0;
						flush_active <= 1'b0;
					end else
						v_row <= v_row + 13'd1;
					if (flush_r != 4'd0)
						flush_r <= flush_r - 4'd1;
					// frd 仅在回绕拍更新: 整行保持稳定驱动 v_in_vld/v_cur,
					// 若每拍跟随 fr, 最后补零行(行 54)首拍后 frd=0 断流冻结
					flush_r_d <= flush_r;
				end else
					v_col <= v_col + 13'd1;
			end
		end
	end

	assign v_in_vld = h_in_vld || (flush_r_d != 4'd0);
	// v_cur 用 flush_r 延迟 2 拍：行末位置 LB_W-1 的写入晚 1 拍(地址 v_col_d),
	// 该溢出写拍仍取真实行尾数据; 1 拍延迟会把它清零, 行缓冲位置 70 缺数据
	wire [7:0]  v_din    = (flush_r_d2 != 4'd0) ? 8'd0 : h_out_d;  // 写数据(位置 v_col_d)
	// OR 链当前行用 h_out(位置 v_col)而非 h_out_d: 读侧 addrb=v_col+1 提前
	// 1 拍补偿读延迟, 与写侧(h_out_d+v_col_d)对齐, 消除输出错 1 列(右缘显形)
	wire [7:0]  v_cur    = (flush_r_d2 != 4'd0) ? 8'd0 : h_out;
	// 缓冲写入：缓冲 r_mod 写入 v_cur（补零行同样写入 0，保持轮转一致）
	// 写地址 = 延迟 1 拍的列（写数据 v_cur 为 1 拍前的像素，地址与数据对齐）
	wire [12:0] w_lb_addr_w = v_col_d;
	// 读地址提前 1 拍(v_col+1, 回绕 0): 行缓冲读延迟 1 拍, 使 lb_read 与
	// v_cur(h_out) 同为位置 v_col, 消除垂直级 1 拍错位(输出右缘数据污染)
	wire [12:0] w_lb_addr_r = (v_col == LB_W-1) ? 13'd0 : v_col + 13'd1;
	wire [7:0]  lb_read [0:T-2];
	wire        lb_we   [0:T-2];

	generate
		for (g = 0; g < T-1; g = g + 1) begin : gen_vlb
			// 写使能与写地址(v_col_d)/写数据(h_out_d)同为延迟 1 拍链：
			// v_in_vld_d 覆盖行尾位置 70 的溢出写拍(下一行 vc=0, v_in_vld=0 但
			// v_in_vld_d=1)；|| v_in_vld 保留行 0 首拍(延迟链尚未建立)的正常写
			assign lb_we[g] = (v_in_vld || v_in_vld_d) && (r_mod == g[3:0]);
			xpm_memory_sdpram #(
				.MEMORY_SIZE(LB_W*8), .MEMORY_PRIMITIVE("auto"),
				.WRITE_DATA_WIDTH_A(8), .READ_DATA_WIDTH_B(8),
				.READ_LATENCY_B(1), .WRITE_MODE_A("read_first")
			) u_lb (
				.clka(clk), .ena(1'b1), .wea(lb_we[g]), .addra(w_lb_addr_w),
				.dina(v_din),
				.clkb(clk), .enb(!stall), .addrb(w_lb_addr_r), .doutb(lb_read[g])
			);
		end
	endgenerate

	// 垂直因果 OR：当前行 v_cur OR T-1 个缓冲旧行（缓冲 g 旧内容 = 行 v_row-g-1）
	wire [7:0] w_v_or [0:T-1];
	generate
		for (g = 0; g < T; g = g + 1) begin : gen_ver
			if (g == 0) assign w_v_or[0] = v_cur;
			else        assign w_v_or[g] = w_v_or[g-1] | lb_read[g-1];
		end
	endgenerate

	// ==================== 输出：丢弃每行前 T-1 列、每帧前 T-1 行（前向窗口对齐） ====================
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin out_r <= 8'd0; out_vld <= 1'b0; out_row_cnt <= 13'd0; out_col_cnt <= 13'd0; end
		else if (!stall) begin
			out_r <= w_v_or[T-1];
			out_row_cnt <= v_row;
			out_col_cnt <= v_col;
			out_vld <= v_in_vld;
		end
	end

	reg  [12:0] out_row, out_pix;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin out_row <= 13'd0; out_pix <= 13'd0; end
		else if (!stall && w_emit) begin
			if (out_pix == IMG_W-1) begin
				out_pix <= 13'd0;
				// 每帧回绕: 修复 tuser 仅复位后首帧有效(旧版 out_row 单调递增不回绕)
				out_row <= (out_row == IMG_H-1) ? 13'd0 : out_row + 13'd1;
			end else
				out_pix <= out_pix + 13'd1;
		end
	end

	assign m_axis_tvalid = w_emit;
	assign m_axis_tdata  = out_r;
	assign m_axis_tuser  = w_emit && (out_row == 13'd0) && (out_pix == 13'd0);
	assign m_axis_tlast  = w_emit && (out_pix == IMG_W-1);

endmodule
