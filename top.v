`timescale 1ns / 1ps
//============================================================
// top.v  -  Basys3 電子鎖
//
// 使用 PicoRV32 (RV32I) CPU 核心
//
// 記憶體映射：
//   0x00000000 ~ 0x000003FF  ROM (256 x 32-bit, 存 firm.mem)
//   0x10000000               IO 讀  SW[15:0]
//   0x10000004               IO 讀  BTNC
//   0x10000008               IO 寫  display_mode (0=待機, 1=PASS, 2=Err)
//
// 密碼: SW = 16'h1234
//   SW[15:12] = 1,  SW[11:8] = 2,  SW[7:4] = 3,  SW[3:0] = 4
//
// 按鍵:
//   btnC (Center) = 確認輸入
//   btnU (Up)     = 重置 (active-high → resetn active-low)
//
// 七段顯示器:
//   an[3:0]  低有效
//   seg[7:0] 低有效  bit[7]=DP
//============================================================

module top (
    input  wire        clk,
    input  wire        btnC,   // 確認鍵
    input  wire        btnU,   // 重置鍵

    input  wire [15:0] sw,

    output reg  [3:0]  an,
    output reg  [7:0]  seg
);

    //------------------------------------------------------------
    // Reset  (PicoRV32 需要 active-low resetn)
    //------------------------------------------------------------
    wire resetn;
    assign resetn = ~btnU;

    //------------------------------------------------------------
    // ROM - 256 words, 初始化自 firm.mem
    //------------------------------------------------------------
    reg [31:0] rom [0:255];
    initial $readmemh("firm.mem", rom);

    //------------------------------------------------------------
    // PicoRV32 原生記憶體匯流排
    //------------------------------------------------------------
    wire        mem_valid;
    wire        mem_instr;
    wire        mem_ready;
    wire [31:0] mem_addr;
    wire [31:0] mem_wdata;
    wire [ 3:0] mem_wstrb;
    wire [31:0] mem_rdata;

    //------------------------------------------------------------
    // display_mode 暫存器  (由 CPU 寫入)
    //------------------------------------------------------------
    reg [1:0] display_mode;

    //------------------------------------------------------------
    // 記憶體解碼
    //------------------------------------------------------------
    wire sel_rom  = mem_valid && (mem_addr[31:10] == 22'd0);
    wire sel_io   = mem_valid && (mem_addr[31:16] == 16'h1000);

    // ROM 讀取 (1-cycle ready)
    wire [31:0] rom_rdata = rom[mem_addr[9:2]];

    // IO 讀取
    wire [31:0] io_rdata;
    assign io_rdata =
        (mem_addr[3:0] == 4'h0) ? {16'd0, sw}    :  // 0x10000000 → SW
        (mem_addr[3:0] == 4'h4) ? {31'd0, btnC}  :  // 0x10000004 → BTNC
                                   32'd0;

    // mem_rdata 多工
    assign mem_rdata =
        sel_rom ? rom_rdata :
        sel_io  ? io_rdata  :
                  32'd0;

    // mem_ready：ROM 或 IO 皆單週期
    assign mem_ready = sel_rom | sel_io;

    // IO 寫入
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            display_mode <= 2'd0;
        end else if (sel_io && (mem_wstrb != 4'd0)) begin
            if (mem_addr[3:0] == 4'h8)           // 0x10000008 → display_mode
                display_mode <= mem_wdata[1:0];
        end
    end

    //------------------------------------------------------------
    // PicoRV32 核心例化
    //------------------------------------------------------------
    picorv32 #(
        .ENABLE_COUNTERS    (0),
        .ENABLE_COUNTERS64  (0),
        .ENABLE_REGS_16_31  (1),
        .ENABLE_REGS_DUALPORT(1),
        .CATCH_MISALIGN     (0),
        .CATCH_ILLINSN      (0),
        .ENABLE_MUL         (0),
        .ENABLE_DIV         (0),
        .ENABLE_IRQ         (0),
        .PROGADDR_RESET     (32'h0000_0000),
        .STACKADDR          (32'h0000_03FC)
    ) u_cpu (
        .clk        (clk),
        .resetn     (resetn),
        .trap       (),

        .mem_valid  (mem_valid),
        .mem_instr  (mem_instr),
        .mem_ready  (mem_ready),

        .mem_addr   (mem_addr),
        .mem_wdata  (mem_wdata),
        .mem_wstrb  (mem_wstrb),
        .mem_rdata  (mem_rdata),

        // Look-Ahead (未使用)
        .mem_la_read  (),
        .mem_la_write (),
        .mem_la_addr  (),

        // PCPI (未使用)
        .pcpi_wr    (1'b0),
        .pcpi_rd    (32'd0),
        .pcpi_wait  (1'b0),
        .pcpi_ready (1'b0),

        // IRQ (未使用)
        .irq        (32'd0),
        .eoi        ()
    );

    //------------------------------------------------------------
    // 七段掃描時脈
    // 100 MHz / 2^17 ≈ 763 Hz → 每位約 191 Hz，人眼感覺不閃爍
    //------------------------------------------------------------
    reg [16:0] scan_div;
    always @(posedge clk or negedge resetn)
        if (!resetn) scan_div <= 17'd0;
        else         scan_div <= scan_div + 1'd1;

    wire [1:0] scan_sel = scan_div[16:15];

    //------------------------------------------------------------
    // 決定各位顯示字元
    //------------------------------------------------------------
    reg [7:0] digit3, digit2, digit1, digit0;

    always @(*) begin
        case (display_mode)

            2'd0: begin  // 待機：顯示目前 SW 值 (HEX)
                digit3 = sw[15:12];
                digit2 = sw[11:8];
                digit1 = sw[7:4];
                digit0 = sw[3:0];
            end

            2'd1: begin  // 成功：PASS
                digit3 = 8'h0A;  // P
                digit2 = 8'h0B;  // A
                digit1 = 8'h0C;  // S
                digit0 = 8'h0C;  // S
            end

            2'd2: begin  // 失敗：Err_
                digit3 = 8'h0D;  // E
                digit2 = 8'h0E;  // r
                digit1 = 8'h0E;  // r
                digit0 = 8'h0F;  // (blank)
            end

            default: begin
                digit3 = 8'h00;
                digit2 = 8'h00;
                digit1 = 8'h00;
                digit0 = 8'h00;
            end
        endcase
    end

    //------------------------------------------------------------
    // 掃描多工：選擇當前 digit
    //------------------------------------------------------------
    reg [7:0] cur_char;

    always @(*) begin
        case (scan_sel)
            2'b00: begin an = 4'b1110; cur_char = digit0; end
            2'b01: begin an = 4'b1101; cur_char = digit1; end
            2'b10: begin an = 4'b1011; cur_char = digit2; end
            default: begin an = 4'b0111; cur_char = digit3; end
        endcase
    end

    //------------------------------------------------------------
    // 七段字型 (低有效，bit7 = DP 常暗)
    //
    //  Segments:  seg[6:0] = {g,f,e,d,c,b,a}
    //  seg[7] = DP (小數點，固定不亮 = 1)
    //
    //  0→3: 數字 0~9
    //  A: P   B: A   C: S   D: E   E: r   F: blank
    //------------------------------------------------------------
    always @(*) begin
        case (cur_char)
            8'h0: seg = 8'b1100_0000; // 0
            8'h1: seg = 8'b1111_1001; // 1
            8'h2: seg = 8'b1010_0100; // 2
            8'h3: seg = 8'b1011_0000; // 3
            8'h4: seg = 8'b1001_1001; // 4
            8'h5: seg = 8'b1001_0010; // 5
            8'h6: seg = 8'b1000_0010; // 6
            8'h7: seg = 8'b1111_1000; // 7
            8'h8: seg = 8'b1000_0000; // 8
            8'h9: seg = 8'b1001_0000; // 9
            8'hA: seg = 8'b1000_1100; // P
            8'hB: seg = 8'b1000_1000; // A
            8'hC: seg = 8'b1001_0010; // S
            8'hD: seg = 8'b1000_0110; // E
            8'hE: seg = 8'b1010_1111; // r
            8'hF: seg = 8'b1111_1111; // blank
            default: seg = 8'b1111_1111;
        endcase
    end

endmodule
