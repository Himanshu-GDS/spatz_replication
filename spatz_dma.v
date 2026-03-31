`timescale 1ns / 1ps
// ============================================================================
// Spatz DMA Engine -- External Memory to TCDM
// Packs 4 x 32-bit words into 1 x 128-bit TCDM word
// ============================================================================
module spatz_dma #(
    parameter DATA_W     = 128,
    parameter ADDR_W     = 17,
    parameter EXT_DATA_W = 32
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    dma_start,
    output reg                     dma_done,
    input  wire [ADDR_W-1:0]      dma_dst_addr,
    input  wire [15:0]             dma_length,
    input  wire [ADDR_W-1:0]      dma_stride,
    input  wire [15:0]             dma_src_offset,
    output reg  [15:0]             dma_ext_idx,
    input  wire [EXT_DATA_W-1:0]  dma_ext_data,
    output reg  [ADDR_W-1:0]      tcdm_addr,
    output reg  [DATA_W-1:0]      tcdm_wdata,
    output reg                     tcdm_we,
    output reg                     tcdm_req,
    input  wire                    tcdm_gnt
);

    localparam PACK_RATIO = DATA_W / EXT_DATA_W;

    localparam S_IDLE    = 3'd0;
    localparam S_SETUP   = 3'd1;
    localparam S_FETCH   = 3'd2;
    localparam S_CAPTURE = 3'd3;
    localparam S_WRITE   = 3'd4;
    localparam S_DONE    = 3'd5;

    reg [2:0]  state;
    reg [15:0] ext_cnt;
    reg [15:0] length_r;
    reg [15:0] src_off_r;
    reg [ADDR_W-1:0] dst_r;
    reg [DATA_W-1:0] acc_buf;
    reg [1:0]  sub_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            dma_done <= 0;
            tcdm_req <= 0; tcdm_we <= 0;
            sub_cnt  <= 0; ext_cnt <= 0;
            dma_ext_idx <= 0;
        end else begin
            dma_done <= 0;
            tcdm_req <= 0; tcdm_we <= 0;

            case (state)
                S_IDLE: begin
                    if (dma_start) begin
                        dst_r     <= dma_dst_addr;
                        length_r  <= dma_length;
                        src_off_r <= dma_src_offset;
                        ext_cnt   <= 0;
                        sub_cnt   <= 0;
                        acc_buf   <= {DATA_W{1'b0}};
                        state     <= S_SETUP;
                    end
                end

                S_SETUP: begin
                    if (ext_cnt >= length_r) begin
                        if (sub_cnt != 0) state <= S_WRITE;
                        else              state <= S_DONE;
                    end else begin
                        dma_ext_idx <= src_off_r + ext_cnt;
                        state <= S_FETCH;
                    end
                end

                S_FETCH: begin
                    acc_buf[sub_cnt*EXT_DATA_W +: EXT_DATA_W] <= dma_ext_data;
                    ext_cnt <= ext_cnt + 1;
                    if (sub_cnt == PACK_RATIO - 1) begin
                        sub_cnt <= 0;
                        state   <= S_WRITE;
                    end else begin
                        sub_cnt <= sub_cnt + 1;
                        state   <= S_CAPTURE;
                    end
                end

                S_CAPTURE: begin
                    dma_ext_idx <= src_off_r + ext_cnt;
                    state <= S_FETCH;
                end

                S_WRITE: begin
                    tcdm_addr  <= dst_r;
                    tcdm_wdata <= acc_buf;
                    tcdm_we    <= 1;
                    tcdm_req   <= 1;
                    dst_r      <= dst_r + (DATA_W / 8);
                    acc_buf    <= {DATA_W{1'b0}};
                    state      <= S_SETUP;
                end

                S_DONE: begin
                    dma_done <= 1;
                    state    <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule