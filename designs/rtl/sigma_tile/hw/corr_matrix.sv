`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITMO
// Engineer: BTishchuk
// 
// Create Date: 04.11.2021 11:42:09
// Design Name: 
// Module Name: sobel
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Sobel filter
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

// Assumption - each color component has 8 bits
`include "sigma_tile.svh";

module sobel#(
    parameter MATRIX_BASE_ADDR  = 32'h00100036,
    parameter FIFO_BASE_ADDR    = 99, //!
	parameter TWIDTH     = 3,   // Window width
	parameter CHNUM      = 2,
	parameter INT_LENGTH = 10,  // Accumulation length
	parameter IDWIDTH    = 8,   // Sample data width
	parameter NPIPES     = 2    // Number of pipes
)
(
     input logic clk,
     input logic rst,
     MemSplit32.Slave mif,    
     input logic acc_start_i, // Start calculation
     output logic acc_ready_o,
     output logic calc_fin_o
);

localparam MUL_NUM = CHNUM * TWIDTH;
localparam CMUL_NUM      = ( (MUL_NUM * MUL_NUM )- MUL_NUM ) / 2 + MUL_NUM;
localparam CMUL_NUM_PC   =  2;
localparam FIFO_DEPTH    = FIFO_BASE_ADDR + 4 * MUL_NUM; 
localparam ACCUM_WIDTH   = 32;
localparam CALC_LATENCY  = CMUL_NUM/2 + 2;
                                                             
logic [MUL_NUM -1:0] i_ptr_next [CMUL_NUM_PC -1:0];
logic [MUL_NUM -1:0] j_ptr_next [CMUL_NUM_PC -1:0];
logic [MUL_NUM -1:0] j_incr_sel [CMUL_NUM_PC -1:0];
logic [MUL_NUM -1:0] j_incr     [CMUL_NUM_PC -1:0];
logic [MUL_NUM -1:0] j_init     [CMUL_NUM_PC -1:0];
logic [MUL_NUM -1:0] i_incr     [CMUL_NUM_PC -1:0];
logic [MUL_NUM -1:0] i_incr_sel [CMUL_NUM_PC -1:0];
logic [MUL_NUM -1:0] i_init     [CMUL_NUM_PC -1:0];
logic [MUL_NUM -1:0] i_ptr_ff   [CMUL_NUM_PC -1:0];
logic [MUL_NUM -1:0] j_ptr_ff   [CMUL_NUM_PC -1:0];
logic [IDWIDTH -1:0] i_imag_part    [CMUL_NUM_PC -1:0];
logic [IDWIDTH -1:0] i_real_part    [CMUL_NUM_PC -1:0];
logic [IDWIDTH -1:0] j_imag_part    [CMUL_NUM_PC -1:0];
logic [IDWIDTH -1:0] j_real_part    [CMUL_NUM_PC -1:0];
logic [IDWIDTH -1:0] j_real_part_ff [CMUL_NUM_PC -1:0];
logic [IDWIDTH -1:0] j_imag_part_ff [CMUL_NUM_PC -1:0];
logic [IDWIDTH -1:0] i_imag_part_ff [CMUL_NUM_PC -1:0];
logic [IDWIDTH -1:0] i_real_part_ff [CMUL_NUM_PC -1:0];

logic [2*IDWIDTH -1:0] ac_prod     [CMUL_NUM_PC -1:0];
logic [2*IDWIDTH -1:0] bd_prod     [CMUL_NUM_PC -1:0];
logic [2*IDWIDTH -1:0] ad_prod     [CMUL_NUM_PC -1:0];
logic [2*IDWIDTH -1:0] bc_prod     [CMUL_NUM_PC -1:0];
logic [2*IDWIDTH   :0] o_real       [CMUL_NUM_PC -1:0];
logic [2*IDWIDTH   :0] o_imag       [CMUL_NUM_PC -1:0];
logic            [3:0] ram_addr;
logic                  ram_we;
logic [ACCUM_WIDTH:0]  ram_din_imag  [CMUL_NUM_PC -1:0];
logic [ACCUM_WIDTH:0]  ram_dout_imag [CMUL_NUM_PC -1:0];

logic smpl_rdy_new;
logic [IDWIDTH -1:0] smpl_rvec_ff    [MUL_NUM -1:0];
logic [IDWIDTH -1:0] smpl_ivec_ff    [MUL_NUM -1:0];
logic                smpl_vec_wr;
logic                smpl_vec_en    [MUL_NUM/2 -1:0];

assign smpl_vec_wr = mif.req & mif.we 
                    & (mif.addr >= FIFO_BASE_ADDR) 
                    & (mif.addr < (FIFO_BASE_ADDR + FIFO_DEPTH));
 
  for ( genvar ii = 0; ii < MUL_NUM/2; ii = ii + 1 ) begin : g_sample_fill
    assign smpl_vec_en[ii] = smpl_vec_wr & (ii == (mif.addr - FIFO_BASE_ADDR )); 
    
    always_ff @(posedge clk)
        if (smpl_vec_en[ii]) begin
            smpl_rvec_ff[2*ii] <= mif.data[8:0];
            smpl_ivec_ff[2*ii] <= mif.data[15:8];          
            smpl_rvec_ff[2*ii + 1] <= mif.data[23:16];
            smpl_ivec_ff[2*ii + 1] <= mif.data[31:24];   
        end
            
  end : g_sample_fill
                        
for ( genvar ii = 0; ii < CMUL_NUM_PC; ii = ii + 1 ) begin  : g_dsp_ptr

assign j_incr_sel[ii] = ( ii == 0 ) ? ( j_ptr_ff[CMUL_NUM_PC -1][MUL_NUM -1] ? 1 : j_ptr_ff[CMUL_NUM_PC -1] << 1)
                                    : ( j_ptr_next[ii - 1][MUL_NUM -1] ? 1 : j_ptr_next[ii - 1] << 1 );

assign j_incr[ii]     = ( j_incr_sel[ii] == 1 ) ? i_incr[ii] : j_incr_sel[ii];

assign j_init[ii]     = ( ii == 0 ) ? 1
                                    : ( j_incr_sel[ii] == 1 ) ? ( i_init[ii])
                                                              : ( j_init[ii -1][MUL_NUM -1] ? 1 : j_init[ii -1] << 1 );

assign j_ptr_next[ii] = smpl_rdy_new  ? j_init[ii] : j_incr[ii];

assign i_incr_sel[ii] = ( ii == 0 ) ? i_ptr_ff[CMUL_NUM_PC -1] : i_ptr_next[ii -1];

assign i_incr[ii] = ( j_incr_sel[ii] == 1 ) ? ( i_incr_sel[ii] << 1 )
                                            : i_incr_sel[ii];

assign i_init[ii] = ( ii == 0 ) ? 1
                                : (j_incr_sel[ii] == 1 ) ? (i_init[ii-1][MUL_NUM -1] ? 1 : i_init[ii-1] << 1 )
                                                         : i_init[ii -1];

assign i_ptr_next[ii] =  smpl_rdy_new ? i_init[ii] : i_incr[ii];

    always_ff @(posedge clk or posedge rst) begin
        if(rst) begin
            i_ptr_ff[ii] <= 1;
            j_ptr_ff[ii] <= 1;
        end else
             begin
                j_ptr_ff[ii] <= j_ptr_next[ii];
                i_ptr_ff[ii] <= i_ptr_next[ii];
            end
    end
    // I operand
    always_comb begin
        i_imag_part[ii] = '0;
        i_real_part[ii] = '0;
        for (integer i = 0; i < MUL_NUM; i = i + 1) begin
            i_imag_part[ii] |= {IDWIDTH{i_ptr_ff[ii][i]}} & smpl_ivec_ff[i];
            i_real_part[ii] |= {IDWIDTH{i_ptr_ff[ii][i]}} & smpl_rvec_ff[i];
        end
    end

    // J operand
    always_comb begin
        j_imag_part[ii] = '0;
        j_real_part[ii] = '0;
        for (integer i = 0; i < MUL_NUM; i = i + 1) begin
            j_imag_part[ii] |= ~( {IDWIDTH{j_ptr_ff[ii][i]}} & smpl_ivec_ff[i] ) + 1;
            j_real_part[ii] |= {IDWIDTH{j_ptr_ff[ii][i]}} & smpl_rvec_ff[i];
        end
    end

    always_ff @(posedge clk or negedge rst)
        if(rst) begin
            i_imag_part_ff[ii] <= '0;
            i_real_part_ff[ii] <= '0;
            j_imag_part_ff[ii] <= '0;
            j_real_part_ff[ii] <= '0;
        end else begin
            i_imag_part_ff[ii] <= i_imag_part[ii];
            i_real_part_ff[ii] <= i_real_part[ii];
            j_imag_part_ff[ii] <= j_imag_part[ii];
            j_real_part_ff[ii] <= j_real_part[ii];
        end 

    mut_gen_0 i_mult_ac
        (
        .CLK (clk),
        .A(i_real_part_ff[ii]),
        .B(j_real_part_ff[ii]),
        .P(ac_prod)
        );

    mut_gen_0 i_mult_bd
        (
        .CLK (clk),
        .A(i_imag_part_ff[ii]),
        .B(j_imag_part_ff[ii]),
        .P(bd_prod)
        );
    
    mut_gen_0 i_mult_ad
        (
        .CLK (clk),
        .A(i_real_part_ff[ii]),
        .B(j_imag_part_ff[ii]),
        .P(ad_prod)
        );
        
    mut_gen_0 i_mult_bc
        (
        .CLK (clk),
        .A(i_imag_part_ff[ii]),
        .B(j_real_part_ff[ii]),
        .P(bc_prod)
        );
    
    c_add_sub_0 i_bc_add_ad
        (
        .A(ad_prod),
        .B(bc_prod),
        .ADD(1'b1),
        .S(o_imag)
        );
      
    c_add_sub_0 i_ac_sub_bd
        (
        .A(ac_prod),
        .B(bd_prod),
        .ADD(1'b0),
        .S(o_real)
        );
        
   ram 
    #(
        .dat_width(32), 
        .adr_width(4),
        .mem_size (12)
      ) i_rr_ram_imag (
        .dat_i(ram_din_imag[ii]),
        .adr_i(ram_addr),
        .we_i (ram_we),
        .dat_o(ram_dout_imag[ii]),
        .clk  (clk)
      );   
      
   ram 
    #(
        .dat_width(32), 
        .adr_width(4),
        .mem_size (12)
      ) i_rr_ram_real (
        .dat_i(ram_din_imag[ii]),
        .adr_i(ram_addr),
        .we_i (ram_we),
        .dat_o(ram_dout_imag[ii]),
        .clk  (clk)
      );    
        
end :  g_dsp_ptr


       
 
endmodule
