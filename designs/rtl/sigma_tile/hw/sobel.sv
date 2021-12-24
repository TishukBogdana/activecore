`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITMO
// Engineer: BTishchuk
// 
// Create Date: 04.11.2021 11:42:09
// Design Name: 
// Module Name: correlation matrix calculation accelerator
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
`include "sigma_tile.svh"

module sobel#(
    parameter MATRIX_BASE_ADDR  = 32'h80000080,
    parameter FIFO_BASE_ADDR    = 32'h80000040, 
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
     output logic acc_ack_o,
     output logic acc_resp_o,
     output logic  [31:0] acc_rdata_o,

     input logic acc_on_i,    // Accelerator Start
     output logic acc_ready_o, // Accelerator is ready to accept next sample
     output logic calc_fin_o   // Finish caclucation for integration time
);

localparam MUL_NUM = CHNUM * TWIDTH;
localparam CMUL_NUM      = ( (MUL_NUM * MUL_NUM )- MUL_NUM ) / 2 + MUL_NUM;
localparam CMUL_NUM_PC   =  2;
localparam FIFO_DEPTH    = 4 * MUL_NUM; 
localparam ACCUM_WIDTH   = 32;
localparam CALC_LATENCY  = CMUL_NUM + CMUL_NUM%2;
                                                             
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
logic                ij_ptr_en;

logic [2*IDWIDTH -1:0] ac_prod     [CMUL_NUM_PC -1:0];
logic [2*IDWIDTH -1:0] bd_prod     [CMUL_NUM_PC -1:0];
logic [2*IDWIDTH -1:0] ad_prod     [CMUL_NUM_PC -1:0];
logic [2*IDWIDTH -1:0] bc_prod     [CMUL_NUM_PC -1:0];
logic [2*IDWIDTH   :0] o_real       [CMUL_NUM_PC -1:0];
logic [2*IDWIDTH   :0] o_imag       [CMUL_NUM_PC -1:0];
logic            [3:0] ram_addr;
logic                  ram_we;
logic [ACCUM_WIDTH-1:0]  sum_real  [CMUL_NUM_PC -1:0];
logic [ACCUM_WIDTH-1:0]  sum_imag  [CMUL_NUM_PC -1:0];

logic [ACCUM_WIDTH-1:0]  ram_din_imag  [CMUL_NUM_PC -1:0];
logic [ACCUM_WIDTH-1:0]  ram_dout_imag [CMUL_NUM_PC -1:0];
logic [ACCUM_WIDTH-1:0]  ram_din_real  [CMUL_NUM_PC -1:0];
logic [ACCUM_WIDTH-1:0]  ram_dout_real [CMUL_NUM_PC -1:0];

logic smpl_rdy_new;
logic [IDWIDTH -1:0] smpl_rvec_ff    [MUL_NUM -1:0];
logic [IDWIDTH -1:0] smpl_ivec_ff    [MUL_NUM -1:0];
logic                smpl_vec_wr;
logic                smpl_vec_en    [MUL_NUM/2 -1:0];

logic [3:0] ram_we_ptr_ff;
logic [3:0] ram_we_ptr_next;
logic       ram_we_ptr_en;

// control logic 
logic acc_on_ff;
logic acc_sw_on;
logic [4:0] proc_ctr_ff;
logic smpl_accepted_ff;
logic [$clog2(INT_LENGTH):0]integr_ctr_ff;
logic [4:0] proc_ctr_next;
logic [$clog2(INT_LENGTH):0]integr_ctr_next;
logic resp_ff;
logic mem_bk_ff;
logic mem_imagsel_ff;


logic mem_rst_ff;
logic [3:0] mem_rst_cntr_ff;

always @(posedge clk or posedge rst )
    if(rst) begin
      mem_rst_ff <= '1;
      mem_rst_cntr_ff <= '0;  
    end else begin
      mem_rst_ff  <=   mem_rst_ff & (mem_rst_cntr_ff < 12);
      mem_rst_cntr_ff <= mem_rst_ff & (mem_rst_cntr_ff < 12) ? mem_rst_cntr_ff + 1 : '0;
    end
    
// output port assignments
assign acc_ready_o = (proc_ctr_ff == CALC_LATENCY );
assign calc_fin_o  = (integr_ctr_ff == INT_LENGTH);

// control logic
assign smpl_rdy_new =  acc_sw_on | smpl_vec_wr;
assign acc_sw_on = ~acc_on_ff & acc_on_i; 

assign proc_ctr_next = smpl_accepted_ff ? 1 : ( (proc_ctr_ff == CALC_LATENCY ) ? '0 : proc_ctr_ff + |proc_ctr_ff);

assign integr_ctr_next = (integr_ctr_ff == INT_LENGTH) ? '0 : (  (proc_ctr_ff == CALC_LATENCY ) ? integr_ctr_ff + 1
                                                                                                : integr_ctr_ff);
                                                                                               
always_ff @(posedge clk or posedge rst)
    if(rst) begin
        acc_on_ff <= '0;
        integr_ctr_ff <= '0;
        ram_we_ptr_ff <= '0;
        smpl_accepted_ff <= '0;
    end else begin
        acc_on_ff <= acc_on_i; 
       
        integr_ctr_ff <= integr_ctr_next;
        ram_we_ptr_ff <= ram_we_ptr_next;
        smpl_accepted_ff <= smpl_rdy_new;
    end
always_ff @(posedge clk or posedge rst)
  if(rst)   
     proc_ctr_ff <= '0; 
  else if (acc_sw_on | acc_on_ff)
     proc_ctr_ff <= proc_ctr_next; 
               
assign smpl_vec_wr = mif.req & mif.we 
                    & (mif.addr == FIFO_BASE_ADDR); 
 
  for ( genvar ii = 0; ii < TWIDTH; ii = ii + 1 ) begin : g_sample_fill
    
    always_ff @(posedge clk or posedge rst)
        if (rst) begin
        smpl_rvec_ff[ii] <= '0;
        smpl_ivec_ff[ii] <= '0;
        smpl_rvec_ff[ii + TWIDTH] <= '0;
        smpl_ivec_ff[ii + TWIDTH] <= '0;
        end else if (smpl_vec_wr) begin
            smpl_rvec_ff[ii] <= (ii == TWIDTH-1) ? mif.wdata[7:0] : smpl_rvec_ff[ii+1] ;
            smpl_ivec_ff[ii] <= (ii == TWIDTH-1) ? mif.wdata[15:8] :  smpl_ivec_ff[ii + 1] ;          
            smpl_rvec_ff[ii + TWIDTH] <= (ii == TWIDTH-1) ? mif.wdata[23:16] : smpl_rvec_ff[ii + TWIDTH + 1];
            smpl_ivec_ff[ii + TWIDTH] <= (ii == TWIDTH-1) ? mif.wdata[31:24] : smpl_ivec_ff[ii + TWIDTH + 1];   
        end
            
  end : g_sample_fill


assign ram_we = proc_ctr_ff[0] | mem_rst_ff;
        
assign ram_addr = mem_rst_ff ? mem_rst_cntr_ff : ((|proc_ctr_ff | smpl_accepted_ff ) ? ( proc_ctr_ff [4:1] ): mif.addr[4+:4]);
  
assign ij_ptr_en = smpl_rdy_new | (acc_on_ff & proc_ctr_ff[0]);
                      
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
             if (ij_ptr_en) begin
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

    mult_gen_0 i_mult_ac
        (
        .CLK (clk),
        .A(i_real_part[ii]),
        .B(j_real_part[ii]),
        .P(ac_prod[ii])
        );

    mult_gen_0 i_mult_bd
        (
        .CLK (clk),
        .A(i_imag_part[ii]),
        .B(j_imag_part[ii]),
        .P(bd_prod[ii])
        );
    
    mult_gen_0 i_mult_ad
        (
        .CLK (clk),
        .A(i_real_part[ii]),
        .B(j_imag_part[ii]),
        .P(ad_prod[ii])
        );
        
    mult_gen_0 i_mult_bc
        (
        .CLK (clk),
        .A(i_imag_part[ii]),
        .B(j_real_part[ii]),
        .P(bc_prod[ii])
        );
    
    c_addsub_0 i_bc_add_ad
        (
        .A(ad_prod[ii]),
        .B(bc_prod[ii]),
        .ADD(1'b1),
        .S(o_imag[ii])
        );
      
    c_addsub_0 i_ac_sub_bd
        (
        .A(ac_prod[ii]),
        .B(bd_prod[ii]),
        .ADD(1'b0),
        .S(o_real[ii])
        );
   
   c_addsub_1 acc_real
    (
    .A({{15{o_real[ii][2*IDWIDTH]}},o_real[ii]}),
    .B(ram_dout_real[ii]),
    .S(sum_real[ii])
    );
    
   c_addsub_1 acc_imag
    (
    .A({{15{o_imag[ii][2*IDWIDTH]}},o_imag[ii]}),
    .B(ram_dout_imag[ii]),
    .S(sum_imag[ii])
    );
    
    assign  ram_din_imag[ii] = mem_rst_ff ? '0 :  sum_imag[ii];
    assign  ram_din_real[ii] = mem_rst_ff ? '0 :  sum_real[ii];
     
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
        .dat_i(ram_din_real[ii]),
        .adr_i(ram_addr),
        .we_i (ram_we),
        .dat_o(ram_dout_real[ii]),
        .clk  (clk)
      );    
        
end :  g_dsp_ptr

always_comb begin
    acc_rdata_o = '0;
    for ( integer ii = 0; ii < CMUL_NUM_PC; ii = ii + 1 )
        acc_rdata_o |= {31{(ii == mem_bk_ff)}} & (mem_imagsel_ff ? ram_dout_imag[ii] : ram_dout_real[ii] ); 
end
// MIF outputs
always_ff @(posedge clk or posedge rst)
    if (rst) begin
        resp_ff   <= '0;
        mem_bk_ff <= '0;
        mem_imagsel_ff <= '0;
    end else begin
        resp_ff   <= mif.req;
        mem_bk_ff <= mif.addr[3];
        mem_imagsel_ff <= mif.addr[2];
    end
    
assign acc_ack_o = ~(|proc_ctr_ff | smpl_accepted_ff | mem_rst_ff);
assign acc_resp_o = resp_ff;
   
 
endmodule
