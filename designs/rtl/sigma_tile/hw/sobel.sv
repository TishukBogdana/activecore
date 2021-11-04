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
    parameter MWSIZE = 7, // MAXIMUM WINDOW SIZE
    parameter WSIZE_ADDR = 32'h00100032,
    parameter COEF_BIT   = 6,
    parameter SFR_BITSEL = 16,
    parameter FIFO_ADDR  = 32'h00100036
)
(
     input logic clk,
     input logic rst_n,
     MemSplit32.Slave mif
);

localparam WSIZE_RST_VAL = 3;

logic [$clog2(MWSIZE) -1:0] wsize_ff;
logic                       wsize_en;
logic                             coef_we;
logic [$clog2 (MWSIZE**2) -1 : 0] coef_addr;
logic [15:0]                      coef_rdata;

assign wsize_en = mif.req & (mif.addr == WSIZE_ADDR);

always_ff @(posedge clk or negedge rst_n)
    if(~rst_n)
        wsize_ff <= WSIZE_RST_VAL;
    else if (wsize_en)
        wsize_ff <= mif.wdata [$clog2(MWSIZE) -1:0];
        
assign coef_we = mif.we 
               & mif.req 
               & (mif.addr[COEF_BIT])
               & (~|(mif.addr[SFR_BITSEL-1:COEF_BIT+1]));
            
assign coef_addr = mif.addr[COEF_BIT:0];

 ram 
  #(
    .dat_width  (16), 
    .adr_width  (7),
    .mem_size   (128)
  ) coef_ram (
    .dat_i (mif.wdata [15:0]),
    .adr_i (coef_addr),
    .we_i  (coef_we),
    .dat_o (coef_rdata),
    .clk   (clk)
  );      
        
 
endmodule
