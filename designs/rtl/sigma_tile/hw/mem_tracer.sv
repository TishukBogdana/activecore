`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: BTishchuk
// 
// Create Date: 08.09.2021 21:19:28
// Design Name: 
// Module Name: mem_tracer
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`include "sigma_tile.svh"

module mem_tracer
#(
    parameter CAPACITY = 256 
)(
    input clk,
    input rst_n,
    
    MemSplit32.Monitor cpu_data_if, 
    MemSplit32.Slave extnl_if
    );
    // ---------------------
    localparam MEM_ADDR_WIDTH = $clog2(CAPACITY );
    
    // ---------------------
    logic mem_we;
    logic [31:0] mem_wtran_addr;
    logic [31:0] mem_rtran_addr;
    logic [31:0] mem_wtran_data;
    logic [31:0] mem_rtran_data;
    logic [MEM_ADDR_WIDTH-1:0] mem_addr;
    logic [MEM_ADDR_WIDTH-1:0] wr_ptr_ff;
    logic [MEM_ADDR_WIDTH-1:0] head_ptr_ff;
    logic tran_we_ff [CAPACITY -1 : 0];
    logic overflow_ff;
    logic wr_tran_acc;
    logic rd_tran_acc;
    logic rd_tran_close_ff;
    logic [31:0] rd_tran_addr_ff;
    
    assign wr_tran_acc = ( cpu_data_if.req & cpu_data_if.ack & cpu_data_if.we ) ;
    assign rd_tran_acc = cpu_data_if.req & cpu_data_if.ack & ~cpu_data_if.we;
    assign mem_we =  wr_tran_acc | rd_tran_close_ff;
    
    //! Can WE req be in the same time with rdata resp
    
    assign mem_wtran_data = wr_tran_acc ? cpu_data_if.wdata : cpu_data_if.rdata;
    assign mem_wtran_addr = wr_tran_acc ? cpu_data_if.addr : rd_tran_addr_ff;
    assign mem_addr = mem_we ? wr_ptr_ff : extnl_if.addr;
    always_ff @(posedge clk or negedge rst_n)
        if (~rst_n)
            overflow_ff <= 0;
        else
            overflow_ff <= overflow_ff | (&wr_ptr_ff );
            
    always_ff @(posedge clk or negedge rst_n)
        if(~rst_n)
            rd_tran_close_ff <=0;
        else
            rd_tran_close_ff <= rd_tran_acc;
            
    always_ff @(posedge clk) 
        if ( rd_tran_acc )       
            rd_tran_addr_ff <= cpu_data_if.addr;
    always_ff @(posedge clk or negedge rst_n)
        if (~rst_n)
            wr_ptr_ff <= 0;
        else if ( mem_we )
            wr_ptr_ff <= wr_ptr_ff + 1;               
    ram #(
        .adr_width(MEM_ADDR_WIDTH),
        .mem_size(CAPACITY )
    ) i_dataram (
        .adr_i (mem_addr),
        .dat_i (mem_wtran_data),
        .we_i  (mem_we),
        .dat_o (mem_rtran_data)
    );
    
    ram #(
        .adr_width(MEM_ADDR_WIDTH),
        .mem_size(CAPACITY )
    ) i_adrram (
        .adr_i (mem_addr),
        .dat_i (mem_wtran_addr),
        .we_i  (mem_we),
        .dat_o (mem_rtran_addr)
    );
    
    for ( genvar ii = 0; ii < CAPACITY; ii = ii + 1 )
        always_ff @(posedge clk )
            if (mem_we)
                tran_we_ff[ii] <= (ii == wr_ptr_ff ) ? wr_tran_acc : tran_we_ff[ii];
            
endmodule
