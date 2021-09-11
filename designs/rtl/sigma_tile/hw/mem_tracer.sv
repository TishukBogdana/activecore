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
    input rst,
    input [3:0] trace_ctrl_i,
    output trace_flush_end_o,
    MemSplit32.Monitor cpu_data_if, 
    MemSplit32.Slave extnl_if
    );
    // ---------------------
    localparam MEM_ADDR_WIDTH = $clog2(CAPACITY );
    localparam logic [31:0] MEM_FLUSH_DATA = '0;
    // ---------------------
    logic mem_we;
    logic [31:0] mem_wtran_addr;
    logic [31:0] mem_rtran_addr;
    logic [31:0] mem_wtran_data;
    logic [31:0] mem_rtran_data;
    logic [MEM_ADDR_WIDTH-1:0] mem_addr;
    logic [MEM_ADDR_WIDTH-1:0] mem_flush_addr_ff;
    logic [MEM_ADDR_WIDTH+1:0] mem_rd_addr_ff;
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
    assign mem_we = (( wr_tran_acc | rd_tran_close_ff ) & trace_ctrl_i[TRACE_EN_BIT]) | trace_ctrl_i[TRACE_FLUSH_BIT];
    
    //! Can WE req be in the same time with rdata resp
    
    assign mem_wtran_data =  trace_ctrl_i[TRACE_FLUSH_BIT] ? MEM_FLUSH_DATA 
                                                           : ( wr_tran_acc ? cpu_data_if.wdata : cpu_data_if.rdata );
    assign mem_wtran_addr =  trace_ctrl_i[TRACE_FLUSH_BIT] ? MEM_FLUSH_DATA
                                                           : ( wr_tran_acc ? cpu_data_if.addr : rd_tran_addr_ff );
    assign mem_addr = mem_we ? ( trace_ctrl_i[TRACE_FLUSH_BIT] ? mem_flush_addr_ff : wr_ptr_ff) 
                             : extnl_if.addr[MEM_ADDR_WIDTH + 1:2];
    
    always_ff @(posedge clk or posedge rst )
        if(rst)
            mem_flush_addr_ff <= '0;
        else if (trace_ctrl_i[TRACE_FLUSH_BIT])
            mem_flush_addr_ff <= mem_flush_addr_ff + 1;
                          
    always_ff @(posedge clk or posedge rst)
        if(rst)
            rd_tran_close_ff <=0;
        else
            rd_tran_close_ff <= rd_tran_acc;
            
    always_ff @(posedge clk) 
        if ( rd_tran_acc )       
            rd_tran_addr_ff <= cpu_data_if.addr;
            
    always_ff @(posedge clk or posedge rst)
        if (rst)
            wr_ptr_ff <= 0;
        else
            wr_ptr_ff <=  trace_ctrl_i[TRACE_FLUSH_BIT] ? '0 : wr_ptr_ff + 1 ;  
            
     always_ff @(posedge clk or posedge rst)
        if (rst)
            overflow_ff <= 0;
        else
            overflow_ff <= ( overflow_ff | (&wr_ptr_ff )) 
                         & ~trace_ctrl_i[TRACE_FLUSH_BIT];  
                       
     always_ff @(posedge clk or posedge rst)
        if (rst)
            head_ptr_ff <= 0;
        else if ( mem_we )
            head_ptr_ff <= trace_ctrl_i[TRACE_FLUSH_BIT] ? '0 : ( head_ptr_ff + overflow_ff );        
            
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
        always_ff @(posedge clk or posedge rst)
            if(rst)
                tran_we_ff[ii] <= '0;
            else if (mem_we)
                tran_we_ff[ii] <= trace_ctrl_i[TRACE_FLUSH_BIT] ? '0 : ( (ii == wr_ptr_ff ) ? wr_tran_acc 
                                                                                            : tran_we_ff[ii]);
                
   // Extrenal read logic
   always_ff @(posedge clk or posedge rst)
     if(rst)
        mem_rd_addr_ff <=0;
     else 
        mem_rd_addr_ff <= extnl_if.addr[MEM_ADDR_WIDTH+1:0];
        
   assign extnl_if.rdata = ~(|mem_rd_addr_ff[1:0]) ? mem_rtran_addr 
                                                  : ((mem_rd_addr_ff[1:0] == 1'b1) ? mem_rtran_data
                                                                                   : tran_we_ff[mem_rd_addr_ff[MEM_ADDR_WIDTH+1:2]]); 
   assign extnl_if.ack = ~mem_we;
   
  // assign extnl_if.resp = 
   
   assign trace_flush_end_o = &(mem_flush_addr_ff);
            
endmodule
