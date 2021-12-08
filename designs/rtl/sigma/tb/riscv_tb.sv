/*
 * riscv_tb.sv
 *
 *  Created on: 24.09.2017
 *      Author: Alexander Antonov <antonov.alex.alex@gmail.com>
 *     License: See LICENSE file for details
 */


`timescale 1ns / 1ps

`define HALF_PERIOD			5						//external 100 MHZ
`define DIVIDER_115200		32'd8680
`define DIVIDER_19200		32'd52083
`define DIVIDER_9600		32'd104166
`define DIVIDER_4800		32'd208333
`define DIVIDER_2400		32'd416666

module riscv_tb ();
//
reg CLK_100MHZ, RST, rx;
reg [31:0] SW;
wire [31:0] LED;
reg irq_btn;
logic [15:0] complex_arr [19:0] = {
16'h4121,
16'h0707,
16'h910B,
16'h7132,
16'h7164,
16'h3201,
16'hD159,
16'h0AB1,
16'hC203,
16'h4101,
16'h0102,
16'h3755,
16'h0145,
16'h5161,
16'h4171,
16'h7171,
16'h6373,
16'h7252,
16'h5262,
16'h9131};

logic [31:0] ref_real [20:0];
logic [31:0] ref_imag [20:0];

int accum_ctr = '0;	
int window_ptr  = '0;
int array_ptr = '0;

sigma
#(
	//.CPU("riscv_1stage")
	//.CPU("riscv_2stage")
	//.CPU("riscv_3stage")
	//.CPU("riscv_4stage")
	.CPU("riscv_5stage")
	//.CPU("riscv_6stage")

	, .UDM_RTX_EXTERNAL_OVERRIDE("YES")
	, .delay_test_flag(0)
	
	, .mem_init("YES")
	, .mem_type("elf")
	, .mem_data("C://Users//Bogdana//FILES//ifmo//Master//2_course//SoCSECUR//activecore//designs//rtl//sigma//sw//benchmarks//accel_driver.riscv")
	, .mem_size(8192)
) sigma
(
	.clk_i(CLK_100MHZ)
	, .arst_i(RST)
	, .irq_btn_i(irq_btn)
	, .rx_i(rx)
	//, .tx_o()
	, .gpio_bi(SW)
	, .gpio_bo(LED)
);

//////////////////////////
/////////tasks////////////
//////////////////////////

reg parity;
integer i, j, k;

reg [32:0] rate;
reg [1:0] configuration;


////wait////
task WAIT
	(
	 input reg [15:0] periods
	 );
begin
for (i=0; i<periods; i=i+1)
	begin
	#(`HALF_PERIOD*2);
	end
end
endtask


////reset all////
task RESET_ALL ();
begin
	CLK_100MHZ = 1'b0;
	RST = 1'b1;
	irq_btn = 1'b0;
	rx = 1'b1;
	#(`HALF_PERIOD/2);
	RST = 1;
	#(`HALF_PERIOD*6);
	RST = 0;
end
endtask

`define UDM_RX_SIGNAL rx
`define UDM_BLOCK sigma.udm
`include "udm.svh"
udm_driver udm;

///////////////////
// initial block //
localparam CPU_RAM_ADDR         = 32'h00000000;
localparam CSR_LED_ADDR         = 32'h80000000;
localparam CSR_SW_ADDR          = 32'h80000004;
localparam RES_BUF_ADDR         = 32'h80000100;
localparam INTEG_LENGTH         = 10;
localparam WIND_SIZE            = 3;
	   logic [7:0] reala;
	   logic [7:0] realb;
	   logic [7:0] imaga;
	   logic [7:0] imagb;
logic [31:0] read_addr;	
logic [15:0] window [2*WIND_SIZE -1:0];

initial
begin
	$display ("### SIMULATION STARTED ###");
    read_addr = RES_BUF_ADDR;
	SW = 8'h30;
	RESET_ALL();
	WAIT(1000);
	irq_btn = 1'b0;
	WAIT(100);
	irq_btn = 1'b0;
	WAIT(50);
	udm.check();
	//udm.hreset();
	// WAIT(100);
	
	//udm.wr32(CSR_LED_ADDR, 32'hdeadbeef);
	
	WAIT(500);
	
    for (int i = 0; i < 21; i = i + 1) begin
        ref_real[i] = '0;
        ref_imag[i] = '0;
    end
	
	while (accum_ctr < INTEG_LENGTH) begin
	   array_ptr = '0;
	   
      if (!accum_ctr) begin
	       
	   for (int j = 0; j < WIND_SIZE; j++ ) begin 
	       window[j] = complex_arr[2*j];
	       window[j+WIND_SIZE] = complex_arr[2*j + 1];
	   end 
	   accum_ctr +=3;   
	  end else begin
	     
	  for (int j = 0; j < WIND_SIZE; j++ ) begin 
	       window[j] = (j == (WIND_SIZE -1)) ? complex_arr[accum_ctr*2] : window[j+1] ;
	       window[j+WIND_SIZE] = (j == (WIND_SIZE -1)) ? complex_arr[2*accum_ctr + 1]: window[j+WIND_SIZE+1];
	   end  
	     accum_ctr +=1; 
	  end
	  
	   for (int j = 0; j < 2*WIND_SIZE; j++ ) begin
	       for (int k = j; k < 2*WIND_SIZE; k++ ) begin
	       reala = window[j][7:0];
	       realb = window[k][7:0];
	       imaga = window[j][15:8];
	       imagb = (~window[k][15:8] + 1);
	       #10;

	           ref_real[array_ptr + k - j] +=   {{24{reala[7]}}, {reala}} * {{24{realb[7]}}, realb }
	                                         -  {{24{imaga[7]}}, imaga }* {{ 24 { imagb[7]}} , imagb};
	           ref_imag[array_ptr + k - j] +=   {{24{reala[7]}}, reala}* {{24{imagb[7]}}, imagb}
	                                          + {{24{realb[7]}}, realb} * {{24{imaga[7]}}, imaga};
	       end
	       array_ptr += 2*WIND_SIZE -j;
	   end
	end
				
	for (int i = 0; i < 21; i = i + 1) begin
	   $display ("element real/imag %d", i);
	   $display ("assumed  real %x",ref_real[i] );
	   $display ("assumed  imag %x",ref_imag[i] );
	   udm.rd32(read_addr + i*8);
	   udm.rd32(read_addr + 4 + i*8);
	end

	
	$display ("### TEST PROCEDURE FINISHED ###");
	$stop;
end
//
always #`HALF_PERIOD CLK_100MHZ = ~CLK_100MHZ;

always #1000 SW = SW + 8'h1;
//
endmodule
