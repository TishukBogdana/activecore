/*
 * sfr.sv
 *
 *  Created on: 27.12.2017
 *      Author: Alexander Antonov <antonov.alex.alex@gmail.com>
 *     License: See LICENSE file for details
 */


`include "sigma_tile.svh"

module sfr
#(
	parameter corenum=0
	, parameter SW_RESET_DEFAULT=0
	, parameter IRQ_NUM_POW=4
)
(
	input [0:0] clk_i
	, input [0:0] rst_i

	, MemSplit32.Slave host

	, output logic sw_reset_o

	, output logic [(2**IRQ_NUM_POW)-1:0] irq_en_bo
	, output logic irq_timer

	, output logic sgi_req_o
	, output logic [IRQ_NUM_POW-1:0] sgi_code_bo
	, output logic [3:0] trace_ctrl_o
	, input logic trace_flush_end_i
);

localparam IDCODE_ADDR 			= 8'h00;
localparam CTRL_ADDR 			= 8'h04;
localparam CORENUM_ADDR 		= 8'h08;

localparam IRQ_EN_ADDR 			= 8'h10;
localparam SGI_ADDR 			= 8'h14;

localparam TIMER_CTRL_ADDR 		= 8'h20;
localparam TIMER_PERIOD_ADDR 	= 8'h24;
localparam TIMER_VALUE_ADDR 	= 8'h28;
localparam TRACE_CTRL_ADDR 	    = 8'h32;
localparam TRACE_MASK           = 4'hE;

logic sw_reset, sw_reset_autoclr;
always @(posedge clk_i) sw_reset_o <= rst_i | sw_reset;

logic timer_inprogress, timer_reload;
logic [31:0] timer_period;
logic [31:0] timer_value, timer_value_inc;
logic [3:0]  tracer_ctrl_ff;
assign timer_value_inc = timer_value + 1;

always @(posedge clk_i)
	begin
	if (rst_i)
		begin
		host.resp <= 1'b0;
		sw_reset <= SW_RESET_DEFAULT;
		sw_reset_autoclr <= 1'b0;
		irq_en_bo <= 0;
		irq_timer <= 1'b0;
		sgi_req_o <= 0;
		sgi_code_bo <= 0;
		timer_inprogress <= 1'b0;
		timer_reload <= 1'b0;
		timer_period <= 0;
		timer_value <= 0;
		tracer_ctrl_ff <= 1;
		end
	else
		begin
		host.resp <= 1'b0;
		sgi_req_o <= 0;
		irq_timer <= 1'b0;

		if (sw_reset && sw_reset_autoclr) sw_reset <= 1'b0;

		if (sw_reset_o)
			begin
			timer_inprogress <= 1'b0;
			timer_reload <= 1'b0;
			irq_timer <= 1'b0;
			timer_period <= 0;
			timer_value <= 0;
			tracer_ctrl_ff <= 0;
			end

		if (timer_inprogress)
			begin
			if (timer_value_inc == timer_period)
				begin
				timer_inprogress <= timer_reload;
				irq_timer <= 1'b1;
				timer_value <= 0;
				end
			else timer_value <= timer_value_inc;
			end
			
        if (trace_flush_end_i) begin
            tracer_ctrl_ff <= tracer_ctrl_ff & TRACE_MASK;
        end
        
		if (host.req)
			begin
			if (host.we)
				begin
				if (host.addr[7:0] == CTRL_ADDR)
					begin
					sw_reset <= host.wdata[0];
					sw_reset_autoclr <= host.wdata[1];
					end
				if (host.addr[7:0] == IRQ_EN_ADDR)
					begin
					irq_en_bo <= host.wdata;
					end
				if (host.addr[7:0] == SGI_ADDR)
					begin
					sgi_req_o <= 1;
					sgi_code_bo <= host.wdata;
					end
				if (host.addr[7:0] == TIMER_CTRL_ADDR)
					begin
					timer_value <= 0;
					timer_inprogress <= host.wdata[0];
					timer_reload <= host.wdata[1];
					end
				if (host.addr[7:0] == TIMER_PERIOD_ADDR)
					begin
					timer_period <= host.wdata;
					end
				if (host.addr[7:0] == TRACE_CTRL_ADDR)
					begin
					tracer_ctrl_ff <= {host.wdata[3:1],  host.wdata[0] | tracer_ctrl_ff[0]} ;
					end
				end
			else
				begin
				host.resp <= 1'b1;
				if (host.addr[7:0] == IDCODE_ADDR)  		host.rdata <= 32'hdeadbeef;
				if (host.addr[7:0] == CTRL_ADDR)    		host.rdata <= {31'h0, sw_reset};
				if (host.addr[7:0] == CORENUM_ADDR) 		host.rdata <= corenum;
				if (host.addr[7:0] == IRQ_EN_ADDR) 			host.rdata <= irq_en_bo;
				if (host.addr[7:0] == TIMER_CTRL_ADDR) 		host.rdata <= {30'h0, timer_reload, timer_inprogress};
				if (host.addr[7:0] == TIMER_PERIOD_ADDR) 	host.rdata <= timer_period;
				if (host.addr[7:0] == TIMER_VALUE_ADDR) 	host.rdata <= timer_value;
				if (host.addr[7:0] == TRACE_CTRL_ADDR) 	    host.rdata <= tracer_ctrl_ff;
				end
			end
		end
	end

assign host.ack = host.req;

assign trace_ctrl_o  = tracer_ctrl_ff;

endmodule
