/*
 * sigma.sv
 *
 *  Created on: 24.09.2017
 *      Author: Alexander Antonov <antonov.alex.alex@gmail.com>
 *     License: See LICENSE file for details
 */


`include "sigma_tile.svh"

module sigma
#(
	parameter CPU = "none",
	parameter UDM_BUS_TIMEOUT = (1024*1024*100),
	parameter UDM_RTX_EXTERNAL_OVERRIDE = "NO",
	parameter delay_test_flag = 0,
	parameter mem_init="YES",
	parameter mem_type="elf",
	parameter mem_data = "data.hex",
	parameter mem_size = 1024
)
(
	input clk_i
	, input arst_i
	, input irq_btn_i
	, input rx_i
	, output tx_o
	, input [31:0] gpio_bi
	, output [31:0] gpio_bo
);

logic acc_ack;
logic acc_resp;
logic [31:0] acc_rdata;
logic  acc_start;
logic  acc_on;
logic acc_ready;
logic calc_fin;	
logic accel_sel;

wire srst;
reset_sync reset_sync
(
	.clk_i(clk_i),
	.arst_i(arst_i),
	.srst_o(srst)
);

wire udm_reset;
wire cpu_reset;
assign cpu_reset = srst | udm_reset;

wire irq_btn_debounced;
debouncer debouncer
(
	.clk_i(clk_i)
	, .rst_i(srst)
	, .in(irq_btn_i)
	, .out(irq_btn_debounced)
);

MemSplit32 hif();
MemSplit32 xif();

sigma_tile #(
	.corenum(0)
	, .mem_init(mem_init)
	, .mem_type(mem_type)
	, .mem_data(mem_data)
	, .mem_size(mem_size)
	, .CPU(CPU)
	, .PATH_THROUGH("YES")
) sigma_tile (
	.clk_i(clk_i)
	, .rst_i(cpu_reset)

	, .irq_debounced_bi({0, irq_btn_debounced, 3'h0})
	
	, .hif(hif)
	, .xif(xif)
    , .accel_rdy_i (acc_ready)
	, .accel_int_fin_i (calc_fin)
	, .accel_sw_on_o (acc_on)
	, .accel_start_o (acc_start)
);

sobel i_rr_corr 
(
     .clk (clk_i),
     .rst (arst_i),
     .mif (xif),   
     .acc_ack_o(acc_ack),
     .acc_resp_o(acc_resp),
     .acc_rdata_o(acc_rdata),  
     .acc_on_i(acc_on),   
     .acc_ready_o(acc_ready), 
     .calc_fin_o(calc_fin)    
);

udm #(
    .BUS_TIMEOUT(UDM_BUS_TIMEOUT)
    , .RTX_EXTERNAL_OVERRIDE(UDM_RTX_EXTERNAL_OVERRIDE)
) udm (
	.clk_i(clk_i)
	, .rst_i(srst)

	, .rx_i(rx_i)
	, .tx_o(tx_o)

	, .rst_o(udm_reset)
	
	, .bus_req_o(hif.req)
	, .bus_we_o(hif.we)
	, .bus_addr_bo(hif.addr)
	, .bus_be_bo(hif.be)
	, .bus_wdata_bo(hif.wdata)
	, .bus_ack_i(hif.ack)
	, .bus_resp_i(hif.resp)
	, .bus_rdata_bi(hif.rdata)
);


localparam CSR_LED_ADDR         = 32'h80000000;
localparam CSR_SW_ADDR          = 32'h80000004;

logic [31:0] gpio_bo_reg;
assign gpio_bo = gpio_bo_reg;
logic [31:0] gpio_bi_reg;
always @(posedge clk_i) gpio_bi_reg <= gpio_bi;

assign xif.ack = xif.req;   // xif always ready to accept request
logic csr_resp;
logic [31:0] csr_rdata;

// bus request
always @(posedge clk_i)
    begin
    
    csr_resp <= 1'b0;
    
    if (xif.req && xif.ack)
        begin
        
        if (xif.we)     // writing
            begin
            if (xif.addr == CSR_LED_ADDR) gpio_bo_reg <= xif.wdata;
            end
        
        else            // reading
            begin
            if (xif.addr == CSR_LED_ADDR)
                begin
                csr_resp <= 1'b1;
                csr_rdata <= gpio_bo_reg;
                end
            if (xif.addr == CSR_SW_ADDR)
                begin
                csr_resp <= 1'b1;
                csr_rdata <= gpio_bi_reg;
                end
            end
        end
    end
assign accel_sel = xif.req &  |(xif.addr[8] | xif.addr[6]);

// bus response
always @*
    begin
    xif.resp = csr_resp | acc_resp;
    xif.rdata = 0;
    if (csr_resp) xif.rdata = csr_rdata;
    else if (acc_resp) xif.rdata = acc_rdata;
    end

endmodule
