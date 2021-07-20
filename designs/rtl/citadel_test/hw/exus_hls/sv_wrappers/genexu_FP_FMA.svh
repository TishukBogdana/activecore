// ===========================================================
// RTL generated by ActiveCore framework
// Date: 2021-07-20 21:52:32
// Copyright Alexander Antonov <antonov.alex.alex@gmail.com>
// ===========================================================

`ifndef __genexu_FP_FMA_h_
`define __genexu_FP_FMA_h_

`ifndef __genstructdel_resp_struct_
`define __genstructdel_resp_struct_
typedef struct packed {
	logic unsigned [31:0] trx_id;
	logic unsigned [31:0] tag;
	logic unsigned [31:0] wdata;
} resp_struct;
`endif // __genstructdel_resp_struct_

`ifndef __genstructdel_req_struct_
`define __genstructdel_req_struct_
typedef struct packed {
	logic unsigned [31:0] trx_id;
	logic unsigned [31:0] rd_tag;
	logic unsigned [31:0] opcode;
	logic unsigned [31:0] rs0_rdata;
	logic unsigned [31:0] rs1_rdata;
	logic unsigned [31:0] rs2_rdata;
} req_struct;
`endif // __genstructdel_req_struct_

`endif // __genexu_FP_FMA_h_
