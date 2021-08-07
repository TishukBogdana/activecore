/*
 * reorder_buffer.kt
 *
 *  Created on: 24.12.2020
 *      Author: Alexander Antonov <antonov.alex.alex@gmail.com>
 *     License: See LICENSE file for details
 */

package reordex

import hwast.*
import cyclix.*

open class rob(cyclix_gen : cyclix.Generic,
               name_prefix : String,
               TRX_BUF_SIZE : Int,
               MultiExu_CFG : Reordex_CFG,
               val cdb_num : Int) : trx_buffer(cyclix_gen, name_prefix, TRX_BUF_SIZE, MultiExu_CFG) {

    var trx_id          = AddStageVar(hw_structvar("trx_id",            DATA_TYPE.BV_UNSIGNED, GetWidthToContain(MultiExu_CFG.trx_inflight_num) -1, 0, "0"))
    var rd_tag_prev     = AddStageVar(hw_structvar("rd_tag_prev",       DATA_TYPE.BV_UNSIGNED, MultiExu_CFG.PRF_addr_width-1, 0, "0"))
    var rd_tag_prev_clr = AddStageVar(hw_structvar("rd_tag_prev_clr",   DATA_TYPE.BV_UNSIGNED, 0, 0, "0"))
    var cdb_id          = AddStageVar(hw_structvar("cdb_id",            DATA_TYPE.BV_UNSIGNED, GetWidthToContain(cdb_num) -1, 0, "0"))
    val rdy             = AddStageVar(hw_structvar("rdy",               DATA_TYPE.BV_UNSIGNED, 0, 0, "0"))

    var TRX_ID_COUNTER  = cyclix_gen.uglobal(name_prefix + "_TRX_ID_COUNTER", GetWidthToContain(TRX_BUF_SIZE) -1, 0, "0")

    open fun Commit(global_structures: __global_structures) {
        preinit_ctrls()
        init_locals()

        cyclix_gen.begif(ctrl_active)
        run {
            cyclix_gen.begif(rdy)
            run {
                cyclix_gen.begif(rd_tag_prev_clr)
                run {
                    global_structures.FreePRF(rd_tag_prev)
                }; cyclix_gen.endif()
                cyclix_gen.assign(pop, 1)
            }; cyclix_gen.endif()
        }; cyclix_gen.endif()

        // popping
        cyclix_gen.begif(pop)
        run {
            pop_trx()
        }; cyclix_gen.endif()
    }
}

class rob_risc(name: String,
               cyclix_gen : cyclix.Generic,
               name_prefix : String,
               TRX_BUF_SIZE : Int,
               MultiExu_CFG : Reordex_CFG,
               cdb_num : Int) : rob(cyclix_gen, name_prefix, TRX_BUF_SIZE, MultiExu_CFG, cdb_num) {

    var curinstr_addr   = AdduStageVar("curinstr_addr", 31, 0, "0")
    var nextinstr_addr  = AdduStageVar("nextinstr_addr", 31, 0, "0")

    var rd_req          = AdduStageVar("rd_req", 0, 0, "0")
    var rd_addr         = AdduStageVar("rd_addr", 4, 0, "0")
    var rd_wdata        = AdduStageVar("rd_wdata", MultiExu_CFG.RF_width-1, 0, "0")

    var mem_req         = AdduStageVar("mem_req", 0, 0, "0")
    var mem_cmd         = AdduStageVar("mem_cmd", 0, 0, "0")
    var mem_addr        = AdduStageVar("mem_addr", 31, 0, "0")
    var mem_be          = AdduStageVar("mem_be", 3, 0, "0")

    // control transfer signals
    var jump_req        = AdduStageVar("jump_req", 0, 0, "0")
    var jump_req_cond   = AdduStageVar("jump_req_cond", 0, 0, "0")
    var jump_src        = AdduStageVar("jump_src", 0, 0, "0")
    var jump_vector     = AdduStageVar("jump_vector", 31, 0, "0")

    var busreq_mem_struct = hw_struct(name + "_busreq_mem_struct")
    val data_name_prefix = "genmcopipe_data_mem_"
    var rd_struct = hw_struct("genpmodule_" + name + "_" + data_name_prefix + "genstruct_fifo_wdata")

    var data_req_fifo   = cyclix_gen.fifo_out((data_name_prefix + "req"), rd_struct)
    var data_resp_fifo  = cyclix_gen.ufifo_in((data_name_prefix + "resp"), 31, 0)

    var irq_fifo        = cyclix_gen.ufifo_in("irq_fifo", 7, 0)

    var rf_dim = hw_dim_static()
    var Backoff_ARF = cyclix_gen.uglobal("Backoff_ARF", rf_dim, "0")

    var expected_instraddr = cyclix_gen.uglobal("expected_instraddr", 31, 0, hw_imm(32, IMM_BASE_TYPE.HEX, "200"))

    init {
        busreq_mem_struct.addu("addr",     31, 0, "0")
        busreq_mem_struct.addu("be",       3,  0, "0")
        busreq_mem_struct.addu("wdata",    31, 0, "0")

        rd_struct.addu("we", 0, 0, "0")
        rd_struct.add("wdata", hw_type(busreq_mem_struct), "0")

        rf_dim.add(31, 0)
        rf_dim.add(31, 0)
    }

    fun Commit(global_structures: __global_structures, pc : hw_var, bufs_to_rollback : ArrayList<hw_stage>) {

        var mem_rd_inprogress   = cyclix_gen.uglobal("mem_rd_inprogress", 0, 0, "0")
        var mem_data_wdata      = cyclix_gen.local("mem_data_wdata", data_req_fifo.vartype, "0")
        var mem_data_rdata      = cyclix_gen.local("mem_data_rdata", data_resp_fifo.vartype, "0")

        var backoff_cmd         = cyclix_gen.ulocal("backoff_cmd", 0, 0, "0")

        preinit_ctrls()
        init_locals()

        cyclix_gen.begif(mem_rd_inprogress)
        run {
            cyclix_gen.begif(cyclix_gen.fifo_rd_unblk(data_resp_fifo, mem_data_rdata))
            run {
                cyclix_gen.assign(mem_rd_inprogress, 0)
                cyclix_gen.assign(pop, 1)
            }; cyclix_gen.endif()
        }; cyclix_gen.endif()
        cyclix_gen.begelse()
        run {
            cyclix_gen.begif(ctrl_active)
            run {

                cyclix_gen.begif(cyclix_gen.eq2(expected_instraddr, curinstr_addr))     // branch prediction fine
                run {
                    cyclix_gen.assign(pop, 1)

                    cyclix_gen.begif(mem_req)
                    run {
                        cyclix_gen.assign(mem_data_wdata.GetFracRef("we"), mem_cmd)
                        cyclix_gen.assign(mem_data_wdata.GetFracRef("wdata").GetFracRef("addr"), mem_addr)
                        cyclix_gen.assign(mem_data_wdata.GetFracRef("wdata").GetFracRef("be"), mem_be)
                        cyclix_gen.assign(mem_data_wdata.GetFracRef("wdata").GetFracRef("wdata"), rd_wdata)
                        cyclix_gen.fifo_wr_unblk(data_req_fifo, mem_data_wdata)
                        cyclix_gen.begif(!mem_cmd)
                        run {
                            cyclix_gen.assign(pop, 0)
                        }; cyclix_gen.endif()
                    }; cyclix_gen.endif()
                }; cyclix_gen.endif()

                cyclix_gen.begelse()
                run {
                    cyclix_gen.assign(backoff_cmd, 1)
                    cyclix_gen.assign(pc, expected_instraddr)
                    for (buf_to_rollback in bufs_to_rollback) {
                        buf_to_rollback.Reset()
                    }
                    global_structures.RollBack(Backoff_ARF)
                }; cyclix_gen.endif()

            }; cyclix_gen.endif()
        }; cyclix_gen.endif()

        // uop completed successfully, popping
        cyclix_gen.begif(pop)
        run {

            cyclix_gen.begif(rd_tag_prev_clr)
            run {
                global_structures.FreePRF(rd_tag_prev)
                cyclix_gen.begif(rd_req)
                run {
                    cyclix_gen.assign(Backoff_ARF.GetFracRef(rd_addr), rd_wdata)
                }; cyclix_gen.endif()
            }; cyclix_gen.endif()

            cyclix_gen.assign(expected_instraddr, nextinstr_addr)
            cyclix_gen.begif(jump_req)
            run {
                cyclix_gen.assign(expected_instraddr, jump_vector)
            }; cyclix_gen.endif()

            pop_trx()
        }; cyclix_gen.endif()

        finalize_ctrls()
    }
}