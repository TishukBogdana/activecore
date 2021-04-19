/*
 * hw_var.kt
 *
 *  Created on: 05.06.2019
 *      Author: Alexander Antonov <antonov.alex.alex@gmail.com>
 *     License: See LICENSE file for details
 */

package hwast

open class hw_var(name : String, vartype : hw_type, defimm : hw_imm) : hw_structvar(name, vartype, defimm) {

    var read_done: Boolean
    var write_done: Boolean

    var reset_pref : Boolean

    var default_astc = hw_astc()

    init {
        read_done = false
        write_done = false
        token_printable = name
        reset_pref = true
    }

    constructor(name : String, vartype : hw_type, defval : String)
            : this(name, vartype, hw_imm(defval))

    constructor(name: String, VarType : DATA_TYPE, dimensions : hw_dim_static, defimm : hw_imm)
            : this(name, hw_type(VarType, dimensions), defimm)

    constructor(name: String, VarType : DATA_TYPE, dimensions : hw_dim_static, defval : String)
            : this(name, hw_type(VarType, dimensions), defval)

    constructor(name: String, VarType: DATA_TYPE, msb: Int, lsb: Int, defimm : hw_imm)
            : this(name, hw_type(VarType, msb, lsb), defimm)

    constructor(name: String, VarType: DATA_TYPE, msb: Int, lsb: Int, defval: String)
            : this(name, hw_type(VarType, msb, lsb), defval)

    constructor(name: String, VarType: DATA_TYPE, defimm : hw_imm)
            : this(name, hw_type(VarType, defimm.imm_value), defimm)

    constructor(name: String, VarType: DATA_TYPE, defval: String)
            : this(name, hw_type(VarType, defval), defval)

    constructor(name: String, src_struct: hw_struct, dimensions : hw_dim_static)
            : this(name, hw_type(src_struct, dimensions), "0")

    constructor(name: String, src_struct: hw_struct, msb: Int, lsb: Int)
            : this(name, hw_type(src_struct, msb, lsb), "0")

    constructor(name: String, src_struct: hw_struct)
            : this(name, src_struct, 0, 0)

    constructor(name : String, msb : Int, lsb : Int, defval : String)
            : this(name, DATA_TYPE.BV_UNSIGNED, msb, lsb, hw_imm(defval))

    fun assign(depow_fracs: hw_fracs, src: hw_param) {
        default_astc.assign(this, depow_fracs, src)
    }

    fun assign(depow_fracs: hw_fracs, src: Int) {
        default_astc.assign(this, depow_fracs, src)
    }

    fun assign(src: hw_param) {
        default_astc.assign(this, src)
    }

    fun assign(src: Int) {
        default_astc.assign(this, src)
    }

    operator fun not(): hw_var {
        return default_astc.AddExpr_op1(OP1_BITWISE_NOT, this)
    }

    operator fun plus(src: hw_param): hw_var {
        return default_astc.AddExpr_op2(OP2_ARITH_ADD, this, src)
    }

    operator fun plus(src: Int): hw_var {
        return default_astc.AddExpr_op2(OP2_ARITH_ADD, this, src)
    }

    operator fun minus(src: hw_param): hw_var {
        return default_astc.AddExpr_op2(OP2_ARITH_SUB, this, src)
    }

    operator fun minus(src: Int): hw_var {
        return default_astc.AddExpr_op2(OP2_ARITH_SUB, this, src)
    }

    operator fun times(src: hw_param): hw_var {
        return default_astc.AddExpr_op2(OP2_ARITH_MUL, this, src)
    }

    operator fun times(src: Int): hw_var {
        return default_astc.AddExpr_op2(OP2_ARITH_MUL, this, src)
    }

    operator fun div(src: hw_param): hw_var {
        return default_astc.AddExpr_op2(OP2_ARITH_DIV, this, src)
    }

    operator fun div(src: Int): hw_var {
        return default_astc.AddExpr_op2(OP2_ARITH_DIV, this, src)
    }

    operator fun rem(src: hw_param): hw_var {
        return default_astc.AddExpr_op2(OP2_ARITH_MOD, this, src)
    }

    operator fun rem(src: Int): hw_var {
        return default_astc.AddExpr_op2(OP2_ARITH_MOD, this, src)
    }

    operator fun get(index: hw_param): hw_var {
        return default_astc.AddExpr_op2(OP2_INDEXED, this, index)
    }

    operator fun get(index: Int): hw_var {
        return default_astc.AddExpr_op2(OP2_INDEXED, this, index)
    }

    operator fun get(msb: hw_param, lsb: hw_param): hw_var {
        return default_astc.AddExpr_op3(OP3_RANGED, this, msb, lsb)
    }

    operator fun get(msb: Int, lsb: Int): hw_var {
        return default_astc.AddExpr_op3(OP3_RANGED, this, hw_imm(msb), hw_imm(lsb))
    }

    operator fun set(index: hw_param, src: hw_param) {
        var depow_fracs = hw_fracs()
        if (index is hw_imm) depow_fracs.add(hw_frac_C(index))
        else depow_fracs.add(hw_frac_V(index as hw_var))
        default_astc.assign(this, depow_fracs, src)
    }

    operator fun set(index: Int, src: hw_param) {
        var depow_fracs = hw_fracs()
        depow_fracs.add(hw_frac_C(index))
        default_astc.assign(this, depow_fracs, src)
    }

    operator fun set(msb: hw_param, lsb: hw_param, src: hw_param) {
        var depow_fracs = hw_fracs()

        if (msb is hw_imm) {
            if (lsb is hw_imm) {
                depow_fracs.add(hw_frac_CC(msb , lsb))
            } else {
                depow_fracs.add(hw_frac_CV(msb, lsb as hw_var))
            }
        } else {
            if (lsb is hw_imm) {
                depow_fracs.add(hw_frac_VC(msb as hw_var, lsb))
            } else {
                depow_fracs.add(hw_frac_VV(msb as hw_var, lsb as hw_var))
            }
        }
        default_astc.assign(this, depow_fracs, src)
    }

    operator fun set(msb: Int, lsb: Int, src: hw_param) {
        var depow_fracs = hw_fracs()
        depow_fracs.add(hw_frac_CC(msb, lsb))
        default_astc.assign(this, depow_fracs, src)
    }

    fun GetDepowered(depow_fracs: hw_fracs): hw_type {
        var ret_dim = hw_dim_static()
        var ret_vartype : DATA_TYPE
        var ret_struct : hw_struct

        // copying dimensions of a variable
        for (i in 0 until vartype.dimensions.size) ret_dim.add(vartype.dimensions[i])
        ret_vartype = vartype.DataType
        ret_struct = vartype.src_struct

        // println("detaching dimensions")
        for (depow_fraction in depow_fracs) {

            //println("depow_fraction ..." + depow_fraction.toString())
            if (ret_dim.isSingle()) {
                // println("undimensioned var")
                ret_dim.clear()
                if (depow_fraction is hw_frac_SubStruct) {
                    // println("retrieving structure...")
                    ret_vartype = depow_fraction.src_struct[depow_fraction.subStructIndex].vartype.DataType
                    ret_struct = depow_fraction.src_struct[depow_fraction.subStructIndex].vartype.src_struct
                    for (dimension in depow_fraction.src_struct[depow_fraction.subStructIndex].vartype.dimensions) {
                        ret_dim.add(dimension)
                    }
                    // println("retrieving structure: done")
                } else {
                    // indexing 1-bit (dim) var
                    ret_dim.add(hw_dim_range_static(0, 0))
                }

            } else {
                // println("dimensioned var")
                if (depow_fraction is hw_frac_SubStruct) ERROR("Depower index generation incorrect, accessing substruct in multidimensional variable")
                else {
                    if ((depow_fraction is hw_frac_C) || (depow_fraction is hw_frac_V)) {
                        // taking indexed variable - detachment of last dimensions
                        ret_dim.removeAt(ret_dim.lastIndex)
                    } else if (depow_fraction is hw_frac_CC) {
                        // replacing last dimension with taken
                        ret_dim.removeAt(ret_dim.lastIndex)
                        ret_dim.add(hw_dim_range_static(depow_fraction.msb.toInt(), depow_fraction.lsb.toInt()))
                    } else continue
                }
            }
        }

        return hw_type(ret_vartype, ret_struct, ret_dim)
    }

    open fun GetFracRef(depow_fracs: hw_fracs) : hw_var_frac {
        depow_fracs.FillSubStructs(this)
        var new_hw_var_frac = hw_var_frac(this, depow_fracs, GetDepowered(depow_fracs))
        new_hw_var_frac.default_astc = default_astc
        return new_hw_var_frac
    }

    open fun GetFracRef(vararg depow_frac: hw_frac) : hw_var_frac {
        var depow_fracs = hw_fracs()
        for (frac in depow_frac) depow_fracs.add(frac)
        return GetFracRef(depow_fracs)
    }

    open fun GetFracRef(index : Int) : hw_var_frac {
        var depow_fracs = hw_fracs()
        depow_fracs.add(index)
        return GetFracRef(depow_fracs)
    }
}

var DUMMY_VAR = hw_var("DUMMY_VAR", hw_type(DATA_TYPE.BV_UNSIGNED, 0, 0), "0")

class hw_var_frac(var src_var : hw_var, var depow_fractions: hw_fracs, vartype : hw_type) : hw_var(src_var.name, vartype, src_var.defimm) {
    init {
        hw_fractured(src_var, depow_fractions)
    }

    override fun GetFracRef(depow_fracs: hw_fracs) : hw_var_frac {
        depow_fracs.FillSubStructs(this)
        var new_depow_fracs = hw_fracs()
        for (frac in depow_fractions) new_depow_fracs.add(frac)
        for (frac in depow_fracs) new_depow_fracs.add(frac)
        var new_hw_var_frac = hw_var_frac(src_var, new_depow_fracs, GetDepowered(depow_fracs))
        new_hw_var_frac.default_astc = default_astc
        return new_hw_var_frac
    }

    override fun GetFracRef(vararg depow_frac: hw_frac) : hw_var_frac {
        var depow_fracs = hw_fracs()
        for (frac in depow_frac) depow_fracs.add(frac)
        return GetFracRef(depow_fracs)
    }
}
