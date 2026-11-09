/*
 * RX gdb server stub
 *
 * Copyright (c) 2019 Yoshinori Sato
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/gdbstub.h"
#include "gdbstub/helpers.h"

int rx_cpu_gdb_read_register(CPUState *cs, GByteArray *mem_buf, int n)
{
    CPURXState *env = cpu_env(cs);

    switch (n) {
    case 0 ... 15:
        return gdb_get_reg32(mem_buf, env->regs[n]);
    case 16:
        return gdb_get_reg32(mem_buf, (env->psw_u) ? env->regs[0] : env->usp);
    case 17:
        return gdb_get_reg32(mem_buf, (!env->psw_u) ? env->regs[0] : env->isp);
    case 18:
        return gdb_get_reg32(mem_buf, rx_cpu_pack_psw(env));
    case 19:
        return gdb_get_reg32(mem_buf, env->pc);
    case 20:
        return gdb_get_reg32(mem_buf, env->intb);
    case 21:
        return gdb_get_reg32(mem_buf, env->bpsw);
    case 22:
        return gdb_get_reg32(mem_buf, env->bpc);
    case 23:
        return gdb_get_reg32(mem_buf, env->fintv);
    case 24:
        return gdb_get_reg32(mem_buf, env->fpsw);
    case 25:
        return 0;
    }
    return 0;
}

int rx_cpu_gdb_write_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    CPURXState *env = cpu_env(cs);
    uint32_t psw;
    switch (n) {
    case 0 ... 15:
        env->regs[n] = ldl_le_p(mem_buf);
        if (n == 0) {
            if (env->psw_u) {
                env->usp = env->regs[0];
            } else {
                env->isp = env->regs[0];
            }
        }
        break;
    case 16:
        env->usp = ldl_le_p(mem_buf);
        if (env->psw_u) {
            env->regs[0] = ldl_le_p(mem_buf);
        }
        break;
    case 17:
        env->isp = ldl_le_p(mem_buf);
        if (!env->psw_u) {
            env->regs[0] = ldl_le_p(mem_buf);
        }
        break;
    case 18:
        psw = ldl_le_p(mem_buf);
        rx_cpu_unpack_psw(env, psw, 1);
        break;
    case 19:
        env->pc = ldl_le_p(mem_buf);
        break;
    case 20:
        env->intb = ldl_le_p(mem_buf);
        break;
    case 21:
        env->bpsw = ldl_le_p(mem_buf);
        break;
    case 22:
        env->bpc = ldl_le_p(mem_buf);
        break;
    case 23:
        env->fintv = ldl_le_p(mem_buf);
        break;
    case 24:
        env->fpsw = ldl_le_p(mem_buf);
        break;
    case 25:
        return 8;
    default:
        return 0;
    }

    return 4;
}

/* The DFPU's registers (rx-dfpu.xml): dr0-dr15, dpsw, dcmr, decnt, depc. */
static int rx_dfpu_gdb_get_reg(CPUState *cs, GByteArray *mem_buf, int n)
{
    CPURXState *env = cpu_env(cs);

    switch (n) {
    case 0 ... 15:
        return gdb_get_reg64(mem_buf, env->dr[n]);
    case 16:
        return gdb_get_reg32(mem_buf, env->dpsw);
    case 17:
        return gdb_get_reg32(mem_buf, env->dcmr);
    case 18:
        return gdb_get_reg32(mem_buf, env->decnt);
    case 19:
        return gdb_get_reg32(mem_buf, env->depc);
    }
    return 0;
}

static int rx_dfpu_gdb_set_reg(CPUState *cs, uint8_t *mem_buf, int n)
{
    CPURXState *env = cpu_env(cs);

    switch (n) {
    case 0 ... 15:
        env->dr[n] = ldq_le_p(mem_buf);
        return 8;
    case 16:
        /* As written, but with DFS and the rounding mode kept in step. */
        rx_cpu_set_dpsw(env, ldl_le_p(mem_buf));
        return 4;
    case 17:
        env->dcmr = ldl_le_p(mem_buf) & 1;
        return 4;
    case 18:
        env->decnt = ldl_le_p(mem_buf) & (R_DECNT_EHM_MASK | R_DECNT_EHS_MASK);
        return 4;
    case 19:
        env->depc = ldl_le_p(mem_buf);
        return 4;
    }
    return 0;
}

void rx_cpu_register_gdb_regs(CPUState *cs)
{
    if (rx_has_feature(cpu_env(cs), RX_FEATURE_DFPU)) {
        gdb_register_coprocessor(cs, rx_dfpu_gdb_get_reg, rx_dfpu_gdb_set_reg,
                                 gdb_find_static_feature("rx-dfpu.xml"));
    }
}
