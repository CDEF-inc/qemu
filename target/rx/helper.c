/*
 *  RX emulation
 *
 *  Copyright (c) 2019 Yoshinori Sato
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
#include "qemu/bitops.h"
#include "cpu.h"
#include "exec/log.h"
#include "accel/tcg/cpu-ldst.h"
#include "hw/core/irq.h"
#include "qemu/plugin.h"

void rx_cpu_unpack_psw(CPURXState *env, uint32_t psw, int rte)
{
    if (env->psw_pm == 0) {
        env->psw_ipl = FIELD_EX32(psw, PSW, IPL);
        if (!rx_has_feature(env, RX_FEATURE_V3E) ||
            rx_world(env) == RX_WORLD_NONSECURE) {
            /* IPL[4] is RXv3e only, and always 0 in the non-secure world */
            env->psw_ipl &= 0xf;
        }
        if (rte) {
            /* PSW.PM can write RTE and RTFI */
            env->psw_pm = FIELD_EX32(psw, PSW, PM);
        }
        env->psw_u = FIELD_EX32(psw, PSW, U);
        env->psw_i = FIELD_EX32(psw, PSW, I);
    }
    env->psw_o = FIELD_EX32(psw, PSW, O) << 31;
    env->psw_s = FIELD_EX32(psw, PSW, S) << 31;
    env->psw_z = 1 - FIELD_EX32(psw, PSW, Z);
    env->psw_c = FIELD_EX32(psw, PSW, C);
}

/*
 * RXv3e worlds: the control register idx (RX_WCR_*) of a world. For the
 * current world that is the register in env; its ISP is R0 while PSW.U
 * is 0, so callers read and write the ISP through rx_world_isp_*.
 */
uint32_t *rx_world_cr(CPURXState *env, int world, int idx)
{
    if (world != rx_world(env)) {
        return &env->world_cr[world][idx];
    }
    switch (idx) {
    case RX_WCR_ISP:
        return env->psw_u ? &env->isp : &env->regs[0];
    case RX_WCR_INTB:
        return &env->intb;
    case RX_WCR_BPC:
        return &env->bpc;
    case RX_WCR_BPSW:
        return &env->bpsw;
    case RX_WCR_EXTB:
        return &env->extb;
    default:
        return &env->ivc;
    }
}

/* Make another world current, exchanging its banked control registers */
void rx_world_switch(CPURXState *env, int world)
{
    int cur = rx_world(env);
    int i;

    if (world == cur) {
        return;
    }
    for (i = 0; i < RX_WCR_NUM; i++) {
        uint32_t *reg = rx_world_cr(env, cur, i);

        env->world_cr[cur][i] = *reg;
        *reg = env->world_cr[world][i];
    }
    env->wid = FIELD_DP32(env->wid, WID, WID, world);
}

void rx_cpu_do_interrupt(CPUState *cs)
{
    CPURXState *env = cpu_env(cs);
    uint32_t save_psw;
    uint64_t last_pc = env->pc;

    env->in_sleep = 0;

    if (env->psw_u) {
        env->usp = env->regs[0];
    } else {
        env->isp = env->regs[0];
    }
    save_psw = rx_cpu_pack_psw(env);
    env->psw_pm = env->psw_i = env->psw_u = 0;

    if (cpu_test_interrupt(cs, CPU_INTERRUPT_FIR)) {
        env->bpc = env->pc;
        env->bpsw = save_psw;
        env->pc = env->fintv;
        env->psw_ipl = 15;
        cpu_reset_interrupt(cs, CPU_INTERRUPT_FIR);
        qemu_set_irq(env->ack, env->ack_irq);
        qemu_plugin_vcpu_interrupt_cb(cs, last_pc);
        qemu_log_mask(CPU_LOG_INT, "fast interrupt raised\n");
    } else if (cpu_test_interrupt(cs, CPU_INTERRUPT_HARD)) {
        env->isp -= 4;
        cpu_stl_le_data(env, env->isp, save_psw);
        env->isp -= 4;
        cpu_stl_le_data(env, env->isp, env->pc);
        env->pc = cpu_ldl_le_data(env, env->intb + env->ack_irq * 4);
        env->psw_ipl = env->ack_ipl;
        cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
        qemu_set_irq(env->ack, env->ack_irq);
        qemu_plugin_vcpu_interrupt_cb(cs, last_pc);
        qemu_log_mask(CPU_LOG_INT, "interrupt 0x%02x raised\n", env->ack_irq);
    } else {
        uint32_t vec = cs->exception_index;
        const char *expname = "unknown exception";

        env->isp -= 4;
        cpu_stl_le_data(env, env->isp, save_psw);
        env->isp -= 4;
        cpu_stl_le_data(env, env->isp, env->pc);

        if (vec < 0x100) {
            env->pc = cpu_ldl_le_data(env, env->extb + vec * 4);
        } else {
            env->pc = cpu_ldl_le_data(env, env->intb + (vec & 0xff) * 4);
        }

        if (vec == 30) {
            /* Non-maskable interrupt */
            qemu_plugin_vcpu_interrupt_cb(cs, last_pc);
        } else {
            qemu_plugin_vcpu_exception_cb(cs, last_pc);
        }

        switch (vec) {
        case 20:
            expname = "privilege violation";
            break;
        case 21:
            expname = "access exception";
            break;
        case 23:
            expname = "illegal instruction";
            break;
        case 25:
            expname = "fpu exception";
            break;
        case 30:
            expname = "non-maskable interrupt";
            break;
        case 0x100 ... 0x1ff:
            expname = "unconditional trap";
        }
        qemu_log_mask(CPU_LOG_INT, "exception 0x%02x [%s] raised\n",
                      (vec & 0xff), expname);
    }
    env->regs[0] = env->isp;
}

bool rx_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    CPURXState *env = cpu_env(cs);
    int accept = 0;
    /* hardware interrupt (Normal) */
    if ((interrupt_request & CPU_INTERRUPT_HARD) &&
        env->psw_i && (env->psw_ipl < env->req_ipl)) {
        env->ack_irq = env->req_irq;
        env->ack_ipl = env->req_ipl;
        accept = 1;
    }
    /* hardware interrupt (FIR) */
    if ((interrupt_request & CPU_INTERRUPT_FIR) &&
        env->psw_i && (env->psw_ipl < 15)) {
        accept = 1;
    }
    if (accept) {
        rx_cpu_do_interrupt(cs);
        return true;
    }
    return false;
}

hwaddr rx_cpu_get_phys_addr_debug(CPUState *cs, vaddr addr)
{
    return addr;
}
