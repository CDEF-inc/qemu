/*
 *  RX translation
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
#include "qemu/bswap.h"
#include "qemu/qemu-print.h"
#include "cpu.h"
#include "tcg/tcg-op.h"
#include "exec/helper-proto.h"
#include "exec/helper-gen.h"
#include "exec/translator.h"
#include "exec/translation-block.h"
#include "exec/log.h"

#define HELPER_H "helper.h"
#include "exec/helper-info.c.inc"
#undef  HELPER_H


typedef struct DisasContext {
    DisasContextBase base;
    CPURXState *env;
    uint32_t pc;
    uint32_t tb_flags;
    uint32_t features;
} DisasContext;

typedef struct DisasCompare {
    TCGv_i32 value;
    TCGv_i32 temp;
    TCGCond cond;
} DisasCompare;

const char *rx_crname(uint8_t cr)
{
    static const char *cr_names[] = {
        "psw", "pc", "usp", "fpsw", "", "", "", "",
        "bpsw", "bpc", "isp", "fintv", "intb", "extb", "wpri", "wid"
    };
    if (cr >= ARRAY_SIZE(cr_names)) {
        return "illegal";
    }
    return cr_names[cr];
}

/* Target-specific values for dc->base.is_jmp.  */
#define DISAS_JUMP    DISAS_TARGET_0
#define DISAS_UPDATE  DISAS_TARGET_1
#define DISAS_EXIT    DISAS_TARGET_2

/* global register indexes */
static TCGv_i32 cpu_regs[16];
static TCGv_i32 cpu_psw_o, cpu_psw_s, cpu_psw_z, cpu_psw_c;
static TCGv_i32 cpu_psw_i, cpu_psw_pm, cpu_psw_u, cpu_psw_ipl;
static TCGv_i32 cpu_usp, cpu_fpsw, cpu_bpsw, cpu_bpc, cpu_isp;
static TCGv_i32 cpu_fintv, cpu_intb, cpu_pc, cpu_extb, cpu_li;

#define cpu_sp cpu_regs[0]

static inline MemOp mo_endian(DisasContext *dc)
{
    return MO_LE;
}

/* decoder helper */
static uint32_t decode_load_bytes(DisasContext *ctx, uint32_t insn,
                                  int i, int n)
{
    while (++i <= n) {
        uint8_t b = translator_ldub(ctx->env, &ctx->base, ctx->base.pc_next++);
        insn |= b << (32 - i * 8);
    }
    return insn;
}

static uint32_t li(DisasContext *ctx, int sz)
{
    vaddr addr;
    uint32_t tmp;
    MemOp endian = mo_endian(ctx);
    CPURXState *env = ctx->env;
    addr = ctx->base.pc_next;

    switch (sz) {
    case 1:
        ctx->base.pc_next += 1;
        return (int8_t)translator_ldub(env, &ctx->base, addr);
    case 2:
        ctx->base.pc_next += 2;
        return (int16_t) translator_lduw_end(env, &ctx->base, addr, endian);
    case 3:
        ctx->base.pc_next += 3;
        tmp = (int8_t)translator_ldub(env, &ctx->base, addr + 2);
        tmp <<= 16;
        tmp |= translator_lduw_end(env, &ctx->base, addr, endian);
        return tmp;
    case 0:
        ctx->base.pc_next += 4;
        return translator_ldl_end(env, &ctx->base, addr, endian);
    default:
        g_assert_not_reached();
    }
    return 0;
}

static int bdsp_s(DisasContext *ctx, int d)
{
    /*
     * 0 -> 8
     * 1 -> 9
     * 2 -> 10
     * 3 -> 3
     * :
     * 7 -> 7
     */
    if (d < 3) {
        d += 8;
    }
    return d;
}

/* Whether the core has all of the RX_FEATURE_* bits in @features */
/*
 * mvfac* #imm: the high bit of the shift is inverted; the reserved field
 * 01b acts as #2 (measured on the RX72M)
 */
static int mvfac_imm(DisasContext *ctx, int x)
{
    return x == 1 ? 2 : x ^ 2;
}

static bool has_feature(DisasContext *ctx, uint32_t features)
{
    return (ctx->features & features) == features;
}

/* The current RXv3e world (RX_WORLD_*) */
static int ctx_world(DisasContext *ctx)
{
    return FIELD_EX32(ctx->tb_flags, TB, WORLD);
}

/* Include the auto-generated decoder. */
#include "decode-insns.c.inc"

void rx_cpu_dump_state(CPUState *cs, FILE *f, int flags)
{
    CPURXState *env = cpu_env(cs);
    int i;
    uint32_t psw;

    psw = rx_cpu_pack_psw(env);
    qemu_fprintf(f, "pc=0x%08x psw=0x%08x\n",
                 env->pc, psw);
    for (i = 0; i < 16; i += 4) {
        qemu_fprintf(f, "r%d=0x%08x r%d=0x%08x r%d=0x%08x r%d=0x%08x\n",
                     i, env->regs[i], i + 1, env->regs[i + 1],
                     i + 2, env->regs[i + 2], i + 3, env->regs[i + 3]);
    }
}

static void gen_goto_tb(DisasContext *dc, unsigned tb_slot_idx, vaddr dest)
{
    if (translator_use_goto_tb(&dc->base, dest)) {
        tcg_gen_goto_tb(tb_slot_idx);
        tcg_gen_movi_i32(cpu_pc, dest);
        tcg_gen_exit_tb(dc->base.tb, tb_slot_idx);
    } else {
        tcg_gen_movi_i32(cpu_pc, dest);
        tcg_gen_lookup_and_goto_ptr();
    }
    dc->base.is_jmp = DISAS_NORETURN;
}

/* generic load wrapper */
static void rx_gen_ld(DisasContext *ctx, MemOp size, TCGv_i32 reg, TCGv_i32 mem)
{
    tcg_gen_qemu_ld_i32(reg, mem, 0, size | MO_SIGN | mo_endian(ctx));
}

/* unsigned load wrapper */
static void rx_gen_ldu(DisasContext *ctx, MemOp size, TCGv_i32 reg, TCGv_i32 mem)
{
    tcg_gen_qemu_ld_i32(reg, mem, 0, size | mo_endian(ctx));
}

/* generic store wrapper */
static void rx_gen_st(DisasContext *ctx, MemOp size, TCGv_i32 reg, TCGv_i32 mem)
{
    tcg_gen_qemu_st_i32(reg, mem, 0, size | mo_endian(ctx));
}

/* [ri, rb] */
static void rx_gen_regindex(DisasContext *ctx, TCGv_i32 mem,
                            int size, int ri, int rb)
{
    tcg_gen_shli_i32(mem, cpu_regs[ri], size);
    tcg_gen_add_i32(mem, mem, cpu_regs[rb]);
}

/* dsp[reg] */
static TCGv_i32 rx_index_addr(DisasContext *ctx, TCGv_i32 mem,
                              int ld, int size, int reg)
{
    uint32_t dsp;

    switch (ld) {
    case 0:
        return cpu_regs[reg];
    case 1:
        dsp = translator_ldub(ctx->env, &ctx->base, ctx->base.pc_next) << size;
        tcg_gen_addi_i32(mem, cpu_regs[reg], dsp);
        ctx->base.pc_next += 1;
        return mem;
    case 2:
        dsp = translator_lduw_end(ctx->env, &ctx->base, ctx->base.pc_next,
                                  mo_endian(ctx)) << size;
        tcg_gen_addi_i32(mem, cpu_regs[reg], dsp);
        ctx->base.pc_next += 2;
        return mem;
    default:
        g_assert_not_reached();
    }
}

static inline MemOp mi_to_mop(unsigned mi)
{
    static const MemOp mop[5] = { MO_SB, MO_SW, MO_UL, MO_UW, MO_UB };
    tcg_debug_assert(mi < 5);
    return mop[mi];
}

/* load source operand */
static TCGv_i32 rx_load_source(DisasContext *ctx, TCGv_i32 mem,
                               int ld, int mi, int rs)
{
    TCGv_i32 addr;
    MemOp mop;
    if (ld < 3) {
        mop = mi_to_mop(mi);
        addr = rx_index_addr(ctx, mem, ld, mop & MO_SIZE, rs);
        tcg_gen_qemu_ld_i32(mem, addr, 0, mop | mo_endian(ctx));
        return mem;
    } else {
        return cpu_regs[rs];
    }
}

/* Processor mode check */
static int is_privileged(DisasContext *ctx, int is_exception)
{
    if (FIELD_EX32(ctx->tb_flags, PSW, PM)) {
        if (is_exception) {
            gen_helper_raise_privilege_violation(tcg_env);
        }
        return 0;
    } else {
        return 1;
    }
}

/* generate QEMU condition */
static void psw_cond(DisasCompare *dc, uint32_t cond)
{
    tcg_debug_assert(cond < 16);
    switch (cond) {
    case 0: /* z */
        dc->cond = TCG_COND_EQ;
        dc->value = cpu_psw_z;
        break;
    case 1: /* nz */
        dc->cond = TCG_COND_NE;
        dc->value = cpu_psw_z;
        break;
    case 2: /* c */
        dc->cond = TCG_COND_NE;
        dc->value = cpu_psw_c;
        break;
    case 3: /* nc */
        dc->cond = TCG_COND_EQ;
        dc->value = cpu_psw_c;
        break;
    case 4: /* gtu (C& ~Z) == 1 */
    case 5: /* leu (C& ~Z) == 0 */
        tcg_gen_setcondi_i32(TCG_COND_NE, dc->temp, cpu_psw_z, 0);
        tcg_gen_and_i32(dc->temp, dc->temp, cpu_psw_c);
        dc->cond = (cond == 4) ? TCG_COND_NE : TCG_COND_EQ;
        dc->value = dc->temp;
        break;
    case 6: /* pz (S == 0) */
        dc->cond = TCG_COND_GE;
        dc->value = cpu_psw_s;
        break;
    case 7: /* n (S == 1) */
        dc->cond = TCG_COND_LT;
        dc->value = cpu_psw_s;
        break;
    case 8: /* ge (S^O)==0 */
    case 9: /* lt (S^O)==1 */
        tcg_gen_xor_i32(dc->temp, cpu_psw_o, cpu_psw_s);
        dc->cond = (cond == 8) ? TCG_COND_GE : TCG_COND_LT;
        dc->value = dc->temp;
        break;
    case 10: /* gt ((S^O)|Z)==0 */
    case 11: /* le ((S^O)|Z)==1 */
        tcg_gen_xor_i32(dc->temp, cpu_psw_o, cpu_psw_s);
        tcg_gen_sari_i32(dc->temp, dc->temp, 31);
        tcg_gen_andc_i32(dc->temp, cpu_psw_z, dc->temp);
        dc->cond = (cond == 10) ? TCG_COND_NE : TCG_COND_EQ;
        dc->value = dc->temp;
        break;
    case 12: /* o */
        dc->cond = TCG_COND_LT;
        dc->value = cpu_psw_o;
        break;
    case 13: /* no */
        dc->cond = TCG_COND_GE;
        dc->value = cpu_psw_o;
        break;
    case 14: /* always true */
        dc->cond = TCG_COND_ALWAYS;
        dc->value = dc->temp;
        break;
    case 15: /* always false */
        dc->cond = TCG_COND_NEVER;
        dc->value = dc->temp;
        break;
    }
}

/*
 * mvfc / pushc of the RXv3e control registers: IVC (5) and WID (15) in
 * every world (WID.BWID reads as 0 outside the root world), WPRI (14) in
 * the root and secure worlds (Table 1.1; 0 where not accessible).
 */
static void move_from_cr_v3e(DisasContext *ctx, TCGv_i32 ret, int cr)
{
    int world = ctx_world(ctx);

    switch (cr) {
    case 5:
        tcg_gen_ld_i32(ret, tcg_env, offsetof(CPURXState, ivc));
        break;
    case 14:
        if (world == RX_WORLD_NONSECURE) {
            tcg_gen_movi_i32(ret, 0);
        } else {
            tcg_gen_ld_i32(ret, tcg_env, offsetof(CPURXState, wpri));
        }
        break;
    default:
        tcg_gen_ld_i32(ret, tcg_env, offsetof(CPURXState, wid));
        if (world != RX_WORLD_ROOT) {
            tcg_gen_andi_i32(ret, ret, ~R_WID_BWID_MASK);
        }
        break;
    }
}

static void move_from_cr(DisasContext *ctx, TCGv_i32 ret, int cr, uint32_t pc)
{
    if (cr == 11 && ctx_world(ctx) != RX_WORLD_ROOT) {
        /* FINTV is not accessible outside the root world */
        tcg_gen_movi_i32(ret, 0);
        return;
    }
    switch (cr) {
    case 0:     /* PSW */
        gen_helper_pack_psw(ret, tcg_env);
        break;
    case 1:     /* PC */
        tcg_gen_movi_i32(ret, pc);
        break;
    case 2:     /* USP */
        if (FIELD_EX32(ctx->tb_flags, PSW, U)) {
            tcg_gen_mov_i32(ret, cpu_sp);
        } else {
            tcg_gen_mov_i32(ret, cpu_usp);
        }
        break;
    case 3:     /* FPSW */
        tcg_gen_mov_i32(ret, cpu_fpsw);
        break;
    case 8:     /* BPSW */
        tcg_gen_mov_i32(ret, cpu_bpsw);
        break;
    case 9:     /* BPC */
        tcg_gen_mov_i32(ret, cpu_bpc);
        break;
    case 10:    /* ISP */
        if (FIELD_EX32(ctx->tb_flags, PSW, U)) {
            tcg_gen_mov_i32(ret, cpu_isp);
        } else {
            tcg_gen_mov_i32(ret, cpu_sp);
        }
        break;
    case 11:    /* FINTV */
        tcg_gen_mov_i32(ret, cpu_fintv);
        break;
    case 12:    /* INTB */
        tcg_gen_mov_i32(ret, cpu_intb);
        break;
    case 13:    /* EXTB */
        if (has_feature(ctx, RX_FEATURE_V2)) {
            tcg_gen_mov_i32(ret, cpu_extb);
            break;
        }
        /* fall through */
    case 5:     /* IVC */
    case 14:    /* WPRI */
    case 15:    /* WID */
        if (has_feature(ctx, RX_FEATURE_V3E) && cr != 13) {
            move_from_cr_v3e(ctx, ret, cr);
            break;
        }
        /* fall through */
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "Unimplement control register %d", cr);
        /* Unimplement registers return 0 */
        tcg_gen_movi_i32(ret, 0);
        break;
    }
}

static void move_to_cr(DisasContext *ctx, TCGv_i32 val, int cr)
{
    int world = ctx_world(ctx);

    /*
     * RXv3e (Table 1.1): FINTV only in the root world, WPRI in the root and
     * secure worlds, IVC and WID only in supervisor mode in the root world.
     */
    if ((cr == 11 && world != RX_WORLD_ROOT) ||
        (cr == 14 && world == RX_WORLD_NONSECURE) ||
        ((cr == 5 || cr == 15) &&
         (world != RX_WORLD_ROOT || !is_privileged(ctx, 0)))) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "disallow control register write %s", rx_crname(cr));
        return;
    }
    if (cr >= 8 && !is_privileged(ctx, 0)) {
        /* Some control registers can only be written in privileged mode. */
        qemu_log_mask(LOG_GUEST_ERROR,
                      "disallow control register write %s", rx_crname(cr));
        return;
    }
    switch (cr) {
    case 0:     /* PSW */
        gen_helper_set_psw(tcg_env, val);
        if (is_privileged(ctx, 0)) {
            /* PSW.{I,U} may be updated here. exit TB. */
            ctx->base.is_jmp = DISAS_UPDATE;
        }
        break;
    /* case 1: to PC not supported */
    case 2:     /* USP */
        if (FIELD_EX32(ctx->tb_flags, PSW, U)) {
            tcg_gen_mov_i32(cpu_sp, val);
        } else {
            tcg_gen_mov_i32(cpu_usp, val);
        }
        break;
    case 3:     /* FPSW */
        gen_helper_set_fpsw(tcg_env, val);
        break;
    case 8:     /* BPSW */
        if (ctx_world(ctx) == RX_WORLD_NONSECURE) {
            /* Bit 28 of BPSW_NS is always 0 (PSW.IPL[4] in that world) */
            tcg_gen_andi_i32(cpu_bpsw, val, ~(1u << 28));
        } else {
            tcg_gen_mov_i32(cpu_bpsw, val);
        }
        break;
    case 9:     /* BPC */
        tcg_gen_mov_i32(cpu_bpc, val);
        break;
    case 10:    /* ISP */
        if (FIELD_EX32(ctx->tb_flags, PSW, U)) {
            tcg_gen_mov_i32(cpu_isp, val);
        } else {
            tcg_gen_mov_i32(cpu_sp, val);
        }
        break;
    case 11:    /* FINTV */
        tcg_gen_mov_i32(cpu_fintv, val);
        break;
    case 12:    /* INTB */
        tcg_gen_mov_i32(cpu_intb, val);
        break;
    case 13:    /* EXTB */
        if (has_feature(ctx, RX_FEATURE_V2)) {
            tcg_gen_mov_i32(cpu_extb, val);
            break;
        }
        /* fall through */
    case 5:     /* IVC */
    case 14:    /* WPRI */
    case 15:    /* WID */
        if (has_feature(ctx, RX_FEATURE_V3E) && cr != 13) {
            gen_helper_mvtc_v3e(tcg_env, tcg_constant_i32(cr), val);
            break;
        }
        /* fall through */
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Unimplement control register %d", cr);
        break;
    }
}

static void push(DisasContext *ctx, TCGv_i32 val)
{
    tcg_gen_subi_i32(cpu_sp, cpu_sp, 4);
    rx_gen_st(ctx, MO_32, val, cpu_sp);
}

static void pop(DisasContext *ctx, TCGv_i32 ret)
{
    rx_gen_ld(ctx, MO_32, ret, cpu_sp);
    tcg_gen_addi_i32(cpu_sp, cpu_sp, 4);
}

/* mov.<bwl> rs,dsp5[rd] */
static bool trans_MOV_rm(DisasContext *ctx, arg_MOV_rm *a)
{
    TCGv_i32 mem;
    mem = tcg_temp_new_i32();
    tcg_gen_addi_i32(mem, cpu_regs[a->rd], a->dsp << a->sz);
    rx_gen_st(ctx, a->sz, cpu_regs[a->rs], mem);
    return true;
}

/* mov.<bwl> dsp5[rs],rd */
static bool trans_MOV_mr(DisasContext *ctx, arg_MOV_mr *a)
{
    TCGv_i32 mem;
    mem = tcg_temp_new_i32();
    tcg_gen_addi_i32(mem, cpu_regs[a->rs], a->dsp << a->sz);
    rx_gen_ld(ctx, a->sz, cpu_regs[a->rd], mem);
    return true;
}

/* mov.l #uimm4,rd */
/* mov.l #uimm8,rd */
/* mov.l #imm,rd */
static bool trans_MOV_ir(DisasContext *ctx, arg_MOV_ir *a)
{
    tcg_gen_movi_i32(cpu_regs[a->rd], a->imm);
    return true;
}

/* mov.<bwl> #uimm8,dsp[rd] */
/* mov.<bwl> #imm, dsp[rd] */
static bool trans_MOV_im(DisasContext *ctx, arg_MOV_im *a)
{
    TCGv_i32 imm, mem;
    if (a->sz == 3) {
        return false;
    }
    imm = tcg_constant_i32(a->imm);
    mem = tcg_temp_new_i32();
    tcg_gen_addi_i32(mem, cpu_regs[a->rd], a->dsp << a->sz);
    rx_gen_st(ctx, a->sz, imm, mem);
    return true;
}

/* mov.<bwl> [ri,rb],rd */
static bool trans_MOV_ar(DisasContext *ctx, arg_MOV_ar *a)
{
    TCGv_i32 mem;
    mem = tcg_temp_new_i32();
    rx_gen_regindex(ctx, mem, a->sz, a->ri, a->rb);
    rx_gen_ld(ctx, a->sz, cpu_regs[a->rd], mem);
    return true;
}

/* mov.<bwl> rd,[ri,rb] */
static bool trans_MOV_ra(DisasContext *ctx, arg_MOV_ra *a)
{
    TCGv_i32 mem;
    mem = tcg_temp_new_i32();
    rx_gen_regindex(ctx, mem, a->sz, a->ri, a->rb);
    rx_gen_st(ctx, a->sz, cpu_regs[a->rs], mem);
    return true;
}

/* mov.<bwl> dsp[rs],dsp[rd] */
/* mov.<bwl> rs,dsp[rd] */
/* mov.<bwl> dsp[rs],rd */
/* mov.<bwl> rs,rd */
static bool trans_MOV_mm(DisasContext *ctx, arg_MOV_mm *a)
{
    TCGv_i32 tmp, mem, addr;

    if (a->lds == 3 && a->ldd == 3) {
        /* mov.<bwl> rs,rd */
        tcg_gen_ext_i32(cpu_regs[a->rd], cpu_regs[a->rs], a->sz | MO_SIGN);
        return true;
    }

    mem = tcg_temp_new_i32();
    if (a->lds == 3) {
        /* mov.<bwl> rs,dsp[rd] */
        addr = rx_index_addr(ctx, mem, a->ldd, a->sz, a->rs);
        rx_gen_st(ctx, a->sz, cpu_regs[a->rd], addr);
    } else if (a->ldd == 3) {
        /* mov.<bwl> dsp[rs],rd */
        addr = rx_index_addr(ctx, mem, a->lds, a->sz, a->rs);
        rx_gen_ld(ctx, a->sz, cpu_regs[a->rd], addr);
    } else {
        /* mov.<bwl> dsp[rs],dsp[rd] */
        tmp = tcg_temp_new_i32();
        addr = rx_index_addr(ctx, mem, a->lds, a->sz, a->rs);
        rx_gen_ld(ctx, a->sz, tmp, addr);
        addr = rx_index_addr(ctx, mem, a->ldd, a->sz, a->rd);
        rx_gen_st(ctx, a->sz, tmp, addr);
    }
    return true;
}

/* mov.<bwl> rs,[rd+] */
/* mov.<bwl> rs,[-rd] */
static bool trans_MOV_rp(DisasContext *ctx, arg_MOV_rp *a)
{
    TCGv_i32 val;
    if (a->sz == 3) {
        return false;
    }
    val = tcg_temp_new_i32();
    tcg_gen_mov_i32(val, cpu_regs[a->rs]);
    if (a->ad == 1) {
        tcg_gen_subi_i32(cpu_regs[a->rd], cpu_regs[a->rd], 1 << a->sz);
    }
    rx_gen_st(ctx, a->sz, val, cpu_regs[a->rd]);
    if (a->ad == 0) {
        tcg_gen_addi_i32(cpu_regs[a->rd], cpu_regs[a->rd], 1 << a->sz);
    }
    return true;
}

/* mov.<bwl> [rd+],rs */
/* mov.<bwl> [-rd],rs */
static bool trans_MOV_pr(DisasContext *ctx, arg_MOV_pr *a)
{
    TCGv_i32 val;
    if (a->sz == 3) {
        return false;
    }
    val = tcg_temp_new_i32();
    if (a->ad == 1) {
        tcg_gen_subi_i32(cpu_regs[a->rd], cpu_regs[a->rd], 1 << a->sz);
    }
    rx_gen_ld(ctx, a->sz, val, cpu_regs[a->rd]);
    if (a->ad == 0) {
        tcg_gen_addi_i32(cpu_regs[a->rd], cpu_regs[a->rd], 1 << a->sz);
    }
    tcg_gen_mov_i32(cpu_regs[a->rs], val);
    return true;
}

/* movu.<bw> dsp5[rs],rd */
/* movu.<bw> dsp[rs],rd */
static bool trans_MOVU_mr(DisasContext *ctx, arg_MOVU_mr *a)
{
    TCGv_i32 mem;
    mem = tcg_temp_new_i32();
    tcg_gen_addi_i32(mem, cpu_regs[a->rs], a->dsp << a->sz);
    rx_gen_ldu(ctx, a->sz, cpu_regs[a->rd], mem);
    return true;
}

/* movu.<bw> rs,rd */
static bool trans_MOVU_rr(DisasContext *ctx, arg_MOVU_rr *a)
{
    tcg_gen_ext_i32(cpu_regs[a->rd], cpu_regs[a->rs], a->sz);
    return true;
}

/* movu.<bw> [ri,rb],rd */
static bool trans_MOVU_ar(DisasContext *ctx, arg_MOVU_ar *a)
{
    TCGv_i32 mem;
    mem = tcg_temp_new_i32();
    rx_gen_regindex(ctx, mem, a->sz, a->ri, a->rb);
    rx_gen_ldu(ctx, a->sz, cpu_regs[a->rd], mem);
    return true;
}

/* movu.<bw> [rd+],rs */
/* mov.<bw> [-rd],rs */
static bool trans_MOVU_pr(DisasContext *ctx, arg_MOVU_pr *a)
{
    TCGv_i32 val;
    val = tcg_temp_new_i32();
    if (a->ad == 1) {
        tcg_gen_subi_i32(cpu_regs[a->rd], cpu_regs[a->rd], 1 << a->sz);
    }
    rx_gen_ldu(ctx, a->sz, val, cpu_regs[a->rd]);
    if (a->ad == 0) {
        tcg_gen_addi_i32(cpu_regs[a->rd], cpu_regs[a->rd], 1 << a->sz);
    }
    tcg_gen_mov_i32(cpu_regs[a->rs], val);
    return true;
}


/* pop rd */
static bool trans_POP(DisasContext *ctx, arg_POP *a)
{
    /* mov.l [r0+], rd */
    arg_MOV_rp mov_a;
    mov_a.rd = 0;
    mov_a.rs = a->rd;
    mov_a.ad = 0;
    mov_a.sz = MO_32;
    trans_MOV_pr(ctx, &mov_a);
    return true;
}

/* popc cr */
static bool trans_POPC(DisasContext *ctx, arg_POPC *a)
{
    TCGv_i32 val;
    val = tcg_temp_new_i32();
    pop(ctx, val);
    move_to_cr(ctx, val, a->cr);
    return true;
}

/* popm rd-rd2 */
static bool trans_POPM(DisasContext *ctx, arg_POPM *a)
{
    int r;
    if (a->rd == 0 || a->rd >= a->rd2) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Invalid  register ranges r%d-r%d", a->rd, a->rd2);
    }
    r = a->rd;
    while (r <= a->rd2 && r < 16) {
        pop(ctx, cpu_regs[r++]);
    }
    return true;
}


/* push.<bwl> rs */
static bool trans_PUSH_r(DisasContext *ctx, arg_PUSH_r *a)
{
    TCGv_i32 val;
    val = tcg_temp_new_i32();
    tcg_gen_mov_i32(val, cpu_regs[a->rs]);
    tcg_gen_subi_i32(cpu_sp, cpu_sp, 4);
    rx_gen_st(ctx, a->sz, val, cpu_sp);
    return true;
}

/* push.<bwl> dsp[rs] */
static bool trans_PUSH_m(DisasContext *ctx, arg_PUSH_m *a)
{
    TCGv_i32 mem, val, addr;
    mem = tcg_temp_new_i32();
    val = tcg_temp_new_i32();
    addr = rx_index_addr(ctx, mem, a->ld, a->sz, a->rs);
    rx_gen_ld(ctx, a->sz, val, addr);
    tcg_gen_subi_i32(cpu_sp, cpu_sp, 4);
    rx_gen_st(ctx, a->sz, val, cpu_sp);
    return true;
}

/* pushc rx */
static bool trans_PUSHC(DisasContext *ctx, arg_PUSHC *a)
{
    TCGv_i32 val;
    val = tcg_temp_new_i32();
    move_from_cr(ctx, val, a->cr, ctx->pc);
    push(ctx, val);
    return true;
}

/* pushm rs-rs2 */
static bool trans_PUSHM(DisasContext *ctx, arg_PUSHM *a)
{
    int r;

    if (a->rs == 0 || a->rs >= a->rs2) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Invalid  register ranges r%d-r%d", a->rs, a->rs2);
    }
    r = a->rs2;
    while (r >= a->rs && r >= 0) {
        push(ctx, cpu_regs[r--]);
    }
    return true;
}

/* xchg rs,rd */
static bool trans_XCHG_rr(DisasContext *ctx, arg_XCHG_rr *a)
{
    TCGv_i32 tmp;
    tmp = tcg_temp_new_i32();
    tcg_gen_mov_i32(tmp, cpu_regs[a->rs]);
    tcg_gen_mov_i32(cpu_regs[a->rs], cpu_regs[a->rd]);
    tcg_gen_mov_i32(cpu_regs[a->rd], tmp);
    return true;
}

/* xchg dsp[rs].<mi>,rd */
static bool trans_XCHG_mr(DisasContext *ctx, arg_XCHG_mr *a)
{
    TCGv_i32 mem, addr;
    mem = tcg_temp_new_i32();
    switch (a->mi) {
    case 0: /* dsp[rs].b */
    case 1: /* dsp[rs].w */
    case 2: /* dsp[rs].l */
        addr = rx_index_addr(ctx, mem, a->ld, a->mi, a->rs);
        break;
    case 3: /* dsp[rs].uw */
    case 4: /* dsp[rs].ub */
        addr = rx_index_addr(ctx, mem, a->ld, 4 - a->mi, a->rs);
        break;
    default:
        g_assert_not_reached();
    }
    tcg_gen_atomic_xchg_i32(cpu_regs[a->rd], addr, cpu_regs[a->rd],
                            0, mi_to_mop(a->mi));
    return true;
}

static void stcond(TCGCond cond, int rd, int imm)
{
    TCGv_i32 z;
    TCGv_i32 _imm;
    z = tcg_constant_i32(0);
    _imm = tcg_constant_i32(imm);
    tcg_gen_movcond_i32(cond, cpu_regs[rd], cpu_psw_z, z,
                        _imm, cpu_regs[rd]);
}

/* stz #imm,rd */
static bool trans_STZ(DisasContext *ctx, arg_STZ *a)
{
    stcond(TCG_COND_EQ, a->rd, a->imm);
    return true;
}

/* stnz #imm,rd */
static bool trans_STNZ(DisasContext *ctx, arg_STNZ *a)
{
    stcond(TCG_COND_NE, a->rd, a->imm);
    return true;
}

/* stz rs,rd */
static bool trans_STZ_rr(DisasContext *ctx, arg_STZ_rr *a)
{
    if (!has_feature(ctx, RX_FEATURE_V2)) {
        return false;
    }
    tcg_gen_movcond_i32(TCG_COND_EQ, cpu_regs[a->rd], cpu_psw_z,
                        tcg_constant_i32(0), cpu_regs[a->rs], cpu_regs[a->rd]);
    return true;
}

/* stnz rs,rd */
static bool trans_STNZ_rr(DisasContext *ctx, arg_STNZ_rr *a)
{
    if (!has_feature(ctx, RX_FEATURE_V2)) {
        return false;
    }
    tcg_gen_movcond_i32(TCG_COND_NE, cpu_regs[a->rd], cpu_psw_z,
                        tcg_constant_i32(0), cpu_regs[a->rs], cpu_regs[a->rd]);
    return true;
}

/* movli [rs],rd */
static bool trans_MOVLI(DisasContext *ctx, arg_MOVLI *a)
{
    if (!has_feature(ctx, RX_FEATURE_V2)) {
        return false;
    }
    rx_gen_ld(ctx, MO_32, cpu_regs[a->rd], cpu_regs[a->rs]);
    tcg_gen_movi_i32(cpu_li, 1);
    return true;
}

/* movco rs,[rd] */
static bool trans_MOVCO(DisasContext *ctx, arg_MOVCO *a)
{
    TCGLabel *fail, *done;

    if (!has_feature(ctx, RX_FEATURE_V2)) {
        return false;
    }
    fail = gen_new_label();
    done = gen_new_label();
    tcg_gen_brcondi_i32(TCG_COND_EQ, cpu_li, 0, fail);
    rx_gen_st(ctx, MO_32, cpu_regs[a->rs], cpu_regs[a->rd]);
    tcg_gen_movi_i32(cpu_regs[a->rs], 0);
    tcg_gen_br(done);
    gen_set_label(fail);
    tcg_gen_movi_i32(cpu_regs[a->rs], 1);
    gen_set_label(done);
    tcg_gen_movi_i32(cpu_li, 0);
    return true;
}

/* sccnd.<bwl> rd */
/* sccnd.<bwl> dsp:[rd] */
static bool trans_SCCnd(DisasContext *ctx, arg_SCCnd *a)
{
    DisasCompare dc;
    TCGv_i32 val, mem, addr;
    dc.temp = tcg_temp_new_i32();
    psw_cond(&dc, a->cd);
    if (a->ld < 3) {
        val = tcg_temp_new_i32();
        mem = tcg_temp_new_i32();
        tcg_gen_setcondi_i32(dc.cond, val, dc.value, 0);
        addr = rx_index_addr(ctx, mem, a->sz, a->ld, a->rd);
        rx_gen_st(ctx, a->sz, val, addr);
    } else {
        tcg_gen_setcondi_i32(dc.cond, cpu_regs[a->rd], dc.value, 0);
    }
    return true;
}

/* rtsd #imm */
static bool trans_RTSD_i(DisasContext *ctx, arg_RTSD_i *a)
{
    tcg_gen_addi_i32(cpu_sp, cpu_sp, a->imm  << 2);
    pop(ctx, cpu_pc);
    ctx->base.is_jmp = DISAS_JUMP;
    return true;
}

/* rtsd #imm, rd-rd2 */
static bool trans_RTSD_irr(DisasContext *ctx, arg_RTSD_irr *a)
{
    int dst;
    int adj;

    if (a->rd2 >= a->rd) {
        adj = a->imm - (a->rd2 - a->rd + 1);
    } else {
        adj = a->imm - (15 - a->rd + 1);
    }

    tcg_gen_addi_i32(cpu_sp, cpu_sp, adj << 2);
    dst = a->rd;
    while (dst <= a->rd2 && dst < 16) {
        pop(ctx, cpu_regs[dst++]);
    }
    pop(ctx, cpu_pc);
    ctx->base.is_jmp = DISAS_JUMP;
    return true;
}

typedef void (*op2fn)(TCGv_i32 ret, TCGv_i32 arg1);
typedef void (*op3fn)(TCGv_i32 ret, TCGv_i32 arg1, TCGv_i32 arg2);

static void rx_gen_op_rr(op2fn opr, int dst, int src)
{
    opr(cpu_regs[dst], cpu_regs[src]);
}

static void rx_gen_op_rrr(op3fn opr, int dst, int src, int src2)
{
    opr(cpu_regs[dst], cpu_regs[src], cpu_regs[src2]);
}

static void rx_gen_op_irr(op3fn opr, int dst, int src, uint32_t src2)
{
    TCGv_i32 imm = tcg_constant_i32(src2);
    opr(cpu_regs[dst], cpu_regs[src], imm);
}

static void rx_gen_op_mr(op3fn opr, DisasContext *ctx,
                         int dst, int src, int ld, int mi)
{
    TCGv_i32 val, mem;
    mem = tcg_temp_new_i32();
    val = rx_load_source(ctx, mem, ld, mi, src);
    opr(cpu_regs[dst], cpu_regs[dst], val);
}

static void rx_and(TCGv_i32 ret, TCGv_i32 arg1, TCGv_i32 arg2)
{
    tcg_gen_and_i32(cpu_psw_s, arg1, arg2);
    tcg_gen_mov_i32(cpu_psw_z, cpu_psw_s);
    tcg_gen_mov_i32(ret, cpu_psw_s);
}

/* and #uimm:4, rd */
/* and #imm, rd */
static bool trans_AND_ir(DisasContext *ctx, arg_AND_ir *a)
{
    rx_gen_op_irr(rx_and, a->rd, a->rd, a->imm);
    return true;
}

/* and dsp[rs], rd */
/* and rs,rd */
static bool trans_AND_mr(DisasContext *ctx, arg_AND_mr *a)
{
    rx_gen_op_mr(rx_and, ctx, a->rd, a->rs, a->ld, a->mi);
    return true;
}

/* and rs,rs2,rd */
static bool trans_AND_rrr(DisasContext *ctx, arg_AND_rrr *a)
{
    rx_gen_op_rrr(rx_and, a->rd, a->rs, a->rs2);
    return true;
}

static void rx_or(TCGv_i32 ret, TCGv_i32 arg1, TCGv_i32 arg2)
{
    tcg_gen_or_i32(cpu_psw_s, arg1, arg2);
    tcg_gen_mov_i32(cpu_psw_z, cpu_psw_s);
    tcg_gen_mov_i32(ret, cpu_psw_s);
}

/* or #uimm:4, rd */
/* or #imm, rd */
static bool trans_OR_ir(DisasContext *ctx, arg_OR_ir *a)
{
    rx_gen_op_irr(rx_or, a->rd, a->rd, a->imm);
    return true;
}

/* or dsp[rs], rd */
/* or rs,rd */
static bool trans_OR_mr(DisasContext *ctx, arg_OR_mr *a)
{
    rx_gen_op_mr(rx_or, ctx, a->rd, a->rs, a->ld, a->mi);
    return true;
}

/* or rs,rs2,rd */
static bool trans_OR_rrr(DisasContext *ctx, arg_OR_rrr *a)
{
    rx_gen_op_rrr(rx_or, a->rd, a->rs, a->rs2);
    return true;
}

static void rx_xor(TCGv_i32 ret, TCGv_i32 arg1, TCGv_i32 arg2)
{
    tcg_gen_xor_i32(cpu_psw_s, arg1, arg2);
    tcg_gen_mov_i32(cpu_psw_z, cpu_psw_s);
    tcg_gen_mov_i32(ret, cpu_psw_s);
}

/* xor #imm, rd */
static bool trans_XOR_ir(DisasContext *ctx, arg_XOR_ir *a)
{
    rx_gen_op_irr(rx_xor, a->rd, a->rd, a->imm);
    return true;
}

/* xor dsp[rs], rd */
/* xor rs,rd */
static bool trans_XOR_mr(DisasContext *ctx, arg_XOR_mr *a)
{
    rx_gen_op_mr(rx_xor, ctx, a->rd, a->rs, a->ld, a->mi);
    return true;
}

/* xor rs,rs2,rd */
static bool trans_XOR_rrr(DisasContext *ctx, arg_XOR_rrr *a)
{
    if (!has_feature(ctx, RX_FEATURE_V3)) {
        return false;
    }
    rx_gen_op_rrr(rx_xor, a->rd, a->rs, a->rs2);
    return true;
}

/*
 * bfmov / bfmovz #slsb, #dlsb, #width, rs, rd: width bits at slsb of rs
 * to dlsb of rd. The operands are encoded in #imm16 as
 * ((dlsb + width) & 31) << 10 | dlsb << 5 | ((dlsb - slsb) & 31); width
 * is 1 to 31 (0 here moves nothing).
 */
static void rx_bfmov(DisasContext *ctx, int rs, int rd, bool zero)
{
    uint32_t imm = li(ctx, 2) & 0xffff;
    int dlsb = extract32(imm, 5, 5);
    int slsb = (dlsb - extract32(imm, 0, 5)) & 31;
    int width = (extract32(imm, 10, 5) - dlsb) & 31;
    uint32_t mask = (uint32_t)(MAKE_64BIT_MASK(0, width) << dlsb);
    TCGv_i32 val = tcg_temp_new_i32();

    tcg_gen_shri_i32(val, cpu_regs[rs], slsb);
    tcg_gen_shli_i32(val, val, dlsb);
    tcg_gen_andi_i32(val, val, mask);
    if (zero) {
        tcg_gen_mov_i32(cpu_regs[rd], val);
    } else {
        tcg_gen_andi_i32(cpu_regs[rd], cpu_regs[rd], ~mask);
        tcg_gen_or_i32(cpu_regs[rd], cpu_regs[rd], val);
    }
}

static bool trans_BFMOV(DisasContext *ctx, arg_BFMOV *a)
{
    if (!has_feature(ctx, RX_FEATURE_V3)) {
        return false;
    }
    rx_bfmov(ctx, a->rs, a->rd, false);
    return true;
}

static bool trans_BFMOVZ(DisasContext *ctx, arg_BFMOVZ *a)
{
    if (!has_feature(ctx, RX_FEATURE_V3)) {
        return false;
    }
    rx_bfmov(ctx, a->rs, a->rd, true);
    return true;
}

/*
 * Double-precision FPU (RXv3, the dfpu property). DR0-DR15 and the control
 * registers live in env; the operations are helpers (op_helper.c).
 */
static bool dfpu(DisasContext *ctx)
{
    return has_feature(ctx, RX_FEATURE_DFPU);
}

static int dr_offset(int n)
{
    return offsetof(CPURXState, dr) + n * sizeof(uint64_t);
}

static int dcr_offset(int n)
{
    static const int offset[] = {
        offsetof(CPURXState, dpsw), offsetof(CPURXState, dcmr),
        offsetof(CPURXState, decnt), offsetof(CPURXState, depc),
    };
    return offset[n];
}

#define DFPU_OP2(name, op)                                              \
    static bool trans_##name(DisasContext *ctx, arg_##name * a)         \
    {                                                                   \
        if (!dfpu(ctx)) {                                               \
            return false;                                               \
        }                                                               \
        gen_helper_##op(tcg_env, tcg_constant_i32(a->rd),               \
                        tcg_constant_i32(a->rs), tcg_constant_i32(a->rs2), \
                        tcg_constant_i32(ctx->pc));                     \
        return true;                                                    \
    }

DFPU_OP2(DADD, dadd)
DFPU_OP2(DSUB, dsub)
DFPU_OP2(DMUL, dmul)
DFPU_OP2(DDIV, ddiv)

static bool trans_DCMP(DisasContext *ctx, arg_DCMP * a)
{
    if (!dfpu(ctx)) {
        return false;
    }
    gen_helper_dcmp(tcg_env, tcg_constant_i32(a->cm), tcg_constant_i32(a->rs),
                    tcg_constant_i32(a->rs2), tcg_constant_i32(ctx->pc));
    return true;
}

#define DFPU_OP1(name, op)                                              \
    static bool trans_##name(DisasContext *ctx, arg_##name * a)         \
    {                                                                   \
        if (!dfpu(ctx)) {                                               \
            return false;                                               \
        }                                                               \
        gen_helper_##op(tcg_env, tcg_constant_i32(a->rd),               \
                        tcg_constant_i32(a->rs), tcg_constant_i32(ctx->pc)); \
        return true;                                                    \
    }

DFPU_OP1(DSQRT, dsqrt)
DFPU_OP1(DTOI, dtoi)
DFPU_OP1(DTOU, dtou)
DFPU_OP1(DTOF, dtof)
DFPU_OP1(DROUND, dround)

/* ftod / itod / utod rs, drd */
#define DFPU_FROM_GPR(name, op)                                         \
    static bool trans_##name(DisasContext *ctx, arg_##name * a)         \
    {                                                                   \
        if (!dfpu(ctx)) {                                               \
            return false;                                               \
        }                                                               \
        gen_helper_##op(tcg_env, tcg_constant_i32(a->rd), cpu_regs[a->rs], \
                        tcg_constant_i32(ctx->pc));                     \
        return true;                                                    \
    }

DFPU_FROM_GPR(FTOD, ftod)
DFPU_FROM_GPR(ITOD, itod)
DFPU_FROM_GPR(UTOD, utod)

/* dmov.d drs, drd; dabs / dneg drs, drd (no flags) */
static void gen_dfpu_sign(int rd, int rs, int op)
{
    TCGv_i64 val = tcg_temp_new_i64();

    tcg_gen_ld_i64(val, tcg_env, dr_offset(rs));
    if (op == 1) {
        tcg_gen_andi_i64(val, val, INT64_MAX);
    } else if (op == 2) {
        tcg_gen_xori_i64(val, val, INT64_MIN);
    }
    tcg_gen_st_i64(val, tcg_env, dr_offset(rd));
}

static bool trans_DMOV_dd(DisasContext *ctx, arg_DMOV_dd *a)
{
    if (!dfpu(ctx)) {
        return false;
    }
    gen_dfpu_sign(a->rd, a->rs, 0);
    return true;
}

static bool trans_DABS(DisasContext *ctx, arg_DABS *a)
{
    if (!dfpu(ctx)) {
        return false;
    }
    gen_dfpu_sign(a->rd, a->rs, 1);
    return true;
}

static bool trans_DNEG(DisasContext *ctx, arg_DNEG *a)
{
    if (!dfpu(ctx)) {
        return false;
    }
    gen_dfpu_sign(a->rd, a->rs, 2);
    return true;
}

/*
 * Write a 32-bit value to the upper (hi) or lower half of drd; dmov.d to
 * the upper half (zero) clears the lower half.
 */
static void gen_dmov_to_dr(int rd, TCGv_i32 val, bool hi, bool zero)
{
    TCGv_i64 dr = tcg_temp_new_i64();
    TCGv_i64 v = tcg_temp_new_i64();

    tcg_gen_extu_i32_i64(v, val);
    if (zero) {
        tcg_gen_shli_i64(dr, v, 32);
    } else {
        tcg_gen_ld_i64(dr, tcg_env, dr_offset(rd));
        tcg_gen_deposit_i64(dr, dr, v, hi ? 32 : 0, 32);
    }
    tcg_gen_st_i64(dr, tcg_env, dr_offset(rd));
}

/* dmov.d / dmov.l rs, drhd */
static bool trans_DMOV_rdh(DisasContext *ctx, arg_DMOV_rdh * a)
{
    if (!dfpu(ctx)) {
        return false;
    }
    gen_dmov_to_dr(a->rd, cpu_regs[a->rs], true, a->d);
    return true;
}

/* dmov.l rs, drld */
static bool trans_DMOV_rdl(DisasContext *ctx, arg_DMOV_rdl *a)
{
    if (!dfpu(ctx)) {
        return false;
    }
    gen_dmov_to_dr(a->rd, cpu_regs[a->rs], false, false);
    return true;
}

/* dmov.d / dmov.l #imm, drhd; dmov.l #imm, drld */
static bool trans_DMOV_ir(DisasContext *ctx, arg_DMOV_ir *a)
{
    if (!dfpu(ctx) || (a->sub != 0 && a->sub != 2 && a->sub != 3)) {
        return false;
    }
    gen_dmov_to_dr(a->rd, tcg_constant_i32(li(ctx, 0)), a->sub != 0,
                   a->sub == 3);
    return true;
}

/* dmov.l drhs / drls, rd */
static void gen_dmov_from_dr(int rd, int rs, bool hi)
{
    TCGv_i64 dr = tcg_temp_new_i64();

    tcg_gen_ld_i64(dr, tcg_env, dr_offset(rs));
    if (hi) {
        tcg_gen_extrh_i64_i32(cpu_regs[rd], dr);
    } else {
        tcg_gen_extrl_i64_i32(cpu_regs[rd], dr);
    }
}

static bool trans_DMOV_dhr(DisasContext *ctx, arg_DMOV_dhr *a)
{
    if (!dfpu(ctx)) {
        return false;
    }
    gen_dmov_from_dr(a->rd, a->rs, true);
    return true;
}

static bool trans_DMOV_dlr(DisasContext *ctx, arg_DMOV_dlr *a)
{
    if (!dfpu(ctx)) {
        return false;
    }
    gen_dmov_from_dr(a->rd, a->rs, false);
    return true;
}

/*
 * dmov.d drs, dsp[rd] / dmov.d dsp[rs], drd: dsp comes before the byte
 * with the DR number in its upper half. dsp is scaled by 4, not 8 as the
 * RXv3 manual's note says (measured on the RX72M; Renesas CC-RX encodes
 * 8[R7] as dsp 2).
 */
static int dmov_mem_dr(DisasContext *ctx)
{
    int b = translator_ldub(ctx->env, &ctx->base, ctx->base.pc_next++);

    return (b & 0xf) ? -1 : b >> 4;
}

static bool trans_DMOV_dm(DisasContext *ctx, arg_DMOV_dm *a)
{
    TCGv_i32 mem, addr;
    TCGv_i64 val;
    int dr;

    if (!dfpu(ctx) || a->ld == 3) {
        return false;
    }
    mem = tcg_temp_new_i32();
    addr = rx_index_addr(ctx, mem, a->ld, MO_32, a->rd);
    dr = dmov_mem_dr(ctx);
    if (dr < 0) {
        return false;
    }
    val = tcg_temp_new_i64();
    tcg_gen_ld_i64(val, tcg_env, dr_offset(dr));
    tcg_gen_qemu_st_i64(val, addr, 0, MO_64 | mo_endian(ctx));
    return true;
}

static bool trans_DMOV_md(DisasContext *ctx, arg_DMOV_md *a)
{
    TCGv_i32 mem, addr;
    TCGv_i64 val;
    int dr;

    if (!dfpu(ctx) || a->ld == 3) {
        return false;
    }
    mem = tcg_temp_new_i32();
    addr = rx_index_addr(ctx, mem, a->ld, MO_32, a->rs);
    dr = dmov_mem_dr(ctx);
    if (dr < 0) {
        return false;
    }
    val = tcg_temp_new_i64();
    tcg_gen_qemu_ld_i64(val, addr, 0, MO_64 | mo_endian(ctx));
    tcg_gen_st_i64(val, tcg_env, dr_offset(dr));
    return true;
}

/* mvtdc rs, dcrd / mvfdc dcrs, rd */
static bool trans_MVTDC(DisasContext *ctx, arg_MVTDC *a)
{
    if (!dfpu(ctx) || a->cr > 3) {
        return false;
    }
    gen_helper_mvtdc(tcg_env, tcg_constant_i32(a->cr), cpu_regs[a->rs]);
    return true;
}

static bool trans_MVFDC(DisasContext *ctx, arg_MVFDC *a)
{
    if (!dfpu(ctx) || a->cr > 3) {
        return false;
    }
    tcg_gen_ld_i32(cpu_regs[a->rd], tcg_env, dcr_offset(a->cr));
    return true;
}

/* mvfdr: Z = DCMR.RES */
static bool trans_MVFDR(DisasContext *ctx, arg_MVFDR *a)
{
    if (!dfpu(ctx)) {
        return false;
    }
    tcg_gen_ld_i32(cpu_psw_z, tcg_env, offsetof(CPURXState, dcmr));
    tcg_gen_andi_i32(cpu_psw_z, cpu_psw_z, 1);
    tcg_gen_xori_i32(cpu_psw_z, cpu_psw_z, 1);
    return true;
}

/*
 * dpushm / dpopm .d drs-drs2 (8 bytes each) and .l dcrs-dcrs2 (4 bytes
 * each); n is the number of registers less one. dpushm pushes from the
 * last register, so the first one ends up at the lowest address.
 */
static bool dpushm_ok(DisasContext *ctx, int rs, int n, int last)
{
    return dfpu(ctx) && rs + n <= last;
}

static bool trans_DPUSHM_d(DisasContext *ctx, arg_DPUSHM_d *a)
{
    TCGv_i64 val;
    int r;

    if (!dpushm_ok(ctx, a->rs, a->n, 15)) {
        return false;
    }
    val = tcg_temp_new_i64();
    for (r = a->rs + a->n; r >= a->rs; r--) {
        tcg_gen_ld_i64(val, tcg_env, dr_offset(r));
        tcg_gen_subi_i32(cpu_sp, cpu_sp, 8);
        tcg_gen_qemu_st_i64(val, cpu_sp, 0, MO_64 | mo_endian(ctx));
    }
    return true;
}

static bool trans_DPOPM_d(DisasContext *ctx, arg_DPOPM_d *a)
{
    TCGv_i64 val;
    int r;

    if (!dpushm_ok(ctx, a->rs, a->n, 15)) {
        return false;
    }
    val = tcg_temp_new_i64();
    for (r = a->rs; r <= a->rs + a->n; r++) {
        tcg_gen_qemu_ld_i64(val, cpu_sp, 0, MO_64 | mo_endian(ctx));
        tcg_gen_addi_i32(cpu_sp, cpu_sp, 8);
        tcg_gen_st_i64(val, tcg_env, dr_offset(r));
    }
    return true;
}

static bool trans_DPUSHM_l(DisasContext *ctx, arg_DPUSHM_l *a)
{
    TCGv_i32 val;
    int r;

    if (!dpushm_ok(ctx, a->rs, a->n, 3)) {
        return false;
    }
    val = tcg_temp_new_i32();
    for (r = a->rs + a->n; r >= a->rs; r--) {
        tcg_gen_ld_i32(val, tcg_env, dcr_offset(r));
        push(ctx, val);
    }
    return true;
}

static bool trans_DPOPM_l(DisasContext *ctx, arg_DPOPM_l *a)
{
    TCGv_i32 val;
    int r;

    if (!dpushm_ok(ctx, a->rs, a->n, 3)) {
        return false;
    }
    val = tcg_temp_new_i32();
    for (r = a->rs; r <= a->rs + a->n; r++) {
        pop(ctx, val);
        gen_helper_set_dcr(tcg_env, tcg_constant_i32(r), val);
    }
    return true;
}

/*
 * save / rstr: the register bank save function, optional on RXv3 cores
 * (the CPU property banks; 0 makes the instructions undefined).
 */
static bool rx_reg_banks(DisasContext *ctx)
{
    return has_feature(ctx, RX_FEATURE_V3) && ctx->env->nr_banks;
}

/* save #uimm8 */
static bool trans_SAVE_i(DisasContext *ctx, arg_SAVE_i *a)
{
    if (!rx_reg_banks(ctx)) {
        return false;
    }
    if (is_privileged(ctx, 1)) {
        gen_helper_save(tcg_env, tcg_constant_i32(a->imm));
    }
    return true;
}

/* save rs */
static bool trans_SAVE_r(DisasContext *ctx, arg_SAVE_r *a)
{
    if (!rx_reg_banks(ctx)) {
        return false;
    }
    if (is_privileged(ctx, 1)) {
        gen_helper_save(tcg_env, cpu_regs[a->rs]);
    }
    return true;
}

/* rstr #uimm8 */
static bool trans_RSTR_i(DisasContext *ctx, arg_RSTR_i *a)
{
    if (!rx_reg_banks(ctx)) {
        return false;
    }
    if (is_privileged(ctx, 1)) {
        gen_helper_rstr(tcg_env, tcg_constant_i32(a->imm));
    }
    return true;
}

/* rstr rs */
static bool trans_RSTR_r(DisasContext *ctx, arg_RSTR_r *a)
{
    if (!rx_reg_banks(ctx)) {
        return false;
    }
    if (is_privileged(ctx, 1)) {
        gen_helper_rstr(tcg_env, cpu_regs[a->rs]);
    }
    return true;
}

static void rx_tst(TCGv_i32 ret, TCGv_i32 arg1, TCGv_i32 arg2)
{
    tcg_gen_and_i32(cpu_psw_s, arg1, arg2);
    tcg_gen_mov_i32(cpu_psw_z, cpu_psw_s);
}

/* tst #imm, rd */
static bool trans_TST_ir(DisasContext *ctx, arg_TST_ir *a)
{
    rx_gen_op_irr(rx_tst, a->rd, a->rd, a->imm);
    return true;
}

/* tst dsp[rs], rd */
/* tst rs, rd */
static bool trans_TST_mr(DisasContext *ctx, arg_TST_mr *a)
{
    rx_gen_op_mr(rx_tst, ctx, a->rd, a->rs, a->ld, a->mi);
    return true;
}

static void rx_not(TCGv_i32 ret, TCGv_i32 arg1)
{
    tcg_gen_not_i32(ret, arg1);
    tcg_gen_mov_i32(cpu_psw_z, ret);
    tcg_gen_mov_i32(cpu_psw_s, ret);
}

/* not rd */
/* not rs, rd */
static bool trans_NOT_rr(DisasContext *ctx, arg_NOT_rr *a)
{
    rx_gen_op_rr(rx_not, a->rd, a->rs);
    return true;
}

static void rx_neg(TCGv_i32 ret, TCGv_i32 arg1)
{
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_psw_o, arg1, 0x80000000);
    tcg_gen_neg_i32(ret, arg1);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_psw_c, ret, 0);
    tcg_gen_mov_i32(cpu_psw_z, ret);
    tcg_gen_mov_i32(cpu_psw_s, ret);
}


/* neg rd */
/* neg rs, rd */
static bool trans_NEG_rr(DisasContext *ctx, arg_NEG_rr *a)
{
    rx_gen_op_rr(rx_neg, a->rd, a->rs);
    return true;
}

/* ret = arg1 + arg2 + psw_c */
static void rx_adc(TCGv_i32 ret, TCGv_i32 arg1, TCGv_i32 arg2)
{
    TCGv_i32 z = tcg_constant_i32(0);
    tcg_gen_add2_i32(cpu_psw_s, cpu_psw_c, arg1, z, cpu_psw_c, z);
    tcg_gen_add2_i32(cpu_psw_s, cpu_psw_c, cpu_psw_s, cpu_psw_c, arg2, z);
    tcg_gen_xor_i32(cpu_psw_o, cpu_psw_s, arg1);
    tcg_gen_xor_i32(cpu_psw_z, arg1, arg2);
    tcg_gen_andc_i32(cpu_psw_o, cpu_psw_o, cpu_psw_z);
    tcg_gen_mov_i32(cpu_psw_z, cpu_psw_s);
    tcg_gen_mov_i32(ret, cpu_psw_s);
}

/* adc #imm, rd */
static bool trans_ADC_ir(DisasContext *ctx, arg_ADC_ir *a)
{
    rx_gen_op_irr(rx_adc, a->rd, a->rd, a->imm);
    return true;
}

/* adc rs, rd */
static bool trans_ADC_rr(DisasContext *ctx, arg_ADC_rr *a)
{
    rx_gen_op_rrr(rx_adc, a->rd, a->rd, a->rs);
    return true;
}

/* adc dsp[rs], rd */
static bool trans_ADC_mr(DisasContext *ctx, arg_ADC_mr *a)
{
    /* mi only 2 */
    if (a->mi != 2) {
        return false;
    }
    rx_gen_op_mr(rx_adc, ctx, a->rd, a->rs, a->ld, a->mi);
    return true;
}

/* ret = arg1 + arg2 */
static void rx_add(TCGv_i32 ret, TCGv_i32 arg1, TCGv_i32 arg2)
{
    TCGv_i32 z = tcg_constant_i32(0);
    tcg_gen_add2_i32(cpu_psw_s, cpu_psw_c, arg1, z, arg2, z);
    tcg_gen_xor_i32(cpu_psw_o, cpu_psw_s, arg1);
    tcg_gen_xor_i32(cpu_psw_z, arg1, arg2);
    tcg_gen_andc_i32(cpu_psw_o, cpu_psw_o, cpu_psw_z);
    tcg_gen_mov_i32(cpu_psw_z, cpu_psw_s);
    tcg_gen_mov_i32(ret, cpu_psw_s);
}

/* add #uimm4, rd */
/* add #imm, rs, rd */
static bool trans_ADD_irr(DisasContext *ctx, arg_ADD_irr *a)
{
    rx_gen_op_irr(rx_add, a->rd, a->rs2, a->imm);
    return true;
}

/* add rs, rd */
/* add dsp[rs], rd */
static bool trans_ADD_mr(DisasContext *ctx, arg_ADD_mr *a)
{
    rx_gen_op_mr(rx_add, ctx, a->rd, a->rs, a->ld, a->mi);
    return true;
}

/* add rs, rs2, rd */
static bool trans_ADD_rrr(DisasContext *ctx, arg_ADD_rrr *a)
{
    rx_gen_op_rrr(rx_add, a->rd, a->rs, a->rs2);
    return true;
}

/* ret = arg1 - arg2 */
static void rx_sub(TCGv_i32 ret, TCGv_i32 arg1, TCGv_i32 arg2)
{
    tcg_gen_sub_i32(cpu_psw_s, arg1, arg2);
    tcg_gen_setcond_i32(TCG_COND_GEU, cpu_psw_c, arg1, arg2);
    tcg_gen_xor_i32(cpu_psw_o, cpu_psw_s, arg1);
    tcg_gen_xor_i32(cpu_psw_z, arg1, arg2);
    tcg_gen_and_i32(cpu_psw_o, cpu_psw_o, cpu_psw_z);
    tcg_gen_mov_i32(cpu_psw_z, cpu_psw_s);
    /* CMP not required return */
    if (ret) {
        tcg_gen_mov_i32(ret, cpu_psw_s);
    }
}

static void rx_cmp(TCGv_i32 dummy, TCGv_i32 arg1, TCGv_i32 arg2)
{
    rx_sub(NULL, arg1, arg2);
}

/* ret = arg1 - arg2 - !psw_c */
/* -> ret = arg1 + ~arg2 + psw_c */
static void rx_sbb(TCGv_i32 ret, TCGv_i32 arg1, TCGv_i32 arg2)
{
    TCGv_i32 temp;
    temp = tcg_temp_new_i32();
    tcg_gen_not_i32(temp, arg2);
    rx_adc(ret, arg1, temp);
}

/* cmp #imm4, rs2 */
/* cmp #imm8, rs2 */
/* cmp #imm, rs2 */
static bool trans_CMP_ir(DisasContext *ctx, arg_CMP_ir *a)
{
    rx_gen_op_irr(rx_cmp, 0, a->rs2, a->imm);
    return true;
}

/* cmp rs, rs2 */
/* cmp dsp[rs], rs2 */
static bool trans_CMP_mr(DisasContext *ctx, arg_CMP_mr *a)
{
    rx_gen_op_mr(rx_cmp, ctx, a->rd, a->rs, a->ld, a->mi);
    return true;
}

/* sub #imm4, rd */
static bool trans_SUB_ir(DisasContext *ctx, arg_SUB_ir *a)
{
    rx_gen_op_irr(rx_sub, a->rd, a->rd, a->imm);
    return true;
}

/* sub rs, rd */
/* sub dsp[rs], rd */
static bool trans_SUB_mr(DisasContext *ctx, arg_SUB_mr *a)
{
    rx_gen_op_mr(rx_sub, ctx, a->rd, a->rs, a->ld, a->mi);
    return true;
}

/* sub rs2, rs, rd */
static bool trans_SUB_rrr(DisasContext *ctx, arg_SUB_rrr *a)
{
    rx_gen_op_rrr(rx_sub, a->rd, a->rs2, a->rs);
    return true;
}

/* sbb rs, rd */
static bool trans_SBB_rr(DisasContext *ctx, arg_SBB_rr *a)
{
    rx_gen_op_rrr(rx_sbb, a->rd, a->rd, a->rs);
    return true;
}

/* sbb dsp[rs], rd */
static bool trans_SBB_mr(DisasContext *ctx, arg_SBB_mr *a)
{
    /* mi only 2 */
    if (a->mi != 2) {
        return false;
    }
    rx_gen_op_mr(rx_sbb, ctx, a->rd, a->rs, a->ld, a->mi);
    return true;
}

/* abs rd */
/* abs rs, rd */
static bool trans_ABS_rr(DisasContext *ctx, arg_ABS_rr *a)
{
    rx_gen_op_rr(tcg_gen_abs_i32, a->rd, a->rs);
    return true;
}

/* max #imm, rd */
static bool trans_MAX_ir(DisasContext *ctx, arg_MAX_ir *a)
{
    rx_gen_op_irr(tcg_gen_smax_i32, a->rd, a->rd, a->imm);
    return true;
}

/* max rs, rd */
/* max dsp[rs], rd */
static bool trans_MAX_mr(DisasContext *ctx, arg_MAX_mr *a)
{
    rx_gen_op_mr(tcg_gen_smax_i32, ctx, a->rd, a->rs, a->ld, a->mi);
    return true;
}

/* min #imm, rd */
static bool trans_MIN_ir(DisasContext *ctx, arg_MIN_ir *a)
{
    rx_gen_op_irr(tcg_gen_smin_i32, a->rd, a->rd, a->imm);
    return true;
}

/* min rs, rd */
/* min dsp[rs], rd */
static bool trans_MIN_mr(DisasContext *ctx, arg_MIN_mr *a)
{
    rx_gen_op_mr(tcg_gen_smin_i32, ctx, a->rd, a->rs, a->ld, a->mi);
    return true;
}

/* mul #uimm4, rd */
/* mul #imm, rd */
static bool trans_MUL_ir(DisasContext *ctx, arg_MUL_ir *a)
{
    rx_gen_op_irr(tcg_gen_mul_i32, a->rd, a->rd, a->imm);
    return true;
}

/* mul rs, rd */
/* mul dsp[rs], rd */
static bool trans_MUL_mr(DisasContext *ctx, arg_MUL_mr *a)
{
    rx_gen_op_mr(tcg_gen_mul_i32, ctx, a->rd, a->rs, a->ld, a->mi);
    return true;
}

/* mul rs, rs2, rd */
static bool trans_MUL_rrr(DisasContext *ctx, arg_MUL_rrr *a)
{
    rx_gen_op_rrr(tcg_gen_mul_i32, a->rd, a->rs, a->rs2);
    return true;
}

/* emul #imm, rd */
static bool trans_EMUL_ir(DisasContext *ctx, arg_EMUL_ir *a)
{
    TCGv_i32 imm = tcg_constant_i32(a->imm);
    if (a->rd > 14) {
        qemu_log_mask(LOG_GUEST_ERROR, "rd too large %d", a->rd);
    }
    tcg_gen_muls2_i32(cpu_regs[a->rd], cpu_regs[(a->rd + 1) & 15],
                      cpu_regs[a->rd], imm);
    return true;
}

/* emul rs, rd */
/* emul dsp[rs], rd */
static bool trans_EMUL_mr(DisasContext *ctx, arg_EMUL_mr *a)
{
    TCGv_i32 val, mem;
    if (a->rd > 14) {
        qemu_log_mask(LOG_GUEST_ERROR, "rd too large %d", a->rd);
    }
    mem = tcg_temp_new_i32();
    val = rx_load_source(ctx, mem, a->ld, a->mi, a->rs);
    tcg_gen_muls2_i32(cpu_regs[a->rd], cpu_regs[(a->rd + 1) & 15],
                      cpu_regs[a->rd], val);
    return true;
}

/* emulu #imm, rd */
static bool trans_EMULU_ir(DisasContext *ctx, arg_EMULU_ir *a)
{
    TCGv_i32 imm = tcg_constant_i32(a->imm);
    if (a->rd > 14) {
        qemu_log_mask(LOG_GUEST_ERROR, "rd too large %d", a->rd);
    }
    tcg_gen_mulu2_i32(cpu_regs[a->rd], cpu_regs[(a->rd + 1) & 15],
                      cpu_regs[a->rd], imm);
    return true;
}

/* emulu rs, rd */
/* emulu dsp[rs], rd */
static bool trans_EMULU_mr(DisasContext *ctx, arg_EMULU_mr *a)
{
    TCGv_i32 val, mem;
    if (a->rd > 14) {
        qemu_log_mask(LOG_GUEST_ERROR, "rd too large %d", a->rd);
    }
    mem = tcg_temp_new_i32();
    val = rx_load_source(ctx, mem, a->ld, a->mi, a->rs);
    tcg_gen_mulu2_i32(cpu_regs[a->rd], cpu_regs[(a->rd + 1) & 15],
                      cpu_regs[a->rd], val);
    return true;
}

static void rx_div(TCGv_i32 ret, TCGv_i32 arg1, TCGv_i32 arg2)
{
    gen_helper_div(ret, tcg_env, arg1, arg2);
}

static void rx_divu(TCGv_i32 ret, TCGv_i32 arg1, TCGv_i32 arg2)
{
    gen_helper_divu(ret, tcg_env, arg1, arg2);
}

/* div #imm, rd */
static bool trans_DIV_ir(DisasContext *ctx, arg_DIV_ir *a)
{
    rx_gen_op_irr(rx_div, a->rd, a->rd, a->imm);
    return true;
}

/* div rs, rd */
/* div dsp[rs], rd */
static bool trans_DIV_mr(DisasContext *ctx, arg_DIV_mr *a)
{
    rx_gen_op_mr(rx_div, ctx, a->rd, a->rs, a->ld, a->mi);
    return true;
}

/* divu #imm, rd */
static bool trans_DIVU_ir(DisasContext *ctx, arg_DIVU_ir *a)
{
    rx_gen_op_irr(rx_divu, a->rd, a->rd, a->imm);
    return true;
}

/* divu rs, rd */
/* divu dsp[rs], rd */
static bool trans_DIVU_mr(DisasContext *ctx, arg_DIVU_mr *a)
{
    rx_gen_op_mr(rx_divu, ctx, a->rd, a->rs, a->ld, a->mi);
    return true;
}


/* shll #imm:5, rd */
/* shll #imm:5, rs2, rd */
static bool trans_SHLL_irr(DisasContext *ctx, arg_SHLL_irr *a)
{
    TCGv_i32 tmp;
    tmp = tcg_temp_new_i32();
    if (a->imm) {
        tcg_gen_sari_i32(cpu_psw_c, cpu_regs[a->rs2], 32 - a->imm);
        tcg_gen_shli_i32(cpu_regs[a->rd], cpu_regs[a->rs2], a->imm);
        tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_psw_o, cpu_psw_c, 0);
        tcg_gen_setcondi_i32(TCG_COND_EQ, tmp, cpu_psw_c, 0xffffffff);
        tcg_gen_or_i32(cpu_psw_o, cpu_psw_o, tmp);
        tcg_gen_setcondi_i32(TCG_COND_NE, cpu_psw_c, cpu_psw_c, 0);
    } else {
        tcg_gen_mov_i32(cpu_regs[a->rd], cpu_regs[a->rs2]);
        tcg_gen_movi_i32(cpu_psw_c, 0);
        tcg_gen_movi_i32(cpu_psw_o, 0);
    }
    tcg_gen_mov_i32(cpu_psw_z, cpu_regs[a->rd]);
    tcg_gen_mov_i32(cpu_psw_s, cpu_regs[a->rd]);
    return true;
}

/* shll rs, rd */
static bool trans_SHLL_rr(DisasContext *ctx, arg_SHLL_rr *a)
{
    TCGLabel *noshift, *done;
    TCGv_i32 count, tmp;

    noshift = gen_new_label();
    done = gen_new_label();
    /* if (cpu_regs[a->rs]) { */
    tcg_gen_brcondi_i32(TCG_COND_EQ, cpu_regs[a->rs], 0, noshift);
    count = tcg_temp_new_i32();
    tmp = tcg_temp_new_i32();
    tcg_gen_andi_i32(tmp, cpu_regs[a->rs], 31);
    tcg_gen_sub_i32(count, tcg_constant_i32(32), tmp);
    tcg_gen_sar_i32(cpu_psw_c, cpu_regs[a->rd], count);
    tcg_gen_shl_i32(cpu_regs[a->rd], cpu_regs[a->rd], tmp);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_psw_o, cpu_psw_c, 0);
    tcg_gen_setcondi_i32(TCG_COND_EQ, tmp, cpu_psw_c, 0xffffffff);
    tcg_gen_or_i32(cpu_psw_o, cpu_psw_o, tmp);
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_psw_c, cpu_psw_c, 0);
    tcg_gen_br(done);
    /* } else { */
    gen_set_label(noshift);
    tcg_gen_movi_i32(cpu_psw_c, 0);
    tcg_gen_movi_i32(cpu_psw_o, 0);
    /* } */
    gen_set_label(done);
    tcg_gen_mov_i32(cpu_psw_z, cpu_regs[a->rd]);
    tcg_gen_mov_i32(cpu_psw_s, cpu_regs[a->rd]);
    return true;
}

static void shiftr_imm(uint32_t rd, uint32_t rs, uint32_t imm,
                       unsigned int alith)
{
    static void (* const gen_sXri[])(TCGv_i32 ret, TCGv_i32 arg1, int arg2) = {
        tcg_gen_shri_i32, tcg_gen_sari_i32,
    };
    tcg_debug_assert(alith < 2);
    if (imm) {
        gen_sXri[alith](cpu_regs[rd], cpu_regs[rs], imm - 1);
        tcg_gen_andi_i32(cpu_psw_c, cpu_regs[rd], 0x00000001);
        gen_sXri[alith](cpu_regs[rd], cpu_regs[rd], 1);
    } else {
        tcg_gen_mov_i32(cpu_regs[rd], cpu_regs[rs]);
        tcg_gen_movi_i32(cpu_psw_c, 0);
    }
    tcg_gen_movi_i32(cpu_psw_o, 0);
    tcg_gen_mov_i32(cpu_psw_z, cpu_regs[rd]);
    tcg_gen_mov_i32(cpu_psw_s, cpu_regs[rd]);
}

static void shiftr_reg(uint32_t rd, uint32_t rs, unsigned int alith)
{
    TCGLabel *noshift, *done;
    TCGv_i32 count;
    static void (* const gen_sXri[])(TCGv_i32 ret, TCGv_i32 arg1, int arg2) = {
        tcg_gen_shri_i32, tcg_gen_sari_i32,
    };
    static void (* const gen_sXr[])(TCGv_i32 ret,
                                    TCGv_i32 arg1, TCGv_i32 arg2) = {
        tcg_gen_shr_i32, tcg_gen_sar_i32,
    };
    tcg_debug_assert(alith < 2);
    noshift = gen_new_label();
    done = gen_new_label();
    count = tcg_temp_new_i32();
    /* if (cpu_regs[rs]) { */
    tcg_gen_brcondi_i32(TCG_COND_EQ, cpu_regs[rs], 0, noshift);
    tcg_gen_andi_i32(count, cpu_regs[rs], 31);
    tcg_gen_subi_i32(count, count, 1);
    gen_sXr[alith](cpu_regs[rd], cpu_regs[rd], count);
    tcg_gen_andi_i32(cpu_psw_c, cpu_regs[rd], 0x00000001);
    gen_sXri[alith](cpu_regs[rd], cpu_regs[rd], 1);
    tcg_gen_br(done);
    /* } else { */
    gen_set_label(noshift);
    tcg_gen_movi_i32(cpu_psw_c, 0);
    /* } */
    gen_set_label(done);
    tcg_gen_movi_i32(cpu_psw_o, 0);
    tcg_gen_mov_i32(cpu_psw_z, cpu_regs[rd]);
    tcg_gen_mov_i32(cpu_psw_s, cpu_regs[rd]);
}

/* shar #imm:5, rd */
/* shar #imm:5, rs2, rd */
static bool trans_SHAR_irr(DisasContext *ctx, arg_SHAR_irr *a)
{
    shiftr_imm(a->rd, a->rs2, a->imm, 1);
    return true;
}

/* shar rs, rd */
static bool trans_SHAR_rr(DisasContext *ctx, arg_SHAR_rr *a)
{
    shiftr_reg(a->rd, a->rs, 1);
    return true;
}

/* shlr #imm:5, rd */
/* shlr #imm:5, rs2, rd */
static bool trans_SHLR_irr(DisasContext *ctx, arg_SHLR_irr *a)
{
    shiftr_imm(a->rd, a->rs2, a->imm, 0);
    return true;
}

/* shlr rs, rd */
static bool trans_SHLR_rr(DisasContext *ctx, arg_SHLR_rr *a)
{
    shiftr_reg(a->rd, a->rs, 0);
    return true;
}

/* rolc rd */
static bool trans_ROLC(DisasContext *ctx, arg_ROLC *a)
{
    TCGv_i32 tmp;
    tmp = tcg_temp_new_i32();
    tcg_gen_shri_i32(tmp, cpu_regs[a->rd], 31);
    tcg_gen_shli_i32(cpu_regs[a->rd], cpu_regs[a->rd], 1);
    tcg_gen_or_i32(cpu_regs[a->rd], cpu_regs[a->rd], cpu_psw_c);
    tcg_gen_mov_i32(cpu_psw_c, tmp);
    tcg_gen_mov_i32(cpu_psw_z, cpu_regs[a->rd]);
    tcg_gen_mov_i32(cpu_psw_s, cpu_regs[a->rd]);
    return true;
}

/* rorc rd */
static bool trans_RORC(DisasContext *ctx, arg_RORC *a)
{
    TCGv_i32 tmp;
    tmp = tcg_temp_new_i32();
    tcg_gen_andi_i32(tmp, cpu_regs[a->rd], 0x00000001);
    tcg_gen_shri_i32(cpu_regs[a->rd], cpu_regs[a->rd], 1);
    tcg_gen_shli_i32(cpu_psw_c, cpu_psw_c, 31);
    tcg_gen_or_i32(cpu_regs[a->rd], cpu_regs[a->rd], cpu_psw_c);
    tcg_gen_mov_i32(cpu_psw_c, tmp);
    tcg_gen_mov_i32(cpu_psw_z, cpu_regs[a->rd]);
    tcg_gen_mov_i32(cpu_psw_s, cpu_regs[a->rd]);
    return true;
}

enum {ROTR = 0, ROTL = 1};
enum {ROT_IMM = 0, ROT_REG = 1};
static void rx_rot(int ir, int dir, int rd, int src)
{
    switch (dir) {
    case ROTL:
        if (ir == ROT_IMM) {
            tcg_gen_rotli_i32(cpu_regs[rd], cpu_regs[rd], src);
        } else {
            tcg_gen_rotl_i32(cpu_regs[rd], cpu_regs[rd], cpu_regs[src]);
        }
        tcg_gen_andi_i32(cpu_psw_c, cpu_regs[rd], 0x00000001);
        break;
    case ROTR:
        if (ir == ROT_IMM) {
            tcg_gen_rotri_i32(cpu_regs[rd], cpu_regs[rd], src);
        } else {
            tcg_gen_rotr_i32(cpu_regs[rd], cpu_regs[rd], cpu_regs[src]);
        }
        tcg_gen_shri_i32(cpu_psw_c, cpu_regs[rd], 31);
        break;
    }
    tcg_gen_mov_i32(cpu_psw_z, cpu_regs[rd]);
    tcg_gen_mov_i32(cpu_psw_s, cpu_regs[rd]);
}

/* rotl #imm, rd */
static bool trans_ROTL_ir(DisasContext *ctx, arg_ROTL_ir *a)
{
    rx_rot(ROT_IMM, ROTL, a->rd, a->imm);
    return true;
}

/* rotl rs, rd */
static bool trans_ROTL_rr(DisasContext *ctx, arg_ROTL_rr *a)
{
    rx_rot(ROT_REG, ROTL, a->rd, a->rs);
    return true;
}

/* rotr #imm, rd */
static bool trans_ROTR_ir(DisasContext *ctx, arg_ROTR_ir *a)
{
    rx_rot(ROT_IMM, ROTR, a->rd, a->imm);
    return true;
}

/* rotr rs, rd */
static bool trans_ROTR_rr(DisasContext *ctx, arg_ROTR_rr *a)
{
    rx_rot(ROT_REG, ROTR, a->rd, a->rs);
    return true;
}

/* revl rs, rd */
static bool trans_REVL(DisasContext *ctx, arg_REVL *a)
{
    tcg_gen_bswap32_i32(cpu_regs[a->rd], cpu_regs[a->rs]);
    return true;
}

/* revw rs, rd */
static bool trans_REVW(DisasContext *ctx, arg_REVW *a)
{
    TCGv_i32 tmp;
    tmp = tcg_temp_new_i32();
    tcg_gen_andi_i32(tmp, cpu_regs[a->rs], 0x00ff00ff);
    tcg_gen_shli_i32(tmp, tmp, 8);
    tcg_gen_shri_i32(cpu_regs[a->rd], cpu_regs[a->rs], 8);
    tcg_gen_andi_i32(cpu_regs[a->rd], cpu_regs[a->rd], 0x00ff00ff);
    tcg_gen_or_i32(cpu_regs[a->rd], cpu_regs[a->rd], tmp);
    return true;
}

/* conditional branch helper */
static void rx_bcnd_main(DisasContext *ctx, int cd, int dst)
{
    DisasCompare dc;
    TCGLabel *t, *done;

    switch (cd) {
    case 0 ... 13:
        dc.temp = tcg_temp_new_i32();
        psw_cond(&dc, cd);
        t = gen_new_label();
        done = gen_new_label();
        tcg_gen_brcondi_i32(dc.cond, dc.value, 0, t);
        gen_goto_tb(ctx, 0, ctx->base.pc_next);
        tcg_gen_br(done);
        gen_set_label(t);
        gen_goto_tb(ctx, 1, ctx->pc + dst);
        gen_set_label(done);
        break;
    case 14:
        /* always true case */
        gen_goto_tb(ctx, 0, ctx->pc + dst);
        break;
    case 15:
        /* always false case */
        /* Nothing do */
        break;
    }
}

/* beq dsp:3 / bne dsp:3 */
/* beq dsp:8 / bne dsp:8 */
/* bc dsp:8 / bnc dsp:8 */
/* bgtu dsp:8 / bleu dsp:8 */
/* bpz dsp:8 / bn dsp:8 */
/* bge dsp:8 / blt dsp:8 */
/* bgt dsp:8 / ble dsp:8 */
/* bo dsp:8 / bno dsp:8 */
/* beq dsp:16 / bne dsp:16 */
static bool trans_BCnd(DisasContext *ctx, arg_BCnd *a)
{
    rx_bcnd_main(ctx, a->cd, a->dsp);
    return true;
}

/* bra dsp:3 */
/* bra dsp:8 */
/* bra dsp:16 */
/* bra dsp:24 */
static bool trans_BRA(DisasContext *ctx, arg_BRA *a)
{
    rx_bcnd_main(ctx, 14, a->dsp);
    return true;
}

/* bra rs */
static bool trans_BRA_l(DisasContext *ctx, arg_BRA_l *a)
{
    tcg_gen_addi_i32(cpu_pc, cpu_regs[a->rd], ctx->pc);
    ctx->base.is_jmp = DISAS_JUMP;
    return true;
}

static void rx_save_pc(DisasContext *ctx)
{
    TCGv_i32 pc = tcg_constant_i32(ctx->base.pc_next);
    push(ctx, pc);
}

/* jmp rs */
static bool trans_JMP(DisasContext *ctx, arg_JMP *a)
{
    tcg_gen_mov_i32(cpu_pc, cpu_regs[a->rs]);
    ctx->base.is_jmp = DISAS_JUMP;
    return true;
}

/* jsr rs */
static bool trans_JSR(DisasContext *ctx, arg_JSR *a)
{
    rx_save_pc(ctx);
    tcg_gen_mov_i32(cpu_pc, cpu_regs[a->rs]);
    ctx->base.is_jmp = DISAS_JUMP;
    return true;
}

/* bsr dsp:16 */
/* bsr dsp:24 */
static bool trans_BSR(DisasContext *ctx, arg_BSR *a)
{
    rx_save_pc(ctx);
    rx_bcnd_main(ctx, 14, a->dsp);
    return true;
}

/* bsr rs */
static bool trans_BSR_l(DisasContext *ctx, arg_BSR_l *a)
{
    rx_save_pc(ctx);
    tcg_gen_addi_i32(cpu_pc, cpu_regs[a->rd], ctx->pc);
    ctx->base.is_jmp = DISAS_JUMP;
    return true;
}

/* rts */
static bool trans_RTS(DisasContext *ctx, arg_RTS *a)
{
    pop(ctx, cpu_pc);
    ctx->base.is_jmp = DISAS_JUMP;
    return true;
}

/* nop */
static bool trans_NOP(DisasContext *ctx, arg_NOP *a)
{
    return true;
}

/* scmpu */
static bool trans_SCMPU(DisasContext *ctx, arg_SCMPU *a)
{
    gen_helper_scmpu(tcg_env);
    return true;
}

/* smovu */
static bool trans_SMOVU(DisasContext *ctx, arg_SMOVU *a)
{
    gen_helper_smovu(tcg_env);
    return true;
}

/* smovf */
static bool trans_SMOVF(DisasContext *ctx, arg_SMOVF *a)
{
    gen_helper_smovf(tcg_env);
    return true;
}

/* smovb */
static bool trans_SMOVB(DisasContext *ctx, arg_SMOVB *a)
{
    gen_helper_smovb(tcg_env);
    return true;
}

#define STRING(op)                              \
    do {                                        \
        TCGv_i32 size = tcg_constant_i32(a->sz);    \
        gen_helper_##op(tcg_env, size);         \
    } while (0)

/* suntile.<bwl> */
static bool trans_SUNTIL(DisasContext *ctx, arg_SUNTIL *a)
{
    STRING(suntil);
    return true;
}

/* swhile.<bwl> */
static bool trans_SWHILE(DisasContext *ctx, arg_SWHILE *a)
{
    STRING(swhile);
    return true;
}
/* sstr.<bwl> */
static bool trans_SSTR(DisasContext *ctx, arg_SSTR *a)
{
    STRING(sstr);
    return true;
}

/* rmpa.<bwl> */
static bool trans_RMPA(DisasContext *ctx, arg_RMPA *a)
{
    STRING(rmpa);
    return true;
}

/*
 * DSP instructions. RXv1 has the A0 forms of mulhi / mullo / machi /
 * maclo, mvtachi / mvtaclo, mvfachi / mvfacmi #0 and racw; RXv2 adds A1,
 * the shift of mvfac* and the other instructions.
 */
static bool dsp_allowed(DisasContext *ctx, bool v2_only, int acc)
{
    return has_feature(ctx, RX_FEATURE_V2) || (!v2_only && acc == 0);
}

enum {
    DSP_HI,     /* upper 16 bits of both */
    DSP_LO,     /* lower 16 bits of both */
    DSP_LH,     /* lower 16 bits of rs, upper 16 bits of rs2 */
    DSP_E,      /* all 32 bits */
};

/* The product that mul* / mac* / msb* / em*a apply to the accumulator */
static TCGv_i64 dsp_product(int sel, int rs, int rs2)
{
    TCGv_i32 t0 = tcg_temp_new_i32();
    TCGv_i32 t1 = tcg_temp_new_i32();
    TCGv_i64 ret = tcg_temp_new_i64();

    switch (sel) {
    case DSP_HI:
        tcg_gen_sari_i32(t0, cpu_regs[rs], 16);
        tcg_gen_sari_i32(t1, cpu_regs[rs2], 16);
        break;
    case DSP_LO:
        tcg_gen_ext16s_i32(t0, cpu_regs[rs]);
        tcg_gen_ext16s_i32(t1, cpu_regs[rs2]);
        break;
    case DSP_LH:
        tcg_gen_ext16s_i32(t0, cpu_regs[rs]);
        tcg_gen_sari_i32(t1, cpu_regs[rs2], 16);
        break;
    default:
        /* 32 x 32 bits, not shifted */
        tcg_gen_muls2_i32(t0, t1, cpu_regs[rs], cpu_regs[rs2]);
        tcg_gen_concat_i32_i64(ret, t0, t1);
        return ret;
    }
    /* 16 x 16 bits, placed at bit 16 */
    tcg_gen_mul_i32(t0, t0, t1);
    tcg_gen_ext_i32_i64(ret, t0);
    tcg_gen_shli_i64(ret, ret, 16);
    return ret;
}

#define DSP_MUL(name, sel, op, v2_only)                                 \
    static bool trans_##name(DisasContext *ctx, arg_##name * a)         \
    {                                                                   \
        if (!dsp_allowed(ctx, v2_only, a->a)) {                         \
            return false;                                               \
        }                                                               \
        gen_helper_acc_##op(tcg_env, tcg_constant_i32(a->a),            \
                            dsp_product(sel, a->rs, a->rs2));           \
        return true;                                                    \
    }

DSP_MUL(MULHI, DSP_HI, set, false)
DSP_MUL(MULLO, DSP_LO, set, false)
DSP_MUL(MULLH, DSP_LH, set, true)
DSP_MUL(EMULA, DSP_E, set, true)
DSP_MUL(MACHI, DSP_HI, add, false)
DSP_MUL(MACLO, DSP_LO, add, false)
DSP_MUL(MACLH, DSP_LH, add, true)
DSP_MUL(EMACA, DSP_E, add, true)
DSP_MUL(MSBHI, DSP_HI, sub, true)
DSP_MUL(MSBLO, DSP_LO, sub, true)
DSP_MUL(MSBLH, DSP_LH, sub, true)
DSP_MUL(EMSBA, DSP_E, sub, true)

/* mvfac* #imm, a, rd: bits pos+31..pos of a << imm (imm is 0 to 2) */
#define DSP_MVFAC(name, pos, v2_only)                                   \
    static bool trans_##name(DisasContext *ctx, arg_##name * a)         \
    {                                                                   \
        if (!dsp_allowed(ctx, v2_only || a->imm != 0, a->a)) {         \
            return false;                                               \
        }                                                               \
        gen_helper_mvfac(cpu_regs[a->rd], tcg_env,                      \
                         tcg_constant_i32(a->a), tcg_constant_i32(a->imm), \
                         tcg_constant_i32(pos));                        \
        return true;                                                    \
    }

DSP_MVFAC(MVFACHI, 32, false)
DSP_MVFAC(MVFACMI, 16, false)
DSP_MVFAC(MVFACLO, 0, true)
DSP_MVFAC(MVFACGU, 64, true)

/* mvtac* rs, a: bits pos+31..pos of a */
#define DSP_MVTAC(name, pos, v2_only)                                   \
    static bool trans_##name(DisasContext *ctx, arg_##name * a)         \
    {                                                                   \
        if (!dsp_allowed(ctx, v2_only, a->a)) {                         \
            return false;                                               \
        }                                                               \
        gen_helper_mvtac(tcg_env, tcg_constant_i32(a->a),               \
                         tcg_constant_i32(pos), cpu_regs[a->rs]);       \
        return true;                                                    \
    }

DSP_MVTAC(MVTACHI, 32, false)
DSP_MVTAC(MVTACLO, 0, false)
DSP_MVTAC(MVTACGU, 64, true)

/* racw / racl / rdacw / rdacl #imm, a (imm is 1 or 2) */
#define DSP_RAC(name, mode, v2_only)                                    \
    static bool trans_##name(DisasContext *ctx, arg_##name * a)         \
    {                                                                   \
        if (!dsp_allowed(ctx, v2_only, a->a)) {                         \
            return false;                                               \
        }                                                               \
        gen_helper_rac(tcg_env, tcg_constant_i32(a->a),                 \
                       tcg_constant_i32(a->imm + 1), tcg_constant_i32(mode)); \
        return true;                                                    \
    }

DSP_RAC(RACW, RX_RAC_ROUND, false)
DSP_RAC(RACL, RX_RAC_ROUND | RX_RAC_LONG, true)
DSP_RAC(RDACW, 0, true)
DSP_RAC(RDACL, RX_RAC_LONG, true)

/* sat rd */
static bool trans_SAT(DisasContext *ctx, arg_SAT *a)
{
    TCGv_i32 tmp, z;
    tmp = tcg_temp_new_i32();
    z = tcg_constant_i32(0);
    /* S == 1 -> 0xffffffff / S == 0 -> 0x00000000 */
    tcg_gen_sari_i32(tmp, cpu_psw_s, 31);
    /* S == 1 -> 0x7fffffff / S == 0 -> 0x80000000 */
    tcg_gen_xori_i32(tmp, tmp, 0x80000000);
    tcg_gen_movcond_i32(TCG_COND_LT, cpu_regs[a->rd],
                        cpu_psw_o, z, tmp, cpu_regs[a->rd]);
    return true;
}

/* satr */
static bool trans_SATR(DisasContext *ctx, arg_SATR *a)
{
    gen_helper_satr(tcg_env);
    return true;
}

#define cat3(a, b, c) a##b##c
#define FOP(name, op)                                                   \
    static bool cat3(trans_, name, _ir)(DisasContext *ctx,              \
                                        cat3(arg_, name, _ir) * a)      \
    {                                                                   \
        TCGv_i32 imm;                                                   \
        if (!has_feature(ctx, RX_FEATURE_FPU)) {                        \
            return false;                                               \
        }                                                               \
        imm = tcg_constant_i32(li(ctx, 0));                             \
        gen_helper_##op(cpu_regs[a->rd], tcg_env,                       \
                        cpu_regs[a->rd], imm);                          \
        return true;                                                    \
    }                                                                   \
    static bool cat3(trans_, name, _mr)(DisasContext *ctx,              \
                                        cat3(arg_, name, _mr) * a)      \
    {                                                                   \
        TCGv_i32 val, mem;                                              \
        if (!has_feature(ctx, RX_FEATURE_FPU)) {                        \
            return false;                                               \
        }                                                               \
        mem = tcg_temp_new_i32();                                       \
        val = rx_load_source(ctx, mem, a->ld, MO_32, a->rs);            \
        gen_helper_##op(cpu_regs[a->rd], tcg_env,                       \
                        cpu_regs[a->rd], val);                          \
        return true;                                                    \
    }

#define FCONVOP(name, op, feat)                                 \
    static bool trans_##name(DisasContext *ctx, arg_##name * a) \
    {                                                           \
        TCGv_i32 val, mem;                                      \
        if (!has_feature(ctx, feat)) {                          \
            return false;                                       \
        }                                                       \
        mem = tcg_temp_new_i32();                               \
        val = rx_load_source(ctx, mem, a->ld, MO_32, a->rs);    \
        gen_helper_##op(cpu_regs[a->rd], tcg_env, val);         \
        return true;                                            \
    }

FOP(FADD, fadd)
FOP(FSUB, fsub)
FOP(FMUL, fmul)
FOP(FDIV, fdiv)

/* fadd/fsub/fmul rs, rs2, rd: rd = rs2 op rs (RXv2) */
#define FOP3(name, op)                                                  \
    static bool cat3(trans_, name, _rrr)(DisasContext *ctx,             \
                                         cat3(arg_, name, _rrr) * a)    \
    {                                                                   \
        if (!has_feature(ctx, RX_FEATURE_V2 | RX_FEATURE_FPU)) {        \
            return false;                                               \
        }                                                               \
        gen_helper_##op(cpu_regs[a->rd], tcg_env,                       \
                        cpu_regs[a->rs2], cpu_regs[a->rs]);             \
        return true;                                                    \
    }

FOP3(FADD, fadd)
FOP3(FSUB, fsub)
FOP3(FMUL, fmul)

/* fcmp #imm, rd */
static bool trans_FCMP_ir(DisasContext *ctx, arg_FCMP_ir * a)
{
    TCGv_i32 imm;
    if (!has_feature(ctx, RX_FEATURE_FPU)) {
        return false;
    }
    imm = tcg_constant_i32(li(ctx, 0));
    gen_helper_fcmp(tcg_env, cpu_regs[a->rd], imm);
    return true;
}

/* fcmp dsp[rs], rd */
/* fcmp rs, rd */
static bool trans_FCMP_mr(DisasContext *ctx, arg_FCMP_mr *a)
{
    TCGv_i32 val, mem;
    if (!has_feature(ctx, RX_FEATURE_FPU)) {
        return false;
    }
    mem = tcg_temp_new_i32();
    val = rx_load_source(ctx, mem, a->ld, MO_32, a->rs);
    gen_helper_fcmp(tcg_env, cpu_regs[a->rd], val);
    return true;
}

FCONVOP(FTOI, ftoi, RX_FEATURE_FPU)
FCONVOP(ROUND, round, RX_FEATURE_FPU)
FCONVOP(FSQRT, fsqrt, RX_FEATURE_V2 | RX_FEATURE_FPU)
FCONVOP(FTOU, ftou, RX_FEATURE_V2 | RX_FEATURE_FPU)

/* itof rs, rd */
/* itof dsp[rs], rd */
static bool trans_ITOF(DisasContext *ctx, arg_ITOF * a)
{
    TCGv_i32 val, mem;
    if (!has_feature(ctx, RX_FEATURE_FPU)) {
        return false;
    }
    mem = tcg_temp_new_i32();
    val = rx_load_source(ctx, mem, a->ld, a->mi, a->rs);
    gen_helper_itof(cpu_regs[a->rd], tcg_env, val);
    return true;
}

/* utof rs, rd */
/* utof dsp[rs], rd */
static bool trans_UTOF(DisasContext *ctx, arg_UTOF * a)
{
    TCGv_i32 val, mem;
    if (!has_feature(ctx, RX_FEATURE_V2 | RX_FEATURE_FPU)) {
        return false;
    }
    mem = tcg_temp_new_i32();
    val = rx_load_source(ctx, mem, a->ld, a->mi, a->rs);
    gen_helper_utof(cpu_regs[a->rd], tcg_env, val);
    return true;
}

static void rx_bsetm(DisasContext *ctx, TCGv_i32 mem, TCGv_i32 mask)
{
    TCGv_i32 val;
    val = tcg_temp_new_i32();
    rx_gen_ld(ctx, MO_8, val, mem);
    tcg_gen_or_i32(val, val, mask);
    rx_gen_st(ctx, MO_8, val, mem);
}

static void rx_bclrm(DisasContext *ctx, TCGv_i32 mem, TCGv_i32 mask)
{
    TCGv_i32 val;
    val = tcg_temp_new_i32();
    rx_gen_ld(ctx, MO_8, val, mem);
    tcg_gen_andc_i32(val, val, mask);
    rx_gen_st(ctx, MO_8, val, mem);
}

static void rx_btstm(DisasContext *ctx, TCGv_i32 mem, TCGv_i32 mask)
{
    TCGv_i32 val;
    val = tcg_temp_new_i32();
    rx_gen_ld(ctx, MO_8, val, mem);
    tcg_gen_and_i32(val, val, mask);
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_psw_c, val, 0);
    tcg_gen_mov_i32(cpu_psw_z, cpu_psw_c);
}

static void rx_bnotm(DisasContext *ctx, TCGv_i32 mem, TCGv_i32 mask)
{
    TCGv_i32 val;
    val = tcg_temp_new_i32();
    rx_gen_ld(ctx, MO_8, val, mem);
    tcg_gen_xor_i32(val, val, mask);
    rx_gen_st(ctx, MO_8, val, mem);
}

static void rx_bsetr(DisasContext *ctx, TCGv_i32 reg, TCGv_i32 mask)
{
    tcg_gen_or_i32(reg, reg, mask);
}

static void rx_bclrr(DisasContext *ctx, TCGv_i32 reg, TCGv_i32 mask)
{
    tcg_gen_andc_i32(reg, reg, mask);
}

static void rx_btstr(DisasContext *ctx, TCGv_i32 reg, TCGv_i32 mask)
{
    TCGv_i32 t0;
    t0 = tcg_temp_new_i32();
    tcg_gen_and_i32(t0, reg, mask);
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_psw_c, t0, 0);
    tcg_gen_mov_i32(cpu_psw_z, cpu_psw_c);
}

static void rx_bnotr(DisasContext *ctx, TCGv_i32 reg, TCGv_i32 mask)
{
    tcg_gen_xor_i32(reg, reg, mask);
}

#define BITOP(name, op)                                                 \
    static bool cat3(trans_, name, _im)(DisasContext *ctx,              \
                                        cat3(arg_, name, _im) * a)      \
    {                                                                   \
        TCGv_i32 mask, mem, addr;                                       \
        mem = tcg_temp_new_i32();                                       \
        mask = tcg_constant_i32(1 << a->imm);                           \
        addr = rx_index_addr(ctx, mem, a->ld, MO_8, a->rs);             \
        cat3(rx_, op, m)(ctx, addr, mask);                              \
        return true;                                                    \
    }                                                                   \
    static bool cat3(trans_, name, _ir)(DisasContext *ctx,              \
                                        cat3(arg_, name, _ir) * a)      \
    {                                                                   \
        TCGv_i32 mask;                                                  \
        mask = tcg_constant_i32(1 << a->imm);                           \
        cat3(rx_, op, r)(ctx, cpu_regs[a->rd], mask);                   \
        return true;                                                    \
    }                                                                   \
    static bool cat3(trans_, name, _rr)(DisasContext *ctx,              \
                                        cat3(arg_, name, _rr) * a)      \
    {                                                                   \
        TCGv_i32 mask, b;                                               \
        mask = tcg_temp_new_i32();                                      \
        b = tcg_temp_new_i32();                                         \
        tcg_gen_andi_i32(b, cpu_regs[a->rs], 31);                       \
        tcg_gen_shl_i32(mask, tcg_constant_i32(1), b);                  \
        cat3(rx_, op, r)(ctx, cpu_regs[a->rd], mask);                   \
        return true;                                                    \
    }                                                                   \
    static bool cat3(trans_, name, _rm)(DisasContext *ctx,              \
                                        cat3(arg_, name, _rm) * a)      \
    {                                                                   \
        TCGv_i32 mask, mem, addr, b;                                    \
        mask = tcg_temp_new_i32();                                      \
        b = tcg_temp_new_i32();                                         \
        tcg_gen_andi_i32(b, cpu_regs[a->rd], 7);                        \
        tcg_gen_shl_i32(mask, tcg_constant_i32(1), b);                  \
        mem = tcg_temp_new_i32();                                       \
        addr = rx_index_addr(ctx, mem, a->ld, MO_8, a->rs);             \
        cat3(rx_, op, m)(ctx, addr, mask);                              \
        return true;                                                    \
    }

BITOP(BSET, bset)
BITOP(BCLR, bclr)
BITOP(BTST, btst)
BITOP(BNOT, bnot)

static void bmcnd_op(TCGv_i32 val, TCGCond cond, int pos)
{
    TCGv_i32 bit;
    DisasCompare dc;
    dc.temp = tcg_temp_new_i32();
    bit = tcg_temp_new_i32();
    psw_cond(&dc, cond);
    tcg_gen_andi_i32(val, val, ~(1 << pos));
    tcg_gen_setcondi_i32(dc.cond, bit, dc.value, 0);
    tcg_gen_deposit_i32(val, val, bit, pos, 1);
 }

/* bmcnd #imm, dsp[rd] */
static bool trans_BMCnd_im(DisasContext *ctx, arg_BMCnd_im *a)
{
    TCGv_i32 val, mem, addr;
    val = tcg_temp_new_i32();
    mem = tcg_temp_new_i32();
    addr = rx_index_addr(ctx, mem, a->ld, MO_8, a->rd);
    rx_gen_ld(ctx, MO_8, val, addr);
    bmcnd_op(val, a->cd, a->imm);
    rx_gen_st(ctx, MO_8, val, addr);
    return true;
}

/* bmcond #imm, rd */
static bool trans_BMCnd_ir(DisasContext *ctx, arg_BMCnd_ir *a)
{
    bmcnd_op(cpu_regs[a->rd], a->cd, a->imm);
    return true;
}

enum {
    PSW_C = 0,
    PSW_Z = 1,
    PSW_S = 2,
    PSW_O = 3,
    PSW_I = 8,
    PSW_U = 9,
};

static void clrsetpsw(DisasContext *ctx, int cb, int val)
{
    if (cb < 8) {
        switch (cb) {
        case PSW_C:
            tcg_gen_movi_i32(cpu_psw_c, val);
            break;
        case PSW_Z:
            tcg_gen_movi_i32(cpu_psw_z, val == 0);
            break;
        case PSW_S:
            tcg_gen_movi_i32(cpu_psw_s, val ? -1 : 0);
            break;
        case PSW_O:
            tcg_gen_movi_i32(cpu_psw_o, val << 31);
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "Invalid destination %d", cb);
            break;
        }
    } else if (is_privileged(ctx, 0)) {
        switch (cb) {
        case PSW_I:
            tcg_gen_movi_i32(cpu_psw_i, val);
            ctx->base.is_jmp = DISAS_UPDATE;
            break;
        case PSW_U:
            if (FIELD_EX32(ctx->tb_flags, PSW, U) != val) {
                ctx->tb_flags = FIELD_DP32(ctx->tb_flags, PSW, U, val);
                tcg_gen_movi_i32(cpu_psw_u, val);
                tcg_gen_mov_i32(val ? cpu_isp : cpu_usp, cpu_sp);
                tcg_gen_mov_i32(cpu_sp, val ? cpu_usp : cpu_isp);
            }
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "Invalid destination %d", cb);
            break;
        }
    }
}

/* clrpsw psw */
static bool trans_CLRPSW(DisasContext *ctx, arg_CLRPSW *a)
{
    clrsetpsw(ctx, a->cb, 0);
    return true;
}

/* setpsw psw */
static bool trans_SETPSW(DisasContext *ctx, arg_SETPSW *a)
{
    clrsetpsw(ctx, a->cb, 1);
    return true;
}

/* mvtipl #imm */
static bool trans_MVTIPL(DisasContext *ctx, arg_MVTIPL *a)
{
    int imm = a->imm;

    /*
     * The bits above the IPL field are ignored (measured on the RX72M).
     * IPL[4] is RXv3e only, and always 0 in the non-secure world.
     */
    imm &= has_feature(ctx, RX_FEATURE_V3E) ? 0x1f : 0xf;
    if (ctx_world(ctx) == RX_WORLD_NONSECURE) {
        imm &= 0xf;
    }
    if (is_privileged(ctx, 1)) {
        tcg_gen_movi_i32(cpu_psw_ipl, imm);
        ctx->base.is_jmp = DISAS_UPDATE;
    }
    return true;
}

/* mvtc #imm, rd */
static bool trans_MVTC_i(DisasContext *ctx, arg_MVTC_i *a)
{
    TCGv_i32 imm;

    imm = tcg_constant_i32(a->imm);
    move_to_cr(ctx, imm, a->cr);
    return true;
}

/* mvtc rs, rd */
static bool trans_MVTC_r(DisasContext *ctx, arg_MVTC_r *a)
{
    move_to_cr(ctx, cpu_regs[a->rs], a->cr);
    return true;
}

/* mvfc rs, rd */
static bool trans_MVFC(DisasContext *ctx, arg_MVFC *a)
{
    move_from_cr(ctx, cpu_regs[a->rd], a->cr, ctx->pc);
    return true;
}

/* rtfi */
static bool trans_RTFI(DisasContext *ctx, arg_RTFI *a)
{
    TCGv_i32 psw;
    if (is_privileged(ctx, 1)) {
        psw = tcg_temp_new_i32();
        tcg_gen_mov_i32(cpu_pc, cpu_bpc);
        tcg_gen_mov_i32(psw, cpu_bpsw);
        gen_helper_set_psw_rte(tcg_env, psw);
        tcg_gen_movi_i32(cpu_li, 0);
        ctx->base.is_jmp = DISAS_EXIT;
    }
    return true;
}

/* rte */
static bool trans_RTE(DisasContext *ctx, arg_RTE *a)
{
    TCGv_i32 psw;
    if (is_privileged(ctx, 1)) {
        psw = tcg_temp_new_i32();
        pop(ctx, cpu_pc);
        pop(ctx, psw);
        gen_helper_set_psw_rte(tcg_env, psw);
        tcg_gen_movi_i32(cpu_li, 0);
        ctx->base.is_jmp = DISAS_EXIT;
    }
    return true;
}

/* brk */
static bool trans_BRK(DisasContext *ctx, arg_BRK *a)
{
    tcg_gen_movi_i32(cpu_pc, ctx->base.pc_next);
    gen_helper_rxbrk(tcg_env);
    ctx->base.is_jmp = DISAS_NORETURN;
    return true;
}

/* int #imm */
/* RXv3e: IVC.UIR restricts int in user mode */
static void gen_int_check(DisasContext *ctx, TCGv_i32 vec, bool is_reg)
{
    if (has_feature(ctx, RX_FEATURE_V3E) &&
        FIELD_EX32(ctx->tb_flags, PSW, PM)) {
        gen_helper_int_check(tcg_env, vec, tcg_constant_i32(is_reg));
    }
}

static bool trans_INT(DisasContext *ctx, arg_INT *a)
{
    TCGv_i32 vec;

    tcg_debug_assert(a->imm < 0x100);
    vec = tcg_constant_i32(a->imm);
    gen_int_check(ctx, vec, false);
    tcg_gen_movi_i32(cpu_pc, ctx->base.pc_next);
    gen_helper_rxint(tcg_env, vec);
    ctx->base.is_jmp = DISAS_NORETURN;
    return true;
}

/* int rs (RXv3e): the vector number is the lower 8 bits of rs */
static bool trans_INT_r(DisasContext *ctx, arg_INT_r *a)
{
    TCGv_i32 vec;

    if (!has_feature(ctx, RX_FEATURE_V3E)) {
        return false;
    }
    vec = tcg_temp_new_i32();
    tcg_gen_andi_i32(vec, cpu_regs[a->rs], 0xff);
    gen_int_check(ctx, vec, true);
    tcg_gen_movi_i32(cpu_pc, ctx->base.pc_next);
    gen_helper_rxint(tcg_env, vec);
    ctx->base.is_jmp = DISAS_NORETURN;
    return true;
}

/* trcall #imm (RXv3e) */
static bool trans_TRCALL(DisasContext *ctx, arg_TRCALL *a)
{
    if (!has_feature(ctx, RX_FEATURE_V3E)) {
        return false;
    }
    if (is_privileged(ctx, 1)) {
        gen_helper_trcall(tcg_env, tcg_constant_i32(a->imm),
                          tcg_constant_i32(ctx->base.pc_next));
        ctx->base.is_jmp = DISAS_EXIT;
    }
    return true;
}

/* trrte, trmvfc and trmvtc are for the root world only (RXv3e) */
static bool root_only(DisasContext *ctx)
{
    return has_feature(ctx, RX_FEATURE_V3E) &&
           ctx_world(ctx) == RX_WORLD_ROOT;
}

/* trrte */
static bool trans_TRRTE(DisasContext *ctx, arg_TRRTE *a)
{
    if (!root_only(ctx)) {
        return false;
    }
    if (is_privileged(ctx, 1)) {
        gen_helper_trrte(tcg_env);
        ctx->base.is_jmp = DISAS_EXIT;
    }
    return true;
}

/* cr of trmvfc / trmvtc: IVC, BPSW, BPC, ISP, INTB, EXTB, TRCTB, WINTB */
static bool trmv_cr(int cr)
{
    return cr == 5 || (cr >= 8 && cr != 11);
}

/* trmvfc cr, rd */
static bool trans_TRMVFC(DisasContext *ctx, arg_TRMVFC *a)
{
    if (!root_only(ctx) || !trmv_cr(a->cr)) {
        return false;
    }
    if (is_privileged(ctx, 1)) {
        gen_helper_trmvfc(cpu_regs[a->rd], tcg_env, tcg_constant_i32(a->cr));
    }
    return true;
}

static bool gen_trmvtc(DisasContext *ctx, int cr, TCGv_i32 val)
{
    if (!root_only(ctx) || !trmv_cr(cr)) {
        return false;
    }
    if (is_privileged(ctx, 1)) {
        gen_helper_trmvtc(tcg_env, tcg_constant_i32(cr), val);
    }
    return true;
}

/* trmvtc rs, cr */
static bool trans_TRMVTC_r(DisasContext *ctx, arg_TRMVTC_r *a)
{
    return gen_trmvtc(ctx, a->cr, cpu_regs[a->rs]);
}

/* trmvtc #imm, cr */
static bool trans_TRMVTC_i(DisasContext *ctx, arg_TRMVTC_i *a)
{
    return gen_trmvtc(ctx, a->cr, tcg_constant_i32(a->imm));
}

/* wait */
static bool trans_WAIT(DisasContext *ctx, arg_WAIT *a)
{
    if (is_privileged(ctx, 1)) {
        tcg_gen_movi_i32(cpu_pc, ctx->base.pc_next);
        gen_helper_wait(tcg_env);
    }
    return true;
}

static void rx_tr_init_disas_context(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);
    ctx->env = cpu_env(cs);
    ctx->tb_flags = ctx->base.tb->flags;
    ctx->features = ctx->env->features;
}

static void rx_tr_tb_start(DisasContextBase *dcbase, CPUState *cs)
{
}

static void rx_tr_insn_start(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);

    tcg_gen_insn_start(ctx->base.pc_next, 0, 0);
}

static void rx_tr_translate_insn(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);
    uint32_t insn;

    ctx->pc = ctx->base.pc_next;
    insn = decode_load(ctx);
    if (!decode(ctx, insn)) {
        gen_helper_raise_illegal_instruction(tcg_env);
    }
}

static void rx_tr_tb_stop(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);

    switch (ctx->base.is_jmp) {
    case DISAS_NEXT:
    case DISAS_TOO_MANY:
        gen_goto_tb(ctx, 0, dcbase->pc_next);
        break;
    case DISAS_JUMP:
        tcg_gen_lookup_and_goto_ptr();
        break;
    case DISAS_UPDATE:
        tcg_gen_movi_i32(cpu_pc, ctx->base.pc_next);
        /* fall through */
    case DISAS_EXIT:
        tcg_gen_exit_tb(NULL, 0);
        break;
    case DISAS_NORETURN:
        break;
    default:
        g_assert_not_reached();
    }
}

static const TranslatorOps rx_tr_ops = {
    .init_disas_context = rx_tr_init_disas_context,
    .tb_start           = rx_tr_tb_start,
    .insn_start         = rx_tr_insn_start,
    .translate_insn     = rx_tr_translate_insn,
    .tb_stop            = rx_tr_tb_stop,
};

void rx_translate_code(CPUState *cs, TranslationBlock *tb,
                       int *max_insns, vaddr pc, void *host_pc)
{
    DisasContext dc;

    translator_loop(cs, tb, max_insns, pc, host_pc, &rx_tr_ops, &dc.base,
                    TCG_TYPE_VA);
}

#define ALLOC_REGISTER(sym, name) \
    cpu_##sym = tcg_global_mem_new_i32(tcg_env, \
                                       offsetof(CPURXState, sym), name)

void rx_translate_init(void)
{
    static const char * const regnames[NUM_REGS] = {
        "R0", "R1", "R2", "R3", "R4", "R5", "R6", "R7",
        "R8", "R9", "R10", "R11", "R12", "R13", "R14", "R15"
    };
    int i;

    for (i = 0; i < NUM_REGS; i++) {
        cpu_regs[i] = tcg_global_mem_new_i32(tcg_env,
                                              offsetof(CPURXState, regs[i]),
                                              regnames[i]);
    }
    ALLOC_REGISTER(pc, "PC");
    ALLOC_REGISTER(psw_o, "PSW(O)");
    ALLOC_REGISTER(psw_s, "PSW(S)");
    ALLOC_REGISTER(psw_z, "PSW(Z)");
    ALLOC_REGISTER(psw_c, "PSW(C)");
    ALLOC_REGISTER(psw_u, "PSW(U)");
    ALLOC_REGISTER(psw_i, "PSW(I)");
    ALLOC_REGISTER(psw_pm, "PSW(PM)");
    ALLOC_REGISTER(psw_ipl, "PSW(IPL)");
    ALLOC_REGISTER(usp, "USP");
    ALLOC_REGISTER(fpsw, "FPSW");
    ALLOC_REGISTER(extb, "EXTB");
    ALLOC_REGISTER(li, "LI");
    ALLOC_REGISTER(bpsw, "BPSW");
    ALLOC_REGISTER(bpc, "BPC");
    ALLOC_REGISTER(isp, "ISP");
    ALLOC_REGISTER(fintv, "FINTV");
    ALLOC_REGISTER(intb, "INTB");
}
