/*
 *  RX helper functions
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
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-ldst.h"
#include "accel/tcg/cpu-loop.h"
#include "fpu/softfloat.h"
#include "tcg/debug-assert.h"

static inline G_NORETURN
void raise_exception(CPURXState *env, int index,
                     uintptr_t retaddr);

static void _set_psw(CPURXState *env, uint32_t psw, uint32_t rte)
{
    uint32_t prev_u;
    prev_u = env->psw_u;
    rx_cpu_unpack_psw(env, psw, rte);
    if (prev_u != env->psw_u) {
        /* switch r0  */
        if (env->psw_u) {
            env->isp = env->regs[0];
            env->regs[0] = env->usp;
        } else {
            env->usp = env->regs[0];
            env->regs[0] = env->isp;
        }
    }
}

void helper_set_psw(CPURXState *env, uint32_t psw)
{
    _set_psw(env, psw, 0);
}

void helper_set_psw_rte(CPURXState *env, uint32_t psw)
{
    _set_psw(env, psw, 1);
}

uint32_t helper_pack_psw(CPURXState *env)
{
    return rx_cpu_pack_psw(env);
}

#define SET_FPSW(b)                                             \
    do {                                                        \
        env->fpsw = FIELD_DP32(env->fpsw, FPSW, C ## b, 1);     \
        if (!FIELD_EX32(env->fpsw, FPSW, E ## b)) {             \
            env->fpsw = FIELD_DP32(env->fpsw, FPSW, F ## b, 1); \
        }                                                       \
    } while (0)

/* fp operations */
/*
 * Update FPSW, then Z and S from the result: a floating-point result
 * sets Z for +0 and -0, an integer one (ftoi, round, ftou) only for 0.
 */
static void update_fpsw_flags(CPURXState *env, uint32_t ret, bool integer,
                              uintptr_t retaddr)
{
    int xcpt, cause, enable;

    xcpt = get_float_exception_flags(&env->fp_status);
    set_float_exception_flags(0, &env->fp_status);

    /* Clear the cause entries */
    env->fpsw = FIELD_DP32(env->fpsw, FPSW, CAUSE, 0);

    /* set FPSW */
    if (unlikely(xcpt)) {
        /*
         * Denormals are always flushed. With DN = 1 that is the result: a
         * flushed result is an underflow (without inexact). With DN = 0 a
         * denormal operand or an underflow is unimplemented processing,
         * which cannot be masked and leaves the destination unchanged.
         */
        if (xcpt & (float_flag_input_denormal_flushed
                    | float_flag_output_denormal_flushed)
            && !FIELD_EX32(env->fpsw, FPSW, DN)) {
            env->fpsw = FIELD_DP32(env->fpsw, FPSW, CE, 1);
            raise_exception(env, 25, retaddr);
        }
        if (xcpt & float_flag_output_denormal_flushed) {
            xcpt |= float_flag_underflow;
        }
        if (xcpt & float_flag_underflow) {
            xcpt &= ~float_flag_inexact;
        }
        if (xcpt & float_flag_invalid) {
            SET_FPSW(V);
        }
        if (xcpt & float_flag_divbyzero) {
            SET_FPSW(Z);
        }
        if (xcpt & float_flag_overflow) {
            SET_FPSW(O);
        }
        if (xcpt & float_flag_underflow) {
            SET_FPSW(U);
        }
        if (xcpt & float_flag_inexact) {
            SET_FPSW(X);
        }

        /* update FPSW_FLAG_S */
        if (FIELD_EX32(env->fpsw, FPSW, FLAGS) != 0) {
            env->fpsw = FIELD_DP32(env->fpsw, FPSW, FS, 1);
        }

        /*
         * Generate an exception if enabled; the destination and the
         * Z and S flags are left unchanged.
         */
        cause = FIELD_EX32(env->fpsw, FPSW, CAUSE);
        enable = FIELD_EX32(env->fpsw, FPSW, ENABLE);
        if (cause & enable) {
            raise_exception(env, 25, retaddr);
        }
    }

    env->psw_z = integer ? ret : ret & ~(1 << 31);
    env->psw_s = ret;
}

static void update_fpsw(CPURXState *env, float32 ret, uintptr_t retaddr)
{
    update_fpsw_flags(env, ret, false, retaddr);
}

static void update_fpsw_int(CPURXState *env, uint32_t ret, uintptr_t retaddr)
{
    update_fpsw_flags(env, ret, true, retaddr);
}

void helper_set_fpsw(CPURXState *env, uint32_t val)
{
    static const int roundmode[] = {
        float_round_nearest_even,
        float_round_to_zero,
        float_round_up,
        float_round_down,
    };
    uint32_t fpsw = env->fpsw;
    fpsw |= 0x7fffff03;
    val &= ~0x80000000;
    fpsw &= val;
    fpsw = FIELD_DP32(fpsw, FPSW, FS, FIELD_EX32(fpsw, FPSW, FLAGS) != 0);
    env->fpsw = fpsw;
    set_float_rounding_mode(roundmode[FIELD_EX32(env->fpsw, FPSW, RM)],
                            &env->fp_status);
}

#define FLOATOP(op, func)                                           \
    float32 helper_##op(CPURXState *env, float32 t0, float32 t1)    \
    {                                                               \
        float32 ret;                                                \
        ret = func(t0, t1, &env->fp_status);                        \
        update_fpsw(env, *(uint32_t *)&ret, GETPC());               \
        return ret;                                                 \
    }

FLOATOP(fadd, float32_add)
FLOATOP(fsub, float32_sub)
FLOATOP(fmul, float32_mul)
FLOATOP(fdiv, float32_div)

void helper_fcmp(CPURXState *env, float32 t0, float32 t1)
{
    int st;
    st = float32_compare(t0, t1, &env->fp_status);
    update_fpsw(env, 0, GETPC());
    env->psw_z = 1;
    env->psw_s = env->psw_o = 0;
    switch (st) {
    case float_relation_equal:
        env->psw_z = 0;
        break;
    case float_relation_less:
        env->psw_s = -1;
        break;
    case float_relation_unordered:
        env->psw_o = -1;
        break;
    }
}

/*
 * Integer conversion of a NaN is an invalid operation whose result
 * follows the sign of the NaN (RXv3 manual table 1.6).
 */
static uint32_t nan_to_int(float32 t0, uint32_t pos, uint32_t neg)
{
    return float32_is_neg(t0) ? neg : pos;
}

uint32_t helper_ftoi(CPURXState *env, float32 t0)
{
    uint32_t ret;
    ret = float32_to_int32_round_to_zero(t0, &env->fp_status);
    if (float32_is_any_nan(t0)) {
        ret = nan_to_int(t0, INT32_MAX, INT32_MIN);
    }
    update_fpsw_int(env, ret, GETPC());
    return ret;
}

uint32_t helper_round(CPURXState *env, float32 t0)
{
    uint32_t ret;
    ret = float32_to_int32(t0, &env->fp_status);
    if (float32_is_any_nan(t0)) {
        ret = nan_to_int(t0, INT32_MAX, INT32_MIN);
    }
    update_fpsw_int(env, ret, GETPC());
    return ret;
}

uint32_t helper_ftou(CPURXState *env, float32 t0)
{
    uint32_t ret;
    ret = float32_to_uint32_round_to_zero(t0, &env->fp_status);
    if (float32_is_any_nan(t0)) {
        ret = nan_to_int(t0, UINT32_MAX, 0);
    }
    update_fpsw_int(env, ret, GETPC());
    return ret;
}

float32 helper_fsqrt(CPURXState *env, float32 t0)
{
    float32 ret;
    ret = float32_sqrt(t0, &env->fp_status);
    update_fpsw(env, ret, GETPC());
    return ret;
}

float32 helper_utof(CPURXState *env, uint32_t t0)
{
    float32 ret;
    ret = uint32_to_float32(t0, &env->fp_status);
    update_fpsw(env, ret, GETPC());
    return ret;
}

float32 helper_itof(CPURXState *env, uint32_t t0)
{
    float32 ret;
    ret = int32_to_float32(t0, &env->fp_status);
    update_fpsw(env, ret, GETPC());
    return ret;
}

/* string operations */
void helper_scmpu(CPURXState *env)
{
    uint8_t tmp0, tmp1;
    if (env->regs[3] == 0) {
        return;
    }
    do {
        tmp0 = cpu_ldub_data_ra(env, env->regs[1]++, GETPC());
        tmp1 = cpu_ldub_data_ra(env, env->regs[2]++, GETPC());
        env->regs[3]--;
        if (tmp0 != tmp1 || tmp0 == '\0') {
            break;
        }
    } while (env->regs[3] != 0);
    env->psw_z = tmp0 - tmp1;
    env->psw_c = (tmp0 >= tmp1);
}

static uint32_t (* const cpu_ldufn[])(CPUArchState *env,
                                     abi_ptr ptr,
                                     uintptr_t retaddr) = {
    cpu_ldub_data_ra, cpu_lduw_le_data_ra, cpu_ldl_le_data_ra,
};

static uint32_t (* const cpu_ldfn[])(CPUArchState *env,
                                     abi_ptr ptr,
                                     uintptr_t retaddr) = {
    cpu_ldub_data_ra, cpu_lduw_le_data_ra, cpu_ldl_le_data_ra,
};

static void (* const cpu_stfn[])(CPUArchState *env,
                                 abi_ptr ptr,
                                 uint32_t val,
                                 uintptr_t retaddr) = {
    cpu_stb_data_ra, cpu_stw_le_data_ra, cpu_stl_le_data_ra,
};

void helper_sstr(CPURXState *env, uint32_t sz)
{
    tcg_debug_assert(sz < 3);
    while (env->regs[3] != 0) {
        cpu_stfn[sz](env, env->regs[1], env->regs[2], GETPC());
        env->regs[1] += 1 << sz;
        env->regs[3]--;
    }
}

#define OP_SMOVU 1
#define OP_SMOVF 0
#define OP_SMOVB 2

static void smov(uint32_t mode, CPURXState *env)
{
    uint8_t tmp;
    int dir;

    dir = (mode & OP_SMOVB) ? -1 : 1;
    while (env->regs[3] != 0) {
        tmp = cpu_ldub_data_ra(env, env->regs[2], GETPC());
        cpu_stb_data_ra(env, env->regs[1], tmp, GETPC());
        env->regs[1] += dir;
        env->regs[2] += dir;
        env->regs[3]--;
        if ((mode & OP_SMOVU) && tmp == 0) {
            break;
        }
    }
}

void helper_smovu(CPURXState *env)
{
    smov(OP_SMOVU, env);
}

void helper_smovf(CPURXState *env)
{
    smov(OP_SMOVF, env);
}

void helper_smovb(CPURXState *env)
{
    smov(OP_SMOVB, env);
}


void helper_suntil(CPURXState *env, uint32_t sz)
{
    uint32_t tmp;
    tcg_debug_assert(sz < 3);
    if (env->regs[3] == 0) {
        return;
    }
    do {
        tmp = cpu_ldufn[sz](env, env->regs[1], GETPC());
        env->regs[1] += 1 << sz;
        env->regs[3]--;
        if (tmp == env->regs[2]) {
            break;
        }
    } while (env->regs[3] != 0);
    env->psw_z = tmp - env->regs[2];
    env->psw_c = (tmp <= env->regs[2]);
}

void helper_swhile(CPURXState *env, uint32_t sz)
{
    uint32_t tmp;
    tcg_debug_assert(sz < 3);
    if (env->regs[3] == 0) {
        return;
    }
    do {
        tmp = cpu_ldufn[sz](env, env->regs[1], GETPC());
        env->regs[1] += 1 << sz;
        env->regs[3]--;
        if (tmp != env->regs[2]) {
            break;
        }
    } while (env->regs[3] != 0);
    env->psw_z = env->regs[3];
    env->psw_c = (tmp <= env->regs[2]);
}

/* accumulator operations */
void helper_rmpa(CPURXState *env, uint32_t sz)
{
    uint64_t result_l, prev;
    int32_t result_h;
    int64_t tmp0, tmp1;

    if (env->regs[3] == 0) {
        return;
    }
    result_l = env->regs[5];
    result_l <<= 32;
    result_l |= env->regs[4];
    result_h = env->regs[6];
    env->psw_o = 0;

    while (env->regs[3] != 0) {
        tmp0 = cpu_ldfn[sz](env, env->regs[1], GETPC());
        tmp1 = cpu_ldfn[sz](env, env->regs[2], GETPC());
        tmp0 *= tmp1;
        prev = result_l;
        result_l += tmp0;
        /* carry / bollow */
        if (tmp0 < 0) {
            if (prev > result_l) {
                result_h--;
            }
        } else {
            if (prev < result_l) {
                result_h++;
            }
        }

        env->regs[1] += 1 << sz;
        env->regs[2] += 1 << sz;
    }
    env->psw_s = result_h;
    env->psw_o = (result_h != 0 && result_h != -1) << 31;
    env->regs[6] = result_h;
    env->regs[5] = result_l >> 32;
    env->regs[4] = result_l & 0xffffffff;
}

/*
 * The accumulators are 72 bits wide on RXv2 and later and 64 bits wide on
 * RXv1; they are kept sign-extended to 128 bits.
 */
static Int128 sext128(Int128 v, int bits)
{
    return int128_rshift(int128_lshift(v, 128 - bits), 128 - bits);
}

static int acc_bits(CPURXState *env)
{
    return rx_has_feature(env, RX_FEATURE_V2) ? 72 : 64;
}

static void acc_set(CPURXState *env, uint32_t a, Int128 v)
{
    env->acc[a] = sext128(v, acc_bits(env));
}

/* mul* / emula, mac* / emaca and msb* / emsba with their product */
void helper_acc_set(CPURXState *env, uint32_t a, int64_t prod)
{
    acc_set(env, a, int128_exts64(prod));
}

void helper_acc_add(CPURXState *env, uint32_t a, int64_t prod)
{
    acc_set(env, a, int128_add(env->acc[a], int128_exts64(prod)));
}

void helper_acc_sub(CPURXState *env, uint32_t a, int64_t prod)
{
    acc_set(env, a, int128_sub(env->acc[a], int128_exts64(prod)));
}

/* mvfacgu / mvfachi / mvfacmi / mvfaclo: bits pos+31..pos of A << shift */
uint32_t helper_mvfac(CPURXState *env, uint32_t a, uint32_t shift,
                      uint32_t pos)
{
    Int128 tmp = sext128(int128_lshift(env->acc[a], shift), 72);

    return int128_getlo(int128_rshift(tmp, pos));
}

/*
 * mvtacgu / mvtachi / mvtaclo: replace bits pos+31..pos of A with val. The
 * RXv3 manual ORs in the source sign-extended to 72 bits for mvtacgu and
 * mvtachi, and zero-extended for mvtaclo.
 */
void helper_mvtac(CPURXState *env, uint32_t a, uint32_t pos, uint32_t val)
{
    Int128 field = int128_lshift(int128_make64(0xffffffff), pos);
    Int128 src = pos ? int128_exts64((int32_t)val) : int128_make64(val);

    acc_set(env, a, int128_or(int128_and(env->acc[a], int128_not(field)),
                              int128_lshift(src, pos)));
}

/*
 * racw / racl / rdacw / rdacl: shift A left, round (RX_RAC_ROUND) and
 * saturate to a word or a longword (RX_RAC_LONG) in bits 63..32.
 */
void helper_rac(CPURXState *env, uint32_t a, uint32_t shift, uint32_t mode)
{
    Int128 tmp, max, min;

    if (mode & RX_RAC_LONG) {
        max = int128_make64(0x7fffffff00000000ULL);
        min = int128_exts64(INT64_MIN);
    } else {
        max = int128_make64(0x00007fff00000000ULL);
        min = int128_exts64(-0x800000000000LL);
    }
    tmp = sext128(int128_lshift(env->acc[a], shift), acc_bits(env));
    if (mode & RX_RAC_ROUND) {
        tmp = int128_add(tmp, int128_make64(0x80000000));
    }
    if (int128_gt(tmp, max)) {
        tmp = max;
    } else if (int128_lt(tmp, min)) {
        tmp = min;
    } else {
        tmp = int128_and(tmp, int128_not(int128_make64(0xffffffff)));
    }
    acc_set(env, a, tmp);
}

void helper_satr(CPURXState *env)
{
    if (env->psw_o >> 31) {
        if ((int)env->psw_s < 0) {
            env->regs[6] = 0x00000000;
            env->regs[5] = 0x7fffffff;
            env->regs[4] = 0xffffffff;
        } else {
            env->regs[6] = 0xffffffff;
            env->regs[5] = 0x80000000;
            env->regs[4] = 0x00000000;
        }
    }
}

/* div */
uint32_t helper_div(CPURXState *env, uint32_t num, uint32_t den)
{
    uint32_t ret = num;
    if (!((num == INT_MIN && den == -1) || den == 0)) {
        ret = (int32_t)num / (int32_t)den;
        env->psw_o = 0;
    } else {
        env->psw_o = -1;
    }
    return ret;
}

uint32_t helper_divu(CPURXState *env, uint32_t num, uint32_t den)
{
    uint32_t ret = num;
    if (den != 0) {
        ret = num / den;
        env->psw_o = 0;
    } else {
        env->psw_o = -1;
    }
    return ret;
}

/* exception */
static inline G_NORETURN
void raise_exception(CPURXState *env, int index,
                     uintptr_t retaddr)
{
    CPUState *cs = env_cpu(env);

    cs->exception_index = index;
    cpu_loop_exit_restore(cs, retaddr);
}

G_NORETURN void helper_raise_privilege_violation(CPURXState *env)
{
    raise_exception(env, 20, GETPC());
}

G_NORETURN void helper_raise_access_fault(CPURXState *env)
{
    raise_exception(env, 21, GETPC());
}

G_NORETURN void helper_raise_illegal_instruction(CPURXState *env)
{
    raise_exception(env, 23, GETPC());
}

G_NORETURN void helper_wait(CPURXState *env)
{
    CPUState *cs = env_cpu(env);

    cs->halted = 1;
    env->in_sleep = 1;
    env->psw_i = 1;
    raise_exception(env, EXCP_HLT, 0);
}

G_NORETURN void helper_rxint(CPURXState *env, uint32_t vec)
{
    raise_exception(env, 0x100 + vec, 0);
}

G_NORETURN void helper_rxbrk(CPURXState *env)
{
    raise_exception(env, 0x100, 0);
}
