/*
 *  RX emulation definition
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

#ifndef RX_CPU_H
#define RX_CPU_H

#include "qemu/bitops.h"
#include "qemu/int128.h"
#include "hw/core/registerfields.h"
#include "cpu-qom.h"

#include "exec/cpu-common.h"
#include "exec/cpu-interrupt.h"
#include "qemu/cpu-float.h"

#ifdef CONFIG_USER_ONLY
#error "RX does not support user mode emulation"
#endif

/* PSW define */
REG32(PSW, 0)
FIELD(PSW, C, 0, 1)
FIELD(PSW, Z, 1, 1)
FIELD(PSW, S, 2, 1)
FIELD(PSW, O, 3, 1)
FIELD(PSW, I, 16, 1)
FIELD(PSW, U, 17, 1)
FIELD(PSW, PM, 20, 1)
FIELD(PSW, IPL, 24, 5)            /* IPL[4] on RXv3e only */

/* FPSW define */
REG32(FPSW, 0)
FIELD(FPSW, RM, 0, 2)
FIELD(FPSW, CV, 2, 1)
FIELD(FPSW, CO, 3, 1)
FIELD(FPSW, CZ, 4, 1)
FIELD(FPSW, CU, 5, 1)
FIELD(FPSW, CX, 6, 1)
FIELD(FPSW, CE, 7, 1)
FIELD(FPSW, CAUSE, 2, 6)
FIELD(FPSW, DN, 8, 1)
FIELD(FPSW, EV, 10, 1)
FIELD(FPSW, EO, 11, 1)
FIELD(FPSW, EZ, 12, 1)
FIELD(FPSW, EU, 13, 1)
FIELD(FPSW, EX, 14, 1)
FIELD(FPSW, ENABLE, 10, 5)
FIELD(FPSW, FV, 26, 1)
FIELD(FPSW, FO, 27, 1)
FIELD(FPSW, FZ, 28, 1)
FIELD(FPSW, FU, 29, 1)
FIELD(FPSW, FX, 30, 1)
FIELD(FPSW, FLAGS, 26, 4)
FIELD(FPSW, FS, 31, 1)

/* DPSW define (double-precision FPU; the same layout as the FPSW) */
REG32(DPSW, 0)
FIELD(DPSW, DRM, 0, 2)
FIELD(DPSW, CAUSE, 2, 6)
FIELD(DPSW, DCE, 7, 1)
FIELD(DPSW, DDN, 8, 1)
FIELD(DPSW, ENABLE, 10, 5)
FIELD(DPSW, FLAGS, 26, 4)
FIELD(DPSW, DFS, 31, 1)

/* IVC define (RXv3e) */
REG32(IVC, 0)
FIELD(IVC, BWINM, 0, 8)
FIELD(IVC, UIR, 16, 2)
FIELD(IVC, NSIC, 24, 1)
FIELD(IVC, EXCSE, 28, 1)

/* WID define (RXv3e) */
REG32(WID, 0)
FIELD(WID, WID, 0, 2)
FIELD(WID, BWID, 8, 2)
FIELD(WID, BITW, 16, 2)
FIELD(WID, BWI, 31, 1)

/* WPRI define (RXv3e) */
REG32(WPRI, 0)
FIELD(WPRI, WIPL, 24, 5)

/* TB flags: PSW.PM and PSW.U at their PSW positions, and the world */
REG32(TB, 0)
FIELD(TB, WORLD, 0, 2)

/* The worlds of the RXv3e Protect Zone function */
enum {
    RX_WORLD_ROOT = 0,
    RX_WORLD_SECURE = 1,
    RX_WORLD_NONSECURE = 2,
};

/* The control registers that RXv3e implements once per world */
enum {
    RX_WCR_ISP, RX_WCR_INTB, RX_WCR_BPC, RX_WCR_BPSW, RX_WCR_EXTB, RX_WCR_IVC,
    RX_WCR_NUM,
};

/* DECNT define */
REG32(DECNT, 0)
FIELD(DECNT, EHM, 0, 1)
FIELD(DECNT, EHS, 16, 1)

enum {
    NUM_REGS = 16,
    RX_MAX_BANKS = 256,         /* save register banks (RXv3) */
};

/* A save register bank of the RXv3 register bank save function */
typedef struct RXRegBank {
    uint32_t regs[NUM_REGS - 1];        /* R1 to R15 */
    uint32_t usp;
    uint32_t fpsw;
    Int128 acc[2];
} RXRegBank;

/* Instruction set and optional units of a CPU model */
enum {
    RX_FEATURE_V2   = 1 << 0,   /* RXv2 instructions */
    RX_FEATURE_V3   = 1 << 1,   /* RXv3 instructions */
    RX_FEATURE_V3E  = 1 << 2,   /* RXv3e instructions */
    RX_FEATURE_FPU  = 1 << 3,   /* single-precision FPU */
    RX_FEATURE_DFPU = 1 << 4,   /* double-precision FPU (RXv3 and later) */
};

/* Modes of the rac helper (RACW, RACL, RDACW, RDACL) */
enum {
    RX_RAC_ROUND = 1 << 0,      /* add 80000000h first (RACW, RACL) */
    RX_RAC_LONG  = 1 << 1,      /* saturate to a longword (RACL, RDACL) */
};

typedef struct CPUArchState {
    /* CPU registers */
    uint32_t regs[NUM_REGS];    /* general registers */
    uint32_t psw_o;             /* O bit of status register */
    uint32_t psw_s;             /* S bit of status register */
    uint32_t psw_z;             /* Z bit of status register */
    uint32_t psw_c;             /* C bit of status register */
    uint32_t psw_u;
    uint32_t psw_i;
    uint32_t psw_pm;
    uint32_t psw_ipl;
    uint32_t bpsw;              /* backup status */
    uint32_t bpc;               /* backup pc */
    uint32_t isp;               /* global base register */
    uint32_t usp;               /* vector base register */
    uint32_t pc;                /* program counter */
    uint32_t intb;              /* interrupt vector */
    uint32_t fintv;
    uint32_t fpsw;
    Int128 acc[2];              /* accumulators A0 and A1, sign-extended */
    uint32_t extb;              /* exception table register (RXv2) */
    uint32_t li;                /* LI flag of MOVLI / MOVCO (RXv2) */
    uint64_t dr[16];            /* double-precision FPU (RXv3): DR0-DR15 */
    uint32_t dpsw;
    uint32_t dcmr;
    uint32_t decnt;
    uint32_t depc;
    /*
     * RXv3e: isp, intb, bpc, bpsw, extb and ivc are those of the current
     * world; world_cr keeps the other worlds' (RX_WCR_*).
     */
    uint32_t ivc;
    uint32_t wid;
    uint32_t wpri;
    uint32_t trctb;
    uint32_t wintb;
    uint32_t world_cr[3][RX_WCR_NUM];

    /* Fields up to this point are cleared by a CPU reset */
    struct {} end_reset_fields;

    /* Internal use */
    uint32_t features;          /* RX_FEATURE_* of the CPU model */
    uint32_t nr_banks;          /* number of save register banks */
    RXRegBank banks[RX_MAX_BANKS];  /* undefined after a reset */
    uint32_t in_sleep;
    uint32_t req_irq;           /* Requested interrupt no (hard) */
    uint32_t req_ipl;           /* Requested interrupt level */
    uint32_t ack_irq;           /* execute irq */
    uint32_t ack_ipl;           /* execute ipl */
    float_status fp_status;
    float_status dfp_status;    /* double-precision FPU */
    qemu_irq ack;               /* Interrupt acknowledge */
} CPURXState;

/*
 * RXCPU:
 * @env: #CPURXState
 *
 * A RX CPU
 */
struct ArchCPU {
    CPUState parent_obj;

    CPURXState env;
};

/*
 * RXCPUClass:
 * @parent_realize: The parent class' realize handler.
 * @parent_phases: The parent class' reset phase handlers.
 *
 * A RX CPU model.
 */
struct RXCPUClass {
    CPUClass parent_class;

    DeviceRealize parent_realize;
    ResettablePhases parent_phases;
    uint32_t features;          /* RX_FEATURE_* of the model */
    uint32_t nr_banks;          /* save register banks of the model */
};

#define CPU_RESOLVING_TYPE TYPE_RX_CPU

const char *rx_crname(uint8_t cr);
void rx_cpu_do_interrupt(CPUState *cpu);
bool rx_cpu_exec_interrupt(CPUState *cpu, int int_req);
hwaddr rx_cpu_get_phys_addr_debug(CPUState *cpu, vaddr addr);
void rx_cpu_dump_state(CPUState *cpu, FILE *f, int flags);
int rx_cpu_gdb_read_register(CPUState *cpu, GByteArray *buf, int reg);
int rx_cpu_gdb_write_register(CPUState *cpu, uint8_t *buf, int reg);

void rx_translate_init(void);
void rx_translate_code(CPUState *cs, TranslationBlock *tb,
                       int *max_insns, vaddr pc, void *host_pc);
void rx_cpu_unpack_psw(CPURXState *env, uint32_t psw, int rte);

static inline bool rx_has_feature(CPURXState *env, uint32_t feature)
{
    return (env->features & feature) != 0;
}

/* The current world (RX_WORLD_*); always root before RXv3e */
static inline int rx_world(CPURXState *env)
{
    return FIELD_EX32(env->wid, WID, WID);
}

void rx_world_switch(CPURXState *env, int world);
uint32_t *rx_world_cr(CPURXState *env, int world, int idx);

#define CPU_INTERRUPT_SOFT CPU_INTERRUPT_TGT_INT_0
#define CPU_INTERRUPT_FIR  CPU_INTERRUPT_TGT_INT_1

#define RX_CPU_IRQ 0
#define RX_CPU_FIR 1

static inline uint32_t rx_cpu_pack_psw(CPURXState *env)
{
    uint32_t psw = 0;
    psw = FIELD_DP32(psw, PSW, IPL, env->psw_ipl);
    psw = FIELD_DP32(psw, PSW, PM,  env->psw_pm);
    psw = FIELD_DP32(psw, PSW, U,   env->psw_u);
    psw = FIELD_DP32(psw, PSW, I,   env->psw_i);
    psw = FIELD_DP32(psw, PSW, O,   env->psw_o >> 31);
    psw = FIELD_DP32(psw, PSW, S,   env->psw_s >> 31);
    psw = FIELD_DP32(psw, PSW, Z,   env->psw_z == 0);
    psw = FIELD_DP32(psw, PSW, C,   env->psw_c);
    return psw;
}

#endif /* RX_CPU_H */
