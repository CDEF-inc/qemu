/*
 * QEMU RX CPU
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
#include "qemu/qemu-print.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/core/qdev-properties.h"
#include "cpu.h"
#include "migration/vmstate.h"
#include "exec/cputlb.h"
#include "exec/page-protection.h"
#include "exec/translation-block.h"
#include "exec/target_page.h"
#include "hw/core/loader.h"
#include "fpu/softfloat.h"
#include "tcg/debug-assert.h"
#include "accel/tcg/cpu-ops.h"

static void rx_cpu_set_pc(CPUState *cs, vaddr value)
{
    RXCPU *cpu = RX_CPU(cs);

    cpu->env.pc = value;
}

static vaddr rx_cpu_get_pc(CPUState *cs)
{
    RXCPU *cpu = RX_CPU(cs);

    return cpu->env.pc;
}

static TCGTBCPUState rx_get_tb_cpu_state(CPUState *cs)
{
    CPURXState *env = cpu_env(cs);
    uint32_t flags = 0;

    flags = FIELD_DP32(flags, PSW, PM, env->psw_pm);
    flags = FIELD_DP32(flags, PSW, U, env->psw_u);
    flags = FIELD_DP32(flags, TB, WORLD, rx_world(env));

    return (TCGTBCPUState){ .pc = env->pc, .flags = flags };
}

static void rx_cpu_synchronize_from_tb(CPUState *cs,
                                       const TranslationBlock *tb)
{
    RXCPU *cpu = RX_CPU(cs);

    tcg_debug_assert(!tcg_cflags_has(cs, CF_PCREL));
    cpu->env.pc = tb->pc;
}

static void rx_restore_state_to_opc(CPUState *cs,
                                    const TranslationBlock *tb,
                                    const uint64_t *data)
{
    RXCPU *cpu = RX_CPU(cs);

    cpu->env.pc = data[0];
}

static bool rx_cpu_has_work(CPUState *cs)
{
    return cpu_test_interrupt(cs, CPU_INTERRUPT_HARD | CPU_INTERRUPT_FIR);
}

static int rx_cpu_mmu_index(CPUState *cs, bool ifunc)
{
    return 0;
}

static void rx_cpu_reset_hold(Object *obj, ResetType type)
{
    CPUState *cs = CPU(obj);
    RXCPUClass *rcc = RX_CPU_GET_CLASS(obj);
    CPURXState *env = cpu_env(cs);
    uint32_t *resetvec;

    if (rcc->parent_phases.hold) {
        rcc->parent_phases.hold(obj, type);
    }

    memset(env, 0, offsetof(CPURXState, end_reset_fields));

    resetvec = rom_ptr(0xfffffffc, 4);
    if (resetvec) {
        /* In the case of kernel, it is ignored because it is not set. */
        env->pc = ldl_le_p(resetvec);
    }
    rx_cpu_unpack_psw(env, 0, 1);
    env->regs[0] = env->isp = env->usp = 0;
    env->extb = 0xffffff80;
    env->trctb = env->wintb = 0x80000000;  /* RXv3e */
    env->fpsw = R_FPSW_DN_MASK;
    set_float_exception_flags(0, &env->fp_status);
    set_flush_to_zero(1, &env->fp_status);
    set_flush_inputs_to_zero(1, &env->fp_status);
    /*
     * TODO: this is not the correct NaN propagation rule for this
     * architecture. The "RX Family User's Manual: Software" table 1.6
     * defines the propagation rules as "prefer SNaN over QNaN;
     * then prefer dest over source", which is float_2nan_prop_s_ab.
     */
    set_float_2nan_prop_rule(float_2nan_prop_x87, &env->fp_status);
    /*
     * Default NaN value: 7fffffff (sign bit clear, all fraction bits set),
     * the result of an invalid operation without NaN operands
     * ("RX Family RXv3 Instruction Set Architecture" table 1.7).
     */
    set_float_default_nan_pattern(0b01111111, &env->fp_status);
    /*
     * TODO: "RX Family RXv1 Instruction Set Architecture" is not 100% clear
     * on whether flush-to-zero should happen before or after rounding, but
     * section 1.3.2 says that it happens when underflow is detected, and
     * implies that underflow is detected after rounding. So this may not
     * be the correct setting.
     */
    set_float_ftz_detection(float_ftz_before_rounding, &env->fp_status);

    /*
     * Double-precision FPU (RXv3 manual 1.10): DPSW resets with DDN = 1 and
     * DECNT with EHM = 1. Denormals are always flushed, as for the FPSW
     * (see update_dfpu). Table 1.14: an SNaN operand first, then src2
     * (the first operand of the softfloat calls); an invalid operation
     * without NaN operands gives 7fffffffffffffff.
     */
    env->dpsw = R_DPSW_DDN_MASK;
    env->decnt = R_DECNT_EHM_MASK;
    set_float_rounding_mode(float_round_nearest_even, &env->dfp_status);
    set_float_exception_flags(0, &env->dfp_status);
    set_flush_to_zero(1, &env->dfp_status);
    set_flush_inputs_to_zero(1, &env->dfp_status);
    set_float_2nan_prop_rule(float_2nan_prop_s_ab, &env->dfp_status);
    set_float_default_nan_pattern(0b01111111, &env->dfp_status);
    set_float_ftz_detection(float_ftz_before_rounding, &env->dfp_status);
}

static ObjectClass *rx_cpu_class_by_name(const char *cpu_model)
{
    ObjectClass *oc;
    char *typename;

    oc = object_class_by_name(cpu_model);
    if (oc != NULL && object_class_dynamic_cast(oc, TYPE_RX_CPU) != NULL) {
        return oc;
    }
    typename = g_strdup_printf(RX_CPU_TYPE_NAME("%s"), cpu_model);
    oc = object_class_by_name(typename);
    g_free(typename);

    return oc;
}

static void rx_cpu_realize(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    RXCPUClass *rcc = RX_CPU_GET_CLASS(dev);
    CPURXState *env = cpu_env(cs);
    Error *local_err = NULL;

    if (rx_has_feature(env, RX_FEATURE_DFPU) &&
        !rx_has_feature(env, RX_FEATURE_V3)) {
        error_setg(errp, "dfpu needs an RXv3 or later CPU");
        return;
    }
    if (env->nr_banks > RX_MAX_BANKS) {
        error_setg(errp, "banks must be at most %d", RX_MAX_BANKS);
        return;
    }
    if (env->nr_banks && !rx_has_feature(env, RX_FEATURE_V3)) {
        error_setg(errp, "register banks need an RXv3 or later CPU");
        return;
    }

    cpu_exec_realizefn(cs, &local_err);
    if (local_err != NULL) {
        error_propagate(errp, local_err);
        return;
    }

    qemu_init_vcpu(cs);
    cpu_reset(cs);

    rcc->parent_realize(dev, errp);
}

static void rx_cpu_set_irq(void *opaque, int no, int request)
{
    RXCPU *cpu = opaque;
    CPUState *cs = CPU(cpu);
    int irq = request & 0xff;

    static const int mask[] = {
        [RX_CPU_IRQ] = CPU_INTERRUPT_HARD,
        [RX_CPU_FIR] = CPU_INTERRUPT_FIR,
    };
    if (irq) {
        cpu->env.req_irq = irq;
        cpu->env.req_ipl = (request >> 8) & 0x0f;
        cpu_interrupt(cs, mask[no]);
    } else {
        cpu_reset_interrupt(cs, mask[no]);
    }
}

static void rx_cpu_disas_set_info(const CPUState *cpu, disassemble_info *info)
{
    info->endian = BFD_ENDIAN_LITTLE;
    info->mach = bfd_mach_rx;
    info->print_insn = print_insn_rx;
}

static bool rx_cpu_tlb_fill(CPUState *cs, vaddr addr, int size,
                            MMUAccessType access_type, int mmu_idx,
                            bool probe, uintptr_t retaddr)
{
    uint32_t address, physical, prot;

    /* Linear mapping */
    address = physical = addr & TARGET_PAGE_MASK;
    prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
    tlb_set_page(cs, address, physical, prot, mmu_idx, TARGET_PAGE_SIZE);
    return true;
}

static void rx_cpu_init(Object *obj)
{
    RXCPU *cpu = RX_CPU(obj);

    cpu->env.features = RX_CPU_GET_CLASS(obj)->features;
    cpu->env.nr_banks = RX_CPU_GET_CLASS(obj)->nr_banks;
    qdev_init_gpio_in(DEVICE(cpu), rx_cpu_set_irq, 2);
}

/*
 * The translator reads the features when it translates a block, and they
 * are not part of the TB flags, so they cannot change once the CPU is
 * realized (and rx_cpu_realize() checks their combination).
 */
static void rx_cpu_set_feature(Object *obj, const char *name,
                               uint32_t feature, bool value, Error **errp)
{
    CPURXState *env = &RX_CPU(obj)->env;

    if (DEVICE(obj)->realized) {
        qdev_prop_set_after_realize(DEVICE(obj), name, errp);
        return;
    }
    if (value) {
        env->features |= feature;
    } else {
        env->features &= ~feature;
    }
}

static bool rx_cpu_get_fpu(Object *obj, Error **errp)
{
    return rx_has_feature(&RX_CPU(obj)->env, RX_FEATURE_FPU);
}

static void rx_cpu_set_fpu(Object *obj, bool value, Error **errp)
{
    rx_cpu_set_feature(obj, "fpu", RX_FEATURE_FPU, value, errp);
}

static bool rx_cpu_get_dfpu(Object *obj, Error **errp)
{
    return rx_has_feature(&RX_CPU(obj)->env, RX_FEATURE_DFPU);
}

static void rx_cpu_set_dfpu(Object *obj, bool value, Error **errp)
{
    rx_cpu_set_feature(obj, "dfpu", RX_FEATURE_DFPU, value, errp);
}

static void rx_cpu_get_banks(Object *obj, Visitor *v, const char *name,
                             void *opaque, Error **errp)
{
    visit_type_uint32(v, name, &RX_CPU(obj)->env.nr_banks, errp);
}

static void rx_cpu_set_banks(Object *obj, Visitor *v, const char *name,
                             void *opaque, Error **errp)
{
    /* Like the features, read by the translator */
    if (DEVICE(obj)->realized) {
        qdev_prop_set_after_realize(DEVICE(obj), name, errp);
        return;
    }
    visit_type_uint32(v, name, &RX_CPU(obj)->env.nr_banks, errp);
}

#include "hw/core/sysemu-cpu-ops.h"

static const struct SysemuCPUOps rx_sysemu_ops = {
    .has_work = rx_cpu_has_work,
    .get_phys_addr_debug = rx_cpu_get_phys_addr_debug,
};

static const TCGCPUOps rx_tcg_ops = {
    /* MTTCG not yet supported: require strict ordering */
    .guest_default_memory_order = TCG_MO_ALL,
    .mttcg_supported = false,

    .initialize = rx_translate_init,
    .translate_code = rx_translate_code,
    .get_tb_cpu_state = rx_get_tb_cpu_state,
    .synchronize_from_tb = rx_cpu_synchronize_from_tb,
    .restore_state_to_opc = rx_restore_state_to_opc,
    .mmu_index = rx_cpu_mmu_index,
    .tlb_fill = rx_cpu_tlb_fill,
    .pointer_wrap = cpu_pointer_wrap_uint32,

    .cpu_exec_interrupt = rx_cpu_exec_interrupt,
    .cpu_exec_halt = rx_cpu_has_work,
    .cpu_exec_reset = cpu_reset,
    .do_interrupt = rx_cpu_do_interrupt,
};

static void rx_cpu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    CPUClass *cc = CPU_CLASS(klass);
    RXCPUClass *rcc = RX_CPU_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    device_class_set_parent_realize(dc, rx_cpu_realize,
                                    &rcc->parent_realize);
    resettable_class_set_parent_phases(rc, NULL, rx_cpu_reset_hold, NULL,
                                       &rcc->parent_phases);

    cc->class_by_name = rx_cpu_class_by_name;
    cc->dump_state = rx_cpu_dump_state;
    cc->set_pc = rx_cpu_set_pc;
    cc->get_pc = rx_cpu_get_pc;

    cc->sysemu_ops = &rx_sysemu_ops;
    cc->gdb_read_register = rx_cpu_gdb_read_register;
    cc->gdb_write_register = rx_cpu_gdb_write_register;
    cc->disas_set_info = rx_cpu_disas_set_info;

    cc->gdb_core_xml_file = "rx-core.xml";
    cc->tcg_ops = &rx_tcg_ops;

    object_class_property_add_bool(klass, "fpu", rx_cpu_get_fpu,
                                   rx_cpu_set_fpu);
    object_class_property_set_description(klass, "fpu",
        "Single-precision floating-point unit");
    object_class_property_add_bool(klass, "dfpu", rx_cpu_get_dfpu,
                                   rx_cpu_set_dfpu);
    object_class_property_set_description(klass, "dfpu",
        "Double-precision floating-point unit (RXv3 and later)");
    object_class_property_add(klass, "banks", "uint32", rx_cpu_get_banks,
                              rx_cpu_set_banks, NULL, NULL);
    object_class_property_set_description(klass, "banks",
        "Number of save register banks for SAVE / RSTR (RXv3 and later)");
}

/*
 * The generic models enable the units that LLVM's -mcpu of the same name
 * assumes: RXv1 cores may lack the FPU (RX100, RX200), RXv2 and later
 * have it, and RXv3 parts commonly have the double-precision FPU. Use
 * fpu=on|off and dfpu=on|off to model a particular part. RXv3 cores get 16
 * save register banks, as on the RX72N and RX66N (banks=N changes it).
 */
static void rx_cpu_class_init_features(ObjectClass *oc, const void *data)
{
    RXCPUClass *rcc = RX_CPU_CLASS(oc);

    rcc->features = (uintptr_t)data;
    rcc->nr_banks = (rcc->features & RX_FEATURE_V3) ? 16 : 0;
}

#define DEFINE_RX_CPU_TYPE(type, feats)                 \
    {                                                   \
        .name = type,                                   \
        .parent = TYPE_RX_CPU,                          \
        .class_init = rx_cpu_class_init_features,       \
        .class_data = (const void *)(uintptr_t)(feats), \
    }

static const TypeInfo rx_cpu_types[] = {
    {
        .name = TYPE_RX_CPU,
        .parent = TYPE_CPU,
        .instance_size = sizeof(RXCPU),
        .instance_align = __alignof(RXCPU),
        .instance_init = rx_cpu_init,
        .abstract = true,
        .class_size = sizeof(RXCPUClass),
        .class_init = rx_cpu_class_init,
    },
    DEFINE_RX_CPU_TYPE(TYPE_RX62N_CPU, RX_FEATURE_FPU),
    DEFINE_RX_CPU_TYPE(TYPE_RXV1_CPU, 0),
    DEFINE_RX_CPU_TYPE(TYPE_RXV2_CPU, RX_FEATURE_V2 | RX_FEATURE_FPU),
    DEFINE_RX_CPU_TYPE(TYPE_RXV3_CPU, RX_FEATURE_V2 | RX_FEATURE_V3 |
                       RX_FEATURE_FPU | RX_FEATURE_DFPU),
    DEFINE_RX_CPU_TYPE(TYPE_RXV3E_CPU, RX_FEATURE_V2 | RX_FEATURE_V3 |
                       RX_FEATURE_V3E | RX_FEATURE_FPU | RX_FEATURE_DFPU),
};

DEFINE_TYPES(rx_cpu_types)
