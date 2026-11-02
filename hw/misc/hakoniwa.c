/*
 * Hakoniwa asset bridge
 *
 * Runs the guest as a Hakoniwa (箱庭) asset: the guest's virtual time
 * advances in lock step with the Hakoniwa simulation time, and two PDU
 * channels are exposed to the guest as a small MMIO window.
 *
 * Needs -icount shift=N,sleep=off, so that virtual time only moves with
 * the instructions the guest executes. delta-us is the guest step; the
 * Hakoniwa conductor and the other assets must use delta-us / 2 (see
 * hako_on_manual_timing_control).
 *
 * Register map (little endian, 32-bit accesses):
 *   0x000 TIME_LO   Hakoniwa simulation time in usec, low word (RO)
 *   0x004 TIME_HI   high word (RO)
 *   0x008 STEP      number of completed Hakoniwa steps (RO)
 *   0x100 IN[]      last PDU read from in-channel (RO)
 *   0x200 OUT[]     PDU written to out-channel at the next step (RW)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/thread.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "exec/icount.h"
#include "hw/core/qdev-properties.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "system/runstate.h"
#include "hako_asset.h"

#define TYPE_HAKONIWA_ASSET "hakoniwa-asset"
OBJECT_DECLARE_SIMPLE_TYPE(HakoniwaAssetState, HAKONIWA_ASSET)

#define HAKO_REG_TIME_LO    0x000
#define HAKO_REG_TIME_HI    0x004
#define HAKO_REG_STEP       0x008
#define HAKO_REG_IN         0x100
#define HAKO_REG_OUT        0x200
#define HAKO_PDU_MAX        0x100
#define HAKO_MMIO_SIZE      0x300

struct HakoniwaAssetState {
    DeviceState parent_obj;

    MemoryRegion mmio;
    QEMUTimer *timer;
    QemuThread thread;
    /* QEMU -> asset thread: the guest has run up to the step boundary. */
    QemuSemaphore guest_done;
    /* asset thread -> QEMU: Hakoniwa time moved on, PDUs exchanged. */
    QemuSemaphore step_go;
    bool stopped;

    uint64_t sim_time_us;
    uint32_t step;
    uint8_t in[HAKO_PDU_MAX];
    uint8_t out[HAKO_PDU_MAX];

    char *asset;
    char *config;
    char *robot;
    uint64_t addr;
    uint32_t delta_us;
    uint32_t in_channel;
    uint32_t out_channel;
    uint32_t in_size;
    uint32_t out_size;
};

/* hako_asset callbacks take no user pointer; there is one bridge per QEMU. */
static HakoniwaAssetState *hako_state;

static int hako_on_initialize(hako_asset_context_t *ctx)
{
    return 0;
}

static int hako_on_reset(hako_asset_context_t *ctx)
{
    return 0;
}

/*
 * One loop per guest step of delta_us, which is two Hakoniwa steps of
 * delta_us / 2. Hakoniwa time is a barrier: no asset gets past time t
 * until every asset has called usleep() after reaching t - delta. With one
 * barrier per step, a reader could see the PDU from either side of the
 * writer's update. So every asset writes in the first half step and reads
 * in the second:
 *
 *   write OUT | barrier | read IN | barrier | (next step) write OUT ...
 *
 * and every read sees exactly the writes of the same step.
 */
static int hako_on_manual_timing_control(hako_asset_context_t *ctx)
{
    HakoniwaAssetState *s = hako_state;
    hako_time_t half = s->delta_us / 2;

    for (;;) {
        qemu_sem_wait(&s->guest_done);
        hako_asset_pdu_write(s->robot, s->out_channel, (char *)s->out,
                             s->out_size);
        /* After "hako-cmd stop" this returns only once reset follows. */
        if (hako_asset_usleep(half) != 0) {
            break;
        }
        hako_asset_pdu_read(s->robot, s->in_channel, (char *)s->in,
                            s->in_size);
        if (hako_asset_usleep(half) != 0) {
            break;
        }
        s->sim_time_us = hako_asset_simulation_time();
        s->step++;
        qemu_sem_post(&s->step_go);
    }
    s->stopped = true;
    qemu_sem_post(&s->step_go);
    return 0;
}

static hako_asset_callbacks_t hako_callbacks = {
    .on_initialize = hako_on_initialize,
    .on_manual_timing_control = hako_on_manual_timing_control,
    .on_reset = hako_on_reset,
};

static void *hako_thread_fn(void *opaque)
{
    HakoniwaAssetState *s = opaque;

    if (hako_asset_register(s->asset, s->config, &hako_callbacks,
                            s->delta_us / 2,
                            HAKO_ASSET_MODEL_CONTROLLER) != 0) {
        error_report("hakoniwa: hako_asset_register(%s) failed: %s",
                     s->asset, hako_asset_last_error());
        s->stopped = true;
        qemu_sem_post(&s->step_go);
        return NULL;
    }
    /* Blocks until the simulation is started, then runs the callbacks. */
    hako_asset_start();
    return NULL;
}

/*
 * Fires on QEMU_CLOCK_VIRTUAL every delta_us. It runs with the BQL held, so
 * the vCPU cannot move on while we wait for Hakoniwa.
 */
static void hako_timer_cb(void *opaque)
{
    HakoniwaAssetState *s = opaque;

    qemu_sem_post(&s->guest_done);
    qemu_sem_wait(&s->step_go);
    if (s->stopped) {
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_HOST_UI);
        return;
    }
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                        (int64_t)s->delta_us * SCALE_US);
}

static uint64_t hako_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    HakoniwaAssetState *s = opaque;

    switch (addr) {
    case HAKO_REG_TIME_LO:
        return (uint32_t)s->sim_time_us;
    case HAKO_REG_TIME_HI:
        return s->sim_time_us >> 32;
    case HAKO_REG_STEP:
        return s->step;
    case HAKO_REG_IN ... HAKO_REG_IN + HAKO_PDU_MAX - 4:
        return ldl_le_p(&s->in[addr - HAKO_REG_IN]);
    case HAKO_REG_OUT ... HAKO_REG_OUT + HAKO_PDU_MAX - 4:
        return ldl_le_p(&s->out[addr - HAKO_REG_OUT]);
    default:
        return 0;
    }
}

static void hako_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    HakoniwaAssetState *s = opaque;

    if (addr >= HAKO_REG_OUT && addr <= HAKO_REG_OUT + HAKO_PDU_MAX - 4) {
        stl_le_p(&s->out[addr - HAKO_REG_OUT], val);
    }
}

static const MemoryRegionOps hako_mmio_ops = {
    .read = hako_mmio_read,
    .write = hako_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hako_realize(DeviceState *dev, Error **errp)
{
    HakoniwaAssetState *s = HAKONIWA_ASSET(dev);

    if (hako_state) {
        error_setg(errp, "only one %s is supported", TYPE_HAKONIWA_ASSET);
        return;
    }
    if (!icount_enabled()) {
        error_setg(errp, "%s needs -icount shift=N,sleep=off",
                   TYPE_HAKONIWA_ASSET);
        return;
    }
    if (!s->asset || !s->config || !s->robot) {
        error_setg(errp, "asset, config and robot must be set");
        return;
    }
    if (s->delta_us < 2 || s->delta_us % 2) {
        error_setg(errp, "delta-us must be even");
        return;
    }
    if (s->in_size > HAKO_PDU_MAX || s->out_size > HAKO_PDU_MAX) {
        error_setg(errp, "PDU size must be at most %d", HAKO_PDU_MAX);
        return;
    }
    hako_state = s;

    memory_region_init_io(&s->mmio, OBJECT(dev), &hako_mmio_ops, s,
                          TYPE_HAKONIWA_ASSET, HAKO_MMIO_SIZE);
    memory_region_add_subregion(get_system_memory(), s->addr, &s->mmio);

    qemu_sem_init(&s->guest_done, 0);
    qemu_sem_init(&s->step_go, 0);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, hako_timer_cb, s);
    /* The first step waits at time 0, before the guest runs at all. */
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    qemu_thread_create(&s->thread, "hakoniwa", hako_thread_fn, s,
                       QEMU_THREAD_DETACHED);
}

static const Property hako_props[] = {
    DEFINE_PROP_STRING("asset", HakoniwaAssetState, asset),
    DEFINE_PROP_STRING("config", HakoniwaAssetState, config),
    DEFINE_PROP_STRING("robot", HakoniwaAssetState, robot),
    DEFINE_PROP_UINT64("addr", HakoniwaAssetState, addr, 0),
    DEFINE_PROP_UINT32("delta-us", HakoniwaAssetState, delta_us, 1000),
    DEFINE_PROP_UINT32("in-channel", HakoniwaAssetState, in_channel, 0),
    DEFINE_PROP_UINT32("out-channel", HakoniwaAssetState, out_channel, 1),
    DEFINE_PROP_UINT32("in-size", HakoniwaAssetState, in_size, 8),
    DEFINE_PROP_UINT32("out-size", HakoniwaAssetState, out_size, 8),
};

static void hako_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = hako_realize;
    device_class_set_props(dc, hako_props);
    dc->desc = "Hakoniwa asset bridge";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo hako_info = {
    .name = TYPE_HAKONIWA_ASSET,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(HakoniwaAssetState),
    .class_init = hako_class_init,
};

static void hako_register_types(void)
{
    type_register_static(&hako_info);
}

type_init(hako_register_types)
