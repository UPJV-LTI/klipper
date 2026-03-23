// Handling of stepper drivers.
//
// Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_is_before
#include "command.h" // DECL_COMMAND
#include "sched.h" // struct timer
#include "stepper.h" // stepper_event
#include "trsync.h" // trsync_add_signal

DECL_CONSTANT("STEPPER_STEP_BOTH_EDGE", 1);

#if CONFIG_INLINE_STEPPER_HACK && CONFIG_WANT_STEPPER_OPTIMIZED_BOTH_EDGE
 #define HAVE_EDGE_OPTIMIZATION 1
 #define HAVE_AVR_OPTIMIZATION 0
#elif CONFIG_INLINE_STEPPER_HACK && CONFIG_MACH_AVR
 #define HAVE_EDGE_OPTIMIZATION 0
 #define HAVE_AVR_OPTIMIZATION 1
#else
 #define HAVE_EDGE_OPTIMIZATION 0
 #define HAVE_AVR_OPTIMIZATION 0
#endif

struct stepper_move {
    struct move_node node;
    uint32_t interval;
    int16_t add;
    uint16_t count;
    uint8_t flags;
};

enum { MF_DIR=1<<0 };

struct stepper {
    struct timer time;
    uint32_t interval;
    int16_t add;
    uint32_t count;
    uint32_t next_step_time, step_pulse_ticks;
    struct gpio_out step_pin, dir_pin;
    uint32_t position;
    struct move_queue_head mq;
    struct trsync_signal stop_signal;
    // gcc (pre v6) does better optimization when uint8_t are bitfields
    uint8_t flags : 8;
    // NULL for regular steppers; non-NULL activates inkjet nozzle mode
    struct inkjet_pins *inkjet;
};

enum { POSITION_BIAS=0x40000000 };

enum {
    SF_LAST_DIR=1<<0, SF_NEXT_DIR=1<<1, SF_INVERT_STEP=1<<2, SF_NEED_RESET=1<<3,
    SF_SINGLE_SCHED=1<<4, SF_OPTIMIZED_PATH=1<<5, SF_HAVE_ADD=1<<6
};

// ============================================================
// HPC6602 Inkjet nozzle mode
// Replaces the E0 stepper step pulse with nozzle firing logic.
// 12 nozzles addressed via 4 GPIO bits (pin_a..pin_d) + one
// fire-pulse pin (pin_pulse).  The 12-bit nozzle_code bitmask
// (values 0x000..0xFFF) selects which nozzles fire on each
// "step" event.  0x1000 (4096) means "no fire" (idle / E1 mode).
// ============================================================

struct inkjet_pins {
    struct timer         code_timer;       // schedules future nozzle_code updates
    struct gpio_out      pin_a, pin_b, pin_c, pin_d, pin_pulse;
    uint16_t             nozzle_code;      // active pattern (4096 = disabled)
    uint16_t             pending_code;     // code to apply at code_timer.waketime
    uint8_t              code_timer_active;
};

// Setup a stepper for the next move in its queue
static uint_fast8_t
stepper_load_next(struct stepper *s)
{
    if (move_queue_empty(&s->mq)) {
        // There is no next move - the queue is empty
        s->count = 0;
        return SF_DONE;
    }

    // Read next 'struct stepper_move'
    struct move_node *mn = move_queue_pop(&s->mq);
    struct stepper_move *m = container_of(mn, struct stepper_move, node);
    uint32_t move_interval = m->interval;
    uint_fast16_t move_count = m->count;
    int_fast16_t move_add = m->add;
    uint_fast8_t need_dir_change = m->flags & MF_DIR;
    move_free(m);

    // Add all steps to s->position (stepper_get_position() can calc mid-move)
    s->position = (need_dir_change ? -s->position : s->position) + move_count;

    // Load next move into 'struct stepper'
    s->add = move_add;
    s->interval = move_interval + move_add;
    if (HAVE_EDGE_OPTIMIZATION && s->flags & SF_OPTIMIZED_PATH) {
        // Using optimized stepper_event_edge()
        s->time.waketime += move_interval;
        s->count = move_count;
    } else if (HAVE_AVR_OPTIMIZATION && s->flags & SF_OPTIMIZED_PATH) {
        // Using optimized stepper_event_avr()
        s->time.waketime += move_interval;
        s->count = move_count;
        s->flags = move_add ? s->flags | SF_HAVE_ADD : s->flags & ~SF_HAVE_ADD;
    } else {
        // Using fully scheduled stepper_event_full() code (the scheduler
        // may be called twice for each step)
        uint_fast8_t was_active = !!s->count;
        uint32_t min_next_time = s->time.waketime;
        s->next_step_time += move_interval;
        s->time.waketime = s->next_step_time;
        s->count = (s->flags & SF_SINGLE_SCHED ? move_count
                    : (uint32_t)move_count * 2);
        if (was_active && timer_is_before(s->next_step_time, min_next_time)) {
            // Actively stepping and next step event close to the last unstep
            int32_t diff = s->next_step_time - min_next_time;
            if (diff < (int32_t)-timer_from_us(1000))
                shutdown("Stepper too far in past");
            s->time.waketime = min_next_time;
        }
        if (was_active && need_dir_change) {
            // Must ensure minimum time between step change and dir change
            if (s->flags & SF_SINGLE_SCHED)
                while (timer_is_before(timer_read_time(), min_next_time))
                    ;
            gpio_out_toggle_noirq(s->dir_pin);
            uint32_t curtime = timer_read_time();
            min_next_time = curtime + s->step_pulse_ticks;
            if (timer_is_before(s->time.waketime, min_next_time))
                s->time.waketime = min_next_time;
            return SF_RESCHEDULE;
        }
    }

    // Set new direction (if needed)
    if (need_dir_change)
        gpio_out_toggle_noirq(s->dir_pin);
    return SF_RESCHEDULE;
}

// Edge optimization only enabled when fastest rate notably slower than 100ns
#define EDGE_STEP_TICKS DIV_ROUND_UP(CONFIG_CLOCK_FREQ, 8000000)
#if HAVE_EDGE_OPTIMIZATION
 DECL_CONSTANT("STEPPER_OPTIMIZED_EDGE", EDGE_STEP_TICKS);
#endif

// Optimized step function to step on each step pin edge
static uint_fast8_t
stepper_event_edge(struct timer *t)
{
    struct stepper *s = container_of(t, struct stepper, time);
    gpio_out_toggle_noirq(s->step_pin);
    uint32_t count = s->count - 1;
    if (likely(count)) {
        s->count = count;
        s->time.waketime += s->interval;
        s->interval += s->add;
        return SF_RESCHEDULE;
    }
    return stepper_load_next(s);
}

#define AVR_STEP_TICKS 40 // minimum instructions between step gpio pulses
#if HAVE_AVR_OPTIMIZATION
 DECL_CONSTANT("STEPPER_OPTIMIZED_UNSTEP", AVR_STEP_TICKS);
#endif

// AVR optimized step function
static uint_fast8_t
stepper_event_avr(struct timer *t)
{
    struct stepper *s = container_of(t, struct stepper, time);
    gpio_out_toggle_noirq(s->step_pin);
    uint16_t *pcount = (void*)&s->count, count = *pcount - 1;
    if (likely(count)) {
        *pcount = count;
        s->time.waketime += s->interval;
        gpio_out_toggle_noirq(s->step_pin);
        if (s->flags & SF_HAVE_ADD)
            s->interval += s->add;
        return SF_RESCHEDULE;
    }
    uint_fast8_t ret = stepper_load_next(s);
    gpio_out_toggle_noirq(s->step_pin);
    return ret;
}

// Regular "fully scheduled" step function
static uint_fast8_t
stepper_event_full(struct timer *t)
{
    struct stepper *s = container_of(t, struct stepper, time);
    gpio_out_toggle_noirq(s->step_pin);
    uint32_t curtime = timer_read_time();
    uint32_t min_next_time = curtime + s->step_pulse_ticks;
    uint32_t count = s->count - 1;
    if (likely(count & 1 && !(s->flags & SF_SINGLE_SCHED)))
        // Schedule unstep event
        goto reschedule_min;
    if (likely(count)) {
        s->next_step_time += s->interval;
        s->interval += s->add;
        if (unlikely(timer_is_before(s->next_step_time, min_next_time)))
            // The next step event is too close - push it back
            goto reschedule_min;
        s->count = count;
        s->time.waketime = s->next_step_time;
        return SF_RESCHEDULE;
    }
    s->time.waketime = min_next_time;
    return stepper_load_next(s);
reschedule_min:
    s->count = count;
    s->time.waketime = min_next_time;
    return SF_RESCHEDULE;
}

// Optimized entry point for step function (may be inlined into sched.c code)
uint_fast8_t
stepper_event(struct timer *t)
{
    if (HAVE_EDGE_OPTIMIZATION)
        return stepper_event_edge(t);
    if (HAVE_AVR_OPTIMIZATION)
        return stepper_event_avr(t);
    return stepper_event_full(t);
}

void
command_config_stepper(uint32_t *args)
{
    struct stepper *s = oid_alloc(args[0], command_config_stepper, sizeof(*s));
    int_fast8_t invert_step = args[3];
    if (invert_step > 0)
        s->flags = SF_INVERT_STEP;
    else if (invert_step < 0)
        s->flags = SF_SINGLE_SCHED;
    s->step_pin = gpio_out_setup(args[1], s->flags & SF_INVERT_STEP);
    s->dir_pin = gpio_out_setup(args[2], 0);
    s->position = -POSITION_BIAS;
    s->step_pulse_ticks = args[4];
    move_queue_setup(&s->mq, sizeof(struct stepper_move));
    if (HAVE_EDGE_OPTIMIZATION) {
        if (invert_step < 0 && s->step_pulse_ticks <= EDGE_STEP_TICKS)
            s->flags |= SF_OPTIMIZED_PATH;
        else
            s->time.func = stepper_event_full;
    } else if (HAVE_AVR_OPTIMIZATION) {
        if (invert_step >= 0 && s->step_pulse_ticks <= AVR_STEP_TICKS)
            s->flags |= SF_SINGLE_SCHED | SF_OPTIMIZED_PATH;
        else
            s->time.func = stepper_event_full;
    } else if (!CONFIG_INLINE_STEPPER_HACK) {
        s->time.func = stepper_event_full;
    }
}
DECL_COMMAND(command_config_stepper, "config_stepper oid=%c step_pin=%c"
             " dir_pin=%c invert_step=%c step_pulse_ticks=%u");

// Return the 'struct stepper' for a given stepper oid
static struct stepper *
stepper_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_stepper);
}

// Schedule a set of steps with a given timing
void
command_queue_step(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    struct stepper_move *m = move_alloc();
    m->interval = args[1];
    m->count = args[2];
    if (!m->count)
        shutdown("Invalid count parameter");
    m->add = args[3];
    m->flags = 0;

    irq_disable();
    uint8_t flags = s->flags;
    if (!!(flags & SF_LAST_DIR) != !!(flags & SF_NEXT_DIR)) {
        flags ^= SF_LAST_DIR;
        m->flags |= MF_DIR;
    }
    if (s->count) {
        s->flags = flags;
        move_queue_push(&m->node, &s->mq);
    } else if (flags & SF_NEED_RESET) {
        move_free(m);
    } else {
        s->flags = flags;
        move_queue_push(&m->node, &s->mq);
        stepper_load_next(s);
        sched_add_timer(&s->time);
    }
    irq_enable();
}
DECL_COMMAND(command_queue_step,
             "queue_step oid=%c interval=%u count=%hu add=%hi");

// Set the direction of the next queued step
void
command_set_next_step_dir(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint8_t nextdir = args[1] ? SF_NEXT_DIR : 0;
    irq_disable();
    s->flags = (s->flags & ~SF_NEXT_DIR) | nextdir;
    irq_enable();
}
DECL_COMMAND(command_set_next_step_dir, "set_next_step_dir oid=%c dir=%c");

// Set an absolute time that the next step will be relative to
void
command_reset_step_clock(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint32_t waketime = args[1];
    irq_disable();
    if (s->count)
        shutdown("Can't reset time when stepper active");
    s->next_step_time = s->time.waketime = waketime;
    s->flags &= ~SF_NEED_RESET;
    irq_enable();
}
DECL_COMMAND(command_reset_step_clock, "reset_step_clock oid=%c clock=%u");

// Return the current stepper position.  Caller must disable irqs.
static uint32_t
stepper_get_position(struct stepper *s)
{
    uint32_t position = s->position;
    // If stepper is mid-move, subtract out steps not yet taken
    if (s->flags & SF_SINGLE_SCHED)
        position -= s->count;
    else
        position -= s->count / 2;
    // The top bit of s->position is an optimized reverse direction flag
    if (position & 0x80000000)
        return -position;
    return position;
}

// Report the current position of the stepper
void
command_stepper_get_position(uint32_t *args)
{
    uint8_t oid = args[0];
    struct stepper *s = stepper_oid_lookup(oid);
    irq_disable();
    uint32_t position = stepper_get_position(s);
    irq_enable();
    sendf("stepper_position oid=%c pos=%i", oid, position - POSITION_BIAS);
}
DECL_COMMAND(command_stepper_get_position, "stepper_get_position oid=%c");

// Stop all moves for a given stepper (caller must disable IRQs)
static void
stepper_stop(struct trsync_signal *tss, uint8_t reason)
{
    struct stepper *s = container_of(tss, struct stepper, stop_signal);
    sched_del_timer(&s->time);
    s->next_step_time = s->time.waketime = 0;
    s->position = -stepper_get_position(s);
    s->count = 0;
    s->flags = ((s->flags & (SF_INVERT_STEP|SF_SINGLE_SCHED|SF_OPTIMIZED_PATH))
                | SF_NEED_RESET);
    gpio_out_write(s->dir_pin, 0);
    if (!(s->flags & SF_SINGLE_SCHED)
        || (HAVE_AVR_OPTIMIZATION && s->flags & SF_OPTIMIZED_PATH))
        // Must return step pin to "unstep" state
        gpio_out_write(s->step_pin, s->flags & SF_INVERT_STEP);
    while (!move_queue_empty(&s->mq)) {
        struct move_node *mn = move_queue_pop(&s->mq);
        struct stepper_move *m = container_of(mn, struct stepper_move, node);
        move_free(m);
    }
}

// Set the stepper to stop on a "trigger event" (used in homing)
void
command_stepper_stop_on_trigger(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    struct trsync *ts = trsync_oid_lookup(args[1]);
    trsync_add_signal(ts, &s->stop_signal, stepper_stop);
}
DECL_COMMAND(command_stepper_stop_on_trigger,
             "stepper_stop_on_trigger oid=%c trsync_oid=%c");

void
stepper_shutdown(void)
{
    uint8_t i;
    struct stepper *s;
    foreach_oid(i, s, command_config_stepper) {
        move_queue_clear(&s->mq);
        stepper_stop(&s->stop_signal, 0);
    }
}
DECL_SHUTDOWN(stepper_shutdown);

// ============================================================
// HPC6602 Inkjet nozzle mode implementation
// ============================================================

// Fire all selected nozzles in nozzle_code one by one.
// For each bit n (0..11) that is set: output 4-bit address on
// pin_a/b/c/d, assert pin_pulse for ~5 µs, then clear everything.
static void
inkjet_fire_nozzles(struct inkjet_pins *ink)
{
    uint16_t code = ink->nozzle_code;
    if (!code || code >= 4096)
        return;

    for (uint8_t n = 0; n < 12; n++) {
        if (!(code & (1 << n)))
            continue;
        // Set 4-bit nozzle address
        gpio_out_write(ink->pin_a, (n >> 0) & 1);
        gpio_out_write(ink->pin_b, (n >> 1) & 1);
        gpio_out_write(ink->pin_c, (n >> 2) & 1);
        gpio_out_write(ink->pin_d, (n >> 3) & 1);
        // Fire pulse (~5 µs)
        gpio_out_write(ink->pin_pulse, 1);
        uint32_t end = timer_read_time() + timer_from_us(5);
        while (timer_is_before(timer_read_time(), end))
            ;
        // Clear everything
        gpio_out_write(ink->pin_pulse, 0);
        gpio_out_write(ink->pin_a, 0);
        gpio_out_write(ink->pin_b, 0);
        gpio_out_write(ink->pin_c, 0);
        gpio_out_write(ink->pin_d, 0);
    }
}

// Step event for inkjet steppers: fire nozzles instead of toggling step pin.
// Uses SF_SINGLE_SCHED so each event fires exactly once per "step count".
static uint_fast8_t
inkjet_event_full(struct timer *t)
{
    struct stepper *s = container_of(t, struct stepper, time);

    // Fire the nozzle pattern
    inkjet_fire_nozzles(s->inkjet);

    uint32_t curtime = timer_read_time();
    uint32_t min_next_time = curtime + s->step_pulse_ticks;
    uint32_t count = s->count - 1;
    if (likely(count)) {
        s->next_step_time += s->interval;
        s->interval += s->add;
        if (unlikely(timer_is_before(s->next_step_time, min_next_time)))
            goto reschedule_min;
        s->count = count;
        s->time.waketime = s->next_step_time;
        return SF_RESCHEDULE;
    }
    s->time.waketime = min_next_time;
    return stepper_load_next(s);
reschedule_min:
    s->count = count;
    s->time.waketime = min_next_time;
    return SF_RESCHEDULE;
}

// MCU timer callback: apply a previously scheduled nozzle_code change.
static uint_fast8_t
inkjet_code_update_event(struct timer *t)
{
    struct inkjet_pins *ink = container_of(t, struct inkjet_pins, code_timer);
    ink->nozzle_code = ink->pending_code;
    ink->code_timer_active = 0;
    return SF_DONE;
}

// Command: attach inkjet nozzle pins to an existing stepper oid.
// Must be sent after config_stepper for the same oid.
// The stepper's step/dir pins remain configured but are never toggled
// while inkjet mode is active.
void
command_config_inkjet_nozzle(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    struct inkjet_pins *ink = alloc_chunk(sizeof(*ink));
    ink->pin_a      = gpio_out_setup(args[1], 0);
    ink->pin_b      = gpio_out_setup(args[2], 0);
    ink->pin_c      = gpio_out_setup(args[3], 0);
    ink->pin_d      = gpio_out_setup(args[4], 0);
    ink->pin_pulse  = gpio_out_setup(args[5], 0);
    ink->nozzle_code        = 4096;  // disabled by default
    ink->pending_code       = 4096;
    ink->code_timer_active  = 0;
    ink->code_timer.func    = inkjet_code_update_event;
    s->inkjet = ink;
    // Force single-schedule mode (one event per step) and use inkjet handler.
    // Clear any AVR / edge optimisation flags so inkjet_event_full is used.
    s->flags = (s->flags & ~SF_OPTIMIZED_PATH) | SF_SINGLE_SCHED;
    s->time.func = inkjet_event_full;
}
DECL_COMMAND(command_config_inkjet_nozzle,
             "config_inkjet_nozzle oid=%c pin_a=%c pin_b=%c pin_c=%c"
             " pin_d=%c pin_pulse=%c");

// Command: schedule a nozzle_code update at a specific MCU clock time.
// Sending clock=0 applies the change immediately.
void
command_stepper_set_nozzle_code(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    if (!s->inkjet)
        return;
    uint32_t waketime   = args[1];
    uint16_t code       = (uint16_t)args[2];
    struct inkjet_pins *ink = s->inkjet;

    if (!waketime || !timer_is_before(timer_read_time(), waketime)) {
        // Apply immediately
        if (ink->code_timer_active) {
            sched_del_timer(&ink->code_timer);
            ink->code_timer_active = 0;
        }
        ink->nozzle_code = code;
        return;
    }
    // Schedule for future clock time
    ink->pending_code = code;
    if (ink->code_timer_active)
        sched_del_timer(&ink->code_timer);
    ink->code_timer.waketime = waketime;
    sched_add_timer(&ink->code_timer);
    ink->code_timer_active = 1;
}
DECL_COMMAND(command_stepper_set_nozzle_code,
             "stepper_set_nozzle_code oid=%c clock=%u nozzle_code=%hu");
